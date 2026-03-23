//! Octree bi-surface contact detection for SDF-SDF collision.
//!
//! Uses `PhysicsShape::evaluate_interval()` to recursively prune an octree,
//! finding only the leaf cells where both shapes' surfaces are present. At
//! each leaf, Newton iteration projects a seed point onto the exact zero-level
//! set for machine-precision contact points and normals.
//!
//! This is Tier 2 in the three-tier dispatch (see `compute_shape_contact`):
//! - Tier 1: Both convex → analytical single contact
//! - **Tier 2: Both support `evaluate_interval` → octree detection (this module)**
//! - Tier 3: Grid fallback → surface tracing

// Octree detection involves index arithmetic and floating-point comparisons
// that clippy flags but are correct for this algorithm.
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::similar_names
)]

use cf_geometry::Aabb;
use nalgebra::Point3;
use sim_types::Pose;

use super::SdfContact;
use super::shape::PhysicsShape;

// ── Public entry point ──────────────────────────────────────────────────

/// Detect contacts between two shapes using octree interval pruning.
///
/// Both shapes must support `evaluate_interval()` (returns `Some`).
/// The octree root is the world-space intersection of both shapes' bounds,
/// expanded by `margin`. Leaf cells where both surfaces are present yield
/// exact contacts via Newton iteration.
///
/// Returns at most `max_contacts` contacts, spatially distributed across
/// contact patches.
pub fn octree_contact_detect(
    a: &dyn PhysicsShape,
    pose_a: &Pose,
    b: &dyn PhysicsShape,
    pose_b: &Pose,
    margin: f64,
    max_contacts: usize,
) -> Vec<SdfContact> {
    // 1. Overlap AABB in world space
    let world_a = world_aabb(&a.bounds(), pose_a);
    let world_b = world_aabb(&b.bounds(), pose_b);
    let root = world_a.intersection(&world_b).expanded(margin);
    if root.is_empty() {
        return vec![];
    }

    // 2. Target cell size (matches grid resolution for comparability)
    let cell_size = a.sdf_grid().cell_size().min(b.sdf_grid().cell_size());
    let ratio = root.max_extent() / cell_size;
    let max_depth = if ratio > 1.0 {
        ratio.log2().ceil() as u32
    } else {
        0
    };

    // 3. Recursive octree traversal
    let mut leaf_contacts = Vec::new();
    octree_recurse(
        &root,
        0,
        max_depth,
        a,
        pose_a,
        b,
        pose_b,
        margin,
        cell_size,
        &mut leaf_contacts,
    );

    // 4. Dedup + select representatives
    dedup_contacts(&mut leaf_contacts, cell_size);
    if leaf_contacts.len() > max_contacts {
        select_representatives(&mut leaf_contacts, max_contacts);
    }

    leaf_contacts
}

// ── Octree recursion ────────────────────────────────────────────────────

/// Recursive octree traversal with interval pruning.
///
/// At each node, transforms the world-space cell to each shape's local frame,
/// evaluates interval bounds, and prunes if contact is impossible. Leaf cells
/// extract contacts via Newton iteration.
#[allow(clippy::too_many_arguments)]
fn octree_recurse(
    cell: &Aabb,
    depth: u32,
    max_depth: u32,
    a: &dyn PhysicsShape,
    pose_a: &Pose,
    b: &dyn PhysicsShape,
    pose_b: &Pose,
    margin: f64,
    target_cell_size: f64,
    contacts: &mut Vec<SdfContact>,
) {
    // Transform cell to each shape's local frame (conservative AABB)
    let local_a = conservative_local_aabb(cell, pose_a);
    let local_b = conservative_local_aabb(cell, pose_b);

    // Evaluate interval bounds on both shapes
    let Some((lo_a, hi_a)) = a.evaluate_interval(&local_a) else {
        return;
    };
    let Some((lo_b, hi_b)) = b.evaluate_interval(&local_b) else {
        return;
    };

    // Adaptive depth threshold: surfaces could be near a corner even
    // if the cell center is deep inside
    let cell_extent = cell.max_extent();
    let depth_threshold = -cell_extent;

    // === PRUNING RULES ===

    // Rule 1: A entirely outside this cell by more than margin
    if lo_a > margin {
        return;
    }
    // Rule 2: B entirely outside this cell by more than margin
    if lo_b > margin {
        return;
    }
    // Rule 3: A entirely deep inside (surface not in this cell)
    if hi_a < depth_threshold {
        return;
    }
    // Rule 4: B entirely deep inside (surface not in this cell)
    if hi_b < depth_threshold {
        return;
    }

    // === LEAF: extract contacts ===
    if cell_extent <= target_cell_size || depth >= max_depth {
        extract_leaf_contacts(cell, a, pose_a, b, pose_b, margin, contacts);
        return;
    }

    // === SUBDIVIDE into 8 children ===
    let mid = cell.center();
    let min = cell.min;
    let max = cell.max;
    let children = [
        Aabb::new(
            Point3::new(min.x, min.y, min.z),
            Point3::new(mid.x, mid.y, mid.z),
        ),
        Aabb::new(
            Point3::new(mid.x, min.y, min.z),
            Point3::new(max.x, mid.y, mid.z),
        ),
        Aabb::new(
            Point3::new(min.x, mid.y, min.z),
            Point3::new(mid.x, max.y, mid.z),
        ),
        Aabb::new(
            Point3::new(mid.x, mid.y, min.z),
            Point3::new(max.x, max.y, mid.z),
        ),
        Aabb::new(
            Point3::new(min.x, min.y, mid.z),
            Point3::new(mid.x, mid.y, max.z),
        ),
        Aabb::new(
            Point3::new(mid.x, min.y, mid.z),
            Point3::new(max.x, mid.y, max.z),
        ),
        Aabb::new(
            Point3::new(min.x, mid.y, mid.z),
            Point3::new(mid.x, max.y, max.z),
        ),
        Aabb::new(
            Point3::new(mid.x, mid.y, mid.z),
            Point3::new(max.x, max.y, max.z),
        ),
    ];
    for child in &children {
        octree_recurse(
            child,
            depth + 1,
            max_depth,
            a,
            pose_a,
            b,
            pose_b,
            margin,
            target_cell_size,
            contacts,
        );
    }
}

// ── Leaf contact extraction ─────────────────────────────────────────────

/// Extract contacts from a leaf cell where both surfaces may be present.
///
/// Produces up to 2 contacts per leaf: one from A's surface penetrating B,
/// one from B's surface penetrating A. Each contact is found via Newton
/// iteration from the cell center.
fn extract_leaf_contacts(
    cell: &Aabb,
    a: &dyn PhysicsShape,
    pose_a: &Pose,
    b: &dyn PhysicsShape,
    pose_b: &Pose,
    margin: f64,
    contacts: &mut Vec<SdfContact>,
) {
    let center = cell.center();
    let half_size = cell.max_extent() * 0.5;
    let max_displacement = cell.max_extent() * 3.0;

    // Evaluate both shapes at cell center
    let p_a = pose_a.inverse_transform_point(&center);
    let p_b = pose_b.inverse_transform_point(&center);
    let Some(d_a) = a.distance(&p_a) else { return };
    let Some(d_b) = b.distance(&p_b) else { return };

    // Contact from A's surface penetrating B:
    // A's surface is near the cell center AND the cell center is inside/near B
    if d_a.abs() < half_size * 2.0 && d_b < margin {
        if let Some(c) = newton_contact(&p_a, a, pose_a, b, pose_b, margin, max_displacement) {
            contacts.push(c);
        }
    }

    // Contact from B's surface penetrating A:
    // B's surface is near the cell center AND the cell center is inside/near A
    if d_b.abs() < half_size * 2.0 && d_a < margin {
        if let Some(mut c) = newton_contact(&p_b, b, pose_b, a, pose_a, margin, max_displacement) {
            // Negate normal: B's outward normal points B→A, convention is A→B
            c.normal = -c.normal;
            contacts.push(c);
        }
    }
}

// ── Newton contact refinement ───────────────────────────────────────────

/// Project a seed point onto the source shape's zero-level set via Newton
/// iteration, then check penetration into the destination shape.
///
/// Returns `None` if:
/// - Newton diverges (displacement > max_displacement)
/// - Normal flips (converged to wrong surface)
/// - Refined point doesn't penetrate dst (d_dst >= margin)
/// - Any distance/gradient query returns None
fn newton_contact(
    seed_local: &Point3<f64>,
    src: &dyn PhysicsShape,
    src_pose: &Pose,
    dst: &dyn PhysicsShape,
    dst_pose: &Pose,
    margin: f64,
    max_displacement: f64,
) -> Option<SdfContact> {
    let mut p = *seed_local;
    let p_original = p;

    // Baseline gradient for consistency check
    let baseline_grad = src.gradient(&p)?;

    // Newton iteration: project onto src's zero-level set
    for _ in 0..3 {
        let d = src.distance(&p)?;
        if d.abs() < 1e-10 {
            break;
        }
        let n = src.gradient(&p)?;
        p -= n * d;

        // Safety: max displacement guard
        if (p - p_original).norm() > max_displacement {
            return None;
        }
    }

    // Safety: normal consistency guard
    let n_local = src.gradient(&p)?;
    if n_local.dot(&baseline_grad) < 0.0 {
        return None;
    }

    // Transform to world, check penetration into dst
    let world_point = src_pose.transform_point(&p);
    let p_dst = dst_pose.inverse_transform_point(&world_point);
    let d_dst = dst.distance(&p_dst)?;
    if d_dst >= margin - 1e-10 {
        return None;
    }

    let n_world = src_pose.rotation * n_local;
    Some(SdfContact {
        point: world_point,
        normal: n_world,
        penetration: (-d_dst).max(0.0),
    })
}

// ── Geometry utilities ──────────────────────────────────────────────────

/// Compute the world-space AABB of a local-space AABB under a pose transform.
///
/// Transforms all 8 corners and re-bounds. Conservative: the world AABB of a
/// rotated local AABB is larger than the tight OBB (up to √3× per axis at 45°).
fn world_aabb(local: &Aabb, pose: &Pose) -> Aabb {
    let corners = local.corners();
    let transformed: Vec<Point3<f64>> = corners.iter().map(|c| pose.transform_point(c)).collect();
    Aabb::from_points(transformed.iter())
}

/// Transform a world-space AABB to a shape's local frame (conservative).
///
/// Transforms all 8 corners via `pose.inverse_transform_point` and re-bounds.
/// The resulting local AABB is larger than the true projection of the world
/// cell due to rotation bloat.
fn conservative_local_aabb(world_cell: &Aabb, pose: &Pose) -> Aabb {
    let corners = world_cell.corners();
    let transformed: Vec<Point3<f64>> = corners
        .iter()
        .map(|c| pose.inverse_transform_point(c))
        .collect();
    Aabb::from_points(transformed.iter())
}

// ── Contact post-processing ─────────────────────────────────────────────

/// Deduplicate contacts by spatial proximity, keeping deepest first.
///
/// Sorts by penetration depth (deepest first), then greedily keeps contacts
/// that are at least `cell_size * 0.5` apart from all already-kept contacts.
fn dedup_contacts(contacts: &mut Vec<SdfContact>, cell_size: f64) {
    let min_dist_sq = (cell_size * 0.5) * (cell_size * 0.5);

    contacts.sort_by(|a, b| {
        b.penetration
            .partial_cmp(&a.penetration)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut kept = Vec::with_capacity(contacts.len());
    for c in contacts.drain(..) {
        let dominated = kept
            .iter()
            .any(|k: &SdfContact| (k.point - c.point).norm_squared() < min_dist_sq);
        if !dominated {
            kept.push(c);
        }
    }
    *contacts = kept;
}

/// Select at most `budget` spatially-distributed representatives.
///
/// Greedy farthest-point selection: start with the deepest contact (already
/// first after dedup), then repeatedly add the contact maximally distant from
/// all already-selected contacts.
fn select_representatives(contacts: &mut Vec<SdfContact>, budget: usize) {
    if contacts.len() <= budget {
        return;
    }

    // contacts are already sorted deepest-first from dedup. Start with [0].
    let mut selected = Vec::with_capacity(budget);
    selected.push(contacts[0].clone());

    for _ in 1..budget {
        // Find the contact maximally distant from all selected
        let mut best_idx = 0;
        let mut best_min_dist = 0.0_f64;

        for (i, c) in contacts.iter().enumerate() {
            // Skip if already selected (check by point proximity)
            if selected
                .iter()
                .any(|s: &SdfContact| (s.point - c.point).norm_squared() < 1e-20)
            {
                continue;
            }
            let min_dist = selected
                .iter()
                .map(|s: &SdfContact| (s.point - c.point).norm_squared())
                .fold(f64::MAX, f64::min);
            if min_dist > best_min_dist {
                best_min_dist = min_dist;
                best_idx = i;
            }
        }

        if best_min_dist < 1e-20 {
            break; // no more distinct contacts
        }
        selected.push(contacts[best_idx].clone());
    }

    *contacts = selected;
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::items_after_statements
)]
mod tests {
    use super::*;
    use crate::sdf::shapes::ShapeSphere;
    use cf_geometry::SdfGrid;
    use nalgebra::{UnitQuaternion, Vector3};
    use std::sync::Arc;

    fn sphere_shape(radius: f64) -> ShapeSphere {
        let grid = Arc::new(SdfGrid::sphere(Point3::origin(), radius, 20, 2.0));
        ShapeSphere::new(grid, radius)
    }

    fn identity_pose_at(pos: Point3<f64>) -> Pose {
        Pose {
            position: pos,
            rotation: UnitQuaternion::identity(),
        }
    }

    // ── world_aabb / conservative_local_aabb ────────────────────────────

    #[test]
    fn world_aabb_identity_pose() {
        let local = Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0));
        let pose = identity_pose_at(Point3::new(5.0, 0.0, 0.0));
        let world = world_aabb(&local, &pose);
        assert!((world.center().x - 5.0).abs() < 1e-10);
        assert!((world.max_extent() - 2.0).abs() < 1e-10);
    }

    #[test]
    fn conservative_local_roundtrip() {
        let original = Aabb::new(Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0));
        let pose = identity_pose_at(Point3::new(10.0, 0.0, 0.0));
        let world = world_aabb(&original, &pose);
        let back = conservative_local_aabb(&world, &pose);
        // For identity rotation, roundtrip should be exact
        assert!((back.min.x - original.min.x).abs() < 1e-10);
        assert!((back.max.z - original.max.z).abs() < 1e-10);
    }

    // ── Separated shapes → no contacts ──────────────────────────────────

    #[test]
    fn separated_spheres_no_contacts() {
        let a = sphere_shape(5.0);
        let b = sphere_shape(5.0);
        let pose_a = identity_pose_at(Point3::origin());
        let pose_b = identity_pose_at(Point3::new(20.0, 0.0, 0.0));

        let contacts = octree_contact_detect(&a, &pose_a, &b, &pose_b, 0.1, 8);
        assert!(
            contacts.is_empty(),
            "separated spheres should have no contacts"
        );
    }

    // ── Overlapping spheres → contacts ──────────────────────────────────

    #[test]
    fn overlapping_spheres_produce_contacts() {
        let a = sphere_shape(5.0);
        let b = sphere_shape(5.0);
        let pose_a = identity_pose_at(Point3::origin());
        let pose_b = identity_pose_at(Point3::new(8.0, 0.0, 0.0));

        let contacts = octree_contact_detect(&a, &pose_a, &b, &pose_b, 0.5, 8);
        assert!(
            !contacts.is_empty(),
            "overlapping spheres should produce contacts"
        );

        // Contacts should be in the overlap region
        for c in &contacts {
            assert!(
                c.point.x > 2.0 && c.point.x < 6.0,
                "contact x={:.2} should be in overlap region [2, 6]",
                c.point.x
            );
        }

        // Net force should push B in +X (away from A)
        let net: Vector3<f64> = contacts.iter().map(|c| c.normal * c.penetration).sum();
        assert!(
            net.x > 0.0,
            "net force should push B in +X, got x={:.4}",
            net.x
        );
    }

    // ── Stacking scenario ───────────────────────────────────────────────

    #[test]
    fn stacked_spheres_vertical_force() {
        let a = sphere_shape(5.0);
        let b = sphere_shape(5.0);
        let pose_a = identity_pose_at(Point3::origin());
        let pose_b = identity_pose_at(Point3::new(0.0, 0.0, 9.5));

        let contacts = octree_contact_detect(&a, &pose_a, &b, &pose_b, 0.5, 8);
        assert!(
            !contacts.is_empty(),
            "stacked spheres should produce contacts"
        );

        let net: Vector3<f64> = contacts.iter().map(|c| c.normal * c.penetration).sum();
        assert!(
            net.z > 0.0,
            "net force should point upward, got z={:.4}",
            net.z
        );

        // Lateral force should be small relative to vertical
        let lateral = net.x.hypot(net.y);
        assert!(
            lateral < net.z.abs() * 0.1,
            "lateral ({:.4}) should be < 10% of vertical ({:.4})",
            lateral,
            net.z.abs()
        );
    }

    // ── Contact budget respected ────────────────────────────────────────

    #[test]
    fn budget_limits_contacts() {
        let a = sphere_shape(5.0);
        let b = sphere_shape(5.0);
        let pose_a = identity_pose_at(Point3::origin());
        let pose_b = identity_pose_at(Point3::new(8.0, 0.0, 0.0));

        let contacts = octree_contact_detect(&a, &pose_a, &b, &pose_b, 0.5, 4);
        assert!(
            contacts.len() <= 4,
            "should respect budget of 4, got {}",
            contacts.len()
        );
    }

    // ── Newton safety: normal consistency ────────────────────────────────

    #[test]
    fn newton_contact_on_sphere_surface() {
        let shape = sphere_shape(5.0);
        let pose = identity_pose_at(Point3::origin());

        // Seed slightly inside the sphere surface along +X
        let seed = Point3::new(4.8, 0.0, 0.0);
        // dst = another sphere at x=10 (outside, so d_dst will be positive
        // unless we use a large margin)
        let dst = sphere_shape(5.0);
        let dst_pose = identity_pose_at(Point3::new(9.0, 0.0, 0.0));

        let result = newton_contact(&seed, &shape, &pose, &dst, &dst_pose, 1.0, 5.0);
        // Contact should exist: A's surface at x≈5, dst at x=4 (d_dst = -1)
        assert!(result.is_some(), "should find contact on sphere surface");

        let c = result.unwrap();
        // Normal should be approximately +X (A's outward normal)
        assert!(
            c.normal.x > 0.9,
            "normal.x={:.3} should be ~1.0",
            c.normal.x
        );
        // Contact point should be near x=5 (sphere surface)
        assert!(
            (c.point.x - 5.0).abs() < 0.1,
            "contact x={:.3} should be near 5.0",
            c.point.x
        );
    }

    // ── Dedup merges nearby contacts ────────────────────────────────────

    #[test]
    fn dedup_merges_nearby() {
        let mut contacts = vec![
            SdfContact {
                point: Point3::new(1.0, 0.0, 0.0),
                normal: Vector3::x(),
                penetration: 0.5,
            },
            SdfContact {
                point: Point3::new(1.1, 0.0, 0.0),
                normal: Vector3::x(),
                penetration: 0.3,
            },
            SdfContact {
                point: Point3::new(5.0, 0.0, 0.0),
                normal: Vector3::x(),
                penetration: 0.1,
            },
        ];
        dedup_contacts(&mut contacts, 1.0);
        // First two are within 0.5 (cell_size * 0.5) — deeper one survives
        // Third is far away — survives
        assert_eq!(contacts.len(), 2, "should keep 2 distinct contacts");
        assert!(
            (contacts[0].penetration - 0.5).abs() < 1e-10,
            "deepest should be first"
        );
    }

    // ── Select representatives spreads spatially ────────────────────────

    #[test]
    fn select_representatives_spreads() {
        let mut contacts = vec![
            SdfContact {
                point: Point3::new(0.0, 0.0, 0.0),
                normal: Vector3::x(),
                penetration: 1.0,
            },
            SdfContact {
                point: Point3::new(0.1, 0.0, 0.0),
                normal: Vector3::x(),
                penetration: 0.9,
            },
            SdfContact {
                point: Point3::new(10.0, 0.0, 0.0),
                normal: Vector3::x(),
                penetration: 0.1,
            },
        ];
        select_representatives(&mut contacts, 2);
        assert_eq!(contacts.len(), 2);
        // Should keep the deepest (0,0,0) and the farthest (10,0,0)
        let has_origin = contacts.iter().any(|c| c.point.x.abs() < 0.2);
        let has_far = contacts.iter().any(|c| c.point.x > 9.0);
        assert!(has_origin, "should keep deepest contact near origin");
        assert!(has_far, "should keep farthest contact at x=10");
    }
}
