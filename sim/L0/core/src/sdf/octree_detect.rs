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

    // 2. Target cell size — 2× grid resolution.
    // Contact distribution needs spatial coverage, not sub-cell precision.
    // Newton refinement gives machine-precision contacts at any cell size;
    // the cell size only controls how many seed points are generated.
    // 2× keeps contact density adequate while halving octree depth (~8× less work).
    let cell_size = a.sdf_grid().cell_size().min(b.sdf_grid().cell_size()) * 2.0;
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

// ── Diagnostics variant ─────────────────────────────────────────────────

/// Octree detection statistics.
#[derive(Debug, Clone, Default)]
pub struct OctreeStats {
    /// Number of `evaluate_interval` calls (2 per octree node visited).
    pub interval_evals: usize,
    /// Number of leaf cells that attempted contact extraction.
    pub leaf_cells: usize,
}

/// Like [`octree_contact_detect`] but also returns performance statistics.
pub fn octree_contact_detect_with_stats(
    a: &dyn PhysicsShape,
    pose_a: &Pose,
    b: &dyn PhysicsShape,
    pose_b: &Pose,
    margin: f64,
    max_contacts: usize,
) -> (Vec<SdfContact>, OctreeStats) {
    let world_a = world_aabb(&a.bounds(), pose_a);
    let world_b = world_aabb(&b.bounds(), pose_b);
    let root = world_a.intersection(&world_b).expanded(margin);
    if root.is_empty() {
        return (vec![], OctreeStats::default());
    }

    let cell_size = a.sdf_grid().cell_size().min(b.sdf_grid().cell_size());
    let ratio = root.max_extent() / cell_size;
    let max_depth = if ratio > 1.0 {
        ratio.log2().ceil() as u32
    } else {
        0
    };

    let mut leaf_contacts = Vec::new();
    let mut stats = OctreeStats::default();
    octree_recurse_stats(
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
        &mut stats,
    );

    dedup_contacts(&mut leaf_contacts, cell_size);
    if leaf_contacts.len() > max_contacts {
        select_representatives(&mut leaf_contacts, max_contacts);
    }

    (leaf_contacts, stats)
}

/// Recursive octree with statistics collection.
#[allow(clippy::too_many_arguments)]
fn octree_recurse_stats(
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
    stats: &mut OctreeStats,
) {
    let local_a = conservative_local_aabb(cell, pose_a);
    let local_b = conservative_local_aabb(cell, pose_b);

    let Some((lo_a, hi_a)) = a.evaluate_interval(&local_a) else {
        return;
    };
    let Some((lo_b, hi_b)) = b.evaluate_interval(&local_b) else {
        return;
    };
    stats.interval_evals += 2; // one per shape

    let cell_extent = cell.max_extent();
    let depth_threshold = -cell_extent;

    if lo_a > margin {
        return;
    }
    if lo_b > margin {
        return;
    }
    if hi_a < depth_threshold {
        return;
    }
    if hi_b < depth_threshold {
        return;
    }

    if cell_extent <= target_cell_size || depth >= max_depth {
        stats.leaf_cells += 1;
        extract_leaf_contacts(cell, a, pose_a, b, pose_b, margin, contacts);
        return;
    }

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
        octree_recurse_stats(
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
            stats,
        );
    }
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

// ── Plane-specific octree detection ──────────────────────────────────────

/// Detect contacts between an SDF shape and an infinite plane using octree
/// interval pruning.
///
/// The shape must support `evaluate_interval()`. The plane's interval over
/// any AABB is trivially exact (linear function → extrema at corners).
/// The octree prunes cells that are entirely above the plane or entirely
/// deep inside the shape.
pub fn octree_plane_detect(
    shape: &dyn PhysicsShape,
    shape_pose: &Pose,
    plane_normal: &nalgebra::Vector3<f64>,
    plane_offset: f64,
    margin: f64,
    max_contacts: usize,
) -> Vec<SdfContact> {
    // Root: shape's world-space AABB (plane is infinite — no intersection needed)
    let root = world_aabb(&shape.bounds(), shape_pose).expanded(margin);
    if root.is_empty() {
        return vec![];
    }

    // Use a coarser target cell for plane contacts (4× grid resolution).
    // Plane contacts need spatial distribution across the face for torque
    // resistance, not sub-millimeter precision. Coarser cells cut octree
    // depth by ~2 levels, reducing traversal work ~64× while still producing
    // well-separated contacts (e.g., 2.5mm spacing on a 10mm face → 4+ contacts).
    let cell_size = shape.sdf_grid().cell_size() * 4.0;
    let ratio = root.max_extent() / cell_size;
    let max_depth = if ratio > 1.0 {
        ratio.log2().ceil() as u32
    } else {
        0
    };

    let mut leaf_contacts = Vec::new();
    octree_plane_recurse(
        &root,
        0,
        max_depth,
        shape,
        shape_pose,
        plane_normal,
        plane_offset,
        margin,
        cell_size,
        &mut leaf_contacts,
    );

    dedup_contacts(&mut leaf_contacts, cell_size);
    if leaf_contacts.len() > max_contacts {
        select_representatives(&mut leaf_contacts, max_contacts);
    }
    leaf_contacts
}

/// Recursive octree for shape-vs-plane contact detection.
#[allow(clippy::too_many_arguments)]
fn octree_plane_recurse(
    cell: &Aabb,
    depth: u32,
    max_depth: u32,
    shape: &dyn PhysicsShape,
    shape_pose: &Pose,
    plane_normal: &nalgebra::Vector3<f64>,
    plane_offset: f64,
    margin: f64,
    target_cell_size: f64,
    contacts: &mut Vec<SdfContact>,
) {
    // Shape interval in local frame
    let local_cell = conservative_local_aabb(cell, shape_pose);
    let Some((lo_shape, hi_shape)) = shape.evaluate_interval(&local_cell) else {
        return;
    };

    // Plane interval over the world-space cell (exact for linear function)
    let (lo_plane, _hi_plane) = plane_interval(cell, plane_normal, plane_offset);

    let cell_extent = cell.max_extent();

    // Prune: shape surface not in this cell
    if lo_shape > margin {
        return; // entirely outside shape
    }
    if hi_shape < -cell_extent {
        return; // entirely deep inside shape
    }

    // Prune: cell entirely above the plane (positive side, no contact)
    if lo_plane > margin {
        return;
    }
    // Prune: cell entirely deep below the plane (shape surface would need
    // to be below the plane too — but we only care about the intersection)
    // Actually: we need shape surface AND below-plane. If the cell is
    // entirely far below the plane, and the shape surface is here, that's
    // still a valid deep-penetration contact. So only prune if the shape
    // surface is NOT here.

    // Leaf: extract contacts
    if cell_extent <= target_cell_size || depth >= max_depth {
        extract_plane_leaf(
            cell,
            shape,
            shape_pose,
            plane_normal,
            plane_offset,
            margin,
            contacts,
        );
        return;
    }

    // Subdivide
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
        octree_plane_recurse(
            child,
            depth + 1,
            max_depth,
            shape,
            shape_pose,
            plane_normal,
            plane_offset,
            margin,
            target_cell_size,
            contacts,
        );
    }
}

/// Extract contacts from a leaf cell for shape-vs-plane.
///
/// Projects the cell center onto the shape's surface via Newton, then
/// checks if the refined point is below the plane.
fn extract_plane_leaf(
    cell: &Aabb,
    shape: &dyn PhysicsShape,
    shape_pose: &Pose,
    plane_normal: &nalgebra::Vector3<f64>,
    plane_offset: f64,
    margin: f64,
    contacts: &mut Vec<SdfContact>,
) {
    let center = cell.center();
    let max_displacement = cell.max_extent() * 3.0;

    let p_local = shape_pose.inverse_transform_point(&center);
    let Some(d_shape) = shape.distance(&p_local) else {
        return;
    };

    // Shape surface must be near this cell
    let half_size = cell.max_extent() * 0.5;
    if d_shape.abs() > half_size * 2.0 {
        return;
    }

    // Newton-refine onto shape surface
    let mut p = p_local;
    let p_original = p;
    let Some(baseline_grad) = shape.gradient(&p) else {
        return;
    };

    for _ in 0..3 {
        let Some(d) = shape.distance(&p) else { return };
        if d.abs() < 1e-10 {
            break;
        }
        let Some(n) = shape.gradient(&p) else { return };
        p -= n * d;
        if (p - p_original).norm() > max_displacement {
            return;
        }
    }

    let Some(n_local) = shape.gradient(&p) else {
        return;
    };
    if n_local.dot(&baseline_grad) < 0.0 {
        return;
    }

    // Check if refined point is below the plane
    let world_point = shape_pose.transform_point(&p);
    let dist_to_plane = world_point.coords.dot(plane_normal) - plane_offset;
    if dist_to_plane >= margin - 1e-10 {
        return;
    }

    contacts.push(SdfContact {
        point: world_point,
        normal: *plane_normal, // Plane contact: normal is the plane normal
        penetration: (-dist_to_plane).max(0.0),
    });
}

/// Compute the plane's interval over a world-space AABB.
///
/// Exact for a linear function: evaluate dot product at all 8 corners,
/// return (min, max).
fn plane_interval(aabb: &Aabb, normal: &nalgebra::Vector3<f64>, offset: f64) -> (f64, f64) {
    let corners = aabb.corners();
    let mut lo = f64::MAX;
    let mut hi = f64::MIN;
    for c in &corners {
        let d = c.coords.dot(normal) - offset;
        lo = lo.min(d);
        hi = hi.max(d);
    }
    (lo, hi)
}

// ── Normal refinement for grid contacts ─────────────────────────────────

/// Refine the normal of an existing contact using a shape's analytical
/// distance/gradient.
///
/// Projects the contact point onto the shape's zero-level set via Newton
/// iteration and recomputes the normal. The contact position and depth
/// are NOT changed — only the normal is improved.
///
/// For grid-backed shapes, this improves normals from O(cell_size) to
/// O(cell_size²) error. For analytical shapes, this gives machine precision.
///
/// Falls back to no-op if Newton diverges, normal flips, or queries return
/// None.
pub fn refine_contact_normal(
    contact: &mut SdfContact,
    shape: &dyn PhysicsShape,
    shape_pose: &Pose,
    cell_size: f64,
) {
    let max_displacement = cell_size * 3.0;

    let mut p = shape_pose.inverse_transform_point(&contact.point);
    let p_original = p;

    let Some(baseline_grad) = shape.gradient(&p) else {
        return;
    };

    // Newton iteration: project onto zero-level set
    for _ in 0..3 {
        let Some(d) = shape.distance(&p) else { return };
        if d.abs() < 1e-10 {
            break;
        }
        let Some(n) = shape.gradient(&p) else { return };
        p -= n * d;
        if (p - p_original).norm() > max_displacement {
            return; // diverged — keep original
        }
    }

    // Normal consistency check
    let Some(n_local) = shape.gradient(&p) else {
        return;
    };
    if n_local.dot(&baseline_grad) < 0.0 {
        return; // flipped — keep original
    }

    // Update only the normal (preserve original point and depth)
    contact.normal = shape_pose.rotation * n_local;
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

        // Lateral force should be small relative to vertical.
        // At coarser octree target (2× grid cell), contacts are sparser
        // and may have slightly more asymmetry.
        let lateral = net.x.hypot(net.y);
        assert!(
            lateral < net.z.abs() * 0.3,
            "lateral ({:.4}) should be < 30% of vertical ({:.4})",
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
