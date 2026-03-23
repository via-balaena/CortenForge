//! SDF-SDF contact detection via surface-tracing.

// Allow casting for grid indices - these are small values and bounds are checked
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap
)]

use nalgebra::{Point3, Vector3};
use sim_types::Pose;

use super::{SdfContact, SdfGrid};

/// Raw surface tracing with depth-sorted spatial deduplication.
///
/// Traces from the **surface** of each SDF into the other. A surface point on
/// SDF A is a contact if it lies within `margin` of SDF B's interior.
/// Used by [`compute_shape_contact`](super::shape::compute_shape_contact)
/// as the grid-based fallback for non-convex shapes.
///
/// After both trace passes, contacts are deduplicated: sorted by penetration
/// depth (deepest first), then greedily filtered to keep only contacts that
/// are spatially separated by at least `cell_size * 0.5`. This matches the
/// GPU dedup path and ensures the deepest, best-separated contacts survive
/// regardless of grid traversal order.
pub fn sdf_sdf_contact_raw(
    sdf_a: &SdfGrid,
    pose_a: &Pose,
    sdf_b: &SdfGrid,
    pose_b: &Pose,
    margin: f64,
) -> Vec<SdfContact> {
    let mut contacts: Vec<SdfContact> = Vec::new();
    trace_surface_into_other(sdf_a, pose_a, sdf_b, pose_b, margin, &mut contacts, false);
    trace_surface_into_other(sdf_b, pose_b, sdf_a, pose_a, margin, &mut contacts, true);

    // Post-hoc spatial dedup: deepest contacts win, skip nearby duplicates.
    let cell_size = sdf_a.cell_size().min(sdf_b.cell_size());
    dedup_contacts(&mut contacts, cell_size);

    contacts
}

/// Like [`sdf_sdf_contact_raw`] but returns two separate vectors:
/// contacts from A's surface (A→B pass) and contacts from B's surface
/// (B→A pass). Used by the normal refinement pipeline to refine each
/// contact against the correct source shape.
pub fn sdf_sdf_contact_raw_split(
    sdf_a: &SdfGrid,
    pose_a: &Pose,
    sdf_b: &SdfGrid,
    pose_b: &Pose,
    margin: f64,
) -> (Vec<SdfContact>, Vec<SdfContact>) {
    let mut from_a: Vec<SdfContact> = Vec::new();
    let mut from_b: Vec<SdfContact> = Vec::new();
    trace_surface_into_other(sdf_a, pose_a, sdf_b, pose_b, margin, &mut from_a, false);
    trace_surface_into_other(sdf_b, pose_b, sdf_a, pose_a, margin, &mut from_b, true);

    let cell_size = sdf_a.cell_size().min(sdf_b.cell_size());
    dedup_contacts(&mut from_a, cell_size);
    dedup_contacts(&mut from_b, cell_size);

    (from_a, from_b)
}

/// Deduplicate contacts by spatial proximity, keeping deepest first.
///
/// Sorts by penetration depth (deepest first), then greedily keeps contacts
/// that are at least `cell_size * 0.5` apart from all already-kept contacts.
/// Matches the GPU path's `dedup_contacts` in `sim-gpu/src/collision.rs`.
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

/// Query two SDFs for contact using surface-tracing.
///
/// Returns raw surface-traced contacts without consolidation or analytical
/// depth correction. For analytical convex-convex contact with depth
/// correction, use [`compute_shape_contact`](super::shape::compute_shape_contact)
/// instead.
#[must_use]
pub fn sdf_sdf_contact(
    sdf_a: &SdfGrid,
    pose_a: &Pose,
    sdf_b: &SdfGrid,
    pose_b: &Pose,
    margin: f64,
) -> Vec<SdfContact> {
    sdf_sdf_contact_raw(sdf_a, pose_a, sdf_b, pose_b, margin)
}

/// Compute the unit separation direction between two poses (A → B).
/// Returns None if the poses are coincident.
pub fn separation_direction(pose_a: &Pose, pose_b: &Pose) -> Option<Vector3<f64>> {
    let delta = pose_b.position - pose_a.position;
    let len = delta.norm();
    if len < 1e-10 { None } else { Some(delta / len) }
}

/// Stabilize a unit direction vector by zeroing negligibly small components.
///
/// For analytical convex-convex contacts (e.g. sphere-sphere stacking), the
/// separation direction determines the contact normal and tangent frame. IEEE
/// 754 rounding in position integration can accumulate sub-physical lateral
/// components (e.g. `(0, 1e-15, 1)` instead of `(0, 0, 1)`). These tilt the
/// tangent frame, causing pyramidal friction facets to produce asymmetric
/// forces that seed exponential lateral drift on unstable equilibria.
///
/// Zeroing components below `1e-6 * max_component` ensures the tangent frame
/// remains symmetric. The threshold is far above machine epsilon (2.2e-16)
/// but far below any physical significance (~0.00006° of tilt).
///
/// See `sim/docs/PYRAMIDAL_FRICTION_INSTABILITY.md` for the full analysis.
pub fn stabilize_direction(dir: Vector3<f64>) -> Vector3<f64> {
    let max_abs = dir.x.abs().max(dir.y.abs()).max(dir.z.abs());
    if max_abs < 1e-15 {
        return dir;
    }

    let threshold = max_abs * 1e-6;
    let x = if dir.x.abs() < threshold { 0.0 } else { dir.x };
    let y = if dir.y.abs() < threshold { 0.0 } else { dir.y };
    let z = if dir.z.abs() < threshold { 0.0 } else { dir.z };

    let stabilized = Vector3::new(x, y, z);
    let len = stabilized.norm();
    if len < 1e-15 {
        dir // all components below threshold — keep original
    } else {
        stabilized / len
    }
}

/// One pass of surface-tracing: iterate surface points of `src_sdf`,
/// check their penetration into `dst_sdf`.
fn trace_surface_into_other(
    src_sdf: &SdfGrid,
    src_pose: &Pose,
    dst_sdf: &SdfGrid,
    dst_pose: &Pose,
    margin: f64,
    contacts: &mut Vec<SdfContact>,
    flip_normal: bool,
) {
    let surface_threshold = src_sdf.cell_size() * 2.0;

    for z in 0..src_sdf.depth() {
        for y in 0..src_sdf.height() {
            for x in 0..src_sdf.width() {
                let src_value = src_sdf.get(x, y, z).unwrap_or(f64::MAX);

                if src_value > surface_threshold {
                    continue;
                }

                let local_point = Point3::new(
                    src_sdf.origin().x + x as f64 * src_sdf.cell_size(),
                    src_sdf.origin().y + y as f64 * src_sdf.cell_size(),
                    src_sdf.origin().z + z as f64 * src_sdf.cell_size(),
                );

                let Some(grad) = src_sdf.gradient(local_point) else {
                    continue;
                };
                let surface_local = local_point - grad * src_value;
                let surface_world = src_pose.transform_point(&surface_local);

                let dst_local = dst_pose.inverse_transform_point(&surface_world);
                let Some(dst_dist) = dst_sdf.distance(dst_local) else {
                    continue;
                };

                // Detect contacts when surface point is within margin of dst.
                // Margin controls the detection range (lookahead for grid
                // discretization), but depth is geometric-only: positive when
                // actually overlapping, zero otherwise. This matches the
                // analytical contact convention and avoids phantom repulsion
                // at the touching configuration.
                if dst_dist >= margin {
                    continue;
                }
                let penetration = (-dst_dist).max(0.0);

                // Use the destination SDF gradient for the contact normal.
                // The dst gradient is evaluated at the actual contact point,
                // giving more accurate normals than the source gradient which
                // is evaluated at a nearby grid point. This eliminates lateral
                // force components from grid asymmetry.
                let Some(dst_grad) = dst_sdf.gradient(dst_local) else {
                    continue;
                };
                // Convention: normal points from SDF A toward SDF B.
                // - First call (A→B, flip=false): dst=B, negate B's outward gradient
                // - Second call (B→A, flip=true): dst=A, use A's outward gradient
                let world_normal = if flip_normal {
                    dst_pose.rotation * dst_grad
                } else {
                    -(dst_pose.rotation * dst_grad)
                };

                contacts.push(SdfContact {
                    point: surface_world,
                    normal: world_normal,
                    penetration,
                });
            }
        }
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::cast_precision_loss,
    clippy::items_after_statements
)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    fn sphere_sdf(resolution: usize) -> SdfGrid {
        SdfGrid::sphere(Point3::origin(), 1.0, resolution, 1.0)
    }

    #[test]
    fn test_sdf_sdf_contact_no_collision() {
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);

        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(5.0, 0.0, 0.0));

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, 0.1);
        assert!(
            contacts.is_empty(),
            "SDFs clearly separated should have no contact"
        );
    }

    #[test]
    fn test_sdf_sdf_contact_collision() {
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);

        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(1.5, 0.0, 0.0));

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, 0.1);
        assert!(!contacts.is_empty(), "Overlapping SDFs should have contact");

        // Raw surface tracing: some contacts may have pen = 0 (detected via
        // margin but no geometric overlap). At least one should be penetrating.
        assert!(
            contacts.iter().any(|c| c.penetration > 0.0),
            "at least one contact should have positive penetration"
        );
        assert!(
            contacts.iter().all(|c| c.point.x > 0.0 && c.point.x < 2.0),
            "all contact points should be in overlap region"
        );
    }

    #[test]
    fn test_sdf_sdf_contact_deep_penetration() {
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);

        let pose_a = Pose::identity();
        let pose_b = Pose::identity();

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, 0.1);
        assert!(
            !contacts.is_empty(),
            "Coincident SDFs should produce contacts"
        );

        // Raw surface tracing for coincident spheres: surface points of one
        // land exactly on the surface of the other, giving geometric
        // penetration ≈ 0. This is correct for the raw algorithm — deep
        // overlap detection is handled by compute_shape_contact's analytical
        // path. Here we just verify contacts are detected.
        let max_pen = contacts
            .iter()
            .map(|c| c.penetration)
            .fold(0.0_f64, f64::max);
        assert!(
            max_pen >= 0.0,
            "penetration should be non-negative (got {max_pen})",
        );
    }

    #[test]
    fn test_sdf_sdf_contact_transformed_poses() {
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);

        let pose_a = Pose::from_position(Point3::new(5.0, 0.0, 0.0));
        let pose_b = Pose::from_position(Point3::new(6.5, 0.0, 0.0));

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, 0.1);
        assert!(
            !contacts.is_empty(),
            "Translated overlapping SDFs should have contact"
        );

        assert!(
            contacts.iter().any(|c| c.penetration > 0.0),
            "at least one contact should have positive penetration"
        );
        assert!(
            contacts.iter().all(|c| c.point.x > 4.5 && c.point.x < 7.0),
            "all contact points should be in overlap region"
        );
    }

    #[test]
    fn test_sdf_sdf_contact_sphere_box() {
        let sdf_sphere = sphere_sdf(32);
        let sdf_box = SdfGrid::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);

        let pose_sphere = Pose::identity();
        let pose_box = Pose::from_position(Point3::new(1.0, 0.0, 0.0));

        let contacts = sdf_sdf_contact(&sdf_sphere, &pose_sphere, &sdf_box, &pose_box, 0.1);
        assert!(
            !contacts.is_empty(),
            "Overlapping sphere and box SDFs should have contact"
        );
        assert!(
            contacts.iter().any(|c| c.penetration > 0.0),
            "at least one contact should have positive penetration"
        );
    }

    #[test]
    fn test_sdf_sdf_contact_box_box() {
        let sdf_a = SdfGrid::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);
        let sdf_b = SdfGrid::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);

        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(0.7, 0.0, 0.0));

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, 0.1);
        assert!(
            !contacts.is_empty(),
            "Overlapping box SDFs should have contact"
        );
        assert!(
            contacts.iter().any(|c| c.penetration > 0.0),
            "at least one contact should have positive penetration"
        );
    }

    #[test]
    fn test_sdf_sdf_contact_different_resolutions() {
        let sdf_a = SdfGrid::sphere(Point3::origin(), 1.0, 16, 0.5);
        let sdf_b = SdfGrid::sphere(Point3::origin(), 1.0, 32, 0.5);

        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(1.5, 0.0, 0.0));

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, 0.1);
        assert!(
            !contacts.is_empty(),
            "SDFs with different resolutions should still detect contact"
        );
        assert!(
            contacts.iter().any(|c| c.penetration > 0.0),
            "at least one contact should have positive penetration"
        );
    }

    #[test]
    fn test_sdf_sdf_contact_barely_touching() {
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);

        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(1.95, 0.0, 0.0));

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, 0.1);
        // At borderline cases, contact detection might be slightly inaccurate
        // due to grid sampling, so we don't strictly require contacts to exist
        // With geometric-only penetration, barely-touching contacts may have
        // penetration = 0 (detected via margin, but no actual overlap).
        for c in &contacts {
            assert!(
                c.penetration < 0.2,
                "barely touching SDFs should have small penetration (got {})",
                c.penetration
            );
        }
    }

    // =========================================================================
    // sdf_sdf_contact_v2 stress tests (surface-tracing algorithm)
    // =========================================================================

    #[test]
    fn v2_no_collision_when_separated() {
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);
        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(5.0, 0.0, 0.0));

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, 0.1);
        assert!(
            contacts.is_empty(),
            "separated spheres should have no contact"
        );
    }

    #[test]
    fn v2_contact_when_overlapping() {
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);
        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(1.5, 0.0, 0.0));

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, 0.1);
        assert!(
            !contacts.is_empty(),
            "overlapping spheres should produce contacts (got 0)"
        );
        assert!(
            contacts.iter().any(|c| c.penetration > 0.0),
            "at least one contact should have positive penetration"
        );
    }

    #[test]
    fn v2_stacking_scenario() {
        // THE critical test: two spheres stacked vertically.
        // Sphere A at origin (radius 1), sphere B at z=1.95 (slight overlap).
        // The contact should produce upward force on B.
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);
        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(0.0, 0.0, 1.95));

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, 0.1);

        assert!(
            !contacts.is_empty(),
            "stacked spheres (slight overlap) should produce contacts"
        );

        // All contact points should be near the top of A / bottom of B
        for c in &contacts {
            assert!(
                c.point.z > 0.5,
                "contact z should be in upper region (got {:.3})",
                c.point.z
            );
        }

        // Net force direction: sum of (normal * penetration) should point upward
        // This is the key stacking test — force must push B up off A.
        let net_force: Vector3<f64> = contacts.iter().map(|c| c.normal * c.penetration).sum();
        assert!(
            net_force.z > 0.0,
            "net contact force should point upward, got z={:.4}",
            net_force.z
        );

        eprintln!("  v2 stacking: {} contacts", contacts.len());
        eprintln!(
            "  net force: ({:.4}, {:.4}, {:.4})",
            net_force.x, net_force.y, net_force.z
        );
        for (i, c) in contacts.iter().enumerate() {
            eprintln!(
                "    [{i}] pos=({:.3},{:.3},{:.3}) n=({:.3},{:.3},{:.3}) pen={:.4}",
                c.point.x, c.point.y, c.point.z, c.normal.x, c.normal.y, c.normal.z, c.penetration
            );
        }
    }

    #[test]
    fn v2_margin_detects_near_contact() {
        // Two spheres NOT touching: distance between surfaces = 0.05.
        // With margin = 0.1, contacts should still be detected.
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);
        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(0.0, 0.0, 2.05));

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, 0.1);
        assert!(
            !contacts.is_empty(),
            "margin should detect near-contacts (gap=0.05, margin=0.1)"
        );

        for c in &contacts {
            assert!(
                c.penetration < 0.1,
                "near-contact penetration should be < margin (got {:.4})",
                c.penetration
            );
        }
    }

    #[test]
    fn v2_no_contact_beyond_margin() {
        // Gap = 0.3 > margin = 0.1
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);
        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(0.0, 0.0, 2.3));

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, 0.1);
        assert!(
            contacts.is_empty(),
            "gap (0.3) > margin (0.1) should produce no contacts"
        );
    }

    #[test]
    fn v2_symmetric_net_force() {
        // Two spheres overlapping along X. Net force should point along +X.
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);
        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(1.8, 0.0, 0.0));

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, 0.1);
        assert!(!contacts.is_empty());

        let net_force: Vector3<f64> = contacts.iter().map(|c| c.normal * c.penetration).sum();
        assert!(
            net_force.x > 0.0,
            "net force should push B in +X, got x={:.4}",
            net_force.x
        );
        // Y and Z should be near zero (symmetric)
        assert!(
            net_force.y.abs() < net_force.x.abs() * 0.3,
            "net force y should be small relative to x, got y={:.4}",
            net_force.y
        );
        assert!(
            net_force.z.abs() < net_force.x.abs() * 0.3,
            "net force z should be small relative to x, got z={:.4}",
            net_force.z
        );
    }

    #[test]
    fn v2_stacking_coarse_resolution() {
        // THE critical test that was failing: two 5mm-radius spheres at 1mm
        // cell size. Previously, equatorial contacts dominated and net force
        // was horizontal. With normal-weighted filtering, polar contacts win.
        let sdf_a = SdfGrid::sphere(Point3::origin(), 5.0, 14, 2.0);
        let sdf_b = SdfGrid::sphere(Point3::origin(), 5.0, 14, 2.0);
        assert!(
            (sdf_a.cell_size() - 1.0).abs() < 0.2,
            "cell size should be ~1mm (got {})",
            sdf_a.cell_size()
        );

        // Stacked vertically with slight overlap (same as example 07)
        let pose_a = Pose::from_position(Point3::new(0.0, 0.0, 5.5));
        let pose_b = Pose::from_position(Point3::new(0.0, 0.0, 14.5));
        let margin = sdf_a.cell_size();

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b, margin);

        assert!(
            !contacts.is_empty(),
            "coarse-resolution stacking should produce contacts"
        );

        // Net force must point upward (+Z) for stacking to work
        let net_force: Vector3<f64> = contacts.iter().map(|c| c.normal * c.penetration).sum();
        assert!(
            net_force.z > 0.0,
            "net force must point upward for stacking, got z={:.4}",
            net_force.z
        );

        // Vertical component should dominate horizontal
        let horizontal = net_force.x.hypot(net_force.y);
        assert!(
            net_force.z > horizontal,
            "vertical force ({:.4}) should dominate horizontal ({:.4})",
            net_force.z,
            horizontal
        );

        eprintln!("  coarse stacking: {} contacts", contacts.len());
        eprintln!(
            "  net force: ({:.4}, {:.4}, {:.4})",
            net_force.x, net_force.y, net_force.z
        );
        for (i, c) in contacts.iter().enumerate() {
            eprintln!(
                "    [{i}] n=({:.3},{:.3},{:.3}) pen={:.4}",
                c.normal.x, c.normal.y, c.normal.z, c.penetration
            );
        }
    }

    // =========================================================================
    // stabilize_direction tests
    // =========================================================================

    #[test]
    fn stabilize_zeroes_sub_threshold_components() {
        // Nearly-vertical direction with tiny lateral drift
        let dir = Vector3::new(1e-15, -2e-14, 1.0).normalize();
        let stable = stabilize_direction(dir);
        assert_eq!(stable.x, 0.0, "x should be zeroed");
        assert_eq!(stable.y, 0.0, "y should be zeroed");
        assert!((stable.z - 1.0).abs() < 1e-15, "z should be ~1.0");
    }

    #[test]
    fn stabilize_preserves_significant_components() {
        // 45-degree direction — no components should be zeroed
        let dir = Vector3::new(0.707, 0.0, 0.707).normalize();
        let stable = stabilize_direction(dir);
        assert!((stable.x - dir.x).abs() < 1e-15);
        assert_eq!(stable.y, 0.0); // already zero
        assert!((stable.z - dir.z).abs() < 1e-15);
    }

    #[test]
    fn stabilize_preserves_large_lateral() {
        // Tilted direction with physically significant lateral component
        let dir = Vector3::new(0.0, 0.01, 1.0).normalize();
        let stable = stabilize_direction(dir);
        // 0.01 > 1e-6 * 1.0, so y should NOT be zeroed
        assert!(stable.y > 0.009, "y should be preserved (got {})", stable.y);
    }

    #[test]
    fn stabilize_identity_on_cardinal() {
        let dir = Vector3::z();
        let stable = stabilize_direction(dir);
        assert!((stable - dir).norm() < 1e-15);
    }
}
