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

/// Query two SDFs for contact using surface-tracing.
///
/// Traces from the **surface** of each SDF into the other. A surface point on
/// SDF A is a contact if it lies within `margin` of SDF B's interior. This
/// produces focused contact patches with physically meaningful penetration
/// depths and correct normals.
///
/// `margin` — contacts are detected when a surface point is within this
/// distance of the other SDF's interior. A positive margin allows
/// pre-penetration detection. Recommended: `cell_size * 1.0`.
///
/// Returns multiple contacts (up to 20) to distribute constraint forces
/// across the contact patch.
#[must_use]
pub fn sdf_sdf_contact(
    sdf_a: &SdfGrid,
    pose_a: &Pose,
    sdf_b: &SdfGrid,
    pose_b: &Pose,
    margin: f64,
) -> Vec<SdfContact> {
    let mut contacts: Vec<SdfContact> = Vec::new();

    // Trace from A's surface into B
    trace_surface_into_other(sdf_a, pose_a, sdf_b, pose_b, margin, &mut contacts, false);
    // Trace from B's surface into A
    trace_surface_into_other(sdf_b, pose_b, sdf_a, pose_a, margin, &mut contacts, true);

    // Normal-weighted filtering: prefer contacts whose normals align with the
    // separation direction. At coarse resolution, equatorial contacts (normals
    // perpendicular to separation) dominate but produce zero separation force.
    // Drop them so the polar contacts (which actually push bodies apart) win.
    let sep_dir = separation_direction(pose_a, pose_b);
    if let Some(dir) = sep_dir {
        // Score = penetration × |normal · separation_dir|
        // This weights contacts by both depth AND alignment with separation.
        contacts.retain(|c| c.normal.dot(&dir).abs() > MIN_NORMAL_ALIGNMENT);
        contacts.sort_by(|a, b| {
            let score_a = a.penetration * a.normal.dot(&dir).abs();
            let score_b = b.penetration * b.normal.dot(&dir).abs();
            score_b
                .partial_cmp(&score_a)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    } else {
        // Coincident poses — no separation direction. Sort by penetration only.
        contacts.sort_by(|a, b| {
            b.penetration
                .partial_cmp(&a.penetration)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    contacts.truncate(MAX_SDF_SDF_CONTACTS);
    contacts
}

/// Maximum number of contacts returned by `sdf_sdf_contact`.
const MAX_SDF_SDF_CONTACTS: usize = 20;

/// Minimum |normal · separation_dir| to keep a contact. Contacts below this
/// threshold have normals nearly perpendicular to the separation axis and
/// contribute negligible separation force.
const MIN_NORMAL_ALIGNMENT: f64 = 0.1;

/// Compute the unit separation direction between two poses (A → B).
/// Returns None if the poses are coincident.
fn separation_direction(pose_a: &Pose, pose_b: &Pose) -> Option<Vector3<f64>> {
    let delta = pose_b.position - pose_a.position;
    let len = delta.norm();
    if len < 1e-10 { None } else { Some(delta / len) }
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
    let dedup_dist_sq = (src_sdf.cell_size() * 0.5) * (src_sdf.cell_size() * 0.5);

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

                // Contact if surface point is within margin of dst's interior.
                // Report margin-inclusive depth so the solver's position correction
                // is non-zero even before full geometric penetration.
                let penetration = margin - dst_dist;
                if penetration <= 0.0 {
                    continue;
                }

                let world_normal = if flip_normal {
                    -(src_pose.rotation * grad)
                } else {
                    src_pose.rotation * grad
                };

                let dominated = contacts
                    .iter()
                    .any(|c| (c.point - surface_world).norm_squared() < dedup_dist_sq);

                if !dominated {
                    contacts.push(SdfContact {
                        point: surface_world,
                        normal: world_normal,
                        penetration,
                    });
                }
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

        let c = &contacts[0];
        assert!(c.penetration > 0.0, "should have positive penetration");
        assert!(
            c.point.x > 0.0 && c.point.x < 2.0,
            "contact point should be in overlap region"
        );
        // Multi-contact: overlapping spheres should produce multiple contacts
        assert!(
            contacts.len() > 1,
            "overlapping spheres should produce multiple contacts (got {})",
            contacts.len()
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

        // With surface projection, coincident spheres have small surface penetration
        // (surface of one lands on/near surface of the other). This is physically
        // correct — the overlap is total but the surface-to-surface penetration is
        // small near the grid resolution.
        let c = &contacts[0];
        assert!(
            c.penetration > 0.0,
            "should have positive penetration (got {})",
            c.penetration
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

        let c = &contacts[0];
        assert!(c.penetration > 0.0);
        assert!(
            c.point.x > 4.5 && c.point.x < 7.0,
            "contact point should be in overlap region"
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
        assert!(contacts[0].penetration > 0.0);
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
        assert!(contacts[0].penetration > 0.0);
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
        assert!(contacts[0].penetration > 0.0);
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
        assert!(contacts[0].penetration > 0.0);
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
}
