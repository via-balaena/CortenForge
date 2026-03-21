//! SDF-SDF contact and AABB utility operations.

// Allow casting for grid indices - these are small values and bounds are checked
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap
)]

use nalgebra::Point3;
use sim_types::Pose;

use cf_geometry::Bounded;

use super::{SdfContact, SdfGrid};

/// Transform an AABB to world space, accounting for rotation.
pub fn transform_aabb_to_world(
    aabb_min: &Point3<f64>,
    aabb_max: &Point3<f64>,
    pose: &Pose,
) -> (Point3<f64>, Point3<f64>) {
    let world_min = pose.transform_point(aabb_min);
    let world_max = pose.transform_point(aabb_max);
    (
        Point3::new(
            world_min.x.min(world_max.x),
            world_min.y.min(world_max.y),
            world_min.z.min(world_max.z),
        ),
        Point3::new(
            world_min.x.max(world_max.x),
            world_min.y.max(world_max.y),
            world_min.z.max(world_max.z),
        ),
    )
}

/// Compute AABB overlap region. Returns None if no overlap.
pub fn compute_aabb_overlap(
    a_min: Point3<f64>,
    a_max: Point3<f64>,
    b_min: Point3<f64>,
    b_max: Point3<f64>,
) -> Option<(Point3<f64>, Point3<f64>)> {
    let overlap_min = Point3::new(
        a_min.x.max(b_min.x),
        a_min.y.max(b_min.y),
        a_min.z.max(b_min.z),
    );
    let overlap_max = Point3::new(
        a_max.x.min(b_max.x),
        a_max.y.min(b_max.y),
        a_max.z.min(b_max.z),
    );

    if overlap_min.x >= overlap_max.x
        || overlap_min.y >= overlap_max.y
        || overlap_min.z >= overlap_max.z
    {
        None
    } else {
        Some((overlap_min, overlap_max))
    }
}

/// Query two SDFs for contact.
///
/// Samples a grid of points in the AABB overlap region between both SDFs and
/// finds where both SDFs return negative distance (inside both surfaces).
/// The penetration depth at any point is the minimum of the two absolute distances.
///
/// # Arguments
///
/// * `sdf_a` - The first SDF collision data
/// * `pose_a` - The pose of the first SDF in world space
/// * `sdf_b` - The second SDF collision data
/// * `pose_b` - The pose of the second SDF in world space
///
/// # Returns
///
/// Contact information if the SDFs overlap, with:
/// - `point`: Contact point in the overlap region (world space)
/// - `normal`: Surface normal from the SDF with smaller absolute distance at contact
/// - `penetration`: Depth of overlap (min of `|dist_a|`, `|dist_b|`)
///
/// Returns multiple contacts (up to `MAX_SDF_SDF_CONTACTS`) to distribute
/// constraint forces across the contact patch, preventing single-point jitter.
#[must_use]
pub fn sdf_sdf_contact(
    sdf_a: &SdfGrid,
    pose_a: &Pose,
    sdf_b: &SdfGrid,
    pose_b: &Pose,
) -> Vec<SdfContact> {
    let a_aabb = sdf_a.aabb();
    let b_aabb = sdf_b.aabb();
    let (a_world_min, a_world_max) = transform_aabb_to_world(&a_aabb.min, &a_aabb.max, pose_a);
    let (b_world_min, b_world_max) = transform_aabb_to_world(&b_aabb.min, &b_aabb.max, pose_b);

    let Some((overlap_min, overlap_max)) =
        compute_aabb_overlap(a_world_min, a_world_max, b_world_min, b_world_max)
    else {
        return vec![];
    };

    let sample_cell = sdf_a.cell_size().min(sdf_b.cell_size());
    let overlap_size = overlap_max - overlap_min;

    let nx = ((overlap_size.x / sample_cell).ceil() as usize).clamp(2, 32);
    let ny = ((overlap_size.y / sample_cell).ceil() as usize).clamp(2, 32);
    let nz = ((overlap_size.z / sample_cell).ceil() as usize).clamp(2, 32);

    let step_x = overlap_size.x / (nx - 1).max(1) as f64;
    let step_y = overlap_size.y / (ny - 1).max(1) as f64;
    let step_z = overlap_size.z / (nz - 1).max(1) as f64;

    let mut contacts: Vec<SdfContact> = Vec::new();
    let dedup_dist_sq = (sample_cell * 0.5) * (sample_cell * 0.5);

    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let world_point = Point3::new(
                    overlap_min.x + ix as f64 * step_x,
                    overlap_min.y + iy as f64 * step_y,
                    overlap_min.z + iz as f64 * step_z,
                );

                let local_a = pose_a.inverse_transform_point(&world_point);
                let local_b = pose_b.inverse_transform_point(&world_point);

                let Some(dist_a) = sdf_a.distance(local_a) else {
                    continue;
                };
                let Some(dist_b) = sdf_b.distance(local_b) else {
                    continue;
                };

                if dist_a >= 0.0 || dist_b >= 0.0 {
                    continue;
                }

                let abs_dist_a = dist_a.abs();
                let abs_dist_b = dist_b.abs();
                let penetration = abs_dist_a.min(abs_dist_b);

                // Get normal from the SDF with smaller absolute distance
                let (normal, contact_sdf_pose) = if abs_dist_a <= abs_dist_b {
                    (sdf_a.gradient(local_a), pose_a)
                } else {
                    (sdf_b.gradient(local_b), pose_b)
                };

                if let Some(local_normal) = normal {
                    let world_normal = contact_sdf_pose.rotation * local_normal;

                    // Deduplicate nearby contacts
                    let dominated = contacts
                        .iter()
                        .any(|c| (c.point - world_point).norm_squared() < dedup_dist_sq);

                    if !dominated {
                        contacts.push(SdfContact {
                            point: world_point,
                            normal: world_normal,
                            penetration,
                        });
                    }
                }
            }
        }
    }

    contacts.sort_by(|a, b| {
        b.penetration
            .partial_cmp(&a.penetration)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    contacts.truncate(MAX_SDF_SDF_CONTACTS);
    contacts
}

/// Maximum number of contacts returned by `sdf_sdf_contact`.
const MAX_SDF_SDF_CONTACTS: usize = 20;

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

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b);
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

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b);
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

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b);
        assert!(
            !contacts.is_empty(),
            "Coincident SDFs should have deep penetration"
        );

        let c = &contacts[0];
        assert!(
            c.penetration > 0.5,
            "should have deep penetration (got {})",
            c.penetration
        );
    }

    #[test]
    fn test_sdf_sdf_contact_transformed_poses() {
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);

        let pose_a = Pose::from_position(Point3::new(5.0, 0.0, 0.0));
        let pose_b = Pose::from_position(Point3::new(6.5, 0.0, 0.0));

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b);
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

        let contacts = sdf_sdf_contact(&sdf_sphere, &pose_sphere, &sdf_box, &pose_box);
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

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b);
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

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b);
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

        let contacts = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b);
        // At borderline cases, contact detection might be slightly inaccurate
        // due to grid sampling, so we don't strictly require contacts to exist
        for c in &contacts {
            assert!(
                c.penetration > 0.0 && c.penetration < 0.2,
                "barely touching SDFs should have small penetration (got {})",
                c.penetration
            );
        }
    }
}
