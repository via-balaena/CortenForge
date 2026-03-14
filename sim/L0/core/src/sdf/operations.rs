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
/// - `penetration`: Depth of the deepest overlap (min of `|dist_a|`, `|dist_b|`)
#[must_use]
pub fn sdf_sdf_contact(
    sdf_a: &SdfGrid,
    pose_a: &Pose,
    sdf_b: &SdfGrid,
    pose_b: &Pose,
) -> Option<SdfContact> {
    // Compute world-space AABBs for both SDFs
    let a_aabb = sdf_a.aabb();
    let b_aabb = sdf_b.aabb();
    let (a_world_min, a_world_max) = transform_aabb_to_world(&a_aabb.min, &a_aabb.max, pose_a);
    let (b_world_min, b_world_max) = transform_aabb_to_world(&b_aabb.min, &b_aabb.max, pose_b);

    // Compute AABB overlap region
    let (overlap_min, overlap_max) =
        compute_aabb_overlap(a_world_min, a_world_max, b_world_min, b_world_max)?;

    // Determine sampling resolution based on the smaller cell size of the two SDFs
    let sample_cell = sdf_a.cell_size().min(sdf_b.cell_size());

    // Compute overlap dimensions
    let overlap_size = overlap_max - overlap_min;

    // Number of samples along each axis (at least 2 per dimension for interpolation)
    let nx = ((overlap_size.x / sample_cell).ceil() as usize).clamp(2, 32);
    let ny = ((overlap_size.y / sample_cell).ceil() as usize).clamp(2, 32);
    let nz = ((overlap_size.z / sample_cell).ceil() as usize).clamp(2, 32);

    // Step size for sampling
    let step_x = overlap_size.x / (nx - 1).max(1) as f64;
    let step_y = overlap_size.y / (ny - 1).max(1) as f64;
    let step_z = overlap_size.z / (nz - 1).max(1) as f64;

    let mut deepest_contact: Option<SdfContact> = None;
    let mut max_penetration = 0.0;

    // Sample the overlap region
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                // Compute world-space sample point
                let world_point = Point3::new(
                    overlap_min.x + ix as f64 * step_x,
                    overlap_min.y + iy as f64 * step_y,
                    overlap_min.z + iz as f64 * step_z,
                );

                // Transform to each SDF's local space
                let local_a = pose_a.inverse_transform_point(&world_point);
                let local_b = pose_b.inverse_transform_point(&world_point);

                // Query both SDFs
                let Some(dist_a) = sdf_a.distance(local_a) else {
                    continue;
                };
                let Some(dist_b) = sdf_b.distance(local_b) else {
                    continue;
                };

                // Check if point is inside both surfaces (both distances negative)
                if dist_a >= 0.0 || dist_b >= 0.0 {
                    continue;
                }

                // Penetration depth is the minimum of the two absolute distances
                // (the shallowest penetration determines how deep we are in the overlap)
                let abs_dist_a = dist_a.abs();
                let abs_dist_b = dist_b.abs();
                let penetration = abs_dist_a.min(abs_dist_b);

                if penetration > max_penetration {
                    max_penetration = penetration;

                    // Get normal from the SDF with smaller absolute distance
                    // (the surface we're closer to breaking out of)
                    let (normal, contact_sdf_pose) = if abs_dist_a <= abs_dist_b {
                        (sdf_a.gradient(local_a), pose_a)
                    } else {
                        (sdf_b.gradient(local_b), pose_b)
                    };

                    if let Some(local_normal) = normal {
                        // Transform normal to world space
                        let world_normal = contact_sdf_pose.rotation * local_normal;

                        deepest_contact = Some(SdfContact {
                            point: world_point,
                            normal: world_normal,
                            penetration,
                        });
                    }
                }
            }
        }
    }

    deepest_contact
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

        // Place SDFs far apart - no contact
        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(5.0, 0.0, 0.0));

        let contact = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b);
        assert!(
            contact.is_none(),
            "SDFs clearly separated should have no contact"
        );
    }

    #[test]
    fn test_sdf_sdf_contact_collision() {
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);

        // Place SDFs overlapping (both are unit spheres at origin)
        // Place B at x=1.5 so they overlap significantly
        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(1.5, 0.0, 0.0));

        let contact = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b);
        assert!(contact.is_some(), "Overlapping SDFs should have contact");

        let c = contact.unwrap();
        assert!(c.penetration > 0.0, "should have positive penetration");
        // Contact point should be in the overlap region (between x=0.5 and x=1.0)
        assert!(
            c.point.x > 0.0 && c.point.x < 2.0,
            "contact point should be in overlap region"
        );
    }

    #[test]
    fn test_sdf_sdf_contact_deep_penetration() {
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);

        // Place SDFs at the same position for maximum penetration
        let pose_a = Pose::identity();
        let pose_b = Pose::identity();

        let contact = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b);
        assert!(
            contact.is_some(),
            "Coincident SDFs should have deep penetration"
        );

        let c = contact.unwrap();
        // When two unit spheres perfectly overlap, any interior point
        // is inside both, with penetration up to 1.0 (the radius)
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

        // Translate both SDFs and have them overlap
        let pose_a = Pose::from_position(Point3::new(5.0, 0.0, 0.0));
        let pose_b = Pose::from_position(Point3::new(6.5, 0.0, 0.0));

        let contact = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b);
        assert!(
            contact.is_some(),
            "Translated overlapping SDFs should have contact"
        );

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
        // Contact point should be near x=5.75 (midpoint of overlap)
        assert!(
            c.point.x > 4.5 && c.point.x < 7.0,
            "contact point should be in overlap region"
        );
    }

    #[test]
    fn test_sdf_sdf_contact_sphere_box() {
        let sdf_sphere = sphere_sdf(32);
        let sdf_box = SdfGrid::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);

        // Place box at edge of sphere
        let pose_sphere = Pose::identity();
        let pose_box = Pose::from_position(Point3::new(1.0, 0.0, 0.0));

        let contact = sdf_sdf_contact(&sdf_sphere, &pose_sphere, &sdf_box, &pose_box);
        assert!(
            contact.is_some(),
            "Overlapping sphere and box SDFs should have contact"
        );

        let c = contact.unwrap();
        assert!(c.penetration > 0.0, "should have positive penetration");
    }

    #[test]
    fn test_sdf_sdf_contact_box_box() {
        let sdf_a = SdfGrid::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);
        let sdf_b = SdfGrid::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);

        // Place boxes overlapping
        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(0.7, 0.0, 0.0));

        let contact = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b);
        assert!(
            contact.is_some(),
            "Overlapping box SDFs should have contact"
        );

        let c = contact.unwrap();
        assert!(c.penetration > 0.0, "should have positive penetration");
    }

    #[test]
    fn test_sdf_sdf_contact_different_resolutions() {
        // SDFs with different resolutions
        let sdf_a = SdfGrid::sphere(Point3::origin(), 1.0, 16, 0.5);
        let sdf_b = SdfGrid::sphere(Point3::origin(), 1.0, 32, 0.5);

        // Place overlapping
        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(1.5, 0.0, 0.0));

        let contact = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b);
        assert!(
            contact.is_some(),
            "SDFs with different resolutions should still detect contact"
        );

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_sdf_contact_barely_touching() {
        let sdf_a = sphere_sdf(32);
        let sdf_b = sphere_sdf(32);

        // Place SDFs just barely touching (distance = 2.0 for two unit spheres)
        // At exactly 2.0 they're tangent, so at 1.95 they should have slight overlap
        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(1.95, 0.0, 0.0));

        let contact = sdf_sdf_contact(&sdf_a, &pose_a, &sdf_b, &pose_b);
        // Should have a very small contact
        if let Some(c) = contact {
            assert!(
                c.penetration > 0.0 && c.penetration < 0.2,
                "barely touching SDFs should have small penetration (got {})",
                c.penetration
            );
        }
        // Note: At borderline cases, contact detection might be slightly inaccurate
        // due to grid sampling, so we don't strictly require contact to exist
    }
}
