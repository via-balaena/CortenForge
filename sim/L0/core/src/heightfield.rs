//! Height field collision queries for terrain simulation.
//!
//! The height field data type ([`HeightFieldData`]) is provided by `cf-geometry`.
//! This module provides physics-layer contact queries against height fields.
//!
//! # Contact Queries
//!
//! - [`heightfield_sphere_contact`] — sphere vs terrain
//! - [`heightfield_point_contact`] — point vs terrain
//! - [`heightfield_capsule_contact`] — capsule vs terrain
//! - [`heightfield_box_contact`] — box vs terrain
//!
//! Contact query functions use safe fallbacks for cell indices:
//! `cell_at(...).unwrap_or((0, 0))` provides a valid cell index even when the
//! sampled point is at the exact boundary.

// Allow casting for grid indices - these are small values and bounds are checked
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap
)]

pub use cf_geometry::HeightFieldData;
use nalgebra::{Point3, Vector3};
use sim_types::Pose;

/// Result of a height field contact query.
#[derive(Debug, Clone)]
pub struct HeightFieldContact {
    /// Contact point on the height field surface (world space).
    pub point: Point3<f64>,
    /// Surface normal at contact point (pointing up from terrain).
    pub normal: Vector3<f64>,
    /// Penetration depth (positive when object is below surface).
    pub penetration: f64,
    /// Cell indices where contact occurred.
    pub cell: (usize, usize),
}

/// Query a height field for contact with a sphere.
///
/// Returns contact information if the sphere penetrates the height field.
///
/// # Arguments
///
/// * `heightfield` - The height field data
/// * `hf_pose` - The pose of the height field in world space
/// * `sphere_center` - Center of the sphere in world space
/// * `sphere_radius` - Radius of the sphere
#[must_use]
pub fn heightfield_sphere_contact(
    heightfield: &HeightFieldData,
    hf_pose: &Pose,
    sphere_center: Point3<f64>,
    sphere_radius: f64,
) -> Option<HeightFieldContact> {
    // Transform sphere center to height field local space
    let local_center = hf_pose.inverse_transform_point(&sphere_center);

    // Check if sphere is within XY bounds (with margin for radius)
    let x = local_center.x;
    let y = local_center.y;

    if x < -sphere_radius || y < -sphere_radius {
        return None;
    }
    if x > heightfield.extent_x() + sphere_radius || y > heightfield.extent_y() + sphere_radius {
        return None;
    }

    // Clamp to height field bounds for sampling
    let sample_x = x.clamp(0.0, heightfield.extent_x());
    let sample_y = y.clamp(0.0, heightfield.extent_y());

    // Get terrain height at sphere position
    let terrain_height = heightfield.sample_clamped(sample_x, sample_y);

    // Compute penetration
    let sphere_bottom = local_center.z - sphere_radius;
    let penetration = terrain_height - sphere_bottom;

    if penetration <= 0.0 {
        return None;
    }

    // Get surface normal
    let local_normal = heightfield.normal_clamped(sample_x, sample_y);

    // Contact point is on the terrain surface
    let local_point = Point3::new(sample_x, sample_y, terrain_height);

    // Get cell indices
    let cell = heightfield.cell_at(sample_x, sample_y).unwrap_or((0, 0));

    // Transform back to world space
    let world_point = hf_pose.transform_point(&local_point);
    let world_normal = hf_pose.rotation * local_normal;

    Some(HeightFieldContact {
        point: world_point,
        normal: world_normal,
        penetration,
        cell,
    })
}

/// Query a height field for contact with a point.
///
/// Returns contact information if the point is below the height field surface.
///
/// # Arguments
///
/// * `heightfield` - The height field data
/// * `hf_pose` - The pose of the height field in world space
/// * `point` - The point to test in world space
#[must_use]
pub fn heightfield_point_contact(
    heightfield: &HeightFieldData,
    hf_pose: &Pose,
    point: Point3<f64>,
) -> Option<HeightFieldContact> {
    // Transform point to height field local space
    let local_point = hf_pose.inverse_transform_point(&point);

    let x = local_point.x;
    let y = local_point.y;

    // Check bounds (use negated comparison to catch NaN)
    if !(x >= 0.0) || !(y >= 0.0) {
        return None;
    }
    if x > heightfield.extent_x() || y > heightfield.extent_y() {
        return None;
    }

    // Get terrain height
    let terrain_height = heightfield.sample(x, y)?;

    // Compute penetration
    let penetration = terrain_height - local_point.z;

    if penetration <= 0.0 {
        return None;
    }

    // Get surface normal
    let local_normal = heightfield.normal(x, y)?;

    // Contact point is on the terrain surface
    let local_contact = Point3::new(x, y, terrain_height);

    // Get cell indices
    let cell = heightfield.cell_at(x, y).unwrap_or((0, 0));

    // Transform back to world space
    let world_point = hf_pose.transform_point(&local_contact);
    let world_normal = hf_pose.rotation * local_normal;

    Some(HeightFieldContact {
        point: world_point,
        normal: world_normal,
        penetration,
        cell,
    })
}

/// Query a height field for contact with a capsule.
///
/// Tests both capsule endpoints and the closest point on the capsule axis.
///
/// # Arguments
///
/// * `heightfield` - The height field data
/// * `hf_pose` - The pose of the height field in world space
/// * `capsule_start` - Start point of capsule axis in world space
/// * `capsule_end` - End point of capsule axis in world space
/// * `capsule_radius` - Radius of the capsule
#[must_use]
pub fn heightfield_capsule_contact(
    heightfield: &HeightFieldData,
    hf_pose: &Pose,
    capsule_start: Point3<f64>,
    capsule_end: Point3<f64>,
    capsule_radius: f64,
) -> Option<HeightFieldContact> {
    // Transform capsule to height field local space
    let local_start = hf_pose.inverse_transform_point(&capsule_start);
    let local_end = hf_pose.inverse_transform_point(&capsule_end);

    // Find the lowest point on the capsule axis
    let axis = local_end - local_start;
    let axis_len = axis.norm();

    if axis_len < 1e-10 {
        // Degenerate capsule - treat as sphere
        return heightfield_sphere_contact(heightfield, hf_pose, capsule_start, capsule_radius);
    }

    // Sample multiple points along the axis for better contact
    let test_points = [
        local_start,
        local_start + axis * 0.25,
        local_start + axis * 0.5,
        local_start + axis * 0.75,
        local_end,
    ];

    let mut deepest_contact: Option<HeightFieldContact> = None;
    let mut max_penetration = 0.0;

    for test_point in &test_points {
        let x = test_point.x;
        let y = test_point.y;

        // Skip if outside bounds
        if x < -capsule_radius || y < -capsule_radius {
            continue;
        }
        if x > heightfield.extent_x() + capsule_radius
            || y > heightfield.extent_y() + capsule_radius
        {
            continue;
        }

        let sample_x = x.clamp(0.0, heightfield.extent_x());
        let sample_y = y.clamp(0.0, heightfield.extent_y());

        let terrain_height = heightfield.sample_clamped(sample_x, sample_y);
        let capsule_bottom = test_point.z - capsule_radius;
        let penetration = terrain_height - capsule_bottom;

        if penetration > max_penetration {
            max_penetration = penetration;
            let local_normal = heightfield.normal_clamped(sample_x, sample_y);
            let local_contact = Point3::new(sample_x, sample_y, terrain_height);
            let cell = heightfield.cell_at(sample_x, sample_y).unwrap_or((0, 0));

            let world_point = hf_pose.transform_point(&local_contact);
            let world_normal = hf_pose.rotation * local_normal;

            deepest_contact = Some(HeightFieldContact {
                point: world_point,
                normal: world_normal,
                penetration,
                cell,
            });
        }
    }

    deepest_contact
}

/// Query a height field for contact with a box.
///
/// Tests all 8 corners of the box against the height field.
///
/// # Arguments
///
/// * `heightfield` - The height field data
/// * `hf_pose` - The pose of the height field in world space
/// * `box_pose` - The pose of the box in world space
/// * `half_extents` - Half-extents of the box
#[must_use]
pub fn heightfield_box_contact(
    heightfield: &HeightFieldData,
    hf_pose: &Pose,
    box_pose: &Pose,
    half_extents: &Vector3<f64>,
) -> Option<HeightFieldContact> {
    // Test all 8 corners of the box
    let corners = [
        Point3::new(-half_extents.x, -half_extents.y, -half_extents.z),
        Point3::new(half_extents.x, -half_extents.y, -half_extents.z),
        Point3::new(-half_extents.x, half_extents.y, -half_extents.z),
        Point3::new(half_extents.x, half_extents.y, -half_extents.z),
        Point3::new(-half_extents.x, -half_extents.y, half_extents.z),
        Point3::new(half_extents.x, -half_extents.y, half_extents.z),
        Point3::new(-half_extents.x, half_extents.y, half_extents.z),
        Point3::new(half_extents.x, half_extents.y, half_extents.z),
    ];

    let mut deepest_contact: Option<HeightFieldContact> = None;
    let mut max_penetration = 0.0;

    for local_corner in &corners {
        let world_corner = box_pose.transform_point(local_corner);
        let hf_local = hf_pose.inverse_transform_point(&world_corner);

        let x = hf_local.x;
        let y = hf_local.y;

        // Skip if outside bounds
        if x < 0.0 || y < 0.0 {
            continue;
        }
        if x > heightfield.extent_x() || y > heightfield.extent_y() {
            continue;
        }

        if let Some(terrain_height) = heightfield.sample(x, y) {
            let penetration = terrain_height - hf_local.z;

            if penetration > max_penetration {
                max_penetration = penetration;
                let local_normal = heightfield.normal_clamped(x, y);
                let local_contact = Point3::new(x, y, terrain_height);
                let cell = heightfield.cell_at(x, y).unwrap_or((0, 0));

                let world_point = hf_pose.transform_point(&local_contact);
                let world_normal = hf_pose.rotation * local_normal;

                deepest_contact = Some(HeightFieldContact {
                    point: world_point,
                    normal: world_normal,
                    penetration,
                    cell,
                });
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
    clippy::cast_precision_loss
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_heightfield_sphere_contact() {
        let hf = HeightFieldData::flat(10, 10, 1.0, 0.0);
        let hf_pose = Pose::identity();

        // Sphere above surface - no contact
        let center = Point3::new(5.0, 5.0, 2.0);
        let contact = heightfield_sphere_contact(&hf, &hf_pose, center, 1.0);
        assert!(contact.is_none());

        // Sphere penetrating surface
        let center = Point3::new(5.0, 5.0, 0.5);
        let contact = heightfield_sphere_contact(&hf, &hf_pose, center, 1.0);
        assert!(contact.is_some());
        let c = contact.unwrap();
        assert_relative_eq!(c.penetration, 0.5, epsilon = 1e-6);
        assert_relative_eq!(c.normal.z, 1.0, epsilon = 1e-3);

        // Sphere outside XY bounds
        let center = Point3::new(-5.0, 5.0, 0.0);
        let contact = heightfield_sphere_contact(&hf, &hf_pose, center, 1.0);
        assert!(contact.is_none());
    }

    #[test]
    fn test_heightfield_sphere_contact_on_slope() {
        // z = 0.5 * x slope
        let hf = HeightFieldData::from_fn(20, 20, 1.0, |x, _y| x * 0.5);
        let hf_pose = Pose::identity();

        // Sphere at x=10 should have terrain height of 5
        let center = Point3::new(10.0, 10.0, 5.5);
        let contact = heightfield_sphere_contact(&hf, &hf_pose, center, 1.0);
        assert!(contact.is_some());
        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
        // Normal should be tilted
        assert!(c.normal.x < 0.0);
        assert!(c.normal.z > 0.0);
    }

    #[test]
    fn test_heightfield_capsule_contact() {
        let hf = HeightFieldData::flat(10, 10, 1.0, 0.0);
        let hf_pose = Pose::identity();

        // Horizontal capsule above surface
        let start = Point3::new(3.0, 5.0, 2.0);
        let end = Point3::new(7.0, 5.0, 2.0);
        let contact = heightfield_capsule_contact(&hf, &hf_pose, start, end, 0.5);
        assert!(contact.is_none());

        // Horizontal capsule penetrating
        let start = Point3::new(3.0, 5.0, 0.3);
        let end = Point3::new(7.0, 5.0, 0.3);
        let contact = heightfield_capsule_contact(&hf, &hf_pose, start, end, 0.5);
        assert!(contact.is_some());
        let c = contact.unwrap();
        assert_relative_eq!(c.penetration, 0.2, epsilon = 1e-6);
    }

    #[test]
    fn test_heightfield_box_contact() {
        let hf = HeightFieldData::flat(10, 10, 1.0, 0.0);
        let hf_pose = Pose::identity();
        let half_extents = Vector3::new(0.5, 0.5, 0.5);

        // Box above surface
        let box_pose = Pose::from_position(Point3::new(5.0, 5.0, 2.0));
        let contact = heightfield_box_contact(&hf, &hf_pose, &box_pose, &half_extents);
        assert!(contact.is_none());

        // Box penetrating surface
        let box_pose = Pose::from_position(Point3::new(5.0, 5.0, 0.3));
        let contact = heightfield_box_contact(&hf, &hf_pose, &box_pose, &half_extents);
        assert!(contact.is_some());
        let c = contact.unwrap();
        assert_relative_eq!(c.penetration, 0.2, epsilon = 1e-6);
    }

    #[test]
    fn test_heightfield_with_pose() {
        let hf = HeightFieldData::flat(5, 5, 1.0, 0.0);

        // Elevate the height field by 10 units
        let hf_pose = Pose::from_position(Point3::new(0.0, 0.0, 10.0));

        // Sphere at z=10.5 with radius 1.0 has bottom at z=9.5, terrain at z=10
        // So penetration should be 10 - 9.5 = 0.5
        let center = Point3::new(2.0, 2.0, 10.5);
        let contact = heightfield_sphere_contact(&hf, &hf_pose, center, 1.0);
        assert!(contact.is_some());
        let c = contact.unwrap();
        assert_relative_eq!(c.penetration, 0.5, epsilon = 1e-6);

        // Contact point should be at z=10 (transformed surface)
        assert_relative_eq!(c.point.z, 10.0, epsilon = 1e-6);
    }
}
