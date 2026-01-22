//! Height field collision shape for terrain simulation.
//!
//! This module provides height field collision detection for representing
//! terrain, ground surfaces, and other height-based collision geometry.
//!
//! # Overview
//!
//! A height field is a 2D grid of height values that defines a 3D surface.
//! It's commonly used for:
//! - Terrain and landscapes
//! - Ground surfaces with variation
//! - Procedurally generated environments
//!
//! # Coordinate System
//!
//! The height field is defined in the XY plane with heights along Z:
//! - Origin is at the corner (0, 0) of the grid
//! - X axis spans `[0, width * cell_size]`
//! - Y axis spans `[0, depth * cell_size]`
//! - Z values come from the height data
//!
//! ```text
//!    Z (up)
//!    │
//!    │  ╱────╲
//!    │ ╱      ╲
//!    │╱        ╲
//!    └────────────→ X
//!   ╱
//!  ╱
//! ↙ Y
//! ```
//!
//! # Performance
//!
//! Height field collision uses a grid-based approach:
//! - O(1) cell lookup for point queries
//! - O(k) for shape queries where k is the number of cells intersected
//! - Efficient for large terrains due to implicit spatial structure

// Allow casting for grid indices - these are small values and bounds are checked
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap
)]

use nalgebra::{Point3, Vector3};
use sim_types::Pose;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Height field collision data.
///
/// Stores a 2D grid of height values for terrain collision detection.
/// Heights are stored in row-major order (X varies fastest).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct HeightFieldData {
    /// Height values in row-major order.
    /// Access pattern: `heights[y * width + x]`
    heights: Vec<f64>,
    /// Number of columns (samples along X axis).
    width: usize,
    /// Number of rows (samples along Y axis).
    depth: usize,
    /// Size of each cell in meters.
    cell_size: f64,
    /// Minimum height value (cached for AABB).
    min_height: f64,
    /// Maximum height value (cached for AABB).
    max_height: f64,
}

impl HeightFieldData {
    /// Create a new height field from height data.
    ///
    /// # Arguments
    ///
    /// * `heights` - Height values in row-major order
    /// * `width` - Number of columns (samples along X)
    /// * `depth` - Number of rows (samples along Y)
    /// * `cell_size` - Size of each cell in meters
    ///
    /// # Panics
    ///
    /// Panics if `heights.len() != width * depth` or if dimensions are zero.
    #[must_use]
    pub fn new(heights: Vec<f64>, width: usize, depth: usize, cell_size: f64) -> Self {
        assert!(
            width > 0 && depth > 0,
            "HeightField dimensions must be positive"
        );
        assert!(
            heights.len() == width * depth,
            "HeightField data length {} doesn't match dimensions {}x{}",
            heights.len(),
            width,
            depth
        );
        assert!(cell_size > 0.0, "HeightField cell_size must be positive");

        let (min_height, max_height) = heights
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), &h| {
                (min.min(h), max.max(h))
            });

        Self {
            heights,
            width,
            depth,
            cell_size,
            min_height,
            max_height,
        }
    }

    /// Create a flat height field at a given height.
    #[must_use]
    pub fn flat(width: usize, depth: usize, cell_size: f64, height: f64) -> Self {
        let heights = vec![height; width * depth];
        Self::new(heights, width, depth, cell_size)
    }

    /// Create a height field from a function.
    ///
    /// The function receives world-space (x, y) coordinates and returns the height.
    #[must_use]
    pub fn from_fn<F>(width: usize, depth: usize, cell_size: f64, f: F) -> Self
    where
        F: Fn(f64, f64) -> f64,
    {
        let mut heights = Vec::with_capacity(width * depth);
        for y in 0..depth {
            for x in 0..width {
                let wx = x as f64 * cell_size;
                let wy = y as f64 * cell_size;
                heights.push(f(wx, wy));
            }
        }
        Self::new(heights, width, depth, cell_size)
    }

    /// Get the width (number of columns).
    #[must_use]
    pub fn width(&self) -> usize {
        self.width
    }

    /// Get the depth (number of rows).
    #[must_use]
    pub fn depth(&self) -> usize {
        self.depth
    }

    /// Get the cell size in meters.
    #[must_use]
    pub fn cell_size(&self) -> f64 {
        self.cell_size
    }

    /// Get the total X extent in meters.
    #[must_use]
    pub fn extent_x(&self) -> f64 {
        (self.width - 1) as f64 * self.cell_size
    }

    /// Get the total Y extent in meters.
    #[must_use]
    pub fn extent_y(&self) -> f64 {
        (self.depth - 1) as f64 * self.cell_size
    }

    /// Get the minimum height value.
    #[must_use]
    pub fn min_height(&self) -> f64 {
        self.min_height
    }

    /// Get the maximum height value.
    #[must_use]
    pub fn max_height(&self) -> f64 {
        self.max_height
    }

    /// Get the height at grid coordinates (x, y).
    ///
    /// Returns `None` if coordinates are out of bounds.
    #[must_use]
    pub fn get(&self, x: usize, y: usize) -> Option<f64> {
        if x < self.width && y < self.depth {
            Some(self.heights[y * self.width + x])
        } else {
            None
        }
    }

    /// Get the height at grid coordinates, with bounds clamping.
    #[must_use]
    pub fn get_clamped(&self, x: i32, y: i32) -> f64 {
        let x = x.clamp(0, (self.width - 1) as i32) as usize;
        let y = y.clamp(0, (self.depth - 1) as i32) as usize;
        self.heights[y * self.width + x]
    }

    /// Get the interpolated height at world-space (x, y) coordinates.
    ///
    /// Uses bilinear interpolation for smooth height queries.
    /// Returns `None` if the point is outside the height field bounds.
    #[must_use]
    pub fn sample(&self, x: f64, y: f64) -> Option<f64> {
        if x < 0.0 || y < 0.0 {
            return None;
        }

        let max_x = self.extent_x();
        let max_y = self.extent_y();

        if x > max_x || y > max_y {
            return None;
        }

        // Convert to grid coordinates
        let gx = x / self.cell_size;
        let gy = y / self.cell_size;

        // Get integer cell indices
        let x0 = gx.floor() as usize;
        let y0 = gy.floor() as usize;
        let x1 = (x0 + 1).min(self.width - 1);
        let y1 = (y0 + 1).min(self.depth - 1);

        // Get fractional parts
        let fx = gx - x0 as f64;
        let fy = gy - y0 as f64;

        // Get corner heights
        let h00 = self.heights[y0 * self.width + x0];
        let h10 = self.heights[y0 * self.width + x1];
        let h01 = self.heights[y1 * self.width + x0];
        let h11 = self.heights[y1 * self.width + x1];

        // Bilinear interpolation
        let h0 = h00 + fx * (h10 - h00);
        let h1 = h01 + fx * (h11 - h01);
        let h = h0 + fy * (h1 - h0);

        Some(h)
    }

    /// Get the interpolated height, clamping to bounds if outside.
    #[must_use]
    pub fn sample_clamped(&self, x: f64, y: f64) -> f64 {
        let x = x.clamp(0.0, self.extent_x());
        let y = y.clamp(0.0, self.extent_y());
        // Safe to unwrap because we clamped to valid bounds
        self.sample(x, y).unwrap_or(self.min_height)
    }

    /// Get the surface normal at world-space (x, y) coordinates.
    ///
    /// Uses finite differences to compute the gradient and normal.
    /// Returns `None` if the point is outside the height field bounds.
    #[must_use]
    pub fn normal(&self, x: f64, y: f64) -> Option<Vector3<f64>> {
        // Use small offset for finite difference
        let eps = self.cell_size * 0.1;

        let hc = self.sample(x, y)?;
        let hx = self.sample_clamped(x + eps, y);
        let hy = self.sample_clamped(x, y + eps);

        // Compute gradient
        let dx = (hx - hc) / eps;
        let dy = (hy - hc) / eps;

        // Normal is perpendicular to gradient: (-dz/dx, -dz/dy, 1) normalized
        let normal = Vector3::new(-dx, -dy, 1.0).normalize();
        Some(normal)
    }

    /// Get the surface normal, clamping to bounds if outside.
    #[must_use]
    pub fn normal_clamped(&self, x: f64, y: f64) -> Vector3<f64> {
        let x = x.clamp(0.0, self.extent_x());
        let y = y.clamp(0.0, self.extent_y());
        self.normal(x, y).unwrap_or_else(Vector3::z)
    }

    /// Compute the AABB (axis-aligned bounding box) for this height field.
    #[must_use]
    pub fn aabb(&self) -> (Point3<f64>, Point3<f64>) {
        let min = Point3::new(0.0, 0.0, self.min_height);
        let max = Point3::new(self.extent_x(), self.extent_y(), self.max_height);
        (min, max)
    }

    /// Get the cell indices containing a world-space point.
    ///
    /// Returns `None` if the point is outside the height field bounds.
    #[must_use]
    pub fn cell_at(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        if x < 0.0 || y < 0.0 {
            return None;
        }

        let cx = (x / self.cell_size).floor() as usize;
        let cy = (y / self.cell_size).floor() as usize;

        if cx >= self.width - 1 || cy >= self.depth - 1 {
            return None;
        }

        Some((cx, cy))
    }

    /// Get all cells that intersect with an AABB.
    ///
    /// Returns iterator over `(cell_x, cell_y)` pairs.
    pub fn cells_in_aabb(
        &self,
        min: Point3<f64>,
        max: Point3<f64>,
    ) -> impl Iterator<Item = (usize, usize)> + '_ {
        // Clamp to height field bounds
        let x_start = (min.x / self.cell_size).floor().max(0.0) as usize;
        let y_start = (min.y / self.cell_size).floor().max(0.0) as usize;
        let x_end = ((max.x / self.cell_size).ceil() as usize).min(self.width - 1);
        let y_end = ((max.y / self.cell_size).ceil() as usize).min(self.depth - 1);

        (y_start..y_end).flat_map(move |y| (x_start..x_end).map(move |x| (x, y)))
    }

    /// Get the four corner heights of a cell.
    ///
    /// Returns `None` if cell indices are out of bounds.
    #[must_use]
    pub fn cell_heights(&self, cx: usize, cy: usize) -> Option<[f64; 4]> {
        if cx >= self.width - 1 || cy >= self.depth - 1 {
            return None;
        }

        Some([
            self.heights[cy * self.width + cx],           // (cx, cy)
            self.heights[cy * self.width + cx + 1],       // (cx+1, cy)
            self.heights[(cy + 1) * self.width + cx],     // (cx, cy+1)
            self.heights[(cy + 1) * self.width + cx + 1], // (cx+1, cy+1)
        ])
    }

    /// Get the world-space position of a grid vertex.
    #[must_use]
    pub fn vertex_position(&self, x: usize, y: usize) -> Option<Point3<f64>> {
        let height = self.get(x, y)?;
        Some(Point3::new(
            x as f64 * self.cell_size,
            y as f64 * self.cell_size,
            height,
        ))
    }
}

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

    // Check bounds
    if x < 0.0 || y < 0.0 {
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
    fn test_heightfield_flat() {
        let hf = HeightFieldData::flat(10, 10, 1.0, 5.0);
        assert_eq!(hf.width(), 10);
        assert_eq!(hf.depth(), 10);
        assert_eq!(hf.cell_size(), 1.0);
        assert_relative_eq!(hf.min_height(), 5.0);
        assert_relative_eq!(hf.max_height(), 5.0);

        // Sample at various points
        assert_relative_eq!(hf.sample(0.0, 0.0).unwrap(), 5.0);
        assert_relative_eq!(hf.sample(4.5, 4.5).unwrap(), 5.0);
        assert_relative_eq!(hf.sample(9.0, 9.0).unwrap(), 5.0);
    }

    #[test]
    fn test_heightfield_from_fn() {
        // Create a simple slope: z = x
        let hf = HeightFieldData::from_fn(5, 5, 1.0, |x, _y| x);

        assert_relative_eq!(hf.get(0, 0).unwrap(), 0.0);
        assert_relative_eq!(hf.get(1, 0).unwrap(), 1.0);
        assert_relative_eq!(hf.get(2, 0).unwrap(), 2.0);
        assert_relative_eq!(hf.get(4, 0).unwrap(), 4.0);
    }

    #[test]
    fn test_heightfield_bilinear_interpolation() {
        let hf = HeightFieldData::new(
            vec![0.0, 2.0, 1.0, 3.0], // 2x2 grid
            2,
            2,
            1.0,
        );

        // Corner values
        assert_relative_eq!(hf.sample(0.0, 0.0).unwrap(), 0.0);
        assert_relative_eq!(hf.sample(1.0, 0.0).unwrap(), 2.0);
        assert_relative_eq!(hf.sample(0.0, 1.0).unwrap(), 1.0);
        assert_relative_eq!(hf.sample(1.0, 1.0).unwrap(), 3.0);

        // Center should be average of all four: (0 + 2 + 1 + 3) / 4 = 1.5
        assert_relative_eq!(hf.sample(0.5, 0.5).unwrap(), 1.5);

        // Edge midpoints
        assert_relative_eq!(hf.sample(0.5, 0.0).unwrap(), 1.0); // avg of 0 and 2
        assert_relative_eq!(hf.sample(0.5, 1.0).unwrap(), 2.0); // avg of 1 and 3
    }

    #[test]
    fn test_heightfield_normal() {
        // Flat terrain should have normal pointing up
        let flat = HeightFieldData::flat(5, 5, 1.0, 0.0);
        let normal = flat.normal(2.0, 2.0).unwrap();
        assert_relative_eq!(normal.z, 1.0, epsilon = 1e-3);
        assert_relative_eq!(normal.x.abs(), 0.0, epsilon = 1e-3);
        assert_relative_eq!(normal.y.abs(), 0.0, epsilon = 1e-3);

        // Slope terrain: z = x
        let slope = HeightFieldData::from_fn(10, 10, 1.0, |x, _y| x * 0.5);
        let normal = slope.normal(5.0, 5.0).unwrap();
        // Normal should point in -X, +Z direction
        assert!(normal.x < 0.0, "normal.x should be negative");
        assert!(normal.z > 0.0, "normal.z should be positive");
    }

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
    fn test_heightfield_cells_in_aabb() {
        let hf = HeightFieldData::flat(10, 10, 1.0, 0.0);

        let min = Point3::new(2.5, 2.5, -1.0);
        let max = Point3::new(5.5, 5.5, 1.0);

        let cells: Vec<_> = hf.cells_in_aabb(min, max).collect();

        // Should cover cells from (2,2) to (5,5)
        assert!(!cells.is_empty());
        assert!(cells.contains(&(2, 2)));
        assert!(cells.contains(&(3, 3)));
        assert!(cells.contains(&(4, 4)));
        assert!(cells.contains(&(5, 5)));
    }

    #[test]
    fn test_heightfield_aabb() {
        let hf = HeightFieldData::from_fn(10, 10, 2.0, |x, y| x + y);
        let (min, max) = hf.aabb();

        assert_relative_eq!(min.x, 0.0);
        assert_relative_eq!(min.y, 0.0);
        assert_relative_eq!(max.x, 18.0); // (10-1) * 2.0
        assert_relative_eq!(max.y, 18.0);
        assert_relative_eq!(min.z, hf.min_height());
        assert_relative_eq!(max.z, hf.max_height());
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
