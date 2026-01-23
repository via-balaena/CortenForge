//! Signed Distance Field (SDF) collision shape for physics simulation.
//!
//! This module provides SDF-based collision detection for representing complex
//! geometry with a cleaner API than triangle meshes. SDFs store the signed distance
//! to the nearest surface at grid points, enabling smooth collision with arbitrary shapes.
//!
//! # Overview
//!
//! A Signed Distance Field represents geometry implicitly:
//! - Positive values indicate points outside the surface
//! - Negative values indicate points inside the surface
//! - Zero values are on the surface boundary
//! - The gradient at any point gives the surface normal direction
//!
//! # Advantages over Triangle Meshes
//!
//! - **Cleaner collision detection**: Distance queries are direct, no ray casting needed
//! - **Smooth normals**: Gradient computation gives smooth surface normals
//! - **Non-convex support**: Works naturally with non-convex and even self-intersecting geometry
//! - **Gradient-based optimization**: Useful for trajectory optimization and learning
//!
//! # Coordinate System
//!
//! The SDF grid is defined in local coordinates:
//! - Origin at the minimum corner of the bounding box
//! - Grid cells are uniform in all dimensions
//! - Values are stored in ZYX order (Z varies slowest)
//!
//! # Performance
//!
//! SDF collision uses trilinear interpolation:
//! - O(1) distance queries with interpolation
//! - O(1) gradient/normal computation
//! - Memory usage: O(n³) for n×n×n grid
//!
//! # Example
//!
//! ```
//! use sim_core::{SdfCollisionData, SdfContact, CollisionShape};
//! use nalgebra::{Point3, Vector3};
//! use std::sync::Arc;
//!
//! // Create an SDF for a sphere (for demonstration)
//! let resolution = 32;
//! let cell_size = 0.15;
//! let sphere_radius = 1.0;
//!
//! let sdf = SdfCollisionData::from_fn(
//!     resolution, resolution, resolution,
//!     cell_size,
//!     Point3::new(-2.0, -2.0, -2.0), // origin
//!     |p| p.coords.norm() - sphere_radius, // sphere SDF
//! );
//!
//! // Query distance at a point within the grid
//! let dist = sdf.distance(Point3::new(1.0, 0.0, 0.0));
//! assert!(dist.is_some());
//!
//! // Create collision shape
//! let shape = CollisionShape::sdf(Arc::new(sdf));
//! ```

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

/// Signed distance field data for collision detection.
///
/// Stores a 3D grid of signed distance values for efficient collision queries.
/// Values are stored in ZYX order (Z varies slowest, X varies fastest).
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SdfCollisionData {
    /// Signed distance values in ZYX order.
    /// Access pattern: `values[z * width * height + y * width + x]`
    values: Vec<f64>,
    /// Number of samples along X axis.
    width: usize,
    /// Number of samples along Y axis.
    height: usize,
    /// Number of samples along Z axis.
    depth: usize,
    /// Size of each cell in meters (uniform in all dimensions).
    cell_size: f64,
    /// Origin of the SDF grid in local coordinates (minimum corner).
    origin: Point3<f64>,
    /// Cached minimum SDF value (most negative = deepest inside).
    min_value: f64,
    /// Cached maximum SDF value (most positive = furthest outside).
    max_value: f64,
}

impl SdfCollisionData {
    /// Create a new SDF from distance values.
    ///
    /// # Arguments
    ///
    /// * `values` - Signed distance values in ZYX order
    /// * `width` - Number of samples along X
    /// * `height` - Number of samples along Y
    /// * `depth` - Number of samples along Z
    /// * `cell_size` - Size of each cell in meters
    /// * `origin` - Position of the minimum corner in local coordinates
    ///
    /// # Panics
    ///
    /// Panics if `values.len() != width * height * depth` or dimensions are zero.
    #[must_use]
    pub fn new(
        values: Vec<f64>,
        width: usize,
        height: usize,
        depth: usize,
        cell_size: f64,
        origin: Point3<f64>,
    ) -> Self {
        assert!(
            width > 0 && height > 0 && depth > 0,
            "SDF dimensions must be positive"
        );
        assert!(
            values.len() == width * height * depth,
            "SDF data length {} doesn't match dimensions {}x{}x{}",
            values.len(),
            width,
            height,
            depth
        );
        assert!(cell_size > 0.0, "SDF cell_size must be positive");

        let (min_value, max_value) = values
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), &v| {
                (min.min(v), max.max(v))
            });

        Self {
            values,
            width,
            height,
            depth,
            cell_size,
            origin,
            min_value,
            max_value,
        }
    }

    /// Create an SDF from a signed distance function.
    ///
    /// # Arguments
    ///
    /// * `width` - Number of samples along X
    /// * `height` - Number of samples along Y
    /// * `depth` - Number of samples along Z
    /// * `cell_size` - Size of each cell in meters
    /// * `origin` - Position of the minimum corner in local coordinates
    /// * `f` - Function that takes a local-space point and returns signed distance
    #[must_use]
    pub fn from_fn<F>(
        width: usize,
        height: usize,
        depth: usize,
        cell_size: f64,
        origin: Point3<f64>,
        f: F,
    ) -> Self
    where
        F: Fn(Point3<f64>) -> f64,
    {
        let mut values = Vec::with_capacity(width * height * depth);

        for z in 0..depth {
            for y in 0..height {
                for x in 0..width {
                    let p = Point3::new(
                        origin.x + x as f64 * cell_size,
                        origin.y + y as f64 * cell_size,
                        origin.z + z as f64 * cell_size,
                    );
                    values.push(f(p));
                }
            }
        }

        Self::new(values, width, height, depth, cell_size, origin)
    }

    /// Create an SDF representing a sphere.
    ///
    /// Useful for testing and as a simple primitive.
    #[must_use]
    pub fn sphere(center: Point3<f64>, radius: f64, resolution: usize, padding: f64) -> Self {
        let extent = radius + padding;
        let cell_size = (2.0 * extent) / (resolution - 1) as f64;
        let origin = Point3::new(center.x - extent, center.y - extent, center.z - extent);

        Self::from_fn(resolution, resolution, resolution, cell_size, origin, |p| {
            (p - center).norm() - radius
        })
    }

    /// Create an SDF representing a box.
    #[must_use]
    pub fn box_shape(
        center: Point3<f64>,
        half_extents: Vector3<f64>,
        resolution: usize,
        padding: f64,
    ) -> Self {
        let max_extent = half_extents.x.max(half_extents.y).max(half_extents.z) + padding;
        let cell_size = (2.0 * max_extent) / (resolution - 1) as f64;
        let origin = Point3::new(
            center.x - max_extent,
            center.y - max_extent,
            center.z - max_extent,
        );

        Self::from_fn(resolution, resolution, resolution, cell_size, origin, |p| {
            // Box SDF: max of absolute distances to each face
            let q = Vector3::new(
                (p.x - center.x).abs() - half_extents.x,
                (p.y - center.y).abs() - half_extents.y,
                (p.z - center.z).abs() - half_extents.z,
            );
            let outside = Vector3::new(q.x.max(0.0), q.y.max(0.0), q.z.max(0.0)).norm();
            let inside = q.x.max(q.y).max(q.z).min(0.0);
            outside + inside
        })
    }

    /// Get the grid width (samples along X).
    #[must_use]
    pub fn width(&self) -> usize {
        self.width
    }

    /// Get the grid height (samples along Y).
    #[must_use]
    pub fn height(&self) -> usize {
        self.height
    }

    /// Get the grid depth (samples along Z).
    #[must_use]
    pub fn depth(&self) -> usize {
        self.depth
    }

    /// Get the cell size in meters.
    #[must_use]
    pub fn cell_size(&self) -> f64 {
        self.cell_size
    }

    /// Get the origin (minimum corner) of the grid.
    #[must_use]
    pub fn origin(&self) -> Point3<f64> {
        self.origin
    }

    /// Get the total X extent in meters.
    #[must_use]
    pub fn extent_x(&self) -> f64 {
        (self.width - 1) as f64 * self.cell_size
    }

    /// Get the total Y extent in meters.
    #[must_use]
    pub fn extent_y(&self) -> f64 {
        (self.height - 1) as f64 * self.cell_size
    }

    /// Get the total Z extent in meters.
    #[must_use]
    pub fn extent_z(&self) -> f64 {
        (self.depth - 1) as f64 * self.cell_size
    }

    /// Get the minimum SDF value (deepest inside the surface).
    #[must_use]
    pub fn min_value(&self) -> f64 {
        self.min_value
    }

    /// Get the maximum SDF value (furthest outside the surface).
    #[must_use]
    pub fn max_value(&self) -> f64 {
        self.max_value
    }

    /// Get the SDF value at grid coordinates (x, y, z).
    ///
    /// Returns `None` if coordinates are out of bounds.
    #[must_use]
    pub fn get(&self, x: usize, y: usize, z: usize) -> Option<f64> {
        if x < self.width && y < self.height && z < self.depth {
            Some(self.values[z * self.width * self.height + y * self.width + x])
        } else {
            None
        }
    }

    /// Get the SDF value at grid coordinates, clamping to bounds.
    #[must_use]
    pub fn get_clamped(&self, x: i32, y: i32, z: i32) -> f64 {
        let x = x.clamp(0, (self.width - 1) as i32) as usize;
        let y = y.clamp(0, (self.height - 1) as i32) as usize;
        let z = z.clamp(0, (self.depth - 1) as i32) as usize;
        self.values[z * self.width * self.height + y * self.width + x]
    }

    /// Convert local coordinates to grid coordinates.
    fn local_to_grid(&self, point: Point3<f64>) -> (f64, f64, f64) {
        let gx = (point.x - self.origin.x) / self.cell_size;
        let gy = (point.y - self.origin.y) / self.cell_size;
        let gz = (point.z - self.origin.z) / self.cell_size;
        (gx, gy, gz)
    }

    /// Check if a point is within the grid bounds.
    #[must_use]
    pub fn contains(&self, point: Point3<f64>) -> bool {
        let (gx, gy, gz) = self.local_to_grid(point);
        gx >= 0.0
            && gy >= 0.0
            && gz >= 0.0
            && gx <= (self.width - 1) as f64
            && gy <= (self.height - 1) as f64
            && gz <= (self.depth - 1) as f64
    }

    /// Get the interpolated signed distance at a local-space point.
    ///
    /// Uses trilinear interpolation for smooth distance queries.
    /// Returns `None` if the point is outside the grid bounds.
    #[must_use]
    #[allow(clippy::similar_names)] // v000, v100, v00, v10, etc. are intentional for trilinear interp
    pub fn distance(&self, point: Point3<f64>) -> Option<f64> {
        let (gx, gy, gz) = self.local_to_grid(point);

        // Check bounds
        if gx < 0.0
            || gy < 0.0
            || gz < 0.0
            || gx > (self.width - 1) as f64
            || gy > (self.height - 1) as f64
            || gz > (self.depth - 1) as f64
        {
            return None;
        }

        // Get integer cell indices
        let x0 = gx.floor() as usize;
        let y0 = gy.floor() as usize;
        let z0 = gz.floor() as usize;
        let x1 = (x0 + 1).min(self.width - 1);
        let y1 = (y0 + 1).min(self.height - 1);
        let z1 = (z0 + 1).min(self.depth - 1);

        // Get fractional parts
        let fx = gx - x0 as f64;
        let fy = gy - y0 as f64;
        let fz = gz - z0 as f64;

        // Get corner values (8 corners of the cell)
        let v000 = self.values[z0 * self.width * self.height + y0 * self.width + x0];
        let v100 = self.values[z0 * self.width * self.height + y0 * self.width + x1];
        let v010 = self.values[z0 * self.width * self.height + y1 * self.width + x0];
        let v110 = self.values[z0 * self.width * self.height + y1 * self.width + x1];
        let v001 = self.values[z1 * self.width * self.height + y0 * self.width + x0];
        let v101 = self.values[z1 * self.width * self.height + y0 * self.width + x1];
        let v011 = self.values[z1 * self.width * self.height + y1 * self.width + x0];
        let v111 = self.values[z1 * self.width * self.height + y1 * self.width + x1];

        // Trilinear interpolation
        let v00 = v000 + fx * (v100 - v000);
        let v10 = v010 + fx * (v110 - v010);
        let v01 = v001 + fx * (v101 - v001);
        let v11 = v011 + fx * (v111 - v011);

        let v0 = v00 + fy * (v10 - v00);
        let v1 = v01 + fy * (v11 - v01);

        let v = v0 + fz * (v1 - v0);

        Some(v)
    }

    /// Get the signed distance, clamping to bounds if outside.
    #[must_use]
    pub fn distance_clamped(&self, point: Point3<f64>) -> f64 {
        let clamped = Point3::new(
            point
                .x
                .clamp(self.origin.x, self.origin.x + self.extent_x()),
            point
                .y
                .clamp(self.origin.y, self.origin.y + self.extent_y()),
            point
                .z
                .clamp(self.origin.z, self.origin.z + self.extent_z()),
        );
        self.distance(clamped).unwrap_or(self.max_value)
    }

    /// Get the gradient (surface normal direction) at a local-space point.
    ///
    /// Uses finite differences on the interpolated distance field.
    /// Returns `None` if the point is outside the grid bounds.
    #[must_use]
    pub fn gradient(&self, point: Point3<f64>) -> Option<Vector3<f64>> {
        // Use small offset for finite difference
        let eps = self.cell_size * 0.5;

        let d = self.distance(point)?;
        let dx = self.distance_clamped(Point3::new(point.x + eps, point.y, point.z));
        let dy = self.distance_clamped(Point3::new(point.x, point.y + eps, point.z));
        let dz = self.distance_clamped(Point3::new(point.x, point.y, point.z + eps));

        let grad = Vector3::new((dx - d) / eps, (dy - d) / eps, (dz - d) / eps);

        // Return normalized gradient, or zero if gradient is too small
        let norm = grad.norm();
        if norm > 1e-10 {
            Some(grad / norm)
        } else {
            Some(Vector3::z()) // Default to +Z if gradient is degenerate
        }
    }

    /// Get the surface normal at a local-space point.
    ///
    /// The normal points outward from the surface (in the direction of increasing distance).
    #[must_use]
    pub fn normal(&self, point: Point3<f64>) -> Option<Vector3<f64>> {
        self.gradient(point)
    }

    /// Get the gradient, clamping to bounds if outside.
    #[must_use]
    pub fn gradient_clamped(&self, point: Point3<f64>) -> Vector3<f64> {
        let clamped = Point3::new(
            point
                .x
                .clamp(self.origin.x, self.origin.x + self.extent_x()),
            point
                .y
                .clamp(self.origin.y, self.origin.y + self.extent_y()),
            point
                .z
                .clamp(self.origin.z, self.origin.z + self.extent_z()),
        );
        self.gradient(clamped).unwrap_or_else(Vector3::z)
    }

    /// Compute the AABB (axis-aligned bounding box) for this SDF.
    #[must_use]
    pub fn aabb(&self) -> (Point3<f64>, Point3<f64>) {
        let min = self.origin;
        let max = Point3::new(
            self.origin.x + self.extent_x(),
            self.origin.y + self.extent_y(),
            self.origin.z + self.extent_z(),
        );
        (min, max)
    }

    /// Find the closest point on the surface to a query point.
    ///
    /// Uses gradient descent starting from the query point.
    /// Returns `None` if the point is outside the grid bounds.
    #[must_use]
    pub fn closest_surface_point(&self, point: Point3<f64>) -> Option<Point3<f64>> {
        let mut p = point;
        let max_iterations = 10;

        for _ in 0..max_iterations {
            let d = self.distance(p)?;
            if d.abs() < self.cell_size * 0.01 {
                // Close enough to surface
                return Some(p);
            }

            let grad = self.gradient(p)?;
            p -= grad * d; // Move toward surface

            // Clamp to bounds
            p = Point3::new(
                p.x.clamp(self.origin.x, self.origin.x + self.extent_x()),
                p.y.clamp(self.origin.y, self.origin.y + self.extent_y()),
                p.z.clamp(self.origin.z, self.origin.z + self.extent_z()),
            );
        }

        Some(p)
    }
}

/// Result of an SDF contact query.
#[derive(Debug, Clone)]
pub struct SdfContact {
    /// Contact point on the SDF surface (world space).
    pub point: Point3<f64>,
    /// Surface normal at contact point (pointing outward from SDF).
    pub normal: Vector3<f64>,
    /// Penetration depth (positive when object is inside the SDF surface).
    pub penetration: f64,
}

/// Query an SDF for contact with a sphere.
///
/// Returns contact information if the sphere penetrates the SDF surface.
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `sphere_center` - Center of the sphere in world space
/// * `sphere_radius` - Radius of the sphere
#[must_use]
pub fn sdf_sphere_contact(
    sdf: &SdfCollisionData,
    sdf_pose: &Pose,
    sphere_center: Point3<f64>,
    sphere_radius: f64,
) -> Option<SdfContact> {
    // Transform sphere center to SDF local space
    let local_center = sdf_pose.inverse_transform_point(&sphere_center);

    // Query SDF distance at sphere center
    let distance = sdf.distance(local_center)?;

    // Compute penetration: negative distance means inside surface
    // Contact occurs when distance < sphere_radius
    let penetration = sphere_radius - distance;

    if penetration <= 0.0 {
        return None;
    }

    // Get surface normal
    let local_normal = sdf.gradient(local_center)?;

    // Contact point is on the SDF surface, along the normal from sphere center
    let local_contact = local_center - local_normal * distance;

    // Transform back to world space
    let world_point = sdf_pose.transform_point(&local_contact);
    let world_normal = sdf_pose.rotation * local_normal;

    Some(SdfContact {
        point: world_point,
        normal: world_normal,
        penetration,
    })
}

/// Query an SDF for contact with a point.
///
/// Returns contact information if the point is inside the SDF surface.
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `point` - The point to test in world space
#[must_use]
pub fn sdf_point_contact(
    sdf: &SdfCollisionData,
    sdf_pose: &Pose,
    point: Point3<f64>,
) -> Option<SdfContact> {
    // Transform point to SDF local space
    let local_point = sdf_pose.inverse_transform_point(&point);

    // Query SDF distance
    let distance = sdf.distance(local_point)?;

    // Penetration is negative of distance (positive when inside)
    let penetration = -distance;

    if penetration <= 0.0 {
        return None;
    }

    // Get surface normal
    let local_normal = sdf.gradient(local_point)?;

    // Contact point is on the surface, projected along normal
    let local_contact = local_point + local_normal * distance;

    // Transform back to world space
    let world_point = sdf_pose.transform_point(&local_contact);
    let world_normal = sdf_pose.rotation * local_normal;

    Some(SdfContact {
        point: world_point,
        normal: world_normal,
        penetration,
    })
}

/// Query an SDF for contact with a capsule.
///
/// Tests multiple points along the capsule axis for the deepest penetration.
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `capsule_start` - Start point of capsule axis in world space
/// * `capsule_end` - End point of capsule axis in world space
/// * `capsule_radius` - Radius of the capsule
#[must_use]
pub fn sdf_capsule_contact(
    sdf: &SdfCollisionData,
    sdf_pose: &Pose,
    capsule_start: Point3<f64>,
    capsule_end: Point3<f64>,
    capsule_radius: f64,
) -> Option<SdfContact> {
    // Transform capsule to SDF local space
    let local_start = sdf_pose.inverse_transform_point(&capsule_start);
    let local_end = sdf_pose.inverse_transform_point(&capsule_end);

    // Find the lowest distance point on the capsule axis
    let axis = local_end - local_start;
    let axis_len = axis.norm();

    if axis_len < 1e-10 {
        // Degenerate capsule - treat as sphere
        return sdf_sphere_contact(sdf, sdf_pose, capsule_start, capsule_radius);
    }

    // Sample multiple points along the axis for better contact
    let test_points = [
        local_start,
        local_start + axis * 0.25,
        local_start + axis * 0.5,
        local_start + axis * 0.75,
        local_end,
    ];

    let mut deepest_contact: Option<SdfContact> = None;
    let mut max_penetration = 0.0;

    for test_point in &test_points {
        if let Some(distance) = sdf.distance(*test_point) {
            let penetration = capsule_radius - distance;

            if penetration > max_penetration {
                max_penetration = penetration;

                if let Some(local_normal) = sdf.gradient(*test_point) {
                    let local_contact = *test_point - local_normal * distance;

                    let world_point = sdf_pose.transform_point(&local_contact);
                    let world_normal = sdf_pose.rotation * local_normal;

                    deepest_contact = Some(SdfContact {
                        point: world_point,
                        normal: world_normal,
                        penetration,
                    });
                }
            }
        }
    }

    deepest_contact
}

/// Query an SDF for contact with a box.
///
/// Tests all 8 corners of the box against the SDF.
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `box_pose` - The pose of the box in world space
/// * `half_extents` - Half-extents of the box
#[must_use]
pub fn sdf_box_contact(
    sdf: &SdfCollisionData,
    sdf_pose: &Pose,
    box_pose: &Pose,
    half_extents: &Vector3<f64>,
) -> Option<SdfContact> {
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

    let mut deepest_contact: Option<SdfContact> = None;
    let mut max_penetration = 0.0;

    for local_corner in &corners {
        let world_corner = box_pose.transform_point(local_corner);
        let sdf_local = sdf_pose.inverse_transform_point(&world_corner);

        if let Some(distance) = sdf.distance(sdf_local) {
            let penetration = -distance; // Positive when inside

            if penetration > max_penetration {
                max_penetration = penetration;

                if let Some(local_normal) = sdf.gradient(sdf_local) {
                    let local_contact = sdf_local + local_normal * distance;

                    let world_point = sdf_pose.transform_point(&local_contact);
                    let world_normal = sdf_pose.rotation * local_normal;

                    deepest_contact = Some(SdfContact {
                        point: world_point,
                        normal: world_normal,
                        penetration,
                    });
                }
            }
        }
    }

    deepest_contact
}

/// Query an SDF for contact with a cylinder.
///
/// Samples 26 points on the cylinder surface:
/// - 2 cap centers (top and bottom)
/// - 16 cap edge points (8 per cap)
/// - 8 middle circumference points
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `cylinder_pose` - The pose of the cylinder in world space
/// * `half_height` - Half-height of the cylinder along its local Z-axis
/// * `radius` - Radius of the cylinder
#[must_use]
pub fn sdf_cylinder_contact(
    sdf: &SdfCollisionData,
    sdf_pose: &Pose,
    cylinder_pose: &Pose,
    half_height: f64,
    radius: f64,
) -> Option<SdfContact> {
    use std::f64::consts::PI;

    // Build sample points in cylinder local space (Z-axis aligned)
    let mut sample_points = Vec::with_capacity(26);

    // 2 cap centers
    sample_points.push(Point3::new(0.0, 0.0, half_height)); // Top cap center
    sample_points.push(Point3::new(0.0, 0.0, -half_height)); // Bottom cap center

    // 8 points around each cap edge (16 total)
    for i in 0..8_u32 {
        let angle = f64::from(i) * PI / 4.0; // 0, 45, 90, ... degrees
        let x = radius * angle.cos();
        let y = radius * angle.sin();
        sample_points.push(Point3::new(x, y, half_height)); // Top cap edge
        sample_points.push(Point3::new(x, y, -half_height)); // Bottom cap edge
    }

    // 8 points around middle circumference
    for i in 0..8_u32 {
        let angle = f64::from(i) * PI / 4.0;
        let x = radius * angle.cos();
        let y = radius * angle.sin();
        sample_points.push(Point3::new(x, y, 0.0)); // Middle circumference
    }

    // Find the deepest penetrating point
    let mut deepest_contact: Option<SdfContact> = None;
    let mut max_penetration = 0.0;

    for local_point in &sample_points {
        let world_point = cylinder_pose.transform_point(local_point);
        let sdf_local = sdf_pose.inverse_transform_point(&world_point);

        if let Some(distance) = sdf.distance(sdf_local) {
            let penetration = -distance; // Positive when inside

            if penetration > max_penetration {
                max_penetration = penetration;

                if let Some(local_normal) = sdf.gradient(sdf_local) {
                    let local_contact = sdf_local + local_normal * distance;

                    let world_contact = sdf_pose.transform_point(&local_contact);
                    let world_normal = sdf_pose.rotation * local_normal;

                    deepest_contact = Some(SdfContact {
                        point: world_contact,
                        normal: world_normal,
                        penetration,
                    });
                }
            }
        }
    }

    deepest_contact
}

/// Query an SDF for contact with an ellipsoid.
///
/// Samples 26 points on the ellipsoid surface:
/// - 6 axis-aligned points (±x, ±y, ±z scaled by radii)
/// - 8 diagonal points (corners of inscribed cube, projected to surface)
/// - 12 edge midpoints
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `ellipsoid_pose` - The pose of the ellipsoid in world space
/// * `radii` - Radii along each local axis (X, Y, Z)
#[must_use]
pub fn sdf_ellipsoid_contact(
    sdf: &SdfCollisionData,
    sdf_pose: &Pose,
    ellipsoid_pose: &Pose,
    radii: &Vector3<f64>,
) -> Option<SdfContact> {
    // Build sample points in ellipsoid local space
    let mut sample_points = Vec::with_capacity(26);

    // 6 axis-aligned points (poles)
    sample_points.push(Point3::new(radii.x, 0.0, 0.0));
    sample_points.push(Point3::new(-radii.x, 0.0, 0.0));
    sample_points.push(Point3::new(0.0, radii.y, 0.0));
    sample_points.push(Point3::new(0.0, -radii.y, 0.0));
    sample_points.push(Point3::new(0.0, 0.0, radii.z));
    sample_points.push(Point3::new(0.0, 0.0, -radii.z));

    // 8 diagonal points (corners of inscribed cube, projected to ellipsoid surface)
    // For a point (±1, ±1, ±1) normalized and scaled by radii
    let inv_sqrt3 = 1.0 / 3.0_f64.sqrt();
    for &sx in &[-1.0, 1.0] {
        for &sy in &[-1.0, 1.0] {
            for &sz in &[-1.0, 1.0] {
                sample_points.push(Point3::new(
                    radii.x * sx * inv_sqrt3,
                    radii.y * sy * inv_sqrt3,
                    radii.z * sz * inv_sqrt3,
                ));
            }
        }
    }

    // 12 edge midpoints (between pairs of axis-aligned points)
    // XY plane edges
    let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
    sample_points.push(Point3::new(radii.x * inv_sqrt2, radii.y * inv_sqrt2, 0.0));
    sample_points.push(Point3::new(radii.x * inv_sqrt2, -radii.y * inv_sqrt2, 0.0));
    sample_points.push(Point3::new(-radii.x * inv_sqrt2, radii.y * inv_sqrt2, 0.0));
    sample_points.push(Point3::new(-radii.x * inv_sqrt2, -radii.y * inv_sqrt2, 0.0));
    // XZ plane edges
    sample_points.push(Point3::new(radii.x * inv_sqrt2, 0.0, radii.z * inv_sqrt2));
    sample_points.push(Point3::new(radii.x * inv_sqrt2, 0.0, -radii.z * inv_sqrt2));
    sample_points.push(Point3::new(-radii.x * inv_sqrt2, 0.0, radii.z * inv_sqrt2));
    sample_points.push(Point3::new(-radii.x * inv_sqrt2, 0.0, -radii.z * inv_sqrt2));
    // YZ plane edges
    sample_points.push(Point3::new(0.0, radii.y * inv_sqrt2, radii.z * inv_sqrt2));
    sample_points.push(Point3::new(0.0, radii.y * inv_sqrt2, -radii.z * inv_sqrt2));
    sample_points.push(Point3::new(0.0, -radii.y * inv_sqrt2, radii.z * inv_sqrt2));
    sample_points.push(Point3::new(0.0, -radii.y * inv_sqrt2, -radii.z * inv_sqrt2));

    // Find the deepest penetrating point
    let mut deepest_contact: Option<SdfContact> = None;
    let mut max_penetration = 0.0;

    for local_point in &sample_points {
        let world_point = ellipsoid_pose.transform_point(local_point);
        let sdf_local = sdf_pose.inverse_transform_point(&world_point);

        if let Some(distance) = sdf.distance(sdf_local) {
            let penetration = -distance; // Positive when inside

            if penetration > max_penetration {
                max_penetration = penetration;

                if let Some(local_normal) = sdf.gradient(sdf_local) {
                    let local_contact = sdf_local + local_normal * distance;

                    let world_contact = sdf_pose.transform_point(&local_contact);
                    let world_normal = sdf_pose.rotation * local_normal;

                    deepest_contact = Some(SdfContact {
                        point: world_contact,
                        normal: world_normal,
                        penetration,
                    });
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
    clippy::cast_precision_loss
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn sphere_sdf(resolution: usize) -> SdfCollisionData {
        SdfCollisionData::sphere(Point3::origin(), 1.0, resolution, 1.0)
    }

    #[test]
    fn test_sdf_sphere_creation() {
        let sdf = sphere_sdf(16);
        assert_eq!(sdf.width(), 16);
        assert_eq!(sdf.height(), 16);
        assert_eq!(sdf.depth(), 16);
        assert!(sdf.min_value() < 0.0); // Inside the sphere
        assert!(sdf.max_value() > 0.0); // Outside the sphere
    }

    #[test]
    fn test_sdf_distance_at_center() {
        let sdf = sphere_sdf(32);
        let dist = sdf.distance(Point3::origin()).unwrap();
        // At center of unit sphere, distance should be approximately -1.0
        // (exact value depends on grid resolution and padding)
        assert_relative_eq!(dist, -1.0, epsilon = 0.15);
    }

    #[test]
    fn test_sdf_distance_at_surface() {
        let sdf = sphere_sdf(32);
        // At surface of unit sphere
        let dist = sdf.distance(Point3::new(1.0, 0.0, 0.0)).unwrap();
        assert_relative_eq!(dist, 0.0, epsilon = 0.1);
    }

    #[test]
    fn test_sdf_distance_outside() {
        let sdf = sphere_sdf(32);
        let dist = sdf.distance(Point3::new(1.5, 0.0, 0.0)).unwrap();
        assert_relative_eq!(dist, 0.5, epsilon = 0.1);
    }

    #[test]
    fn test_sdf_gradient_at_surface() {
        let sdf = sphere_sdf(32);
        // Gradient at +X surface should point +X
        let grad = sdf.gradient(Point3::new(1.0, 0.0, 0.0)).unwrap();
        assert!(grad.x > 0.8, "gradient.x should be positive");
        assert_relative_eq!(grad.norm(), 1.0, epsilon = 0.01);
    }

    #[test]
    fn test_sdf_sphere_contact_no_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Sphere outside SDF surface - no contact
        let center = Point3::new(3.0, 0.0, 0.0);
        let contact = sdf_sphere_contact(&sdf, &sdf_pose, center, 0.5);
        assert!(contact.is_none());
    }

    #[test]
    fn test_sdf_sphere_contact_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Sphere overlapping SDF surface
        let center = Point3::new(1.3, 0.0, 0.0); // 1.3 - 0.5 = 0.8 (inside unit sphere)
        let contact = sdf_sphere_contact(&sdf, &sdf_pose, center, 0.5);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0, "should have positive penetration");
        assert!(c.normal.x > 0.5, "normal should point outward (+X)");
    }

    #[test]
    fn test_sdf_point_contact() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Point inside sphere
        let point = Point3::new(0.5, 0.0, 0.0);
        let contact = sdf_point_contact(&sdf, &sdf_pose, point);
        assert!(contact.is_some());
        let c = contact.unwrap();
        assert!(c.penetration > 0.0);

        // Point outside sphere
        let point = Point3::new(1.5, 0.0, 0.0);
        let contact = sdf_point_contact(&sdf, &sdf_pose, point);
        assert!(contact.is_none());
    }

    #[test]
    fn test_sdf_box_contact() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();
        let half_extents = Vector3::new(0.3, 0.3, 0.3);

        // Box outside sphere
        let box_pose = Pose::from_position(Point3::new(2.0, 0.0, 0.0));
        let contact = sdf_box_contact(&sdf, &sdf_pose, &box_pose, &half_extents);
        assert!(contact.is_none());

        // Box intersecting sphere
        let box_pose = Pose::from_position(Point3::new(0.9, 0.0, 0.0));
        let contact = sdf_box_contact(&sdf, &sdf_pose, &box_pose, &half_extents);
        assert!(contact.is_some());
        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_capsule_contact() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Capsule outside sphere
        let start = Point3::new(3.0, -1.0, 0.0);
        let end = Point3::new(3.0, 1.0, 0.0);
        let contact = sdf_capsule_contact(&sdf, &sdf_pose, start, end, 0.2);
        assert!(contact.is_none());

        // Capsule intersecting sphere
        let start = Point3::new(0.8, -0.5, 0.0);
        let end = Point3::new(0.8, 0.5, 0.0);
        let contact = sdf_capsule_contact(&sdf, &sdf_pose, start, end, 0.3);
        assert!(contact.is_some());
        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_with_pose() {
        let sdf = sphere_sdf(32);

        // Translate the SDF by (5, 0, 0)
        let sdf_pose = Pose::from_position(Point3::new(5.0, 0.0, 0.0));

        // Sphere at (5, 0, 0) should be at origin of SDF local space
        let center = Point3::new(5.0, 0.0, 0.0);
        let contact = sdf_sphere_contact(&sdf, &sdf_pose, center, 0.5);
        assert!(contact.is_some());
        let c = contact.unwrap();
        // Sphere center is at SDF center (inside), so should have penetration
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_aabb() {
        let sdf = SdfCollisionData::sphere(Point3::new(1.0, 2.0, 3.0), 1.0, 16, 0.5);
        let (min, max) = sdf.aabb();

        // AABB should contain the sphere with padding
        assert!(min.x < 0.0);
        assert!(min.y < 1.5);
        assert!(min.z < 2.5);
        assert!(max.x > 2.0);
        assert!(max.y > 2.5);
        assert!(max.z > 3.5);
    }

    #[test]
    fn test_sdf_box_shape() {
        let sdf =
            SdfCollisionData::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);

        // At center, should be inside (negative distance)
        let dist = sdf.distance(Point3::origin()).unwrap();
        assert!(dist < 0.0, "center should be inside box");

        // At corner, should be at surface (near zero)
        let dist = sdf.distance(Point3::new(0.5, 0.5, 0.5)).unwrap();
        assert_relative_eq!(dist, 0.0, epsilon = 0.1);

        // Outside the box
        let dist = sdf.distance(Point3::new(1.0, 0.0, 0.0)).unwrap();
        assert!(dist > 0.0, "should be outside box");
    }

    #[test]
    fn test_sdf_from_fn() {
        // Create a plane SDF: distance to z=0 plane
        let sdf = SdfCollisionData::from_fn(
            8,
            8,
            8,
            0.5,
            Point3::new(-2.0, -2.0, -2.0),
            |p| p.z, // signed distance to z=0 plane
        );

        // Points above z=0 should be positive
        let dist = sdf.distance(Point3::new(0.0, 0.0, 1.0)).unwrap();
        assert!(dist > 0.0);

        // Points below z=0 should be negative
        let dist = sdf.distance(Point3::new(0.0, 0.0, -1.0)).unwrap();
        assert!(dist < 0.0);
    }

    // =========================================================================
    // Cylinder contact tests
    // =========================================================================

    #[test]
    fn test_sdf_cylinder_contact_no_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Cylinder far outside sphere - no contact
        let cylinder_pose = Pose::from_position(Point3::new(3.0, 0.0, 0.0));
        let contact = sdf_cylinder_contact(&sdf, &sdf_pose, &cylinder_pose, 0.5, 0.3);
        assert!(contact.is_none());
    }

    #[test]
    fn test_sdf_cylinder_contact_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Cylinder intersecting sphere (cylinder axis along Z)
        let cylinder_pose = Pose::from_position(Point3::new(0.7, 0.0, 0.0));
        let contact = sdf_cylinder_contact(&sdf, &sdf_pose, &cylinder_pose, 0.5, 0.3);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0, "should have positive penetration");
        // Normal should point roughly outward from SDF center (+X direction)
        assert!(c.normal.x > 0.5, "normal should point outward");
    }

    #[test]
    fn test_sdf_cylinder_contact_deep_penetration() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Cylinder at center of sphere (maximum penetration)
        let cylinder_pose = Pose::identity();
        let contact = sdf_cylinder_contact(&sdf, &sdf_pose, &cylinder_pose, 0.3, 0.2);
        assert!(contact.is_some());

        let c = contact.unwrap();
        // Should have significant penetration (cylinder inside sphere)
        assert!(c.penetration > 0.5, "should have deep penetration");
    }

    #[test]
    fn test_sdf_cylinder_contact_with_rotation() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Cylinder rotated 90 degrees (axis along X instead of Z)
        use nalgebra::UnitQuaternion;
        let rotation = UnitQuaternion::from_euler_angles(0.0, std::f64::consts::FRAC_PI_2, 0.0);
        let cylinder_pose = Pose::from_position_rotation(Point3::new(0.0, 0.7, 0.0), rotation);

        let contact = sdf_cylinder_contact(&sdf, &sdf_pose, &cylinder_pose, 0.5, 0.3);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_cylinder_contact_cap_hit() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Cylinder positioned so cap center contacts the sphere
        // Cylinder along Z-axis, cap should hit sphere from +Z direction
        let cylinder_pose = Pose::from_position(Point3::new(0.0, 0.0, 0.6));
        let contact = sdf_cylinder_contact(&sdf, &sdf_pose, &cylinder_pose, 0.5, 0.2);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
        // Normal should point roughly in +Z direction
        assert!(c.normal.z > 0.5, "normal should point in +Z direction");
    }

    // =========================================================================
    // Ellipsoid contact tests
    // =========================================================================

    #[test]
    fn test_sdf_ellipsoid_contact_no_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Ellipsoid far outside sphere - no contact
        let ellipsoid_pose = Pose::from_position(Point3::new(3.0, 0.0, 0.0));
        let radii = Vector3::new(0.3, 0.3, 0.3);
        let contact = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii);
        assert!(contact.is_none());
    }

    #[test]
    fn test_sdf_ellipsoid_contact_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Ellipsoid intersecting sphere
        let ellipsoid_pose = Pose::from_position(Point3::new(0.8, 0.0, 0.0));
        let radii = Vector3::new(0.4, 0.3, 0.3);
        let contact = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0, "should have positive penetration");
        // Normal should point roughly outward from SDF center (+X direction)
        assert!(c.normal.x > 0.5, "normal should point outward");
    }

    #[test]
    fn test_sdf_ellipsoid_contact_deep_penetration() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Small ellipsoid at center of sphere (maximum penetration)
        let ellipsoid_pose = Pose::identity();
        let radii = Vector3::new(0.2, 0.2, 0.2);
        let contact = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii);
        assert!(contact.is_some());

        let c = contact.unwrap();
        // Should have significant penetration
        assert!(c.penetration > 0.5, "should have deep penetration");
    }

    #[test]
    fn test_sdf_ellipsoid_contact_elongated() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Elongated ellipsoid (like a cigar shape)
        let ellipsoid_pose = Pose::from_position(Point3::new(0.0, 0.0, 0.6));
        let radii = Vector3::new(0.2, 0.2, 0.5);
        let contact = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
        // The elongated Z-axis should produce a contact with normal roughly in Z direction
        assert!(c.normal.z.abs() > 0.5, "normal should point in Z direction");
    }

    #[test]
    fn test_sdf_ellipsoid_contact_with_rotation() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Ellipsoid rotated 45 degrees around Z-axis
        use nalgebra::UnitQuaternion;
        let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_4);
        let ellipsoid_pose = Pose::from_position_rotation(Point3::new(0.8, 0.0, 0.0), rotation);
        let radii = Vector3::new(0.4, 0.2, 0.2);

        let contact = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_ellipsoid_contact_flat_disk() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Flat disk-like ellipsoid (like a pancake)
        let ellipsoid_pose = Pose::from_position(Point3::new(0.0, 0.0, 0.8));
        let radii = Vector3::new(0.4, 0.4, 0.1);
        let contact = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_ellipsoid_with_translated_sdf() {
        let sdf = sphere_sdf(32);
        // Translate the SDF by (5, 0, 0)
        let sdf_pose = Pose::from_position(Point3::new(5.0, 0.0, 0.0));

        // Ellipsoid at (5, 0, 0) should be at origin of SDF local space
        let ellipsoid_pose = Pose::from_position(Point3::new(5.0, 0.0, 0.0));
        let radii = Vector3::new(0.3, 0.3, 0.3);

        let contact = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii);
        assert!(contact.is_some());

        let c = contact.unwrap();
        // Ellipsoid at SDF center should have penetration
        assert!(c.penetration > 0.0);
    }
}
