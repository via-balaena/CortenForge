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
//! # Robustness (Out-of-Bounds Handling)
//!
//! All public query methods handle out-of-bounds points gracefully without panicking:
//!
//! | Method | Out-of-Bounds Behavior |
//! |--------|------------------------|
//! | [`SdfCollisionData::distance`] | Returns `None` |
//! | [`SdfCollisionData::distance_clamped`] | Clamps to grid bounds, returns `max_value` as fallback |
//! | [`SdfCollisionData::gradient`] | Returns `None` |
//! | [`SdfCollisionData::gradient_clamped`] | Clamps to grid bounds, returns `+Z` as fallback |
//!
//! These safe fallbacks ensure the collision system never panics on edge cases:
//! - Points far outside the grid are treated as "very far away" (`max_value`)
//! - Degenerate gradients default to the up vector (`Vector3::z()`)
//!
//! This design follows the principle: **degenerate inputs produce sensible defaults, not panics**.
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

mod interpolation;
mod operations;
mod primitives;

pub use operations::sdf_sdf_contact;
pub use primitives::{
    sdf_box_contact, sdf_capsule_contact, sdf_convex_mesh_contact, sdf_cylinder_contact,
    sdf_ellipsoid_contact, sdf_heightfield_contact, sdf_plane_contact, sdf_point_contact,
    sdf_sphere_contact, sdf_triangle_mesh_contact,
};

use nalgebra::{Point3, Vector3};

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
    pub(crate) values: Vec<f64>,
    /// Number of samples along X axis.
    pub(crate) width: usize,
    /// Number of samples along Y axis.
    pub(crate) height: usize,
    /// Number of samples along Z axis.
    pub(crate) depth: usize,
    /// Size of each cell in meters (uniform in all dimensions).
    pub(crate) cell_size: f64,
    /// Origin of the SDF grid in local coordinates (minimum corner).
    pub(crate) origin: Point3<f64>,
    /// Cached minimum SDF value (most negative = deepest inside).
    pub(crate) min_value: f64,
    /// Cached maximum SDF value (most positive = furthest outside).
    pub(crate) max_value: f64,
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
    ///
    /// # Panics
    ///
    /// Panics if resolution is less than 2.
    #[must_use]
    pub fn sphere(center: Point3<f64>, radius: f64, resolution: usize, padding: f64) -> Self {
        assert!(resolution >= 2, "SDF resolution must be at least 2");
        let extent = radius + padding;
        let cell_size = (2.0 * extent) / (resolution - 1) as f64;
        let origin = Point3::new(center.x - extent, center.y - extent, center.z - extent);

        Self::from_fn(resolution, resolution, resolution, cell_size, origin, |p| {
            (p - center).norm() - radius
        })
    }

    /// Create an SDF representing a box.
    ///
    /// # Panics
    ///
    /// Panics if resolution is less than 2.
    #[must_use]
    pub fn box_shape(
        center: Point3<f64>,
        half_extents: Vector3<f64>,
        resolution: usize,
        padding: f64,
    ) -> Self {
        assert!(resolution >= 2, "SDF resolution must be at least 2");
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
