//! Signed Distance Field (SDF) collision queries for physics simulation.
//!
//! The SDF data type ([`SdfGrid`]) is provided by `cf-geometry`. This module
//! provides physics-layer contact queries against SDF grids.
//!
//! # Overview
//!
//! A Signed Distance Field represents geometry implicitly:
//! - Positive values indicate points outside the surface
//! - Negative values indicate points inside the surface
//! - Zero values are on the surface boundary
//! - The gradient at any point gives the surface normal direction
//!
//! # Contact Queries
//!
//! 10 primitive-vs-SDF contact functions plus SDF-vs-SDF:
//!
//! - [`sdf_sphere_contact`], [`sdf_point_contact`], [`sdf_capsule_contact`],
//!   [`sdf_box_contact`], [`sdf_cylinder_contact`], [`sdf_ellipsoid_contact`],
//!   [`sdf_convex_mesh_contact`], [`sdf_triangle_mesh_contact`],
//!   [`sdf_plane_contact`], [`sdf_heightfield_contact`], [`sdf_sdf_contact`]
//!
//! # Example
//!
//! ```
//! use sim_core::{SdfGrid, SdfContact};
//! use nalgebra::{Point3, Vector3};
//!
//! // Create an SDF for a sphere (for demonstration)
//! let resolution = 32;
//! let cell_size = 0.15;
//! let sphere_radius = 1.0;
//!
//! let sdf = SdfGrid::from_fn(
//!     resolution, resolution, resolution,
//!     cell_size,
//!     Point3::new(-2.0, -2.0, -2.0), // origin
//!     |p| p.coords.norm() - sphere_radius, // sphere SDF
//! );
//!
//! // Query distance at a point within the grid
//! let dist = sdf.distance(Point3::new(1.0, 0.0, 0.0));
//! assert!(dist.is_some());
//! ```

mod operations;
mod primitives;
pub mod shape;
pub mod shapes;

#[cfg(test)]
mod concave_collision_tests;

pub use cf_geometry::SdfGrid;
pub use operations::sdf_sdf_contact;
pub use primitives::{
    sdf_box_contact, sdf_capsule_contact, sdf_convex_mesh_contact, sdf_cylinder_contact,
    sdf_ellipsoid_contact, sdf_heightfield_contact, sdf_plane_contact, sdf_point_contact,
    sdf_sphere_contact, sdf_triangle_mesh_contact,
};
pub use shape::{PhysicsShape, compute_shape_contact, compute_shape_plane_contact};
pub use shapes::{ShapeConcave, ShapeConvex, ShapeSphere};

use nalgebra::{Point3, Vector3};

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
