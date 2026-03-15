//! Signed distance field computation for triangle meshes.
//!
//! This crate provides tools for computing signed distance fields (SDFs)
//! from triangle meshes. An SDF represents the distance to the nearest
//! surface at every point in space, with the sign indicating inside/outside.
//!
//! # Layer 0
//!
//! This is a Layer 0 crate with zero Bevy dependencies.
//!
//! # Example
//!
//! ```
//! use mesh_types::{IndexedMesh, Point3};
//! use mesh_sdf::{SignedDistanceField, signed_distance};
//!
//! // Create a simple triangle mesh
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
//! mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
//! mesh.vertices.push(Point3::new(5.0, 10.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // For multiple queries, create an SDF once
//! let sdf = SignedDistanceField::new(mesh.clone()).unwrap();
//! let dist1 = sdf.distance(Point3::new(5.0, 5.0, 5.0));
//! let dist2 = sdf.distance(Point3::new(5.0, 5.0, -5.0));
//!
//! // For one-off queries, use the standalone function
//! let dist = signed_distance(Point3::new(0.0, 0.0, 1.0), &mesh);
//! ```
//!
//! # Use Cases
//!
//! - **Mesh offsetting**: Compute offset surfaces by moving along the SDF gradient
//! - **Collision detection**: Quick inside/outside tests
//! - **Ray marching**: Efficient rendering of implicit surfaces
//! - **Boolean operations**: Combine meshes using SDF operations (union, intersection, difference)

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]

mod error;
mod query;
mod sdf;

pub use error::{SdfError, SdfResult};
pub use query::{
    closest_point_on_triangle, point_in_mesh, point_segment_distance_squared,
    ray_triangle_intersect,
};
pub use sdf::{SignedDistanceField, signed_distance, unsigned_distance};
