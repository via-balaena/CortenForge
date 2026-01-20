//! Mesh offset operations using signed distance fields.
//!
//! This crate provides functionality to offset (expand or contract) triangle
//! meshes by a given distance. It uses signed distance field (SDF) techniques
//! combined with marching cubes isosurface extraction.
//!
//! # Overview
//!
//! The offset process works as follows:
//! 1. Compute a signed distance field for the input mesh
//! 2. Create a 3D grid and sample SDF values, adjusted by the offset distance
//! 3. Extract the zero-isosurface using marching cubes
//!
//! Positive offset distances expand the mesh outward (dilation), while
//! negative distances shrink it inward (erosion).
//!
//! # Example
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_offset::{offset_mesh, OffsetConfig};
//!
//! // Create a unit cube
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(-0.5, -0.5, -0.5));
//! mesh.vertices.push(Vertex::from_coords(0.5, -0.5, -0.5));
//! mesh.vertices.push(Vertex::from_coords(0.5, 0.5, -0.5));
//! mesh.vertices.push(Vertex::from_coords(-0.5, 0.5, -0.5));
//! mesh.vertices.push(Vertex::from_coords(-0.5, -0.5, 0.5));
//! mesh.vertices.push(Vertex::from_coords(0.5, -0.5, 0.5));
//! mesh.vertices.push(Vertex::from_coords(0.5, 0.5, 0.5));
//! mesh.vertices.push(Vertex::from_coords(-0.5, 0.5, 0.5));
//!
//! // Bottom face
//! mesh.faces.push([0, 1, 2]);
//! mesh.faces.push([0, 2, 3]);
//! // Top face
//! mesh.faces.push([4, 6, 5]);
//! mesh.faces.push([4, 7, 6]);
//! // Front face
//! mesh.faces.push([0, 5, 1]);
//! mesh.faces.push([0, 4, 5]);
//! // Back face
//! mesh.faces.push([2, 7, 3]);
//! mesh.faces.push([2, 6, 7]);
//! // Left face
//! mesh.faces.push([0, 3, 7]);
//! mesh.faces.push([0, 7, 4]);
//! // Right face
//! mesh.faces.push([1, 5, 6]);
//! mesh.faces.push([1, 6, 2]);
//!
//! // Offset the mesh outward
//! let config = OffsetConfig::preview();
//! let result = offset_mesh(&mesh, 0.1, &config);
//! assert!(result.is_ok());
//! ```
//!
//! # Quality vs Performance
//!
//! The `OffsetConfig` struct provides presets for different use cases:
//!
//! - `OffsetConfig::preview()` - Fast but coarse, good for interactive preview
//! - `OffsetConfig::default()` - Balanced quality and speed
//! - `OffsetConfig::high_quality()` - Fine resolution for final output

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]

mod error;
mod grid;
mod marching_cubes;
mod offset;

pub use error::{OffsetError, OffsetResult};
pub use grid::ScalarGrid;
pub use marching_cubes::{MarchingCubesConfig, marching_cubes};
pub use offset::{OffsetConfig, offset_mesh, offset_mesh_default};
