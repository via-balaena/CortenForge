//! Isotropic mesh remeshing algorithms.
//!
//! This crate provides mesh remeshing algorithms to improve mesh quality
//! by achieving uniform edge lengths while preserving the surface shape.
//!
//! The main algorithm performs iterative edge operations:
//! - **Split**: Long edges are split at their midpoint
//! - **Collapse**: Short edges are collapsed to a single vertex
//! - **Flip**: Edges are flipped to improve triangle quality
//! - **Smooth**: Vertices are smoothed tangentially
//!
//! # Examples
//!
//! Basic remeshing with target edge length:
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_remesh::{remesh, RemeshParams};
//!
//! // Create a simple triangle mesh
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.5, 0.866, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Remesh with target edge length
//! let params = RemeshParams::with_edge_length(0.5)
//!     .with_iterations(5);
//! let result = remesh(&mesh, &params)?;
//!
//! // Check that remeshing occurred
//! assert!(result.final_faces >= result.original_faces);
//! # Ok::<(), mesh_remesh::RemeshError>(())
//! ```
//!
//! High-quality remeshing:
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_remesh::{remesh, RemeshParams};
//!
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.5, 0.866, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Use high-quality preset
//! let params = RemeshParams::high_quality()
//!     .with_target_length(0.3);
//!
//! let result = remesh(&mesh, &params)?;
//!
//! // Remeshing succeeded
//! assert!(result.final_faces >= 1);
//! # Ok::<(), mesh_remesh::RemeshError>(())
//! ```

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]

mod error;
mod params;
mod remesh;
mod result;

pub use error::{RemeshError, RemeshResult};
pub use params::RemeshParams;
pub use remesh::remesh;
pub use result::{EdgeStatistics, RemeshResult as RemeshOutput};
