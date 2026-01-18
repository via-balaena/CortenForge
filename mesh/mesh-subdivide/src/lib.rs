//! Mesh subdivision algorithms.
//!
//! This crate provides mesh subdivision algorithms for increasing mesh density:
//!
//! - **Midpoint subdivision**: Splits each triangle into 4 by adding edge midpoints
//! - **Loop subdivision**: Smoothing subdivision that produces C1 continuous surfaces
//! - **Flat subdivision**: Like midpoint but preserves original geometry
//!
//! # Examples
//!
//! Basic midpoint subdivision:
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_subdivide::{subdivide_mesh, SubdivideParams};
//!
//! // Create a simple triangle mesh
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Subdivide once (each triangle becomes 4)
//! let params = SubdivideParams::midpoint();
//! let result = subdivide_mesh(&mesh, &params)?;
//!
//! assert_eq!(result.final_faces, 4);
//! assert_eq!(result.final_vertices, 6);
//! # Ok::<(), mesh_subdivide::SubdivideError>(())
//! ```
//!
//! Loop subdivision for smooth surfaces:
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_subdivide::{subdivide_mesh, SubdivideParams};
//!
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Use Loop subdivision for smoothing
//! let params = SubdivideParams::loop_subdivision()
//!     .with_iterations(2)
//!     .with_preserve_boundaries(true);
//!
//! let result = subdivide_mesh(&mesh, &params)?;
//!
//! // 1 * 4^2 = 16 faces after 2 iterations
//! assert_eq!(result.final_faces, 16);
//! # Ok::<(), mesh_subdivide::SubdivideError>(())
//! ```

mod error;
mod params;
mod result;
mod subdivide;

pub use error::{SubdivideError, SubdivideResult};
pub use params::{SubdivideParams, SubdivisionMethod};
pub use result::SubdivisionResult;
pub use subdivide::subdivide_mesh;
