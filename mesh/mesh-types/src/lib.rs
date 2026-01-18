//! Core mesh types for CortenForge.
//!
//! This crate provides the foundational types for mesh processing:
//!
//! - [`Vertex`] - A point in 3D space with optional attributes
//! - [`IndexedMesh`] - A triangle mesh with indexed vertices
//! - [`Triangle`] - A concrete triangle with vertex positions
//! - [`Aabb`] - Axis-aligned bounding box
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//! - CLI tools
//! - Web applications (WASM)
//! - Servers
//! - Other game engines
//! - Python bindings
//!
//! # Units
//!
//! This library is **unit-agnostic**. All coordinates are `f64`.
//! Downstream crates (mesh-repair, mesh-shell) assume millimeters.
//!
//! # Coordinate System
//!
//! Uses a **right-handed coordinate system**:
//! - X: width (left/right)
//! - Y: depth (front/back)
//! - Z: height (up/down)
//!
//! Face winding is **counter-clockwise (CCW) when viewed from outside**.
//! Normals point outward by the right-hand rule.
//!
//! # Example
//!
//! ```
//! use mesh_types::{Vertex, IndexedMesh, Point3, MeshTopology};
//!
//! // Create a simple triangle mesh
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
//! mesh.vertices.push(Vertex::new(Point3::new(1.0, 0.0, 0.0)));
//! mesh.vertices.push(Vertex::new(Point3::new(0.5, 1.0, 0.0)));
//! mesh.faces.push([0, 1, 2]);
//!
//! assert_eq!(mesh.face_count(), 1);
//! assert!(!mesh.is_empty());
//! ```
//!
//! # Quality Standards
//!
//! This crate maintains A-grade standards per [STANDARDS.md](../../STANDARDS.md):
//! - ≥90% test coverage
//! - Zero clippy/doc warnings
//! - Zero `unwrap`/`expect` in library code

#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

mod bounds;
mod mesh;
mod traits;
mod triangle;
mod vertex;

// Re-export core types
pub use bounds::Aabb;
pub use mesh::{unit_cube, IndexedMesh};
pub use traits::{MeshBounds, MeshTopology};
pub use triangle::Triangle;
pub use vertex::{Vertex, VertexAttributes, VertexColor};

// Re-export nalgebra types for convenience
pub use nalgebra::{Point3, Vector3};
