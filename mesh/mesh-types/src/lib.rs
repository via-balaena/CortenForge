//! Core mesh types for CortenForge.
//!
//! This crate provides the foundational types for mesh processing:
//!
//! - [`IndexedMesh`] - Indexed triangle mesh from cf-geometry (positions + faces)
//! - [`AttributedMesh`] - Mesh with per-vertex attributes (normals, colors, zones)
//! - [`Triangle`] - A concrete triangle with vertex positions (from cf-geometry)
//! - [`Aabb`] - Axis-aligned bounding box (from cf-geometry)
//! - [`Bounded`] - Trait for types that have a bounding box (from cf-geometry)
//! - [`VertexColor`] - RGB color with 8-bit components
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
//! use mesh_types::{IndexedMesh, Point3};
//!
//! // Create a simple triangle mesh
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
//! mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
//! mesh.vertices.push(Point3::new(0.5, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! assert_eq!(mesh.face_count(), 1);
//! assert!(!mesh.is_empty());
//! ```
//!
//! # Quality Standards
//!
//! This crate maintains A-grade standards per [STANDARDS.md](../../docs/STANDARDS.md):
//! - Zero clippy/doc warnings
//! - Zero `unwrap`/`expect` in library code

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]

mod attributed;
mod mesh;
mod vertex;

// Re-export geometric primitives from cf-geometry (canonical source)
pub use cf_geometry::{Aabb, Bounded, IndexedMesh, Triangle};

// Re-export mesh-types domain types
pub use attributed::AttributedMesh;
pub use mesh::{place_on_z_zero, unit_cube};
pub use vertex::VertexColor;

// Re-export nalgebra types for convenience
pub use nalgebra::{Point3, Vector3};
