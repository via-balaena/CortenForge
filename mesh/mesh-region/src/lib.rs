//! Region selection, material zones, and thickness mapping for triangle meshes.
//!
//! This crate provides tools for defining and working with regions on a mesh,
//! enabling variable thickness, material zones, and selective operations.
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
//! # Use Cases
//!
//! - Defining thick heel cups and thin arch areas on a skate boot
//! - Marking ventilation zones vs structural zones on a helmet
//! - Specifying material transitions in multi-material prints
//! - Selecting vertices/faces for localized operations
//!
//! # Overview
//!
//! The crate is organized around these main types:
//!
//! - [`MeshRegion`] - A named subset of mesh vertices/faces
//! - [`RegionMap`] - A collection of named regions
//! - [`RegionSelector`] - Criteria for selecting vertices/faces
//! - [`ThicknessMap`] - Per-vertex/face thickness values
//! - [`MaterialZone`] - Material assignment for a region
//! - [`MaterialMap`] - Collection of material zones
//!
//! # Quick Start
//!
//! ## Creating Regions
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex, Point3};
//! use mesh_region::{MeshRegion, RegionSelector};
//!
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
//! mesh.vertices.push(Vertex::new(Point3::new(10.0, 0.0, 0.0)));
//! mesh.vertices.push(Vertex::new(Point3::new(5.0, 10.0, 5.0)));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Create a region using a spatial selector
//! let selector = RegionSelector::sphere(Point3::new(5.0, 5.0, 2.5), 10.0);
//! let region = MeshRegion::from_selector(&mesh, "near_center", &selector);
//!
//! println!("Region '{}' has {} vertices", region.name(), region.vertex_count());
//! ```
//!
//! ## Combining Selectors
//!
//! ```
//! use mesh_types::Point3;
//! use mesh_region::RegionSelector;
//! use nalgebra::Vector3;
//!
//! // Select vertices in the top half AND within 50mm of center
//! let selector = RegionSelector::half_space(
//!     Point3::new(0.0, 0.0, 50.0),
//!     Vector3::new(0.0, 0.0, 1.0),
//! ).and(
//!     RegionSelector::sphere(Point3::new(0.0, 0.0, 0.0), 50.0)
//! );
//! ```
//!
//! ## Variable Thickness
//!
//! ```
//! use mesh_region::{ThicknessMap, MeshRegion};
//!
//! let mut thickness = ThicknessMap::new(2.0); // 2mm default
//!
//! // Set thicker heel region
//! let heel = MeshRegion::from_vertices("heel", [0, 1, 2, 3]);
//! thickness.set_region_thickness(&heel, 5.0);
//!
//! // Query thickness at any vertex
//! let t = thickness.get_vertex_thickness(0);
//! ```
//!
//! ## Material Zones
//!
//! ```
//! use mesh_region::{MaterialZone, MaterialMap, MeshRegion};
//!
//! let mut materials = MaterialMap::with_default("PLA");
//!
//! let heel = MeshRegion::from_vertices("heel", [0, 1, 2, 3]);
//! let zone = MaterialZone::new(heel, "TPU-95A")
//!     .with_shore_hardness(95.0)
//!     .with_color(255, 128, 0);
//!
//! materials.add(zone);
//!
//! // Query material at any vertex
//! let mat = materials.material_for_vertex(0);
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

mod error;
mod material;
mod region;
mod region_map;
mod selector;
mod thickness_map;

pub use error::{RegionError, RegionResult};
pub use material::{MaterialMap, MaterialProperties, MaterialZone};
pub use region::MeshRegion;
pub use region_map::RegionMap;
pub use selector::{FloodFillCriteria, RegionSelector};
pub use thickness_map::ThicknessMap;

// Re-export for convenience
pub use mesh_types::{IndexedMesh, Point3, Vector3, Vertex};
