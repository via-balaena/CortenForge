//! Multi-part assembly management for triangle meshes.
//!
//! This crate provides tools for managing assemblies of multiple mesh parts,
//! supporting hierarchical relationships, connections, and export.
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
//! - Skate boot assembly (boot + blade holder + liner)
//! - Helmet assembly (shell + liner + visor mount)
//! - Multi-part custom devices with snap-fit connections
//! - Bill of materials (BOM) generation
//! - 3MF export with multiple objects
//!
//! # Overview
//!
//! The crate is organized around these main types:
//!
//! - [`Assembly`] - A collection of parts with hierarchy and connections
//! - [`Part`] - A single mesh component with transform and metadata
//! - [`Connection`] - Relationship between two parts (snap-fit, press-fit, etc.)
//! - [`BillOfMaterials`] - Structured part list with dimensions and materials
//!
//! # Quick Start
//!
//! ## Creating an Assembly
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex, Point3};
//! use mesh_assembly::{Assembly, Part, Connection};
//!
//! // Create a new assembly
//! let mut assembly = Assembly::new("skate_boot");
//!
//! // Create a simple boot mesh
//! let mut boot = IndexedMesh::new();
//! boot.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
//! boot.vertices.push(Vertex::new(Point3::new(10.0, 0.0, 0.0)));
//! boot.vertices.push(Vertex::new(Point3::new(5.0, 10.0, 5.0)));
//! boot.faces.push([0, 1, 2]);
//!
//! // Add the boot as the main shell
//! assembly.add_part(
//!     Part::new("shell", boot.clone())
//!         .with_material("PA12")
//! ).unwrap();
//!
//! // Add a liner as a child part
//! assembly.add_part(
//!     Part::new("liner", boot)
//!         .with_parent("shell")
//!         .with_material("TPU")
//!         .with_translation(0.0, 0.0, 1.0)
//! ).unwrap();
//!
//! // Define a connection between parts
//! assembly.define_connection(
//!     Connection::snap_fit("shell", "liner")
//! ).unwrap();
//!
//! println!("Assembly '{}' has {} parts", assembly.name(), assembly.part_count());
//! ```
//!
//! ## Generating a Bill of Materials
//!
//! ```
//! use mesh_types::IndexedMesh;
//! use mesh_assembly::{Assembly, Part};
//!
//! let mut assembly = Assembly::new("product");
//!
//! assembly.add_part(Part::new("body", IndexedMesh::new()).with_material("ABS")).unwrap();
//! assembly.add_part(Part::new("cover", IndexedMesh::new()).with_material("PC")).unwrap();
//!
//! let bom = assembly.generate_bom();
//! println!("Total parts: {}", bom.total_parts());
//! println!("Materials: {:?}", bom.unique_materials());
//! ```
//!
//! ## Exporting to 3MF
//!
//! ```no_run
//! use mesh_types::{IndexedMesh, Vertex, Point3};
//! use mesh_assembly::{Assembly, Part, save_assembly, AssemblyExportFormat};
//! use std::path::Path;
//!
//! let mut assembly = Assembly::new("my_product");
//!
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
//! mesh.vertices.push(Vertex::new(Point3::new(1.0, 0.0, 0.0)));
//! mesh.vertices.push(Vertex::new(Point3::new(0.5, 1.0, 0.0)));
//! mesh.faces.push([0, 1, 2]);
//!
//! assembly.add_part(Part::new("part1", mesh)).unwrap();
//!
//! // Save as 3MF (requires export-3mf feature)
//! save_assembly(&assembly, Path::new("output.3mf"), None).unwrap();
//! ```
//!
//! # Features
//!
//! - `export-3mf` (default) - Enable 3MF export support (requires zip crate)
//!
//! # Quality Standards
//!
//! This crate maintains A-grade standards per [STANDARDS.md](../../STANDARDS.md):
//! - 90% test coverage target
//! - Zero clippy/doc warnings
//! - Zero `unwrap`/`expect` in library code

#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

mod assembly;
mod bom;
mod connection;
mod error;
mod export;
mod part;
mod validation;

pub use assembly::Assembly;
pub use bom::{BillOfMaterials, BomItem};
pub use connection::{Connection, ConnectionParams, ConnectionType};
pub use error::{AssemblyError, AssemblyResult};
pub use export::{AssemblyExportFormat, save_assembly};
pub use part::Part;
pub use validation::{AssemblyValidation, ClearanceResult, InterferenceResult};

// Re-export commonly used types for convenience
pub use mesh_types::{IndexedMesh, Point3, Vector3, Vertex};
pub use nalgebra::Isometry3;
