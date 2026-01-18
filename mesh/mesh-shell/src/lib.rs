//! Shell generation around 3D meshes for 3D printing.
//!
//! This crate provides tools for generating printable shells around 3D meshes.
//! A shell consists of inner and outer surfaces connected by a rim, creating
//! a watertight volume suitable for 3D printing.
//!
//! # Layer 0
//!
//! This is a Layer 0 crate with zero Bevy dependencies.
//!
//! # Quick Start with `ShellBuilder`
//!
//! The recommended way to generate shells is using the [`ShellBuilder`]:
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_shell::ShellBuilder;
//!
//! // Create a mesh (in practice, load from file)
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(5.0, 10.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Generate shell with defaults
//! let result = ShellBuilder::new(&mesh)
//!     .wall_thickness(2.0)
//!     .fast()
//!     .build();
//! ```
//!
//! # Advanced Configuration
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_shell::ShellBuilder;
//!
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(5.0, 10.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! let result = ShellBuilder::new(&mesh)
//!     .offset(1.0)           // Optional: apply 1mm offset first
//!     .wall_thickness(2.0)   // 2mm walls
//!     .voxel_size(0.5)       // Fine resolution
//!     .high_quality()        // SDF-based walls
//!     .build();
//! ```
//!
//! # Low-Level API
//!
//! For more control, use the low-level functions directly:
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_shell::{generate_shell, ShellParams, validate_shell};
//!
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(5.0, 10.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Generate shell with custom params
//! let params = ShellParams::fast();
//! let (shell, stats) = generate_shell(&mesh, &params).expect("shell generation failed");
//!
//! // Validate the result
//! let validation = validate_shell(&shell);
//! if validation.is_printable() {
//!     println!("Shell is ready for printing!");
//! }
//! ```
//!
//! # Wall Generation Methods
//!
//! Two methods are available for generating the outer surface:
//!
//! - **Normal-based** (fast): Each vertex is offset along its normal.
//!   Fast but wall thickness varies at corners.
//!
//! - **SDF-based** (high quality): Uses signed distance field for consistent
//!   wall thickness everywhere. Slower but more accurate.
//!
//! Use `.fast()` or `.high_quality()` on `ShellBuilder` to select.

mod builder;
mod error;
mod shell;

pub use error::{ShellError, ShellResult};

// Builder API (recommended)
pub use builder::{ShellBuildResult, ShellBuilder};

// Shell generation
pub use shell::{
    generate_shell, generate_shell_no_validation, ShellGenerationResult, ShellParams,
    WallGenerationMethod,
};

// Shell validation
pub use shell::{validate_shell, ShellIssue, ShellValidationResult};

// Rim generation and boundary analysis
pub use shell::{analyze_boundary, generate_rim, BoundaryAnalysis, RimResult};
