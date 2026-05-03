//! Complete mesh processing toolkit for 3D printing, CAD, and scan processing.
//!
//! This umbrella crate re-exports all mesh-* crates, providing a unified API
//! for mesh processing operations. All crates are Layer 0 (zero Bevy dependencies)
//! and can be used in CLI tools, WASM, servers, or Python bindings.
//!
//! # Quick Start
//!
//! ```no_run
//! use mesh::prelude::*;
//!
//! // Load a mesh
//! let mesh = mesh::io::load_mesh("model.stl").unwrap();
//!
//! // Validate and repair
//! let report = mesh::repair::validate_mesh(&mesh);
//! let mut repaired = mesh.clone();
//! let params = mesh::repair::RepairParams::default();
//! let _ = mesh::repair::repair_mesh(&mut repaired, &params);
//!
//! // Generate a shell for 3D printing
//! let result = mesh::shell::ShellBuilder::new(&repaired)
//!     .wall_thickness(2.0)
//!     .fast()
//!     .build()
//!     .unwrap();
//!
//! // Save the result
//! mesh::io::save_mesh(&result.mesh, "shell.stl").unwrap();
//! ```
//!
//! # Module Organization
//!
//! ## Foundation
//! - [`types`] - Core data structures: `IndexedMesh`, `Triangle`, `Aabb`, `AttributedMesh`
//! - [`io`] - File I/O for STL, OBJ, PLY, 3MF, STEP formats
//!
//! ## Core Operations
//! - [`repair`] - Mesh validation and repair (holes, winding, intersections)
//! - [`sdf`] - Signed distance field computation
//! - [`offset`] - Mesh offset via SDF and marching cubes
//! - [`shell`] - Shell generation for 3D printing
//!
//! ## Analysis & Measurement
//! - [`measure`] - Dimensions, volume, surface area, cross-sections
//!
//! ## 3D Printing
//! - [`printability`] - Print validation, overhang detection, orientation
//! - [`lattice`] - Lattice and infill generation (TPMS, struts)

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]
#![doc(html_root_url = "https://docs.rs/mesh/1.0.0")]

// =============================================================================
// Re-exports
// =============================================================================

/// Core data structures: `IndexedMesh`, `Triangle`, `Aabb`, `AttributedMesh`.
pub use mesh_types as types;

/// File I/O for STL, OBJ, PLY, 3MF, STEP formats.
pub use mesh_io as io;

/// Mesh validation and repair.
pub use mesh_repair as repair;

/// Signed distance field computation.
pub use mesh_sdf as sdf;

/// Mesh offset via SDF and marching cubes.
pub use mesh_offset as offset;

/// Shell generation for 3D printing.
pub use mesh_shell as shell;

/// Dimensions, volume, surface area, cross-sections.
pub use mesh_measure as measure;

/// Print validation, overhang detection, orientation.
pub use mesh_printability as printability;

/// Lattice and infill generation.
pub use mesh_lattice as lattice;

// =============================================================================
// Prelude
// =============================================================================

/// Common imports for mesh processing.
///
/// This module re-exports the most commonly used types and traits.
///
/// # Usage
///
/// ```
/// use mesh::prelude::*;
/// ```
pub mod prelude {
    // Core types
    pub use mesh_types::{
        Aabb, AttributedMesh, Bounded, IndexedMesh, Point3, Triangle, VertexColor,
    };

    // I/O
    pub use mesh_io::{MeshFormat, load_mesh, save_mesh};

    // Repair
    pub use mesh_repair::{MeshReport, repair_mesh, validate_mesh};

    // Shell (main use case)
    pub use mesh_shell::ShellBuilder;
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
#[allow(clippy::let_underscore_must_use)]
mod tests {
    use super::*;

    #[test]
    fn test_prelude_imports() {
        // Verify prelude types are accessible
        use prelude::*;

        let mesh = IndexedMesh::new();
        assert_eq!(mesh.vertex_count(), 0);
        assert_eq!(mesh.face_count(), 0);
    }

    #[test]
    fn test_module_reexports() {
        // Verify all modules are accessible
        let _ = types::IndexedMesh::new();
        let _ = repair::ValidationOptions::default();
        let _ = shell::ShellParams::default();
    }
}
