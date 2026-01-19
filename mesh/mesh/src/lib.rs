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
//! - [`types`] - Core data structures: `IndexedMesh`, `Vertex`, `Triangle`, `Aabb`
//! - [`io`] - File I/O for STL, OBJ, PLY, 3MF, STEP formats
//! - [`transform`] - Transformations, PCA, RANSAC plane fitting
//!
//! ## Core Operations
//! - [`repair`] - Mesh validation and repair (holes, winding, intersections)
//! - [`sdf`] - Signed distance field computation
//! - [`offset`] - Mesh offset via SDF and marching cubes
//! - [`shell`] - Shell generation for 3D printing
//!
//! ## Topology Operations
//! - [`decimate`] - Mesh simplification (QEM-based)
//! - [`subdivide`] - Subdivision schemes (Loop, midpoint)
//! - [`remesh`] - Isotropic remeshing for uniform edge lengths
//!
//! ## Analysis & Measurement
//! - [`measure`] - Dimensions, volume, surface area, cross-sections
//! - [`thickness`] - Wall thickness analysis
//! - [`slice`] - Layer slicing for 3D printing preview
//! - [`geodesic`] - Geodesic distance computation
//! - [`zones`] - Zone assignment and mesh segmentation
//!
//! ## Deformation & Registration
//! - [`morph`] - RBF and FFD mesh deformation
//! - [`registration`] - ICP and landmark-based alignment
//! - [`template`] - Template fitting for parametric customization
//!
//! ## Boolean Operations
//! - [`boolean`] - CSG operations (union, intersection, difference)
//!
//! ## 3D Printing
//! - [`printability`] - Print validation, overhang detection, orientation
//! - [`lattice`] - Lattice and infill generation (TPMS, struts)
//!
//! ## Scan Processing
//! - [`scan`] - Point cloud processing, denoising, reconstruction
//!
//! ## Assembly & Regions
//! - [`assembly`] - Multi-part assembly management
//! - [`region`] - Region selection and material zones
//!
//! ## Mesh Generation
//! - [`from_curves`] - Generate meshes from curves (tubes, sweeps)
//!
//! ## GPU Acceleration (feature-gated)
//! - [`gpu`] - GPU-accelerated SDF computation (requires `gpu` feature)
//!
//! # Feature Flags
//!
//! - `gpu` - Enable GPU-accelerated operations via WGPU

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]
#![doc(html_root_url = "https://docs.rs/mesh/0.7.0")]

// =============================================================================
// Re-exports
// =============================================================================

/// Core data structures: `IndexedMesh`, `Vertex`, `Triangle`, `Aabb`.
pub use mesh_types as types;

/// File I/O for STL, OBJ, PLY, 3MF, STEP formats.
pub use mesh_io as io;

/// Transformations, PCA, RANSAC plane fitting.
pub use mesh_transform as transform;

/// Mesh validation and repair.
pub use mesh_repair as repair;

/// Signed distance field computation.
pub use mesh_sdf as sdf;

/// Mesh offset via SDF and marching cubes.
pub use mesh_offset as offset;

/// Shell generation for 3D printing.
pub use mesh_shell as shell;

/// Mesh simplification (QEM-based decimation).
pub use mesh_decimate as decimate;

/// Subdivision schemes (Loop, midpoint).
pub use mesh_subdivide as subdivide;

/// Isotropic remeshing for uniform edge lengths.
pub use mesh_remesh as remesh;

/// Dimensions, volume, surface area, cross-sections.
pub use mesh_measure as measure;

/// Wall thickness analysis.
pub use mesh_thickness as thickness;

/// Layer slicing for 3D printing preview.
pub use mesh_slice as slice;

/// Geodesic distance computation.
pub use mesh_geodesic as geodesic;

/// Zone assignment and mesh segmentation.
pub use mesh_zones as zones;

/// RBF and FFD mesh deformation.
pub use mesh_morph as morph;

/// ICP and landmark-based alignment.
pub use mesh_registration as registration;

/// Template fitting for parametric customization.
pub use mesh_template as template;

/// CSG operations (union, intersection, difference).
pub use mesh_boolean as boolean;

/// Print validation, overhang detection, orientation.
pub use mesh_printability as printability;

/// Lattice and infill generation.
pub use mesh_lattice as lattice;

/// Point cloud processing, denoising, reconstruction.
pub use mesh_scan as scan;

/// Multi-part assembly management.
pub use mesh_assembly as assembly;

/// Region selection and material zones.
pub use mesh_region as region;

/// Generate meshes from curves (tubes, sweeps).
pub use mesh_from_curves as from_curves;

/// GPU-accelerated operations (requires `gpu` feature).
#[cfg(feature = "gpu")]
pub use mesh_gpu as gpu;

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
    pub use mesh_types::{Aabb, IndexedMesh, MeshBounds, MeshTopology, Triangle, Vertex};

    // I/O
    pub use mesh_io::{MeshFormat, load_mesh, save_mesh};

    // Transform
    pub use mesh_transform::Transform3D;

    // Repair
    pub use mesh_repair::{MeshReport, repair_mesh, validate_mesh};

    // Shell (main use case)
    pub use mesh_shell::ShellBuilder;
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
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
