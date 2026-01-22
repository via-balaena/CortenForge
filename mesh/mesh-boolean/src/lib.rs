//! Boolean operations (CSG) for triangle meshes.
//!
//! This crate provides constructive solid geometry operations for combining
//! triangle meshes: union (A ∪ B), intersection (A ∩ B), and difference (A - B).
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
//! # Features
//!
//! - **BVH-accelerated** intersection detection (O(n log n + k))
//! - **Edge insertion** for clean intersection boundaries
//! - **Coplanar face handling** with configurable strategies
//! - **Multi-mesh operations** with parallel tree reduction
//! - **GPU acceleration** (optional, via `gpu` feature)
//! - **Configuration presets** for different use cases (scans, CAD)
//!
//! # Quick Start
//!
//! ```ignore
//! use mesh_boolean::{union, difference, intersection};
//! use mesh_types::IndexedMesh;
//!
//! // Simple API
//! let combined = union(&mesh_a, &mesh_b)?;
//! let subtracted = difference(&mesh_a, &mesh_b)?;
//! let overlap = intersection(&mesh_a, &mesh_b)?;
//! ```
//!
//! # Configuration
//!
//! For more control, use the `*_with_config` variants:
//!
//! ```ignore
//! use mesh_boolean::{union_with_config, BooleanConfig, CleanupLevel};
//!
//! // Use scan preset (looser tolerances for noisy data)
//! let config = BooleanConfig::for_scans()
//!     .with_cleanup(CleanupLevel::Full);
//!
//! let result = union_with_config(&scan_a, &scan_b, &config)?;
//! println!("Union has {} faces", result.mesh.faces.len());
//! ```
//!
//! # Multi-Mesh Operations
//!
//! For combining many meshes efficiently:
//!
//! ```ignore
//! use mesh_boolean::{multi_union, BooleanConfig};
//!
//! // Parallel tree-based union (O(n log n) instead of O(n²))
//! let meshes = vec![mesh1, mesh2, mesh3, mesh4, mesh5];
//! let result = multi_union(&meshes, &BooleanConfig::default())?;
//! ```
//!
//! # GPU Acceleration
//!
//! Enable the `gpu` feature for GPU-accelerated face classification:
//!
//! ```toml
//! [dependencies]
//! mesh-boolean = { version = "0.7", features = ["gpu"] }
//! ```
//!
//! ```ignore
//! use mesh_boolean::gpu::{GpuContext, is_gpu_available};
//!
//! if is_gpu_available() {
//!     let gpu = GpuContext::new()?;
//!     let classifications = gpu.classify_faces(&mesh_a, &mesh_b, 1e-6)?;
//! }
//! ```
//!
//! # Presets
//!
//! | Preset | Use Case | Tolerances |
//! |--------|----------|------------|
//! | `default()` | General purpose | Balanced |
//! | `for_scans()` | 3D scan data | Loose (handles noise) |
//! | `for_cad()` | CAD geometry | Tight (preserves precision) |
//! | `strict()` | Perfect input | Tightest |
//!
//! # Quality Standards
//!
//! This crate maintains A-grade standards per [STANDARDS.md](../../STANDARDS.md):
//! - ≥90% test coverage
//! - Zero clippy/doc warnings
//! - Zero `unwrap`/`expect` in library code

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
// Allow some pedantic lints that conflict with API design choices
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::must_use_candidate)]
#![allow(clippy::similar_names)]
// Allow single-char names in math-heavy code (standard in graphics/geometry algorithms)
#![allow(clippy::many_single_char_names)]
// Allow cast truncation - mesh indices are validated at runtime
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]
#![allow(clippy::cast_precision_loss)]
// Allow some nursery lints that are too strict
#![allow(clippy::missing_const_for_fn)]
#![allow(clippy::suboptimal_flops)]
#![allow(clippy::cognitive_complexity)]
#![allow(clippy::too_many_lines)]
// Allow doc examples using ignore (they need mesh fixtures)
#![allow(clippy::missing_panics_doc)]

pub mod bvh;
pub mod classify;
pub mod config;
pub mod coplanar;
pub mod edge_insert;
pub mod error;
#[cfg(feature = "gpu")]
pub mod gpu;
pub mod intersect;
pub mod multi;
pub mod operation;

// Re-export main types and functions for convenient access
pub use config::{BooleanConfig, BooleanOp, CleanupLevel, CoplanarStrategy};
pub use error::{BooleanError, BooleanResult};
pub use operation::{
    BooleanOperationResult, BooleanStats, boolean_operation, difference, difference_with_config,
    intersection, intersection_with_config, union, union_with_config,
};

pub use multi::{
    MultiMeshResult, MultiMeshStats, concatenate_meshes, multi_intersection, multi_union,
    sequential_difference,
};

// Re-export mesh types for convenience
pub use mesh_types::{IndexedMesh, Point3, Vector3, Vertex};

/// Prelude module for convenient imports.
///
/// # Example
///
/// ```ignore
/// use mesh_boolean::prelude::*;
///
/// let result = union(&mesh_a, &mesh_b)?;
/// ```
pub mod prelude {
    pub use crate::config::{BooleanConfig, BooleanOp, CleanupLevel, CoplanarStrategy};
    pub use crate::error::{BooleanError, BooleanResult};
    pub use crate::multi::{
        concatenate_meshes, multi_intersection, multi_union, sequential_difference,
    };
    pub use crate::operation::{
        boolean_operation, difference, difference_with_config, intersection,
        intersection_with_config, union, union_with_config,
    };
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::redundant_clone,
    clippy::cast_lossless,
    clippy::cast_precision_loss
)]
mod tests {
    use super::*;

    fn create_test_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        let vertices = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
        ];

        for v in &vertices {
            mesh.vertices.push(Vertex::new(*v));
        }

        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        mesh.faces.push([4, 5, 6]);
        mesh.faces.push([4, 6, 7]);
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        mesh.faces.push([0, 4, 7]);
        mesh.faces.push([0, 7, 3]);
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);

        mesh
    }

    #[test]
    fn test_simple_union() {
        let cube_a = create_test_cube();

        let mut cube_b = IndexedMesh::new();
        for v in &[
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(6.0, 0.0, 0.0),
            Point3::new(6.0, 1.0, 0.0),
            Point3::new(5.0, 1.0, 0.0),
            Point3::new(5.0, 0.0, 1.0),
            Point3::new(6.0, 0.0, 1.0),
            Point3::new(6.0, 1.0, 1.0),
            Point3::new(5.0, 1.0, 1.0),
        ] {
            cube_b.vertices.push(Vertex::new(*v));
        }
        cube_b.faces.push([0, 2, 1]);
        cube_b.faces.push([0, 3, 2]);
        cube_b.faces.push([4, 5, 6]);
        cube_b.faces.push([4, 6, 7]);
        cube_b.faces.push([0, 1, 5]);
        cube_b.faces.push([0, 5, 4]);
        cube_b.faces.push([2, 3, 7]);
        cube_b.faces.push([2, 7, 6]);
        cube_b.faces.push([0, 4, 7]);
        cube_b.faces.push([0, 7, 3]);
        cube_b.faces.push([1, 2, 6]);
        cube_b.faces.push([1, 6, 5]);

        let result = union(&cube_a, &cube_b);
        assert!(result.is_ok());

        let op_result = result.unwrap();
        assert_eq!(op_result.mesh.faces.len(), 24);
    }

    #[test]
    fn test_config_presets() {
        let default = BooleanConfig::default();
        let scans = BooleanConfig::for_scans();
        let cad = BooleanConfig::for_cad();
        let strict = BooleanConfig::strict();

        // Scans should have looser tolerances
        assert!(scans.vertex_weld_tolerance > default.vertex_weld_tolerance);

        // CAD should have tighter tolerances
        assert!(cad.vertex_weld_tolerance < default.vertex_weld_tolerance);

        // Strict should be tightest
        assert!(strict.vertex_weld_tolerance < cad.vertex_weld_tolerance);
    }

    #[test]
    fn test_builder_pattern() {
        let config = BooleanConfig::default()
            .with_cleanup(CleanupLevel::Full)
            .with_coplanar_strategy(CoplanarStrategy::Exclude)
            .with_parallel(false);

        assert_eq!(config.cleanup, CleanupLevel::Full);
        assert_eq!(config.coplanar_strategy, CoplanarStrategy::Exclude);
        assert!(!config.parallel);
    }

    #[test]
    fn test_multi_union() {
        let cubes: Vec<IndexedMesh> = (0..3)
            .map(|i| {
                let mut mesh = IndexedMesh::new();
                let offset = i as f64 * 3.0;
                for v in &[
                    Point3::new(offset, 0.0, 0.0),
                    Point3::new(offset + 1.0, 0.0, 0.0),
                    Point3::new(offset + 1.0, 1.0, 0.0),
                    Point3::new(offset, 1.0, 0.0),
                    Point3::new(offset, 0.0, 1.0),
                    Point3::new(offset + 1.0, 0.0, 1.0),
                    Point3::new(offset + 1.0, 1.0, 1.0),
                    Point3::new(offset, 1.0, 1.0),
                ] {
                    mesh.vertices.push(Vertex::new(*v));
                }
                mesh.faces.push([0, 2, 1]);
                mesh.faces.push([0, 3, 2]);
                mesh.faces.push([4, 5, 6]);
                mesh.faces.push([4, 6, 7]);
                mesh.faces.push([0, 1, 5]);
                mesh.faces.push([0, 5, 4]);
                mesh.faces.push([2, 3, 7]);
                mesh.faces.push([2, 7, 6]);
                mesh.faces.push([0, 4, 7]);
                mesh.faces.push([0, 7, 3]);
                mesh.faces.push([1, 2, 6]);
                mesh.faces.push([1, 6, 5]);
                mesh
            })
            .collect();

        let result = multi_union(&cubes, &BooleanConfig::default());
        assert!(result.is_ok());

        let multi_result = result.unwrap();
        assert_eq!(multi_result.mesh.faces.len(), 36); // 3 cubes × 12 faces
        assert_eq!(multi_result.stats.input_count, 3);
    }
}
