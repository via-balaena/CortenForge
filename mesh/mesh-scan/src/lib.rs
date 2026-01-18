//! 3D scan processing algorithms.
//!
//! This crate provides a complete toolkit for processing 3D scan data:
//!
//! - **Point Cloud** - Point cloud data structure with I/O and normals
//! - **Cleanup** - Remove outliers, spikes, and disconnected components
//! - **Denoising** - Smooth noisy mesh surfaces while preserving features
//! - **Multi-scan** - Align and merge multiple overlapping scans
//! - **Reconstruction** - Convert point clouds to triangle meshes
//!
//! # Layer 0
//!
//! This is a Layer 0 crate with zero Bevy dependencies. All types are
//! designed for use in pure Rust computational pipelines.
//!
//! # Quick Start
//!
//! ## Point Cloud Processing
//!
//! ```
//! use mesh_scan::pointcloud::PointCloud;
//! use mesh_scan::reconstruct::{ReconstructionParams, to_mesh};
//! use nalgebra::{Point3, Vector3};
//!
//! // Create and populate a point cloud with normals
//! let mut cloud = PointCloud::new();
//! cloud.add_point_with_normal(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
//! cloud.add_point_with_normal(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
//! cloud.add_point_with_normal(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
//!
//! // Convert to mesh
//! let mesh = to_mesh(&cloud).unwrap();
//! ```
//!
//! ## Scan Cleanup
//!
//! ```
//! use mesh_scan::cleanup::{cleanup_scan, CleanupParams};
//! use mesh_types::{IndexedMesh, Vertex};
//!
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! let params = CleanupParams::for_object_scan();
//! let result = cleanup_scan(&mesh, &params).unwrap();
//! println!("{}", result);
//! ```
//!
//! ## Mesh Denoising
//!
//! ```
//! use mesh_scan::denoise::{denoise_mesh, DenoiseParams};
//! use mesh_types::{IndexedMesh, Vertex};
//!
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! let params = DenoiseParams::for_scans();
//! let result = denoise_mesh(&mesh, &params).unwrap();
//! println!("{}", result);
//! ```
//!
//! ## Multi-scan Alignment
//!
//! ```
//! use mesh_scan::multiscan::{align_multiple_scans, merge_scans, MultiAlignmentParams, MergeParams};
//! use mesh_types::{IndexedMesh, Vertex};
//!
//! // Create two overlapping scans
//! let mut scan1 = IndexedMesh::new();
//! for i in 0..5 {
//!     scan1.vertices.push(Vertex::from_coords(f64::from(i), 0.0, 0.0));
//! }
//!
//! let mut scan2 = IndexedMesh::new();
//! for i in 0..5 {
//!     scan2.vertices.push(Vertex::from_coords(f64::from(i) + 0.1, 0.0, 0.0));
//! }
//!
//! let scans = vec![scan1, scan2];
//!
//! // Align scans
//! let alignment = align_multiple_scans(&scans, &MultiAlignmentParams::fast()).unwrap();
//!
//! // Merge aligned scans
//! let merged = merge_scans(&scans, &alignment.transforms, &MergeParams::default()).unwrap();
//! println!("{}", merged);
//! ```
//!
//! # Module Overview
//!
//! | Module | Purpose |
//! |--------|---------|
//! | [`pointcloud`] | Point cloud data structure and operations |
//! | [`cleanup`] | Scan cleanup (outliers, spikes, components) |
//! | [`denoise`] | Mesh denoising algorithms |
//! | [`multiscan`] | Multi-scan alignment and merging |
//! | [`reconstruct`] | Surface reconstruction from point clouds |
//!
//! # Complete Workflow Example
//!
//! ```no_run
//! use mesh_scan::pointcloud::PointCloud;
//! use mesh_scan::cleanup::{cleanup_scan, CleanupParams};
//! use mesh_scan::denoise::{denoise_mesh, DenoiseParams};
//! use mesh_scan::reconstruct::{to_mesh, ReconstructionParams};
//!
//! // 1. Load point cloud
//! let cloud = PointCloud::load("scan.xyz").unwrap();
//!
//! // 2. Estimate normals
//! let mut cloud = cloud;
//! cloud.estimate_normals(15).unwrap();
//! cloud.orient_normals_outward().unwrap();
//!
//! // 3. Reconstruct mesh
//! let mesh = to_mesh(&cloud).unwrap();
//!
//! // 4. Clean up the mesh
//! let cleaned = cleanup_scan(&mesh, &CleanupParams::for_object_scan()).unwrap();
//!
//! // 5. Denoise
//! let denoised = denoise_mesh(&cleaned.mesh, &DenoiseParams::for_scans()).unwrap();
//!
//! println!("Final mesh has {} vertices", denoised.mesh.vertices.len());
//! ```

#![warn(missing_docs)]
#![warn(clippy::all, clippy::pedantic, clippy::nursery)]
#![deny(clippy::unwrap_used, clippy::expect_used)]
#![allow(clippy::module_name_repetitions)]
// Allow certain pedantic lints that are too strict for this crate
#![allow(clippy::struct_excessive_bools)] // CleanupParams needs multiple bool options
#![allow(clippy::missing_const_for_fn)] // Not all functions benefit from const
#![allow(clippy::or_fun_call)] // or_insert(Vector3::zeros()) is acceptable
#![allow(clippy::cast_precision_loss)] // Expected when converting counts to f64
#![allow(clippy::cast_possible_truncation)] // Expected when converting u64 to usize
#![allow(clippy::needless_range_loop)] // Sometimes indices are clearer
#![allow(clippy::trivially_copy_pass_by_ref)] // Consistency in function signatures
#![allow(clippy::manual_let_else)] // Match expressions can be clearer
#![allow(clippy::items_after_statements)] // Sometimes helper functions are clearer after use
#![allow(clippy::unnecessary_wraps)] // Consistency in error handling APIs

pub mod cleanup;
pub mod denoise;
pub mod error;
pub mod multiscan;
pub mod pointcloud;
pub mod reconstruct;

// Re-export main types at crate root for convenience
pub use cleanup::{cleanup_scan, CleanupParams, CleanupResult};
pub use denoise::{denoise_mesh, DenoiseMethod, DenoiseParams, DenoiseResult};
pub use error::{ScanError, ScanResult};
pub use multiscan::{
    align_and_merge, align_multiple_scans, merge_scans, MergeParams, MergeResult,
    MultiAlignmentParams, MultiAlignmentResult, OverlapHandling,
};
pub use pointcloud::{CloudPoint, PointCloud};
pub use reconstruct::{
    reconstruct_surface, to_mesh, ReconstructionAlgorithm, ReconstructionParams,
    ReconstructionResult,
};

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::{IndexedMesh, Vertex};
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_point_cloud_workflow() {
        // Create point cloud
        let mut cloud = PointCloud::new();
        cloud.add_point_with_normal(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        cloud.add_point_with_normal(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        cloud.add_point_with_normal(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0));

        assert_eq!(cloud.len(), 3);
        assert!(cloud.has_normals());
    }

    #[test]
    fn test_cleanup_workflow() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        let params = CleanupParams::minimal();
        let result = cleanup_scan(&mesh, &params).unwrap();

        assert!(result.mesh.vertices.len() > 0);
    }

    #[test]
    fn test_denoise_workflow() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        let params = DenoiseParams::for_scans();
        let result = denoise_mesh(&mesh, &params).unwrap();

        assert_eq!(result.mesh.vertices.len(), 3);
    }

    #[test]
    fn test_multiscan_workflow() {
        let mut scan1 = IndexedMesh::new();
        for i in 0..5 {
            scan1.vertices.push(Vertex::from_coords(f64::from(i), 0.0, 0.0));
        }

        let mut scan2 = IndexedMesh::new();
        for i in 0..5 {
            scan2.vertices.push(Vertex::from_coords(f64::from(i) + 0.1, 0.0, 0.0));
        }

        let scans = vec![scan1, scan2];
        let alignment = align_multiple_scans(&scans, &MultiAlignmentParams::fast()).unwrap();

        assert_eq!(alignment.transforms.len(), 2);
    }

    #[test]
    fn test_reconstruction_workflow() {
        let mut cloud = PointCloud::new();
        cloud.add_point_with_normal(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        cloud.add_point_with_normal(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        cloud.add_point_with_normal(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0));

        let params = ReconstructionParams::ball_pivoting(2.0);
        let result = reconstruct_surface(&cloud, &params).unwrap();

        assert_eq!(result.mesh.vertices.len(), 3);
    }

    #[test]
    fn test_re_exports() {
        // Verify all re-exports are accessible
        let _: ScanError;
        let _: CleanupParams = CleanupParams::default();
        let _: DenoiseParams = DenoiseParams::default();
        let _: MergeParams = MergeParams::default();
        let _: MultiAlignmentParams = MultiAlignmentParams::default();
        let _: ReconstructionParams = ReconstructionParams::default();
    }
}
