//! Surface reconstruction from point clouds.
//!
//! This module provides algorithms for converting point clouds into
//! triangle meshes (surface reconstruction):
//!
//! - **Ball Pivoting** - Fast reconstruction for clean, dense point clouds
//!
//! # Quick Start
//!
//! ```
//! use mesh_scan::reconstruct::{reconstruct_surface, ReconstructionParams, ReconstructionAlgorithm};
//! use mesh_scan::pointcloud::PointCloud;
//! use nalgebra::{Point3, Vector3};
//!
//! // Create point cloud with normals
//! let mut cloud = PointCloud::new();
//! cloud.add_point_with_normal(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
//! cloud.add_point_with_normal(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
//! cloud.add_point_with_normal(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
//!
//! // Reconstruct surface
//! let params = ReconstructionParams::ball_pivoting(2.0);
//! let result = reconstruct_surface(&cloud, &params).unwrap();
//! println!("{}", result);
//! ```
//!
//! # Algorithm Selection
//!
//! | Algorithm | Best For | Requirements |
//! |-----------|----------|--------------|
//! | Ball Pivoting | Dense, clean scans | Oriented normals |
//!
//! # Workflow
//!
//! 1. **Prepare Point Cloud**: Ensure the point cloud has oriented normals
//!    (use `PointCloud::estimate_normals` and `orient_normals_outward`)
//!
//! 2. **Choose Parameters**: Select algorithm and tune parameters based on
//!    point density (use `BallPivotingParams::estimate_radius`)
//!
//! 3. **Reconstruct**: Call `reconstruct_surface` to generate the mesh

pub mod ball_pivoting;

pub use ball_pivoting::{ball_pivoting, BallPivotingParams, BallPivotingResult};

use mesh_types::IndexedMesh;

use crate::error::{ScanError, ScanResult};
use crate::pointcloud::PointCloud;

/// Reconstruction algorithm to use.
#[derive(Debug, Clone)]
pub enum ReconstructionAlgorithm {
    /// Ball Pivoting Algorithm with given radius.
    BallPivoting {
        /// Ball radius (should be slightly larger than average point spacing).
        radius: f64,
    },

    /// Ball Pivoting with auto-estimated radius.
    BallPivotingAuto {
        /// Number of neighbors for radius estimation (default: 10).
        k_neighbors: usize,

        /// Scale factor for estimated radius (default: 1.0).
        radius_scale: f64,
    },
}

impl Default for ReconstructionAlgorithm {
    fn default() -> Self {
        Self::BallPivotingAuto {
            k_neighbors: 10,
            radius_scale: 1.0,
        }
    }
}

/// Parameters for surface reconstruction.
#[derive(Debug, Clone)]
pub struct ReconstructionParams {
    /// The reconstruction algorithm to use.
    pub algorithm: ReconstructionAlgorithm,

    /// Whether to estimate normals if not present (default: true).
    pub estimate_normals_if_missing: bool,

    /// Number of neighbors for normal estimation (default: 15).
    pub normal_estimation_k: usize,

    /// Whether to orient normals outward after estimation (default: true).
    pub orient_normals: bool,
}

impl Default for ReconstructionParams {
    fn default() -> Self {
        Self {
            algorithm: ReconstructionAlgorithm::default(),
            estimate_normals_if_missing: true,
            normal_estimation_k: 15,
            orient_normals: true,
        }
    }
}

impl ReconstructionParams {
    /// Creates new parameters with defaults.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates parameters for ball pivoting with given radius.
    #[must_use]
    pub fn ball_pivoting(radius: f64) -> Self {
        Self {
            algorithm: ReconstructionAlgorithm::BallPivoting { radius },
            ..Self::default()
        }
    }

    /// Creates parameters for ball pivoting with auto-estimated radius.
    #[must_use]
    pub fn ball_pivoting_auto() -> Self {
        Self {
            algorithm: ReconstructionAlgorithm::BallPivotingAuto {
                k_neighbors: 10,
                radius_scale: 1.0,
            },
            ..Self::default()
        }
    }

    /// Sets the reconstruction algorithm.
    #[must_use]
    pub const fn with_algorithm(mut self, algorithm: ReconstructionAlgorithm) -> Self {
        self.algorithm = algorithm;
        self
    }

    /// Sets whether to estimate normals if missing.
    #[must_use]
    pub const fn with_estimate_normals(mut self, estimate: bool) -> Self {
        self.estimate_normals_if_missing = estimate;
        self
    }

    /// Sets the number of neighbors for normal estimation.
    #[must_use]
    pub const fn with_normal_estimation_k(mut self, k: usize) -> Self {
        self.normal_estimation_k = k;
        self
    }

    /// Sets whether to orient normals.
    #[must_use]
    pub const fn with_orient_normals(mut self, orient: bool) -> Self {
        self.orient_normals = orient;
        self
    }

    /// Creates parameters for fast reconstruction.
    #[must_use]
    pub const fn fast() -> Self {
        Self {
            algorithm: ReconstructionAlgorithm::BallPivotingAuto {
                k_neighbors: 5,
                radius_scale: 1.2,
            },
            estimate_normals_if_missing: true,
            normal_estimation_k: 8,
            orient_normals: true,
        }
    }

    /// Creates parameters for high-quality reconstruction.
    #[must_use]
    pub const fn high_quality() -> Self {
        Self {
            algorithm: ReconstructionAlgorithm::BallPivotingAuto {
                k_neighbors: 15,
                radius_scale: 0.9,
            },
            estimate_normals_if_missing: true,
            normal_estimation_k: 20,
            orient_normals: true,
        }
    }
}

/// Result of surface reconstruction.
#[derive(Debug, Clone)]
pub struct ReconstructionResult {
    /// The reconstructed mesh.
    pub mesh: IndexedMesh,

    /// Number of triangles created.
    pub triangle_count: usize,

    /// Number of points that were not connected.
    pub orphan_points: usize,

    /// Estimated surface coverage ratio (0.0 - 1.0).
    pub coverage_ratio: f64,

    /// Number of boundary edges (open edges).
    pub boundary_edge_count: usize,

    /// Whether normals were estimated.
    pub normals_estimated: bool,

    /// The actual radius used (for ball pivoting).
    pub actual_radius: Option<f64>,
}

impl std::fmt::Display for ReconstructionResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Reconstruction: {} triangles, {:.1}% coverage, {} boundary edges",
            self.triangle_count,
            self.coverage_ratio * 100.0,
            self.boundary_edge_count
        )
    }
}

/// Reconstructs a mesh from a point cloud.
///
/// # Arguments
///
/// * `cloud` - Point cloud to reconstruct
/// * `params` - Reconstruction parameters
///
/// # Returns
///
/// The reconstructed mesh and statistics.
///
/// # Errors
///
/// Returns an error if:
/// - Point cloud is empty
/// - Normal estimation fails
/// - Reconstruction fails
///
/// # Example
///
/// ```
/// use mesh_scan::reconstruct::{reconstruct_surface, ReconstructionParams};
/// use mesh_scan::pointcloud::PointCloud;
/// use nalgebra::{Point3, Vector3};
///
/// let mut cloud = PointCloud::new();
/// cloud.add_point_with_normal(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
/// cloud.add_point_with_normal(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
/// cloud.add_point_with_normal(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
///
/// let result = reconstruct_surface(&cloud, &ReconstructionParams::default()).unwrap();
/// println!("{}", result);
/// ```
pub fn reconstruct_surface(
    cloud: &PointCloud,
    params: &ReconstructionParams,
) -> ScanResult<ReconstructionResult> {
    if cloud.is_empty() {
        return Err(ScanError::EmptyPointCloud);
    }

    // Estimate normals if needed
    let mut working_cloud = cloud.clone();
    let normals_estimated = if !working_cloud.has_normals() && params.estimate_normals_if_missing {
        working_cloud.estimate_normals(params.normal_estimation_k)?;
        if params.orient_normals {
            working_cloud.orient_normals_outward()?;
        }
        true
    } else {
        false
    };

    // Run reconstruction algorithm
    match &params.algorithm {
        ReconstructionAlgorithm::BallPivoting { radius } => {
            let bp_params = BallPivotingParams::new(*radius);
            let result = ball_pivoting(&working_cloud, &bp_params)?;

            Ok(ReconstructionResult {
                mesh: result.mesh,
                triangle_count: result.triangle_count,
                orphan_points: result.orphan_points,
                coverage_ratio: result.coverage_ratio,
                boundary_edge_count: result.boundary_edge_count,
                normals_estimated,
                actual_radius: Some(*radius),
            })
        }

        ReconstructionAlgorithm::BallPivotingAuto {
            k_neighbors,
            radius_scale,
        } => {
            let estimated_radius =
                BallPivotingParams::estimate_radius(&working_cloud, *k_neighbors) * radius_scale;
            let bp_params = BallPivotingParams::new(estimated_radius);
            let result = ball_pivoting(&working_cloud, &bp_params)?;

            Ok(ReconstructionResult {
                mesh: result.mesh,
                triangle_count: result.triangle_count,
                orphan_points: result.orphan_points,
                coverage_ratio: result.coverage_ratio,
                boundary_edge_count: result.boundary_edge_count,
                normals_estimated,
                actual_radius: Some(estimated_radius),
            })
        }
    }
}

/// Convenience method to convert a point cloud to a mesh.
///
/// Uses default reconstruction parameters.
///
/// # Arguments
///
/// * `cloud` - Point cloud to convert
///
/// # Returns
///
/// The reconstructed mesh.
///
/// # Errors
///
/// Returns an error if reconstruction fails.
///
/// # Example
///
/// ```
/// use mesh_scan::reconstruct::to_mesh;
/// use mesh_scan::pointcloud::PointCloud;
/// use nalgebra::{Point3, Vector3};
///
/// let mut cloud = PointCloud::new();
/// cloud.add_point_with_normal(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
/// cloud.add_point_with_normal(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
/// cloud.add_point_with_normal(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
///
/// let mesh = to_mesh(&cloud).unwrap();
/// println!("Created mesh with {} vertices", mesh.vertices.len());
/// ```
pub fn to_mesh(cloud: &PointCloud) -> ScanResult<IndexedMesh> {
    let result = reconstruct_surface(cloud, &ReconstructionParams::default())?;
    Ok(result.mesh)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, Vector3};

    fn make_triangle_cloud() -> PointCloud {
        let mut cloud = PointCloud::new();
        cloud.add_point_with_normal(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        cloud.add_point_with_normal(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        cloud.add_point_with_normal(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        cloud
    }

    #[test]
    fn test_reconstruction_params_default() {
        let params = ReconstructionParams::default();
        assert!(params.estimate_normals_if_missing);
        assert!(params.orient_normals);
    }

    #[test]
    fn test_reconstruction_params_builder() {
        let params = ReconstructionParams::ball_pivoting(2.0)
            .with_estimate_normals(false)
            .with_orient_normals(false);

        assert!(matches!(
            params.algorithm,
            ReconstructionAlgorithm::BallPivoting { radius } if (radius - 2.0).abs() < 1e-10
        ));
        assert!(!params.estimate_normals_if_missing);
    }

    #[test]
    fn test_reconstruction_params_presets() {
        let fast = ReconstructionParams::fast();
        assert!(fast.estimate_normals_if_missing);

        let hq = ReconstructionParams::high_quality();
        assert!(hq.normal_estimation_k > fast.normal_estimation_k);
    }

    #[test]
    fn test_reconstruct_empty_cloud() {
        let cloud = PointCloud::new();
        let result = reconstruct_surface(&cloud, &ReconstructionParams::default());
        assert!(matches!(result, Err(ScanError::EmptyPointCloud)));
    }

    #[test]
    fn test_reconstruct_simple() {
        let cloud = make_triangle_cloud();
        let params = ReconstructionParams::ball_pivoting(2.0);
        let result = reconstruct_surface(&cloud, &params).unwrap();

        assert_eq!(result.mesh.vertices.len(), 3);
        assert!(!result.normals_estimated); // Cloud had normals
        assert!(result.actual_radius.is_some());
    }

    #[test]
    fn test_reconstruct_auto_radius() {
        let cloud = make_triangle_cloud();
        let params = ReconstructionParams::ball_pivoting_auto();
        let result = reconstruct_surface(&cloud, &params).unwrap();

        assert!(result.actual_radius.is_some());
    }

    #[test]
    fn test_reconstruct_with_normal_estimation() {
        let mut cloud = PointCloud::new();
        // Add points without normals
        for i in 0..3 {
            for j in 0..3 {
                cloud.add_point(Point3::new(f64::from(i), f64::from(j), 0.0));
            }
        }

        let params = ReconstructionParams::ball_pivoting(2.0).with_estimate_normals(true);
        let result = reconstruct_surface(&cloud, &params).unwrap();

        assert!(result.normals_estimated);
    }

    #[test]
    fn test_to_mesh() {
        let cloud = make_triangle_cloud();
        let mesh = to_mesh(&cloud).unwrap();
        assert_eq!(mesh.vertices.len(), 3);
    }

    #[test]
    fn test_reconstruction_result_display() {
        let result = ReconstructionResult {
            mesh: IndexedMesh::new(),
            triangle_count: 50,
            orphan_points: 3,
            coverage_ratio: 0.85,
            boundary_edge_count: 12,
            normals_estimated: true,
            actual_radius: Some(1.5),
        };

        let display = format!("{result}");
        assert!(display.contains("50"));
        assert!(display.contains("85.0%"));
        assert!(display.contains("12"));
    }

    #[test]
    fn test_algorithm_default() {
        let algo = ReconstructionAlgorithm::default();
        assert!(matches!(
            algo,
            ReconstructionAlgorithm::BallPivotingAuto { .. }
        ));
    }
}
