//! RANSAC plane fitting algorithm.
//!
//! Robustly fits a plane to noisy point data by iteratively
//! sampling minimal sets and finding the best consensus.

use mesh_types::IndexedMesh;
use nalgebra::Vector3;
use rand::prelude::*;

use crate::error::{TransformError, TransformResult};
use crate::plane::Plane;

/// Configuration for RANSAC plane fitting.
#[derive(Debug, Clone)]
pub struct RansacConfig {
    /// Maximum number of iterations.
    pub max_iterations: usize,
    /// Distance threshold for classifying inliers.
    pub inlier_threshold: f64,
    /// Minimum fraction of points that must be inliers for success.
    pub min_inlier_ratio: f64,
    /// Optional seed for reproducible results.
    pub seed: Option<u64>,
}

impl Default for RansacConfig {
    fn default() -> Self {
        Self {
            max_iterations: 1000,
            inlier_threshold: 0.01,
            min_inlier_ratio: 0.5,
            seed: None,
        }
    }
}

impl RansacConfig {
    /// Create a new RANSAC configuration with default values.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the maximum number of iterations.
    #[must_use]
    pub const fn with_max_iterations(mut self, iterations: usize) -> Self {
        self.max_iterations = iterations;
        self
    }

    /// Set the inlier distance threshold.
    #[must_use]
    pub const fn with_inlier_threshold(mut self, threshold: f64) -> Self {
        self.inlier_threshold = threshold;
        self
    }

    /// Set the minimum inlier ratio for success.
    #[must_use]
    pub const fn with_min_inlier_ratio(mut self, ratio: f64) -> Self {
        self.min_inlier_ratio = ratio;
        self
    }

    /// Set a random seed for reproducibility.
    #[must_use]
    pub const fn with_seed(mut self, seed: u64) -> Self {
        self.seed = Some(seed);
        self
    }
}

/// Result of RANSAC plane fitting.
#[derive(Debug, Clone)]
pub struct RansacResult {
    /// The fitted plane.
    pub plane: Plane,
    /// Indices of inlier vertices.
    pub inliers: Vec<usize>,
    /// Number of iterations performed.
    pub iterations: usize,
}

impl RansacResult {
    /// Get the inlier ratio.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    // Precision loss: point counts beyond 2^52 are unsupported
    pub fn inlier_ratio(&self, total_points: usize) -> f64 {
        if total_points == 0 {
            return 0.0;
        }
        self.inliers.len() as f64 / total_points as f64
    }
}

/// Fit a plane to mesh vertices using RANSAC.
///
/// RANSAC (Random Sample Consensus) is robust to outliers by
/// iteratively sampling minimal sets of points and keeping
/// the plane with the most inliers.
///
/// # Arguments
///
/// * `mesh` - The mesh containing vertices to fit
/// * `config` - RANSAC configuration parameters
///
/// # Returns
///
/// The best fitting plane and its inliers.
///
/// # Errors
///
/// Returns an error if:
/// - The mesh has fewer than 3 vertices
/// - No valid plane is found meeting the inlier threshold
///
/// # Example
///
/// ```
/// use mesh_transform::{ransac_plane, RansacConfig};
/// use mesh_types::{IndexedMesh, Vertex};
///
/// // Create a mesh with points roughly on a plane
/// let mut mesh = IndexedMesh::new();
/// for i in 0..10 {
///     for j in 0..10 {
///         mesh.vertices.push(Vertex::from_coords(
///             i as f64,
///             j as f64,
///             0.0, // All on z=0 plane
///         ));
///     }
/// }
/// mesh.faces.push([0, 1, 10]); // Need at least one face
///
/// let config = RansacConfig::new()
///     .with_inlier_threshold(0.1)
///     .with_seed(42);
///
/// let result = ransac_plane(&mesh, &config);
/// assert!(result.is_ok());
///
/// let fitted = result.unwrap();
/// // Normal should be approximately ±Z
/// assert!(fitted.plane.normal.z.abs() > 0.99);
/// ```
pub fn ransac_plane(mesh: &IndexedMesh, config: &RansacConfig) -> TransformResult<RansacResult> {
    let n = mesh.vertices.len();

    if n < 3 {
        return Err(TransformError::InsufficientPoints {
            required: 3,
            actual: n,
        });
    }

    // Convert vertices to Vector3
    let points: Vec<Vector3<f64>> = mesh
        .vertices
        .iter()
        .map(|v| Vector3::new(v.position.x, v.position.y, v.position.z))
        .collect();

    ransac_plane_from_points(&points, config)
}

/// Fit a plane to a slice of points using RANSAC.
///
/// # Arguments
///
/// * `points` - The points to fit
/// * `config` - RANSAC configuration parameters
///
/// # Errors
///
/// Returns an error if no valid plane is found.
#[allow(clippy::cast_precision_loss)]
// Precision loss: point counts beyond 2^52 are unsupported
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
// Truncation/sign loss: min_inliers is bounded by point count which fits in usize
pub fn ransac_plane_from_points(
    points: &[Vector3<f64>],
    config: &RansacConfig,
) -> TransformResult<RansacResult> {
    let n = points.len();

    if n < 3 {
        return Err(TransformError::InsufficientPoints {
            required: 3,
            actual: n,
        });
    }

    let mut rng: Box<dyn RngCore> = if let Some(seed) = config.seed {
        Box::new(rand::rngs::StdRng::seed_from_u64(seed))
    } else {
        Box::new(rand::thread_rng())
    };

    let min_inliers = (n as f64 * config.min_inlier_ratio).ceil() as usize;
    let mut best_plane: Option<Plane> = None;
    let mut best_inliers: Vec<usize> = Vec::new();
    let mut iterations_used = 0;

    for iteration in 0..config.max_iterations {
        iterations_used = iteration + 1;

        // Sample 3 random distinct points
        let i0 = rng.gen_range(0..n);
        let mut i1 = rng.gen_range(0..n);
        while i1 == i0 {
            i1 = rng.gen_range(0..n);
        }
        let mut i2 = rng.gen_range(0..n);
        while i2 == i0 || i2 == i1 {
            i2 = rng.gen_range(0..n);
        }

        // Try to create a plane from these points
        let Some(candidate) = Plane::from_points(points[i0], points[i1], points[i2]) else {
            continue; // Collinear points, try again
        };

        // Count inliers
        let inliers: Vec<usize> = (0..n)
            .filter(|&i| candidate.is_inlier(points[i], config.inlier_threshold))
            .collect();

        // Update best if this is better
        if inliers.len() > best_inliers.len() {
            best_plane = Some(candidate);
            best_inliers = inliers;

            // Early termination if we have enough inliers
            if best_inliers.len() >= min_inliers && best_inliers.len() >= n * 9 / 10 {
                break;
            }
        }
    }

    // Check if we found a good enough plane
    let Some(plane) = best_plane else {
        return Err(TransformError::RansacFailed {
            iterations: iterations_used,
        });
    };

    if best_inliers.len() < min_inliers {
        return Err(TransformError::RansacFailed {
            iterations: iterations_used,
        });
    }

    // Optionally: refine the plane using all inliers
    let refined_plane = refine_plane_from_inliers(points, &best_inliers).unwrap_or(plane);

    Ok(RansacResult {
        plane: refined_plane,
        inliers: best_inliers,
        iterations: iterations_used,
    })
}

/// Refine a plane fit using least-squares on the inliers.
#[allow(clippy::cast_precision_loss)]
// Precision loss: inlier counts beyond 2^52 are unsupported
#[allow(clippy::suspicious_operation_groupings)]
// This is correct math: computing 2x2 minors of a 3x3 covariance matrix
#[allow(clippy::similar_names)]
// Similar names: cov_xx, cov_xy, cov_xz, etc. are standard covariance matrix notation
fn refine_plane_from_inliers(points: &[Vector3<f64>], inliers: &[usize]) -> Option<Plane> {
    if inliers.len() < 3 {
        return None;
    }

    // Compute centroid of inliers
    let mut centroid = Vector3::zeros();
    for &i in inliers {
        centroid += points[i];
    }
    centroid /= inliers.len() as f64;

    // Compute covariance matrix elements
    let mut cov_xx = 0.0;
    let mut cov_xy = 0.0;
    let mut cov_xz = 0.0;
    let mut cov_yy = 0.0;
    let mut cov_yz = 0.0;
    let mut cov_zz = 0.0;

    for &i in inliers {
        let d = points[i] - centroid;
        cov_xx += d.x * d.x;
        cov_xy += d.x * d.y;
        cov_xz += d.x * d.z;
        cov_yy += d.y * d.y;
        cov_yz += d.y * d.z;
        cov_zz += d.z * d.z;
    }

    // Find the eigenvector with smallest eigenvalue
    // Using direct formula for 3x3 symmetric matrix - computing 2x2 minors
    let det_x = cov_yy * cov_zz - cov_yz * cov_yz;
    let det_y = cov_xx * cov_zz - cov_xz * cov_xz;
    let det_z = cov_xx * cov_yy - cov_xy * cov_xy;

    // The normal is the eigenvector corresponding to smallest eigenvalue
    // As an approximation, use the axis with largest determinant
    let normal = if det_x >= det_y && det_x >= det_z {
        Vector3::new(det_x, cov_xz * cov_yz - cov_xy * cov_zz, cov_xy * cov_yz - cov_xz * cov_yy)
    } else if det_y >= det_x && det_y >= det_z {
        Vector3::new(cov_xz * cov_yz - cov_xy * cov_zz, det_y, cov_xy * cov_xz - cov_yz * cov_xx)
    } else {
        Vector3::new(cov_xy * cov_yz - cov_xz * cov_yy, cov_xy * cov_xz - cov_yz * cov_xx, det_z)
    };

    Plane::new(centroid, normal)
}

/// Create a default `RansacResult` for testing fallback scenarios.
#[cfg(test)]
fn default_ransac_result() -> RansacResult {
    RansacResult {
        plane: Plane {
            point: Vector3::zeros(),
            normal: Vector3::z(),
        },
        inliers: vec![],
        iterations: 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Vertex;

    fn create_planar_mesh() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        // Grid of points on z=0 plane
        for i in 0u32..5 {
            for j in 0u32..5 {
                mesh.vertices.push(Vertex::from_coords(
                    f64::from(i),
                    f64::from(j),
                    0.0,
                ));
            }
        }
        mesh.faces.push([0, 1, 5]);
        mesh
    }

    fn create_planar_mesh_with_noise() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        // Grid of points on z=0 plane with some noise
        let noise = [0.001, -0.002, 0.001, 0.0, -0.001];
        for i in 0u32..5 {
            for j in 0u32..5 {
                mesh.vertices.push(Vertex::from_coords(
                    f64::from(i),
                    f64::from(j),
                    noise[(i + j) as usize % 5],
                ));
            }
        }
        // Add one outlier
        mesh.vertices.push(Vertex::from_coords(2.0, 2.0, 5.0));
        mesh.faces.push([0, 1, 5]);
        mesh
    }

    #[test]
    fn ransac_perfect_plane() {
        let mesh = create_planar_mesh();
        let config = RansacConfig::new()
            .with_inlier_threshold(0.01)
            .with_seed(42);

        let result = ransac_plane(&mesh, &config);
        assert!(result.is_ok());

        let fitted = result.unwrap_or_else(|_| default_ransac_result());

        // Normal should be ±Z
        assert!(
            fitted.plane.normal.z.abs() > 0.99,
            "Expected Z-normal, got {:?}",
            fitted.plane.normal
        );

        // All points should be inliers
        assert_eq!(fitted.inliers.len(), 25);
    }

    #[test]
    fn ransac_with_outlier() {
        let mesh = create_planar_mesh_with_noise();
        let config = RansacConfig::new()
            .with_inlier_threshold(0.01)
            .with_seed(42);

        let result = ransac_plane(&mesh, &config);
        assert!(result.is_ok());

        let fitted = result.unwrap_or_else(|_| default_ransac_result());

        // Outlier should not be included
        assert!(fitted.inliers.len() >= 25);
        assert!(!fitted.inliers.contains(&25)); // Index of outlier
    }

    #[test]
    fn ransac_insufficient_points() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));

        let config = RansacConfig::default();
        let result = ransac_plane(&mesh, &config);

        assert!(result.is_err());
        assert!(matches!(
            result,
            Err(TransformError::InsufficientPoints { .. })
        ));
    }

    #[test]
    fn ransac_config_builder() {
        let config = RansacConfig::new()
            .with_max_iterations(500)
            .with_inlier_threshold(0.05)
            .with_min_inlier_ratio(0.6)
            .with_seed(123);

        assert_eq!(config.max_iterations, 500);
        assert_relative_eq!(config.inlier_threshold, 0.05);
        assert_relative_eq!(config.min_inlier_ratio, 0.6);
        assert_eq!(config.seed, Some(123));
    }

    #[test]
    fn ransac_reproducible_with_seed() {
        let mesh = create_planar_mesh_with_noise();
        let config = RansacConfig::new().with_seed(12345);

        let result1 = ransac_plane(&mesh, &config);
        let result2 = ransac_plane(&mesh, &config);

        assert!(result1.is_ok());
        assert!(result2.is_ok());

        let r1 = result1.unwrap_or_else(|_| default_ransac_result());
        let r2 = result2.unwrap_or_else(|_| default_ransac_result());

        // With same seed, should get same results
        assert_eq!(r1.inliers.len(), r2.inliers.len());
    }

    #[test]
    fn ransac_from_points_direct() {
        let points = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
            Vector3::new(0.5, 0.5, 0.0),
        ];

        let config = RansacConfig::new()
            .with_inlier_threshold(0.01)
            .with_seed(42);

        let result = ransac_plane_from_points(&points, &config);
        assert!(result.is_ok());

        let fitted = result.unwrap_or_else(|_| default_ransac_result());

        assert!(fitted.plane.normal.z.abs() > 0.99);
        assert_eq!(fitted.inliers.len(), 5);
    }

    #[test]
    fn inlier_ratio() {
        let result = RansacResult {
            plane: Plane {
                point: Vector3::zeros(),
                normal: Vector3::z(),
            },
            inliers: vec![0, 1, 2, 3, 4],
            iterations: 10,
        };

        assert_relative_eq!(result.inlier_ratio(10), 0.5, epsilon = 1e-10);
        assert_relative_eq!(result.inlier_ratio(5), 1.0, epsilon = 1e-10);
        assert_relative_eq!(result.inlier_ratio(0), 0.0, epsilon = 1e-10);
    }
}
