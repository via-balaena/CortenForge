//! Normal estimation and orientation for point clouds.
//!
//! This module provides functions for estimating surface normals from point
//! cloud data using PCA (Principal Component Analysis) and for orienting
//! normals consistently.
//!
//! # Example
//!
//! ```
//! use mesh_scan::pointcloud::PointCloud;
//! use nalgebra::Point3;
//!
//! // Create a planar point cloud (with small z variation to avoid KD-tree issues)
//! let positions: Vec<_> = (0..20)
//!     .flat_map(|i| (0..20).map(move |j| {
//!         let z = (i * 20 + j) as f64 * 0.0001;
//!         Point3::new(i as f64 * 0.1, j as f64 * 0.1, z)
//!     }))
//!     .collect();
//!
//! let mut cloud = PointCloud::from_positions(&positions);
//!
//! // Estimate normals using 10 nearest neighbors
//! cloud.estimate_normals(10).unwrap();
//!
//! // Orient normals consistently outward
//! cloud.orient_normals_outward().unwrap();
//!
//! assert!(cloud.has_normals());
//! ```

use kiddo::{KdTree, SquaredEuclidean};
use nalgebra::{Matrix3, Point3, SymmetricEigen, Vector3};

use super::PointCloud;
use crate::error::{ScanError, ScanResult};

impl PointCloud {
    /// Estimates normals for all points using PCA on k nearest neighbors.
    ///
    /// For each point, finds the k nearest neighbors and computes the normal
    /// as the eigenvector corresponding to the smallest eigenvalue of the
    /// covariance matrix.
    ///
    /// # Arguments
    ///
    /// * `k` - Number of neighbors to use for normal estimation. Typical values
    ///   are 10-30. Higher values give smoother normals but may lose detail.
    ///
    /// # Errors
    ///
    /// Returns an error if the point cloud has fewer than 3 points or k is 0.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    /// use nalgebra::Point3;
    ///
    /// // Create points with variation in all dimensions
    /// let positions: Vec<_> = (0..100)
    ///     .map(|i| Point3::new(i as f64 * 0.1, i as f64 * 0.001, i as f64 * 0.002))
    ///     .collect();
    /// let mut cloud = PointCloud::from_positions(&positions);
    ///
    /// cloud.estimate_normals(10).unwrap();
    /// assert!(cloud.has_normals());
    /// ```
    pub fn estimate_normals(&mut self, k: usize) -> ScanResult<()> {
        if self.points.len() < 3 {
            return Err(ScanError::InsufficientPoints {
                required: 3,
                actual: self.points.len(),
            });
        }

        if k == 0 {
            return Err(ScanError::InvalidParameter {
                reason: "k must be greater than 0".to_string(),
            });
        }

        // Build KD-tree
        let kdtree = build_kdtree(&self.points)?;

        // Compute normals in parallel
        let normals: Vec<Vector3<f64>> = self
            .points
            .iter()
            .map(|point| estimate_point_normal(&point.position, &kdtree, &self.points, k))
            .collect();

        // Assign normals to points
        for (point, normal) in self.points.iter_mut().zip(normals) {
            point.normal = Some(normal);
        }

        Ok(())
    }

    /// Orients all normals to point outward (away from the centroid).
    ///
    /// Uses a simple heuristic: flip normals that point toward the centroid.
    /// This works well for convex or mostly convex point clouds.
    ///
    /// # Errors
    ///
    /// Returns an error if the point cloud is empty or has no normals.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    /// use nalgebra::Point3;
    ///
    /// // Create points with variation in all dimensions
    /// let positions: Vec<_> = (0..100)
    ///     .map(|i| Point3::new(i as f64 * 0.1, i as f64 * 0.001, i as f64 * 0.002))
    ///     .collect();
    /// let mut cloud = PointCloud::from_positions(&positions);
    /// cloud.estimate_normals(10).unwrap();
    /// cloud.orient_normals_outward().unwrap();
    /// ```
    pub fn orient_normals_outward(&mut self) -> ScanResult<()> {
        if self.points.is_empty() {
            return Err(ScanError::EmptyPointCloud);
        }

        if !self.has_normals() {
            return Err(ScanError::NormalEstimationFailed {
                reason: "point cloud has no normals to orient".to_string(),
            });
        }

        let centroid = self.centroid().ok_or(ScanError::EmptyPointCloud)?;

        for point in &mut self.points {
            if let Some(normal) = &mut point.normal {
                // Vector from centroid to point
                let outward = point.position - centroid;

                // Flip normal if it points inward
                if normal.dot(&outward) < 0.0 {
                    *normal = -*normal;
                }
            }
        }

        Ok(())
    }

    /// Orients normals consistently using propagation from a seed point.
    ///
    /// Starts from the point with the highest z-coordinate (assumed to be
    /// on the "top" of the object) and propagates orientation to neighbors.
    ///
    /// This method is more robust than `orient_normals_outward` for non-convex
    /// objects but is slower.
    ///
    /// # Errors
    ///
    /// Returns an error if the point cloud is empty or has no normals.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_scan::pointcloud::PointCloud;
    /// use nalgebra::Point3;
    ///
    /// // Create points with variation in all dimensions
    /// let positions: Vec<_> = (0..100)
    ///     .map(|i| Point3::new(i as f64 * 0.1, i as f64 * 0.001, i as f64 * 0.002))
    ///     .collect();
    /// let mut cloud = PointCloud::from_positions(&positions);
    /// cloud.estimate_normals(10).unwrap();
    /// cloud.orient_normals_consistent(10).unwrap();
    /// ```
    pub fn orient_normals_consistent(&mut self, k: usize) -> ScanResult<()> {
        if self.points.is_empty() {
            return Err(ScanError::EmptyPointCloud);
        }

        if !self.has_normals() {
            return Err(ScanError::NormalEstimationFailed {
                reason: "point cloud has no normals to orient".to_string(),
            });
        }

        // Build KD-tree
        let kdtree = build_kdtree(&self.points)?;

        // Find seed point (highest z)
        let seed_idx = self
            .points
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| {
                a.position
                    .z
                    .partial_cmp(&b.position.z)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .ok_or(ScanError::EmptyPointCloud)?;

        // Ensure seed normal points "up"
        if let Some(normal) = &mut self.points[seed_idx].normal {
            if normal.z < 0.0 {
                *normal = -*normal;
            }
        }

        // BFS propagation
        let mut visited = vec![false; self.points.len()];
        let mut queue = std::collections::VecDeque::new();

        visited[seed_idx] = true;
        queue.push_back(seed_idx);

        while let Some(current_idx) = queue.pop_front() {
            let current_point = &self.points[current_idx];
            let current_normal = current_point.normal.unwrap_or(Vector3::z());
            let query_point = [
                current_point.position.x,
                current_point.position.y,
                current_point.position.z,
            ];

            // Find k nearest neighbors
            let neighbors = kdtree.nearest_n::<SquaredEuclidean>(&query_point, k);

            for neighbor in neighbors {
                let neighbor_idx = neighbor.item as usize;

                if visited[neighbor_idx] {
                    continue;
                }

                visited[neighbor_idx] = true;

                // Orient neighbor normal to be consistent with current
                if let Some(neighbor_normal) = &mut self.points[neighbor_idx].normal {
                    if neighbor_normal.dot(&current_normal) < 0.0 {
                        *neighbor_normal = -*neighbor_normal;
                    }
                }

                queue.push_back(neighbor_idx);
            }
        }

        Ok(())
    }
}

/// Builds a KD-tree from the point cloud.
fn build_kdtree(points: &[super::CloudPoint]) -> ScanResult<KdTree<f64, 3>> {
    let mut kdtree: KdTree<f64, 3> = KdTree::new();

    for (i, point) in points.iter().enumerate() {
        let coords = [point.position.x, point.position.y, point.position.z];
        #[allow(clippy::cast_possible_truncation)]
        let idx = i as u64;
        kdtree.add(&coords, idx);
    }

    Ok(kdtree)
}

/// Estimates the normal for a single point using PCA.
fn estimate_point_normal(
    point: &Point3<f64>,
    kdtree: &KdTree<f64, 3>,
    points: &[super::CloudPoint],
    k: usize,
) -> Vector3<f64> {
    let query = [point.x, point.y, point.z];
    let neighbors = kdtree.nearest_n::<SquaredEuclidean>(&query, k);

    if neighbors.len() < 3 {
        // Not enough neighbors, return default normal
        return Vector3::z();
    }

    // Collect neighbor positions
    let neighbor_positions: Vec<Point3<f64>> = neighbors
        .iter()
        .map(|n| points[n.item as usize].position)
        .collect();

    // Compute centroid
    let centroid: Vector3<f64> = neighbor_positions.iter().map(|p| p.coords).sum();
    #[allow(clippy::cast_precision_loss)]
    let centroid = centroid / neighbor_positions.len() as f64;

    // Build covariance matrix
    let mut cov = Matrix3::zeros();
    for p in &neighbor_positions {
        let diff = p.coords - centroid;
        cov += diff * diff.transpose();
    }

    // Compute eigenvalues and eigenvectors
    let eigen = SymmetricEigen::new(cov);
    let eigenvalues = eigen.eigenvalues;
    let eigenvectors = eigen.eigenvectors;

    // Find smallest eigenvalue index
    let min_idx = if eigenvalues[0] <= eigenvalues[1] && eigenvalues[0] <= eigenvalues[2] {
        0
    } else if eigenvalues[1] <= eigenvalues[2] {
        1
    } else {
        2
    };

    // Normal is eigenvector with smallest eigenvalue
    let normal = eigenvectors.column(min_idx);
    let normal = Vector3::new(normal[0], normal[1], normal[2]);

    // Normalize
    let norm = normal.norm();
    if norm > 1e-10 {
        normal / norm
    } else {
        Vector3::z()
    }
}

/// Parameters for normal estimation.
#[derive(Debug, Clone)]
pub struct NormalEstimationParams {
    /// Number of neighbors to use for normal estimation. Default: 20.
    pub k_neighbors: usize,

    /// Whether to orient normals outward after estimation. Default: true.
    pub orient_outward: bool,

    /// Whether to use consistent orientation propagation. Default: false.
    /// This is slower but better for non-convex objects.
    pub consistent_orientation: bool,
}

impl Default for NormalEstimationParams {
    fn default() -> Self {
        Self {
            k_neighbors: 20,
            orient_outward: true,
            consistent_orientation: false,
        }
    }
}

impl NormalEstimationParams {
    /// Creates new parameters with defaults.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the number of neighbors.
    #[must_use]
    pub const fn with_k_neighbors(mut self, k: usize) -> Self {
        self.k_neighbors = k;
        self
    }

    /// Sets whether to orient normals outward.
    #[must_use]
    pub const fn with_orient_outward(mut self, orient: bool) -> Self {
        self.orient_outward = orient;
        self
    }

    /// Sets whether to use consistent orientation propagation.
    #[must_use]
    pub const fn with_consistent_orientation(mut self, consistent: bool) -> Self {
        self.consistent_orientation = consistent;
        self
    }
}

/// Estimates normals for a point cloud with configurable parameters.
///
/// # Errors
///
/// Returns an error if the point cloud is too small or parameters are invalid.
///
/// # Example
///
/// ```
/// use mesh_scan::pointcloud::PointCloud;
/// use mesh_scan::pointcloud::normals::{estimate_normals_with_params, NormalEstimationParams};
/// use nalgebra::Point3;
///
/// // Create points with variation in all dimensions
/// let positions: Vec<_> = (0..100)
///     .map(|i| Point3::new(i as f64 * 0.1, i as f64 * 0.001, i as f64 * 0.002))
///     .collect();
/// let mut cloud = PointCloud::from_positions(&positions);
///
/// let params = NormalEstimationParams::new()
///     .with_k_neighbors(15)
///     .with_orient_outward(true);
///
/// estimate_normals_with_params(&mut cloud, &params).unwrap();
/// ```
pub fn estimate_normals_with_params(
    cloud: &mut PointCloud,
    params: &NormalEstimationParams,
) -> ScanResult<()> {
    cloud.estimate_normals(params.k_neighbors)?;

    if params.consistent_orientation {
        cloud.orient_normals_consistent(params.k_neighbors)?;
    } else if params.orient_outward {
        cloud.orient_normals_outward()?;
    }

    Ok(())
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::cast_lossless,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::redundant_clone,
    clippy::needless_collect
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn make_planar_cloud(n: usize) -> PointCloud {
        // Add small z variations to avoid kiddo KD-tree axis collision
        let positions: Vec<_> = (0..n)
            .flat_map(|i| {
                (0..n).map(move |j| {
                    let z = (i * n + j) as f64 * 0.0001; // Small variation
                    Point3::new(f64::from(i as u32), f64::from(j as u32), z)
                })
            })
            .collect();
        PointCloud::from_positions(&positions)
    }

    fn make_sphere_cloud(n: usize, radius: f64) -> PointCloud {
        use std::f64::consts::PI;

        let mut positions = Vec::with_capacity(n * n);

        for i in 0..n {
            let theta = PI * f64::from(i as u32) / f64::from((n - 1) as u32);
            for j in 0..n {
                let phi = 2.0 * PI * f64::from(j as u32) / f64::from(n as u32);
                let x = radius * theta.sin() * phi.cos();
                let y = radius * theta.sin() * phi.sin();
                let z = radius * theta.cos();
                positions.push(Point3::new(x, y, z));
            }
        }

        PointCloud::from_positions(&positions)
    }

    #[test]
    fn test_estimate_normals_planar() {
        let mut cloud = make_planar_cloud(10);
        cloud.estimate_normals(10).unwrap();

        assert!(cloud.has_normals());

        // All normals should be approximately (0, 0, ±1) for a planar cloud
        for point in &cloud.points {
            let normal = point.normal.unwrap();
            assert!(normal.x.abs() < 0.1);
            assert!(normal.y.abs() < 0.1);
            assert!(normal.z.abs() > 0.9);
        }
    }

    #[test]
    fn test_estimate_normals_insufficient_points() {
        let mut cloud =
            PointCloud::from_positions(&[Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);

        let result = cloud.estimate_normals(10);
        assert!(matches!(result, Err(ScanError::InsufficientPoints { .. })));
    }

    #[test]
    fn test_estimate_normals_invalid_k() {
        let mut cloud = make_planar_cloud(10);
        let result = cloud.estimate_normals(0);
        assert!(matches!(result, Err(ScanError::InvalidParameter { .. })));
    }

    #[test]
    fn test_orient_normals_outward() {
        let mut cloud = make_sphere_cloud(10, 1.0);
        cloud.estimate_normals(10).unwrap();
        cloud.orient_normals_outward().unwrap();

        // For a sphere centered at origin, normals should point outward
        // (in the same direction as the position vector)
        // Check that most normals point outward (allowing for edge cases at poles)
        let mut outward_count = 0;
        let mut total_count = 0;

        for point in &cloud.points {
            let normal = point.normal.unwrap();
            let outward = point.position.coords;
            let outward_norm = outward.norm();

            // Skip points very close to origin (degenerate case)
            if outward_norm < 0.01 {
                continue;
            }

            total_count += 1;
            let outward_dir = outward / outward_norm;

            // Dot product should be positive (same direction)
            if normal.dot(&outward_dir) > 0.0 {
                outward_count += 1;
            }
        }

        // At least 80% should point outward (allowing for edge cases)
        let ratio = f64::from(outward_count) / f64::from(total_count);
        assert!(
            ratio >= 0.8,
            "Expected most normals to point outward, got {outward_count}/{total_count} = {ratio}"
        );
    }

    #[test]
    fn test_orient_normals_empty_cloud() {
        let mut cloud = PointCloud::new();
        let result = cloud.orient_normals_outward();
        assert!(matches!(result, Err(ScanError::EmptyPointCloud)));
    }

    #[test]
    fn test_orient_normals_no_normals() {
        let mut cloud = PointCloud::from_positions(&[Point3::origin()]);
        let result = cloud.orient_normals_outward();
        assert!(matches!(
            result,
            Err(ScanError::NormalEstimationFailed { .. })
        ));
    }

    #[test]
    fn test_orient_normals_consistent() {
        let mut cloud = make_sphere_cloud(10, 1.0);
        cloud.estimate_normals(10).unwrap();
        cloud.orient_normals_consistent(10).unwrap();

        // Check that normals are consistent
        assert!(cloud.has_normals());
    }

    #[test]
    fn test_normal_estimation_params_default() {
        let params = NormalEstimationParams::default();
        assert_eq!(params.k_neighbors, 20);
        assert!(params.orient_outward);
        assert!(!params.consistent_orientation);
    }

    #[test]
    fn test_normal_estimation_params_builder() {
        let params = NormalEstimationParams::new()
            .with_k_neighbors(15)
            .with_orient_outward(false)
            .with_consistent_orientation(true);

        assert_eq!(params.k_neighbors, 15);
        assert!(!params.orient_outward);
        assert!(params.consistent_orientation);
    }

    #[test]
    fn test_estimate_normals_with_params() {
        let mut cloud = make_planar_cloud(10);
        let params = NormalEstimationParams::new()
            .with_k_neighbors(10)
            .with_orient_outward(true);

        estimate_normals_with_params(&mut cloud, &params).unwrap();
        assert!(cloud.has_normals());
    }

    #[test]
    fn test_estimate_normals_with_consistent_orientation() {
        let mut cloud = make_sphere_cloud(10, 1.0);
        let params = NormalEstimationParams::new()
            .with_k_neighbors(10)
            .with_consistent_orientation(true);

        estimate_normals_with_params(&mut cloud, &params).unwrap();
        assert!(cloud.has_normals());
    }

    #[test]
    fn test_single_point_normal() {
        let points = vec![super::super::CloudPoint::from_coords(0.0, 0.0, 0.0)];
        let kdtree = build_kdtree(&points).unwrap();
        let normal = estimate_point_normal(&Point3::origin(), &kdtree, &points, 10);

        // Should return default normal
        assert_relative_eq!(normal.z, 1.0, epsilon = 1e-10);
    }
}
