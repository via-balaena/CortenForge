//! Statistical outlier removal for point clouds and meshes.
//!
//! This module provides functions for removing outlier points that are
//! statistically far from their neighbors, which often indicates noise
//! or erroneous scan data.
//!
//! # Algorithm
//!
//! For each point:
//! 1. Find the k nearest neighbors
//! 2. Compute the mean distance to those neighbors
//! 3. Compute the global mean and standard deviation of mean distances
//! 4. Remove points where mean distance > `global_mean + std_multiplier * std_dev`
//!
//! # Example
//!
//! ```
//! use mesh_scan::cleanup::outlier::{remove_outliers, OutlierParams};
//! use mesh_scan::pointcloud::PointCloud;
//! use nalgebra::Point3;
//!
//! // Create points with small variations in all dimensions to avoid KD-tree axis collision
//! let mut positions: Vec<_> = (0..100)
//!     .map(|i| Point3::new(i as f64 * 0.1, i as f64 * 0.001, i as f64 * 0.002))
//!     .collect();
//!
//! // Add an outlier
//! positions.push(Point3::new(5.0, 100.0, 0.0));
//!
//! let cloud = PointCloud::from_positions(&positions);
//! let params = OutlierParams::default();
//! let filtered = remove_outliers(&cloud, &params);
//!
//! // Outlier should be removed
//! assert!(filtered.len() < cloud.len());
//! ```

use kiddo::{KdTree, SquaredEuclidean};

use crate::error::{ScanError, ScanResult};
use crate::pointcloud::{CloudPoint, PointCloud};
use mesh_types::IndexedMesh;

/// Parameters for statistical outlier removal.
#[derive(Debug, Clone)]
pub struct OutlierParams {
    /// Number of neighbors to consider. Default: 20.
    pub k_neighbors: usize,

    /// Standard deviation multiplier for outlier threshold. Default: 2.0.
    /// Points with mean distance > mean + `std_multiplier` * std are removed.
    pub std_multiplier: f64,
}

impl Default for OutlierParams {
    fn default() -> Self {
        Self {
            k_neighbors: 20,
            std_multiplier: 2.0,
        }
    }
}

impl OutlierParams {
    /// Creates new parameters with defaults.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the number of neighbors to consider.
    #[must_use]
    pub const fn with_k_neighbors(mut self, k: usize) -> Self {
        self.k_neighbors = k;
        self
    }

    /// Sets the standard deviation multiplier.
    #[must_use]
    pub const fn with_std_multiplier(mut self, multiplier: f64) -> Self {
        self.std_multiplier = multiplier;
        self
    }

    /// Creates parameters for aggressive outlier removal.
    ///
    /// Uses a lower std multiplier (1.0) to remove more points.
    #[must_use]
    pub const fn aggressive() -> Self {
        Self {
            k_neighbors: 30,
            std_multiplier: 1.0,
        }
    }

    /// Creates parameters for conservative outlier removal.
    ///
    /// Uses a higher std multiplier (3.0) to only remove obvious outliers.
    #[must_use]
    pub const fn conservative() -> Self {
        Self {
            k_neighbors: 10,
            std_multiplier: 3.0,
        }
    }
}

/// Result of outlier removal operation.
#[derive(Debug, Clone)]
pub struct OutlierRemovalResult {
    /// The filtered point cloud with outliers removed.
    pub cloud: PointCloud,

    /// Number of points in the original cloud.
    pub original_count: usize,

    /// Number of outliers removed.
    pub outliers_removed: usize,

    /// Mean distance threshold used.
    pub distance_threshold: f64,
}

impl OutlierRemovalResult {
    /// Returns the percentage of points that were outliers.
    #[must_use]
    pub fn outlier_percentage(&self) -> f64 {
        if self.original_count == 0 {
            return 0.0;
        }
        #[allow(clippy::cast_precision_loss)]
        {
            100.0 * self.outliers_removed as f64 / self.original_count as f64
        }
    }
}

impl std::fmt::Display for OutlierRemovalResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Outlier removal: {} → {} points ({} removed, {:.1}%)",
            self.original_count,
            self.cloud.len(),
            self.outliers_removed,
            self.outlier_percentage()
        )
    }
}

/// Removes statistical outliers from a point cloud.
///
/// # Arguments
///
/// * `cloud` - The input point cloud
/// * `params` - Parameters controlling the outlier detection
///
/// # Returns
///
/// A new point cloud with outliers removed.
///
/// # Example
///
/// ```
/// use mesh_scan::cleanup::outlier::{remove_outliers, OutlierParams};
/// use mesh_scan::pointcloud::PointCloud;
/// use nalgebra::Point3;
///
/// // Create points with variations in all dimensions
/// let positions: Vec<_> = (0..100)
///     .map(|i| Point3::new(i as f64 * 0.1, i as f64 * 0.001, i as f64 * 0.002))
///     .collect();
/// let cloud = PointCloud::from_positions(&positions);
///
/// let filtered = remove_outliers(&cloud, &OutlierParams::default());
/// ```
#[must_use]
pub fn remove_outliers(cloud: &PointCloud, params: &OutlierParams) -> PointCloud {
    if cloud.points.len() <= params.k_neighbors {
        return cloud.clone();
    }

    let (keep_mask, _threshold) = compute_outlier_mask(cloud, params);

    let points: Vec<CloudPoint> = cloud
        .points
        .iter()
        .zip(keep_mask.iter())
        .filter_map(|(p, &keep)| if keep { Some(p.clone()) } else { None })
        .collect();

    PointCloud { points }
}

/// Removes statistical outliers and returns detailed results.
///
/// # Arguments
///
/// * `cloud` - The input point cloud
/// * `params` - Parameters controlling the outlier detection
///
/// # Returns
///
/// Detailed results including the filtered cloud and statistics.
///
/// # Example
///
/// ```
/// use mesh_scan::cleanup::outlier::{remove_outliers_with_result, OutlierParams};
/// use mesh_scan::pointcloud::PointCloud;
/// use nalgebra::Point3;
///
/// // Create points with variations in all dimensions
/// let positions: Vec<_> = (0..100)
///     .map(|i| Point3::new(i as f64 * 0.1, i as f64 * 0.001, i as f64 * 0.002))
///     .collect();
/// let cloud = PointCloud::from_positions(&positions);
///
/// let result = remove_outliers_with_result(&cloud, &OutlierParams::default());
/// println!("{}", result);
/// ```
#[must_use]
pub fn remove_outliers_with_result(
    cloud: &PointCloud,
    params: &OutlierParams,
) -> OutlierRemovalResult {
    let original_count = cloud.points.len();

    if original_count <= params.k_neighbors {
        return OutlierRemovalResult {
            cloud: cloud.clone(),
            original_count,
            outliers_removed: 0,
            distance_threshold: 0.0,
        };
    }

    let (keep_mask, threshold) = compute_outlier_mask(cloud, params);

    let points: Vec<CloudPoint> = cloud
        .points
        .iter()
        .zip(keep_mask.iter())
        .filter_map(|(p, &keep)| if keep { Some(p.clone()) } else { None })
        .collect();

    let outliers_removed = original_count - points.len();

    OutlierRemovalResult {
        cloud: PointCloud { points },
        original_count,
        outliers_removed,
        distance_threshold: threshold,
    }
}

/// Computes a mask indicating which points to keep.
fn compute_outlier_mask(cloud: &PointCloud, params: &OutlierParams) -> (Vec<bool>, f64) {
    // Build KD-tree
    let mut kdtree: KdTree<f64, 3> = KdTree::new();
    for (i, point) in cloud.points.iter().enumerate() {
        let coords = [point.position.x, point.position.y, point.position.z];
        #[allow(clippy::cast_possible_truncation)]
        let idx = i as u64;
        kdtree.add(&coords, idx);
    }

    // Compute mean distance to k neighbors for each point (parallel)
    let mean_distances: Vec<f64> = cloud
        .points
        .iter()
        .map(|point| {
            let query = [point.position.x, point.position.y, point.position.z];
            let neighbors = kdtree.nearest_n::<SquaredEuclidean>(&query, params.k_neighbors + 1);

            // Skip the first neighbor (self) and compute mean distance
            let sum: f64 = neighbors.iter().skip(1).map(|n| n.distance.sqrt()).sum();

            #[allow(clippy::cast_precision_loss)]
            let mean = if neighbors.len() > 1 {
                sum / (neighbors.len() - 1) as f64
            } else {
                0.0
            };

            mean
        })
        .collect();

    // Compute global mean and std dev
    #[allow(clippy::cast_precision_loss)]
    let global_mean = mean_distances.iter().sum::<f64>() / mean_distances.len() as f64;

    #[allow(clippy::cast_precision_loss)]
    let variance = mean_distances
        .iter()
        .map(|d| (d - global_mean).powi(2))
        .sum::<f64>()
        / mean_distances.len() as f64;

    let std_dev = variance.sqrt();
    let threshold = params.std_multiplier.mul_add(std_dev, global_mean);

    // Create keep mask
    let keep_mask: Vec<bool> = mean_distances.iter().map(|&d| d <= threshold).collect();

    (keep_mask, threshold)
}

/// Removes outlier vertices from a mesh.
///
/// This treats mesh vertices as a point cloud and removes vertices
/// whose average distance to their k nearest neighbors exceeds
/// the statistical threshold. Faces referencing removed vertices
/// are also removed.
///
/// # Arguments
///
/// * `mesh` - The input mesh
/// * `params` - Parameters controlling outlier detection
///
/// # Returns
///
/// A new mesh with outlier vertices and their faces removed.
///
/// # Errors
///
/// Returns an error if the mesh is empty.
///
/// # Example
///
/// ```
/// use mesh_scan::cleanup::outlier::{remove_mesh_outliers, OutlierParams};
/// use mesh_types::{IndexedMesh, Vertex};
///
/// let mut mesh = IndexedMesh::new();
/// for i in 0..10 {
///     mesh.vertices.push(Vertex::from_coords(i as f64, 0.0, 0.0));
/// }
/// // Add outlier
/// mesh.vertices.push(Vertex::from_coords(5.0, 100.0, 0.0));
///
/// let result = remove_mesh_outliers(&mesh, &OutlierParams::default()).unwrap();
/// ```
pub fn remove_mesh_outliers(mesh: &IndexedMesh, params: &OutlierParams) -> ScanResult<IndexedMesh> {
    if mesh.vertices.is_empty() {
        return Err(ScanError::EmptyMesh);
    }

    if mesh.vertices.len() <= params.k_neighbors {
        return Ok(mesh.clone());
    }

    // Convert mesh vertices to point cloud
    let cloud = PointCloud::from_mesh(mesh);

    // Compute outlier mask
    let (keep_mask, _threshold) = compute_outlier_mask(&cloud, params);

    // Build vertex index mapping (old -> new)
    let mut new_index: Vec<Option<u32>> = vec![None; mesh.vertices.len()];
    let mut new_vertices = Vec::new();

    for (old_idx, (&keep, vertex)) in keep_mask.iter().zip(mesh.vertices.iter()).enumerate() {
        if keep {
            #[allow(clippy::cast_possible_truncation)]
            {
                new_index[old_idx] = Some(new_vertices.len() as u32);
            }
            new_vertices.push(vertex.clone());
        }
    }

    // Remap faces, dropping any that reference removed vertices
    let new_faces: Vec<[u32; 3]> = mesh
        .faces
        .iter()
        .filter_map(|face| {
            let i0 = new_index[face[0] as usize]?;
            let i1 = new_index[face[1] as usize]?;
            let i2 = new_index[face[2] as usize]?;
            Some([i0, i1, i2])
        })
        .collect();

    Ok(IndexedMesh {
        vertices: new_vertices,
        faces: new_faces,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Point3;

    fn make_line_cloud(n: usize) -> PointCloud {
        // Add small variations to avoid kiddo KD-tree axis collision
        let positions: Vec<_> = (0..n)
            .map(|i| {
                let x = f64::from(i as u32) * 0.1;
                let y = (i as f64) * 0.001; // Small variation
                let z = (i as f64) * 0.002; // Small variation
                Point3::new(x, y, z)
            })
            .collect();
        PointCloud::from_positions(&positions)
    }

    fn make_cloud_with_outlier(n: usize) -> PointCloud {
        // Add small variations to avoid kiddo KD-tree axis collision
        let mut positions: Vec<_> = (0..n)
            .map(|i| {
                let x = f64::from(i as u32) * 0.1;
                let y = (i as f64) * 0.001; // Small variation
                let z = (i as f64) * 0.002; // Small variation
                Point3::new(x, y, z)
            })
            .collect();
        // Add outlier far from the line
        positions.push(Point3::new(5.0, 100.0, 0.0));
        PointCloud::from_positions(&positions)
    }

    #[test]
    fn test_outlier_params_default() {
        let params = OutlierParams::default();
        assert_eq!(params.k_neighbors, 20);
        assert_relative_eq!(params.std_multiplier, 2.0);
    }

    #[test]
    fn test_outlier_params_builder() {
        let params = OutlierParams::new()
            .with_k_neighbors(30)
            .with_std_multiplier(1.5);

        assert_eq!(params.k_neighbors, 30);
        assert_relative_eq!(params.std_multiplier, 1.5);
    }

    #[test]
    fn test_outlier_params_presets() {
        let aggressive = OutlierParams::aggressive();
        assert_eq!(aggressive.k_neighbors, 30);
        assert_relative_eq!(aggressive.std_multiplier, 1.0);

        let conservative = OutlierParams::conservative();
        assert_eq!(conservative.k_neighbors, 10);
        assert_relative_eq!(conservative.std_multiplier, 3.0);
    }

    #[test]
    fn test_remove_outliers_clean_cloud() {
        let cloud = make_line_cloud(100);
        let params = OutlierParams::default();
        let filtered = remove_outliers(&cloud, &params);

        // Most points should be retained in a clean cloud
        // The statistical method may remove some edge points
        assert!(filtered.len() >= cloud.len() / 2); // At least half retained
    }

    #[test]
    fn test_remove_outliers_with_outlier() {
        let cloud = make_cloud_with_outlier(100);
        let params = OutlierParams::default();
        let filtered = remove_outliers(&cloud, &params);

        // The outlier should be removed
        assert!(filtered.len() < cloud.len());
    }

    #[test]
    fn test_remove_outliers_small_cloud() {
        let cloud = make_line_cloud(5);
        let params = OutlierParams::new().with_k_neighbors(10);
        let filtered = remove_outliers(&cloud, &params);

        // Should return original when cloud is smaller than k
        assert_eq!(filtered.len(), cloud.len());
    }

    #[test]
    fn test_remove_outliers_empty_cloud() {
        let cloud = PointCloud::new();
        let params = OutlierParams::default();
        let filtered = remove_outliers(&cloud, &params);

        assert!(filtered.is_empty());
    }

    #[test]
    fn test_remove_outliers_with_result() {
        let cloud = make_cloud_with_outlier(100);
        let params = OutlierParams::default();
        let result = remove_outliers_with_result(&cloud, &params);

        assert_eq!(result.original_count, cloud.len());
        assert!(result.outliers_removed > 0);
        assert!(result.cloud.len() < cloud.len());
        assert!(result.distance_threshold > 0.0);
    }

    #[test]
    fn test_outlier_removal_result_display() {
        let result = OutlierRemovalResult {
            cloud: PointCloud::new(),
            original_count: 100,
            outliers_removed: 5,
            distance_threshold: 0.5,
        };

        let display = format!("{result}");
        assert!(display.contains("100"));
        assert!(display.contains('5'));
    }

    #[test]
    fn test_outlier_percentage() {
        let result = OutlierRemovalResult {
            cloud: PointCloud::new(),
            original_count: 100,
            outliers_removed: 10,
            distance_threshold: 0.5,
        };

        assert_relative_eq!(result.outlier_percentage(), 10.0);
    }

    #[test]
    fn test_outlier_percentage_empty() {
        let result = OutlierRemovalResult {
            cloud: PointCloud::new(),
            original_count: 0,
            outliers_removed: 0,
            distance_threshold: 0.0,
        };

        assert_relative_eq!(result.outlier_percentage(), 0.0);
    }

    #[test]
    fn test_remove_mesh_outliers() {
        use mesh_types::Vertex;

        let mut mesh = IndexedMesh::new();
        // Add small variations to avoid kiddo KD-tree axis collision
        for i in 0..50 {
            mesh.vertices.push(Vertex::from_coords(
                f64::from(i) * 0.1,
                f64::from(i) * 0.001,
                f64::from(i) * 0.002,
            ));
        }
        // Add outlier
        mesh.vertices.push(Vertex::from_coords(5.0, 100.0, 0.0));

        let params = OutlierParams::default();
        let result = remove_mesh_outliers(&mesh, &params).unwrap();

        assert!(result.vertices.len() < mesh.vertices.len());
    }

    #[test]
    fn test_remove_mesh_outliers_empty() {
        let mesh = IndexedMesh::new();
        let params = OutlierParams::default();
        let result = remove_mesh_outliers(&mesh, &params);

        assert!(matches!(result, Err(ScanError::EmptyMesh)));
    }

    #[test]
    fn test_remove_mesh_outliers_small() {
        use mesh_types::Vertex;

        let mut mesh = IndexedMesh::new();
        for i in 0..5 {
            mesh.vertices
                .push(Vertex::from_coords(f64::from(i), 0.0, 0.0));
        }

        let params = OutlierParams::new().with_k_neighbors(10);
        let result = remove_mesh_outliers(&mesh, &params).unwrap();

        // Should return original when too small
        assert_eq!(result.vertices.len(), mesh.vertices.len());
    }

    #[test]
    fn test_remove_mesh_outliers_with_faces() {
        use mesh_types::Vertex;

        let mut mesh = IndexedMesh::new();
        // Create a simple triangle grid with small z variation to avoid kiddo axis collision
        for i in 0..10 {
            for j in 0..10 {
                mesh.vertices.push(Vertex::from_coords(
                    f64::from(i),
                    f64::from(j),
                    f64::from(i * 10 + j) * 0.001,
                ));
            }
        }

        // Add some faces
        for i in 0..9_u32 {
            for j in 0..9_u32 {
                let idx = i * 10 + j;
                mesh.faces.push([idx, idx + 1, idx + 10]);
                mesh.faces.push([idx + 1, idx + 11, idx + 10]);
            }
        }

        // Add outlier
        mesh.vertices.push(Vertex::from_coords(5.0, 5.0, 100.0));

        let params = OutlierParams::default();
        let result = remove_mesh_outliers(&mesh, &params).unwrap();

        // Outlier vertex should be removed
        assert!(result.vertices.len() < mesh.vertices.len());
        // Faces should still be valid
        for face in &result.faces {
            assert!((face[0] as usize) < result.vertices.len());
            assert!((face[1] as usize) < result.vertices.len());
            assert!((face[2] as usize) < result.vertices.len());
        }
    }
}
