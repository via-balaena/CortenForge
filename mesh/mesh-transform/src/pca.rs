//! Principal Component Analysis for mesh vertices.
//!
//! Computes the principal axes of a point cloud, useful for
//! aligning meshes to coordinate axes.

use mesh_types::IndexedMesh;
use nalgebra::{Matrix3, SymmetricEigen, Vector3};

use crate::error::{TransformError, TransformResult};

/// Result of PCA computation on a point cloud.
///
/// Contains the principal axes sorted by variance (largest first).
#[derive(Debug, Clone)]
pub struct PcaResult {
    /// The centroid of the point cloud.
    pub centroid: Vector3<f64>,
    /// Principal axes (eigenvectors), sorted by eigenvalue descending.
    /// `axes[0]` is the direction of maximum variance.
    pub axes: [Vector3<f64>; 3],
    /// Eigenvalues (variances) along each axis, sorted descending.
    pub eigenvalues: [f64; 3],
}

impl PcaResult {
    /// Get the primary axis (direction of maximum variance).
    #[must_use]
    pub const fn primary_axis(&self) -> Vector3<f64> {
        self.axes[0]
    }

    /// Get the secondary axis (direction of second-most variance).
    #[must_use]
    pub const fn secondary_axis(&self) -> Vector3<f64> {
        self.axes[1]
    }

    /// Get the tertiary axis (direction of minimum variance).
    #[must_use]
    pub const fn tertiary_axis(&self) -> Vector3<f64> {
        self.axes[2]
    }

    /// Check if the point cloud is approximately flat (one eigenvalue near zero).
    ///
    /// # Arguments
    ///
    /// * `threshold` - Ratio of smallest to largest eigenvalue below which is flat
    #[must_use]
    pub fn is_flat(&self, threshold: f64) -> bool {
        if self.eigenvalues[0] < f64::EPSILON {
            return true;
        }
        self.eigenvalues[2] / self.eigenvalues[0] < threshold
    }

    /// Check if the point cloud is approximately linear (two eigenvalues near zero).
    ///
    /// # Arguments
    ///
    /// * `threshold` - Ratio threshold for considering eigenvalues as near zero
    #[must_use]
    pub fn is_linear(&self, threshold: f64) -> bool {
        if self.eigenvalues[0] < f64::EPSILON {
            return true;
        }
        self.eigenvalues[1] / self.eigenvalues[0] < threshold
    }
}

/// Compute PCA on the vertices of a mesh.
///
/// Returns the principal axes and eigenvalues, which can be used
/// to align the mesh to coordinate axes.
///
/// # Arguments
///
/// * `mesh` - The mesh to analyze
///
/// # Returns
///
/// `Some(PcaResult)` if successful, `None` if the mesh has fewer than 3 vertices.
///
/// # Example
///
/// ```
/// use mesh_transform::pca_axes;
/// use mesh_types::{IndexedMesh, Vertex};
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(10.0, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let pca = pca_axes(&mesh);
/// assert!(pca.is_some());
///
/// // Primary axis should be roughly along X (the long direction)
/// let result = pca.unwrap();
/// assert!(result.primary_axis().x.abs() > 0.9);
/// ```
#[must_use]
#[allow(clippy::cast_precision_loss)]
// Precision loss: vertex counts beyond 2^52 are unsupported
pub fn pca_axes(mesh: &IndexedMesh) -> Option<PcaResult> {
    if mesh.vertices.len() < 3 {
        return None;
    }

    // Compute centroid
    let mut centroid = Vector3::zeros();
    for v in &mesh.vertices {
        centroid.x += v.position.x;
        centroid.y += v.position.y;
        centroid.z += v.position.z;
    }
    let count = mesh.vertices.len() as f64;
    centroid /= count;

    // Build covariance matrix
    let mut covariance = Matrix3::zeros();
    for v in &mesh.vertices {
        let point = Vector3::new(
            v.position.x - centroid.x,
            v.position.y - centroid.y,
            v.position.z - centroid.z,
        );
        covariance += point * point.transpose();
    }
    covariance /= count;

    // Eigen decomposition
    let eigen = SymmetricEigen::new(covariance);
    let eigenvalues = eigen.eigenvalues;
    let eigenvectors = eigen.eigenvectors;

    // Sort by eigenvalue (descending)
    let mut indices = [0usize, 1, 2];
    indices.sort_by(|&a, &b| {
        eigenvalues[b]
            .partial_cmp(&eigenvalues[a])
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let axes = [
        eigenvectors.column(indices[0]).into_owned(),
        eigenvectors.column(indices[1]).into_owned(),
        eigenvectors.column(indices[2]).into_owned(),
    ];

    let sorted_eigenvalues = [
        eigenvalues[indices[0]],
        eigenvalues[indices[1]],
        eigenvalues[indices[2]],
    ];

    Some(PcaResult {
        centroid,
        axes,
        eigenvalues: sorted_eigenvalues,
    })
}

/// Compute PCA with error handling.
///
/// Returns an error if the mesh doesn't have enough vertices.
///
/// # Errors
///
/// Returns `TransformError::InsufficientPoints` if mesh has fewer than 3 vertices.
pub fn pca_axes_checked(mesh: &IndexedMesh) -> TransformResult<PcaResult> {
    if mesh.vertices.len() < 3 {
        return Err(TransformError::InsufficientPoints {
            required: 3,
            actual: mesh.vertices.len(),
        });
    }
    pca_axes(mesh).ok_or_else(|| TransformError::PcaFailed {
        reason: "eigendecomposition failed".to_string(),
    })
}

/// Compute PCA on a slice of points.
///
/// # Arguments
///
/// * `points` - The points to analyze
///
/// # Returns
///
/// `Some(PcaResult)` if successful, `None` if there are fewer than 3 points.
#[must_use]
#[allow(clippy::cast_precision_loss)]
// Precision loss: point counts beyond 2^52 are unsupported
pub fn pca_from_points(points: &[Vector3<f64>]) -> Option<PcaResult> {
    if points.len() < 3 {
        return None;
    }

    // Compute centroid
    let mut centroid = Vector3::zeros();
    for p in points {
        centroid += p;
    }
    let count = points.len() as f64;
    centroid /= count;

    // Build covariance matrix
    let mut covariance = Matrix3::zeros();
    for p in points {
        let centered = p - centroid;
        covariance += centered * centered.transpose();
    }
    covariance /= count;

    // Eigen decomposition
    let eigen = SymmetricEigen::new(covariance);
    let eigenvalues = eigen.eigenvalues;
    let eigenvectors = eigen.eigenvectors;

    // Sort by eigenvalue (descending)
    let mut indices = [0usize, 1, 2];
    indices.sort_by(|&a, &b| {
        eigenvalues[b]
            .partial_cmp(&eigenvalues[a])
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let axes = [
        eigenvectors.column(indices[0]).into_owned(),
        eigenvectors.column(indices[1]).into_owned(),
        eigenvectors.column(indices[2]).into_owned(),
    ];

    let sorted_eigenvalues = [
        eigenvalues[indices[0]],
        eigenvalues[indices[1]],
        eigenvalues[indices[2]],
    ];

    Some(PcaResult {
        centroid,
        axes,
        eigenvalues: sorted_eigenvalues,
    })
}

/// Create a default `PcaResult` for testing fallback scenarios.
#[cfg(test)]
fn default_pca_result() -> PcaResult {
    PcaResult {
        centroid: Vector3::zeros(),
        axes: [Vector3::x(), Vector3::y(), Vector3::z()],
        eigenvalues: [0.0, 0.0, 0.0],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Vertex;

    fn create_elongated_mesh() -> IndexedMesh {
        // Create a mesh elongated along X axis
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.5));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.5));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([1, 3, 2]);
        mesh
    }

    #[test]
    fn pca_primary_axis_is_longest() {
        let mesh = create_elongated_mesh();
        let result = pca_axes(&mesh);
        assert!(result.is_some());

        let pca = result.unwrap_or_else(default_pca_result);

        // Primary axis should be mostly along X
        assert!(
            pca.primary_axis().x.abs() > 0.9,
            "Expected primary axis along X, got {:?}",
            pca.primary_axis()
        );

        // Eigenvalues should be descending
        assert!(pca.eigenvalues[0] >= pca.eigenvalues[1]);
        assert!(pca.eigenvalues[1] >= pca.eigenvalues[2]);
    }

    #[test]
    fn pca_too_few_vertices() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        // Only 2 vertices

        let result = pca_axes(&mesh);
        assert!(result.is_none());
    }

    #[test]
    fn pca_checked_error() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));

        let result = pca_axes_checked(&mesh);
        assert!(result.is_err());
        assert!(matches!(
            result,
            Err(TransformError::InsufficientPoints { .. })
        ));
    }

    #[test]
    fn pca_flat_detection() {
        // Create a flat mesh (all vertices in XY plane)
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 10.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 10.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        let result = pca_axes(&mesh);
        assert!(result.is_some());

        let pca = result.unwrap_or_else(|| PcaResult {
            centroid: Vector3::zeros(),
            axes: [Vector3::x(), Vector3::y(), Vector3::z()],
            eigenvalues: [1.0, 1.0, 1.0],
        });

        assert!(pca.is_flat(0.01), "Expected flat mesh detection");
    }

    #[test]
    fn pca_centroid() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(2.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 2.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        let result = pca_axes(&mesh);
        assert!(result.is_some());

        let pca = result.unwrap_or_else(default_pca_result);

        // Centroid should be at (1, 2/3, 0)
        assert_relative_eq!(pca.centroid.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(pca.centroid.y, 2.0 / 3.0, epsilon = 1e-10);
        assert_relative_eq!(pca.centroid.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn pca_from_points_works() {
        let points = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(10.0, 1.0, 0.0),
        ];

        let result = pca_from_points(&points);
        assert!(result.is_some());

        let pca = result.unwrap_or_else(default_pca_result);

        // Primary axis should be along X
        assert!(pca.primary_axis().x.abs() > 0.9);
    }

    #[test]
    fn pca_axes_are_orthogonal() {
        let mesh = create_elongated_mesh();
        let result = pca_axes(&mesh);
        assert!(result.is_some());

        let pca = result.unwrap_or_else(default_pca_result);

        // Check orthogonality
        let dot_01 = pca.axes[0].dot(&pca.axes[1]);
        let dot_02 = pca.axes[0].dot(&pca.axes[2]);
        let dot_12 = pca.axes[1].dot(&pca.axes[2]);

        assert_relative_eq!(dot_01, 0.0, epsilon = 1e-10);
        assert_relative_eq!(dot_02, 0.0, epsilon = 1e-10);
        assert_relative_eq!(dot_12, 0.0, epsilon = 1e-10);
    }
}
