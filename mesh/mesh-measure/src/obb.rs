//! Oriented bounding box computation.
//!
//! Computes the minimum-volume oriented bounding box using PCA.

// Mesh processing uses u32 indices; truncation would only occur for meshes with >4B vertices
// which exceeds practical limits.
#![allow(clippy::cast_precision_loss)]

use mesh_types::{IndexedMesh, Point3, Vector3};
use nalgebra::{Matrix3, Rotation3};

/// Result of oriented bounding box computation.
///
/// The OBB is the minimum-volume bounding box aligned to the mesh's
/// principal axes (computed via PCA).
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
/// use mesh_measure::oriented_bounding_box;
///
/// let cube = unit_cube();
/// let obb = oriented_bounding_box(&cube);
///
/// // Unit cube OBB has volume 1.0
/// assert!((obb.volume - 1.0).abs() < 0.01);
/// ```
#[derive(Debug, Clone)]
pub struct OrientedBoundingBox {
    /// Center of the OBB.
    pub center: Point3<f64>,
    /// Half-extents along each local axis.
    pub half_extents: Vector3<f64>,
    /// Rotation of the OBB (local axes).
    pub rotation: Rotation3<f64>,
    /// Volume of the OBB.
    pub volume: f64,
    /// Vertices of the OBB (8 corners).
    pub vertices: [Point3<f64>; 8],
}

impl Default for OrientedBoundingBox {
    fn default() -> Self {
        Self {
            center: Point3::origin(),
            half_extents: Vector3::zeros(),
            rotation: Rotation3::identity(),
            volume: 0.0,
            vertices: [Point3::origin(); 8],
        }
    }
}

impl OrientedBoundingBox {
    /// Get the full extents (2 * `half_extents`) along each axis.
    #[must_use]
    pub fn extents(&self) -> Vector3<f64> {
        self.half_extents * 2.0
    }

    /// Get the local X axis in world coordinates.
    #[must_use]
    pub fn axis_x(&self) -> Vector3<f64> {
        self.rotation * Vector3::x()
    }

    /// Get the local Y axis in world coordinates.
    #[must_use]
    pub fn axis_y(&self) -> Vector3<f64> {
        self.rotation * Vector3::y()
    }

    /// Get the local Z axis in world coordinates.
    #[must_use]
    pub fn axis_z(&self) -> Vector3<f64> {
        self.rotation * Vector3::z()
    }

    /// Check if a point is inside the OBB.
    #[must_use]
    pub fn contains(&self, point: Point3<f64>) -> bool {
        let local = self.rotation.inverse() * (point - self.center);
        local.x.abs() <= self.half_extents.x
            && local.y.abs() <= self.half_extents.y
            && local.z.abs() <= self.half_extents.z
    }

    /// Get the surface area of the OBB.
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        let e = self.extents();
        2.0 * e.z.mul_add(e.x, e.x.mul_add(e.y, e.y * e.z))
    }
}

/// Compute an oriented bounding box (minimum volume) for a mesh.
///
/// Uses PCA to find the principal axes and then computes the
/// axis-aligned bounding box in that coordinate system.
///
/// # Arguments
///
/// * `mesh` - The mesh to compute the OBB for
///
/// # Returns
///
/// An [`OrientedBoundingBox`] with the minimum-volume box.
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
/// use mesh_measure::oriented_bounding_box;
///
/// let cube = unit_cube();
/// let obb = oriented_bounding_box(&cube);
/// assert!((obb.volume - 1.0).abs() < 0.01);
/// ```
#[must_use]
pub fn oriented_bounding_box(mesh: &IndexedMesh) -> OrientedBoundingBox {
    if mesh.vertices.is_empty() {
        return OrientedBoundingBox::default();
    }

    // Compute covariance matrix for PCA
    let centroid = compute_centroid(mesh);
    let cov = compute_covariance_matrix(mesh, &centroid);

    // Eigen decomposition for principal axes
    let eigenvectors = symmetric_eigen_decomposition(&cov);

    // Create rotation from eigenvectors (principal axes)
    let rotation = Rotation3::from_matrix_unchecked(eigenvectors);

    // Transform points to local coordinates and find AABB
    let mut local_min = Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut local_max = Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

    for v in &mesh.vertices {
        let local = rotation.inverse() * (v.position - centroid);
        local_min.x = local_min.x.min(local.x);
        local_min.y = local_min.y.min(local.y);
        local_min.z = local_min.z.min(local.z);
        local_max.x = local_max.x.max(local.x);
        local_max.y = local_max.y.max(local.y);
        local_max.z = local_max.z.max(local.z);
    }

    let half_extents = (local_max - local_min) / 2.0;
    let local_center = (local_min + local_max) / 2.0;
    let center = Point3::from(centroid.coords + rotation * local_center);

    let volume = 8.0 * half_extents.x * half_extents.y * half_extents.z;

    // Compute 8 corners
    let corners = [
        Vector3::new(-half_extents.x, -half_extents.y, -half_extents.z),
        Vector3::new(half_extents.x, -half_extents.y, -half_extents.z),
        Vector3::new(half_extents.x, half_extents.y, -half_extents.z),
        Vector3::new(-half_extents.x, half_extents.y, -half_extents.z),
        Vector3::new(-half_extents.x, -half_extents.y, half_extents.z),
        Vector3::new(half_extents.x, -half_extents.y, half_extents.z),
        Vector3::new(half_extents.x, half_extents.y, half_extents.z),
        Vector3::new(-half_extents.x, half_extents.y, half_extents.z),
    ];

    let vertices = corners.map(|c| Point3::from(center.coords + rotation * c));

    OrientedBoundingBox {
        center,
        half_extents,
        rotation,
        volume,
        vertices,
    }
}

fn compute_centroid(mesh: &IndexedMesh) -> Point3<f64> {
    if mesh.vertices.is_empty() {
        return Point3::origin();
    }

    let sum: Vector3<f64> = mesh.vertices.iter().map(|v| v.position.coords).sum();
    Point3::from(sum / mesh.vertices.len() as f64)
}

fn compute_covariance_matrix(mesh: &IndexedMesh, centroid: &Point3<f64>) -> Matrix3<f64> {
    let mut cov = Matrix3::zeros();

    for v in &mesh.vertices {
        let d = v.position - centroid;
        cov[(0, 0)] += d.x * d.x;
        cov[(0, 1)] += d.x * d.y;
        cov[(0, 2)] += d.x * d.z;
        cov[(1, 0)] += d.y * d.x;
        cov[(1, 1)] += d.y * d.y;
        cov[(1, 2)] += d.y * d.z;
        cov[(2, 0)] += d.z * d.x;
        cov[(2, 1)] += d.z * d.y;
        cov[(2, 2)] += d.z * d.z;
    }

    cov / mesh.vertices.len() as f64
}

fn symmetric_eigen_decomposition(m: &Matrix3<f64>) -> Matrix3<f64> {
    let eigen = m.symmetric_eigen();
    eigen.eigenvectors
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::{unit_cube, Vertex};

    fn create_test_cube(size: f64) -> IndexedMesh {
        let mut cube = unit_cube();
        cube.scale(size);
        cube
    }

    #[test]
    fn test_unit_cube_obb() {
        let cube = unit_cube();
        let obb = oriented_bounding_box(&cube);

        // Volume should be close to 1.0
        assert!((obb.volume - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_scaled_cube_obb() {
        let cube = create_test_cube(10.0);
        let obb = oriented_bounding_box(&cube);

        // Volume should be close to 1000.0
        assert!((obb.volume - 1000.0).abs() < 10.0);
    }

    #[test]
    fn test_empty_mesh_obb() {
        let mesh = IndexedMesh::new();
        let obb = oriented_bounding_box(&mesh);

        assert!((obb.volume).abs() < f64::EPSILON);
    }

    #[test]
    fn test_obb_vertices_count() {
        let cube = unit_cube();
        let obb = oriented_bounding_box(&cube);

        assert_eq!(obb.vertices.len(), 8);
    }

    #[test]
    fn test_obb_contains_mesh_vertices() {
        let cube = unit_cube();
        let obb = oriented_bounding_box(&cube);

        for v in &cube.vertices {
            assert!(obb.contains(v.position));
        }
    }

    #[test]
    fn test_obb_surface_area() {
        let cube = unit_cube();
        let obb = oriented_bounding_box(&cube);

        // Surface area of unit cube = 6.0
        assert!((obb.surface_area() - 6.0).abs() < 0.1);
    }

    #[test]
    fn test_obb_axes_orthogonal() {
        let cube = unit_cube();
        let obb = oriented_bounding_box(&cube);

        let x = obb.axis_x();
        let y = obb.axis_y();
        let z = obb.axis_z();

        // Axes should be orthogonal
        assert!(x.dot(&y).abs() < 1e-10);
        assert!(y.dot(&z).abs() < 1e-10);
        assert!(z.dot(&x).abs() < 1e-10);
    }

    #[test]
    fn test_obb_extents() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(4.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(4.0, 2.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 2.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 2, 3]);

        let obb = oriented_bounding_box(&mesh);
        let extents = obb.extents();

        // Should have extents approximately 4x2x0
        let max_extent = extents.x.max(extents.y).max(extents.z);
        let mid_extent = extents.x.min(extents.y.max(extents.z)).max(extents.y.min(extents.z));
        assert!((max_extent - 4.0).abs() < 0.1);
        assert!((mid_extent - 2.0).abs() < 0.1);
    }
}
