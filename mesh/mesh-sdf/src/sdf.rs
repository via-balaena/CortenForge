//! Signed distance field computation.
//!
//! Computes the signed distance from any point to the nearest surface of a mesh.

use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::error::{SdfError, SdfResult};
use crate::query::{closest_point_on_triangle, point_in_mesh};

/// A signed distance field for a mesh.
///
/// Provides efficient distance queries after precomputation.
#[derive(Debug, Clone)]
pub struct SignedDistanceField {
    /// The mesh for which this SDF was computed.
    mesh: IndexedMesh,
    /// Cached face normals.
    face_normals: Vec<Vector3<f64>>,
}

impl SignedDistanceField {
    /// Create a new signed distance field from a mesh.
    ///
    /// # Arguments
    ///
    /// * `mesh` - The mesh to compute the SDF for
    ///
    /// # Errors
    ///
    /// Returns an error if the mesh is empty.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{IndexedMesh, Vertex};
    /// use mesh_sdf::SignedDistanceField;
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    /// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
    /// mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
    /// mesh.faces.push([0, 1, 2]);
    ///
    /// let sdf = SignedDistanceField::new(mesh);
    /// assert!(sdf.is_ok());
    /// ```
    pub fn new(mesh: IndexedMesh) -> SdfResult<Self> {
        if mesh.faces.is_empty() {
            return Err(SdfError::EmptyMesh);
        }

        let face_normals = compute_face_normals(&mesh);

        Ok(Self { mesh, face_normals })
    }

    /// Query the signed distance at a point.
    ///
    /// Returns the distance to the nearest surface. Positive values indicate
    /// the point is outside the mesh, negative values indicate inside.
    ///
    /// # Arguments
    ///
    /// * `point` - The point to query
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{IndexedMesh, Vertex};
    /// use mesh_sdf::SignedDistanceField;
    /// use nalgebra::Point3;
    ///
    /// // Create a simple triangle
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    /// mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
    /// mesh.vertices.push(Vertex::from_coords(5.0, 10.0, 0.0));
    /// mesh.faces.push([0, 1, 2]);
    ///
    /// let sdf = SignedDistanceField::new(mesh).unwrap();
    ///
    /// // Point above the triangle
    /// let dist = sdf.distance(Point3::new(5.0, 5.0, 5.0));
    /// assert!(dist > 0.0); // Positive = outside
    /// ```
    #[must_use]
    pub fn distance(&self, point: Point3<f64>) -> f64 {
        let (unsigned_dist, closest_face) = self.unsigned_distance_with_face(point);

        // Determine sign based on whether point is inside or outside
        let sign = self.compute_sign(point, closest_face);

        sign * unsigned_dist
    }

    /// Query the unsigned distance at a point.
    ///
    /// Returns the absolute distance to the nearest surface.
    #[must_use]
    pub fn unsigned_distance(&self, point: Point3<f64>) -> f64 {
        self.unsigned_distance_with_face(point).0
    }

    /// Query the closest point on the mesh surface.
    ///
    /// # Arguments
    ///
    /// * `point` - The query point
    ///
    /// # Returns
    ///
    /// The closest point on the mesh surface.
    #[must_use]
    pub fn closest_point(&self, point: Point3<f64>) -> Point3<f64> {
        let mut min_dist_sq = f64::MAX;
        let mut closest = point;

        for face in &self.mesh.faces {
            let v0 = &self.mesh.vertices[face[0] as usize].position;
            let v1 = &self.mesh.vertices[face[1] as usize].position;
            let v2 = &self.mesh.vertices[face[2] as usize].position;

            let candidate = closest_point_on_triangle(point, *v0, *v1, *v2);
            let dist_sq = (candidate - point).norm_squared();

            if dist_sq < min_dist_sq {
                min_dist_sq = dist_sq;
                closest = candidate;
            }
        }

        closest
    }

    /// Check if a point is inside the mesh.
    ///
    /// Uses ray casting to determine inside/outside status.
    #[must_use]
    pub fn is_inside(&self, point: Point3<f64>) -> bool {
        point_in_mesh(point, &self.mesh)
    }

    /// Get a reference to the underlying mesh.
    #[must_use]
    pub fn mesh(&self) -> &IndexedMesh {
        &self.mesh
    }

    /// Compute unsigned distance and return the closest face index.
    fn unsigned_distance_with_face(&self, point: Point3<f64>) -> (f64, usize) {
        let mut min_dist_sq = f64::MAX;
        let mut closest_face = 0;

        for (face_idx, face) in self.mesh.faces.iter().enumerate() {
            let v0 = &self.mesh.vertices[face[0] as usize].position;
            let v1 = &self.mesh.vertices[face[1] as usize].position;
            let v2 = &self.mesh.vertices[face[2] as usize].position;

            let closest = closest_point_on_triangle(point, *v0, *v1, *v2);
            let dist_sq = (closest - point).norm_squared();

            if dist_sq < min_dist_sq {
                min_dist_sq = dist_sq;
                closest_face = face_idx;
            }
        }

        (min_dist_sq.sqrt(), closest_face)
    }

    /// Compute the sign (+1 outside, -1 inside) for a point.
    fn compute_sign(&self, point: Point3<f64>, closest_face: usize) -> f64 {
        // Use face normal to determine sign
        let face = &self.mesh.faces[closest_face];
        let v0 = &self.mesh.vertices[face[0] as usize].position;
        let normal = &self.face_normals[closest_face];

        let to_point = point - v0;
        if to_point.dot(normal) >= 0.0 {
            1.0
        } else {
            -1.0
        }
    }
}

/// Compute face normals for all faces in a mesh.
fn compute_face_normals(mesh: &IndexedMesh) -> Vec<Vector3<f64>> {
    mesh.faces
        .iter()
        .map(|face| {
            let v0 = &mesh.vertices[face[0] as usize].position;
            let v1 = &mesh.vertices[face[1] as usize].position;
            let v2 = &mesh.vertices[face[2] as usize].position;

            let e1 = *v1 - *v0;
            let e2 = *v2 - *v0;
            let normal = e1.cross(&e2);

            normal.try_normalize(f64::EPSILON).unwrap_or(Vector3::z())
        })
        .collect()
}

/// Compute the signed distance from a point to a mesh without precomputation.
///
/// For one-off queries, this is simpler than creating a `SignedDistanceField`.
/// For multiple queries, prefer creating a `SignedDistanceField` once and reusing it.
///
/// # Arguments
///
/// * `point` - The point to query
/// * `mesh` - The mesh
///
/// # Returns
///
/// The signed distance (positive = outside, negative = inside).
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_sdf::signed_distance;
/// use nalgebra::Point3;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(5.0, 10.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let dist = signed_distance(Point3::new(5.0, 5.0, 5.0), &mesh);
/// ```
#[must_use]
pub fn signed_distance(point: Point3<f64>, mesh: &IndexedMesh) -> f64 {
    if mesh.faces.is_empty() {
        return f64::MAX;
    }

    let normals = compute_face_normals(mesh);
    let mut min_dist_sq = f64::MAX;
    let mut closest_face = 0;

    for (face_idx, face) in mesh.faces.iter().enumerate() {
        let v0 = &mesh.vertices[face[0] as usize].position;
        let v1 = &mesh.vertices[face[1] as usize].position;
        let v2 = &mesh.vertices[face[2] as usize].position;

        let closest = closest_point_on_triangle(point, *v0, *v1, *v2);
        let dist_sq = (closest - point).norm_squared();

        if dist_sq < min_dist_sq {
            min_dist_sq = dist_sq;
            closest_face = face_idx;
        }
    }

    let unsigned_dist = min_dist_sq.sqrt();

    // Determine sign
    let face = &mesh.faces[closest_face];
    let v0 = &mesh.vertices[face[0] as usize].position;
    let normal = &normals[closest_face];
    let to_point = point - v0;

    if to_point.dot(normal) >= 0.0 {
        unsigned_dist
    } else {
        -unsigned_dist
    }
}

/// Compute the unsigned distance from a point to a mesh.
///
/// # Arguments
///
/// * `point` - The point to query
/// * `mesh` - The mesh
///
/// # Returns
///
/// The unsigned (absolute) distance to the nearest surface.
#[must_use]
pub fn unsigned_distance(point: Point3<f64>, mesh: &IndexedMesh) -> f64 {
    if mesh.faces.is_empty() {
        return f64::MAX;
    }

    let mut min_dist_sq = f64::MAX;

    for face in &mesh.faces {
        let v0 = &mesh.vertices[face[0] as usize].position;
        let v1 = &mesh.vertices[face[1] as usize].position;
        let v2 = &mesh.vertices[face[2] as usize].position;

        let closest = closest_point_on_triangle(point, *v0, *v1, *v2);
        let dist_sq = (closest - point).norm_squared();

        if dist_sq < min_dist_sq {
            min_dist_sq = dist_sq;
        }
    }

    min_dist_sq.sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Vertex;

    fn simple_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(5.0, 10.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    fn unit_tetrahedron() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.866, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.289, 0.816));

        // CCW winding when viewed from outside
        mesh.faces.push([0, 2, 1]); // bottom
        mesh.faces.push([0, 1, 3]); // front
        mesh.faces.push([1, 2, 3]); // right
        mesh.faces.push([2, 0, 3]); // left
        mesh
    }

    #[test]
    fn sdf_new_empty_mesh() {
        let mesh = IndexedMesh::new();
        let result = SignedDistanceField::new(mesh);
        assert!(result.is_err());
    }

    #[test]
    fn sdf_new_valid_mesh() {
        let mesh = simple_triangle();
        let result = SignedDistanceField::new(mesh);
        assert!(result.is_ok());
    }

    #[test]
    fn sdf_distance_above_triangle() {
        let mesh = simple_triangle();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Point directly above center of triangle
        let dist = sdf.distance(Point3::new(5.0, 3.33, 5.0));
        assert_relative_eq!(dist.abs(), 5.0, epsilon = 0.1);
    }

    #[test]
    fn sdf_distance_on_surface() {
        let mesh = simple_triangle();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Point on the triangle
        let dist = sdf.distance(Point3::new(5.0, 3.0, 0.0));
        assert_relative_eq!(dist.abs(), 0.0, epsilon = 0.01);
    }

    #[test]
    fn sdf_closest_point() {
        let mesh = simple_triangle();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Point above center
        let query = Point3::new(5.0, 3.0, 5.0);
        let closest = sdf.closest_point(query);

        // Closest point should be on the plane z=0
        assert_relative_eq!(closest.z, 0.0, epsilon = 0.01);
    }

    #[test]
    fn signed_distance_standalone() {
        let mesh = simple_triangle();
        let point = Point3::new(5.0, 3.0, 5.0);

        let dist = signed_distance(point, &mesh);
        assert!(dist > 0.0); // Above the triangle = outside
    }

    #[test]
    fn unsigned_distance_standalone() {
        let mesh = simple_triangle();
        let point = Point3::new(5.0, 3.0, 5.0);

        let dist = unsigned_distance(point, &mesh);
        assert!(dist >= 0.0);
        assert_relative_eq!(dist, 5.0, epsilon = 0.1);
    }

    #[test]
    fn sdf_inside_tetrahedron() {
        let mesh = unit_tetrahedron();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Centroid of tetrahedron is inside
        let centroid = Point3::new(0.5, 0.385, 0.204);
        assert!(sdf.is_inside(centroid) || sdf.distance(centroid) < 0.1);
    }

    #[test]
    fn sdf_outside_tetrahedron() {
        let mesh = unit_tetrahedron();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Point far outside
        let outside = Point3::new(10.0, 10.0, 10.0);
        assert!(!sdf.is_inside(outside));
    }

    #[test]
    fn sdf_mesh_accessor() {
        let mesh = simple_triangle();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        assert_eq!(sdf.mesh().faces.len(), 1);
    }

    #[test]
    fn empty_mesh_distance() {
        let mesh = IndexedMesh::new();
        let dist = unsigned_distance(Point3::new(0.0, 0.0, 0.0), &mesh);
        assert_eq!(dist, f64::MAX);
    }
}
