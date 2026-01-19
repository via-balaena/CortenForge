//! 3D transformation matrix operations.

use mesh_types::IndexedMesh;
use nalgebra::{Matrix4, Vector3, Vector4};

/// A 3D transformation represented as a 4x4 matrix.
///
/// Supports common operations like translation, rotation, scaling,
/// and composition of transformations.
///
/// # Example
///
/// ```
/// use mesh_transform::Transform3D;
///
/// let translate = Transform3D::translation(1.0, 2.0, 3.0);
/// let scale = Transform3D::uniform_scale(2.0);
/// let combined = translate.then(&scale);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Transform3D {
    /// The 4x4 transformation matrix in column-major order.
    matrix: Matrix4<f64>,
}

impl Default for Transform3D {
    fn default() -> Self {
        Self::identity()
    }
}

impl Transform3D {
    /// Create a new transformation from a 4x4 matrix.
    #[must_use]
    pub const fn from_matrix(matrix: Matrix4<f64>) -> Self {
        Self { matrix }
    }

    /// Create the identity transformation (no change).
    #[must_use]
    pub fn identity() -> Self {
        Self {
            matrix: Matrix4::identity(),
        }
    }

    /// Create a translation transformation.
    #[must_use]
    pub fn translation(tx: f64, ty: f64, tz: f64) -> Self {
        Self {
            matrix: Matrix4::new_translation(&Vector3::new(tx, ty, tz)),
        }
    }

    /// Create a translation from a vector.
    #[must_use]
    pub fn from_translation(v: Vector3<f64>) -> Self {
        Self::translation(v.x, v.y, v.z)
    }

    /// Create a uniform scaling transformation.
    #[must_use]
    pub fn uniform_scale(factor: f64) -> Self {
        Self::scale(factor, factor, factor)
    }

    /// Create a non-uniform scaling transformation.
    #[must_use]
    pub fn scale(sx: f64, sy: f64, sz: f64) -> Self {
        Self {
            matrix: Matrix4::new_nonuniform_scaling(&Vector3::new(sx, sy, sz)),
        }
    }

    /// Create a rotation around the X axis.
    ///
    /// # Arguments
    ///
    /// * `angle` - Rotation angle in radians
    #[must_use]
    pub fn rotation_x(angle: f64) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        #[rustfmt::skip]
        let matrix = Matrix4::new(
            1.0,   0.0,    0.0, 0.0,
            0.0, cos_a, -sin_a, 0.0,
            0.0, sin_a,  cos_a, 0.0,
            0.0,   0.0,    0.0, 1.0,
        );
        Self { matrix }
    }

    /// Create a rotation around the Y axis.
    ///
    /// # Arguments
    ///
    /// * `angle` - Rotation angle in radians
    #[must_use]
    pub fn rotation_y(angle: f64) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        #[rustfmt::skip]
        let matrix = Matrix4::new(
             cos_a, 0.0, sin_a, 0.0,
               0.0, 1.0,   0.0, 0.0,
            -sin_a, 0.0, cos_a, 0.0,
               0.0, 0.0,   0.0, 1.0,
        );
        Self { matrix }
    }

    /// Create a rotation around the Z axis.
    ///
    /// # Arguments
    ///
    /// * `angle` - Rotation angle in radians
    #[must_use]
    pub fn rotation_z(angle: f64) -> Self {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        #[rustfmt::skip]
        let matrix = Matrix4::new(
            cos_a, -sin_a, 0.0, 0.0,
            sin_a,  cos_a, 0.0, 0.0,
              0.0,    0.0, 1.0, 0.0,
              0.0,    0.0, 0.0, 1.0,
        );
        Self { matrix }
    }

    /// Create a rotation around an arbitrary axis.
    ///
    /// Uses Rodrigues' rotation formula.
    ///
    /// # Arguments
    ///
    /// * `axis` - The axis to rotate around (will be normalized)
    /// * `angle` - Rotation angle in radians
    ///
    /// # Returns
    ///
    /// The rotation transformation, or identity if axis is zero.
    #[must_use]
    #[allow(clippy::many_single_char_names)]
    // Single-char names: standard mathematical notation for rotation formula
    #[allow(clippy::suboptimal_flops)]
    // Suboptimal flops: prefer readable Rodrigues formula over mul_add optimization
    pub fn rotation_axis(axis: Vector3<f64>, angle: f64) -> Self {
        let norm = axis.norm();
        if norm < f64::EPSILON {
            return Self::identity();
        }

        let axis = axis / norm;
        let c = angle.cos();
        let s = angle.sin();
        let t = 1.0 - c;

        let x = axis.x;
        let y = axis.y;
        let z = axis.z;

        #[rustfmt::skip]
        let matrix = Matrix4::new(
            t*x*x + c,     t*x*y - s*z,   t*x*z + s*y,   0.0,
            t*x*y + s*z,   t*y*y + c,     t*y*z - s*x,   0.0,
            t*x*z - s*y,   t*y*z + s*x,   t*z*z + c,     0.0,
            0.0,           0.0,           0.0,           1.0,
        );
        Self { matrix }
    }

    /// Create a transformation that rotates from one direction to another.
    ///
    /// # Arguments
    ///
    /// * `from` - Source direction (will be normalized)
    /// * `to` - Target direction (will be normalized)
    ///
    /// # Returns
    ///
    /// A rotation that maps `from` to `to`.
    #[must_use]
    pub fn rotation_between(from: Vector3<f64>, to: Vector3<f64>) -> Self {
        let from_norm = from.norm();
        let to_norm = to.norm();

        if from_norm < f64::EPSILON || to_norm < f64::EPSILON {
            return Self::identity();
        }

        let from = from / from_norm;
        let to = to / to_norm;

        let dot = from.dot(&to);

        // Vectors are nearly parallel
        if dot > 1.0 - f64::EPSILON {
            return Self::identity();
        }

        // Vectors are nearly anti-parallel
        if dot < -1.0 + f64::EPSILON {
            // Find a perpendicular axis
            let axis = if from.x.abs() < 0.9 {
                Vector3::x().cross(&from).normalize()
            } else {
                Vector3::y().cross(&from).normalize()
            };
            return Self::rotation_axis(axis, std::f64::consts::PI);
        }

        let axis = from.cross(&to);
        let angle = dot.acos();
        Self::rotation_axis(axis, angle)
    }

    /// Get the underlying 4x4 matrix.
    #[must_use]
    pub const fn matrix(&self) -> &Matrix4<f64> {
        &self.matrix
    }

    /// Compose this transformation with another (self then other).
    ///
    /// The result applies `self` first, then `other`.
    #[must_use]
    pub fn then(&self, other: &Self) -> Self {
        Self {
            matrix: other.matrix * self.matrix,
        }
    }

    /// Compute the inverse transformation.
    ///
    /// # Returns
    ///
    /// `Some(inverse)` if the matrix is invertible, `None` otherwise.
    #[must_use]
    pub fn inverse(&self) -> Option<Self> {
        self.matrix.try_inverse().map(|m| Self { matrix: m })
    }

    /// Transform a point (applies translation).
    #[must_use]
    pub fn transform_point(&self, point: Vector3<f64>) -> Vector3<f64> {
        let p = Vector4::new(point.x, point.y, point.z, 1.0);
        let result = self.matrix * p;
        Vector3::new(result.x, result.y, result.z)
    }

    /// Transform a direction vector (ignores translation).
    #[must_use]
    pub fn transform_vector(&self, vector: Vector3<f64>) -> Vector3<f64> {
        let v = Vector4::new(vector.x, vector.y, vector.z, 0.0);
        let result = self.matrix * v;
        Vector3::new(result.x, result.y, result.z)
    }

    /// Transform a normal vector (uses inverse transpose).
    ///
    /// # Returns
    ///
    /// The transformed normal, or the original if the matrix is not invertible.
    #[must_use]
    pub fn transform_normal(&self, normal: Vector3<f64>) -> Vector3<f64> {
        // For normals, we need to use the inverse transpose of the upper 3x3
        let m = self.matrix.fixed_view::<3, 3>(0, 0);
        m.try_inverse()
            .map_or(normal, |inv| inv.transpose() * normal)
    }

    /// Apply this transformation to all vertices of a mesh.
    ///
    /// Creates a new mesh with transformed positions and normals.
    #[must_use]
    pub fn apply_to_mesh(&self, mesh: &IndexedMesh) -> IndexedMesh {
        let mut result = mesh.clone();

        for vertex in &mut result.vertices {
            let pos = Vector3::new(vertex.position.x, vertex.position.y, vertex.position.z);
            let transformed = self.transform_point(pos);
            vertex.position.x = transformed.x;
            vertex.position.y = transformed.y;
            vertex.position.z = transformed.z;

            if let Some(norm) = vertex.attributes.normal {
                let transformed_normal = self.transform_normal(norm);
                if let Some(normalized) = transformed_normal.try_normalize(f64::EPSILON) {
                    vertex.attributes.normal = Some(normalized);
                }
            }
        }

        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    #[test]
    fn identity_transformation() {
        let t = Transform3D::identity();
        let p = Vector3::new(1.0, 2.0, 3.0);
        let result = t.transform_point(p);

        assert_relative_eq!(result.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn translation() {
        let t = Transform3D::translation(10.0, 20.0, 30.0);
        let p = Vector3::new(1.0, 2.0, 3.0);
        let result = t.transform_point(p);

        assert_relative_eq!(result.x, 11.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 22.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 33.0, epsilon = 1e-10);
    }

    #[test]
    fn translation_does_not_affect_vectors() {
        let t = Transform3D::translation(10.0, 20.0, 30.0);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let result = t.transform_vector(v);

        assert_relative_eq!(result.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn uniform_scale() {
        let t = Transform3D::uniform_scale(2.0);
        let p = Vector3::new(1.0, 2.0, 3.0);
        let result = t.transform_point(p);

        assert_relative_eq!(result.x, 2.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 4.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 6.0, epsilon = 1e-10);
    }

    #[test]
    fn rotation_z_90_degrees() {
        let t = Transform3D::rotation_z(PI / 2.0);
        let p = Vector3::new(1.0, 0.0, 0.0);
        let result = t.transform_point(p);

        assert_relative_eq!(result.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 1.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn composition() {
        let translate = Transform3D::translation(1.0, 0.0, 0.0);
        let scale = Transform3D::uniform_scale(2.0);

        // Translate then scale
        let combined = translate.then(&scale);
        let p = Vector3::new(0.0, 0.0, 0.0);
        let result = combined.transform_point(p);

        // (0,0,0) + (1,0,0) = (1,0,0), then * 2 = (2,0,0)
        assert_relative_eq!(result.x, 2.0, epsilon = 1e-10);
    }

    #[test]
    fn inverse() {
        let t = Transform3D::translation(10.0, 20.0, 30.0);
        let inv = t.inverse();
        assert!(inv.is_some());

        let inv = inv.unwrap_or_default();
        let p = Vector3::new(15.0, 25.0, 35.0);
        let result = inv.transform_point(p);

        assert_relative_eq!(result.x, 5.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 5.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 5.0, epsilon = 1e-10);
    }

    #[test]
    fn rotation_between_parallel() {
        let from = Vector3::new(1.0, 0.0, 0.0);
        let to = Vector3::new(2.0, 0.0, 0.0);
        let t = Transform3D::rotation_between(from, to);

        // Should be identity since vectors are parallel
        let result = t.transform_point(Vector3::new(1.0, 2.0, 3.0));
        assert_relative_eq!(result.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn rotation_between_perpendicular() {
        let from = Vector3::new(1.0, 0.0, 0.0);
        let to = Vector3::new(0.0, 1.0, 0.0);
        let t = Transform3D::rotation_between(from, to);

        // X axis should map to Y axis
        let result = t.transform_vector(Vector3::new(1.0, 0.0, 0.0));
        assert_relative_eq!(result.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 1.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn apply_to_mesh() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices
            .push(mesh_types::Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices
            .push(mesh_types::Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.vertices
            .push(mesh_types::Vertex::from_coords(0.0, 0.0, 1.0));
        mesh.faces.push([0, 1, 2]);

        let t = Transform3D::translation(10.0, 20.0, 30.0);
        let transformed = t.apply_to_mesh(&mesh);

        assert_relative_eq!(transformed.vertices[0].position.x, 11.0, epsilon = 1e-10);
        assert_relative_eq!(transformed.vertices[1].position.y, 21.0, epsilon = 1e-10);
        assert_relative_eq!(transformed.vertices[2].position.z, 31.0, epsilon = 1e-10);
    }

    #[test]
    fn default_is_identity() {
        let t = Transform3D::default();
        let p = Vector3::new(5.0, 10.0, 15.0);
        let result = t.transform_point(p);

        assert_relative_eq!(result.x, 5.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 10.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 15.0, epsilon = 1e-10);
    }

    #[test]
    fn from_matrix() {
        let m = Matrix4::identity();
        let t = Transform3D::from_matrix(m);
        let p = Vector3::new(1.0, 2.0, 3.0);
        let result = t.transform_point(p);

        assert_relative_eq!(result.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn from_translation_vector() {
        let t = Transform3D::from_translation(Vector3::new(5.0, 6.0, 7.0));
        let p = Vector3::new(1.0, 2.0, 3.0);
        let result = t.transform_point(p);

        assert_relative_eq!(result.x, 6.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 8.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 10.0, epsilon = 1e-10);
    }

    #[test]
    fn rotation_x_90_degrees() {
        let t = Transform3D::rotation_x(PI / 2.0);
        let p = Vector3::new(0.0, 1.0, 0.0);
        let result = t.transform_point(p);

        // Y axis rotates to Z axis
        assert_relative_eq!(result.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn rotation_y_90_degrees() {
        let t = Transform3D::rotation_y(PI / 2.0);
        let p = Vector3::new(1.0, 0.0, 0.0);
        let result = t.transform_point(p);

        // X axis rotates to -Z axis
        assert_relative_eq!(result.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, -1.0, epsilon = 1e-10);
    }

    #[test]
    fn rotation_axis_zero_returns_identity() {
        let t = Transform3D::rotation_axis(Vector3::zeros(), PI);
        let p = Vector3::new(1.0, 2.0, 3.0);
        let result = t.transform_point(p);

        assert_relative_eq!(result.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn rotation_between_zero_vector() {
        let t = Transform3D::rotation_between(Vector3::zeros(), Vector3::x());
        let p = Vector3::new(1.0, 2.0, 3.0);
        let result = t.transform_point(p);

        // Should return identity
        assert_relative_eq!(result.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn rotation_between_anti_parallel() {
        let t = Transform3D::rotation_between(Vector3::x(), -Vector3::x());
        let p = Vector3::new(1.0, 0.0, 0.0);
        let result = t.transform_point(p);

        // X should map to -X (180 degree rotation)
        assert_relative_eq!(result.x, -1.0, epsilon = 1e-10);
    }

    #[test]
    fn non_uniform_scale() {
        let t = Transform3D::scale(2.0, 3.0, 4.0);
        let p = Vector3::new(1.0, 1.0, 1.0);
        let result = t.transform_point(p);

        assert_relative_eq!(result.x, 2.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 3.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 4.0, epsilon = 1e-10);
    }

    #[test]
    fn matrix_accessor() {
        let t = Transform3D::identity();
        let m = t.matrix();
        assert_relative_eq!(m[(0, 0)], 1.0, epsilon = 1e-10);
        assert_relative_eq!(m[(1, 1)], 1.0, epsilon = 1e-10);
        assert_relative_eq!(m[(0, 1)], 0.0, epsilon = 1e-10);
    }

    #[test]
    fn transform_normal_non_invertible() {
        // Create a degenerate matrix (all zeros except w=1)
        let t = Transform3D::scale(0.0, 0.0, 0.0);
        let n = Vector3::new(0.0, 0.0, 1.0);
        let result = t.transform_normal(n);

        // Should return original since matrix is not invertible
        assert_relative_eq!(result.z, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn apply_to_mesh_with_normals() {
        let mut mesh = IndexedMesh::new();
        let mut v = mesh_types::Vertex::from_coords(1.0, 0.0, 0.0);
        v.attributes.normal = Some(Vector3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(v);
        mesh.faces.push([0, 0, 0]); // Degenerate but valid for testing

        let t = Transform3D::rotation_z(PI / 2.0);
        let transformed = t.apply_to_mesh(&mesh);

        // Position rotates
        assert_relative_eq!(transformed.vertices[0].position.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(transformed.vertices[0].position.y, 1.0, epsilon = 1e-10);

        // Normal should also rotate
        let norm = transformed.vertices[0]
            .attributes
            .normal
            .unwrap_or_else(Vector3::zeros);
        assert_relative_eq!(norm.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(norm.y, 1.0, epsilon = 1e-10);
    }
}
