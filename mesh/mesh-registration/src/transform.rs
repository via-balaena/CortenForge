//! Rigid transformation type for registration results.

use nalgebra::{Matrix4, Point3, UnitQuaternion, Vector3};

/// A rigid transformation consisting of rotation, translation, and optional uniform scale.
///
/// The transformation is applied in the order: scale -> rotate -> translate.
///
/// # Example
///
/// ```
/// use mesh_registration::RigidTransform;
/// use nalgebra::{Point3, UnitQuaternion, Vector3};
/// use std::f64::consts::PI;
///
/// // Create a transform that rotates 90 degrees around Z and translates
/// let rotation = UnitQuaternion::from_axis_angle(
///     &Vector3::z_axis(),
///     PI / 2.0,
/// );
/// let translation = Vector3::new(1.0, 2.0, 3.0);
/// let transform = RigidTransform::new(rotation, translation);
///
/// // Apply to a point
/// let point = Point3::new(1.0, 0.0, 0.0);
/// let transformed = transform.transform_point(&point);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidTransform {
    /// Rotation as a unit quaternion.
    pub rotation: UnitQuaternion<f64>,
    /// Translation vector.
    pub translation: Vector3<f64>,
    /// Uniform scale factor (default 1.0).
    pub scale: f64,
}

impl Default for RigidTransform {
    fn default() -> Self {
        Self::identity()
    }
}

impl RigidTransform {
    /// Creates a new rigid transform with the given rotation and translation.
    ///
    /// Scale is set to 1.0 (no scaling).
    #[must_use]
    pub const fn new(rotation: UnitQuaternion<f64>, translation: Vector3<f64>) -> Self {
        Self {
            rotation,
            translation,
            scale: 1.0,
        }
    }

    /// Creates a new rigid transform with rotation, translation, and scale.
    #[must_use]
    pub const fn with_scale(
        rotation: UnitQuaternion<f64>,
        translation: Vector3<f64>,
        scale: f64,
    ) -> Self {
        Self {
            rotation,
            translation,
            scale,
        }
    }

    /// Creates an identity transform (no rotation, translation, or scaling).
    #[must_use]
    pub fn identity() -> Self {
        Self {
            rotation: UnitQuaternion::identity(),
            translation: Vector3::zeros(),
            scale: 1.0,
        }
    }

    /// Creates a transform with only translation.
    #[must_use]
    pub fn from_translation(translation: Vector3<f64>) -> Self {
        Self {
            rotation: UnitQuaternion::identity(),
            translation,
            scale: 1.0,
        }
    }

    /// Creates a transform with only rotation.
    #[must_use]
    pub fn from_rotation(rotation: UnitQuaternion<f64>) -> Self {
        Self {
            rotation,
            translation: Vector3::zeros(),
            scale: 1.0,
        }
    }

    /// Creates a transform with only uniform scale.
    #[must_use]
    pub fn from_scale(scale: f64) -> Self {
        Self {
            rotation: UnitQuaternion::identity(),
            translation: Vector3::zeros(),
            scale,
        }
    }

    /// Transforms a 3D point.
    ///
    /// The transformation order is: scale -> rotate -> translate.
    #[must_use]
    pub fn transform_point(&self, point: &Point3<f64>) -> Point3<f64> {
        let scaled = point.coords * self.scale;
        let rotated = self.rotation * scaled;
        Point3::from(rotated + self.translation)
    }

    /// Transforms a 3D vector (direction).
    ///
    /// Vectors are scaled and rotated but not translated.
    #[must_use]
    pub fn transform_vector(&self, vector: &Vector3<f64>) -> Vector3<f64> {
        self.rotation * (vector * self.scale)
    }

    /// Composes this transform with another (self * other).
    ///
    /// The result applies `other` first, then `self`.
    #[must_use]
    pub fn compose(&self, other: &Self) -> Self {
        // Combined scale
        let combined_scale = self.scale * other.scale;

        // Combined rotation
        let combined_rotation = self.rotation * other.rotation;

        // Translation: self.translation + self.rotation * (self.scale * other.translation)
        let combined_translation =
            self.translation + self.rotation * (other.translation * self.scale);

        Self {
            rotation: combined_rotation,
            translation: combined_translation,
            scale: combined_scale,
        }
    }

    /// Computes the inverse of this transform.
    #[must_use]
    pub fn inverse(&self) -> Self {
        let inv_scale = 1.0 / self.scale;
        let inv_rotation = self.rotation.inverse();
        let inv_translation = inv_rotation * (-self.translation * inv_scale);

        Self {
            rotation: inv_rotation,
            translation: inv_translation,
            scale: inv_scale,
        }
    }

    /// Converts to a 4x4 homogeneous transformation matrix.
    #[must_use]
    pub fn to_matrix4(&self) -> Matrix4<f64> {
        let mut mat = Matrix4::identity();

        // Set rotation (upper-left 3x3)
        let rot_mat = self.rotation.to_rotation_matrix();
        for i in 0..3 {
            for j in 0..3 {
                mat[(i, j)] = rot_mat[(i, j)] * self.scale;
            }
        }

        // Set translation (last column)
        mat[(0, 3)] = self.translation.x;
        mat[(1, 3)] = self.translation.y;
        mat[(2, 3)] = self.translation.z;

        mat
    }

    /// Returns true if this transform is approximately the identity.
    #[must_use]
    pub fn is_identity(&self, epsilon: f64) -> bool {
        let angle = self.rotation.angle();
        angle.abs() < epsilon
            && self.translation.norm() < epsilon
            && (self.scale - 1.0).abs() < epsilon
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    #[test]
    fn test_identity_transform() {
        let transform = RigidTransform::identity();
        let point = Point3::new(1.0, 2.0, 3.0);
        let result = transform.transform_point(&point);
        assert_relative_eq!(result.coords, point.coords, epsilon = 1e-10);
    }

    #[test]
    fn test_translation_only() {
        let translation = Vector3::new(1.0, 2.0, 3.0);
        let transform = RigidTransform::from_translation(translation);
        let point = Point3::new(0.0, 0.0, 0.0);
        let result = transform.transform_point(&point);
        assert_relative_eq!(result.coords, translation, epsilon = 1e-10);
    }

    #[test]
    fn test_rotation_90_degrees_z() {
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0);
        let transform = RigidTransform::from_rotation(rotation);
        let point = Point3::new(1.0, 0.0, 0.0);
        let result = transform.transform_point(&point);
        assert_relative_eq!(result.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 1.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_scale_only() {
        let transform = RigidTransform::from_scale(2.0);
        let point = Point3::new(1.0, 2.0, 3.0);
        let result = transform.transform_point(&point);
        assert_relative_eq!(result.x, 2.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 4.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 6.0, epsilon = 1e-10);
    }

    #[test]
    fn test_compose_translations() {
        let t1 = RigidTransform::from_translation(Vector3::new(1.0, 0.0, 0.0));
        let t2 = RigidTransform::from_translation(Vector3::new(0.0, 2.0, 0.0));
        let composed = t1.compose(&t2);

        let point = Point3::new(0.0, 0.0, 0.0);
        let result = composed.transform_point(&point);
        assert_relative_eq!(result.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_inverse() {
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 4.0);
        let translation = Vector3::new(1.0, 2.0, 3.0);
        let transform = RigidTransform::with_scale(rotation, translation, 1.5);

        let point = Point3::new(1.0, 2.0, 3.0);
        let transformed = transform.transform_point(&point);
        let inverse = transform.inverse();
        let recovered = inverse.transform_point(&transformed);

        assert_relative_eq!(recovered.coords, point.coords, epsilon = 1e-10);
    }

    #[test]
    fn test_transform_vector() {
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0);
        let translation = Vector3::new(100.0, 100.0, 100.0); // Should be ignored
        let transform = RigidTransform::new(rotation, translation);

        let vector = Vector3::new(1.0, 0.0, 0.0);
        let result = transform.transform_vector(&vector);

        // Rotated 90 degrees around Z: (1,0,0) -> (0,1,0)
        assert_relative_eq!(result.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(result.y, 1.0, epsilon = 1e-10);
        assert_relative_eq!(result.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_to_matrix4() {
        let transform = RigidTransform::from_translation(Vector3::new(1.0, 2.0, 3.0));
        let mat = transform.to_matrix4();

        assert_relative_eq!(mat[(0, 3)], 1.0, epsilon = 1e-10);
        assert_relative_eq!(mat[(1, 3)], 2.0, epsilon = 1e-10);
        assert_relative_eq!(mat[(2, 3)], 3.0, epsilon = 1e-10);
        assert_relative_eq!(mat[(3, 3)], 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_is_identity() {
        let identity = RigidTransform::identity();
        assert!(identity.is_identity(1e-10));

        let translation = RigidTransform::from_translation(Vector3::new(0.001, 0.0, 0.0));
        assert!(!translation.is_identity(1e-10));
        assert!(translation.is_identity(0.01));
    }

    #[test]
    fn test_default_is_identity() {
        let default_transform = RigidTransform::default();
        let identity = RigidTransform::identity();
        assert_eq!(default_transform, identity);
    }
}
