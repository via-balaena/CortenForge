//! 3D transforms for sensor calibration.

use glam::{Mat4, Quat, Vec3};
use serde::{Deserialize, Serialize};

use crate::error::{FusionError, Result};

/// A 3D rigid body transform (rotation + translation).
///
/// Represents the transform from one coordinate frame to another.
///
/// # Example
///
/// ```
/// use sensor_fusion::Transform3D;
/// use glam::Vec3;
///
/// // Identity transform
/// let t = Transform3D::identity();
/// let point = Vec3::new(1.0, 2.0, 3.0);
/// let result = t.apply_point(point);
/// assert!((result - point).length() < 1e-6);
///
/// // Translation only
/// let t = Transform3D::from_translation(Vec3::new(10.0, 0.0, 0.0));
/// let result = t.apply_point(Vec3::ZERO);
/// assert!((result.x - 10.0).abs() < 1e-6);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Transform3D {
    /// Rotation component (quaternion).
    #[serde(with = "quat_serde")]
    pub rotation: Quat,

    /// Translation component.
    #[serde(with = "vec3_serde")]
    pub translation: Vec3,
}

mod quat_serde {
    use glam::Quat;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    #[derive(Serialize, Deserialize)]
    struct QuatData {
        x: f32,
        y: f32,
        z: f32,
        w: f32,
    }

    pub fn serialize<S: Serializer>(q: &Quat, s: S) -> std::result::Result<S::Ok, S::Error> {
        QuatData {
            x: q.x,
            y: q.y,
            z: q.z,
            w: q.w,
        }
        .serialize(s)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> std::result::Result<Quat, D::Error> {
        let data = QuatData::deserialize(d)?;
        Ok(Quat::from_xyzw(data.x, data.y, data.z, data.w))
    }
}

mod vec3_serde {
    use glam::Vec3;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    #[derive(Serialize, Deserialize)]
    struct Vec3Data {
        x: f32,
        y: f32,
        z: f32,
    }

    pub fn serialize<S: Serializer>(v: &Vec3, s: S) -> std::result::Result<S::Ok, S::Error> {
        Vec3Data {
            x: v.x,
            y: v.y,
            z: v.z,
        }
        .serialize(s)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> std::result::Result<Vec3, D::Error> {
        let data = Vec3Data::deserialize(d)?;
        Ok(Vec3::new(data.x, data.y, data.z))
    }
}

impl Default for Transform3D {
    fn default() -> Self {
        Self::identity()
    }
}

impl Transform3D {
    /// Creates an identity transform (no rotation or translation).
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            rotation: Quat::IDENTITY,
            translation: Vec3::ZERO,
        }
    }

    /// Creates a transform with only translation.
    #[must_use]
    pub const fn from_translation(translation: Vec3) -> Self {
        Self {
            rotation: Quat::IDENTITY,
            translation,
        }
    }

    /// Creates a transform with only rotation.
    #[must_use]
    pub const fn from_rotation(rotation: Quat) -> Self {
        Self {
            rotation,
            translation: Vec3::ZERO,
        }
    }

    /// Creates a transform from rotation and translation.
    #[must_use]
    pub const fn new(rotation: Quat, translation: Vec3) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    /// Creates a transform from Euler angles (in radians).
    ///
    /// Uses YXZ rotation order (yaw, pitch, roll).
    #[must_use]
    pub fn from_euler(yaw: f32, pitch: f32, roll: f32) -> Self {
        Self {
            rotation: Quat::from_euler(glam::EulerRot::YXZ, yaw, pitch, roll),
            translation: Vec3::ZERO,
        }
    }

    /// Creates a transform from a 4x4 matrix.
    ///
    /// Extracts rotation and translation from the matrix.
    #[must_use]
    pub fn from_matrix(mat: Mat4) -> Self {
        let (_, rotation, translation) = mat.to_scale_rotation_translation();
        Self {
            rotation,
            translation,
        }
    }

    /// Converts the transform to a 4x4 matrix.
    #[must_use]
    pub fn to_matrix(&self) -> Mat4 {
        Mat4::from_rotation_translation(self.rotation, self.translation)
    }

    /// Applies the transform to a point.
    #[must_use]
    pub fn apply_point(&self, point: Vec3) -> Vec3 {
        self.rotation * point + self.translation
    }

    /// Applies the transform to a direction vector.
    ///
    /// Only rotation is applied, not translation.
    #[must_use]
    pub fn apply_direction(&self, direction: Vec3) -> Vec3 {
        self.rotation * direction
    }

    /// Returns the inverse transform.
    #[must_use]
    pub fn inverse(&self) -> Self {
        let inv_rotation = self.rotation.inverse();
        Self {
            rotation: inv_rotation,
            translation: inv_rotation * (-self.translation),
        }
    }

    /// Composes this transform with another (self * other).
    ///
    /// The result applies `other` first, then `self`.
    #[must_use]
    pub fn compose(&self, other: &Self) -> Self {
        Self {
            rotation: self.rotation * other.rotation,
            translation: self.rotation * other.translation + self.translation,
        }
    }

    /// Interpolates between two transforms.
    ///
    /// Uses spherical linear interpolation (slerp) for rotation.
    #[must_use]
    pub fn lerp(&self, other: &Self, t: f32) -> Self {
        Self {
            rotation: self.rotation.slerp(other.rotation, t),
            translation: self.translation.lerp(other.translation, t),
        }
    }

    /// Returns true if this is approximately the identity transform.
    #[must_use]
    pub fn is_identity(&self, epsilon: f32) -> bool {
        let rot_diff = (self.rotation - Quat::IDENTITY).length();
        let trans_diff = self.translation.length();
        rot_diff < epsilon && trans_diff < epsilon
    }
}

/// A chain of transforms for sensor calibration.
///
/// Useful for expressing transforms through multiple coordinate frames.
///
/// # Example
///
/// ```
/// use sensor_fusion::{Transform3D, TransformChain};
/// use glam::Vec3;
///
/// let mut chain = TransformChain::new();
/// chain.push("world_to_robot", Transform3D::from_translation(Vec3::new(10.0, 0.0, 0.0)));
/// chain.push("robot_to_sensor", Transform3D::from_translation(Vec3::new(0.0, 1.0, 0.0)));
///
/// // Get composed transform
/// let combined = chain.compose().unwrap();
/// let point = combined.apply_point(Vec3::ZERO);
/// assert!((point.x - 10.0).abs() < 1e-6);
/// assert!((point.y - 1.0).abs() < 1e-6);
/// ```
#[derive(Debug, Clone, Default)]
pub struct TransformChain {
    transforms: Vec<(String, Transform3D)>,
}

impl TransformChain {
    /// Creates an empty transform chain.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns the number of transforms in the chain.
    #[must_use]
    pub fn len(&self) -> usize {
        self.transforms.len()
    }

    /// Returns true if the chain is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.transforms.is_empty()
    }

    /// Adds a named transform to the chain.
    pub fn push(&mut self, name: impl Into<String>, transform: Transform3D) {
        self.transforms.push((name.into(), transform));
    }

    /// Gets a transform by name.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<&Transform3D> {
        self.transforms
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, t)| t)
    }

    /// Gets a mutable transform by name.
    #[must_use]
    pub fn get_mut(&mut self, name: &str) -> Option<&mut Transform3D> {
        self.transforms
            .iter_mut()
            .find(|(n, _)| n == name)
            .map(|(_, t)| t)
    }

    /// Removes a transform by name.
    pub fn remove(&mut self, name: &str) -> Option<Transform3D> {
        if let Some(idx) = self.transforms.iter().position(|(n, _)| n == name) {
            Some(self.transforms.remove(idx).1)
        } else {
            None
        }
    }

    /// Clears all transforms.
    pub fn clear(&mut self) {
        self.transforms.clear();
    }

    /// Composes all transforms in order.
    ///
    /// Returns the combined transform that applies all transforms in sequence.
    ///
    /// # Errors
    ///
    /// Returns an error if the chain is empty.
    pub fn compose(&self) -> Result<Transform3D> {
        if self.transforms.is_empty() {
            return Err(FusionError::insufficient_data("empty transform chain"));
        }

        let mut result = Transform3D::identity();
        for (_, transform) in &self.transforms {
            result = result.compose(transform);
        }
        Ok(result)
    }

    /// Composes transforms up to and including the named transform.
    ///
    /// # Errors
    ///
    /// Returns an error if the named transform is not found.
    pub fn compose_to(&self, name: &str) -> Result<Transform3D> {
        let mut result = Transform3D::identity();

        for (n, transform) in &self.transforms {
            result = result.compose(transform);
            if n == name {
                return Ok(result);
            }
        }

        Err(FusionError::transform(format!(
            "transform not found: {name}"
        )))
    }

    /// Returns an iterator over the transforms.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &Transform3D)> {
        self.transforms.iter().map(|(n, t)| (n.as_str(), t))
    }

    /// Returns the names of all transforms.
    #[must_use]
    pub fn names(&self) -> Vec<&str> {
        self.transforms.iter().map(|(n, _)| n.as_str()).collect()
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
    use std::f32::consts::PI;

    #[test]
    fn transform_identity() {
        let t = Transform3D::identity();
        let point = Vec3::new(1.0, 2.0, 3.0);
        let result = t.apply_point(point);
        assert!((result - point).length() < 1e-6);
    }

    #[test]
    fn transform_default() {
        let t = Transform3D::default();
        assert!(t.is_identity(1e-6));
    }

    #[test]
    fn transform_translation() {
        let t = Transform3D::from_translation(Vec3::new(10.0, 20.0, 30.0));
        let result = t.apply_point(Vec3::ZERO);
        assert!((result.x - 10.0).abs() < 1e-6);
        assert!((result.y - 20.0).abs() < 1e-6);
        assert!((result.z - 30.0).abs() < 1e-6);
    }

    #[test]
    fn transform_rotation_z() {
        // Rotate 90 degrees around Z
        let t = Transform3D::from_rotation(Quat::from_rotation_z(PI / 2.0));
        let point = Vec3::new(1.0, 0.0, 0.0);
        let result = t.apply_point(point);

        assert!(result.x.abs() < 1e-6);
        assert!((result.y - 1.0).abs() < 1e-6);
        assert!(result.z.abs() < 1e-6);
    }

    #[test]
    fn transform_direction() {
        let t = Transform3D::from_translation(Vec3::new(100.0, 0.0, 0.0));
        let direction = Vec3::new(1.0, 0.0, 0.0);
        let result = t.apply_direction(direction);

        // Translation should not affect direction
        assert!((result - direction).length() < 1e-6);
    }

    #[test]
    fn transform_inverse() {
        let t = Transform3D::new(Quat::from_rotation_y(PI / 4.0), Vec3::new(10.0, 20.0, 30.0));
        let inv = t.inverse();
        let composed = t.compose(&inv);

        assert!(composed.is_identity(1e-5));
    }

    #[test]
    fn transform_compose() {
        let t1 = Transform3D::from_translation(Vec3::new(10.0, 0.0, 0.0));
        let t2 = Transform3D::from_translation(Vec3::new(0.0, 5.0, 0.0));
        let composed = t1.compose(&t2);

        let result = composed.apply_point(Vec3::ZERO);
        assert!((result.x - 10.0).abs() < 1e-6);
        assert!((result.y - 5.0).abs() < 1e-6);
    }

    #[test]
    fn transform_lerp() {
        let t1 = Transform3D::from_translation(Vec3::ZERO);
        let t2 = Transform3D::from_translation(Vec3::new(10.0, 0.0, 0.0));
        let mid = t1.lerp(&t2, 0.5);

        assert!((mid.translation.x - 5.0).abs() < 1e-6);
    }

    #[test]
    fn transform_to_matrix() {
        let t = Transform3D::new(Quat::IDENTITY, Vec3::new(1.0, 2.0, 3.0));
        let mat = t.to_matrix();
        let restored = Transform3D::from_matrix(mat);

        assert!((t.translation - restored.translation).length() < 1e-6);
    }

    #[test]
    fn transform_euler() {
        let t = Transform3D::from_euler(0.0, 0.0, 0.0);
        assert!(t.is_identity(1e-6));
    }

    #[test]
    fn transform_serialization() {
        let t = Transform3D::new(Quat::from_rotation_z(0.5), Vec3::new(1.0, 2.0, 3.0));

        let json = serde_json::to_string(&t);
        assert!(json.is_ok());

        let parsed: std::result::Result<Transform3D, _> =
            serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
    }

    #[test]
    fn chain_new() {
        let chain = TransformChain::new();
        assert!(chain.is_empty());
        assert_eq!(chain.len(), 0);
    }

    #[test]
    fn chain_push_get() {
        let mut chain = TransformChain::new();
        chain.push("t1", Transform3D::from_translation(Vec3::X));

        assert_eq!(chain.len(), 1);
        assert!(chain.get("t1").is_some());
        assert!(chain.get("t2").is_none());
    }

    #[test]
    fn chain_remove() {
        let mut chain = TransformChain::new();
        chain.push("t1", Transform3D::identity());

        assert!(chain.remove("t1").is_some());
        assert!(chain.is_empty());
        assert!(chain.remove("t1").is_none());
    }

    #[test]
    fn chain_compose() {
        let mut chain = TransformChain::new();
        chain.push(
            "t1",
            Transform3D::from_translation(Vec3::new(10.0, 0.0, 0.0)),
        );
        chain.push(
            "t2",
            Transform3D::from_translation(Vec3::new(0.0, 5.0, 0.0)),
        );

        let composed = chain.compose().unwrap();
        let result = composed.apply_point(Vec3::ZERO);

        assert!((result.x - 10.0).abs() < 1e-6);
        assert!((result.y - 5.0).abs() < 1e-6);
    }

    #[test]
    fn chain_compose_empty() {
        let chain = TransformChain::new();
        assert!(chain.compose().is_err());
    }

    #[test]
    fn chain_compose_to() {
        let mut chain = TransformChain::new();
        chain.push(
            "t1",
            Transform3D::from_translation(Vec3::new(10.0, 0.0, 0.0)),
        );
        chain.push(
            "t2",
            Transform3D::from_translation(Vec3::new(0.0, 5.0, 0.0)),
        );
        chain.push(
            "t3",
            Transform3D::from_translation(Vec3::new(0.0, 0.0, 3.0)),
        );

        let to_t2 = chain.compose_to("t2").unwrap();
        let result = to_t2.apply_point(Vec3::ZERO);

        assert!((result.x - 10.0).abs() < 1e-6);
        assert!((result.y - 5.0).abs() < 1e-6);
        assert!(result.z.abs() < 1e-6); // t3 not included
    }

    #[test]
    fn chain_compose_to_not_found() {
        let mut chain = TransformChain::new();
        chain.push("t1", Transform3D::identity());

        assert!(chain.compose_to("t2").is_err());
    }

    #[test]
    fn chain_names() {
        let mut chain = TransformChain::new();
        chain.push("a", Transform3D::identity());
        chain.push("b", Transform3D::identity());

        let names = chain.names();
        assert_eq!(names, vec!["a", "b"]);
    }

    #[test]
    fn chain_iter() {
        let mut chain = TransformChain::new();
        chain.push("a", Transform3D::identity());
        chain.push("b", Transform3D::identity());

        let count = chain.iter().count();
        assert_eq!(count, 2);
    }

    #[test]
    fn chain_clear() {
        let mut chain = TransformChain::new();
        chain.push("a", Transform3D::identity());
        chain.clear();

        assert!(chain.is_empty());
    }

    #[test]
    fn chain_get_mut() {
        let mut chain = TransformChain::new();
        chain.push("t1", Transform3D::identity());

        if let Some(t) = chain.get_mut("t1") {
            t.translation = Vec3::new(5.0, 0.0, 0.0);
        }

        let t = chain.get("t1").unwrap();
        assert!((t.translation.x - 5.0).abs() < 1e-6);
    }
}
