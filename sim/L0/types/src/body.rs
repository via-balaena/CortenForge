//! Body identity and pose types.
//!
//! This module provides types for identifying bodies and representing their
//! position and orientation in 3D space.

use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Unique identifier for a rigid body in the simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BodyId(pub u64);

impl BodyId {
    /// Create a new body ID.
    #[must_use]
    pub const fn new(id: u64) -> Self {
        Self(id)
    }

    /// Get the raw ID value.
    #[must_use]
    pub const fn raw(self) -> u64 {
        self.0
    }
}

impl From<u64> for BodyId {
    fn from(id: u64) -> Self {
        Self(id)
    }
}

impl std::fmt::Display for BodyId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Body({})", self.0)
    }
}

/// Position and orientation of a rigid body.
///
/// Represents the pose (configuration) of a body in 3D space using
/// a position vector and a unit quaternion for orientation.
///
/// # Example
///
/// ```
/// use sim_types::Pose;
/// use nalgebra::{Point3, UnitQuaternion, Vector3};
///
/// // Create a pose at position (1, 2, 3) with identity rotation
/// let pose = Pose::from_position(Point3::new(1.0, 2.0, 3.0));
///
/// // Transform a local point to world coordinates
/// let local = Point3::new(1.0, 0.0, 0.0);
/// let world = pose.transform_point(&local);
/// assert_eq!(world, Point3::new(2.0, 2.0, 3.0));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Pose {
    /// Position in world coordinates.
    pub position: Point3<f64>,
    /// Orientation as a unit quaternion.
    pub rotation: UnitQuaternion<f64>,
}

impl Default for Pose {
    fn default() -> Self {
        Self::identity()
    }
}

impl Pose {
    /// Create an identity pose (origin, no rotation).
    #[must_use]
    pub fn identity() -> Self {
        Self {
            position: Point3::origin(),
            rotation: UnitQuaternion::identity(),
        }
    }

    /// Create a pose from position only (identity rotation).
    #[must_use]
    pub fn from_position(position: Point3<f64>) -> Self {
        Self {
            position,
            rotation: UnitQuaternion::identity(),
        }
    }

    /// Create a pose from position and rotation.
    #[must_use]
    pub const fn from_position_rotation(
        position: Point3<f64>,
        rotation: UnitQuaternion<f64>,
    ) -> Self {
        Self { position, rotation }
    }

    /// Create a pose from an isometry.
    #[must_use]
    pub fn from_isometry(iso: Isometry3<f64>) -> Self {
        Self {
            position: Point3::from(iso.translation.vector),
            rotation: iso.rotation,
        }
    }

    /// Convert to an isometry.
    #[must_use]
    pub fn to_isometry(&self) -> Isometry3<f64> {
        Isometry3::from_parts(self.position.coords.into(), self.rotation)
    }

    /// Transform a point from local to world coordinates.
    #[must_use]
    pub fn transform_point(&self, local: &Point3<f64>) -> Point3<f64> {
        self.position + self.rotation * local.coords
    }

    /// Transform a vector from local to world coordinates (rotation only).
    #[must_use]
    pub fn transform_vector(&self, local: &Vector3<f64>) -> Vector3<f64> {
        self.rotation * local
    }

    /// Transform a point from world to local coordinates.
    #[must_use]
    pub fn inverse_transform_point(&self, world: &Point3<f64>) -> Point3<f64> {
        Point3::from(self.rotation.inverse() * (world - self.position))
    }

    /// Transform a vector from world to local coordinates.
    #[must_use]
    pub fn inverse_transform_vector(&self, world: &Vector3<f64>) -> Vector3<f64> {
        self.rotation.inverse() * world
    }

    /// Get the forward direction (local +Y in world coordinates).
    #[must_use]
    pub fn forward(&self) -> Vector3<f64> {
        self.transform_vector(&Vector3::y())
    }

    /// Get the right direction (local +X in world coordinates).
    #[must_use]
    pub fn right(&self) -> Vector3<f64> {
        self.transform_vector(&Vector3::x())
    }

    /// Get the up direction (local +Z in world coordinates).
    #[must_use]
    pub fn up(&self) -> Vector3<f64> {
        self.transform_vector(&Vector3::z())
    }

    /// Compute the inverse pose.
    #[must_use]
    pub fn inverse(&self) -> Self {
        let inv_rotation = self.rotation.inverse();
        Self {
            position: Point3::from(-(inv_rotation * self.position.coords)),
            rotation: inv_rotation,
        }
    }

    /// Compose two poses: self * other.
    #[must_use]
    pub fn compose(&self, other: &Self) -> Self {
        Self {
            position: self.transform_point(&other.position),
            rotation: self.rotation * other.rotation,
        }
    }

    /// Linear interpolation between two poses.
    ///
    /// Uses SLERP for rotation interpolation.
    #[must_use]
    pub fn lerp(&self, other: &Self, t: f64) -> Self {
        let t = t.clamp(0.0, 1.0);
        Self {
            position: Point3::from(self.position.coords.lerp(&other.position.coords, t)),
            rotation: self.rotation.slerp(&other.rotation, t),
        }
    }

    /// Check if the pose contains `NaN` or `Inf` values.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.position.coords.iter().all(|x| x.is_finite())
            && self.rotation.coords.iter().all(|x| x.is_finite())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_body_id() {
        let id = BodyId::new(42);
        assert_eq!(id.raw(), 42);
        assert_eq!(id.to_string(), "Body(42)");

        let id2: BodyId = 42.into();
        assert_eq!(id, id2);
    }

    #[test]
    fn test_pose_identity() {
        let pose = Pose::identity();
        let p = Point3::new(1.0, 2.0, 3.0);
        let transformed = pose.transform_point(&p);
        assert_relative_eq!(transformed.coords, p.coords, epsilon = 1e-10);
    }

    #[test]
    fn test_pose_translation() {
        let pose = Pose::from_position(Point3::new(10.0, 0.0, 0.0));
        let local = Point3::origin();
        let world = pose.transform_point(&local);
        assert_relative_eq!(world.x, 10.0, epsilon = 1e-10);
    }

    #[test]
    fn test_pose_rotation() {
        // 90 degree rotation around Z
        let pose = Pose::from_position_rotation(
            Point3::origin(),
            UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2),
        );

        let local = Vector3::new(1.0, 0.0, 0.0);
        let world = pose.transform_vector(&local);

        assert_relative_eq!(world.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(world.y, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_pose_inverse() {
        let pose = Pose::from_position_rotation(
            Point3::new(1.0, 2.0, 3.0),
            UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3),
        );

        let inv = pose.inverse();
        let composed = pose.compose(&inv);

        assert_relative_eq!(composed.position.coords, Vector3::zeros(), epsilon = 1e-10);
    }

    #[test]
    fn test_pose_compose() {
        let p1 = Pose::from_position(Point3::new(1.0, 0.0, 0.0));
        let p2 = Pose::from_position(Point3::new(0.0, 1.0, 0.0));

        let composed = p1.compose(&p2);
        assert_relative_eq!(composed.position.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(composed.position.y, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_pose_lerp() {
        let p1 = Pose::from_position(Point3::new(0.0, 0.0, 0.0));
        let p2 = Pose::from_position(Point3::new(10.0, 0.0, 0.0));

        let mid = p1.lerp(&p2, 0.5);
        assert_relative_eq!(mid.position.x, 5.0, epsilon = 1e-10);
    }

    /// A pose with both a translation and a rotation, so a test that passes
    /// under it cannot be passing because one half happened to be identity.
    fn posed() -> Pose {
        Pose::from_position_rotation(
            Point3::new(1.0, 2.0, 3.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2),
        )
    }

    /// `from_isometry` and `to_isometry` are mutual inverses.
    #[test]
    fn isometry_conversion_round_trips() {
        let pose = posed();
        let back = Pose::from_isometry(pose.to_isometry());

        assert_relative_eq!(back.position.coords, pose.position.coords, epsilon = 1e-12);
        assert_relative_eq!(back.rotation, pose.rotation, epsilon = 1e-12);
    }

    /// ★ The point transform must undo itself, translation included.
    #[test]
    fn inverse_transform_point_undoes_transform_point() {
        let pose = posed();
        let local = Point3::new(0.5, -1.5, 2.0);

        let round_tripped = pose.inverse_transform_point(&pose.transform_point(&local));

        assert_relative_eq!(round_tripped.coords, local.coords, epsilon = 1e-12);
    }

    /// ★★ Vectors are rotated but NOT translated — the distinction between the
    /// point and vector transforms, and the classic way to get them confused.
    ///
    /// Under `posed()` (translation (1,2,3), +90° about Z), local +X maps to
    /// world +Y as a *direction*. Applying the point transform to the same
    /// input would additionally shift by the translation, so this pins the
    /// vector path specifically rather than round-tripping (which would pass
    /// even if both paths translated).
    #[test]
    fn transform_vector_rotates_without_translating() {
        let pose = posed();
        let world = pose.transform_vector(&Vector3::x());

        assert_relative_eq!(world, Vector3::y(), epsilon = 1e-12);
        assert_relative_eq!(
            pose.inverse_transform_vector(&world),
            Vector3::x(),
            epsilon = 1e-12
        );
    }

    /// `forward`/`right`/`up` name local +Y/+X/+Z expressed in world axes.
    /// Under a +90° yaw about Z: +X→+Y, +Y→−X, and +Z is unmoved.
    #[test]
    fn basis_accessors_are_the_local_axes_in_world_coordinates() {
        let pose = posed();

        assert_relative_eq!(pose.right(), Vector3::y(), epsilon = 1e-12);
        assert_relative_eq!(pose.forward(), -Vector3::x(), epsilon = 1e-12);
        assert_relative_eq!(pose.up(), Vector3::z(), epsilon = 1e-12);
    }

    /// ★ `is_finite` guards position AND rotation — two `&&` operands, so each
    /// needs its own case or one of them can be deleted with every test still
    /// green.
    #[test]
    fn is_finite_rejects_a_non_finite_position_or_rotation() {
        assert!(posed().is_finite());

        let bad_position = Pose::from_position_rotation(
            Point3::new(f64::NAN, 0.0, 0.0),
            UnitQuaternion::identity(),
        );
        assert!(!bad_position.is_finite(), "NaN position must be rejected");

        let bad_rotation = Pose::from_position_rotation(
            Point3::origin(),
            UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(f64::NAN, 0.0, 0.0, 0.0)),
        );
        assert!(!bad_rotation.is_finite(), "NaN rotation must be rejected");

        let infinite = Pose::from_position_rotation(
            Point3::new(0.0, f64::INFINITY, 0.0),
            UnitQuaternion::identity(),
        );
        assert!(!infinite.is_finite(), "Inf is not finite either");
    }
}
