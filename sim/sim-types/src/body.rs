//! Rigid body state types.
//!
//! This module provides types for representing rigid body state in 6 degrees
//! of freedom: position, orientation, linear velocity, and angular velocity.

use nalgebra::{Isometry3, Matrix3, Point3, UnitQuaternion, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Unique identifier for a rigid body in the simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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

/// Linear and angular velocity of a rigid body.
///
/// # Example
///
/// ```
/// use sim_types::Twist;
/// use nalgebra::Vector3;
///
/// // Create a twist with linear velocity only
/// let twist = Twist::linear(Vector3::new(1.0, 0.0, 0.0));
/// assert_eq!(twist.linear.x, 1.0);
/// assert_eq!(twist.angular.norm(), 0.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Twist {
    /// Linear velocity in world coordinates (m/s).
    pub linear: Vector3<f64>,
    /// Angular velocity in world coordinates (rad/s).
    pub angular: Vector3<f64>,
}

impl Default for Twist {
    fn default() -> Self {
        Self::zero()
    }
}

impl Twist {
    /// Create a twist with specified linear and angular velocity.
    #[must_use]
    pub const fn new(linear: Vector3<f64>, angular: Vector3<f64>) -> Self {
        Self { linear, angular }
    }

    /// Create a zero twist (at rest).
    #[must_use]
    pub fn zero() -> Self {
        Self {
            linear: Vector3::zeros(),
            angular: Vector3::zeros(),
        }
    }

    /// Create a twist with linear velocity only.
    #[must_use]
    pub fn linear(v: Vector3<f64>) -> Self {
        Self {
            linear: v,
            angular: Vector3::zeros(),
        }
    }

    /// Create a twist with angular velocity only.
    #[must_use]
    pub fn angular(omega: Vector3<f64>) -> Self {
        Self {
            linear: Vector3::zeros(),
            angular: omega,
        }
    }

    /// Compute the velocity at a point offset from the body origin.
    ///
    /// `v_point` = `v_linear` + omega × r
    #[must_use]
    pub fn velocity_at_point(&self, offset: &Vector3<f64>) -> Vector3<f64> {
        self.linear + self.angular.cross(offset)
    }

    /// Compute kinetic energy given mass properties.
    #[must_use]
    pub fn kinetic_energy(&self, mass: f64, inertia: &Matrix3<f64>) -> f64 {
        let linear_ke = 0.5 * mass * self.linear.norm_squared();
        let angular_ke = 0.5 * self.angular.dot(&(inertia * self.angular));
        linear_ke + angular_ke
    }

    /// Compute momentum given mass.
    #[must_use]
    pub fn linear_momentum(&self, mass: f64) -> Vector3<f64> {
        self.linear * mass
    }

    /// Compute angular momentum given inertia tensor.
    #[must_use]
    pub fn angular_momentum(&self, inertia: &Matrix3<f64>) -> Vector3<f64> {
        inertia * self.angular
    }

    /// Scale the twist by a factor.
    #[must_use]
    pub fn scale(&self, factor: f64) -> Self {
        Self {
            linear: self.linear * factor,
            angular: self.angular * factor,
        }
    }

    /// Add two twists.
    #[must_use]
    pub fn add(&self, other: &Self) -> Self {
        Self {
            linear: self.linear + other.linear,
            angular: self.angular + other.angular,
        }
    }

    /// Check if the twist contains `NaN` or `Inf` values.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.linear.iter().all(|x| x.is_finite()) && self.angular.iter().all(|x| x.is_finite())
    }

    /// Get the linear speed (magnitude of linear velocity).
    #[must_use]
    pub fn speed(&self) -> f64 {
        self.linear.norm()
    }

    /// Get the angular speed (magnitude of angular velocity).
    #[must_use]
    pub fn angular_speed(&self) -> f64 {
        self.angular.norm()
    }
}

/// Complete state of a rigid body.
///
/// Combines pose (position + orientation) with twist (linear + angular velocity).
///
/// # Example
///
/// ```
/// use sim_types::{RigidBodyState, Pose, Twist};
/// use nalgebra::{Point3, Vector3};
///
/// let state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 1.0)));
/// assert_eq!(state.pose.position.z, 1.0);
/// assert!(state.twist.speed() < 1e-10);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RigidBodyState {
    /// Position and orientation.
    pub pose: Pose,
    /// Linear and angular velocity.
    pub twist: Twist,
}

impl RigidBodyState {
    /// Create a state from pose and twist.
    #[must_use]
    pub const fn new(pose: Pose, twist: Twist) -> Self {
        Self { pose, twist }
    }

    /// Create a state at rest at the given pose.
    #[must_use]
    pub fn at_rest(pose: Pose) -> Self {
        Self {
            pose,
            twist: Twist::zero(),
        }
    }

    /// Create a state at the origin, at rest.
    #[must_use]
    pub fn origin() -> Self {
        Self::default()
    }

    /// Check if the state contains `NaN` or `Inf` values.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.pose.is_finite() && self.twist.is_finite()
    }

    /// Linear interpolation between two states.
    #[must_use]
    pub fn lerp(&self, other: &Self, t: f64) -> Self {
        let t = t.clamp(0.0, 1.0);
        Self {
            pose: self.pose.lerp(&other.pose, t),
            twist: Twist {
                linear: self.twist.linear.lerp(&other.twist.linear, t),
                angular: self.twist.angular.lerp(&other.twist.angular, t),
            },
        }
    }
}

/// Mass properties of a rigid body.
///
/// Contains mass, center of mass offset, and inertia tensor.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MassProperties {
    /// Total mass in kg.
    pub mass: f64,
    /// Center of mass offset from body origin in local coordinates.
    pub center_of_mass: Vector3<f64>,
    /// Inertia tensor about center of mass in local coordinates (kg·m²).
    pub inertia: Matrix3<f64>,
}

impl MassProperties {
    /// Create mass properties with given values.
    ///
    /// # Arguments
    ///
    /// * `mass` - Total mass (must be positive)
    /// * `center_of_mass` - COM offset from body origin
    /// * `inertia` - Inertia tensor about COM
    #[must_use]
    pub const fn new(mass: f64, center_of_mass: Vector3<f64>, inertia: Matrix3<f64>) -> Self {
        Self {
            mass,
            center_of_mass,
            inertia,
        }
    }

    /// Create mass properties for a point mass at the origin.
    #[must_use]
    pub fn point_mass(mass: f64) -> Self {
        Self {
            mass,
            center_of_mass: Vector3::zeros(),
            inertia: Matrix3::zeros(),
        }
    }

    /// Create mass properties for a uniform sphere.
    ///
    /// Inertia of a solid sphere: I = (2/5) * m * r²
    #[must_use]
    pub fn sphere(mass: f64, radius: f64) -> Self {
        let i = 0.4 * mass * radius * radius;
        Self {
            mass,
            center_of_mass: Vector3::zeros(),
            inertia: Matrix3::from_diagonal(&Vector3::new(i, i, i)),
        }
    }

    /// Create mass properties for a uniform box.
    ///
    /// Inertia of a solid box with dimensions (x, y, z):
    /// - Ixx = (1/12) * m * (y² + z²)
    /// - Iyy = (1/12) * m * (x² + z²)
    /// - Izz = (1/12) * m * (x² + y²)
    #[must_use]
    pub fn box_shape(mass: f64, half_extents: Vector3<f64>) -> Self {
        let x2 = 4.0 * half_extents.x * half_extents.x;
        let y2 = 4.0 * half_extents.y * half_extents.y;
        let z2 = 4.0 * half_extents.z * half_extents.z;

        let ixx = mass * (y2 + z2) / 12.0;
        let iyy = mass * (x2 + z2) / 12.0;
        let izz = mass * (x2 + y2) / 12.0;

        Self {
            mass,
            center_of_mass: Vector3::zeros(),
            inertia: Matrix3::from_diagonal(&Vector3::new(ixx, iyy, izz)),
        }
    }

    /// Create mass properties for a uniform cylinder (aligned with Z axis).
    ///
    /// Inertia of a solid cylinder:
    /// - Ixx = Iyy = (1/12) * m * (3r² + h²)
    /// - Izz = (1/2) * m * r²
    #[must_use]
    pub fn cylinder(mass: f64, radius: f64, half_height: f64) -> Self {
        let r2 = radius * radius;
        let h2 = 4.0 * half_height * half_height;

        let ixx = mass * (3.0 * r2 + h2) / 12.0;
        let izz = 0.5 * mass * r2;

        Self {
            mass,
            center_of_mass: Vector3::zeros(),
            inertia: Matrix3::from_diagonal(&Vector3::new(ixx, ixx, izz)),
        }
    }

    /// Get the inverse mass (0 if mass is infinite/static).
    #[must_use]
    pub fn inverse_mass(&self) -> f64 {
        if self.mass <= 0.0 || self.mass.is_infinite() {
            0.0
        } else {
            1.0 / self.mass
        }
    }

    /// Get the inverse inertia tensor.
    ///
    /// Returns None if the inertia is singular.
    #[must_use]
    pub fn inverse_inertia(&self) -> Option<Matrix3<f64>> {
        self.inertia.try_inverse()
    }

    /// Check if this represents a static (immovable) body.
    #[must_use]
    pub fn is_static(&self) -> bool {
        self.mass <= 0.0 || self.mass.is_infinite()
    }

    /// Validate that the mass properties are physically valid.
    pub fn validate(&self) -> crate::Result<()> {
        if self.mass < 0.0 {
            return Err(crate::SimError::invalid_mass("mass cannot be negative"));
        }

        if !self.mass.is_finite() && self.mass != f64::INFINITY {
            return Err(crate::SimError::invalid_mass(
                "mass must be finite or infinity (static)",
            ));
        }

        if !self.center_of_mass.iter().all(|x| x.is_finite()) {
            return Err(crate::SimError::invalid_mass(
                "center of mass must be finite",
            ));
        }

        // Check that inertia tensor is positive semi-definite
        // (eigenvalues should be non-negative for physical inertia)
        let eigenvalues = self.inertia.symmetric_eigenvalues();
        if eigenvalues.iter().any(|&e| e < -1e-10) {
            return Err(crate::SimError::invalid_mass(
                "inertia tensor must be positive semi-definite",
            ));
        }

        Ok(())
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

    #[test]
    fn test_twist_velocity_at_point() {
        // Spinning around Z axis
        let twist = Twist::angular(Vector3::new(0.0, 0.0, 1.0));
        let offset = Vector3::new(1.0, 0.0, 0.0);

        let v = twist.velocity_at_point(&offset);
        // omega × r = (0,0,1) × (1,0,0) = (0,1,0)
        assert_relative_eq!(v.y, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_twist_kinetic_energy() {
        let twist = Twist::linear(Vector3::new(1.0, 0.0, 0.0));
        let mass = 2.0;
        let inertia = Matrix3::identity();

        let ke = twist.kinetic_energy(mass, &inertia);
        // KE = 0.5 * m * v² = 0.5 * 2 * 1 = 1
        assert_relative_eq!(ke, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_mass_properties_sphere() {
        let props = MassProperties::sphere(1.0, 1.0);
        let expected_i = 0.4; // (2/5) * 1 * 1²

        assert_relative_eq!(props.inertia[(0, 0)], expected_i, epsilon = 1e-10);
        assert_relative_eq!(props.inertia[(1, 1)], expected_i, epsilon = 1e-10);
        assert_relative_eq!(props.inertia[(2, 2)], expected_i, epsilon = 1e-10);
    }

    #[test]
    fn test_mass_properties_box() {
        let props = MassProperties::box_shape(12.0, Vector3::new(0.5, 0.5, 0.5));
        // For a 1x1x1 box with mass 12:
        // I = (1/12) * 12 * (1 + 1) = 2
        assert_relative_eq!(props.inertia[(0, 0)], 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_mass_properties_validation() {
        let valid = MassProperties::sphere(1.0, 1.0);
        assert!(valid.validate().is_ok());

        let negative_mass = MassProperties::new(-1.0, Vector3::zeros(), Matrix3::identity());
        assert!(negative_mass.validate().is_err());
    }

    #[test]
    fn test_rigid_body_state_interpolation() {
        let s1 = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.0)));
        let s2 = RigidBodyState::at_rest(Pose::from_position(Point3::new(10.0, 0.0, 0.0)));

        let mid = s1.lerp(&s2, 0.5);
        assert_relative_eq!(mid.pose.position.x, 5.0, epsilon = 1e-10);
    }
}
