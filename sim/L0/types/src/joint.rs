//! Joint types for articulated bodies.
//!
//! Joints connect rigid bodies and constrain their relative motion.
//! This module provides types for representing joint state, limits, and types.

use nalgebra::Vector3;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Unique identifier for a joint in the simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointId(pub u64);

impl JointId {
    /// Create a new joint ID.
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

impl From<u64> for JointId {
    fn from(id: u64) -> Self {
        Self(id)
    }
}

impl std::fmt::Display for JointId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Joint({})", self.0)
    }
}

/// Type of joint constraint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum JointType {
    /// Fixed joint - no relative motion allowed.
    Fixed,
    /// Revolute joint - rotation around a single axis.
    Revolute,
    /// Prismatic joint - translation along a single axis.
    Prismatic,
    /// Spherical joint - rotation around all axes (ball joint).
    Spherical,
    /// Cylindrical joint - rotation and translation along the same axis.
    Cylindrical,
    /// Planar joint - translation in a plane.
    Planar,
    /// Free joint - 6 DOF (floating base).
    Free,
}

impl JointType {
    /// Get the number of degrees of freedom for this joint type.
    #[must_use]
    pub const fn dof(self) -> usize {
        match self {
            Self::Fixed => 0,
            Self::Revolute | Self::Prismatic => 1,
            Self::Cylindrical => 2,
            Self::Planar | Self::Spherical => 3,
            Self::Free => 6,
        }
    }

    /// Check if this joint type has rotational degrees of freedom.
    #[must_use]
    pub const fn has_rotation(self) -> bool {
        matches!(
            self,
            Self::Revolute | Self::Spherical | Self::Cylindrical | Self::Free
        )
    }

    /// Check if this joint type has translational degrees of freedom.
    #[must_use]
    pub const fn has_translation(self) -> bool {
        matches!(
            self,
            Self::Prismatic | Self::Cylindrical | Self::Planar | Self::Free
        )
    }
}

impl std::fmt::Display for JointType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Fixed => write!(f, "fixed"),
            Self::Revolute => write!(f, "revolute"),
            Self::Prismatic => write!(f, "prismatic"),
            Self::Spherical => write!(f, "spherical"),
            Self::Cylindrical => write!(f, "cylindrical"),
            Self::Planar => write!(f, "planar"),
            Self::Free => write!(f, "free"),
        }
    }
}

/// Position and velocity limits for a joint.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointLimits {
    /// Minimum position (angle for revolute, distance for prismatic).
    pub position_min: f64,
    /// Maximum position.
    pub position_max: f64,
    /// Maximum velocity magnitude.
    pub velocity_max: f64,
    /// Maximum effort (force/torque) magnitude.
    pub effort_max: f64,
}

impl Default for JointLimits {
    fn default() -> Self {
        Self::unlimited()
    }
}

impl JointLimits {
    /// Create limits with specified bounds.
    #[must_use]
    pub fn new(position_min: f64, position_max: f64, velocity_max: f64, effort_max: f64) -> Self {
        Self {
            position_min,
            position_max,
            velocity_max,
            effort_max,
        }
    }

    /// Create unlimited joint limits.
    #[must_use]
    pub fn unlimited() -> Self {
        Self {
            position_min: f64::NEG_INFINITY,
            position_max: f64::INFINITY,
            velocity_max: f64::INFINITY,
            effort_max: f64::INFINITY,
        }
    }

    /// Create symmetric position limits around zero.
    #[must_use]
    pub fn symmetric(position_range: f64, velocity_max: f64, effort_max: f64) -> Self {
        Self {
            position_min: -position_range,
            position_max: position_range,
            velocity_max,
            effort_max,
        }
    }

    /// Create typical revolute joint limits (in radians).
    #[must_use]
    pub fn revolute(min_angle: f64, max_angle: f64, max_speed: f64, max_torque: f64) -> Self {
        Self::new(min_angle, max_angle, max_speed, max_torque)
    }

    /// Create typical prismatic joint limits (in meters).
    #[must_use]
    pub fn prismatic(min_pos: f64, max_pos: f64, max_speed: f64, max_force: f64) -> Self {
        Self::new(min_pos, max_pos, max_speed, max_force)
    }

    /// Check if a position is within limits.
    #[must_use]
    pub fn position_in_range(&self, position: f64) -> bool {
        position >= self.position_min && position <= self.position_max
    }

    /// Check if a velocity is within limits.
    #[must_use]
    pub fn velocity_in_range(&self, velocity: f64) -> bool {
        velocity.abs() <= self.velocity_max
    }

    /// Check if an effort is within limits.
    #[must_use]
    pub fn effort_in_range(&self, effort: f64) -> bool {
        effort.abs() <= self.effort_max
    }

    /// Clamp a position to be within limits.
    #[must_use]
    pub fn clamp_position(&self, position: f64) -> f64 {
        position.clamp(self.position_min, self.position_max)
    }

    /// Clamp a velocity to be within limits.
    #[must_use]
    pub fn clamp_velocity(&self, velocity: f64) -> f64 {
        velocity.clamp(-self.velocity_max, self.velocity_max)
    }

    /// Clamp an effort to be within limits.
    #[must_use]
    pub fn clamp_effort(&self, effort: f64) -> f64 {
        effort.clamp(-self.effort_max, self.effort_max)
    }

    /// Get the position range.
    #[must_use]
    pub fn position_range(&self) -> f64 {
        self.position_max - self.position_min
    }

    /// Check if position limits are bounded (not infinite).
    #[must_use]
    pub fn has_position_limits(&self) -> bool {
        self.position_min.is_finite() && self.position_max.is_finite()
    }
}

/// State of a single-DOF joint (revolute or prismatic).
///
/// For joints with multiple DOF, use arrays of `JointState`.
///
/// # Example
///
/// ```
/// use sim_types::JointState;
///
/// let state = JointState::new(0.5, 0.1);
/// assert_eq!(state.position, 0.5);
/// assert_eq!(state.velocity, 0.1);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointState {
    /// Joint position (angle in radians for revolute, distance in meters for prismatic).
    pub position: f64,
    /// Joint velocity (angular velocity or linear velocity).
    pub velocity: f64,
}

impl JointState {
    /// Create a joint state with the given position and velocity.
    #[must_use]
    pub fn new(position: f64, velocity: f64) -> Self {
        Self { position, velocity }
    }

    /// Create a joint state at the given position with zero velocity.
    #[must_use]
    pub fn at_position(position: f64) -> Self {
        Self {
            position,
            velocity: 0.0,
        }
    }

    /// Create a joint state at zero position and velocity.
    #[must_use]
    pub fn zero() -> Self {
        Self::default()
    }

    /// Check if the state satisfies the given limits.
    #[must_use]
    pub fn within_limits(&self, limits: &JointLimits) -> bool {
        limits.position_in_range(self.position) && limits.velocity_in_range(self.velocity)
    }

    /// Clamp the state to be within the given limits.
    #[must_use]
    pub fn clamped(&self, limits: &JointLimits) -> Self {
        Self {
            position: limits.clamp_position(self.position),
            velocity: limits.clamp_velocity(self.velocity),
        }
    }

    /// Linear interpolation between two joint states.
    #[must_use]
    pub fn lerp(&self, other: &Self, t: f64) -> Self {
        let t = t.clamp(0.0, 1.0);
        Self {
            position: self.position + (other.position - self.position) * t,
            velocity: self.velocity + (other.velocity - self.velocity) * t,
        }
    }

    /// Check if the state contains `NaN` or `Inf` values.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.position.is_finite() && self.velocity.is_finite()
    }

    /// Compute the kinetic energy of this joint given inertia.
    #[must_use]
    pub fn kinetic_energy(&self, inertia: f64) -> f64 {
        0.5 * inertia * self.velocity * self.velocity
    }
}

/// Extended joint state with acceleration and effort.
///
/// Used when full dynamics information is needed.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointStateExtended {
    /// Basic joint state (position, velocity).
    pub state: JointState,
    /// Joint acceleration.
    pub acceleration: f64,
    /// Applied effort (torque for revolute, force for prismatic).
    pub effort: f64,
}

impl JointStateExtended {
    /// Create an extended joint state.
    #[must_use]
    pub fn new(position: f64, velocity: f64, acceleration: f64, effort: f64) -> Self {
        Self {
            state: JointState::new(position, velocity),
            acceleration,
            effort,
        }
    }

    /// Create from basic state with zero acceleration and effort.
    #[must_use]
    pub fn from_state(state: JointState) -> Self {
        Self {
            state,
            acceleration: 0.0,
            effort: 0.0,
        }
    }

    /// Get the position.
    #[must_use]
    pub fn position(&self) -> f64 {
        self.state.position
    }

    /// Get the velocity.
    #[must_use]
    pub fn velocity(&self) -> f64 {
        self.state.velocity
    }
}

/// Axis specification for joints.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointAxis {
    /// Axis direction in the parent body's local frame (normalized).
    pub direction: Vector3<f64>,
}

impl Default for JointAxis {
    fn default() -> Self {
        Self::z()
    }
}

impl JointAxis {
    /// Create a joint axis along the given direction (will be normalized).
    #[must_use]
    pub fn new(direction: Vector3<f64>) -> Self {
        let norm = direction.norm();
        if norm < 1e-10 {
            Self::z()
        } else {
            Self {
                direction: direction / norm,
            }
        }
    }

    /// X-axis.
    #[must_use]
    pub fn x() -> Self {
        Self {
            direction: Vector3::x(),
        }
    }

    /// Y-axis.
    #[must_use]
    pub fn y() -> Self {
        Self {
            direction: Vector3::y(),
        }
    }

    /// Z-axis.
    #[must_use]
    pub fn z() -> Self {
        Self {
            direction: Vector3::z(),
        }
    }

    /// Negative X-axis.
    #[must_use]
    pub fn neg_x() -> Self {
        Self {
            direction: -Vector3::x(),
        }
    }

    /// Negative Y-axis.
    #[must_use]
    pub fn neg_y() -> Self {
        Self {
            direction: -Vector3::y(),
        }
    }

    /// Negative Z-axis.
    #[must_use]
    pub fn neg_z() -> Self {
        Self {
            direction: -Vector3::z(),
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    #[test]
    fn test_joint_id() {
        let id = JointId::new(42);
        assert_eq!(id.raw(), 42);
        assert_eq!(id.to_string(), "Joint(42)");
    }

    #[test]
    fn test_joint_type_dof() {
        assert_eq!(JointType::Fixed.dof(), 0);
        assert_eq!(JointType::Revolute.dof(), 1);
        assert_eq!(JointType::Prismatic.dof(), 1);
        assert_eq!(JointType::Spherical.dof(), 3);
        assert_eq!(JointType::Free.dof(), 6);
    }

    #[test]
    fn test_joint_limits() {
        let limits = JointLimits::revolute(-PI, PI, 10.0, 100.0);

        assert!(limits.position_in_range(0.0));
        assert!(limits.position_in_range(PI - 0.1));
        assert!(!limits.position_in_range(PI + 0.1));

        assert!(limits.velocity_in_range(5.0));
        assert!(!limits.velocity_in_range(15.0));

        assert_relative_eq!(limits.clamp_position(100.0), PI, epsilon = 1e-10);
        assert_relative_eq!(limits.clamp_position(-100.0), -PI, epsilon = 1e-10);
    }

    #[test]
    fn test_joint_limits_symmetric() {
        let limits = JointLimits::symmetric(PI, 5.0, 50.0);
        assert_relative_eq!(limits.position_min, -PI, epsilon = 1e-10);
        assert_relative_eq!(limits.position_max, PI, epsilon = 1e-10);
    }

    #[test]
    fn test_joint_state() {
        let state = JointState::new(1.0, 2.0);
        assert_eq!(state.position, 1.0);
        assert_eq!(state.velocity, 2.0);

        let limits = JointLimits::symmetric(0.5, 1.0, 10.0);
        assert!(!state.within_limits(&limits));

        let clamped = state.clamped(&limits);
        assert!(clamped.within_limits(&limits));
    }

    #[test]
    fn test_joint_state_lerp() {
        let s1 = JointState::new(0.0, 0.0);
        let s2 = JointState::new(10.0, 20.0);

        let mid = s1.lerp(&s2, 0.5);
        assert_relative_eq!(mid.position, 5.0, epsilon = 1e-10);
        assert_relative_eq!(mid.velocity, 10.0, epsilon = 1e-10);
    }

    #[test]
    fn test_joint_axis() {
        let axis = JointAxis::new(Vector3::new(1.0, 1.0, 0.0));
        let expected = Vector3::new(1.0, 1.0, 0.0).normalize();
        assert_relative_eq!(axis.direction, expected, epsilon = 1e-10);
    }

    #[test]
    fn test_joint_type_properties() {
        assert!(JointType::Revolute.has_rotation());
        assert!(!JointType::Revolute.has_translation());

        assert!(!JointType::Prismatic.has_rotation());
        assert!(JointType::Prismatic.has_translation());

        assert!(JointType::Free.has_rotation());
        assert!(JointType::Free.has_translation());
    }

    #[test]
    fn test_joint_state_extended() {
        let extended = JointStateExtended::new(1.0, 2.0, 3.0, 4.0);
        assert_relative_eq!(extended.position(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(extended.velocity(), 2.0, epsilon = 1e-10);
        assert_relative_eq!(extended.acceleration, 3.0, epsilon = 1e-10);
        assert_relative_eq!(extended.effort, 4.0, epsilon = 1e-10);
    }

    #[test]
    fn test_joint_state_extended_from_state() {
        let state = JointState::new(1.5, 2.5);
        let extended = JointStateExtended::from_state(state);
        assert_relative_eq!(extended.position(), 1.5, epsilon = 1e-10);
        assert_relative_eq!(extended.velocity(), 2.5, epsilon = 1e-10);
        assert_relative_eq!(extended.acceleration, 0.0, epsilon = 1e-10);
        assert_relative_eq!(extended.effort, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_joint_axis_presets() {
        let x = JointAxis::x();
        let y = JointAxis::y();
        let z = JointAxis::z();
        let neg_x = JointAxis::neg_x();
        let neg_y = JointAxis::neg_y();
        let neg_z = JointAxis::neg_z();

        assert_relative_eq!(x.direction, Vector3::x(), epsilon = 1e-10);
        assert_relative_eq!(y.direction, Vector3::y(), epsilon = 1e-10);
        assert_relative_eq!(z.direction, Vector3::z(), epsilon = 1e-10);
        assert_relative_eq!(neg_x.direction, -Vector3::x(), epsilon = 1e-10);
        assert_relative_eq!(neg_y.direction, -Vector3::y(), epsilon = 1e-10);
        assert_relative_eq!(neg_z.direction, -Vector3::z(), epsilon = 1e-10);
    }
}
