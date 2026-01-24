//! Joint types and their constraint formulations.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_types::BodyId;

use crate::{JointLimits, JointMotor, JointState};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Trait for all joint types.
pub trait Joint {
    /// Get the parent body ID.
    fn parent(&self) -> BodyId;

    /// Get the child body ID.
    fn child(&self) -> BodyId;

    /// Get the number of degrees of freedom.
    fn dof(&self) -> usize;

    /// Get the joint type.
    fn joint_type(&self) -> JointType;

    /// Get the joint limits, if any.
    fn limits(&self) -> Option<&JointLimits>;

    /// Get the joint motor, if any.
    fn motor(&self) -> Option<&JointMotor>;

    /// Get the damping coefficient.
    fn damping(&self) -> f64;

    /// Get the parent anchor point in parent body frame.
    fn parent_anchor(&self) -> Point3<f64>;

    /// Get the child anchor point in child body frame.
    fn child_anchor(&self) -> Point3<f64>;
}

/// Type of joint constraint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum JointType {
    /// Fixed joint (0 DOF) - rigid connection.
    Fixed,
    /// Revolute joint (1 DOF) - single axis rotation.
    Revolute,
    /// Prismatic joint (1 DOF) - single axis translation.
    Prismatic,
    /// Cylindrical joint (2 DOF) - rotation and translation along the same axis.
    Cylindrical,
    /// Planar joint (3 DOF) - translation in a plane plus rotation about the plane normal.
    Planar,
    /// Spherical joint (3 DOF) - ball and socket.
    Spherical,
    /// Universal joint (2 DOF) - two perpendicular rotation axes.
    Universal,
    /// Free joint (6 DOF) - floating base, no constraints.
    Free,
}

impl JointType {
    /// Get the number of degrees of freedom for this joint type.
    #[must_use]
    pub fn dof(&self) -> usize {
        match self {
            Self::Fixed => 0,
            Self::Revolute | Self::Prismatic => 1,
            Self::Cylindrical | Self::Universal => 2,
            Self::Planar | Self::Spherical => 3,
            Self::Free => 6,
        }
    }

    /// Get the number of constrained DOF (6 - dof).
    #[must_use]
    pub fn constrained_dof(&self) -> usize {
        6 - self.dof()
    }

    /// Check if this joint type has rotational degrees of freedom.
    #[must_use]
    pub const fn has_rotation(&self) -> bool {
        matches!(
            self,
            Self::Revolute
                | Self::Spherical
                | Self::Cylindrical
                | Self::Universal
                | Self::Planar
                | Self::Free
        )
    }

    /// Check if this joint type has translational degrees of freedom.
    #[must_use]
    pub const fn has_translation(&self) -> bool {
        matches!(
            self,
            Self::Prismatic | Self::Cylindrical | Self::Planar | Self::Free
        )
    }
}

/// Degrees of freedom specification for a joint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointDof {
    /// Number of rotational DOF.
    pub rotational: usize,
    /// Number of translational DOF.
    pub translational: usize,
}

impl JointDof {
    /// Total DOF.
    #[must_use]
    pub fn total(&self) -> usize {
        self.rotational + self.translational
    }
}

// ============================================================================
// Revolute Joint
// ============================================================================

/// A revolute (hinge) joint allowing rotation about a single axis.
///
/// Revolute joints are the most common type, used for:
/// - Robot arm elbows and shoulders
/// - Door hinges
/// - Wheel axles
///
/// # Constraint Formulation
///
/// The revolute joint constrains:
/// - 3 translational DOF (anchor points must coincide)
/// - 2 rotational DOF (only rotation about axis is free)
///
/// Total: 5 constraints, 1 DOF.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RevoluteJoint {
    /// Parent body.
    parent: BodyId,
    /// Child body.
    child: BodyId,
    /// Rotation axis in parent frame (unit vector).
    axis: Vector3<f64>,
    /// Anchor point in parent body frame.
    parent_anchor: Point3<f64>,
    /// Anchor point in child body frame.
    child_anchor: Point3<f64>,
    /// Position limits.
    limits: Option<JointLimits>,
    /// Motor for active control.
    motor: Option<JointMotor>,
    /// Viscous damping coefficient (Nm·s/rad).
    damping: f64,
    /// Current joint state.
    state: JointState,
}

impl RevoluteJoint {
    /// Create a new revolute joint.
    ///
    /// # Arguments
    ///
    /// * `parent` - Parent body ID
    /// * `child` - Child body ID
    /// * `axis` - Rotation axis in parent frame (will be normalized)
    #[must_use]
    pub fn new(parent: BodyId, child: BodyId, axis: Vector3<f64>) -> Self {
        Self {
            parent,
            child,
            axis: axis.normalize(),
            parent_anchor: Point3::origin(),
            child_anchor: Point3::origin(),
            limits: None,
            motor: None,
            damping: 0.0,
            state: JointState::zero(1),
        }
    }

    /// Set the anchor point in parent frame.
    #[must_use]
    pub fn with_parent_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.parent_anchor = anchor;
        self
    }

    /// Set the anchor point in child frame.
    #[must_use]
    pub fn with_child_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.child_anchor = anchor;
        self
    }

    /// Set both anchors to the same point (in their respective frames).
    #[must_use]
    pub fn with_anchors(mut self, parent: Point3<f64>, child: Point3<f64>) -> Self {
        self.parent_anchor = parent;
        self.child_anchor = child;
        self
    }

    /// Set joint limits.
    #[must_use]
    pub fn with_limits(mut self, limits: JointLimits) -> Self {
        self.limits = Some(limits);
        self
    }

    /// Set the motor.
    #[must_use]
    pub fn with_motor(mut self, motor: JointMotor) -> Self {
        self.motor = Some(motor);
        self
    }

    /// Set the damping coefficient.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping.max(0.0);
        self
    }

    /// Get the rotation axis.
    #[must_use]
    pub fn axis(&self) -> &Vector3<f64> {
        &self.axis
    }

    /// Get the current joint angle.
    #[must_use]
    pub fn angle(&self) -> f64 {
        self.state.q()
    }

    /// Set the current joint angle.
    pub fn set_angle(&mut self, angle: f64) {
        self.state = JointState::single(angle);
    }

    /// Get the joint state.
    #[must_use]
    pub fn state(&self) -> &JointState {
        &self.state
    }

    /// Set the joint state.
    pub fn set_state(&mut self, state: JointState) {
        self.state = state;
    }

    /// Compute the rotation from parent to child frame.
    #[must_use]
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(self.axis), self.angle())
    }

    /// Compute motor and limit forces.
    #[must_use]
    pub fn compute_joint_force(&self, velocity: f64) -> f64 {
        let mut force = 0.0;

        // Motor force
        if let Some(motor) = &self.motor {
            force += motor.compute_force(self.angle(), velocity);
        }

        // Limit force (for soft limits)
        if let Some(limits) = &self.limits {
            force += limits.compute_force(self.angle(), velocity);
        }

        // Damping force
        force -= self.damping * velocity;

        force
    }
}

impl Joint for RevoluteJoint {
    fn parent(&self) -> BodyId {
        self.parent
    }

    fn child(&self) -> BodyId {
        self.child
    }

    fn dof(&self) -> usize {
        1
    }

    fn joint_type(&self) -> JointType {
        JointType::Revolute
    }

    fn limits(&self) -> Option<&JointLimits> {
        self.limits.as_ref()
    }

    fn motor(&self) -> Option<&JointMotor> {
        self.motor.as_ref()
    }

    fn damping(&self) -> f64 {
        self.damping
    }

    fn parent_anchor(&self) -> Point3<f64> {
        self.parent_anchor
    }

    fn child_anchor(&self) -> Point3<f64> {
        self.child_anchor
    }
}

// ============================================================================
// Prismatic Joint
// ============================================================================

/// A prismatic (sliding) joint allowing translation along a single axis.
///
/// Prismatic joints are used for:
/// - Linear actuators
/// - Sliding mechanisms
/// - Telescoping structures
///
/// # Constraint Formulation
///
/// The prismatic joint constrains:
/// - 2 translational DOF (perpendicular to axis)
/// - 3 rotational DOF (no rotation allowed)
///
/// Total: 5 constraints, 1 DOF.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PrismaticJoint {
    /// Parent body.
    parent: BodyId,
    /// Child body.
    child: BodyId,
    /// Sliding axis in parent frame (unit vector).
    axis: Vector3<f64>,
    /// Anchor point in parent body frame.
    parent_anchor: Point3<f64>,
    /// Anchor point in child body frame.
    child_anchor: Point3<f64>,
    /// Position limits.
    limits: Option<JointLimits>,
    /// Motor for active control.
    motor: Option<JointMotor>,
    /// Viscous damping coefficient (N·s/m).
    damping: f64,
    /// Current joint state.
    state: JointState,
}

impl PrismaticJoint {
    /// Create a new prismatic joint.
    #[must_use]
    pub fn new(parent: BodyId, child: BodyId, axis: Vector3<f64>) -> Self {
        Self {
            parent,
            child,
            axis: axis.normalize(),
            parent_anchor: Point3::origin(),
            child_anchor: Point3::origin(),
            limits: None,
            motor: None,
            damping: 0.0,
            state: JointState::zero(1),
        }
    }

    /// Set the anchor point in parent frame.
    #[must_use]
    pub fn with_parent_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.parent_anchor = anchor;
        self
    }

    /// Set the anchor point in child frame.
    #[must_use]
    pub fn with_child_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.child_anchor = anchor;
        self
    }

    /// Set joint limits.
    #[must_use]
    pub fn with_limits(mut self, limits: JointLimits) -> Self {
        self.limits = Some(limits);
        self
    }

    /// Set the motor.
    #[must_use]
    pub fn with_motor(mut self, motor: JointMotor) -> Self {
        self.motor = Some(motor);
        self
    }

    /// Set the damping coefficient.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping.max(0.0);
        self
    }

    /// Get the sliding axis.
    #[must_use]
    pub fn axis(&self) -> &Vector3<f64> {
        &self.axis
    }

    /// Get the current displacement.
    #[must_use]
    pub fn displacement(&self) -> f64 {
        self.state.q()
    }

    /// Set the current displacement.
    pub fn set_displacement(&mut self, displacement: f64) {
        self.state = JointState::single(displacement);
    }

    /// Get the joint state.
    #[must_use]
    pub fn state(&self) -> &JointState {
        &self.state
    }

    /// Compute the translation from parent to child.
    #[must_use]
    pub fn translation(&self) -> Vector3<f64> {
        self.axis * self.displacement()
    }

    /// Compute motor and limit forces.
    #[must_use]
    pub fn compute_joint_force(&self, velocity: f64) -> f64 {
        let mut force = 0.0;

        if let Some(motor) = &self.motor {
            force += motor.compute_force(self.displacement(), velocity);
        }

        if let Some(limits) = &self.limits {
            force += limits.compute_force(self.displacement(), velocity);
        }

        force -= self.damping * velocity;

        force
    }
}

impl Joint for PrismaticJoint {
    fn parent(&self) -> BodyId {
        self.parent
    }

    fn child(&self) -> BodyId {
        self.child
    }

    fn dof(&self) -> usize {
        1
    }

    fn joint_type(&self) -> JointType {
        JointType::Prismatic
    }

    fn limits(&self) -> Option<&JointLimits> {
        self.limits.as_ref()
    }

    fn motor(&self) -> Option<&JointMotor> {
        self.motor.as_ref()
    }

    fn damping(&self) -> f64 {
        self.damping
    }

    fn parent_anchor(&self) -> Point3<f64> {
        self.parent_anchor
    }

    fn child_anchor(&self) -> Point3<f64> {
        self.child_anchor
    }
}

// ============================================================================
// Fixed Joint
// ============================================================================

/// A fixed joint (weld) with no degrees of freedom.
///
/// Fixed joints rigidly connect two bodies, useful for:
/// - Attaching fixtures
/// - Creating compound shapes
/// - Breakable connections (when combined with force thresholds)
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FixedJoint {
    /// Parent body.
    parent: BodyId,
    /// Child body.
    child: BodyId,
    /// Anchor point in parent body frame.
    parent_anchor: Point3<f64>,
    /// Anchor point in child body frame.
    child_anchor: Point3<f64>,
    /// Relative orientation from parent to child.
    relative_rotation: UnitQuaternion<f64>,
}

impl FixedJoint {
    /// Create a new fixed joint.
    #[must_use]
    pub fn new(parent: BodyId, child: BodyId) -> Self {
        Self {
            parent,
            child,
            parent_anchor: Point3::origin(),
            child_anchor: Point3::origin(),
            relative_rotation: UnitQuaternion::identity(),
        }
    }

    /// Set the anchor point in parent frame.
    #[must_use]
    pub fn with_parent_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.parent_anchor = anchor;
        self
    }

    /// Set the anchor point in child frame.
    #[must_use]
    pub fn with_child_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.child_anchor = anchor;
        self
    }

    /// Set the relative rotation from parent to child.
    #[must_use]
    pub fn with_relative_rotation(mut self, rotation: UnitQuaternion<f64>) -> Self {
        self.relative_rotation = rotation;
        self
    }

    /// Get the relative rotation.
    #[must_use]
    pub fn relative_rotation(&self) -> &UnitQuaternion<f64> {
        &self.relative_rotation
    }
}

impl Joint for FixedJoint {
    fn parent(&self) -> BodyId {
        self.parent
    }

    fn child(&self) -> BodyId {
        self.child
    }

    fn dof(&self) -> usize {
        0
    }

    fn joint_type(&self) -> JointType {
        JointType::Fixed
    }

    fn limits(&self) -> Option<&JointLimits> {
        None
    }

    fn motor(&self) -> Option<&JointMotor> {
        None
    }

    fn damping(&self) -> f64 {
        0.0
    }

    fn parent_anchor(&self) -> Point3<f64> {
        self.parent_anchor
    }

    fn child_anchor(&self) -> Point3<f64> {
        self.child_anchor
    }
}

// ============================================================================
// Spherical Joint
// ============================================================================

/// A spherical (ball-and-socket) joint with 3 rotational DOF.
///
/// Spherical joints are used for:
/// - Hip joints
/// - Shoulder joints
/// - Gimbal mounts
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SphericalJoint {
    /// Parent body.
    parent: BodyId,
    /// Child body.
    child: BodyId,
    /// Anchor point in parent body frame.
    parent_anchor: Point3<f64>,
    /// Anchor point in child body frame.
    child_anchor: Point3<f64>,
    /// Angular damping coefficient.
    damping: f64,
    /// Current orientation (as quaternion, but only 3 DOF).
    rotation: UnitQuaternion<f64>,
}

impl SphericalJoint {
    /// Create a new spherical joint.
    #[must_use]
    pub fn new(parent: BodyId, child: BodyId) -> Self {
        Self {
            parent,
            child,
            parent_anchor: Point3::origin(),
            child_anchor: Point3::origin(),
            damping: 0.0,
            rotation: UnitQuaternion::identity(),
        }
    }

    /// Set the anchor point in parent frame.
    #[must_use]
    pub fn with_parent_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.parent_anchor = anchor;
        self
    }

    /// Set the anchor point in child frame.
    #[must_use]
    pub fn with_child_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.child_anchor = anchor;
        self
    }

    /// Set the damping coefficient.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping.max(0.0);
        self
    }

    /// Get the current rotation.
    #[must_use]
    pub fn rotation(&self) -> &UnitQuaternion<f64> {
        &self.rotation
    }

    /// Set the rotation.
    pub fn set_rotation(&mut self, rotation: UnitQuaternion<f64>) {
        self.rotation = rotation;
    }
}

impl Joint for SphericalJoint {
    fn parent(&self) -> BodyId {
        self.parent
    }

    fn child(&self) -> BodyId {
        self.child
    }

    fn dof(&self) -> usize {
        3
    }

    fn joint_type(&self) -> JointType {
        JointType::Spherical
    }

    fn limits(&self) -> Option<&JointLimits> {
        None // Spherical joints typically use cone limits, not 1D limits
    }

    fn motor(&self) -> Option<&JointMotor> {
        None // Would need 3D motor
    }

    fn damping(&self) -> f64 {
        self.damping
    }

    fn parent_anchor(&self) -> Point3<f64> {
        self.parent_anchor
    }

    fn child_anchor(&self) -> Point3<f64> {
        self.child_anchor
    }
}

// ============================================================================
// Universal Joint
// ============================================================================

/// A universal joint with 2 perpendicular rotation axes.
///
/// Universal joints are used for:
/// - Cardan shafts
/// - Wrist joints
/// - Steering mechanisms
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct UniversalJoint {
    /// Parent body.
    parent: BodyId,
    /// Child body.
    child: BodyId,
    /// First rotation axis (in parent frame).
    axis1: Vector3<f64>,
    /// Second rotation axis (in child frame, perpendicular to axis1).
    axis2: Vector3<f64>,
    /// Anchor point in parent body frame.
    parent_anchor: Point3<f64>,
    /// Anchor point in child body frame.
    child_anchor: Point3<f64>,
    /// Damping coefficient.
    damping: f64,
    /// Current angles [angle1, angle2].
    angles: [f64; 2],
}

impl UniversalJoint {
    /// Create a new universal joint with default axes (X and Y).
    #[must_use]
    pub fn new(parent: BodyId, child: BodyId) -> Self {
        Self {
            parent,
            child,
            axis1: Vector3::x(),
            axis2: Vector3::y(),
            parent_anchor: Point3::origin(),
            child_anchor: Point3::origin(),
            damping: 0.0,
            angles: [0.0, 0.0],
        }
    }

    /// Set the rotation axes.
    #[must_use]
    pub fn with_axes(mut self, axis1: Vector3<f64>, axis2: Vector3<f64>) -> Self {
        self.axis1 = axis1.normalize();
        self.axis2 = axis2.normalize();
        self
    }

    /// Set the anchor point in parent frame.
    #[must_use]
    pub fn with_parent_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.parent_anchor = anchor;
        self
    }

    /// Set the anchor point in child frame.
    #[must_use]
    pub fn with_child_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.child_anchor = anchor;
        self
    }

    /// Set the damping coefficient.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping.max(0.0);
        self
    }

    /// Get the first axis.
    #[must_use]
    pub fn axis1(&self) -> &Vector3<f64> {
        &self.axis1
    }

    /// Get the second axis.
    #[must_use]
    pub fn axis2(&self) -> &Vector3<f64> {
        &self.axis2
    }

    /// Get the current angles.
    #[must_use]
    pub fn angles(&self) -> &[f64; 2] {
        &self.angles
    }

    /// Set the angles.
    pub fn set_angles(&mut self, angle1: f64, angle2: f64) {
        self.angles = [angle1, angle2];
    }
}

impl Joint for UniversalJoint {
    fn parent(&self) -> BodyId {
        self.parent
    }

    fn child(&self) -> BodyId {
        self.child
    }

    fn dof(&self) -> usize {
        2
    }

    fn joint_type(&self) -> JointType {
        JointType::Universal
    }

    fn limits(&self) -> Option<&JointLimits> {
        None // Would need 2D limits
    }

    fn motor(&self) -> Option<&JointMotor> {
        None // Would need 2D motor
    }

    fn damping(&self) -> f64 {
        self.damping
    }

    fn parent_anchor(&self) -> Point3<f64> {
        self.parent_anchor
    }

    fn child_anchor(&self) -> Point3<f64> {
        self.child_anchor
    }
}

// ============================================================================
// Free Joint
// ============================================================================

/// A free joint (floating base) with 6 degrees of freedom.
///
/// Free joints allow complete freedom of motion - 3 translational and 3 rotational DOF.
/// They are used for:
/// - Floating-base robots (quadrupeds, humanoids, drones)
/// - Unattached objects
/// - Root bodies in kinematic trees
///
/// # Constraint Formulation
///
/// Free joints have **zero constraints** - they don't constrain any DOF.
/// The child body can move and rotate freely relative to the parent.
/// However, they still participate in the constraint system for mass matrix handling.
///
/// Total: 0 constraints, 6 DOF.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FreeJoint {
    /// Parent body (typically world/ground).
    parent: BodyId,
    /// Child body (the floating base).
    child: BodyId,
    /// Anchor point in parent body frame.
    parent_anchor: Point3<f64>,
    /// Anchor point in child body frame.
    child_anchor: Point3<f64>,
    /// Linear damping coefficient (N·s/m).
    linear_damping: f64,
    /// Angular damping coefficient (Nm·s/rad).
    angular_damping: f64,
    /// Current position (relative to parent anchor).
    position: Vector3<f64>,
    /// Current orientation (relative to parent).
    rotation: UnitQuaternion<f64>,
}

impl FreeJoint {
    /// Create a new free joint.
    ///
    /// # Arguments
    ///
    /// * `parent` - Parent body ID (typically world/ground)
    /// * `child` - Child body ID (the floating base)
    #[must_use]
    pub fn new(parent: BodyId, child: BodyId) -> Self {
        Self {
            parent,
            child,
            parent_anchor: Point3::origin(),
            child_anchor: Point3::origin(),
            linear_damping: 0.0,
            angular_damping: 0.0,
            position: Vector3::zeros(),
            rotation: UnitQuaternion::identity(),
        }
    }

    /// Set the anchor point in parent frame.
    #[must_use]
    pub fn with_parent_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.parent_anchor = anchor;
        self
    }

    /// Set the anchor point in child frame.
    #[must_use]
    pub fn with_child_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.child_anchor = anchor;
        self
    }

    /// Set linear damping coefficient.
    #[must_use]
    pub fn with_linear_damping(mut self, damping: f64) -> Self {
        self.linear_damping = damping.max(0.0);
        self
    }

    /// Set angular damping coefficient.
    #[must_use]
    pub fn with_angular_damping(mut self, damping: f64) -> Self {
        self.angular_damping = damping.max(0.0);
        self
    }

    /// Set both linear and angular damping.
    #[must_use]
    pub fn with_damping(mut self, linear: f64, angular: f64) -> Self {
        self.linear_damping = linear.max(0.0);
        self.angular_damping = angular.max(0.0);
        self
    }

    /// Get the current position.
    #[must_use]
    pub fn position(&self) -> &Vector3<f64> {
        &self.position
    }

    /// Set the position.
    pub fn set_position(&mut self, position: Vector3<f64>) {
        self.position = position;
    }

    /// Get the current rotation.
    #[must_use]
    pub fn rotation(&self) -> &UnitQuaternion<f64> {
        &self.rotation
    }

    /// Set the rotation.
    pub fn set_rotation(&mut self, rotation: UnitQuaternion<f64>) {
        self.rotation = rotation;
    }

    /// Get linear damping coefficient.
    #[must_use]
    pub fn linear_damping(&self) -> f64 {
        self.linear_damping
    }

    /// Get angular damping coefficient.
    #[must_use]
    pub fn angular_damping(&self) -> f64 {
        self.angular_damping
    }

    /// Compute damping forces for given velocities.
    #[must_use]
    pub fn compute_damping_force(
        &self,
        linear_velocity: &Vector3<f64>,
        angular_velocity: &Vector3<f64>,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let linear_force = -self.linear_damping * linear_velocity;
        let angular_torque = -self.angular_damping * angular_velocity;
        (linear_force, angular_torque)
    }
}

impl Joint for FreeJoint {
    fn parent(&self) -> BodyId {
        self.parent
    }

    fn child(&self) -> BodyId {
        self.child
    }

    fn dof(&self) -> usize {
        6
    }

    fn joint_type(&self) -> JointType {
        JointType::Free
    }

    fn limits(&self) -> Option<&JointLimits> {
        None // Free joints have no limits
    }

    fn motor(&self) -> Option<&JointMotor> {
        None // Free joints have no motors (would need 6D motor)
    }

    fn damping(&self) -> f64 {
        // Return average of linear and angular damping
        f64::midpoint(self.linear_damping, self.angular_damping)
    }

    fn parent_anchor(&self) -> Point3<f64> {
        self.parent_anchor
    }

    fn child_anchor(&self) -> Point3<f64> {
        self.child_anchor
    }
}

// ============================================================================
// Planar Joint
// ============================================================================

/// A planar joint allowing motion in a plane (3 DOF).
///
/// Planar joints allow translation in two dimensions plus rotation about the
/// plane normal. They are used for:
/// - Mobile robots on flat surfaces
/// - 2D simulation scenarios
/// - Constrained XY motion
///
/// # Constraint Formulation
///
/// The planar joint constrains:
/// - 1 translational DOF (perpendicular to plane)
/// - 2 rotational DOF (tilt about in-plane axes)
///
/// Total: 3 constraints, 3 DOF (x, y translation + rotation about normal).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PlanarJoint {
    /// Parent body.
    parent: BodyId,
    /// Child body.
    child: BodyId,
    /// Plane normal in parent frame (defines the rotation axis and constrained translation).
    normal: Vector3<f64>,
    /// Anchor point in parent body frame.
    parent_anchor: Point3<f64>,
    /// Anchor point in child body frame.
    child_anchor: Point3<f64>,
    /// Damping coefficient for in-plane translation.
    translation_damping: f64,
    /// Damping coefficient for rotation about normal.
    rotation_damping: f64,
    /// Current in-plane position [x, y] in local frame.
    position: [f64; 2],
    /// Current rotation angle about normal.
    angle: f64,
}

impl PlanarJoint {
    /// Create a new planar joint with default XY plane (Z normal).
    #[must_use]
    pub fn new(parent: BodyId, child: BodyId) -> Self {
        Self {
            parent,
            child,
            normal: Vector3::z(),
            parent_anchor: Point3::origin(),
            child_anchor: Point3::origin(),
            translation_damping: 0.0,
            rotation_damping: 0.0,
            position: [0.0, 0.0],
            angle: 0.0,
        }
    }

    /// Create a planar joint with specified normal.
    #[must_use]
    pub fn with_normal(parent: BodyId, child: BodyId, normal: Vector3<f64>) -> Self {
        Self {
            parent,
            child,
            normal: normal.normalize(),
            parent_anchor: Point3::origin(),
            child_anchor: Point3::origin(),
            translation_damping: 0.0,
            rotation_damping: 0.0,
            position: [0.0, 0.0],
            angle: 0.0,
        }
    }

    /// Set the anchor point in parent frame.
    #[must_use]
    pub fn with_parent_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.parent_anchor = anchor;
        self
    }

    /// Set the anchor point in child frame.
    #[must_use]
    pub fn with_child_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.child_anchor = anchor;
        self
    }

    /// Set translation damping coefficient.
    #[must_use]
    pub fn with_translation_damping(mut self, damping: f64) -> Self {
        self.translation_damping = damping.max(0.0);
        self
    }

    /// Set rotation damping coefficient.
    #[must_use]
    pub fn with_rotation_damping(mut self, damping: f64) -> Self {
        self.rotation_damping = damping.max(0.0);
        self
    }

    /// Set both damping coefficients.
    #[must_use]
    pub fn with_damping(mut self, translation: f64, rotation: f64) -> Self {
        self.translation_damping = translation.max(0.0);
        self.rotation_damping = rotation.max(0.0);
        self
    }

    /// Get the plane normal.
    #[must_use]
    pub fn normal(&self) -> &Vector3<f64> {
        &self.normal
    }

    /// Get the current in-plane position.
    #[must_use]
    pub fn position(&self) -> &[f64; 2] {
        &self.position
    }

    /// Set the in-plane position.
    pub fn set_position(&mut self, x: f64, y: f64) {
        self.position = [x, y];
    }

    /// Get the current rotation angle.
    #[must_use]
    pub fn angle(&self) -> f64 {
        self.angle
    }

    /// Set the rotation angle.
    pub fn set_angle(&mut self, angle: f64) {
        self.angle = angle;
    }

    /// Get translation damping.
    #[must_use]
    pub fn translation_damping(&self) -> f64 {
        self.translation_damping
    }

    /// Get rotation damping.
    #[must_use]
    pub fn rotation_damping(&self) -> f64 {
        self.rotation_damping
    }

    /// Compute the local coordinate frame basis vectors.
    /// Returns (`x_axis`, `y_axis`) in world frame.
    #[must_use]
    pub fn compute_basis(&self) -> (Vector3<f64>, Vector3<f64>) {
        // Find perpendicular vectors in the plane
        let arbitrary = if self.normal.x.abs() < 0.9 {
            Vector3::x()
        } else {
            Vector3::y()
        };
        let x_axis = self.normal.cross(&arbitrary).normalize();
        let y_axis = self.normal.cross(&x_axis);
        (x_axis, y_axis)
    }

    /// Compute the translation vector in world frame.
    #[must_use]
    pub fn translation(&self) -> Vector3<f64> {
        let (x_axis, y_axis) = self.compute_basis();
        x_axis * self.position[0] + y_axis * self.position[1]
    }

    /// Compute the rotation quaternion about the normal.
    #[must_use]
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(self.normal), self.angle)
    }
}

impl Joint for PlanarJoint {
    fn parent(&self) -> BodyId {
        self.parent
    }

    fn child(&self) -> BodyId {
        self.child
    }

    fn dof(&self) -> usize {
        3
    }

    fn joint_type(&self) -> JointType {
        JointType::Planar
    }

    fn limits(&self) -> Option<&JointLimits> {
        None // Would need 3D limits (2D position + angle)
    }

    fn motor(&self) -> Option<&JointMotor> {
        None // Would need 3D motor
    }

    fn damping(&self) -> f64 {
        f64::midpoint(self.translation_damping, self.rotation_damping)
    }

    fn parent_anchor(&self) -> Point3<f64> {
        self.parent_anchor
    }

    fn child_anchor(&self) -> Point3<f64> {
        self.child_anchor
    }
}

// ============================================================================
// Cylindrical Joint
// ============================================================================

/// A cylindrical joint allowing rotation and translation along the same axis (2 DOF).
///
/// Cylindrical joints are a combination of revolute and prismatic joints
/// sharing the same axis. They are used for:
/// - Screw mechanisms
/// - Telescoping rotary shafts
/// - Leadscrew-nut assemblies
///
/// # Constraint Formulation
///
/// The cylindrical joint constrains:
/// - 2 translational DOF (perpendicular to axis)
/// - 2 rotational DOF (perpendicular to axis)
///
/// Total: 4 constraints, 2 DOF (rotation + translation along axis).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CylindricalJoint {
    /// Parent body.
    parent: BodyId,
    /// Child body.
    child: BodyId,
    /// Joint axis in parent frame (unit vector).
    axis: Vector3<f64>,
    /// Anchor point in parent body frame.
    parent_anchor: Point3<f64>,
    /// Anchor point in child body frame.
    child_anchor: Point3<f64>,
    /// Position limits for rotation.
    rotation_limits: Option<JointLimits>,
    /// Position limits for translation.
    translation_limits: Option<JointLimits>,
    /// Motor for rotation control.
    rotation_motor: Option<JointMotor>,
    /// Motor for translation control.
    translation_motor: Option<JointMotor>,
    /// Damping coefficient for rotation (Nm·s/rad).
    rotation_damping: f64,
    /// Damping coefficient for translation (N·s/m).
    translation_damping: f64,
    /// Current rotation angle.
    angle: f64,
    /// Current displacement along axis.
    displacement: f64,
}

impl CylindricalJoint {
    /// Create a new cylindrical joint.
    ///
    /// # Arguments
    ///
    /// * `parent` - Parent body ID
    /// * `child` - Child body ID
    /// * `axis` - Joint axis in parent frame (will be normalized)
    #[must_use]
    pub fn new(parent: BodyId, child: BodyId, axis: Vector3<f64>) -> Self {
        Self {
            parent,
            child,
            axis: axis.normalize(),
            parent_anchor: Point3::origin(),
            child_anchor: Point3::origin(),
            rotation_limits: None,
            translation_limits: None,
            rotation_motor: None,
            translation_motor: None,
            rotation_damping: 0.0,
            translation_damping: 0.0,
            angle: 0.0,
            displacement: 0.0,
        }
    }

    /// Set the anchor point in parent frame.
    #[must_use]
    pub fn with_parent_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.parent_anchor = anchor;
        self
    }

    /// Set the anchor point in child frame.
    #[must_use]
    pub fn with_child_anchor(mut self, anchor: Point3<f64>) -> Self {
        self.child_anchor = anchor;
        self
    }

    /// Set rotation limits.
    #[must_use]
    pub fn with_rotation_limits(mut self, limits: JointLimits) -> Self {
        self.rotation_limits = Some(limits);
        self
    }

    /// Set translation limits.
    #[must_use]
    pub fn with_translation_limits(mut self, limits: JointLimits) -> Self {
        self.translation_limits = Some(limits);
        self
    }

    /// Set rotation motor.
    #[must_use]
    pub fn with_rotation_motor(mut self, motor: JointMotor) -> Self {
        self.rotation_motor = Some(motor);
        self
    }

    /// Set translation motor.
    #[must_use]
    pub fn with_translation_motor(mut self, motor: JointMotor) -> Self {
        self.translation_motor = Some(motor);
        self
    }

    /// Set rotation damping coefficient.
    #[must_use]
    pub fn with_rotation_damping(mut self, damping: f64) -> Self {
        self.rotation_damping = damping.max(0.0);
        self
    }

    /// Set translation damping coefficient.
    #[must_use]
    pub fn with_translation_damping(mut self, damping: f64) -> Self {
        self.translation_damping = damping.max(0.0);
        self
    }

    /// Set both damping coefficients.
    #[must_use]
    pub fn with_damping(mut self, rotation: f64, translation: f64) -> Self {
        self.rotation_damping = rotation.max(0.0);
        self.translation_damping = translation.max(0.0);
        self
    }

    /// Get the joint axis.
    #[must_use]
    pub fn axis(&self) -> &Vector3<f64> {
        &self.axis
    }

    /// Get the current rotation angle.
    #[must_use]
    pub fn angle(&self) -> f64 {
        self.angle
    }

    /// Set the rotation angle.
    pub fn set_angle(&mut self, angle: f64) {
        self.angle = angle;
    }

    /// Get the current displacement.
    #[must_use]
    pub fn displacement(&self) -> f64 {
        self.displacement
    }

    /// Set the displacement.
    pub fn set_displacement(&mut self, displacement: f64) {
        self.displacement = displacement;
    }

    /// Set both angle and displacement.
    pub fn set_state(&mut self, angle: f64, displacement: f64) {
        self.angle = angle;
        self.displacement = displacement;
    }

    /// Get rotation limits.
    #[must_use]
    pub fn rotation_limits(&self) -> Option<&JointLimits> {
        self.rotation_limits.as_ref()
    }

    /// Get translation limits.
    #[must_use]
    pub fn translation_limits(&self) -> Option<&JointLimits> {
        self.translation_limits.as_ref()
    }

    /// Get rotation motor.
    #[must_use]
    pub fn rotation_motor(&self) -> Option<&JointMotor> {
        self.rotation_motor.as_ref()
    }

    /// Get translation motor.
    #[must_use]
    pub fn translation_motor(&self) -> Option<&JointMotor> {
        self.translation_motor.as_ref()
    }

    /// Get rotation damping.
    #[must_use]
    pub fn rotation_damping(&self) -> f64 {
        self.rotation_damping
    }

    /// Get translation damping.
    #[must_use]
    pub fn translation_damping(&self) -> f64 {
        self.translation_damping
    }

    /// Compute the rotation from parent to child.
    #[must_use]
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(self.axis), self.angle)
    }

    /// Compute the translation from parent to child.
    #[must_use]
    pub fn translation(&self) -> Vector3<f64> {
        self.axis * self.displacement
    }

    /// Compute rotation and translation motor/limit forces.
    #[must_use]
    pub fn compute_joint_forces(&self, angular_velocity: f64, linear_velocity: f64) -> (f64, f64) {
        let mut rotation_force = 0.0;
        let mut translation_force = 0.0;

        // Rotation motor force
        if let Some(motor) = &self.rotation_motor {
            rotation_force += motor.compute_force(self.angle, angular_velocity);
        }

        // Rotation limit force
        if let Some(limits) = &self.rotation_limits {
            rotation_force += limits.compute_force(self.angle, angular_velocity);
        }

        // Rotation damping
        rotation_force -= self.rotation_damping * angular_velocity;

        // Translation motor force
        if let Some(motor) = &self.translation_motor {
            translation_force += motor.compute_force(self.displacement, linear_velocity);
        }

        // Translation limit force
        if let Some(limits) = &self.translation_limits {
            translation_force += limits.compute_force(self.displacement, linear_velocity);
        }

        // Translation damping
        translation_force -= self.translation_damping * linear_velocity;

        (rotation_force, translation_force)
    }
}

impl Joint for CylindricalJoint {
    fn parent(&self) -> BodyId {
        self.parent
    }

    fn child(&self) -> BodyId {
        self.child
    }

    fn dof(&self) -> usize {
        2
    }

    fn joint_type(&self) -> JointType {
        JointType::Cylindrical
    }

    fn limits(&self) -> Option<&JointLimits> {
        // Return rotation limits as primary (for compatibility with 1D limit interface)
        self.rotation_limits.as_ref()
    }

    fn motor(&self) -> Option<&JointMotor> {
        // Return rotation motor as primary (for compatibility with 1D motor interface)
        self.rotation_motor.as_ref()
    }

    fn damping(&self) -> f64 {
        f64::midpoint(self.rotation_damping, self.translation_damping)
    }

    fn parent_anchor(&self) -> Point3<f64> {
        self.parent_anchor
    }

    fn child_anchor(&self) -> Point3<f64> {
        self.child_anchor
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_revolute_joint() {
        let joint = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());

        assert_eq!(joint.dof(), 1);
        assert_eq!(joint.joint_type(), JointType::Revolute);
        assert_relative_eq!(joint.angle(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_revolute_rotation() {
        let mut joint = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());
        joint.set_angle(std::f64::consts::FRAC_PI_2);

        let rot = joint.rotation();
        let rotated = rot * Vector3::x();

        // 90 degree rotation around Z should map X to Y
        assert_relative_eq!(rotated.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(rotated.y, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_prismatic_joint() {
        let joint = PrismaticJoint::new(BodyId::new(0), BodyId::new(1), Vector3::x());

        assert_eq!(joint.dof(), 1);
        assert_eq!(joint.joint_type(), JointType::Prismatic);
    }

    #[test]
    fn test_prismatic_translation() {
        let mut joint = PrismaticJoint::new(BodyId::new(0), BodyId::new(1), Vector3::x());
        joint.set_displacement(0.5);

        let trans = joint.translation();
        assert_relative_eq!(trans.x, 0.5, epsilon = 1e-10);
        assert_relative_eq!(trans.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(trans.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_fixed_joint() {
        let joint = FixedJoint::new(BodyId::new(0), BodyId::new(1));

        assert_eq!(joint.dof(), 0);
        assert_eq!(joint.joint_type(), JointType::Fixed);
    }

    #[test]
    fn test_spherical_joint() {
        let joint = SphericalJoint::new(BodyId::new(0), BodyId::new(1));

        assert_eq!(joint.dof(), 3);
        assert_eq!(joint.joint_type(), JointType::Spherical);
    }

    #[test]
    fn test_universal_joint() {
        let joint = UniversalJoint::new(BodyId::new(0), BodyId::new(1));

        assert_eq!(joint.dof(), 2);
        assert_eq!(joint.joint_type(), JointType::Universal);
    }

    #[test]
    fn test_joint_with_limits_and_motor() {
        let joint = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z())
            .with_limits(JointLimits::symmetric(1.0))
            .with_motor(JointMotor::velocity(1.0, 10.0))
            .with_damping(0.5);

        assert!(joint.limits().is_some());
        assert!(joint.motor().is_some());
        assert_relative_eq!(joint.damping(), 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_revolute_joint_force() {
        let joint = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z())
            .with_motor(JointMotor::velocity(1.0, 10.0))
            .with_damping(1.0);

        // Moving slower than target, positive motor force minus damping
        let force = joint.compute_joint_force(0.5);
        // Motor gives max force (10), damping gives -0.5
        // Net should be positive
        assert!(force > 0.0);
    }

    // ========================================================================
    // Free Joint Tests
    // ========================================================================

    #[test]
    fn test_free_joint_creation() {
        let joint = FreeJoint::new(BodyId::new(0), BodyId::new(1));

        assert_eq!(joint.parent(), BodyId::new(0));
        assert_eq!(joint.child(), BodyId::new(1));
        assert_eq!(joint.dof(), 6);
        assert_eq!(joint.joint_type(), JointType::Free);
    }

    #[test]
    fn test_free_joint_has_no_constraints() {
        let joint_type = JointType::Free;
        assert_eq!(joint_type.dof(), 6);
        assert_eq!(joint_type.constrained_dof(), 0);
    }

    #[test]
    fn test_free_joint_position_and_rotation() {
        let mut joint = FreeJoint::new(BodyId::new(0), BodyId::new(1));

        joint.set_position(Vector3::new(1.0, 2.0, 3.0));
        assert_relative_eq!(joint.position().x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(joint.position().y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(joint.position().z, 3.0, epsilon = 1e-10);

        let quat = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::z()),
            std::f64::consts::FRAC_PI_2,
        );
        joint.set_rotation(quat);
        assert_relative_eq!(
            joint.rotation().angle(),
            std::f64::consts::FRAC_PI_2,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_free_joint_damping() {
        let joint = FreeJoint::new(BodyId::new(0), BodyId::new(1)).with_damping(1.0, 2.0);

        assert_relative_eq!(joint.linear_damping(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(joint.angular_damping(), 2.0, epsilon = 1e-10);

        let (linear, angular) =
            joint.compute_damping_force(&Vector3::new(1.0, 0.0, 0.0), &Vector3::new(0.0, 1.0, 0.0));
        assert_relative_eq!(linear.x, -1.0, epsilon = 1e-10);
        assert_relative_eq!(angular.y, -2.0, epsilon = 1e-10);
    }

    // ========================================================================
    // Planar Joint Tests
    // ========================================================================

    #[test]
    fn test_planar_joint_creation() {
        let joint = PlanarJoint::new(BodyId::new(0), BodyId::new(1));

        assert_eq!(joint.parent(), BodyId::new(0));
        assert_eq!(joint.child(), BodyId::new(1));
        assert_eq!(joint.dof(), 3);
        assert_eq!(joint.joint_type(), JointType::Planar);
    }

    #[test]
    fn test_planar_joint_constraints() {
        let joint_type = JointType::Planar;
        assert_eq!(joint_type.dof(), 3);
        assert_eq!(joint_type.constrained_dof(), 3);
    }

    #[test]
    fn test_planar_joint_custom_normal() {
        let joint = PlanarJoint::with_normal(
            BodyId::new(0),
            BodyId::new(1),
            Vector3::new(0.0, 1.0, 0.0), // XZ plane
        );

        assert_relative_eq!(joint.normal().y, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_planar_joint_position_and_angle() {
        let mut joint = PlanarJoint::new(BodyId::new(0), BodyId::new(1));

        joint.set_position(1.0, 2.0);
        assert_relative_eq!(joint.position()[0], 1.0, epsilon = 1e-10);
        assert_relative_eq!(joint.position()[1], 2.0, epsilon = 1e-10);

        joint.set_angle(std::f64::consts::FRAC_PI_4);
        assert_relative_eq!(joint.angle(), std::f64::consts::FRAC_PI_4, epsilon = 1e-10);
    }

    #[test]
    fn test_planar_joint_translation() {
        let mut joint = PlanarJoint::new(BodyId::new(0), BodyId::new(1));
        joint.set_position(3.0, 4.0);

        let trans = joint.translation();
        // Default XY plane: position [3, 4] maps to x=3, y=4
        assert_relative_eq!(trans.norm(), 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_planar_joint_rotation() {
        let mut joint = PlanarJoint::new(BodyId::new(0), BodyId::new(1));
        joint.set_angle(std::f64::consts::FRAC_PI_2);

        let rot = joint.rotation();
        let rotated = rot * Vector3::x();

        // 90 degree rotation around Z should map X to Y
        assert_relative_eq!(rotated.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(rotated.y, 1.0, epsilon = 1e-10);
    }

    // ========================================================================
    // Cylindrical Joint Tests
    // ========================================================================

    #[test]
    fn test_cylindrical_joint_creation() {
        let joint = CylindricalJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());

        assert_eq!(joint.parent(), BodyId::new(0));
        assert_eq!(joint.child(), BodyId::new(1));
        assert_eq!(joint.dof(), 2);
        assert_eq!(joint.joint_type(), JointType::Cylindrical);
    }

    #[test]
    fn test_cylindrical_joint_constraints() {
        let joint_type = JointType::Cylindrical;
        assert_eq!(joint_type.dof(), 2);
        assert_eq!(joint_type.constrained_dof(), 4);
    }

    #[test]
    fn test_cylindrical_joint_axis() {
        let joint =
            CylindricalJoint::new(BodyId::new(0), BodyId::new(1), Vector3::new(1.0, 0.0, 0.0));

        assert_relative_eq!(joint.axis().x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(joint.axis().y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(joint.axis().z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_cylindrical_joint_state() {
        let mut joint = CylindricalJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());

        joint.set_angle(std::f64::consts::FRAC_PI_4);
        joint.set_displacement(0.5);

        assert_relative_eq!(joint.angle(), std::f64::consts::FRAC_PI_4, epsilon = 1e-10);
        assert_relative_eq!(joint.displacement(), 0.5, epsilon = 1e-10);

        joint.set_state(1.0, 2.0);
        assert_relative_eq!(joint.angle(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(joint.displacement(), 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_cylindrical_joint_rotation() {
        let mut joint = CylindricalJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());
        joint.set_angle(std::f64::consts::FRAC_PI_2);

        let rot = joint.rotation();
        let rotated = rot * Vector3::x();

        // 90 degree rotation around Z should map X to Y
        assert_relative_eq!(rotated.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(rotated.y, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_cylindrical_joint_translation() {
        let mut joint = CylindricalJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());
        joint.set_displacement(0.5);

        let trans = joint.translation();
        assert_relative_eq!(trans.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(trans.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(trans.z, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_cylindrical_joint_with_limits_and_motors() {
        let joint = CylindricalJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z())
            .with_rotation_limits(JointLimits::symmetric(1.0))
            .with_translation_limits(JointLimits::new(-0.5, 0.5))
            .with_rotation_motor(JointMotor::velocity(1.0, 10.0))
            .with_translation_motor(JointMotor::velocity(0.5, 5.0))
            .with_damping(0.1, 0.2);

        assert!(joint.rotation_limits().is_some());
        assert!(joint.translation_limits().is_some());
        assert!(joint.rotation_motor().is_some());
        assert!(joint.translation_motor().is_some());
        assert_relative_eq!(joint.rotation_damping(), 0.1, epsilon = 1e-10);
        assert_relative_eq!(joint.translation_damping(), 0.2, epsilon = 1e-10);
    }

    #[test]
    fn test_cylindrical_joint_force() {
        let joint = CylindricalJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z())
            .with_rotation_motor(JointMotor::velocity(1.0, 10.0))
            .with_translation_motor(JointMotor::velocity(0.5, 5.0))
            .with_damping(0.1, 0.1);

        let (rot_force, trans_force) = joint.compute_joint_forces(0.5, 0.25);

        // Both forces should be positive (trying to reach target velocity)
        assert!(rot_force > 0.0);
        assert!(trans_force > 0.0);
    }

    // ========================================================================
    // Joint Type Property Tests
    // ========================================================================

    #[test]
    fn test_joint_type_has_rotation() {
        assert!(JointType::Revolute.has_rotation());
        assert!(!JointType::Prismatic.has_rotation());
        assert!(JointType::Spherical.has_rotation());
        assert!(JointType::Cylindrical.has_rotation());
        assert!(JointType::Planar.has_rotation());
        assert!(JointType::Free.has_rotation());
        assert!(!JointType::Fixed.has_rotation());
    }

    #[test]
    fn test_joint_type_has_translation() {
        assert!(!JointType::Revolute.has_translation());
        assert!(JointType::Prismatic.has_translation());
        assert!(!JointType::Spherical.has_translation());
        assert!(JointType::Cylindrical.has_translation());
        assert!(JointType::Planar.has_translation());
        assert!(JointType::Free.has_translation());
        assert!(!JointType::Fixed.has_translation());
    }
}
