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
    /// Spherical joint (3 DOF) - ball and socket.
    Spherical,
    /// Universal joint (2 DOF) - two perpendicular rotation axes.
    Universal,
}

impl JointType {
    /// Get the number of degrees of freedom for this joint type.
    #[must_use]
    pub fn dof(&self) -> usize {
        match self {
            Self::Fixed => 0,
            Self::Revolute | Self::Prismatic => 1,
            Self::Universal => 2,
            Self::Spherical => 3,
        }
    }

    /// Get the number of constrained DOF (6 - dof).
    #[must_use]
    pub fn constrained_dof(&self) -> usize {
        6 - self.dof()
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
}
