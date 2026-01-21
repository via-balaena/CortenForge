//! Fixed tendons for joint coupling (MuJoCo-style).
//!
//! Fixed tendons create linear relationships between multiple joints,
//! allowing for differential drives, synergistic control, and underactuated
//! mechanisms.
//!
//! # MuJoCo Compatibility
//!
//! This implementation follows MuJoCo's fixed tendon model:
//!
//! ```text
//! L = L₀ + Σᵢ cᵢ qᵢ
//! ```
//!
//! Where:
//! - `L` is the tendon length
//! - `L₀` is the rest length
//! - `cᵢ` are coupling coefficients (moment arms)
//! - `qᵢ` are joint positions
//!
//! # Applications
//!
//! - **Differential drives**: One motor controls two joints oppositely
//! - **Parallel mechanisms**: Multiple joints constrained to move together
//! - **Underactuated hands**: Many fingers driven by fewer motors
//! - **Pulley systems**: Mechanical advantage through cable routing

use crate::{TendonActuator, cable::CableProperties};
use sim_types::JointId;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A coupling coefficient for a fixed tendon.
///
/// Each coefficient defines how a joint's position contributes to tendon length:
/// `ΔL = coefficient × joint_position`
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TendonCoefficient {
    /// The joint this coefficient applies to.
    pub joint: JointId,

    /// The coupling coefficient (moment arm in meters for revolute joints).
    ///
    /// Positive coefficient means joint flexion lengthens the tendon.
    /// Negative coefficient means joint flexion shortens the tendon.
    pub coefficient: f64,
}

impl TendonCoefficient {
    /// Create a new tendon coefficient.
    #[must_use]
    pub const fn new(joint: JointId, coefficient: f64) -> Self {
        Self { joint, coefficient }
    }

    /// Create a coefficient with a constant moment arm (for revolute joints).
    #[must_use]
    pub fn moment_arm(joint: JointId, moment_arm: f64) -> Self {
        Self::new(joint, moment_arm)
    }
}

/// A fixed tendon coupling multiple joints.
///
/// Fixed tendons are MuJoCo-style tendons where the tendon length is a
/// linear combination of joint positions. This is an idealized model that
/// doesn't account for 3D routing, but is computationally efficient and
/// works well for many applications.
///
/// # Example
///
/// ```
/// use sim_tendon::{FixedTendon, TendonCoefficient, CableProperties, TendonActuator};
/// use sim_types::JointId;
///
/// // Differential drive: one motor, two joints
/// let tendon = FixedTendon::new("differential")
///     .with_coefficient(JointId::new(0), 0.05)   // 5cm moment arm
///     .with_coefficient(JointId::new(1), -0.05)  // Opposite direction
///     .with_rest_length(0.5)
///     .with_cable(CableProperties::steel_cable(0.002));
///
/// // Compute length at given joint positions
/// let positions = [0.5, -0.5]; // Both joints move opposite, effects add
/// let length = tendon.compute_length(&positions);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FixedTendon {
    /// Name of the tendon.
    name: String,

    /// Coupling coefficients for each joint.
    coefficients: Vec<TendonCoefficient>,

    /// Rest length (L₀).
    rest_length: f64,

    /// Cable/tendon material properties.
    cable: CableProperties,

    /// Range limits (min, max) for tendon length.
    ///
    /// If set, the tendon will apply limit forces when outside this range.
    range: Option<(f64, f64)>,

    /// Limit stiffness (force per unit violation).
    limit_stiffness: f64,
}

impl Default for FixedTendon {
    fn default() -> Self {
        Self::new("unnamed")
    }
}

impl FixedTendon {
    /// Create a new fixed tendon.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            coefficients: Vec::new(),
            rest_length: 0.0,
            cable: CableProperties::default(),
            range: None,
            limit_stiffness: 10_000.0,
        }
    }

    /// Add a coupling coefficient.
    #[must_use]
    pub fn with_coefficient(mut self, joint: JointId, coefficient: f64) -> Self {
        self.coefficients
            .push(TendonCoefficient::new(joint, coefficient));
        self
    }

    /// Add a tendon coefficient.
    #[must_use]
    pub fn with_tendon_coefficient(mut self, coeff: TendonCoefficient) -> Self {
        self.coefficients.push(coeff);
        self
    }

    /// Set the rest length.
    #[must_use]
    pub fn with_rest_length(mut self, rest_length: f64) -> Self {
        self.rest_length = rest_length;
        self.cable.rest_length = rest_length;
        self
    }

    /// Set the cable properties.
    #[must_use]
    pub fn with_cable(mut self, cable: CableProperties) -> Self {
        self.cable = cable;
        // Sync rest length
        self.cable.rest_length = self.rest_length;
        self
    }

    /// Set the tendon stiffness.
    #[must_use]
    pub fn with_stiffness(mut self, stiffness: f64) -> Self {
        self.cable.stiffness = stiffness;
        self
    }

    /// Set the tendon damping.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.cable.damping = damping;
        self
    }

    /// Set tendon length range limits.
    #[must_use]
    pub fn with_range(mut self, min_length: f64, max_length: f64) -> Self {
        self.range = Some((min_length, max_length));
        self
    }

    /// Set limit stiffness.
    #[must_use]
    pub fn with_limit_stiffness(mut self, stiffness: f64) -> Self {
        self.limit_stiffness = stiffness;
        self
    }

    /// Get the tendon name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the coefficients.
    #[must_use]
    pub fn coefficients(&self) -> &[TendonCoefficient] {
        &self.coefficients
    }

    /// Get the cable properties.
    #[must_use]
    pub fn cable(&self) -> &CableProperties {
        &self.cable
    }

    /// Get the rest length.
    #[must_use]
    pub fn rest_length_value(&self) -> f64 {
        self.rest_length
    }

    /// Get the range limits.
    #[must_use]
    pub fn range(&self) -> Option<(f64, f64)> {
        self.range
    }

    /// Compute the length contribution from joints.
    ///
    /// Returns `Σᵢ cᵢ qᵢ`.
    fn compute_length_contribution(&self, joint_positions: &[f64]) -> f64 {
        let mut contribution = 0.0;

        for coeff in &self.coefficients {
            let idx = coeff.joint.raw() as usize;
            if idx < joint_positions.len() {
                contribution += coeff.coefficient * joint_positions[idx];
            }
        }

        contribution
    }

    /// Compute joint torques from tendon tension.
    ///
    /// # Arguments
    ///
    /// * `tension` - Tendon tension in Newtons
    ///
    /// # Returns
    ///
    /// Vector of (joint_id, torque/force) pairs.
    #[must_use]
    pub fn compute_joint_forces(&self, tension: f64) -> Vec<(JointId, f64)> {
        self.coefficients
            .iter()
            .map(|c| (c.joint, c.coefficient * tension))
            .collect()
    }

    /// Compute limit force if outside range.
    fn compute_limit_force(&self, length: f64, velocity: f64) -> f64 {
        match self.range {
            Some((min, max)) => {
                if length < min {
                    // Below minimum - push back
                    let violation = min - length;
                    let damping = 0.1 * self.limit_stiffness; // 10% critical
                    self.limit_stiffness * violation - damping * velocity
                } else if length > max {
                    // Above maximum - push back
                    let violation = length - max;
                    let damping = 0.1 * self.limit_stiffness;
                    -(self.limit_stiffness * violation + damping * velocity)
                } else {
                    0.0
                }
            }
            None => 0.0,
        }
    }

    /// Create a differential tendon coupling two joints.
    ///
    /// When the motor pulls, one joint flexes and the other extends.
    #[must_use]
    pub fn differential(
        name: impl Into<String>,
        joint1: JointId,
        joint2: JointId,
        moment_arm: f64,
    ) -> Self {
        Self::new(name)
            .with_coefficient(joint1, moment_arm)
            .with_coefficient(joint2, -moment_arm)
    }

    /// Create a parallel tendon coupling joints to move together.
    ///
    /// All coupled joints move in the same direction.
    #[must_use]
    pub fn parallel(name: impl Into<String>, joints: &[JointId], moment_arm: f64) -> Self {
        let mut tendon = Self::new(name);
        for &joint in joints {
            tendon = tendon.with_coefficient(joint, moment_arm);
        }
        tendon
    }
}

impl TendonActuator for FixedTendon {
    fn rest_length(&self) -> f64 {
        self.rest_length
    }

    fn compute_length(&self, joint_positions: &[f64]) -> f64 {
        self.rest_length + self.compute_length_contribution(joint_positions)
    }

    fn compute_velocity(&self, _joint_positions: &[f64], joint_velocities: &[f64]) -> f64 {
        // Velocity is derivative of length w.r.t. time
        // dL/dt = Σᵢ cᵢ dqᵢ/dt
        let mut velocity = 0.0;

        for coeff in &self.coefficients {
            let idx = coeff.joint.raw() as usize;
            if idx < joint_velocities.len() {
                velocity += coeff.coefficient * joint_velocities[idx];
            }
        }

        velocity
    }

    fn compute_force(&self, joint_positions: &[f64], joint_velocities: &[f64]) -> f64 {
        let length = self.compute_length(joint_positions);
        let velocity = self.compute_velocity(joint_positions, joint_velocities);

        // Cable force (tension)
        let cable_force = self.cable.compute_force(length, velocity);

        // Limit force
        let limit_force = self.compute_limit_force(length, velocity);

        // Total force (limit force can be negative, but cables can't push)
        (cable_force + limit_force).max(0.0)
    }

    fn jacobian(&self, _joint_positions: &[f64]) -> Vec<f64> {
        // For fixed tendons, the Jacobian is just the coefficients
        // ordered by joint index
        let max_joint = self
            .coefficients
            .iter()
            .map(|c| c.joint.raw() as usize)
            .max()
            .unwrap_or(0);

        let mut jac = vec![0.0; max_joint + 1];

        for coeff in &self.coefficients {
            let idx = coeff.joint.raw() as usize;
            jac[idx] = coeff.coefficient;
        }

        jac
    }

    fn num_joints(&self) -> usize {
        self.coefficients.len()
    }
}

/// A group of fixed tendons that can be controlled together.
///
/// Useful for robot hands and other underactuated systems where multiple
/// tendons work together.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FixedTendonGroup {
    /// The tendons in this group.
    tendons: Vec<FixedTendon>,
}

impl FixedTendonGroup {
    /// Create a new empty tendon group.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a tendon to the group.
    #[must_use]
    pub fn with_tendon(mut self, tendon: FixedTendon) -> Self {
        self.tendons.push(tendon);
        self
    }

    /// Get the number of tendons.
    #[must_use]
    pub fn len(&self) -> usize {
        self.tendons.len()
    }

    /// Check if the group is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.tendons.is_empty()
    }

    /// Get a tendon by index.
    #[must_use]
    pub fn tendon(&self, index: usize) -> Option<&FixedTendon> {
        self.tendons.get(index)
    }

    /// Get a tendon by name.
    #[must_use]
    pub fn tendon_by_name(&self, name: &str) -> Option<&FixedTendon> {
        self.tendons.iter().find(|t| t.name() == name)
    }

    /// Compute forces on all joints from all tendons.
    ///
    /// # Arguments
    ///
    /// * `joint_positions` - Current joint positions
    /// * `joint_velocities` - Current joint velocities
    ///
    /// # Returns
    ///
    /// Vector of (joint_id, total_force) pairs.
    #[must_use]
    pub fn compute_joint_forces(
        &self,
        joint_positions: &[f64],
        joint_velocities: &[f64],
    ) -> Vec<(JointId, f64)> {
        use std::collections::HashMap;

        let mut forces: HashMap<JointId, f64> = HashMap::new();

        for tendon in &self.tendons {
            let tension = tendon.compute_force(joint_positions, joint_velocities);
            for (joint, force) in tendon.compute_joint_forces(tension) {
                *forces.entry(joint).or_insert(0.0) += force;
            }
        }

        forces.into_iter().collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_fixed_tendon_length() {
        let tendon = FixedTendon::new("test")
            .with_coefficient(JointId::new(0), 0.05)
            .with_rest_length(0.3);

        // At zero angle
        assert_relative_eq!(tendon.compute_length(&[0.0]), 0.3, epsilon = 1e-10);

        // At 1 radian
        assert_relative_eq!(tendon.compute_length(&[1.0]), 0.35, epsilon = 1e-10);

        // At -1 radian
        assert_relative_eq!(tendon.compute_length(&[-1.0]), 0.25, epsilon = 1e-10);
    }

    #[test]
    fn test_fixed_tendon_velocity() {
        let tendon = FixedTendon::new("test")
            .with_coefficient(JointId::new(0), 0.05)
            .with_rest_length(0.3);

        // Joint moving at 1 rad/s
        let velocity = tendon.compute_velocity(&[0.0], &[1.0]);
        assert_relative_eq!(velocity, 0.05, epsilon = 1e-10);
    }

    #[test]
    fn test_fixed_tendon_force() {
        let tendon = FixedTendon::new("test")
            .with_coefficient(JointId::new(0), 0.05)
            .with_rest_length(0.3)
            .with_stiffness(10000.0)
            .with_damping(0.0);

        // Stretched by 0.05m (1 radian × 0.05 moment arm)
        // Force = 10000 × 0.05 = 500N
        let force = tendon.compute_force(&[1.0], &[0.0]);
        assert_relative_eq!(force, 500.0, epsilon = 1e-10);

        // Slack (negative angle)
        let force = tendon.compute_force(&[-1.0], &[0.0]);
        assert_relative_eq!(force, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_differential_tendon() {
        let tendon = FixedTendon::differential("diff", JointId::new(0), JointId::new(1), 0.05);

        // Coefficients are +0.05 and -0.05
        // At (0, 0): contribution = 0
        let length_b = tendon.compute_length(&[0.0, 0.0]);
        assert_relative_eq!(length_b, 0.0, epsilon = 1e-10); // rest_length is 0

        // When both joints move same direction, they cancel out:
        // 0.05 * 0.5 + (-0.05) * 0.5 = 0
        let length_c = tendon.compute_length(&[0.5, 0.5]);
        assert_relative_eq!(length_c, length_b, epsilon = 1e-10);

        // When joints move opposite directions, effects add:
        // 0.05 * 0.5 + (-0.05) * (-0.5) = 0.025 + 0.025 = 0.05
        let length_a = tendon.compute_length(&[0.5, -0.5]);
        assert_relative_eq!(length_a - length_b, 0.05, epsilon = 1e-10);

        // Single joint only
        let length_d = tendon.compute_length(&[0.5, 0.0]);
        assert_relative_eq!(length_d - length_b, 0.025, epsilon = 1e-10);
    }

    #[test]
    fn test_jacobian() {
        let tendon = FixedTendon::new("test")
            .with_coefficient(JointId::new(0), 0.05)
            .with_coefficient(JointId::new(2), -0.03);

        let jac = tendon.jacobian(&[0.0, 0.0, 0.0]);

        assert_eq!(jac.len(), 3);
        assert_relative_eq!(jac[0], 0.05, epsilon = 1e-10);
        assert_relative_eq!(jac[1], 0.0, epsilon = 1e-10);
        assert_relative_eq!(jac[2], -0.03, epsilon = 1e-10);
    }

    #[test]
    fn test_joint_forces() {
        let tendon = FixedTendon::new("test")
            .with_coefficient(JointId::new(0), 0.05)
            .with_coefficient(JointId::new(1), -0.03);

        let forces = tendon.compute_joint_forces(100.0);

        assert_eq!(forces.len(), 2);
        assert_relative_eq!(forces[0].1, 5.0, epsilon = 1e-10);
        assert_relative_eq!(forces[1].1, -3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_range_limits() {
        let tendon = FixedTendon::new("test")
            .with_coefficient(JointId::new(0), 0.05)
            .with_rest_length(0.3)
            .with_stiffness(0.0) // No cable stiffness
            .with_range(0.25, 0.35)
            .with_limit_stiffness(10000.0);

        // Within range - no force
        let force = tendon.compute_force(&[0.0], &[0.0]);
        assert_relative_eq!(force, 0.0, epsilon = 1e-10);

        // Below minimum (0.20 < 0.25) - force pushes back
        let force = tendon.compute_force(&[-2.0], &[0.0]); // length = 0.3 - 0.1 = 0.2
        assert!(force > 0.0); // Should produce positive force
    }

    #[test]
    fn test_tendon_group() {
        let group = FixedTendonGroup::new()
            .with_tendon(
                FixedTendon::new("t1")
                    .with_coefficient(JointId::new(0), 0.05)
                    .with_rest_length(0.3)
                    .with_stiffness(10000.0),
            )
            .with_tendon(
                FixedTendon::new("t2")
                    .with_coefficient(JointId::new(0), 0.03)
                    .with_rest_length(0.3)
                    .with_stiffness(10000.0),
            );

        assert_eq!(group.len(), 2);
        assert!(group.tendon_by_name("t1").is_some());
    }
}
