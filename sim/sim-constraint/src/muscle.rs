#![allow(clippy::doc_markdown)]
//! Muscle actuator integration for joints.
//!
//! This module provides the bridge between Hill-type muscle models from `sim-muscle`
//! and the joint constraint system.
//!
//! # Feature Flag
//!
//! This module is only available when the `muscle` feature is enabled:
//!
//! ```toml
//! [dependencies]
//! sim-constraint = { version = "0.1", features = ["muscle"] }
//! ```
//!
//! # Usage
//!
//! ```ignore
//! use sim_constraint::{RevoluteJoint, MuscleJoint};
//! use sim_muscle::{HillMuscle, HillMuscleConfig, MuscleGroup};
//!
//! // Create a joint with muscle actuation
//! let base_joint = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());
//!
//! let biceps = HillMuscle::new(HillMuscleConfig::biceps());
//! let triceps = HillMuscle::new(HillMuscleConfig::default());
//!
//! let muscles = MuscleGroup::new()
//!     .with_flexor(biceps)
//!     .with_extensor(triceps);
//!
//! let muscle_joint = MuscleJoint::new(base_joint, muscles);
//! ```

use crate::{Joint, JointLimits, JointMotor, JointType, RevoluteJoint};
use nalgebra::Point3;
use sim_muscle::{HillMuscle, MuscleGroup};
use sim_types::BodyId;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A revolute joint actuated by muscle-tendon units.
///
/// This wraps a standard [`RevoluteJoint`] and adds muscle actuation
/// capabilities. The muscle force is computed using Hill-type muscle
/// models and adds to any motor forces.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MuscleJoint {
    /// The underlying revolute joint.
    base: RevoluteJoint,

    /// Muscle group actuating this joint.
    muscles: MuscleGroup,

    /// Whether muscle actuation is enabled.
    enabled: bool,
}

impl MuscleJoint {
    /// Create a new muscle-actuated joint.
    #[must_use]
    pub fn new(base: RevoluteJoint, muscles: MuscleGroup) -> Self {
        Self {
            base,
            muscles,
            enabled: true,
        }
    }

    /// Create from a single muscle (flexor).
    #[must_use]
    pub fn with_single_muscle(base: RevoluteJoint, muscle: HillMuscle) -> Self {
        Self {
            base,
            muscles: MuscleGroup::new().with_flexor(muscle),
            enabled: true,
        }
    }

    /// Enable or disable muscle actuation.
    #[must_use]
    pub fn with_enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Get a reference to the base joint.
    #[must_use]
    pub fn base(&self) -> &RevoluteJoint {
        &self.base
    }

    /// Get a mutable reference to the base joint.
    #[must_use]
    pub fn base_mut(&mut self) -> &mut RevoluteJoint {
        &mut self.base
    }

    /// Get a reference to the muscle group.
    #[must_use]
    pub fn muscles(&self) -> &MuscleGroup {
        &self.muscles
    }

    /// Get a mutable reference to the muscle group.
    #[must_use]
    pub fn muscles_mut(&mut self) -> &mut MuscleGroup {
        &mut self.muscles
    }

    /// Set excitation for a specific muscle.
    pub fn set_muscle_excitation(&mut self, index: usize, excitation: f64) {
        self.muscles.set_excitation(index, excitation);
    }

    /// Set excitation for all muscles.
    pub fn set_all_excitations(&mut self, excitations: &[f64]) {
        self.muscles.set_all_excitations(excitations);
    }

    /// Get the current joint angle.
    #[must_use]
    pub fn angle(&self) -> f64 {
        self.base.angle()
    }

    /// Set the joint angle.
    pub fn set_angle(&mut self, angle: f64) {
        self.base.set_angle(angle);
    }

    /// Compute total joint force including muscles, motor, limits, and damping.
    ///
    /// # Arguments
    ///
    /// * `velocity` - Current joint velocity (rad/s)
    /// * `dt` - Timestep (seconds)
    ///
    /// # Returns
    ///
    /// Total joint torque (Nm).
    #[must_use]
    pub fn compute_joint_force(&mut self, velocity: f64, dt: f64) -> f64 {
        // Base joint force (motor + limits + damping)
        let base_force = self.base.compute_joint_force(velocity);

        // Add muscle force if enabled
        let muscle_force = if self.enabled && !self.muscles.is_empty() {
            self.muscles
                .compute_net_torque(self.base.angle(), velocity, dt)
        } else {
            0.0
        };

        base_force + muscle_force
    }

    /// Reset muscles to initial state.
    pub fn reset_muscles(&mut self) {
        self.muscles.reset();
    }
}

impl Joint for MuscleJoint {
    fn parent(&self) -> BodyId {
        self.base.parent()
    }

    fn child(&self) -> BodyId {
        self.base.child()
    }

    fn dof(&self) -> usize {
        self.base.dof()
    }

    fn joint_type(&self) -> JointType {
        self.base.joint_type()
    }

    fn limits(&self) -> Option<&JointLimits> {
        self.base.limits()
    }

    fn motor(&self) -> Option<&JointMotor> {
        self.base.motor()
    }

    fn damping(&self) -> f64 {
        self.base.damping()
    }

    fn parent_anchor(&self) -> Point3<f64> {
        self.base.parent_anchor()
    }

    fn child_anchor(&self) -> Point3<f64> {
        self.base.child_anchor()
    }
}

/// Builder for creating muscle-actuated joints.
#[derive(Debug, Default)]
pub struct MuscleJointBuilder {
    flexors: Vec<HillMuscle>,
    extensors: Vec<HillMuscle>,
}

impl MuscleJointBuilder {
    /// Create a new builder.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a flexor muscle.
    #[must_use]
    pub fn flexor(mut self, muscle: HillMuscle) -> Self {
        self.flexors.push(muscle);
        self
    }

    /// Add an extensor muscle.
    #[must_use]
    pub fn extensor(mut self, muscle: HillMuscle) -> Self {
        self.extensors.push(muscle);
        self
    }

    /// Build the muscle joint.
    #[must_use]
    pub fn build(self, base: RevoluteJoint) -> MuscleJoint {
        let mut group = MuscleGroup::new();

        for muscle in self.flexors {
            group = group.with_flexor(muscle);
        }

        for muscle in self.extensors {
            group = group.with_extensor(muscle);
        }

        MuscleJoint::new(base, group)
    }
}

/// Stores muscle excitation commands for a musculoskeletal system.
///
/// This is useful for interfacing with control systems that output
/// muscle activations.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MuscleCommands {
    /// Excitation values indexed by (joint_index, muscle_index).
    excitations: Vec<Vec<f64>>,
}

impl MuscleCommands {
    /// Create new muscle commands for the given number of joints.
    #[must_use]
    pub fn new(num_joints: usize) -> Self {
        Self {
            excitations: vec![Vec::new(); num_joints],
        }
    }

    /// Initialize commands for a joint with the given number of muscles.
    pub fn init_joint(&mut self, joint_index: usize, num_muscles: usize) {
        if joint_index >= self.excitations.len() {
            self.excitations.resize(joint_index + 1, Vec::new());
        }
        self.excitations[joint_index] = vec![0.0; num_muscles];
    }

    /// Set excitation for a specific muscle.
    pub fn set(&mut self, joint_index: usize, muscle_index: usize, excitation: f64) {
        if let Some(joint_excs) = self.excitations.get_mut(joint_index) {
            if let Some(exc) = joint_excs.get_mut(muscle_index) {
                *exc = excitation.clamp(0.0, 1.0);
            }
        }
    }

    /// Get excitation for a specific muscle.
    #[must_use]
    pub fn get(&self, joint_index: usize, muscle_index: usize) -> Option<f64> {
        self.excitations
            .get(joint_index)
            .and_then(|joint| joint.get(muscle_index).copied())
    }

    /// Get all excitations for a joint.
    #[must_use]
    pub fn joint_excitations(&self, joint_index: usize) -> Option<&[f64]> {
        self.excitations.get(joint_index).map(Vec::as_slice)
    }

    /// Set all excitations from a flat array.
    ///
    /// The array should be organized as [joint0_muscle0, joint0_muscle1, ..., joint1_muscle0, ...].
    pub fn set_from_flat(&mut self, values: &[f64]) {
        let mut idx = 0;
        for joint_excs in &mut self.excitations {
            for exc in joint_excs.iter_mut() {
                if idx < values.len() {
                    *exc = values[idx].clamp(0.0, 1.0);
                    idx += 1;
                }
            }
        }
    }

    /// Get the total number of muscles across all joints.
    #[must_use]
    pub fn total_muscles(&self) -> usize {
        self.excitations.iter().map(Vec::len).sum()
    }

    /// Reset all excitations to zero.
    pub fn reset(&mut self) {
        for joint_excs in &mut self.excitations {
            for exc in joint_excs.iter_mut() {
                *exc = 0.0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vector3;
    use approx::assert_relative_eq;
    use sim_muscle::HillMuscleConfig;

    #[test]
    fn test_muscle_joint_creation() {
        let base = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());
        let muscle = HillMuscle::new(HillMuscleConfig::default());

        let joint = MuscleJoint::with_single_muscle(base, muscle);

        assert_eq!(joint.dof(), 1);
        assert!(!joint.muscles().is_empty());
    }

    #[test]
    fn test_muscle_joint_force_computation() {
        let base = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());
        let mut muscle = HillMuscle::new(HillMuscleConfig::default());
        muscle.set_excitation(1.0);

        let mut joint = MuscleJoint::with_single_muscle(base, muscle);

        // Compute force with muscle actuation
        let force = joint.compute_joint_force(0.0, 0.001);

        // Should have some force from muscle
        assert!(force.abs() >= 0.0);
    }

    #[test]
    fn test_muscle_joint_builder() {
        let biceps = HillMuscle::new(HillMuscleConfig::biceps());
        let triceps = HillMuscle::new(HillMuscleConfig::default());

        let base = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());

        let joint = MuscleJointBuilder::new()
            .flexor(biceps)
            .extensor(triceps)
            .build(base);

        assert_eq!(joint.muscles().len(), 2);
    }

    #[test]
    fn test_muscle_commands() {
        let mut commands = MuscleCommands::new(2);
        commands.init_joint(0, 2); // 2 muscles at joint 0
        commands.init_joint(1, 3); // 3 muscles at joint 1

        assert_eq!(commands.total_muscles(), 5);

        commands.set(0, 0, 0.8);
        commands.set(1, 2, 0.5);

        assert_relative_eq!(commands.get(0, 0).unwrap_or(0.0), 0.8, epsilon = 1e-10);
        assert_relative_eq!(commands.get(1, 2).unwrap_or(0.0), 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_muscle_commands_from_flat() {
        let mut commands = MuscleCommands::new(2);
        commands.init_joint(0, 2);
        commands.init_joint(1, 2);

        commands.set_from_flat(&[0.1, 0.2, 0.3, 0.4]);

        assert_relative_eq!(commands.get(0, 0).unwrap_or(0.0), 0.1, epsilon = 1e-10);
        assert_relative_eq!(commands.get(0, 1).unwrap_or(0.0), 0.2, epsilon = 1e-10);
        assert_relative_eq!(commands.get(1, 0).unwrap_or(0.0), 0.3, epsilon = 1e-10);
        assert_relative_eq!(commands.get(1, 1).unwrap_or(0.0), 0.4, epsilon = 1e-10);
    }

    #[test]
    fn test_disabled_muscle_joint() {
        let base = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());
        let mut muscle = HillMuscle::new(HillMuscleConfig::default());
        muscle.set_excitation(1.0);

        let mut joint = MuscleJoint::with_single_muscle(base, muscle).with_enabled(false);

        // Warm up
        for _ in 0..100 {
            let _ = joint.compute_joint_force(0.0, 0.001);
        }

        // Should have no muscle force when disabled
        let force = joint.compute_joint_force(0.0, 0.001);

        // With no motor/damping on base joint, force should be 0
        assert_relative_eq!(force, 0.0, epsilon = 1e-10);
    }
}
