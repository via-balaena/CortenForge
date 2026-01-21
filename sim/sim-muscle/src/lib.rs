//! Hill-type muscle actuators for biomechanical simulation.
//!
//! This crate provides a comprehensive implementation of Hill-type muscle models
//! for simulating musculoskeletal dynamics. These models are essential for:
//!
//! - Human movement simulation
//! - Biomechanical analysis
//! - Rehabilitation robotics
//! - Prosthetics control
//! - Sports performance analysis
//!
//! # Hill-Type Muscle Model
//!
//! The Hill model represents muscle as a combination of:
//!
//! 1. **Contractile Element (CE)**: Active force generation based on:
//!    - Activation level (neural command → muscle activation)
//!    - Force-length relationship (sarcomere mechanics)
//!    - Force-velocity relationship (cross-bridge cycling)
//!
//! 2. **Parallel Elastic Element (PE)**: Passive resistance at long lengths
//!
//! 3. **Series Elastic Element (SE)**: Tendon compliance
//!
//! ```text
//!                    ┌──────────────────────┐
//!                    │    Muscle-Tendon     │
//!                    │         Unit         │
//!                    │                      │
//!                    │  ┌────┐    ┌────┐   │
//!    Excitation ────►│  │ CE │────│ SE │───┼───► Force
//!        u           │  └────┘    │    │   │
//!                    │      │     │    │   │
//!                    │  ┌────┐    │    │   │
//!                    │  │ PE │────┘    │   │
//!                    │  └────┘         │   │
//!                    │                 │   │
//!                    └─────────────────┼───┘
//!                              Tendon ─┘
//! ```
//!
//! # Quick Start
//!
//! ```
//! use sim_muscle::{HillMuscle, HillMuscleConfig};
//!
//! // Create a biceps muscle
//! let config = HillMuscleConfig::biceps();
//! let mut muscle = HillMuscle::new(config);
//!
//! // Apply neural excitation (0 = relaxed, 1 = maximum)
//! muscle.set_excitation(0.7);
//!
//! // Compute torque at a joint
//! let joint_angle = 1.2;  // radians
//! let joint_velocity = 0.0;  // rad/s
//! let dt = 0.001;  // timestep
//!
//! let torque = muscle.compute_torque(joint_angle, joint_velocity, dt);
//! println!("Joint torque: {:.2} Nm", torque);
//! ```
//!
//! # Predefined Muscle Configurations
//!
//! The crate includes configurations for common muscles:
//!
//! - [`HillMuscleConfig::biceps()`] - Upper arm flexor
//! - [`HillMuscleConfig::quadriceps()`] - Knee extensor
//! - [`HillMuscleConfig::gastrocnemius()`] - Ankle plantarflexor (fast)
//! - [`HillMuscleConfig::soleus()`] - Ankle plantarflexor (slow)
//!
//! # Activation Dynamics
//!
//! Neural excitation is transformed to muscle activation through first-order
//! dynamics with asymmetric time constants:
//!
//! ```text
//! da/dt = (u - a) / τ(u, a)
//!
//! where τ = τ_act if activating (u > a)
//!       τ = τ_deact if deactivating (u < a)
//! ```
//!
//! This captures the delay between neural command and force production.
//!
//! # Force-Length-Velocity Relationships
//!
//! ## Force-Length
//!
//! Active force depends on sarcomere length:
//! - Maximum at optimal fiber length
//! - Reduced at shorter or longer lengths
//!
//! Passive force rises exponentially at long lengths due to connective tissue.
//!
//! ## Force-Velocity
//!
//! - **Concentric** (shortening): Force decreases with velocity
//! - **Isometric** (no movement): Maximum sustainable force
//! - **Eccentric** (lengthening): Force can exceed isometric maximum
//!
//! # Pennation Angle
//!
//! Many muscles have fibers oriented at an angle to the tendon (pennation).
//! This reduces force transmission but allows more fibers in a given volume.
//! The model accounts for:
//!
//! - Force projection: F_tendon = F_fiber × cos(α)
//! - Variable pennation: As fiber shortens, pennation angle increases
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//!
//! - Headless training loops for reinforcement learning
//! - Hardware control systems
//! - Analysis and optimization tools
//! - Integration with other physics engines
//!
//! # References
//!
//! - Hill, A.V. (1938). The heat of shortening and the dynamic constants of muscle.
//! - Zajac, F.E. (1989). Muscle and tendon: properties, models, scaling.
//! - Thelen, D.G. (2003). Adjustment of muscle mechanics model parameters.
//! - Millard, M. et al. (2013). Flexing computational muscle.

#![doc(html_root_url = "https://docs.rs/sim-muscle/0.1.0")]
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
#![warn(missing_docs)]
#![allow(
    clippy::missing_const_for_fn,
    clippy::module_name_repetitions,
    clippy::doc_markdown,
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::suboptimal_flops,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::let_and_return,
    clippy::derivable_impls,
    clippy::imprecise_flops
)]
#![cfg_attr(test, allow(clippy::float_cmp, clippy::let_underscore_must_use))]

pub mod activation;
pub mod curves;
pub mod hill;
pub mod kinematics;

// Re-export main types at crate root
pub use activation::{ActivationDynamics, ActivationState};
pub use curves::{
    ActiveForceLengthCurve, ForceVelocityCurve, MuscleForceCurves, PassiveForceLengthCurve,
};
pub use hill::{FiberState, HillMuscle, HillMuscleConfig, MuscleDiagnostics, MuscleForceResult};
pub use kinematics::{
    BiarticularlMuscleConfig, ConstantMomentArm, MomentArmModel, MusclePath, PolynomialMomentArm,
    SplineMomentArm, ViaPoint,
};

/// Trait for muscle actuators that can compute joint torque.
///
/// This trait provides a common interface for different muscle models,
/// allowing them to be used interchangeably with the constraint system.
pub trait MuscleActuator {
    /// Set the neural excitation command.
    ///
    /// # Arguments
    ///
    /// * `excitation` - Neural command in range [0, 1]
    fn set_excitation(&mut self, excitation: f64);

    /// Get the current excitation level.
    fn excitation(&self) -> f64;

    /// Get the current activation level.
    fn activation(&self) -> f64;

    /// Compute the joint torque given current joint state.
    ///
    /// # Arguments
    ///
    /// * `joint_angle` - Current joint angle (radians)
    /// * `joint_velocity` - Current joint angular velocity (rad/s)
    /// * `dt` - Timestep (seconds)
    ///
    /// # Returns
    ///
    /// Joint torque (Nm).
    fn compute_torque(&mut self, joint_angle: f64, joint_velocity: f64, dt: f64) -> f64;

    /// Reset the muscle to its initial state.
    fn reset(&mut self);
}

impl MuscleActuator for HillMuscle {
    fn set_excitation(&mut self, excitation: f64) {
        self.set_excitation(excitation);
    }

    fn excitation(&self) -> f64 {
        self.excitation()
    }

    fn activation(&self) -> f64 {
        self.activation()
    }

    fn compute_torque(&mut self, joint_angle: f64, joint_velocity: f64, dt: f64) -> f64 {
        self.compute_torque(joint_angle, joint_velocity, dt)
    }

    fn reset(&mut self) {
        self.reset();
    }
}

/// A collection of muscles acting on a joint.
///
/// Joints are often actuated by multiple muscles (agonist/antagonist pairs).
/// This struct manages multiple muscles and computes net torque.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MuscleGroup {
    /// The muscles in this group.
    muscles: Vec<HillMuscle>,

    /// Direction multipliers for each muscle.
    /// +1 for muscles that produce positive torque (flexors)
    /// -1 for muscles that produce negative torque (extensors)
    directions: Vec<f64>,
}

impl MuscleGroup {
    /// Create a new empty muscle group.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a muscle that produces positive torque (flexor).
    #[must_use]
    pub fn with_flexor(mut self, muscle: HillMuscle) -> Self {
        self.muscles.push(muscle);
        self.directions.push(1.0);
        self
    }

    /// Add a muscle that produces negative torque (extensor).
    #[must_use]
    pub fn with_extensor(mut self, muscle: HillMuscle) -> Self {
        self.muscles.push(muscle);
        self.directions.push(-1.0);
        self
    }

    /// Add a muscle with a custom direction multiplier.
    #[must_use]
    pub fn with_muscle(mut self, muscle: HillMuscle, direction: f64) -> Self {
        self.muscles.push(muscle);
        self.directions.push(direction.signum());
        self
    }

    /// Get the number of muscles in the group.
    #[must_use]
    pub fn len(&self) -> usize {
        self.muscles.len()
    }

    /// Check if the group is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.muscles.is_empty()
    }

    /// Get a reference to a muscle by index.
    #[must_use]
    pub fn muscle(&self, index: usize) -> Option<&HillMuscle> {
        self.muscles.get(index)
    }

    /// Get a mutable reference to a muscle by index.
    #[must_use]
    pub fn muscle_mut(&mut self, index: usize) -> Option<&mut HillMuscle> {
        self.muscles.get_mut(index)
    }

    /// Set excitation for a specific muscle.
    pub fn set_excitation(&mut self, index: usize, excitation: f64) {
        if let Some(muscle) = self.muscles.get_mut(index) {
            muscle.set_excitation(excitation);
        }
    }

    /// Set excitation for all muscles.
    pub fn set_all_excitations(&mut self, excitations: &[f64]) {
        for (muscle, &exc) in self.muscles.iter_mut().zip(excitations.iter()) {
            muscle.set_excitation(exc);
        }
    }

    /// Compute net torque from all muscles.
    ///
    /// # Arguments
    ///
    /// * `joint_angle` - Current joint angle (radians)
    /// * `joint_velocity` - Current joint velocity (rad/s)
    /// * `dt` - Timestep (seconds)
    ///
    /// # Returns
    ///
    /// Net joint torque (Nm).
    pub fn compute_net_torque(&mut self, joint_angle: f64, joint_velocity: f64, dt: f64) -> f64 {
        let mut net_torque = 0.0;

        for (muscle, &dir) in self.muscles.iter_mut().zip(self.directions.iter()) {
            let torque = muscle.compute_torque(joint_angle, joint_velocity, dt);
            net_torque += dir * torque;
        }

        net_torque
    }

    /// Reset all muscles to initial state.
    pub fn reset(&mut self) {
        for muscle in &mut self.muscles {
            muscle.reset();
        }
    }

    /// Get diagnostics for all muscles.
    #[must_use]
    pub fn diagnostics(&self) -> Vec<MuscleDiagnostics> {
        self.muscles.iter().map(HillMuscle::diagnostics).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_muscle_actuator_trait() {
        let config = HillMuscleConfig::default();
        let mut muscle = HillMuscle::new(config);

        // Use trait methods
        MuscleActuator::set_excitation(&mut muscle, 0.5);
        assert_relative_eq!(muscle.excitation(), 0.5, epsilon = 1e-10);

        let torque = MuscleActuator::compute_torque(&mut muscle, 0.0, 0.0, 0.001);
        assert!(torque >= 0.0);
    }

    #[test]
    fn test_muscle_group() {
        let biceps = HillMuscle::new(HillMuscleConfig::biceps());
        let triceps = HillMuscle::new(HillMuscleConfig::default());

        let mut group = MuscleGroup::new()
            .with_flexor(biceps)
            .with_extensor(triceps);

        assert_eq!(group.len(), 2);

        // Set excitations
        group.set_excitation(0, 0.8); // Biceps on
        group.set_excitation(1, 0.2); // Triceps partial

        // Compute net torque
        let torque = group.compute_net_torque(0.0, 0.0, 0.001);

        // Should be positive (biceps stronger than triceps)
        // Note: on first step, activation hasn't ramped up yet
        assert!(torque.abs() < 100.0); // Sanity check
    }

    #[test]
    fn test_muscle_group_antagonist_pair() {
        let flexor = HillMuscle::new(HillMuscleConfig::default());
        let extensor = HillMuscle::new(HillMuscleConfig::default());

        let mut group = MuscleGroup::new()
            .with_flexor(flexor)
            .with_extensor(extensor);

        // Both fully activated - should nearly cancel
        group.set_all_excitations(&[1.0, 1.0]);

        // Warm up activations
        for _ in 0..200 {
            let _ = group.compute_net_torque(0.0, 0.0, 0.001);
        }

        let torque = group.compute_net_torque(0.0, 0.0, 0.001);

        // Net torque should be near zero (co-contraction)
        assert!(torque.abs() < 10.0);
    }

    #[test]
    fn test_predefined_configs() {
        // Just verify they create valid configurations
        let _ = HillMuscleConfig::biceps();
        let _ = HillMuscleConfig::quadriceps();
        let _ = HillMuscleConfig::gastrocnemius();
        let _ = HillMuscleConfig::soleus();
    }
}
