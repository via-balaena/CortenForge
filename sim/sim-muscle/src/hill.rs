//! Hill-type muscle model.
//!
//! This module implements a complete Hill-type muscle-tendon unit (MTU) for
//! biomechanical simulation. The model captures:
//!
//! - Activation dynamics (neural command to muscle activation)
//! - Force-length relationship (sarcomere mechanics)
//! - Force-velocity relationship (cross-bridge cycling)
//! - Pennation angle effects (fiber arrangement)
//! - Series elastic tendon (compliant tendon option)
//!
//! # Architecture
//!
//! ```text
//!                    ┌────────────────────────────────────┐
//!    Neural         │         Muscle-Tendon Unit         │         Joint
//!   Excitation ────►│ ┌──────────────┐   ┌─────────────┐ │──────► Torque
//!       u           │ │   Muscle     │   │   Tendon    │ │
//!                   │ │ (Contractile │───│  (Elastic   │ │
//!                   │ │   Element)   │   │   Element)  │ │
//!                   │ └──────────────┘   └─────────────┘ │
//!                   └────────────────────────────────────┘
//! ```
//!
//! # Usage
//!
//! ```
//! use sim_muscle::{HillMuscle, HillMuscleConfig};
//!
//! // Create a typical biceps muscle
//! let config = HillMuscleConfig::biceps();
//! let mut muscle = HillMuscle::new(config);
//!
//! // Apply neural excitation (0 to 1)
//! muscle.set_excitation(0.8);
//!
//! // Step the simulation
//! let joint_angle = 1.2; // radians
//! let joint_velocity = 0.1; // rad/s
//! let dt = 0.001; // 1 ms
//!
//! let torque = muscle.compute_torque(joint_angle, joint_velocity, dt);
//! ```
//!
//! # References
//!
//! - Zajac, F.E. (1989). Muscle and tendon: properties, models, scaling.
//! - Thelen, D.G. (2003). Adjustment of muscle mechanics model parameters.
//! - Millard, M. et al. (2013). Flexing computational muscle: modeling and
//!   simulation of musculotendon dynamics.

use crate::activation::{ActivationDynamics, ActivationState};
use crate::curves::MuscleForceCurves;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Configuration parameters for a Hill-type muscle.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct HillMuscleConfig {
    /// Maximum isometric force the muscle can produce (N).
    pub max_isometric_force: f64,

    /// Optimal fiber length where maximum force is generated (m).
    pub optimal_fiber_length: f64,

    /// Tendon slack length - length at which tendon begins to bear load (m).
    pub tendon_slack_length: f64,

    /// Pennation angle at optimal fiber length (radians).
    /// Angle between muscle fibers and the line of action.
    pub pennation_angle_optimal: f64,

    /// Maximum contraction velocity (lengths per second).
    /// Typical values: 5-15 L_opt/s.
    pub max_contraction_velocity: f64,

    /// Activation dynamics parameters.
    pub activation_dynamics: ActivationDynamics,

    /// Force curve parameters.
    pub force_curves: MuscleForceCurves,

    /// Moment arm at the joint (m).
    /// Converts muscle force to joint torque.
    /// Can be constant or computed from joint angle (see `MomentArmModel`).
    pub moment_arm: f64,

    /// Tendon stiffness parameter.
    /// Higher values = stiffer tendon (approaches rigid).
    /// Typical: 30-50 for compliant tendon, >100 for nearly rigid.
    pub tendon_stiffness: f64,

    /// Whether to use a rigid tendon model (faster, less accurate).
    pub rigid_tendon: bool,
}

impl Default for HillMuscleConfig {
    fn default() -> Self {
        Self {
            max_isometric_force: 1000.0,    // 1000 N
            optimal_fiber_length: 0.10,     // 10 cm
            tendon_slack_length: 0.20,      // 20 cm
            pennation_angle_optimal: 0.0,   // Parallel fibers
            max_contraction_velocity: 10.0, // 10 L_opt/s
            activation_dynamics: ActivationDynamics::default(),
            force_curves: MuscleForceCurves::default(),
            moment_arm: 0.05, // 5 cm
            tendon_stiffness: 35.0,
            rigid_tendon: true, // Faster by default
        }
    }
}

impl HillMuscleConfig {
    /// Create configuration for a biceps-like muscle.
    #[must_use]
    pub fn biceps() -> Self {
        Self {
            max_isometric_force: 700.0,  // ~700 N
            optimal_fiber_length: 0.116, // 11.6 cm
            tendon_slack_length: 0.272,  // 27.2 cm
            pennation_angle_optimal: 0.0,
            max_contraction_velocity: 10.0,
            moment_arm: 0.04, // ~4 cm average
            ..Default::default()
        }
    }

    /// Create configuration for a quadriceps-like muscle.
    #[must_use]
    pub fn quadriceps() -> Self {
        Self {
            max_isometric_force: 7000.0,    // ~7000 N (combined)
            optimal_fiber_length: 0.084,    // 8.4 cm
            tendon_slack_length: 0.346,     // 34.6 cm
            pennation_angle_optimal: 0.087, // ~5 degrees
            max_contraction_velocity: 8.0,
            moment_arm: 0.05, // ~5 cm
            ..Default::default()
        }
    }

    /// Create configuration for a gastrocnemius-like muscle.
    #[must_use]
    pub fn gastrocnemius() -> Self {
        Self {
            max_isometric_force: 1500.0,    // ~1500 N
            optimal_fiber_length: 0.055,    // 5.5 cm
            tendon_slack_length: 0.400,     // 40 cm (Achilles)
            pennation_angle_optimal: 0.297, // ~17 degrees
            max_contraction_velocity: 12.0,
            moment_arm: 0.05, // ~5 cm
            ..Default::default()
        }
    }

    /// Create configuration for a soleus-like muscle.
    #[must_use]
    pub fn soleus() -> Self {
        Self {
            max_isometric_force: 3500.0,    // ~3500 N
            optimal_fiber_length: 0.030,    // 3 cm
            tendon_slack_length: 0.268,     // 26.8 cm
            pennation_angle_optimal: 0.436, // ~25 degrees
            max_contraction_velocity: 6.0,  // Slower (slow-twitch dominant)
            activation_dynamics: ActivationDynamics::slow_twitch(),
            moment_arm: 0.05,
            ..Default::default()
        }
    }

    /// Set custom force curves.
    #[must_use]
    pub fn with_force_curves(mut self, curves: MuscleForceCurves) -> Self {
        self.force_curves = curves;
        self
    }

    /// Set custom activation dynamics.
    #[must_use]
    pub fn with_activation_dynamics(mut self, dynamics: ActivationDynamics) -> Self {
        self.activation_dynamics = dynamics;
        self
    }

    /// Enable or disable rigid tendon model.
    #[must_use]
    pub fn with_rigid_tendon(mut self, rigid: bool) -> Self {
        self.rigid_tendon = rigid;
        self
    }

    /// Set the moment arm.
    #[must_use]
    pub fn with_moment_arm(mut self, moment_arm: f64) -> Self {
        self.moment_arm = moment_arm.abs();
        self
    }

    /// Compute the total muscle-tendon length at a given joint angle.
    ///
    /// This is a simplified linear model. For more accuracy, use
    /// polynomial moment arm models.
    #[must_use]
    pub fn musculotendon_length(&self, joint_angle: f64) -> f64 {
        // L_mt = L_t0 + L_opt + r * theta
        // (simplified model assuming linear relationship)
        self.tendon_slack_length + self.optimal_fiber_length - self.moment_arm * joint_angle
    }
}

/// State of the muscle fiber.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FiberState {
    /// Current fiber length (m).
    pub length: f64,

    /// Current fiber velocity (m/s).
    /// Negative = shortening, Positive = lengthening.
    pub velocity: f64,
}

impl FiberState {
    /// Create fiber state at rest at optimal length.
    #[must_use]
    pub fn at_optimal(optimal_length: f64) -> Self {
        Self {
            length: optimal_length,
            velocity: 0.0,
        }
    }

    /// Compute normalized fiber length (L / L_opt).
    #[must_use]
    pub fn normalized_length(&self, optimal_length: f64) -> f64 {
        self.length / optimal_length
    }

    /// Compute normalized fiber velocity (v / v_max).
    #[must_use]
    pub fn normalized_velocity(&self, optimal_length: f64, max_velocity: f64) -> f64 {
        self.velocity / (optimal_length * max_velocity)
    }
}

/// Complete Hill-type muscle model.
///
/// This struct contains all state and parameters needed to simulate
/// a muscle-tendon unit.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct HillMuscle {
    /// Configuration parameters.
    config: HillMuscleConfig,

    /// Activation state (excitation and activation).
    activation: ActivationState,

    /// Fiber state (length and velocity).
    fiber: FiberState,

    /// Previous muscle-tendon length for velocity estimation.
    prev_mt_length: f64,

    /// Name or identifier for the muscle.
    name: String,
}

impl HillMuscle {
    /// Create a new Hill muscle with the given configuration.
    #[must_use]
    pub fn new(config: HillMuscleConfig) -> Self {
        let optimal = config.optimal_fiber_length;
        Self {
            fiber: FiberState::at_optimal(optimal),
            prev_mt_length: config.tendon_slack_length + optimal,
            activation: ActivationState::default(),
            config,
            name: String::new(),
        }
    }

    /// Create a muscle with a name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = name.into();
        self
    }

    /// Get the muscle name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the configuration.
    #[must_use]
    pub fn config(&self) -> &HillMuscleConfig {
        &self.config
    }

    /// Get the current activation level.
    #[must_use]
    pub fn activation(&self) -> f64 {
        self.activation.activation()
    }

    /// Get the current neural excitation.
    #[must_use]
    pub fn excitation(&self) -> f64 {
        self.activation.excitation
    }

    /// Get the current fiber state.
    #[must_use]
    pub fn fiber_state(&self) -> &FiberState {
        &self.fiber
    }

    /// Set the neural excitation command.
    ///
    /// # Arguments
    ///
    /// * `excitation` - Neural command in range [0, 1]
    pub fn set_excitation(&mut self, excitation: f64) {
        self.activation.set_excitation(excitation);
    }

    /// Compute the current pennation angle.
    ///
    /// As the muscle shortens, the pennation angle increases to conserve
    /// muscle width (constant width assumption).
    #[must_use]
    pub fn pennation_angle(&self) -> f64 {
        if self.config.pennation_angle_optimal.abs() < 1e-6 {
            // Parallel fibers
            return 0.0;
        }

        // Constant muscle width model
        let width = self.config.optimal_fiber_length * self.config.pennation_angle_optimal.sin();
        let sin_alpha = width / self.fiber.length;

        // Clamp to valid range
        sin_alpha.clamp(-1.0, 1.0).asin()
    }

    /// Compute the fiber force along the tendon direction.
    ///
    /// This accounts for pennation angle:
    /// F_tendon = F_fiber * cos(alpha)
    #[must_use]
    pub fn compute_tendon_force(&self) -> f64 {
        let a = self.activation.activation();
        let l_norm = self
            .fiber
            .normalized_length(self.config.optimal_fiber_length);
        let v_norm = self.fiber.normalized_velocity(
            self.config.optimal_fiber_length,
            self.config.max_contraction_velocity,
        );

        let force_multiplier = self.config.force_curves.evaluate(a, l_norm, v_norm);
        let fiber_force = self.config.max_isometric_force * force_multiplier;

        // Project through pennation angle
        let pennation = self.pennation_angle();
        fiber_force * pennation.cos()
    }

    /// Compute joint torque given current joint state.
    ///
    /// This is the main interface for using the muscle in simulation.
    ///
    /// # Arguments
    ///
    /// * `joint_angle` - Current joint angle (radians)
    /// * `joint_velocity` - Current joint velocity (rad/s)
    /// * `dt` - Timestep (seconds)
    ///
    /// # Returns
    ///
    /// Joint torque (Nm). Positive torque acts in the direction of
    /// muscle shortening (usually flexion).
    pub fn compute_torque(&mut self, joint_angle: f64, joint_velocity: f64, dt: f64) -> f64 {
        // Update activation dynamics
        self.activation.update(&self.config.activation_dynamics, dt);

        // Compute muscle-tendon length from joint kinematics
        let mt_length = self.config.musculotendon_length(joint_angle);

        // Compute MT velocity from moment arm and joint velocity
        // dL_mt/dt = -r * dθ/dt (negative because muscle shortens with positive flexion)
        let mt_velocity = -self.config.moment_arm * joint_velocity;

        // Update fiber state
        self.update_fiber_state_with_velocity(mt_length, mt_velocity, dt);

        // Compute force
        let tendon_force = self.compute_tendon_force();

        // Convert to torque
        tendon_force * self.config.moment_arm
    }

    /// Update fiber length and velocity from muscle-tendon kinematics.
    ///
    /// This version uses finite difference to estimate MT velocity.
    #[allow(dead_code)]
    fn update_fiber_state(&mut self, mt_length: f64, dt: f64) {
        // Estimate MT velocity from finite difference
        let mt_velocity = if dt > 0.0 {
            (mt_length - self.prev_mt_length) / dt
        } else {
            0.0
        };
        self.prev_mt_length = mt_length;

        self.update_fiber_state_with_velocity(mt_length, mt_velocity, dt);
    }

    /// Update fiber length and velocity with known MT velocity.
    fn update_fiber_state_with_velocity(&mut self, mt_length: f64, mt_velocity: f64, _dt: f64) {
        self.prev_mt_length = mt_length;

        if self.config.rigid_tendon {
            // Rigid tendon: all length change goes to fiber
            self.update_fiber_rigid_tendon(mt_length, mt_velocity);
        } else {
            // Compliant tendon: equilibrium iteration
            self.update_fiber_compliant_tendon(mt_length, mt_velocity);
        }
    }

    /// Update fiber state assuming rigid tendon.
    fn update_fiber_rigid_tendon(&mut self, mt_length: f64, mt_velocity: f64) {
        // L_fiber = (L_mt - L_tendon) / cos(alpha)
        // For rigid tendon: L_tendon = L_slack (tendon doesn't stretch)
        let pennation = self.pennation_angle();
        let cos_alpha = pennation.cos().max(0.1);

        self.fiber.length = ((mt_length - self.config.tendon_slack_length) / cos_alpha).max(0.01);

        // Velocity projection
        self.fiber.velocity = mt_velocity / cos_alpha;
    }

    /// Update fiber state with compliant tendon (equilibrium iteration).
    fn update_fiber_compliant_tendon(&mut self, mt_length: f64, mt_velocity: f64) {
        // Use Newton iteration to find fiber length that satisfies:
        // F_fiber * cos(alpha) = F_tendon
        //
        // This is more accurate but slower.

        let max_iter = 10;
        let tol = 1e-6;

        for _ in 0..max_iter {
            let pennation = self.pennation_angle();
            let cos_alpha = pennation.cos().max(0.1);

            // Tendon length from geometry
            let tendon_length = mt_length - self.fiber.length * cos_alpha;

            // Tendon force from strain
            let tendon_strain =
                (tendon_length - self.config.tendon_slack_length) / self.config.tendon_slack_length;
            let tendon_force = if tendon_strain > 0.0 {
                self.config.max_isometric_force
                    * self.config.tendon_stiffness
                    * tendon_strain.powi(2)
            } else {
                0.0
            };

            // Fiber force (at current estimated velocity)
            let l_norm = self
                .fiber
                .normalized_length(self.config.optimal_fiber_length);
            let v_norm = self.fiber.normalized_velocity(
                self.config.optimal_fiber_length,
                self.config.max_contraction_velocity,
            );
            let a = self.activation.activation();

            let fiber_force_multiplier = self.config.force_curves.evaluate(a, l_norm, v_norm);
            let fiber_force = self.config.max_isometric_force * fiber_force_multiplier * cos_alpha;

            // Force error
            let error = tendon_force - fiber_force;

            if error.abs() < tol {
                break;
            }

            // Newton update (simplified)
            let stiffness = self.config.max_isometric_force
                * self.config.force_curves.active_fl.derivative(l_norm).abs()
                + 1.0;
            let delta_length = error / stiffness;

            self.fiber.length = (self.fiber.length - delta_length * 0.5).clamp(
                0.5 * self.config.optimal_fiber_length,
                1.5 * self.config.optimal_fiber_length,
            );
        }

        // Update velocity using MT velocity and tendon compliance
        let pennation = self.pennation_angle();
        let cos_alpha = pennation.cos().max(0.1);
        self.fiber.velocity = mt_velocity / cos_alpha;
    }

    /// Reset the muscle to initial state.
    pub fn reset(&mut self) {
        self.activation = ActivationState::default();
        self.fiber = FiberState::at_optimal(self.config.optimal_fiber_length);
        self.prev_mt_length = self.config.tendon_slack_length + self.config.optimal_fiber_length;
    }

    /// Get diagnostic information about current muscle state.
    #[must_use]
    pub fn diagnostics(&self) -> MuscleDiagnostics {
        let l_norm = self
            .fiber
            .normalized_length(self.config.optimal_fiber_length);
        let v_norm = self.fiber.normalized_velocity(
            self.config.optimal_fiber_length,
            self.config.max_contraction_velocity,
        );
        let a = self.activation.activation();

        let fl_active = self.config.force_curves.active_fl.evaluate(l_norm);
        let fl_passive = self.config.force_curves.passive_fl.evaluate(l_norm);
        let fv = self.config.force_curves.fv.evaluate(v_norm);

        MuscleDiagnostics {
            activation: a,
            normalized_fiber_length: l_norm,
            normalized_fiber_velocity: v_norm,
            pennation_angle: self.pennation_angle(),
            active_force_length_multiplier: fl_active,
            passive_force_length_multiplier: fl_passive,
            force_velocity_multiplier: fv,
            fiber_force: self.config.max_isometric_force
                * self.config.force_curves.evaluate(a, l_norm, v_norm),
            tendon_force: self.compute_tendon_force(),
        }
    }
}

/// Diagnostic information about muscle state.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MuscleDiagnostics {
    /// Current activation level (0-1).
    pub activation: f64,

    /// Normalized fiber length (L / L_opt).
    pub normalized_fiber_length: f64,

    /// Normalized fiber velocity (v / v_max).
    pub normalized_fiber_velocity: f64,

    /// Current pennation angle (radians).
    pub pennation_angle: f64,

    /// Active force-length multiplier (0-1).
    pub active_force_length_multiplier: f64,

    /// Passive force-length multiplier.
    pub passive_force_length_multiplier: f64,

    /// Force-velocity multiplier.
    pub force_velocity_multiplier: f64,

    /// Fiber force (N).
    pub fiber_force: f64,

    /// Tendon force (N).
    pub tendon_force: f64,
}

/// Result of muscle force computation.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MuscleForceResult {
    /// Total muscle force along line of action (N).
    pub force: f64,

    /// Active force component (N).
    pub active_force: f64,

    /// Passive force component (N).
    pub passive_force: f64,

    /// Joint torque (Nm).
    pub torque: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_muscle_creation() {
        let config = HillMuscleConfig::default();
        let muscle = HillMuscle::new(config);

        assert_relative_eq!(muscle.activation(), 0.01, epsilon = 0.01);
        assert!(muscle.fiber_state().length > 0.0);
    }

    #[test]
    fn test_muscle_activation() {
        let mut muscle = HillMuscle::new(HillMuscleConfig::default());
        let dt = 0.001;

        muscle.set_excitation(1.0);

        // Activation should rise over time
        let a0 = muscle.activation();
        for _ in 0..100 {
            let _ = muscle.compute_torque(0.0, 0.0, dt);
        }
        let a1 = muscle.activation();

        assert!(a1 > a0);
    }

    #[test]
    fn test_muscle_produces_torque() {
        let mut muscle = HillMuscle::new(HillMuscleConfig::default());
        let dt = 0.001;

        // Warm up activation
        muscle.set_excitation(1.0);
        for _ in 0..200 {
            let _ = muscle.compute_torque(0.0, 0.0, dt);
        }

        let torque = muscle.compute_torque(0.0, 0.0, dt);
        assert!(torque > 0.0);
    }

    #[test]
    fn test_biceps_config() {
        let config = HillMuscleConfig::biceps();
        let muscle = HillMuscle::new(config);

        assert!(muscle.config.max_isometric_force > 0.0);
        assert!(muscle.config.optimal_fiber_length > 0.0);
    }

    #[test]
    fn test_pennation_angle() {
        // Muscle with pennation
        let config = HillMuscleConfig {
            pennation_angle_optimal: 0.3, // ~17 degrees
            ..HillMuscleConfig::default()
        };
        let muscle = HillMuscle::new(config);

        let angle = muscle.pennation_angle();
        assert!(angle > 0.0);
        assert!(angle < std::f64::consts::FRAC_PI_2);
    }

    #[test]
    fn test_force_length_sensitivity() {
        let config = HillMuscleConfig::default();
        let mut muscle = HillMuscle::new(config.clone());
        let dt = 0.001;

        // Warm up
        muscle.set_excitation(1.0);
        for _ in 0..200 {
            let _ = muscle.compute_torque(0.0, 0.0, dt);
        }

        // Force at different lengths
        let torque_short = muscle.compute_torque(-0.5, 0.0, dt);
        muscle.reset();
        muscle.set_excitation(1.0);
        for _ in 0..200 {
            let _ = muscle.compute_torque(0.0, 0.0, dt);
        }
        let torque_optimal = muscle.compute_torque(0.0, 0.0, dt);

        // Torque should be different at different lengths
        assert!(torque_optimal != torque_short);
    }

    #[test]
    fn test_diagnostics() {
        let mut muscle = HillMuscle::new(HillMuscleConfig::default());
        muscle.set_excitation(0.5);
        let _ = muscle.compute_torque(0.0, 0.0, 0.001);

        let diag = muscle.diagnostics();

        assert!(diag.activation > 0.0);
        assert!(diag.normalized_fiber_length > 0.0);
        assert!(diag.active_force_length_multiplier >= 0.0);
        assert!(diag.active_force_length_multiplier <= 1.0);
    }
}
