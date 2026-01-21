//! Muscle activation dynamics.
//!
//! Activation dynamics model the delay and filtering between neural excitation
//! (the control signal) and muscle activation (the actual contractile state).
//!
//! # Background
//!
//! When a motor neuron fires, it takes time for calcium to be released and
//! bind to troponin, allowing myosin heads to attach to actin. This process
//! is modeled as a first-order differential equation with different time
//! constants for activation (rising) and deactivation (falling).
//!
//! # Model
//!
//! The standard activation dynamics model is:
//!
//! ```text
//! da/dt = (u - a) / τ(u, a)
//!
//! where:
//!   a = activation level (0 to 1)
//!   u = neural excitation (0 to 1)
//!   τ = time constant (depends on whether activating or deactivating)
//! ```
//!
//! Activation is faster than deactivation:
//! - τ_act ≈ 10-20 ms (activation)
//! - τ_deact ≈ 40-80 ms (deactivation)

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Configuration for muscle activation dynamics.
///
/// Models the neural excitation-to-activation transformation with
/// asymmetric time constants for activation and deactivation.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ActivationDynamics {
    /// Time constant for activation (rising), in seconds.
    /// Typical range: 0.01 - 0.02 s (10-20 ms).
    pub tau_activation: f64,

    /// Time constant for deactivation (falling), in seconds.
    /// Typical range: 0.04 - 0.08 s (40-80 ms).
    pub tau_deactivation: f64,

    /// Minimum activation level to prevent numerical issues.
    /// Typical value: 0.01 (1% baseline activation).
    pub min_activation: f64,
}

impl Default for ActivationDynamics {
    fn default() -> Self {
        Self {
            tau_activation: 0.015,   // 15 ms
            tau_deactivation: 0.060, // 60 ms
            min_activation: 0.01,    // 1% baseline
        }
    }
}

impl ActivationDynamics {
    /// Create activation dynamics with custom time constants.
    ///
    /// # Arguments
    ///
    /// * `tau_act` - Activation time constant (seconds)
    /// * `tau_deact` - Deactivation time constant (seconds)
    #[must_use]
    pub fn new(tau_act: f64, tau_deact: f64) -> Self {
        Self {
            tau_activation: tau_act.max(0.001),
            tau_deactivation: tau_deact.max(0.001),
            min_activation: 0.01,
        }
    }

    /// Create fast-twitch muscle activation dynamics.
    ///
    /// Fast-twitch (Type II) fibers have faster activation/deactivation.
    #[must_use]
    pub fn fast_twitch() -> Self {
        Self {
            tau_activation: 0.010,   // 10 ms
            tau_deactivation: 0.040, // 40 ms
            min_activation: 0.01,
        }
    }

    /// Create slow-twitch muscle activation dynamics.
    ///
    /// Slow-twitch (Type I) fibers have slower activation/deactivation.
    #[must_use]
    pub fn slow_twitch() -> Self {
        Self {
            tau_activation: 0.020,   // 20 ms
            tau_deactivation: 0.080, // 80 ms
            min_activation: 0.01,
        }
    }

    /// Set the minimum activation level.
    #[must_use]
    pub fn with_min_activation(mut self, min_activation: f64) -> Self {
        self.min_activation = min_activation.clamp(0.0, 0.5);
        self
    }

    /// Compute the effective time constant based on whether activating or deactivating.
    ///
    /// Uses the asymmetric model where activation is faster than deactivation.
    #[must_use]
    pub fn time_constant(&self, excitation: f64, activation: f64) -> f64 {
        if excitation > activation {
            // Activating: use faster time constant
            self.tau_activation
        } else {
            // Deactivating: use slower time constant
            self.tau_deactivation
        }
    }

    /// Compute the activation derivative (da/dt).
    ///
    /// # Arguments
    ///
    /// * `excitation` - Neural excitation signal (0 to 1)
    /// * `activation` - Current activation level (0 to 1)
    ///
    /// # Returns
    ///
    /// The rate of change of activation (1/s).
    #[must_use]
    pub fn derivative(&self, excitation: f64, activation: f64) -> f64 {
        let u = excitation.clamp(0.0, 1.0);
        let a = activation.clamp(self.min_activation, 1.0);
        let tau = self.time_constant(u, a);

        (u - a) / tau
    }

    /// Update activation using semi-implicit Euler integration.
    ///
    /// This method is unconditionally stable for any timestep.
    ///
    /// # Arguments
    ///
    /// * `excitation` - Neural excitation signal (0 to 1)
    /// * `activation` - Current activation level (0 to 1)
    /// * `dt` - Timestep (seconds)
    ///
    /// # Returns
    ///
    /// The new activation level.
    #[must_use]
    pub fn integrate(&self, excitation: f64, activation: f64, dt: f64) -> f64 {
        let u = excitation.clamp(0.0, 1.0);
        let a = activation.clamp(self.min_activation, 1.0);
        let tau = self.time_constant(u, a);

        // Semi-implicit Euler: a_new = (a + dt * u / tau) / (1 + dt / tau)
        // This is equivalent to solving: a_new = a + dt * (u - a_new) / tau
        let new_activation = (a + dt * u / tau) / (1.0 + dt / tau);

        new_activation.clamp(self.min_activation, 1.0)
    }
}

/// State of muscle activation.
///
/// Tracks the current activation level and provides methods for updating it.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ActivationState {
    /// Current activation level (0 to 1).
    pub activation: f64,

    /// Current neural excitation (0 to 1).
    pub excitation: f64,
}

impl Default for ActivationState {
    fn default() -> Self {
        Self {
            activation: 0.01, // Minimum baseline
            excitation: 0.0,
        }
    }
}

impl ActivationState {
    /// Create a new activation state with the given initial activation.
    #[must_use]
    pub fn new(activation: f64) -> Self {
        Self {
            activation: activation.clamp(0.0, 1.0),
            excitation: activation.clamp(0.0, 1.0),
        }
    }

    /// Create a fully activated state.
    #[must_use]
    pub fn fully_activated() -> Self {
        Self {
            activation: 1.0,
            excitation: 1.0,
        }
    }

    /// Set the neural excitation command.
    pub fn set_excitation(&mut self, excitation: f64) {
        self.excitation = excitation.clamp(0.0, 1.0);
    }

    /// Update the activation state.
    ///
    /// # Arguments
    ///
    /// * `dynamics` - The activation dynamics parameters
    /// * `dt` - Timestep (seconds)
    pub fn update(&mut self, dynamics: &ActivationDynamics, dt: f64) {
        self.activation = dynamics.integrate(self.excitation, self.activation, dt);
    }

    /// Get the current activation level.
    #[must_use]
    pub fn activation(&self) -> f64 {
        self.activation
    }

    /// Check if the muscle is essentially inactive.
    #[must_use]
    pub fn is_inactive(&self, threshold: f64) -> bool {
        self.activation < threshold && self.excitation < threshold
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_activation_dynamics_default() {
        let dynamics = ActivationDynamics::default();

        assert!(dynamics.tau_activation > 0.0);
        assert!(dynamics.tau_deactivation > dynamics.tau_activation);
        assert!(dynamics.min_activation > 0.0);
    }

    #[test]
    fn test_asymmetric_time_constants() {
        let dynamics = ActivationDynamics::default();

        // Activating: excitation > activation
        let tau_up = dynamics.time_constant(1.0, 0.5);
        assert_relative_eq!(tau_up, dynamics.tau_activation, epsilon = 1e-10);

        // Deactivating: excitation < activation
        let tau_down = dynamics.time_constant(0.0, 0.5);
        assert_relative_eq!(tau_down, dynamics.tau_deactivation, epsilon = 1e-10);
    }

    #[test]
    fn test_activation_increases_with_excitation() {
        let dynamics = ActivationDynamics::default();
        let dt = 0.001; // 1 ms

        let a0 = 0.1;
        let a1 = dynamics.integrate(1.0, a0, dt);

        // Activation should increase toward excitation
        assert!(a1 > a0);
        assert!(a1 < 1.0);
    }

    #[test]
    fn test_activation_decreases_without_excitation() {
        let dynamics = ActivationDynamics::default();
        let dt = 0.001;

        let a0 = 0.5;
        let a1 = dynamics.integrate(0.0, a0, dt);

        // Activation should decrease toward min_activation
        assert!(a1 < a0);
        assert!(a1 >= dynamics.min_activation);
    }

    #[test]
    fn test_activation_converges_to_excitation() {
        let dynamics = ActivationDynamics::default();
        let dt = 0.001;
        let target = 0.7;

        let mut a = 0.1;
        for _ in 0..1000 {
            a = dynamics.integrate(target, a, dt);
        }

        // Should converge close to target after 1 second
        assert_relative_eq!(a, target, epsilon = 0.01);
    }

    #[test]
    fn test_activation_state_update() {
        let dynamics = ActivationDynamics::default();
        let mut state = ActivationState::new(0.1);

        state.set_excitation(1.0);
        state.update(&dynamics, 0.001);

        assert!(state.activation > 0.1);
        assert!(state.activation < 1.0);
    }

    #[test]
    fn test_activation_clamping() {
        let dynamics = ActivationDynamics::default();

        // Excitation out of bounds should be clamped
        let a1 = dynamics.integrate(2.0, 0.5, 0.001);
        assert!(a1 <= 1.0);

        let a2 = dynamics.integrate(-1.0, 0.5, 0.001);
        assert!(a2 >= dynamics.min_activation);
    }

    #[test]
    fn test_fast_twitch_is_faster() {
        let fast = ActivationDynamics::fast_twitch();
        let slow = ActivationDynamics::slow_twitch();

        assert!(fast.tau_activation < slow.tau_activation);
        assert!(fast.tau_deactivation < slow.tau_deactivation);
    }
}
