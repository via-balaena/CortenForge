//! MuJoCo-style physics pipeline for articulated rigid body simulation.
//!
//! This module implements the physics pipeline following `MuJoCo`'s exact
//! computation order and algorithms. The goal is binary-compatible physics
//! behavior with `MuJoCo` for validation purposes.
//!
//! # Pipeline Stages
//!
//! Following `MuJoCo`'s `mj_step`:
//!
//! 1. **Forward Position** (`mj_fwdPosition`)
//!    - Forward kinematics: compute body poses from joint positions
//!    - Compute mass matrix M via Composite Rigid Body Algorithm (CRBA)
//!    - Collision detection
//!    - Build constraint Jacobians
//!
//! 2. **Forward Velocity** (`mj_fwdVelocity`)
//!    - Compute bias forces via Recursive Newton-Euler (RNE)
//!    - Compute passive forces (springs, dampers)
//!
//! 3. **Forward Actuation** (`mj_fwdActuation`)
//!    - Compute actuator forces from controls
//!
//! 4. **Forward Acceleration** (`mj_fwdAcceleration`)
//!    - Compute unconstrained acceleration: `qacc_smooth = M^(-1) * qfrc_smooth`
//!    - Solve constraints via PGS
//!    - Compute final acceleration: `qacc = qacc_smooth + M^(-1) * J^T * λ`
//!
//! 5. **Integration** (`mj_Euler`)
//!    - Semi-implicit Euler: velocity then position (using NEW velocity)
//!
//! # Current Status
//!
//! - Phase 1: Single pendulum (1-DOF revolute) - COMPLETE
//! - Phase 2: Constraints (PGS solver) - TODO
//! - Phase 3: Multi-body (CRBA, RNE) - TODO

use std::f64::consts::PI;

/// Simple pendulum state for Phase 1 testing.
///
/// A single pendulum is a 1-DOF system where:
/// - `qpos`: joint angle θ (radians), 0 = hanging down
/// - `qvel`: joint angular velocity θ̇ (rad/s)
#[derive(Debug, Clone)]
pub struct SimplePendulum {
    /// Joint angle (θ) in radians
    pub qpos: f64,
    /// Joint angular velocity (θ̇) in rad/s
    pub qvel: f64,
    /// Pendulum length from pivot to center of mass (m)
    pub length: f64,
    /// Pendulum mass (kg)
    pub mass: f64,
    /// Gravitational acceleration (m/s²)
    pub gravity: f64,
}

impl SimplePendulum {
    /// Create a new pendulum with default parameters.
    #[must_use]
    pub fn new(length: f64, mass: f64) -> Self {
        Self {
            qpos: 0.0,
            qvel: 0.0,
            length,
            mass,
            gravity: 9.81,
        }
    }

    /// Create a pendulum starting at the given angle.
    #[must_use]
    pub fn with_initial_angle(mut self, angle: f64) -> Self {
        self.qpos = angle;
        self
    }

    /// Create a pendulum with the given initial angular velocity.
    #[must_use]
    pub fn with_initial_velocity(mut self, velocity: f64) -> Self {
        self.qvel = velocity;
        self
    }

    /// Create a pendulum with custom gravity.
    #[must_use]
    pub fn with_gravity(mut self, gravity: f64) -> Self {
        self.gravity = gravity;
        self
    }

    /// Compute the effective inertia M for this pendulum.
    ///
    /// For a point mass at distance L from the pivot:
    /// M = m * L²
    #[must_use]
    pub fn inertia(&self) -> f64 {
        self.mass * self.length * self.length
    }

    /// Compute the natural frequency of small oscillations.
    ///
    /// ω = √(g/L)
    #[must_use]
    pub fn natural_frequency(&self) -> f64 {
        (self.gravity / self.length).sqrt()
    }

    /// Compute the period of small oscillations.
    ///
    /// T = 2π√(L/g)
    #[must_use]
    pub fn period(&self) -> f64 {
        2.0 * PI * (self.length / self.gravity).sqrt()
    }

    /// Compute the total mechanical energy.
    ///
    /// E = T + V where:
    /// - T = ½ * I * ω² = ½ * m * L² * θ̇² (kinetic energy)
    /// - V = m * g * h = m * g * L * (1 - cos(θ)) (potential energy)
    #[must_use]
    pub fn total_energy(&self) -> f64 {
        let kinetic = 0.5 * self.mass * self.length * self.length * self.qvel * self.qvel;
        let height = self.length * (1.0 - self.qpos.cos());
        let potential = self.mass * self.gravity * height;
        kinetic + potential
    }

    /// Compute the bias force (gravity torque) using RNE.
    ///
    /// The bias force is the generalized force needed to produce zero acceleration.
    /// For a pendulum: `qfrc_bias` = m * g * L * sin(θ)
    #[must_use]
    pub fn bias_force(&self) -> f64 {
        self.mass * self.gravity * self.length * self.qpos.sin()
    }

    /// Step the pendulum forward by dt using `MuJoCo`'s semi-implicit Euler.
    ///
    /// This follows `MuJoCo`'s `mj_Euler` exactly:
    /// 1. Compute acceleration: `qacc = M⁻¹ * (-qfrc_bias)`
    /// 2. Update velocity: `qvel_new = qvel + dt * qacc`
    /// 3. Update position: `qpos_new = qpos + dt * qvel_new` (uses NEW velocity!)
    pub fn step(&mut self, dt: f64) {
        let m_inv = 1.0 / self.inertia();
        let qfrc_bias = self.bias_force();

        // qfrc_smooth = qfrc_passive + qfrc_actuator + qfrc_applied - qfrc_bias
        // For a free pendulum with no actuation: qfrc_smooth = -qfrc_bias
        let qfrc_smooth = -qfrc_bias;

        // qacc = M⁻¹ * qfrc_smooth
        let qacc = m_inv * qfrc_smooth;

        // Semi-implicit Euler: velocity first, then position with NEW velocity
        let qvel_new = self.qvel + dt * qacc;
        let qpos_new = self.qpos + dt * qvel_new;

        self.qvel = qvel_new;
        self.qpos = qpos_new;
    }

    /// Run the pendulum for a given duration.
    ///
    /// Returns the final state after simulation.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub fn run_for(&mut self, duration: f64, dt: f64) {
        let steps = (duration / dt).ceil() as usize;
        for _ in 0..steps {
            self.step(dt);
        }
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    const DT: f64 = 1.0 / 240.0; // MuJoCo default timestep

    #[test]
    fn test_pendulum_period() {
        // For L=1m, g=9.81, T = 2π√(1/9.81) ≈ 2.006s
        let pendulum = SimplePendulum::new(1.0, 1.0);
        let expected_period = 2.0 * PI * (1.0 / 9.81_f64).sqrt();
        assert_relative_eq!(pendulum.period(), expected_period, epsilon = 1e-10);
    }

    #[test]
    fn test_pendulum_small_angle_oscillation() {
        // For small angles, the pendulum should complete one period in T = 2π√(L/g) seconds
        let mut pendulum = SimplePendulum::new(1.0, 1.0).with_initial_angle(0.1); // 0.1 rad ≈ 5.7°

        let initial_angle = pendulum.qpos;
        let period = pendulum.period();

        // Run for exactly one period
        pendulum.run_for(period, DT);

        // Should be back near the initial angle (small angle approximation)
        // Allow some error due to discrete timesteps and non-linearity
        assert_relative_eq!(pendulum.qpos, initial_angle, epsilon = 0.02);
    }

    #[test]
    fn test_pendulum_energy_conservation() {
        // Start at 60 degrees - a significant angle
        let initial_angle = PI / 3.0;
        let mut pendulum = SimplePendulum::new(1.0, 1.0).with_initial_angle(initial_angle);

        let initial_energy = pendulum.total_energy();

        // Run for 10 seconds
        pendulum.run_for(10.0, DT);

        let final_energy = pendulum.total_energy();

        // Energy should be conserved to within 1%
        let relative_error = (final_energy - initial_energy).abs() / initial_energy;
        assert!(
            relative_error < 0.01,
            "Energy error {:.2}% exceeds 1%",
            relative_error * 100.0
        );
    }

    #[test]
    fn test_pendulum_equilibrium() {
        // A pendulum at θ=0 with zero velocity should stay at rest
        let mut pendulum = SimplePendulum::new(1.0, 1.0);

        let initial_pos = pendulum.qpos;
        let initial_vel = pendulum.qvel;

        pendulum.run_for(5.0, DT);

        assert_relative_eq!(pendulum.qpos, initial_pos, epsilon = 1e-10);
        assert_relative_eq!(pendulum.qvel, initial_vel, epsilon = 1e-10);
    }

    #[test]
    fn test_pendulum_falls_from_horizontal() {
        // Start at 90 degrees (horizontal) - should swing down
        let mut pendulum = SimplePendulum::new(1.0, 1.0).with_initial_angle(PI / 2.0);

        // For large angles, the period is longer than the small-angle approximation.
        // At 90°, the actual period is about 18% longer than 2π√(L/g).
        // We'll track the minimum position to verify it passes near θ=0.
        let mut min_pos = pendulum.qpos;
        let mut max_velocity = pendulum.qvel.abs();

        // Run for a while, tracking extremes
        let run_time = pendulum.period() * 0.6; // More than quarter, less than half
        let steps = (run_time / DT).ceil() as usize;

        for _ in 0..steps {
            pendulum.step(DT);
            if pendulum.qpos.abs() < min_pos.abs() {
                min_pos = pendulum.qpos;
            }
            if pendulum.qvel.abs() > max_velocity {
                max_velocity = pendulum.qvel.abs();
            }
        }

        // Should have passed near θ=0 at some point
        assert!(
            min_pos.abs() < 0.1,
            "Pendulum should pass near bottom, minimum distance was {}",
            min_pos.abs()
        );

        // Should achieve significant velocity at some point
        assert!(
            max_velocity > 3.0,
            "Pendulum should achieve significant velocity, max was {max_velocity}",
        );
    }

    #[test]
    fn test_pendulum_symmetric_swing() {
        // Starting at +45 degrees, it should swing to approximately -45 degrees
        let initial_angle = PI / 4.0;
        let mut pendulum = SimplePendulum::new(1.0, 1.0).with_initial_angle(initial_angle);

        // Run for half period - should be at opposite extreme
        let half_period = pendulum.period() / 2.0;
        pendulum.run_for(half_period, DT);

        // Should be at approximately -initial_angle
        // (not exact due to non-linear pendulum equation, but close for small-medium angles)
        assert!(
            (pendulum.qpos + initial_angle).abs() < 0.15,
            "Expected θ ≈ {:.2}, got θ = {:.2}",
            -initial_angle,
            pendulum.qpos
        );
    }

    #[test]
    fn test_pendulum_velocity_at_bottom() {
        // From energy conservation: at bottom, all PE becomes KE
        // ½ * m * L² * ω² = m * g * L * (1 - cos(θ₀))
        // ω = √(2g/L * (1 - cos(θ₀)))
        let initial_angle = PI / 4.0;
        let mut pendulum = SimplePendulum::new(1.0, 1.0).with_initial_angle(initial_angle);

        let expected_max_velocity =
            (2.0 * pendulum.gravity / pendulum.length * (1.0 - initial_angle.cos())).sqrt();

        // Run for quarter period to reach bottom
        let quarter_period = pendulum.period() / 4.0;
        pendulum.run_for(quarter_period, DT);

        // The velocity magnitude should match theoretical (approximately)
        // Note: the sign may be negative depending on swing direction
        assert!(
            (pendulum.qvel.abs() - expected_max_velocity).abs() < 0.1,
            "Expected |ω| ≈ {:.2}, got ω = {:.2}",
            expected_max_velocity,
            pendulum.qvel
        );
    }

    #[test]
    fn test_pendulum_long_term_stability() {
        // Run for 60 seconds and verify energy is still conserved
        let initial_angle = PI / 3.0;
        let mut pendulum = SimplePendulum::new(1.0, 1.0).with_initial_angle(initial_angle);

        let initial_energy = pendulum.total_energy();

        // Run for 60 seconds (about 30 periods)
        pendulum.run_for(60.0, DT);

        let final_energy = pendulum.total_energy();

        // Energy should still be conserved to within 2% after a long run
        let relative_error = (final_energy - initial_energy).abs() / initial_energy;
        assert!(
            relative_error < 0.02,
            "Long-term energy error {:.2}% exceeds 2%",
            relative_error * 100.0
        );
    }

    #[test]
    fn test_pendulum_different_parameters() {
        // Test with different length and mass
        let mut pendulum = SimplePendulum::new(2.0, 0.5).with_initial_angle(0.5);

        let initial_energy = pendulum.total_energy();
        let period = pendulum.period();

        // Verify period is correct: T = 2π√(2/9.81) ≈ 2.838s
        let expected_period = 2.0 * PI * (2.0 / 9.81_f64).sqrt();
        assert_relative_eq!(period, expected_period, epsilon = 1e-10);

        // Run for one period
        pendulum.run_for(period, DT);

        // Should return near initial position
        assert!(
            (pendulum.qpos - 0.5).abs() < 0.02,
            "Expected θ ≈ 0.5, got θ = {:.3}",
            pendulum.qpos
        );

        // Energy should be conserved
        let final_energy = pendulum.total_energy();
        let relative_error = (final_energy - initial_energy).abs() / initial_energy;
        assert!(
            relative_error < 0.01,
            "Energy error {:.2}% exceeds 1%",
            relative_error * 100.0
        );
    }
}
