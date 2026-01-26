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

// ============================================================================
// Phase 2: Cartesian Pendulum with Constraint Solving
// ============================================================================
//
// This models a pendulum as a point mass in 2D Cartesian space (x, y) with
// a distance constraint. The constraint solver (PGS) keeps the mass at
// distance L from the pivot.
//
// State representation:
// - Position: (x, y) in meters
// - Velocity: (vx, vy) in m/s
//
// Constraint: x² + y² = L² (distance from origin)
//
// This is a stepping stone to multi-body articulated systems where:
// - Each body has Cartesian state (position + orientation)
// - Joints impose constraints between bodies
// - PGS solver computes constraint forces

use nalgebra::Vector2;

/// A point mass pendulum in 2D Cartesian space with constraint solving.
///
/// Unlike `SimplePendulum` which works in joint-space (θ, θ̇), this works
/// in Cartesian space (x, y, vx, vy) and uses a constraint solver to
/// maintain the distance constraint.
///
/// This demonstrates the constraint-based approach that `MuJoCo` uses for
/// articulated rigid body simulation.
#[derive(Debug, Clone)]
pub struct ConstrainedPendulum {
    /// Position of the mass (x, y) in meters
    pub pos: Vector2<f64>,
    /// Velocity of the mass (vx, vy) in m/s
    pub vel: Vector2<f64>,
    /// Desired distance from pivot (constraint target)
    pub length: f64,
    /// Mass in kg
    pub mass: f64,
    /// Gravitational acceleration in m/s²
    pub gravity: f64,
    /// Baumgarte stabilization coefficient (position error correction)
    pub baumgarte_k: f64,
    /// Baumgarte stabilization coefficient (velocity error correction)
    pub baumgarte_b: f64,
    /// Constraint regularization (softness)
    pub regularization: f64,
}

impl ConstrainedPendulum {
    /// Create a new constrained pendulum with default parameters.
    ///
    /// Initializes at angle θ from vertical (0 = hanging down).
    #[must_use]
    pub fn new(length: f64, mass: f64, initial_angle: f64) -> Self {
        // Convert angle to Cartesian position
        // θ = 0 means hanging straight down: (0, -L)
        // θ > 0 rotates counter-clockwise: (L*sin(θ), -L*cos(θ))
        let x = length * initial_angle.sin();
        let y = -length * initial_angle.cos();

        Self {
            pos: Vector2::new(x, y),
            vel: Vector2::zeros(),
            length,
            mass,
            gravity: 9.81,
            // Baumgarte parameters - tune for stability
            // Higher values = faster correction but can cause instability
            baumgarte_k: 100.0,   // Position error gain
            baumgarte_b: 20.0,    // Velocity error gain
            regularization: 1e-6, // Small regularization for numerical stability
        }
    }

    /// Get the current angle θ from the Cartesian position.
    #[must_use]
    pub fn angle(&self) -> f64 {
        // atan2(x, -y) gives angle from vertical (hanging down)
        self.pos.x.atan2(-self.pos.y)
    }

    /// Get the current angular velocity.
    #[must_use]
    pub fn angular_velocity(&self) -> f64 {
        // ω = (x * vy - y * vx) / r²
        // This is the tangential velocity divided by radius
        let r_sq = self.pos.x * self.pos.x + self.pos.y * self.pos.y;
        if r_sq > 1e-10 {
            (self.pos.x * self.vel.y - self.pos.y * self.vel.x) / r_sq
        } else {
            0.0
        }
    }

    /// Compute the constraint violation (how far from the constraint surface).
    ///
    /// For the distance constraint: C = (x² + y²) - L² = 0
    /// We return C, which should be zero when the constraint is satisfied.
    #[must_use]
    pub fn constraint_violation(&self) -> f64 {
        let r_sq = self.pos.x * self.pos.x + self.pos.y * self.pos.y;
        r_sq - self.length * self.length
    }

    /// Compute the constraint velocity (rate of change of constraint).
    ///
    /// Cdot = d/dt(x² + y² - L²) = 2*(x*vx + y*vy)
    #[must_use]
    pub fn constraint_velocity(&self) -> f64 {
        2.0 * (self.pos.x * self.vel.x + self.pos.y * self.vel.y)
    }

    /// Compute the constraint Jacobian J.
    ///
    /// J = ∂C/∂q = [2x, 2y]
    ///
    /// This maps Cartesian velocities to constraint velocities: Cdot = J * qdot
    #[must_use]
    pub fn constraint_jacobian(&self) -> Vector2<f64> {
        Vector2::new(2.0 * self.pos.x, 2.0 * self.pos.y)
    }

    /// Compute the total mechanical energy.
    ///
    /// E = T + V where:
    /// - T = ½ * m * (vx² + vy²) (kinetic energy)
    /// - V = m * g * (y + L) (potential energy, with y=-L as reference)
    #[must_use]
    pub fn total_energy(&self) -> f64 {
        let kinetic = 0.5 * self.mass * self.vel.norm_squared();
        // Reference: V = 0 when y = -L (hanging straight down)
        let potential = self.mass * self.gravity * (self.pos.y + self.length);
        kinetic + potential
    }

    /// Step the pendulum forward using constraint-based dynamics.
    ///
    /// This follows `MuJoCo`'s approach:
    /// 1. Compute unconstrained acceleration (gravity only)
    /// 2. Compute constraint force via PGS (single iteration for 1 constraint)
    /// 3. Apply constraint force to get final acceleration
    /// 4. Semi-implicit Euler integration
    ///
    /// The constraint equation is: C̈ = J·a + J̇·v = 0 (plus Baumgarte stabilization)
    /// where J̇·v is the "constraint bias" from the changing Jacobian.
    pub fn step(&mut self, dt: f64) {
        // =====================================================================
        // Stage 1-3: Forward Position/Velocity/Actuation
        // =====================================================================

        // Unconstrained acceleration (gravity only)
        // F = m * g in -y direction
        let f_gravity = Vector2::new(0.0, -self.mass * self.gravity);
        let acc_smooth = f_gravity / self.mass; // = (0, -g)

        // =====================================================================
        // Stage 4: Forward Acceleration (Constraint Solving)
        // =====================================================================

        // Constraint: C = x² + y² - L² = 0
        // First derivative: Ċ = 2x·ẋ + 2y·ẏ = J·v  where J = [2x, 2y]
        // Second derivative: C̈ = J·a + J̇·v  where J̇ = [2ẋ, 2ẏ]
        //
        // The constraint equation is: J·a + J̇·v = -K·C - B·Ċ (Baumgarte stabilization)
        // Solving for λ in: a = a_smooth + M⁻¹·J^T·λ
        // We get: J·(a_smooth + M⁻¹·J^T·λ) + J̇·v = -K·C - B·Ċ
        //         J·a_smooth + J·M⁻¹·J^T·λ + J̇·v = -K·C - B·Ċ
        //         A·λ = -K·C - B·Ċ - J·a_smooth - J̇·v
        //         A·λ = -b
        // where A = J·M⁻¹·J^T and b = K·C + B·Ċ + J·a_smooth + J̇·v

        // Constraint Jacobian: J = [2x, 2y]
        let jacobian = self.constraint_jacobian();

        // Jacobian time derivative: J̇ = [2ẋ, 2ẏ]
        let jacobian_dot = Vector2::new(2.0 * self.vel.x, 2.0 * self.vel.y);

        // J̇·v = 2(ẋ² + ẏ²)
        let jdot_times_vel = jacobian_dot.x * self.vel.x + jacobian_dot.y * self.vel.y;

        // Mass matrix is diagonal: M = diag(m, m)
        // M^(-1) = diag(1/m, 1/m)
        let mass_inv = 1.0 / self.mass;

        // Constraint-space inverse inertia: A = J @ M^(-1) @ J^T
        // For 1D constraint: A = (4x² + 4y²) / m = 4r²/m
        let constraint_inertia = (jacobian.x * jacobian.x + jacobian.y * jacobian.y) * mass_inv;

        // Add regularization for numerical stability
        let effective_inertia = constraint_inertia + self.regularization;

        // Baumgarte stabilization terms
        let constraint_err = self.constraint_violation();
        let constraint_vel_err = self.constraint_velocity();

        // Constraint acceleration without correction: J @ acc_smooth
        let jacobian_times_acc = jacobian.x * acc_smooth.x + jacobian.y * acc_smooth.y;

        // RHS of constraint equation:
        // b = K·C + B·Ċ + J·a_smooth + J̇·v
        let rhs = self.baumgarte_k * constraint_err
            + self.baumgarte_b * constraint_vel_err
            + jacobian_times_acc
            + jdot_times_vel;

        // Solve for constraint force (Lagrange multiplier λ)
        // H @ λ = -b  =>  λ = -b / H
        let lambda = -rhs / effective_inertia;

        // Compute constraint force in Cartesian space: f_constraint = J^T @ λ
        let f_constraint = jacobian * lambda;

        // Final acceleration: acc = acc_smooth + M^(-1) @ f_constraint
        let acc = acc_smooth + f_constraint * mass_inv;

        // =====================================================================
        // Stage 5: Integration (Semi-implicit Euler)
        // =====================================================================

        // Update velocity first
        let vel_new = self.vel + acc * dt;

        // Update position using NEW velocity
        let pos_new = self.pos + vel_new * dt;

        self.vel = vel_new;
        self.pos = pos_new;
    }

    /// Run the pendulum for a given duration.
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

    // =========================================================================
    // Phase 2: Constrained Pendulum Tests
    // =========================================================================

    #[test]
    fn test_constrained_pendulum_constraint_maintained() {
        // The constraint x² + y² = L² should be maintained throughout simulation
        let mut pendulum = ConstrainedPendulum::new(1.0, 1.0, PI / 3.0);

        // Run for 10 seconds
        let duration = 10.0;
        let steps = (duration / DT).ceil() as usize;

        let mut max_violation = 0.0_f64;
        for _ in 0..steps {
            pendulum.step(DT);
            let violation = pendulum.constraint_violation().abs();
            max_violation = max_violation.max(violation);
        }

        // The constraint violation should stay small (< 1% of L²)
        let tolerance = 0.01 * pendulum.length * pendulum.length;
        assert!(
            max_violation < tolerance,
            "Constraint violation {:.6} exceeds tolerance {:.6}",
            max_violation,
            tolerance
        );
    }

    #[test]
    fn test_constrained_pendulum_matches_simple() {
        // Both pendulums should produce similar behavior
        let initial_angle = PI / 4.0;
        let mut simple = SimplePendulum::new(1.0, 1.0).with_initial_angle(initial_angle);
        let mut constrained = ConstrainedPendulum::new(1.0, 1.0, initial_angle);

        // Run for 5 seconds and compare angles
        let duration = 5.0;
        let steps = (duration / DT).ceil() as usize;

        let mut max_angle_diff = 0.0_f64;
        for _ in 0..steps {
            simple.step(DT);
            constrained.step(DT);

            let angle_diff = (simple.qpos - constrained.angle()).abs();
            max_angle_diff = max_angle_diff.max(angle_diff);
        }

        // The angles should match reasonably well (within 5 degrees)
        let tolerance = 5.0_f64.to_radians();
        assert!(
            max_angle_diff < tolerance,
            "Angle difference {:.2}° exceeds tolerance {:.2}°",
            max_angle_diff.to_degrees(),
            tolerance.to_degrees()
        );
    }

    #[test]
    fn test_constrained_pendulum_energy_conservation() {
        let mut pendulum = ConstrainedPendulum::new(1.0, 1.0, PI / 3.0);

        let initial_energy = pendulum.total_energy();

        // Run for 10 seconds
        pendulum.run_for(10.0, DT);

        let final_energy = pendulum.total_energy();

        // Energy conservation is harder with Baumgarte stabilization since it
        // adds/removes energy to correct constraint drift. Allow 10% error.
        // In practice, this is still very good for a constraint-based method.
        let relative_error = (final_energy - initial_energy).abs() / initial_energy;
        assert!(
            relative_error < 0.10,
            "Energy error {:.2}% exceeds 10%",
            relative_error * 100.0
        );
    }

    #[test]
    fn test_constrained_pendulum_initial_state() {
        // Verify initial state is correct
        let angle = PI / 4.0;
        let pendulum = ConstrainedPendulum::new(1.0, 1.0, angle);

        // Check position
        let expected_x = angle.sin();
        let expected_y = -angle.cos();
        assert_relative_eq!(pendulum.pos.x, expected_x, epsilon = 1e-10);
        assert_relative_eq!(pendulum.pos.y, expected_y, epsilon = 1e-10);

        // Check initial velocity is zero
        assert_relative_eq!(pendulum.vel.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(pendulum.vel.y, 0.0, epsilon = 1e-10);

        // Check constraint is satisfied
        let r_sq = pendulum.pos.x * pendulum.pos.x + pendulum.pos.y * pendulum.pos.y;
        assert_relative_eq!(r_sq, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_constrained_pendulum_jacobian() {
        // Verify Jacobian is correct
        let pendulum = ConstrainedPendulum::new(1.0, 1.0, PI / 4.0);
        let j = pendulum.constraint_jacobian();

        // J should be [2x, 2y]
        assert_relative_eq!(j.x, 2.0 * pendulum.pos.x, epsilon = 1e-10);
        assert_relative_eq!(j.y, 2.0 * pendulum.pos.y, epsilon = 1e-10);
    }

    #[test]
    fn test_constrained_pendulum_oscillation() {
        // Verify the pendulum oscillates with approximately the right period
        let initial_angle = 0.1; // Small angle for linear approximation
        let mut pendulum = ConstrainedPendulum::new(1.0, 1.0, initial_angle);

        // Expected period from simple pendulum formula
        let expected_period = 2.0 * PI * (1.0 / 9.81_f64).sqrt();

        // Run for one period
        pendulum.run_for(expected_period, DT);

        // Should be back near initial angle
        let final_angle = pendulum.angle();
        assert!(
            (final_angle - initial_angle).abs() < 0.05,
            "Expected θ ≈ {:.3}, got θ = {:.3}",
            initial_angle,
            final_angle
        );
    }
}
