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
//! - Phase 2: Constrained pendulum (PGS-style constraint solver) - COMPLETE
//! - Phase 3: Double pendulum (2-DOF, CRBA + RNE) - COMPLETE
//! - Phase 4: Multi-body (n-DOF, general articulated bodies) - TODO
//! - Phase 5: Ball joints (3-DOF spherical joints) - TODO
//! - Phase 6: Contact (ground plane, sphere collisions) - TODO

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

// ============================================================================
// Phase 3: Double Pendulum (Two-Link Chain)
// ============================================================================
//
// This implements a double pendulum following MuJoCo's approach:
// - Joint-space representation: (θ₁, θ₂, θ̇₁, θ̇₂)
// - 2×2 inertia matrix M(θ) computed via CRBA
// - Bias forces (Coriolis + gravity) computed via RNE
// - Semi-implicit Euler integration
//
// The double pendulum is a classic chaotic system - small differences in
// initial conditions lead to vastly different trajectories. However, it
// should conserve energy and the links should stay connected.

use nalgebra::Matrix2;

/// A double pendulum (two-link chain) in joint space.
///
/// This is the first multi-DOF system, requiring:
/// - Full 2×2 inertia matrix M(θ) via Composite Rigid Body Algorithm
/// - Coriolis and gravitational bias forces via Recursive Newton-Euler
/// - Matrix solve for accelerations: M @ qacc = `qfrc_smooth`
///
/// The double pendulum exhibits chaotic behavior - it's very sensitive
/// to initial conditions, but should conserve energy.
#[derive(Debug, Clone)]
pub struct DoublePendulum {
    /// Joint angles [θ₁, θ₂] in radians
    /// θ₁: angle of first link from vertical
    /// θ₂: angle of second link relative to first link
    pub qpos: Vector2<f64>,
    /// Joint angular velocities [θ̇₁, θ̇₂] in rad/s
    pub qvel: Vector2<f64>,
    /// Length of first link (m)
    pub length1: f64,
    /// Length of second link (m)
    pub length2: f64,
    /// Mass of first link (kg) - treated as point mass at end of link
    pub mass1: f64,
    /// Mass of second link (kg) - treated as point mass at end of link
    pub mass2: f64,
    /// Gravitational acceleration (m/s²)
    pub gravity: f64,
}

impl DoublePendulum {
    /// Create a new double pendulum with equal link lengths and masses.
    #[must_use]
    pub fn new(length: f64, mass: f64) -> Self {
        Self {
            qpos: Vector2::zeros(),
            qvel: Vector2::zeros(),
            length1: length,
            length2: length,
            mass1: mass,
            mass2: mass,
            gravity: 9.81,
        }
    }

    /// Create a double pendulum with different parameters for each link.
    #[must_use]
    pub fn with_parameters(length1: f64, length2: f64, mass1: f64, mass2: f64) -> Self {
        Self {
            qpos: Vector2::zeros(),
            qvel: Vector2::zeros(),
            length1,
            length2,
            mass1,
            mass2,
            gravity: 9.81,
        }
    }

    /// Set initial joint angles.
    #[must_use]
    pub fn with_angles(mut self, theta1: f64, theta2: f64) -> Self {
        self.qpos = Vector2::new(theta1, theta2);
        self
    }

    /// Set initial joint velocities.
    #[must_use]
    pub fn with_velocities(mut self, omega1: f64, omega2: f64) -> Self {
        self.qvel = Vector2::new(omega1, omega2);
        self
    }

    /// Compute the position of the first mass in Cartesian coordinates.
    ///
    /// Convention: θ=0 is hanging straight down (along -y axis)
    #[must_use]
    pub fn position1(&self) -> Vector2<f64> {
        let theta1 = self.qpos[0];
        Vector2::new(self.length1 * theta1.sin(), -self.length1 * theta1.cos())
    }

    /// Compute the position of the second mass in Cartesian coordinates.
    #[must_use]
    #[allow(clippy::similar_names)] // theta12 = theta1 + theta2 is intentional
    pub fn position2(&self) -> Vector2<f64> {
        let theta1 = self.qpos[0];
        let theta2 = self.qpos[1];
        let theta12 = theta1 + theta2;

        let pos1 = self.position1();
        Vector2::new(
            pos1.x + self.length2 * theta12.sin(),
            pos1.y - self.length2 * theta12.cos(),
        )
    }

    /// Compute the 2×2 inertia matrix M(θ) via CRBA.
    ///
    /// For a double pendulum with point masses at the ends of massless rods:
    ///
    /// M₁₁ = (m₁ + m₂)L₁² + m₂L₂² + 2m₂L₁L₂cos(θ₂)
    /// M₁₂ = M₂₁ = m₂L₂² + m₂L₁L₂cos(θ₂)
    /// M₂₂ = m₂L₂²
    ///
    /// This comes from the CRBA: we accumulate the inertia of the subtree
    /// (second link) onto the first link, accounting for the relative angle.
    #[must_use]
    pub fn inertia_matrix(&self) -> Matrix2<f64> {
        let m1 = self.mass1;
        let m2 = self.mass2;
        let l1 = self.length1;
        let l2 = self.length2;
        let cos_theta2 = self.qpos[1].cos();

        let m11 = (m1 + m2) * l1 * l1 + m2 * l2 * l2 + 2.0 * m2 * l1 * l2 * cos_theta2;
        let m12 = m2 * l2 * l2 + m2 * l1 * l2 * cos_theta2;
        let m22 = m2 * l2 * l2;

        Matrix2::new(m11, m12, m12, m22)
    }

    /// Compute the bias forces (Coriolis + gravity) via RNE.
    ///
    /// The bias forces include:
    /// - Coriolis/centrifugal terms from the velocities
    /// - Gravitational torques
    ///
    /// C₁ = -m₂L₁L₂sin(θ₂)(2θ̇₁θ̇₂ + θ̇₂²) + (m₁+m₂)gL₁sin(θ₁) + m₂gL₂sin(θ₁+θ₂)
    /// C₂ = m₂L₁L₂sin(θ₂)θ̇₁² + m₂gL₂sin(θ₁+θ₂)
    #[must_use]
    #[allow(clippy::similar_names)] // sin_theta12 = sin(theta1 + theta2) is intentional
    pub fn bias_forces(&self) -> Vector2<f64> {
        let m1 = self.mass1;
        let m2 = self.mass2;
        let l1 = self.length1;
        let l2 = self.length2;
        let g = self.gravity;

        let theta1 = self.qpos[0];
        let theta2 = self.qpos[1];
        let omega1 = self.qvel[0];
        let omega2 = self.qvel[1];

        let sin_theta1 = theta1.sin();
        let sin_theta2 = theta2.sin();
        let sin_theta12 = (theta1 + theta2).sin();

        // Coriolis/centrifugal terms
        let coriolis1 = -m2 * l1 * l2 * sin_theta2 * (2.0 * omega1 * omega2 + omega2 * omega2);
        let coriolis2 = m2 * l1 * l2 * sin_theta2 * omega1 * omega1;

        // Gravitational terms
        let gravity1 = (m1 + m2) * g * l1 * sin_theta1 + m2 * g * l2 * sin_theta12;
        let gravity2 = m2 * g * l2 * sin_theta12;

        // Total bias force = Coriolis + Gravity
        // Note: In MuJoCo convention, we SUBTRACT this from applied forces
        Vector2::new(coriolis1 + gravity1, coriolis2 + gravity2)
    }

    /// Compute the total mechanical energy.
    ///
    /// E = T + V where:
    /// - T = ½ q̇ᵀ M q̇ (kinetic energy)
    /// - V = potential energy from height of both masses
    #[must_use]
    pub fn total_energy(&self) -> f64 {
        // Kinetic energy: T = ½ q̇ᵀ M q̇
        let inertia = self.inertia_matrix();
        let kinetic = 0.5 * self.qvel.dot(&(inertia * self.qvel));

        // Potential energy: V = m₁gh₁ + m₂gh₂
        // Reference: V = 0 when both masses are at lowest point (θ₁=θ₂=0)
        let pos1 = self.position1();
        let pos2 = self.position2();

        // Heights relative to lowest possible position
        // For mass 1: lowest is y = -L₁, so h₁ = y₁ + L₁
        // For mass 2: lowest is y = -L₁ - L₂, so h₂ = y₂ + L₁ + L₂
        let h1 = pos1.y + self.length1;
        let h2 = pos2.y + self.length1 + self.length2;

        let potential = self.mass1 * self.gravity * h1 + self.mass2 * self.gravity * h2;

        kinetic + potential
    }

    /// Step the double pendulum forward using `MuJoCo`'s approach.
    ///
    /// 1. Compute inertia matrix M(θ) via CRBA
    /// 2. Compute bias forces via RNE
    /// 3. Solve M @ qacc = -bias for accelerations
    /// 4. Semi-implicit Euler integration
    pub fn step(&mut self, dt: f64) {
        // =====================================================================
        // Stage 1: Forward Position (compute M via CRBA)
        // =====================================================================
        let inertia = self.inertia_matrix();

        // =====================================================================
        // Stage 2: Forward Velocity (compute bias via RNE)
        // =====================================================================
        let qfrc_bias = self.bias_forces();

        // =====================================================================
        // Stage 3-4: Forward Actuation & Acceleration
        // =====================================================================
        // qfrc_smooth = qfrc_passive + qfrc_actuator + qfrc_applied - qfrc_bias
        // For free pendulum: qfrc_smooth = -qfrc_bias
        let qfrc_smooth = -qfrc_bias;

        // Solve M @ qacc = qfrc_smooth for qacc
        // For a 2x2 system, we can use direct inversion
        let qacc = inertia
            .try_inverse()
            .map_or_else(Vector2::zeros, |m_inv| m_inv * qfrc_smooth);

        // =====================================================================
        // Stage 5: Integration (Semi-implicit Euler)
        // =====================================================================
        let qvel_new = self.qvel + qacc * dt;
        let qpos_new = self.qpos + qvel_new * dt;

        self.qvel = qvel_new;
        self.qpos = qpos_new;
    }

    /// Run the double pendulum for a given duration.
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

    // =========================================================================
    // Phase 3: Double Pendulum Tests
    // =========================================================================

    #[test]
    fn test_double_pendulum_inertia_matrix_symmetric() {
        // The inertia matrix must always be symmetric
        let pendulum = DoublePendulum::new(1.0, 1.0).with_angles(PI / 4.0, PI / 6.0);
        let inertia = pendulum.inertia_matrix();

        assert_relative_eq!(inertia[(0, 1)], inertia[(1, 0)], epsilon = 1e-10);
    }

    #[test]
    fn test_double_pendulum_inertia_matrix_positive_definite() {
        // The inertia matrix must be positive definite (all eigenvalues > 0)
        let pendulum = DoublePendulum::new(1.0, 1.0).with_angles(PI / 3.0, PI / 5.0);
        let inertia = pendulum.inertia_matrix();

        // For a 2x2 symmetric matrix, positive definite iff:
        // - M₁₁ > 0
        // - det(M) > 0
        assert!(inertia[(0, 0)] > 0.0, "M₁₁ should be positive");
        let det = inertia[(0, 0)] * inertia[(1, 1)] - inertia[(0, 1)] * inertia[(1, 0)];
        assert!(det > 0.0, "Determinant should be positive, got {det}");
    }

    #[test]
    fn test_double_pendulum_energy_conservation() {
        // Energy should be conserved in a free double pendulum
        // Use smaller initial angles to reduce chaotic behavior
        let mut pendulum = DoublePendulum::new(1.0, 1.0).with_angles(PI / 6.0, PI / 8.0);

        let initial_energy = pendulum.total_energy();

        // Run for 10 seconds at higher resolution
        let high_res_dt = 1.0 / 480.0; // 480 Hz for better accuracy
        pendulum.run_for(10.0, high_res_dt);

        let final_energy = pendulum.total_energy();

        // Double pendulum is more numerically challenging; allow 1% energy drift
        let relative_error = (final_energy - initial_energy).abs() / initial_energy;
        assert!(
            relative_error < 0.01,
            "Energy error {:.2}% exceeds 1%",
            relative_error * 100.0
        );
    }

    #[test]
    fn test_double_pendulum_equilibrium() {
        // At θ₁=θ₂=0 with zero velocity, the double pendulum should stay at rest
        let mut pendulum = DoublePendulum::new(1.0, 1.0);

        let initial_pos = pendulum.qpos;
        let initial_vel = pendulum.qvel;

        pendulum.run_for(5.0, DT);

        assert_relative_eq!(pendulum.qpos[0], initial_pos[0], epsilon = 1e-10);
        assert_relative_eq!(pendulum.qpos[1], initial_pos[1], epsilon = 1e-10);
        assert_relative_eq!(pendulum.qvel[0], initial_vel[0], epsilon = 1e-10);
        assert_relative_eq!(pendulum.qvel[1], initial_vel[1], epsilon = 1e-10);
    }

    #[test]
    fn test_double_pendulum_links_connected() {
        // The position2 should be reachable from position1 by the second link
        let mut pendulum = DoublePendulum::new(1.0, 1.0).with_angles(PI / 3.0, PI / 4.0);

        // Run simulation and verify links stay connected
        let duration = 10.0;
        let steps = (duration / DT).ceil() as usize;

        for _ in 0..steps {
            pendulum.step(DT);

            let pos1 = pendulum.position1();
            let pos2 = pendulum.position2();

            // Distance from pos1 to pos2 should equal length2
            let link2_length = (pos2 - pos1).norm();
            assert!(
                (link2_length - pendulum.length2).abs() < 1e-10,
                "Second link length mismatch: expected {}, got {}",
                pendulum.length2,
                link2_length
            );

            // Distance from origin to pos1 should equal length1
            let link1_length = pos1.norm();
            assert!(
                (link1_length - pendulum.length1).abs() < 1e-10,
                "First link length mismatch: expected {}, got {}",
                pendulum.length1,
                link1_length
            );
        }
    }

    #[test]
    fn test_double_pendulum_falls_from_horizontal() {
        // Starting with first link horizontal, it should fall
        let mut pendulum = DoublePendulum::new(1.0, 1.0).with_angles(PI / 2.0, 0.0);

        let initial_velocity = pendulum.qvel[0].abs();

        // Run for 0.5 seconds
        pendulum.run_for(0.5, DT);

        // First joint should have gained significant velocity
        assert!(
            pendulum.qvel[0].abs() > initial_velocity + 1.0,
            "First joint should have fallen and gained velocity"
        );
    }

    #[test]
    fn test_double_pendulum_small_angle_first_link_only() {
        // With θ₂=0 and small θ₁, behavior should be close to simple pendulum
        let initial_angle = 0.1; // Small angle
        let mut double = DoublePendulum::new(1.0, 1.0).with_angles(initial_angle, 0.0);

        // The effective length is L1 + L2 = 2.0 for oscillation of the compound system
        // But behavior is more complex - just verify it oscillates
        let mut found_sign_change = false;
        let steps = (5.0 / DT).ceil() as usize;

        for _ in 0..steps {
            let old_pos = double.qpos[0];
            double.step(DT);
            if old_pos * double.qpos[0] < 0.0 {
                found_sign_change = true;
                break;
            }
        }

        assert!(found_sign_change, "Double pendulum should oscillate");
    }

    #[test]
    fn test_double_pendulum_position_formulas() {
        // Verify position1 and position2 are computed correctly
        let theta1 = PI / 4.0;
        let theta2 = PI / 6.0;
        let l1 = 1.0;
        let l2 = 0.8;

        let pendulum =
            DoublePendulum::with_parameters(l1, l2, 1.0, 1.0).with_angles(theta1, theta2);

        let pos1 = pendulum.position1();
        let pos2 = pendulum.position2();

        // Position 1: (L₁sin(θ₁), -L₁cos(θ₁))
        let expected_x1 = l1 * theta1.sin();
        let expected_y1 = -l1 * theta1.cos();
        assert_relative_eq!(pos1.x, expected_x1, epsilon = 1e-10);
        assert_relative_eq!(pos1.y, expected_y1, epsilon = 1e-10);

        // Position 2: pos1 + (L₂sin(θ₁+θ₂), -L₂cos(θ₁+θ₂))
        let theta12 = theta1 + theta2;
        let expected_x2 = expected_x1 + l2 * theta12.sin();
        let expected_y2 = expected_y1 - l2 * theta12.cos();
        assert_relative_eq!(pos2.x, expected_x2, epsilon = 1e-10);
        assert_relative_eq!(pos2.y, expected_y2, epsilon = 1e-10);
    }

    #[test]
    fn test_double_pendulum_long_term_stability() {
        // Run for 30 seconds and verify energy is still reasonable
        // Use smaller angles to reduce chaos and higher resolution
        let mut pendulum = DoublePendulum::new(1.0, 1.0).with_angles(PI / 6.0, PI / 8.0);

        let initial_energy = pendulum.total_energy();

        // Run for 30 seconds at high resolution
        let high_res_dt = 1.0 / 480.0;
        pendulum.run_for(30.0, high_res_dt);

        let final_energy = pendulum.total_energy();

        // Allow 2% energy drift over long term with high-res integration
        let relative_error = (final_energy - initial_energy).abs() / initial_energy;
        assert!(
            relative_error < 0.02,
            "Long-term energy error {:.2}% exceeds 2%",
            relative_error * 100.0
        );
    }

    #[test]
    fn test_double_pendulum_different_parameters() {
        // Test with different link lengths and masses
        let mut pendulum =
            DoublePendulum::with_parameters(1.5, 0.8, 2.0, 0.5).with_angles(PI / 4.0, PI / 3.0);

        let initial_energy = pendulum.total_energy();

        // Run for 10 seconds
        pendulum.run_for(10.0, DT);

        let final_energy = pendulum.total_energy();

        // Energy should be conserved
        let relative_error = (final_energy - initial_energy).abs() / initial_energy;
        assert!(
            relative_error < 0.02,
            "Energy error {:.2}% exceeds 2%",
            relative_error * 100.0
        );
    }
}
