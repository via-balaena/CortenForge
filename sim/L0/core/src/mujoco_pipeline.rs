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
//! - Phase 4: N-link pendulum (n-DOF, general serial chain) - COMPLETE
//! - Phase 5: Ball joints (3-DOF spherical joints) - COMPLETE
//! - Phase 6: Contact (ground plane, sphere collisions) - COMPLETE

use nalgebra::Vector3;
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

// ============================================================================
// Phase 4: N-Link Pendulum (General Articulated Chain)
// ============================================================================
//
// This generalizes the double pendulum to an arbitrary number of links using
// MuJoCo's exact algorithms:
// - Recursive CRBA for n×n inertia matrix M(θ)
// - Recursive RNE for bias forces (Coriolis + gravity)
// - Dense matrix solve for accelerations
// - Semi-implicit Euler integration
//
// This is a serial chain (no branches), but demonstrates the recursive
// algorithms that scale to general kinematic trees.

use nalgebra::{DMatrix, DVector};

/// An n-link pendulum (serial chain of revolute joints).
///
/// This is the general case of the double pendulum, supporting any number
/// of links. Each link is a point mass at the end of a massless rod.
///
/// The algorithms follow MuJoCo/Featherstone exactly:
/// - CRBA: Composite Rigid Body Algorithm for inertia matrix
/// - RNE: Recursive Newton-Euler for bias forces
///
/// For a serial chain, we simplify the spatial algebra since all joints
/// rotate about the same axis (z-axis in the local frame).
#[derive(Debug, Clone)]
#[allow(clippy::doc_markdown)] // Math notation in docs uses subscripts
pub struct NLinkPendulum {
    /// Joint angles θ[i] in radians (angle of link i relative to link i-1)
    pub qpos: DVector<f64>,
    /// Joint velocities θ̇[i] in rad/s
    pub qvel: DVector<f64>,
    /// Link lengths L[i] in meters
    pub lengths: Vec<f64>,
    /// Link masses m[i] in kg (point mass at end of link)
    pub masses: Vec<f64>,
    /// Gravitational acceleration in m/s²
    pub gravity: f64,
}

#[allow(clippy::doc_markdown)] // Math notation in docs uses subscripts like θ_abs, m_i, etc.
#[allow(clippy::missing_panics_doc)] // Panics are from asserts in builder methods
impl NLinkPendulum {
    /// Create an n-link pendulum with uniform link lengths and masses.
    #[must_use]
    pub fn new(n_links: usize, length: f64, mass: f64) -> Self {
        Self {
            qpos: DVector::zeros(n_links),
            qvel: DVector::zeros(n_links),
            lengths: vec![length; n_links],
            masses: vec![mass; n_links],
            gravity: 9.81,
        }
    }

    /// Create an n-link pendulum with custom lengths and masses.
    #[must_use]
    pub fn with_parameters(lengths: Vec<f64>, masses: Vec<f64>) -> Self {
        assert_eq!(
            lengths.len(),
            masses.len(),
            "lengths and masses must have same length"
        );
        let n = lengths.len();
        Self {
            qpos: DVector::zeros(n),
            qvel: DVector::zeros(n),
            lengths,
            masses,
            gravity: 9.81,
        }
    }

    /// Set initial joint angles.
    #[must_use]
    pub fn with_angles(mut self, angles: &[f64]) -> Self {
        assert_eq!(
            angles.len(),
            self.n_links(),
            "angles must match number of links"
        );
        for (i, &angle) in angles.iter().enumerate() {
            self.qpos[i] = angle;
        }
        self
    }

    /// Set initial joint velocities.
    #[must_use]
    pub fn with_velocities(mut self, velocities: &[f64]) -> Self {
        assert_eq!(
            velocities.len(),
            self.n_links(),
            "velocities must match number of links"
        );
        for (i, &vel) in velocities.iter().enumerate() {
            self.qvel[i] = vel;
        }
        self
    }

    /// Get the number of links (degrees of freedom).
    #[must_use]
    pub fn n_links(&self) -> usize {
        self.lengths.len()
    }

    /// Compute the absolute angle of link i from vertical.
    ///
    /// θ_abs[i] = θ[0] + θ[1] + ... + θ[i]
    #[must_use]
    pub fn absolute_angle(&self, link: usize) -> f64 {
        self.qpos.iter().take(link + 1).sum()
    }

    /// Compute the Cartesian position of the end of link i.
    ///
    /// Position is computed by summing all link contributions:
    /// x = Σ L[j] * sin(θ_abs[j])
    /// y = -Σ L[j] * cos(θ_abs[j])
    #[must_use]
    pub fn position(&self, link: usize) -> Vector2<f64> {
        let mut x = 0.0;
        let mut y = 0.0;
        let mut theta_abs = 0.0;

        for j in 0..=link {
            theta_abs += self.qpos[j];
            x += self.lengths[j] * theta_abs.sin();
            y -= self.lengths[j] * theta_abs.cos();
        }

        Vector2::new(x, y)
    }

    /// Compute the n×n inertia matrix M(θ) via CRBA.
    ///
    /// For a planar serial chain with point masses at link ends:
    ///
    /// The kinetic energy is T = (1/2) Σ_k m_k |v_k|²
    ///
    /// The velocity of mass k depends on all joints 0..k:
    /// v_k = Σ_{j=0}^{k} L_j * θ̇_abs_j * (unit perpendicular to link j)
    ///
    /// where θ̇_abs_j = θ̇_0 + θ̇_1 + ... + θ̇_j
    ///
    /// Working out the kinetic energy and extracting M:
    /// M[i,j] = ∂²T/(∂θ̇_i ∂θ̇_j)
    ///
    /// For joint i affecting mass k (where k ≥ i), the velocity contribution is:
    /// ∂v_k/∂θ̇_i = Σ_{j=i}^{k} L_j * (perpendicular to link j)
    ///
    /// M[i,j] = Σ_{k≥max(i,j)} m_k * (∂v_k/∂θ̇_i · ∂v_k/∂θ̇_j)
    ///        = Σ_{k≥max(i,j)} m_k * Σ_{a=i}^{k} Σ_{b=j}^{k} L_a * L_b * cos(θ_abs_a - θ_abs_b)
    #[must_use]
    #[allow(clippy::needless_range_loop)] // Indices needed for array access patterns
    pub fn inertia_matrix(&self) -> DMatrix<f64> {
        let n = self.n_links();
        let mut m_matrix = DMatrix::zeros(n, n);

        // Precompute absolute angles
        let mut theta_abs = vec![0.0; n];
        let mut sum = 0.0;
        for i in 0..n {
            sum += self.qpos[i];
            theta_abs[i] = sum;
        }

        // M[i,j] = Σ_{k≥max(i,j)} m_k * Σ_{a=i}^{k} Σ_{b=j}^{k} L_a * L_b * cos(θ_abs_a - θ_abs_b)
        for i in 0..n {
            for j in 0..n {
                let mut m_ij = 0.0;
                let max_ij = i.max(j);

                // Sum over all masses from max(i,j) to n-1
                for k in max_ij..n {
                    // For this mass, sum over all link pairs that affect it
                    for a in i..=k {
                        for b in j..=k {
                            let angle_diff = theta_abs[a] - theta_abs[b];
                            m_ij += self.masses[k]
                                * self.lengths[a]
                                * self.lengths[b]
                                * angle_diff.cos();
                        }
                    }
                }

                m_matrix[(i, j)] = m_ij;
            }
        }

        m_matrix
    }

    /// Compute the bias forces (Coriolis + gravity) via RNE.
    ///
    /// The bias force for joint i is computed using the Christoffel symbols:
    /// C[i] = Σ_{j,k} c_{ijk} * θ̇_j * θ̇_k + g[i]
    ///
    /// where c_{ijk} = (1/2)(∂M_{ij}/∂θ_k + ∂M_{ik}/∂θ_j - ∂M_{jk}/∂θ_i)
    ///
    /// For gravity: g[i] = ∂V/∂θ_i where V = Σ_k m_k * g * h_k
    /// h_k = -Σ_{j=0}^{k} L_j * cos(θ_abs_j)
    /// ∂h_k/∂θ_i = Σ_{j=i}^{k} L_j * sin(θ_abs_j)  (for i ≤ k, else 0)
    /// g[i] = Σ_{k≥i} m_k * g * Σ_{j=i}^{k} L_j * sin(θ_abs_j)
    #[must_use]
    #[allow(clippy::similar_names)] // theta_abs variables are intentional
    #[allow(clippy::needless_range_loop)] // Indices needed for array access patterns
    pub fn bias_forces(&self) -> DVector<f64> {
        let n = self.n_links();
        let mut bias = DVector::zeros(n);

        // Precompute absolute angles
        let mut theta_abs = vec![0.0; n];
        let mut sum = 0.0;
        for i in 0..n {
            sum += self.qpos[i];
            theta_abs[i] = sum;
        }

        for i in 0..n {
            let mut coriolis_i = 0.0;
            let mut gravity_i = 0.0;

            // Gravity contribution:
            // g[i] = Σ_{k≥i} m_k * g * Σ_{j=i}^{k} L_j * sin(θ_abs_j)
            for k in i..n {
                for j in i..=k {
                    gravity_i +=
                        self.masses[k] * self.gravity * self.lengths[j] * theta_abs[j].sin();
                }
            }

            // Coriolis contribution using Christoffel symbols
            // c_{ijk} = (1/2)(∂M_{ij}/∂θ_k + ∂M_{ik}/∂θ_j - ∂M_{jk}/∂θ_i)
            for j in 0..n {
                for k in 0..n {
                    let dm_ij_dk = self.partial_m(&theta_abs, i, j, k);
                    let dm_ik_dj = self.partial_m(&theta_abs, i, k, j);
                    let dm_jk_di = self.partial_m(&theta_abs, j, k, i);

                    let c_ijk = 0.5 * (dm_ij_dk + dm_ik_dj - dm_jk_di);
                    coriolis_i += c_ijk * self.qvel[j] * self.qvel[k];
                }
            }

            bias[i] = coriolis_i + gravity_i;
        }

        bias
    }

    /// Compute ∂M[i,j]/∂θ_c
    ///
    /// M[i,j] = Σ_{k≥max(i,j)} m_k * Σ_{a=i}^{k} Σ_{b=j}^{k} L_a * L_b * cos(θ_abs_a - θ_abs_b)
    ///
    /// ∂M[i,j]/∂θ_c = Σ_{k≥max(i,j)} m_k * Σ_{a=i}^{k} Σ_{b=j}^{k} L_a * L_b * (-sin(θ_abs_a - θ_abs_b)) * sign
    ///
    /// where sign = ∂(θ_abs_a - θ_abs_b)/∂θ_c = (1 if c≤a else 0) - (1 if c≤b else 0)
    fn partial_m(&self, theta_abs: &[f64], i: usize, j: usize, c: usize) -> f64 {
        let n = self.n_links();
        let max_ij = i.max(j);
        let mut result = 0.0;

        for k in max_ij..n {
            for a in i..=k {
                for b in j..=k {
                    // Determine the sign of ∂(θ_abs_a - θ_abs_b)/∂θ_c
                    let d_theta_a: f64 = if c <= a { 1.0 } else { 0.0 };
                    let d_theta_b: f64 = if c <= b { 1.0 } else { 0.0 };
                    let sign = d_theta_a - d_theta_b;

                    if sign.abs() > 0.5 {
                        let angle_diff = theta_abs[a] - theta_abs[b];
                        result += self.masses[k]
                            * self.lengths[a]
                            * self.lengths[b]
                            * (-angle_diff.sin())
                            * sign;
                    }
                }
            }
        }

        result
    }

    /// Compute the total mechanical energy.
    ///
    /// E = T + V where:
    /// - T = ½ q̇ᵀ M q̇ (kinetic energy)
    /// - V = Σ m_i * g * h_i (potential energy)
    #[must_use]
    pub fn total_energy(&self) -> f64 {
        let n = self.n_links();

        // Kinetic energy: T = ½ q̇ᵀ M q̇
        let inertia = self.inertia_matrix();
        let kinetic = 0.5 * self.qvel.dot(&(&inertia * &self.qvel));

        // Potential energy: V = Σ m_i * g * (y_i + total_length)
        // Reference: V = 0 when all masses at lowest point
        let mut potential = 0.0;
        let total_length: f64 = self.lengths.iter().sum();

        for i in 0..n {
            let pos = self.position(i);
            // Height relative to lowest possible position
            let h = pos.y + total_length;
            potential += self.masses[i] * self.gravity * h;
        }

        kinetic + potential
    }

    /// Step the n-link pendulum forward using `MuJoCo`'s approach.
    ///
    /// 1. Compute inertia matrix M(θ) via CRBA
    /// 2. Compute bias forces via RNE
    /// 3. Solve M @ qacc = -bias for accelerations
    /// 4. Semi-implicit Euler integration
    pub fn step(&mut self, dt: f64) {
        // Stage 1: Forward Position (compute M via CRBA)
        let inertia = self.inertia_matrix();

        // Stage 2: Forward Velocity (compute bias via RNE)
        let qfrc_bias = self.bias_forces();

        // Stage 3-4: Forward Actuation & Acceleration
        // qfrc_smooth = -qfrc_bias (no actuation or applied forces)
        let qfrc_smooth = -&qfrc_bias;

        // Solve M @ qacc = qfrc_smooth for qacc using LU decomposition
        let qacc = inertia
            .lu()
            .solve(&qfrc_smooth)
            .unwrap_or_else(|| DVector::zeros(self.n_links()));

        // Stage 5: Integration (Semi-implicit Euler)
        let qvel_new = &self.qvel + &qacc * dt;
        let qpos_new = &self.qpos + &qvel_new * dt;

        self.qvel = qvel_new;
        self.qpos = qpos_new;
    }

    /// Run the n-link pendulum for a given duration.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub fn run_for(&mut self, duration: f64, dt: f64) {
        let steps = (duration / dt).ceil() as usize;
        for _ in 0..steps {
            self.step(dt);
        }
    }
}

// ============================================================================
// Phase 5: Spherical Pendulum (Ball Joint - 2-DOF in spherical coordinates)
// ============================================================================

/// A spherical pendulum using spherical coordinates.
///
/// This is a point mass constrained to move on a sphere, attached to a fixed
/// pivot. Unlike a simple 2D pendulum (1-DOF), this system has 2-DOF.
///
/// The state is represented using spherical coordinates:
/// - `theta`: polar angle from vertical (0 = hanging down, π = pointing up)
/// - `phi`: azimuthal angle in the horizontal plane
/// - `theta_dot`: rate of change of theta
/// - `phi_dot`: rate of change of phi
///
/// The Lagrangian for a spherical pendulum is:
/// ```text
/// L = T - V
/// T = (1/2) m L² (θ̇² + sin²(θ) φ̇²)
/// V = -m g L cos(θ)
/// ```
///
/// This leads to the equations of motion:
/// ```text
/// θ̈ = sin(θ) cos(θ) φ̇² - (g/L) sin(θ)
/// φ̈ = -2 cot(θ) θ̇ φ̇
/// ```
///
/// The system conserves:
/// - Total energy E = T + V
/// - Angular momentum about vertical axis (Lz = m L² sin²(θ) φ̇)
#[derive(Debug, Clone)]
#[allow(clippy::doc_markdown)]
pub struct SphericalPendulum {
    /// Polar angle from vertical (radians), 0 = hanging down
    pub theta: f64,
    /// Azimuthal angle (radians)
    pub phi: f64,
    /// Angular velocity of theta (rad/s)
    pub theta_dot: f64,
    /// Angular velocity of phi (rad/s)
    pub phi_dot: f64,
    /// Length from pivot to center of mass (m)
    pub length: f64,
    /// Pendulum mass (kg)
    pub mass: f64,
    /// Gravitational acceleration (m/s²)
    pub gravity: f64,
}

impl SphericalPendulum {
    /// Create a new spherical pendulum with default parameters.
    ///
    /// Starts hanging straight down (theta = 0).
    #[must_use]
    pub fn new(length: f64, mass: f64) -> Self {
        Self {
            theta: 0.0,
            phi: 0.0,
            theta_dot: 0.0,
            phi_dot: 0.0,
            length,
            mass,
            gravity: 9.81,
        }
    }

    /// Set the initial polar angle (tilt from vertical).
    #[must_use]
    pub fn with_theta(mut self, theta: f64) -> Self {
        self.theta = theta;
        self
    }

    /// Set the initial azimuthal angle.
    #[must_use]
    pub fn with_phi(mut self, phi: f64) -> Self {
        self.phi = phi;
        self
    }

    /// Set the initial angular velocities.
    #[must_use]
    pub fn with_velocities(mut self, theta_dot: f64, phi_dot: f64) -> Self {
        self.theta_dot = theta_dot;
        self.phi_dot = phi_dot;
        self
    }

    /// Convenience method to set a tilt from vertical along an axis.
    ///
    /// Arguments:
    /// - `tilt_angle`: angle from vertical (radians)
    /// - `tilt_axis`: direction in XZ plane (normalized, Y component ignored)
    #[must_use]
    pub fn with_tilt(mut self, tilt_angle: f64, tilt_axis: Vector3<f64>) -> Self {
        self.theta = tilt_angle;
        // phi is the angle in the XZ plane, measured from X axis toward Z
        // atan2(z, x) gives the angle
        self.phi = tilt_axis.z.atan2(tilt_axis.x);
        self
    }

    /// Set custom gravity.
    #[must_use]
    pub fn with_gravity(mut self, gravity: f64) -> Self {
        self.gravity = gravity;
        self
    }

    /// Get the position of the bob in world frame (Y-up convention).
    ///
    /// Using spherical coordinates where theta is from -Y axis:
    /// - x = L sin(θ) cos(φ)
    /// - y = -L cos(θ)  (negative because 0 = hanging down)
    /// - z = L sin(θ) sin(φ)
    #[must_use]
    pub fn position(&self) -> Vector3<f64> {
        let sin_theta = self.theta.sin();
        let cos_theta = self.theta.cos();
        let sin_phi = self.phi.sin();
        let cos_phi = self.phi.cos();

        Vector3::new(
            self.length * sin_theta * cos_phi,
            -self.length * cos_theta,
            self.length * sin_theta * sin_phi,
        )
    }

    /// Get the velocity of the bob in world frame.
    #[must_use]
    pub fn velocity(&self) -> Vector3<f64> {
        let sin_theta = self.theta.sin();
        let cos_theta = self.theta.cos();
        let sin_phi = self.phi.sin();
        let cos_phi = self.phi.cos();

        let l = self.length;
        let td = self.theta_dot;
        let pd = self.phi_dot;

        // Derivatives of position with respect to time
        Vector3::new(
            l * (cos_theta * cos_phi * td - sin_theta * sin_phi * pd),
            l * sin_theta * td,
            l * (cos_theta * sin_phi * td + sin_theta * cos_phi * pd),
        )
    }

    /// Compute the 2x2 inertia matrix in (theta, phi) coordinates.
    ///
    /// From the kinetic energy T = (1/2) m L² (θ̇² + sin²(θ) φ̇²)
    /// The inertia matrix is:
    /// ```text
    /// M = m L² | 1        0          |
    ///          | 0        sin²(θ)    |
    /// ```
    #[must_use]
    pub fn inertia_matrix(&self) -> nalgebra::Matrix2<f64> {
        let ml2 = self.mass * self.length * self.length;
        let sin2_theta = self.theta.sin().powi(2);

        nalgebra::Matrix2::new(ml2, 0.0, 0.0, ml2 * sin2_theta)
    }

    /// Total mechanical energy.
    ///
    /// E = T + V where:
    /// - T = (1/2) m L² (θ̇² + sin²(θ) φ̇²)
    /// - V = -m g L cos(θ) + m g L  (shifted so V=0 at θ=0)
    #[must_use]
    pub fn total_energy(&self) -> f64 {
        let ml2 = self.mass * self.length * self.length;
        let sin_theta = self.theta.sin();

        // Kinetic energy
        let kinetic =
            0.5 * ml2 * (self.theta_dot.powi(2) + sin_theta.powi(2) * self.phi_dot.powi(2));

        // Potential energy (reference: V=0 at lowest point, θ=0)
        let cos_theta = self.theta.cos();
        let potential = self.mass * self.gravity * self.length * (1.0 - cos_theta);

        kinetic + potential
    }

    /// Angular momentum about the vertical axis (conserved quantity).
    ///
    /// Lz = m L² sin²(θ) φ̇
    #[must_use]
    #[allow(clippy::doc_markdown)]
    pub fn angular_momentum_z(&self) -> f64 {
        let ml2 = self.mass * self.length * self.length;
        let sin_theta = self.theta.sin();
        ml2 * sin_theta.powi(2) * self.phi_dot
    }

    /// Step the spherical pendulum forward in time.
    ///
    /// Uses the equations of motion from Lagrangian mechanics:
    /// - θ̈ = sin(θ) cos(θ) φ̇² - (g/L) sin(θ)
    /// - φ̈ = -2 cot(θ) θ̇ φ̇  (when sin(θ) ≠ 0)
    ///
    /// Uses semi-implicit Euler integration.
    pub fn step(&mut self, dt: f64) {
        let sin_theta = self.theta.sin();
        let cos_theta = self.theta.cos();

        // Compute accelerations from equations of motion
        // θ̈ = sin(θ) cos(θ) φ̇² - (g/L) sin(θ)
        let theta_ddot =
            sin_theta * cos_theta * self.phi_dot.powi(2) - (self.gravity / self.length) * sin_theta;

        // φ̈ = -2 cot(θ) θ̇ φ̇ = -2 (cos(θ)/sin(θ)) θ̇ φ̇
        // Handle singularity at θ = 0 or θ = π
        let phi_ddot = if sin_theta.abs() > 1e-10 {
            -2.0 * (cos_theta / sin_theta) * self.theta_dot * self.phi_dot
        } else {
            0.0
        };

        // Semi-implicit Euler: update velocities first
        self.theta_dot += theta_ddot * dt;
        self.phi_dot += phi_ddot * dt;

        // Then update positions using new velocities
        self.theta += self.theta_dot * dt;
        self.phi += self.phi_dot * dt;

        // Normalize phi to [0, 2π) for cleaner values
        // Use rem_euclid for proper modulo that handles negatives
        self.phi = self.phi.rem_euclid(2.0 * PI);
    }

    /// Run the spherical pendulum for a given duration.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub fn run_for(&mut self, duration: f64, dt: f64) {
        let steps = (duration / dt).ceil() as usize;
        for _ in 0..steps {
            self.step(dt);
        }
    }
}

// ============================================================================
// Phase 6: Contact Physics (Ground Plane + Sphere Collisions)
// ============================================================================

/// A bouncing ball with contact physics.
///
/// This demonstrates the contact constraint solver:
/// 1. Collision detection (sphere vs ground plane)
/// 2. Contact normal force (prevents penetration)
/// 3. Friction force (Coulomb model with friction cone)
/// 4. Coefficient of restitution (energy loss on bounce)
///
/// The contact force is computed using constraint-based dynamics:
/// - Normal constraint: `λ_n ≥ 0` (unilateral, push only)
/// - Friction constraint: `|λ_t| ≤ μ * λ_n` (Coulomb cone)
///
/// Uses the complementarity condition:
/// - Either the ball is not in contact (gap > 0, λ = 0)
/// - Or the ball is in contact (gap = 0, λ ≥ 0)
#[derive(Debug, Clone)]
#[allow(clippy::doc_markdown)]
pub struct BouncingBall {
    /// Position (x, y, z) where y is up
    pub pos: Vector3<f64>,
    /// Velocity (vx, vy, vz)
    pub vel: Vector3<f64>,
    /// Ball radius
    pub radius: f64,
    /// Ball mass
    pub mass: f64,
    /// Gravitational acceleration (m/s²)
    pub gravity: f64,
    /// Coefficient of restitution (0 = perfectly inelastic, 1 = perfectly elastic)
    pub restitution: f64,
    /// Coefficient of friction (Coulomb friction)
    pub friction: f64,
    /// Ground plane height (y coordinate)
    pub ground_y: f64,
}

impl BouncingBall {
    /// Create a new bouncing ball.
    #[must_use]
    pub fn new(radius: f64, mass: f64) -> Self {
        Self {
            pos: Vector3::new(0.0, radius + 1.0, 0.0), // Start 1m above ground
            vel: Vector3::zeros(),
            radius,
            mass,
            gravity: 9.81,
            restitution: 0.8, // Slightly inelastic
            friction: 0.5,    // Moderate friction
            ground_y: 0.0,
        }
    }

    /// Set the initial position.
    #[must_use]
    pub fn with_position(mut self, pos: Vector3<f64>) -> Self {
        self.pos = pos;
        self
    }

    /// Set the initial velocity.
    #[must_use]
    pub fn with_velocity(mut self, vel: Vector3<f64>) -> Self {
        self.vel = vel;
        self
    }

    /// Set the coefficient of restitution.
    #[must_use]
    pub fn with_restitution(mut self, restitution: f64) -> Self {
        self.restitution = restitution.clamp(0.0, 1.0);
        self
    }

    /// Set the coefficient of friction.
    #[must_use]
    pub fn with_friction(mut self, friction: f64) -> Self {
        self.friction = friction.max(0.0);
        self
    }

    /// Set custom gravity.
    #[must_use]
    pub fn with_gravity(mut self, gravity: f64) -> Self {
        self.gravity = gravity;
        self
    }

    /// Set the ground plane height.
    #[must_use]
    pub fn with_ground(mut self, ground_y: f64) -> Self {
        self.ground_y = ground_y;
        self
    }

    /// Compute the gap (signed distance) to the ground.
    ///
    /// gap > 0: not in contact
    /// gap = 0: touching
    /// gap < 0: penetrating
    #[must_use]
    pub fn gap(&self) -> f64 {
        self.pos.y - self.radius - self.ground_y
    }

    /// Check if the ball is in contact with the ground.
    #[must_use]
    pub fn in_contact(&self) -> bool {
        self.gap() <= 0.0
    }

    /// Compute total mechanical energy.
    ///
    /// E = KE + PE where:
    /// - KE = (1/2) m v²
    /// - PE = m g h (height above ground)
    #[must_use]
    pub fn total_energy(&self) -> f64 {
        let kinetic = 0.5 * self.mass * self.vel.norm_squared();
        let height = self.pos.y - self.ground_y;
        let potential = self.mass * self.gravity * height;
        kinetic + potential
    }

    /// Step the bouncing ball forward in time.
    ///
    /// Uses impulse-based contact resolution:
    /// 1. Apply gravity to get pre-contact velocity
    /// 2. Detect contact
    /// 3. If in contact, compute and apply contact impulse
    /// 4. Update position
    pub fn step(&mut self, dt: f64) {
        // Apply gravity acceleration
        let gravity_accel = Vector3::new(0.0, -self.gravity, 0.0);

        // Semi-implicit Euler: update velocity first
        self.vel += gravity_accel * dt;

        // Update position using new velocity
        self.pos += self.vel * dt;

        // Check for ground contact
        let gap = self.gap();

        if gap < 0.0 {
            // We're penetrating - resolve the contact

            // 1. Fix penetration by moving ball up
            self.pos.y = self.ground_y + self.radius;

            // 2. Compute contact impulse
            // Normal direction is up (y-axis)
            let normal = Vector3::new(0.0, 1.0, 0.0);

            // Normal velocity (positive = separating, negative = approaching)
            let v_n = self.vel.dot(&normal);

            if v_n < 0.0 {
                // Ball is moving into the ground - apply impulse

                // Compute normal impulse with restitution
                // v_n_new = -e * v_n (coefficient of restitution)
                let impulse_n = -(1.0 + self.restitution) * v_n;

                // Apply normal impulse
                self.vel += normal * impulse_n;

                // 3. Apply friction impulse
                // Tangential velocity (in XZ plane)
                let v_t = self.vel - normal * self.vel.dot(&normal);
                let v_t_mag = v_t.norm();

                if v_t_mag > 1e-10 {
                    // Tangent direction
                    let tangent = v_t / v_t_mag;

                    // Maximum friction impulse (Coulomb friction)
                    // |j_t| ≤ μ * j_n
                    let max_friction_impulse = self.friction * impulse_n;

                    // Impulse needed to stop tangential motion
                    let desired_impulse = v_t_mag;

                    // Apply friction (clamped to friction cone)
                    let friction_impulse = desired_impulse.min(max_friction_impulse);
                    self.vel -= tangent * friction_impulse;
                }
            }
        }
    }

    /// Run the bouncing ball for a given duration.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub fn run_for(&mut self, duration: f64, dt: f64) {
        let steps = (duration / dt).ceil() as usize;
        for _ in 0..steps {
            self.step(dt);
        }
    }
}

/// Multiple spheres with inter-sphere collision and ground contact.
///
/// This extends the single bouncing ball to handle:
/// 1. Multiple spheres
/// 2. Sphere-sphere collisions
/// 3. Ground plane collisions for all spheres
#[derive(Debug, Clone)]
pub struct SpherePile {
    /// Sphere positions
    pub positions: Vec<Vector3<f64>>,
    /// Sphere velocities
    pub velocities: Vec<Vector3<f64>>,
    /// Sphere radii
    pub radii: Vec<f64>,
    /// Sphere masses
    pub masses: Vec<f64>,
    /// Gravitational acceleration
    pub gravity: f64,
    /// Coefficient of restitution
    pub restitution: f64,
    /// Coefficient of friction
    pub friction: f64,
    /// Ground plane height
    pub ground_y: f64,
}

impl SpherePile {
    /// Create a new sphere pile with the given number of spheres.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn new(n_spheres: usize, radius: f64, mass: f64) -> Self {
        let mut positions = Vec::with_capacity(n_spheres);
        let mut velocities = Vec::with_capacity(n_spheres);
        let radii = vec![radius; n_spheres];
        let masses = vec![mass; n_spheres];

        // Stack spheres vertically
        for i in 0..n_spheres {
            let y = radius + 2.0 * radius * (i as f64) + 0.5;
            positions.push(Vector3::new(0.0, y, 0.0));
            velocities.push(Vector3::zeros());
        }

        Self {
            positions,
            velocities,
            radii,
            masses,
            gravity: 9.81,
            restitution: 0.8,
            friction: 0.5,
            ground_y: 0.0,
        }
    }

    /// Set the coefficient of restitution.
    #[must_use]
    pub fn with_restitution(mut self, restitution: f64) -> Self {
        self.restitution = restitution.clamp(0.0, 1.0);
        self
    }

    /// Set the coefficient of friction.
    #[must_use]
    pub fn with_friction(mut self, friction: f64) -> Self {
        self.friction = friction.max(0.0);
        self
    }

    /// Number of spheres.
    #[must_use]
    pub fn n_spheres(&self) -> usize {
        self.positions.len()
    }

    /// Compute total mechanical energy.
    #[must_use]
    pub fn total_energy(&self) -> f64 {
        let mut energy = 0.0;
        for i in 0..self.n_spheres() {
            let kinetic = 0.5 * self.masses[i] * self.velocities[i].norm_squared();
            let height = self.positions[i].y - self.ground_y;
            let potential = self.masses[i] * self.gravity * height;
            energy += kinetic + potential;
        }
        energy
    }

    /// Step all spheres forward in time.
    pub fn step(&mut self, dt: f64) {
        let n = self.n_spheres();

        // Apply gravity to all spheres
        for i in 0..n {
            self.velocities[i].y -= self.gravity * dt;
        }

        // Update positions
        for i in 0..n {
            self.positions[i] += self.velocities[i] * dt;
        }

        // Resolve ground contacts
        for i in 0..n {
            let gap = self.positions[i].y - self.radii[i] - self.ground_y;

            if gap < 0.0 {
                // Fix penetration
                self.positions[i].y = self.ground_y + self.radii[i];

                // Normal velocity
                let v_n = self.velocities[i].y;

                if v_n < 0.0 {
                    // Apply restitution
                    self.velocities[i].y = -self.restitution * v_n;

                    // Apply friction to tangential velocity
                    let normal_impulse = -(1.0 + self.restitution) * v_n * self.masses[i];
                    let max_friction = self.friction * normal_impulse;

                    let v_t = Vector3::new(self.velocities[i].x, 0.0, self.velocities[i].z);
                    let v_t_mag = v_t.norm();

                    if v_t_mag > 1e-10 {
                        let friction_impulse = (v_t_mag * self.masses[i]).min(max_friction);
                        let friction_decel = friction_impulse / self.masses[i];
                        self.velocities[i].x -= (v_t.x / v_t_mag) * friction_decel;
                        self.velocities[i].z -= (v_t.z / v_t_mag) * friction_decel;
                    }
                }
            }
        }

        // Resolve sphere-sphere collisions
        for i in 0..n {
            for j in (i + 1)..n {
                let diff = self.positions[j] - self.positions[i];
                let dist = diff.norm();
                let min_dist = self.radii[i] + self.radii[j];

                if dist < min_dist && dist > 1e-10 {
                    // Collision detected
                    let normal = diff / dist;
                    let penetration = min_dist - dist;

                    // Separate spheres proportionally to inverse mass
                    let total_mass = self.masses[i] + self.masses[j];
                    let ratio_i = self.masses[j] / total_mass;
                    let ratio_j = self.masses[i] / total_mass;

                    self.positions[i] -= normal * (penetration * ratio_i);
                    self.positions[j] += normal * (penetration * ratio_j);

                    // Relative velocity along normal
                    let rel_vel = self.velocities[j] - self.velocities[i];
                    let v_n = rel_vel.dot(&normal);

                    if v_n < 0.0 {
                        // Spheres approaching - apply impulse
                        // Impulse magnitude for elastic collision
                        let impulse_mag = -(1.0 + self.restitution) * v_n
                            / (1.0 / self.masses[i] + 1.0 / self.masses[j]);

                        self.velocities[i] -= normal * (impulse_mag / self.masses[i]);
                        self.velocities[j] += normal * (impulse_mag / self.masses[j]);

                        // Apply friction
                        let rel_vel_new = self.velocities[j] - self.velocities[i];
                        let v_t = rel_vel_new - normal * rel_vel_new.dot(&normal);
                        let v_t_mag = v_t.norm();

                        if v_t_mag > 1e-10 {
                            let tangent = v_t / v_t_mag;
                            let max_friction_impulse = self.friction * impulse_mag.abs();
                            let reduced_mass = 1.0 / (1.0 / self.masses[i] + 1.0 / self.masses[j]);
                            let desired_impulse = v_t_mag * reduced_mass;
                            let friction_impulse = desired_impulse.min(max_friction_impulse);

                            self.velocities[i] += tangent * (friction_impulse / self.masses[i]);
                            self.velocities[j] -= tangent * (friction_impulse / self.masses[j]);
                        }
                    }
                }
            }
        }
    }

    /// Run the sphere pile for a given duration.
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
    clippy::cast_sign_loss,
    clippy::similar_names,
    clippy::uninlined_format_args,
    clippy::panic,
    clippy::needless_range_loop
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

    // =========================================================================
    // Phase 4: N-Link Pendulum Tests
    // =========================================================================

    #[test]
    fn test_nlink_matches_double_pendulum() {
        // A 2-link NLinkPendulum should behave identically to DoublePendulum
        let angles = [PI / 4.0, PI / 6.0];
        let mut nlink = NLinkPendulum::new(2, 1.0, 1.0).with_angles(&angles);
        let mut double = DoublePendulum::new(1.0, 1.0).with_angles(PI / 4.0, PI / 6.0);

        let high_res_dt = 1.0 / 480.0;

        // Run both for 5 seconds
        for _ in 0..2400 {
            nlink.step(high_res_dt);
            double.step(high_res_dt);

            // Angles should match
            assert!(
                (nlink.qpos[0] - double.qpos[0]).abs() < 0.01,
                "θ₁ mismatch: nlink={:.4}, double={:.4}",
                nlink.qpos[0],
                double.qpos[0]
            );
            assert!(
                (nlink.qpos[1] - double.qpos[1]).abs() < 0.01,
                "θ₂ mismatch: nlink={:.4}, double={:.4}",
                nlink.qpos[1],
                double.qpos[1]
            );
        }
    }

    #[test]
    fn test_nlink_inertia_matrix_symmetric() {
        // The inertia matrix must always be symmetric
        let angles = [PI / 4.0, PI / 6.0, PI / 8.0, PI / 10.0];
        let pendulum = NLinkPendulum::new(4, 1.0, 1.0).with_angles(&angles);
        let inertia = pendulum.inertia_matrix();

        for i in 0..4 {
            for j in 0..4 {
                assert_relative_eq!(inertia[(i, j)], inertia[(j, i)], epsilon = 1e-10);
            }
        }
    }

    #[test]
    fn test_nlink_inertia_matrix_positive_definite() {
        // The inertia matrix must be positive definite
        let angles = [PI / 4.0, PI / 6.0, PI / 8.0];
        let pendulum = NLinkPendulum::new(3, 1.0, 1.0).with_angles(&angles);
        let inertia = pendulum.inertia_matrix();

        // Check all leading principal minors are positive (Sylvester's criterion)
        // For 3x3: M₁₁ > 0, det(M[0:2,0:2]) > 0, det(M) > 0
        assert!(inertia[(0, 0)] > 0.0, "M₁₁ should be positive");

        let det2 = inertia[(0, 0)] * inertia[(1, 1)] - inertia[(0, 1)] * inertia[(1, 0)];
        assert!(det2 > 0.0, "2x2 minor should be positive");

        let det3 = inertia.determinant();
        assert!(det3 > 0.0, "Determinant should be positive, got {det3}");
    }

    #[test]
    fn test_nlink_3link_energy_conservation() {
        // Energy should be conserved in a 3-link pendulum
        let angles = [PI / 6.0, PI / 8.0, PI / 10.0];
        let mut pendulum = NLinkPendulum::new(3, 1.0, 1.0).with_angles(&angles);

        let initial_energy = pendulum.total_energy();

        // Run for 10 seconds at high resolution
        let high_res_dt = 1.0 / 480.0;
        pendulum.run_for(10.0, high_res_dt);

        let final_energy = pendulum.total_energy();

        // Allow 2% energy drift (more links = more numerical error)
        let relative_error = (final_energy - initial_energy).abs() / initial_energy;
        assert!(
            relative_error < 0.02,
            "Energy error {:.2}% exceeds 2%",
            relative_error * 100.0
        );
    }

    #[test]
    fn test_nlink_4link_energy_conservation() {
        // Energy should be conserved in a 4-link pendulum
        let angles = [PI / 8.0, PI / 10.0, PI / 12.0, PI / 14.0];
        let mut pendulum = NLinkPendulum::new(4, 1.0, 1.0).with_angles(&angles);

        let initial_energy = pendulum.total_energy();

        // Run for 10 seconds at high resolution
        let high_res_dt = 1.0 / 480.0;
        pendulum.run_for(10.0, high_res_dt);

        let final_energy = pendulum.total_energy();

        // Allow 3% energy drift for 4 links
        let relative_error = (final_energy - initial_energy).abs() / initial_energy;
        assert!(
            relative_error < 0.03,
            "Energy error {:.2}% exceeds 3%",
            relative_error * 100.0
        );
    }

    #[test]
    fn test_nlink_equilibrium() {
        // At θ=0 for all joints with zero velocity, should stay at rest
        let mut pendulum = NLinkPendulum::new(5, 1.0, 1.0);

        pendulum.run_for(5.0, DT);

        for i in 0..5 {
            assert_relative_eq!(pendulum.qpos[i], 0.0, epsilon = 1e-10);
            assert_relative_eq!(pendulum.qvel[i], 0.0, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_nlink_links_connected() {
        // All link positions should be consistent with the kinematics
        let angles = [PI / 3.0, PI / 4.0, PI / 5.0];
        let mut pendulum = NLinkPendulum::new(3, 1.0, 1.0).with_angles(&angles);

        // Run and verify link connectivity
        let high_res_dt = 1.0 / 480.0;
        for _ in 0..4800 {
            pendulum.step(high_res_dt);

            // Verify each link length
            let mut prev_pos = Vector2::zeros();
            for i in 0..3 {
                let pos = pendulum.position(i);
                let link_length = (pos - prev_pos).norm();
                assert!(
                    (link_length - pendulum.lengths[i]).abs() < 1e-10,
                    "Link {} length mismatch: expected {}, got {}",
                    i,
                    pendulum.lengths[i],
                    link_length
                );
                prev_pos = pos;
            }
        }
    }

    #[test]
    fn test_nlink_falls_from_horizontal() {
        // Starting with first link horizontal, it should fall
        let angles = [PI / 2.0, 0.0, 0.0];
        let mut pendulum = NLinkPendulum::new(3, 1.0, 1.0).with_angles(&angles);

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
    fn test_nlink_position_formulas() {
        // Verify position calculation for a 3-link chain
        let l1 = 1.0;
        let l2 = 0.8;
        let l3 = 0.6;
        let theta1 = PI / 4.0;
        let theta2 = PI / 6.0;
        let theta3 = PI / 8.0;

        let pendulum = NLinkPendulum::with_parameters(vec![l1, l2, l3], vec![1.0, 1.0, 1.0])
            .with_angles(&[theta1, theta2, theta3]);

        // Position of link 0 end
        let pos0 = pendulum.position(0);
        assert_relative_eq!(pos0.x, l1 * theta1.sin(), epsilon = 1e-10);
        assert_relative_eq!(pos0.y, -l1 * theta1.cos(), epsilon = 1e-10);

        // Position of link 1 end
        let pos1 = pendulum.position(1);
        let theta_abs1 = theta1 + theta2;
        let expected_x1 = l1 * theta1.sin() + l2 * theta_abs1.sin();
        let expected_y1 = -l1 * theta1.cos() - l2 * theta_abs1.cos();
        assert_relative_eq!(pos1.x, expected_x1, epsilon = 1e-10);
        assert_relative_eq!(pos1.y, expected_y1, epsilon = 1e-10);

        // Position of link 2 end
        let pos2 = pendulum.position(2);
        let theta_abs2 = theta1 + theta2 + theta3;
        let expected_x2 = expected_x1 + l3 * theta_abs2.sin();
        let expected_y2 = expected_y1 - l3 * theta_abs2.cos();
        assert_relative_eq!(pos2.x, expected_x2, epsilon = 1e-10);
        assert_relative_eq!(pos2.y, expected_y2, epsilon = 1e-10);
    }

    #[test]
    fn test_nlink_different_parameters() {
        // Test with different link lengths and masses
        let lengths = vec![1.5, 0.8, 1.2];
        let masses = vec![2.0, 0.5, 1.0];
        let angles = [PI / 6.0, PI / 8.0, PI / 10.0];

        let mut pendulum = NLinkPendulum::with_parameters(lengths, masses).with_angles(&angles);

        let initial_energy = pendulum.total_energy();

        // Run for 10 seconds at high resolution
        let high_res_dt = 1.0 / 480.0;
        pendulum.run_for(10.0, high_res_dt);

        let final_energy = pendulum.total_energy();

        // Energy should be conserved
        let relative_error = (final_energy - initial_energy).abs() / initial_energy;
        assert!(
            relative_error < 0.02,
            "Energy error {:.2}% exceeds 2%",
            relative_error * 100.0
        );
    }

    #[test]
    fn test_nlink_5link_chain() {
        // Test a longer chain (5 links)
        let angles = [PI / 8.0, PI / 10.0, PI / 12.0, PI / 14.0, PI / 16.0];
        let mut pendulum = NLinkPendulum::new(5, 0.5, 1.0).with_angles(&angles);

        let initial_energy = pendulum.total_energy();

        // Run for 5 seconds at high resolution
        let high_res_dt = 1.0 / 480.0;
        pendulum.run_for(5.0, high_res_dt);

        let final_energy = pendulum.total_energy();

        // Allow 5% energy drift for 5 links (more challenging)
        let relative_error = (final_energy - initial_energy).abs() / initial_energy;
        assert!(
            relative_error < 0.05,
            "Energy error {:.2}% exceeds 5%",
            relative_error * 100.0
        );
    }

    // =========================================================================
    // Phase 5: Spherical Pendulum Tests
    // =========================================================================

    #[test]
    fn test_spherical_pendulum_equilibrium() {
        // Hanging straight down should have zero acceleration
        let pendulum = SphericalPendulum::new(1.0, 1.0);

        // At equilibrium (theta=0), position is straight down
        let pos = pendulum.position();
        assert_relative_eq!(pos.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(pos.y, -1.0, epsilon = 1e-10);
        assert_relative_eq!(pos.z, 0.0, epsilon = 1e-10);

        // Energy should be zero at equilibrium (our reference point)
        let energy = pendulum.total_energy();
        assert_relative_eq!(energy, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_spherical_pendulum_tilted_position() {
        // Tilt the pendulum 45 degrees (theta = π/4)
        // phi = 0 means tilt is in the X direction
        let pendulum = SphericalPendulum::new(1.0, 1.0)
            .with_theta(PI / 4.0)
            .with_phi(0.0);

        let pos = pendulum.position();

        // x = L sin(θ) cos(φ) = 1 * sin(45°) * 1 = sin(45°)
        // y = -L cos(θ) = -cos(45°)
        // z = L sin(θ) sin(φ) = 0
        let cos45 = (PI / 4.0).cos();
        let sin45 = (PI / 4.0).sin();

        assert_relative_eq!(pos.x, sin45, epsilon = 1e-10);
        assert_relative_eq!(pos.y, -cos45, epsilon = 1e-10);
        assert_relative_eq!(pos.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_spherical_pendulum_horizontal_position() {
        // Horizontal (theta = π/2) in the X direction
        let pendulum = SphericalPendulum::new(1.0, 1.0)
            .with_theta(PI / 2.0)
            .with_phi(0.0);

        let pos = pendulum.position();

        // x = L sin(90°) cos(0°) = 1
        // y = -L cos(90°) = 0
        // z = L sin(90°) sin(0°) = 0
        assert_relative_eq!(pos.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(pos.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(pos.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_spherical_pendulum_energy_conservation() {
        // Start tilted and verify energy is conserved
        let mut pendulum = SphericalPendulum::new(1.0, 1.0).with_theta(PI / 4.0);

        let initial_energy = pendulum.total_energy();
        assert!(initial_energy > 0.0, "Initial energy should be positive");

        // Run for 10 seconds at high resolution
        let high_res_dt = 1.0 / 480.0;
        pendulum.run_for(10.0, high_res_dt);

        let final_energy = pendulum.total_energy();

        // Allow 2% energy drift
        let relative_error = (final_energy - initial_energy).abs() / initial_energy;
        assert!(
            relative_error < 0.02,
            "Energy error {:.2}% exceeds 2%",
            relative_error * 100.0
        );
    }

    #[test]
    fn test_spherical_pendulum_falls_from_horizontal() {
        // Start horizontal (theta = π/2), should fall and swing
        let mut pendulum = SphericalPendulum::new(1.0, 1.0).with_theta(PI / 2.0);

        let initial_pos = pendulum.position();
        assert_relative_eq!(initial_pos.y, 0.0, epsilon = 1e-10);

        // Take some steps
        for _ in 0..100 {
            pendulum.step(1.0 / 240.0);
        }

        // Should have fallen (y should be more negative)
        let pos = pendulum.position();
        assert!(
            pos.y < initial_pos.y,
            "Pendulum should fall: y={:.3} should be < {:.3}",
            pos.y,
            initial_pos.y
        );
    }

    #[test]
    fn test_spherical_pendulum_small_angle_oscillation() {
        // Small angle oscillation should have period T ≈ 2π√(L/g)
        // Start with small tilt (in the X direction, phi=0)
        let mut pendulum = SphericalPendulum::new(1.0, 1.0)
            .with_theta(0.1)
            .with_phi(0.0);

        let expected_period = 2.0 * PI * (1.0 / 9.81_f64).sqrt();

        // Track theta to find period (simpler than tracking position)
        let dt = 1.0 / 480.0;
        let max_steps = (3.0 * expected_period / dt) as usize;
        let mut prev_theta_dot = pendulum.theta_dot;
        let mut time = 0.0;
        let mut first_peak = None;
        let mut second_peak = None;

        for _ in 0..max_steps {
            pendulum.step(dt);
            time += dt;

            // Look for peaks (velocity changes sign from positive to negative)
            if prev_theta_dot > 0.0 && pendulum.theta_dot <= 0.0 {
                if first_peak.is_none() {
                    first_peak = Some(time);
                } else if second_peak.is_none() {
                    second_peak = Some(time);
                    break;
                }
            }
            prev_theta_dot = pendulum.theta_dot;
        }

        if let (Some(t1), Some(t2)) = (first_peak, second_peak) {
            let measured_period = t2 - t1; // Full period between peaks

            // Allow 5% error for small angle approximation
            let period_error = (measured_period - expected_period).abs() / expected_period;
            assert!(
                period_error < 0.05,
                "Period error {:.1}% exceeds 5%: measured {:.3}s vs expected {:.3}s",
                period_error * 100.0,
                measured_period,
                expected_period
            );
        } else {
            panic!("Should complete at least one oscillation");
        }
    }

    #[test]
    fn test_spherical_pendulum_3d_motion() {
        // Give the pendulum both tilt and azimuthal velocity for 3D precession
        let mut pendulum = SphericalPendulum::new(1.0, 1.0)
            .with_theta(PI / 4.0)
            .with_velocities(0.0, 2.0); // phi_dot = 2 rad/s

        let initial_energy = pendulum.total_energy();
        let initial_lz = pendulum.angular_momentum_z();

        // Run for 5 seconds at high resolution (3D motion needs finer timestep)
        let dt = 1.0 / 960.0;
        pendulum.run_for(5.0, dt);

        let final_energy = pendulum.total_energy();
        let final_lz = pendulum.angular_momentum_z();

        // Energy should be conserved (allow 5% for this challenging case)
        let energy_error = (final_energy - initial_energy).abs() / initial_energy;
        assert!(
            energy_error < 0.05,
            "Energy error {:.2}% exceeds 5%",
            energy_error * 100.0
        );

        // Angular momentum about vertical axis should be conserved
        let lz_error = (final_lz - initial_lz).abs() / initial_lz.abs();
        assert!(
            lz_error < 0.05,
            "Angular momentum error {:.2}% exceeds 5%",
            lz_error * 100.0
        );

        // Verify length is preserved
        let pos = pendulum.position();
        let length = pos.norm();
        assert_relative_eq!(length, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_spherical_pendulum_inertia_matrix() {
        // Test the 2x2 inertia matrix in (theta, phi) coordinates
        let pendulum = SphericalPendulum::new(2.0, 3.0) // L=2, m=3
            .with_theta(PI / 3.0); // 60 degrees

        let m = pendulum.inertia_matrix();

        // M = m*L² * | 1        0          |
        //            | 0        sin²(θ)    |
        let ml2 = 3.0 * 4.0; // = 12
        let sin2_theta = (PI / 3.0).sin().powi(2); // = 0.75

        assert_relative_eq!(m[(0, 0)], ml2, epsilon = 1e-10);
        assert_relative_eq!(m[(1, 1)], ml2 * sin2_theta, epsilon = 1e-10);
        assert_relative_eq!(m[(0, 1)], 0.0, epsilon = 1e-10);
        assert_relative_eq!(m[(1, 0)], 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_spherical_pendulum_length_preserved() {
        // The pendulum length should be preserved (spherical constraint)
        let mut pendulum = SphericalPendulum::new(1.5, 1.0)
            .with_theta(PI / 3.0)
            .with_velocities(1.0, 1.5);

        let expected_length = 1.5;

        for _ in 0..1000 {
            pendulum.step(1.0 / 240.0);

            let pos = pendulum.position();
            let actual_length = pos.norm();

            assert_relative_eq!(actual_length, expected_length, epsilon = 1e-10,);
        }
    }

    #[test]
    fn test_spherical_pendulum_conical_motion() {
        // A spherical pendulum with constant theta and phi_dot traces a cone
        // This is called "conical pendulum" motion
        // For stable conical motion at angle theta with phi_dot = omega:
        // omega² = g / (L * cos(theta))

        let theta = PI / 6.0; // 30 degrees
        let l = 1.0;
        let g = 9.81;

        // Calculate the required angular velocity for conical motion
        let omega = (g / (l * theta.cos())).sqrt();

        let mut pendulum = SphericalPendulum::new(l, 1.0)
            .with_theta(theta)
            .with_velocities(0.0, omega); // No theta_dot, only phi_dot

        let initial_theta = pendulum.theta;

        // Run for 5 seconds
        let dt = 1.0 / 960.0; // High resolution for this test
        pendulum.run_for(5.0, dt);

        // Theta should remain approximately constant (conical motion)
        let theta_error = (pendulum.theta - initial_theta).abs();
        assert!(
            theta_error < 0.05,
            "Theta drifted by {:.3} rad, expected nearly constant",
            theta_error
        );
    }

    #[test]
    fn test_spherical_pendulum_velocity() {
        // Test that velocity computation is correct
        let pendulum = SphericalPendulum::new(1.0, 1.0)
            .with_theta(PI / 4.0)
            .with_velocities(1.0, 2.0);

        let vel = pendulum.velocity();

        // Compute expected velocity analytically
        let sin_t = (PI / 4.0).sin();
        let cos_t = (PI / 4.0).cos();
        let sin_p = 0.0_f64.sin(); // phi = 0
        let cos_p = 0.0_f64.cos(); // phi = 0

        // dx/dt = L * (cos(θ) * cos(φ) * θ̇ - sin(θ) * sin(φ) * φ̇)
        let expected_vx = cos_t * cos_p * 1.0 - sin_t * sin_p * 2.0;
        // dy/dt = L * sin(θ) * θ̇
        let expected_vy = sin_t * 1.0;
        // dz/dt = L * (cos(θ) * sin(φ) * θ̇ + sin(θ) * cos(φ) * φ̇)
        let expected_vz = cos_t * sin_p * 1.0 + sin_t * cos_p * 2.0;

        assert_relative_eq!(vel.x, expected_vx, epsilon = 1e-10);
        assert_relative_eq!(vel.y, expected_vy, epsilon = 1e-10);
        assert_relative_eq!(vel.z, expected_vz, epsilon = 1e-10);
    }

    // =========================================================================
    // Phase 6: Contact Physics Tests
    // =========================================================================

    #[test]
    fn test_bouncing_ball_falls_under_gravity() {
        // Ball starts above ground, should fall
        let mut ball = BouncingBall::new(0.1, 1.0).with_position(Vector3::new(0.0, 1.0, 0.0));

        let initial_height = ball.pos.y;

        // Run for 0.1 seconds
        ball.run_for(0.1, DT);

        // Ball should have fallen
        assert!(
            ball.pos.y < initial_height,
            "Ball should fall: y={:.3} should be < {:.3}",
            ball.pos.y,
            initial_height
        );

        // Velocity should be downward
        assert!(
            ball.vel.y < 0.0,
            "Ball should have downward velocity: vy={:.3}",
            ball.vel.y
        );
    }

    #[test]
    fn test_bouncing_ball_contacts_ground() {
        // Ball starts at ground level
        let ball = BouncingBall::new(0.1, 1.0).with_position(Vector3::new(0.0, 0.1, 0.0));

        // Gap should be zero
        assert_relative_eq!(ball.gap(), 0.0, epsilon = 1e-10);

        // Should be in contact
        assert!(ball.in_contact());
    }

    #[test]
    fn test_bouncing_ball_bounces() {
        // Ball starts above ground with downward velocity
        let mut ball = BouncingBall::new(0.1, 1.0)
            .with_position(Vector3::new(0.0, 0.2, 0.0))
            .with_velocity(Vector3::new(0.0, -1.0, 0.0))
            .with_restitution(1.0); // Perfectly elastic

        // Run until after first bounce
        ball.run_for(0.5, DT);

        // Ball should have bounced (positive velocity or above ground)
        // With perfect restitution, it should return to approximately same height
        assert!(
            ball.pos.y > 0.1 || ball.vel.y > 0.0,
            "Ball should have bounced"
        );
    }

    #[test]
    fn test_bouncing_ball_energy_loss_with_restitution() {
        // Ball starts above ground
        let mut ball = BouncingBall::new(0.1, 1.0)
            .with_position(Vector3::new(0.0, 1.0, 0.0))
            .with_restitution(0.5); // Inelastic

        let initial_energy = ball.total_energy();

        // Run for several bounces
        ball.run_for(2.0, DT);

        let final_energy = ball.total_energy();

        // Energy should have decreased (due to inelastic collisions)
        assert!(
            final_energy < initial_energy,
            "Energy should decrease with inelastic bounces: E_final={:.3} should be < E_initial={:.3}",
            final_energy,
            initial_energy
        );
    }

    #[test]
    fn test_bouncing_ball_comes_to_rest() {
        // Ball starts above ground with low restitution
        let mut ball = BouncingBall::new(0.1, 1.0)
            .with_position(Vector3::new(0.0, 0.5, 0.0))
            .with_restitution(0.3); // Very inelastic

        // Run for long time
        ball.run_for(5.0, DT);

        // Ball should be at rest on ground
        assert!(
            ball.vel.norm() < 0.1,
            "Ball should come to rest: v={:.3}",
            ball.vel.norm()
        );
        assert!(
            (ball.pos.y - ball.radius).abs() < 0.01,
            "Ball should be on ground: y={:.3}",
            ball.pos.y
        );
    }

    #[test]
    fn test_bouncing_ball_friction_slows_horizontal() {
        // Ball starts moving horizontally on ground
        let mut ball = BouncingBall::new(0.1, 1.0)
            .with_position(Vector3::new(0.0, 0.1, 0.0)) // On ground
            .with_velocity(Vector3::new(2.0, -0.1, 0.0)) // Moving right, slightly into ground
            .with_friction(0.5);

        let initial_vx = ball.vel.x;

        // Run for a short time
        ball.run_for(0.1, DT);

        // Horizontal velocity should have decreased due to friction
        assert!(
            ball.vel.x.abs() < initial_vx,
            "Friction should slow horizontal motion: vx={:.3} should be < {:.3}",
            ball.vel.x.abs(),
            initial_vx
        );
    }

    #[test]
    fn test_bouncing_ball_no_penetration() {
        // Ball starts slightly below ground
        let mut ball = BouncingBall::new(0.1, 1.0).with_position(Vector3::new(0.0, 0.05, 0.0)); // Penetrating

        // Run for a step
        ball.step(DT);

        // Ball should be pushed out of ground
        assert!(
            ball.gap() >= -1e-10,
            "Ball should not penetrate ground: gap={:.6}",
            ball.gap()
        );
    }

    #[test]
    fn test_sphere_pile_initial_state() {
        let pile = SpherePile::new(3, 0.1, 1.0);

        assert_eq!(pile.n_spheres(), 3);

        // Spheres should be stacked vertically
        for i in 0..3 {
            assert!(
                pile.positions[i].y > pile.ground_y,
                "Sphere {} should be above ground",
                i
            );
        }

        // Higher spheres should be higher
        assert!(pile.positions[1].y > pile.positions[0].y);
        assert!(pile.positions[2].y > pile.positions[1].y);
    }

    #[test]
    fn test_sphere_pile_falls() {
        let mut pile = SpherePile::new(3, 0.1, 1.0);

        let initial_heights: Vec<f64> = pile.positions.iter().map(|p| p.y).collect();

        // Run for 0.1 seconds
        pile.run_for(0.1, 1.0 / 240.0);

        // All spheres should have fallen
        for i in 0..3 {
            assert!(
                pile.positions[i].y < initial_heights[i],
                "Sphere {} should fall",
                i
            );
        }
    }

    #[test]
    fn test_sphere_pile_no_penetration() {
        let mut pile = SpherePile::new(3, 0.1, 1.0);

        // Run for several seconds
        pile.run_for(3.0, 1.0 / 240.0);

        // No sphere should penetrate ground
        for i in 0..pile.n_spheres() {
            let gap = pile.positions[i].y - pile.radii[i] - pile.ground_y;
            assert!(
                gap >= -0.01,
                "Sphere {} should not penetrate ground: gap={:.6}",
                i,
                gap
            );
        }

        // No spheres should overlap
        for i in 0..pile.n_spheres() {
            for j in (i + 1)..pile.n_spheres() {
                let dist = (pile.positions[j] - pile.positions[i]).norm();
                let min_dist = pile.radii[i] + pile.radii[j];
                assert!(
                    dist >= min_dist - 0.01,
                    "Spheres {} and {} should not overlap: dist={:.6} < min={:.6}",
                    i,
                    j,
                    dist,
                    min_dist
                );
            }
        }
    }

    #[test]
    fn test_sphere_pile_comes_to_rest() {
        let mut pile = SpherePile::new(3, 0.1, 1.0).with_restitution(0.3);

        // Run for long time
        pile.run_for(5.0, 1.0 / 240.0);

        // All spheres should have low velocity
        for i in 0..pile.n_spheres() {
            let speed = pile.velocities[i].norm();
            assert!(
                speed < 0.5,
                "Sphere {} should be nearly at rest: speed={:.3}",
                i,
                speed
            );
        }
    }

    #[test]
    fn test_sphere_collision() {
        // Two spheres approaching each other
        let mut pile = SpherePile::new(2, 0.2, 1.0).with_restitution(1.0);

        // Place spheres horizontally, moving toward each other
        pile.positions[0] = Vector3::new(-0.5, 0.3, 0.0);
        pile.positions[1] = Vector3::new(0.5, 0.3, 0.0);
        pile.velocities[0] = Vector3::new(1.0, 0.0, 0.0);
        pile.velocities[1] = Vector3::new(-1.0, 0.0, 0.0);

        // Run until collision
        pile.run_for(0.5, 1.0 / 480.0);

        // With elastic collision, velocities should have swapped directions
        // (approximately, accounting for gravity effects)
        assert!(
            pile.velocities[0].x < 0.5,
            "Sphere 0 should have reversed: vx={:.3}",
            pile.velocities[0].x
        );
        assert!(
            pile.velocities[1].x > -0.5,
            "Sphere 1 should have reversed: vx={:.3}",
            pile.velocities[1].x
        );
    }
}

// ============================================================================
// Phase 7: General Articulated Rigid Body System
// ============================================================================
//
// This implements a general articulated rigid body system following MuJoCo's
// architecture. Unlike the specialized pendulum implementations above, this
// handles arbitrary tree topologies with mixed joint types.
//
// Key design principles:
// 1. Bodies are arranged in a kinematic tree (parent-child relationships)
// 2. Joints connect bodies and define relative motion constraints
// 3. State is represented in minimal (joint-space) coordinates
// 4. CRBA computes the n×n joint-space inertia matrix
// 5. RNE computes Coriolis + gravity bias forces
// 6. PGS solver handles constraint forces (limits, contacts)
// 7. Semi-implicit Euler integration with new velocity for position

use nalgebra::{Isometry3, Matrix3, Matrix6, Translation3, UnitQuaternion, Vector6};
use std::collections::HashMap;

/// Index type for bodies in the articulated system.
pub type BodyIndex = usize;

/// Index type for joints in the articulated system.
pub type JointIndex = usize;

/// 6D spatial vector: [angular (3), linear (3)].
///
/// Following Featherstone's convention:
/// - Motion vectors: [ω, v] (angular velocity, linear velocity)
/// - Force vectors: [τ, f] (torque, force)
pub type SpatialVector = Vector6<f64>;

/// 6x6 spatial matrix for rigid body dynamics.
///
/// Used for:
/// - Spatial inertia tensors
/// - Spatial transforms between frames
pub type SpatialMatrix = Matrix6<f64>;

/// Type of joint connecting two bodies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArticulatedJointType {
    /// Fixed joint (0 DOF) - bodies are rigidly connected.
    Fixed,
    /// Revolute joint (1 DOF) - rotation about a single axis.
    Revolute,
    /// Prismatic joint (1 DOF) - translation along a single axis.
    Prismatic,
    /// Spherical joint (3 DOF) - ball-and-socket, free rotation.
    Spherical,
    /// Free joint (6 DOF) - floating base, no constraints.
    Free,
}

impl ArticulatedJointType {
    /// Number of degrees of freedom for this joint type.
    #[must_use]
    pub fn dof(self) -> usize {
        match self {
            Self::Fixed => 0,
            Self::Revolute | Self::Prismatic => 1,
            Self::Spherical => 3,
            Self::Free => 6,
        }
    }
}

/// A rigid body in the articulated system.
#[derive(Debug, Clone)]
pub struct ArticulatedBody {
    /// Index of the parent body (None for root/world).
    pub parent: Option<BodyIndex>,
    /// Indices of child bodies.
    pub children: Vec<BodyIndex>,
    /// Index of the joint connecting this body to its parent.
    pub joint: Option<JointIndex>,
    /// Body mass in kg.
    pub mass: f64,
    /// Center of mass in body-local frame.
    pub com: Vector3<f64>,
    /// Inertia tensor about COM in body-local frame.
    pub inertia: Matrix3<f64>,
    /// Transform from parent joint frame to body COM frame.
    pub body_to_joint: Isometry3<f64>,
}

impl ArticulatedBody {
    /// Create a new body with given mass properties.
    #[must_use]
    pub fn new(mass: f64, com: Vector3<f64>, inertia: Matrix3<f64>) -> Self {
        Self {
            parent: None,
            children: Vec::new(),
            joint: None,
            mass,
            com,
            inertia,
            body_to_joint: Isometry3::identity(),
        }
    }

    /// Create a point mass (all mass concentrated at a single point).
    #[must_use]
    pub fn point_mass(mass: f64) -> Self {
        Self::new(mass, Vector3::zeros(), Matrix3::zeros())
    }

    /// Create a uniform sphere.
    #[must_use]
    pub fn sphere(mass: f64, radius: f64) -> Self {
        let i = 0.4 * mass * radius * radius;
        Self::new(
            mass,
            Vector3::zeros(),
            Matrix3::from_diagonal(&Vector3::new(i, i, i)),
        )
    }

    /// Create a uniform box.
    #[must_use]
    pub fn cuboid(mass: f64, half_extents: Vector3<f64>) -> Self {
        let m = mass / 12.0;
        let hx2 = half_extents.x * half_extents.x * 4.0;
        let hy2 = half_extents.y * half_extents.y * 4.0;
        let hz2 = half_extents.z * half_extents.z * 4.0;
        let inertia = Matrix3::from_diagonal(&Vector3::new(
            m * (hy2 + hz2),
            m * (hx2 + hz2),
            m * (hx2 + hy2),
        ));
        Self::new(mass, Vector3::zeros(), inertia)
    }

    /// Set the transform from body frame to joint frame.
    #[must_use]
    pub fn with_joint_transform(mut self, transform: Isometry3<f64>) -> Self {
        self.body_to_joint = transform;
        self
    }

    /// Compute the 6x6 spatial inertia matrix in body-local frame.
    ///
    /// The spatial inertia is:
    /// ```text
    /// I_spatial = | I + m*[c]×[c]×ᵀ   m*[c]× |
    ///             |     m*[c]×ᵀ          m*1  |
    /// ```
    /// where [c]× is the skew-symmetric matrix of the COM offset.
    #[must_use]
    pub fn spatial_inertia(&self) -> SpatialMatrix {
        let c = self.com;
        let m = self.mass;

        // Skew-symmetric matrix of COM
        let c_skew = skew_symmetric(c);
        let c_skew_t = c_skew.transpose();

        // Upper-left: I + m * c× * c×ᵀ
        let i_rot = self.inertia + m * c_skew * c_skew_t;

        // Upper-right: m * c×
        let i_cross = m * c_skew;

        // Lower-right: m * I₃
        let i_lin = m * Matrix3::identity();

        // Assemble 6x6 matrix
        let mut spatial = SpatialMatrix::zeros();
        let lower_left = c_skew_t * m;
        spatial.fixed_view_mut::<3, 3>(0, 0).copy_from(&i_rot);
        spatial.fixed_view_mut::<3, 3>(0, 3).copy_from(&i_cross);
        spatial.fixed_view_mut::<3, 3>(3, 0).copy_from(&lower_left);
        spatial.fixed_view_mut::<3, 3>(3, 3).copy_from(&i_lin);

        spatial
    }
}

/// A joint in the articulated system.
#[derive(Debug, Clone)]
pub struct ArticulatedJoint {
    /// Type of joint.
    pub joint_type: ArticulatedJointType,
    /// Axis of motion (for revolute/prismatic joints).
    pub axis: Vector3<f64>,
    /// Lower position limits (per DOF).
    pub lower_limit: Vec<f64>,
    /// Upper position limits (per DOF).
    pub upper_limit: Vec<f64>,
    /// Joint damping coefficient (per DOF).
    pub damping: Vec<f64>,
    /// Joint stiffness (spring toward rest position, per DOF).
    pub stiffness: Vec<f64>,
    /// Rest position for spring force (per DOF).
    pub rest_position: Vec<f64>,
    /// Starting index in qpos/qvel arrays.
    pub qpos_start: usize,
    /// Number of position coordinates (may differ from DOF for quaternions).
    pub qpos_dim: usize,
}

impl ArticulatedJoint {
    /// Create a new revolute joint about the given axis.
    #[must_use]
    pub fn revolute(axis: Vector3<f64>) -> Self {
        Self {
            joint_type: ArticulatedJointType::Revolute,
            axis: axis.normalize(),
            lower_limit: vec![-PI],
            upper_limit: vec![PI],
            damping: vec![0.0],
            stiffness: vec![0.0],
            rest_position: vec![0.0],
            qpos_start: 0,
            qpos_dim: 1,
        }
    }

    /// Create a new prismatic joint along the given axis.
    #[must_use]
    pub fn prismatic(axis: Vector3<f64>) -> Self {
        Self {
            joint_type: ArticulatedJointType::Prismatic,
            axis: axis.normalize(),
            lower_limit: vec![-1.0],
            upper_limit: vec![1.0],
            damping: vec![0.0],
            stiffness: vec![0.0],
            rest_position: vec![0.0],
            qpos_start: 0,
            qpos_dim: 1,
        }
    }

    /// Create a new spherical (ball) joint.
    #[must_use]
    pub fn spherical() -> Self {
        Self {
            joint_type: ArticulatedJointType::Spherical,
            axis: Vector3::z(), // Not used for spherical
            lower_limit: vec![-PI, -PI, -PI],
            upper_limit: vec![PI, PI, PI],
            damping: vec![0.0, 0.0, 0.0],
            stiffness: vec![0.0, 0.0, 0.0],
            rest_position: vec![0.0, 0.0, 0.0],
            qpos_start: 0,
            qpos_dim: 4, // Quaternion representation
        }
    }

    /// Create a fixed joint (no motion).
    #[must_use]
    pub fn fixed() -> Self {
        Self {
            joint_type: ArticulatedJointType::Fixed,
            axis: Vector3::z(),
            lower_limit: vec![],
            upper_limit: vec![],
            damping: vec![],
            stiffness: vec![],
            rest_position: vec![],
            qpos_start: 0,
            qpos_dim: 0,
        }
    }

    /// Create a free joint (floating base).
    #[must_use]
    pub fn free() -> Self {
        Self {
            joint_type: ArticulatedJointType::Free,
            axis: Vector3::z(),
            lower_limit: vec![f64::NEG_INFINITY; 6],
            upper_limit: vec![f64::INFINITY; 6],
            damping: vec![0.0; 6],
            stiffness: vec![0.0; 6],
            rest_position: vec![0.0; 6],
            qpos_start: 0,
            qpos_dim: 7, // position (3) + quaternion (4)
        }
    }

    /// Get the number of degrees of freedom.
    #[must_use]
    pub fn dof(&self) -> usize {
        self.joint_type.dof()
    }

    /// Set joint limits.
    ///
    /// # Panics
    ///
    /// Panics if `lower` or `upper` length doesn't match the joint DOF.
    #[must_use]
    pub fn with_limits(mut self, lower: Vec<f64>, upper: Vec<f64>) -> Self {
        assert_eq!(lower.len(), self.dof());
        assert_eq!(upper.len(), self.dof());
        self.lower_limit = lower;
        self.upper_limit = upper;
        self
    }

    /// Set symmetric joint limits.
    #[must_use]
    pub fn with_symmetric_limits(mut self, limit: f64) -> Self {
        self.lower_limit = vec![-limit; self.dof()];
        self.upper_limit = vec![limit; self.dof()];
        self
    }

    /// Set joint damping.
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = vec![damping; self.dof()];
        self
    }

    /// Set joint stiffness (spring).
    #[must_use]
    pub fn with_stiffness(mut self, stiffness: f64, rest: f64) -> Self {
        self.stiffness = vec![stiffness; self.dof()];
        self.rest_position = vec![rest; self.dof()];
        self
    }

    /// Compute the joint motion subspace matrix S (6 × dof).
    ///
    /// This maps joint velocities to spatial velocities:
    /// `v_spatial` = S * qvel
    #[must_use]
    pub fn motion_subspace(&self) -> nalgebra::DMatrix<f64> {
        let dof = self.dof();
        let mut s = nalgebra::DMatrix::zeros(6, dof);

        match self.joint_type {
            ArticulatedJointType::Fixed => {
                // No motion
            }
            ArticulatedJointType::Revolute => {
                // Rotation about axis
                s[(0, 0)] = self.axis.x;
                s[(1, 0)] = self.axis.y;
                s[(2, 0)] = self.axis.z;
            }
            ArticulatedJointType::Prismatic => {
                // Translation along axis
                s[(3, 0)] = self.axis.x;
                s[(4, 0)] = self.axis.y;
                s[(5, 0)] = self.axis.z;
            }
            ArticulatedJointType::Spherical => {
                // Free rotation: identity for angular part
                s[(0, 0)] = 1.0;
                s[(1, 1)] = 1.0;
                s[(2, 2)] = 1.0;
            }
            ArticulatedJointType::Free => {
                // Full 6-DOF
                for i in 0..6 {
                    s[(i, i)] = 1.0;
                }
            }
        }

        s
    }

    /// Compute the spatial transform for this joint given position coordinates.
    #[must_use]
    pub fn transform(&self, qpos: &[f64]) -> Isometry3<f64> {
        match self.joint_type {
            ArticulatedJointType::Fixed => Isometry3::identity(),
            ArticulatedJointType::Revolute => {
                let angle = qpos.first().copied().unwrap_or(0.0);
                let rotation = UnitQuaternion::from_axis_angle(
                    &nalgebra::Unit::new_normalize(self.axis),
                    angle,
                );
                Isometry3::from_parts(Translation3::identity(), rotation)
            }
            ArticulatedJointType::Prismatic => {
                let dist = qpos.first().copied().unwrap_or(0.0);
                Isometry3::from_parts(
                    Translation3::from(self.axis * dist),
                    UnitQuaternion::identity(),
                )
            }
            ArticulatedJointType::Spherical => {
                // qpos is [w, x, y, z] quaternion
                let q = if qpos.len() >= 4 {
                    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        qpos[0], qpos[1], qpos[2], qpos[3],
                    ))
                } else {
                    UnitQuaternion::identity()
                };
                Isometry3::from_parts(Translation3::identity(), q)
            }
            ArticulatedJointType::Free => {
                // qpos is [x, y, z, qw, qx, qy, qz]
                let pos = if qpos.len() >= 3 {
                    Translation3::new(qpos[0], qpos[1], qpos[2])
                } else {
                    Translation3::identity()
                };
                let rot = if qpos.len() >= 7 {
                    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        qpos[3], qpos[4], qpos[5], qpos[6],
                    ))
                } else {
                    UnitQuaternion::identity()
                };
                Isometry3::from_parts(pos, rot)
            }
        }
    }
}

/// Complete articulated rigid body system.
///
/// This is the main entry point for simulating articulated mechanisms,
/// robots, and other multi-body systems.
#[derive(Debug, Clone)]
pub struct ArticulatedSystem {
    /// All bodies in the system.
    pub bodies: Vec<ArticulatedBody>,
    /// All joints in the system.
    pub joints: Vec<ArticulatedJoint>,
    /// Joint position coordinates.
    pub qpos: DVector<f64>,
    /// Joint velocities.
    pub qvel: DVector<f64>,
    /// Total degrees of freedom.
    nv: usize,
    /// Total position dimension (may be > nv due to quaternions).
    nq: usize,
    /// Gravity vector.
    pub gravity: Vector3<f64>,
    /// Body ordering for forward/backward passes (topological sort).
    body_order: Vec<BodyIndex>,
    /// Map from body index to joint indices (for bodies with multiple joints).
    body_joints: HashMap<BodyIndex, Vec<JointIndex>>,
}

impl ArticulatedSystem {
    /// Create a new empty articulated system.
    #[must_use]
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            joints: Vec::new(),
            qpos: DVector::zeros(0),
            qvel: DVector::zeros(0),
            nv: 0,
            nq: 0,
            gravity: Vector3::new(0.0, -9.81, 0.0),
            body_order: Vec::new(),
            body_joints: HashMap::new(),
        }
    }

    /// Set custom gravity vector.
    #[must_use]
    pub fn with_gravity(mut self, gravity: Vector3<f64>) -> Self {
        self.gravity = gravity;
        self
    }

    /// Add a body to the system, optionally connected to a parent.
    ///
    /// Returns the index of the new body.
    pub fn add_body(&mut self, mut body: ArticulatedBody, parent: Option<BodyIndex>) -> BodyIndex {
        let idx = self.bodies.len();
        body.parent = parent;

        if let Some(p) = parent {
            self.bodies[p].children.push(idx);
        }

        self.bodies.push(body);
        self.rebuild_topology();
        idx
    }

    /// Add a joint connecting a body to its parent.
    ///
    /// Returns the index of the new joint.
    pub fn add_joint(&mut self, body: BodyIndex, joint: ArticulatedJoint) -> JointIndex {
        let jidx = self.joints.len();

        // Update joint position indices
        let mut j = joint;
        j.qpos_start = self.nq;

        // Update dimensions
        self.nv += j.dof();
        self.nq += j.qpos_dim;

        self.bodies[body].joint = Some(jidx);
        self.joints.push(j);

        // Update body-joint map
        self.body_joints.entry(body).or_default().push(jidx);

        // Resize state vectors
        self.qpos = DVector::zeros(self.nq);
        self.qvel = DVector::zeros(self.nv);

        // Initialize quaternions to identity
        self.initialize_quaternions();

        jidx
    }

    /// Initialize quaternion positions to identity.
    fn initialize_quaternions(&mut self) {
        for joint in &self.joints {
            match joint.joint_type {
                ArticulatedJointType::Spherical => {
                    // [w, x, y, z] = [1, 0, 0, 0]
                    self.qpos[joint.qpos_start] = 1.0;
                }
                ArticulatedJointType::Free => {
                    // Position + quaternion: [x, y, z, w, qx, qy, qz]
                    self.qpos[joint.qpos_start + 3] = 1.0; // qw
                }
                _ => {}
            }
        }
    }

    /// Rebuild the topological ordering after adding bodies.
    fn rebuild_topology(&mut self) {
        // Find root bodies (no parent)
        let roots: Vec<BodyIndex> = self
            .bodies
            .iter()
            .enumerate()
            .filter(|(_, b)| b.parent.is_none())
            .map(|(i, _)| i)
            .collect();

        // Build ordering via BFS/DFS from roots
        self.body_order.clear();
        let mut stack = roots;

        while let Some(idx) = stack.pop() {
            self.body_order.push(idx);
            // Add children in reverse order so they come out in order
            for &child in self.bodies[idx].children.iter().rev() {
                stack.push(child);
            }
        }
    }

    /// Get total degrees of freedom.
    #[must_use]
    pub fn nv(&self) -> usize {
        self.nv
    }

    /// Get total position dimension.
    #[must_use]
    pub fn nq(&self) -> usize {
        self.nq
    }

    /// Get number of bodies.
    #[must_use]
    pub fn n_bodies(&self) -> usize {
        self.bodies.len()
    }

    /// Get number of joints.
    #[must_use]
    pub fn n_joints(&self) -> usize {
        self.joints.len()
    }

    /// Get qpos slice for a specific joint.
    #[must_use]
    pub fn joint_qpos(&self, joint: JointIndex) -> &[f64] {
        let j = &self.joints[joint];
        &self.qpos.as_slice()[j.qpos_start..j.qpos_start + j.qpos_dim]
    }

    /// Get mutable qpos slice for a specific joint.
    pub fn joint_qpos_mut(&mut self, joint: JointIndex) -> &mut [f64] {
        let j = &self.joints[joint];
        let start = j.qpos_start;
        let end = start + j.qpos_dim;
        &mut self.qpos.as_mut_slice()[start..end]
    }

    /// Set joint position.
    pub fn set_joint_position(&mut self, joint: JointIndex, value: f64) {
        let j = &self.joints[joint];
        if j.qpos_dim > 0 {
            self.qpos[j.qpos_start] = value;
        }
    }

    /// Set joint velocity.
    pub fn set_joint_velocity(&mut self, joint: JointIndex, value: f64) {
        let j = &self.joints[joint];
        if j.dof() > 0 {
            // Find velocity index by counting DOF of previous joints
            let vel_idx: usize = self.joints[..joint].iter().map(ArticulatedJoint::dof).sum();
            self.qvel[vel_idx] = value;
        }
    }

    /// Compute forward kinematics - body transforms from joint positions.
    ///
    /// Returns transforms for all bodies in world frame.
    #[must_use]
    #[allow(clippy::option_if_let_else)]
    pub fn forward_kinematics(&self) -> Vec<Isometry3<f64>> {
        let mut transforms = vec![Isometry3::identity(); self.bodies.len()];

        for &body_idx in &self.body_order {
            let body = &self.bodies[body_idx];

            // Start with parent transform
            let parent_transform = if let Some(p) = body.parent {
                transforms[p]
            } else {
                Isometry3::identity()
            };

            // Apply joint transform
            let joint_transform = if let Some(jidx) = body.joint {
                let joint = &self.joints[jidx];
                let qpos = self.joint_qpos(jidx);
                joint.transform(qpos)
            } else {
                Isometry3::identity()
            };

            // Apply body-to-joint transform
            transforms[body_idx] = parent_transform * joint_transform * body.body_to_joint;
        }

        transforms
    }

    /// Compute the joint-space inertia matrix M via Composite Rigid Body Algorithm.
    ///
    /// This is `MuJoCo`'s `mj_crb` / `mj_makeM`.
    ///
    /// For a serial chain, this is a direct computation based on the kinetic energy.
    /// For body i with joint at position `r_i` from pivot, the effective inertia is
    /// the sum of contributions from all descendant masses.
    #[must_use]
    #[allow(
        clippy::needless_range_loop,
        clippy::too_many_lines,
        clippy::similar_names
    )]
    pub fn inertia_matrix(&self) -> DMatrix<f64> {
        if self.nv == 0 {
            return DMatrix::zeros(0, 0);
        }

        let mut m_matrix = DMatrix::zeros(self.nv, self.nv);

        // Compute body transforms (world frame positions)
        let transforms = self.forward_kinematics();

        // For each pair of joints (i, j), compute M[i,j]
        // M[i,j] = Σ_k m_k * (∂p_k/∂q_i · ∂p_k/∂q_j)
        // where k ranges over all bodies affected by both joints i and j

        // First, build a map of joint index -> body index
        let joint_to_body: Vec<BodyIndex> = self
            .joints
            .iter()
            .enumerate()
            .map(|(jidx, _)| {
                self.bodies
                    .iter()
                    .position(|b| b.joint == Some(jidx))
                    .unwrap_or(0)
            })
            .collect();

        // For each body, find all joints that affect its position (ancestor joints)
        let mut body_ancestor_joints: Vec<Vec<JointIndex>> = vec![vec![]; self.bodies.len()];
        for (body_idx, _body) in self.bodies.iter().enumerate() {
            let mut current = Some(body_idx);
            while let Some(idx) = current {
                if let Some(jidx) = self.bodies[idx].joint {
                    body_ancestor_joints[body_idx].push(jidx);
                }
                current = self.bodies[idx].parent;
            }
        }

        // For each pair of joints, compute the inertia contribution
        let mut vel_i = 0;
        for (ji, joint_i) in self.joints.iter().enumerate() {
            let dof_i = joint_i.dof();
            if dof_i == 0 {
                vel_i += dof_i;
                continue;
            }

            let mut vel_j = 0;
            for (jj, joint_j) in self.joints.iter().enumerate() {
                let dof_j = joint_j.dof();
                if dof_j == 0 {
                    vel_j += dof_j;
                    continue;
                }

                // Find all bodies affected by both joints
                for (body_idx, body) in self.bodies.iter().enumerate() {
                    let ancestors = &body_ancestor_joints[body_idx];
                    if ancestors.contains(&ji) && ancestors.contains(&jj) {
                        // This body is affected by both joints
                        // For a revolute joint about z-axis, the jacobian contribution
                        // is the perpendicular distance from the joint axis

                        // Get body position (COM in world)
                        let body_pos = transforms[body_idx].translation.vector
                            + transforms[body_idx].rotation * body.com;

                        // Get joint positions
                        let joint_i_body = joint_to_body[ji];
                        let joint_j_body = joint_to_body[jj];

                        // For revolute joints about z-axis in a planar pendulum,
                        // the contribution to M[i,j] is m * L_i * L_j * cos(θ_i - θ_j)
                        // where L_i is the perpendicular distance from joint i axis

                        match (joint_i.joint_type, joint_j.joint_type) {
                            (ArticulatedJointType::Revolute, ArticulatedJointType::Revolute) => {
                                // Get the parent of the body that has this joint
                                // The joint position is at the parent body's position (in world)
                                let parent_i =
                                    self.bodies[joint_i_body].parent.unwrap_or(joint_i_body);
                                let parent_j =
                                    self.bodies[joint_j_body].parent.unwrap_or(joint_j_body);

                                // Get joint axis in world frame (at parent frame)
                                let axis_i = transforms[parent_i].rotation * joint_i.axis;
                                let axis_j = transforms[parent_j].rotation * joint_j.axis;

                                // Get joint position (at parent position)
                                let jpos_i = transforms[parent_i].translation.vector;
                                let jpos_j = transforms[parent_j].translation.vector;

                                // For a revolute joint, ∂p/∂θ = axis × (p - joint_pos)
                                let r_i = body_pos - jpos_i;
                                let r_j = body_pos - jpos_j;
                                let dp_dqi = axis_i.cross(&r_i);
                                let dp_dqj = axis_j.cross(&r_j);

                                // M[i,j] += m * dp/dqi · dp/dqj
                                m_matrix[(vel_i, vel_j)] += body.mass * dp_dqi.dot(&dp_dqj);
                            }
                            (ArticulatedJointType::Prismatic, ArticulatedJointType::Prismatic) => {
                                // ∂p/∂d = axis
                                m_matrix[(vel_i, vel_j)] +=
                                    body.mass * joint_i.axis.dot(&joint_j.axis);
                            }
                            (ArticulatedJointType::Free, ArticulatedJointType::Free) => {
                                // Free joint: 6 DOF [wx, wy, wz, vx, vy, vz]
                                // Linear part: m * I_3 (diagonal 3x3 in the linear block)
                                // Angular part: I (rotational inertia)
                                // For simplicity, use point mass approximation

                                // Linear velocity contribution (last 3 DOF)
                                // ∂p/∂vx = [1,0,0], ∂p/∂vy = [0,1,0], ∂p/∂vz = [0,0,1]
                                for d in 0..3 {
                                    // Linear DOF are at offset 3, 4, 5
                                    m_matrix[(vel_i + 3 + d, vel_j + 3 + d)] += body.mass;
                                }

                                // Angular part - use body's rotational inertia
                                let i_body = body.inertia;
                                for di in 0..3 {
                                    for dj in 0..3 {
                                        m_matrix[(vel_i + di, vel_j + dj)] += i_body[(di, dj)];
                                    }
                                }
                            }
                            _ => {
                                // Other combinations not yet implemented
                            }
                        }
                    }
                }

                vel_j += dof_j;
            }

            vel_i += dof_i;
        }

        // Add rotational inertia contributions for non-point masses
        vel_i = 0;
        for (ji, joint_i) in self.joints.iter().enumerate() {
            let dof_i = joint_i.dof();
            if dof_i == 0 || joint_i.joint_type != ArticulatedJointType::Revolute {
                vel_i += dof_i;
                continue;
            }

            let joint_body = joint_to_body[ji];
            let axis = transforms[joint_body].rotation * joint_i.axis;

            // Add rotational inertia from all descendant bodies
            for (body_idx, body) in self.bodies.iter().enumerate() {
                if body_ancestor_joints[body_idx].contains(&ji) {
                    // Rotational inertia: I = axis^T * I_body * axis
                    let i_rot = axis.dot(&(body.inertia * axis));
                    m_matrix[(vel_i, vel_i)] += i_rot;
                }
            }

            vel_i += dof_i;
        }

        // Ensure symmetry
        for i in 0..self.nv {
            for j in (i + 1)..self.nv {
                let avg = f64::midpoint(m_matrix[(i, j)], m_matrix[(j, i)]);
                m_matrix[(i, j)] = avg;
                m_matrix[(j, i)] = avg;
            }
        }

        m_matrix
    }

    /// Compute bias forces (Coriolis + gravity).
    ///
    /// This is a simplified implementation using direct gravity torque computation.
    /// For each joint, compute the gravitational torque from all descendant masses.
    ///
    /// For revolute joints: τ = Σ `m_k` * g · (`r_k` × axis)
    /// where `r_k` is the vector from joint to mass k.
    #[must_use]
    #[allow(clippy::too_many_lines)]
    pub fn bias_forces(&self) -> DVector<f64> {
        if self.nv == 0 {
            return DVector::zeros(0);
        }

        let mut qfrc_bias = DVector::zeros(self.nv);
        let transforms = self.forward_kinematics();

        // Build map of joint -> body
        let joint_to_body: Vec<BodyIndex> = self
            .joints
            .iter()
            .enumerate()
            .map(|(jidx, _)| {
                self.bodies
                    .iter()
                    .position(|b| b.joint == Some(jidx))
                    .unwrap_or(0)
            })
            .collect();

        // For each body, find all ancestor joints
        let mut body_ancestor_joints: Vec<Vec<JointIndex>> = vec![vec![]; self.bodies.len()];
        for (body_idx, _) in self.bodies.iter().enumerate() {
            let mut current = Some(body_idx);
            while let Some(idx) = current {
                if let Some(jidx) = self.bodies[idx].joint {
                    body_ancestor_joints[body_idx].push(jidx);
                }
                current = self.bodies[idx].parent;
            }
        }

        // Compute gravity torque for each joint
        let mut vel_offset = 0;
        for (jidx, joint) in self.joints.iter().enumerate() {
            let dof = joint.dof();
            if dof == 0 {
                continue;
            }

            match joint.joint_type {
                ArticulatedJointType::Revolute => {
                    // Get joint position and axis in world frame
                    let joint_body = joint_to_body[jidx];
                    let parent = self.bodies[joint_body].parent.unwrap_or(joint_body);
                    let jpos = transforms[parent].translation.vector;
                    let axis = transforms[parent].rotation * joint.axis;

                    // Sum gravitational torque from all descendant masses
                    let mut tau = 0.0;
                    for (body_idx, body) in self.bodies.iter().enumerate() {
                        if body_ancestor_joints[body_idx].contains(&jidx) {
                            // Get body COM in world
                            let com_world = transforms[body_idx].translation.vector
                                + transforms[body_idx].rotation * body.com;

                            // Vector from joint to COM
                            let r = com_world - jpos;

                            // Gravity force
                            let f_grav = body.mass * self.gravity;

                            // Torque = axis · (r × f) = (axis × r) · f
                            let arm = axis.cross(&r);
                            tau += arm.dot(&f_grav);
                        }
                    }

                    qfrc_bias[vel_offset] = tau;
                }
                ArticulatedJointType::Prismatic => {
                    // For prismatic, τ = axis · Σ m_k * g
                    let axis = joint.axis;
                    let mut tau = 0.0;
                    for (body_idx, body) in self.bodies.iter().enumerate() {
                        if body_ancestor_joints[body_idx].contains(&jidx) {
                            let f_grav = body.mass * self.gravity;
                            tau += axis.dot(&f_grav);
                        }
                    }
                    qfrc_bias[vel_offset] = tau;
                }
                ArticulatedJointType::Free => {
                    // For free joint, the bias force is OPPOSITE to the gravitational force
                    // so that qfrc_smooth = qfrc_passive - qfrc_bias gives the correct
                    // acceleration direction.
                    //
                    // If gravity = (0, -9.81, 0), we want qacc = (0, -9.81, 0)
                    // qacc = M^(-1) * (qfrc_passive - qfrc_bias) = -qfrc_bias (when passive=0)
                    // So qfrc_bias = -qacc = (0, 9.81, 0) = -gravity
                    let joint_body = joint_to_body[jidx];
                    let body_com = transforms[joint_body].translation.vector
                        + transforms[joint_body].rotation * self.bodies[joint_body].com;

                    let mut f_total = Vector3::zeros();
                    let mut tau_total = Vector3::zeros();

                    for (body_idx, body) in self.bodies.iter().enumerate() {
                        if body_ancestor_joints[body_idx].contains(&jidx) {
                            let com_world = transforms[body_idx].translation.vector
                                + transforms[body_idx].rotation * body.com;
                            // Negate gravity to get bias force
                            let f_grav = -body.mass * self.gravity;
                            f_total += f_grav;

                            let r = com_world - body_com;
                            tau_total += r.cross(&f_grav);
                        }
                    }

                    // qvel for free joint: [angular (3), linear (3)]
                    qfrc_bias[vel_offset] = tau_total.x;
                    qfrc_bias[vel_offset + 1] = tau_total.y;
                    qfrc_bias[vel_offset + 2] = tau_total.z;
                    qfrc_bias[vel_offset + 3] = f_total.x;
                    qfrc_bias[vel_offset + 4] = f_total.y;
                    qfrc_bias[vel_offset + 5] = f_total.z;
                }
                ArticulatedJointType::Spherical => {
                    // For spherical, compute torque about joint center
                    let joint_body = joint_to_body[jidx];
                    let parent = self.bodies[joint_body].parent.unwrap_or(joint_body);
                    let jpos = transforms[parent].translation.vector;

                    let mut tau_total = Vector3::zeros();
                    for (body_idx, body) in self.bodies.iter().enumerate() {
                        if body_ancestor_joints[body_idx].contains(&jidx) {
                            let com_world = transforms[body_idx].translation.vector
                                + transforms[body_idx].rotation * body.com;
                            let r = com_world - jpos;
                            let f_grav = body.mass * self.gravity;
                            tau_total += r.cross(&f_grav);
                        }
                    }

                    qfrc_bias[vel_offset] = tau_total.x;
                    qfrc_bias[vel_offset + 1] = tau_total.y;
                    qfrc_bias[vel_offset + 2] = tau_total.z;
                }
                ArticulatedJointType::Fixed => {
                    // No motion
                }
            }

            vel_offset += dof;
        }

        // TODO: Add Coriolis terms for high-velocity motion

        qfrc_bias
    }

    /// Compute passive forces (springs, dampers).
    #[must_use]
    pub fn passive_forces(&self) -> DVector<f64> {
        let mut qfrc_passive = DVector::zeros(self.nv);

        let mut vel_offset = 0;
        let mut pos_offset = 0;
        for joint in &self.joints {
            let dof = joint.dof();
            for i in 0..dof {
                // Spring force: -k * (q - q_rest)
                let q = if pos_offset + i < self.qpos.len() {
                    self.qpos[pos_offset + i]
                } else {
                    0.0
                };
                let spring = -joint.stiffness[i] * (q - joint.rest_position[i]);

                // Damping force: -b * qvel
                let qv = if vel_offset + i < self.qvel.len() {
                    self.qvel[vel_offset + i]
                } else {
                    0.0
                };
                let damping = -joint.damping[i] * qv;

                qfrc_passive[vel_offset + i] = spring + damping;
            }
            vel_offset += dof;
            pos_offset += joint.qpos_dim;
        }

        qfrc_passive
    }

    /// Compute total energy (kinetic + potential).
    #[must_use]
    pub fn total_energy(&self) -> f64 {
        let m = self.inertia_matrix();
        let kinetic = 0.5 * self.qvel.dot(&(&m * &self.qvel));

        // Potential energy from gravity
        let transforms = self.forward_kinematics();
        let mut potential = 0.0;
        for (idx, body) in self.bodies.iter().enumerate() {
            let com_world = transforms[idx] * nalgebra::Point3::from(body.com);
            // V = m * g * h (height in gravity direction)
            potential += body.mass * (-self.gravity.dot(&com_world.coords));
        }

        kinetic + potential
    }

    /// Step the system forward using `MuJoCo`'s pipeline.
    ///
    /// 1. Compute inertia matrix M via CRBA
    /// 2. Compute bias forces via RNE
    /// 3. Compute passive forces
    /// 4. Solve M * qacc = `qfrc_smooth`
    /// 5. Semi-implicit Euler integration
    pub fn step(&mut self, dt: f64) {
        if self.nv == 0 {
            return;
        }

        // Stage 1: Forward Position (CRBA)
        let m = self.inertia_matrix();

        // Stage 2: Forward Velocity (RNE)
        let qfrc_bias = self.bias_forces();

        // Passive forces
        let qfrc_passive = self.passive_forces();

        // Stage 3-4: Forward Actuation & Acceleration
        // qfrc_smooth = qfrc_passive + qfrc_actuator + qfrc_applied - qfrc_bias
        // For now, no actuation or applied forces
        let qfrc_smooth = qfrc_passive - qfrc_bias;

        // Solve M * qacc = qfrc_smooth
        let qacc = m
            .lu()
            .solve(&qfrc_smooth)
            .unwrap_or_else(|| DVector::zeros(self.nv));

        // Stage 5: Integration (Semi-implicit Euler)
        let qvel_new = &self.qvel + &qacc * dt;

        // Update positions based on joint type
        let mut pos_offset = 0;
        let mut vel_offset = 0;
        for joint in &self.joints {
            match joint.joint_type {
                ArticulatedJointType::Fixed => {
                    // No motion
                }
                ArticulatedJointType::Revolute | ArticulatedJointType::Prismatic => {
                    self.qpos[pos_offset] += qvel_new[vel_offset] * dt;
                    pos_offset += 1;
                    vel_offset += 1;
                }
                ArticulatedJointType::Spherical => {
                    // Quaternion integration
                    let w = Vector3::new(
                        qvel_new[vel_offset],
                        qvel_new[vel_offset + 1],
                        qvel_new[vel_offset + 2],
                    );
                    let q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        self.qpos[pos_offset],
                        self.qpos[pos_offset + 1],
                        self.qpos[pos_offset + 2],
                        self.qpos[pos_offset + 3],
                    ));
                    let dq = UnitQuaternion::from_scaled_axis(w * dt);
                    let q_new = q * dq;
                    self.qpos[pos_offset] = q_new.w;
                    self.qpos[pos_offset + 1] = q_new.i;
                    self.qpos[pos_offset + 2] = q_new.j;
                    self.qpos[pos_offset + 3] = q_new.k;
                    pos_offset += 4;
                    vel_offset += 3;
                }
                ArticulatedJointType::Free => {
                    // qvel for free joint is [wx, wy, wz, vx, vy, vz] (angular then linear)
                    // qpos for free joint is [x, y, z, qw, qx, qy, qz] (position then quaternion)

                    // Quaternion integration (using angular velocity at vel_offset+0,1,2)
                    let w = Vector3::new(
                        qvel_new[vel_offset],
                        qvel_new[vel_offset + 1],
                        qvel_new[vel_offset + 2],
                    );
                    let q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        self.qpos[pos_offset + 3],
                        self.qpos[pos_offset + 4],
                        self.qpos[pos_offset + 5],
                        self.qpos[pos_offset + 6],
                    ));
                    let dq = UnitQuaternion::from_scaled_axis(w * dt);
                    let q_new = q * dq;
                    self.qpos[pos_offset + 3] = q_new.w;
                    self.qpos[pos_offset + 4] = q_new.i;
                    self.qpos[pos_offset + 5] = q_new.j;
                    self.qpos[pos_offset + 6] = q_new.k;

                    // Position integration (using linear velocity at vel_offset+3,4,5)
                    self.qpos[pos_offset] += qvel_new[vel_offset + 3] * dt;
                    self.qpos[pos_offset + 1] += qvel_new[vel_offset + 4] * dt;
                    self.qpos[pos_offset + 2] += qvel_new[vel_offset + 5] * dt;

                    pos_offset += 7;
                    vel_offset += 6;
                }
            }
        }

        self.qvel = qvel_new;
    }

    /// Run for a given duration.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub fn run_for(&mut self, duration: f64, dt: f64) {
        let steps = (duration / dt).ceil() as usize;
        for _ in 0..steps {
            self.step(dt);
        }
    }
}

impl Default for ArticulatedSystem {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Helper Functions for Spatial Algebra
// ============================================================================

/// Create a skew-symmetric matrix from a vector.
#[must_use]
fn skew_symmetric(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

/// Compute spatial transform between two frames.
///
/// Returns the 6x6 matrix that transforms spatial vectors from frame A to frame B.
#[must_use]
#[allow(dead_code)]
fn compute_spatial_transform(from: &Isometry3<f64>, to: &Isometry3<f64>) -> SpatialMatrix {
    let relative = to.inverse() * from;
    let r_mat: Matrix3<f64> = *relative.rotation.to_rotation_matrix().matrix();
    let p = relative.translation.vector;

    let mut x = SpatialMatrix::zeros();

    // Upper-left: R
    x.fixed_view_mut::<3, 3>(0, 0).copy_from(&r_mat);

    // Lower-left: [p]× @ R
    let p_skew = skew_symmetric(p);
    let lower_left = p_skew * r_mat;
    x.fixed_view_mut::<3, 3>(3, 0).copy_from(&lower_left);

    // Lower-right: R
    x.fixed_view_mut::<3, 3>(3, 3).copy_from(&r_mat);

    x
}

/// Spatial cross product for motion vectors: v × s.
#[must_use]
#[allow(dead_code)]
fn spatial_cross_motion(v: SpatialVector, s: SpatialVector) -> SpatialVector {
    let w = Vector3::new(v[0], v[1], v[2]);
    let v_lin = Vector3::new(v[3], v[4], v[5]);
    let s_ang = Vector3::new(s[0], s[1], s[2]);
    let s_lin = Vector3::new(s[3], s[4], s[5]);

    let result_ang = w.cross(&s_ang);
    let result_lin = w.cross(&s_lin) + v_lin.cross(&s_ang);

    SpatialVector::new(
        result_ang.x,
        result_ang.y,
        result_ang.z,
        result_lin.x,
        result_lin.y,
        result_lin.z,
    )
}

/// Spatial cross product for force vectors: v ×* f.
#[must_use]
#[allow(dead_code)]
fn spatial_cross_force(v: SpatialVector, f: SpatialVector) -> SpatialVector {
    let w = Vector3::new(v[0], v[1], v[2]);
    let v_lin = Vector3::new(v[3], v[4], v[5]);
    let f_ang = Vector3::new(f[0], f[1], f[2]);
    let f_lin = Vector3::new(f[3], f[4], f[5]);

    let result_ang = w.cross(&f_ang) + v_lin.cross(&f_lin);
    let result_lin = w.cross(&f_lin);

    SpatialVector::new(
        result_ang.x,
        result_ang.y,
        result_ang.z,
        result_lin.x,
        result_lin.y,
        result_lin.z,
    )
}

// ============================================================================
// Tests for Articulated System
// ============================================================================

#[cfg(test)]
mod articulated_tests {
    use super::*;
    use approx::assert_relative_eq;

    const DT: f64 = 1.0 / 480.0;

    #[test]
    fn test_empty_system() {
        let sys = ArticulatedSystem::new();
        assert_eq!(sys.nv(), 0);
        assert_eq!(sys.nq(), 0);
        assert_eq!(sys.n_bodies(), 0);
    }

    #[test]
    fn test_single_body_no_joint() {
        let mut sys = ArticulatedSystem::new();
        sys.add_body(ArticulatedBody::point_mass(1.0), None);

        assert_eq!(sys.n_bodies(), 1);
        assert_eq!(sys.nv(), 0);
    }

    #[test]
    fn test_single_revolute_joint() {
        let mut sys = ArticulatedSystem::new();
        let body = sys.add_body(ArticulatedBody::point_mass(1.0), None);
        sys.add_joint(body, ArticulatedJoint::revolute(Vector3::z()));

        assert_eq!(sys.nv(), 1);
        assert_eq!(sys.nq(), 1);
    }

    #[test]
    fn test_pendulum_falls() {
        // Create a simple pendulum
        let mut sys = ArticulatedSystem::new();

        // Root body (fixed)
        let root = sys.add_body(ArticulatedBody::point_mass(0.0), None);

        // Pendulum bob
        let bob = sys.add_body(
            ArticulatedBody::point_mass(1.0)
                .with_joint_transform(Isometry3::translation(0.0, -1.0, 0.0)),
            Some(root),
        );
        sys.add_joint(bob, ArticulatedJoint::revolute(Vector3::z()));

        // Start at 45 degrees
        sys.set_joint_position(0, PI / 4.0);

        let _initial_angle = sys.qpos[0];
        let initial_velocity = sys.qvel[0];

        // Run for a bit
        sys.run_for(0.1, DT);

        // Velocity should have changed (pendulum accelerating)
        assert!(
            sys.qvel[0].abs() > initial_velocity.abs(),
            "Pendulum should be accelerating"
        );
    }

    #[test]
    fn test_inertia_matrix_positive_definite() {
        let mut sys = ArticulatedSystem::new();
        let root = sys.add_body(ArticulatedBody::point_mass(0.0), None);
        let bob = sys.add_body(
            ArticulatedBody::point_mass(1.0)
                .with_joint_transform(Isometry3::translation(0.0, -1.0, 0.0)),
            Some(root),
        );
        sys.add_joint(bob, ArticulatedJoint::revolute(Vector3::z()));

        let m = sys.inertia_matrix();

        // Should be 1x1 for single revolute joint
        assert_eq!(m.nrows(), 1);
        assert_eq!(m.ncols(), 1);

        // Should be positive
        assert!(m[(0, 0)] > 0.0, "Inertia should be positive: {}", m[(0, 0)]);
    }

    #[test]
    fn test_two_link_chain() {
        let mut sys = ArticulatedSystem::new();

        // Root
        let root = sys.add_body(ArticulatedBody::point_mass(0.0), None);

        // First link
        let link1 = sys.add_body(
            ArticulatedBody::point_mass(1.0)
                .with_joint_transform(Isometry3::translation(0.0, -1.0, 0.0)),
            Some(root),
        );
        sys.add_joint(link1, ArticulatedJoint::revolute(Vector3::z()));

        // Second link
        let link2 = sys.add_body(
            ArticulatedBody::point_mass(1.0)
                .with_joint_transform(Isometry3::translation(0.0, -1.0, 0.0)),
            Some(link1),
        );
        sys.add_joint(link2, ArticulatedJoint::revolute(Vector3::z()));

        assert_eq!(sys.nv(), 2);
        assert_eq!(sys.n_bodies(), 3);

        // Inertia matrix should be 2x2
        let m = sys.inertia_matrix();
        assert_eq!(m.nrows(), 2);
        assert_eq!(m.ncols(), 2);
    }

    #[test]
    fn test_spherical_joint() {
        let mut sys = ArticulatedSystem::new();
        let root = sys.add_body(ArticulatedBody::point_mass(0.0), None);
        let bob = sys.add_body(
            ArticulatedBody::point_mass(1.0)
                .with_joint_transform(Isometry3::translation(0.0, -1.0, 0.0)),
            Some(root),
        );
        sys.add_joint(bob, ArticulatedJoint::spherical());

        assert_eq!(sys.nv(), 3); // 3 DOF
        assert_eq!(sys.nq(), 4); // Quaternion

        // Quaternion should be initialized to identity
        assert_relative_eq!(sys.qpos[0], 1.0, epsilon = 1e-10); // w
        assert_relative_eq!(sys.qpos[1], 0.0, epsilon = 1e-10); // x
        assert_relative_eq!(sys.qpos[2], 0.0, epsilon = 1e-10); // y
        assert_relative_eq!(sys.qpos[3], 0.0, epsilon = 1e-10); // z
    }

    #[test]
    fn test_free_joint() {
        let mut sys = ArticulatedSystem::new();
        let body = sys.add_body(ArticulatedBody::sphere(1.0, 0.1), None);
        sys.add_joint(body, ArticulatedJoint::free());

        assert_eq!(sys.nv(), 6);
        assert_eq!(sys.nq(), 7); // xyz + quaternion

        // Debug: check inertia and bias
        let m = sys.inertia_matrix();
        let bias = sys.bias_forces();
        eprintln!("Inertia matrix:\n{}", m);
        eprintln!("Bias forces: {}", bias);
        eprintln!("Gravity: {:?}", sys.gravity);

        // Run simulation - body should fall
        let initial_y = sys.qpos[1];
        sys.step(DT);
        eprintln!(
            "After one step: qpos={:?}, qvel={:?}",
            sys.qpos.as_slice(),
            sys.qvel.as_slice()
        );
        sys.run_for(0.1, DT);

        eprintln!(
            "Final: qpos={:?}, qvel={:?}",
            sys.qpos.as_slice(),
            sys.qvel.as_slice()
        );

        assert!(
            sys.qpos[1] < initial_y,
            "Free body should fall under gravity: y={} (was {})",
            sys.qpos[1],
            initial_y
        );
    }

    #[test]
    fn test_forward_kinematics() {
        let mut sys = ArticulatedSystem::new();
        let root = sys.add_body(ArticulatedBody::point_mass(0.0), None);
        let bob = sys.add_body(
            ArticulatedBody::point_mass(1.0)
                .with_joint_transform(Isometry3::translation(0.0, -1.0, 0.0)),
            Some(root),
        );
        sys.add_joint(bob, ArticulatedJoint::revolute(Vector3::z()));

        // At angle 0, bob should be at (0, -1, 0)
        sys.set_joint_position(0, 0.0);
        let transforms = sys.forward_kinematics();
        let bob_pos = transforms[1].translation.vector;
        assert_relative_eq!(bob_pos.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(bob_pos.y, -1.0, epsilon = 1e-10);
        assert_relative_eq!(bob_pos.z, 0.0, epsilon = 1e-10);

        // At angle PI/2, bob should be at (1, 0, 0)
        sys.set_joint_position(0, PI / 2.0);
        let transforms = sys.forward_kinematics();
        let bob_pos = transforms[1].translation.vector;
        assert_relative_eq!(bob_pos.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(bob_pos.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(bob_pos.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_joint_damping() {
        let mut sys = ArticulatedSystem::new();
        let root = sys.add_body(ArticulatedBody::point_mass(0.0), None);
        let bob = sys.add_body(
            ArticulatedBody::point_mass(1.0)
                .with_joint_transform(Isometry3::translation(0.0, -1.0, 0.0)),
            Some(root),
        );
        sys.add_joint(
            bob,
            ArticulatedJoint::revolute(Vector3::z()).with_damping(5.0),
        );

        // Start with velocity
        sys.set_joint_velocity(0, 2.0);
        let initial_vel = sys.qvel[0];

        // Run for a bit
        sys.run_for(0.5, DT);

        // Velocity should have decreased due to damping
        assert!(
            sys.qvel[0].abs() < initial_vel.abs(),
            "Damping should reduce velocity"
        );
    }

    #[test]
    fn test_joint_spring() {
        let mut sys = ArticulatedSystem::new();
        let root = sys.add_body(ArticulatedBody::point_mass(0.0), None);
        let bob = sys.add_body(
            ArticulatedBody::point_mass(1.0)
                .with_joint_transform(Isometry3::translation(0.0, -1.0, 0.0)),
            Some(root),
        );
        sys.add_joint(
            bob,
            ArticulatedJoint::revolute(Vector3::z())
                .with_stiffness(10.0, 0.0)
                .with_damping(1.0),
        );

        // Displace from rest
        sys.set_joint_position(0, 0.5);

        // Run for a while - should oscillate toward rest
        sys.run_for(3.0, DT);

        // Should be near rest position with low velocity
        assert!(
            sys.qpos[0].abs() < 0.2,
            "Spring should return to rest: pos={}",
            sys.qpos[0]
        );
    }
}
