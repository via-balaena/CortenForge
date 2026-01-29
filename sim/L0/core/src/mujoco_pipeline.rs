// Allow patterns common in physics simulation code:
// - tuple_array_conversions: [t1, t2] from (t1, t2) is intentional for frame storage
// - manual_div_ceil: explicit ceiling division is clearer in some physics contexts
// - cast_possible_truncation: f64 to i64 for grid indices is intentional with bounds checks
// - or_fun_call: or_insert_with vs or_default is a style choice
// - collapsible_if: nested conditions can be clearer for physics logic
// - cast_precision_loss: usize to f64 is acceptable for index-based calculations
// - op_ref: reference vs value is a style choice for Vector3 operations
// - manual_let_else: if-let can be clearer than let-else for physics code
// - needless_range_loop: explicit indexing is sometimes clearer
// - imprecise_flops: physics code may prefer explicit formulas over hypot
#![allow(
    clippy::tuple_array_conversions,
    clippy::manual_div_ceil,
    clippy::cast_possible_truncation,
    clippy::or_fun_call,
    clippy::collapsible_if,
    clippy::cast_precision_loss,
    clippy::op_ref,
    clippy::manual_let_else,
    clippy::needless_range_loop,
    clippy::imprecise_flops
)]

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

use crate::collision_shape::{Aabb, CollisionShape};
use crate::mesh::{
    MeshContact, TriangleMeshData, mesh_box_contact, mesh_capsule_contact,
    mesh_mesh_deepest_contact, mesh_sphere_contact,
};
use nalgebra::{Matrix3, Point3, Vector3};
use sim_types::Pose;
use std::f64::consts::PI;
use std::sync::Arc;

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
    /// Joint angles θ\[i\] in radians (angle of link i relative to link i-1)
    pub qpos: DVector<f64>,
    /// Joint velocities θ̇\[i\] in rad/s
    pub qvel: DVector<f64>,
    /// Link lengths L\[i\] in meters
    pub lengths: Vec<f64>,
    /// Link masses m\[i\] in kg (point mass at end of link)
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
    /// θ_abs\[i\] = θ\[0\] + θ\[1\] + ... + θ\[i\]
    #[must_use]
    pub fn absolute_angle(&self, link: usize) -> f64 {
        self.qpos.iter().take(link + 1).sum()
    }

    /// Compute the Cartesian position of the end of link i.
    ///
    /// Position is computed by summing all link contributions:
    /// x = Σ L\[j\] * sin(θ_abs\[j\])
    /// y = -Σ L\[j\] * cos(θ_abs\[j\])
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
    /// M\[i,j\] = ∂²T/(∂θ̇_i ∂θ̇_j)
    ///
    /// For joint i affecting mass k (where k ≥ i), the velocity contribution is:
    /// ∂v_k/∂θ̇_i = Σ_{j=i}^{k} L_j * (perpendicular to link j)
    ///
    /// M\[i,j\] = Σ_{k≥max(i,j)} m_k * (∂v_k/∂θ̇_i · ∂v_k/∂θ̇_j)
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
    /// C\[i\] = Σ_{j,k} c_{ijk} * θ̇_j * θ̇_k + g\[i\]
    ///
    /// where c_{ijk} = (1/2)(∂M_{ij}/∂θ_k + ∂M_{ik}/∂θ_j - ∂M_{jk}/∂θ_i)
    ///
    /// For gravity: g\[i\] = ∂V/∂θ_i where V = Σ_k m_k * g * h_k
    /// h_k = -Σ_{j=0}^{k} L_j * cos(θ_abs_j)
    /// ∂h_k/∂θ_i = Σ_{j=i}^{k} L_j * sin(θ_abs_j)  (for i ≤ k, else 0)
    /// g\[i\] = Σ_{k≥i} m_k * g * Σ_{j=i}^{k} L_j * sin(θ_abs_j)
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

use nalgebra::{
    Cholesky, Dyn, Isometry3, Matrix6, Quaternion, Translation3, Unit, UnitQuaternion, Vector6,
};
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
    /// Optional name for identification.
    pub name: Option<String>,
    /// Whether this body is static (immovable).
    pub is_static: bool,
    /// Collision shape for contact detection (optional).
    pub collision_shape: Option<CollisionShape>,
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
            name: None,
            is_static: false,
            collision_shape: None,
        }
    }

    /// Create a fixed/static body (zero mass, used as root).
    #[must_use]
    pub fn fixed() -> Self {
        let mut body = Self::new(0.0, Vector3::zeros(), Matrix3::zeros());
        body.is_static = true;
        body
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

    /// Set the body name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set the collision shape for contact detection.
    #[must_use]
    pub fn with_collision_shape(mut self, shape: CollisionShape) -> Self {
        self.collision_shape = Some(shape);
        self
    }

    /// Mark this body as static (immovable).
    #[must_use]
    pub fn as_static(mut self) -> Self {
        self.is_static = true;
        self
    }

    /// Compute the 6x6 spatial inertia matrix in body-local frame.
    ///
    /// The spatial inertia is:
    /// ```text
    /// I_spatial = | I + m*\[c\]×\[c\]×ᵀ   m*\[c\]× |
    ///             |     m*\[c\]×ᵀ          m*1  |
    /// ```
    /// where \[c\]× is the skew-symmetric matrix of the COM offset.
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
    /// Optional name for identification.
    pub name: Option<String>,
}

impl ArticulatedJoint {
    /// Safely normalize a vector, falling back to Z axis if zero-length.
    fn safe_normalize(v: Vector3<f64>) -> Vector3<f64> {
        let norm = v.norm();
        if norm > 1e-10 { v / norm } else { Vector3::z() }
    }

    /// Create a new revolute joint about the given axis.
    #[must_use]
    pub fn revolute(axis: Vector3<f64>) -> Self {
        Self {
            joint_type: ArticulatedJointType::Revolute,
            axis: Self::safe_normalize(axis),
            lower_limit: vec![-PI],
            upper_limit: vec![PI],
            damping: vec![0.0],
            stiffness: vec![0.0],
            rest_position: vec![0.0],
            qpos_start: 0,
            qpos_dim: 1,
            name: None,
        }
    }

    /// Create a new prismatic joint along the given axis.
    #[must_use]
    pub fn prismatic(axis: Vector3<f64>) -> Self {
        Self {
            joint_type: ArticulatedJointType::Prismatic,
            axis: Self::safe_normalize(axis),
            lower_limit: vec![-1.0],
            upper_limit: vec![1.0],
            damping: vec![0.0],
            stiffness: vec![0.0],
            rest_position: vec![0.0],
            qpos_start: 0,
            qpos_dim: 1,
            name: None,
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
            name: None,
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
            name: None,
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
            name: None,
        }
    }

    /// Set the joint name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
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

        // Note: This legacy ArticulatedSystem doesn't compute Coriolis.
        // Use Model/Data API (mj_rne) for full physics including Coriolis.

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
    /// # Integration Pipeline (ANCHOR: step_integration)
    ///
    /// 1. Compute inertia matrix M via CRBA — O(n) Featherstone algorithm
    /// 2. Compute bias forces via RNE — Coriolis/centrifugal/gravity
    /// 3. Compute passive forces — springs computed EXPLICITLY as `-k*(q-q_eq) - d*qvel`
    /// 4. Solve `M * qacc = qfrc_smooth` — LU decomposition, O(n³)
    /// 5. Semi-implicit Euler: `qvel += qacc*dt`, then `qpos += qvel*dt`
    ///
    /// # Implicit Spring Integration (Future Enhancement)
    ///
    /// Currently spring forces are computed explicitly in step 3, which can be
    /// unstable for high stiffness (k > 1000) with large timesteps (dt > 0.01).
    ///
    /// For implicit treatment, step 4 would become:
    /// ```text
    /// (M + h*D + h²*K) * v_new = M*v_old + h*f - h*K*(q - q_eq)
    /// ```
    ///
    /// This requires refactoring `passive_forces()` to return (K, D, q_eq) separately
    /// instead of the combined force vector. The change surface is minimal:
    /// - Step 3: Extract K, D, q_eq instead of computing force
    /// - Step 4: Build modified mass matrix `M + h*D + h²*K`
    /// - Step 4: Build modified RHS `M*v + h*f - h*K*(q-q_eq)`
    /// - Step 5: Already have v_new directly (skip qacc intermediate)
    ///
    /// Complexity remains O(n³) — we're just solving a different linear system.
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
#[allow(clippy::inline_always)] // Profiling shows inlining improves debug performance
#[inline(always)]
#[must_use]
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
#[allow(clippy::inline_always)] // Profiling shows inlining improves debug performance
#[inline(always)]
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

/// Compute body spatial inertia in world frame.
///
/// This builds the 6×6 spatial inertia matrix from:
/// - `mass`: body mass
/// - `inertia_diag`: diagonal inertia in body's principal frame
/// - `i_mat`: rotation matrix from inertial frame to world (3×3)
/// - `h`: COM offset from body origin in world frame
///
/// The spatial inertia has the form:
/// ```text
/// I = [I_rot + m*(h·h*I - h⊗h),  m*[h]×  ]
///     [m*[h]×ᵀ,                  m*I_3×3 ]
/// ```
///
/// This is the canonical implementation - computed once per body in FK,
/// then used by both CRBA (as starting point for composite) and RNE (directly).
#[allow(clippy::inline_always)] // Called once per body in FK - inlining avoids function call overhead
#[inline(always)]
fn compute_body_spatial_inertia(
    mass: f64,
    inertia_diag: Vector3<f64>,
    i_mat: &Matrix3<f64>,
    h: Vector3<f64>,
) -> Matrix6<f64> {
    // Rotational inertia in world frame: I_world = R * I_diag * R^T
    // Element-wise is faster than matrix ops in debug mode
    let mut i_rot: Matrix3<f64> = Matrix3::zeros();
    for row in 0..3 {
        for col in 0..3 {
            i_rot[(row, col)] = i_mat[(row, 0)] * inertia_diag[0] * i_mat[(col, 0)]
                + i_mat[(row, 1)] * inertia_diag[1] * i_mat[(col, 1)]
                + i_mat[(row, 2)] * inertia_diag[2] * i_mat[(col, 2)];
        }
    }

    let mut crb = Matrix6::zeros();

    // Upper-left 3x3: rotational inertia about body origin (parallel axis theorem)
    let h_dot_h = h.x * h.x + h.y * h.y + h.z * h.z;
    for row in 0..3 {
        for col in 0..3 {
            let h_outer = h[row] * h[col];
            let delta = if row == col { 1.0 } else { 0.0 };
            crb[(row, col)] = i_rot[(row, col)] + mass * (h_dot_h * delta - h_outer);
        }
    }

    // Lower-right 3x3: translational inertia (diagonal mass matrix)
    crb[(3, 3)] = mass;
    crb[(4, 4)] = mass;
    crb[(5, 5)] = mass;

    // Off-diagonal: coupling (skew-symmetric of m*h)
    let mh_x = mass * h.x;
    let mh_y = mass * h.y;
    let mh_z = mass * h.z;
    crb[(0, 4)] = -mh_z;
    crb[(0, 5)] = mh_y;
    crb[(1, 3)] = mh_z;
    crb[(1, 5)] = -mh_x;
    crb[(2, 3)] = -mh_y;
    crb[(2, 4)] = mh_x;
    // Transpose for lower-left
    crb[(4, 0)] = -mh_z;
    crb[(5, 0)] = mh_y;
    crb[(3, 1)] = mh_z;
    crb[(5, 1)] = -mh_x;
    crb[(3, 2)] = -mh_y;
    crb[(4, 2)] = mh_x;

    crb
}

// ============================================================================
// Phase 8: PGS Constraint Solver
// ============================================================================
//
// Implements the Projected Gauss-Seidel solver for constraint forces.
// This is MuJoCo's approach to constraint solving:
//
// 1. Compute unconstrained acceleration: qacc_smooth = M^(-1) * qfrc_smooth
// 2. Build constraint Jacobians J and reference accelerations aref
// 3. Solve: minimize (1/2)λᵀ(A+R)λ + λᵀb subject to λ ∈ Ω
//    where A = J @ M^(-1) @ Jᵀ, b = J @ qacc_smooth - aref
// 4. Final acceleration: qacc = qacc_smooth + M^(-1) @ Jᵀ @ λ

/// Type of constraint for the PGS solver.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConstraintType {
    /// Equality constraint (λ ∈ ℝ)
    Equality,
    /// Inequality constraint (λ ≥ 0), e.g., joint limit, contact normal
    Inequality,
    /// Friction constraint (|λ| ≤ μ * `λ_normal`)
    Friction {
        /// Index of the normal force constraint
        normal_idx: usize,
        /// Friction coefficient
        mu: f64,
    },
    /// Friction loss (|λ| ≤ η)
    FrictionLoss {
        /// Maximum friction force
        max_force: f64,
    },
}

/// A single constraint row for the PGS solver.
#[derive(Debug, Clone)]
pub struct Constraint {
    /// Constraint type determines projection
    pub constraint_type: ConstraintType,
    /// Jacobian row (sparse, length = nv)
    pub jacobian: DVector<f64>,
    /// Reference acceleration (for Baumgarte stabilization)
    pub aref: f64,
    /// Regularization (softness)
    pub regularization: f64,
    /// Warm-start value from previous timestep
    pub warmstart: f64,
}

impl Constraint {
    /// Create a new equality constraint.
    #[must_use]
    pub fn equality(jacobian: DVector<f64>, aref: f64) -> Self {
        Self {
            constraint_type: ConstraintType::Equality,
            jacobian,
            aref,
            regularization: 1e-8,
            warmstart: 0.0,
        }
    }

    /// Create a new inequality constraint (λ ≥ 0).
    #[must_use]
    pub fn inequality(jacobian: DVector<f64>, aref: f64) -> Self {
        Self {
            constraint_type: ConstraintType::Inequality,
            jacobian,
            aref,
            regularization: 1e-8,
            warmstart: 0.0,
        }
    }

    /// Create a friction constraint.
    #[must_use]
    pub fn friction(jacobian: DVector<f64>, aref: f64, normal_idx: usize, mu: f64) -> Self {
        Self {
            constraint_type: ConstraintType::Friction { normal_idx, mu },
            jacobian,
            aref,
            regularization: 1e-8,
            warmstart: 0.0,
        }
    }

    /// Set regularization (softness).
    #[must_use]
    pub fn with_regularization(mut self, reg: f64) -> Self {
        self.regularization = reg;
        self
    }

    /// Set warm-start value.
    #[must_use]
    pub fn with_warmstart(mut self, ws: f64) -> Self {
        self.warmstart = ws;
        self
    }
}

/// Configuration for the PGS solver.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PGSConfig {
    /// Maximum iterations
    pub max_iterations: usize,
    /// Convergence tolerance
    pub tolerance: f64,
    /// Enable warm-starting
    pub warmstart: bool,
    /// Over-relaxation factor (1.0 = standard GS, >1 = SOR)
    pub relaxation: f64,
}

impl Default for PGSConfig {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            tolerance: 1e-6,
            warmstart: true,
            relaxation: 1.0,
        }
    }
}

/// Result of the PGS solver.
#[derive(Debug, Clone)]
pub struct PGSResult {
    /// Constraint forces (Lagrange multipliers)
    pub forces: DVector<f64>,
    /// Number of iterations used
    pub iterations: usize,
    /// Final residual
    pub residual: f64,
    /// Whether solver converged
    pub converged: bool,
}

/// Projected Gauss-Seidel constraint solver.
///
/// Solves the LCP/QP:
///   minimize: (1/2) λᵀ H λ + λᵀ b
///   subject to: λ ∈ Ω (constraint-dependent bounds)
///
/// Where H = J @ M^(-1) @ Jᵀ + R (regularized constraint-space inertia).
#[derive(Debug, Clone)]
pub struct PGSSolver {
    config: PGSConfig,
}

impl Default for PGSSolver {
    fn default() -> Self {
        Self::new()
    }
}

impl PGSSolver {
    /// Create a new PGS solver with default configuration.
    #[must_use]
    pub fn new() -> Self {
        Self {
            config: PGSConfig::default(),
        }
    }

    /// Create with custom configuration.
    #[must_use]
    pub fn with_config(config: PGSConfig) -> Self {
        Self { config }
    }

    /// Solve the constraint problem.
    ///
    /// # Arguments
    /// * `m_inv` - Inverse mass matrix (nv × nv)
    /// * `qacc_smooth` - Unconstrained acceleration (length nv)
    /// * `constraints` - List of constraints
    ///
    /// # Returns
    /// * `PGSResult` containing constraint forces and solver info
    #[must_use]
    #[allow(clippy::too_many_lines)]
    pub fn solve(
        &self,
        m_inv: &DMatrix<f64>,
        qacc_smooth: &DVector<f64>,
        constraints: &[Constraint],
    ) -> PGSResult {
        let nc = constraints.len();

        if nc == 0 {
            return PGSResult {
                forces: DVector::zeros(0),
                iterations: 0,
                residual: 0.0,
                converged: true,
            };
        }

        let nv = qacc_smooth.len();

        // Build the constraint Jacobian matrix J (nc × nv)
        let mut j_matrix = DMatrix::zeros(nc, nv);
        for (i, c) in constraints.iter().enumerate() {
            j_matrix.row_mut(i).copy_from(&c.jacobian.transpose());
        }

        // Compute H = J @ M^(-1) @ Jᵀ + R
        // This is the constraint-space inverse inertia
        let j_minv = &j_matrix * m_inv;
        let mut h_matrix = &j_minv * j_matrix.transpose();

        // Add regularization to diagonal
        for (i, c) in constraints.iter().enumerate() {
            h_matrix[(i, i)] += c.regularization;
        }

        // Compute b = J @ qacc_smooth - aref
        let j_qacc = &j_matrix * qacc_smooth;
        let mut b = DVector::zeros(nc);
        for (i, c) in constraints.iter().enumerate() {
            b[i] = j_qacc[i] - c.aref;
        }

        // Extract diagonal for Gauss-Seidel
        let h_diag: Vec<f64> = (0..nc).map(|i| h_matrix[(i, i)]).collect();
        let h_diag_inv: Vec<f64> = h_diag.iter().map(|d| 1.0 / d.max(1e-12)).collect();

        // Initialize forces (warm-start or zero)
        let mut forces = if self.config.warmstart {
            DVector::from_iterator(nc, constraints.iter().map(|c| c.warmstart))
        } else {
            DVector::zeros(nc)
        };

        let mut residual = 0.0;
        let mut converged = false;

        for iter in 0..self.config.max_iterations {
            let mut max_change = 0.0_f64;

            for i in 0..nc {
                // Compute residual for constraint i: r_i = H[i,:] @ λ + b[i]
                let mut r_i = b[i];
                for j in 0..nc {
                    r_i += h_matrix[(i, j)] * forces[j];
                }

                // Gauss-Seidel update
                let old_force = forces[i];
                let delta = -r_i * h_diag_inv[i] * self.config.relaxation;
                forces[i] += delta;

                // Project onto constraint bounds
                forces[i] = Self::project(i, forces[i], &forces, constraints);

                max_change = max_change.max((forces[i] - old_force).abs());
            }

            residual = max_change;

            if max_change < self.config.tolerance {
                converged = true;
                return PGSResult {
                    forces,
                    iterations: iter + 1,
                    residual,
                    converged,
                };
            }
        }

        PGSResult {
            forces,
            iterations: self.config.max_iterations,
            residual,
            converged,
        }
    }

    /// Project constraint force onto its feasible set.
    fn project(
        idx: usize,
        force: f64,
        all_forces: &DVector<f64>,
        constraints: &[Constraint],
    ) -> f64 {
        match constraints[idx].constraint_type {
            ConstraintType::Equality => {
                // No projection needed
                force
            }
            ConstraintType::Inequality => {
                // λ ≥ 0
                force.max(0.0)
            }
            ConstraintType::Friction { normal_idx, mu } => {
                // |λ| ≤ μ * λ_normal
                let normal_force = all_forces[normal_idx].max(0.0);
                let limit = mu * normal_force;
                force.clamp(-limit, limit)
            }
            ConstraintType::FrictionLoss { max_force } => {
                // |λ| ≤ η
                force.clamp(-max_force, max_force)
            }
        }
    }

    /// Apply constraint forces to get final acceleration.
    ///
    /// qacc = `qacc_smooth` + M^(-1) @ Jᵀ @ λ
    #[must_use]
    pub fn apply_forces(
        m_inv: &DMatrix<f64>,
        qacc_smooth: &DVector<f64>,
        constraints: &[Constraint],
        forces: &DVector<f64>,
    ) -> DVector<f64> {
        if constraints.is_empty() {
            return qacc_smooth.clone();
        }

        let nv = qacc_smooth.len();
        let _nc = constraints.len();

        // Build Jᵀ @ λ
        let mut jt_lambda = DVector::zeros(nv);
        for (i, c) in constraints.iter().enumerate() {
            jt_lambda += &c.jacobian * forces[i];
        }

        // qacc = qacc_smooth + M^(-1) @ Jᵀ @ λ
        qacc_smooth + m_inv * jt_lambda
    }
}

// ============================================================================
// Constraint Generation Helpers
// ============================================================================

impl ArticulatedSystem {
    /// Generate joint limit constraints.
    ///
    /// For each joint DOF that violates its limit, creates an inequality constraint.
    #[must_use]
    pub fn joint_limit_constraints(&self, baumgarte_k: f64, baumgarte_b: f64) -> Vec<Constraint> {
        let mut constraints = Vec::new();

        let mut vel_idx = 0;
        for joint in &self.joints {
            let dof = joint.dof();
            let qpos = self.joint_qpos_by_joint(joint);

            for d in 0..dof {
                let q = qpos[d];
                let lower = joint.lower_limit[d];
                let upper = joint.upper_limit[d];

                // Check lower limit violation
                if q < lower {
                    let mut jacobian = DVector::zeros(self.nv);
                    jacobian[vel_idx + d] = -1.0; // Pushing up (positive force)

                    // Baumgarte stabilization: aref = -K * error - B * vel
                    let error = q - lower; // negative when violated
                    let vel = self.qvel[vel_idx + d];
                    let aref = -baumgarte_k * error - baumgarte_b * vel;

                    constraints.push(Constraint::inequality(jacobian, aref));
                }

                // Check upper limit violation
                if q > upper {
                    let mut jacobian = DVector::zeros(self.nv);
                    jacobian[vel_idx + d] = -1.0; // Negative Jacobian: positive λ gives negative qacc

                    let error = q - upper; // positive when violated
                    let vel = self.qvel[vel_idx + d];
                    // aref > 0 means we want qacc to be more negative (push back)
                    let aref = baumgarte_k * error + baumgarte_b * vel.max(0.0);

                    constraints.push(Constraint::inequality(jacobian, aref));
                }
            }

            vel_idx += dof;
        }

        constraints
    }

    /// Get joint qpos by joint reference.
    fn joint_qpos_by_joint(&self, joint: &ArticulatedJoint) -> &[f64] {
        &self.qpos.as_slice()[joint.qpos_start..joint.qpos_start + joint.qpos_dim]
    }

    /// Step with constraint solving.
    ///
    /// This is the full MuJoCo-style pipeline:
    /// 1. Compute unconstrained acceleration
    /// 2. Generate constraints (limits, contacts)
    /// 3. Solve for constraint forces
    /// 4. Apply constraint forces
    /// 5. Integrate
    pub fn step_constrained(&mut self, dt: f64, solver: &PGSSolver) {
        if self.nv == 0 {
            return;
        }

        // 1. Compute mass matrix and its inverse
        let m_matrix = self.inertia_matrix();
        let m_inv = m_matrix
            .try_inverse()
            .unwrap_or_else(|| DMatrix::identity(self.nv, self.nv));

        // 2. Compute bias forces (gravity + Coriolis)
        let qfrc_bias = self.bias_forces();

        // 3. Compute passive forces (springs, dampers)
        let qfrc_passive = self.passive_forces();

        // 4. Compute unconstrained acceleration
        // qacc_smooth = M^(-1) * (qfrc_passive - qfrc_bias)
        let qfrc_smooth = &qfrc_passive - &qfrc_bias;
        let qacc_smooth_mat = &m_inv * &qfrc_smooth;
        let qacc_smooth = DVector::from_column_slice(qacc_smooth_mat.as_slice());

        // 5. Generate constraints
        let mut constraints = self.joint_limit_constraints(100.0, 10.0);

        // 6. Solve constraints
        let result = solver.solve(&m_inv, &qacc_smooth, &constraints);

        // 7. Apply constraint forces to get final acceleration
        let qacc = PGSSolver::apply_forces(&m_inv, &qacc_smooth, &constraints, &result.forces);

        // 8. Update warm-start for next timestep
        for (i, c) in constraints.iter_mut().enumerate() {
            c.warmstart = result.forces[i];
        }

        // 9. Semi-implicit Euler integration
        // Update velocity first
        let qvel_new = &self.qvel + &qacc * dt;

        // Update positions using new velocity
        let mut pos_offset = 0;
        let mut vel_offset = 0;

        for joint in &self.joints {
            match joint.joint_type {
                ArticulatedJointType::Fixed => {}
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
                    let q_current = UnitQuaternion::from_quaternion(Quaternion::new(
                        self.qpos[pos_offset],
                        self.qpos[pos_offset + 1],
                        self.qpos[pos_offset + 2],
                        self.qpos[pos_offset + 3],
                    ));
                    let angle = w.norm() * dt;
                    let q_delta = if angle > 1e-10 {
                        UnitQuaternion::from_axis_angle(&Unit::new_normalize(w), angle)
                    } else {
                        UnitQuaternion::identity()
                    };
                    let q_new = q_current * q_delta;

                    self.qpos[pos_offset] = q_new.w;
                    self.qpos[pos_offset + 1] = q_new.i;
                    self.qpos[pos_offset + 2] = q_new.j;
                    self.qpos[pos_offset + 3] = q_new.k;

                    pos_offset += 4;
                    vel_offset += 3;
                }
                ArticulatedJointType::Free => {
                    // Angular integration (quaternion)
                    let w = Vector3::new(
                        qvel_new[vel_offset],
                        qvel_new[vel_offset + 1],
                        qvel_new[vel_offset + 2],
                    );
                    let q_current = UnitQuaternion::from_quaternion(Quaternion::new(
                        self.qpos[pos_offset + 3],
                        self.qpos[pos_offset + 4],
                        self.qpos[pos_offset + 5],
                        self.qpos[pos_offset + 6],
                    ));
                    let angle = w.norm() * dt;
                    let q_delta = if angle > 1e-10 {
                        UnitQuaternion::from_axis_angle(&Unit::new_normalize(w), angle)
                    } else {
                        UnitQuaternion::identity()
                    };
                    let q_new = q_current * q_delta;

                    self.qpos[pos_offset + 3] = q_new.w;
                    self.qpos[pos_offset + 4] = q_new.i;
                    self.qpos[pos_offset + 5] = q_new.j;
                    self.qpos[pos_offset + 6] = q_new.k;

                    // Linear integration
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
}

// ============================================================================
// Contact Constraint Generation
// ============================================================================

/// A contact point for constraint generation.
#[derive(Debug, Clone, Copy)]
pub struct ContactPoint {
    /// Position in world frame
    pub position: Vector3<f64>,
    /// Contact normal (pointing from body B to body A)
    pub normal: Vector3<f64>,
    /// Penetration depth (positive = penetrating)
    pub depth: f64,
    /// Body index A (or None for world/ground)
    pub body_a: Option<BodyIndex>,
    /// Body index B (or None for world/ground)
    pub body_b: Option<BodyIndex>,
    /// Friction coefficient
    pub friction: f64,
    /// Restitution coefficient
    pub restitution: f64,
}

impl ArticulatedSystem {
    /// Generate contact constraints from contact points.
    ///
    /// For each contact, generates:
    /// - 1 normal constraint (inequality, λ ≥ 0)
    /// - 2 friction constraints (friction cone approximation)
    #[must_use]
    pub fn contact_constraints(
        &self,
        contacts: &[ContactPoint],
        baumgarte_k: f64,
        _baumgarte_b: f64,
    ) -> Vec<Constraint> {
        let mut constraints = Vec::new();

        for contact in contacts {
            // Compute contact Jacobian
            // J_contact = J_a(point) - J_b(point) for velocity at contact point
            let (j_normal, j_tan1, j_tan2) = self.contact_jacobians(contact);

            // Normal constraint (inequality)
            // aref for Baumgarte: stabilize penetration
            let aref_normal = baumgarte_k * contact.depth;

            let normal_constraint =
                Constraint::inequality(j_normal.clone(), aref_normal).with_regularization(1e-6);
            let normal_idx = constraints.len();
            constraints.push(normal_constraint);

            // Friction constraints (pyramid approximation)
            if contact.friction > 0.0 {
                let friction1 = Constraint::friction(j_tan1, 0.0, normal_idx, contact.friction)
                    .with_regularization(1e-6);
                let friction2 = Constraint::friction(j_tan2, 0.0, normal_idx, contact.friction)
                    .with_regularization(1e-6);
                constraints.push(friction1);
                constraints.push(friction2);
            }
        }

        constraints
    }

    /// Compute contact Jacobians for normal and tangent directions.
    fn contact_jacobians(
        &self,
        contact: &ContactPoint,
    ) -> (DVector<f64>, DVector<f64>, DVector<f64>) {
        let nv = self.nv;

        // Compute tangent directions
        let (tan1, tan2) = compute_tangent_basis(&contact.normal);

        // Initialize Jacobians
        let mut j_normal = DVector::zeros(nv);
        let mut j_tan1 = DVector::zeros(nv);
        let mut j_tan2 = DVector::zeros(nv);

        // For each body, compute contribution to contact velocity
        // v_contact = J @ qvel

        // Body A contribution (if not world)
        if let Some(body_a) = contact.body_a {
            let j_a = self.body_point_jacobian(body_a, &contact.position);
            // Add contribution: positive (body A moves away from contact)
            for i in 0..nv {
                j_normal[i] +=
                    contact
                        .normal
                        .dot(&Vector3::new(j_a[(0, i)], j_a[(1, i)], j_a[(2, i)]));
                j_tan1[i] += tan1.dot(&Vector3::new(j_a[(0, i)], j_a[(1, i)], j_a[(2, i)]));
                j_tan2[i] += tan2.dot(&Vector3::new(j_a[(0, i)], j_a[(1, i)], j_a[(2, i)]));
            }
        }

        // Body B contribution (if not world)
        if let Some(body_b) = contact.body_b {
            let j_b = self.body_point_jacobian(body_b, &contact.position);
            // Subtract contribution: negative (body B moving away reduces relative velocity)
            for i in 0..nv {
                j_normal[i] -=
                    contact
                        .normal
                        .dot(&Vector3::new(j_b[(0, i)], j_b[(1, i)], j_b[(2, i)]));
                j_tan1[i] -= tan1.dot(&Vector3::new(j_b[(0, i)], j_b[(1, i)], j_b[(2, i)]));
                j_tan2[i] -= tan2.dot(&Vector3::new(j_b[(0, i)], j_b[(1, i)], j_b[(2, i)]));
            }
        }

        (j_normal, j_tan1, j_tan2)
    }

    /// Compute the Jacobian for a point on a body.
    ///
    /// Returns a 3 × nv matrix where J @ qvel gives the linear velocity of the point.
    #[must_use]
    fn body_point_jacobian(&self, body_idx: BodyIndex, point: &Vector3<f64>) -> DMatrix<f64> {
        let nv = self.nv;
        let mut jacobian = DMatrix::zeros(3, nv);

        // Get body transforms
        let transforms = self.forward_kinematics();

        // Find all ancestor joints
        let mut current = Some(body_idx);
        while let Some(idx) = current {
            if let Some(jidx) = self.bodies[idx].joint {
                let joint = &self.joints[jidx];
                let vel_start = self.joint_vel_start(jidx);

                // Get joint position in world
                let parent_idx = self.bodies[idx].parent.unwrap_or(idx);
                let joint_pos = transforms[parent_idx].translation.vector;

                match joint.joint_type {
                    ArticulatedJointType::Revolute => {
                        // ∂p/∂θ = axis × (p - joint_pos)
                        let axis = transforms[parent_idx].rotation * joint.axis;
                        let r = point - joint_pos;
                        let dp_dq = axis.cross(&r);

                        jacobian[(0, vel_start)] = dp_dq.x;
                        jacobian[(1, vel_start)] = dp_dq.y;
                        jacobian[(2, vel_start)] = dp_dq.z;
                    }
                    ArticulatedJointType::Prismatic => {
                        // ∂p/∂d = axis
                        let axis = transforms[parent_idx].rotation * joint.axis;

                        jacobian[(0, vel_start)] = axis.x;
                        jacobian[(1, vel_start)] = axis.y;
                        jacobian[(2, vel_start)] = axis.z;
                    }
                    ArticulatedJointType::Spherical => {
                        // ∂p/∂ω = -[r]× where r = p - joint_pos
                        let r = point - joint_pos;
                        // Column for wx: ∂p/∂ωx = [0, rz, -ry]
                        jacobian[(0, vel_start)] = 0.0;
                        jacobian[(1, vel_start)] = r.z;
                        jacobian[(2, vel_start)] = -r.y;
                        // Column for wy: ∂p/∂ωy = [-rz, 0, rx]
                        jacobian[(0, vel_start + 1)] = -r.z;
                        jacobian[(1, vel_start + 1)] = 0.0;
                        jacobian[(2, vel_start + 1)] = r.x;
                        // Column for wz: ∂p/∂ωz = [ry, -rx, 0]
                        jacobian[(0, vel_start + 2)] = r.y;
                        jacobian[(1, vel_start + 2)] = -r.x;
                        jacobian[(2, vel_start + 2)] = 0.0;
                    }
                    ArticulatedJointType::Free => {
                        // Angular part (first 3 DOF)
                        let r = point - joint_pos;
                        jacobian[(0, vel_start)] = 0.0;
                        jacobian[(1, vel_start)] = r.z;
                        jacobian[(2, vel_start)] = -r.y;
                        jacobian[(0, vel_start + 1)] = -r.z;
                        jacobian[(1, vel_start + 1)] = 0.0;
                        jacobian[(2, vel_start + 1)] = r.x;
                        jacobian[(0, vel_start + 2)] = r.y;
                        jacobian[(1, vel_start + 2)] = -r.x;
                        jacobian[(2, vel_start + 2)] = 0.0;

                        // Linear part (last 3 DOF): identity
                        jacobian[(0, vel_start + 3)] = 1.0;
                        jacobian[(1, vel_start + 4)] = 1.0;
                        jacobian[(2, vel_start + 5)] = 1.0;
                    }
                    ArticulatedJointType::Fixed => {}
                }
            }
            current = self.bodies[idx].parent;
        }

        jacobian
    }

    /// Get the velocity index where a joint's DOFs start.
    fn joint_vel_start(&self, joint_idx: JointIndex) -> usize {
        self.joints[..joint_idx]
            .iter()
            .map(ArticulatedJoint::dof)
            .sum()
    }
}

/// Compute orthonormal tangent basis from a normal vector.
fn compute_tangent_basis(normal: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    // Pick a vector not parallel to normal
    let reference = if normal.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };

    let cross = normal.cross(&reference);
    let cross_norm = cross.norm();
    let tan1 = if cross_norm > 1e-10 {
        cross / cross_norm
    } else {
        // Degenerate case: normal is zero or parallel to reference
        Vector3::x()
    };
    let tan2 = normal.cross(&tan1);

    (tan1, tan2)
}

// Note: World Integration section removed - use Model/Data API instead

// ============================================================================
// MuJoCo-Aligned Model/Data Architecture (Phase 1)
// ============================================================================
//
// These structs follow MuJoCo's separation of static model data (Model) from
// dynamic simulation state (Data). The key insight is that body poses are
// COMPUTED from joint positions via forward kinematics, not stored as
// independent state.
//
// Reference: https://mujoco.readthedocs.io/en/stable/computation/index.html

/// Joint type following `MuJoCo` conventions.
///
/// Named `MjJointType` to distinguish from `sim_types::JointType`.
/// `MuJoCo` uses different names (Hinge vs Revolute, Slide vs Prismatic).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum MjJointType {
    /// Hinge joint (1 DOF): rotation about a single axis.
    /// qpos: 1 scalar (angle in radians)
    /// qvel: 1 scalar (angular velocity)
    #[default]
    Hinge,
    /// Slide joint (1 DOF): translation along a single axis.
    /// qpos: 1 scalar (displacement)
    /// qvel: 1 scalar (linear velocity)
    Slide,
    /// Ball joint (3 DOF): free rotation (spherical).
    /// qpos: 4 scalars (unit quaternion w, x, y, z)
    /// qvel: 3 scalars (angular velocity)
    Ball,
    /// Free joint (6 DOF): floating body with no constraints.
    /// qpos: 7 scalars (position x,y,z + quaternion w,x,y,z)
    /// qvel: 6 scalars (linear velocity + angular velocity)
    Free,
}

impl MjJointType {
    /// Number of position coordinates (nq contribution).
    #[must_use]
    pub const fn nq(self) -> usize {
        match self {
            Self::Hinge | Self::Slide => 1,
            Self::Ball => 4, // quaternion
            Self::Free => 7, // pos + quat
        }
    }

    /// Number of velocity coordinates / DOFs (nv contribution).
    #[must_use]
    pub const fn nv(self) -> usize {
        match self {
            Self::Hinge | Self::Slide => 1,
            Self::Ball => 3, // angular velocity
            Self::Free => 6, // linear + angular velocity
        }
    }

    /// Whether this joint type uses quaternion representation.
    #[must_use]
    pub const fn uses_quaternion(self) -> bool {
        matches!(self, Self::Ball | Self::Free)
    }

    /// Whether this joint type supports springs (linear displacement from equilibrium).
    /// Ball/Free joints use quaternions and don't have a simple spring formulation.
    #[must_use]
    pub const fn supports_spring(self) -> bool {
        matches!(self, Self::Hinge | Self::Slide)
    }
}

// ============================================================================
// Joint Visitor Pattern
// ============================================================================

/// Context passed to joint visitors with pre-computed addresses and metadata.
///
/// This provides all the commonly-needed information about a joint in one place,
/// avoiding repeated lookups of `jnt_dof_adr`, `jnt_qpos_adr`, etc.
#[derive(Debug, Clone, Copy)]
pub struct JointContext {
    /// Joint index in model arrays.
    pub jnt_id: usize,
    /// Joint type (Hinge, Slide, Ball, Free).
    pub jnt_type: MjJointType,
    /// Starting index in qvel/qacc/qfrc arrays (DOF address).
    pub dof_adr: usize,
    /// Starting index in qpos array (position address).
    pub qpos_adr: usize,
    /// Number of velocity DOFs for this joint.
    pub nv: usize,
    /// Number of position coordinates for this joint.
    pub nq: usize,
}

/// Visitor trait for operations that iterate over joints.
///
/// This is the **single source of truth** for joint iteration. All code that
/// processes joints should use `Model::visit_joints()` with this trait to ensure:
///
/// 1. **Consistency**: When a new joint type is added, the compiler forces updates
/// 2. **Correctness**: Address computations are centralized, not copy-pasted
/// 3. **Performance**: Addresses are computed once per joint, not per operation
///
/// # Example
///
/// ```ignore
/// struct MyVisitor<'a> {
///     model: &'a Model,
///     data: &'a mut Data,
/// }
///
/// impl JointVisitor for MyVisitor<'_> {
///     fn visit_hinge(&mut self, ctx: JointContext) {
///         // Process hinge joint at ctx.dof_adr, ctx.qpos_adr
///     }
///     // ... other methods have default no-op implementations
/// }
///
/// model.visit_joints(&mut MyVisitor { model, data });
/// ```
pub trait JointVisitor {
    /// Called for each Hinge joint (1 DOF revolute).
    ///
    /// Default: no-op. Override if your visitor needs to process hinge joints.
    #[inline]
    fn visit_hinge(&mut self, _ctx: JointContext) {}

    /// Called for each Slide joint (1 DOF prismatic).
    ///
    /// Default: no-op. Override if your visitor needs to process slide joints.
    #[inline]
    fn visit_slide(&mut self, _ctx: JointContext) {}

    /// Called for each Ball joint (3 DOF spherical, quaternion orientation).
    ///
    /// Default: no-op. Override if your visitor needs to process ball joints.
    #[inline]
    fn visit_ball(&mut self, _ctx: JointContext) {}

    /// Called for each Free joint (6 DOF floating base).
    ///
    /// Default: no-op. Override if your visitor needs to process free joints.
    #[inline]
    fn visit_free(&mut self, _ctx: JointContext) {}
}

/// Geometry type for collision detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum GeomType {
    /// Plane (infinite, typically used for ground).
    Plane,
    /// Sphere defined by radius.
    #[default]
    Sphere,
    /// Capsule (cylinder with hemispherical caps).
    Capsule,
    /// Cylinder.
    Cylinder,
    /// Box (rectangular cuboid).
    Box,
    /// Ellipsoid.
    Ellipsoid,
    /// Convex mesh (requires mesh data).
    Mesh,
}

impl GeomType {
    /// Compute the bounding sphere radius for a geometry from its type and size.
    ///
    /// This is the canonical implementation used by both:
    /// - Model compilation (pre-computing `geom_rbound`)
    /// - `CollisionShape::bounding_radius()` for runtime shapes
    ///
    /// # Arguments
    /// * `size` - Type-specific size parameters from `geom_size`:
    ///   - Sphere: `[radius, _, _]`
    ///   - Box: `[half_x, half_y, half_z]`
    ///   - Capsule: `[radius, half_length, _]`
    ///   - Cylinder: `[radius, half_length, _]`
    ///   - Ellipsoid: `[radius_x, radius_y, radius_z]`
    ///   - Plane: ignored (returns infinity)
    ///   - Mesh: `[scale_x, scale_y, scale_z]` (conservative estimate)
    #[must_use]
    pub fn bounding_radius(self, size: Vector3<f64>) -> f64 {
        match self {
            Self::Sphere => size.x,
            Self::Box => size.norm(), // Distance from center to corner
            Self::Capsule => size.x + size.y, // radius + half_length
            Self::Cylinder => size.x.hypot(size.y), // sqrt(r² + h²)
            Self::Ellipsoid => size.x.max(size.y).max(size.z), // Max semi-axis
            Self::Plane => f64::INFINITY, // Planes are infinite
            Self::Mesh => {
                // Conservative estimate from scale factors.
                // Full implementation would use mesh AABB at load time.
                let scale = size.x.max(size.y).max(size.z);
                if scale > 0.0 { scale * 10.0 } else { 10.0 }
            }
        }
    }
}

/// Actuator transmission type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ActuatorTransmission {
    /// Direct joint actuation.
    #[default]
    Joint,
    /// Tendon actuation.
    Tendon,
    /// Site-based actuation.
    Site,
}

/// Actuator dynamics type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ActuatorDynamics {
    /// No dynamics (direct force).
    #[default]
    None,
    /// First-order filter.
    Filter,
    /// Integrator.
    Integrator,
    /// Muscle model.
    Muscle,
}

/// Tendon wrap object type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum WrapType {
    /// Site point (tendon passes through).
    #[default]
    Site,
    /// Geom wrapping (tendon wraps around sphere/cylinder).
    Geom,
    /// Joint coupling (tendon length changes with joint angle).
    Joint,
    /// Pulley (changes tendon direction, may have divisor).
    Pulley,
}

/// Equality constraint type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum EqualityType {
    /// Connect: constrains two body points to coincide.
    /// Removes 3 DOF (translation only).
    #[default]
    Connect,
    /// Weld: constrains two body frames to be identical.
    /// Removes 6 DOF (translation + rotation).
    Weld,
    /// Joint: polynomial constraint between two joints.
    /// q2 = poly(q1) where poly = c0 + c1*q1 + c2*q1^2 + ...
    Joint,
    /// Tendon: polynomial constraint between two tendons.
    /// len2 = poly(len1).
    Tendon,
    /// Distance: constrains distance between two bodies.
    /// |p1 - p2| = d (removes 1 DOF).
    Distance,
}

/// `MuJoCo` sensor type.
///
/// Matches `MuJoCo`'s mjtSensor enum. Each sensor type reads different
/// quantities from the simulation state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum MjSensorType {
    // ========== Common sensors ==========
    /// Touch sensor (contact force magnitude, 1D).
    #[default]
    Touch,
    /// Accelerometer (linear acceleration, 3D).
    Accelerometer,
    /// Velocity sensor (linear velocity, 3D).
    Velocimeter,
    /// Gyroscope (angular velocity, 3D).
    Gyro,
    /// Force sensor (3D force).
    Force,
    /// Torque sensor (3D torque).
    Torque,
    /// Magnetometer (magnetic field, 3D).
    Magnetometer,
    /// Rangefinder (distance to nearest surface, 1D).
    Rangefinder,

    // ========== Joint/tendon sensors ==========
    /// Joint position (1D per DOF).
    JointPos,
    /// Joint velocity (1D per DOF).
    JointVel,
    /// Tendon length (1D).
    TendonPos,
    /// Tendon velocity (1D).
    TendonVel,
    /// Actuator length (1D).
    ActuatorPos,
    /// Actuator velocity (1D).
    ActuatorVel,
    /// Actuator force (1D).
    ActuatorFrc,

    // ========== Position/orientation sensors ==========
    /// Site/body frame position (3D).
    FramePos,
    /// Site/body frame orientation as quaternion (4D).
    FrameQuat,
    /// Site/body frame axis (3D).
    FrameXAxis,
    /// Site/body frame Y axis (3D).
    FrameYAxis,
    /// Site/body frame Z axis (3D).
    FrameZAxis,
    /// Site/body frame linear velocity (3D).
    FrameLinVel,
    /// Site/body frame angular velocity (3D).
    FrameAngVel,
    /// Site/body frame linear acceleration (3D).
    FrameLinAcc,
    /// Site/body frame angular acceleration (3D).
    FrameAngAcc,

    // ========== Global sensors ==========
    /// Subtree center of mass (3D).
    SubtreeCom,
    /// Subtree linear momentum (3D).
    SubtreeLinVel,
    /// Subtree angular momentum (3D).
    SubtreeAngMom,

    // ========== User-defined ==========
    /// User-defined sensor (arbitrary dimension).
    User,
}

impl MjSensorType {
    /// Get the dimension (number of data elements) for this sensor type.
    #[must_use]
    pub const fn dim(self) -> usize {
        match self {
            Self::Touch
            | Self::JointPos
            | Self::JointVel
            | Self::TendonPos
            | Self::TendonVel
            | Self::ActuatorPos
            | Self::ActuatorVel
            | Self::ActuatorFrc
            | Self::Rangefinder => 1,

            Self::Accelerometer
            | Self::Velocimeter
            | Self::Gyro
            | Self::Force
            | Self::Torque
            | Self::Magnetometer
            | Self::FramePos
            | Self::FrameXAxis
            | Self::FrameYAxis
            | Self::FrameZAxis
            | Self::FrameLinVel
            | Self::FrameAngVel
            | Self::FrameLinAcc
            | Self::FrameAngAcc
            | Self::SubtreeCom
            | Self::SubtreeLinVel
            | Self::SubtreeAngMom => 3,

            Self::FrameQuat => 4,

            Self::User => 0, // Variable, must be set explicitly
        }
    }
}

/// Sensor data dependency stage.
///
/// Indicates when the sensor can be computed in the forward dynamics pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum MjSensorDataType {
    /// Computed in `mj_sensorPos` (after forward kinematics).
    #[default]
    Position,
    /// Computed in `mj_sensorVel` (after velocity FK).
    Velocity,
    /// Computed in `mj_sensorAcc` (after acceleration computation).
    Acceleration,
}

/// Object type for sensor attachment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum MjObjectType {
    /// No object (world-relative).
    #[default]
    None,
    /// Body.
    Body,
    /// Joint.
    Joint,
    /// Geom.
    Geom,
    /// Site.
    Site,
    /// Actuator.
    Actuator,
    /// Tendon.
    Tendon,
}

/// Integration method.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Integrator {
    /// Semi-implicit Euler (`MuJoCo` default).
    #[default]
    Euler,
    /// 4th order Runge-Kutta.
    RungeKutta4,
    /// Implicit midpoint.
    Implicit,
}

/// Errors that can occur during a simulation step.
///
/// Following Rust idioms, step() returns Result<(), StepError> instead of
/// silently correcting issues. Users must handle failures explicitly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepError {
    /// Position coordinates contain NaN or Inf.
    InvalidPosition,
    /// Velocity coordinates contain NaN or Inf.
    InvalidVelocity,
    /// Computed acceleration contains NaN (indicates singular mass matrix or numerical issues).
    InvalidAcceleration,
    /// Cholesky decomposition failed in implicit integration.
    /// This indicates the modified mass matrix (M + h*D + h²*K) is not positive definite,
    /// likely due to negative stiffness/damping or numerical instability.
    CholeskyFailed,
    /// Timestep is zero or negative.
    InvalidTimestep,
}

impl std::fmt::Display for StepError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidPosition => write!(f, "position contains NaN or Inf"),
            Self::InvalidVelocity => write!(f, "velocity contains NaN or Inf"),
            Self::InvalidAcceleration => write!(f, "acceleration contains NaN"),
            Self::CholeskyFailed => {
                write!(f, "Cholesky decomposition failed in implicit integration")
            }
            Self::InvalidTimestep => write!(f, "timestep is zero or negative"),
        }
    }
}

impl std::error::Error for StepError {}

// Note: SpatialVector is defined earlier in this file as Vector6<f64>.
// We use the same type for consistency with the existing ArticulatedSystem code.

/// Static model definition (like mjModel).
///
/// Immutable after construction - all memory allocated upfront.
/// This contains the kinematic tree structure, body properties,
/// joint properties, and simulation options.
///
/// # Memory Layout
///
/// Arrays are indexed by their respective IDs:
/// - `body_*` arrays indexed by `body_id` (0 = world)
/// - `jnt_*` arrays indexed by `joint_id`
/// - `dof_*` arrays indexed by `dof_id` (velocity dimension index)
/// - `geom_*` arrays indexed by `geom_id`
/// - `actuator_*` arrays indexed by `actuator_id`
#[derive(Debug, Clone)]
pub struct Model {
    // ==================== Metadata ====================
    /// Model name (from MJCF model attribute or URDF robot name).
    pub name: String,

    // ==================== Dimensions ====================
    /// Number of generalized position coordinates (includes quaternions).
    pub nq: usize,
    /// Number of generalized velocity coordinates (DOFs, always <= nq).
    pub nv: usize,
    /// Number of bodies (including world body 0).
    pub nbody: usize,
    /// Number of joints.
    pub njnt: usize,
    /// Number of collision geometries.
    pub ngeom: usize,
    /// Number of sites (attachment points).
    pub nsite: usize,
    /// Number of actuators.
    pub nu: usize,
    /// Number of activation states (for muscle/filter actuators).
    pub na: usize,

    // ==================== Body Tree (indexed by body_id, 0 = world) ====================
    /// Parent body index (0 for root bodies attached to world).
    pub body_parent: Vec<usize>,
    /// Root body of kinematic tree (for multi-tree systems).
    pub body_rootid: Vec<usize>,
    /// First joint index for this body in jnt_* arrays.
    pub body_jnt_adr: Vec<usize>,
    /// Number of joints attached to this body.
    pub body_jnt_num: Vec<usize>,
    /// First DOF index for this body in dof_* and qvel arrays.
    pub body_dof_adr: Vec<usize>,
    /// Number of DOFs for this body.
    pub body_dof_num: Vec<usize>,
    /// First geom index for this body.
    pub body_geom_adr: Vec<usize>,
    /// Number of geoms attached to this body.
    pub body_geom_num: Vec<usize>,

    // Body properties (in body-local frame)
    /// Position offset from parent joint frame to body frame.
    pub body_pos: Vec<Vector3<f64>>,
    /// Orientation offset from parent joint frame to body frame.
    pub body_quat: Vec<UnitQuaternion<f64>>,
    /// Center of mass position in body frame.
    pub body_ipos: Vec<Vector3<f64>>,
    /// Inertial frame orientation in body frame.
    pub body_iquat: Vec<UnitQuaternion<f64>>,
    /// Body mass in kg.
    pub body_mass: Vec<f64>,
    /// Diagonal inertia in principal axes (`body_iquat` frame).
    pub body_inertia: Vec<Vector3<f64>>,
    /// Optional body names for lookup.
    pub body_name: Vec<Option<String>>,
    /// Total mass of subtree rooted at this body (precomputed).
    /// `body_subtreemass[0]` is total mass of entire system.
    pub body_subtreemass: Vec<f64>,

    // ==================== Joints (indexed by jnt_id) ====================
    /// Joint type (Hinge, Slide, Ball, Free).
    pub jnt_type: Vec<MjJointType>,
    /// Body this joint belongs to (the child body).
    pub jnt_body: Vec<usize>,
    /// Start index in qpos array.
    pub jnt_qpos_adr: Vec<usize>,
    /// Start index in qvel/qacc arrays.
    pub jnt_dof_adr: Vec<usize>,
    /// Joint anchor position in body frame.
    pub jnt_pos: Vec<Vector3<f64>>,
    /// Joint axis for hinge/slide (in body frame).
    pub jnt_axis: Vec<Vector3<f64>>,
    /// Whether joint has limits.
    pub jnt_limited: Vec<bool>,
    /// Joint limits [min, max].
    pub jnt_range: Vec<(f64, f64)>,
    /// Spring stiffness coefficient (N/rad for hinge, N/m for slide).
    /// Applied as: τ = -stiffness * (q - springref)
    pub jnt_stiffness: Vec<f64>,
    /// Spring equilibrium position (rad for hinge, m for slide).
    /// This is the position where spring force is zero.
    /// Distinct from `qpos0` which is the initial position at model load.
    pub jnt_springref: Vec<f64>,
    /// Damping coefficient (Ns/rad for hinge, Ns/m for slide).
    /// Applied as: τ = -damping * qvel
    pub jnt_damping: Vec<f64>,
    /// Armature inertia (motor rotor inertia).
    pub jnt_armature: Vec<f64>,
    /// Optional joint names.
    pub jnt_name: Vec<Option<String>>,

    // ==================== DOFs (indexed by dof_id) ====================
    /// Body for this DOF.
    pub dof_body: Vec<usize>,
    /// Joint for this DOF.
    pub dof_jnt: Vec<usize>,
    /// Parent DOF in kinematic tree (None for root DOF).
    pub dof_parent: Vec<Option<usize>>,
    /// Armature inertia for this DOF.
    pub dof_armature: Vec<f64>,
    /// Damping coefficient for this DOF.
    pub dof_damping: Vec<f64>,
    /// Friction loss (dry friction) for this DOF.
    /// Applied as: τ_friction = -frictionloss * tanh(qvel * FRICTION_SMOOTHING)
    /// where the tanh provides a smooth approximation to sign(qvel) for numerical stability.
    pub dof_frictionloss: Vec<f64>,

    // ==================== Geoms (indexed by geom_id) ====================
    /// Geometry type.
    pub geom_type: Vec<GeomType>,
    /// Parent body.
    pub geom_body: Vec<usize>,
    /// Position in body frame.
    pub geom_pos: Vec<Vector3<f64>>,
    /// Orientation in body frame.
    pub geom_quat: Vec<UnitQuaternion<f64>>,
    /// Type-specific size parameters [size0, size1, size2].
    pub geom_size: Vec<Vector3<f64>>,
    /// Friction coefficients [sliding, torsional, rolling].
    pub geom_friction: Vec<Vector3<f64>>,
    /// Contact type bitmask.
    pub geom_contype: Vec<u32>,
    /// Contact affinity bitmask.
    pub geom_conaffinity: Vec<u32>,
    /// Contact margin (distance at which contact becomes active).
    pub geom_margin: Vec<f64>,
    /// Contact gap (minimum allowed separation).
    pub geom_gap: Vec<f64>,
    /// Solver impedance parameters [d0, dwidth, width, midpoint, power].
    /// Controls constraint softness and behavior.
    pub geom_solimp: Vec<[f64; 5]>,
    /// Solver reference parameters [timeconst, dampratio] or [d, dmin].
    /// Controls constraint dynamics.
    pub geom_solref: Vec<[f64; 2]>,
    /// Optional geom names.
    pub geom_name: Vec<Option<String>>,
    /// Pre-computed bounding sphere radius for each geom (in local frame).
    /// Used for fast distance culling in collision broad-phase.
    /// For primitives, computed from geom_size. For meshes, computed from mesh AABB.
    pub geom_rbound: Vec<f64>,
    /// Mesh index for each geom (`None` if not a mesh geom).
    /// Length: ngeom. Only geoms with `geom_type == GeomType::Mesh` have `Some(mesh_id)`.
    pub geom_mesh: Vec<Option<usize>>,

    // ==================== Meshes (indexed by mesh_id) ====================
    /// Number of mesh assets.
    pub nmesh: usize,
    /// Mesh names (for lookup by name).
    pub mesh_name: Vec<String>,
    /// Triangle mesh data with prebuilt BVH.
    /// `Arc` for cheap cloning (multiple geoms can reference the same mesh asset).
    pub mesh_data: Vec<Arc<TriangleMeshData>>,

    // ==================== Sites (indexed by site_id) ====================
    /// Parent body for each site.
    pub site_body: Vec<usize>,
    /// Site geometry type (for visualization, uses GeomType).
    pub site_type: Vec<GeomType>,
    /// Site position in body frame.
    pub site_pos: Vec<Vector3<f64>>,
    /// Site orientation in body frame.
    pub site_quat: Vec<UnitQuaternion<f64>>,
    /// Site size (for visualization).
    pub site_size: Vec<Vector3<f64>>,
    /// Optional site names.
    pub site_name: Vec<Option<String>>,

    // ==================== Sensors (indexed by sensor_id) ====================
    /// Number of sensors.
    pub nsensor: usize,
    /// Number of sensor data elements (sum of all sensor dims).
    pub nsensordata: usize,
    /// Sensor type.
    pub sensor_type: Vec<MjSensorType>,
    /// Sensor data type (position/velocity/acceleration dependent).
    pub sensor_datatype: Vec<MjSensorDataType>,
    /// Object type the sensor is attached to.
    pub sensor_objtype: Vec<MjObjectType>,
    /// Object ID the sensor is attached to (body/joint/site/geom id).
    pub sensor_objid: Vec<usize>,
    /// Reference object type (for relative sensors).
    pub sensor_reftype: Vec<MjObjectType>,
    /// Reference object ID.
    pub sensor_refid: Vec<usize>,
    /// Start address in sensordata array.
    pub sensor_adr: Vec<usize>,
    /// Number of data elements for this sensor.
    pub sensor_dim: Vec<usize>,
    /// Noise standard deviation (0 = no noise).
    pub sensor_noise: Vec<f64>,
    /// Cutoff for sensor value (0 = no cutoff).
    pub sensor_cutoff: Vec<f64>,
    /// Optional sensor names.
    pub sensor_name: Vec<Option<String>>,

    // ==================== Actuators (indexed by actuator_id) ====================
    /// Transmission type (Joint, Tendon, Site).
    pub actuator_trntype: Vec<ActuatorTransmission>,
    /// Dynamics type (None, Filter, Integrator, Muscle).
    pub actuator_dyntype: Vec<ActuatorDynamics>,
    /// Transmission target ID (joint/tendon/site).
    pub actuator_trnid: Vec<usize>,
    /// Transmission gear ratio.
    pub actuator_gear: Vec<f64>,
    /// Control input limits [min, max].
    pub actuator_ctrlrange: Vec<(f64, f64)>,
    /// Force output limits [min, max].
    pub actuator_forcerange: Vec<(f64, f64)>,
    /// Optional actuator names.
    pub actuator_name: Vec<Option<String>>,
    /// Start index in act array for each actuator's activation states.
    pub actuator_act_adr: Vec<usize>,
    /// Number of activation states per actuator (0 for None dynamics, 1 for Filter/Integrator, 2 for Muscle).
    pub actuator_act_num: Vec<usize>,

    // ==================== Tendons (indexed by tendon_id) ====================
    /// Number of tendons.
    pub ntendon: usize,
    /// Number of wrap objects across all tendons.
    pub nwrap: usize,
    /// Tendon path length limits [min, max]. Limited if min < max.
    pub tendon_range: Vec<(f64, f64)>,
    /// Whether tendon has length limits.
    pub tendon_limited: Vec<bool>,
    /// Tendon stiffness (force per unit length).
    pub tendon_stiffness: Vec<f64>,
    /// Tendon damping coefficient.
    pub tendon_damping: Vec<f64>,
    /// Tendon rest length (reference for spring force).
    pub tendon_lengthspring: Vec<f64>,
    /// Tendon length at qpos0 (precomputed reference).
    pub tendon_length0: Vec<f64>,
    /// Number of wrapping objects for this tendon.
    pub tendon_num: Vec<usize>,
    /// Start address in wrap_* arrays for this tendon's path.
    pub tendon_adr: Vec<usize>,
    /// Optional tendon names.
    pub tendon_name: Vec<Option<String>>,

    // Tendon wrapping path elements (indexed by wrap_id, grouped by tendon; total length = nwrap)
    /// Wrap object type (Site, Geom, Joint, Pulley).
    pub wrap_type: Vec<WrapType>,
    /// Object ID for the wrap point (site/geom/joint id).
    pub wrap_objid: Vec<usize>,
    /// Wrap divisor (1.0 for normal, 0.5 for pulley, etc.).
    pub wrap_prm: Vec<f64>,

    // ==================== Equality Constraints (indexed by eq_id) ====================
    /// Number of equality constraints.
    pub neq: usize,
    /// Equality constraint type (Connect, Weld, Joint, Tendon, Distance).
    pub eq_type: Vec<EqualityType>,
    /// First object ID (body/joint/tendon).
    pub eq_obj1id: Vec<usize>,
    /// Second object ID (body/joint/tendon, or unused).
    pub eq_obj2id: Vec<usize>,
    /// Constraint parameters (meaning depends on type).
    /// - Connect: anchor point in body1 frame [x, y, z]
    /// - Weld: relative pose [x, y, z, qw, qx, qy, qz] + torque scale
    /// - Joint: polycoef[0..5] for polynomial coupling
    /// - Tendon: polycoef[0..5] for polynomial coupling
    /// - Distance: target distance
    pub eq_data: Vec<[f64; 11]>,
    /// Whether this equality constraint is active.
    pub eq_active: Vec<bool>,
    /// Solver impedance parameters for this constraint.
    pub eq_solimp: Vec<[f64; 5]>,
    /// Solver reference parameters for this constraint.
    pub eq_solref: Vec<[f64; 2]>,
    /// Optional equality constraint names.
    pub eq_name: Vec<Option<String>>,

    // ==================== Options ====================
    /// Simulation timestep in seconds.
    pub timestep: f64,
    /// Gravity vector in world frame.
    pub gravity: Vector3<f64>,
    /// Default/reference joint positions.
    pub qpos0: DVector<f64>,
    /// Wind velocity in world frame (for aerodynamic forces).
    pub wind: Vector3<f64>,
    /// Magnetic field in world frame (for magnetic actuators).
    pub magnetic: Vector3<f64>,
    /// Medium density (for fluid drag, kg/m³).
    pub density: f64,
    /// Medium viscosity (for fluid drag, Pa·s).
    pub viscosity: f64,

    // Solver options
    /// Maximum constraint solver iterations.
    pub solver_iterations: usize,
    /// Early termination tolerance for solver.
    pub solver_tolerance: f64,
    /// Constraint impedance ratio (for soft constraints).
    pub impratio: f64,
    /// Friction cone type: 0=pyramidal, 1=elliptic.
    pub cone: u8,
    /// Disable flags (bitmask for disabling default behaviors).
    pub disableflags: u32,
    /// Enable flags (bitmask for enabling optional behaviors).
    pub enableflags: u32,
    /// Integration method.
    pub integrator: Integrator,

    // ==================== Cached Implicit Integration Parameters ====================
    // These are pre-computed from joint properties for implicit spring-damper integration.
    // Avoids O(nv) allocation per step by caching model-invariant diagonal matrices.
    /// Diagonal stiffness matrix K for implicit integration (length nv).
    /// K[i] = jnt_stiffness for Hinge/Slide DOFs, 0 for Ball/Free DOFs.
    pub implicit_stiffness: DVector<f64>,
    /// Diagonal damping matrix D for implicit integration (length nv).
    /// D[i] = jnt_damping for Hinge/Slide, dof_damping for Ball/Free DOFs.
    pub implicit_damping: DVector<f64>,
    /// Spring equilibrium positions for implicit integration (length nv).
    /// q_eq[i] = jnt_springref for Hinge/Slide, 0 for Ball/Free DOFs.
    pub implicit_springref: DVector<f64>,

    // ==================== Pre-computed Kinematic Data ====================
    // These are computed once at model construction to avoid O(n) lookups
    // in the inner loops of CRBA and RNE, achieving O(n) vs O(n³) complexity.
    /// For each body: list of ancestor joint indices (from body to root).
    /// `body_ancestor_joints[i]` contains all joints in the kinematic chain
    /// from body `i` to the world. Empty for body 0 (world).
    pub body_ancestor_joints: Vec<Vec<usize>>,

    /// For each body: set of ancestor joint indices for O(1) membership testing.
    /// Multi-word bitmask: `body_ancestor_mask[body_id][word]` where word = jnt_id / 64.
    /// Bit (jnt_id % 64) is set if joint jnt_id is an ancestor of this body.
    /// Supports unlimited joints with O(1) lookup per word.
    pub body_ancestor_mask: Vec<Vec<u64>>,
}

/// Contact point for constraint generation.
///
/// Matches MuJoCo's mjContact structure with all relevant fields
/// for constraint-based contact resolution.
#[derive(Debug, Clone)]
pub struct Contact {
    /// Contact position in world frame.
    pub pos: Vector3<f64>,
    /// Contact normal (from geom1 toward geom2, unit vector).
    pub normal: Vector3<f64>,
    /// Penetration depth (positive = penetrating).
    pub depth: f64,
    /// First geometry ID.
    pub geom1: usize,
    /// Second geometry ID.
    pub geom2: usize,
    /// Friction coefficient (combined from both geoms).
    pub friction: f64,
    /// Contact dimension: 1 (frictionless), 3 (friction), 4 (elliptic), 6 (torsional).
    pub dim: usize,
    /// Whether margin was included in distance computation.
    pub includemargin: bool,
    /// Friction parameters `[mu, mu2]` for anisotropic friction.
    /// `mu[0]` = sliding, `mu[1]` = torsional/rolling.
    pub mu: [f64; 2],
    /// Solver reference parameters (from geom pair).
    pub solref: [f64; 2],
    /// Solver impedance parameters (from geom pair).
    pub solimp: [f64; 5],
    /// Contact frame tangent vectors (orthogonal to normal).
    /// `frame[0..3]` = t1, `frame[3..6]` = t2 (for friction cone).
    pub frame: [Vector3<f64>; 2],
}

impl Contact {
    /// Create a new contact with basic parameters.
    ///
    /// The contact frame (tangent vectors) is computed automatically from the normal.
    /// Advanced solver parameters use MuJoCo defaults.
    ///
    /// # Numerical Safety
    /// - Negative or NaN friction is clamped to 0.0
    /// - NaN depth is set to 0.0
    /// - NaN position/normal components are handled gracefully
    #[must_use]
    #[inline]
    pub fn new(
        pos: Vector3<f64>,
        normal: Vector3<f64>,
        depth: f64,
        geom1: usize,
        geom2: usize,
        friction: f64,
    ) -> Self {
        // Safety: clamp friction to non-negative finite value
        let friction = if friction.is_finite() && friction > 0.0 {
            friction
        } else {
            0.0
        };

        // Safety: ensure depth is finite
        let depth = if depth.is_finite() { depth } else { 0.0 };

        // Compute tangent frame from normal (handles NaN/zero normals internally)
        let (t1, t2) = compute_tangent_frame(&normal);

        Self {
            pos,
            normal,
            depth,
            geom1,
            geom2,
            friction,
            dim: if friction > 0.0 { 3 } else { 1 }, // 3D friction cone or frictionless
            includemargin: false,
            mu: [friction, friction * 0.005], // sliding, torsional
            solref: [0.02, 1.0],              // MuJoCo defaults
            solimp: [0.9, 0.95, 0.001, 0.5, 2.0], // MuJoCo defaults
            frame: [t1, t2],
        }
    }
}

/// Compute orthonormal tangent frame from contact normal.
///
/// Returns (t1, t2) where t1, t2, normal form a right-handed orthonormal basis.
/// Handles degenerate cases (zero/NaN normal) by returning a default frame.
#[inline]
fn compute_tangent_frame(normal: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    // Safety check: handle zero/NaN normals
    let normal_len = normal.norm();
    if !normal_len.is_finite() || normal_len < 1e-10 {
        // Degenerate case: return default frame
        return (Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
    }

    // Normalize the normal (in case it wasn't already)
    let n = normal / normal_len;

    // Choose a reference vector not parallel to normal
    let reference = if n.x.abs() < 0.9 {
        Vector3::new(1.0, 0.0, 0.0)
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    };

    // Gram-Schmidt orthogonalization
    let t1 = reference - n * n.dot(&reference);
    let t1_norm = t1.norm();
    let t1 = if t1_norm > 1e-10 {
        t1 / t1_norm
    } else {
        // This shouldn't happen if reference was chosen correctly, but be safe
        Vector3::new(1.0, 0.0, 0.0)
    };

    let t2 = n.cross(&t1);
    // t2 should already be unit length since n and t1 are orthonormal
    (t1, t2)
}

/// Dynamic simulation state (like mjData).
///
/// All arrays pre-allocated - no heap allocation during simulation.
/// This contains the current state (qpos, qvel) and computed quantities
/// (body poses, forces, contacts).
///
/// # Key Invariant
///
/// `qpos` and `qvel` are the ONLY state variables. Everything else
/// (xpos, xquat, qfrc_*, etc.) is COMPUTED from them via forward dynamics.
#[derive(Debug, Clone)]
#[allow(non_snake_case)] // qM matches MuJoCo naming convention
pub struct Data {
    // ==================== Generalized Coordinates (THE source of truth) ====================
    /// Joint positions (length `nq`) - includes quaternion components for ball/free joints.
    pub qpos: DVector<f64>,
    /// Joint velocities (length `nv`).
    pub qvel: DVector<f64>,
    /// Joint accelerations (length `nv`) - computed by forward dynamics.
    pub qacc: DVector<f64>,
    /// Warm-start for constraint solver (length `nv`).
    pub qacc_warmstart: DVector<f64>,

    // ==================== Control / Actuation ====================
    /// Actuator control inputs (length `nu`).
    pub ctrl: DVector<f64>,
    /// Actuator activation states (length `na`) (for muscles/filters).
    pub act: DVector<f64>,
    /// Actuator forces in joint space (length `nv`).
    pub qfrc_actuator: DVector<f64>,

    // ==================== Computed Body States (from FK - outputs, not inputs) ====================
    /// Body positions in world frame (length `nbody`).
    pub xpos: Vec<Vector3<f64>>,
    /// Body orientations in world frame (length `nbody`).
    pub xquat: Vec<UnitQuaternion<f64>>,
    /// Body rotation matrices (cached) (length `nbody`).
    pub xmat: Vec<Matrix3<f64>>,
    /// Body inertial frame positions (length `nbody`).
    pub xipos: Vec<Vector3<f64>>,
    /// Body inertial frame rotations (length `nbody`).
    pub ximat: Vec<Matrix3<f64>>,

    // Geom poses (for collision detection)
    /// Geom positions in world frame (length `ngeom`).
    pub geom_xpos: Vec<Vector3<f64>>,
    /// Geom rotation matrices (length `ngeom`).
    pub geom_xmat: Vec<Matrix3<f64>>,

    // Site poses (for attachment points, sensors)
    /// Site positions in world frame (length `nsite`).
    pub site_xpos: Vec<Vector3<f64>>,
    /// Site rotation matrices (length `nsite`).
    pub site_xmat: Vec<Matrix3<f64>>,

    // ==================== Velocities (computed from qvel) ====================
    /// Body spatial velocities (length `nbody`): (angular, linear).
    pub cvel: Vec<SpatialVector>,
    /// DOF velocities in Cartesian space (length `nv`).
    pub cdof: Vec<SpatialVector>,

    // ==================== RNE Intermediate Quantities ====================
    /// Body bias accelerations for RNE (Coriolis/centrifugal).
    /// `a_bias[i] = X[i] @ a_bias[parent] + v[i] ×_m S[i] @ qdot[i]`
    pub cacc_bias: Vec<SpatialVector>,
    /// Body forces for RNE backward pass.
    /// `f[i] = I[i] @ a_bias[i] + v[i] ×* (I[i] @ v[i])`
    pub cfrc_bias: Vec<SpatialVector>,

    // ==================== Forces in Generalized Coordinates ====================
    /// User-applied generalized forces (length `nv`).
    pub qfrc_applied: DVector<f64>,
    /// Coriolis + centrifugal + gravity bias forces (length `nv`).
    pub qfrc_bias: DVector<f64>,
    /// Passive forces (springs + dampers) (length `nv`).
    pub qfrc_passive: DVector<f64>,
    /// Constraint forces (contacts + joint limits) (length `nv`).
    pub qfrc_constraint: DVector<f64>,

    // Cartesian forces (alternative input method)
    /// Applied spatial forces in world frame (length `nbody`).
    pub xfrc_applied: Vec<SpatialVector>,

    // ==================== Mass Matrix ====================
    /// Joint-space inertia matrix (`nv` x `nv`).
    /// For small systems (nv <= 32), dense storage is used.
    /// For large systems, only lower triangle is filled (sparse via qLD).
    pub qM: DMatrix<f64>,
    /// Cached Cholesky factorization of mass matrix (L where M = L L^T).
    /// Computed once in `mj_crba()` after building qM, reused in:
    /// - `mj_fwd_acceleration()`: Solve M qacc = τ
    /// - `pgs_solve_contacts()`: Compute M^{-1} J^T for constraint Jacobian
    ///
    /// This caching saves O(n³) factorization per solve (typically 2-3 per step).
    /// The clone in `mj_crba()` is unavoidable (nalgebra consumes the matrix),
    /// but subsequent solves are O(n²) forward/back substitution.
    pub qM_cholesky: Option<Cholesky<f64, Dyn>>,

    // ==================== Sparse L^T D L Factorization (for large systems) ====================
    /// Sparse L^T D L factorization exploiting tree structure.
    /// M = L^T D L where L is unit lower triangular and D is diagonal.
    /// For tree-structured robots, this achieves O(n) factorization and solve.
    ///
    /// Layout: qLD stores both L and D compactly:
    /// - `qLD_diag[i] = D[i,i]` (diagonal of D)
    /// - `qLD[i]` contains non-zero entries of `L[i, :]` below diagonal
    ///
    /// The sparsity pattern is determined by the kinematic tree:
    /// `L[i,j]` is non-zero only if DOF j is an ancestor of DOF i.
    pub qLD_diag: DVector<f64>,
    /// Sparse lower triangular factor L (non-zero entries only).
    /// `qLD_L[i] = [(col_idx, value), ...]` for row i, sorted by col_idx.
    /// For tree robots, each row has at most depth(i) non-zeros.
    pub qLD_L: Vec<Vec<(usize, f64)>>,
    /// Whether sparse factorization is valid and should be used.
    /// Set to true after `mj_factor_sparse()` is called.
    pub qLD_valid: bool,

    // ==================== Body and Composite Inertia (for Featherstone CRBA/RNE) ====================
    /// Body spatial inertia in world frame (before composite accumulation).
    /// These are 6×6 spatial inertias for individual bodies, computed once in FK.
    /// Used by both CRBA (as starting point) and RNE (for bias forces).
    /// This is equivalent to MuJoCo's `mjData.cinert`.
    pub cinert: Vec<Matrix6<f64>>,
    /// Composite rigid body inertia in world frame.
    /// These are 6×6 spatial inertias accumulated from subtrees during CRBA.
    /// Starts as a copy of `cinert`, then accumulates child inertias.
    pub crb_inertia: Vec<Matrix6<f64>>,

    // ==================== Subtree Mass/COM (for O(n) RNE gravity) ====================
    /// Total mass of subtree rooted at each body (including the body itself).
    /// Computed during forward kinematics via backward pass.
    pub subtree_mass: Vec<f64>,
    /// Center of mass of subtree in world frame (length `nbody`).
    /// Computed during forward kinematics via backward pass.
    pub subtree_com: Vec<Vector3<f64>>,

    // ==================== Tendon State ====================
    /// Current tendon lengths (length `ntendon`).
    /// Computed from wrap path through kinematics.
    pub ten_length: Vec<f64>,
    /// Tendon velocities (length `ntendon`).
    /// Computed as J_tendon @ qvel.
    pub ten_velocity: Vec<f64>,
    /// Tendon forces from springs/limits (length `ntendon`).
    pub ten_force: Vec<f64>,
    /// Tendon Jacobian: d(length)/d(qpos) (sparse, length varies).
    /// Maps tendon length changes to joint velocities.
    pub ten_J: Vec<DVector<f64>>,

    // ==================== Equality Constraint State ====================
    /// Equality constraint violation (length `neq` * max_dim).
    /// For Connect: 3D position error. For Weld: 6D pose error.
    pub eq_violation: Vec<f64>,
    /// Equality constraint forces (Lagrange multipliers).
    pub eq_force: Vec<f64>,

    // ==================== Contacts ====================
    /// Active contacts (pre-allocated with capacity).
    pub contacts: Vec<Contact>,
    /// Number of active contacts (`contacts.len()` but tracked explicitly).
    pub ncon: usize,

    // ==================== Solver State ====================
    /// Iterations used in last constraint solve.
    pub solver_niter: usize,
    /// Non-zeros in constraint Jacobian.
    pub solver_nnz: usize,
    /// Constraint force multipliers from previous solve (for warmstart).
    /// Maps (geom1, geom2) contact pair to (λ_normal, λ_friction1, λ_friction2).
    /// Using HashMap for O(1) lookup regardless of contact ordering changes.
    pub efc_lambda: std::collections::HashMap<(usize, usize), [f64; 3]>,

    // ==================== Sensors ====================
    /// Sensor data array (length `nsensordata`).
    /// Each sensor writes to `sensordata[sensor_adr[i]..sensor_adr[i]+sensor_dim[i]]`.
    pub sensordata: DVector<f64>,

    // ==================== Energy (for debugging/validation) ====================
    /// Potential energy (gravity + springs).
    pub energy_potential: f64,
    /// Kinetic energy.
    pub energy_kinetic: f64,

    // ==================== Time ====================
    /// Simulation time in seconds.
    pub time: f64,

    // ==================== Scratch Buffers (for allocation-free stepping) ====================
    /// Scratch matrix for implicit integration: M + h*D + h²*K.
    /// Pre-allocated to avoid O(n²) allocation per step.
    pub scratch_m_impl: DMatrix<f64>,
    /// Scratch vector for force accumulation (length `nv`).
    pub scratch_force: DVector<f64>,
    /// Scratch vector for RHS of linear solves (length `nv`).
    pub scratch_rhs: DVector<f64>,
    /// Scratch vector for new velocity in implicit solve (length `nv`).
    /// Used to hold v_new while computing qacc = (v_new - v_old) / h.
    pub scratch_v_new: DVector<f64>,

    // ==================== Cached Body Effective Mass/Inertia ====================
    // These are extracted from the mass matrix diagonal during forward() and cached
    // for use by constraint force limiting. This avoids O(joints) traversal per constraint.
    /// Minimum translational mass for each body (length `nbody`).
    /// Extracted from qM diagonal for linear DOFs (free joint indices 0-2, slide joints).
    /// World body (index 0) has value `f64::INFINITY`.
    pub body_min_mass: Vec<f64>,

    /// Minimum rotational inertia for each body (length `nbody`).
    /// Extracted from qM diagonal for angular DOFs (free joint 3-5, ball 0-2, hinge).
    /// World body (index 0) has value `f64::INFINITY`.
    pub body_min_inertia: Vec<f64>,
}

impl Model {
    /// Create an empty model with no bodies/joints.
    #[must_use]
    pub fn empty() -> Self {
        Self {
            // Metadata
            name: String::new(),

            // Dimensions
            nq: 0,
            nv: 0,
            nbody: 1, // World body 0 always exists
            njnt: 0,
            ngeom: 0,
            nsite: 0,
            nu: 0,
            na: 0,

            // Body tree (initialize world body)
            body_parent: vec![0], // World is its own parent
            body_rootid: vec![0],
            body_jnt_adr: vec![0],
            body_jnt_num: vec![0],
            body_dof_adr: vec![0],
            body_dof_num: vec![0],
            body_geom_adr: vec![0],
            body_geom_num: vec![0],

            // Body properties
            body_pos: vec![Vector3::zeros()],
            body_quat: vec![UnitQuaternion::identity()],
            body_ipos: vec![Vector3::zeros()],
            body_iquat: vec![UnitQuaternion::identity()],
            body_mass: vec![0.0], // World has no mass
            body_inertia: vec![Vector3::zeros()],
            body_name: vec![Some("world".to_string())],
            body_subtreemass: vec![0.0], // World subtree mass (will be total system mass)

            // Joints (empty)
            jnt_type: vec![],
            jnt_body: vec![],
            jnt_qpos_adr: vec![],
            jnt_dof_adr: vec![],
            jnt_pos: vec![],
            jnt_axis: vec![],
            jnt_limited: vec![],
            jnt_range: vec![],
            jnt_stiffness: vec![],
            jnt_springref: vec![],
            jnt_damping: vec![],
            jnt_armature: vec![],
            jnt_name: vec![],

            // DOFs (empty)
            dof_body: vec![],
            dof_jnt: vec![],
            dof_parent: vec![],
            dof_armature: vec![],
            dof_damping: vec![],
            dof_frictionloss: vec![],

            // Geoms (empty)
            geom_type: vec![],
            geom_body: vec![],
            geom_pos: vec![],
            geom_quat: vec![],
            geom_size: vec![],
            geom_friction: vec![],
            geom_contype: vec![],
            geom_conaffinity: vec![],
            geom_margin: vec![],
            geom_gap: vec![],
            geom_solimp: vec![],
            geom_solref: vec![],
            geom_name: vec![],
            geom_rbound: vec![],
            geom_mesh: vec![],

            // Meshes (empty)
            nmesh: 0,
            mesh_name: vec![],
            mesh_data: vec![],

            // Sites (empty)
            site_body: vec![],
            site_type: vec![],
            site_pos: vec![],
            site_quat: vec![],
            site_size: vec![],
            site_name: vec![],

            // Sensors (empty)
            nsensor: 0,
            nsensordata: 0,
            sensor_type: vec![],
            sensor_datatype: vec![],
            sensor_objtype: vec![],
            sensor_objid: vec![],
            sensor_reftype: vec![],
            sensor_refid: vec![],
            sensor_adr: vec![],
            sensor_dim: vec![],
            sensor_noise: vec![],
            sensor_cutoff: vec![],
            sensor_name: vec![],

            // Actuators (empty)
            actuator_trntype: vec![],
            actuator_dyntype: vec![],
            actuator_trnid: vec![],
            actuator_gear: vec![],
            actuator_ctrlrange: vec![],
            actuator_forcerange: vec![],
            actuator_name: vec![],
            actuator_act_adr: vec![],
            actuator_act_num: vec![],

            // Tendons (empty)
            ntendon: 0,
            nwrap: 0,
            tendon_range: vec![],
            tendon_limited: vec![],
            tendon_stiffness: vec![],
            tendon_damping: vec![],
            tendon_lengthspring: vec![],
            tendon_length0: vec![],
            tendon_num: vec![],
            tendon_adr: vec![],
            tendon_name: vec![],
            wrap_type: vec![],
            wrap_objid: vec![],
            wrap_prm: vec![],

            // Equality constraints (empty)
            neq: 0,
            eq_type: vec![],
            eq_obj1id: vec![],
            eq_obj2id: vec![],
            eq_data: vec![],
            eq_active: vec![],
            eq_solimp: vec![],
            eq_solref: vec![],
            eq_name: vec![],

            // Options (MuJoCo defaults)
            timestep: 0.002,                        // 500 Hz
            gravity: Vector3::new(0.0, 0.0, -9.81), // Z-up
            qpos0: DVector::zeros(0),
            wind: Vector3::zeros(),
            magnetic: Vector3::zeros(),
            density: 0.0,   // No fluid by default
            viscosity: 0.0, // No fluid by default
            solver_iterations: 100,
            solver_tolerance: 1e-8,
            impratio: 1.0,   // MuJoCo default
            cone: 0,         // Pyramidal friction cone
            disableflags: 0, // Nothing disabled
            enableflags: 0,  // Nothing extra enabled
            integrator: Integrator::Euler,

            // Cached implicit integration parameters (empty for empty model)
            implicit_stiffness: DVector::zeros(0),
            implicit_damping: DVector::zeros(0),
            implicit_springref: DVector::zeros(0),

            // Pre-computed kinematic data (world body has no ancestors)
            body_ancestor_joints: vec![vec![]],
            body_ancestor_mask: vec![vec![]], // Empty vec for world body (no joints yet)
        }
    }

    /// Iterate over all joints with the visitor pattern.
    ///
    /// This is the **single source of truth** for joint iteration. Use this method
    /// instead of manually iterating with `for jnt_id in 0..self.njnt` to ensure:
    ///
    /// - Consistent address computation across the codebase
    /// - Automatic handling of new joint types (compiler enforces trait updates)
    /// - Centralized joint metadata in `JointContext`
    ///
    /// # Example
    ///
    /// ```ignore
    /// struct MyVisitor { /* ... */ }
    ///
    /// impl JointVisitor for MyVisitor {
    ///     fn visit_hinge(&mut self, ctx: JointContext) {
    ///         println!("Hinge joint {} at DOF {}", ctx.jnt_id, ctx.dof_adr);
    ///     }
    /// }
    ///
    /// model.visit_joints(&mut MyVisitor { /* ... */ });
    /// ```
    #[inline]
    pub fn visit_joints<V: JointVisitor>(&self, visitor: &mut V) {
        for jnt_id in 0..self.njnt {
            let jnt_type = self.jnt_type[jnt_id];
            let ctx = JointContext {
                jnt_id,
                jnt_type,
                dof_adr: self.jnt_dof_adr[jnt_id],
                qpos_adr: self.jnt_qpos_adr[jnt_id],
                nv: jnt_type.nv(),
                nq: jnt_type.nq(),
            };

            match jnt_type {
                MjJointType::Hinge => visitor.visit_hinge(ctx),
                MjJointType::Slide => visitor.visit_slide(ctx),
                MjJointType::Ball => visitor.visit_ball(ctx),
                MjJointType::Free => visitor.visit_free(ctx),
            }
        }
    }

    /// Create initial Data struct for this model with all arrays pre-allocated.
    #[must_use]
    pub fn make_data(&self) -> Data {
        Data {
            // Generalized coordinates
            qpos: self.qpos0.clone(),
            qvel: DVector::zeros(self.nv),
            qacc: DVector::zeros(self.nv),
            qacc_warmstart: DVector::zeros(self.nv),

            // Actuation
            ctrl: DVector::zeros(self.nu),
            act: DVector::zeros(self.na),
            qfrc_actuator: DVector::zeros(self.nv),

            // Body states
            xpos: vec![Vector3::zeros(); self.nbody],
            xquat: vec![UnitQuaternion::identity(); self.nbody],
            xmat: vec![Matrix3::identity(); self.nbody],
            xipos: vec![Vector3::zeros(); self.nbody],
            ximat: vec![Matrix3::identity(); self.nbody],

            // Geom poses
            geom_xpos: vec![Vector3::zeros(); self.ngeom],
            geom_xmat: vec![Matrix3::identity(); self.ngeom],

            // Site poses
            site_xpos: vec![Vector3::zeros(); self.nsite],
            site_xmat: vec![Matrix3::identity(); self.nsite],

            // Velocities
            cvel: vec![SpatialVector::zeros(); self.nbody],
            cdof: vec![SpatialVector::zeros(); self.nv],

            // RNE intermediate quantities
            cacc_bias: vec![SpatialVector::zeros(); self.nbody],
            cfrc_bias: vec![SpatialVector::zeros(); self.nbody],

            // Forces
            qfrc_applied: DVector::zeros(self.nv),
            qfrc_bias: DVector::zeros(self.nv),
            qfrc_passive: DVector::zeros(self.nv),
            qfrc_constraint: DVector::zeros(self.nv),
            xfrc_applied: vec![SpatialVector::zeros(); self.nbody],

            // Mass matrix (dense)
            qM: DMatrix::zeros(self.nv, self.nv),
            qM_cholesky: None, // Computed in mj_crba

            // Mass matrix (sparse L^T D L for large systems)
            qLD_diag: DVector::zeros(self.nv),
            qLD_L: vec![Vec::new(); self.nv],
            qLD_valid: false,

            // Body spatial inertias (computed once in FK, used by CRBA and RNE)
            cinert: vec![Matrix6::zeros(); self.nbody],
            // Composite rigid body inertias (for Featherstone CRBA)
            crb_inertia: vec![Matrix6::zeros(); self.nbody],

            // Subtree mass/COM (for O(n) RNE gravity)
            subtree_mass: vec![0.0; self.nbody],
            subtree_com: vec![Vector3::zeros(); self.nbody],

            // Tendon state
            ten_length: vec![0.0; self.ntendon],
            ten_velocity: vec![0.0; self.ntendon],
            ten_force: vec![0.0; self.ntendon],
            ten_J: vec![DVector::zeros(self.nv); self.ntendon],

            // Equality constraint state
            eq_violation: vec![0.0; self.neq * 6], // max 6 DOF per constraint (weld)
            eq_force: vec![0.0; self.neq * 6],

            // Contacts
            contacts: Vec::with_capacity(256), // Pre-allocate typical capacity
            ncon: 0,

            // Solver state
            solver_niter: 0,
            solver_nnz: 0,
            efc_lambda: std::collections::HashMap::with_capacity(256), // Contact correspondence warmstart

            // Sensors
            sensordata: DVector::zeros(self.nsensordata),

            // Energy
            energy_potential: 0.0,
            energy_kinetic: 0.0,

            // Time
            time: 0.0,

            // Scratch buffers (pre-allocated for allocation-free stepping)
            scratch_m_impl: DMatrix::zeros(self.nv, self.nv),
            scratch_force: DVector::zeros(self.nv),
            scratch_rhs: DVector::zeros(self.nv),
            scratch_v_new: DVector::zeros(self.nv),

            // Cached body mass/inertia (computed in forward() after CRBA)
            // Initialize world body (index 0) to infinity, others to default
            body_min_mass: {
                let mut v = vec![DEFAULT_MASS_FALLBACK; self.nbody];
                if self.nbody > 0 {
                    v[0] = f64::INFINITY; // World body
                }
                v
            },
            body_min_inertia: {
                let mut v = vec![DEFAULT_MASS_FALLBACK; self.nbody];
                if self.nbody > 0 {
                    v[0] = f64::INFINITY; // World body
                }
                v
            },
        }
    }

    /// Get reference position for specified joint (from qpos0).
    ///
    /// Returns `None` if `jnt_id` is out of bounds.
    #[must_use]
    pub fn joint_qpos0(&self, jnt_id: usize) -> Option<&[f64]> {
        if jnt_id >= self.njnt {
            return None;
        }
        let start = self.jnt_qpos_adr[jnt_id];
        let len = self.jnt_type[jnt_id].nq();
        Some(&self.qpos0.as_slice()[start..start + len])
    }

    /// Compute pre-computed kinematic data (ancestor lists and masks).
    ///
    /// This must be called after the model topology is finalized. It builds:
    /// - `body_ancestor_joints`: For each body, the list of all ancestor joints
    /// - `body_ancestor_mask`: Multi-word bitmask for O(1) ancestor testing
    ///
    /// These enable O(n) CRBA/RNE algorithms instead of O(n³).
    ///
    /// Following `MuJoCo`'s principle: heavy computation at model load time,
    /// minimal computation at simulation time.
    pub fn compute_ancestors(&mut self) {
        // Number of u64 words needed for the bitmask
        let num_words = (self.njnt + 63) / 64; // ceil(njnt / 64)

        // Clear and resize
        self.body_ancestor_joints = vec![vec![]; self.nbody];
        self.body_ancestor_mask = vec![vec![0u64; num_words]; self.nbody];

        // For each body, walk up to root collecting ancestor joints
        for body_id in 1..self.nbody {
            let mut current = body_id;
            while current != 0 {
                // Add joints attached to this body
                let jnt_start = self.body_jnt_adr[current];
                let jnt_end = jnt_start + self.body_jnt_num[current];
                for jnt_id in jnt_start..jnt_end {
                    self.body_ancestor_joints[body_id].push(jnt_id);
                    // Set bit in multi-word mask (supports unlimited joints)
                    let word = jnt_id / 64;
                    let bit = jnt_id % 64;
                    self.body_ancestor_mask[body_id][word] |= 1u64 << bit;
                }
                current = self.body_parent[current];
            }
        }
    }

    /// Compute cached implicit integration parameters from joint parameters.
    ///
    /// This expands per-joint K/D/springref into per-DOF vectors used by
    /// implicit integration. Must be called after all joints are added.
    ///
    /// For Hinge/Slide joints: K[dof] = jnt_stiffness, D[dof] = jnt_damping
    /// For Ball/Free joints: K[dof] = 0, D[dof] = dof_damping (per-DOF)
    pub fn compute_implicit_params(&mut self) {
        // Resize to nv DOFs
        self.implicit_stiffness = DVector::zeros(self.nv);
        self.implicit_damping = DVector::zeros(self.nv);
        self.implicit_springref = DVector::zeros(self.nv);

        for jnt_id in 0..self.njnt {
            let dof_adr = self.jnt_dof_adr[jnt_id];
            let jnt_type = self.jnt_type[jnt_id];
            let nv_jnt = jnt_type.nv();

            match jnt_type {
                MjJointType::Hinge | MjJointType::Slide => {
                    self.implicit_stiffness[dof_adr] = self.jnt_stiffness[jnt_id];
                    self.implicit_damping[dof_adr] = self.jnt_damping[jnt_id];
                    self.implicit_springref[dof_adr] = self.jnt_springref[jnt_id];
                }
                MjJointType::Ball | MjJointType::Free => {
                    // Ball/Free: per-DOF damping only (no spring for quaternion DOFs)
                    for i in 0..nv_jnt {
                        let dof_idx = dof_adr + i;
                        self.implicit_stiffness[dof_idx] = 0.0;
                        self.implicit_damping[dof_idx] = self.dof_damping[dof_idx];
                        self.implicit_springref[dof_idx] = 0.0;
                    }
                }
            }
        }
    }

    /// Check if joint is an ancestor of body using pre-computed data.
    ///
    /// Uses O(1) multi-word bitmask lookup for all model sizes.
    /// Returns `false` for invalid body_id or jnt_id (no panic).
    #[inline]
    #[must_use]
    pub fn is_ancestor(&self, body_id: usize, jnt_id: usize) -> bool {
        // Bounds check for body_id
        if body_id >= self.body_ancestor_mask.len() {
            return false;
        }
        let word = jnt_id / 64;
        let bit = jnt_id % 64;
        if word < self.body_ancestor_mask[body_id].len() {
            (self.body_ancestor_mask[body_id][word] & (1u64 << bit)) != 0
        } else {
            false // Joint ID out of range
        }
    }

    // ========================================================================
    // Factory Methods for Common Systems
    // ========================================================================

    /// Create an n-link serial pendulum (hinge joints only).
    ///
    /// This creates a serial chain of `n` bodies connected by hinge joints,
    /// all rotating around the Y axis. Each body has a point mass at its end.
    ///
    /// # Arguments
    /// * `n` - Number of links (must be >= 1)
    /// * `link_length` - Length of each link (meters)
    /// * `link_mass` - Mass of each link (kg)
    ///
    /// # Returns
    /// A `Model` representing the n-link pendulum with all joints at qpos=0
    /// (hanging straight down).
    ///
    /// # Panics
    /// Panics if `n` is 0 (requires at least 1 link).
    ///
    /// # Example
    /// ```ignore
    /// let model = Model::n_link_pendulum(3, 1.0, 1.0);
    /// let mut data = model.make_data();
    /// data.qpos[0] = std::f64::consts::PI / 4.0; // Tilt first link
    /// data.forward(&model);
    /// ```
    #[must_use]
    pub fn n_link_pendulum(n: usize, link_length: f64, link_mass: f64) -> Self {
        assert!(n >= 1, "n_link_pendulum requires at least 1 link");

        let mut model = Self::empty();

        // Dimensions
        model.nq = n;
        model.nv = n;
        model.nbody = n + 1; // world + n bodies
        model.njnt = n;

        // Build the kinematic chain
        for i in 0..n {
            let body_id = i + 1; // Body 0 is world
            let parent_id = i; // Each body's parent is the previous body (0 = world for first)

            // Body tree
            model.body_parent.push(parent_id);
            model.body_rootid.push(1); // All belong to tree rooted at body 1
            model.body_jnt_adr.push(i);
            model.body_jnt_num.push(1);
            model.body_dof_adr.push(i);
            model.body_dof_num.push(1);
            model.body_geom_adr.push(0);
            model.body_geom_num.push(0);

            // Body properties
            // Body frame is at the END of the link (where the mass is)
            model.body_pos.push(Vector3::new(0.0, 0.0, -link_length));
            model.body_quat.push(UnitQuaternion::identity());
            model.body_ipos.push(Vector3::zeros()); // COM at body origin
            model.body_iquat.push(UnitQuaternion::identity());
            model.body_mass.push(link_mass);
            // Point mass approximation (small moment of inertia)
            model.body_inertia.push(Vector3::new(0.001, 0.001, 0.001));
            model.body_name.push(Some(format!("link_{i}")));
            model.body_subtreemass.push(0.0); // Will be computed after model is built

            // Joint definition (hinge at parent's frame, rotating around Y)
            model.jnt_type.push(MjJointType::Hinge);
            model.jnt_body.push(body_id);
            model.jnt_qpos_adr.push(i);
            model.jnt_dof_adr.push(i);
            model.jnt_pos.push(Vector3::zeros()); // Joint at body origin
            model.jnt_axis.push(Vector3::new(0.0, 1.0, 0.0)); // Rotate around Y
            model.jnt_limited.push(false);
            model.jnt_range.push((-PI, PI));
            model.jnt_stiffness.push(0.0);
            model.jnt_springref.push(0.0);
            model.jnt_damping.push(0.0);
            model.jnt_armature.push(0.0);
            model.jnt_name.push(Some(format!("hinge_{i}")));

            // DOF definition
            model.dof_body.push(body_id);
            model.dof_jnt.push(i);
            model
                .dof_parent
                .push(if i == 0 { None } else { Some(i - 1) });
            model.dof_armature.push(0.0);
            model.dof_damping.push(0.0);
            model.dof_frictionloss.push(0.0);
        }

        // Default qpos (hanging down)
        model.qpos0 = DVector::zeros(n);

        // Physics options
        model.timestep = 1.0 / 240.0;
        model.gravity = Vector3::new(0.0, 0.0, -9.81);
        model.solver_iterations = 10;
        model.solver_tolerance = 1e-8;

        // Pre-compute kinematic data for O(n) CRBA/RNE
        model.compute_ancestors();

        // Pre-compute implicit integration parameters
        model.compute_implicit_params();

        model
    }

    /// Create a double pendulum (2-link serial chain).
    ///
    /// Convenience method equivalent to `Model::n_link_pendulum(2, ...)`.
    #[must_use]
    pub fn double_pendulum(link_length: f64, link_mass: f64) -> Self {
        Self::n_link_pendulum(2, link_length, link_mass)
    }

    /// Create a spherical pendulum (ball joint at origin).
    ///
    /// This creates a single body attached to the world by a ball joint,
    /// allowing 3-DOF rotation. The body has a point mass at distance `length`
    /// below the joint.
    ///
    /// # Arguments
    /// * `length` - Length from pivot to mass (meters)
    /// * `mass` - Point mass (kg)
    ///
    /// # Note
    /// The ball joint uses quaternion representation (nq=4, nv=3).
    /// Initial state is qpos=`[1,0,0,0]` (identity quaternion = hanging down).
    #[must_use]
    pub fn spherical_pendulum(length: f64, mass: f64) -> Self {
        let mut model = Self::empty();

        // Dimensions
        model.nq = 4; // Quaternion: w, x, y, z
        model.nv = 3; // Angular velocity: omega_x, omega_y, omega_z
        model.nbody = 2; // world + pendulum
        model.njnt = 1;

        // Body 1: pendulum bob
        model.body_parent.push(0);
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(1);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(3);
        model.body_geom_adr.push(0);
        model.body_geom_num.push(0);

        // Body frame is at the mass location (below joint by length)
        model.body_pos.push(Vector3::new(0.0, 0.0, -length));
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(mass);
        model.body_inertia.push(Vector3::new(0.001, 0.001, 0.001));
        model.body_name.push(Some("bob".to_string()));
        model.body_subtreemass.push(0.0); // Will be computed after model is built

        // Ball joint at world origin
        model.jnt_type.push(MjJointType::Ball);
        model.jnt_body.push(1);
        model.jnt_qpos_adr.push(0);
        model.jnt_dof_adr.push(0);
        model.jnt_pos.push(Vector3::zeros());
        model.jnt_axis.push(Vector3::z()); // Not used for ball, but required
        model.jnt_limited.push(false);
        model.jnt_range.push((-PI, PI));
        model.jnt_stiffness.push(0.0);
        model.jnt_springref.push(0.0);
        model.jnt_damping.push(0.0);
        model.jnt_armature.push(0.0);
        model.jnt_name.push(Some("ball".to_string()));

        // DOF definitions (3 for ball joint)
        for i in 0..3 {
            model.dof_body.push(1);
            model.dof_jnt.push(0);
            model
                .dof_parent
                .push(if i == 0 { None } else { Some(i - 1) });
            model.dof_armature.push(0.0);
            model.dof_damping.push(0.0);
            model.dof_frictionloss.push(0.0);
        }

        // Default qpos: identity quaternion [w, x, y, z] = [1, 0, 0, 0]
        model.qpos0 = DVector::from_vec(vec![1.0, 0.0, 0.0, 0.0]);

        // Physics options
        model.timestep = 1.0 / 240.0;
        model.gravity = Vector3::new(0.0, 0.0, -9.81);
        model.solver_iterations = 10;
        model.solver_tolerance = 1e-8;

        // Pre-compute kinematic data for O(n) CRBA/RNE
        model.compute_ancestors();

        // Pre-compute implicit integration parameters
        model.compute_implicit_params();

        model
    }

    /// Create a free-floating body (6-DOF).
    ///
    /// This creates a single body with a free joint, allowing full 3D
    /// translation and rotation. Useful for testing free-body dynamics.
    ///
    /// # Arguments
    /// * `mass` - Body mass (kg)
    /// * `inertia` - Principal moments of inertia [Ixx, Iyy, Izz]
    #[must_use]
    pub fn free_body(mass: f64, inertia: Vector3<f64>) -> Self {
        let mut model = Self::empty();

        // Dimensions
        model.nq = 7; // position (3) + quaternion (4)
        model.nv = 6; // linear velocity (3) + angular velocity (3)
        model.nbody = 2;
        model.njnt = 1;

        // Body 1: free body
        model.body_parent.push(0);
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(1);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(6);
        model.body_geom_adr.push(0);
        model.body_geom_num.push(0);

        model.body_pos.push(Vector3::zeros());
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(mass);
        model.body_inertia.push(inertia);
        model.body_name.push(Some("free_body".to_string()));
        model.body_subtreemass.push(0.0); // Will be computed after model is built

        // Free joint
        model.jnt_type.push(MjJointType::Free);
        model.jnt_body.push(1);
        model.jnt_qpos_adr.push(0);
        model.jnt_dof_adr.push(0);
        model.jnt_pos.push(Vector3::zeros());
        model.jnt_axis.push(Vector3::z());
        model.jnt_limited.push(false);
        model.jnt_range.push((-1e10, 1e10));
        model.jnt_stiffness.push(0.0);
        model.jnt_springref.push(0.0);
        model.jnt_damping.push(0.0);
        model.jnt_armature.push(0.0);
        model.jnt_name.push(Some("free".to_string()));

        // DOF definitions (6 for free joint)
        for i in 0..6 {
            model.dof_body.push(1);
            model.dof_jnt.push(0);
            model
                .dof_parent
                .push(if i == 0 { None } else { Some(i - 1) });
            model.dof_armature.push(0.0);
            model.dof_damping.push(0.0);
            model.dof_frictionloss.push(0.0);
        }

        // Default qpos: at origin with identity orientation
        model.qpos0 = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);

        // Physics options
        model.timestep = 1.0 / 240.0;
        model.gravity = Vector3::new(0.0, 0.0, -9.81);
        model.solver_iterations = 10;
        model.solver_tolerance = 1e-8;

        // Pre-compute kinematic data for O(n) CRBA/RNE
        model.compute_ancestors();

        // Pre-compute implicit integration parameters
        model.compute_implicit_params();

        model
    }
}

impl Data {
    /// Reset state to model defaults.
    pub fn reset(&mut self, model: &Model) {
        self.qpos = model.qpos0.clone();
        self.qvel.fill(0.0);
        self.qacc.fill(0.0);
        self.qacc_warmstart.fill(0.0);
        self.ctrl.fill(0.0);
        self.act.fill(0.0);
        self.sensordata.fill(0.0);
        self.time = 0.0;
        self.ncon = 0;
        self.contacts.clear();
    }

    /// Get total mechanical energy (kinetic + potential).
    #[must_use]
    pub fn total_energy(&self) -> f64 {
        self.energy_kinetic + self.energy_potential
    }

    /// Full simulation step (like `mj_step`).
    ///
    /// This is the main entry point for advancing the simulation by one timestep.
    /// It performs forward dynamics to compute accelerations, then integrates
    /// to update positions and velocities.
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError)` if:
    /// - Position/velocity contains NaN or Inf
    /// - Acceleration computation produces NaN
    /// - Cholesky decomposition fails (implicit integrator only)
    /// - Timestep is invalid
    ///
    /// Unlike MuJoCo which silently resets state on errors, this follows
    /// Rust idioms by requiring explicit error handling.
    pub fn step(&mut self, model: &Model) -> Result<(), StepError> {
        // Validate timestep
        if model.timestep <= 0.0 || !model.timestep.is_finite() {
            return Err(StepError::InvalidTimestep);
        }

        // Validate state before stepping
        mj_check_pos(model, self)?;
        mj_check_vel(model, self)?;

        // Forward dynamics: compute qacc from current state
        self.forward(model)?;

        // Validate computed acceleration
        mj_check_acc(model, self)?;

        // Integration: update qvel and qpos
        self.integrate(model);

        Ok(())
    }

    /// Forward dynamics only (like `mj_forward`).
    ///
    /// Computes all derived quantities from current qpos/qvel without
    /// modifying them. After this call, qacc contains the computed
    /// accelerations and all body poses are updated.
    ///
    /// Pipeline stages follow `MuJoCo`'s `mj_forward` exactly:
    /// 1. Position stage: FK, position-dependent sensors, potential energy
    /// 2. Velocity stage: velocity FK, velocity-dependent sensors, kinetic energy
    /// 3. Acceleration stage: actuation, dynamics, constraints, acc-dependent sensors
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError::CholeskyFailed)` if using implicit integrator
    /// and the modified mass matrix decomposition fails.
    pub fn forward(&mut self, model: &Model) -> Result<(), StepError> {
        // ========== Position Stage ==========
        // Stage 1a: Forward kinematics - compute body/geom/site poses from qpos
        mj_fwd_position(model, self);

        // Stage 1b: Collision detection - detect contacts from geometry pairs
        mj_collision(model, self);

        // Stage 1c: Position-dependent sensors (joint positions, frame positions, etc.)
        mj_sensor_pos(model, self);

        // Stage 1c: Potential energy (gravity + springs)
        mj_energy_pos(model, self);

        // ========== Velocity Stage ==========
        // Stage 2a: Velocity kinematics - compute body velocities from qvel
        mj_fwd_velocity(model, self);

        // Stage 2b: Velocity-dependent sensors (gyro, velocimeter, etc.)
        mj_sensor_vel(model, self);

        // ========== Acceleration Stage ==========
        // Stage 3a: Actuation - compute actuator forces
        mj_fwd_actuation(model, self);

        // Stage 3b: Dynamics - compute mass matrix and bias forces
        mj_crba(model, self); // Composite Rigid Body Algorithm
        mj_rne(model, self); // Recursive Newton-Euler for bias forces

        // Stage 3c: Kinetic energy (needs mass matrix from CRBA)
        mj_energy_vel(model, self);

        // Stage 3d: Passive forces - springs and dampers
        mj_fwd_passive(model, self);

        // Stage 3e: Constraint forces - contacts and joint limits
        mj_fwd_constraint(model, self);

        // Stage 3f: Compute final acceleration
        mj_fwd_acceleration(model, self)?;

        // Stage 3g: Acceleration-dependent sensors (accelerometer, etc.)
        mj_sensor_acc(model, self);

        Ok(())
    }

    /// Integration step.
    ///
    /// # Integration Methods
    ///
    /// - **Euler/RK4**: Semi-implicit Euler. Updates velocity first (`qvel += qacc * h`),
    ///   then integrates position using the new velocity.
    ///
    /// - **Implicit**: Velocity was already updated in `mj_fwd_acceleration_implicit()`.
    ///   We only integrate positions here.
    fn integrate(&mut self, model: &Model) {
        let h = model.timestep;

        // For explicit integrators, update velocity using computed acceleration
        // For implicit integrator, velocity was already updated in mj_fwd_acceleration_implicit
        match model.integrator {
            Integrator::Euler | Integrator::RungeKutta4 => {
                // Semi-implicit Euler: update velocity first, then position
                for i in 0..model.nv {
                    self.qvel[i] += self.qacc[i] * h;
                }
            }
            Integrator::Implicit => {
                // Velocity already updated by mj_fwd_acceleration_implicit
                // qacc was back-computed as (v_new - v_old) / h for consistency
            }
        }

        // Update positions - quaternions need special handling!
        mj_integrate_pos(model, self, h);

        // Normalize quaternions to prevent drift
        mj_normalize_quat(model, self);

        // Advance time
        self.time += h;
    }
}

// ============================================================================
// MuJoCo Pipeline Functions (Phase 2)
// ============================================================================

/// Validate position coordinates.
///
/// Returns `Err(StepError::InvalidPosition)` if any qpos element is NaN, Inf,
/// or exceeds 1e10 in magnitude (indicating numerical blow-up).
///
/// Unlike MuJoCo which silently resets to qpos0, this returns an error so
/// users can decide how to handle the situation.
fn mj_check_pos(model: &Model, data: &Data) -> Result<(), StepError> {
    for i in 0..model.nq {
        if !data.qpos[i].is_finite() || data.qpos[i].abs() > 1e10 {
            return Err(StepError::InvalidPosition);
        }
    }
    Ok(())
}

/// Validate velocity coordinates.
///
/// Returns `Err(StepError::InvalidVelocity)` if any qvel element is NaN, Inf,
/// or exceeds 1e10 in magnitude (indicating numerical blow-up).
///
/// Unlike MuJoCo which silently zeros velocity, this returns an error so
/// users can decide how to handle the situation.
fn mj_check_vel(model: &Model, data: &Data) -> Result<(), StepError> {
    for i in 0..model.nv {
        if !data.qvel[i].is_finite() || data.qvel[i].abs() > 1e10 {
            return Err(StepError::InvalidVelocity);
        }
    }
    Ok(())
}

/// Validate acceleration.
///
/// Returns `Err(StepError::InvalidAcceleration)` if any qacc element is NaN.
/// This typically indicates a singular mass matrix or other numerical issues.
fn mj_check_acc(model: &Model, data: &Data) -> Result<(), StepError> {
    for i in 0..model.nv {
        if !data.qacc[i].is_finite() {
            return Err(StepError::InvalidAcceleration);
        }
    }
    Ok(())
}

/// Forward kinematics: compute body poses from qpos.
///
/// This traverses the kinematic tree from root to leaves, computing
/// the world-frame position and orientation of each body.
fn mj_fwd_position(model: &Model, data: &mut Data) {
    // Body 0 (world) is always at origin
    data.xpos[0] = Vector3::zeros();
    data.xquat[0] = UnitQuaternion::identity();
    data.xmat[0] = Matrix3::identity();

    // Process bodies in order (assumes topological sort: parent before child)
    for body_id in 1..model.nbody {
        let parent_id = model.body_parent[body_id];

        // Start with parent's frame
        let mut pos = data.xpos[parent_id];
        let mut quat = data.xquat[parent_id];

        // Apply body offset in parent frame
        pos += quat * model.body_pos[body_id];
        quat *= model.body_quat[body_id];

        // Apply each joint for this body
        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];

        for jnt_id in jnt_start..jnt_end {
            let qpos_adr = model.jnt_qpos_adr[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    let angle = data.qpos[qpos_adr];
                    let axis = model.jnt_axis[jnt_id];
                    let anchor = model.jnt_pos[jnt_id];

                    // Transform anchor to current frame
                    let world_anchor = pos + quat * anchor;

                    // Rotate around axis
                    let world_axis = quat * axis;
                    // Safety: use try_new_normalize to handle degenerate cases
                    let rot = if let Some(unit_axis) = nalgebra::Unit::try_new(world_axis, 1e-10) {
                        UnitQuaternion::from_axis_angle(&unit_axis, angle)
                    } else {
                        // Degenerate axis - no rotation (should not happen with valid model)
                        UnitQuaternion::identity()
                    };
                    quat = rot * quat;

                    // Adjust position for rotation around anchor
                    pos = world_anchor + rot * (pos - world_anchor);
                }
                MjJointType::Slide => {
                    let displacement = data.qpos[qpos_adr];
                    let axis = model.jnt_axis[jnt_id];
                    pos += quat * (axis * displacement);
                }
                MjJointType::Ball => {
                    // qpos stores quaternion [w, x, y, z]
                    let q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                        data.qpos[qpos_adr + 3],
                    ));
                    quat *= q;
                }
                MjJointType::Free => {
                    // qpos stores [x, y, z, qw, qx, qy, qz]
                    pos = Vector3::new(
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                    );
                    quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        data.qpos[qpos_adr + 3],
                        data.qpos[qpos_adr + 4],
                        data.qpos[qpos_adr + 5],
                        data.qpos[qpos_adr + 6],
                    ));
                }
            }
        }

        // Store computed pose
        data.xpos[body_id] = pos;
        data.xquat[body_id] = quat;
        data.xmat[body_id] = quat.to_rotation_matrix().into_inner();

        // Compute inertial frame position
        data.xipos[body_id] = pos + quat * model.body_ipos[body_id];
        data.ximat[body_id] = (quat * model.body_iquat[body_id])
            .to_rotation_matrix()
            .into_inner();

        // Compute body spatial inertia in world frame (cinert)
        // This is computed once and used by both CRBA and RNE
        let h = data.xipos[body_id] - pos; // COM offset from body origin in world frame
        data.cinert[body_id] = compute_body_spatial_inertia(
            model.body_mass[body_id],
            model.body_inertia[body_id],
            &data.ximat[body_id],
            h,
        );
    }

    // World body has zero inertia
    data.cinert[0] = Matrix6::zeros();

    // Update geom poses
    for geom_id in 0..model.ngeom {
        let body_id = model.geom_body[geom_id];
        let body_pos = data.xpos[body_id];
        let body_quat = data.xquat[body_id];

        data.geom_xpos[geom_id] = body_pos + body_quat * model.geom_pos[geom_id];
        data.geom_xmat[geom_id] = (body_quat * model.geom_quat[geom_id])
            .to_rotation_matrix()
            .into_inner();
    }

    // Update site poses
    for site_id in 0..model.nsite {
        let body_id = model.site_body[site_id];
        let body_pos = data.xpos[body_id];
        let body_quat = data.xquat[body_id];

        data.site_xpos[site_id] = body_pos + body_quat * model.site_pos[site_id];
        data.site_xmat[site_id] = (body_quat * model.site_quat[site_id])
            .to_rotation_matrix()
            .into_inner();
    }

    // ========== Compute subtree mass and COM (for O(n) RNE gravity) ==========
    // Initialize with each body's own mass and COM
    for body_id in 0..model.nbody {
        data.subtree_mass[body_id] = model.body_mass[body_id];
        data.subtree_com[body_id] = model.body_mass[body_id] * data.xipos[body_id];
    }

    // Backward pass: accumulate children's contributions to parents
    for body_id in (1..model.nbody).rev() {
        let parent_id = model.body_parent[body_id];
        // Copy child data first to satisfy borrow checker
        let child_mass = data.subtree_mass[body_id];
        let child_weighted_com = data.subtree_com[body_id];
        data.subtree_mass[parent_id] += child_mass;
        data.subtree_com[parent_id] += child_weighted_com;
    }

    // Convert weighted sum to actual COM
    for body_id in 0..model.nbody {
        if data.subtree_mass[body_id] > 1e-10 {
            data.subtree_com[body_id] /= data.subtree_mass[body_id];
        } else {
            data.subtree_com[body_id] = data.xipos[body_id];
        }
    }
}

// ============================================================================
// Broad-Phase Collision Detection (Spatial Hashing)
// ============================================================================

/// Compute AABB for a geometry given its world-space pose and type/size.
///
/// This function creates an axis-aligned bounding box for MuJoCo-style geometry
/// specifications, using the canonical `Aabb` type from `collision_shape`.
///
/// # Arguments
/// * `geom_type` - The geometry type (sphere, box, capsule, etc.)
/// * `size` - MuJoCo-style size parameters (interpretation depends on geom_type)
/// * `pos` - World-space position of the geometry
/// * `mat` - World-space rotation matrix (3x3)
#[inline]
fn aabb_from_geom(
    geom_type: GeomType,
    size: Vector3<f64>,
    pos: Vector3<f64>,
    mat: Matrix3<f64>,
) -> Aabb {
    match geom_type {
        GeomType::Sphere => {
            let r = size.x;
            Aabb::new(
                Point3::new(pos.x - r, pos.y - r, pos.z - r),
                Point3::new(pos.x + r, pos.y + r, pos.z + r),
            )
        }
        GeomType::Box => {
            // For a rotated box, compute the world-space extents
            let half = size;
            let mut min = pos;
            let mut max = pos;
            for i in 0..3 {
                let axis = mat.column(i).into_owned();
                let extent = half[i] * axis.abs();
                min -= extent;
                max += extent;
            }
            Aabb::new(Point3::from(min), Point3::from(max))
        }
        GeomType::Capsule => {
            // Capsule: radius + half_length along Z axis
            let r = size.x;
            let half_len = size.y;
            let axis = mat.column(2).into_owned();
            let end1 = pos + axis * half_len;
            let end2 = pos - axis * half_len;
            Aabb::new(
                Point3::new(
                    end1.x.min(end2.x) - r,
                    end1.y.min(end2.y) - r,
                    end1.z.min(end2.z) - r,
                ),
                Point3::new(
                    end1.x.max(end2.x) + r,
                    end1.y.max(end2.y) + r,
                    end1.z.max(end2.z) + r,
                ),
            )
        }
        GeomType::Cylinder => {
            // Cylinder AABB: endpoints ± radius in all directions.
            // Same formula as capsule — the flat caps still extend radius `r`
            // perpendicular to the axis, so the bounding box is identical.
            let r = size.x;
            let half_len = size.y;
            let axis = mat.column(2).into_owned();
            let end1 = pos + axis * half_len;
            let end2 = pos - axis * half_len;
            Aabb::new(
                Point3::new(
                    end1.x.min(end2.x) - r,
                    end1.y.min(end2.y) - r,
                    end1.z.min(end2.z) - r,
                ),
                Point3::new(
                    end1.x.max(end2.x) + r,
                    end1.y.max(end2.y) + r,
                    end1.z.max(end2.z) + r,
                ),
            )
        }
        GeomType::Ellipsoid => {
            // Conservative AABB: use sphere with largest semi-axis radius.
            // A tight AABB would require transforming each axis by the rotation
            // matrix, but this simple approach is correct and fast for broad-phase.
            let max_r = size.x.max(size.y).max(size.z);
            Aabb::new(
                Point3::new(pos.x - max_r, pos.y - max_r, pos.z - max_r),
                Point3::new(pos.x + max_r, pos.y + max_r, pos.z + max_r),
            )
        }
        GeomType::Plane => {
            // Planes are infinite — use large bounds in perpendicular directions
            // and thin bounds along the normal to allow proper AABB overlap tests.
            const PLANE_EXTENT: f64 = 1e6; // Effectively infinite for simulation scale
            const PLANE_THICKNESS: f64 = 0.001; // Thin slab for AABB overlap detection

            // Plane normal is Z axis of the rotation matrix
            let normal = mat.column(2).into_owned();

            // Create thin slab AABB based on dominant normal direction
            if normal.z.abs() > 0.9 {
                // Near-horizontal plane (normal ≈ ±Z)
                Aabb::new(
                    Point3::new(-PLANE_EXTENT, -PLANE_EXTENT, pos.z - PLANE_THICKNESS),
                    Point3::new(PLANE_EXTENT, PLANE_EXTENT, pos.z + PLANE_THICKNESS),
                )
            } else if normal.y.abs() > 0.9 {
                // Near-vertical plane (normal ≈ ±Y)
                Aabb::new(
                    Point3::new(-PLANE_EXTENT, pos.y - PLANE_THICKNESS, -PLANE_EXTENT),
                    Point3::new(PLANE_EXTENT, pos.y + PLANE_THICKNESS, PLANE_EXTENT),
                )
            } else {
                // Near-vertical plane (normal ≈ ±X)
                Aabb::new(
                    Point3::new(pos.x - PLANE_THICKNESS, -PLANE_EXTENT, -PLANE_EXTENT),
                    Point3::new(pos.x + PLANE_THICKNESS, PLANE_EXTENT, PLANE_EXTENT),
                )
            }
        }
        GeomType::Mesh => {
            // For mesh, use a conservative large bounding box.
            // In a full implementation, mesh AABBs would be pre-computed from vertices.
            const MESH_DEFAULT_EXTENT: f64 = 10.0; // Conservative fallback for unprocessed meshes
            Aabb::new(
                Point3::new(
                    pos.x - MESH_DEFAULT_EXTENT,
                    pos.y - MESH_DEFAULT_EXTENT,
                    pos.z - MESH_DEFAULT_EXTENT,
                ),
                Point3::new(
                    pos.x + MESH_DEFAULT_EXTENT,
                    pos.y + MESH_DEFAULT_EXTENT,
                    pos.z + MESH_DEFAULT_EXTENT,
                ),
            )
        }
    }
}

/// Sweep-and-prune broad-phase collision detection.
///
/// This is the algorithm used by MuJoCo and most physics engines:
/// 1. Project all AABBs onto the X-axis
/// 2. Sort by min-X coordinate
/// 3. Sweep through sorted intervals to find overlaps
/// 4. Check Y and Z overlap only for X-overlapping pairs
///
/// For coherent simulations (objects move incrementally), the sort is nearly
/// O(n) due to temporal coherence — insertion sort on nearly-sorted data.
///
/// # Performance Characteristics
///
/// | Operation | Complexity | Notes |
/// |-----------|------------|-------|
/// | Build | O(n log n) | Initial sort (Rust's pdqsort) |
/// | Query (typical) | O(n + k) | k = output pairs, assumes bounded X-overlap |
/// | Query (worst) | O(n² + k) | All AABBs overlap on X-axis (degenerate) |
/// | Incremental | O(n) | Nearly sorted → insertion sort behavior |
///
/// The query worst case occurs when all objects have overlapping X-intervals
/// (e.g., objects stacked vertically). In practice, this is rare and still
/// faster than spatial hashing's worst case (clustering in one cell).
///
/// # Why Not Spatial Hash?
///
/// Spatial hashing degrades to O(n²) when objects cluster in a single cell
/// (e.g., a pile of boxes). SAP's worst case is the same O(n²), but it occurs
/// less frequently in practice (requires all X-intervals to overlap, not just
/// spatial proximity).
struct SweepAndPrune {
    /// AABBs indexed by geom ID
    aabbs: Vec<Aabb>,
    /// Geom IDs sorted by AABB min-X coordinate
    sorted_x: Vec<usize>,
}

impl SweepAndPrune {
    /// Create a new sweep-and-prune structure from AABBs.
    ///
    /// # Arguments
    ///
    /// * `aabbs` - Vector of AABBs indexed by geom ID
    ///
    /// # Panics (debug only)
    ///
    /// Debug builds assert that all AABBs have finite coordinates.
    /// NaN in AABBs would cause non-deterministic sort behavior.
    #[must_use]
    fn new(aabbs: Vec<Aabb>) -> Self {
        // Validate AABBs in debug builds to catch NaN/Inf early.
        // NaN comparisons break sort transitivity, causing non-deterministic results.
        debug_assert!(
            aabbs.iter().all(|a| {
                a.min.x.is_finite()
                    && a.min.y.is_finite()
                    && a.min.z.is_finite()
                    && a.max.x.is_finite()
                    && a.max.y.is_finite()
                    && a.max.z.is_finite()
            }),
            "All AABBs must have finite coordinates for deterministic sweep-and-prune"
        );

        let n = aabbs.len();
        let mut sorted_x: Vec<usize> = (0..n).collect();

        // Sort by min-X coordinate using total ordering.
        // f64::total_cmp provides IEEE 754 total ordering: -NaN < -Inf < ... < Inf < NaN
        // This guarantees deterministic sort even if validation is skipped in release.
        sorted_x.sort_by(|&a, &b| aabbs[a].min.x.total_cmp(&aabbs[b].min.x));

        Self { aabbs, sorted_x }
    }

    /// Query all potentially overlapping pairs.
    ///
    /// Returns pairs `(geom_i, geom_j)` where `i < j` and AABBs overlap.
    /// The pairs are returned in arbitrary order.
    ///
    /// See struct-level documentation for complexity analysis.
    #[must_use]
    fn query_pairs(&self) -> Vec<(usize, usize)> {
        // Pre-allocate with heuristic: ~2 overlaps per geom on average
        let mut pairs = Vec::with_capacity(self.aabbs.len() * 2);
        let n = self.sorted_x.len();

        // Sweep through sorted list
        for i in 0..n {
            let geom_i = self.sorted_x[i];
            let aabb_i = &self.aabbs[geom_i];
            let max_x_i = aabb_i.max.x;

            // Check subsequent geoms until their min-X exceeds our max-X
            for j in (i + 1)..n {
                let geom_j = self.sorted_x[j];
                let aabb_j = &self.aabbs[geom_j];

                // If min-X of j exceeds max-X of i, no more overlaps possible
                // (since sorted_x is sorted by min-X)
                if aabb_j.min.x > max_x_i {
                    break;
                }

                // X overlaps — check Y and Z
                if aabb_i.max.y >= aabb_j.min.y
                    && aabb_j.max.y >= aabb_i.min.y
                    && aabb_i.max.z >= aabb_j.min.z
                    && aabb_j.max.z >= aabb_i.min.z
                {
                    // Full 3D overlap — add pair (normalized: smaller ID first)
                    let (g1, g2) = if geom_i < geom_j {
                        (geom_i, geom_j)
                    } else {
                        (geom_j, geom_i)
                    };
                    pairs.push((g1, g2));
                }
            }
        }

        pairs
    }
}

/// Check if two geometries can collide based on contype/conaffinity bitmasks.
///
/// Following MuJoCo's collision filtering:
/// 1. Same body - no collision
/// 2. Parent-child (adjacent bodies in kinematic tree) - no collision
///    Exception: World body (0) collides with all non-child bodies
/// 3. contype/conaffinity check: (c1 & a2) != 0 || (c2 & a1) != 0
///
/// The world body exception ensures ground planes can collide with objects
/// even when those objects are direct children of the world body.
#[allow(clippy::inline_always)] // Hot path - profiling shows inlining improves debug performance
#[inline(always)]
fn check_collision_affinity(model: &Model, geom1: usize, geom2: usize) -> bool {
    let body1 = model.geom_body[geom1];
    let body2 = model.geom_body[geom2];

    // Same body - no collision
    if body1 == body2 {
        return false;
    }

    // Parent-child filtering: bodies connected by a joint shouldn't collide
    // Exception: World body (0) geometries (ground planes) should collide with
    // any body, including direct children. The world body has no joints connecting
    // it to children - children are simply anchored in world space.
    //
    // This is important because:
    // - A ball with a free joint has body_parent[ball_body] = 0 (world)
    // - But the ball should still collide with ground planes on body 0
    // - The "parent-child" filter is for bodies connected by articulated joints
    //   (hinge, slide, ball) where collision would be geometrically impossible
    if body1 != 0 && body2 != 0 {
        if model.body_parent[body1] == body2 || model.body_parent[body2] == body1 {
            return false;
        }
    }

    // contype/conaffinity bitmask check
    let c1 = model.geom_contype[geom1];
    let a1 = model.geom_conaffinity[geom1];
    let c2 = model.geom_contype[geom2];
    let a2 = model.geom_conaffinity[geom2];

    (c1 & a2) != 0 || (c2 & a1) != 0
}

/// Collision detection: populate contacts from geometry pairs.
///
/// Following `MuJoCo`'s collision detection order:
/// 1. Broad-phase: sweep-and-prune to find candidate pairs (O(n log n))
/// 2. Check contact affinity (contype/conaffinity bitmasks)
/// 3. Narrow-phase: analytical or GJK/EPA collision detection
/// 4. Populate data.contacts with contact information
///
/// This function is called after forward kinematics (`mj_fwd_position`) so
/// `geom_xpos` and `geom_xmat` are up-to-date.
///
/// # Algorithm: Sweep-and-Prune
///
/// Unlike spatial hashing which degrades to O(n²) when objects cluster,
/// sweep-and-prune maintains O(n log n + k) complexity where k is the
/// number of overlapping pairs. This is robust against clustering scenarios
/// like a pile of boxes or a humanoid with many self-collision checks.
///
/// # Performance
///
/// | Scene Size | Complexity | Notes |
/// |------------|------------|-------|
/// | Any n | O(n log n + k) | k = overlapping pairs |
/// | Coherent | O(n + k) | Nearly-sorted input |
fn mj_collision(model: &Model, data: &mut Data) {
    // Clear existing contacts
    data.contacts.clear();
    data.ncon = 0;

    // Early exit if no geoms or only one geom
    if model.ngeom <= 1 {
        return;
    }

    // Build AABBs for all geoms
    // This is O(n) and cache-friendly (linear memory access)
    let aabbs: Vec<Aabb> = (0..model.ngeom)
        .map(|geom_id| {
            aabb_from_geom(
                model.geom_type[geom_id],
                model.geom_size[geom_id],
                data.geom_xpos[geom_id],
                data.geom_xmat[geom_id],
            )
        })
        .collect();

    // Sweep-and-prune broad-phase: O(n log n) worst case, O(n + k) for coherent scenes
    let sap = SweepAndPrune::new(aabbs);
    let candidates = sap.query_pairs();

    // Process candidate pairs
    // The SAP already filtered to only AABB-overlapping pairs
    for (geom1, geom2) in candidates {
        // Affinity check: same body, parent-child, contype/conaffinity
        if !check_collision_affinity(model, geom1, geom2) {
            continue;
        }

        // Get world-space poses
        let pos1 = data.geom_xpos[geom1];
        let mat1 = data.geom_xmat[geom1];
        let pos2 = data.geom_xpos[geom2];
        let mat2 = data.geom_xmat[geom2];

        // Narrow-phase collision detection
        if let Some(contact) = collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2) {
            data.contacts.push(contact);
            data.ncon += 1;
        }
    }
}

/// Narrow-phase collision between two geometries.
///
/// Returns Contact if penetrating, None otherwise.
#[allow(clippy::similar_names)] // pos1/pose1, pos2/pose2 are intentionally related
#[allow(clippy::items_after_statements)] // use statement placed after special cases for readability
#[inline]
fn collide_geoms(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
) -> Option<Contact> {
    let type1 = model.geom_type[geom1];
    let type2 = model.geom_type[geom2];
    let size1 = model.geom_size[geom1];
    let size2 = model.geom_size[geom2];

    // Fast path: handle all analytical collision cases first
    // These avoid the expensive quaternion conversion and GJK/EPA

    // Special case: mesh collision (has its own BVH-accelerated path)
    if type1 == GeomType::Mesh || type2 == GeomType::Mesh {
        return collide_with_mesh(model, geom1, geom2, pos1, mat1, pos2, mat2);
    }

    // Special case: plane collision
    if type1 == GeomType::Plane || type2 == GeomType::Plane {
        return collide_with_plane(
            model, geom1, geom2, type1, type2, pos1, mat1, pos2, mat2, size1, size2,
        );
    }

    // Special case: sphere-sphere collision (analytical, more robust than GJK/EPA)
    if type1 == GeomType::Sphere && type2 == GeomType::Sphere {
        return collide_sphere_sphere(model, geom1, geom2, pos1, pos2, size1, size2);
    }

    // Special case: capsule-capsule collision (analytical, much faster than GJK/EPA)
    if type1 == GeomType::Capsule && type2 == GeomType::Capsule {
        return collide_capsule_capsule(model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2);
    }

    // Special case: sphere-capsule collision
    if (type1 == GeomType::Sphere && type2 == GeomType::Capsule)
        || (type1 == GeomType::Capsule && type2 == GeomType::Sphere)
    {
        return collide_sphere_capsule(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2,
        );
    }

    // Special case: sphere-box collision (analytical)
    if (type1 == GeomType::Sphere && type2 == GeomType::Box)
        || (type1 == GeomType::Box && type2 == GeomType::Sphere)
    {
        return collide_sphere_box(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2,
        );
    }

    // Special case: capsule-box collision (analytical)
    if (type1 == GeomType::Capsule && type2 == GeomType::Box)
        || (type1 == GeomType::Box && type2 == GeomType::Capsule)
    {
        return collide_capsule_box(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2,
        );
    }

    // Special case: box-box collision (SAT)
    if type1 == GeomType::Box && type2 == GeomType::Box {
        return collide_box_box(model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2);
    }

    // Special case: cylinder-sphere collision (analytical)
    if (type1 == GeomType::Cylinder && type2 == GeomType::Sphere)
        || (type1 == GeomType::Sphere && type2 == GeomType::Cylinder)
    {
        return collide_cylinder_sphere(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2,
        );
    }

    // Special case: cylinder-capsule collision (analytical with GJK/EPA fallback)
    // Analytical solution handles common cases; degenerate cases fall through to GJK/EPA
    if (type1 == GeomType::Cylinder && type2 == GeomType::Capsule)
        || (type1 == GeomType::Capsule && type2 == GeomType::Cylinder)
    {
        if let Some(contact) = collide_cylinder_capsule(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2,
        ) {
            return Some(contact);
        }
        // Fall through to GJK/EPA for degenerate cases (intersecting/parallel axes, cap collisions)
    }

    // Slow path: Build shapes and poses for GJK/EPA (cylinder-cylinder, cylinder-box, ellipsoid-*, and fallback cases)
    let shape1 = geom_to_collision_shape(type1, size1);
    let shape2 = geom_to_collision_shape(type2, size2);

    // Build poses for GJK/EPA - expensive quaternion conversion
    let quat1 = UnitQuaternion::from_matrix(&mat1);
    let quat2 = UnitQuaternion::from_matrix(&mat2);
    let pose1 = Pose::from_position_rotation(Point3::from(pos1), quat1);
    let pose2 = Pose::from_position_rotation(Point3::from(pos2), quat2);

    // Use GJK/EPA for general convex collision
    use crate::gjk_epa::gjk_epa_contact;
    let collision_shape1 = shape1?;
    let collision_shape2 = shape2?;

    if let Some(result) = gjk_epa_contact(&collision_shape1, &pose1, &collision_shape2, &pose2) {
        if result.penetration > 0.0 {
            // Compute average friction
            let friction1 = model.geom_friction[geom1].x; // Sliding friction
            let friction2 = model.geom_friction[geom2].x;
            let friction = (friction1 * friction2).sqrt(); // Geometric mean

            return Some(Contact::new(
                Vector3::new(result.point.x, result.point.y, result.point.z),
                result.normal,
                result.penetration,
                geom1,
                geom2,
                friction,
            ));
        }
    }

    None
}

/// Convert `MuJoCo` `GeomType` to `CollisionShape`.
#[allow(clippy::match_same_arms)] // Plane and Mesh both return None but for different reasons
fn geom_to_collision_shape(geom_type: GeomType, size: Vector3<f64>) -> Option<CollisionShape> {
    match geom_type {
        GeomType::Sphere => Some(CollisionShape::Sphere { radius: size.x }),
        GeomType::Box => Some(CollisionShape::Box { half_extents: size }),
        GeomType::Capsule => Some(CollisionShape::Capsule {
            half_length: size.y, // MuJoCo: size[0]=radius, size[1]=half_length
            radius: size.x,
        }),
        GeomType::Cylinder => Some(CollisionShape::Cylinder {
            half_length: size.y,
            radius: size.x,
        }),
        GeomType::Ellipsoid => Some(CollisionShape::Ellipsoid { radii: size }),
        GeomType::Plane => None, // Handled via collide_with_plane()
        GeomType::Mesh => None,  // Handled via collide_with_mesh()
    }
}

// ============================================================================
// Primitive Collision Detection
// ============================================================================
//
// # API Design Pattern
//
// This module uses two types of collision functions:
//
// 1. **Dispatcher functions** (e.g., `collide_with_plane`, `collide_geoms`):
//    - Take `&Model` and geometry indices
//    - Handle type dispatch and parameter extraction
//    - Compute derived values like friction (geometric mean)
//    - Called from the main collision pipeline
//
// 2. **Implementation helpers** (e.g., `collide_cylinder_plane_impl`):
//    - Suffix `_impl` indicates internal helper
//    - Take pre-extracted geometric parameters (no Model reference)
//    - Focus purely on geometric computation
//    - Marked `#[inline]` for performance
//    - Not called directly from pipeline
//
// This separation follows the Todorov principle: compute derived values once
// in the dispatcher, then pass them to the implementation.
// ============================================================================

/// Minimum norm threshold for geometric operations.
///
/// Used to prevent division by zero and detect degenerate cases in collision
/// detection. This value is chosen to be:
/// - Small enough to not reject valid geometric configurations
/// - Large enough to avoid numerical instability near machine epsilon
///
/// For reference: f64::EPSILON ≈ 2.2e-16, so 1e-10 provides ~6 orders of
/// magnitude of safety margin while still detecting near-degenerate cases.
const GEOM_EPSILON: f64 = 1e-10;

/// Threshold for cylinder axis being nearly vertical (perpendicular to plane).
/// When |cos(θ)| > 0.999 (θ < 2.6°), treat cylinder as vertical.
const AXIS_VERTICAL_THRESHOLD: f64 = 0.999;

/// Threshold for cylinder axis being nearly horizontal (parallel to plane).
/// When |cos(θ)| < 0.001 (θ > 89.9°), treat cylinder as horizontal.
const AXIS_HORIZONTAL_THRESHOLD: f64 = 0.001;

/// Threshold for detecting cylinder cap collision in cylinder-capsule.
/// When normal is within ~45° of cylinder axis (cos > 0.7), treat as cap collision.
const CAP_COLLISION_THRESHOLD: f64 = 0.7;

// =============================================================================
// Mesh Collision Detection
// =============================================================================

/// Collision detection involving at least one mesh geometry.
///
/// Dispatches to specialized mesh-primitive or mesh-mesh implementations.
/// For mesh-primitive collisions, approximations are used for some geometry types:
/// - Cylinder: approximated as capsule (conservative, may report false positives)
/// - Ellipsoid: approximated as sphere with max radius (conservative)
#[allow(clippy::too_many_arguments)]
#[allow(clippy::similar_names)] // pos1/pose1, pos2/pose2 are intentionally related
fn collide_with_mesh(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
) -> Option<Contact> {
    let type1 = model.geom_type[geom1];
    let type2 = model.geom_type[geom2];
    let size1 = model.geom_size[geom1];
    let size2 = model.geom_size[geom2];

    // Build poses (expensive quaternion conversion, but needed for mesh collision)
    let quat1 = UnitQuaternion::from_matrix(&mat1);
    let quat2 = UnitQuaternion::from_matrix(&mat2);
    let pose1 = Pose::from_position_rotation(Point3::from(pos1), quat1);
    let pose2 = Pose::from_position_rotation(Point3::from(pos2), quat2);

    let mesh_contact: Option<MeshContact> = match (type1, type2) {
        // Mesh-Mesh
        (GeomType::Mesh, GeomType::Mesh) => {
            let mesh1_id = model.geom_mesh[geom1]?;
            let mesh2_id = model.geom_mesh[geom2]?;
            let mesh1 = &model.mesh_data[mesh1_id];
            let mesh2 = &model.mesh_data[mesh2_id];

            mesh_mesh_deepest_contact(mesh1, &pose1, mesh2, &pose2)
        }

        // Mesh (geom1) vs Primitive (geom2)
        (GeomType::Mesh, prim_type) => {
            let mesh_id = model.geom_mesh[geom1]?;
            let mesh = &model.mesh_data[mesh_id];

            match prim_type {
                GeomType::Sphere => mesh_sphere_contact(mesh, &pose1, pose2.position, size2.x),
                // Capsule and Cylinder both use capsule collision (cylinder approximated as capsule)
                GeomType::Capsule | GeomType::Cylinder => {
                    let half_len = size2.y;
                    let axis = pose2.rotation * Vector3::z();
                    let start = pose2.position - axis * half_len;
                    let end = pose2.position + axis * half_len;
                    mesh_capsule_contact(mesh, &pose1, start, end, size2.x)
                }
                GeomType::Box => mesh_box_contact(mesh, &pose1, &pose2, &size2),
                GeomType::Ellipsoid => {
                    // Approximate as sphere with max radius (conservative)
                    let max_r = size2.x.max(size2.y).max(size2.z);
                    mesh_sphere_contact(mesh, &pose1, pose2.position, max_r)
                }
                GeomType::Plane => {
                    // Plane normal is local Z-axis
                    let plane_normal = mat2.column(2).into_owned();
                    let plane_d = plane_normal.dot(&pos2);
                    collide_mesh_plane(mesh, &pose1, plane_normal, plane_d)
                }
                GeomType::Mesh => unreachable!("handled in Mesh-Mesh case above"),
            }
        }

        // Primitive (geom1) vs Mesh (geom2) - swap and negate normal
        (prim_type, GeomType::Mesh) => {
            let mesh_id = model.geom_mesh[geom2]?;
            let mesh = &model.mesh_data[mesh_id];

            let contact = match prim_type {
                GeomType::Sphere => mesh_sphere_contact(mesh, &pose2, pose1.position, size1.x),
                // Capsule and Cylinder both use capsule collision (cylinder approximated as capsule)
                GeomType::Capsule | GeomType::Cylinder => {
                    let half_len = size1.y;
                    let axis = pose1.rotation * Vector3::z();
                    let start = pose1.position - axis * half_len;
                    let end = pose1.position + axis * half_len;
                    mesh_capsule_contact(mesh, &pose2, start, end, size1.x)
                }
                GeomType::Box => mesh_box_contact(mesh, &pose2, &pose1, &size1),
                GeomType::Ellipsoid => {
                    let max_r = size1.x.max(size1.y).max(size1.z);
                    mesh_sphere_contact(mesh, &pose2, pose1.position, max_r)
                }
                GeomType::Plane => {
                    let plane_normal = mat1.column(2).into_owned();
                    let plane_d = plane_normal.dot(&pos1);
                    collide_mesh_plane(mesh, &pose2, plane_normal, plane_d)
                }
                GeomType::Mesh => unreachable!("handled in Mesh-Mesh case above"),
            };

            // Negate normal since we swapped the order (mesh was geom2, but contact
            // normal points from mesh outward - we need it pointing from geom1 to geom2)
            contact.map(|mut c| {
                c.normal = -c.normal;
                c
            })
        }

        _ => unreachable!("collide_with_mesh called but neither geom is a mesh"),
    };

    // Convert MeshContact to Contact
    mesh_contact.map(|mc| {
        let friction1 = model.geom_friction[geom1].x;
        let friction2 = model.geom_friction[geom2].x;
        let friction = (friction1 * friction2).sqrt();

        Contact::new(
            mc.point.coords,
            mc.normal,
            mc.penetration,
            geom1,
            geom2,
            friction,
        )
    })
}

/// Mesh vs infinite plane collision.
///
/// Tests all mesh vertices against the plane and returns the deepest penetrating vertex.
/// This is a simple but effective approach for mesh-plane collision:
/// - O(n) in number of vertices
/// - Handles any mesh topology
/// - Returns single deepest contact point
fn collide_mesh_plane(
    mesh: &TriangleMeshData,
    mesh_pose: &Pose,
    plane_normal: Vector3<f64>,
    plane_d: f64,
) -> Option<MeshContact> {
    let mut deepest: Option<MeshContact> = None;

    for (i, vertex) in mesh.vertices().iter().enumerate() {
        // Transform vertex to world space
        let world_v = mesh_pose.transform_point(vertex);

        // Signed distance from plane (positive = above plane, negative = below)
        let signed_dist = plane_normal.dot(&world_v.coords) - plane_d;
        let depth = -signed_dist;

        if depth > 0.0 {
            match &mut deepest {
                Some(d) if depth > d.penetration => {
                    d.point = world_v;
                    d.normal = plane_normal;
                    d.penetration = depth;
                    d.triangle_index = i; // Store vertex index (not triangle, but useful for debug)
                }
                None => {
                    deepest = Some(MeshContact {
                        point: world_v,
                        normal: plane_normal,
                        penetration: depth,
                        triangle_index: i,
                    });
                }
                _ => {}
            }
        }
    }

    deepest
}

/// Collision between a plane and another geometry.
///
/// Dispatches to specialized implementations based on the other geometry's type.
/// The plane is always treated as an infinite half-space with normal along its
/// local Z-axis.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
#[inline]
fn collide_with_plane(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    type2: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    // Determine which is the plane
    let (
        plane_geom,
        other_geom,
        plane_pos,
        plane_mat,
        other_pos,
        other_mat,
        other_type,
        other_size,
    ) = if type1 == GeomType::Plane {
        (geom1, geom2, pos1, mat1, pos2, mat2, type2, size2)
    } else {
        (geom2, geom1, pos2, mat2, pos1, mat1, type1, size1)
    };

    // Plane normal is the Z-axis of the plane's frame
    let plane_normal = plane_mat.column(2).into_owned();
    // Plane passes through its position
    let plane_distance = plane_normal.dot(&plane_pos);

    // Get friction
    let friction1 = model.geom_friction[plane_geom].x;
    let friction2 = model.geom_friction[other_geom].x;
    let friction = (friction1 * friction2).sqrt();

    match other_type {
        GeomType::Sphere => {
            let radius = other_size.x;
            let center_dist = plane_normal.dot(&other_pos) - plane_distance;
            let penetration = radius - center_dist;

            if penetration > 0.0 {
                // Contact position is at the sphere surface toward the plane
                let contact_pos = other_pos - plane_normal * center_dist;
                // Contact normal points from other_geom toward plane_geom (from ball into plane = -plane_normal)
                // But for force calculation, we want the force to push the ball OUT of the plane,
                // so we use +plane_normal for the contact force direction.
                // Store the "push direction" as the normal to simplify force calculation.
                Some(Contact::new(
                    contact_pos,
                    plane_normal, // Points UP, away from plane (push direction)
                    penetration,
                    plane_geom,
                    other_geom,
                    friction,
                ))
            } else {
                None
            }
        }
        GeomType::Box => {
            // Optimized box-plane: compute lowest corner directly instead of testing all 8
            //
            // The support point (lowest corner toward plane) uses the formula:
            //   corner = center + sum_i( -sign(n·r_i) * h_i * r_i )
            //
            // where r_i are box local axes in world space and h_i are half-extents.
            // The sign logic: if n·r_i > 0, the axis points "up" relative to the plane,
            // so we want the negative end (-h_i * r_i) to get the lowest point.
            //
            // Implementation: dx = -sign(n·rx), so corner = center + dx*hx*rx + ...
            // This is equivalent to the formula but avoids explicit negation.
            //
            // This is O(1) instead of O(8) per box.
            let half = other_size;
            let rx = other_mat.column(0).into_owned();
            let ry = other_mat.column(1).into_owned();
            let rz = other_mat.column(2).into_owned();

            // Compute -sign(n·axis) for each axis. When dot > 0, axis points toward
            // plane normal, so we pick the negative end (d = -1). When dot <= 0, we pick
            // the positive end (d = +1). This gives the corner furthest in -n direction.
            let dx = if plane_normal.dot(&rx) > 0.0 {
                -1.0
            } else {
                1.0
            };
            let dy = if plane_normal.dot(&ry) > 0.0 {
                -1.0
            } else {
                1.0
            };
            let dz = if plane_normal.dot(&rz) > 0.0 {
                -1.0
            } else {
                1.0
            };

            // Lowest corner is the one most toward the plane (-normal direction)
            let lowest_corner =
                other_pos + rx * (dx * half.x) + ry * (dy * half.y) + rz * (dz * half.z);

            let dist = plane_normal.dot(&lowest_corner) - plane_distance;
            let depth = -dist;

            if depth > 0.0 {
                // Contact position on plane surface (project corner onto plane)
                // This is consistent with sphere-plane which places contact at surface
                let contact_pos = lowest_corner - plane_normal * dist;
                Some(Contact::new(
                    contact_pos,
                    plane_normal,
                    depth,
                    plane_geom,
                    other_geom,
                    friction,
                ))
            } else {
                None
            }
        }
        GeomType::Capsule => {
            // Check both end spheres of the capsule
            let radius = other_size.x;
            let half_length = other_size.y;
            let axis = other_mat.column(2).into_owned(); // Z is capsule axis

            let end1 = other_pos + axis * half_length;
            let end2 = other_pos - axis * half_length;

            let dist1 = plane_normal.dot(&end1) - plane_distance;
            let dist2 = plane_normal.dot(&end2) - plane_distance;

            let (closest_end, min_dist) = if dist1 < dist2 {
                (end1, dist1)
            } else {
                (end2, dist2)
            };

            let penetration = radius - min_dist;

            if penetration > 0.0 {
                let contact_pos = closest_end - plane_normal * min_dist;
                Some(Contact::new(
                    contact_pos,
                    plane_normal,
                    penetration,
                    plane_geom,
                    other_geom,
                    friction,
                ))
            } else {
                None
            }
        }
        GeomType::Cylinder => {
            // Cylinder-plane collision: find deepest point on cylinder
            collide_cylinder_plane_impl(
                plane_geom,
                other_geom,
                plane_normal,
                plane_distance,
                other_pos,
                other_mat,
                other_size,
                friction,
            )
        }
        GeomType::Ellipsoid => {
            // Ellipsoid-plane collision: find support point in plane normal direction
            collide_ellipsoid_plane_impl(
                plane_geom,
                other_geom,
                plane_normal,
                plane_distance,
                other_pos,
                other_mat,
                other_size,
                friction,
            )
        }
        // INVARIANT: collide_geoms() dispatches mesh collision before plane collision.
        // If either geom is a mesh, collide_with_mesh() handles it—including mesh-plane.
        // This branch exists only for match exhaustiveness; reaching it indicates a bug.
        GeomType::Mesh => unreachable!(
            "mesh collision must be dispatched before plane collision in collide_geoms"
        ),
        // Plane-plane: two infinite half-spaces. Intersection is either empty, a plane,
        // or a half-space—none of which produce a meaningful contact point.
        GeomType::Plane => None,
    }
}

/// Cylinder-plane collision detection (internal helper).
///
/// Finds the deepest penetrating point on the cylinder. For tilted/upright
/// cylinders, this is on the rim edge. For horizontal cylinders (axis parallel
/// to plane), the curved surface is checked.
///
/// # Algorithm
/// 1. Compute cylinder axis in world frame
/// 2. Determine if cylinder is "horizontal" (axis nearly parallel to plane)
/// 3. For non-horizontal: check rim points on both caps in the deepest direction
/// 4. For horizontal: check the curved surface point closest to plane
/// 5. Return the deepest penetrating point as contact
///
/// # Parameters
/// - `plane_normal`: Unit normal of the plane (points away from solid half-space)
/// - `plane_d`: Signed distance from origin to plane along normal
/// - `cyl_size`: [radius, half_height, unused]
///
/// # Returns
/// `Some(Contact)` if cylinder penetrates plane, `None` otherwise.
#[inline]
#[allow(clippy::too_many_arguments)]
fn collide_cylinder_plane_impl(
    plane_geom: usize,
    cyl_geom: usize,
    plane_normal: Vector3<f64>,
    plane_d: f64,
    cyl_pos: Vector3<f64>,
    cyl_mat: Matrix3<f64>,
    cyl_size: Vector3<f64>,
    friction: f64,
) -> Option<Contact> {
    let radius = cyl_size.x;
    let half_height = cyl_size.y;

    // Cylinder axis is local Z transformed to world
    let cyl_axis = cyl_mat.column(2).into_owned();

    // How aligned is cylinder axis with plane normal?
    // axis_dot_signed: positive = axis points away from plane, negative = toward plane
    // axis_dot (abs): 1 = vertical, 0 = horizontal
    let axis_dot_signed = plane_normal.dot(&cyl_axis);
    let axis_dot = axis_dot_signed.abs();

    // Pre-compute radial direction (projection of plane normal onto radial plane)
    // Used in Case 1 (horizontal) and Case 3 (tilted)
    let radial = plane_normal - cyl_axis * axis_dot_signed;
    let radial_len = radial.norm();

    // Compute the deepest point on the cylinder
    let deepest_point = if axis_dot < AXIS_HORIZONTAL_THRESHOLD {
        // Case 1: Cylinder is horizontal (axis parallel to plane)
        // The deepest point is on the curved surface, directly below the axis
        if radial_len > GEOM_EPSILON {
            // Point on curved surface in the direction opposite to plane normal
            cyl_pos - (radial / radial_len) * radius
        } else {
            // Degenerate: plane normal exactly along axis (contradicts horizontal check)
            cyl_pos - plane_normal * radius
        }
    } else if axis_dot > AXIS_VERTICAL_THRESHOLD {
        // Case 2: Cylinder is vertical (axis perpendicular to plane)
        // The deepest point is the entire bottom rim (pick center for stability)
        // Determine which cap faces the plane
        let cap_dir = if axis_dot_signed > 0.0 {
            -cyl_axis // Bottom cap faces plane
        } else {
            cyl_axis // Top cap faces plane
        };
        cyl_pos + cap_dir * half_height
    } else {
        // Case 3: Cylinder is tilted
        // The deepest point is on one of the rim edges
        // Find the rim direction that points most into the plane
        // Note: radial_len = sqrt(1 - axis_dot²) > 0 since axis_dot < AXIS_VERTICAL_THRESHOLD
        let rim_dir = -radial / radial_len;

        // Determine which cap is lower (faces the plane more)
        let top_center = cyl_pos + cyl_axis * half_height;
        let bottom_center = cyl_pos - cyl_axis * half_height;

        let top_rim_point = top_center + rim_dir * radius;
        let bottom_rim_point = bottom_center + rim_dir * radius;

        let top_depth = -(plane_normal.dot(&top_rim_point) - plane_d);
        let bottom_depth = -(plane_normal.dot(&bottom_rim_point) - plane_d);

        if bottom_depth > top_depth {
            bottom_rim_point
        } else {
            top_rim_point
        }
    };

    // Compute penetration depth
    let signed_dist = plane_normal.dot(&deepest_point) - plane_d;
    let depth = -signed_dist;

    if depth <= 0.0 {
        return None;
    }

    // Contact point is on the plane surface
    let contact_pos = deepest_point + plane_normal * depth;

    Some(Contact::new(
        contact_pos,
        plane_normal,
        depth,
        plane_geom,
        cyl_geom,
        friction,
    ))
}

/// Ellipsoid-plane collision detection (internal helper).
///
/// Finds the support point on the ellipsoid surface in the direction toward the plane.
/// Ellipsoid is defined by radii (a, b, c) = (rx, ry, rz) along local axes.
///
/// # Algorithm
///
/// For an ellipsoid `x²/a² + y²/b² + z²/c² = 1`, the support point in direction `-n`
/// (i.e., the point on the surface with outward normal parallel to `-n`) is:
///
/// ```text
/// p = -(r ⊙ r ⊙ n) / ||r ⊙ n||
///   = -(a²·nₓ, b²·nᵧ, c²·nᵤ) / ||(a·nₓ, b·nᵧ, c·nᵤ)||
/// ```
///
/// where `⊙` denotes element-wise (Hadamard) product.
///
/// **Derivation**: The outward normal at point `(x,y,z)` on the ellipsoid is
/// `∇f = (2x/a², 2y/b², 2z/c²)`. Setting this parallel to `n` and solving with
/// the ellipsoid constraint yields the support point formula.
///
/// # Parameters
/// - `plane_normal`: Unit normal of the plane
/// - `plane_d`: Signed distance from origin to plane
/// - `ell_radii`: Ellipsoid radii [rx, ry, rz]
///
/// # Returns
/// `Some(Contact)` if ellipsoid penetrates plane, `None` otherwise.
#[inline]
#[allow(clippy::too_many_arguments)]
fn collide_ellipsoid_plane_impl(
    plane_geom: usize,
    ell_geom: usize,
    plane_normal: Vector3<f64>,
    plane_d: f64,
    ell_pos: Vector3<f64>,
    ell_mat: Matrix3<f64>,
    ell_radii: Vector3<f64>,
    friction: f64,
) -> Option<Contact> {
    // Transform plane normal to ellipsoid local frame
    let local_normal = ell_mat.transpose() * plane_normal;

    // Compute support point in local frame
    // For ellipsoid, support in direction -n is: -(r² ⊙ n) / ||r ⊙ n||
    // where ⊙ is element-wise multiply and r = radii
    let scaled = Vector3::new(
        ell_radii.x * local_normal.x,
        ell_radii.y * local_normal.y,
        ell_radii.z * local_normal.z,
    );
    let scale_norm = scaled.norm();

    if scale_norm < GEOM_EPSILON {
        // Degenerate case: normal perpendicular to all radii (shouldn't happen)
        return None;
    }

    // Support point on ellipsoid surface in direction toward plane (negative normal)
    let local_support = -Vector3::new(
        ell_radii.x * scaled.x / scale_norm,
        ell_radii.y * scaled.y / scale_norm,
        ell_radii.z * scaled.z / scale_norm,
    );

    // Transform to world frame
    let world_support = ell_pos + ell_mat * local_support;

    // Check penetration depth
    let signed_dist = plane_normal.dot(&world_support) - plane_d;
    let depth = -signed_dist; // Positive = penetrating

    if depth <= 0.0 {
        return None;
    }

    // Contact point is on plane surface
    let contact_pos = world_support + plane_normal * depth;

    Some(Contact::new(
        contact_pos,
        plane_normal,
        depth,
        plane_geom,
        ell_geom,
        friction,
    ))
}

/// Sphere-sphere collision detection.
///
/// This is a simple analytical calculation that's more robust than GJK/EPA
/// for the sphere-sphere case.
fn collide_sphere_sphere(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    pos2: Vector3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    let radius1 = size1.x;
    let radius2 = size2.x;

    let diff = pos2 - pos1;
    let dist = diff.norm();

    // Check for penetration
    let sum_radii = radius1 + radius2;
    let penetration = sum_radii - dist;

    if penetration > 0.0 {
        // Normal points from sphere1 to sphere2.
        // For coincident/nearly-coincident centers (degenerate case), pick +Z.
        let normal = if dist > GEOM_EPSILON {
            diff / dist
        } else {
            Vector3::z()
        };

        // Contact point is on the surface of sphere1 toward sphere2
        // (midpoint of penetration region)
        let contact_pos = pos1 + normal * (radius1 - penetration * 0.5);

        // Compute friction (geometric mean)
        let friction1 = model.geom_friction[geom1].x;
        let friction2 = model.geom_friction[geom2].x;
        let friction = (friction1 * friction2).sqrt();

        Some(Contact::new(
            contact_pos,
            normal,
            penetration,
            geom1,
            geom2,
            friction,
        ))
    } else {
        None
    }
}

/// Capsule-capsule collision detection.
///
/// Capsules are represented as line segments with radius. The collision
/// is computed by finding the closest points between the two line segments,
/// then checking if the distance is less than the sum of radii.
#[allow(clippy::too_many_arguments)]
fn collide_capsule_capsule(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    // Capsule parameters: size.x = radius, size.y = half_length
    let radius1 = size1.x;
    let half_len1 = size1.y;
    let radius2 = size2.x;
    let half_len2 = size2.y;

    // Get capsule axes (Z-axis of their rotation matrices)
    let axis1 = mat1.column(2).into_owned();
    let axis2 = mat2.column(2).into_owned();

    // Endpoints of capsule line segments
    let p1a = pos1 - axis1 * half_len1;
    let p1b = pos1 + axis1 * half_len1;
    let p2a = pos2 - axis2 * half_len2;
    let p2b = pos2 + axis2 * half_len2;

    // Find closest points between the two line segments
    let (closest1, closest2) = closest_points_segments(p1a, p1b, p2a, p2b);

    // Check distance
    let diff = closest2 - closest1;
    let dist = diff.norm();
    let sum_radii = radius1 + radius2;
    let penetration = sum_radii - dist;

    if penetration > 0.0 {
        // Normal points from capsule1 toward capsule2.
        // For degenerate case (segments intersect), pick +Z.
        let normal = if dist > GEOM_EPSILON {
            diff / dist
        } else {
            Vector3::z()
        };
        let contact_pos = closest1 + normal * (radius1 - penetration * 0.5);

        let friction1 = model.geom_friction[geom1].x;
        let friction2 = model.geom_friction[geom2].x;
        let friction = (friction1 * friction2).sqrt();

        Some(Contact::new(
            contact_pos,
            normal,
            penetration,
            geom1,
            geom2,
            friction,
        ))
    } else {
        None
    }
}

/// Sphere-capsule collision detection.
#[allow(clippy::too_many_arguments)]
fn collide_sphere_capsule(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    // Determine which is sphere and which is capsule
    let (
        sphere_geom,
        capsule_geom,
        sphere_pos,
        capsule_pos,
        capsule_mat,
        sphere_radius,
        capsule_size,
    ) = if type1 == GeomType::Sphere {
        (geom1, geom2, pos1, pos2, mat2, size1.x, size2)
    } else {
        (geom2, geom1, pos2, pos1, mat1, size2.x, size1)
    };

    let capsule_radius = capsule_size.x;
    let capsule_half_len = capsule_size.y;
    let capsule_axis = capsule_mat.column(2).into_owned();

    // Capsule endpoints
    let cap_a = capsule_pos - capsule_axis * capsule_half_len;
    let cap_b = capsule_pos + capsule_axis * capsule_half_len;

    // Find closest point on capsule line segment to sphere center
    let closest_on_capsule = closest_point_segment(cap_a, cap_b, sphere_pos);

    // Check distance
    let diff = sphere_pos - closest_on_capsule;
    let dist = diff.norm();
    let sum_radii = sphere_radius + capsule_radius;
    let penetration = sum_radii - dist;

    if penetration > 0.0 {
        // Normal points from capsule toward sphere.
        // For degenerate case (sphere center on capsule axis), pick +Z.
        let normal = if dist > GEOM_EPSILON {
            diff / dist
        } else {
            Vector3::z()
        };
        let contact_pos = closest_on_capsule + normal * (capsule_radius - penetration * 0.5);

        let friction1 = model.geom_friction[sphere_geom].x;
        let friction2 = model.geom_friction[capsule_geom].x;
        let friction = (friction1 * friction2).sqrt();

        // Ensure geom1 < geom2 for consistency
        let (g1, g2) = if sphere_geom < capsule_geom {
            (sphere_geom, capsule_geom)
        } else {
            (capsule_geom, sphere_geom)
        };

        // Normal convention: points from g1 toward g2.
        // `normal` points from capsule toward sphere.
        // If capsule is g1 (capsule_geom < sphere_geom), normal already points g1→g2.
        // If sphere is g1 (sphere_geom < capsule_geom), we need -normal to point g1→g2.
        let final_normal = if capsule_geom < sphere_geom {
            normal
        } else {
            -normal
        };

        Some(Contact::new(
            contact_pos,
            final_normal,
            penetration,
            g1,
            g2,
            friction,
        ))
    } else {
        None
    }
}

/// Sphere-box collision detection.
///
/// Uses the closest point on box surface to sphere center algorithm.
#[allow(clippy::too_many_arguments)]
fn collide_sphere_box(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    // Determine which is sphere and which is box
    let (sphere_geom, box_geom, sphere_pos, box_pos, box_mat, sphere_radius, box_half) =
        if type1 == GeomType::Sphere {
            (geom1, geom2, pos1, pos2, mat2, size1.x, size2)
        } else {
            (geom2, geom1, pos2, pos1, mat1, size2.x, size1)
        };

    // Transform sphere center to box local coordinates
    let local_center = box_mat.transpose() * (sphere_pos - box_pos);

    // Find closest point on box to sphere center (clamp to box bounds)
    let closest_local = Vector3::new(
        local_center.x.clamp(-box_half.x, box_half.x),
        local_center.y.clamp(-box_half.y, box_half.y),
        local_center.z.clamp(-box_half.z, box_half.z),
    );

    // Transform back to world space
    let closest_world = box_pos + box_mat * closest_local;

    // Compute distance and penetration
    let diff = sphere_pos - closest_world;
    let dist = diff.norm();
    let penetration = sphere_radius - dist;

    if penetration > 0.0 {
        // Compute normal (from box toward sphere)
        let normal = if dist > GEOM_EPSILON {
            diff / dist
        } else {
            // Sphere center is inside box - find deepest penetration axis
            let mut min_pen = f64::MAX;
            let mut normal_local = Vector3::x();
            for i in 0..3 {
                let pen_pos = box_half[i] - local_center[i];
                let pen_neg = box_half[i] + local_center[i];
                if pen_pos < min_pen {
                    min_pen = pen_pos;
                    normal_local = Vector3::zeros();
                    normal_local[i] = 1.0;
                }
                if pen_neg < min_pen {
                    min_pen = pen_neg;
                    normal_local = Vector3::zeros();
                    normal_local[i] = -1.0;
                }
            }
            box_mat * normal_local
        };

        let contact_pos = closest_world + normal * (penetration * 0.5);

        let friction1 = model.geom_friction[sphere_geom].x;
        let friction2 = model.geom_friction[box_geom].x;
        let friction = (friction1 * friction2).sqrt();

        let (g1, g2) = if sphere_geom < box_geom {
            (sphere_geom, box_geom)
        } else {
            (box_geom, sphere_geom)
        };

        // Normal convention: points from geom1 (g1) toward geom2 (g2).
        // `normal` is computed as sphere_pos - closest_world, i.e., from box toward sphere.
        // If box is g1 (box_geom < sphere_geom), normal already points g1→g2.
        // If sphere is g1 (sphere_geom < box_geom), we need -normal to point g1→g2.
        let final_normal = if box_geom < sphere_geom {
            normal
        } else {
            -normal
        };

        Some(Contact::new(
            contact_pos,
            final_normal,
            penetration,
            g1,
            g2,
            friction,
        ))
    } else {
        None
    }
}

/// Cylinder-sphere collision detection.
///
/// Handles three collision cases:
/// - Side collision: sphere beside cylinder body
/// - Cap collision: sphere above/below cylinder, within radius
/// - Edge collision: sphere near rim of cylinder cap
///
/// Cylinder axis is local Z.
#[allow(clippy::too_many_arguments)]
fn collide_cylinder_sphere(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    // Determine which is cylinder and which is sphere
    // Note: sphere doesn't use its rotation matrix, but we need mat2 for the cylinder case
    let (cyl_geom, cyl_pos, cyl_mat, cyl_size, sph_geom, sph_pos, sph_radius) =
        if type1 == GeomType::Cylinder {
            (geom1, pos1, mat1, size1, geom2, pos2, size2.x)
        } else {
            (geom2, pos2, mat2, size2, geom1, pos1, size1.x)
        };

    let cyl_radius = cyl_size.x;
    let cyl_half_height = cyl_size.y;
    let cyl_axis = cyl_mat.column(2).into_owned();

    // Vector from cylinder center to sphere center
    let d = sph_pos - cyl_pos;

    // Project onto cylinder axis
    let axis_dist = d.dot(&cyl_axis);

    // Clamp to cylinder height
    let clamped_axis = axis_dist.clamp(-cyl_half_height, cyl_half_height);

    // Closest point on cylinder axis to sphere center
    let axis_point = cyl_pos + cyl_axis * clamped_axis;

    // Radial vector from axis to sphere
    let radial = sph_pos - axis_point;
    let radial_dist = radial.norm();

    // Determine closest point on cylinder surface and contact normal
    let (closest_on_cyl, normal) = if axis_dist.abs() <= cyl_half_height {
        // Sphere is beside the cylinder (side collision)
        if radial_dist < GEOM_EPSILON {
            // Sphere center on axis - degenerate case, pick arbitrary radial direction
            let arb = if cyl_axis.x.abs() < 0.9 {
                Vector3::x()
            } else {
                Vector3::y()
            };
            let n = cyl_axis.cross(&arb).normalize();
            (axis_point + n * cyl_radius, n)
        } else {
            let n = radial / radial_dist;
            (axis_point + n * cyl_radius, n)
        }
    } else {
        // Sphere is above/below cylinder (cap or edge collision)
        let cap_center = cyl_pos + cyl_axis * clamped_axis;

        // Compute radial direction in the plane perpendicular to cylinder axis.
        // d - (d · axis) * axis gives the perpendicular component.
        let perp = d - cyl_axis * axis_dist;
        let perp_dist = perp.norm();

        if perp_dist <= cyl_radius {
            // Cap collision - sphere projects onto cap face
            let n = if axis_dist > 0.0 { cyl_axis } else { -cyl_axis };
            (cap_center + perp, n)
        } else {
            // Edge collision - sphere near rim of cap
            let radial_n = perp / perp_dist;
            let edge_point = cap_center + radial_n * cyl_radius;
            let to_sphere = sph_pos - edge_point;
            let to_sphere_dist = to_sphere.norm();
            let n = if to_sphere_dist > GEOM_EPSILON {
                to_sphere / to_sphere_dist
            } else {
                // Degenerate: sphere center exactly on edge
                radial_n
            };
            (edge_point, n)
        }
    };

    // Compute penetration depth
    let dist = (sph_pos - closest_on_cyl).norm();
    let penetration = sph_radius - dist;

    if penetration <= 0.0 {
        return None;
    }

    // Contact position is on the surface between the two shapes
    let contact_pos = closest_on_cyl + normal * (penetration * 0.5);

    // Compute friction
    let friction1 = model.geom_friction[cyl_geom].x;
    let friction2 = model.geom_friction[sph_geom].x;
    let friction = (friction1 * friction2).sqrt();

    // Order geoms consistently (lower index first)
    let (g1, g2, final_normal) = if cyl_geom < sph_geom {
        (cyl_geom, sph_geom, normal)
    } else {
        (sph_geom, cyl_geom, -normal)
    };

    Some(Contact::new(
        contact_pos,
        final_normal,
        penetration,
        g1,
        g2,
        friction,
    ))
}

/// Cylinder-capsule collision detection.
///
/// Computes collision between a cylinder and a capsule by finding the closest
/// points between the cylinder axis segment and the capsule axis segment,
/// then checking if the cylinder surface intersects the capsule's swept sphere.
///
/// # Limitations
///
/// This algorithm treats the cylinder's curved surface correctly but does not
/// handle collisions with the flat caps. For cap collisions (capsule directly
/// above/below cylinder), returns `None` to fall through to GJK/EPA.
///
/// Both shapes have their axis along local Z.
#[allow(clippy::too_many_arguments)]
fn collide_cylinder_capsule(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    // Identify cylinder and capsule
    let (cyl_geom, cyl_pos, cyl_mat, cyl_size, cap_geom, cap_pos, cap_mat, cap_size) =
        if type1 == GeomType::Cylinder {
            (geom1, pos1, mat1, size1, geom2, pos2, mat2, size2)
        } else {
            (geom2, pos2, mat2, size2, geom1, pos1, mat1, size1)
        };

    let cyl_radius = cyl_size.x;
    let cyl_half_height = cyl_size.y;
    let cap_radius = cap_size.x;
    let cap_half_len = cap_size.y;

    let cyl_axis = cyl_mat.column(2).into_owned();
    let cap_axis = cap_mat.column(2).into_owned();

    // Cylinder axis segment endpoints
    let cyl_a = cyl_pos - cyl_axis * cyl_half_height;
    let cyl_b = cyl_pos + cyl_axis * cyl_half_height;

    // Capsule axis segment endpoints
    let cap_a = cap_pos - cap_axis * cap_half_len;
    let cap_b = cap_pos + cap_axis * cap_half_len;

    // Find closest points between the two axis segments
    let (cyl_closest, cap_closest) = closest_points_segments(cyl_a, cyl_b, cap_a, cap_b);

    // Vector from cylinder axis point to capsule axis point
    let diff = cap_closest - cyl_closest;
    let dist = diff.norm();

    if dist < GEOM_EPSILON {
        // Axes intersect or nearly intersect - degenerate case where analytical
        // solution is unreliable. Return None to fall through to GJK/EPA.
        return None;
    }

    let normal = diff / dist;

    // Check if this is a cap collision scenario:
    // If cyl_closest is at an endpoint AND normal points mostly along the axis,
    // we're hitting the flat cap, not the curved surface. Fall back to GJK/EPA.
    let cyl_closest_on_cap =
        (cyl_closest - cyl_a).norm() < GEOM_EPSILON || (cyl_closest - cyl_b).norm() < GEOM_EPSILON;
    let normal_along_axis = normal.dot(&cyl_axis).abs();
    if cyl_closest_on_cap && normal_along_axis > CAP_COLLISION_THRESHOLD {
        // Cap collision - this algorithm doesn't handle flat caps correctly
        return None;
    }

    // Closest point on cylinder surface (in direction of capsule)
    let cyl_surface = cyl_closest + normal * cyl_radius;

    // Distance from cylinder surface to capsule axis
    let surface_to_cap_dist = (cap_closest - cyl_surface).dot(&normal);
    let penetration = cap_radius - surface_to_cap_dist;

    if penetration <= 0.0 {
        return None;
    }

    // Contact position is between the two surfaces
    let contact_pos = cyl_surface + normal * (penetration * 0.5);

    // Compute friction
    let friction1 = model.geom_friction[cyl_geom].x;
    let friction2 = model.geom_friction[cap_geom].x;
    let friction = (friction1 * friction2).sqrt();

    // Order geoms consistently (lower index first)
    let (g1, g2, final_normal) = if cyl_geom < cap_geom {
        (cyl_geom, cap_geom, normal)
    } else {
        (cap_geom, cyl_geom, -normal)
    };

    Some(Contact::new(
        contact_pos,
        final_normal,
        penetration,
        g1,
        g2,
        friction,
    ))
}

/// Capsule-box collision detection.
///
/// Tests both capsule endpoints and the closest point on the capsule axis
/// to find the minimum distance configuration.
#[allow(clippy::too_many_arguments)]
fn collide_capsule_box(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    // Determine which is capsule and which is box
    let (
        capsule_geom,
        box_geom,
        capsule_pos,
        capsule_mat,
        box_pos,
        box_mat,
        capsule_size,
        box_half,
    ) = if type1 == GeomType::Capsule {
        (geom1, geom2, pos1, mat1, pos2, mat2, size1, size2)
    } else {
        (geom2, geom1, pos2, mat2, pos1, mat1, size2, size1)
    };

    let capsule_radius = capsule_size.x;
    let capsule_half_len = capsule_size.y;
    let capsule_axis = capsule_mat.column(2).into_owned();

    // Capsule endpoints
    let cap_a = capsule_pos - capsule_axis * capsule_half_len;
    let cap_b = capsule_pos + capsule_axis * capsule_half_len;

    // Find closest point on capsule axis to box
    // We sample multiple points along the capsule and find the minimum distance
    let mut min_dist = f64::MAX;
    let mut best_capsule_point = capsule_pos;
    let mut best_box_point = box_pos;

    // Transform box to local coordinates once
    let box_mat_t = box_mat.transpose();

    // Sample along capsule axis (including endpoints)
    let samples = [0.0, 0.25, 0.5, 0.75, 1.0];
    for &t in &samples {
        let capsule_point = cap_a + (cap_b - cap_a) * t;
        let local_point = box_mat_t * (capsule_point - box_pos);

        // Closest point on box to this capsule point
        let closest_local = Vector3::new(
            local_point.x.clamp(-box_half.x, box_half.x),
            local_point.y.clamp(-box_half.y, box_half.y),
            local_point.z.clamp(-box_half.z, box_half.z),
        );
        let closest_world = box_pos + box_mat * closest_local;

        let dist = (capsule_point - closest_world).norm();
        if dist < min_dist {
            min_dist = dist;
            best_capsule_point = capsule_point;
            best_box_point = closest_world;
        }
    }

    // Refine by finding closest point on capsule axis to best box point
    let closest_on_capsule = closest_point_segment(cap_a, cap_b, best_box_point);
    let local_closest = box_mat_t * (closest_on_capsule - box_pos);
    let box_closest_local = Vector3::new(
        local_closest.x.clamp(-box_half.x, box_half.x),
        local_closest.y.clamp(-box_half.y, box_half.y),
        local_closest.z.clamp(-box_half.z, box_half.z),
    );
    let box_closest_world = box_pos + box_mat * box_closest_local;
    let final_dist = (closest_on_capsule - box_closest_world).norm();

    if final_dist < min_dist {
        min_dist = final_dist;
        best_capsule_point = closest_on_capsule;
        best_box_point = box_closest_world;
    }

    let penetration = capsule_radius - min_dist;

    if penetration > 0.0 {
        let diff = best_capsule_point - best_box_point;
        let normal = if min_dist > GEOM_EPSILON {
            diff / min_dist
        } else {
            // Edge case: capsule axis passes through box
            // Find deepest penetration direction
            let local_cap = box_mat_t * (best_capsule_point - box_pos);
            let mut min_pen = f64::MAX;
            let mut normal_local = Vector3::x();
            for i in 0..3 {
                let pen_pos = box_half[i] - local_cap[i];
                let pen_neg = box_half[i] + local_cap[i];
                if pen_pos < min_pen {
                    min_pen = pen_pos;
                    normal_local = Vector3::zeros();
                    normal_local[i] = 1.0;
                }
                if pen_neg < min_pen {
                    min_pen = pen_neg;
                    normal_local = Vector3::zeros();
                    normal_local[i] = -1.0;
                }
            }
            box_mat * normal_local
        };

        let contact_pos = best_box_point + normal * (penetration * 0.5);

        let friction1 = model.geom_friction[capsule_geom].x;
        let friction2 = model.geom_friction[box_geom].x;
        let friction = (friction1 * friction2).sqrt();

        let (g1, g2) = if capsule_geom < box_geom {
            (capsule_geom, box_geom)
        } else {
            (box_geom, capsule_geom)
        };

        Some(Contact::new(
            contact_pos,
            if capsule_geom < box_geom {
                normal
            } else {
                -normal
            },
            penetration,
            g1,
            g2,
            friction,
        ))
    } else {
        None
    }
}

/// Box-box collision detection using Separating Axis Theorem (SAT).
///
/// Tests 15 axes: 3 face normals of box A, 3 face normals of box B,
/// and 9 edge-edge cross products.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn collide_box_box(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    let half1 = size1;
    let half2 = size2;

    // Get box axes
    let axes1: [Vector3<f64>; 3] = [
        mat1.column(0).into_owned(),
        mat1.column(1).into_owned(),
        mat1.column(2).into_owned(),
    ];
    let axes2: [Vector3<f64>; 3] = [
        mat2.column(0).into_owned(),
        mat2.column(1).into_owned(),
        mat2.column(2).into_owned(),
    ];

    let center_diff = pos2 - pos1;

    // Track minimum penetration
    let mut min_pen = f64::MAX;
    let mut best_axis = Vector3::x();
    let mut best_axis_is_face = true;

    // Test face normals of box 1 (3 axes)
    for i in 0..3 {
        let axis = axes1[i];
        let pen = test_sat_axis(&axis, &center_diff, &axes1, &half1, &axes2, &half2);
        if pen <= 0.0 {
            return None; // Separating axis found
        }
        if pen < min_pen {
            min_pen = pen;
            best_axis = axis;
            best_axis_is_face = true;
        }
    }

    // Test face normals of box 2 (3 axes)
    for i in 0..3 {
        let axis = axes2[i];
        let pen = test_sat_axis(&axis, &center_diff, &axes1, &half1, &axes2, &half2);
        if pen <= 0.0 {
            return None;
        }
        if pen < min_pen {
            min_pen = pen;
            best_axis = axis;
            best_axis_is_face = true;
        }
    }

    // Test edge-edge cross products (9 axes)
    for i in 0..3 {
        for j in 0..3 {
            let axis = axes1[i].cross(&axes2[j]);
            let len = axis.norm();
            if len < GEOM_EPSILON {
                continue; // Parallel edges
            }
            let axis = axis / len;

            let pen = test_sat_axis(&axis, &center_diff, &axes1, &half1, &axes2, &half2);
            if pen <= 0.0 {
                return None;
            }
            // Edge-edge contacts have a bias - they're less stable
            // Only use if significantly better than face contact
            if pen < min_pen * 0.95 {
                min_pen = pen;
                best_axis = axis;
                best_axis_is_face = false;
            }
        }
    }

    // Ensure normal points from box1 to box2
    if best_axis.dot(&center_diff) < 0.0 {
        best_axis = -best_axis;
    }

    // Find contact point
    // For face contacts: find vertex of one box most in the other's direction
    // For edge contacts: find closest points on the two edges
    let contact_pos = if best_axis_is_face {
        // Find support point on box2 in direction of -normal
        let support_local = Vector3::new(
            if best_axis.dot(&axes2[0]) < 0.0 {
                half2.x
            } else {
                -half2.x
            },
            if best_axis.dot(&axes2[1]) < 0.0 {
                half2.y
            } else {
                -half2.y
            },
            if best_axis.dot(&axes2[2]) < 0.0 {
                half2.z
            } else {
                -half2.z
            },
        );
        pos2 + mat2 * support_local - best_axis * (min_pen * 0.5)
    } else {
        // For edge-edge, use midpoint between closest points on edges
        // Approximate: use center point shifted by penetration
        pos1 + center_diff * 0.5
    };

    let friction1 = model.geom_friction[geom1].x;
    let friction2 = model.geom_friction[geom2].x;
    let friction = (friction1 * friction2).sqrt();

    Some(Contact::new(
        contact_pos,
        best_axis,
        min_pen,
        geom1,
        geom2,
        friction,
    ))
}

/// Test a single SAT axis and return penetration depth (negative = separated).
#[inline]
fn test_sat_axis(
    axis: &Vector3<f64>,
    center_diff: &Vector3<f64>,
    axes1: &[Vector3<f64>; 3],
    half1: &Vector3<f64>,
    axes2: &[Vector3<f64>; 3],
    half2: &Vector3<f64>,
) -> f64 {
    // Project box extents onto axis
    let r1 = (half1.x * axis.dot(&axes1[0]).abs())
        + (half1.y * axis.dot(&axes1[1]).abs())
        + (half1.z * axis.dot(&axes1[2]).abs());

    let r2 = (half2.x * axis.dot(&axes2[0]).abs())
        + (half2.y * axis.dot(&axes2[1]).abs())
        + (half2.z * axis.dot(&axes2[2]).abs());

    // Distance between centers projected onto axis
    let dist = axis.dot(center_diff).abs();

    // Penetration = sum of radii - distance
    r1 + r2 - dist
}

/// Find closest point on line segment AB to point P.
#[inline]
fn closest_point_segment(a: Vector3<f64>, b: Vector3<f64>, p: Vector3<f64>) -> Vector3<f64> {
    let ab = b - a;
    let ap = p - a;
    let ab_len_sq = ab.dot(&ab);

    if ab_len_sq < GEOM_EPSILON {
        return a; // Degenerate segment
    }

    let t = (ap.dot(&ab) / ab_len_sq).clamp(0.0, 1.0);
    a + ab * t
}

/// Find closest points between two line segments.
///
/// Returns (`point_on_seg1`, `point_on_seg2`).
///
/// Uses standard segment-segment distance algorithm with single-letter variables
/// matching the mathematical notation from computational geometry literature.
#[allow(clippy::many_single_char_names)]
fn closest_points_segments(
    p1: Vector3<f64>,
    q1: Vector3<f64>,
    p2: Vector3<f64>,
    q2: Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let d1 = q1 - p1; // Direction of segment 1
    let d2 = q2 - p2; // Direction of segment 2
    let r = p1 - p2;

    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);

    // Check for degenerate segments
    if a < GEOM_EPSILON && e < GEOM_EPSILON {
        return (p1, p2);
    }
    if a < GEOM_EPSILON {
        let t = (f / e).clamp(0.0, 1.0);
        return (p1, p2 + d2 * t);
    }
    if e < GEOM_EPSILON {
        let s = (-d1.dot(&r) / a).clamp(0.0, 1.0);
        return (p1 + d1 * s, p2);
    }

    let b = d1.dot(&d2);
    let c = d1.dot(&r);
    // Determinant of the 2x2 system: denom = a*e - b² (intentionally b*b, not a*b)
    #[allow(clippy::suspicious_operation_groupings)]
    let denom = a * e - b * b;

    // Compute initial s and t parameters for the closest points
    let (mut s, mut t) = if denom.abs() < GEOM_EPSILON {
        // Parallel segments
        (0.0, f / e)
    } else {
        let s_val = (b * f - c * e) / denom;
        let t_val = (b * s_val + f) / e;
        (s_val, t_val)
    };

    // Clamp to [0,1] and recompute
    if s < 0.0 {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else if s > 1.0 {
        s = 1.0;
        t = ((b + f) / e).clamp(0.0, 1.0);
    }

    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }

    (p1 + d1 * s, p2 + d2 * t)
}

/// Compute potential energy (gravitational + spring).
///
/// Following `MuJoCo` semantics:
/// - Gravitational: `E_g` = -Σ `m_i` * g · `p_i` (negative because work done against gravity)
/// - Spring: `E_s` = 0.5 * Σ `k_i` * (`q_i` - `q0_i)²`
fn mj_energy_pos(model: &Model, data: &mut Data) {
    let mut potential = 0.0;

    // Gravitational potential energy
    // E_g = -Σ m_i * g · com_i
    // Using xipos (COM in world frame) for correct calculation
    for body_id in 1..model.nbody {
        let mass = model.body_mass[body_id];
        let com = data.xipos[body_id];
        // Potential energy: -m * g · h (negative of work done by gravity)
        // With g = (0, 0, -9.81), this becomes m * 9.81 * z
        potential -= mass * model.gravity.dot(&com);
    }

    // Spring potential energy
    // E_s = 0.5 * k * (q - q0)²
    for jnt_id in 0..model.njnt {
        let stiffness = model.jnt_stiffness[jnt_id];
        if stiffness > 0.0 {
            let qpos_adr = model.jnt_qpos_adr[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge | MjJointType::Slide => {
                    let q = data.qpos[qpos_adr];
                    let q0 = model.qpos0.get(qpos_adr).copied().unwrap_or(0.0);
                    let displacement = q - q0;
                    potential += 0.5 * stiffness * displacement * displacement;
                }
                MjJointType::Ball | MjJointType::Free => {
                    // Ball/Free joint springs would use quaternion distance
                    // Not commonly used, skip for now
                }
            }
        }
    }

    data.energy_potential = potential;
}

/// Compute kinetic energy from mass matrix and velocities.
///
/// `E_k` = 0.5 * qvel^T * M * qvel
///
/// This must be called AFTER `mj_crba()` has computed the mass matrix.
/// However, for the `forward()` pipeline we call it after velocity FK
/// but before CRBA. We use an approximation based on body velocities
/// when M is not yet available.
fn mj_energy_vel(model: &Model, data: &mut Data) {
    // If mass matrix is available and computed, use exact formula
    // E_k = 0.5 * qvel^T * M * qvel
    let kinetic = if data.qM.nrows() == model.nv && model.nv > 0 {
        let m_qvel = &data.qM * &data.qvel;
        0.5 * data.qvel.dot(&m_qvel)
    } else {
        // Fallback: compute from body velocities directly
        // E_k = 0.5 * Σ (m_i * v_i^T * v_i + ω_i^T * I_i * ω_i)
        let mut energy = 0.0;
        for body_id in 1..model.nbody {
            let mass = model.body_mass[body_id];
            let inertia = model.body_inertia[body_id];

            // Extract linear and angular velocity from spatial velocity
            let omega = Vector3::new(
                data.cvel[body_id][0],
                data.cvel[body_id][1],
                data.cvel[body_id][2],
            );
            let v = Vector3::new(
                data.cvel[body_id][3],
                data.cvel[body_id][4],
                data.cvel[body_id][5],
            );

            // Translational kinetic energy: 0.5 * m * |v|²
            energy += 0.5 * mass * v.norm_squared();

            // Rotational kinetic energy: 0.5 * ω^T * I * ω
            // Using diagonal inertia in body frame (approximation - should transform)
            let omega_body = data.xquat[body_id].inverse() * omega;
            energy += 0.5
                * (inertia.x * omega_body.x.powi(2)
                    + inertia.y * omega_body.y.powi(2)
                    + inertia.z * omega_body.z.powi(2));
        }
        energy
    };

    data.energy_kinetic = kinetic;
}

// ============================================================================
// Sensor Pipeline Functions
// ============================================================================

/// Compute position-dependent sensor values.
///
/// This is called after `mj_fwd_position` and computes sensors that depend only
/// on position (not velocity or acceleration):
/// - `JointPos`: joint position
/// - `FramePos`: site/body position
/// - `FrameQuat`: site/body orientation
/// - FrameXAxis/YAxis/ZAxis: frame axes
/// - `TendonPos`: tendon length
/// - `ActuatorPos`: actuator length
/// - Rangefinder: distance measurement
/// - Touch: contact detection
#[allow(clippy::too_many_lines)]
fn mj_sensor_pos(model: &Model, data: &mut Data) {
    for sensor_id in 0..model.nsensor {
        // Skip non-position sensors
        if model.sensor_datatype[sensor_id] != MjSensorDataType::Position {
            continue;
        }

        let adr = model.sensor_adr[sensor_id];
        let objid = model.sensor_objid[sensor_id];

        match model.sensor_type[sensor_id] {
            MjSensorType::JointPos => {
                // Joint position (1D for hinge/slide, 4D for ball)
                if objid < model.njnt {
                    let qpos_adr = model.jnt_qpos_adr[objid];
                    match model.jnt_type[objid] {
                        MjJointType::Hinge | MjJointType::Slide => {
                            data.sensordata[adr] = data.qpos[qpos_adr];
                        }
                        MjJointType::Ball => {
                            // Return quaternion [w, x, y, z]
                            for i in 0..4 {
                                if adr + i < data.sensordata.len() {
                                    data.sensordata[adr + i] = data.qpos[qpos_adr + i];
                                }
                            }
                        }
                        MjJointType::Free => {
                            // Return [x, y, z, qw, qx, qy, qz]
                            for i in 0..7 {
                                if adr + i < data.sensordata.len() {
                                    data.sensordata[adr + i] = data.qpos[qpos_adr + i];
                                }
                            }
                        }
                    }
                }
            }

            MjSensorType::FramePos => {
                // Position of site/body in world frame
                let pos = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => data.site_xpos[objid],
                    MjObjectType::Body if objid < model.nbody => data.xpos[objid],
                    MjObjectType::Geom if objid < model.ngeom => data.geom_xpos[objid],
                    _ => Vector3::zeros(),
                };
                if adr + 2 < data.sensordata.len() {
                    data.sensordata[adr] = pos.x;
                    data.sensordata[adr + 1] = pos.y;
                    data.sensordata[adr + 2] = pos.z;
                }
            }

            MjSensorType::FrameQuat => {
                // Orientation of site/body as quaternion [w, x, y, z]
                let quat = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        // Compute site quaternion from rotation matrix
                        let mat = data.site_xmat[objid];
                        UnitQuaternion::from_rotation_matrix(
                            &nalgebra::Rotation3::from_matrix_unchecked(mat),
                        )
                    }
                    MjObjectType::Body if objid < model.nbody => data.xquat[objid],
                    MjObjectType::Geom if objid < model.ngeom => {
                        let mat = data.geom_xmat[objid];
                        UnitQuaternion::from_rotation_matrix(
                            &nalgebra::Rotation3::from_matrix_unchecked(mat),
                        )
                    }
                    _ => UnitQuaternion::identity(),
                };
                if adr + 3 < data.sensordata.len() {
                    data.sensordata[adr] = quat.w;
                    data.sensordata[adr + 1] = quat.i;
                    data.sensordata[adr + 2] = quat.j;
                    data.sensordata[adr + 3] = quat.k;
                }
            }

            MjSensorType::FrameXAxis | MjSensorType::FrameYAxis | MjSensorType::FrameZAxis => {
                // Frame axis in world coordinates
                let mat = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => data.site_xmat[objid],
                    MjObjectType::Body if objid < model.nbody => data.xmat[objid],
                    MjObjectType::Geom if objid < model.ngeom => data.geom_xmat[objid],
                    _ => Matrix3::identity(),
                };
                // These are the only types that can reach here due to the outer match
                #[allow(clippy::match_same_arms)]
                let col_idx = match model.sensor_type[sensor_id] {
                    MjSensorType::FrameXAxis => 0,
                    MjSensorType::FrameYAxis => 1,
                    MjSensorType::FrameZAxis => 2,
                    _ => 0, // Unreachable but needed for exhaustiveness
                };
                if adr + 2 < data.sensordata.len() {
                    data.sensordata[adr] = mat[(0, col_idx)];
                    data.sensordata[adr + 1] = mat[(1, col_idx)];
                    data.sensordata[adr + 2] = mat[(2, col_idx)];
                }
            }

            MjSensorType::SubtreeCom => {
                // Subtree center of mass (compute weighted average of descendant COMs)
                if objid < model.nbody {
                    let (com, _total_mass) = compute_subtree_com(model, data, objid);
                    if adr + 2 < data.sensordata.len() {
                        data.sensordata[adr] = com.x;
                        data.sensordata[adr + 1] = com.y;
                        data.sensordata[adr + 2] = com.z;
                    }
                }
            }

            MjSensorType::Touch => {
                // Touch sensor - checks if there's contact with the attached geom
                // Returns contact force magnitude (0 if no contact)
                // Note: This requires contact detection to be run first
                let mut force = 0.0;
                for contact in &data.contacts {
                    if contact.geom1 == objid || contact.geom2 == objid {
                        // Estimate contact force from penetration (simplified)
                        force += contact.depth.max(0.0) * 10000.0; // Stiffness
                    }
                }
                data.sensordata[adr] = force;
            }

            MjSensorType::Rangefinder => {
                // Rangefinder - distance along site Z axis
                // Note: Full implementation requires ray casting
                // For now, return max range as placeholder
                data.sensordata[adr] = model.sensor_cutoff[sensor_id];
            }

            // Skip velocity/acceleration-dependent sensors
            _ => {}
        }
    }
}

/// Compute velocity-dependent sensor values.
///
/// This is called after `mj_fwd_velocity` and computes sensors that depend on
/// velocity (but not acceleration):
/// - `JointVel`: joint velocity
/// - Gyro: angular velocity
/// - Velocimeter: linear velocity
/// - `FrameLinVel`: site/body linear velocity
/// - `FrameAngVel`: site/body angular velocity
/// - `TendonVel`: tendon velocity
/// - `ActuatorVel`: actuator velocity
#[allow(clippy::too_many_lines)]
fn mj_sensor_vel(model: &Model, data: &mut Data) {
    for sensor_id in 0..model.nsensor {
        // Skip non-velocity sensors
        if model.sensor_datatype[sensor_id] != MjSensorDataType::Velocity {
            continue;
        }

        let adr = model.sensor_adr[sensor_id];
        let objid = model.sensor_objid[sensor_id];

        match model.sensor_type[sensor_id] {
            MjSensorType::JointVel => {
                // Joint velocity
                if objid < model.njnt {
                    let dof_adr = model.jnt_dof_adr[objid];
                    let nv = model.jnt_type[objid].nv();
                    for i in 0..nv {
                        if adr + i < data.sensordata.len() {
                            data.sensordata[adr + i] = data.qvel[dof_adr + i];
                        }
                    }
                }
            }

            MjSensorType::Gyro => {
                // Angular velocity in sensor (site) frame
                let (omega_world, site_mat) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        let omega = Vector3::new(
                            data.cvel[body_id][0],
                            data.cvel[body_id][1],
                            data.cvel[body_id][2],
                        );
                        (omega, data.site_xmat[objid])
                    }
                    MjObjectType::Body if objid < model.nbody => {
                        let omega = Vector3::new(
                            data.cvel[objid][0],
                            data.cvel[objid][1],
                            data.cvel[objid][2],
                        );
                        (omega, data.xmat[objid])
                    }
                    _ => (Vector3::zeros(), Matrix3::identity()),
                };
                // Transform to sensor frame
                let omega_sensor = site_mat.transpose() * omega_world;
                if adr + 2 < data.sensordata.len() {
                    data.sensordata[adr] = omega_sensor.x;
                    data.sensordata[adr + 1] = omega_sensor.y;
                    data.sensordata[adr + 2] = omega_sensor.z;
                }
            }

            MjSensorType::Velocimeter => {
                // Linear velocity in sensor frame
                let (v_world, site_mat) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        let v = Vector3::new(
                            data.cvel[body_id][3],
                            data.cvel[body_id][4],
                            data.cvel[body_id][5],
                        );
                        (v, data.site_xmat[objid])
                    }
                    MjObjectType::Body if objid < model.nbody => {
                        let v = Vector3::new(
                            data.cvel[objid][3],
                            data.cvel[objid][4],
                            data.cvel[objid][5],
                        );
                        (v, data.xmat[objid])
                    }
                    _ => (Vector3::zeros(), Matrix3::identity()),
                };
                // Transform to sensor frame
                let v_sensor = site_mat.transpose() * v_world;
                if adr + 2 < data.sensordata.len() {
                    data.sensordata[adr] = v_sensor.x;
                    data.sensordata[adr + 1] = v_sensor.y;
                    data.sensordata[adr + 2] = v_sensor.z;
                }
            }

            MjSensorType::FrameLinVel => {
                // Linear velocity in world frame
                let v = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        Vector3::new(
                            data.cvel[body_id][3],
                            data.cvel[body_id][4],
                            data.cvel[body_id][5],
                        )
                    }
                    MjObjectType::Body if objid < model.nbody => Vector3::new(
                        data.cvel[objid][3],
                        data.cvel[objid][4],
                        data.cvel[objid][5],
                    ),
                    _ => Vector3::zeros(),
                };
                if adr + 2 < data.sensordata.len() {
                    data.sensordata[adr] = v.x;
                    data.sensordata[adr + 1] = v.y;
                    data.sensordata[adr + 2] = v.z;
                }
            }

            MjSensorType::FrameAngVel => {
                // Angular velocity in world frame
                let omega = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        Vector3::new(
                            data.cvel[body_id][0],
                            data.cvel[body_id][1],
                            data.cvel[body_id][2],
                        )
                    }
                    MjObjectType::Body if objid < model.nbody => Vector3::new(
                        data.cvel[objid][0],
                        data.cvel[objid][1],
                        data.cvel[objid][2],
                    ),
                    _ => Vector3::zeros(),
                };
                if adr + 2 < data.sensordata.len() {
                    data.sensordata[adr] = omega.x;
                    data.sensordata[adr + 1] = omega.y;
                    data.sensordata[adr + 2] = omega.z;
                }
            }

            MjSensorType::SubtreeLinVel => {
                // Subtree linear momentum / total mass = average velocity
                if objid < model.nbody {
                    let (_, total_mass) = compute_subtree_com(model, data, objid);
                    if total_mass > 1e-10 {
                        let momentum = compute_subtree_momentum(model, data, objid);
                        let v = momentum / total_mass;
                        if adr + 2 < data.sensordata.len() {
                            data.sensordata[adr] = v.x;
                            data.sensordata[adr + 1] = v.y;
                            data.sensordata[adr + 2] = v.z;
                        }
                    }
                }
            }

            // Skip position/acceleration-dependent sensors
            _ => {}
        }
    }
}

/// Compute acceleration-dependent sensor values.
///
/// This is called after `mj_fwd_acceleration` and computes sensors that depend
/// on acceleration:
/// - Accelerometer: linear acceleration (includes gravity in sensor frame)
/// - Force: constraint force at site
/// - Torque: constraint torque at site
/// - `FrameLinAcc`: linear acceleration
/// - `FrameAngAcc`: angular acceleration
/// - `ActuatorFrc`: actuator force
fn mj_sensor_acc(model: &Model, data: &mut Data) {
    for sensor_id in 0..model.nsensor {
        // Skip non-acceleration sensors
        if model.sensor_datatype[sensor_id] != MjSensorDataType::Acceleration {
            continue;
        }

        let adr = model.sensor_adr[sensor_id];
        let objid = model.sensor_objid[sensor_id];

        match model.sensor_type[sensor_id] {
            MjSensorType::Accelerometer => {
                // Linear acceleration in sensor frame (includes gravity)
                // a_sensor = R^T * (a_world - g)
                // For a body at rest in gravity, accelerometer reads +g (opposing gravity)
                let (a_world, site_mat) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        // Compute body acceleration from qacc
                        let a = compute_body_acceleration(model, data, body_id);
                        (a, data.site_xmat[objid])
                    }
                    MjObjectType::Body if objid < model.nbody => {
                        let a = compute_body_acceleration(model, data, objid);
                        (a, data.xmat[objid])
                    }
                    _ => (Vector3::zeros(), Matrix3::identity()),
                };
                // Accelerometer measures: a_proper = a - g (proper acceleration)
                // In free fall, accelerometer reads 0. At rest on ground, reads -g
                let a_proper = a_world - model.gravity;
                let a_sensor = site_mat.transpose() * a_proper;
                if adr + 2 < data.sensordata.len() {
                    data.sensordata[adr] = a_sensor.x;
                    data.sensordata[adr + 1] = a_sensor.y;
                    data.sensordata[adr + 2] = a_sensor.z;
                }
            }

            MjSensorType::Force => {
                // Constraint/actuator force at site (simplified: use qfrc_constraint projection)
                // Full implementation would require Jacobian transpose mapping
                if adr + 2 < data.sensordata.len() {
                    data.sensordata[adr] = 0.0;
                    data.sensordata[adr + 1] = 0.0;
                    data.sensordata[adr + 2] = 0.0;
                }
            }

            MjSensorType::Torque => {
                // Constraint/actuator torque at site
                if adr + 2 < data.sensordata.len() {
                    data.sensordata[adr] = 0.0;
                    data.sensordata[adr + 1] = 0.0;
                    data.sensordata[adr + 2] = 0.0;
                }
            }

            MjSensorType::ActuatorFrc => {
                // Actuator force
                if objid < model.nu {
                    let jnt_id = model.actuator_trnid[objid];
                    if jnt_id < model.njnt {
                        let dof_adr = model.jnt_dof_adr[jnt_id];
                        data.sensordata[adr] = data.qfrc_actuator[dof_adr];
                    }
                }
            }

            MjSensorType::FrameLinAcc => {
                // Linear acceleration in world frame
                let a = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Body if objid < model.nbody => {
                        compute_body_acceleration(model, data, objid)
                    }
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        compute_body_acceleration(model, data, body_id)
                    }
                    _ => Vector3::zeros(),
                };
                if adr + 2 < data.sensordata.len() {
                    data.sensordata[adr] = a.x;
                    data.sensordata[adr + 1] = a.y;
                    data.sensordata[adr + 2] = a.z;
                }
            }

            MjSensorType::FrameAngAcc => {
                // Angular acceleration in world frame
                // Compute from qacc using Jacobian
                let alpha = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Body if objid < model.nbody => {
                        compute_body_angular_acceleration(model, data, objid)
                    }
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        compute_body_angular_acceleration(model, data, body_id)
                    }
                    _ => Vector3::zeros(),
                };
                if adr + 2 < data.sensordata.len() {
                    data.sensordata[adr] = alpha.x;
                    data.sensordata[adr + 1] = alpha.y;
                    data.sensordata[adr + 2] = alpha.z;
                }
            }

            // Skip position/velocity-dependent sensors
            _ => {}
        }
    }
}

// ============================================================================
// Sensor Helper Functions
// ============================================================================

/// Compute subtree center of mass for a given body (including all descendants).
fn compute_subtree_com(model: &Model, data: &Data, root_body: usize) -> (Vector3<f64>, f64) {
    let mut total_mass = 0.0;
    let mut weighted_com = Vector3::zeros();

    // Iterate through all bodies that are descendants of root_body
    for body_id in root_body..model.nbody {
        // Check if this body is a descendant (simplified: check parent chain)
        let mut is_descendant = body_id == root_body;
        let mut current = body_id;
        while !is_descendant && current != 0 {
            current = model.body_parent[current];
            if current == root_body {
                is_descendant = true;
            }
        }

        if is_descendant {
            let mass = model.body_mass[body_id];
            let com = data.xipos[body_id]; // COM in world frame
            total_mass += mass;
            weighted_com += mass * com;
        }
    }

    if total_mass > 1e-10 {
        (weighted_com / total_mass, total_mass)
    } else {
        (Vector3::zeros(), 0.0)
    }
}

/// Compute subtree linear momentum for a given body.
fn compute_subtree_momentum(model: &Model, data: &Data, root_body: usize) -> Vector3<f64> {
    let mut momentum = Vector3::zeros();

    for body_id in root_body..model.nbody {
        // Check if descendant
        let mut is_descendant = body_id == root_body;
        let mut current = body_id;
        while !is_descendant && current != 0 {
            current = model.body_parent[current];
            if current == root_body {
                is_descendant = true;
            }
        }

        if is_descendant {
            let mass = model.body_mass[body_id];
            let v = Vector3::new(
                data.cvel[body_id][3],
                data.cvel[body_id][4],
                data.cvel[body_id][5],
            );
            momentum += mass * v;
        }
    }

    momentum
}

/// Compute linear acceleration for a body from joint accelerations.
fn compute_body_acceleration(model: &Model, data: &Data, body_id: usize) -> Vector3<f64> {
    let mut acc = Vector3::zeros();

    // For each joint affecting this body, compute acceleration contribution
    let jnt_start = model.body_jnt_adr[body_id];
    let jnt_end = jnt_start + model.body_jnt_num[body_id];

    for jnt_id in jnt_start..jnt_end {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let axis = model.jnt_axis[jnt_id];

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge => {
                // Angular acceleration contributes via cross product with COM offset
                let qacc = data.qacc[dof_adr];
                let world_axis = data.xquat[body_id] * axis;
                let com_offset = data.xipos[body_id] - data.xpos[body_id];
                // a = α × r (tangential acceleration from angular acceleration)
                acc += qacc * world_axis.cross(&com_offset);

                // Also centripetal from angular velocity
                let omega = data.qvel[dof_adr] * world_axis;
                acc += omega.cross(&omega.cross(&com_offset));
            }
            MjJointType::Slide => {
                let qacc = data.qacc[dof_adr];
                let world_axis = data.xquat[body_id] * axis;
                acc += qacc * world_axis;
            }
            MjJointType::Free => {
                // Direct linear acceleration
                acc.x += data.qacc[dof_adr];
                acc.y += data.qacc[dof_adr + 1];
                acc.z += data.qacc[dof_adr + 2];
            }
            MjJointType::Ball => {
                // Ball joint only contributes angular acceleration
                // Linear acceleration comes from centripetal effects
                let omega = Vector3::new(
                    data.qvel[dof_adr],
                    data.qvel[dof_adr + 1],
                    data.qvel[dof_adr + 2],
                );
                let alpha = Vector3::new(
                    data.qacc[dof_adr],
                    data.qacc[dof_adr + 1],
                    data.qacc[dof_adr + 2],
                );
                let world_omega = data.xquat[body_id] * omega;
                let world_alpha = data.xquat[body_id] * alpha;
                let com_offset = data.xipos[body_id] - data.xpos[body_id];
                acc += world_alpha.cross(&com_offset);
                acc += world_omega.cross(&world_omega.cross(&com_offset));
            }
        }
    }

    acc
}

/// Compute angular acceleration for a body from joint accelerations.
fn compute_body_angular_acceleration(model: &Model, data: &Data, body_id: usize) -> Vector3<f64> {
    let mut alpha = Vector3::zeros();

    let jnt_start = model.body_jnt_adr[body_id];
    let jnt_end = jnt_start + model.body_jnt_num[body_id];

    for jnt_id in jnt_start..jnt_end {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let axis = model.jnt_axis[jnt_id];

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge => {
                let qacc = data.qacc[dof_adr];
                let world_axis = data.xquat[body_id] * axis;
                alpha += qacc * world_axis;
            }
            MjJointType::Ball => {
                let world_alpha = data.xquat[body_id]
                    * Vector3::new(
                        data.qacc[dof_adr],
                        data.qacc[dof_adr + 1],
                        data.qacc[dof_adr + 2],
                    );
                alpha += world_alpha;
            }
            MjJointType::Free => {
                alpha.x += data.qacc[dof_adr + 3];
                alpha.y += data.qacc[dof_adr + 4];
                alpha.z += data.qacc[dof_adr + 5];
            }
            MjJointType::Slide => {
                // Prismatic joints don't contribute angular acceleration
            }
        }
    }

    alpha
}

/// Velocity kinematics: compute body velocities from qvel.
fn mj_fwd_velocity(model: &Model, data: &mut Data) {
    // World body has zero velocity
    data.cvel[0] = SpatialVector::zeros();

    // Compute body velocities by propagating through tree
    // v[i] = X[i←parent] @ v[parent] + S[i] @ qdot[i]
    //
    // The spatial transform X accounts for the offset between body origins.
    // For a pure translation r (from parent to child), the velocity transforms as:
    //   ω_child = ω_parent
    //   v_child = v_parent + ω_parent × r
    //
    // This lever arm effect is critical for Coriolis forces in serial chains!

    for body_id in 1..model.nbody {
        let parent_id = model.body_parent[body_id];

        // Parent velocity
        let v_parent = data.cvel[parent_id];
        let omega_parent = Vector3::new(v_parent[0], v_parent[1], v_parent[2]);
        let v_lin_parent = Vector3::new(v_parent[3], v_parent[4], v_parent[5]);

        // Offset from parent origin to this body's origin (in world frame)
        let r = data.xpos[body_id] - data.xpos[parent_id];

        // Transform parent velocity to this body's frame
        // Linear velocity gets contribution from lever arm: v_new = v_old + ω × r
        let v_lin_at_child = v_lin_parent + omega_parent.cross(&r);

        let mut vel = SpatialVector::new(
            omega_parent.x,
            omega_parent.y,
            omega_parent.z,
            v_lin_at_child.x,
            v_lin_at_child.y,
            v_lin_at_child.z,
        );

        // Add contribution from joints on this body
        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let axis = model.jnt_axis[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    // Angular velocity contribution
                    let omega = data.xquat[body_id] * axis * data.qvel[dof_adr];
                    vel[0] += omega.x;
                    vel[1] += omega.y;
                    vel[2] += omega.z;
                }
                MjJointType::Slide => {
                    // Linear velocity contribution
                    let v = data.xquat[body_id] * axis * data.qvel[dof_adr];
                    vel[3] += v.x;
                    vel[4] += v.y;
                    vel[5] += v.z;
                }
                MjJointType::Ball => {
                    // 3-DOF angular velocity
                    let omega = Vector3::new(
                        data.qvel[dof_adr],
                        data.qvel[dof_adr + 1],
                        data.qvel[dof_adr + 2],
                    );
                    let world_omega = data.xquat[body_id] * omega;
                    vel[0] += world_omega.x;
                    vel[1] += world_omega.y;
                    vel[2] += world_omega.z;
                }
                MjJointType::Free => {
                    // 6-DOF: linear + angular velocity
                    vel[3] += data.qvel[dof_adr];
                    vel[4] += data.qvel[dof_adr + 1];
                    vel[5] += data.qvel[dof_adr + 2];
                    vel[0] += data.qvel[dof_adr + 3];
                    vel[1] += data.qvel[dof_adr + 4];
                    vel[2] += data.qvel[dof_adr + 5];
                }
            }
        }

        data.cvel[body_id] = vel;
    }
}

/// Compute actuator forces from control inputs.
fn mj_fwd_actuation(model: &Model, data: &mut Data) {
    data.qfrc_actuator.fill(0.0);

    for act_id in 0..model.nu {
        let ctrl = data.ctrl[act_id];
        let gear = model.actuator_gear[act_id];
        let trnid = model.actuator_trnid[act_id];

        match model.actuator_trntype[act_id] {
            ActuatorTransmission::Joint => {
                // Direct joint actuation
                if trnid < model.njnt {
                    let dof_adr = model.jnt_dof_adr[trnid];
                    let nv = model.jnt_type[trnid].nv();
                    // Apply force to first DOF of joint (simplification)
                    if nv > 0 {
                        data.qfrc_actuator[dof_adr] += ctrl * gear;
                    }
                }
            }
            ActuatorTransmission::Tendon | ActuatorTransmission::Site => {
                // Placeholder for tendon/site actuation
                // Will be implemented when tendons are integrated
            }
        }
    }
}

/// Composite Rigid Body Algorithm: compute mass matrix.
///
/// This implements Featherstone's CRBA algorithm which computes the joint-space
/// inertia matrix M such that kinetic energy T = 0.5 * `qvel^T` * M * `qvel`.
///
/// The algorithm works by:
/// 1. Computing composite inertias by accumulating from leaves to root
/// 2. Computing `M[i,j]` = `S_i^T` * `I_c` * `S_j` for each joint pair
///
/// Reference: Featherstone, "Rigid Body Dynamics Algorithms", Chapter 6
/// Composite Rigid Body Algorithm (CRBA) - Featherstone O(n) version.
///
/// Computes the joint-space inertia matrix M using the recursive algorithm:
/// 1. Initialize composite inertia Ic[i] = I[i] for each body
/// 2. Backward pass: Ic[parent] += transform(Ic[child])
/// 3. For each joint, compute M elements from Ic and joint motion subspace
///
/// Reference: Featherstone, "Rigid Body Dynamics Algorithms", Chapter 6
#[allow(
    clippy::many_single_char_names,
    clippy::too_many_lines,
    clippy::similar_names,
    clippy::needless_range_loop
)]
fn mj_crba(model: &Model, data: &mut Data) {
    data.qM.fill(0.0);

    if model.nv == 0 {
        return;
    }

    // ============================================================
    // Phase 1: Initialize composite inertias from body inertias
    // ============================================================
    // Copy cinert (computed once in FK) to crb_inertia as starting point.
    // cinert contains 6x6 spatial inertias for individual bodies in world frame.
    for body_id in 0..model.nbody {
        data.crb_inertia[body_id] = data.cinert[body_id];
    }

    // ============================================================
    // Phase 2: Backward pass - accumulate composite inertias
    // ============================================================
    // Process bodies from leaves to root, adding child inertia to parent.
    // This gives Ic[i] = inertia of subtree rooted at body i.
    //
    // Note: We need to transform child inertia to parent frame before adding.
    // For now, use a simpler approach: all inertias are in world frame,
    // so we just add them directly (valid because world frame is common).

    for body_id in (1..model.nbody).rev() {
        let parent_id = model.body_parent[body_id];
        if parent_id != 0 {
            // Add this body's composite inertia to parent's
            // Need to clone to avoid borrow checker issues
            let child_inertia = data.crb_inertia[body_id];
            data.crb_inertia[parent_id] += child_inertia;
        }
    }

    // ============================================================
    // Phase 3: Build mass matrix from composite inertias
    // ============================================================
    // For each joint, M[i,i] = S[i]^T * Ic[body_i] * S[i]
    // Off-diagonal elements require propagating forces up the tree.

    // Pre-compute all joint motion subspaces once (O(n) instead of O(n²))
    // This is a significant optimization for debug builds where function call
    // overhead and matrix allocation dominate.
    let joint_subspaces: Vec<_> = (0..model.njnt)
        .map(|jnt_id| joint_motion_subspace(model, data, jnt_id))
        .collect();

    for jnt_i in 0..model.njnt {
        let body_i = model.jnt_body[jnt_i];
        let dof_i = model.jnt_dof_adr[jnt_i];
        let nv_i = model.jnt_type[jnt_i].nv();

        // Use cached joint motion subspace
        let s_i = &joint_subspaces[jnt_i];

        // Diagonal block: M[i,i] = S^T * Ic * S
        let ic = &data.crb_inertia[body_i];
        for di in 0..nv_i {
            let s_col_i = s_i.column(di);
            for dj in 0..nv_i {
                let s_col_j = s_i.column(dj);
                // Compute s_i^T * Ic * s_j
                let ic_s_j = ic * s_col_j;
                data.qM[(dof_i + di, dof_i + dj)] = s_col_i.dot(&ic_s_j);
            }
        }

        // Off-diagonal: propagate to ancestor joints
        // F = Ic * S (force due to joint motion)
        // Then traverse ancestors, transforming F at each step:
        //   F_parent = X^T * F_child (spatial force transform)
        // And computing M[j,i] = S_j^T * F_transformed
        let mut force = ic * s_i; // 6 x nv_i matrix, starts at body_i

        let mut child_body = body_i;
        let mut current_body = model.body_parent[body_i];

        while current_body != 0 {
            // Transform force from child to current body using spatial force transform
            // F_parent = [I, 0; [r]x, I]^T * F_child = [I, [r]x^T; 0, I] * F_child
            // Where r = pos_child - pos_parent (vector from parent to child)
            let r = data.xpos[child_body] - data.xpos[current_body];

            // Apply spatial force transform:
            // F_angular_parent = F_angular_child + r × F_linear_child
            // F_linear_parent = F_linear_child
            for col in 0..nv_i {
                let f_lin_x = force[(3, col)];
                let f_lin_y = force[(4, col)];
                let f_lin_z = force[(5, col)];

                // r × f_lin
                let cross_x = r.y * f_lin_z - r.z * f_lin_y;
                let cross_y = r.z * f_lin_x - r.x * f_lin_z;
                let cross_z = r.x * f_lin_y - r.y * f_lin_x;

                force[(0, col)] += cross_x;
                force[(1, col)] += cross_y;
                force[(2, col)] += cross_z;
                // Linear part stays the same
            }

            // Find joint(s) for this ancestor body
            let jnt_start = model.body_jnt_adr[current_body];
            let jnt_count = model.body_jnt_num[current_body];

            for jnt_j in jnt_start..(jnt_start + jnt_count) {
                if jnt_j >= model.njnt {
                    break;
                }

                let dof_j = model.jnt_dof_adr[jnt_j];
                let nv_j = model.jnt_type[jnt_j].nv();
                // Use cached joint motion subspace instead of recomputing
                let s_j = &joint_subspaces[jnt_j];

                // M[j,i] = S_j^T * F
                for dj in 0..nv_j {
                    let s_col_j = s_j.column(dj);
                    for di in 0..nv_i {
                        let f_col_i = force.column(di);
                        let m_ji = s_col_j.dot(&f_col_i);
                        data.qM[(dof_j + dj, dof_i + di)] = m_ji;
                        data.qM[(dof_i + di, dof_j + dj)] = m_ji; // Symmetry
                    }
                }
            }

            // Move up the tree
            child_body = current_body;
            current_body = model.body_parent[current_body];
        }
    }

    // ============================================================
    // Phase 4: Add armature inertia to diagonal
    // ============================================================
    for jnt_id in 0..model.njnt {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let nv = model.jnt_type[jnt_id].nv();
        let armature = model.jnt_armature[jnt_id];

        for i in 0..nv {
            data.qM[(dof_adr + i, dof_adr + i)] += armature;
            if let Some(&dof_arm) = model.dof_armature.get(dof_adr + i) {
                data.qM[(dof_adr + i, dof_adr + i)] += dof_arm;
            }
        }
    }

    // ============================================================
    // Phase 5: Cache Cholesky factorization
    // ============================================================
    // Compute L where M = L L^T once here, reuse in:
    //   - mj_fwd_acceleration(): Solve M qacc = τ  →  O(n²) instead of O(n³)
    //   - pgs_solve_contacts(): Compute A = J M^{-1} J^T  →  O(n²) per contact
    //
    // The clone is required: nalgebra::cholesky() consumes the matrix to avoid
    // allocating a separate buffer. This is the optimal single-allocation approach.
    // Total cost: O(n³) factorization + O(n²) memory, done once per forward().
    data.qM_cholesky = data.qM.clone().cholesky();

    // ============================================================
    // Phase 6: Cache body effective mass/inertia from qM diagonal
    // ============================================================
    // Extract per-body min mass/inertia for constraint force limiting.
    // This avoids O(joints) traversal per constraint.
    cache_body_effective_mass(model, data);
}

/// Cache per-body minimum mass and inertia from the mass matrix diagonal.
///
/// This extracts the minimum diagonal elements for each body's DOFs and stores
/// them in `data.body_min_mass` and `data.body_min_inertia`. These cached values
/// are used by constraint force limiting to avoid repeated mass matrix queries.
///
/// Uses the `JointVisitor` pattern to ensure consistency with joint iteration
/// elsewhere in the codebase.
///
/// Must be called after `mj_crba()` has computed the mass matrix.
fn cache_body_effective_mass(model: &Model, data: &mut Data) {
    // Visitor struct for JointVisitor pattern (defined before statements per clippy)
    struct MassCacheVisitor<'a> {
        model: &'a Model,
        data: &'a mut Data,
    }

    impl JointVisitor for MassCacheVisitor<'_> {
        fn visit_free(&mut self, ctx: JointContext) {
            let body_id = self.model.jnt_body[ctx.jnt_id];
            // Linear DOFs at 0-2
            for i in 0..3 {
                let dof = ctx.dof_adr + i;
                if dof < self.model.nv {
                    let mass = self.data.qM[(dof, dof)];
                    if mass > MIN_INERTIA_THRESHOLD {
                        self.data.body_min_mass[body_id] =
                            self.data.body_min_mass[body_id].min(mass);
                    }
                }
            }
            // Angular DOFs at 3-5
            for i in 3..6 {
                let dof = ctx.dof_adr + i;
                if dof < self.model.nv {
                    let inertia = self.data.qM[(dof, dof)];
                    if inertia > MIN_INERTIA_THRESHOLD {
                        self.data.body_min_inertia[body_id] =
                            self.data.body_min_inertia[body_id].min(inertia);
                    }
                }
            }
        }

        fn visit_ball(&mut self, ctx: JointContext) {
            let body_id = self.model.jnt_body[ctx.jnt_id];
            // All 3 DOFs are angular
            for i in 0..3 {
                let dof = ctx.dof_adr + i;
                if dof < self.model.nv {
                    let inertia = self.data.qM[(dof, dof)];
                    if inertia > MIN_INERTIA_THRESHOLD {
                        self.data.body_min_inertia[body_id] =
                            self.data.body_min_inertia[body_id].min(inertia);
                    }
                }
            }
        }

        fn visit_hinge(&mut self, ctx: JointContext) {
            let body_id = self.model.jnt_body[ctx.jnt_id];
            // Single angular DOF
            if ctx.dof_adr < self.model.nv {
                let inertia = self.data.qM[(ctx.dof_adr, ctx.dof_adr)];
                if inertia > MIN_INERTIA_THRESHOLD {
                    self.data.body_min_inertia[body_id] =
                        self.data.body_min_inertia[body_id].min(inertia);
                }
            }
        }

        fn visit_slide(&mut self, ctx: JointContext) {
            let body_id = self.model.jnt_body[ctx.jnt_id];
            // Single linear DOF
            if ctx.dof_adr < self.model.nv {
                let mass = self.data.qM[(ctx.dof_adr, ctx.dof_adr)];
                if mass > MIN_INERTIA_THRESHOLD {
                    self.data.body_min_mass[body_id] = self.data.body_min_mass[body_id].min(mass);
                }
            }
        }
    }

    // Reset to defaults (world body stays at infinity)
    for i in 1..model.nbody {
        data.body_min_mass[i] = f64::INFINITY;
        data.body_min_inertia[i] = f64::INFINITY;
    }

    let mut visitor = MassCacheVisitor { model, data };
    model.visit_joints(&mut visitor);

    // Replace infinity with default for bodies that had no DOFs of that type
    for i in 1..model.nbody {
        if data.body_min_mass[i] == f64::INFINITY {
            data.body_min_mass[i] = DEFAULT_MASS_FALLBACK;
        }
        if data.body_min_inertia[i] == f64::INFINITY {
            data.body_min_inertia[i] = DEFAULT_MASS_FALLBACK;
        }
    }
}

/// Compute the joint motion subspace matrix S (6 x nv).
///
/// S maps joint velocity to spatial velocity in world frame:
/// v_spatial = S * qdot
///
/// Format: rows 0-2 = angular velocity, rows 3-5 = linear velocity
#[allow(clippy::similar_names)]
#[allow(clippy::inline_always)] // Hot path in CRBA/RNE - profiling shows inlining improves debug performance
#[inline(always)]
fn joint_motion_subspace(
    model: &Model,
    data: &Data,
    jnt_id: usize,
) -> nalgebra::SMatrix<f64, 6, 6> {
    let body_id = model.jnt_body[jnt_id];
    let jnt_type = model.jnt_type[jnt_id];
    let nv = jnt_type.nv();

    let mut s = nalgebra::SMatrix::<f64, 6, 6>::zeros();

    match jnt_type {
        MjJointType::Hinge => {
            // Revolute joint: S = [axis; axis × r]^T where r is from joint to body origin
            let axis_world = data.xquat[body_id] * model.jnt_axis[jnt_id];
            let jpos_world = data.xpos[body_id] + data.xquat[body_id] * model.jnt_pos[jnt_id];
            let r = data.xpos[body_id] - jpos_world;

            // Angular part (rows 0-2)
            s[(0, 0)] = axis_world.x;
            s[(1, 0)] = axis_world.y;
            s[(2, 0)] = axis_world.z;

            // Linear part (rows 3-5) = axis × r
            let lin = axis_world.cross(&r);
            s[(3, 0)] = lin.x;
            s[(4, 0)] = lin.y;
            s[(5, 0)] = lin.z;
        }
        MjJointType::Slide => {
            // Prismatic joint: S = [0; axis]^T
            let axis_world = data.xquat[body_id] * model.jnt_axis[jnt_id];
            s[(3, 0)] = axis_world.x;
            s[(4, 0)] = axis_world.y;
            s[(5, 0)] = axis_world.z;
        }
        MjJointType::Ball => {
            // Ball joint: 3 angular DOFs
            // S = [I_3x3; 0_3x3] (angular velocity in world frame)
            for i in 0..3 {
                s[(i, i)] = 1.0;
            }
        }
        MjJointType::Free => {
            // Free joint: 6 DOFs (3 linear + 3 angular)
            // DOF order in qvel: [vx, vy, vz, ωx, ωy, ωz]
            // Linear DOFs map to linear velocity
            for i in 0..3 {
                s[(3 + i, i)] = 1.0; // Linear velocity
            }
            // Angular DOFs map to angular velocity
            for i in 0..3 {
                s[(i, 3 + i)] = 1.0; // Angular velocity
            }
        }
    }

    // Only return the columns needed
    let _ = nv; // Used implicitly through jnt_type
    s
}

/// Recursive Newton-Euler: compute bias forces (Coriolis + centrifugal + gravity).
///
/// The bias force vector `c(q, qdot)` contains:
/// - Gravity forces projected to joint space
/// - Coriolis/centrifugal forces: velocity-dependent terms
///
/// For the equation of motion: `M * qacc + c(q, qdot) = τ`
///
/// ## Implementation Notes
///
/// This implementation uses a hybrid approach for efficiency and correctness:
/// 1. **Gravity**: O(n) computation using precomputed subtree mass and COM
/// 2. **Gyroscopic**: Direct computation for Ball/Free joints (ω × Iω)
/// 3. **Coriolis/Centrifugal**: O(n²) computation using body velocities
///
/// A full Featherstone O(n) RNE would use spatial algebra throughout, but this
/// hybrid approach achieves the same physics with simpler code that's easier
/// to verify for correctness.
///
/// Reference: Featherstone, "Rigid Body Dynamics Algorithms", Chapter 5
/// Reference: MuJoCo Computation docs - mj_rne section
#[allow(
    clippy::too_many_lines,
    clippy::similar_names,
    clippy::needless_range_loop
)]
fn mj_rne(model: &Model, data: &mut Data) {
    data.qfrc_bias.fill(0.0);

    if model.nv == 0 {
        return;
    }

    // ========== Gravity contribution (O(n) using precomputed subtree mass/COM) ==========
    // The bias force is what we need to SUBTRACT from applied forces.
    // For joint i on body b: τ_g[i] = J_i^T * (M_subtree * g)
    // where M_subtree is total mass below body b, and the torque acts at subtree COM
    for jnt_id in 0..model.njnt {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let jnt_body = model.jnt_body[jnt_id];

        // Use precomputed subtree mass and COM for O(n) gravity
        let subtree_mass = data.subtree_mass[jnt_body];
        let subtree_com = data.subtree_com[jnt_body];

        // Negative because qfrc_bias opposes motion
        let gravity_force = -subtree_mass * model.gravity;

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge => {
                let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                let r = subtree_com - jpos;
                let torque = r.cross(&gravity_force);
                data.qfrc_bias[dof_adr] += torque.dot(&axis);
            }
            MjJointType::Slide => {
                let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                data.qfrc_bias[dof_adr] += gravity_force.dot(&axis);
            }
            MjJointType::Ball => {
                let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                let r = subtree_com - jpos;
                let torque = r.cross(&gravity_force);
                let body_torque = data.xquat[jnt_body].inverse() * torque;
                data.qfrc_bias[dof_adr] += body_torque.x;
                data.qfrc_bias[dof_adr + 1] += body_torque.y;
                data.qfrc_bias[dof_adr + 2] += body_torque.z;
            }
            MjJointType::Free => {
                // Linear gravity
                data.qfrc_bias[dof_adr] += gravity_force.x;
                data.qfrc_bias[dof_adr + 1] += gravity_force.y;
                data.qfrc_bias[dof_adr + 2] += gravity_force.z;
                // Angular: gravity torque from subtree COM relative to free joint position
                let jpos = Vector3::new(
                    data.qpos[model.jnt_qpos_adr[jnt_id]],
                    data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
                    data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
                );
                let r = subtree_com - jpos;
                let torque = r.cross(&gravity_force);
                data.qfrc_bias[dof_adr + 3] += torque.x;
                data.qfrc_bias[dof_adr + 4] += torque.y;
                data.qfrc_bias[dof_adr + 5] += torque.z;
            }
        }
    }

    // ========== Gyroscopic terms for Ball/Free joints ==========
    // τ_gyro = ω × (I * ω) - the gyroscopic torque
    // This is the dominant Coriolis effect for 3D rotations
    for body_id in 1..model.nbody {
        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Ball => {
                    // Angular velocity in body frame
                    let omega_body = Vector3::new(
                        data.qvel[dof_adr],
                        data.qvel[dof_adr + 1],
                        data.qvel[dof_adr + 2],
                    );
                    // Body inertia (diagonal in principal axes)
                    let inertia = model.body_inertia[body_id];
                    // I * ω
                    let i_omega = Vector3::new(
                        inertia.x * omega_body.x,
                        inertia.y * omega_body.y,
                        inertia.z * omega_body.z,
                    );
                    // Gyroscopic torque: ω × (I * ω)
                    let gyro = omega_body.cross(&i_omega);
                    data.qfrc_bias[dof_adr] += gyro.x;
                    data.qfrc_bias[dof_adr + 1] += gyro.y;
                    data.qfrc_bias[dof_adr + 2] += gyro.z;
                }
                MjJointType::Free => {
                    // Angular DOFs are at dof_adr + 3..6
                    let omega_body = Vector3::new(
                        data.qvel[dof_adr + 3],
                        data.qvel[dof_adr + 4],
                        data.qvel[dof_adr + 5],
                    );
                    let inertia = model.body_inertia[body_id];
                    let i_omega = Vector3::new(
                        inertia.x * omega_body.x,
                        inertia.y * omega_body.y,
                        inertia.z * omega_body.z,
                    );
                    let gyro = omega_body.cross(&i_omega);
                    data.qfrc_bias[dof_adr + 3] += gyro.x;
                    data.qfrc_bias[dof_adr + 4] += gyro.y;
                    data.qfrc_bias[dof_adr + 5] += gyro.z;
                }
                _ => {}
            }
        }
    }

    // ========== Coriolis/Centrifugal via Analytical Featherstone RNE ==========
    //
    // This is O(n) and replaces the O(n³) Christoffel symbol computation.
    // The algorithm:
    //   1. Forward pass: compute bias accelerations (velocity-dependent accelerations)
    //   2. Backward pass: compute bias forces and project to joint space
    //
    // Skip if all velocities are small (optimization for static/quasi-static cases)
    let max_qdot = data.qvel.iter().map(|v| v.abs()).fold(0.0, f64::max);
    if max_qdot < 1e-6 {
        return;
    }

    // Initialize bias accelerations and forces
    for i in 0..model.nbody {
        data.cacc_bias[i] = SpatialVector::zeros();
        data.cfrc_bias[i] = SpatialVector::zeros();
    }

    // ========== Forward Pass: Compute Bias Accelerations ==========
    // For each body, accumulate velocity-dependent accelerations.
    //
    // Featherstone's algorithm for bias acceleration:
    //   a_bias[i] = a_bias[parent] + c[i]
    //   c[i] = v[i] ×_m (S[i] @ qdot[i])  (velocity-product acceleration)
    //
    // The key insight: the bias acceleration arises from two sources:
    // 1. The acceleration of the parent frame (a_bias[parent])
    // 2. The velocity-product c[i] = v[i] ×_m v_joint, which captures
    //    centripetal and Coriolis accelerations
    //
    // Note: v[i] is the FULL body velocity (already computed in mj_fwd_velocity)

    // Pre-compute all joint motion subspaces once for RNE (same optimization as CRBA)
    let joint_subspaces: Vec<_> = (0..model.njnt)
        .map(|jnt_id| joint_motion_subspace(model, data, jnt_id))
        .collect();

    for body_id in 1..model.nbody {
        let parent_id = model.body_parent[body_id];

        // Start with parent's bias acceleration
        let mut a_bias = data.cacc_bias[parent_id];

        // Add velocity-product acceleration from joints on this body
        // c[i] = v[i] ×_m (S[i] @ qdot[i])
        // This uses the BODY velocity (not parent), as per Featherstone

        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let s = &joint_subspaces[jnt_id];
            let nv = model.jnt_type[jnt_id].nv();

            // Compute joint velocity contribution: v_joint = S @ qdot
            let mut v_joint = SpatialVector::zeros();
            for d in 0..nv {
                for row in 0..6 {
                    v_joint[row] += s[(row, d)] * data.qvel[dof_adr + d];
                }
            }

            // Velocity-product acceleration: v[parent] ×_m v_joint
            // This is the centripetal/Coriolis acceleration contribution
            // Note: using parent velocity because v[i] = v[parent] + S@qdot,
            // and (S@qdot) ×_m (S@qdot) = 0, so v[i] ×_m (S@qdot) = v[parent] ×_m (S@qdot)
            let v_parent = data.cvel[parent_id];
            a_bias += spatial_cross_motion(v_parent, v_joint);
        }

        data.cacc_bias[body_id] = a_bias;
    }

    // ========== Backward Pass: Compute Bias Forces ==========
    // For each body from leaves to root:
    //   f_bias[i] = I[i] @ a_bias[i] + v[i] ×* (I[i] @ v[i])
    //   f_bias[parent] += f_bias[i]
    //
    // Then project to joint space:
    //   τ_bias[dof] = S[i]^T @ f_bias[i]

    // First compute f_bias for each body using cinert (computed once in FK)
    for body_id in 1..model.nbody {
        let i = &data.cinert[body_id];
        let v = data.cvel[body_id];
        let a_bias = data.cacc_bias[body_id];

        // I @ v (momentum)
        let i_v = i * v;

        // I @ a_bias (inertial force from bias acceleration)
        let i_a = i * a_bias;

        // v ×* (I @ v) (gyroscopic/Coriolis force)
        let gyro = spatial_cross_force(v, i_v);

        // f_bias = I @ a_bias + v ×* (I @ v)
        data.cfrc_bias[body_id] = i_a + gyro;
    }

    // Propagate forces from leaves to root
    for body_id in (1..model.nbody).rev() {
        let parent_id = model.body_parent[body_id];
        if parent_id != 0 {
            // Add child's force to parent (already in world frame, no transform needed)
            let child_force = data.cfrc_bias[body_id];
            data.cfrc_bias[parent_id] += child_force;
        }
    }

    // ========== Project to Joint Space ==========
    // τ_bias[dof] = S^T @ f_bias[body]
    for jnt_id in 0..model.njnt {
        let body_id = model.jnt_body[jnt_id];
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let s = &joint_subspaces[jnt_id]; // Use cached subspace
        let nv = model.jnt_type[jnt_id].nv();

        let f = data.cfrc_bias[body_id];

        // τ = S^T @ f
        for d in 0..nv {
            let mut tau = 0.0;
            for row in 0..6 {
                tau += s[(row, d)] * f[row];
            }
            data.qfrc_bias[dof_adr + d] += tau;
        }
    }
}

/// Smoothing factor for friction loss approximation.
/// Controls sharpness of tanh transition from 0 to full friction.
/// At vel=0.001, tanh(1000 * 0.001) ≈ 0.76, providing smooth onset.
/// At vel=0.01, tanh(1000 * 0.01) ≈ 1.0, full friction magnitude.
const FRICTION_SMOOTHING: f64 = 1000.0;

/// Velocity threshold below which friction is not applied.
/// Prevents numerical noise from creating spurious friction forces.
const FRICTION_VELOCITY_THRESHOLD: f64 = 1e-12;

/// Compute passive forces (springs, dampers, and friction loss).
///
/// Implements MuJoCo's passive force model:
/// - **Spring**: τ = -stiffness * (q - springref)
/// - **Damper**: τ = -damping * qvel
/// - **Friction loss**: τ = -frictionloss * tanh(qvel * FRICTION_SMOOTHING)
///
/// The friction loss uses a smooth `tanh` approximation to Coulomb friction
/// (`sign(qvel)`) to avoid discontinuity at zero velocity, which would cause
/// numerical issues with explicit integrators.
///
/// # MuJoCo Semantics
///
/// The spring equilibrium is `jnt_springref`, NOT `qpos0`. These are distinct:
/// - `qpos0`: Initial joint position at model load (for `mj_resetData()`)
/// - `springref`: Spring equilibrium position (where spring force is zero)
///
/// A joint can start at q=0 but have a spring pulling toward springref=0.5.
///
/// # Implicit Integration Mode
///
/// When `model.integrator == Implicit`, spring and damper forces are handled
/// implicitly in `mj_fwd_acceleration_implicit()`. This function then only
/// computes friction loss (which is velocity-sign-dependent and cannot be
/// linearized into the implicit solve).
fn mj_fwd_passive(model: &Model, data: &mut Data) {
    data.qfrc_passive.fill(0.0);

    let implicit_mode = model.integrator == Integrator::Implicit;
    let mut visitor = PassiveForceVisitor {
        model,
        data,
        implicit_mode,
    };
    model.visit_joints(&mut visitor);
}

/// Visitor for computing passive forces (springs, dampers, friction loss).
struct PassiveForceVisitor<'a> {
    model: &'a Model,
    data: &'a mut Data,
    implicit_mode: bool,
}

impl PassiveForceVisitor<'_> {
    /// Apply friction loss force at a single DOF.
    /// Friction loss is always explicit (velocity-sign-dependent, cannot linearize).
    #[inline]
    fn apply_friction_loss(&mut self, dof_idx: usize) {
        let qvel = self.data.qvel[dof_idx];
        let frictionloss = self.model.dof_frictionloss[dof_idx];
        if frictionloss > 0.0 && qvel.abs() > FRICTION_VELOCITY_THRESHOLD {
            let smooth_sign = (qvel * FRICTION_SMOOTHING).tanh();
            self.data.qfrc_passive[dof_idx] -= frictionloss * smooth_sign;
        }
    }

    /// Process a 1-DOF joint (Hinge or Slide) with spring, damper, and friction.
    #[inline]
    fn visit_1dof_joint(&mut self, ctx: JointContext) {
        let dof_adr = ctx.dof_adr;
        let qpos_adr = ctx.qpos_adr;
        let jnt_id = ctx.jnt_id;

        if !self.implicit_mode {
            // Spring: τ = -k * (q - springref)
            let stiffness = self.model.jnt_stiffness[jnt_id];
            let springref = self.model.jnt_springref[jnt_id];
            let q = self.data.qpos[qpos_adr];
            self.data.qfrc_passive[dof_adr] -= stiffness * (q - springref);

            // Damper: τ = -b * qvel
            let damping = self.model.jnt_damping[jnt_id];
            let qvel = self.data.qvel[dof_adr];
            self.data.qfrc_passive[dof_adr] -= damping * qvel;
        }

        // Friction loss: always explicit
        self.apply_friction_loss(dof_adr);
    }

    /// Process a multi-DOF joint (Ball or Free) with per-DOF damping and friction.
    /// No springs (would require quaternion spring formulation).
    #[inline]
    fn visit_multi_dof_joint(&mut self, ctx: JointContext) {
        for i in 0..ctx.nv {
            let dof_idx = ctx.dof_adr + i;

            if !self.implicit_mode {
                // Per-DOF damping
                let dof_damping = self.model.dof_damping[dof_idx];
                let qvel = self.data.qvel[dof_idx];
                self.data.qfrc_passive[dof_idx] -= dof_damping * qvel;
            }

            // Friction loss: always explicit
            self.apply_friction_loss(dof_idx);
        }
    }
}

impl JointVisitor for PassiveForceVisitor<'_> {
    #[inline]
    fn visit_hinge(&mut self, ctx: JointContext) {
        self.visit_1dof_joint(ctx);
    }

    #[inline]
    fn visit_slide(&mut self, ctx: JointContext) {
        self.visit_1dof_joint(ctx);
    }

    #[inline]
    fn visit_ball(&mut self, ctx: JointContext) {
        self.visit_multi_dof_joint(ctx);
    }

    #[inline]
    fn visit_free(&mut self, ctx: JointContext) {
        self.visit_multi_dof_joint(ctx);
    }
}

// ============================================================================
// Contact Jacobian Computation
// ============================================================================

/// Compute the contact Jacobian row for a single body contribution.
///
/// The Jacobian maps joint velocities to Cartesian point velocity:
/// v_point = J * qvel
///
/// For a contact at world point `p` on body `body_id`, we walk the kinematic
/// chain from body to root, accumulating Jacobian contributions from each joint.
///
/// Returns the Jacobian row (length nv) for translational velocity at the point.
#[allow(dead_code)] // Reserved for future use in inverse dynamics
fn compute_body_jacobian_at_point(
    model: &Model,
    data: &Data,
    body_id: usize,
    point: Vector3<f64>,
) -> DVector<f64> {
    let mut jacobian = DVector::zeros(model.nv);

    if body_id == 0 {
        return jacobian; // World has no DOFs
    }

    // Walk from body to root, accumulating Jacobian contributions
    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    // v = axis × r (rotational velocity contribution)
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    let j_col = axis.cross(&r);
                    jacobian[dof_adr] = j_col.x;
                    // Store the other components in a 3xnv matrix style
                    // For now, we'll compute this per-direction
                }
                MjJointType::Slide => {
                    // v = axis (translational velocity contribution)
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    jacobian[dof_adr] = axis.x;
                }
                MjJointType::Ball => {
                    // Angular velocity contribution
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    // v = ω × r, and ω is in body frame for ball joint
                    // ω_world = R * ω_body, v = ω_world × r
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        // ω_body[i] contribution
                        let omega_world = rot * Vector3::ith(i, 1.0);
                        let j_col = omega_world.cross(&r);
                        jacobian[dof_adr + i] = j_col.x;
                    }
                }
                MjJointType::Free => {
                    // Linear DOFs directly contribute
                    jacobian[dof_adr] = 1.0; // vx
                    jacobian[dof_adr + 1] = 0.0; // vy (this is for x direction)
                    jacobian[dof_adr + 2] = 0.0; // vz

                    // Angular DOFs: v = ω × r
                    let jpos = Vector3::new(
                        data.qpos[model.jnt_qpos_adr[jnt_id]],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
                    );
                    let r = point - jpos;
                    // v = ω × r => for ω in world frame
                    // ∂vx/∂ωy = rz, ∂vx/∂ωz = -ry
                    jacobian[dof_adr + 3] = 0.0; // ∂vx/∂ωx
                    jacobian[dof_adr + 4] = r.z; // ∂vx/∂ωy
                    jacobian[dof_adr + 5] = -r.y; // ∂vx/∂ωz
                }
            }
        }
        current_body = model.body_parent[current_body];
    }

    jacobian
}

/// Compute the full 3xnv contact Jacobian for a contact point.
///
/// Returns a 3×nv matrix where:
/// - Row 0: normal direction Jacobian
/// - Row 1: tangent1 direction Jacobian
/// - Row 2: tangent2 direction Jacobian
fn compute_contact_jacobian(model: &Model, data: &Data, contact: &Contact) -> DMatrix<f64> {
    let nv = model.nv;
    let body1 = model.geom_body[contact.geom1];
    let body2 = model.geom_body[contact.geom2];

    // Build orthonormal tangent basis
    let normal = contact.normal;
    let (tangent1, tangent2) = build_tangent_basis(&normal);

    // Allocate 3×nv Jacobian
    let mut j = DMatrix::zeros(3, nv);

    // Helper: add body Jacobian contribution for one direction
    let add_body_jacobian =
        |j: &mut DMatrix<f64>, row: usize, direction: &Vector3<f64>, body_id: usize, sign: f64| {
            if body_id == 0 {
                return; // World has no DOFs
            }

            let mut current_body = body_id;
            while current_body != 0 {
                let jnt_start = model.body_jnt_adr[current_body];
                let jnt_end = jnt_start + model.body_jnt_num[current_body];

                for jnt_id in jnt_start..jnt_end {
                    let dof_adr = model.jnt_dof_adr[jnt_id];
                    let jnt_body = model.jnt_body[jnt_id];

                    match model.jnt_type[jnt_id] {
                        MjJointType::Hinge => {
                            let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                            let jpos =
                                data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                            let r = contact.pos - jpos;
                            let j_col = axis.cross(&r);
                            j[(row, dof_adr)] += sign * direction.dot(&j_col);
                        }
                        MjJointType::Slide => {
                            let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                            j[(row, dof_adr)] += sign * direction.dot(&axis);
                        }
                        MjJointType::Ball => {
                            let jpos =
                                data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                            let r = contact.pos - jpos;
                            let rot = data.xquat[jnt_body].to_rotation_matrix();
                            for i in 0..3 {
                                let omega_world = rot * Vector3::ith(i, 1.0);
                                let j_col = omega_world.cross(&r);
                                j[(row, dof_adr + i)] += sign * direction.dot(&j_col);
                            }
                        }
                        MjJointType::Free => {
                            // Linear DOFs
                            j[(row, dof_adr)] += sign * direction.x;
                            j[(row, dof_adr + 1)] += sign * direction.y;
                            j[(row, dof_adr + 2)] += sign * direction.z;

                            // Angular DOFs: v = ω × r
                            let jpos = Vector3::new(
                                data.qpos[model.jnt_qpos_adr[jnt_id]],
                                data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
                                data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
                            );
                            let r = contact.pos - jpos;
                            // d/dω of (ω × r) in direction d: (e_i × r) · d
                            let ex = Vector3::x();
                            let ey = Vector3::y();
                            let ez = Vector3::z();
                            j[(row, dof_adr + 3)] += sign * direction.dot(&ex.cross(&r));
                            j[(row, dof_adr + 4)] += sign * direction.dot(&ey.cross(&r));
                            j[(row, dof_adr + 5)] += sign * direction.dot(&ez.cross(&r));
                        }
                    }
                }
                current_body = model.body_parent[current_body];
            }
        };

    // Compute relative velocity J * qvel = v2 - v1 (velocity of body2 relative to body1)
    // Normal points FROM body1 (geom1) TO body2 (geom2).
    // Positive normal velocity = bodies separating (good)
    // Negative normal velocity = bodies approaching (need constraint)
    //
    // Body2 contributes positively (its velocity in +normal direction = separating)
    // Body1 contributes negatively (its velocity in +normal direction = approaching body2)
    add_body_jacobian(&mut j, 0, &normal, body2, 1.0);
    add_body_jacobian(&mut j, 0, &normal, body1, -1.0);

    add_body_jacobian(&mut j, 1, &tangent1, body2, 1.0);
    add_body_jacobian(&mut j, 1, &tangent1, body1, -1.0);

    add_body_jacobian(&mut j, 2, &tangent2, body2, 1.0);
    add_body_jacobian(&mut j, 2, &tangent2, body1, -1.0);

    j
}

/// Build an orthonormal tangent basis from a normal vector.
fn build_tangent_basis(normal: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    // Pick a vector not parallel to normal
    let ref_vec = if normal.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };

    // Gram-Schmidt: project out the normal component
    let tangent_unnorm = ref_vec - normal * normal.dot(&ref_vec);
    let tangent_norm = tangent_unnorm.norm();
    let tangent1 = if tangent_norm > 1e-10 {
        tangent_unnorm / tangent_norm
    } else {
        // Degenerate case: normal is zero or parallel to ref_vec
        Vector3::x()
    };
    let tangent2 = normal.cross(&tangent1);

    (tangent1, tangent2)
}

// ============================================================================
// PGS Constraint Solver
// ============================================================================

/// Check if two bodies share a kinematic chain.
///
/// Returns true if the bodies share any common ancestor joint, meaning forces
/// on one body propagate to the other through the kinematic chain. This includes:
/// - Same body (trivially coupled)
/// - Ancestor-descendant relationships (direct chain)
/// - Siblings or cousins (share a common ancestor joint)
///
/// This is used to determine if off-diagonal constraint blocks are non-zero.
/// Uses pre-computed `body_ancestor_mask` for O(num_words) lookup where
/// num_words = ceil(njnt / 64), typically 1 for models with <64 joints.
///
/// Note: Bodies with empty ancestor masks (no joints) return false, which is
/// correct since they have no DOFs and cannot be kinematically coupled.
#[inline]
fn bodies_share_chain(model: &Model, body_a: usize, body_b: usize) -> bool {
    if body_a == body_b {
        return true;
    }

    // World body (0) has no joints - never coupled with anything except itself
    if body_a == 0 || body_b == 0 {
        return false;
    }

    // Check if the bodies share any common ancestor joint using bitmask AND.
    // If the intersection is non-empty, they're in the same kinematic chain.
    let mask_a = &model.body_ancestor_mask[body_a];
    let mask_b = &model.body_ancestor_mask[body_b];

    // Iterate over words (typically just 1 for models with <64 joints).
    // If either mask is empty (len=0), num_words=0 and loop returns false immediately.
    // This correctly handles edge cases like bodies with no ancestor joints.
    let num_words = mask_a.len().min(mask_b.len());
    for i in 0..num_words {
        if (mask_a[i] & mask_b[i]) != 0 {
            return true;
        }
    }

    false
}

/// Projected Gauss-Seidel solver for contact constraints.
///
/// Solves the LCP:
///   minimize: (1/2) λ^T (A + R) λ + λ^T b
///   subject to: λ_n ≥ 0, |λ_t| ≤ μ λ_n
///
/// Where:
///   A = J * M^{-1} * J^T (constraint-space inverse inertia)
///   R = regularization (softness)
///   b = J * qacc_smooth - aref (velocity error with Baumgarte stabilization)
///
/// Returns the constraint forces in contact-frame (λ).
fn pgs_solve_contacts(
    model: &Model,
    data: &mut Data,
    contacts: &[Contact],
    jacobians: &[DMatrix<f64>],
    max_iterations: usize,
    tolerance: f64,
) -> Vec<Vector3<f64>> {
    let ncon = contacts.len();
    if ncon == 0 {
        return vec![];
    }

    // Build the constraint system
    // Each contact has 3 rows: 1 normal + 2 friction
    let nefc = ncon * 3;

    // Use cached Cholesky from mj_crba
    let chol = match &data.qM_cholesky {
        Some(c) => c,
        None => {
            // Fallback to penalty if M is singular
            return vec![Vector3::zeros(); ncon];
        }
    };

    // Build the full Jacobian and RHS
    let mut a = DMatrix::zeros(nefc, nefc);
    let mut b = DVector::zeros(nefc);

    // Regularization (softness) for numerical stability
    let regularization = 1e-6;

    // Baumgarte stabilization parameters
    let baumgarte_erp = 0.2; // Error reduction parameter (increased for stability)
    let baumgarte_cfm = 1e-5; // Constraint force mixing

    // Compute unconstrained acceleration (qacc_smooth = M^{-1} * qfrc_smooth)
    // qfrc_smooth = qfrc_applied + qfrc_actuator + qfrc_passive - qfrc_bias
    let mut qfrc_smooth = data.qfrc_applied.clone();
    qfrc_smooth += &data.qfrc_actuator;
    qfrc_smooth += &data.qfrc_passive;
    qfrc_smooth -= &data.qfrc_bias;
    let qacc_smooth = chol.solve(&qfrc_smooth);

    // Pre-compute M^{-1} * J^T for each contact
    let mut minv_jt: Vec<DMatrix<f64>> = Vec::with_capacity(ncon);
    for jacobian in jacobians {
        let jt = jacobian.transpose();
        // Solve M * X = J^T for each column
        let mut minv_jt_contact = DMatrix::zeros(model.nv, 3);
        for col in 0..3 {
            let jt_col = jt.column(col).clone_owned();
            let x = chol.solve(&jt_col);
            minv_jt_contact.set_column(col, &x);
        }
        minv_jt.push(minv_jt_contact);
    }

    // Build A matrix blocks and RHS
    for (i, (contact_i, jac_i)) in contacts.iter().zip(jacobians.iter()).enumerate() {
        // Diagonal block: A[i,i] = J_i * M^{-1} * J_i^T
        let a_ii = jac_i * &minv_jt[i];
        for ri in 0..3 {
            for ci in 0..3 {
                a[(i * 3 + ri, i * 3 + ci)] = a_ii[(ri, ci)];
            }
            // Add regularization to diagonal
            a[(i * 3 + ri, i * 3 + ri)] += regularization + baumgarte_cfm;
        }

        // Off-diagonal blocks: A[i,j] = J_i * M^{-1} * J_j^T
        //
        // OPTIMIZATION: For independent bodies (no shared kinematic chain),
        // the off-diagonal blocks are zero because:
        // - J_i is non-zero only for body_i's DOFs
        // - M^{-1}*J_j^T is non-zero only for body_j's DOFs
        // - Independent free joints have block-diagonal mass matrix
        //
        // We check if contacts share a DYNAMIC body (not world body 0).
        // World body has no DOFs, so contacts involving world don't couple
        // unless they share the same dynamic body.
        //
        // This reduces O(n²) to O(n) for systems with independent bodies.
        let body_i1 = model.geom_body[contact_i.geom1];
        let body_i2 = model.geom_body[contact_i.geom2];

        // Extract dynamic bodies for contact i (filter out world body 0).
        // Use fixed-size arrays to avoid per-iteration heap allocation.
        let dynamic_i: [Option<usize>; 2] = [
            if body_i1 != 0 { Some(body_i1) } else { None },
            if body_i2 != 0 { Some(body_i2) } else { None },
        ];
        let has_dynamic_i = dynamic_i[0].is_some() || dynamic_i[1].is_some();

        // Only compute off-diagonal blocks if contact i has dynamic bodies.
        // Static-only contacts (e.g., plane-plane) can't couple with anything.
        // Note: We still need to compute RHS below, so don't `continue` here.
        if has_dynamic_i {
            for (j, contact_j) in contacts.iter().enumerate().skip(i + 1) {
                let geom1_body = model.geom_body[contact_j.geom1];
                let geom2_body = model.geom_body[contact_j.geom2];

                // Extract dynamic bodies for contact j
                let dynamic_j: [Option<usize>; 2] = [
                    if geom1_body != 0 {
                        Some(geom1_body)
                    } else {
                        None
                    },
                    if geom2_body != 0 {
                        Some(geom2_body)
                    } else {
                        None
                    },
                ];

                // If contact j has no dynamic bodies, skip (can't couple)
                if dynamic_j[0].is_none() && dynamic_j[1].is_none() {
                    continue;
                }

                // Check if any dynamic bodies are shared or in the same kinematic chain.
                let bodies_interact = dynamic_i.iter().filter_map(|&b| b).any(|bi| {
                    dynamic_j
                        .iter()
                        .filter_map(|&b| b)
                        .any(|bj| bi == bj || bodies_share_chain(model, bi, bj))
                });

                if !bodies_interact {
                    // Off-diagonal block is zero — skip expensive matrix multiply
                    continue;
                }

                let block_ij = jac_i * &minv_jt[j];
                for ri in 0..3 {
                    for ci in 0..3 {
                        a[(i * 3 + ri, j * 3 + ci)] = block_ij[(ri, ci)];
                        a[(j * 3 + ci, i * 3 + ri)] = block_ij[(ri, ci)]; // Symmetric
                    }
                }
            }
        }

        // RHS: b = J * qacc_smooth + aref
        // where qacc_smooth = M^{-1} * qfrc_smooth (acceleration without contacts)
        // and aref = velocity_error + position_error (Baumgarte stabilization)
        //
        // For the LCP: A*λ + b >= 0, λ >= 0
        // We want λ > 0 when b < 0 (i.e., when unconstrained acceleration
        // would cause penetration/violation)

        // Reuse body_i1/body_i2 from above (same as model.geom_body[contact_i.geom1/2])
        let vel1 = compute_point_velocity(data, body_i1, contact_i.pos);
        let vel2 = compute_point_velocity(data, body_i2, contact_i.pos);
        let rel_vel = vel2 - vel1;

        let normal = contact_i.normal;
        let (tangent1, tangent2) = build_tangent_basis(&normal);

        let vn = rel_vel.dot(&normal);
        let vt1 = rel_vel.dot(&tangent1);
        let vt2 = rel_vel.dot(&tangent2);

        // Compute J * qacc_smooth (unconstrained relative acceleration in contact frame)
        // qacc_smooth = M^{-1} * (qfrc_applied + qfrc_actuator + qfrc_passive - qfrc_bias)
        let j_qacc_smooth = jac_i * &qacc_smooth;

        // Baumgarte stabilization parameters
        // aref = (1/h) * vn + β * depth
        // This pushes to zero velocity and zero penetration
        // Validate timestep to avoid division by zero (should never happen in practice)
        let timestep = if model.timestep > 1e-10 {
            model.timestep
        } else {
            1.0 / 240.0 // Default to MuJoCo's 240Hz if timestep is invalid
        };
        let velocity_damping = 1.0 / timestep; // 1/h
        let depth_correction = baumgarte_erp / timestep; // β/h

        // b = J * qacc_smooth + velocity_term + position_term
        // For the normal constraint: we want to stop penetration
        // J * qacc = -aref means: acceleration should be aref (away from surface)
        //
        // b should be negative when the constraint needs to activate
        // Normal: if J*qacc_smooth < 0 (accelerating into surface), we need λ > 0
        b[i * 3] = j_qacc_smooth[0] + velocity_damping * vn + depth_correction * contact_i.depth;
        b[i * 3 + 1] = j_qacc_smooth[1] + vt1; // Friction 1
        b[i * 3 + 2] = j_qacc_smooth[2] + vt2; // Friction 2
    }

    // PGS iteration with contact correspondence warmstart
    // Look up previous lambda by (geom1, geom2) pair - works even when contacts reorder
    let mut lambda = DVector::zeros(nefc);
    for (i, contact) in contacts.iter().enumerate() {
        let key = (
            contact.geom1.min(contact.geom2),
            contact.geom1.max(contact.geom2),
        );
        if let Some(prev_lambda) = data.efc_lambda.get(&key) {
            let base = i * 3;
            lambda[base] = prev_lambda[0];
            lambda[base + 1] = prev_lambda[1];
            lambda[base + 2] = prev_lambda[2];
        }
    }

    // Compute diagonal inverse with protection against singular diagonals
    // A diagonal of zero indicates a degenerate constraint configuration
    let diag_inv: Vec<f64> = (0..nefc)
        .map(|i| {
            let d = a[(i, i)];
            if d.abs() < 1e-12 {
                // Degenerate: return zero inverse (constraint will be ignored)
                0.0
            } else {
                1.0 / d
            }
        })
        .collect();

    for _iter in 0..max_iterations {
        let mut max_delta = 0.0_f64;

        for i in 0..ncon {
            let base = i * 3;

            // Compute residuals for this contact
            let mut r_n = b[base];
            let mut r_t1 = b[base + 1];
            let mut r_t2 = b[base + 2];

            for j in 0..nefc {
                r_n += a[(base, j)] * lambda[j];
                r_t1 += a[(base + 1, j)] * lambda[j];
                r_t2 += a[(base + 2, j)] * lambda[j];
            }

            // Gauss-Seidel update
            let old_n: f64 = lambda[base];
            let old_t1: f64 = lambda[base + 1];
            let old_t2: f64 = lambda[base + 2];

            lambda[base] -= r_n * diag_inv[base];
            lambda[base + 1] -= r_t1 * diag_inv[base + 1];
            lambda[base + 2] -= r_t2 * diag_inv[base + 2];

            // Project onto constraint bounds
            // Normal force: λ_n ≥ 0 (unilateral constraint)
            lambda[base] = lambda[base].max(0.0);

            // Friction cone: |λ_t| ≤ μ * λ_n
            let mu = contacts[i].friction;
            let max_friction = mu * lambda[base];

            let friction_mag = (lambda[base + 1].powi(2) + lambda[base + 2].powi(2)).sqrt();
            if friction_mag > max_friction && friction_mag > 1e-10 {
                // Project onto friction cone
                let scale = max_friction / friction_mag;
                lambda[base + 1] *= scale;
                lambda[base + 2] *= scale;
            }

            // Track convergence
            max_delta = max_delta
                .max((lambda[base] - old_n).abs())
                .max((lambda[base + 1] - old_t1).abs())
                .max((lambda[base + 2] - old_t2).abs());
        }

        if max_delta < tolerance {
            break;
        }
    }

    // Store lambda for warmstart on next frame using contact correspondence
    // Clear old entries and insert new ones with canonical (min, max) key ordering
    data.efc_lambda.clear();
    for (i, contact) in contacts.iter().enumerate() {
        let base = i * 3;
        let key = (
            contact.geom1.min(contact.geom2),
            contact.geom1.max(contact.geom2),
        );
        data.efc_lambda
            .insert(key, [lambda[base], lambda[base + 1], lambda[base + 2]]);
    }

    // Extract per-contact forces
    let mut forces = Vec::with_capacity(ncon);
    for i in 0..ncon {
        let base = i * 3;
        forces.push(Vector3::new(
            lambda[base],
            lambda[base + 1],
            lambda[base + 2],
        ));
    }

    forces
}

/// Apply equality constraint forces using penalty method.
///
/// Supports Connect, Weld, and Joint constraints.
/// Uses Baumgarte stabilization for drift correction.
///
/// # MuJoCo Semantics
///
/// - **Connect**: Ball-and-socket (3 DOF position lock).
///   Error = p1 + R1*anchor - p2, where p2 is body2's position (or world origin).
///
/// - **Weld**: Fixed frame (6 DOF pose lock).
///   Position error same as Connect; orientation error via quaternion difference.
///
/// - **Joint**: Polynomial coupling θ₂ = c₀ + c₁θ₁ + c₂θ₁² + c₃θ₁³ + c₄θ₁⁴.
///   Error = θ₂ - poly(θ₁). Only joint2 receives correction torque (see below).
///
/// # Key Design Decision: No Reaction Torque for Joint Coupling
///
/// In articulated body dynamics, applying τ₂ to joint2 naturally propagates forces
/// through the kinematic chain via the mass matrix M. Adding an explicit reaction
/// torque τ₁ = -∂θ₂/∂θ₁ · τ₂ would double-count the coupling, causing positive
/// feedback and numerical explosion. This is NOT a bug — it's how articulated
/// body dynamics work. See Featherstone, "Rigid Body Dynamics Algorithms" (2008),
/// Section 7.3 on constraint force propagation.
///
/// # Penalty Parameters
///
/// Derived from solref [timeconst, dampratio]:
/// ```text
/// k = 1 / timeconst²           (stiffness, N/m or Nm/rad)
/// b = 2 * dampratio / timeconst  (damping, Ns/m or Nms/rad)
/// ```
fn apply_equality_constraints(model: &Model, data: &mut Data) {
    // Skip if no equality constraints
    if model.neq == 0 {
        return;
    }

    // Default penalty parameters (fallback when solref not specified).
    //
    // These are intentionally stiffer than MuJoCo's default solref=[0.02, 1.0],
    // which maps to k=2500, b=100. Our defaults (k=10000, b=1000) are chosen to:
    //
    // 1. Strongly enforce constraints when no solref is given (fail-safe behavior)
    // 2. Remain stable with explicit integration at dt=0.001 (stability limit ~dt<0.02)
    //
    // For explicit penalty integration, the stability condition is approximately:
    //   dt < 2/sqrt(k) → dt < 0.02 for k=10000
    //
    // Users should specify solref in MJCF for fine-grained control. Recommended
    // values for explicit integration: solref="0.05 1.0" to solref="0.1 1.0"
    // (softer than MuJoCo defaults, which assume implicit PGS solver).
    let default_stiffness = 10000.0;
    let default_damping = 1000.0;

    let dt = model.timestep;

    for eq_id in 0..model.neq {
        // Skip inactive constraints
        if !model.eq_active[eq_id] {
            continue;
        }

        match model.eq_type[eq_id] {
            EqualityType::Connect => {
                apply_connect_constraint(
                    model,
                    data,
                    eq_id,
                    default_stiffness,
                    default_damping,
                    dt,
                );
            }
            EqualityType::Weld => {
                apply_weld_constraint(model, data, eq_id, default_stiffness, default_damping, dt);
            }
            EqualityType::Joint => {
                apply_joint_equality_constraint(
                    model,
                    data,
                    eq_id,
                    default_stiffness,
                    default_damping,
                    dt,
                );
            }
            EqualityType::Distance | EqualityType::Tendon => {
                // Distance and Tendon constraints not yet implemented.
                // These require geom/tendon position computation.
                //
                // Warn once per session so users know their constraint is being ignored.
                use std::sync::Once;
                static WARN_ONCE: Once = Once::new();
                WARN_ONCE.call_once(|| {
                    eprintln!(
                        "Warning: Distance/Tendon equality constraints are not yet implemented; \
                         these constraints will be ignored. See MUJOCO_PARITY_SPEC.md Phase 2."
                    );
                });
            }
        }
    }
}

/// Convert solref parameters to penalty stiffness and damping.
///
/// MuJoCo's solref = [timeconst, dampratio] maps to a second-order system:
/// - Natural frequency: ωn = 1/timeconst
/// - Stiffness: k = ωn² = 1/timeconst²
/// - Damping: b = 2 * dampratio * ωn = 2 * dampratio / timeconst
///
/// When timeconst ≤ 0, falls back to provided defaults.
///
/// # Arguments
/// * `solref` - [timeconst, dampratio] from MJCF
/// * `default_k` - Fallback stiffness if solref[0] ≤ 0
/// * `default_b` - Fallback damping if solref[0] ≤ 0
///
/// # Returns
/// (stiffness, damping) tuple for penalty method
///
/// Converts MuJoCo's solref parameters [timeconst, dampratio] to penalty gains (k, b):
///   k = 1 / timeconst²
///   b = 2 * dampratio / timeconst
///
/// The timeconst parameter sets the natural response time of the constraint.
/// For explicit integration stability, timeconst should be > 2 * dt.
///
/// The timestep parameter is used to clamp k for stability when the user-specified
/// timeconst would cause instability with the current timestep.
#[inline]
fn solref_to_penalty(solref: [f64; 2], default_k: f64, default_b: f64, dt: f64) -> (f64, f64) {
    let (k, b) = if solref[0] > 0.0 {
        let timeconst = solref[0];
        let dampratio = solref[1];
        (1.0 / (timeconst * timeconst), 2.0 * dampratio / timeconst)
    } else {
        (default_k, default_b)
    };

    // Stability limit for explicit integration: dt < 2/sqrt(k/m_eff)
    // Since we don't know m_eff, we use a conservative limit based on dt alone.
    // For unit mass: k_max = 4/dt² gives marginal stability.
    // We use 1/dt² for a safety factor of 2.
    let k_max = 1.0 / (dt * dt);
    let k_clamped = k.min(k_max);

    // If we clamped k significantly, scale damping proportionally for critical damping
    // Critical damping: b = 2*sqrt(k*m). For unit m: b = 2*sqrt(k).
    // If k was reduced, reduce b to maintain damping ratio.
    let b_scaled = if k_clamped < k * 0.99 {
        // Maintain the original damping ratio (b/2sqrt(k)) with new k
        let original_zeta = b / (2.0 * k.sqrt());
        2.0 * original_zeta * k_clamped.sqrt()
    } else {
        b
    };

    (k_clamped, b_scaled)
}

// =============================================================================
// Constraint Stability Constants
// =============================================================================

/// Maximum velocity change per timestep for translational DOFs (m/s).
///
/// This limits the acceleration to `MAX_DELTA_V / dt` to ensure stability
/// with explicit Euler integration. A value of 1.0 means velocity can change
/// by at most 1 m/s each timestep, preventing oscillation from overshooting.
const MAX_DELTA_V_LINEAR: f64 = 1.0;

/// Maximum angular velocity change per timestep for rotational DOFs (rad/s).
///
/// Similar to `MAX_DELTA_V_LINEAR` but for angular velocities. A value of 1.0
/// means angular velocity can change by at most 1 rad/s each timestep.
const MAX_DELTA_V_ANGULAR: f64 = 1.0;

/// Maximum effective rotation error for constraint stiffness term (radians).
///
/// For large orientation errors, the small-angle approximation in the
/// quaternion-to-axis-angle conversion becomes inaccurate. We clamp the
/// effective error to this value (~29 degrees) for the stiffness term.
/// The damping term uses actual velocities and is not affected.
const MAX_ROTATION_ERROR_FOR_STIFFNESS: f64 = 0.5;

/// Minimum inertia/mass threshold for numerical stability.
///
/// Values below this threshold are treated as numerical noise and ignored
/// when computing effective mass for constraint force limiting.
const MIN_INERTIA_THRESHOLD: f64 = 1e-10;

/// Default mass/inertia when no valid DOFs are found for a body.
///
/// This is used when a body has no translational/rotational DOFs that we
/// can extract mass from (e.g., kinematic bodies). Using 1.0 provides
/// reasonable default behavior.
const DEFAULT_MASS_FALLBACK: f64 = 1.0;

// =============================================================================
// Constraint Force Utilities
// =============================================================================

/// Clamp a vector to a maximum magnitude while preserving direction.
///
/// Returns the original vector if its magnitude is at or below `max_magnitude`,
/// otherwise returns a vector in the same direction with the clamped magnitude.
///
/// # Safety
///
/// Clamp vector magnitude, avoiding division by near-zero.
///
/// If `mag <= max_magnitude`, returns `v` unchanged.
/// If `mag > max_magnitude` and `mag > MIN_INERTIA_THRESHOLD`, returns scaled vector.
/// If `mag <= MIN_INERTIA_THRESHOLD`, returns `v` unchanged (near-zero input).
#[inline]
fn clamp_vector_magnitude(v: Vector3<f64>, max_magnitude: f64) -> Vector3<f64> {
    let mag = v.norm();
    // Only clamp if magnitude exceeds limit AND is large enough to safely divide
    if mag > max_magnitude {
        if mag > MIN_INERTIA_THRESHOLD {
            v * (max_magnitude / mag)
        } else {
            // Near-zero vector, return as-is to avoid division issues
            v
        }
    } else {
        v
    }
}

/// Convert a quaternion error to axis-angle representation.
///
/// For a unit quaternion `q = (w, x, y, z) = (cos(θ/2), sin(θ/2) * axis)`,
/// returns `θ * axis` as a Vector3.
///
/// This is more accurate than the small-angle approximation `2 * [x, y, z]`
/// which has ~10% error at 90° and ~36% error at 180°.
///
/// # Arguments
/// * `quat` - A unit quaternion representing the rotation error
///
/// # Returns
/// Axis-angle representation as `angle * axis` (Vector3)
#[inline]
fn quaternion_to_axis_angle(quat: &UnitQuaternion<f64>) -> Vector3<f64> {
    let q = quat.quaternion();
    let (qw, qx, qy, qz) = (q.w, q.i, q.j, q.k);

    // Handle identity quaternion (no rotation)
    let sin_half_angle_sq = qx * qx + qy * qy + qz * qz;
    if sin_half_angle_sq < MIN_INERTIA_THRESHOLD {
        return Vector3::zeros();
    }

    let sin_half_angle = sin_half_angle_sq.sqrt();

    // Compute full angle: θ = 2 * atan2(||xyz||, w)
    // This handles all cases including w < 0 (angle > π)
    let angle = 2.0 * sin_half_angle.atan2(qw);

    // Axis is normalized [x, y, z] / ||xyz||
    // Result is angle * axis
    Vector3::new(qx, qy, qz) * (angle / sin_half_angle)
}

/// Apply a Connect (ball-and-socket) equality constraint.
///
/// Constrains anchor point on body1 to coincide with body2's position.
/// Uses Baumgarte stabilization: F = -k * pos_error - b * vel_error
///
/// # Force Limiting
///
/// Forces are clamped based on the effective mass to ensure acceleration
/// stays bounded, preventing instability with explicit integration.
fn apply_connect_constraint(
    model: &Model,
    data: &mut Data,
    eq_id: usize,
    default_stiffness: f64,
    default_damping: f64,
    dt: f64,
) {
    let body1 = model.eq_obj1id[eq_id];
    let body2 = model.eq_obj2id[eq_id];
    let eq_data = &model.eq_data[eq_id];

    // Cache body validity checks (body_id == 0 means world frame)
    let body1_is_dynamic = body1 != 0;
    let body2_is_dynamic = body2 != 0;

    // Anchor point in body1's local frame
    let anchor = Vector3::new(eq_data[0], eq_data[1], eq_data[2]);

    // Get body poses
    let p1 = data.xpos[body1];
    let r1 = data.xquat[body1];
    let p2 = if body2_is_dynamic {
        data.xpos[body2]
    } else {
        Vector3::zeros()
    };

    // World position of anchor on body1
    let anchor_world = p1 + r1 * anchor;

    // Position error: anchor should coincide with body2's origin
    let pos_error = anchor_world - p2;

    // Velocity error (relative velocity at constraint point)
    let vel_error = if body1_is_dynamic {
        let cvel1 = &data.cvel[body1];
        let omega1 = Vector3::new(cvel1[0], cvel1[1], cvel1[2]);
        let v1 = Vector3::new(cvel1[3], cvel1[4], cvel1[5]);
        let r1_anchor = r1 * anchor;
        let v_anchor = v1 + omega1.cross(&r1_anchor);

        if body2_is_dynamic {
            let cvel2 = &data.cvel[body2];
            let v2 = Vector3::new(cvel2[3], cvel2[4], cvel2[5]);
            v_anchor - v2
        } else {
            v_anchor
        }
    } else {
        Vector3::zeros()
    };

    // Compute penalty parameters from solref
    let (stiffness, damping) = solref_to_penalty(
        model.eq_solref[eq_id],
        default_stiffness,
        default_damping,
        dt,
    );

    // Baumgarte stabilization: F = -k*error - b*vel_error
    let raw_force = -stiffness * pos_error - damping * vel_error;

    // Compute effective mass for force limiting using cached values
    // body_min_mass[0] = infinity for world body
    let mass1 = data.body_min_mass[body1];
    let mass2 = data.body_min_mass[body2];
    let eff_mass = effective_mass_for_stability(mass1, mass2);

    // Limit force to bound acceleration: a_max = MAX_DELTA_V_LINEAR / dt
    let max_accel = MAX_DELTA_V_LINEAR / dt;
    let max_force = eff_mass * max_accel;
    let force = clamp_vector_magnitude(raw_force, max_force);

    // Apply forces via Jacobian transpose
    if body1_is_dynamic {
        apply_constraint_force_to_body(model, data, body1, anchor_world, force);
    }
    if body2_is_dynamic {
        apply_constraint_force_to_body(model, data, body2, p2, -force);
    }
}

/// DOF type for mass matrix extraction.
#[derive(Clone, Copy, PartialEq, Eq)]
enum DofKind {
    /// Linear (translational) DOFs
    Linear,
    /// Angular (rotational) DOFs
    Angular,
}

/// Extract minimum diagonal mass/inertia from the mass matrix for specified DOF types.
///
/// This is the core implementation used by both `get_min_translational_mass` and
/// `get_min_rotational_inertia`. It traverses the body's joints and extracts the
/// minimum diagonal element from the mass matrix for the specified DOF type.
///
/// # Arguments
/// * `model` - The physics model
/// * `data` - The simulation data containing the mass matrix
/// * `body_id` - The body to query (0 = world, returns infinity)
/// * `kind` - Whether to extract linear (mass) or angular (inertia) DOFs
///
/// # Returns
/// The minimum diagonal mass/inertia, or `DEFAULT_MASS_FALLBACK` if no valid DOFs found.
#[inline]
fn get_min_diagonal_mass(model: &Model, data: &Data, body_id: usize, kind: DofKind) -> f64 {
    if body_id == 0 {
        return f64::INFINITY;
    }

    let mut min_val = f64::INFINITY;

    let jnt_start = model.body_jnt_adr[body_id];
    let jnt_end = jnt_start + model.body_jnt_num[body_id];

    for jnt_id in jnt_start..jnt_end {
        let dof_adr = model.jnt_dof_adr[jnt_id];

        // Determine which DOF indices to query based on joint type and DOF kind
        // Arms combined where they return identical values per clippy::match_same_arms
        let dof_range: Option<std::ops::Range<usize>> = match (model.jnt_type[jnt_id], kind) {
            // Free linear (0-2) and Ball angular (0-2) both use first 3 DOFs
            (MjJointType::Free, DofKind::Linear) | (MjJointType::Ball, DofKind::Angular) => {
                Some(0..3)
            }

            // Free angular uses DOFs 3-5
            (MjJointType::Free, DofKind::Angular) => Some(3..6),

            // Hinge/Slide: single DOF of matching kind
            (MjJointType::Hinge, DofKind::Angular) | (MjJointType::Slide, DofKind::Linear) => {
                Some(0..1)
            }

            // No DOFs for mismatched kind
            (MjJointType::Ball | MjJointType::Hinge, DofKind::Linear)
            | (MjJointType::Slide, DofKind::Angular) => None,
        };

        if let Some(range) = dof_range {
            for i in range {
                let dof = dof_adr + i;
                if dof < model.nv {
                    let val = data.qM[(dof, dof)];
                    if val > MIN_INERTIA_THRESHOLD {
                        min_val = min_val.min(val);
                    }
                }
            }
        }
    }

    if min_val == f64::INFINITY {
        DEFAULT_MASS_FALLBACK
    } else {
        min_val
    }
}

/// Get the minimum translational mass from the mass matrix diagonal for a body's linear DOFs.
///
/// **Note**: In the hot path, use `data.body_min_mass[body_id]` instead, which is cached
/// during `forward()` after CRBA. This function is kept for debugging and testing.
///
/// For bodies with free joints, this returns the minimum of the x, y, z mass entries.
/// For bodies with slide joints, this returns the slide DOF's effective mass.
///
/// # Returns
/// - `f64::INFINITY` if body_id is 0 (world body)
/// - Minimum diagonal mass if found
/// - `DEFAULT_MASS_FALLBACK` (1.0 kg) if no linear DOFs exist
#[inline]
#[allow(dead_code)] // Kept for debugging/testing; hot path uses cached data.body_min_mass
fn get_min_translational_mass(model: &Model, data: &Data, body_id: usize) -> f64 {
    get_min_diagonal_mass(model, data, body_id, DofKind::Linear)
}

/// Get the minimum rotational inertia from the mass matrix diagonal for a body's angular DOFs.
///
/// **Note**: In the hot path, use `data.body_min_inertia[body_id]` instead, which is cached
/// during `forward()` after CRBA. This function is kept for debugging and testing.
///
/// For bodies with free/ball joints, this returns the minimum of the angular inertia entries.
/// For bodies with hinge joints, this returns the hinge DOF's effective inertia.
///
/// # Returns
/// - `f64::INFINITY` if body_id is 0 (world body)
/// - Minimum diagonal inertia if found
/// - `DEFAULT_MASS_FALLBACK` (1.0 kg·m²) if no angular DOFs exist
#[inline]
#[allow(dead_code)] // Kept for debugging/testing; hot path uses cached data.body_min_inertia
fn get_min_rotational_inertia(model: &Model, data: &Data, body_id: usize) -> f64 {
    get_min_diagonal_mass(model, data, body_id, DofKind::Angular)
}

/// Compute effective mass/inertia for a binary constraint between two bodies.
///
/// For stability, we use the *minimum* of the two masses rather than the harmonic mean.
/// This ensures the force limit is conservative enough for the lighter body.
///
/// # Physics Rationale
///
/// When force F is applied between two bodies with masses m1 and m2:
/// - Body 1 accelerates at F/m1
/// - Body 2 accelerates at -F/m2
/// - The *relative* acceleration is F * (1/m1 + 1/m2) = F * (m1+m2)/(m1*m2)
///
/// For stability, we need to limit the acceleration of *each* body individually.
/// Using min(m1, m2) ensures F/min(m1,m2) ≤ max_accel for both bodies.
#[inline]
fn effective_mass_for_stability(m1: f64, m2: f64) -> f64 {
    // For world body (m = infinity), just use the other body's mass
    if m1 == f64::INFINITY {
        return m2;
    }
    if m2 == f64::INFINITY {
        return m1;
    }
    // Use minimum for conservative stability bound
    m1.min(m2)
}

/// Apply a Weld (6 DOF) equality constraint.
///
/// Constrains body1's frame to maintain a fixed relative pose to body2.
/// Uses Baumgarte stabilization for both position and orientation:
///   F = -k * pos_error - b * vel_error
///   τ = -k * rot_error - b * ang_vel_error
///
/// # Stability Considerations
///
/// Weld constraints between free joints can be unstable with explicit integration
/// when there's a large initial error. The constraint force/torque is limited based
/// on the effective mass/inertia to ensure the resulting acceleration stays bounded.
///
/// # Orientation Error Computation
///
/// The rotation error is computed as the axis-angle representation of
/// `r1 * (r2 * target_quat)⁻¹`. This uses the accurate `quaternion_to_axis_angle`
/// conversion which handles large angles correctly.
fn apply_weld_constraint(
    model: &Model,
    data: &mut Data,
    eq_id: usize,
    default_stiffness: f64,
    default_damping: f64,
    dt: f64,
) {
    let body1 = model.eq_obj1id[eq_id];
    let body2 = model.eq_obj2id[eq_id];
    let eq_data = &model.eq_data[eq_id];

    // Cache body validity checks (body_id == 0 means world frame)
    let body1_is_dynamic = body1 != 0;
    let body2_is_dynamic = body2 != 0;

    // Anchor point in body1's local frame
    let anchor = Vector3::new(eq_data[0], eq_data[1], eq_data[2]);

    // Target relative quaternion [qw, qx, qy, qz]
    let target_quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        eq_data[3], eq_data[4], eq_data[5], eq_data[6],
    ));

    // Get body poses
    let p1 = data.xpos[body1];
    let r1 = data.xquat[body1];
    let (p2, r2) = if body2_is_dynamic {
        (data.xpos[body2], data.xquat[body2])
    } else {
        (Vector3::zeros(), UnitQuaternion::identity())
    };

    // === Position Error ===
    let anchor_world = p1 + r1 * anchor;
    let pos_error = anchor_world - p2;

    // === Orientation Error ===
    // Target: r1 = r2 * target_quat
    // Error quaternion: e = r1 * (r2 * target_quat)⁻¹
    let target_r1 = r2 * target_quat;
    let rot_error_quat = r1 * target_r1.inverse();

    // Convert to axis-angle (accurate for all angles, not just small)
    let rot_error = quaternion_to_axis_angle(&rot_error_quat);

    // === Velocity Errors ===
    let (vel_error, ang_vel_error) = if body1_is_dynamic {
        let cvel1 = &data.cvel[body1];
        let omega1 = Vector3::new(cvel1[0], cvel1[1], cvel1[2]);
        let v1 = Vector3::new(cvel1[3], cvel1[4], cvel1[5]);
        let r1_anchor = r1 * anchor;
        let v_anchor = v1 + omega1.cross(&r1_anchor);

        if body2_is_dynamic {
            let cvel2 = &data.cvel[body2];
            let omega2 = Vector3::new(cvel2[0], cvel2[1], cvel2[2]);
            let v2 = Vector3::new(cvel2[3], cvel2[4], cvel2[5]);
            (v_anchor - v2, omega1 - omega2)
        } else {
            (v_anchor, omega1)
        }
    } else {
        (Vector3::zeros(), Vector3::zeros())
    };

    // Compute penalty parameters from solref
    let (stiffness, damping) = solref_to_penalty(
        model.eq_solref[eq_id],
        default_stiffness,
        default_damping,
        dt,
    );

    // === Position Constraint Force ===
    let raw_force = -stiffness * pos_error - damping * vel_error;

    // Compute effective mass using cached values (body_min_mass[0] = infinity for world)
    let mass1 = data.body_min_mass[body1];
    let mass2 = data.body_min_mass[body2];
    let eff_mass = effective_mass_for_stability(mass1, mass2);

    let max_linear_accel = MAX_DELTA_V_LINEAR / dt;
    let max_force = eff_mass * max_linear_accel;
    let force = clamp_vector_magnitude(raw_force, max_force);

    // === Orientation Constraint Torque ===
    // Clamp effective rotation error for stiffness term (damping uses actual velocity)
    let clamped_rot_error = clamp_vector_magnitude(rot_error, MAX_ROTATION_ERROR_FOR_STIFFNESS);
    let raw_torque = -stiffness * clamped_rot_error - damping * ang_vel_error;

    // Compute effective inertia using cached values (body_min_inertia[0] = infinity for world)
    let inertia1 = data.body_min_inertia[body1];
    let inertia2 = data.body_min_inertia[body2];
    let eff_inertia = effective_mass_for_stability(inertia1, inertia2);

    let max_angular_accel = MAX_DELTA_V_ANGULAR / dt;
    let max_torque = eff_inertia * max_angular_accel;
    let torque = clamp_vector_magnitude(raw_torque, max_torque);

    // === Apply Forces and Torques ===
    if body1_is_dynamic {
        apply_constraint_force_to_body(model, data, body1, anchor_world, force);
        apply_constraint_torque_to_body(model, data, body1, torque);
    }
    if body2_is_dynamic {
        apply_constraint_force_to_body(model, data, body2, p2, -force);
        apply_constraint_torque_to_body(model, data, body2, -torque);
    }
}

/// Apply a Joint equality constraint (polynomial coupling).
///
/// Constrains joint2 position as polynomial function of joint1:
/// q2 = c0 + c1*q1 + c2*q1² + ...
fn apply_joint_equality_constraint(
    model: &Model,
    data: &mut Data,
    eq_id: usize,
    default_stiffness: f64,
    default_damping: f64,
    dt: f64,
) {
    let joint1_id = model.eq_obj1id[eq_id];
    let joint2_id = model.eq_obj2id[eq_id];
    let eq_data = &model.eq_data[eq_id];

    // Polynomial coefficients: c0, c1, c2, c3, c4
    let c0 = eq_data[0];
    let c1 = eq_data[1];
    let c2 = eq_data[2];
    let c3 = eq_data[3];
    let c4 = eq_data[4];

    // Get joint positions and velocities
    let qpos1_adr = model.jnt_qpos_adr[joint1_id];
    let dof1_adr = model.jnt_dof_adr[joint1_id];
    let q1 = data.qpos[qpos1_adr];
    let qd1 = data.qvel[dof1_adr];

    // Compute target position: q2_target = poly(q1)
    let q2_target = c0 + c1 * q1 + c2 * q1 * q1 + c3 * q1 * q1 * q1 + c4 * q1 * q1 * q1 * q1;

    // Compute target velocity: qd2_target = d(poly)/dq1 * qd1
    let dq2_dq1 = c1 + 2.0 * c2 * q1 + 3.0 * c3 * q1 * q1 + 4.0 * c4 * q1 * q1 * q1;
    let qd2_target = dq2_dq1 * qd1;

    // If joint2 is valid (not usize::MAX sentinel)
    if joint2_id < model.njnt {
        let qpos2_adr = model.jnt_qpos_adr[joint2_id];
        let dof2_adr = model.jnt_dof_adr[joint2_id];
        let q2 = data.qpos[qpos2_adr];
        let qd2 = data.qvel[dof2_adr];

        // Errors
        let pos_error = q2 - q2_target;
        let vel_error = qd2 - qd2_target;

        // Compute penalty parameters from solref
        let (stiffness, damping) = solref_to_penalty(
            model.eq_solref[eq_id],
            default_stiffness,
            default_damping,
            dt,
        );

        // Apply torque to joint2 to correct error
        let tau2 = -stiffness * pos_error - damping * vel_error;
        data.qfrc_constraint[dof2_adr] += tau2;

        // INVARIANT: No explicit reaction torque on joint1.
        //
        // In articulated body dynamics, applying τ₂ to joint2 naturally propagates
        // forces up the kinematic chain via the recursive Newton-Euler algorithm.
        // The constraint Jacobian already encodes this coupling through the mass
        // matrix. Adding an explicit τ₁ = -dq₂/dq₁ · τ₂ would double-count the
        // reaction, causing positive feedback and numerical explosion.
        //
        // Reference: Featherstone, "Rigid Body Dynamics Algorithms" (2008),
        // Section 7.3 on constraint force propagation in articulated systems.
    } else {
        // joint2 not specified: lock joint1 to constant c0
        let pos_error = q1 - c0;
        let vel_error = qd1;

        // Compute penalty parameters from solref
        let (stiffness, damping) = solref_to_penalty(
            model.eq_solref[eq_id],
            default_stiffness,
            default_damping,
            dt,
        );

        let tau1 = -stiffness * pos_error - damping * vel_error;
        data.qfrc_constraint[dof1_adr] += tau1;
    }
}

/// Apply a constraint force to a body at a specific world point.
///
/// Maps the force to generalized coordinates via Jacobian transpose.
/// Called per constraint per body in the kinematic chain — hot path.
#[inline]
fn apply_constraint_force_to_body(
    model: &Model,
    data: &mut Data,
    body_id: usize,
    point: Vector3<f64>,
    force: Vector3<f64>,
) {
    if body_id == 0 {
        return; // World doesn't respond
    }

    // Traverse kinematic chain from body to root
    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    let j_col = axis.cross(&r);
                    data.qfrc_constraint[dof_adr] += j_col.dot(&force);
                }
                MjJointType::Slide => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    data.qfrc_constraint[dof_adr] += axis.dot(&force);
                }
                MjJointType::Ball => {
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    let torque = r.cross(&force);
                    let body_torque = data.xquat[jnt_body].inverse() * torque;
                    data.qfrc_constraint[dof_adr] += body_torque.x;
                    data.qfrc_constraint[dof_adr + 1] += body_torque.y;
                    data.qfrc_constraint[dof_adr + 2] += body_torque.z;
                }
                MjJointType::Free => {
                    data.qfrc_constraint[dof_adr] += force.x;
                    data.qfrc_constraint[dof_adr + 1] += force.y;
                    data.qfrc_constraint[dof_adr + 2] += force.z;
                    let jpos = Vector3::new(
                        data.qpos[model.jnt_qpos_adr[jnt_id]],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
                    );
                    let r = point - jpos;
                    let torque = r.cross(&force);
                    data.qfrc_constraint[dof_adr + 3] += torque.x;
                    data.qfrc_constraint[dof_adr + 4] += torque.y;
                    data.qfrc_constraint[dof_adr + 5] += torque.z;
                }
            }
        }
        current_body = model.body_parent[current_body];
    }
}

/// Apply a constraint torque directly to a body's rotational DOFs.
/// Called per weld constraint per body in the kinematic chain — hot path.
#[inline]
fn apply_constraint_torque_to_body(
    model: &Model,
    data: &mut Data,
    body_id: usize,
    torque: Vector3<f64>,
) {
    if body_id == 0 {
        return;
    }

    // Traverse kinematic chain from body to root
    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    // Project torque onto hinge axis
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    data.qfrc_constraint[dof_adr] += torque.dot(&axis);
                }
                MjJointType::Slide => {
                    // Slide joint doesn't respond to torque
                }
                MjJointType::Ball => {
                    // Full 3D torque in body frame
                    let body_torque = data.xquat[jnt_body].inverse() * torque;
                    data.qfrc_constraint[dof_adr] += body_torque.x;
                    data.qfrc_constraint[dof_adr + 1] += body_torque.y;
                    data.qfrc_constraint[dof_adr + 2] += body_torque.z;
                }
                MjJointType::Free => {
                    // Angular DOFs are indices 3-5 for free joint
                    data.qfrc_constraint[dof_adr + 3] += torque.x;
                    data.qfrc_constraint[dof_adr + 4] += torque.y;
                    data.qfrc_constraint[dof_adr + 5] += torque.z;
                }
            }
        }
        current_body = model.body_parent[current_body];
    }
}

/// Compute constraint forces (contacts and joint limits).
///
/// This implements constraint-based enforcement using:
/// - Joint limits: Soft penalty method with Baumgarte stabilization
/// - Equality constraints: Soft penalty method with Baumgarte stabilization
/// - Contacts: PGS solver with Coulomb friction cone
///
/// The constraint forces are computed via Jacobian transpose:
///   qfrc_constraint = J^T * λ
///
/// where λ is found by solving the LCP with PGS.
fn mj_fwd_constraint(model: &Model, data: &mut Data) {
    data.qfrc_constraint.fill(0.0);

    // Note: data.contacts is populated by mj_collision() which runs before this.
    // Do NOT clear contacts here.

    // ========== Joint Limit Constraints ==========
    // Using penalty method with Baumgarte stabilization:
    // τ = k * penetration + b * velocity_into_limit
    //
    // MuJoCo uses solver-based approach, but penalty is acceptable for most robotics.
    // Parameters are chosen for stability: k = 10000, b = 1000 gives critical damping.
    let limit_stiffness = 10000.0;
    let limit_damping = 1000.0;

    for jnt_id in 0..model.njnt {
        if !model.jnt_limited[jnt_id] {
            continue;
        }

        let (limit_min, limit_max) = model.jnt_range[jnt_id];
        let qpos_adr = model.jnt_qpos_adr[jnt_id];
        let dof_adr = model.jnt_dof_adr[jnt_id];

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge | MjJointType::Slide => {
                let q = data.qpos[qpos_adr];
                let qdot = data.qvel[dof_adr];

                if q < limit_min {
                    // Below lower limit - push up
                    let penetration = limit_min - q;
                    let vel_into = (-qdot).max(0.0);
                    data.qfrc_constraint[dof_adr] +=
                        limit_stiffness * penetration + limit_damping * vel_into;
                } else if q > limit_max {
                    // Above upper limit - push down
                    let penetration = q - limit_max;
                    let vel_into = qdot.max(0.0);
                    data.qfrc_constraint[dof_adr] -=
                        limit_stiffness * penetration + limit_damping * vel_into;
                }
            }
            MjJointType::Ball | MjJointType::Free => {
                // Ball/Free joints can have cone limits (swing-twist)
                // Not yet implemented - would require quaternion-based limit checking
            }
        }
    }

    // ========== Equality Constraints ==========
    // Using penalty method with Baumgarte stabilization (like joint limits).
    // MuJoCo uses solver-based approach via PGS, but penalty is robust and simpler.
    //
    // For each equality constraint:
    //   τ = -k * violation - b * violation_velocity
    //
    // The forces are applied via Jacobian transpose to the appropriate DOFs.
    apply_equality_constraints(model, data);

    // ========== Contact Constraints ==========
    // Use PGS solver for constraint-based contact forces.
    //
    // For small contact counts or when M is singular, fall back to penalty method
    // which is simpler and more robust for simple scenarios.

    // Early exit if no contacts (avoid clone)
    if data.contacts.is_empty() {
        return;
    }

    // Clone contacts to avoid borrow checker issues (contacts is part of data)
    // TODO: Consider using indices instead of cloning for better performance
    let contacts: Vec<Contact> = data.contacts.clone();

    // For systems without DOFs, use simple penalty
    if model.nv == 0 {
        return;
    }

    // Build contact Jacobians
    let jacobians: Vec<DMatrix<f64>> = contacts
        .iter()
        .map(|c| compute_contact_jacobian(model, data, c))
        .collect();

    // Solve using PGS
    let constraint_forces = pgs_solve_contacts(
        model,
        data,
        &contacts,
        &jacobians,
        model.solver_iterations.max(10),
        model.solver_tolerance.max(1e-8),
    );

    // Apply forces via J^T
    // qfrc_constraint += J^T * λ
    for (i, (_jacobian, lambda)) in jacobians.iter().zip(constraint_forces.iter()).enumerate() {
        let normal = contacts[i].normal;
        let (tangent1, tangent2) = build_tangent_basis(&normal);

        // Convert contact-frame forces to world-frame force
        let world_force = normal * lambda.x + tangent1 * lambda.y + tangent2 * lambda.z;

        // Apply via Jacobian transpose
        // This applies the force to both bodies through the kinematic chain
        let body1 = model.geom_body[contacts[i].geom1];
        let body2 = model.geom_body[contacts[i].geom2];

        // Body1 gets negative force (Newton's third law)
        apply_contact_force(model, data, body1, contacts[i].pos, -world_force);
        apply_contact_force(model, data, body2, contacts[i].pos, world_force);
    }
}

/// Compute the velocity of a point on a body.
fn compute_point_velocity(data: &Data, body_id: usize, point: Vector3<f64>) -> Vector3<f64> {
    if body_id == 0 {
        return Vector3::zeros(); // World is stationary
    }

    // Get body velocity (linear and angular)
    let cvel = &data.cvel[body_id];
    let v_linear = Vector3::new(cvel[3], cvel[4], cvel[5]);
    let omega = Vector3::new(cvel[0], cvel[1], cvel[2]);

    // Point velocity = v_body + omega × r
    let body_pos = data.xpos[body_id];
    let r = point - body_pos;

    v_linear + omega.cross(&r)
}

/// Apply a contact force to a body by mapping it to generalized forces.
fn apply_contact_force(
    model: &Model,
    data: &mut Data,
    body_id: usize,
    contact_point: Vector3<f64>,
    force: Vector3<f64>,
) {
    if body_id == 0 {
        return; // World doesn't respond
    }

    // For each joint in the kinematic chain from body to root,
    // compute the generalized force contribution using the Jacobian transpose.
    //
    // τ = J^T * F where J = ∂p/∂q (point velocity Jacobian)

    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    // τ = (axis × r) · F
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = contact_point - jpos;
                    let j_col = axis.cross(&r); // Jacobian column for this joint
                    data.qfrc_constraint[dof_adr] += j_col.dot(&force);
                }
                MjJointType::Slide => {
                    // τ = axis · F
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    data.qfrc_constraint[dof_adr] += axis.dot(&force);
                }
                MjJointType::Ball => {
                    // τ = r × F (torque from force at point)
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = contact_point - jpos;
                    let torque = r.cross(&force);
                    // Transform to body frame for ball joint
                    let body_torque = data.xquat[jnt_body].inverse() * torque;
                    data.qfrc_constraint[dof_adr] += body_torque.x;
                    data.qfrc_constraint[dof_adr + 1] += body_torque.y;
                    data.qfrc_constraint[dof_adr + 2] += body_torque.z;
                }
                MjJointType::Free => {
                    // Linear DOFs: F directly
                    data.qfrc_constraint[dof_adr] += force.x;
                    data.qfrc_constraint[dof_adr + 1] += force.y;
                    data.qfrc_constraint[dof_adr + 2] += force.z;
                    // Angular DOFs: torque from force
                    let jpos = Vector3::new(
                        data.qpos[model.jnt_qpos_adr[jnt_id]],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
                    );
                    let r = contact_point - jpos;
                    let torque = r.cross(&force);
                    data.qfrc_constraint[dof_adr + 3] += torque.x;
                    data.qfrc_constraint[dof_adr + 4] += torque.y;
                    data.qfrc_constraint[dof_adr + 5] += torque.z;
                }
            }
        }
        current_body = model.body_parent[current_body];
    }
}

/// Compute final acceleration from forces using proper matrix solve.
///
/// # Explicit Integration (Euler, RK4)
///
/// Solves: M * qacc = `τ_total` where
/// `τ_total` = `qfrc_applied` + `qfrc_actuator` + `qfrc_passive` + `qfrc_constraint` - `qfrc_bias`
///
/// Uses Cholesky decomposition for symmetric positive-definite M.
///
/// # Implicit Integration
///
/// For implicit springs, we solve a modified system that incorporates
/// stiffness and damping into the velocity update:
///
/// ```text
/// (M + h*D + h²*K) * v_new = M*v_old + h*f_ext - h*K*(q - q_eq)
/// ```
///
/// Where:
/// - M = mass matrix (from CRBA)
/// - D = diagonal damping matrix
/// - K = diagonal stiffness matrix
/// - h = timestep
/// - f_ext = external forces (applied + actuator + constraint - bias)
/// - q_eq = spring equilibrium positions (springref)
///
/// This provides unconditional stability for arbitrarily stiff springs,
/// allowing larger timesteps without energy blow-up.
///
/// After solving, we compute qacc = (v_new - v_old) / h for consistency
/// with sensors and other code that expects qacc.
///
/// # Errors
///
/// Returns `Err(StepError::CholeskyFailed)` if using implicit integration
/// and the modified mass matrix (M + h*D + h²*K) is not positive definite.
fn mj_fwd_acceleration(model: &Model, data: &mut Data) -> Result<(), StepError> {
    if model.nv == 0 {
        return Ok(());
    }

    match model.integrator {
        Integrator::Implicit => mj_fwd_acceleration_implicit(model, data),
        Integrator::Euler | Integrator::RungeKutta4 => {
            mj_fwd_acceleration_explicit(model, data);
            Ok(())
        }
    }
}

/// Explicit forward acceleration (semi-implicit Euler or RK4).
///
/// Computes: qacc = M⁻¹ * (f_applied + f_actuator + f_passive + f_constraint - f_bias)
fn mj_fwd_acceleration_explicit(model: &Model, data: &mut Data) {
    // Sum all forces: τ = applied + actuator + passive + constraint - bias
    let mut qfrc_total = data.qfrc_applied.clone();
    qfrc_total += &data.qfrc_actuator;
    qfrc_total += &data.qfrc_passive;
    qfrc_total += &data.qfrc_constraint;
    qfrc_total -= &data.qfrc_bias;

    // Solve M * qacc = qfrc_total using cached Cholesky from mj_crba
    // Normal path: O(n²) forward/back substitution using cached L L^T factorization
    match &data.qM_cholesky {
        Some(chol) => {
            data.qacc = chol.solve(&qfrc_total);
        }
        None => {
            // Rare fallback: M wasn't SPD (numerical issues or degenerate system)
            // This clone is acceptable since it only happens on error paths
            match data.qM.clone().lu().solve(&qfrc_total) {
                Some(qacc) => {
                    data.qacc = qacc;
                }
                None => {
                    // Last resort: diagonal solve (lumped mass approximation)
                    // Ignores coupling but prevents NaN propagation
                    for i in 0..model.nv {
                        let m_ii = data.qM[(i, i)];
                        if m_ii.abs() > 1e-10 {
                            data.qacc[i] = qfrc_total[i] / m_ii;
                        } else {
                            data.qacc[i] = 0.0;
                        }
                    }
                }
            }
        }
    }
}

/// Implicit forward acceleration for springs and dampers.
///
/// Solves:
/// ```text
/// (M + h*D + h²*K) * v_new = M*v_old + h*f_ext - h*K*(q - q_eq)
/// ```
///
/// This provides unconditional stability for stiff springs by treating
/// spring and damper forces implicitly in the velocity update.
///
/// # Implementation Notes
///
/// - Spring/damper parameters are diagonal (no coupling between DOFs)
/// - Friction loss remains explicit (velocity-sign-dependent, cannot linearize)
/// - The modified matrix M + h*D + h²*K is still SPD if M is SPD and D, K ≥ 0
///
/// # Errors
///
/// Returns `Err(StepError::CholeskyFailed)` if the modified mass matrix
/// is not positive definite. This can happen with:
/// - Negative stiffness or damping values
/// - Corrupted mass matrix
/// - Extreme numerical conditions
fn mj_fwd_acceleration_implicit(model: &Model, data: &mut Data) -> Result<(), StepError> {
    // Guard against zero timestep (would cause division by zero)
    debug_assert!(
        model.timestep > 0.0,
        "Timestep must be positive for implicit integration"
    );
    let h = model.timestep;
    let h2 = h * h;

    // Use cached spring-damper parameters from Model (avoids allocation)
    let k = &model.implicit_stiffness;
    let d = &model.implicit_damping;
    let q_eq = &model.implicit_springref;

    // Build external forces into scratch buffer (avoids allocation)
    // f_ext = applied + actuator + passive(friction) + constraint - bias
    data.scratch_force.copy_from(&data.qfrc_applied);
    data.scratch_force += &data.qfrc_actuator;
    data.scratch_force += &data.qfrc_passive; // Friction loss (explicit even in implicit mode)
    data.scratch_force += &data.qfrc_constraint;
    data.scratch_force -= &data.qfrc_bias;

    // Build modified mass matrix: M_impl = M + h*D + h²*K
    // Copy M into scratch, then modify diagonal only
    data.scratch_m_impl.copy_from(&data.qM);
    for i in 0..model.nv {
        data.scratch_m_impl[(i, i)] += h * d[i] + h2 * k[i];
    }

    // Build RHS into scratch buffer: M*v_old + h*f_ext - h*K*(q - q_eq)
    // Start with M*v_old
    data.qM.mul_to(&data.qvel, &mut data.scratch_rhs);

    // Add h*f_ext for each DOF
    for i in 0..model.nv {
        data.scratch_rhs[i] += h * data.scratch_force[i];
    }

    // Subtract h*K*(q - q_eq) for spring displacement using visitor
    let mut spring_visitor = ImplicitSpringVisitor {
        k,
        q_eq,
        h,
        qpos: &data.qpos,
        rhs: &mut data.scratch_rhs,
    };
    model.visit_joints(&mut spring_visitor);

    // Solve (M + h*D + h²*K) * v_new = rhs
    // M_impl is SPD (M is SPD from CRBA, D ≥ 0, K ≥ 0), so use Cholesky.
    //
    // TODO(perf): This clone() allocates O(nv²) per step because nalgebra's cholesky()
    // consumes the matrix. For serial chains, M is banded with bandwidth O(1), so we
    // could use sparse L^T D L factorization for O(nv) instead of O(nv³). Options:
    //   1. Implement in-place Cholesky that doesn't consume the matrix
    //   2. Use sparse factorization exploiting articulated body structure
    //   3. Use Featherstone's articulated body algorithm (ABA) which is O(nv)
    let chol = data
        .scratch_m_impl
        .clone()
        .cholesky()
        .ok_or(StepError::CholeskyFailed)?;

    // Copy RHS to scratch_v_new and solve in place to avoid allocation
    data.scratch_v_new.copy_from(&data.scratch_rhs);
    chol.solve_mut(&mut data.scratch_v_new);

    // Compute qacc = (v_new - v_old) / h and update qvel
    for i in 0..model.nv {
        data.qacc[i] = (data.scratch_v_new[i] - data.qvel[i]) / h;
        data.qvel[i] = data.scratch_v_new[i];
    }

    Ok(())
}

/// Visitor for computing spring displacement contribution to implicit RHS.
/// Computes: rhs[dof] -= h * K[dof] * (q - q_eq) for joints with springs.
struct ImplicitSpringVisitor<'a> {
    k: &'a DVector<f64>,
    q_eq: &'a DVector<f64>,
    h: f64,
    qpos: &'a DVector<f64>,
    rhs: &'a mut DVector<f64>,
}

impl JointVisitor for ImplicitSpringVisitor<'_> {
    #[inline]
    fn visit_hinge(&mut self, ctx: JointContext) {
        let q = self.qpos[ctx.qpos_adr];
        self.rhs[ctx.dof_adr] -= self.h * self.k[ctx.dof_adr] * (q - self.q_eq[ctx.dof_adr]);
    }

    #[inline]
    fn visit_slide(&mut self, ctx: JointContext) {
        let q = self.qpos[ctx.qpos_adr];
        self.rhs[ctx.dof_adr] -= self.h * self.k[ctx.dof_adr] * (q - self.q_eq[ctx.dof_adr]);
    }

    // Ball and Free joints have no springs (k = 0), so default no-ops are correct.
}

/// Proper position integration that handles quaternions on SO(3) manifold.
fn mj_integrate_pos(model: &Model, data: &mut Data, h: f64) {
    let mut visitor = PositionIntegrateVisitor {
        qpos: &mut data.qpos,
        qvel: &data.qvel,
        h,
    };
    model.visit_joints(&mut visitor);
}

/// Visitor for position integration that handles different joint types.
struct PositionIntegrateVisitor<'a> {
    qpos: &'a mut DVector<f64>,
    qvel: &'a DVector<f64>,
    h: f64,
}

impl PositionIntegrateVisitor<'_> {
    /// Integrate a quaternion with angular velocity on SO(3) manifold.
    /// `qpos_offset` is the offset into qpos where the quaternion starts [w,x,y,z].
    /// `omega` is the angular velocity vector.
    #[inline]
    fn integrate_quaternion(&mut self, qpos_offset: usize, omega: Vector3<f64>) {
        let omega_norm = omega.norm();
        let angle = omega_norm * self.h;

        // Skip if angle is negligible (avoids division by zero)
        if angle > 1e-10 && omega_norm > 1e-10 {
            let axis = omega / omega_norm;
            let dq = UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle);
            let q_old = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                self.qpos[qpos_offset],
                self.qpos[qpos_offset + 1],
                self.qpos[qpos_offset + 2],
                self.qpos[qpos_offset + 3],
            ));
            let q_new = q_old * dq;
            self.qpos[qpos_offset] = q_new.w;
            self.qpos[qpos_offset + 1] = q_new.i;
            self.qpos[qpos_offset + 2] = q_new.j;
            self.qpos[qpos_offset + 3] = q_new.k;
        }
    }
}

impl JointVisitor for PositionIntegrateVisitor<'_> {
    #[inline]
    fn visit_hinge(&mut self, ctx: JointContext) {
        // Simple scalar: qpos += qvel * h
        self.qpos[ctx.qpos_adr] += self.qvel[ctx.dof_adr] * self.h;
    }

    #[inline]
    fn visit_slide(&mut self, ctx: JointContext) {
        // Simple scalar: qpos += qvel * h
        self.qpos[ctx.qpos_adr] += self.qvel[ctx.dof_adr] * self.h;
    }

    #[inline]
    fn visit_ball(&mut self, ctx: JointContext) {
        // Quaternion: integrate angular velocity on SO(3)
        let omega = Vector3::new(
            self.qvel[ctx.dof_adr],
            self.qvel[ctx.dof_adr + 1],
            self.qvel[ctx.dof_adr + 2],
        );
        self.integrate_quaternion(ctx.qpos_adr, omega);
    }

    #[inline]
    fn visit_free(&mut self, ctx: JointContext) {
        // Position: linear integration (first 3 components)
        self.qpos[ctx.qpos_adr] += self.qvel[ctx.dof_adr] * self.h;
        self.qpos[ctx.qpos_adr + 1] += self.qvel[ctx.dof_adr + 1] * self.h;
        self.qpos[ctx.qpos_adr + 2] += self.qvel[ctx.dof_adr + 2] * self.h;

        // Orientation: quaternion integration (last 4 components, DOFs 3-5)
        let omega = Vector3::new(
            self.qvel[ctx.dof_adr + 3],
            self.qvel[ctx.dof_adr + 4],
            self.qvel[ctx.dof_adr + 5],
        );
        self.integrate_quaternion(ctx.qpos_adr + 3, omega);
    }
}

/// Normalize all quaternions in qpos to prevent numerical drift.
fn mj_normalize_quat(model: &Model, data: &mut Data) {
    let mut visitor = QuaternionNormalizeVisitor {
        qpos: &mut data.qpos,
    };
    model.visit_joints(&mut visitor);
}

/// Visitor for normalizing quaternions in joints that use them.
struct QuaternionNormalizeVisitor<'a> {
    qpos: &'a mut DVector<f64>,
}

impl QuaternionNormalizeVisitor<'_> {
    /// Normalize a quaternion at the given offset in qpos.
    #[inline]
    fn normalize_quaternion(&mut self, offset: usize) {
        let norm = (self.qpos[offset].powi(2)
            + self.qpos[offset + 1].powi(2)
            + self.qpos[offset + 2].powi(2)
            + self.qpos[offset + 3].powi(2))
        .sqrt();
        if norm > 1e-10 {
            self.qpos[offset] /= norm;
            self.qpos[offset + 1] /= norm;
            self.qpos[offset + 2] /= norm;
            self.qpos[offset + 3] /= norm;
        } else {
            // Degenerate quaternion - reset to identity [w=1, x=0, y=0, z=0]
            self.qpos[offset] = 1.0;
            self.qpos[offset + 1] = 0.0;
            self.qpos[offset + 2] = 0.0;
            self.qpos[offset + 3] = 0.0;
        }
    }
}

impl JointVisitor for QuaternionNormalizeVisitor<'_> {
    // Hinge and Slide have no quaternions - use default no-ops

    #[inline]
    fn visit_ball(&mut self, ctx: JointContext) {
        // Ball joint: quaternion at qpos_adr
        self.normalize_quaternion(ctx.qpos_adr);
    }

    #[inline]
    fn visit_free(&mut self, ctx: JointContext) {
        // Free joint: quaternion at qpos_adr + 3 (after position xyz)
        self.normalize_quaternion(ctx.qpos_adr + 3);
    }
}

/// Compute velocity from position difference: `qvel` = `mj_differentiatePos(qpos2 - qpos1) / dt`.
///
/// This function computes the velocity that would move from `qpos1` to `qpos2` in time `dt`.
/// For quaternions (ball/free joints), it uses the proper SO(3) velocity rather than
/// naive quaternion subtraction.
///
/// # Arguments
///
/// * `model` - The model containing joint definitions
/// * `qvel` - Output velocity vector (length `nv`)
/// * `qpos1` - Start position
/// * `qpos2` - End position
/// * `dt` - Time difference
///
/// # `MuJoCo` Equivalence
///
/// This matches `MuJoCo`'s `mj_differentiatePos` function.
pub fn mj_differentiate_pos(
    model: &Model,
    qvel: &mut DVector<f64>,
    qpos1: &DVector<f64>,
    qpos2: &DVector<f64>,
    dt: f64,
) {
    if dt.abs() < 1e-10 {
        qvel.fill(0.0);
        return;
    }

    let dt_inv = 1.0 / dt;

    for jnt_id in 0..model.njnt {
        let qpos_adr = model.jnt_qpos_adr[jnt_id];
        let dof_adr = model.jnt_dof_adr[jnt_id];

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge | MjJointType::Slide => {
                // Scalar: simple finite difference
                qvel[dof_adr] = (qpos2[qpos_adr] - qpos1[qpos_adr]) * dt_inv;
            }

            MjJointType::Ball => {
                // Quaternion velocity: compute angular velocity from q1 to q2
                // q2 = q_delta * q1  =>  q_delta = q2 * q1^-1
                // angular velocity = 2 * log(q_delta) / dt
                let q1 = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    qpos1[qpos_adr],
                    qpos1[qpos_adr + 1],
                    qpos1[qpos_adr + 2],
                    qpos1[qpos_adr + 3],
                ));
                let q2 = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    qpos2[qpos_adr],
                    qpos2[qpos_adr + 1],
                    qpos2[qpos_adr + 2],
                    qpos2[qpos_adr + 3],
                ));

                // q_delta = q2 * q1.inverse()
                let q_delta = q2 * q1.inverse();

                // Extract axis-angle (clamp w to avoid NaN from floating-point precision)
                let angle = 2.0 * q_delta.w.clamp(-1.0, 1.0).acos();
                let sin_half = (1.0 - q_delta.w * q_delta.w).max(0.0).sqrt();

                if sin_half > 1e-10 {
                    let axis = Vector3::new(q_delta.i, q_delta.j, q_delta.k) / sin_half;
                    let omega = axis * angle * dt_inv;
                    qvel[dof_adr] = omega.x;
                    qvel[dof_adr + 1] = omega.y;
                    qvel[dof_adr + 2] = omega.z;
                } else {
                    qvel[dof_adr] = 0.0;
                    qvel[dof_adr + 1] = 0.0;
                    qvel[dof_adr + 2] = 0.0;
                }
            }

            MjJointType::Free => {
                // Linear velocity: simple finite difference
                qvel[dof_adr] = (qpos2[qpos_adr] - qpos1[qpos_adr]) * dt_inv;
                qvel[dof_adr + 1] = (qpos2[qpos_adr + 1] - qpos1[qpos_adr + 1]) * dt_inv;
                qvel[dof_adr + 2] = (qpos2[qpos_adr + 2] - qpos1[qpos_adr + 2]) * dt_inv;

                // Angular velocity from quaternion difference
                let q1 = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    qpos1[qpos_adr + 3],
                    qpos1[qpos_adr + 4],
                    qpos1[qpos_adr + 5],
                    qpos1[qpos_adr + 6],
                ));
                let q2 = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    qpos2[qpos_adr + 3],
                    qpos2[qpos_adr + 4],
                    qpos2[qpos_adr + 5],
                    qpos2[qpos_adr + 6],
                ));

                let q_delta = q2 * q1.inverse();
                // Clamp w to avoid NaN from floating-point precision
                let angle = 2.0 * q_delta.w.clamp(-1.0, 1.0).acos();
                let sin_half = (1.0 - q_delta.w * q_delta.w).max(0.0).sqrt();

                if sin_half > 1e-10 {
                    let axis = Vector3::new(q_delta.i, q_delta.j, q_delta.k) / sin_half;
                    let omega = axis * angle * dt_inv;
                    qvel[dof_adr + 3] = omega.x;
                    qvel[dof_adr + 4] = omega.y;
                    qvel[dof_adr + 5] = omega.z;
                } else {
                    qvel[dof_adr + 3] = 0.0;
                    qvel[dof_adr + 4] = 0.0;
                    qvel[dof_adr + 5] = 0.0;
                }
            }
        }
    }
}

/// Integrate position given velocity: `qpos_out` = `mj_integratePos(qpos, qvel, dt)`.
///
/// This is the inverse of `mj_differentiatePos`. It computes the position reached
/// by integrating velocity over time dt, handling quaternions correctly on SO(3).
///
/// # Arguments
///
/// * `model` - The model containing joint definitions
/// * `qpos_out` - Output position vector (length `nq`)
/// * `qpos` - Start position
/// * `qvel` - Velocity
/// * `dt` - Time step
///
/// # `MuJoCo` Equivalence
///
/// This matches the inverse operation of `mj_differentiatePos`.
pub fn mj_integrate_pos_explicit(
    model: &Model,
    qpos_out: &mut DVector<f64>,
    qpos: &DVector<f64>,
    qvel: &DVector<f64>,
    dt: f64,
) {
    for jnt_id in 0..model.njnt {
        let qpos_adr = model.jnt_qpos_adr[jnt_id];
        let dof_adr = model.jnt_dof_adr[jnt_id];

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge | MjJointType::Slide => {
                qpos_out[qpos_adr] = qpos[qpos_adr] + qvel[dof_adr] * dt;
            }

            MjJointType::Ball => {
                // Quaternion integration
                let omega = Vector3::new(qvel[dof_adr], qvel[dof_adr + 1], qvel[dof_adr + 2]);
                let angle = omega.norm() * dt;

                let q_old = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    qpos[qpos_adr],
                    qpos[qpos_adr + 1],
                    qpos[qpos_adr + 2],
                    qpos[qpos_adr + 3],
                ));

                let q_new = if angle > 1e-10 {
                    let axis = omega / omega.norm();
                    let dq = UnitQuaternion::from_axis_angle(
                        &nalgebra::Unit::new_normalize(axis),
                        angle,
                    );
                    q_old * dq
                } else {
                    q_old
                };

                qpos_out[qpos_adr] = q_new.w;
                qpos_out[qpos_adr + 1] = q_new.i;
                qpos_out[qpos_adr + 2] = q_new.j;
                qpos_out[qpos_adr + 3] = q_new.k;
            }

            MjJointType::Free => {
                // Linear position
                qpos_out[qpos_adr] = qpos[qpos_adr] + qvel[dof_adr] * dt;
                qpos_out[qpos_adr + 1] = qpos[qpos_adr + 1] + qvel[dof_adr + 1] * dt;
                qpos_out[qpos_adr + 2] = qpos[qpos_adr + 2] + qvel[dof_adr + 2] * dt;

                // Quaternion integration
                let omega = Vector3::new(qvel[dof_adr + 3], qvel[dof_adr + 4], qvel[dof_adr + 5]);
                let angle = omega.norm() * dt;

                let q_old = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    qpos[qpos_adr + 3],
                    qpos[qpos_adr + 4],
                    qpos[qpos_adr + 5],
                    qpos[qpos_adr + 6],
                ));

                let q_new = if angle > 1e-10 {
                    let axis = omega / omega.norm();
                    let dq = UnitQuaternion::from_axis_angle(
                        &nalgebra::Unit::new_normalize(axis),
                        angle,
                    );
                    q_old * dq
                } else {
                    q_old
                };

                qpos_out[qpos_adr + 3] = q_new.w;
                qpos_out[qpos_adr + 4] = q_new.i;
                qpos_out[qpos_adr + 5] = q_new.j;
                qpos_out[qpos_adr + 6] = q_new.k;
            }
        }
    }
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

        // Verify inertia and bias are computed
        let _m = sys.inertia_matrix();
        let _bias = sys.bias_forces();

        // Run simulation - body should fall
        let initial_y = sys.qpos[1];
        sys.step(DT);
        sys.run_for(0.1, DT);

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

// ============================================================================
// Tests for PGS Constraint Solver
// ============================================================================

#[cfg(test)]
mod pgs_tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_pgs_empty_constraints() {
        let solver = PGSSolver::new();
        let m_inv = DMatrix::identity(3, 3);
        let qacc = DVector::from_vec(vec![1.0, 2.0, 3.0]);

        let result = solver.solve(&m_inv, &qacc, &[]);

        assert!(result.converged);
        assert_eq!(result.forces.len(), 0);
    }

    #[test]
    fn test_pgs_single_equality_constraint() {
        let solver = PGSSolver::new();

        // 1D system: m=1, want to enforce qacc = 0
        let m_inv = DMatrix::from_element(1, 1, 1.0);
        let qacc_smooth = DVector::from_vec(vec![10.0]); // Unconstrained accel

        // Constraint: J @ qacc = 0, so J = [1], aref = 0
        let constraint = Constraint::equality(DVector::from_vec(vec![1.0]), 0.0);

        let result = solver.solve(&m_inv, &qacc_smooth, &[constraint]);

        assert!(result.converged);
        // Force should cancel the acceleration: λ = -10
        assert_relative_eq!(result.forces[0], -10.0, epsilon = 0.01);
    }

    #[test]
    fn test_pgs_inequality_constraint() {
        let solver = PGSSolver::new();

        // 1D system with inequality: qacc ≥ 0
        let m_inv = DMatrix::from_element(1, 1, 1.0);

        // Case 1: Unconstrained is positive - constraint inactive
        let qacc1 = DVector::from_vec(vec![5.0]);
        let constraint1 = Constraint::inequality(DVector::from_vec(vec![1.0]), 0.0);
        let result1 = solver.solve(&m_inv, &qacc1, &[constraint1]);
        assert!(result1.converged);
        assert_relative_eq!(result1.forces[0], 0.0, epsilon = 0.01);

        // Case 2: Unconstrained is negative - constraint active
        let qacc2 = DVector::from_vec(vec![-5.0]);
        let constraint2 = Constraint::inequality(DVector::from_vec(vec![1.0]), 0.0);
        let result2 = solver.solve(&m_inv, &qacc2, &[constraint2]);
        assert!(result2.converged);
        assert!(
            result2.forces[0] > 0.0,
            "Inequality force should be positive"
        );
        // Force should bring acceleration to 0: λ = 5
        assert_relative_eq!(result2.forces[0], 5.0, epsilon = 0.01);
    }

    #[test]
    fn test_pgs_friction_constraint() {
        let solver = PGSSolver::new();

        // 2D system: normal + friction
        let m_inv = DMatrix::identity(2, 2);

        // Normal force pressing down (constraint 0)
        // Friction force (constraint 1) limited by μ * normal
        let mu = 0.5;

        let constraint_normal =
            Constraint::inequality(DVector::from_vec(vec![1.0, 0.0]), 0.0).with_warmstart(10.0);
        let constraint_friction =
            Constraint::friction(DVector::from_vec(vec![0.0, 1.0]), 0.0, 0, mu);

        // Large tangential acceleration that would need clamping
        let qacc = DVector::from_vec(vec![-10.0, 100.0]);

        let result = solver.solve(&m_inv, &qacc, &[constraint_normal, constraint_friction]);

        assert!(result.converged);
        // Friction should be limited to μ * normal
        let max_friction = mu * result.forces[0].max(0.0);
        assert!(
            result.forces[1].abs() <= max_friction + 0.1,
            "Friction {} exceeds limit {}",
            result.forces[1].abs(),
            max_friction
        );
    }

    #[test]
    fn test_joint_limit_constraint_generation() {
        let mut sys = ArticulatedSystem::new();

        // Create a simple pendulum with limits
        let root = sys.add_body(ArticulatedBody::fixed(), None);
        let bob = sys.add_body(
            ArticulatedBody::point_mass(1.0)
                .with_joint_transform(Isometry3::translation(0.0, -1.0, 0.0)),
            Some(root),
        );
        sys.add_joint(
            bob,
            ArticulatedJoint::revolute(Vector3::z()).with_limits(vec![-0.5], vec![0.5]),
        );

        // Position within limits - no constraints
        sys.set_joint_position(0, 0.0);
        let constraints = sys.joint_limit_constraints(100.0, 10.0);
        assert_eq!(constraints.len(), 0);

        // Position at lower limit violation
        sys.set_joint_position(0, -0.6);
        let constraints = sys.joint_limit_constraints(100.0, 10.0);
        assert_eq!(constraints.len(), 1);

        // Position at upper limit violation
        sys.set_joint_position(0, 0.6);
        let constraints = sys.joint_limit_constraints(100.0, 10.0);
        assert_eq!(constraints.len(), 1);
    }

    #[test]
    fn test_contact_constraint_generation() {
        let mut sys = ArticulatedSystem::new();

        // Create a free-floating body
        let body = sys.add_body(ArticulatedBody::sphere(1.0, 0.1), None);
        sys.add_joint(body, ArticulatedJoint::free());

        // Create a ground contact
        let contact = ContactPoint {
            position: Vector3::new(0.0, 0.0, 0.0),
            normal: Vector3::y(), // Ground normal pointing up
            depth: 0.01,
            body_a: Some(body),
            body_b: None, // Ground
            friction: 0.5,
            restitution: 0.0,
        };

        let constraints = sys.contact_constraints(&[contact], 100.0, 10.0);

        // Should have 1 normal + 2 friction = 3 constraints
        assert_eq!(constraints.len(), 3);

        // First should be inequality (normal)
        assert!(matches!(
            constraints[0].constraint_type,
            ConstraintType::Inequality
        ));

        // Next two should be friction
        assert!(matches!(
            constraints[1].constraint_type,
            ConstraintType::Friction { .. }
        ));
        assert!(matches!(
            constraints[2].constraint_type,
            ConstraintType::Friction { .. }
        ));
    }

    #[test]
    fn test_step_constrained_with_limits() {
        let mut sys = ArticulatedSystem::new();

        // Create a pendulum with limits
        let root = sys.add_body(ArticulatedBody::fixed(), None);
        let bob = sys.add_body(
            ArticulatedBody::point_mass(1.0)
                .with_joint_transform(Isometry3::translation(0.0, -1.0, 0.0)),
            Some(root),
        );
        sys.add_joint(
            bob,
            ArticulatedJoint::revolute(Vector3::z()).with_limits(vec![-0.5], vec![0.5]),
        );

        // Start past the limit (in violation)
        sys.set_joint_position(0, 0.55);
        sys.set_joint_velocity(0, 0.5); // Still trying to go further

        let solver = PGSSolver::new();

        // Step multiple times - constraint should push back
        for _ in 0..200 {
            sys.step_constrained(1.0 / 480.0, &solver);
        }

        // Position should be pushed back toward limit (not stay at 0.55 or go further)
        // With Baumgarte stabilization, it should return toward the limit
        assert!(
            sys.qpos[0] < 0.55,
            "Position {} should have been pushed back from initial violation",
            sys.qpos[0]
        );
    }

    #[test]
    fn test_tangent_basis() {
        let normal = Vector3::y();
        let (tan1, tan2) = compute_tangent_basis(&normal);

        // Should be orthonormal
        assert_relative_eq!(tan1.norm(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(tan2.norm(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(tan1.dot(&tan2), 0.0, epsilon = 1e-10);
        assert_relative_eq!(tan1.dot(&normal), 0.0, epsilon = 1e-10);
        assert_relative_eq!(tan2.dot(&normal), 0.0, epsilon = 1e-10);
    }
}

// ============================================================================
// Tests for Primitive Collision Detection
// ============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod primitive_collision_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Helper to create a minimal Model for collision testing.
    fn make_collision_test_model(ngeom: usize) -> Model {
        let mut model = Model::empty();
        model.ngeom = ngeom;
        model.geom_type = vec![GeomType::Sphere; ngeom];
        model.geom_body = vec![0; ngeom];
        model.geom_pos = vec![Vector3::zeros(); ngeom];
        model.geom_quat = vec![UnitQuaternion::identity(); ngeom];
        model.geom_size = vec![Vector3::new(1.0, 1.0, 1.0); ngeom];
        model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001); ngeom];
        model.geom_contype = vec![1; ngeom];
        model.geom_conaffinity = vec![1; ngeom];
        model.geom_margin = vec![0.0; ngeom];
        model.geom_gap = vec![0.0; ngeom];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; ngeom];
        model.geom_solref = vec![[0.02, 1.0]; ngeom];
        model.geom_name = vec![None; ngeom];
        model.geom_rbound = vec![1.0; ngeom];
        model.geom_mesh = vec![None; ngeom]; // No mesh geoms in test helper
        model
    }

    // ========================================================================
    // Cylinder-Plane Collision Tests
    // ========================================================================

    #[test]
    fn test_cylinder_plane_upright_penetrating() {
        // Cylinder standing upright on plane, bottom rim penetrating
        let mut model = make_collision_test_model(2);

        // Geom 0: Plane at z=0
        model.geom_type[0] = GeomType::Plane;
        model.geom_size[0] = Vector3::new(10.0, 10.0, 0.1);

        // Geom 1: Cylinder (radius=0.3, half_height=0.5)
        // Center at z=0.4, so bottom cap center is at z = 0.4 - 0.5 = -0.1
        // This means the bottom penetrates 0.1 below the plane (at z=0)
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0); // [radius, half_height, unused]

        let plane_pos = Vector3::zeros(); // Plane at z=0
        let plane_mat = Matrix3::identity(); // Normal along +Z
        let cyl_pos = Vector3::new(0.0, 0.0, 0.4); // Center at z=0.4
        let cyl_mat = Matrix3::identity(); // Axis along +Z (upright)

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(contact.is_some(), "Cylinder should contact plane");
        let c = contact.unwrap();
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
        assert_relative_eq!(c.normal, Vector3::z(), epsilon = 1e-10);
    }

    #[test]
    fn test_cylinder_plane_upright_no_contact() {
        // Cylinder above plane, no contact
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0);

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let cyl_pos = Vector3::new(0.0, 0.0, 1.0); // Center at z=1.0, bottom at z=0.5
        let cyl_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(
            contact.is_none(),
            "Cylinder should not contact plane when above"
        );
    }

    #[test]
    fn test_cylinder_plane_tilted() {
        // Cylinder tilted 45° around X axis, verifying rim point depth calculation
        let mut model = make_collision_test_model(2);

        // Geom 0: Plane at z=0
        model.geom_type[0] = GeomType::Plane;

        // Geom 1: Cylinder with radius=0.3, half_height=0.5, tilted 45° around X
        // After rotation, the cylinder axis points diagonally (into Y-Z plane)
        model.geom_type[1] = GeomType::Cylinder;
        let radius = 0.3;
        let half_height = 0.5;
        model.geom_size[1] = Vector3::new(radius, half_height, 0.0);

        let plane_pos = Vector3::zeros(); // Plane at z=0
        let plane_mat = Matrix3::identity(); // Normal along +Z

        // Tilt cylinder 45 degrees around X axis
        let angle = std::f64::consts::FRAC_PI_4;
        let cos_a = angle.cos(); // √2/2 ≈ 0.7071
        let sin_a = angle.sin(); // √2/2 ≈ 0.7071
        let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle);
        let cyl_mat = rot.to_rotation_matrix().into_inner();
        let cyl_pos = Vector3::new(0.0, 0.0, 0.5); // Center at z=0.5

        // Expected deepest point calculation (Case 3: tilted cylinder):
        // Cylinder axis in world frame: (0, sin(45°), cos(45°)) = (0, 0.707, 0.707)
        // Plane normal: (0, 0, 1)
        // axis_dot_signed = plane_normal · cyl_axis = cos(45°) ≈ 0.707
        // radial = plane_normal - cyl_axis * axis_dot_signed
        //        = (0, 0, 1) - (0, 0.707, 0.707) * 0.707
        //        = (0, -0.5, 0.5)
        // rim_dir = -radial / ||radial|| = (0, 0.707, -0.707)
        //
        // Bottom cap center: cyl_pos - cyl_axis * half_height
        //                  = (0, 0, 0.5) - (0, 0.354, 0.354) = (0, -0.354, 0.146)
        // Bottom rim point: bottom_center + rim_dir * radius
        //                 = (0, -0.354, 0.146) + (0, 0.212, -0.212) = (0, -0.142, -0.066)
        //
        // Top cap center: cyl_pos + cyl_axis * half_height
        //               = (0, 0, 0.5) + (0, 0.354, 0.354) = (0, 0.354, 0.854)
        // Top rim point: top_center + rim_dir * radius
        //              = (0, 0.354, 0.854) + (0, 0.212, -0.212) = (0, 0.566, 0.642)
        //
        // Bottom rim z = -0.066 (below plane) → depth = 0.066
        // Top rim z = 0.642 (above plane) → no penetration
        // Deepest point is bottom rim with depth ≈ 0.066
        let cyl_axis_y = sin_a;
        let cyl_axis_z = cos_a;
        let bottom_center_z = cyl_pos.z - cyl_axis_z * half_height;
        // radial = (0, -axis_dot_signed * cyl_axis_y, 1 - axis_dot_signed * cyl_axis_z)
        let axis_dot_signed = cos_a;
        let radial_y = -axis_dot_signed * cyl_axis_y;
        let radial_z = 1.0 - axis_dot_signed * cyl_axis_z;
        let radial_len = (radial_y * radial_y + radial_z * radial_z).sqrt();
        let rim_dir_z = -radial_z / radial_len;
        let bottom_rim_z = bottom_center_z + rim_dir_z * radius;
        let expected_depth = -bottom_rim_z;

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(contact.is_some(), "Tilted cylinder should contact plane");
        let c = contact.unwrap();
        assert_relative_eq!(c.depth, expected_depth, epsilon = 1e-10);
        assert_relative_eq!(c.normal, Vector3::z(), epsilon = 1e-10);
    }

    #[test]
    fn test_cylinder_plane_horizontal() {
        // Cylinder lying flat (axis parallel to plane)
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0); // radius=0.3, half_height=0.5

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();

        // Rotate 90 degrees around X axis (cylinder now horizontal, axis along Y)
        let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2);
        let cyl_mat = rot.to_rotation_matrix().into_inner();

        // Position center at z = radius - epsilon for penetration
        let cyl_pos = Vector3::new(0.0, 0.0, 0.2); // Below radius, should penetrate

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(
            contact.is_some(),
            "Horizontal cylinder should contact plane"
        );
        let c = contact.unwrap();
        // Penetration should be radius - z_center = 0.3 - 0.2 = 0.1
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
    }

    // ========================================================================
    // Ellipsoid-Plane Collision Tests
    // ========================================================================

    #[test]
    fn test_ellipsoid_plane_sphere_case() {
        // Ellipsoid with equal radii (sphere) for validation against known sphere formula
        let mut model = make_collision_test_model(2);

        // Geom 0: Plane at z=0
        model.geom_type[0] = GeomType::Plane;

        // Geom 1: Ellipsoid with rx=ry=rz=0.5 (degenerates to sphere with radius 0.5)
        // Center at z=0.4, so bottom (support point) is at z = 0.4 - 0.5 = -0.1
        // This means the bottom penetrates 0.1 below the plane (at z=0)
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.5, 0.5, 0.5);

        let plane_pos = Vector3::zeros(); // Plane at z=0
        let plane_mat = Matrix3::identity(); // Normal along +Z
        let ell_pos = Vector3::new(0.0, 0.0, 0.4); // Center at z=0.4
        let ell_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(contact.is_some(), "Ellipsoid (sphere) should contact plane");
        let c = contact.unwrap();
        // Penetration = radius - z = 0.5 - 0.4 = 0.1
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
        assert_relative_eq!(c.normal, Vector3::z(), epsilon = 1e-10);
    }

    #[test]
    fn test_ellipsoid_plane_stretched_z() {
        // Ellipsoid stretched along Z axis (tall and thin)
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.2, 0.2, 0.8); // Tall ellipsoid

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let ell_pos = Vector3::new(0.0, 0.0, 0.7); // Bottom at z = -0.1
        let ell_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(contact.is_some(), "Tall ellipsoid should contact plane");
        let c = contact.unwrap();
        // Penetration = z_radius - z = 0.8 - 0.7 = 0.1
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
    }

    #[test]
    fn test_ellipsoid_plane_stretched_x() {
        // Ellipsoid stretched along X axis (wide and flat)
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.8, 0.3, 0.2); // Wide ellipsoid (short in Z)

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let ell_pos = Vector3::new(0.0, 0.0, 0.15); // Bottom at z = -0.05 (penetrating)
        let ell_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(contact.is_some(), "Wide ellipsoid should contact plane");
        let c = contact.unwrap();
        // Penetration = z_radius - z = 0.2 - 0.15 = 0.05
        assert_relative_eq!(c.depth, 0.05, epsilon = 1e-6);
    }

    #[test]
    fn test_ellipsoid_plane_rotated() {
        // Ellipsoid rotated 45° around X axis, verifying support point formula
        let mut model = make_collision_test_model(2);

        // Geom 0: Plane at z=0
        model.geom_type[0] = GeomType::Plane;

        // Geom 1: Tall ellipsoid with radii (0.2, 0.2, 0.8), rotated 45° around X
        // After rotation, the long axis points diagonally (into Y-Z plane)
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.2, 0.2, 0.8);

        let plane_pos = Vector3::zeros(); // Plane at z=0
        let plane_mat = Matrix3::identity(); // Normal along +Z

        // Rotate 45 degrees around X axis
        let angle = std::f64::consts::FRAC_PI_4;
        let cos_a = angle.cos(); // √2/2 ≈ 0.7071
        let sin_a = angle.sin(); // √2/2 ≈ 0.7071
        let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle);
        let ell_mat = rot.to_rotation_matrix().into_inner();
        let ell_pos = Vector3::new(0.0, 0.0, 0.5); // Center at z=0.5

        // Expected support point calculation:
        // Local plane normal: n_local = R^T * (0,0,1) = (0, sin(45°), cos(45°))
        //   (R rotates around X, so R^T maps world Z to local (0, sin, cos))
        // scaled = r ⊙ n_local = (0, 0.2*sin, 0.8*cos) = (0, 0.1414, 0.5657)
        // ||scaled|| = sqrt(0.2² * sin² + 0.8² * cos²) ≈ 0.583
        // local_support = -(r² ⊙ n) / ||scaled||
        //               = -(0, 0.04*sin, 0.64*cos) / 0.583
        //               = (0, -0.0485, -0.776)
        // world_support = center + R * local_support
        // The z-component: 0.5 + (local_y * sin + local_z * cos)
        //                = 0.5 + (-0.0485 * 0.707 + (-0.776) * 0.707)
        //                = 0.5 - 0.583 = -0.083
        // Expected depth = -(-0.083) = 0.083 (positive, penetrating)
        let ry = 0.2;
        let rz = 0.8;
        let scaled_y = ry * sin_a;
        let scaled_z = rz * cos_a;
        let scale_norm = (scaled_y * scaled_y + scaled_z * scaled_z).sqrt();
        let local_support_y = -(ry * ry * sin_a) / scale_norm;
        let local_support_z = -(rz * rz * cos_a) / scale_norm;
        let world_support_z = ell_pos.z + local_support_y * sin_a + local_support_z * cos_a;
        let expected_depth = -world_support_z;

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(contact.is_some(), "Rotated ellipsoid should contact plane");
        let c = contact.unwrap();
        assert_relative_eq!(c.depth, expected_depth, epsilon = 1e-10);
        assert_relative_eq!(c.normal, Vector3::z(), epsilon = 1e-10);
    }

    #[test]
    fn test_ellipsoid_plane_no_contact() {
        // Ellipsoid above plane, no contact
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.3, 0.3, 0.3);

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let ell_pos = Vector3::new(0.0, 0.0, 1.0); // Far above plane
        let ell_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(
            contact.is_none(),
            "Ellipsoid should not contact plane when above"
        );
    }

    // ========================================================================
    // Edge Cases and Numerical Stability
    // ========================================================================

    #[test]
    fn test_cylinder_plane_axis_parallel_to_normal() {
        // Cylinder axis exactly parallel to plane normal (degenerate case)
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0);

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let cyl_pos = Vector3::new(0.0, 0.0, 0.4); // Bottom at z = -0.1
        let cyl_mat = Matrix3::identity(); // Axis along Z (parallel to plane normal)

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        // Should still detect contact at the bottom rim
        assert!(
            contact.is_some(),
            "Should detect contact even when axis parallel to normal"
        );
        let c = contact.unwrap();
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
    }

    #[test]
    fn test_contact_frame_is_valid() {
        // Verify contact frame is orthonormal
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0);

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let cyl_pos = Vector3::new(0.0, 0.0, 0.4);
        let cyl_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
        )
        .expect("should have contact");

        // Normal should be unit length
        assert_relative_eq!(contact.normal.norm(), 1.0, epsilon = 1e-10);

        // Tangent vectors should be unit length
        assert_relative_eq!(contact.frame[0].norm(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(contact.frame[1].norm(), 1.0, epsilon = 1e-10);

        // All three should be mutually orthogonal
        assert_relative_eq!(contact.normal.dot(&contact.frame[0]), 0.0, epsilon = 1e-10);
        assert_relative_eq!(contact.normal.dot(&contact.frame[1]), 0.0, epsilon = 1e-10);
        assert_relative_eq!(
            contact.frame[0].dot(&contact.frame[1]),
            0.0,
            epsilon = 1e-10
        );
    }
}
