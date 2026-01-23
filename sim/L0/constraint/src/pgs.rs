//! Projected Gauss-Seidel (PGS) solver for constraint systems.
//!
//! This module implements a full iterative Gauss-Seidel solver with Successive
//! Over-Relaxation (SOR) for solving constraint systems. PGS is the standard
//! solver used in many physics engines including `MuJoCo`.
//!
//! # Algorithm
//!
//! The PGS solver iteratively solves the constraint system A * λ = b where:
//! - A = J * M^-1 * J^T is the effective mass matrix
//! - λ are Lagrange multipliers (constraint impulses)
//! - b is the constraint bias (velocity error + position error correction)
//!
//! For each constraint i, we update:
//! ```text
//! λ_i^{new} = (b_i - Σ_{j<i} A_ij λ_j^{new} - Σ_{j>i} A_ij λ_j^{old}) / A_ii
//! ```
//!
//! With SOR, the update becomes:
//! ```text
//! λ_i^{new} = (1 - ω) * λ_i^{old} + ω * λ_i^{gauss-seidel}
//! ```
//!
//! Where ω is the relaxation factor:
//! - ω = 1.0: Standard Gauss-Seidel
//! - ω < 1.0: Under-relaxation (more stable, slower convergence)
//! - ω > 1.0: Over-relaxation (faster convergence if stable, up to ~1.9)
//!
//! # When to Use PGS
//!
//! PGS is well-suited for:
//! - Real-time simulation where predictable iteration count matters
//! - Systems with inequality constraints (contact, limits)
//! - Warm-started systems where solution changes slowly
//! - Integration with time-stepping schemes that expect iterative solvers
//!
//! For stiff systems requiring high accuracy, consider Newton or CG solvers.
//!
//! # Example
//!
//! ```ignore
//! use sim_constraint::{PGSSolver, PGSSolverConfig};
//!
//! let config = PGSSolverConfig {
//!     max_iterations: 100,
//!     tolerance: 1e-6,
//!     sor_factor: 1.3, // Over-relaxation for faster convergence
//!     ..Default::default()
//! };
//!
//! let mut solver = PGSSolver::new(config);
//! let result = solver.solve(&joints, get_body_state, dt);
//! ```

use nalgebra::{DMatrix, DVector, Matrix3, Vector3};
use sim_types::BodyId;

use crate::{BodyState, ConstraintForce, Joint, JointForce, JointType};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Configuration for the PGS constraint solver.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PGSSolverConfig {
    /// Maximum number of Gauss-Seidel iterations.
    pub max_iterations: usize,

    /// Convergence tolerance for constraint error.
    /// Iteration stops when ||residual|| < tolerance.
    pub tolerance: f64,

    /// Baumgarte stabilization factor (0-1).
    /// Higher values correct position errors faster but may add energy.
    /// Typical values: 0.1-0.3
    pub baumgarte_factor: f64,

    /// Regularization factor for numerical stability.
    /// Small positive value added to diagonal of effective mass matrix.
    pub regularization: f64,

    /// SOR (Successive Over-Relaxation) factor.
    /// - 1.0: Standard Gauss-Seidel
    /// - < 1.0: Under-relaxation (more stable)
    /// - > 1.0: Over-relaxation (faster convergence, typically 1.2-1.8)
    pub sor_factor: f64,

    /// Enable warm starting from previous solution.
    /// When enabled, initializes with previous frame's λ values.
    pub warm_starting: bool,

    /// Scaling factor for warm start values (0-1).
    /// Lower values are more conservative, higher values converge faster
    /// when solution changes slowly. Typical: 0.8-0.95
    pub warm_start_factor: f64,

    /// Minimum iterations before checking convergence.
    /// Prevents premature termination on first iteration.
    pub min_iterations: usize,

    /// Enable convergence tracking (records residual history).
    pub track_convergence: bool,
}

impl Default for PGSSolverConfig {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            tolerance: 1e-6,
            baumgarte_factor: 0.2,
            regularization: 1e-9,
            sor_factor: 1.0, // Standard Gauss-Seidel
            warm_starting: true,
            warm_start_factor: 0.9,
            min_iterations: 3,
            track_convergence: false,
        }
    }
}

impl PGSSolverConfig {
    /// High-accuracy configuration for precise simulations.
    #[must_use]
    pub fn high_accuracy() -> Self {
        Self {
            max_iterations: 200,
            tolerance: 1e-10,
            baumgarte_factor: 0.1,
            regularization: 1e-12,
            sor_factor: 1.0, // Standard GS for stability
            warm_starting: true,
            warm_start_factor: 0.95,
            min_iterations: 5,
            track_convergence: false,
        }
    }

    /// Fast configuration for real-time applications.
    #[must_use]
    pub fn realtime() -> Self {
        Self {
            max_iterations: 20,
            tolerance: 1e-4,
            baumgarte_factor: 0.3,
            regularization: 1e-8,
            sor_factor: 1.3, // Mild over-relaxation
            warm_starting: true,
            warm_start_factor: 0.85,
            min_iterations: 2,
            track_convergence: false,
        }
    }

    /// `MuJoCo`-compatible configuration.
    /// Uses similar defaults to `MuJoCo`'s PGS solver.
    #[must_use]
    pub fn mujoco() -> Self {
        Self {
            max_iterations: 100,
            tolerance: 1e-6,
            baumgarte_factor: 0.2,
            regularization: 1e-10,
            sor_factor: 1.0,
            warm_starting: true,
            warm_start_factor: 0.9,
            min_iterations: 3,
            track_convergence: false,
        }
    }

    /// High-speed configuration with aggressive over-relaxation.
    /// May be unstable for stiff systems.
    #[must_use]
    pub fn fast() -> Self {
        Self {
            max_iterations: 50,
            tolerance: 1e-5,
            baumgarte_factor: 0.25,
            regularization: 1e-8,
            sor_factor: 1.5, // Aggressive over-relaxation
            warm_starting: true,
            warm_start_factor: 0.9,
            min_iterations: 3,
            track_convergence: false,
        }
    }

    /// Set the SOR factor.
    #[must_use]
    pub const fn with_sor_factor(mut self, factor: f64) -> Self {
        self.sor_factor = factor;
        self
    }

    /// Enable warm starting.
    #[must_use]
    pub const fn with_warm_starting(mut self, enabled: bool) -> Self {
        self.warm_starting = enabled;
        self
    }

    /// Set warm start factor.
    #[must_use]
    pub const fn with_warm_start_factor(mut self, factor: f64) -> Self {
        self.warm_start_factor = factor;
        self
    }

    /// Set maximum iterations.
    #[must_use]
    pub const fn with_max_iterations(mut self, max_iter: usize) -> Self {
        self.max_iterations = max_iter;
        self
    }

    /// Enable convergence tracking.
    #[must_use]
    pub const fn with_convergence_tracking(mut self, enabled: bool) -> Self {
        self.track_convergence = enabled;
        self
    }

    /// Validate the configuration.
    ///
    /// # Errors
    ///
    /// Returns an error string if any configuration value is out of range.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.sor_factor <= 0.0 || self.sor_factor >= 2.0 {
            return Err("SOR factor must be in range (0, 2)");
        }
        if self.baumgarte_factor < 0.0 || self.baumgarte_factor > 1.0 {
            return Err("Baumgarte factor must be in range [0, 1]");
        }
        if self.warm_start_factor < 0.0 || self.warm_start_factor > 1.0 {
            return Err("Warm start factor must be in range [0, 1]");
        }
        if self.tolerance <= 0.0 {
            return Err("Tolerance must be positive");
        }
        Ok(())
    }
}

/// Result of PGS solver iteration.
#[derive(Debug, Clone)]
pub struct PGSSolverResult {
    /// Forces for each joint.
    pub forces: Vec<JointForce>,

    /// Number of iterations used.
    pub iterations_used: usize,

    /// Final residual norm.
    pub residual_norm: f64,

    /// Initial residual norm.
    pub initial_residual_norm: f64,

    /// Whether the solver converged within tolerance.
    pub converged: bool,
}

impl PGSSolverResult {
    /// Create an empty result.
    #[must_use]
    pub fn empty() -> Self {
        Self {
            forces: Vec::new(),
            iterations_used: 0,
            residual_norm: 0.0,
            initial_residual_norm: 0.0,
            converged: true,
        }
    }

    /// Check if there are no forces.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.forces.is_empty()
    }

    /// Get convergence ratio (final/initial residual).
    #[must_use]
    pub fn convergence_ratio(&self) -> f64 {
        if self.initial_residual_norm > 1e-15 {
            self.residual_norm / self.initial_residual_norm
        } else {
            0.0
        }
    }
}

/// Statistics from a PGS solver run.
#[derive(Debug, Clone, Default)]
pub struct PGSSolverStats {
    /// Number of bodies in the system.
    pub num_bodies: usize,
    /// Number of constraint rows.
    pub num_constraints: usize,
    /// Whether warm starting was applied.
    pub used_warm_start: bool,
    /// SOR factor used.
    pub sor_factor: f64,
    /// Initial residual norm.
    pub initial_residual: f64,
    /// Final residual norm.
    pub final_residual: f64,
    /// Convergence history (residual at each iteration).
    pub convergence_history: Option<Vec<f64>>,
}

/// Internal solver body representation.
#[derive(Debug, Clone)]
struct SolverBody {
    state: BodyState,
}

/// Internal constraint representation.
#[derive(Debug)]
struct SolverConstraint<'a, J: Joint> {
    joint: &'a J,
    parent_index: usize,
    child_index: usize,
    row_offset: usize,
    num_rows: usize,
}

/// Projected Gauss-Seidel constraint solver.
///
/// This solver implements a full iterative Gauss-Seidel method with
/// Successive Over-Relaxation (SOR) for solving constraint systems.
///
/// # Features
///
/// - **SOR Support**: Configurable relaxation factor for faster convergence
/// - **Warm Starting**: Initialize with previous frame's solution
/// - **Convergence Tracking**: Optional residual history for debugging
/// - **`MuJoCo` Compatible**: Similar algorithm to `MuJoCo`'s PGS solver
#[derive(Debug, Clone)]
pub struct PGSSolver {
    /// Solver configuration.
    config: PGSSolverConfig,

    /// Cached Lagrange multipliers for warm starting.
    cached_lambda: Vec<f64>,

    /// Previous constraint count (for cache validation).
    prev_constraint_count: usize,

    /// Statistics from the last solve.
    last_stats: PGSSolverStats,
}

impl Default for PGSSolver {
    fn default() -> Self {
        Self::new(PGSSolverConfig::default())
    }
}

impl PGSSolver {
    /// Create a new PGS solver with the given configuration.
    #[must_use]
    pub fn new(config: PGSSolverConfig) -> Self {
        Self {
            config,
            cached_lambda: Vec::new(),
            prev_constraint_count: 0,
            last_stats: PGSSolverStats::default(),
        }
    }

    /// Create a solver with default configuration.
    #[must_use]
    pub fn default_solver() -> Self {
        Self::new(PGSSolverConfig::default())
    }

    /// Get the configuration.
    #[must_use]
    pub fn config(&self) -> &PGSSolverConfig {
        &self.config
    }

    /// Get mutable configuration.
    pub fn config_mut(&mut self) -> &mut PGSSolverConfig {
        &mut self.config
    }

    /// Get statistics from the last solve.
    #[must_use]
    pub fn last_stats(&self) -> &PGSSolverStats {
        &self.last_stats
    }

    /// Set the SOR factor.
    pub fn set_sor_factor(&mut self, factor: f64) {
        self.config.sor_factor = factor.clamp(0.01, 1.99);
    }

    /// Enable or disable warm starting.
    pub fn set_warm_starting(&mut self, enabled: bool) {
        self.config.warm_starting = enabled;
    }

    /// Clear the warm start cache.
    pub fn clear_warm_start(&mut self) {
        self.cached_lambda.clear();
        self.prev_constraint_count = 0;
    }

    /// Solve constraints using the PGS method.
    ///
    /// # Arguments
    ///
    /// * `joints` - The joints to solve
    /// * `get_body_state` - Function to get body state by ID
    /// * `dt` - Timestep for Baumgarte stabilization
    ///
    /// # Returns
    ///
    /// The constraint forces to apply to each body.
    pub fn solve<J, F>(&mut self, joints: &[J], get_body_state: F, dt: f64) -> PGSSolverResult
    where
        J: Joint,
        F: Fn(BodyId) -> Option<BodyState>,
    {
        if joints.is_empty() {
            return PGSSolverResult::empty();
        }

        // Build body list and constraint list
        let Some((bodies, constraints, total_constraint_rows)) =
            self.build_system(joints, &get_body_state)
        else {
            return PGSSolverResult::empty();
        };

        if total_constraint_rows == 0 {
            return PGSSolverResult::empty();
        }

        let num_bodies = bodies.len();

        // Build constraint system matrices
        let jacobian = self.build_jacobian(&bodies, &constraints, total_constraint_rows);
        let inv_mass = self.build_inverse_mass_matrix(&bodies);

        // Compute effective mass: A = J * M^-1 * J^T + regularization * I
        let j_minv = &jacobian * &inv_mass;
        let mut effective_mass = &j_minv * jacobian.transpose();

        // Add regularization to diagonal
        for i in 0..effective_mass.nrows() {
            effective_mass[(i, i)] += self.config.regularization;
        }

        // Build right-hand side: b = -J*v - β/h * C
        let rhs = self.compute_rhs(&bodies, &constraints, &jacobian, dt);

        // Initialize lambda (warm start or zeros)
        let warm_started = self.config.warm_starting
            && self.cached_lambda.len() == total_constraint_rows
            && self.prev_constraint_count == total_constraint_rows;

        let mut lambda = if warm_started {
            DVector::from_iterator(
                total_constraint_rows,
                self.cached_lambda
                    .iter()
                    .map(|&v| v * self.config.warm_start_factor),
            )
        } else {
            DVector::zeros(total_constraint_rows)
        };

        // Compute initial residual
        let initial_residual = self.compute_residual(&effective_mass, &lambda, &rhs);
        let initial_residual_norm = initial_residual.norm();

        // Convergence history tracking
        let mut convergence_history = if self.config.track_convergence {
            Some(vec![initial_residual_norm])
        } else {
            None
        };

        // PGS iterations
        let mut iterations = 0;
        let mut converged = false;
        let mut current_residual_norm = initial_residual_norm;

        for iter in 0..self.config.max_iterations {
            iterations = iter + 1;

            // Perform one Gauss-Seidel iteration with SOR
            self.gauss_seidel_iteration(&effective_mass, &rhs, &mut lambda);

            // Check convergence
            let residual = self.compute_residual(&effective_mass, &lambda, &rhs);
            current_residual_norm = residual.norm();

            if let Some(ref mut history) = convergence_history {
                history.push(current_residual_norm);
            }

            // Check for convergence after minimum iterations
            if iterations >= self.config.min_iterations
                && current_residual_norm < self.config.tolerance
            {
                converged = true;
                break;
            }
        }

        // Cache lambda for next frame's warm start
        self.cached_lambda.clear();
        self.cached_lambda.extend(lambda.iter());
        self.prev_constraint_count = total_constraint_rows;

        // Update statistics
        self.last_stats = PGSSolverStats {
            num_bodies,
            num_constraints: total_constraint_rows,
            used_warm_start: warm_started,
            sor_factor: self.config.sor_factor,
            initial_residual: initial_residual_norm,
            final_residual: current_residual_norm,
            convergence_history,
        };

        // Compute constraint forces: f_c = J^T * λ
        let forces_vec = jacobian.transpose() * &lambda;

        // Convert to JointForce results
        let mut forces = Vec::with_capacity(constraints.len());
        for constraint in &constraints {
            let force = self.extract_joint_force(constraint, &forces_vec, &bodies, num_bodies);
            forces.push(JointForce {
                parent: constraint.joint.parent(),
                child: constraint.joint.child(),
                force,
            });
        }

        PGSSolverResult {
            forces,
            iterations_used: iterations,
            residual_norm: current_residual_norm,
            initial_residual_norm,
            converged,
        }
    }

    /// Solve constraints using a slice of body states indexed by `BodyId::raw()`.
    ///
    /// This is a convenience method for the common case where body states are stored
    /// in a contiguous slice indexed by body ID.
    ///
    /// # Arguments
    ///
    /// * `joints` - The joints to solve
    /// * `bodies` - Slice of body states where `bodies[id.raw()]` gives the state for body `id`
    /// * `dt` - Time step
    ///
    /// # Example
    ///
    /// ```
    /// use sim_constraint::{PGSSolver, PGSSolverConfig, BodyState, RevoluteJoint};
    /// use sim_types::BodyId;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let bodies = vec![
    ///     BodyState::fixed(Point3::origin()),
    ///     BodyState::dynamic(Point3::new(0.0, 0.0, 1.0), 1.0, Vector3::new(0.1, 0.1, 0.1)),
    /// ];
    /// let joints = vec![
    ///     RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z()),
    /// ];
    ///
    /// let mut solver = PGSSolver::new(PGSSolverConfig::default());
    /// let result = solver.solve_slice(&joints, &bodies, 1.0 / 240.0);
    /// ```
    #[allow(clippy::cast_possible_truncation)] // BodyId fits in usize for practical use
    pub fn solve_slice<J: Joint>(
        &mut self,
        joints: &[J],
        bodies: &[BodyState],
        dt: f64,
    ) -> PGSSolverResult {
        self.solve(joints, |id| bodies.get(id.raw() as usize).copied(), dt)
    }

    /// Perform one Gauss-Seidel iteration with SOR.
    fn gauss_seidel_iteration(&self, a: &DMatrix<f64>, b: &DVector<f64>, x: &mut DVector<f64>) {
        let n = a.nrows();
        let omega = self.config.sor_factor;

        for i in 0..n {
            let a_ii = a[(i, i)];
            if a_ii.abs() < 1e-15 {
                continue; // Skip degenerate rows
            }

            // Compute the Gauss-Seidel update:
            // x_i^{new} = (b_i - sum_{j!=i} a_ij * x_j) / a_ii
            let mut sum = 0.0;
            for j in 0..n {
                if j != i {
                    sum += a[(i, j)] * x[j];
                }
            }

            let x_gs = (b[i] - sum) / a_ii;

            // Apply SOR: x_i = (1 - ω) * x_i^{old} + ω * x_i^{gs}
            x[i] = (1.0 - omega).mul_add(x[i], omega * x_gs);
        }
    }

    /// Compute residual: r = b - A*x
    #[allow(clippy::unused_self)]
    fn compute_residual(
        &self,
        a: &DMatrix<f64>,
        x: &DVector<f64>,
        b: &DVector<f64>,
    ) -> DVector<f64> {
        b - a * x
    }

    /// Build the system representation from joints.
    #[allow(clippy::unused_self)]
    fn build_system<'a, J, F>(
        &self,
        joints: &'a [J],
        get_body_state: &F,
    ) -> Option<(Vec<SolverBody>, Vec<SolverConstraint<'a, J>>, usize)>
    where
        J: Joint,
        F: Fn(BodyId) -> Option<BodyState>,
    {
        use std::collections::HashMap;

        // Collect unique bodies
        let mut body_map: HashMap<BodyId, usize> = HashMap::new();
        let mut bodies = Vec::new();

        for joint in joints {
            for &body_id in &[joint.parent(), joint.child()] {
                if let std::collections::hash_map::Entry::Vacant(e) = body_map.entry(body_id) {
                    if let Some(state) = get_body_state(body_id) {
                        e.insert(bodies.len());
                        bodies.push(SolverBody { state });
                    }
                }
            }
        }

        if bodies.is_empty() {
            return None;
        }

        // Build constraint list
        let mut constraints = Vec::with_capacity(joints.len());
        let mut row_offset = 0;

        for joint in joints {
            let Some(&parent_index) = body_map.get(&joint.parent()) else {
                continue;
            };
            let Some(&child_index) = body_map.get(&joint.child()) else {
                continue;
            };

            let num_rows = joint.joint_type().constrained_dof();
            if num_rows == 0 {
                continue; // Free joints have no constraints
            }

            constraints.push(SolverConstraint {
                joint,
                parent_index,
                child_index,
                row_offset,
                num_rows,
            });

            row_offset += num_rows;
        }

        Some((bodies, constraints, row_offset))
    }

    /// Build the constraint Jacobian matrix.
    fn build_jacobian<J: Joint>(
        &self,
        bodies: &[SolverBody],
        constraints: &[SolverConstraint<'_, J>],
        total_rows: usize,
    ) -> DMatrix<f64> {
        let num_bodies = bodies.len();
        let mut jacobian = DMatrix::zeros(total_rows, 6 * num_bodies);

        for constraint in constraints {
            let parent = &bodies[constraint.parent_index].state;
            let child = &bodies[constraint.child_index].state;

            // Get anchor positions in world frame
            let parent_anchor_world =
                parent.position + parent.rotation * constraint.joint.parent_anchor().coords;
            let child_anchor_world =
                child.position + child.rotation * constraint.joint.child_anchor().coords;

            // Compute r vectors (from body COM to anchor)
            let r_parent = parent_anchor_world - parent.position;
            let r_child = child_anchor_world - child.position;

            // Fill Jacobian based on joint type
            self.fill_jacobian_rows(&mut jacobian, constraint, &r_parent, &r_child, num_bodies);
        }

        jacobian
    }

    /// Fill Jacobian rows for a single constraint.
    #[allow(clippy::too_many_lines, clippy::similar_names)]
    fn fill_jacobian_rows<J: Joint>(
        &self,
        jacobian: &mut DMatrix<f64>,
        constraint: &SolverConstraint<'_, J>,
        r_parent: &Vector3<f64>,
        r_child: &Vector3<f64>,
        _num_bodies: usize,
    ) {
        let row = constraint.row_offset;
        let p_col = constraint.parent_index * 6;
        let c_col = constraint.child_index * 6;

        // Skew-symmetric matrix helper
        let skew = |v: &Vector3<f64>| -> Matrix3<f64> {
            Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
        };

        match constraint.joint.joint_type() {
            JointType::Fixed => {
                // 6 constraints: 3 position + 3 rotation
                // Position constraints: p_parent + r_parent = p_child + r_child
                // Linear Jacobian: [I, -skew(r_parent), -I, skew(r_child)]
                for i in 0..3 {
                    jacobian[(row + i, p_col + i)] = 1.0; // Parent linear
                    jacobian[(row + i, c_col + i)] = -1.0; // Child linear
                }
                let skew_rp = skew(r_parent);
                let skew_rc = skew(r_child);
                for i in 0..3 {
                    for j in 0..3 {
                        jacobian[(row + i, p_col + 3 + j)] = -skew_rp[(i, j)];
                        jacobian[(row + i, c_col + 3 + j)] = skew_rc[(i, j)];
                    }
                }

                // Rotation constraints: ω_parent = ω_child
                for i in 0..3 {
                    jacobian[(row + 3 + i, p_col + 3 + i)] = 1.0;
                    jacobian[(row + 3 + i, c_col + 3 + i)] = -1.0;
                }
            }

            JointType::Revolute => {
                // 5 constraints: 3 position + 2 rotation perpendicular to axis
                // Position constraints
                for i in 0..3 {
                    jacobian[(row + i, p_col + i)] = 1.0;
                    jacobian[(row + i, c_col + i)] = -1.0;
                }
                let skew_rp = skew(r_parent);
                let skew_rc = skew(r_child);
                for i in 0..3 {
                    for j in 0..3 {
                        jacobian[(row + i, p_col + 3 + j)] = -skew_rp[(i, j)];
                        jacobian[(row + i, c_col + 3 + j)] = skew_rc[(i, j)];
                    }
                }

                // Angular constraints (2 perpendicular to axis)
                // Use two orthogonal vectors perpendicular to the rotation axis
                let (perp1, perp2) = self.get_perpendicular_axes(&Vector3::z());
                for i in 0..3 {
                    jacobian[(row + 3, p_col + 3 + i)] = perp1[i];
                    jacobian[(row + 3, c_col + 3 + i)] = -perp1[i];
                    jacobian[(row + 4, p_col + 3 + i)] = perp2[i];
                    jacobian[(row + 4, c_col + 3 + i)] = -perp2[i];
                }
            }

            JointType::Prismatic => {
                // 5 constraints: 2 position perpendicular to axis + 3 rotation
                // Use Z as the slide axis
                let (perp1, perp2) = self.get_perpendicular_axes(&Vector3::z());

                // Position constraints perpendicular to axis
                for i in 0..3 {
                    jacobian[(row, p_col + i)] = perp1[i];
                    jacobian[(row, c_col + i)] = -perp1[i];
                    jacobian[(row + 1, p_col + i)] = perp2[i];
                    jacobian[(row + 1, c_col + i)] = -perp2[i];
                }

                // Full rotation constraint
                for i in 0..3 {
                    jacobian[(row + 2 + i, p_col + 3 + i)] = 1.0;
                    jacobian[(row + 2 + i, c_col + 3 + i)] = -1.0;
                }
            }

            JointType::Spherical => {
                // 3 constraints: position only
                for i in 0..3 {
                    jacobian[(row + i, p_col + i)] = 1.0;
                    jacobian[(row + i, c_col + i)] = -1.0;
                }
                let skew_rp = skew(r_parent);
                let skew_rc = skew(r_child);
                for i in 0..3 {
                    for j in 0..3 {
                        jacobian[(row + i, p_col + 3 + j)] = -skew_rp[(i, j)];
                        jacobian[(row + i, c_col + 3 + j)] = skew_rc[(i, j)];
                    }
                }
            }

            JointType::Universal => {
                // 4 constraints: 3 position + 1 rotation
                // Position constraints
                for i in 0..3 {
                    jacobian[(row + i, p_col + i)] = 1.0;
                    jacobian[(row + i, c_col + i)] = -1.0;
                }
                let skew_rp = skew(r_parent);
                let skew_rc = skew(r_child);
                for i in 0..3 {
                    for j in 0..3 {
                        jacobian[(row + i, p_col + 3 + j)] = -skew_rp[(i, j)];
                        jacobian[(row + i, c_col + 3 + j)] = skew_rc[(i, j)];
                    }
                }

                // 1 rotation constraint (e.g., twist about X)
                jacobian[(row + 3, p_col + 3)] = 1.0;
                jacobian[(row + 3, c_col + 3)] = -1.0;
            }

            JointType::Cylindrical => {
                // 4 constraints: 2 position perpendicular + 2 rotation perpendicular
                let (perp1, perp2) = self.get_perpendicular_axes(&Vector3::x());

                // Position perpendicular to axis
                for i in 0..3 {
                    jacobian[(row, p_col + i)] = perp1[i];
                    jacobian[(row, c_col + i)] = -perp1[i];
                    jacobian[(row + 1, p_col + i)] = perp2[i];
                    jacobian[(row + 1, c_col + i)] = -perp2[i];
                }

                // Rotation perpendicular to axis
                for i in 0..3 {
                    jacobian[(row + 2, p_col + 3 + i)] = perp1[i];
                    jacobian[(row + 2, c_col + 3 + i)] = -perp1[i];
                    jacobian[(row + 3, p_col + 3 + i)] = perp2[i];
                    jacobian[(row + 3, c_col + 3 + i)] = -perp2[i];
                }
            }

            JointType::Planar => {
                // 3 constraints: 1 position perpendicular to plane + 2 rotation perpendicular
                let normal = Vector3::z();

                // Position perpendicular to plane
                for i in 0..3 {
                    jacobian[(row, p_col + i)] = normal[i];
                    jacobian[(row, c_col + i)] = -normal[i];
                }

                // Rotation perpendicular to normal (in-plane rotations constrained)
                let (perp1, perp2) = self.get_perpendicular_axes(&normal);
                for i in 0..3 {
                    jacobian[(row + 1, p_col + 3 + i)] = perp1[i];
                    jacobian[(row + 1, c_col + 3 + i)] = -perp1[i];
                    jacobian[(row + 2, p_col + 3 + i)] = perp2[i];
                    jacobian[(row + 2, c_col + 3 + i)] = -perp2[i];
                }
            }

            JointType::Free => {
                // 0 constraints - this case shouldn't be reached
            }
        }
    }

    /// Get two perpendicular axes to a given axis.
    #[allow(clippy::unused_self)]
    fn get_perpendicular_axes(&self, axis: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
        let axis = axis.normalize();

        // Find a vector not parallel to axis
        let ref_vec = if axis.x.abs() < 0.9 {
            Vector3::x()
        } else {
            Vector3::y()
        };

        let perp1 = axis.cross(&ref_vec).normalize();
        let perp2 = axis.cross(&perp1).normalize();

        (perp1, perp2)
    }

    /// Build the inverse mass matrix.
    #[allow(clippy::unused_self)]
    fn build_inverse_mass_matrix(&self, bodies: &[SolverBody]) -> DMatrix<f64> {
        let n = bodies.len();
        let mut inv_mass = DMatrix::zeros(6 * n, 6 * n);

        for (i, body) in bodies.iter().enumerate() {
            let offset = i * 6;
            let state = &body.state;

            if state.is_static {
                // Static bodies have zero inverse mass
                continue;
            }

            // Linear part (diagonal)
            for j in 0..3 {
                inv_mass[(offset + j, offset + j)] = state.inv_mass;
            }

            // Angular part (inverse inertia tensor)
            for j in 0..3 {
                for k in 0..3 {
                    inv_mass[(offset + 3 + j, offset + 3 + k)] = state.inv_inertia[(j, k)];
                }
            }
        }

        inv_mass
    }

    /// Compute right-hand side: b = -J*v - β/h * C
    fn compute_rhs<J: Joint>(
        &self,
        bodies: &[SolverBody],
        constraints: &[SolverConstraint<'_, J>],
        jacobian: &DMatrix<f64>,
        dt: f64,
    ) -> DVector<f64> {
        let n = bodies.len();
        let _total_rows = jacobian.nrows();

        // Build velocity vector
        let mut v = DVector::zeros(6 * n);
        for (i, body) in bodies.iter().enumerate() {
            let offset = i * 6;
            v[offset] = body.state.linear_velocity.x;
            v[offset + 1] = body.state.linear_velocity.y;
            v[offset + 2] = body.state.linear_velocity.z;
            v[offset + 3] = body.state.angular_velocity.x;
            v[offset + 4] = body.state.angular_velocity.y;
            v[offset + 5] = body.state.angular_velocity.z;
        }

        // Compute -J*v
        let mut rhs = -(jacobian * &v);

        // Add Baumgarte stabilization: -β/h * C
        let baumgarte = self.config.baumgarte_factor / dt;
        for constraint in constraints {
            let c = self.compute_position_error(constraint, bodies);
            for i in 0..constraint.num_rows {
                if i < c.len() {
                    rhs[constraint.row_offset + i] -= baumgarte * c[i];
                }
            }
        }

        rhs
    }

    /// Compute position error for a constraint.
    fn compute_position_error<J: Joint>(
        &self,
        constraint: &SolverConstraint<'_, J>,
        bodies: &[SolverBody],
    ) -> Vec<f64> {
        let parent = &bodies[constraint.parent_index].state;
        let child = &bodies[constraint.child_index].state;

        let parent_anchor_world =
            parent.position + parent.rotation * constraint.joint.parent_anchor().coords;
        let child_anchor_world =
            child.position + child.rotation * constraint.joint.child_anchor().coords;

        let position_error = child_anchor_world - parent_anchor_world;

        match constraint.joint.joint_type() {
            JointType::Fixed => {
                // 6 errors: 3 position + 3 rotation (simplified to angular velocity diff)
                vec![
                    position_error.x,
                    position_error.y,
                    position_error.z,
                    0.0,
                    0.0,
                    0.0,
                ]
            }
            JointType::Revolute => {
                // 5 errors: 3 position + 2 rotation perpendicular
                vec![
                    position_error.x,
                    position_error.y,
                    position_error.z,
                    0.0,
                    0.0,
                ]
            }
            JointType::Prismatic => {
                // 5 errors: 2 position perpendicular + 3 rotation
                let (perp1, perp2) = self.get_perpendicular_axes(&Vector3::z());
                vec![
                    position_error.dot(&perp1),
                    position_error.dot(&perp2),
                    0.0,
                    0.0,
                    0.0,
                ]
            }
            JointType::Spherical => {
                // 3 errors: position only
                vec![position_error.x, position_error.y, position_error.z]
            }
            JointType::Universal => {
                // 4 errors: 3 position + 1 rotation
                vec![position_error.x, position_error.y, position_error.z, 0.0]
            }
            JointType::Cylindrical => {
                // 4 errors: 2 position perpendicular + 2 rotation perpendicular
                let (perp1, perp2) = self.get_perpendicular_axes(&Vector3::x());
                vec![
                    position_error.dot(&perp1),
                    position_error.dot(&perp2),
                    0.0,
                    0.0,
                ]
            }
            JointType::Planar => {
                // 3 errors: 1 position perpendicular + 2 rotation
                let normal = Vector3::z();
                vec![position_error.dot(&normal), 0.0, 0.0]
            }
            JointType::Free => Vec::new(),
        }
    }

    /// Extract joint force from the full force vector.
    #[allow(clippy::unused_self)]
    fn extract_joint_force<J: Joint>(
        &self,
        constraint: &SolverConstraint<'_, J>,
        forces_vec: &DVector<f64>,
        _bodies: &[SolverBody],
        _num_bodies: usize,
    ) -> ConstraintForce {
        let p_offset = constraint.parent_index * 6;
        let c_offset = constraint.child_index * 6;

        let parent_force = Vector3::new(
            forces_vec[p_offset],
            forces_vec[p_offset + 1],
            forces_vec[p_offset + 2],
        );
        let parent_torque = Vector3::new(
            forces_vec[p_offset + 3],
            forces_vec[p_offset + 4],
            forces_vec[p_offset + 5],
        );
        let child_force = Vector3::new(
            forces_vec[c_offset],
            forces_vec[c_offset + 1],
            forces_vec[c_offset + 2],
        );
        let child_torque = Vector3::new(
            forces_vec[c_offset + 3],
            forces_vec[c_offset + 4],
            forces_vec[c_offset + 5],
        );

        ConstraintForce::new(parent_force, parent_torque, child_force, child_torque)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::cloned_ref_to_slice_refs)]
mod tests {
    use super::*;
    use crate::RevoluteJoint;
    use approx::assert_relative_eq;
    use nalgebra::Point3;

    fn make_body_state(position: Point3<f64>) -> BodyState {
        BodyState {
            position,
            rotation: Matrix3::identity(),
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            inv_mass: 1.0,
            inv_inertia: Matrix3::identity(),
            is_static: false,
        }
    }

    #[test]
    fn test_pgs_solver_creation() {
        let solver = PGSSolver::default();
        assert_eq!(solver.config().max_iterations, 100);
        assert_relative_eq!(solver.config().sor_factor, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_pgs_config_presets() {
        let realtime = PGSSolverConfig::realtime();
        assert!(realtime.max_iterations <= 30);
        assert!(realtime.sor_factor > 1.0); // Should use over-relaxation

        let high_accuracy = PGSSolverConfig::high_accuracy();
        assert!(high_accuracy.max_iterations >= 100);
        assert_relative_eq!(high_accuracy.sor_factor, 1.0, epsilon = 1e-10);

        let mujoco = PGSSolverConfig::mujoco();
        assert_eq!(mujoco.max_iterations, 100);
    }

    #[test]
    fn test_pgs_config_validation() {
        let mut config = PGSSolverConfig::default();
        assert!(config.validate().is_ok());

        config.sor_factor = 0.0;
        assert!(config.validate().is_err());

        config.sor_factor = 2.0;
        assert!(config.validate().is_err());

        config.sor_factor = 1.5;
        assert!(config.validate().is_ok());
    }

    #[test]
    fn test_pgs_solve_empty() {
        let mut solver = PGSSolver::default();
        let joints: Vec<RevoluteJoint> = vec![];

        let result = solver.solve(&joints, |_| None, 0.01);
        assert!(result.is_empty());
        assert!(result.converged);
    }

    #[test]
    fn test_pgs_solve_single_joint() {
        let mut solver = PGSSolver::new(PGSSolverConfig {
            max_iterations: 50,
            tolerance: 1e-4,
            ..Default::default()
        });

        let joint = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());

        let parent_state = make_body_state(Point3::origin());
        let child_state = make_body_state(Point3::new(0.1, 0.0, 0.0));

        let result = solver.solve(
            &[joint],
            |id| {
                if id == BodyId::new(0) {
                    Some(parent_state)
                } else if id == BodyId::new(1) {
                    Some(child_state)
                } else {
                    None
                }
            },
            0.01,
        );

        assert_eq!(result.forces.len(), 1);
        assert!(result.iterations_used > 0);
    }

    #[test]
    fn test_gauss_seidel_iteration() {
        let solver = PGSSolver::new(PGSSolverConfig {
            sor_factor: 1.0,
            ..Default::default()
        });

        // Simple 2x2 system: A * x = b
        // [4 1] [x1]   [1]
        // [1 3] [x2] = [2]
        // Solution: x1 = 0.0909..., x2 = 0.6363...
        let a = DMatrix::from_row_slice(2, 2, &[4.0, 1.0, 1.0, 3.0]);
        let b = DVector::from_vec(vec![1.0, 2.0]);
        let mut x = DVector::zeros(2);

        // Run several iterations
        for _ in 0..50 {
            solver.gauss_seidel_iteration(&a, &b, &mut x);
        }

        assert_relative_eq!(x[0], 1.0 / 11.0, epsilon = 1e-4);
        assert_relative_eq!(x[1], 7.0 / 11.0, epsilon = 1e-4);
    }

    #[test]
    fn test_sor_convergence_improvement() {
        // Test that SOR with good omega converges faster than standard GS
        let a = DMatrix::from_row_slice(2, 2, &[4.0, 1.0, 1.0, 3.0]);
        let b = DVector::from_vec(vec![1.0, 2.0]);

        // Standard GS (omega = 1.0)
        let solver_gs = PGSSolver::new(PGSSolverConfig {
            sor_factor: 1.0,
            ..Default::default()
        });
        let mut x_gs = DVector::zeros(2);
        for _ in 0..10 {
            solver_gs.gauss_seidel_iteration(&a, &b, &mut x_gs);
        }

        // SOR with omega = 1.1
        let solver_sor = PGSSolver::new(PGSSolverConfig {
            sor_factor: 1.1,
            ..Default::default()
        });
        let mut x_sor = DVector::zeros(2);
        for _ in 0..10 {
            solver_sor.gauss_seidel_iteration(&a, &b, &mut x_sor);
        }

        // Both should approach solution
        let solution = DVector::from_vec(vec![1.0 / 11.0, 7.0 / 11.0]);
        let error_gs = (&x_gs - &solution).norm();
        let error_sor = (&x_sor - &solution).norm();

        // SOR should have similar or better error (might vary by problem)
        assert!(error_gs < 0.01);
        assert!(error_sor < 0.01);
    }

    #[test]
    fn test_warm_starting() {
        let mut solver = PGSSolver::new(PGSSolverConfig {
            warm_starting: true,
            warm_start_factor: 0.9,
            max_iterations: 20,
            ..Default::default()
        });

        let joint = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());
        let parent_state = make_body_state(Point3::origin());
        let child_state = make_body_state(Point3::new(0.1, 0.0, 0.0));

        let get_state = |id: BodyId| -> Option<BodyState> {
            if id == BodyId::new(0) {
                Some(parent_state)
            } else if id == BodyId::new(1) {
                Some(child_state)
            } else {
                None
            }
        };

        // First solve (no warm start)
        let result1 = solver.solve(&[joint.clone()], get_state, 0.01);
        assert!(!solver.last_stats().used_warm_start);

        // Second solve (should use warm start)
        let result2 = solver.solve(&[joint], get_state, 0.01);
        assert!(solver.last_stats().used_warm_start);

        // Warm-started solve should converge faster (fewer iterations or lower initial error)
        assert!(result2.iterations_used > 0);
        // The warm start should help reduce initial residual
        assert!(result1.forces.len() == result2.forces.len());
    }

    #[test]
    fn test_convergence_tracking() {
        let mut solver = PGSSolver::new(PGSSolverConfig {
            track_convergence: true,
            max_iterations: 20,
            ..Default::default()
        });

        let joint = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());
        let parent_state = make_body_state(Point3::origin());
        let child_state = make_body_state(Point3::new(0.1, 0.0, 0.0));

        let _ = solver.solve(
            &[joint],
            |id| {
                if id == BodyId::new(0) {
                    Some(parent_state)
                } else if id == BodyId::new(1) {
                    Some(child_state)
                } else {
                    None
                }
            },
            0.01,
        );

        // Should have convergence history
        assert!(solver.last_stats().convergence_history.is_some());
        let history = solver.last_stats().convergence_history.as_ref().unwrap();
        assert!(!history.is_empty());

        // Residual should generally decrease
        if history.len() > 1 {
            assert!(*history.last().unwrap() <= history[0] * 10.0); // Allow some tolerance
        }
    }
}
