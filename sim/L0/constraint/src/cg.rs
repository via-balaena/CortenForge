//! Conjugate Gradient solver for constraint systems.
//!
//! This module implements an iterative Conjugate Gradient (CG) solver for
//! symmetric positive definite constraint systems. CG is particularly effective
//! for large sparse systems where direct methods become expensive.
//!
//! # Algorithm
//!
//! The CG solver iteratively solves A * x = b where A is symmetric positive definite:
//!
//! ```text
//! 1. r_0 = b - A * x_0 (initial residual)
//! 2. p_0 = r_0 (initial search direction)
//! 3. For k = 0, 1, 2, ...:
//!    α_k = (r_k · r_k) / (p_k · A * p_k)
//!    x_{k+1} = x_k + α_k * p_k
//!    r_{k+1} = r_k - α_k * A * p_k
//!    β_k = (r_{k+1} · r_{k+1}) / (r_k · r_k)
//!    p_{k+1} = r_{k+1} + β_k * p_k
//! ```
//!
//! # Preconditioning
//!
//! For faster convergence, CG can use preconditioning. This module supports:
//!
//! - **None**: No preconditioning (baseline)
//! - **Jacobi**: Diagonal preconditioning (fast, modest improvement)
//! - **Block Jacobi**: Block-diagonal preconditioning (better for constraint systems)
//!
//! # When to Use CG
//!
//! CG is well-suited for:
//! - Large systems (>100 constraints)
//! - Sparse systems where direct factorization is expensive
//! - Systems where an approximate solution is acceptable
//! - Iterative refinement in real-time applications
//!
//! For small dense systems, direct methods (Cholesky) are typically faster.
//!
//! # Example
//!
//! ```ignore
//! use sim_constraint::{CGSolver, CGSolverConfig, Preconditioner};
//!
//! let config = CGSolverConfig {
//!     max_iterations: 100,
//!     tolerance: 1e-6,
//!     preconditioner: Preconditioner::Jacobi,
//!     ..Default::default()
//! };
//!
//! let mut solver = CGSolver::new(config);
//! let result = solver.solve(&joints, get_body_state, dt);
//! ```

use nalgebra::{DMatrix, DVector, Matrix3, Vector3};
use sim_types::BodyId;

use crate::{BodyState, ConstraintForce, Joint, JointForce, JointType};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Preconditioner type for CG solver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Preconditioner {
    /// No preconditioning.
    #[default]
    None,
    /// Jacobi (diagonal) preconditioning.
    /// Fast to compute, provides modest convergence improvement.
    Jacobi,
    /// Block Jacobi preconditioning.
    /// Uses the block structure of constraint systems for better conditioning.
    /// Each joint's constraint rows form a dense block; inverting these blocks
    /// captures within-joint coupling that scalar Jacobi ignores.
    BlockJacobi,
}

/// Internal storage for precomputed preconditioner data.
///
/// Diagonal preconditioners (None, Jacobi) store a vector of scaling factors
/// applied via element-wise multiply. Block Jacobi stores inverted dense blocks
/// for each constraint group, applied via block-wise matrix-vector multiply.
#[derive(Debug, Clone)]
enum PreconditionerData {
    /// Element-wise scaling: `z_i = diag_i * r_i`
    Diagonal(DVector<f64>),
    /// Block-diagonal inverse: `z[rows] = block_inv * r[rows]` for each block.
    Block {
        /// Inverted blocks, one per constraint group.
        block_inverses: Vec<DMatrix<f64>>,
        /// Row offset for each block.
        offsets: Vec<usize>,
        /// Number of rows for each block.
        sizes: Vec<usize>,
        /// Total system size.
        n: usize,
    },
}

/// Configuration for the Conjugate Gradient solver.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CGSolverConfig {
    /// Maximum number of CG iterations.
    pub max_iterations: usize,

    /// Convergence tolerance for residual norm.
    /// Iteration stops when ||r|| / ||b|| < tolerance.
    pub tolerance: f64,

    /// Baumgarte stabilization factor (0-1).
    /// Higher values correct position errors faster but may add energy.
    pub baumgarte_factor: f64,

    /// Regularization factor for numerical stability.
    /// Small positive value added to diagonal of effective mass matrix.
    pub regularization: f64,

    /// Preconditioner type.
    pub preconditioner: Preconditioner,

    /// Enable warm starting from previous solution.
    pub warm_starting: bool,

    /// Scaling factor for warm start values (0-1).
    pub warm_start_factor: f64,

    /// Minimum iterations before checking convergence.
    /// Useful for preventing premature termination.
    pub min_iterations: usize,
}

impl Default for CGSolverConfig {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            tolerance: 1e-6,
            baumgarte_factor: 0.2,
            regularization: 1e-9,
            preconditioner: Preconditioner::Jacobi,
            warm_starting: true,
            warm_start_factor: 0.9,
            min_iterations: 3,
        }
    }
}

impl CGSolverConfig {
    /// High-accuracy configuration for precise simulations.
    #[must_use]
    pub fn high_accuracy() -> Self {
        Self {
            max_iterations: 200,
            tolerance: 1e-10,
            baumgarte_factor: 0.1,
            regularization: 1e-12,
            preconditioner: Preconditioner::BlockJacobi,
            warm_starting: true,
            warm_start_factor: 0.95,
            min_iterations: 5,
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
            preconditioner: Preconditioner::Jacobi,
            warm_starting: true,
            warm_start_factor: 0.85,
            min_iterations: 2,
        }
    }

    /// Configuration optimized for large systems (1000+ constraints).
    #[must_use]
    pub fn large_system() -> Self {
        Self {
            max_iterations: 100,
            tolerance: 1e-6,
            baumgarte_factor: 0.2,
            regularization: 1e-9,
            preconditioner: Preconditioner::BlockJacobi,
            warm_starting: true,
            warm_start_factor: 0.9,
            min_iterations: 5,
        }
    }

    /// Set the preconditioner type.
    #[must_use]
    pub const fn with_preconditioner(mut self, preconditioner: Preconditioner) -> Self {
        self.preconditioner = preconditioner;
        self
    }

    /// Enable or disable warm starting.
    #[must_use]
    pub const fn with_warm_starting(mut self, enabled: bool) -> Self {
        self.warm_starting = enabled;
        self
    }

    /// Set the maximum iterations.
    #[must_use]
    pub const fn with_max_iterations(mut self, max_iter: usize) -> Self {
        self.max_iterations = max_iter;
        self
    }
}

/// Result of CG solver iteration.
#[derive(Debug, Clone)]
pub struct CGSolverResult {
    /// Forces for each joint.
    pub forces: Vec<JointForce>,

    /// Number of CG iterations used.
    pub iterations_used: usize,

    /// Final residual norm.
    pub residual_norm: f64,

    /// Initial residual norm (for convergence ratio).
    pub initial_residual_norm: f64,

    /// Whether the solver converged.
    pub converged: bool,
}

impl CGSolverResult {
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

/// Statistics from a CG solver run.
#[derive(Debug, Clone, Default)]
pub struct CGSolverStats {
    /// Number of bodies in the system.
    pub num_bodies: usize,
    /// Number of constraint rows.
    pub num_constraints: usize,
    /// Whether warm starting was applied.
    pub used_warm_start: bool,
    /// Preconditioner used.
    pub preconditioner: Preconditioner,
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

/// Conjugate Gradient constraint solver.
///
/// This solver uses the Conjugate Gradient method to iteratively solve
/// the constraint system. It's particularly effective for large sparse
/// systems where direct factorization is expensive.
#[derive(Debug, Clone)]
pub struct CGSolver {
    /// Solver configuration.
    config: CGSolverConfig,

    /// Cached Lagrange multipliers for warm starting.
    cached_lambda: Vec<f64>,

    /// Previous constraint count (for cache validation).
    prev_constraint_count: usize,

    /// Statistics from the last solve.
    last_stats: CGSolverStats,

    /// Track convergence history (optional, for debugging).
    track_convergence: bool,
}

#[allow(clippy::unused_self)]
impl CGSolver {
    /// Create a new CG solver with the given configuration.
    #[must_use]
    pub fn new(config: CGSolverConfig) -> Self {
        Self {
            config,
            cached_lambda: Vec::new(),
            prev_constraint_count: 0,
            last_stats: CGSolverStats::default(),
            track_convergence: false,
        }
    }

    /// Create a solver with default configuration.
    #[must_use]
    pub fn default_solver() -> Self {
        Self::new(CGSolverConfig::default())
    }

    /// Get the configuration.
    #[must_use]
    pub fn config(&self) -> &CGSolverConfig {
        &self.config
    }

    /// Get statistics from the last solve.
    #[must_use]
    pub fn last_stats(&self) -> &CGSolverStats {
        &self.last_stats
    }

    /// Enable convergence history tracking.
    pub fn set_track_convergence(&mut self, track: bool) {
        self.track_convergence = track;
    }

    /// Set the preconditioner.
    pub fn set_preconditioner(&mut self, preconditioner: Preconditioner) {
        self.config.preconditioner = preconditioner;
    }

    /// Enable or disable warm starting.
    pub fn set_warm_starting(&mut self, enabled: bool) {
        self.config.warm_starting = enabled;
    }

    /// Get warm start initial guess for lambda.
    fn get_warm_start_lambda(&self, size: usize) -> Option<DVector<f64>> {
        if !self.config.warm_starting {
            return None;
        }

        if self.cached_lambda.len() != size || self.prev_constraint_count != size {
            return None;
        }

        let factor = self.config.warm_start_factor;
        Some(DVector::from_iterator(
            size,
            self.cached_lambda.iter().map(|&v| v * factor),
        ))
    }

    /// Cache lambda values for next frame's warm start.
    fn cache_lambda(&mut self, lambda: &DVector<f64>, constraint_count: usize) {
        self.cached_lambda.clear();
        self.cached_lambda.extend(lambda.iter());
        self.prev_constraint_count = constraint_count;
    }

    /// Solve constraints using the Conjugate Gradient method.
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
    pub fn solve<J, F>(&mut self, joints: &[J], get_body_state: F, dt: f64) -> CGSolverResult
    where
        J: Joint,
        F: Fn(BodyId) -> Option<BodyState>,
    {
        if joints.is_empty() {
            return CGSolverResult::empty();
        }

        // Build body list and constraint list
        let Some((bodies, constraints, total_constraint_rows)) =
            self.build_system(joints, &get_body_state)
        else {
            return CGSolverResult::empty();
        };

        if total_constraint_rows == 0 {
            return CGSolverResult::empty();
        }

        let num_bodies = bodies.len();

        // Build Jacobian and effective mass
        let jacobian = self.build_jacobian(&bodies, &constraints, total_constraint_rows);
        let inv_mass = self.build_inverse_mass_matrix(&bodies);

        // Compute effective mass: A = J * M^-1 * J^T + reg * I
        let j_minv = &jacobian * &inv_mass;
        let mut effective_mass = &j_minv * jacobian.transpose();

        // Add regularization
        for i in 0..effective_mass.nrows() {
            effective_mass[(i, i)] += self.config.regularization;
        }

        // Compute RHS: b = -J * v - β/h * C
        let constraint_error = self.compute_constraint_error(&bodies, &constraints, dt);

        // Get initial guess (warm start or zero)
        let initial_lambda = self
            .get_warm_start_lambda(total_constraint_rows)
            .unwrap_or_else(|| DVector::zeros(total_constraint_rows));

        // Extract constraint block boundaries for block Jacobi preconditioner
        let constraint_blocks: Vec<(usize, usize)> = constraints
            .iter()
            .map(|c| (c.row_offset, c.num_rows))
            .collect();

        // Build preconditioner
        let preconditioner = self.build_preconditioner(&effective_mass, &constraint_blocks);

        // Solve using CG
        let (lambda, iterations, residual_norm, initial_residual, converged, history) = self
            .cg_solve(
                &effective_mass,
                &constraint_error,
                &initial_lambda,
                &preconditioner,
            );

        // Cache for warm starting
        self.cache_lambda(&lambda, total_constraint_rows);

        // Update statistics
        self.last_stats = CGSolverStats {
            num_bodies,
            num_constraints: total_constraint_rows,
            used_warm_start: self.prev_constraint_count == total_constraint_rows
                && self.config.warm_starting,
            preconditioner: self.config.preconditioner,
            initial_residual,
            final_residual: residual_norm,
            convergence_history: if self.track_convergence {
                Some(history)
            } else {
                None
            },
        };

        // Compute forces: f = J^T * lambda
        let forces_vec = jacobian.transpose() * &lambda;

        // Convert to JointForce results
        let mut forces = Vec::with_capacity(constraints.len());
        for constraint in &constraints {
            let force = self.extract_joint_force(constraint, &forces_vec, num_bodies);
            forces.push(JointForce {
                parent: constraint.joint.parent(),
                child: constraint.joint.child(),
                force,
            });
        }

        CGSolverResult {
            forces,
            iterations_used: iterations,
            residual_norm,
            initial_residual_norm: initial_residual,
            converged,
        }
    }

    /// Preconditioned Conjugate Gradient solver.
    ///
    /// Uses standard mathematical notation: x (solution), r (residual), p (search direction),
    /// z (preconditioned residual), a (matrix), b (RHS).
    #[allow(clippy::type_complexity, clippy::many_single_char_names)]
    fn cg_solve(
        &self,
        a: &DMatrix<f64>,
        b: &DVector<f64>,
        x0: &DVector<f64>,
        preconditioner: &PreconditionerData,
    ) -> (DVector<f64>, usize, f64, f64, bool, Vec<f64>) {
        let mut x = x0.clone();
        let mut history = Vec::new();

        // Initial residual: r = b - A * x
        let mut r = b - a * &x;
        let initial_residual = r.norm();

        if self.track_convergence {
            history.push(initial_residual);
        }

        // Check if already converged
        let b_norm = b.norm().max(1e-15);
        if initial_residual / b_norm < self.config.tolerance {
            return (x, 0, initial_residual, initial_residual, true, history);
        }

        // Apply preconditioner: z = M^-1 * r
        let mut z = self.apply_preconditioner(&r, preconditioner);

        // Initial search direction
        let mut p = z.clone();

        // r · z
        let mut r_dot_z = r.dot(&z);

        let mut converged = false;
        let mut iterations = 0;

        for k in 0..self.config.max_iterations {
            iterations = k + 1;

            // A * p
            let ap = a * &p;

            // α = (r · z) / (p · A * p)
            let p_dot_ap = p.dot(&ap);
            if p_dot_ap.abs() < 1e-30 {
                // Breakdown: p is nearly orthogonal to A * p
                break;
            }
            let alpha = r_dot_z / p_dot_ap;

            // x = x + α * p
            x += alpha * &p;

            // r = r - α * A * p
            r -= alpha * &ap;

            let residual_norm = r.norm();

            if self.track_convergence {
                history.push(residual_norm);
            }

            // Check convergence (after minimum iterations)
            if k >= self.config.min_iterations && residual_norm / b_norm < self.config.tolerance {
                converged = true;
                break;
            }

            // Apply preconditioner: z = M^-1 * r
            z = self.apply_preconditioner(&r, preconditioner);

            // β = (r_{k+1} · z_{k+1}) / (r_k · z_k)
            let r_dot_z_new = r.dot(&z);
            let beta = r_dot_z_new / r_dot_z.max(1e-30);
            r_dot_z = r_dot_z_new;

            // p = z + β * p
            p = &z + beta * &p;
        }

        let final_residual = r.norm();
        (
            x,
            iterations,
            final_residual,
            initial_residual,
            converged,
            history,
        )
    }

    /// Build the preconditioner from the effective mass matrix.
    ///
    /// For Block Jacobi, `constraint_blocks` provides `(row_offset, num_rows)` pairs
    /// describing the block structure. Each block corresponds to one joint's constraint rows.
    fn build_preconditioner(
        &self,
        a: &DMatrix<f64>,
        constraint_blocks: &[(usize, usize)],
    ) -> PreconditionerData {
        let n = a.nrows();

        match self.config.preconditioner {
            Preconditioner::None => PreconditionerData::Diagonal(DVector::from_element(n, 1.0)),
            Preconditioner::Jacobi => {
                // Diagonal preconditioner: M_ii = 1 / A_ii
                PreconditionerData::Diagonal(DVector::from_fn(n, |i, _| {
                    let diag = a[(i, i)];
                    if diag.abs() > 1e-15 { 1.0 / diag } else { 1.0 }
                }))
            }
            Preconditioner::BlockJacobi => {
                if constraint_blocks.is_empty() {
                    // No block info — fall back to scalar Jacobi
                    return PreconditionerData::Diagonal(DVector::from_fn(n, |i, _| {
                        let diag = a[(i, i)];
                        if diag.abs() > 1e-15 { 1.0 / diag } else { 1.0 }
                    }));
                }

                let mut block_inverses = Vec::with_capacity(constraint_blocks.len());
                let mut offsets = Vec::with_capacity(constraint_blocks.len());
                let mut sizes = Vec::with_capacity(constraint_blocks.len());

                for &(offset, size) in constraint_blocks {
                    offsets.push(offset);
                    sizes.push(size);

                    if size == 0 {
                        block_inverses.push(DMatrix::zeros(0, 0));
                        continue;
                    }

                    // Extract the diagonal block A[offset..offset+size, offset..offset+size]
                    let block = a.view((offset, offset), (size, size)).clone_owned();

                    // Invert via Cholesky (blocks are SPD from A = J M^-1 J^T + reg*I).
                    // Fall back to scalar Jacobi for this block if Cholesky fails.
                    let inv = block.clone().cholesky().map_or_else(
                        || {
                            // Fallback: diagonal inverse for this block
                            DMatrix::from_fn(size, size, |i, j| {
                                if i == j {
                                    let diag = block[(i, i)];
                                    if diag.abs() > 1e-15 { 1.0 / diag } else { 1.0 }
                                } else {
                                    0.0
                                }
                            })
                        },
                        |chol| chol.inverse(),
                    );

                    block_inverses.push(inv);
                }

                // Verify blocks are contiguous and cover the full system.
                // Gaps would leave z=0 for uncovered rows, silently freezing
                // those DOFs in CG iteration.
                debug_assert_eq!(
                    sizes.iter().sum::<usize>(),
                    n,
                    "Block Jacobi preconditioner blocks cover {} of {n} rows — \
                     gaps will produce incorrect preconditioned residuals",
                    sizes.iter().sum::<usize>(),
                );
                debug_assert!(
                    offsets
                        .iter()
                        .zip(sizes.iter())
                        .zip(offsets.iter().skip(1))
                        .all(|((&off, &sz), &next_off)| off + sz == next_off),
                    "Block Jacobi preconditioner blocks are not contiguous",
                );

                PreconditionerData::Block {
                    block_inverses,
                    offsets,
                    sizes,
                    n,
                }
            }
        }
    }

    /// Apply the preconditioner: z = M^-1 * r.
    fn apply_preconditioner(
        &self,
        r: &DVector<f64>,
        preconditioner: &PreconditionerData,
    ) -> DVector<f64> {
        match preconditioner {
            PreconditionerData::Diagonal(diag) => r.component_mul(diag),
            PreconditionerData::Block {
                block_inverses,
                offsets,
                sizes,
                n,
            } => {
                let mut z = DVector::zeros(*n);
                for (idx, inv) in block_inverses.iter().enumerate() {
                    let offset = offsets[idx];
                    let size = sizes[idx];
                    if size == 0 {
                        continue;
                    }
                    let r_block = r.rows(offset, size);
                    let z_block = inv * r_block;
                    z.rows_mut(offset, size).copy_from(&z_block);
                }
                z
            }
        }
    }

    /// Build the system structure from joints.
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

        let mut body_map: HashMap<BodyId, usize> = HashMap::new();
        let mut bodies: Vec<SolverBody> = Vec::new();
        let mut constraints: Vec<SolverConstraint<'a, J>> = Vec::new();
        let mut row_offset = 0;

        for joint in joints {
            let parent_index = if let Some(&idx) = body_map.get(&joint.parent()) {
                idx
            } else {
                let state = get_body_state(joint.parent())?;
                let idx = bodies.len();
                bodies.push(SolverBody { state });
                body_map.insert(joint.parent(), idx);
                idx
            };

            let child_index = if let Some(&idx) = body_map.get(&joint.child()) {
                idx
            } else {
                let state = get_body_state(joint.child())?;
                let idx = bodies.len();
                bodies.push(SolverBody { state });
                body_map.insert(joint.child(), idx);
                idx
            };

            let num_rows = joint.joint_type().constrained_dof();

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
        let num_cols = 6 * num_bodies;

        let mut jacobian = DMatrix::zeros(total_rows, num_cols);

        for constraint in constraints {
            let parent = &bodies[constraint.parent_index];
            let child = &bodies[constraint.child_index];

            self.fill_jacobian_block(&mut jacobian, constraint, parent, child);
        }

        jacobian
    }

    /// Fill the Jacobian block for a single constraint.
    fn fill_jacobian_block<J: Joint>(
        &self,
        jacobian: &mut DMatrix<f64>,
        constraint: &SolverConstraint<'_, J>,
        parent: &SolverBody,
        child: &SolverBody,
    ) {
        let row = constraint.row_offset;
        let parent_col = constraint.parent_index * 6;
        let child_col = constraint.child_index * 6;

        let parent_anchor_world =
            parent.state.position + parent.state.rotation * constraint.joint.parent_anchor().coords;
        let child_anchor_world =
            child.state.position + child.state.rotation * constraint.joint.child_anchor().coords;

        let r_parent = parent_anchor_world - parent.state.position;
        let r_child = child_anchor_world - child.state.position;

        match constraint.joint.joint_type() {
            JointType::Fixed => {
                self.fill_fixed_jacobian(jacobian, row, parent_col, child_col, &r_parent, &r_child);
            }
            JointType::Revolute => {
                self.fill_revolute_jacobian(
                    jacobian, row, parent_col, child_col, &r_parent, &r_child,
                );
            }
            JointType::Prismatic => {
                self.fill_prismatic_jacobian(
                    jacobian, row, parent_col, child_col, &r_parent, &r_child,
                );
            }
            JointType::Spherical => {
                self.fill_spherical_jacobian(
                    jacobian, row, parent_col, child_col, &r_parent, &r_child,
                );
            }
            JointType::Universal => {
                self.fill_universal_jacobian(
                    jacobian, row, parent_col, child_col, &r_parent, &r_child,
                );
            }
            JointType::Free => {
                // Free joint has 0 constraints, nothing to fill
            }
            JointType::Planar => {
                self.fill_planar_jacobian(
                    jacobian, row, parent_col, child_col, &r_parent, &r_child,
                );
            }
            JointType::Cylindrical => {
                self.fill_cylindrical_jacobian(
                    jacobian, row, parent_col, child_col, &r_parent, &r_child,
                );
            }
        }
    }

    /// Fill Jacobian for fixed joint (6 DOF constrained).
    fn fill_fixed_jacobian(
        &self,
        jacobian: &mut DMatrix<f64>,
        row: usize,
        parent_col: usize,
        child_col: usize,
        r_parent: &Vector3<f64>,
        r_child: &Vector3<f64>,
    ) {
        // Position constraints
        for i in 0..3 {
            jacobian[(row + i, parent_col + i)] = -1.0;
            jacobian[(row + i, child_col + i)] = 1.0;
        }

        let skew_parent = skew_symmetric(r_parent);
        let skew_child = skew_symmetric(r_child);

        for i in 0..3 {
            for j in 0..3 {
                jacobian[(row + i, parent_col + 3 + j)] = -skew_parent[(i, j)];
                jacobian[(row + i, child_col + 3 + j)] = skew_child[(i, j)];
            }
        }

        // Rotation constraints
        for i in 0..3 {
            jacobian[(row + 3 + i, parent_col + 3 + i)] = -1.0;
            jacobian[(row + 3 + i, child_col + 3 + i)] = 1.0;
        }
    }

    /// Fill Jacobian for revolute joint (5 DOF constrained).
    fn fill_revolute_jacobian(
        &self,
        jacobian: &mut DMatrix<f64>,
        row: usize,
        parent_col: usize,
        child_col: usize,
        r_parent: &Vector3<f64>,
        r_child: &Vector3<f64>,
    ) {
        // Position constraints
        for i in 0..3 {
            jacobian[(row + i, parent_col + i)] = -1.0;
            jacobian[(row + i, child_col + i)] = 1.0;
        }

        let skew_parent = skew_symmetric(r_parent);
        let skew_child = skew_symmetric(r_child);

        for i in 0..3 {
            for j in 0..3 {
                jacobian[(row + i, parent_col + 3 + j)] = -skew_parent[(i, j)];
                jacobian[(row + i, child_col + 3 + j)] = skew_child[(i, j)];
            }
        }

        // Rotation constraints perpendicular to axis
        let perp1 = Vector3::new(0.0, 0.0, 1.0);
        let perp2 = Vector3::new(0.0, 1.0, 0.0);

        for &(idx, ref perp) in &[(3, perp1), (4, perp2)] {
            for j in 0..3 {
                jacobian[(row + idx, parent_col + 3 + j)] = -perp[j];
                jacobian[(row + idx, child_col + 3 + j)] = perp[j];
            }
        }
    }

    /// Fill Jacobian for prismatic joint (5 DOF constrained).
    fn fill_prismatic_jacobian(
        &self,
        jacobian: &mut DMatrix<f64>,
        row: usize,
        parent_col: usize,
        child_col: usize,
        r_parent: &Vector3<f64>,
        r_child: &Vector3<f64>,
    ) {
        let perp1 = Vector3::new(0.0, 1.0, 0.0);
        let perp2 = Vector3::new(0.0, 0.0, 1.0);

        for &(idx, ref perp) in &[(0, perp1), (1, perp2)] {
            for j in 0..3 {
                jacobian[(row + idx, parent_col + j)] = -perp[j];
                jacobian[(row + idx, child_col + j)] = perp[j];
            }
        }

        let skew_parent = skew_symmetric(r_parent);
        let skew_child = skew_symmetric(r_child);

        for &(idx, ref perp) in &[(0, perp1), (1, perp2)] {
            let skew_perp_parent = skew_parent.transpose() * perp;
            let skew_perp_child = skew_child.transpose() * perp;
            for j in 0..3 {
                jacobian[(row + idx, parent_col + 3 + j)] = -skew_perp_parent[j];
                jacobian[(row + idx, child_col + 3 + j)] = skew_perp_child[j];
            }
        }

        // Full rotation constraints
        for i in 0..3 {
            jacobian[(row + 2 + i, parent_col + 3 + i)] = -1.0;
            jacobian[(row + 2 + i, child_col + 3 + i)] = 1.0;
        }
    }

    /// Fill Jacobian for spherical joint (3 DOF constrained).
    fn fill_spherical_jacobian(
        &self,
        jacobian: &mut DMatrix<f64>,
        row: usize,
        parent_col: usize,
        child_col: usize,
        r_parent: &Vector3<f64>,
        r_child: &Vector3<f64>,
    ) {
        for i in 0..3 {
            jacobian[(row + i, parent_col + i)] = -1.0;
            jacobian[(row + i, child_col + i)] = 1.0;
        }

        let skew_parent = skew_symmetric(r_parent);
        let skew_child = skew_symmetric(r_child);

        for i in 0..3 {
            for j in 0..3 {
                jacobian[(row + i, parent_col + 3 + j)] = -skew_parent[(i, j)];
                jacobian[(row + i, child_col + 3 + j)] = skew_child[(i, j)];
            }
        }
    }

    /// Fill Jacobian for universal joint (4 DOF constrained).
    fn fill_universal_jacobian(
        &self,
        jacobian: &mut DMatrix<f64>,
        row: usize,
        parent_col: usize,
        child_col: usize,
        r_parent: &Vector3<f64>,
        r_child: &Vector3<f64>,
    ) {
        // Position constraints
        for i in 0..3 {
            jacobian[(row + i, parent_col + i)] = -1.0;
            jacobian[(row + i, child_col + i)] = 1.0;
        }

        let skew_parent = skew_symmetric(r_parent);
        let skew_child = skew_symmetric(r_child);

        for i in 0..3 {
            for j in 0..3 {
                jacobian[(row + i, parent_col + 3 + j)] = -skew_parent[(i, j)];
                jacobian[(row + i, child_col + 3 + j)] = skew_child[(i, j)];
            }
        }

        // One rotation constraint
        let axis = Vector3::new(0.0, 0.0, 1.0);
        for j in 0..3 {
            jacobian[(row + 3, parent_col + 3 + j)] = -axis[j];
            jacobian[(row + 3, child_col + 3 + j)] = axis[j];
        }
    }

    /// Fill Jacobian for planar joint (3 DOF constrained).
    ///
    /// Planar joints constrain: 1 translation (perpendicular to plane) + 2 rotations (tilt).
    fn fill_planar_jacobian(
        &self,
        jacobian: &mut DMatrix<f64>,
        row: usize,
        parent_col: usize,
        child_col: usize,
        r_parent: &Vector3<f64>,
        r_child: &Vector3<f64>,
    ) {
        // Use Z as plane normal (default XY plane)
        let normal = Vector3::new(0.0, 0.0, 1.0);
        let skew_parent = skew_symmetric(r_parent);
        let skew_child = skew_symmetric(r_child);

        // Row 0: Translation constraint perpendicular to plane (along Z)
        for j in 0..3 {
            jacobian[(row, parent_col + j)] = -normal[j];
            jacobian[(row, child_col + j)] = normal[j];
        }
        // Angular contribution to position constraint
        let skew_normal_parent = skew_parent.transpose() * normal;
        let skew_normal_child = skew_child.transpose() * normal;
        for j in 0..3 {
            jacobian[(row, parent_col + 3 + j)] = -skew_normal_parent[j];
            jacobian[(row, child_col + 3 + j)] = skew_normal_child[j];
        }

        // Rows 1-2: Rotation constraints (perpendicular to normal)
        let perp1 = Vector3::new(1.0, 0.0, 0.0);
        let perp2 = Vector3::new(0.0, 1.0, 0.0);

        for &(idx, perp) in &[(1, perp1), (2, perp2)] {
            for j in 0..3 {
                jacobian[(row + idx, parent_col + 3 + j)] = -perp[j];
                jacobian[(row + idx, child_col + 3 + j)] = perp[j];
            }
        }
    }

    /// Fill Jacobian for cylindrical joint (4 DOF constrained).
    ///
    /// Cylindrical joints constrain: 2 translations (perpendicular to axis) + 2 rotations (perpendicular to axis).
    fn fill_cylindrical_jacobian(
        &self,
        jacobian: &mut DMatrix<f64>,
        row: usize,
        parent_col: usize,
        child_col: usize,
        r_parent: &Vector3<f64>,
        r_child: &Vector3<f64>,
    ) {
        // Use X as joint axis (allows rotation and translation along X)
        let perp1 = Vector3::new(0.0, 1.0, 0.0); // Y
        let perp2 = Vector3::new(0.0, 0.0, 1.0); // Z
        let skew_parent = skew_symmetric(r_parent);
        let skew_child = skew_symmetric(r_child);

        // Rows 0-1: Translation constraints perpendicular to axis (Y and Z)
        for &(idx, perp) in &[(0, perp1), (1, perp2)] {
            for j in 0..3 {
                jacobian[(row + idx, parent_col + j)] = -perp[j];
                jacobian[(row + idx, child_col + j)] = perp[j];
            }

            let skew_perp_parent = skew_parent.transpose() * perp;
            let skew_perp_child = skew_child.transpose() * perp;
            for j in 0..3 {
                jacobian[(row + idx, parent_col + 3 + j)] = -skew_perp_parent[j];
                jacobian[(row + idx, child_col + 3 + j)] = skew_perp_child[j];
            }
        }

        // Rows 2-3: Rotation constraints (perpendicular to axis)
        for &(idx, perp) in &[(2, perp1), (3, perp2)] {
            for j in 0..3 {
                jacobian[(row + idx, parent_col + 3 + j)] = -perp[j];
                jacobian[(row + idx, child_col + 3 + j)] = perp[j];
            }
        }
    }

    /// Compute constraint error vector.
    fn compute_constraint_error<J: Joint>(
        &self,
        bodies: &[SolverBody],
        constraints: &[SolverConstraint<'_, J>],
        dt: f64,
    ) -> DVector<f64> {
        let total_rows: usize = constraints.iter().map(|c| c.num_rows).sum();
        let mut error = DVector::zeros(total_rows);

        for constraint in constraints {
            let parent = &bodies[constraint.parent_index];
            let child = &bodies[constraint.child_index];

            self.compute_constraint_error_for_joint(&mut error, constraint, parent, child, dt);
        }

        error
    }

    /// Compute constraint error for a single joint.
    fn compute_constraint_error_for_joint<J: Joint>(
        &self,
        error: &mut DVector<f64>,
        constraint: &SolverConstraint<'_, J>,
        parent: &SolverBody,
        child: &SolverBody,
        dt: f64,
    ) {
        let row = constraint.row_offset;
        let beta = self.config.baumgarte_factor;
        let beta_dt = if dt > 0.0 { beta / dt } else { 0.0 };

        let parent_anchor_world =
            parent.state.position + parent.state.rotation * constraint.joint.parent_anchor().coords;
        let child_anchor_world =
            child.state.position + child.state.rotation * constraint.joint.child_anchor().coords;

        let r_parent = parent_anchor_world - parent.state.position;
        let r_child = child_anchor_world - child.state.position;

        let position_error = child_anchor_world - parent_anchor_world;

        let v_parent =
            parent.state.linear_velocity + parent.state.angular_velocity.cross(&r_parent);
        let v_child = child.state.linear_velocity + child.state.angular_velocity.cross(&r_child);
        let velocity_error = v_child - v_parent;

        let omega_error = child.state.angular_velocity - parent.state.angular_velocity;

        match constraint.joint.joint_type() {
            JointType::Fixed => {
                for i in 0..3 {
                    error[row + i] = velocity_error[i] + beta_dt * position_error[i];
                }
                for i in 0..3 {
                    error[row + 3 + i] = omega_error[i];
                }
            }
            JointType::Revolute => {
                for i in 0..3 {
                    error[row + i] = velocity_error[i] + beta_dt * position_error[i];
                }
                error[row + 3] = omega_error.z;
                error[row + 4] = omega_error.y;
            }
            JointType::Prismatic => {
                error[row] = velocity_error.y + beta_dt * position_error.y;
                error[row + 1] = velocity_error.z + beta_dt * position_error.z;
                for i in 0..3 {
                    error[row + 2 + i] = omega_error[i];
                }
            }
            JointType::Spherical => {
                for i in 0..3 {
                    error[row + i] = velocity_error[i] + beta_dt * position_error[i];
                }
            }
            JointType::Universal => {
                for i in 0..3 {
                    error[row + i] = velocity_error[i] + beta_dt * position_error[i];
                }
                error[row + 3] = omega_error.z;
            }
            JointType::Free => {
                // 0 errors: no constraints
            }
            JointType::Planar => {
                // 3 errors: 1 position (perpendicular to plane) + 2 rotation (tilt)
                error[row] = velocity_error.z + beta_dt * position_error.z;
                error[row + 1] = omega_error.x;
                error[row + 2] = omega_error.y;
            }
            JointType::Cylindrical => {
                // 4 errors: 2 position (perpendicular to axis) + 2 rotation (perpendicular to axis)
                error[row] = velocity_error.y + beta_dt * position_error.y;
                error[row + 1] = velocity_error.z + beta_dt * position_error.z;
                error[row + 2] = omega_error.y;
                error[row + 3] = omega_error.z;
            }
        }
    }

    /// Build the inverse mass matrix (block diagonal).
    fn build_inverse_mass_matrix(&self, bodies: &[SolverBody]) -> DMatrix<f64> {
        let n = bodies.len() * 6;
        let mut inv_mass = DMatrix::zeros(n, n);

        for (i, body) in bodies.iter().enumerate() {
            let offset = i * 6;

            for j in 0..3 {
                inv_mass[(offset + j, offset + j)] = body.state.inv_mass;
            }

            for j in 0..3 {
                for k in 0..3 {
                    inv_mass[(offset + 3 + j, offset + 3 + k)] = body.state.inv_inertia[(j, k)];
                }
            }
        }

        inv_mass
    }

    /// Extract joint forces from the solution vector.
    fn extract_joint_force<J: Joint>(
        &self,
        constraint: &SolverConstraint<'_, J>,
        forces_vec: &DVector<f64>,
        _num_bodies: usize,
    ) -> ConstraintForce {
        let parent_offset = constraint.parent_index * 6;
        let child_offset = constraint.child_index * 6;

        let parent_force = Vector3::new(
            forces_vec[parent_offset],
            forces_vec[parent_offset + 1],
            forces_vec[parent_offset + 2],
        );
        let parent_torque = Vector3::new(
            forces_vec[parent_offset + 3],
            forces_vec[parent_offset + 4],
            forces_vec[parent_offset + 5],
        );
        let child_force = Vector3::new(
            forces_vec[child_offset],
            forces_vec[child_offset + 1],
            forces_vec[child_offset + 2],
        );
        let child_torque = Vector3::new(
            forces_vec[child_offset + 3],
            forces_vec[child_offset + 4],
            forces_vec[child_offset + 5],
        );

        ConstraintForce::new(parent_force, parent_torque, child_force, child_torque)
    }

    /// Clear cached data (invalidates warm starting).
    pub fn clear_cache(&mut self) {
        self.cached_lambda.clear();
        self.prev_constraint_count = 0;
    }

    /// Reset statistics.
    pub fn reset_stats(&mut self) {
        self.last_stats = CGSolverStats::default();
    }
}

impl Default for CGSolver {
    fn default() -> Self {
        Self::default_solver()
    }
}

/// Compute the skew-symmetric matrix for cross product.
fn skew_symmetric(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
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
    fn test_cg_solver_creation() {
        let solver = CGSolver::default_solver();
        assert_eq!(solver.config().max_iterations, 50);
        assert_relative_eq!(solver.config().tolerance, 1e-6, epsilon = 1e-10);
    }

    #[test]
    fn test_cg_solver_config_presets() {
        let high_acc = CGSolverConfig::high_accuracy();
        assert_eq!(high_acc.max_iterations, 200);
        assert_relative_eq!(high_acc.tolerance, 1e-10, epsilon = 1e-15);

        let realtime = CGSolverConfig::realtime();
        assert_eq!(realtime.max_iterations, 20);
        assert_relative_eq!(realtime.tolerance, 1e-4, epsilon = 1e-10);

        let large = CGSolverConfig::large_system();
        assert_eq!(large.max_iterations, 100);
        assert!(matches!(large.preconditioner, Preconditioner::BlockJacobi));
    }

    #[test]
    fn test_cg_solver_empty_joints() {
        let mut solver = CGSolver::default_solver();
        let joints: Vec<RevoluteJoint> = vec![];

        let result = solver.solve(&joints, |_| None, 0.001);
        assert!(result.is_empty());
        assert!(result.converged);
    }

    #[test]
    fn test_cg_solver_single_joint() {
        let mut solver = CGSolver::default_solver();

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
            0.001,
        );

        assert_eq!(result.forces.len(), 1);
        assert!(result.forces[0].force.total_force_magnitude() > 0.0);
    }

    #[test]
    fn test_cg_solver_converges() {
        let mut solver = CGSolver::new(CGSolverConfig {
            max_iterations: 100,
            tolerance: 1e-4,
            ..Default::default()
        });

        let joint = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());

        let parent_state = make_body_state(Point3::origin());
        let child_state = make_body_state(Point3::origin());

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
            0.001,
        );

        // Should converge when constraint is nearly satisfied
        assert!(result.converged || result.residual_norm < 1e-3);
    }

    #[test]
    fn test_cg_solver_multiple_joints() {
        let mut solver = CGSolver::default_solver();

        let joints = vec![
            RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z()),
            RevoluteJoint::new(BodyId::new(1), BodyId::new(2), Vector3::z()),
        ];

        let states = [
            make_body_state(Point3::origin()),
            make_body_state(Point3::new(0.5, 0.0, 0.0)),
            make_body_state(Point3::new(1.0, 0.0, 0.0)),
        ];

        let result = solver.solve(
            &joints,
            |id| {
                let idx = id.0 as usize;
                if idx < states.len() {
                    Some(states[idx])
                } else {
                    None
                }
            },
            0.001,
        );

        assert_eq!(result.forces.len(), 2);
    }

    #[test]
    fn test_cg_solver_with_tracking() {
        let mut solver = CGSolver::default_solver();
        solver.set_track_convergence(true);

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
            0.001,
        );

        // Should have convergence history
        assert!(solver.last_stats().convergence_history.is_some());
    }

    #[test]
    fn test_preconditioner_types() {
        let mut config = CGSolverConfig::default();

        config = config.with_preconditioner(Preconditioner::None);
        assert!(matches!(config.preconditioner, Preconditioner::None));

        config = config.with_preconditioner(Preconditioner::Jacobi);
        assert!(matches!(config.preconditioner, Preconditioner::Jacobi));

        config = config.with_preconditioner(Preconditioner::BlockJacobi);
        assert!(matches!(config.preconditioner, Preconditioner::BlockJacobi));
    }

    #[test]
    fn test_convergence_ratio() {
        let result = CGSolverResult {
            forces: vec![],
            iterations_used: 10,
            residual_norm: 1e-6,
            initial_residual_norm: 1.0,
            converged: true,
        };

        assert_relative_eq!(result.convergence_ratio(), 1e-6, epsilon = 1e-10);
    }

    #[test]
    fn test_cg_solve_simple_system() {
        // Test CG on a simple SPD system
        let solver = CGSolver::new(CGSolverConfig {
            min_iterations: 0, // Allow early convergence
            max_iterations: 100,
            tolerance: 1e-6,
            ..Default::default()
        });

        // A = [4 1; 1 3] (SPD)
        let a = DMatrix::from_row_slice(2, 2, &[4.0, 1.0, 1.0, 3.0]);
        let b = DVector::from_vec(vec![1.0, 2.0]);
        let x0 = DVector::zeros(2);

        let preconditioner = PreconditionerData::Diagonal(DVector::from_element(2, 1.0));

        let (x, _iterations, residual, _initial, converged, _history) =
            solver.cg_solve(&a, &b, &x0, &preconditioner);

        // Either converged or residual is very small
        assert!(converged || residual < 1e-5);

        // Verify solution: A * x ≈ b
        let ax = &a * &x;
        assert_relative_eq!(ax[0], b[0], epsilon = 1e-5);
        assert_relative_eq!(ax[1], b[1], epsilon = 1e-5);
    }

    #[test]
    fn test_warm_starting() {
        let mut solver = CGSolver::new(CGSolverConfig {
            warm_starting: true,
            warm_start_factor: 0.9,
            ..Default::default()
        });

        let joint = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());
        let parent_state = make_body_state(Point3::origin());
        let child_state = make_body_state(Point3::new(0.1, 0.0, 0.0));

        let get_state = |id: BodyId| {
            if id == BodyId::new(0) {
                Some(parent_state)
            } else if id == BodyId::new(1) {
                Some(child_state)
            } else {
                None
            }
        };

        // First solve
        let _ = solver.solve(std::slice::from_ref(&joint), get_state, 0.001);

        // Second solve should use warm start
        let result2 = solver.solve(&[joint], get_state, 0.001);

        // Warm starting was used (cache populated from first solve)
        assert!(!solver.cached_lambda.is_empty());
        assert!(!result2.forces.is_empty());
    }

    #[test]
    fn test_block_jacobi_preconditioner() {
        // Test Block Jacobi on a 6×6 SPD system with two 3×3 blocks.
        // This simulates two joints, each contributing 3 constraint rows.
        let solver = CGSolver::new(CGSolverConfig {
            min_iterations: 0,
            max_iterations: 100,
            tolerance: 1e-10,
            preconditioner: Preconditioner::BlockJacobi,
            ..Default::default()
        });

        // Build a block-diagonal SPD matrix with off-diagonal coupling within blocks.
        // Block 1 (rows 0-2): 3×3 SPD matrix
        // Block 2 (rows 3-5): 3×3 SPD matrix
        // Cross-block coupling (weak): rows 0-2 vs rows 3-5
        #[rustfmt::skip]
        let a = DMatrix::from_row_slice(6, 6, &[
            // Block 1
            10.0, 2.0, 1.0,   0.1, 0.0, 0.0,
             2.0, 8.0, 1.5,   0.0, 0.1, 0.0,
             1.0, 1.5, 6.0,   0.0, 0.0, 0.1,
            // Block 2
             0.1, 0.0, 0.0,  12.0, 3.0, 2.0,
             0.0, 0.1, 0.0,   3.0, 9.0, 1.0,
             0.0, 0.0, 0.1,   2.0, 1.0, 7.0,
        ]);
        let b = DVector::from_vec(vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let x0 = DVector::zeros(6);

        // Constraint blocks: two joints, each with 3 rows
        let blocks = vec![(0, 3), (3, 3)];

        let preconditioner = solver.build_preconditioner(&a, &blocks);
        assert!(matches!(preconditioner, PreconditionerData::Block { .. }));

        let (x, _iters, residual, _initial, converged, _history) =
            solver.cg_solve(&a, &b, &x0, &preconditioner);

        assert!(converged || residual < 1e-8);

        // Verify solution: A * x ≈ b
        let ax = &a * &x;
        for i in 0..6 {
            assert_relative_eq!(ax[i], b[i], epsilon = 1e-6);
        }
    }

    #[test]
    fn test_block_jacobi_fewer_iterations_than_scalar() {
        // Block Jacobi should converge in fewer iterations than scalar Jacobi
        // on a system with strong intra-block coupling.
        #[rustfmt::skip]
        let a = DMatrix::from_row_slice(6, 6, &[
            10.0, 4.0, 3.0,   0.1, 0.0, 0.0,
             4.0, 8.0, 3.5,   0.0, 0.1, 0.0,
             3.0, 3.5, 6.0,   0.0, 0.0, 0.1,
             0.1, 0.0, 0.0,  12.0, 5.0, 4.0,
             0.0, 0.1, 0.0,   5.0, 9.0, 3.0,
             0.0, 0.0, 0.1,   4.0, 3.0, 7.0,
        ]);
        let b = DVector::from_vec(vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let x0 = DVector::zeros(6);
        let blocks = vec![(0, 3), (3, 3)];

        // Solve with scalar Jacobi
        let solver_jacobi = CGSolver::new(CGSolverConfig {
            min_iterations: 0,
            max_iterations: 100,
            tolerance: 1e-10,
            preconditioner: Preconditioner::Jacobi,
            ..Default::default()
        });
        let diag_precond = solver_jacobi.build_preconditioner(&a, &[]);
        let (_x_j, iters_jacobi, _res_j, _init_j, _conv_j, _hist_j) =
            solver_jacobi.cg_solve(&a, &b, &x0, &diag_precond);

        // Solve with Block Jacobi
        let solver_block = CGSolver::new(CGSolverConfig {
            min_iterations: 0,
            max_iterations: 100,
            tolerance: 1e-10,
            preconditioner: Preconditioner::BlockJacobi,
            ..Default::default()
        });
        let block_precond = solver_block.build_preconditioner(&a, &blocks);
        let (_x_b, iters_block, _res_b, _init_b, _conv_b, _hist_b) =
            solver_block.cg_solve(&a, &b, &x0, &block_precond);

        // Block Jacobi should converge in fewer or equal iterations
        assert!(
            iters_block <= iters_jacobi,
            "Block Jacobi ({iters_block} iters) should not be worse than scalar Jacobi ({iters_jacobi} iters)"
        );
    }
}
