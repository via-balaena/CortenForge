//! Parallel constraint solving using rayon.
//!
//! This module provides multi-threaded constraint solving by processing
//! independent constraint islands in parallel. Islands are groups of bodies
//! connected by joints - bodies in different islands have no constraints
//! between them and can be solved independently.
//!
//! # Design Decisions
//!
//! ## Disabled by Default
//!
//! Parallel constraint solving is disabled by default (`ParallelConfig::default()`)
//! because it uses the Newton solver internally, which may produce slightly different
//! results than the default `ConstraintSolver`. Enable explicitly via:
//!
//! ```ignore
//! config.solver.parallel.parallel_constraints = true;
//! ```
//!
//! ## Uses Newton Solver
//!
//! The parallel implementation uses `NewtonConstraintSolver` rather than
//! `ConstraintSolver` because:
//! - Newton solver already has island-based solving infrastructure
//! - It provides fast convergence (2-3 iterations vs 8-16 for Gauss-Seidel)
//! - Each island can be solved independently without shared mutable state
//!
//! ## Body Integration Remains Sequential
//!
//! Body integration (`World::integrate_bodies_parallel`) is sequential despite
//! the name because hashbrown's `HashMap` does not support `par_iter_mut`.
//! True parallel integration would require restructuring to use `Vec<Body>`
//! with an index map. The main performance benefit comes from island-parallel
//! constraint solving, which is the computationally expensive part.
//!
//! ## Minimum Thresholds
//!
//! Parallel solving only activates when there are at least `min_islands_for_parallel`
//! independent islands (default: 2). This avoids rayon overhead for simple scenes
//! like a single articulated robot where everything is one connected island.
//!
//! # Usage
//!
//! ```ignore
//! use sim_constraint::NewtonConstraintSolver;
//!
//! let mut solver = NewtonConstraintSolver::default();
//!
//! // Pre-build body state snapshot for thread-safe access
//! let body_states: HashMap<BodyId, BodyState> = /* ... */;
//!
//! // Build islands
//! let islands = ConstraintIslands::build(&joints);
//!
//! // Solve islands in parallel
//! let result = solver.solve_islands_parallel(
//!     &joints,
//!     &islands,
//!     &body_states,
//!     dt,
//!     2, // min_islands threshold
//! );
//! ```
//!
//! # Thread Safety
//!
//! The parallel solver requires:
//! - An immutable snapshot of body states (built before parallel phase)
//! - Joints that implement `Sync` (all built-in joints do)
//!
//! Results are collected from all islands and merged into a single result.

use hashbrown::HashMap;
use rayon::prelude::*;
use sim_types::BodyId;

use crate::{BodyState, ConstraintIslands, Island, Joint, NewtonSolverConfig, NewtonSolverResult};

/// Solve constraint islands in parallel using rayon.
///
/// This function provides the core parallel solving capability. It:
/// 1. Filters to active (non-static) islands
/// 2. Distributes islands across threads using rayon's work-stealing
/// 3. Solves each island independently using the Newton solver
/// 4. Merges results into a single `NewtonSolverResult`
///
/// # Arguments
///
/// * `config` - Newton solver configuration
/// * `joints` - All joints in the system
/// * `islands` - Pre-computed constraint islands
/// * `body_states` - Pre-built snapshot of body states (must be thread-safe)
/// * `dt` - Timestep for Baumgarte stabilization
/// * `min_islands` - Minimum active islands to trigger parallel solving
///
/// # Returns
///
/// Combined result with forces from all islands, max iterations used,
/// max constraint error, and overall convergence status.
///
/// # Note
///
/// If fewer than `min_islands` active islands exist, falls back to
/// sequential solving to avoid parallel overhead.
pub fn solve_islands_parallel<J>(
    config: &NewtonSolverConfig,
    joints: &[J],
    islands: &ConstraintIslands,
    body_states: &HashMap<BodyId, BodyState>,
    dt: f64,
    min_islands: usize,
) -> NewtonSolverResult
where
    J: Joint + Sync,
{
    if islands.num_islands() == 0 {
        return NewtonSolverResult::empty();
    }

    // Collect active islands
    let active_islands: Vec<(usize, &Island)> = islands.active_islands().collect();

    if active_islands.is_empty() {
        return NewtonSolverResult::empty();
    }

    // Fall back to sequential if too few islands
    if active_islands.len() < min_islands {
        return solve_islands_sequential(config, joints, &active_islands, body_states, dt);
    }

    // Parallel solve each island
    let results: Vec<NewtonSolverResult> = active_islands
        .par_iter()
        .filter_map(|(_, island)| {
            if island.constraint_indices.is_empty() {
                return None;
            }

            // Collect joints for this island
            let island_joints: Vec<&J> = island
                .constraint_indices
                .iter()
                .filter_map(|&idx| joints.get(idx))
                .collect();

            if island_joints.is_empty() {
                return None;
            }

            Some(solve_island_with_snapshot(
                config,
                &island_joints,
                body_states,
                dt,
            ))
        })
        .collect();

    merge_results(&results)
}

/// Sequential fallback for small numbers of islands.
fn solve_islands_sequential<J>(
    config: &NewtonSolverConfig,
    joints: &[J],
    active_islands: &[(usize, &Island)],
    body_states: &HashMap<BodyId, BodyState>,
    dt: f64,
) -> NewtonSolverResult
where
    J: Joint,
{
    let results: Vec<NewtonSolverResult> = active_islands
        .iter()
        .filter_map(|(_, island)| {
            if island.constraint_indices.is_empty() {
                return None;
            }

            let island_joints: Vec<&J> = island
                .constraint_indices
                .iter()
                .filter_map(|&idx| joints.get(idx))
                .collect();

            if island_joints.is_empty() {
                return None;
            }

            Some(solve_island_with_snapshot(
                config,
                &island_joints,
                body_states,
                dt,
            ))
        })
        .collect();

    merge_results(&results)
}

/// Wrapper type that implements Joint for a reference.
struct JointRef<'a, J: Joint>(&'a J);

impl<J: Joint> Joint for JointRef<'_, J> {
    fn parent(&self) -> BodyId {
        self.0.parent()
    }

    fn child(&self) -> BodyId {
        self.0.child()
    }

    fn dof(&self) -> usize {
        self.0.dof()
    }

    fn joint_type(&self) -> crate::JointType {
        self.0.joint_type()
    }

    fn limits(&self) -> Option<&crate::JointLimits> {
        self.0.limits()
    }

    fn motor(&self) -> Option<&crate::JointMotor> {
        self.0.motor()
    }

    fn damping(&self) -> f64 {
        self.0.damping()
    }

    fn parent_anchor(&self) -> nalgebra::Point3<f64> {
        self.0.parent_anchor()
    }

    fn child_anchor(&self) -> nalgebra::Point3<f64> {
        self.0.child_anchor()
    }
}

/// Solve a single island using a body state snapshot.
///
/// This creates a temporary Newton solver and solves the island's constraints
/// using the pre-built body state snapshot for thread-safe access.
fn solve_island_with_snapshot<J>(
    config: &NewtonSolverConfig,
    joints: &[&J],
    body_states: &HashMap<BodyId, BodyState>,
    dt: f64,
) -> NewtonSolverResult
where
    J: Joint,
{
    use crate::NewtonConstraintSolver;

    if joints.is_empty() {
        return NewtonSolverResult::empty();
    }

    // Wrap the joint references so they implement Joint
    let joint_refs: Vec<JointRef<J>> = joints.iter().map(|&j| JointRef(j)).collect();

    // Create a temporary solver with the same config
    let mut solver = NewtonConstraintSolver::new(*config);

    // Solve using the body state snapshot
    solver.solve(
        &joint_refs,
        |body_id| body_states.get(&body_id).copied(),
        dt,
    )
}

/// Merge results from multiple island solves.
fn merge_results(results: &[NewtonSolverResult]) -> NewtonSolverResult {
    if results.is_empty() {
        return NewtonSolverResult::empty();
    }

    let total_forces = results
        .iter()
        .flat_map(|r| r.forces.iter().cloned())
        .collect();

    let max_iterations = results.iter().map(|r| r.iterations_used).max().unwrap_or(0);

    let max_error = results
        .iter()
        .map(|r| r.constraint_error)
        .fold(0.0_f64, f64::max);

    let all_converged = results.iter().all(|r| r.converged);

    NewtonSolverResult {
        forces: total_forces,
        iterations_used: max_iterations,
        constraint_error: max_error,
        converged: all_converged,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RevoluteJoint;
    use nalgebra::{Matrix3, Point3, Vector3};

    fn make_body_state(id: BodyId, is_static: bool) -> (BodyId, BodyState) {
        (
            id,
            BodyState {
                position: Point3::origin(),
                rotation: Matrix3::identity(),
                linear_velocity: Vector3::zeros(),
                angular_velocity: Vector3::zeros(),
                inv_mass: if is_static { 0.0 } else { 1.0 },
                inv_inertia: if is_static {
                    Matrix3::zeros()
                } else {
                    Matrix3::identity()
                },
                is_static,
            },
        )
    }

    #[test]
    fn test_solve_empty_islands() {
        let joints: Vec<RevoluteJoint> = vec![];
        let islands = ConstraintIslands::build(&joints);
        let body_states = HashMap::new();
        let config = NewtonSolverConfig::default();

        let result = solve_islands_parallel(&config, &joints, &islands, &body_states, 0.01, 2);

        assert!(result.forces.is_empty());
        assert!(result.converged);
    }

    #[test]
    fn test_solve_single_joint() {
        let joints = vec![RevoluteJoint::new(
            BodyId::new(0),
            BodyId::new(1),
            Vector3::z(),
        )];

        let islands = ConstraintIslands::build(&joints);

        let body_states: HashMap<BodyId, BodyState> = vec![
            make_body_state(BodyId::new(0), true),
            make_body_state(BodyId::new(1), false),
        ]
        .into_iter()
        .collect();

        let config = NewtonSolverConfig::default();

        let result = solve_islands_parallel(&config, &joints, &islands, &body_states, 0.01, 2);

        // Should fall back to sequential (only 1 island)
        // Result should have forces for the joint
        assert!(result.converged || !result.forces.is_empty());
    }

    #[test]
    fn test_merge_empty_results() {
        let results: Vec<NewtonSolverResult> = vec![];
        let merged = merge_results(&results);

        assert!(merged.forces.is_empty());
        assert!(merged.converged);
        assert_eq!(merged.iterations_used, 0);
    }

    #[test]
    fn test_merge_multiple_results() {
        use crate::{ConstraintForce, JointForce};

        let results = vec![
            NewtonSolverResult {
                forces: vec![JointForce {
                    parent: BodyId::new(0),
                    child: BodyId::new(1),
                    force: ConstraintForce::zero(),
                }],
                iterations_used: 3,
                constraint_error: 0.001,
                converged: true,
            },
            NewtonSolverResult {
                forces: vec![JointForce {
                    parent: BodyId::new(2),
                    child: BodyId::new(3),
                    force: ConstraintForce::zero(),
                }],
                iterations_used: 5,
                constraint_error: 0.002,
                converged: true,
            },
        ];

        let merged = merge_results(&results);

        assert_eq!(merged.forces.len(), 2);
        assert_eq!(merged.iterations_used, 5); // Max of 3 and 5
        assert!((merged.constraint_error - 0.002).abs() < 1e-10); // Max error
        assert!(merged.converged);
    }
}
