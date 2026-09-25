//! The executor trait: one method per phase of a step, so the stepping loop
//! writes the phase order once and runs it on the CPU or the GPU (plan §14d,
//! §15f, §16e).
//!
//! Everything that crosses this boundary is `f64`. An executor keeps its own
//! state at its own precision and converts at the edge, so the loop is the
//! same for every backend and every precision.

use crate::f64::{Pose, SdfGridLayout};

/// The rigid obstacle an executor contacts: a baked distance grid, its pose
/// sampled evenly in time, and the contact law's two parameters.
#[derive(Clone, Debug)]
pub struct Obstacle {
    /// The grid's layout, in the obstacle's body frame.
    pub grid: SdfGridLayout,
    /// The grid's values, in [`SdfGridLayout`]'s order; negative inside.
    pub values: Vec<f64>,
    /// The time of the first pose sample.
    pub start: f64,
    /// The time between pose samples; positive.
    pub interval: f64,
    /// The pose samples, body to world. Between two samples the pose is
    /// interpolated (`pose_interpolate`); outside them it is held.
    pub poses: Vec<Pose>,
    /// The friction coefficient `μ_f`; 0 is frictionless.
    pub friction: f64,
    /// The penalty scale `s` in `k = s · m / Δt²` (plan §15c's 0.5).
    pub penalty_scale: f64,
}

/// What the stepping loop reads every monitor interval, reduced on the
/// executor. Cumulative values run from the start of the run.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Monitors {
    /// Kinetic energy at the latest half-step velocities.
    pub kinetic_energy: f64,
    /// Internal (strain) energy at the current displacements, with the λ
    /// term averaged over nodes as the forces are.
    pub internal_energy: f64,
    /// Kinetic energy of the nodes in contact at the last step: a watch for
    /// friction flutter, which nothing gates (plan §16e).
    pub contact_kinetic_energy: f64,
    /// The contact forces' resultant on the body, world frame, averaged over
    /// the steps since the last read.
    pub contact_force: [f64; 3],
    /// The number of steps since the last read.
    pub steps: u64,
    /// Cumulative: element-steps with `J ≤ 0` (K4).
    pub inverted_element_steps: u64,
    /// Cumulative: the work the contact forces have done on the body.
    pub contact_work: f64,
    /// Cumulative: the energy mass damping has removed.
    pub damping_loss: f64,
    /// Cumulative: the deepest penetration any node has reached (G2).
    pub max_penetration: f64,
}

/// A read of the executor's per-node state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Snapshot {
    /// Each node's displacement.
    pub displacements: Vec<[f64; 3]>,
    /// Each node's velocity at the latest half step.
    pub velocities: Vec<[f64; 3]>,
    /// Each node's contact normal force, summed over the accumulated steps.
    pub normal_force_sums: Vec<f64>,
    /// Each node's friction force, world frame, summed over the accumulated
    /// steps.
    pub friction_sums: Vec<[f64; 3]>,
    /// How many steps the sums hold.
    pub accumulated_steps: u64,
}

/// An explicit executor: the solver's state, and one method per phase.
///
/// A step is the phases in [`crate::stepping`]'s order, and each phase
/// reads what the one before wrote. Nothing is read back to the host except
/// by [`Executor::monitors`], [`Executor::snapshot`] and
/// [`Executor::elastic_rayleigh_quotient`].
pub trait Executor {
    /// The number of nodes.
    fn node_count(&self) -> usize;

    /// The shortest element edge at rest, the length the power iteration's
    /// finite difference is scaled by.
    fn shortest_edge(&self) -> f64;

    /// The executor's machine epsilon: `f32::EPSILON` or `f64::EPSILON`.
    fn epsilon(&self) -> f64;

    /// Replace every node's displacement and half-step velocity, for an
    /// initial condition or to put two executors in the same state.
    ///
    /// # Panics
    /// If either slice's length is not the node count.
    fn set_state(&mut self, displacements: &[[f64; 3]], velocities: &[[f64; 3]]);

    /// Phase 1: each element's dilation `J − 1`, counting inverted elements.
    fn element_dilations(&mut self);

    /// Phase 2: each node's volume change, gathered from its elements.
    fn gather_volume_changes(&mut self);

    /// Phase 3: each node's averaged pressure.
    fn nodal_pressures(&mut self);

    /// Phase 4: each element's elastic forces, into its own slots.
    fn element_forces(&mut self);

    /// Phase 5: each node's elastic force, gathered from its elements' slots.
    fn gather_forces(&mut self);

    /// Phase 6: each surface node's contact with the obstacle at `time`,
    /// with the penalty stiffness for step `dt`.
    fn contact(&mut self, time: f64, dt: f64);

    /// Phase 7: the central-difference update of each node's velocity and
    /// displacement, with mass damping `damping`.
    fn integrate(&mut self, dt: f64, damping: f64);

    /// Phase 8: each node's constraints, and the step's energy terms.
    fn boundary_conditions(&mut self, dt: f64, damping: f64);

    /// Add each node's current contact normal and friction forces to its
    /// window sums.
    fn accumulate(&mut self);

    /// Empty the window sums.
    fn clear_accumulators(&mut self);

    /// Reduce and read the monitors, and restart the since-last-read ones.
    fn monitors(&mut self) -> Monitors;

    /// Read the per-node state.
    fn snapshot(&self) -> Snapshot;

    /// Run `iterations` of the power iteration on `M⁻¹K`, the elastic
    /// stiffness at the current state, and return the Rayleigh quotient,
    /// an estimate of `ω_el²` from below.
    ///
    /// `K v` is the finite difference of phases 1–5 in the direction `v`,
    /// with the step `perturbation` in the largest nodal component. The
    /// vector stays on the executor and is warm-started on the next call.
    /// Held and constrained directions are excluded.
    fn elastic_rayleigh_quotient(&mut self, iterations: usize, perturbation: f64) -> f64;
}

/// Why an [`Obstacle`] was rejected.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum ObstacleError {
    /// The pose track has no samples.
    #[error("the obstacle has no pose samples")]
    NoPoses,
    /// The pose samples' spacing is not positive and finite.
    #[error("the pose interval is {interval}; it must be positive and finite")]
    Interval {
        /// The interval given.
        interval: f64,
    },
    /// The grid's values do not match its layout.
    #[error("the grid has {found} values; its layout needs {expected}")]
    GridSize {
        /// The values given.
        found: usize,
        /// The values the layout needs.
        expected: usize,
    },
    /// The grid's layout, a value, a pose, the friction or the penalty scale
    /// is out of range.
    #[error("the obstacle is invalid: {reason}")]
    Invalid {
        /// What is wrong.
        reason: &'static str,
    },
}

/// Check an obstacle before an executor uploads it.
///
/// # Errors
/// The first problem found.
pub fn check_obstacle(obstacle: &Obstacle) -> Result<(), ObstacleError> {
    let grid = obstacle.grid;
    if obstacle.poses.is_empty() {
        return Err(ObstacleError::NoPoses);
    }
    if !(obstacle.interval.is_finite() && obstacle.interval > 0.0) {
        return Err(ObstacleError::Interval {
            interval: obstacle.interval,
        });
    }
    let expected = [grid.size_x, grid.size_y, grid.size_z]
        .iter()
        .map(|&n| n as usize)
        .product::<usize>();
    if obstacle.values.len() != expected {
        return Err(ObstacleError::GridSize {
            found: obstacle.values.len(),
            expected,
        });
    }
    let reason = if grid.size_x == 0 || grid.size_y == 0 || grid.size_z == 0 {
        Some("the grid has no samples along an axis")
    } else if !(grid.cell_size.is_finite() && grid.cell_size > 0.0) {
        Some("the grid's cell size must be positive and finite")
    } else if ![grid.origin_x, grid.origin_y, grid.origin_z, obstacle.start]
        .iter()
        .chain(&obstacle.values)
        .all(|v| v.is_finite())
    {
        Some("a grid origin, value or the start time is not finite")
    } else if !obstacle.poses.iter().all(|p| {
        let norm = p.qw * p.qw + p.qx * p.qx + p.qy * p.qy + p.qz * p.qz;
        (norm - 1.0).abs() <= 1e-9 && p.tx.is_finite() && p.ty.is_finite() && p.tz.is_finite()
    }) {
        Some("a pose is not a unit quaternion with a finite translation")
    } else if !(obstacle.friction.is_finite() && obstacle.friction >= 0.0) {
        Some("the friction must be finite and not negative")
    } else if !(obstacle.penalty_scale.is_finite() && obstacle.penalty_scale > 0.0) {
        Some("the penalty scale must be positive and finite")
    } else {
        None
    };
    reason.map_or(Ok(()), |reason| Err(ObstacleError::Invalid { reason }))
}
