//! The executor trait: one method per phase of a step, so the stepping loop
//! writes the phase order once and runs it on the CPU or the GPU (plan §14d,
//! §15f, §16e).
//!
//! Everything that crosses this boundary is `f64`. An executor keeps its own
//! state at its own precision and converts at the edge, so the loop is the
//! same for every backend and every precision.

use crate::f64 as shared;
use crate::f64::{Pose, SdfGridLayout, SdfSample};

/// The rigid obstacle an executor contacts: a baked distance grid, with a finer
/// one near its surface if it has one, its pose sampled evenly in time, and the
/// friction coefficient.
///
/// The contact law is the kinematic predictor/corrector
/// (`shared::kinematic_contact`).
#[derive(Clone, Debug)]
pub struct Obstacle {
    /// The grid's layout, in the obstacle's body frame.
    pub grid: SdfGridLayout,
    /// The grid's values, in [`SdfGridLayout`]'s order; negative inside.
    pub values: Vec<f64>,
    /// A finer grid near the surface: a lookup reads it wherever it has all
    /// of that lookup's samples, and [`Obstacle::grid`] elsewhere.
    pub fine: Option<FineGrid>,
    /// The time of the first pose sample.
    pub start: f64,
    /// The time between pose samples; positive.
    pub interval: f64,
    /// The pose samples, body to world. Between two samples the pose is
    /// interpolated (`pose_interpolate`); outside them it is held.
    pub poses: Vec<Pose>,
    /// The friction coefficient `μ_f`; 0 is frictionless.
    pub friction: f64,
}

/// A finer distance grid near an obstacle's surface, stored in bricks of
/// [`shared::SDF_BRICK`] samples a side (plan §16u).
///
/// A lookup reads it where every one of its 64 samples has a brick and its
/// point lies on the lattice; a bake gives bricks to the samples within reach
/// of the surface, so the points near it read the fine grid.
#[derive(Clone, Debug)]
pub struct FineGrid {
    /// The fine lattice's layout, in the obstacle's body frame.
    pub grid: SdfGridLayout,
    /// For each brick of the lattice, in [`shared::sdf_brick_index`]'s order:
    /// its slot among the bricks in `values`, or [`shared::SDF_NO_BRICK`].
    pub map: Vec<u32>,
    /// The bricks' values, [`shared::SDF_BRICK`]³ a brick in slot order, each
    /// brick's in [`shared::sdf_brick_offset`]'s order; negative inside.
    pub values: Vec<f64>,
}

impl FineGrid {
    /// The shared lookup over this grid at a body-frame point, at f64; `None`
    /// where the point is off the lattice or one of its samples has no brick.
    #[must_use]
    pub fn sample(&self, point: [f64; 3]) -> Option<SdfSample> {
        let grid = self.grid;
        if !shared::sdf_on_grid(point, grid) {
            return None;
        }
        let coordinate = shared::sdf_grid_coordinate(point, grid);
        let columns = shared::sdf_tricubic_axis(coordinate[0], grid.size_x);
        let rows = shared::sdf_tricubic_axis(coordinate[1], grid.size_y);
        let layers = shared::sdf_tricubic_axis(coordinate[2], grid.size_z);
        let slots = shared::sdf_stencil_bricks(columns, rows, layers, grid)
            .map(|entry| self.map[entry as usize]);
        if !shared::sdf_fine_present(slots) {
            return None;
        }
        let mut values = [0.0; 64];
        for (index, value) in values.iter_mut().enumerate() {
            let (column, row, layer) =
                (columns[index % 4], rows[index / 4 % 4], layers[index / 16]);
            *value = self.values[shared::sdf_fine_index(
                column, row, layer, columns[0], rows[0], layers[0], slots,
            ) as usize];
        }
        Some(shared::sdf_tricubic(coordinate, values, grid))
    }
}

impl Obstacle {
    /// The obstacle's distance and outward normal at a body-frame point: the
    /// shared lookup over this grid, at f64 (what an executor computes at its
    /// own precision).
    #[must_use]
    pub fn sample(&self, point: [f64; 3]) -> SdfSample {
        if let Some(sample) = self.fine.as_ref().and_then(|fine| fine.sample(point)) {
            return sample;
        }
        let grid = self.grid;
        let coordinate = shared::sdf_grid_coordinate(point, grid);
        let columns = shared::sdf_tricubic_axis(coordinate[0], grid.size_x);
        let rows = shared::sdf_tricubic_axis(coordinate[1], grid.size_y);
        let layers = shared::sdf_tricubic_axis(coordinate[2], grid.size_z);
        let mut values = [0.0; 64];
        for (index, value) in values.iter_mut().enumerate() {
            let (column, row, layer) =
                (columns[index % 4], rows[index / 4 % 4], layers[index / 16]);
            *value = self.values[shared::sdf_grid_index(column, row, layer, grid) as usize];
        }
        shared::sdf_tricubic(coordinate, values, grid)
    }

    /// The pose at `time`, interpolated between its samples as an executor
    /// interpolates it.
    // A track holds far fewer than 2^32 samples.
    #[allow(clippy::cast_possible_truncation)]
    #[must_use]
    pub fn pose_at(&self, time: f64) -> Pose {
        let span =
            shared::pose_sample_span(time, self.start, self.interval, self.poses.len() as u32);
        shared::pose_interpolate(
            self.poses[span.lower as usize],
            self.poses[span.upper as usize],
            span.fraction,
        )
    }

    /// The outward normal at a world point, with the obstacle posed as at
    /// `time`: the lookup's normal, rotated into the world frame.
    #[must_use]
    pub fn world_normal(&self, time: f64, point: [f64; 3]) -> [f64; 3] {
        let pose = self.pose_at(time);
        shared::pose_rotate(pose, self.sample(shared::pose_to_body(pose, point)).normal)
    }
}

/// What the stepping loop reads every monitor interval, reduced on the
/// executor. Cumulative values run from the start of the run.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Monitors {
    /// Kinetic energy at the latest half-step velocities.
    pub kinetic_energy: f64,
    /// Internal (strain) energy at the current displacements, with the λ
    /// term averaged over nodes, and any part of it taken at each element's
    /// own volume, as the forces are.
    pub internal_energy: f64,
    /// Kinetic energy of the nodes in contact at the last step: a watch for
    /// friction flutter, which nothing gates (plan §16e).
    pub contact_kinetic_energy: f64,
    /// The resultant of the contact forces on the soft body's nodes, world
    /// frame, averaged over the steps since the last read. The obstacle's
    /// reaction is its negative.
    pub contact_force: [f64; 3],
    /// The moment of the contact forces on the soft body's nodes about the
    /// obstacle's body origin as posed at the start of each step, world
    /// frame, averaged over the steps since the last read: `Σ (xᵢ − p) × fᵢ`,
    /// with `xᵢ` the node's position at the start of the step (plan §16x).
    /// The twist on the obstacle is its negative.
    pub contact_moment: [f64; 3],
    /// The sum of the nodes' contact normal-force magnitudes, averaged over
    /// the steps since the last read: the Coulomb push's `Σ f_n` (15d.7).
    pub normal_force: f64,
    /// The number of steps since the last read.
    pub steps: u64,
    /// Cumulative: element-steps with `J ≤ 0` (K4).
    pub inverted_element_steps: u64,
    /// Cumulative: the work the contact forces have done on the soft body's
    /// nodes.
    pub contact_work: f64,
    /// Cumulative: the work the obstacle's motion has done against the
    /// contact forces on the soft body's nodes. Over each step it is
    /// `F · Δp + M · φ`, with `F` and `M` that step's resultant and moment
    /// (as in [`Monitors::contact_moment`]), and `Δp` and `φ` the body
    /// origin's move and the world-frame rotation vector of the obstacle's
    /// turn over the step, from its pose track at f64 ([`rigid_motion`]).
    /// Linear in the turn over one step. D1's push over a stretch of the
    /// path is its change over the stretch, per unit of travel (plan §16x).
    pub obstacle_work: f64,
    /// Cumulative: the energy mass damping and the material's viscosity have
    /// removed.
    pub damping_loss: f64,
    /// Cumulative: the deepest penetration any node has reached (G2).
    pub max_penetration: f64,
    /// Cumulative: the deepest any node the contact can move has had its
    /// predicted position inside the obstacle: the depth a contact correction
    /// reads, which an obstacle's fine grid must cover (plan §16u, §16w).
    pub deepest_prediction: f64,
    /// Cumulative: node-steps whose contact correction took its depth from
    /// the obstacle's coarse grid, where the fine one did not reach; every
    /// correction, for an obstacle without a fine grid.
    pub coarse_corrections: u64,
}

impl Monitors {
    /// Whether every value read is finite.
    #[must_use]
    pub fn finite(&self) -> bool {
        [
            self.kinetic_energy,
            self.internal_energy,
            self.contact_kinetic_energy,
            self.normal_force,
            self.contact_work,
            self.obstacle_work,
            self.damping_loss,
            self.max_penetration,
            self.deepest_prediction,
        ]
        .iter()
        .chain(&self.contact_force)
        .chain(&self.contact_moment)
        .all(|v| v.is_finite())
    }
}

/// The rigid motion from pose `from` to pose `to`: the body origin's move,
/// and the turn's rotation vector, both in the world frame.
///
/// The turn is `R_to R_fromᵀ`, so a point of the body moves by
/// `Δp + φ × (x − p_from)` to first order in it.
#[must_use]
pub fn rigid_motion(from: Pose, to: Pose) -> ([f64; 3], [f64; 3]) {
    let moved = [to.tx - from.tx, to.ty - from.ty, to.tz - from.tz];
    // The turn `q_to q_from*`, on the shorter arc.
    let (w1, v1) = (to.qw, [to.qx, to.qy, to.qz]);
    let (w2, v2) = (from.qw, [-from.qx, -from.qy, -from.qz]);
    let mut w = w1 * w2 - shared::vec3_dot(v1, v2);
    let mut v = shared::vec3_add(
        shared::vec3_add(shared::vec3_scale(v2, w1), shared::vec3_scale(v1, w2)),
        shared::vec3_cross(v1, v2),
    );
    if w < 0.0 {
        w = -w;
        v = shared::vec3_scale(v, -1.0);
    }
    let sine = shared::vec3_length(v);
    let turn = if sine > 0.0 {
        shared::vec3_scale(v, 2.0 * sine.atan2(w) / sine)
    } else {
        [0.0; 3]
    };
    (moved, turn)
}

/// A read of the executor's per-node state.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Snapshot {
    /// Each node's displacement.
    pub displacements: Vec<[f64; 3]>,
    /// Each node's velocity at the latest half step.
    pub velocities: Vec<[f64; 3]>,
    /// Each node's friction anchor, the obstacle's body frame; zero off the
    /// surface. Part of the state: [`Executor::set_state`] takes it back.
    pub anchors: Vec<[f64; 3]>,
    /// Each node's displacement, summed over the accumulated steps.
    pub displacement_sums: Vec<[f64; 3]>,
    /// Each node's contact normal force, summed over the accumulated steps.
    pub normal_force_sums: Vec<f64>,
    /// Each node's friction force, world frame, summed over the accumulated
    /// steps.
    pub friction_sums: Vec<[f64; 3]>,
    /// How many steps the sums hold.
    pub accumulated_steps: u64,
}

/// What each phase of the last step wrote, for per-phase conformance between
/// executors (plan §14d, §15g step 4). A debugging read, never per step.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PhaseOutputs {
    /// Phase 1: each element's dilation `J − 1`.
    pub dilations: Vec<f64>,
    /// Phase 2: each node's volume change.
    pub volume_changes: Vec<f64>,
    /// Phase 3: each node's averaged pressure.
    pub pressures: Vec<f64>,
    /// Phase 4: each element's twelve elastic nodal force components.
    pub element_forces: Vec<[f64; 12]>,
    /// Phase 4: each element's twelve viscous nodal force components.
    pub element_viscous_forces: Vec<[f64; 12]>,
    /// Phase 5: each node's elastic force.
    pub elastic_forces: Vec<[f64; 3]>,
    /// Phase 5: each node's viscous force.
    pub viscous_forces: Vec<[f64; 3]>,
    /// Phase 6: each node's contact force, world frame; zero off the surface.
    pub contact_forces: Vec<[f64; 3]>,
    /// Phase 6: each node's contact normal-force magnitude.
    pub normal_forces: Vec<f64>,
}

/// The power iteration's estimate of the mode that sets the stable step
/// (plan §16e, §16p).
///
/// It is the top vector `v` of `M⁻¹(K + βC)`, with `K` the elastic stiffness
/// at the current state and `C` the viscosity's damping.
///
/// Central differences with the damping force at the lagging half-step
/// velocity are stable while `4M − Δt²K − 2ΔtC` is positive definite, so at
/// `β = 2/Δt` this is the vector that loses stability first. The step
/// follows from its two quotients (`StepperConfig::stable_step`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TopMode {
    /// `vᵀKv / vᵀMv`: at `β = 0`, an estimate of `ω_el²` from below.
    pub omega_squared: f64,
    /// `vᵀCv / vᵀMv`, with `C v` minus the viscous forces at velocities `v`:
    /// twice the damping ratio times `ω`. Zero for an elastic material.
    pub damping_quotient: f64,
}

impl TopMode {
    /// The viscous damping ratio `ξ = damping_quotient / (2ω)`; not finite
    /// where the stiffness quotient is not positive.
    #[must_use]
    pub fn damping_ratio(&self) -> f64 {
        self.damping_quotient / (2.0 * self.omega_squared.sqrt())
    }
}

/// An explicit executor: the solver's state, and one method per phase.
///
/// A step is the phases in [`crate::stepping`]'s order, and each phase
/// reads what the one before wrote. Nothing is read back to the host except
/// by [`Executor::monitors`], [`Executor::snapshot`],
/// [`Executor::phase_outputs`] and [`Executor::estimate_top_mode`].
pub trait Executor {
    /// The number of nodes.
    fn node_count(&self) -> usize;

    /// The shortest element edge at rest, the length the power iteration's
    /// finite difference is scaled by.
    fn shortest_edge(&self) -> f64;

    /// The executor's machine epsilon: `f32::EPSILON` or `f64::EPSILON`.
    fn epsilon(&self) -> f64;

    /// Replace the state: every node's displacement and half-step velocity,
    /// each projected onto the node's free directions, and the friction
    /// anchors.
    ///
    /// `anchors` is one body-frame anchor per node (read by
    /// [`Executor::snapshot`]), or `None` to anchor each surface node where
    /// it sits at `time`, so every contact starts sticking there.
    ///
    /// # Panics
    /// If a slice's length is not the node count.
    fn set_state(
        &mut self,
        time: f64,
        displacements: &[[f64; 3]],
        velocities: &[[f64; 3]],
        anchors: Option<&[[f64; 3]]>,
    );

    /// Replace the obstacle's pose track: samples every `interval` from
    /// `start`. A run can change how the obstacle moves without losing its
    /// state (plan §14d's poses streamed a batch at a time; K6's loading legs
    /// that end on a force, 16b).
    ///
    /// # Errors
    /// An [`ObstacleError`] if the track is empty, its start is not finite,
    /// its interval is not positive, or a pose is not a unit quaternion with a
    /// finite translation.
    fn set_poses(&mut self, start: f64, interval: f64, poses: &[Pose])
    -> Result<(), ObstacleError>;

    /// Phase 1: each element's dilation `J − 1`, counting inverted elements.
    fn element_dilations(&mut self);

    /// Phase 2: each node's volume change, gathered from its elements.
    fn gather_volume_changes(&mut self);

    /// Phase 3: each node's averaged pressure.
    fn nodal_pressures(&mut self);

    /// Phase 4: each element's elastic forces, and its viscous forces at the
    /// latest half-step velocities, into its own slots. The λ term's pressure
    /// is the mean of its nodes' (phase 3) plus the part taken at the element's
    /// own volume, `κ_e ln J_e / J_e` from phase 1's dilation
    /// (`sampled_element_pressure`, plan §16y).
    fn element_forces(&mut self);

    /// Phase 5: each node's elastic and viscous forces, gathered from its
    /// elements' slots.
    fn gather_forces(&mut self);

    /// Phase 6: each surface node's contact with the obstacle at `time`, for
    /// a step `dt` with mass damping `damping`: the kinematic law predicts
    /// the step's update, so it needs both.
    fn contact(&mut self, time: f64, dt: f64, damping: f64);

    /// Phase 7: the central-difference update of each node's velocity and
    /// displacement, with mass damping `damping`.
    fn integrate(&mut self, dt: f64, damping: f64);

    /// Phase 8: each node's constraints, and the step's energy terms.
    fn boundary_conditions(&mut self, dt: f64, damping: f64);

    /// Add each node's current displacement, contact normal force and
    /// friction force to its window sums.
    fn accumulate(&mut self);

    /// Empty the window sums.
    fn clear_accumulators(&mut self);

    /// Reduce and read the monitors, and restart the since-last-read ones.
    fn monitors(&mut self) -> Monitors;

    /// Read the per-node state. `&mut` because a GPU executor must finish
    /// its queued work before reading back.
    fn snapshot(&mut self) -> Snapshot;

    /// Read what each phase of the last step wrote.
    fn phase_outputs(&mut self) -> PhaseOutputs;

    /// Run `iterations` of the power iteration on `M⁻¹(K + βC)`, with
    /// `β = viscous_weight`, and return the [`TopMode`] of its last vector.
    ///
    /// Each call starts from the same fixed vector. Started from the last
    /// call's vector instead, the iteration stayed on a lower mode once the
    /// tube was loaded, and read up to 3.9 % low (plan §16m).
    ///
    /// `K v` is the finite difference of the elastic force phases in the
    /// direction `v`, with the step `perturbation` in the largest nodal
    /// component; `C v` is minus the viscous forces at velocities `v`. Held
    /// and constrained directions are excluded.
    fn estimate_top_mode(
        &mut self,
        iterations: usize,
        perturbation: f64,
        viscous_weight: f64,
    ) -> TopMode;
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
    /// The grid's layout, a value, a pose or the friction is out of range.
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
    // Three u32s can multiply past a usize, and the lookup indexes in a u32.
    let Some(expected) = [grid.size_x, grid.size_y, grid.size_z]
        .iter()
        .try_fold(1_usize, |product, &n| product.checked_mul(n as usize))
        .filter(|&n| u32::try_from(n).is_ok())
    else {
        return Err(ObstacleError::Invalid {
            reason: "the grid has more samples than a u32 indexes",
        });
    };
    if obstacle.values.len() != expected {
        return Err(ObstacleError::GridSize {
            found: obstacle.values.len(),
            expected,
        });
    }
    check_poses(obstacle.start, obstacle.interval, &obstacle.poses)?;
    if let Some(fine) = &obstacle.fine {
        check_fine(fine)?;
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
    } else if !(obstacle.friction.is_finite() && obstacle.friction >= 0.0) {
        Some("the friction must be finite and not negative")
    } else {
        None
    };
    reason.map_or(Ok(()), |reason| Err(ObstacleError::Invalid { reason }))
}

/// Check a fine grid: a lattice with samples, a map entry per brick, whole
/// bricks of finite values, and slots that name them.
fn check_fine(fine: &FineGrid) -> Result<(), ObstacleError> {
    let grid = fine.grid;
    let too_long = |n: u32| n > u32::MAX - shared::SDF_BRICK;
    if too_long(grid.size_x) || too_long(grid.size_y) || too_long(grid.size_z) {
        return Err(ObstacleError::Invalid {
            reason: "the fine grid's lattice is too long to count its bricks in a u32",
        });
    }
    let brick = (shared::SDF_BRICK * shared::SDF_BRICK * shared::SDF_BRICK) as usize;
    // `None` past a usize.
    let bricks = [grid.size_x, grid.size_y, grid.size_z]
        .iter()
        .try_fold(1_usize, |product, &n| {
            product.checked_mul(shared::sdf_bricks(n) as usize)
        });
    let slots = fine.values.len() / brick;
    let reason = if grid.size_x == 0 || grid.size_y == 0 || grid.size_z == 0 {
        Some("the fine grid has no samples along an axis")
    } else if !(grid.cell_size.is_finite() && grid.cell_size > 0.0) {
        Some("the fine grid's cell size must be positive and finite")
    } else if bricks.is_none_or(|n| u32::try_from(n).is_err()) {
        Some("the fine grid's lattice has more bricks than a u32 indexes")
    } else if bricks != Some(fine.map.len()) {
        Some("the fine grid's map needs one entry per brick of its lattice")
    } else if !fine.values.len().is_multiple_of(brick) {
        Some("the fine grid's values are not whole bricks")
    } else if u32::try_from(fine.values.len()).is_err() {
        Some("the fine grid has more values than a u32 addresses")
    } else if fine
        .map
        .iter()
        .any(|&slot| slot != shared::SDF_NO_BRICK && slot as usize >= slots)
    {
        Some("a fine grid's map entry names a brick it does not have")
    } else if ![grid.origin_x, grid.origin_y, grid.origin_z]
        .iter()
        .chain(&fine.values)
        .all(|v| v.is_finite())
    {
        Some("a fine grid origin or value is not finite")
    } else {
        None
    };
    reason.map_or(Ok(()), |reason| Err(ObstacleError::Invalid { reason }))
}

/// Check a pose track: samples, a finite start, a positive interval, and unit
/// quaternions with finite translations.
///
/// # Errors
/// The first problem found.
pub fn check_poses(start: f64, interval: f64, poses: &[Pose]) -> Result<(), ObstacleError> {
    if poses.is_empty() {
        return Err(ObstacleError::NoPoses);
    }
    if !start.is_finite() {
        return Err(ObstacleError::Invalid {
            reason: "the pose track's start is not finite",
        });
    }
    if !(interval.is_finite() && interval > 0.0) {
        return Err(ObstacleError::Interval { interval });
    }
    if poses.iter().all(|p| {
        let norm = p.qw * p.qw + p.qx * p.qx + p.qy * p.qy + p.qz * p.qz;
        (norm - 1.0).abs() <= 1e-9 && p.tx.is_finite() && p.ty.is_finite() && p.tz.is_finite()
    }) {
        Ok(())
    } else {
        Err(ObstacleError::Invalid {
            reason: "a pose is not a unit quaternion with a finite translation",
        })
    }
}
