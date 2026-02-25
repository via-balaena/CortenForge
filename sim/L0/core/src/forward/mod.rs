//! Forward dynamics pipeline — top-level orchestration.
//!
//! This module implements `step`, `forward`, and `forward_core` on `Data`,
//! which call sub-modules in physics pipeline order. Corresponds to
//! MuJoCo's `engine_forward.c`.

mod acceleration;
mod actuation;
pub(crate) mod check;
mod muscle;
mod passive;
mod position;
mod velocity;

// Re-exports — pipeline functions for external consumers.
// forward_core() calls these via submodule paths (e.g. position::mj_fwd_position).
// The re-exports here make them available as crate::forward::mj_fwd_position etc.
// Some are not yet imported externally (used only within forward_core), hence allow.
#[allow(unused_imports)]
pub(crate) use acceleration::mj_fwd_acceleration;
#[allow(unused_imports)]
pub(crate) use actuation::{
    mj_actuator_length, mj_fwd_actuation, mj_transmission_body_dispatch, mj_transmission_site,
};
#[allow(unused_imports)]
pub(crate) use muscle::muscle_activation_dynamics;
#[allow(unused_imports)]
pub(crate) use passive::mj_fwd_passive;
pub(crate) use position::mj_fwd_position;
#[allow(unused_imports)]
pub(crate) use velocity::mj_fwd_velocity;

// Re-exports for external consumers (derivatives.rs, collision/, constraint/, etc.)
pub(crate) use actuation::mj_next_activation;
pub(crate) use passive::{ellipsoid_moment, fluid_geom_semi_axes, norm3};
pub(crate) use position::SweepAndPrune;
pub(crate) use position::{aabb_from_geom, closest_point_segment, closest_points_segments};

use crate::types::flags::enabled;
use crate::types::{Data, ENABLE_ENERGY, ENABLE_SLEEP, Integrator, Model, StepError};

impl Data {
    /// Perform one simulation step.
    ///
    /// This is the top-level entry point for advancing the simulation by one timestep.
    /// It performs forward dynamics to compute accelerations, then integrates
    /// to update positions and velocities.
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError)` if:
    /// - Cholesky decomposition fails (implicit integrator only)
    /// - LU decomposition fails (implicit integrator only)
    /// - Timestep is invalid
    ///
    /// NaN/divergence in qpos, qvel, or qacc triggers auto-reset (matching
    /// MuJoCo). Disable with `DISABLE_AUTORESET`. Use `data.divergence_detected()`
    /// to check if a reset occurred.
    pub fn step(&mut self, model: &Model) -> Result<(), StepError> {
        // Validate timestep
        if model.timestep <= 0.0 || !model.timestep.is_finite() {
            return Err(StepError::InvalidTimestep);
        }

        // Validate state before stepping — void, auto-resets internally.
        check::mj_check_pos(model, self);
        check::mj_check_vel(model, self);

        match model.integrator {
            Integrator::RungeKutta4 => {
                // RK4: forward() evaluates initial state (with sensors).
                // mj_runge_kutta() then calls forward_skip_sensors() 3 more times.
                self.forward(model)?;
                check::mj_check_acc(model, self);
                crate::integrate::rk4::mj_runge_kutta(model, self)?;
            }
            Integrator::Euler
            | Integrator::ImplicitSpringDamper
            | Integrator::ImplicitFast
            | Integrator::Implicit => {
                self.forward(model)?;
                check::mj_check_acc(model, self);
                self.integrate(model);
            }
        }

        // Sleep update (§16.12): Phase B island-aware sleep transition.
        // After integration and before warmstart save.
        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
        if sleep_enabled {
            crate::island::mj_sleep(model, self);
            crate::island::mj_update_sleep_arrays(model, self);
        }

        // Save qacc for next-step warmstart (§15.9).
        // Done at the very end of step(), after integration, matching MuJoCo's
        // mj_advance() which saves qacc_warmstart after the step completes.
        self.qacc_warmstart.copy_from(&self.qacc);

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
        self.forward_core(model, true)
    }

    /// Forward dynamics pipeline without sensor evaluation.
    ///
    /// Identical to [`forward()`](Self::forward) but skips all 4 sensor stages.
    /// Used by RK4 intermediate stages.
    pub(crate) fn forward_skip_sensors(&mut self, model: &Model) -> Result<(), StepError> {
        self.forward_core(model, false)
    }

    /// Shared pipeline core with sleep gating (§16.5).
    ///
    /// `compute_sensors`: `true` for `forward()`, `false` for `forward_skip_sensors()`.
    fn forward_core(&mut self, model: &Model, compute_sensors: bool) -> Result<(), StepError> {
        // INVARIANT: forward_core() must NOT call mj_check_pos, mj_check_vel,
        // or mj_check_acc. mj_check_acc() calls forward() after auto-reset —
        // if forward_core() called check functions, a model that diverges from
        // qpos0 would cause infinite recursion. step() orchestrates the
        // check → forward → check sequence externally. This function is a
        // pure computation with no validation side-effects.
        // Sleep is only active after the initial forward pass.
        // The first forward (time == 0.0) must compute FK for all bodies
        // to establish initial positions, even for Init-sleeping bodies.
        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

        // ===== Pre-pipeline: Wake detection (§16.4) =====
        // Must update sleep arrays after user-force wake so that
        // body_awake_ind/dof_awake_ind are current before mj_crba (§16.29.3).
        if sleep_enabled && crate::island::mj_wake(model, self) {
            crate::island::mj_update_sleep_arrays(model, self);
        }

        // ========== Position Stage ==========
        position::mj_fwd_position(model, self);
        crate::dynamics::flex::mj_flex(model, self);

        // §16.15: If FK detected external qpos changes on sleeping bodies, wake them
        if sleep_enabled && crate::island::mj_check_qpos_changed(model, self) {
            crate::island::mj_update_sleep_arrays(model, self);
        }

        actuation::mj_transmission_site(model, self);

        // §16.13.2: Tendon wake — multi-tree tendons with active limits
        if sleep_enabled && crate::island::mj_wake_tendon(model, self) {
            crate::island::mj_update_sleep_arrays(model, self);
        }

        crate::collision::mj_collision(model, self);

        // Wake-on-contact: if sleeping body touched awake body, wake it
        // and re-run collision for the newly-awake tree's geoms (§16.5c)
        if sleep_enabled && crate::island::mj_wake_collision(model, self) {
            crate::island::mj_update_sleep_arrays(model, self);
            crate::collision::mj_collision(model, self);
        }

        // §16.13.3: Equality constraint wake — cross-tree equality coupling
        if sleep_enabled && crate::island::mj_wake_equality(model, self) {
            crate::island::mj_update_sleep_arrays(model, self);
        }

        // §36: Body transmission — requires contacts from mj_collision()
        actuation::mj_transmission_body_dispatch(model, self);

        if compute_sensors {
            crate::sensor::mj_sensor_pos(model, self);
        }
        // S5.1: Gate energy computation on ENABLE_ENERGY; zero when disabled.
        if enabled(model, ENABLE_ENERGY) {
            crate::energy::mj_energy_pos(model, self);
        } else {
            self.energy_potential = 0.0;
        }

        // ========== Velocity Stage ==========
        velocity::mj_fwd_velocity(model, self);
        actuation::mj_actuator_length(model, self);
        if compute_sensors {
            crate::sensor::mj_sensor_vel(model, self);
        }

        // ========== Acceleration Stage ==========
        actuation::mj_fwd_actuation(model, self);
        crate::dynamics::crba::mj_crba(model, self);
        crate::dynamics::rne::mj_rne(model, self);
        // S5.1: Gate energy computation on ENABLE_ENERGY; zero when disabled.
        if enabled(model, ENABLE_ENERGY) {
            crate::energy::mj_energy_vel(model, self);
        } else {
            self.energy_kinetic = 0.0;
        }
        passive::mj_fwd_passive(model, self);

        // §16.11: Island discovery must run BEFORE constraint solve so that
        // contact_island assignments are available for per-island partitioning.
        if sleep_enabled {
            crate::island::mj_island(model, self);
        }

        // §16.16: Per-island constraint solve when islands are active;
        // falls back to global solve when DISABLE_ISLAND or no islands.
        crate::constraint::mj_fwd_constraint_islands(model, self);

        if !self.newton_solved {
            acceleration::mj_fwd_acceleration(model, self)?;
        }

        // (§27F) Pinned flex vertex DOF clamping removed — pinned vertices now have
        // no joints/DOFs (zero body_dof_num), so no qacc/qvel entries to clamp.

        if compute_sensors {
            crate::sensor::mj_sensor_acc(model, self);
            crate::sensor::mj_sensor_postprocess(model, self);
        }

        Ok(())
    }
}
