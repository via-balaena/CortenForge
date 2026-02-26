//! Forward dynamics pipeline — top-level orchestration.
//!
//! This module implements `step`, `forward`, and `forward_core` on `Data`,
//! which call sub-modules in physics pipeline order. Corresponds to
//! MuJoCo's `engine_forward.c`.
//!
//! ## Pipeline Split (§53)
//!
//! The pipeline is factored into two halves for split-step support:
//!
//! - **`forward_pos_vel()`**: Position + velocity stages (wake detection,
//!   FK, collision, velocity FK, energy). Does NOT invoke `cb_control`.
//!
//! - **`forward_acc()`**: Acceleration stage (actuation, CRBA, RNE, passive,
//!   constraints, body accumulators, acc-sensors).
//!
//! `forward_core()` calls both halves with `cb_control` firing at the
//! split boundary. `step1()` runs only `forward_pos_vel()` + `cb_control`;
//! `step2()` runs `forward_acc()` + integration.

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
pub(crate) use acceleration::mj_body_accumulators;
#[allow(unused_imports)]
pub(crate) use acceleration::mj_fwd_acceleration;
#[allow(unused_imports)]
pub(crate) use actuation::{
    mj_actuator_length, mj_fwd_actuation, mj_gravcomp_to_actuator, mj_transmission_body_dispatch,
    mj_transmission_site,
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

use crate::types::flags::{disabled, enabled};
use crate::types::{
    DISABLE_ACTUATION, Data, ENABLE_ENERGY, ENABLE_FWDINV, ENABLE_SLEEP, Integrator, Model,
    StepError,
};

impl Data {
    /// Split-step phase 1: position + velocity stages only.
    ///
    /// Runs the forward pipeline through the velocity stage, then fires
    /// `cb_control`. After `step1()`, the user can inject forces (e.g.,
    /// modify `ctrl`, `qfrc_applied`, or `xfrc_applied`) before calling
    /// [`step2()`](Self::step2) which runs the acceleration stage and
    /// integrates.
    ///
    /// # MuJoCo Equivalence
    ///
    /// Matches `mj_step1()` in `engine_forward.c`: runs position + velocity
    /// stages, fires `mjcb_control`, and returns. `mj_step2()` then runs
    /// actuation → acceleration → constraints → integration.
    ///
    /// # Split-Step Usage
    ///
    /// ```ignore
    /// // Equivalent to data.step(&model) for Euler/Implicit integrators:
    /// data.step1(&model)?;
    /// // Inject forces here (e.g., RL policy output)
    /// data.ctrl[0] = 1.0;
    /// data.step2(&model)?;
    /// ```
    ///
    /// # Important
    ///
    /// - Always uses Euler-style integration in `step2()` (not RK4). RK4's
    ///   multi-stage substeps don't work with force injection between stages.
    /// - `step()` is NOT refactored to call step1/step2 — it remains the
    ///   canonical entry point with full RK4 support.
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError)` if timestep is invalid.
    pub fn step1(&mut self, model: &Model) -> Result<(), StepError> {
        // Validate timestep
        if model.timestep <= 0.0 || !model.timestep.is_finite() {
            return Err(StepError::InvalidTimestep);
        }

        // Validate state before stepping
        check::mj_check_pos(model, self);
        check::mj_check_vel(model, self);

        // §53: Position + velocity stages only (matching MuJoCo's mj_step1).
        self.forward_pos_vel(model, true);

        // §53: cb_control fires between velocity and acceleration stages.
        if let Some(ref cb) = model.cb_control {
            (cb.0)(model, self);
        }

        Ok(())
    }

    /// Split-step phase 2: acceleration stage + integration.
    ///
    /// Runs actuation, dynamics, constraints, acc-sensors, then integrates
    /// positions and velocities using Euler-style integration (regardless of
    /// `model.integrator`), sleep update, and warmstart save.
    ///
    /// Must be called after [`step1()`](Self::step1). The user may modify
    /// `ctrl`, `qfrc_applied`, `xfrc_applied`, etc. between step1 and step2
    /// to inject forces that affect the acceleration computation.
    ///
    /// # MuJoCo Equivalence
    ///
    /// Matches `mj_step2()` in `engine_forward.c`: actuation → acceleration
    /// → constraints → integration → sleep → warmstart.
    ///
    /// # Note
    ///
    /// This always uses Euler-style integration. RK4 is not compatible with
    /// split-step force injection because its multi-stage substeps recompute
    /// `forward()` internally.
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError)` if forward acceleration computation fails
    /// (e.g., Cholesky failure in implicit integrator).
    pub fn step2(&mut self, model: &Model) -> Result<(), StepError> {
        // §53: RK4 is not compatible with split-step — warn if configured.
        if model.integrator == Integrator::RungeKutta4 {
            tracing::warn!(
                "step2() uses Euler integration; RK4 requires step() for correct multi-stage substeps"
            );
        }

        // §53: Acceleration stage (actuation, constraints, sensors).
        self.forward_acc(model, true)?;

        // Validate accelerations
        check::mj_check_acc(model, self);

        // Euler-style integration (matches step() for non-RK4 integrators)
        self.integrate(model);

        // Sleep update (§16.12): Phase B island-aware sleep transition.
        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
        if sleep_enabled {
            crate::island::mj_sleep(model, self);
            crate::island::mj_update_sleep_arrays(model, self);
        }

        // Save qacc for next-step warmstart (§15.9).
        self.qacc_warmstart.copy_from(&self.qacc);

        Ok(())
    }

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
    ///
    /// §53: Calls `forward_pos_vel()` + `cb_control` + `forward_acc()`.
    /// `cb_control` only fires on full forward (compute_sensors=true), not on
    /// RK4 intermediate stages.
    fn forward_core(&mut self, model: &Model, compute_sensors: bool) -> Result<(), StepError> {
        // INVARIANT: forward_core() must NOT call mj_check_pos, mj_check_vel,
        // or mj_check_acc. mj_check_acc() calls forward() after auto-reset —
        // if forward_core() called check functions, a model that diverges from
        // qpos0 would cause infinite recursion. step() orchestrates the
        // check → forward → check sequence externally. This function is a
        // pure computation with no validation side-effects.
        self.forward_pos_vel(model, compute_sensors);

        // §53: cb_control fires between velocity and acceleration stages.
        // Only on full forward (not RK4 intermediate stages).
        // Gated on !DISABLE_ACTUATION — MuJoCo skips mjcb_control when actuation is disabled.
        if compute_sensors && !disabled(model, DISABLE_ACTUATION) {
            if let Some(ref cb) = model.cb_control {
                (cb.0)(model, self);
            }
        }

        self.forward_acc(model, compute_sensors)
    }

    /// Position + velocity stages of the forward pipeline.
    ///
    /// Runs wake detection, forward kinematics, collision, velocity FK,
    /// position/velocity sensors, and energy computation.
    ///
    /// §53: This is the first half of the pipeline, used by both
    /// `forward_core()` and `step1()`.
    fn forward_pos_vel(&mut self, model: &Model, compute_sensors: bool) {
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

        // CRBA: Compute mass matrix M from kinematic tree. Depends only on
        // joint positions (available after FK). MuJoCo computes M in the
        // position stage; we do the same so that mj_energy_vel (velocity
        // stage) has a valid mass matrix.
        crate::dynamics::crba::mj_crba(model, self);

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
        // §53: Kinetic energy belongs to the velocity stage (MuJoCo computes
        // it in mj_step1, before the acceleration stage).
        if enabled(model, ENABLE_ENERGY) {
            crate::energy::mj_energy_vel(model, self);
        } else {
            self.energy_kinetic = 0.0;
        }
    }

    /// Acceleration stage of the forward pipeline.
    ///
    /// Runs actuation, RNE, passive forces, constraint solve,
    /// forward acceleration, body accumulators, acc-sensors, and
    /// forward/inverse comparison. (CRBA runs in forward_pos_vel.)
    ///
    /// §53: This is the second half of the pipeline, used by both
    /// `forward_core()` and `step2()`.
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError)` if implicit acceleration solver fails.
    fn forward_acc(&mut self, model: &Model, compute_sensors: bool) -> Result<(), StepError> {
        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

        // ========== Acceleration Stage ==========
        actuation::mj_fwd_actuation(model, self);
        // Note: CRBA already ran in forward_pos_vel() (position stage).
        // The mass matrix depends only on qpos, which doesn't change between stages.
        crate::dynamics::rne::mj_rne(model, self);
        passive::mj_fwd_passive(model, self);

        // S4.2a: Route gravcomp → qfrc_actuator for jnt_actgravcomp joints.
        // Must run after mj_fwd_passive() which computes qfrc_gravcomp.
        actuation::mj_gravcomp_to_actuator(model, self);

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

        // §51: Compute per-body accelerations and forces after constraints.
        acceleration::mj_body_accumulators(model, self);

        if compute_sensors {
            crate::sensor::mj_sensor_acc(model, self);
            crate::sensor::mj_sensor_postprocess(model, self);
        }

        // §52: Forward/inverse comparison (diagnostic only).
        if enabled(model, ENABLE_FWDINV) {
            self.compare_fwd_inv(model);
        }

        Ok(())
    }

    /// Compare forward and inverse dynamics (§52, `ENABLE_FWDINV`).
    ///
    /// Runs `inverse()` and computes two L2 norms matching MuJoCo's
    /// `mj_compareFwdInv()`:
    /// - `solver_fwdinv[0]`: constraint force discrepancy (reserved, 0.0).
    /// - `solver_fwdinv[1]`: `‖qfrc_inverse - fwd_applied‖` where
    ///   `fwd_applied = qfrc_smooth + qfrc_bias - qfrc_passive`.
    ///
    /// After G22, `qfrc_inverse = qfrc_applied + qfrc_actuator + J^T*xfrc`,
    /// so `solver_fwdinv[1]` measures the round-trip residual of applied forces.
    ///
    /// This is purely diagnostic — no physics effect.
    fn compare_fwd_inv(&mut self, model: &Model) {
        if model.nv == 0 {
            return;
        }

        // Run inverse dynamics to populate qfrc_inverse
        self.inverse(model);

        // solver_fwdinv[0]: constraint discrepancy (reserved — no per-island
        // solver residual tracked yet).
        self.solver_fwdinv[0] = 0.0;

        // solver_fwdinv[1]: applied force discrepancy.
        // Forward: M*qacc = qfrc_smooth + qfrc_constraint
        //   where qfrc_smooth = qfrc_applied + qfrc_actuator + qfrc_passive
        //                       - qfrc_bias + J^T*xfrc_applied
        // Inverse: qfrc_inverse = M*qacc + qfrc_bias - qfrc_passive - qfrc_constraint
        //        = qfrc_applied + qfrc_actuator + J^T*xfrc_applied
        //
        // So the comparison vector is: qfrc_smooth + qfrc_bias - qfrc_passive
        //   = qfrc_applied + qfrc_actuator + J^T*xfrc_applied
        // And we compute ‖qfrc_inverse - comparison‖₂.
        let mut sum_sq = 0.0_f64;
        for i in 0..model.nv {
            let fwd = self.qfrc_smooth[i] + self.qfrc_bias[i] - self.qfrc_passive[i];
            let diff = self.qfrc_inverse[i] - fwd;
            sum_sq += diff * diff;
        }
        self.solver_fwdinv[1] = sum_sq.sqrt();
    }
}
