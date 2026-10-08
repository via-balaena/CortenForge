//! Forward dynamics pipeline — top-level orchestration.
//!
//! This module implements `step`, `forward`, and `forward_core` on `Data`,
//! which call sub-modules in physics pipeline order. Corresponds to
//! MuJoCo's `engine_forward.c`.
//!
//! ## Pipeline stages (§53)
//!
//! As MuJoCo 3.5.0's `mj_forwardSkip`:
//!
//! - **`forward_pos()`**: position stage (wake detection, FK, CRBA,
//!   transmissions, collision, position sensors, potential energy);
//! - **`forward_vel()`**: velocity stage (velocity FK, actuator lengths and
//!   velocities, passive forces with `cb_passive` and passive plugins, the
//!   bias force `qfrc_bias` (RNE), velocity sensors, kinetic energy);
//! - `cb_control`, unless `DISABLE_ACTUATION` is set;
//! - **`forward_acc()`**: acceleration stage (actuation, constraints,
//!   acceleration, body accumulators, acceleration sensors). MuJoCo builds the
//!   constraint rows in its position and velocity stages; sim-core builds them
//!   here, after `cb_control`.
//!
//! `step1()` runs the first two and `cb_control` (whatever the flag);
//! `step2()` runs `forward_acc()` and integrates.

pub(crate) mod acceleration;
mod actuation;
pub(crate) mod check;
mod fiber;
mod hill;
mod millard;
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
// Public re-exports: stable mathematical functions for muscle curves and dynamics.
// Used by examples and external consumers for direct curve evaluation.
pub use actuation::{hill_active_fl, hill_force_velocity, hill_passive_fl};
pub use millard::{
    MillardCurves, MillardMuscleParams, SmoothSegmentedFunction, default_millard_curves,
    millard_active_gain, millard_isometric_path_force, millard_passive_bias, millard_path_force,
};
pub use muscle::{
    muscle_activation_dynamics, muscle_gain_length, muscle_gain_velocity, muscle_passive_force,
};

#[allow(unused_imports)]
pub(crate) use actuation::{
    actuator_ctrl_input, mj_actuator_velocity, mj_fwd_actuation, mj_gravcomp_to_actuator,
    mj_transmission_body_dispatch, mj_transmission_joint_tendon, mj_transmission_site,
    mj_transmission_slidercrank,
};
#[allow(unused_imports)]
pub(crate) use passive::mj_fwd_passive;
pub(crate) use position::mj_fwd_position;
#[allow(unused_imports)]
pub(crate) use velocity::mj_fwd_velocity;
pub(crate) use velocity::mj_subtree_vel;

// Re-exports for external consumers (derivatives.rs, collision/, constraint/, etc.)
pub(crate) use actuation::mj_next_activation;
pub(crate) use passive::{ellipsoid_moment, fluid_geom_semi_axes, norm3};
pub(crate) use position::SweepAndPrune;
pub(crate) use position::{
    aabb_from_geom_aabb, closest_point_segment, closest_points_segments,
    closest_points_segments_parametric,
};

use crate::types::flags::{disabled, enabled};
use crate::types::{
    DISABLE_ACTUATION, Data, ENABLE_ENERGY, ENABLE_FWDINV, ENABLE_SLEEP, Integrator, Model,
    StepError,
};

/// Pipeline stage for skip-stage forward dispatch.
///
/// Matches MuJoCo's `mjtStage` enum. Used by [`Data::forward_skip`] to
/// conditionally skip position-dependent or velocity-dependent stages
/// when only velocities or controls have been perturbed.
///
/// # Ordering
///
/// `PartialOrd` enables `if skipstage < MjStage::Pos` guards matching
/// MuJoCo's integer comparison pattern in `mj_forwardSkip`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum MjStage {
    /// No stage completed — run full pipeline.
    None = 0,
    /// Position stage complete — skip FK, collision, CRBA.
    Pos = 1,
    /// Velocity stage complete — skip FK, collision, CRBA, velocity FK and
    /// passive forces (`cb_passive` included).
    Vel = 2,
}

impl Data {
    /// Split-step phase 1: position + velocity stages only.
    ///
    /// Runs the forward pipeline through the velocity stage (which fires
    /// `cb_passive`), then fires `cb_control`, even with `DISABLE_ACTUATION`
    /// set. After `step1()`, the user can inject forces (e.g.,
    /// modify `ctrl`, `qfrc_applied`, or `xfrc_applied`) before calling
    /// [`step2()`](Self::step2) which runs the acceleration stage and
    /// integrates.
    ///
    /// # MuJoCo Equivalence
    ///
    /// Matches `mj_step1()` in `engine_forward.c`: runs position + velocity
    /// stages, fires `mjcb_control`, and returns. `mj_step2()` then runs
    /// actuation → acceleration → constraints → integration, and fires
    /// neither callback.
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
    /// - Under RK4, `step2()` takes the Euler step (not RK4). RK4's
    ///   multi-stage substeps don't work with force injection between stages.
    /// - `step()` is NOT refactored to call step1/step2 — it remains the
    ///   canonical entry point with full RK4 support.
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError::InvalidTimestep)` if the timestep is not
    /// positive and finite, `Err(StepError::DataShapeMismatch)` if `self`
    /// was made by a model of other dimensions or an input array was resized,
    /// and `Err(StepError::TendonEqualityWithSleep)` if sleep is enabled and a
    /// tendon equality is active.
    pub fn step1(&mut self, model: &Model) -> Result<(), StepError> {
        check::check_step_inputs(model, self)?;
        check::check_tendon_equality_sleep(model)?;

        // Validate state before stepping
        check::mj_check_pos(model, self);
        check::mj_check_vel(model, self);

        // Position and velocity stages, then cb_control whatever
        // DISABLE_ACTUATION says, as MuJoCo's mj_step1 does
        // (engine_forward.c:1481-1483).
        self.forward_pos(model, true);
        self.forward_vel(model, true);
        if let Some(ref cb) = model.cb_control {
            (cb.0)(model, self);
        }

        Ok(())
    }

    /// Split-step phase 2: acceleration stage + integration.
    ///
    /// Runs actuation, constraints, acc-sensors (no callback fires but on a
    /// step that puts a tree to sleep, whose advance runs the forward pass
    /// once more), then advances with [`integrate`](Self::integrate): Euler under Euler
    /// and RK4 (as MuJoCo's `mj_step2`), the integrator's own velocity update
    /// otherwise, with the sleep step and the warmstart save inside it.
    ///
    /// Must be called after [`step1()`](Self::step1). The user may modify
    /// `ctrl`, `qfrc_applied`, `xfrc_applied`, etc. between step1 and step2
    /// to inject forces that affect the acceleration computation.
    ///
    /// # MuJoCo Equivalence
    ///
    /// Matches `mj_step2()` in `engine_forward.c`: actuation → acceleration
    /// → constraints → the advance (history, activations, sleep, velocity,
    /// position, time, warmstart).
    ///
    /// # Note
    ///
    /// Under RK4 this takes the Euler step, as `mj_step2` does. RK4 is not
    /// compatible with split-step force injection because its multi-stage
    /// substeps recompute `forward()` internally.
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError)` if the timestep is not positive and finite,
    /// if `self` was made by a model of other dimensions or an input array
    /// was resized, or if forward acceleration computation fails (e.g.,
    /// Cholesky failure in implicit integrator).
    pub fn step2(&mut self, model: &Model) -> Result<(), StepError> {
        check::check_step_inputs(model, self)?;

        // §53: RK4 is not compatible with split-step — warn if configured.
        if model.integrator == Integrator::RungeKutta4 {
            log::warn!(
                "step2() uses Euler integration; RK4 requires step() for correct multi-stage substeps"
            );
        }

        // §53: Acceleration stage (actuation, constraints, sensors).
        self.forward_acc(model, true)?;

        // Validate accelerations
        check::mj_check_acc(model, self);

        // The advance (as step() for non-RK4 integrators)
        self.integrate_unchecked(model)
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
    /// - The timestep is not positive and finite (`InvalidTimestep`)
    /// - `self` was made by a model of other dimensions, or an input array
    ///   was resized (`DataShapeMismatch`)
    /// - Sleep is enabled and a tendon equality is active
    ///   (`TendonEqualityWithSleep`)
    ///
    /// NaN/divergence in qpos, qvel, or qacc triggers auto-reset (matching
    /// MuJoCo). Disable with `DISABLE_AUTORESET`. Use `data.divergence_detected()`
    /// to check if a reset occurred. A bad control (after clamping to
    /// `ctrlrange`) makes every actuator's control input 0 for that pass (an
    /// actuator with an activation still acts on it), leaves `ctrl` as written
    /// and counts `Warning::BadCtrl` (matching MuJoCo). Under `implicit` and
    /// `implicitfast` the velocity derivative still reads it for an actuator
    /// whose gain has a velocity term, as MuJoCo's does.
    pub fn step(&mut self, model: &Model) -> Result<(), StepError> {
        check::check_step_inputs(model, self)?;
        check::check_tendon_equality_sleep(model)?;

        // Validate state before stepping — void, auto-resets internally.
        check::mj_check_pos(model, self);
        check::mj_check_vel(model, self);

        match model.integrator {
            Integrator::RungeKutta4 => {
                // RK4: forward() evaluates initial state (with sensors).
                // mj_runge_kutta() then calls forward_skip_sensors() 3 more times.
                self.forward(model)?;
                check::mj_check_acc(model, self);
                crate::integrate::rk4::mj_runge_kutta(model, self)
            }
            Integrator::Euler
            | Integrator::ImplicitSpringDamper
            | Integrator::ImplicitFast
            | Integrator::Implicit => {
                self.forward(model)?;
                check::mj_check_acc(model, self);
                self.integrate_unchecked(model)
            }
        }
    }

    /// Forward dynamics only (like `mj_forward`).
    ///
    /// Computes all derived quantities from current qpos/qvel without
    /// modifying them. After this call, qacc contains the computed
    /// accelerations and all body poses are updated.
    ///
    /// Pipeline stages follow `MuJoCo`'s `mj_forward`:
    /// 1. Position stage: FK, position-dependent sensors, potential energy
    /// 2. Velocity stage: velocity FK, passive forces (`cb_passive` fires at
    ///    their end), the bias force, velocity-dependent sensors, kinetic
    ///    energy
    /// 3. `cb_control`, unless `DISABLE_ACTUATION` is set
    /// 4. Acceleration stage: actuation, constraints, acc-dependent sensors
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError::InvalidTimestep)` if the timestep is not
    /// positive and finite, `Err(StepError::DataShapeMismatch)` if `self` was
    /// made by a model of other dimensions or an input array was resized,
    /// `Err(StepError::TendonEqualityWithSleep)` if sleep is enabled and a
    /// tendon equality is active, and `Err(StepError::CholeskyFailed)` if
    /// using implicit integrator and the modified mass matrix decomposition
    /// fails.
    pub fn forward(&mut self, model: &Model) -> Result<(), StepError> {
        check::check_step_inputs(model, self)?;
        check::check_tendon_equality_sleep(model)?;
        self.forward_core(model, true)
    }

    /// Forward dynamics pipeline without sensor evaluation.
    ///
    /// Identical to [`forward()`](Self::forward) but skips all 4 sensor stages.
    /// Used by RK4 intermediate stages; both callbacks fire, as at every
    /// stage of MuJoCo's `mj_RungeKutta`.
    pub(crate) fn forward_skip_sensors(&mut self, model: &Model) -> Result<(), StepError> {
        self.forward_core(model, false)
    }

    /// Forward dynamics with skip-stage optimization (DT-53).
    ///
    /// Conditionally skips position-dependent and/or velocity-dependent
    /// pipeline stages when only velocities or controls have been perturbed.
    /// This avoids recomputing FK, collision detection, CRBA, and velocity FK
    /// when those quantities are unchanged — reducing per-column FD cost by
    /// ~30–50%.
    ///
    /// # Arguments
    ///
    /// * `skipstage` — which stages to skip:
    ///   - [`MjStage::None`]: run full pipeline (equivalent to `forward()`)
    ///   - [`MjStage::Pos`]: skip position stage (FK, collision, CRBA)
    ///   - [`MjStage::Vel`]: skip position and velocity stages, passive forces
    ///     and `cb_passive` included
    /// * `skipsensor` — when `true`, skip all sensor evaluation.
    ///
    /// `cb_control` fires whatever `skipstage` is, unless `DISABLE_ACTUATION`
    /// is set.
    ///
    /// # MuJoCo Equivalence
    ///
    /// Matches `mj_forwardSkip(m, d, skipstage, skipsensor)` in
    /// `engine_forward.c`. This is forward-only — it does **not** integrate.
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError)` if the timestep is not positive and finite,
    /// if `self` was made by a model of other dimensions or an input array
    /// was resized, if `skipstage` is `MjStage::None` with sleep enabled and a
    /// tendon equality active, or if implicit acceleration solver fails.
    pub fn forward_skip(
        &mut self,
        model: &Model,
        skipstage: MjStage,
        skipsensor: bool,
    ) -> Result<(), StepError> {
        check::check_step_inputs(model, self)?;
        if skipstage == MjStage::None {
            check::check_tendon_equality_sleep(model)?;
        }
        self.forward_skip_stages(model, skipstage, skipsensor, true)
    }

    /// [`Self::forward_skip`] without `cb_control`, for the inverse
    /// finite differences: MuJoCo's `mjd_inverseFD` takes its columns through
    /// `mj_inverseSkip`, which fires no control callback
    /// (`engine_inverse.c:184-245`).
    pub(crate) fn forward_skip_without_control(
        &mut self,
        model: &Model,
        skipstage: MjStage,
        skipsensor: bool,
    ) -> Result<(), StepError> {
        check::check_step_inputs(model, self)?;
        self.forward_skip_stages(model, skipstage, skipsensor, false)
    }

    /// [`Self::forward_skip`] without its input checks, for the sleep step
    /// of the advance, whose caller made them.
    pub(crate) fn forward_skip_unchecked(
        &mut self,
        model: &Model,
        skipstage: MjStage,
        skipsensor: bool,
    ) -> Result<(), StepError> {
        self.forward_skip_stages(model, skipstage, skipsensor, true)
    }

    fn forward_skip_stages(
        &mut self,
        model: &Model,
        skipstage: MjStage,
        skipsensor: bool,
        control: bool,
    ) -> Result<(), StepError> {
        let compute_sensors = !skipsensor;

        // MuJoCo mj_forwardSkip (engine_forward.c:1365-1411): the position
        // stage, the velocity stage, then cb_control whatever `skipstage` is.
        if skipstage < MjStage::Pos {
            self.forward_pos(model, compute_sensors);
        }
        if skipstage < MjStage::Vel {
            self.forward_vel(model, compute_sensors);
        }
        if control {
            self.fire_control_gated(model);
        }
        self.forward_acc(model, compute_sensors)?;

        Ok(())
    }

    /// Shared pipeline core with sleep gating (§16.5).
    ///
    /// `compute_sensors`: `true` for `forward()`, `false` for `forward_skip_sensors()`.
    ///
    /// §53: `forward_pos()`, `forward_vel()`, `cb_control` (unless
    /// `DISABLE_ACTUATION`), `forward_acc()`; RK4's stages included.
    fn forward_core(&mut self, model: &Model, compute_sensors: bool) -> Result<(), StepError> {
        // INVARIANT: forward_core() must NOT call mj_check_pos, mj_check_vel,
        // or mj_check_acc. mj_check_acc() calls forward() after auto-reset —
        // if forward_core() called check functions, a model that diverges from
        // qpos0 would cause infinite recursion. step() orchestrates the
        // check → forward → check sequence externally. This function is a
        // pure computation with no validation side-effects.
        self.forward_pos(model, compute_sensors);
        self.forward_vel(model, compute_sensors);
        // cb_control fires on every forward pass, RK4's stages included
        // (engine_forward.c:1399-1401, reached from mj_RungeKutta at :1100).
        self.fire_control_gated(model);
        self.forward_acc(model, compute_sensors)
    }

    /// Position stage: wake detection, forward kinematics, CRBA,
    /// transmissions, collision, position sensors and potential energy.
    fn forward_pos(&mut self, model: &Model, compute_sensors: bool) {
        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

        // ========== Position Stage ==========
        position::mj_fwd_position(model, self);

        // Wake what the user changed (§16.4), after the kinematics found a
        // sleeping pose changed and before the mass matrix, as MuJoCo's
        // `mj_kinematics` runs `mj_wake` (engine_core_smooth.c:236-241); with
        // sleep disabled it wakes every sleeping tree.
        if crate::island::mj_wake(model, self) {
            crate::island::mj_update_sleep_arrays(model, self);
        }
        crate::dynamics::flex::mj_flex(model, self);
        crate::dynamics::flex::mj_flex_edge(model, self);

        // CRBA: Compute mass matrix M from kinematic tree. Depends only on
        // joint positions (available after FK). MuJoCo computes M in the
        // position stage; we do the same so that mj_energy_vel (velocity
        // stage) has a valid mass matrix.
        crate::dynamics::crba::mj_crba(model, self);

        actuation::mj_transmission_joint_tendon(model, self);
        actuation::mj_transmission_site(model, self);
        actuation::mj_transmission_slidercrank(model, self);

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
    }

    /// Velocity stage: velocity FK, actuator lengths and velocities, passive
    /// forces (`cb_passive` and passive plugins fire at their end), the bias
    /// force, velocity sensors and kinetic energy. MuJoCo: `mj_fwdVelocity`
    /// (engine_forward.c:221-259: `mj_passive` at :250, `mj_rne` at :254),
    /// then `mj_sensorVel` and `mj_energyVel` in `mj_forwardSkip`.
    fn forward_vel(&mut self, model: &Model, compute_sensors: bool) {
        velocity::mj_fwd_velocity(model, self);
        actuation::mj_actuator_velocity(model, self);
        passive::mj_fwd_passive(model, self);
        crate::dynamics::rne::mj_rne(model, self);
        if compute_sensors {
            crate::sensor::mj_sensor_vel(model, self);
        }
        if enabled(model, ENABLE_ENERGY) {
            crate::energy::mj_energy_vel(model, self);
            self.capture_energy_initial();
        } else {
            self.energy_kinetic = 0.0;
        }
    }

    /// `cb_control`, unless actuation is disabled (engine_forward.c:1399).
    fn fire_control_gated(&mut self, model: &Model) {
        if !disabled(model, DISABLE_ACTUATION)
            && let Some(ref cb) = model.cb_control
        {
            (cb.0)(model, self);
        }
    }

    /// Record the drift baseline the first time both energies are computed
    /// after `make_data` or a reset (see [`Data::energy_initial`]).
    fn capture_energy_initial(&mut self) {
        if !self.energy_initial_captured {
            self.energy_initial = self.total_energy();
            self.energy_initial_captured = true;
        }
    }

    /// Acceleration stage of the forward pipeline.
    ///
    /// Runs actuation, constraint solve, forward acceleration, body
    /// accumulators, acc-sensors, and forward/inverse comparison. (CRBA runs
    /// in the position stage, passive forces and RNE in the velocity stage.)
    ///
    /// §53: This is the second half of the pipeline, used by both
    /// `forward_core()` and `step2()`.
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError)` if implicit acceleration solver fails.
    fn forward_acc(&mut self, model: &Model, compute_sensors: bool) -> Result<(), StepError> {
        // ========== Acceleration Stage ==========
        actuation::mj_fwd_actuation(model, self);

        // S4.2a: Route gravcomp → qfrc_actuator for jnt_actgravcomp joints.
        // qfrc_gravcomp comes from the velocity stage's passive forces.
        actuation::mj_gravcomp_to_actuator(model, self);

        // One global solve; the islands are built in it but not solved
        // apart (`island/mod.rs`).
        crate::constraint::mj_fwd_constraint(model, self);

        // ImplicitFast/Implicit: always run mj_fwd_acceleration, even when
        // Newton succeeded: it solves M_hat = M − h·∂f/∂v for qacc_implicit,
        // the acceleration `integrate` advances qvel with, which provides the
        // implicit velocity-derivative stabilization that prevents divergence
        // in stiff-constraint + light-body systems (e.g., connect constraints
        // on ball-joint chains). qacc keeps the explicit acceleration.
        // ImplicitSpringDamper does NOT need this — Newton already uses
        // M_impl via build_m_impl_for_newton().
        let needs_implicit_qacc = matches!(
            model.integrator,
            Integrator::ImplicitFast | Integrator::Implicit
        );
        if !self.newton_solved || needs_implicit_qacc {
            acceleration::mj_fwd_acceleration(model, self)?;
        }

        // (§27F) Pinned flex vertex DOF clamping removed — pinned vertices now have
        // no joints/DOFs (zero body_dof_num), so no qacc/qvel entries to clamp.

        // Clear flg_rnepost after constraint solve — cacc/cfrc_int/cfrc_ext are
        // now stale (qacc changed). mj_body_accumulators() will be triggered on
        // demand by sensors that need it, or by inverse().
        self.flg_rnepost = false;

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

        // solver_fwdinv[0]: constraint discrepancy (not computed).
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
