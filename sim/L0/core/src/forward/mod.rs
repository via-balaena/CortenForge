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
//! - **`position_stage()`**: position stage (wake detection, FK, CRBA,
//!   collision, the constraint rows and their islands, transmissions,
//!   position sensors, potential energy);
//! - **`velocity_stage()`**: velocity stage (velocity FK, actuator lengths and
//!   velocities, passive forces with `cb_passive` and passive plugins, the
//!   rows' velocities and reference accelerations, the bias force
//!   `qfrc_bias` (RNE), velocity sensors, kinetic energy);
//! - `cb_control`, unless `DISABLE_ACTUATION` is set;
//! - **`forward_acc()`**: acceleration stage (actuation, the constraint
//!   solve, acceleration, body accumulators, acceleration sensors).
//!
//! `step1()` runs the first two and `cb_control` (whatever the flag);
//! `step2()` runs `forward_acc()` and integrates. `inverse()` runs the first
//! two as MuJoCo's inverse dynamics do (`Pass::Inverse`).

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
pub(crate) use acceleration::{awake_dofs, mj_implicit_factor, mj_implicit_solve, solve_implicit};
// Public re-exports: stable mathematical functions for muscle curves and dynamics.
// Used by examples and external consumers for direct curve evaluation.
pub use actuation::{hill_active_fl, hill_force_velocity, hill_passive_fl};
pub(crate) use hill::muscle_length;
pub use millard::{
    MillardCurves, MillardMuscleParams, SmoothSegmentedFunction, default_millard_curves,
    millard_active_gain, millard_isometric_path_force, millard_passive_bias, millard_path_force,
};
pub use muscle::{
    muscle_activation_dynamics, muscle_bias, muscle_f0, muscle_gain, muscle_gain_length,
    muscle_gain_velocity, muscle_passive_force,
};

#[allow(unused_imports)]
pub(crate) use actuation::{
    MomentRows, acts_through_moment, actuator_ctrl_input, ball_free_transmission,
    mj_actuator_velocity, mj_fwd_actuation, mj_gravcomp_to_actuator, mj_transmission_body_dispatch,
    mj_transmission_joint_tendon, mj_transmission_site, mj_transmission_slidercrank,
};
#[allow(unused_imports)]
pub(crate) use passive::mj_fwd_passive;
pub(crate) use position::mj_fwd_position;
#[allow(unused_imports)]
pub(crate) use velocity::mj_fwd_velocity;
pub(crate) use velocity::mj_subtree_vel;

// Re-exports for external consumers (derivatives.rs, collision/, constraint/, etc.)
pub(crate) use actuation::{
    actuator_plugin, actuator_skipped_asleep, last_activation, mj_next_activation,
    unclamped_next_activation,
};
pub(crate) use passive::{ellipsoid_moment, fluid_geom_semi_axes, norm3};

use crate::types::flags::{disabled, enabled};
use crate::types::{
    DISABLE_ACTUATION, Data, ENABLE_ENERGY, ENABLE_FWDINV, ENABLE_INVDISCRETE, ENABLE_SLEEP,
    Integrator, Model, StepError,
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
    /// Position stage complete — skip FK, CRBA, collision and the constraint
    /// rows and islands (the rows held are solved).
    Pos = 1,
    /// Velocity stage complete — skip the position stage, velocity FK,
    /// passive forces (`cb_passive` included) and the rows' references.
    Vel = 2,
}

/// The pass a position or velocity stage runs in. MuJoCo's inverse dynamics
/// run their own two (`mj_invPosition`, `mj_invVelocity`,
/// `engine_inverse.c:38-72`), which differ from the forward pass's in the
/// position stage's wakes and islands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Pass {
    /// [`Data::forward`]'s, and every pass that steps.
    Forward,
    /// [`Data::inverse`]'s.
    Inverse,
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
    /// `Err(StepError::TendonEqualityWithSleep)` if sleep is enabled and a
    /// tendon equality is active, `Err(StepError::UndefinedContactFrame)`
    /// if a contact's normal, as its collision function wrote it, is shorter
    /// than 0.5, and `Err(StepError::ConstraintWithoutTree)` if, at the dense
    /// size with islands, a contact between two static geoms made rows.
    pub fn step1(&mut self, model: &Model) -> Result<(), StepError> {
        check::check_step_inputs(model, self)?;
        check::check_tendon_equality_sleep(model)?;

        // Validate state before stepping
        check::mj_check_pos(model, self);
        check::mj_check_vel(model, self);

        // Position and velocity stages, then cb_control whatever
        // DISABLE_ACTUATION says, as MuJoCo's mj_step1 does
        // (engine_forward.c:1481-1483).
        self.position_stage(model, true, Pass::Forward)?;
        self.velocity_stage(model, true, Pass::Forward);
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
    /// was resized, with `ENABLE_FWDINV` under an integrator the discrete
    /// inverse dynamics refuse when the comparison reaches the inverse (the
    /// pass made constraint rows; `UnsupportedInverseDiscrete`), or if forward
    /// acceleration computation fails (`CholeskyFailed` under the implicit
    /// spring-damper integrator, or when Newton's sparse Hessian loses rank).
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
        if enabled(model, ENABLE_FWDINV) {
            self.compare_fwd_inv(model)?;
        }

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
    /// - the implicit spring-damper integrator's matrix is not positive
    ///   definite, or Newton's sparse Hessian loses rank (`CholeskyFailed`)
    /// - under the implicit integrator, a diagonal of the LU factor is below
    ///   mjMINVAL in magnitude (`LuSingular`)
    /// - The timestep is not positive and finite (`InvalidTimestep`)
    /// - `self` was made by a model of other dimensions, or an input array
    ///   was resized (`DataShapeMismatch`)
    /// - Sleep is enabled and a tendon equality is active
    ///   (`TendonEqualityWithSleep`)
    /// - `ENABLE_FWDINV` is set under an integrator the discrete inverse
    ///   dynamics refuse, and the forward pass made constraint rows, so that
    ///   the comparison reaches the inverse (`UnsupportedInverseDiscrete`)
    /// - A contact's normal, as its collision function wrote it, is shorter
    ///   than 0.5 (`UndefinedContactFrame`)
    /// - At the dense size with islands, a contact between two static geoms
    ///   made rows (`ConstraintWithoutTree`)
    ///
    /// NaN/divergence in qpos, qvel, or qacc triggers auto-reset (matching
    /// MuJoCo). Disable with `DISABLE_AUTORESET`. Use `data.divergence_detected()`
    /// to check if a reset occurred. A step runs on the calling thread. A bad
    /// control (after clamping to
    /// `ctrlrange`) makes every actuator's control input 0 for that pass (an
    /// actuator with an activation still acts on it), leaves `ctrl` as written
    /// and counts `Warning::BadCtrl` (matching MuJoCo). Under `implicit` and
    /// `implicitfast` the velocity derivative still reads it for an actuator
    /// without dynamics whose gain has a velocity term, as MuJoCo's does.
    pub fn step(&mut self, model: &Model) -> Result<(), StepError> {
        self.step_skip_factor(model, false)
    }

    /// [`Self::step`]; with `skipfactor` the implicit integrators solve with
    /// the factor `self` holds, as MuJoCo's `mj_stepSkip` from the velocity
    /// stage does (`engine_derivative_fd.c:113-148`), through
    /// `mj_implicitSkip`'s `skipfactor`: the finite-difference transitions
    /// step their control and activation columns so. The position and
    /// velocity stages run in full (MuJoCo skips them).
    pub(crate) fn step_skip_factor(
        &mut self,
        model: &Model,
        skipfactor: bool,
    ) -> Result<(), StepError> {
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
                if enabled(model, ENABLE_FWDINV) {
                    self.compare_fwd_inv(model)?;
                }
                crate::integrate::rk4::mj_runge_kutta(model, self)
            }
            Integrator::Euler
            | Integrator::ImplicitSpringDamper
            | Integrator::ImplicitFast
            | Integrator::Implicit => {
                self.forward(model)?;
                check::mj_check_acc(model, self);
                if enabled(model, ENABLE_FWDINV) {
                    self.compare_fwd_inv(model)?;
                }
                self.integrate_skip_unchecked(model, skipfactor)
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
    /// 1. Position stage: FK, CRBA, collision, the constraint rows and their
    ///    islands, position-dependent sensors, potential energy
    /// 2. Velocity stage: velocity FK, passive forces (`cb_passive` fires at
    ///    their end), the rows' velocities and reference accelerations, the
    ///    bias force, velocity-dependent sensors, kinetic energy
    /// 3. `cb_control`, unless `DISABLE_ACTUATION` is set
    /// 4. Acceleration stage: actuation, the constraint solve (`efc_b`, the
    ///    forces), acc-dependent sensors
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError::InvalidTimestep)` if the timestep is not
    /// positive and finite, `Err(StepError::DataShapeMismatch)` if `self` was
    /// made by a model of other dimensions or an input array was resized,
    /// `Err(StepError::TendonEqualityWithSleep)` if sleep is enabled and a
    /// tendon equality is active, `Err(StepError::UndefinedContactFrame)` if a
    /// contact's normal, as its collision function wrote it, is shorter than
    /// 0.5, `Err(StepError::ConstraintWithoutTree)` if, at the dense size with
    /// islands, a contact between two static geoms made rows, and
    /// `Err(StepError::CholeskyFailed)` under the implicit spring-damper
    /// integrator when its modified mass matrix is not positive definite, or
    /// when Newton's sparse Hessian loses rank. The implicit and implicitfast integrators'
    /// factor is the integrator's (`integrate`), not this pass's.
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
    ///   - [`MjStage::Pos`]: skip the position stage (FK, CRBA, collision, the
    ///     constraint rows: the rows held are solved)
    ///   - [`MjStage::Vel`]: skip the position and velocity stages, passive
    ///     forces, `cb_passive` and the rows' references included
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
    /// tendon equality active or a contact's normal shorter than 0.5
    /// (`UndefinedContactFrame`) or, at the dense size with islands, a contact
    /// between two static geoms that made rows (`ConstraintWithoutTree`), or
    /// if the implicit spring-damper integrator's factorization fails or
    /// Newton's sparse Hessian loses rank (`CholeskyFailed`).
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
        self.forward_skip_unchecked(model, skipstage, skipsensor)
    }

    /// [`Self::forward_skip`] without its input checks, for the sleep step
    /// of the advance, whose caller made them.
    pub(crate) fn forward_skip_unchecked(
        &mut self,
        model: &Model,
        skipstage: MjStage,
        skipsensor: bool,
    ) -> Result<(), StepError> {
        let compute_sensors = !skipsensor;

        // MuJoCo mj_forwardSkip (engine_forward.c:1365-1411): the position
        // stage, the velocity stage, then cb_control whatever `skipstage` is.
        if skipstage < MjStage::Pos {
            self.position_stage(model, compute_sensors, Pass::Forward)?;
        }
        if skipstage < MjStage::Vel {
            self.velocity_stage(model, compute_sensors, Pass::Forward);
        }
        self.fire_control_gated(model);
        self.forward_acc(model, compute_sensors)
    }

    /// Shared pipeline core with sleep gating (§16.5).
    ///
    /// `compute_sensors`: `true` for `forward()`, `false` for `forward_skip_sensors()`.
    ///
    /// §53: `position_stage()`, `velocity_stage()`, `cb_control` (unless
    /// `DISABLE_ACTUATION`), `forward_acc()`; RK4's stages included.
    fn forward_core(&mut self, model: &Model, compute_sensors: bool) -> Result<(), StepError> {
        // INVARIANT: forward_core() must NOT call mj_check_pos, mj_check_vel,
        // or mj_check_acc. mj_check_acc() calls forward() after auto-reset —
        // if forward_core() called check functions, a model that diverges from
        // qpos0 would cause infinite recursion. step() orchestrates the
        // check → forward → check sequence externally. This function is a
        // pure computation with no validation side-effects.
        self.position_stage(model, compute_sensors, Pass::Forward)?;
        self.velocity_stage(model, compute_sensors, Pass::Forward);
        // cb_control fires on every forward pass, RK4's stages included
        // (engine_forward.c:1399-1401, reached from mj_RungeKutta at :1100).
        self.fire_control_gated(model);
        self.forward_acc(model, compute_sensors)
    }

    /// Position stage: wake detection, forward kinematics, CRBA, collision,
    /// the constraint rows and their islands, transmissions, position
    /// sensors and potential energy. In [`Pass::Inverse`] it is MuJoCo's
    /// `mj_invPosition` (`engine_inverse.c:38-66`): the kinematics still
    /// wake the trees the user changed (`mj_kinematics` calls `mj_wake`),
    /// but no tendon, contact or equality wakes the trees they join, and the
    /// rows get no islands.
    ///
    /// # Errors
    ///
    /// After each collision, [`StepError::UndefinedContactFrame`]
    /// ([`check::check_contact_frames`]). In [`Pass::Forward`], from the
    /// island partition, [`StepError::ConstraintWithoutTree`] (`mj_island`).
    /// In [`Pass::Inverse`] only, after
    /// collision: [`StepError::ContactWithSleepingGeom`]
    /// ([`check::check_contacts_awake`]), which the forward pass's contact
    /// wake leaves nothing to refuse.
    pub(crate) fn position_stage(
        &mut self,
        model: &Model,
        compute_sensors: bool,
        pass: Pass,
    ) -> Result<(), StepError> {
        let wakes = pass == Pass::Forward && model.enableflags & ENABLE_SLEEP != 0;

        // ========== Position Stage ==========
        // The kinematics wake what the user changed (`mj_wake`), before the
        // tendons and the mass matrix.
        position::mj_fwd_position(model, self);
        crate::dynamics::flex::mj_flex(model, self);
        crate::dynamics::flex::mj_flex_edge(model, self);

        // CRBA: Compute mass matrix M from kinematic tree. Depends only on
        // joint positions (available after FK). MuJoCo computes M in the
        // position stage; we do the same so that mj_energy_vel (velocity
        // stage) has a valid mass matrix.
        crate::dynamics::crba::mj_crba(model, self);

        // §16.13.2: Tendon wake — multi-tree tendons with active limits
        if wakes && crate::island::mj_wake_tendon(model, self) {
            crate::island::mj_update_sleep_arrays(model, self);
        }

        crate::collision::mj_collision(model, self);
        check::check_contact_frames(self)?;

        // Wake-on-contact: if sleeping body touched awake body, wake it
        // and re-run collision for the newly-awake tree's geoms (§16.5c)
        if wakes && crate::island::mj_wake_collision(model, self) {
            crate::island::mj_update_sleep_arrays(model, self);
            crate::collision::mj_collision(model, self);
            check::check_contact_frames(self)?;
        }

        // §16.13.3: Equality constraint wake — cross-tree equality coupling
        if wakes && crate::island::mj_wake_equality(model, self) {
            crate::island::mj_update_sleep_arrays(model, self);
        }
        if pass == Pass::Inverse {
            check::check_contacts_awake(model, self)?;
        }

        // The constraint rows and their islands, after the wakes, as
        // MuJoCo's `mj_fwdPosition` (`engine_forward.c:203-206`).
        crate::constraint::mj_make_constraint(model, self);
        if pass == Pass::Forward {
            crate::island::mj_island(model, self)?;
        }

        // The transmissions, after the wakes, as MuJoCo's `mj_transmission`
        // (`engine_forward.c:209`): an actuator on a tree this pass woke
        // is not skipped as asleep. A body transmission reads the contacts.
        actuation::mj_transmission_joint_tendon(model, self);
        actuation::mj_transmission_site(model, self);
        actuation::mj_transmission_slidercrank(model, self);
        actuation::mj_transmission_body_dispatch(model, self);

        // The rows' inverse inertia while a dual solver runs, as MuJoCo's
        // `mj_fwdPosition` (`engine_forward.c:213`); its inverse position
        // pass makes none (`mj_invPosition`), nor this one under the implicit
        // spring-damper integrator, whose constraint stage makes it over
        // `M_impl`.
        if pass == Pass::Forward && model.integrator != Integrator::ImplicitSpringDamper {
            crate::constraint::mj_project_constraint(model, self);
        }

        if compute_sensors {
            crate::sensor::mj_sensor_pos(model, self);
        }
        // S5.1: Gate energy computation on ENABLE_ENERGY; zero when disabled.
        if enabled(model, ENABLE_ENERGY) {
            crate::energy::mj_energy_pos(model, self);
        } else {
            self.energy_potential = 0.0;
        }
        Ok(())
    }

    /// Velocity stage: velocity FK, actuator lengths and velocities, passive
    /// forces (`cb_passive` and passive plugins fire at their end), the
    /// constraint rows' velocities and reference accelerations, the bias
    /// force, velocity sensors and kinetic energy. MuJoCo: `mj_fwdVelocity`
    /// (engine_forward.c:221-259: `mj_passive` at :250,
    /// `mj_referenceConstraint` at :251, `mj_rne` at :254), then
    /// `mj_sensorVel` and `mj_energyVel` in `mj_forwardSkip`. In
    /// [`Pass::Inverse`] the energy is not taken as the drift baseline
    /// ([`Data::energy_initial`] is a forward pass's).
    pub(crate) fn velocity_stage(&mut self, model: &Model, compute_sensors: bool, pass: Pass) {
        velocity::mj_fwd_velocity(model, self);
        actuation::mj_actuator_velocity(model, self);
        passive::mj_fwd_passive(model, self);
        crate::constraint::mj_reference_constraint(model, self);
        crate::dynamics::rne::mj_rne(model, self);
        if compute_sensors {
            crate::sensor::mj_sensor_vel(model, self);
        }
        if enabled(model, ENABLE_ENERGY) {
            crate::energy::mj_energy_vel(model, self);
            if pass == Pass::Forward {
                self.capture_energy_initial();
            }
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
    /// accumulators and acc-sensors. (CRBA runs in the position stage,
    /// passive forces and RNE in the velocity stage.)
    ///
    /// §53: This is the second half of the pipeline, used by both
    /// `forward_core()` and `step2()`.
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError::CholeskyFailed)` if the implicit spring-damper
    /// integrator's factorization fails.
    fn forward_acc(&mut self, model: &Model, compute_sensors: bool) -> Result<(), StepError> {
        // ========== Acceleration Stage ==========
        actuation::mj_fwd_actuation(model, self);

        // S4.2a: Route gravcomp → qfrc_actuator for jnt_actgravcomp joints.
        // qfrc_gravcomp comes from the velocity stage's passive forces.
        actuation::mj_gravcomp_to_actuator(model, self);

        // ImplicitSpringDamper's solves are over every dof; a sleeping dof
        // keeps its acceleration, as the other integrators' solves over the
        // awake dofs leave it.
        let kept = (model.integrator == Integrator::ImplicitSpringDamper
            && acceleration::awake_dofs(model, self))
        .then(|| self.qacc.clone());

        // The solve over the rows the position stage made, Newton's and CG's
        // island by island as MuJoCo's (`constraint/solver/primal.rs`).
        crate::constraint::mj_fwd_constraint(model, self)?;

        // The solver's acceleration is kept (MuJoCo's `mj_fwdConstraint`: the
        // primal iterate, or `dualFinish` after PGS or noslip).
        // Under implicit and implicitfast, `integrate` factors M − h·D and
        // solves for the acceleration it advances qvel with.
        if !self.solver_qacc {
            acceleration::mj_fwd_acceleration(model, self)?;
        }
        if let Some(kept) = kept {
            for dof in 0..model.nv {
                if crate::island::sleep::dof_asleep(model, self, dof) {
                    self.qacc[dof] = kept[dof];
                }
            }
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

        Ok(())
    }

    /// Compare forward and inverse dynamics (§52, `ENABLE_FWDINV`), as
    /// MuJoCo's `mj_compareFwdInv` (`engine_inverse.c:262-302`): with no
    /// constraint rows both statistics are 0; else the inverse from the
    /// acceleration stage, sensors skipped (MuJoCo's
    /// `mj_inverseSkip(mjSTAGE_VEL, 1)`), gives
    /// - `solver_fwdinv[0]`: `‖qfrc_constraint − its inverse‖`, the change
    ///   in the constraint force when it is remade from `qacc`;
    /// - `solver_fwdinv[1]`: `‖qfrc_applied + qfrc_actuator + Jᵀ·xfrc_applied
    ///   − qfrc_inverse‖`.
    ///
    /// The forward's `qfrc_constraint` and `efc_force` are put back;
    /// `efc_state` keeps the inverse's, as MuJoCo's does. With
    /// `ENABLE_INVDISCRETE` so is the implicit integrators' factor, which
    /// MuJoCo's leaves overwritten (registry `D-FWDINV-FACTOR`).
    ///
    /// A step runs it after its forward pass and `mj_check_acc`, as MuJoCo's
    /// `mj_step`, `mj_step2` and `mj_stepSkip` (`engine_forward.c:1431-1434`,
    /// `:1500-1503`; `engine_derivative_fd.c:122-125`); a forward pass alone
    /// does not. With `ENABLE_INVDISCRETE` the inverse is the discrete one
    /// ([`Data::inverse`]), as MuJoCo's: with rows, RK4 and
    /// ImplicitSpringDamper refuse the step there, before the advance
    /// (`check_inverse_discrete`), where MuJoCo's `mj_discreteAcc` stops;
    /// with none, the comparison returns first and the step runs, as
    /// MuJoCo's (`engine_inverse.c:267-270`).
    fn compare_fwd_inv(&mut self, model: &Model) -> Result<(), StepError> {
        self.solver_fwdinv = [0.0, 0.0];
        if self.efc_type.is_empty() {
            return Ok(());
        }
        check::check_inverse_discrete(model)?;

        let qforce = crate::inverse::applied_forces(model, self);

        let qfrc_constraint = self.qfrc_constraint.clone();
        let efc_force = self.efc_force.clone();
        // The discrete inverse writes the implicit integrators' matrix,
        // `M − h·qDeriv`, unfactored, into `qLU` or `qH`, where a step that
        // skips the factor solves (the transition finite differences'
        // control columns): the forward pass's factor is put back. MuJoCo's
        // comparison leaves it written (registry `D-FWDINV-FACTOR`).
        let factor =
            enabled(model, ENABLE_INVDISCRETE).then(|| (self.qLU.clone(), self.qH.clone()));
        self.inverse_acc(model, false);
        if let Some((qlu, qh)) = factor {
            self.qLU = qlu;
            self.qH = qh;
        }

        let dif: Vec<f64> = (0..model.nv)
            .map(|i| qfrc_constraint[i] - self.qfrc_constraint[i])
            .collect();
        self.solver_fwdinv[0] = crate::linalg::mju_norm(&dif);
        let dif: Vec<f64> = (0..model.nv)
            .map(|i| qforce[i] - self.qfrc_inverse[i])
            .collect();
        self.solver_fwdinv[1] = crate::linalg::mju_norm(&dif);

        self.qfrc_constraint = qfrc_constraint;
        self.efc_force = efc_force;
        Ok(())
    }
}
