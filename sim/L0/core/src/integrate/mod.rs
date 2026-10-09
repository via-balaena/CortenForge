//! Integration dispatch — Euler, implicit tendon K/D, and RK4.
//!
//! Corresponds to MuJoCo's `engine_forward.c` integration section:
//! `mj_Euler`, `mj_RungeKutta`, and implicit spring/damper helpers.
//!
//! - `euler`: Position integration on SO(3) manifold
//! - `implicit`: Tendon implicit stiffness/damping helpers (K/D accumulation)
//! - `rk4`: Standard 4-stage Runge-Kutta integration

pub(crate) mod euler;
pub(crate) mod implicit;
pub(crate) mod rk4;

use crate::dynamics::factor::mj_factor_sparse;
use crate::forward::{MjStage, check, mj_next_activation};
use crate::linalg::mj_solve_sparse;
use crate::types::flags::{actuator_disabled, disabled};
use crate::types::{
    DISABLE_ACTUATION, DISABLE_DAMPER, DISABLE_EULERDAMP, Data, ENABLE_SLEEP, Integrator, Model,
    StepError,
};
use nalgebra::DVector;

use euler::mj_integrate_pos;

/// Whether an Euler step solves `(M + h·D)·qacc_new = qfrc_smooth +
/// qfrc_constraint` for the acceleration it advances `qvel` with (eulerdamp):
/// neither eulerdamp nor dampers disabled, and some awake DOF damped
/// positively, as MuJoCo's `mj_Euler` (`engine_forward.c:956-963`; an
/// undamped model skips the refactorisation).
pub(crate) fn eulerdamp_applies(model: &Model, data: &Data) -> bool {
    if model.disableflags & (DISABLE_EULERDAMP | DISABLE_DAMPER) != 0 {
        return false;
    }
    if model.enableflags & ENABLE_SLEEP != 0 && data.nv_awake < model.nv {
        data.dof_awake_ind[..data.nv_awake]
            .iter()
            .any(|&i| model.implicit_damping[i] > 0.0)
    } else {
        model.implicit_damping.iter().any(|&d| d > 0.0)
    }
}

impl Data {
    /// Advance the state by one timestep after the acceleration stage, as
    /// MuJoCo's `mj_advance` (`engine_forward.c:833-939`): the history
    /// samples, the activations, the sleep step, then velocity, position and
    /// time, the plugins, and `qacc_warmstart = qacc`.
    ///
    /// This is exposed as part of the split-step API ([`step1`](Self::step1) /
    /// [`step2`](Self::step2)). Under RK4, [`step`](Self::step) integrates with
    /// `mj_runge_kutta()` instead; this method takes the Euler step, as
    /// MuJoCo's `mj_step2` does.
    ///
    /// # Sleep
    ///
    /// With sleep enabled, sleep is decided here, after the activations and
    /// before the velocity update, from the velocities the step started
    /// with. On a step that puts a tree to sleep, its velocity and
    /// acceleration are zeroed and the forward pass runs again from the
    /// velocity stage, sensors and both callbacks included, before the awake
    /// trees advance with the acceleration computed before that pass.
    ///
    /// # Integration Methods
    ///
    /// - **Euler**: Semi-implicit Euler. Updates velocity first (`qvel += qacc * h`,
    ///   or with a damped DOF eulerdamp's `(M + h·D)⁻¹ (qfrc_smooth +
    ///   qfrc_constraint)` in place of `qacc`), then integrates position using the
    ///   new velocity.
    ///
    /// - **Implicit, ImplicitFast**: `qvel += qacc_implicit * h`, the acceleration
    ///   `(M − h·∂f/∂v)⁻¹ f` the acceleration stage solved for; `qacc` keeps the
    ///   explicit one, as in MuJoCo.
    ///
    /// - **ImplicitSpringDamper**: `qvel += qacc * h`, where `qacc` is the
    ///   implicit acceleration the acceleration stage (or a Newton solve)
    ///   computed.
    ///
    /// # Errors
    ///
    /// `StepError::InvalidTimestep` if the timestep is not positive and
    /// finite and `StepError::DataShapeMismatch` if `self` was made by a
    /// model of other dimensions or an input array was resized, both before
    /// anything changes; and the error of the forward pass a sleep step runs
    /// (an implicit factorization), which leaves the history samples inserted,
    /// the activations advanced, the slept trees asleep with their velocities
    /// zeroed but the sleep arrays not updated for them, and positions and
    /// time not advanced. MuJoCo's forward pass has no failure there.
    pub fn integrate(&mut self, model: &Model) -> Result<(), StepError> {
        check::check_step_inputs(model, self)?;
        self.integrate_unchecked(model)
    }

    /// [`Self::integrate`] without its input checks, for `step` and `step2`,
    /// which make them first.
    pub(crate) fn integrate_unchecked(&mut self, model: &Model) -> Result<(), StepError> {
        // History first, at the step's time, as MuJoCo's `mj_advance`
        // (`engine_forward.c:837-884`): each buffered actuator's `ctrl`, then
        // each buffered sensor's sample.
        crate::history::advance_ctrl(model, self, self.time);
        crate::history::advance_sensors(model, self);

        let h = model.timestep;
        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

        // Integrate activation per actuator via mj_next_activation() (§34).
        // Handles both integration (Euler/FilterExact) and actlimited clamping.
        // MuJoCo order: activation → sleep → velocity → position.
        // S4.8: Per-actuator disable gating — disabled actuators get act_dot=0,
        // freezing activation state without zeroing it.
        for i in 0..model.nu {
            let act_adr = model.actuator_act_adr[i];
            let act_num = model.actuator_act_num[i];
            let is_disabled = disabled(model, DISABLE_ACTUATION) || actuator_disabled(model, i);
            for k in 0..act_num {
                let j = act_adr + k;
                let act_dot_val = if is_disabled { 0.0 } else { self.act_dot[j] };
                self.act[j] = mj_next_activation(model, i, self.act[j], act_dot_val);
            }
        }

        // The acceleration the velocities advance with, computed before the
        // sleep step as MuJoCo's integrators compute theirs before
        // `mj_advance`: eulerdamp's solve under Euler (and under RK4, which
        // `step2` takes as Euler) with a damped dof; else the live array.
        let mut solved = (matches!(
            model.integrator,
            Integrator::Euler | Integrator::RungeKutta4
        ) && eulerdamp_applies(model, self))
        .then(|| self.eulerdamp_acceleration(model));
        if crate::island::mj_sleep(model, self) > 0 {
            // The re-forward overwrites `qacc` and `qacc_implicit`.
            if solved.is_none() {
                solved = Some(match model.integrator {
                    Integrator::Implicit | Integrator::ImplicitFast => self.qacc_implicit.clone(),
                    _ => self.qacc.clone(),
                });
            }
            self.reforward_after_sleep(model)?;
        }
        let acc = match (&solved, model.integrator) {
            (Some(acc), _) => acc,
            (None, Integrator::Implicit | Integrator::ImplicitFast) => &self.qacc_implicit,
            (None, _) => &self.qacc,
        };

        // §16.27: Use indirection array for cache-friendly iteration over awake DOFs.
        let use_dof_ind = sleep_enabled && self.nv_awake < model.nv;
        let nv = if use_dof_ind { self.nv_awake } else { model.nv };
        for idx in 0..nv {
            let i = if use_dof_ind {
                self.dof_awake_ind[idx]
            } else {
                idx
            };
            self.qvel[i] += acc[i] * h;
        }

        // Positions; a quaternion is normalized before it turns, as MuJoCo's
        // `mju_quatIntegrate` does, and not after.
        mj_integrate_pos(model, self, h);

        // Advance time
        self.time += h;

        advance_plugins(model, self);

        // Save qacc for the next step's warmstart (§15.9), as `mj_advance`
        // ends (engine_forward.c:938).
        self.qacc_warmstart.copy_from(&self.qacc);
        Ok(())
    }

    /// Eulerdamp's acceleration: MuJoCo 3.x solves
    /// `(M + h·D)·qacc_new = qfrc_smooth + qfrc_constraint` (`mj_EulerSkip`),
    /// which handles the mass matrix's off-diagonal coupling in multi-DOF
    /// systems. `qM` and its factorization are restored afterwards.
    fn eulerdamp_acceleration(&mut self, model: &Model) -> DVector<f64> {
        let h = model.timestep;
        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
        let use_dof_ind = sleep_enabled && self.nv_awake < model.nv;

        // Save original factorization (restored after solve)
        let saved_qld = self.qLD_data.clone();
        let saved_inv = self.qLD_diag_inv.clone();

        // Add h·damp to the mass matrix diagonal, every DOF's whatever its
        // sign (`engine_forward.c:986-989`), then refactorize
        for i in 0..model.nv {
            self.qM[(i, i)] += h * model.implicit_damping[i];
        }
        mj_factor_sparse(model, self);

        // RHS = total force = qfrc_smooth + qfrc_constraint
        let mut rhs = DVector::zeros(model.nv);
        let nv = if use_dof_ind { self.nv_awake } else { model.nv };
        for idx in 0..nv {
            let i = if use_dof_ind {
                self.dof_awake_ind[idx]
            } else {
                idx
            };
            rhs[i] = self.qfrc_smooth[i] + self.qfrc_constraint[i];
        }

        // Solve: (M + h·D) · qacc_new = rhs
        let (rowadr, rownnz, colind) = model.qld_csr();
        mj_solve_sparse(
            rowadr,
            rownnz,
            colind,
            &self.qLD_data,
            &self.qLD_diag_inv,
            &mut rhs,
        );

        // Restore original mass matrix diagonal and factorization
        for i in 0..model.nv {
            self.qM[(i, i)] -= h * model.implicit_damping[i];
        }
        self.qLD_data = saved_qld;
        self.qLD_diag_inv = saved_inv;
        rhs
    }

    /// The rest of `mj_advance`'s sleep block (`engine_forward.c:898-905`),
    /// after `mj_sleep` put a tree to sleep: the forward pass from the
    /// velocity stage, sensors and `cb_control` included, while the sleep
    /// arrays still list the slept trees as awake, so their velocity- and
    /// acceleration-stage quantities are computed at the zeroed velocity;
    /// then the sleep arrays.
    pub(crate) fn reforward_after_sleep(&mut self, model: &Model) -> Result<(), StepError> {
        self.forward_skip_unchecked(model, MjStage::Pos, false)?;
        crate::island::mj_update_sleep_arrays(model, self);
        Ok(())
    }
}

/// §66: advance every plugin instance's state, after the state and time
/// advance, as MuJoCo 3.5.0's `mj_advance` does (`engine_forward.c:923-936`),
/// which its Euler, implicit and RK4 integrators all end in.
pub(crate) fn advance_plugins(model: &Model, data: &mut Data) {
    for i in 0..model.nplugin {
        model.plugin_objects[i].advance(model, data, i);
    }
}
