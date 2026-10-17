//! Inverse dynamics computation (§52).
//!
//! Computes the generalized forces that produce the current accelerations:
//! `qfrc_inverse = M * qacc + qfrc_bias - qfrc_passive - qfrc_constraint`.
//!
//! # MuJoCo Equivalence
//!
//! MuJoCo's `mj_inverse` (`mj_inverseSkip`, `engine_inverse.c:184-250`): its
//! own position and velocity stages (`Pass::Inverse`), sensors and
//! energy, the constraint force from `qacc` (`mj_invConstraint`), then the
//! sum; with `mjENBL_INVDISCRETE`, at the discrete-time acceleration
//! (`mj_discreteAcc`, `:76-153`). A sleeping dof's `qfrc_inverse` is the
//! sum over its kept bias force, where MuJoCo's adds to the value its last
//! call left (registry `D-INVERSE-SLEEP`).

use crate::constraint::solver::primal::mj_inv_constraint;
use crate::derivatives::mjd_smooth_vel_bias;
use crate::dynamics::factor::{mj_mul_m, mj_solve_m, mul_sym_on_m_pattern};
use crate::forward::acceleration::{implicit_fast_matrix, implicit_matrix};
use crate::forward::{MjStage, Pass, check, mj_body_accumulators};
use crate::linalg::dot_sparse;
use crate::types::flags::{disabled, enabled};
use crate::types::{DISABLE_EULERDAMP, Data, ENABLE_INVDISCRETE, Integrator, Model, StepError};
use nalgebra::DVector;

impl Data {
    /// Compute inverse dynamics: find forces that produce current accelerations.
    ///
    /// As MuJoCo's `mj_inverse`, from `qpos`, `qvel` and `qacc`: the
    /// position and velocity stages (the forward pass's,
    /// but for its tendon, contact and equality wakes and its islands),
    /// with their sensors and energy; then the constraint force `qacc`
    /// implies, as MuJoCo's `mj_invConstraint`: each row's state and force
    /// at `jar = J·qacc − aref`, which overwrites `efc_state`, `efc_force`
    /// and `qfrc_constraint`; the body accumulators (`cacc`, `cfrc_int`,
    /// `cfrc_ext`) and the acceleration sensors from them; then
    /// `qfrc_inverse`:
    /// ```text
    /// qfrc_inverse = M * qacc + qfrc_bias - qfrc_passive - qfrc_constraint
    /// ```
    /// `cb_passive` fires in the velocity stage; `cb_control` does not fire,
    /// and the actuation stage does not run (`qfrc_actuator` is the last
    /// forward pass's). `qacc` is left as it is.
    ///
    /// With `ENABLE_INVDISCRETE`, as MuJoCo's: `qacc` is taken as the
    /// acceleration the integrator's discrete step applies, and turned into
    /// the continuous-time one first (`mj_discreteAcc`,
    /// `engine_inverse.c:76-153`): under Euler, with eulerdamp on and a dof
    /// damped, `M⁻¹·(M + h·diag(dof_damping))·qacc`, else unchanged; under
    /// implicit, `M⁻¹·(M − h·qDeriv)·qacc` with the RNE's velocity
    /// derivative; under implicitfast, without it. The constraint force,
    /// the acceleration sensors and the sum are taken at it, and `qacc` is
    /// put back after. It leaves `qDeriv` and the implicit integrator's
    /// matrix (`qLU`, or `qH` under implicitfast) as MuJoCo's does.
    ///
    /// At the `qacc` a forward pass solved for, the constraint force is the
    /// solve's (to its tolerance), and `qfrc_inverse` equals
    /// `qfrc_applied + qfrc_actuator + J^T * xfrc_applied`, but under
    /// `ImplicitSpringDamper`, whose springs and dampers the step applies
    /// implicitly, outside `qfrc_passive`. At a sleeping dof it does too,
    /// call after call, where MuJoCo's adds to its last value (registry
    /// `D-INVERSE-SLEEP`).
    ///
    /// # Errors
    ///
    /// Before any work, the input check [`step`](Self::step) makes:
    /// [`StepError::InvalidTimestep`] if the timestep is not positive and
    /// finite, [`StepError::DataShapeMismatch`] if `self` was made by a model
    /// of other dimensions or an input array was resized;
    /// [`StepError::UnsupportedInverseDiscrete`] with `ENABLE_INVDISCRETE`
    /// under RK4 (MuJoCo's refusal) or ImplicitSpringDamper. After collision,
    /// with sleep enabled, [`StepError::ContactWithSleepingGeom`] for a
    /// contact that touches a sleeping tree, which this position stage does
    /// not wake, as MuJoCo's `mj_inverse` stops there; a forward pass at the
    /// state first wakes it.
    pub fn inverse(&mut self, model: &Model) -> Result<(), StepError> {
        self.inverse_skip(model, MjStage::None, false)
    }

    /// [`inverse`](Self::inverse) after `skipstage`, as MuJoCo's
    /// `mj_inverseSkip` (`engine_inverse.c:184-250`): with [`MjStage::Pos`]
    /// the position stage is not run, with [`MjStage::Vel`] neither is the
    /// velocity stage, and their outputs in `self` are used as they are;
    /// with `skipsensor` no sensor and no body accumulator is computed.
    ///
    /// # Errors
    ///
    /// [`inverse`](Self::inverse)'s.
    pub fn inverse_skip(
        &mut self,
        model: &Model,
        skipstage: MjStage,
        skipsensor: bool,
    ) -> Result<(), StepError> {
        check::check_step_inputs(model, self)?;
        check::check_inverse_discrete(model)?;
        self.inverse_skip_unchecked(model, skipstage, skipsensor)
    }

    /// [`Self::inverse_skip`] without its input checks, for the inverse
    /// finite differences, whose caller made them.
    pub(crate) fn inverse_skip_unchecked(
        &mut self,
        model: &Model,
        skipstage: MjStage,
        skipsensor: bool,
    ) -> Result<(), StepError> {
        let compute_sensors = !skipsensor;
        if skipstage < MjStage::Pos {
            self.position_stage(model, compute_sensors, Pass::Inverse)?;
        }
        if skipstage < MjStage::Vel {
            self.velocity_stage(model, compute_sensors, Pass::Inverse);
        }
        self.inverse_acc(model, compute_sensors);
        Ok(())
    }

    /// The inverse dynamics after its velocity stage (`mj_inverseSkip`,
    /// `engine_inverse.c:213-246`): with `ENABLE_INVDISCRETE` the
    /// discrete-time acceleration in `qacc` ([`discrete_acc`]); the
    /// constraint force from `qacc`; with `compute_sensors` the body
    /// accumulators and the acceleration sensors; the sum; then `qacc` put
    /// back. The caller refused an integrator `discrete_acc` does not take
    /// ([`check::check_inverse_discrete`]).
    pub(crate) fn inverse_acc(&mut self, model: &Model, compute_sensors: bool) {
        let saved = enabled(model, ENABLE_INVDISCRETE).then(|| {
            let qacc = self.qacc.clone();
            discrete_acc(model, self);
            qacc
        });

        mj_inv_constraint(self);

        if compute_sensors {
            mj_body_accumulators(model, self);
            crate::sensor::mj_sensor_acc(model, self);
            crate::sensor::mj_sensor_postprocess(model, self);
        }

        self.inverse_sum(model);

        if let Some(qacc) = saved {
            self.qacc = qacc;
        }
    }

    /// `qfrc_inverse = qfrc_bias + (M·qacc − qfrc_passive − qfrc_constraint)`,
    /// summed as MuJoCo's `mj_inverseSkip` does, `M·qacc` by `mj_mulM`
    /// (`engine_inverse.c:235-241`). MuJoCo's starts from `mj_rne`'s bias
    /// written into `qfrc_inverse` at the awake dofs only (`mj_rne`,
    /// `engine_core_smooth.c:2481-2484`); this one from `qfrc_bias`, which
    /// keeps a sleeping dof's.
    pub(crate) fn inverse_sum(&mut self, model: &Model) {
        let ma = mj_mul_m(model, self, self.qacc.as_slice());
        for i in 0..model.nv {
            self.qfrc_inverse[i] =
                self.qfrc_bias[i] + (ma[i] - self.qfrc_passive[i] - self.qfrc_constraint[i]);
        }
    }
}

/// `qacc` from the discrete-time acceleration to the continuous-time one,
/// as MuJoCo's `mj_discreteAcc` (`engine_inverse.c:76-153`): `qfrc = M̂·qacc`
/// with the integrator's matrix `M̂`, then `qacc = M⁻¹·qfrc` by the mass
/// matrix's factor (`mj_solveM`). Euler's is `M + h·diag(dof_damping)`
/// (`mj_mulM`, then each dof's `h·damping·qacc` added), and Euler leaves
/// `qacc` as it is with eulerdamp disabled or no dof damped positively;
/// implicit's is `qLU = M − h·qDeriv` on D's pattern, the RNE's derivative in
/// `qDeriv`, multiplied row by row (`mju_mulMatVecSparse`); implicitfast's
/// `qH = M − h·qDeriv` on M's pattern without it, multiplied as the
/// symmetric `mju_mulSymVecSparse`. RK4 and ImplicitSpringDamper are refused
/// before ([`check::check_inverse_discrete`]) and leave it as it is.
fn discrete_acc(model: &Model, data: &mut Data) {
    let nv = model.nv;
    let h = model.timestep;
    let mut qfrc = DVector::zeros(nv);
    match model.integrator {
        Integrator::Euler => {
            if disabled(model, DISABLE_EULERDAMP) || !model.dof_damping.iter().any(|&b| b > 0.0) {
                return;
            }
            qfrc = mj_mul_m(model, data, data.qacc.as_slice());
            for i in 0..nv {
                qfrc[i] += h * model.dof_damping[i] * data.qacc[i];
            }
        }
        Integrator::Implicit => {
            mjd_smooth_vel_bias(model, data, true);
            implicit_matrix(model, data);
            for i in 0..nv {
                let adr = model.D_rowadr[i];
                let row = &data.qLU[adr..adr + model.D_rownnz[i]];
                qfrc[i] = dot_sparse(row, |k| data.qacc[model.D_colind[adr + k]]);
            }
        }
        Integrator::ImplicitFast => {
            mjd_smooth_vel_bias(model, data, false);
            implicit_fast_matrix(model, data);
            mul_sym_on_m_pattern(
                model,
                |adr, _, _| data.qH[adr],
                data.qacc.as_slice(),
                qfrc.as_mut_slice(),
            );
        }
        Integrator::RungeKutta4 | Integrator::ImplicitSpringDamper => return,
    }
    mj_solve_m(model, data, qfrc.as_mut_slice());
    data.qacc.copy_from(&qfrc);
}
