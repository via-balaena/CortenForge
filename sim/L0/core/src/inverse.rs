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
//! sum. `mjENBL_INVDISCRETE` is not read. A sleeping dof's `qfrc_inverse`
//! is the sum over its kept bias force, where MuJoCo's adds to the value
//! its last call left (registry `D-INVERSE-SLEEP`).

use crate::constraint::solver::primal::mj_inv_constraint;
use crate::dynamics::factor::mj_mul_m;
use crate::forward::{MjStage, Pass, check, mj_body_accumulators};
use crate::types::{Data, Model, StepError};

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
    /// of other dimensions or an input array was resized. After collision,
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
    /// `engine_inverse.c:213-246`): the constraint force from `qacc`; with
    /// `compute_sensors` the body accumulators and the acceleration sensors;
    /// then the sum.
    pub(crate) fn inverse_acc(&mut self, model: &Model, compute_sensors: bool) {
        mj_inv_constraint(self);

        if compute_sensors {
            mj_body_accumulators(model, self);
            crate::sensor::mj_sensor_acc(model, self);
            crate::sensor::mj_sensor_postprocess(model, self);
        }

        self.inverse_sum(model);
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
