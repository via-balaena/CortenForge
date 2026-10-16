//! Inverse dynamics computation (§52).
//!
//! Computes the generalized forces that produce the current accelerations:
//! `qfrc_inverse = M * qacc + qfrc_bias - qfrc_passive - qfrc_constraint`.
//!
//! # MuJoCo Equivalence
//!
//! MuJoCo's `mj_inverse` (`mj_inverseSkip`, `engine_inverse.c:184-250`) from its acceleration
//! stage: the constraint force from `qacc` (`mj_invConstraint`), then the
//! sum. It reads the position and velocity stages a forward pass left
//! (MuJoCo's runs them again), and computes no sensor and no energy;
//! `mjENBL_INVDISCRETE` is not read.

use crate::constraint::solver::primal::mj_inv_constraint;
use crate::dynamics::factor::mj_mul_m;
use crate::forward::mj_body_accumulators;
use crate::types::{Data, Model};

impl Data {
    /// Compute inverse dynamics: find forces that produce current accelerations.
    ///
    /// The constraint force is the one `qacc` implies, as MuJoCo's
    /// `mj_invConstraint`: each row's state and force at
    /// `jar = J·qacc − aref`, which overwrites `efc_state`, `efc_force` and
    /// `qfrc_constraint`. Then `qfrc_inverse` is:
    /// ```text
    /// qfrc_inverse = M * qacc + qfrc_bias - qfrc_passive - qfrc_constraint
    /// ```
    ///
    /// At the `qacc` a forward pass solved for, the constraint force is the
    /// solve's (to its tolerance), and `qfrc_inverse` equals
    /// `qfrc_applied + qfrc_actuator + J^T * xfrc_applied`.
    ///
    /// Also calls `mj_body_accumulators()` to populate `cacc`, `cfrc_int`, `cfrc_ext`.
    ///
    /// # Prerequisites
    ///
    /// `forward()` must have been called first to compute `qM`, the
    /// constraint rows (`efc_J`, `efc_aref`, `efc_D`, `efc_R`, …),
    /// `qfrc_bias` and `qfrc_passive`.
    ///
    /// Unlike [`forward`](Self::forward), this does not check the shape of
    /// `self`. It does not read `model.timestep`.
    ///
    /// # Panics
    ///
    /// May panic if an array of `self` is shorter than `model` requires.
    pub fn inverse(&mut self, model: &Model) {
        if model.nv == 0 {
            return;
        }

        mj_inv_constraint(self);

        // Compute body-level accumulators (§51)
        mj_body_accumulators(model, self);

        self.inverse_sum(model);
    }

    /// `qfrc_inverse = qfrc_bias + (M·qacc − qfrc_passive − qfrc_constraint)`,
    /// summed as MuJoCo's `mj_inverseSkip` does, `M·qacc` by `mj_mulM`
    /// (`engine_inverse.c:235-241`).
    pub(crate) fn inverse_sum(&mut self, model: &Model) {
        let ma = mj_mul_m(model, self, self.qacc.as_slice());
        for i in 0..model.nv {
            self.qfrc_inverse[i] =
                self.qfrc_bias[i] + (ma[i] - self.qfrc_passive[i] - self.qfrc_constraint[i]);
        }
    }
}
