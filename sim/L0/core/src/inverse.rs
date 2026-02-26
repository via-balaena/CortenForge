//! Inverse dynamics computation (§52).
//!
//! Computes the generalized forces that produce the current accelerations:
//! `qfrc_inverse = M * qacc + qfrc_bias - qfrc_passive - qfrc_constraint`.
//!
//! All inputs are already computed by `forward()` — this is purely assembly.
//!
//! # MuJoCo Equivalence
//!
//! Matches `mj_inverse()` in `engine_forward.c`.

use crate::forward::mj_body_accumulators;
use crate::types::{Data, Model};

impl Data {
    /// Compute inverse dynamics: find forces that produce current accelerations.
    ///
    /// Populates `data.qfrc_inverse` with:
    /// ```text
    /// qfrc_inverse = M * qacc + qfrc_bias - qfrc_passive - qfrc_constraint
    /// ```
    ///
    /// After a successful forward/inverse round-trip, `qfrc_inverse` equals
    /// `qfrc_applied + qfrc_actuator + J^T * xfrc_applied` — the total
    /// user-supplied generalized forces.
    ///
    /// Also calls `mj_body_accumulators()` to populate `cacc`, `cfrc_int`, `cfrc_ext`.
    ///
    /// # Prerequisites
    ///
    /// `forward()` must have been called first to compute `qM`, `qacc`,
    /// `qfrc_bias`, `qfrc_passive`, and `qfrc_constraint`.
    pub fn inverse(&mut self, model: &Model) {
        if model.nv == 0 {
            return;
        }

        // Compute body-level accumulators (§51)
        mj_body_accumulators(model, self);

        // qfrc_inverse = M * qacc + qfrc_bias - qfrc_passive - qfrc_constraint
        self.qM.mul_to(&self.qacc, &mut self.qfrc_inverse);
        self.qfrc_inverse += &self.qfrc_bias;
        self.qfrc_inverse -= &self.qfrc_passive;
        self.qfrc_inverse -= &self.qfrc_constraint;
    }
}
