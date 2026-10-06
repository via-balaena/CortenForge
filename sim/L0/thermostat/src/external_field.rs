//! Linear external field for symmetry-breaking in coupled bistable arrays.
//!
//! Implements the linear potential `V = −Σ h_i · x_i` as a
//! [`PassiveComponent`] that contributes a constant force `F_i = h_i` to
//! each DOF. This breaks the symmetry of the double-well potential,
//! biasing elements toward the right well (`h > 0`) or left well
//! (`h < 0`). The continuous analogue of the Ising external field.
//!
//! Phase 5 of the thermodynamic computing initiative uses this component
//! together with [`PairwiseCoupling`] to build a trainable Ising-like
//! system via the Boltzmann machine learning rule.
//!
//! [`PassiveComponent`]: crate::PassiveComponent
//! [`PairwiseCoupling`]: crate::PairwiseCoupling

use sim_core::{DVector, Data, Model};

use crate::component::{PassiveComponent, check_dof, check_position_dof, qpos_index};
use crate::diagnose::Diagnose;
use crate::error::ThermostatError;

/// Linear external field: `V = −Σ h_i · x_i`.
///
/// Contributes a constant force `F_i = h_i` to each DOF. This is a
/// deterministic conservative force — it does not implement
/// [`Stochastic`](crate::Stochastic).
///
/// # Joint type constraint
///
/// Same as [`DoubleWellPotential`](crate::DoubleWellPotential) and
/// [`PairwiseCoupling`](crate::PairwiseCoupling): only correct for slide
/// and hinge joints where `nq = nv = 1`.
pub struct ExternalField {
    /// Per-DOF field strengths.
    field_h: Vec<f64>,
}

impl ExternalField {
    /// Create an external field with per-DOF field strengths.
    #[must_use]
    pub const fn new(field_h: Vec<f64>) -> Self {
        Self { field_h }
    }

    /// Field strengths (read-only).
    #[must_use]
    pub fn field_h(&self) -> &[f64] {
        &self.field_h
    }

    /// Field energy at `data`'s positions: `V = −Σ h_i · x_i`, where `x_i` is DOF `i`'s
    /// position.
    ///
    /// # Errors
    ///
    /// Returns [`ThermostatError::DofOutOfRange`] or [`ThermostatError::NoPositionCoordinate`]
    /// if a DOF the field covers is missing from `model` or has no single position
    /// coordinate.
    pub fn field_energy(&self, model: &Model, data: &Data) -> Result<f64, ThermostatError> {
        for i in 0..self.field_h.len() {
            check_position_dof(model, i, "ExternalField")?;
        }
        Ok(self
            .field_h
            .iter()
            .enumerate()
            .map(|(i, &h)| -h * data.qpos[qpos_index(model, i)])
            .sum())
    }
}

impl PassiveComponent for ExternalField {
    fn apply(&self, _model: &Model, _data: &Data, qfrc_out: &mut DVector<f64>) {
        for (i, &h) in self.field_h.iter().enumerate() {
            qfrc_out[i] += h;
        }
    }

    fn as_diagnose(&self) -> Option<&dyn Diagnose> {
        Some(self)
    }

    /// Accepts a field shorter than the model's DOF count: it acts on the first DOFs only.
    fn validate(&self, model: &Model) -> Result<(), ThermostatError> {
        match self.field_h.len() {
            0 => Ok(()),
            len => check_dof(model, len - 1, "ExternalField"),
        }
    }
}

impl Diagnose for ExternalField {
    fn diagnostic_summary(&self) -> String {
        let h_min = self.field_h.iter().copied().fold(f64::INFINITY, f64::min);
        let h_max = self
            .field_h
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        format!(
            "ExternalField(n={}, h_range=[{h_min:.4}, {h_max:.4}])",
            self.field_h.len()
        )
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    /// An `n`-slide chain at positions `x`.
    fn chain_at(x: &[f64]) -> (Model, Data) {
        let model = sim_core::test_fixtures::bistable_chain(x.len());
        let mut data = model.make_data();
        for (i, &xi) in x.iter().enumerate() {
            data.qpos[i] = xi;
        }
        (model, data)
    }

    #[test]
    fn new_creates_field() {
        let f = ExternalField::new(vec![0.3, -0.2, 0.0, 0.15]);
        assert_eq!(f.field_h().len(), 4);
        assert_eq!(f.field_h()[0], 0.3);
        assert_eq!(f.field_h()[1], -0.2);
    }

    #[test]
    fn field_energy_computation() {
        let f = ExternalField::new(vec![0.5, -0.3]);
        let (model, data) = chain_at(&[1.0, 1.0]);
        // V = -0.5*1.0 + -(-0.3)*1.0 = -0.5 + 0.3 = -0.2
        let energy = f.field_energy(&model, &data).unwrap();
        assert!(
            (energy - (-0.2)).abs() < 1e-15,
            "expected -0.2, got {energy}"
        );
    }

    #[test]
    fn field_energy_zero_field() {
        let f = ExternalField::new(vec![0.0, 0.0]);
        let (model, data) = chain_at(&[5.0, -3.0]);
        assert_eq!(f.field_energy(&model, &data).unwrap(), 0.0);
    }

    #[test]
    fn apply_adds_constant_force() {
        // Verify via finite-difference check on field_energy
        let f = ExternalField::new(vec![0.7, -0.4]);
        let eps = 1e-8;

        for dof in 0..2 {
            let mut x_plus = [0.0, 0.0];
            let mut x_minus = [0.0, 0.0];
            x_plus[dof] += eps;
            x_minus[dof] -= eps;
            let (model, plus) = chain_at(&x_plus);
            let (_, minus) = chain_at(&x_minus);
            let force_fd = -(f.field_energy(&model, &plus).unwrap()
                - f.field_energy(&model, &minus).unwrap())
                / (2.0 * eps);
            assert!(
                (force_fd - f.field_h()[dof]).abs() < 1e-6,
                "DOF {dof}: expected F={}, got {force_fd}",
                f.field_h()[dof]
            );
        }
    }

    #[test]
    fn diagnostic_summary_format() {
        let f = ExternalField::new(vec![0.3, -0.2, 0.0, 0.15]);
        let s = f.diagnostic_summary();
        assert!(s.contains("ExternalField"));
        assert!(s.contains("-0.2000"));
        assert!(s.contains("0.3000"));
        assert!(s.contains("n=4"));
    }

    /// The energy is the plain sum `Σ −h_i·x_i`, bit for bit; a fused multiply-add differs on
    /// these inputs (found with Python's math.fma).
    #[test]
    fn field_energy_is_the_plain_sum() {
        let h = [-0.567, -0.156, -0.942];
        let x = [-1.113, -0.248, -0.017];
        let f = ExternalField::new(h.to_vec());
        let (model, data) = chain_at(&x);
        let plain = (-h[0] * x[0]) + (-h[1] * x[1]) + (-h[2] * x[2]);
        assert_eq!(
            f.field_energy(&model, &data).unwrap().to_bits(),
            plain.to_bits()
        );
    }
}
