//! Bilinear pairwise coupling for coupled bistable arrays.
//!
//! Implements the coupling potential `V = −J · Σ_{(i,j) ∈ edges} xᵢ · xⱼ`
//! as a [`PassiveComponent`] that contributes conservative forces to the
//! `qfrc_passive` accumulator. Combined with [`DoubleWellPotential`]
//! instances and a [`LangevinThermostat`] in a [`PassiveStack`], this
//! produces a coupled bistable system whose equilibrium statistics match
//! the Ising model on the same coupling topology.
//!
//! Phase 4 of the thermodynamic computing initiative validates this
//! component against exact Ising predictions on a 4-element chain.
//!
//! [`PassiveComponent`]: crate::PassiveComponent
//! [`DoubleWellPotential`]: crate::DoubleWellPotential
//! [`LangevinThermostat`]: crate::LangevinThermostat
//! [`PassiveStack`]: crate::PassiveStack

use sim_core::{DVector, Data, Model};

use crate::component::PassiveComponent;
use crate::diagnose::Diagnose;

/// Bilinear pairwise coupling: `V = −J · Σ_{(i,j) ∈ edges} xᵢ · xⱼ`.
///
/// Contributes force `F_i = +J · Σ_{j: (i,j) ∈ edges} xⱼ` to the per-DOF
/// force accumulator. Ferromagnetic for `J > 0` (aligned positions are
/// energetically favored). This is the natural continuous analogue of the
/// Ising model's `σᵢσⱼ` coupling.
///
/// Not stochastic — this is a deterministic conservative force.
///
/// # Joint type constraint
///
/// The `edges` field contains pairs of DOF indices used to access both
/// `data.qpos` and `qfrc_out`. This is correct for slide and hinge joints
/// where `nq = nv = 1` (DOF index = qpos index). It does **not** support
/// ball (`nq=4, nv=3`) or free (`nq=7, nv=6`) joints where these indices
/// diverge.
///
/// # Example
///
/// ```ignore
/// use sim_thermostat::{DoubleWellPotential, LangevinThermostat, PairwiseCoupling, PassiveStack};
/// use sim_core::DVector;
///
/// let mut builder = PassiveStack::builder();
/// for i in 0..4 {
///     builder = builder.with(DoubleWellPotential::new(3.0, 1.0, i));
/// }
/// builder = builder.with(PairwiseCoupling::chain(4, 0.5));
/// builder = builder.with(LangevinThermostat::new(
///     DVector::from_element(4, 10.0), 1.0, 42,
/// ));
/// builder.build().install(&mut model);
/// ```
pub struct PairwiseCoupling {
    /// Coupling constant `J`. Positive = ferromagnetic.
    coupling_j: f64,
    /// List of DOF pairs that are coupled.
    edges: Vec<(usize, usize)>,
}

impl PairwiseCoupling {
    /// Create a coupling with explicit edge list.
    ///
    /// # Panics
    /// Panics if any edge has `i == j` (self-coupling).
    #[must_use]
    pub fn new(coupling_j: f64, edges: Vec<(usize, usize)>) -> Self {
        for &(i, j) in &edges {
            assert!(i != j, "self-coupling not supported: edge ({i}, {j})");
        }
        Self { coupling_j, edges }
    }

    /// Create a nearest-neighbor open chain:
    /// edges `[(0,1), (1,2), ..., (n−2, n−1)]`.
    ///
    /// # Panics
    /// Panics if `n < 2`.
    #[must_use]
    pub fn chain(n: usize, coupling_j: f64) -> Self {
        assert!(n >= 2, "chain requires at least 2 elements, got {n}");
        let edges = (0..n - 1).map(|i| (i, i + 1)).collect();
        Self::new(coupling_j, edges)
    }

    /// Create a nearest-neighbor ring: chain + closing edge `(n−1, 0)`.
    ///
    /// # Panics
    /// Panics if `n < 3`.
    #[must_use]
    pub fn ring(n: usize, coupling_j: f64) -> Self {
        assert!(n >= 3, "ring requires at least 3 elements, got {n}");
        let mut edges: Vec<(usize, usize)> = (0..n - 1).map(|i| (i, i + 1)).collect();
        edges.push((n - 1, 0));
        Self::new(coupling_j, edges)
    }

    /// Coupling constant `J`.
    #[must_use]
    pub const fn coupling_j(&self) -> f64 {
        self.coupling_j
    }

    /// Edge list (read-only).
    #[must_use]
    pub fn edges(&self) -> &[(usize, usize)] {
        &self.edges
    }

    /// Total coupling energy: `V = −J · Σ xᵢ · xⱼ`.
    #[must_use]
    pub fn coupling_energy(&self, qpos: &DVector<f64>) -> f64 {
        self.edges
            .iter()
            .map(|&(i, j)| -self.coupling_j * qpos[i] * qpos[j])
            .sum()
    }
}

impl PassiveComponent for PairwiseCoupling {
    fn apply(&self, _model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        for &(i, j) in &self.edges {
            let xi = data.qpos[i];
            let xj = data.qpos[j];
            // V = −J · xi · xj  →  F_i = +J · xj,  F_j = +J · xi
            qfrc_out[i] += self.coupling_j * xj;
            qfrc_out[j] += self.coupling_j * xi;
        }
    }
}

impl Diagnose for PairwiseCoupling {
    fn diagnostic_summary(&self) -> String {
        format!(
            "PairwiseCoupling(J={:.4}, edges={})",
            self.coupling_j,
            self.edges.len()
        )
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    #[should_panic(expected = "self-coupling not supported")]
    fn new_rejects_self_coupling() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = PairwiseCoupling::new(1.0, vec![(0, 0)]);
    }

    #[test]
    fn chain_produces_correct_edges() {
        let c = PairwiseCoupling::chain(4, 0.5);
        assert_eq!(c.edges(), &[(0, 1), (1, 2), (2, 3)]);
        assert_eq!(c.coupling_j(), 0.5);
    }

    #[test]
    fn chain_2_produces_single_edge() {
        let c = PairwiseCoupling::chain(2, 1.0);
        assert_eq!(c.edges(), &[(0, 1)]);
    }

    #[test]
    #[should_panic(expected = "chain requires at least 2")]
    fn chain_1_panics() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = PairwiseCoupling::chain(1, 1.0);
    }

    #[test]
    fn ring_produces_correct_edges() {
        let r = PairwiseCoupling::ring(4, 0.5);
        assert_eq!(r.edges(), &[(0, 1), (1, 2), (2, 3), (3, 0)]);
    }

    #[test]
    #[should_panic(expected = "ring requires at least 3")]
    fn ring_2_panics() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = PairwiseCoupling::ring(2, 1.0);
    }

    #[test]
    fn coupling_energy_all_aligned() {
        // 4-chain, all at +1: V = -J(1·1 + 1·1 + 1·1) = -3J
        let c = PairwiseCoupling::chain(4, 0.5);
        let qpos = DVector::from_element(4, 1.0);
        let energy = c.coupling_energy(&qpos);
        assert!(
            (energy - (-1.5)).abs() < 1e-15,
            "expected -1.5, got {energy}"
        );
    }

    #[test]
    fn coupling_energy_alternating() {
        // 4-chain, alternating +1/-1: V = -J(-1 + -1 + -1) = +3J
        let c = PairwiseCoupling::chain(4, 0.5);
        let qpos = DVector::from_vec(vec![1.0, -1.0, 1.0, -1.0]);
        let energy = c.coupling_energy(&qpos);
        assert!((energy - 1.5).abs() < 1e-15, "expected 1.5, got {energy}");
    }

    #[test]
    fn force_direction_ferromagnetic() {
        // For J > 0, coupling to a positive neighbor should produce
        // a positive force (pull toward alignment).
        let c = PairwiseCoupling::chain(2, 1.0);
        let qpos = DVector::from_vec(vec![0.0, 1.0]);
        // Test via coupling_energy gradient (avoids needing a full Model/Data):
        // F_i = -dV/dx_i. For J > 0, coupling to a positive neighbor
        // should produce a positive force on DOF 0.
        let eps = 1e-8;
        let mut qpos_plus = qpos.clone();
        qpos_plus[0] += eps;
        let mut qpos_minus = qpos;
        qpos_minus[0] -= eps;
        let force_0 =
            -(c.coupling_energy(&qpos_plus) - c.coupling_energy(&qpos_minus)) / (2.0 * eps);
        assert!(
            force_0 > 0.0,
            "ferromagnetic coupling should pull DOF 0 toward positive neighbor, got F={force_0}"
        );
        assert!(
            (force_0 - 1.0).abs() < 1e-6,
            "expected F=1.0, got {force_0}"
        );
    }

    #[test]
    fn diagnostic_summary_format() {
        let c = PairwiseCoupling::chain(4, 0.5);
        let s = c.diagnostic_summary();
        assert!(s.contains("PairwiseCoupling"));
        assert!(s.contains("0.5000"));
        assert!(s.contains('3')); // 3 edges
    }
}
