//! Pairwise coupling for coupled bistable arrays.
//!
//! Implements the coupling potential `V = −Σ J_k · x_i · x_j` for each
//! edge `(i, j)` as a [`PassiveComponent`] that contributes conservative
//! forces to the `qfrc_passive` accumulator. Supports per-edge coupling
//! constants: each edge `k` has its own `J_k`. Combined with
//! [`DoubleWellPotential`] instances and a [`LangevinThermostat`] in a
//! [`PassiveStack`], this produces a coupled bistable system whose
//! equilibrium statistics match the Ising model on the same coupling
//! topology.
//!
//! Phase 4 validates this component (with uniform J) against exact Ising
//! predictions on a 4-element chain. Phase 5 uses per-edge J for
//! Boltzmann learning on a fully-connected graph.
//!
//! [`PassiveComponent`]: crate::PassiveComponent
//! [`DoubleWellPotential`]: crate::DoubleWellPotential
//! [`LangevinThermostat`]: crate::LangevinThermostat
//! [`PassiveStack`]: crate::PassiveStack

use sim_core::{DVector, Data, Model};

use crate::component::PassiveComponent;
use crate::diagnose::Diagnose;

/// Pairwise coupling: `V = −Σ_k J_k · x_i · x_j` for each edge `(i, j)`.
///
/// Contributes force `F_i = +Σ_{j: (i,j) ∈ edges} J_k · x_j` to the
/// per-DOF force accumulator. Each edge has its own coupling constant
/// `J_k`; ferromagnetic for `J_k > 0` (aligned positions energetically
/// favored), anti-ferromagnetic for `J_k < 0`.
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
///     DVector::from_element(4, 10.0), 1.0, 42, 0,
/// ));
/// builder.build().install(&mut model);
/// ```
pub struct PairwiseCoupling {
    /// Per-edge coupling constants. `coupling_j[k]` is the coupling
    /// constant for `edges[k]`.
    coupling_j: Vec<f64>,
    /// List of DOF pairs that are coupled.
    edges: Vec<(usize, usize)>,
}

impl PairwiseCoupling {
    /// Create a coupling with per-edge coupling constants.
    ///
    /// # Panics
    /// - If `coupling_j.len() != edges.len()`.
    /// - If any edge has `i == j` (self-coupling).
    #[must_use]
    pub fn new(coupling_j: Vec<f64>, edges: Vec<(usize, usize)>) -> Self {
        assert!(
            coupling_j.len() == edges.len(),
            "coupling_j length ({}) must match edges length ({})",
            coupling_j.len(),
            edges.len(),
        );
        for &(i, j) in &edges {
            assert!(i != j, "self-coupling not supported: edge ({i}, {j})");
        }
        Self { coupling_j, edges }
    }

    /// Create with uniform coupling constant across all edges.
    ///
    /// # Panics
    /// - If any edge has `i == j` (self-coupling).
    #[must_use]
    pub fn uniform(coupling_j: f64, edges: Vec<(usize, usize)>) -> Self {
        let j_vec = vec![coupling_j; edges.len()];
        Self::new(j_vec, edges)
    }

    /// Create a nearest-neighbor open chain:
    /// edges `[(0,1), (1,2), ..., (n−2, n−1)]`, uniform J.
    ///
    /// # Panics
    /// Panics if `n < 2`.
    #[must_use]
    pub fn chain(n: usize, coupling_j: f64) -> Self {
        assert!(n >= 2, "chain requires at least 2 elements, got {n}");
        let edges = (0..n - 1).map(|i| (i, i + 1)).collect();
        Self::uniform(coupling_j, edges)
    }

    /// Create a nearest-neighbor ring: chain + closing edge `(n−1, 0)`,
    /// uniform J.
    ///
    /// # Panics
    /// Panics if `n < 3`.
    #[must_use]
    pub fn ring(n: usize, coupling_j: f64) -> Self {
        assert!(n >= 3, "ring requires at least 3 elements, got {n}");
        let mut edges: Vec<(usize, usize)> = (0..n - 1).map(|i| (i, i + 1)).collect();
        edges.push((n - 1, 0));
        Self::uniform(coupling_j, edges)
    }

    /// Fully connected graph: all `N(N−1)/2` edges, uniform J.
    ///
    /// Edge ordering: `(0,1), (0,2), ..., (0,N−1), (1,2), ..., (N−2,N−1)`.
    /// Lexicographic.
    ///
    /// # Panics
    /// Panics if `n < 2`.
    #[must_use]
    pub fn fully_connected(n: usize, coupling_j: f64) -> Self {
        assert!(
            n >= 2,
            "fully_connected requires at least 2 elements, got {n}"
        );
        let mut edges = Vec::with_capacity(n * (n - 1) / 2);
        for i in 0..n {
            for j in (i + 1)..n {
                edges.push((i, j));
            }
        }
        Self::uniform(coupling_j, edges)
    }

    /// Per-edge coupling constants (read-only).
    #[must_use]
    pub fn coupling_j(&self) -> &[f64] {
        &self.coupling_j
    }

    /// Edge list (read-only).
    #[must_use]
    pub fn edges(&self) -> &[(usize, usize)] {
        &self.edges
    }

    /// Total coupling energy: `V = −Σ_k J_k · x_i · x_j`.
    #[must_use]
    pub fn coupling_energy(&self, qpos: &DVector<f64>) -> f64 {
        self.edges
            .iter()
            .zip(&self.coupling_j)
            .map(|(&(i, j), &j_k)| -j_k * qpos[i] * qpos[j])
            .sum()
    }
}

impl PassiveComponent for PairwiseCoupling {
    fn apply(&self, _model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        for (&(i, j), &j_k) in self.edges.iter().zip(&self.coupling_j) {
            let xi = data.qpos[i];
            let xj = data.qpos[j];
            // V = −J_k · xi · xj  →  F_i = +J_k · xj,  F_j = +J_k · xi
            qfrc_out[i] += j_k * xj;
            qfrc_out[j] += j_k * xi;
        }
    }
}

impl Diagnose for PairwiseCoupling {
    fn diagnostic_summary(&self) -> String {
        let j_min = self
            .coupling_j
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let j_max = self
            .coupling_j
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        if (j_min - j_max).abs() < 1e-15 {
            format!("PairwiseCoupling(J={j_min:.4}, edges={})", self.edges.len())
        } else {
            format!(
                "PairwiseCoupling(J=[{j_min:.4}, {j_max:.4}], edges={})",
                self.edges.len()
            )
        }
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
        let _ = PairwiseCoupling::new(vec![1.0], vec![(0, 0)]);
    }

    #[test]
    #[should_panic(expected = "coupling_j length (2) must match edges length (1)")]
    fn new_rejects_length_mismatch() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = PairwiseCoupling::new(vec![1.0, 2.0], vec![(0, 1)]);
    }

    #[test]
    fn chain_produces_correct_edges() {
        let c = PairwiseCoupling::chain(4, 0.5);
        assert_eq!(c.edges(), &[(0, 1), (1, 2), (2, 3)]);
        assert_eq!(c.coupling_j(), &[0.5, 0.5, 0.5]);
    }

    #[test]
    fn chain_2_produces_single_edge() {
        let c = PairwiseCoupling::chain(2, 1.0);
        assert_eq!(c.edges(), &[(0, 1)]);
        assert_eq!(c.coupling_j(), &[1.0]);
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
        assert_eq!(r.coupling_j(), &[0.5, 0.5, 0.5, 0.5]);
    }

    #[test]
    #[should_panic(expected = "ring requires at least 3")]
    fn ring_2_panics() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = PairwiseCoupling::ring(2, 1.0);
    }

    #[test]
    fn fully_connected_4_produces_6_edges() {
        let fc = PairwiseCoupling::fully_connected(4, 0.3);
        assert_eq!(
            fc.edges(),
            &[(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
        );
        assert_eq!(fc.coupling_j().len(), 6);
        assert!(fc.coupling_j().iter().all(|&j| (j - 0.3).abs() < 1e-15));
    }

    #[test]
    fn fully_connected_2_produces_single_edge() {
        let fc = PairwiseCoupling::fully_connected(2, 1.0);
        assert_eq!(fc.edges(), &[(0, 1)]);
    }

    #[test]
    #[should_panic(expected = "fully_connected requires at least 2")]
    fn fully_connected_1_panics() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = PairwiseCoupling::fully_connected(1, 1.0);
    }

    #[test]
    fn per_edge_coupling_energy() {
        // 2 edges with different J: edge (0,1) J=1.0, edge (1,2) J=-0.5
        let c = PairwiseCoupling::new(vec![1.0, -0.5], vec![(0, 1), (1, 2)]);
        let qpos = DVector::from_vec(vec![1.0, 1.0, 1.0]);
        // V = -1.0*1*1 + -(-0.5)*1*1 = -1.0 + 0.5... wait:
        // V = Σ -J_k * x_i * x_j
        // = -1.0 * 1 * 1 + -(-0.5) * 1 * 1 = -1.0 + 0.5 = -0.5
        let energy = c.coupling_energy(&qpos);
        assert!(
            (energy - (-0.5)).abs() < 1e-15,
            "expected -0.5, got {energy}"
        );
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
        let c = PairwiseCoupling::chain(2, 1.0);
        let qpos = DVector::from_vec(vec![0.0, 1.0]);
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
    fn diagnostic_summary_uniform() {
        let c = PairwiseCoupling::chain(4, 0.5);
        let s = c.diagnostic_summary();
        assert!(s.contains("PairwiseCoupling"));
        assert!(s.contains("0.5000"));
        assert!(s.contains('3')); // 3 edges
    }

    #[test]
    fn diagnostic_summary_per_edge() {
        let c = PairwiseCoupling::new(vec![0.3, -0.5, 0.8], vec![(0, 1), (1, 2), (2, 3)]);
        let s = c.diagnostic_summary();
        assert!(s.contains("PairwiseCoupling"));
        assert!(s.contains("-0.5000"));
        assert!(s.contains("0.8000"));
        assert!(s.contains('3'));
    }
}
