//! Pairwise coupling for coupled bistable arrays.
//!
//! Implements the coupling potential `V = −Σ J_k · x_i · x_j` for each
//! edge `(i, j)` as a [`PassiveComponent`] that contributes conservative
//! forces to the `qfrc_passive` accumulator, with one `J_k` per edge.
//! [`PairwiseCoupling`]'s doc says which Ising model a coupled array of
//! [`DoubleWellPotential`]s samples.
//!
//! [`PassiveComponent`]: crate::PassiveComponent
//! [`DoubleWellPotential`]: crate::DoubleWellPotential

use sim_core::{DVector, Data, Model};

use crate::component::{PassiveComponent, check_position_dof, qpos_index};
use crate::diagnose::Diagnose;
use crate::error::ThermostatError;
use crate::params::{Domain, check_edges, check_len, or_panic};

const COMPONENT: &str = "PairwiseCoupling";

/// `Ok` if a generated topology's element count `n` is at least `min`.
const fn check_count(
    n: usize,
    min: usize,
    requirement: &'static str,
) -> Result<(), ThermostatError> {
    if n >= min {
        Ok(())
    } else {
        Err(ThermostatError::InvalidCount {
            component: COMPONENT,
            parameter: "n",
            value: n,
            requirement,
        })
    }
}

/// Pairwise coupling: `V = −Σ_k J_k · x_i · x_j` for each edge `(i, j)`.
///
/// Contributes force `F_i = +Σ_{j: (i,j) ∈ edges} J_k · x_j` to the
/// per-DOF force accumulator. Each edge has its own coupling constant
/// `J_k`; ferromagnetic for `J_k > 0` (aligned positions energetically
/// favored), anti-ferromagnetic for `J_k < 0`.
///
/// Not stochastic — this is a deterministic conservative force.
///
/// # Which DOFs
///
/// Each edge is a pair of DOF indices; the forces go to those DOFs.
/// The position is read through the DOF's joint, so the DOF must be a slide
/// or hinge DOF, or one of a free joint's three translation DOFs. A ball
/// joint's DOFs and a free joint's rotation DOFs have no coordinate of their
/// own, so
/// [`PassiveStack::try_install`](crate::PassiveStack::try_install) refuses them.
///
/// # As an Ising model
///
/// With a [`DoubleWellPotential`](crate::DoubleWellPotential) on each DOF (minima at
/// `±x₀`), the coupling's energy at the wells' bottoms is the Ising coupling `J_k·x₀²`
/// between the spins, the signs of the positions. At a temperature the array samples a
/// different Ising model:
/// - at first order its couplings are `μ²·J_k·x₀²`, where `μ` is the mean of `|x|/x₀` in
///   one well (`μ² ≈ 0.91` at `ΔV/kT = 3`, reading only positions beyond `x₀/2`);
/// - at second order, spins that share a neighbour gain a coupling through it;
/// - a well vanishes once a site's neighbours tilt it far enough, at a point that depends
///   on the graph.
///
/// `tests/ising_mapping.rs` measures all three.
/// [`IsingProblem::add_components`](crate::IsingProblem::add_components) builds the
/// components for an Ising problem.
///
/// # Example
///
/// ```
/// use sim_core::DVector;
/// use sim_thermostat::{DoubleWellPotential, LangevinThermostat, PairwiseCoupling, PassiveStack};
///
/// // Four slide particles of mass 1 (sim-core's `test-fixtures` feature; or
/// // `sim_therm_env::generate_mjcf(4, 0, 0.001, (0.0, 1.0))` as MJCF).
/// let mut model = sim_core::test_fixtures::bistable_chain(4);
/// let mut builder = PassiveStack::builder();
/// for i in 0..4 {
///     builder = builder.with(DoubleWellPotential::new(3.0, 1.0, i));
/// }
/// builder
///     .with(PairwiseCoupling::chain(4, 0.5))
///     .with(LangevinThermostat::new(DVector::from_element(4, 10.0), 1.0, 42, 0))
///     .build()
///     .try_install(&mut model)?;
/// let mut data = model.make_data();
/// data.step(&model)?;
/// # Ok::<(), Box<dyn std::error::Error>>(())
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
    /// If [`Self::try_new`] refuses the coupling.
    #[must_use]
    #[track_caller]
    pub fn new(coupling_j: Vec<f64>, edges: Vec<(usize, usize)>) -> Self {
        or_panic(Self::try_new(coupling_j, edges))
    }

    /// [`Self::new`], returning the refusal instead of panicking.
    ///
    /// # Errors
    /// - [`ThermostatError::LengthMismatch`] unless `coupling_j` has one entry per edge.
    /// - [`ThermostatError::InvalidParameter`] if an entry of `coupling_j` is not finite.
    /// - [`ThermostatError::InvalidEdge`] if an edge has `i == j` (self-coupling), or a pair
    ///   appears twice in either order (the two edges would add their `J`s).
    pub fn try_new(
        coupling_j: Vec<f64>,
        edges: Vec<(usize, usize)>,
    ) -> Result<Self, ThermostatError> {
        check_len(
            COMPONENT,
            "coupling_j",
            coupling_j.len(),
            edges.len(),
            "edge",
        )?;
        Domain::Finite.check_each(COMPONENT, "coupling_j", &coupling_j)?;
        check_edges(COMPONENT, None, &edges)?;
        Ok(Self { coupling_j, edges })
    }

    /// Create with uniform coupling constant across all edges.
    ///
    /// # Panics
    /// If [`Self::try_uniform`] refuses the coupling.
    #[must_use]
    #[track_caller]
    pub fn uniform(coupling_j: f64, edges: Vec<(usize, usize)>) -> Self {
        or_panic(Self::try_uniform(coupling_j, edges))
    }

    /// [`Self::uniform`], returning the refusal instead of panicking.
    ///
    /// # Errors
    /// [`ThermostatError::InvalidParameter`] if `coupling_j` is not finite, and
    /// [`ThermostatError::InvalidEdge`] as for [`Self::try_new`].
    pub fn try_uniform(
        coupling_j: f64,
        edges: Vec<(usize, usize)>,
    ) -> Result<Self, ThermostatError> {
        Domain::Finite.check(COMPONENT, "coupling_j", coupling_j)?;
        Self::try_new(vec![coupling_j; edges.len()], edges)
    }

    /// Create a nearest-neighbor open chain:
    /// edges `[(0,1), (1,2), ..., (n−2, n−1)]`, uniform J.
    ///
    /// # Panics
    /// If [`Self::try_chain`] refuses the coupling.
    #[must_use]
    #[track_caller]
    pub fn chain(n: usize, coupling_j: f64) -> Self {
        or_panic(Self::try_chain(n, coupling_j))
    }

    /// [`Self::chain`], returning the refusal instead of panicking.
    ///
    /// # Errors
    /// [`ThermostatError::InvalidCount`] if `n < 2`, and
    /// [`ThermostatError::InvalidParameter`] if `coupling_j` is not finite.
    pub fn try_chain(n: usize, coupling_j: f64) -> Result<Self, ThermostatError> {
        check_count(n, 2, "at least 2 for a chain")?;
        Self::try_uniform(coupling_j, (0..n - 1).map(|i| (i, i + 1)).collect())
    }

    /// Create a nearest-neighbor ring: chain + closing edge `(n−1, 0)`,
    /// uniform J.
    ///
    /// # Panics
    /// If [`Self::try_ring`] refuses the coupling.
    #[must_use]
    #[track_caller]
    pub fn ring(n: usize, coupling_j: f64) -> Self {
        or_panic(Self::try_ring(n, coupling_j))
    }

    /// [`Self::ring`], returning the refusal instead of panicking.
    ///
    /// # Errors
    /// [`ThermostatError::InvalidCount`] if `n < 3`, and
    /// [`ThermostatError::InvalidParameter`] if `coupling_j` is not finite.
    pub fn try_ring(n: usize, coupling_j: f64) -> Result<Self, ThermostatError> {
        check_count(n, 3, "at least 3 for a ring")?;
        let mut edges: Vec<(usize, usize)> = (0..n - 1).map(|i| (i, i + 1)).collect();
        edges.push((n - 1, 0));
        Self::try_uniform(coupling_j, edges)
    }

    /// Fully connected graph: all `N(N−1)/2` edges, uniform J.
    ///
    /// Edge ordering: `(0,1), (0,2), ..., (0,N−1), (1,2), ..., (N−2,N−1)`.
    /// Lexicographic.
    ///
    /// # Panics
    /// If [`Self::try_fully_connected`] refuses the coupling.
    #[must_use]
    #[track_caller]
    pub fn fully_connected(n: usize, coupling_j: f64) -> Self {
        or_panic(Self::try_fully_connected(n, coupling_j))
    }

    /// [`Self::fully_connected`], returning the refusal instead of panicking.
    ///
    /// # Errors
    /// [`ThermostatError::InvalidCount`] if `n < 2`, and
    /// [`ThermostatError::InvalidParameter`] if `coupling_j` is not finite.
    pub fn try_fully_connected(n: usize, coupling_j: f64) -> Result<Self, ThermostatError> {
        check_count(n, 2, "at least 2")?;
        let mut edges = Vec::with_capacity(n * (n - 1) / 2);
        for i in 0..n {
            for j in (i + 1)..n {
                edges.push((i, j));
            }
        }
        Self::try_uniform(coupling_j, edges)
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

    /// Total coupling energy at `data`'s positions: `V = −Σ_k J_k · x_i · x_j`, where `x_i`
    /// is DOF `i`'s position.
    ///
    /// # Errors
    ///
    /// Returns the error [`PassiveComponent::validate`] would: an edge's DOF is missing from
    /// `model` or has no position coordinate of its own.
    pub fn coupling_energy(&self, model: &Model, data: &Data) -> Result<f64, ThermostatError> {
        self.validate(model)?;
        let x = |dof| data.qpos[qpos_index(model, dof)];
        Ok(self
            .edges
            .iter()
            .zip(&self.coupling_j)
            .map(|(&(i, j), &j_k)| -j_k * x(i) * x(j))
            .sum())
    }
}

impl PassiveComponent for PairwiseCoupling {
    fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        for (&(i, j), &j_k) in self.edges.iter().zip(&self.coupling_j) {
            let xi = data.qpos[qpos_index(model, i)];
            let xj = data.qpos[qpos_index(model, j)];
            // V = −J_k · xi · xj  →  F_i = +J_k · xj,  F_j = +J_k · xi
            qfrc_out[i] += j_k * xj;
            qfrc_out[j] += j_k * xi;
        }
    }

    fn as_diagnose(&self) -> Option<&dyn Diagnose> {
        Some(self)
    }

    fn validate(&self, model: &Model) -> Result<(), ThermostatError> {
        self.edges.iter().try_for_each(|&(i, j)| {
            check_position_dof(model, i, "PairwiseCoupling")?;
            check_position_dof(model, j, "PairwiseCoupling")
        })
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
    use crate::params::refused_parameter;

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
    #[should_panic(expected = "PairwiseCoupling: edge (0, 0) joins an element to itself")]
    fn new_rejects_self_coupling() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = PairwiseCoupling::new(vec![1.0], vec![(0, 0)]);
    }

    #[test]
    #[should_panic(
        expected = "PairwiseCoupling: coupling_j has length 2, expected 1 (one per edge)"
    )]
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
    #[should_panic(expected = "PairwiseCoupling: n must be at least 2 for a chain, got 1")]
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
    #[should_panic(expected = "PairwiseCoupling: n must be at least 3 for a ring, got 2")]
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
    #[should_panic(expected = "PairwiseCoupling: n must be at least 2, got 1")]
    fn fully_connected_1_panics() {
        #[allow(clippy::let_underscore_must_use)]
        let _ = PairwiseCoupling::fully_connected(1, 1.0);
    }

    #[test]
    fn every_constructor_refuses_a_coupling_that_is_not_finite() {
        for bad in [f64::NAN, f64::INFINITY] {
            let names = [
                PairwiseCoupling::try_new(vec![1.0, bad], vec![(0, 1), (1, 2)]),
                PairwiseCoupling::try_uniform(bad, vec![(0, 1)]),
                PairwiseCoupling::try_chain(3, bad),
                PairwiseCoupling::try_ring(3, bad),
                PairwiseCoupling::try_fully_connected(3, bad),
            ]
            .map(refused_parameter);
            let expected = [
                "coupling_j[1]",
                "coupling_j",
                "coupling_j",
                "coupling_j",
                "coupling_j",
            ];
            assert_eq!(names, expected.map(|p| Some(p.to_owned())), "{bad}");
        }
    }

    /// A topology too small to build is refused before its coupling is read.
    #[test]
    fn try_constructors_refuse_too_few_elements() {
        assert!(matches!(
            PairwiseCoupling::try_ring(2, f64::NAN),
            Err(ThermostatError::InvalidCount { value: 2, .. })
        ));
        assert!(PairwiseCoupling::try_ring(3, 0.0).is_ok());
    }

    #[test]
    fn per_edge_coupling_energy() {
        // 2 edges with different J: edge (0,1) J=1.0, edge (1,2) J=-0.5
        let c = PairwiseCoupling::new(vec![1.0, -0.5], vec![(0, 1), (1, 2)]);
        let (model, data) = chain_at(&[1.0, 1.0, 1.0]);
        // V = Σ -J_k * x_i * x_j = -1.0 * 1 * 1 + -(-0.5) * 1 * 1 = -1.0 + 0.5 = -0.5
        let energy = c.coupling_energy(&model, &data).unwrap();
        assert!(
            (energy - (-0.5)).abs() < 1e-15,
            "expected -0.5, got {energy}"
        );
    }

    #[test]
    fn coupling_energy_all_aligned() {
        // 4-chain, all at +1: V = -J(1·1 + 1·1 + 1·1) = -3J
        let c = PairwiseCoupling::chain(4, 0.5);
        let (model, data) = chain_at(&[1.0; 4]);
        let energy = c.coupling_energy(&model, &data).unwrap();
        assert!(
            (energy - (-1.5)).abs() < 1e-15,
            "expected -1.5, got {energy}"
        );
    }

    #[test]
    fn coupling_energy_alternating() {
        // 4-chain, alternating +1/-1: V = -J(-1 + -1 + -1) = +3J
        let c = PairwiseCoupling::chain(4, 0.5);
        let (model, data) = chain_at(&[1.0, -1.0, 1.0, -1.0]);
        let energy = c.coupling_energy(&model, &data).unwrap();
        assert!((energy - 1.5).abs() < 1e-15, "expected 1.5, got {energy}");
    }

    #[test]
    fn force_direction_ferromagnetic() {
        let c = PairwiseCoupling::chain(2, 1.0);
        let eps = 1e-8;
        let (model, plus) = chain_at(&[eps, 1.0]);
        let (_, minus) = chain_at(&[-eps, 1.0]);
        let force_0 = -(c.coupling_energy(&model, &plus).unwrap()
            - c.coupling_energy(&model, &minus).unwrap())
            / (2.0 * eps);
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
