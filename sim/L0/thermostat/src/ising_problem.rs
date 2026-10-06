//! An Ising problem, a QUBO's conversion to one, the components that put it on a coupled
//! bistable array, and a latch for the lowest-energy configuration a trajectory reads.

use sim_core::{Data, Model};

use crate::component::{check_position_dof, qpos_index};
use crate::error::ThermostatError;
use crate::params::{Domain, check_edges, check_len, or_panic};
use crate::well_state::WellState;
use crate::{DoubleWellPotential, ExternalField, PairwiseCoupling, PassiveStackBuilder};

const COMPONENT: &str = "IsingProblem";

/// The tilt at which one quartic well loses its second minimum, in units of `ΔV/x₀`:
/// `8/(3√3) ≈ 1.5396`.
fn well_removal_tilt() -> f64 {
    8.0 / (3.0 * 3.0_f64.sqrt())
}

/// An Ising problem in energy units.
///
/// `H(σ) = −Σ_k J_k·σ_i·σ_j − Σ_i h_i·σ_i` over `σ ∈ {−1, +1}ⁿ`, the Hamiltonian of
/// [`exact_distribution`](crate::ising::exact_distribution). Spin `i` lives on DOF `i` once
/// [`Self::add_components`] puts the problem on an array. There is no spin cap here; only the
/// exact solvers enumerate configurations.
#[derive(Clone, Debug, PartialEq)]
pub struct IsingProblem {
    n: usize,
    edges: Vec<(usize, usize)>,
    coupling_j: Vec<f64>,
    field_h: Vec<f64>,
}

impl IsingProblem {
    /// A problem on `n` spins with coupling `coupling_j[k]` on `edges[k]` and field
    /// `field_h[i]` on spin `i`.
    ///
    /// # Panics
    /// If [`Self::try_new`] refuses the problem.
    #[must_use]
    #[track_caller]
    pub fn new(
        n: usize,
        edges: Vec<(usize, usize)>,
        coupling_j: Vec<f64>,
        field_h: Vec<f64>,
    ) -> Self {
        or_panic(Self::try_new(n, edges, coupling_j, field_h))
    }

    /// [`Self::new`], returning the refusal instead of panicking.
    ///
    /// # Errors
    /// - [`ThermostatError::EdgeOutOfRange`], [`ThermostatError::SelfEdge`] or
    ///   [`ThermostatError::RepeatedEdge`] if an edge
    ///   names a spin outside `0..n`, joins a spin to itself, or repeats a pair.
    /// - [`ThermostatError::LengthMismatch`] unless `coupling_j` has one entry per edge and
    ///   `field_h` one per spin.
    /// - [`ThermostatError::InvalidParameter`] if an entry of either is not finite.
    pub fn try_new(
        n: usize,
        edges: Vec<(usize, usize)>,
        coupling_j: Vec<f64>,
        field_h: Vec<f64>,
    ) -> Result<Self, ThermostatError> {
        check_edges(COMPONENT, Some(n), &edges)?;
        check_len(
            COMPONENT,
            "coupling_j",
            coupling_j.len(),
            edges.len(),
            "edge",
        )?;
        check_len(COMPONENT, "field_h", field_h.len(), n, "spin")?;
        Domain::Finite.check_each(COMPONENT, "coupling_j", &coupling_j)?;
        Domain::Finite.check_each(COMPONENT, "field_h", &field_h)?;
        Ok(Self {
            n,
            edges,
            coupling_j,
            field_h,
        })
    }

    /// The problem of minimising a QUBO in 0/1 variables,
    /// `E(x) = Σ_i a_i·x_i + Σ_k b_k·x_i·x_j` (`a = linear`, `b_k = quadratic[k]` on
    /// `pairs[k] = (i, j)`), and its energy offset.
    ///
    /// With `x = (1 + σ)/2`: `J_k = −b_k/4`, `h_i = −a_i/2 − Σ_{k ∋ i} b_k/4`, and
    /// `E(x) = H(σ) + offset` with `offset = Σ_i a_i/2 + Σ_k b_k/4`, so both have the same
    /// minimisers. A diagonal term `Q_ii·x_i²` belongs in `linear` (`x² = x`); a symmetric `Q`
    /// gives each pair once, with `Q_ij + Q_ji`.
    ///
    /// # Panics
    /// If [`Self::try_from_qubo`] refuses the QUBO.
    #[must_use]
    #[track_caller]
    pub fn from_qubo(linear: &[f64], pairs: &[(usize, usize)], quadratic: &[f64]) -> (Self, f64) {
        or_panic(Self::try_from_qubo(linear, pairs, quadratic))
    }

    /// [`Self::from_qubo`], returning the refusal instead of panicking.
    ///
    /// # Errors
    /// - [`ThermostatError::LengthMismatch`] unless `quadratic` has one entry per pair.
    /// - [`ThermostatError::InvalidParameter`] if an entry of `linear` or `quadratic` is not
    ///   finite, or the converted fields or the offset overflow.
    /// - As [`Self::try_new`] for the pairs, on `n = linear.len()` spins.
    pub fn try_from_qubo(
        linear: &[f64],
        pairs: &[(usize, usize)],
        quadratic: &[f64],
    ) -> Result<(Self, f64), ThermostatError> {
        let n = linear.len();
        check_len(COMPONENT, "quadratic", quadratic.len(), pairs.len(), "pair")?;
        Domain::Finite.check_each(COMPONENT, "linear", linear)?;
        Domain::Finite.check_each(COMPONENT, "quadratic", quadratic)?;
        check_edges(COMPONENT, Some(n), pairs)?;
        let mut field_h: Vec<f64> = linear.iter().map(|a| -a / 2.0).collect();
        for (&(i, j), &b) in pairs.iter().zip(quadratic) {
            field_h[i] -= b / 4.0;
            field_h[j] -= b / 4.0;
        }
        let offset = linear.iter().map(|a| a / 2.0).sum::<f64>()
            + quadratic.iter().map(|b| b / 4.0).sum::<f64>();
        Domain::Finite.check(COMPONENT, "offset", offset)?;
        let coupling_j = quadratic.iter().map(|b| -b / 4.0).collect();
        Ok((
            Self::try_new(n, pairs.to_vec(), coupling_j, field_h)?,
            offset,
        ))
    }

    /// The spin count.
    #[must_use]
    pub const fn n(&self) -> usize {
        self.n
    }

    /// The edges.
    #[must_use]
    pub fn edges(&self) -> &[(usize, usize)] {
        &self.edges
    }

    /// The coupling on each edge.
    #[must_use]
    pub fn coupling_j(&self) -> &[f64] {
        &self.coupling_j
    }

    /// The field on each spin.
    #[must_use]
    pub fn field_h(&self) -> &[f64] {
        &self.field_h
    }

    /// Every coupling and field times `factor` (an inverse temperature, or a change of
    /// units): `H` becomes `factor·H`, with the same minimisers.
    ///
    /// # Panics
    /// If [`Self::try_scaled`] refuses it.
    #[must_use]
    #[track_caller]
    pub fn scaled(&self, factor: f64) -> Self {
        or_panic(self.try_scaled(factor))
    }

    /// [`Self::scaled`], returning the refusal instead of panicking.
    ///
    /// # Errors
    /// [`ThermostatError::InvalidParameter`] unless `factor` is finite and positive (a negative
    /// one would swap minimisers and maximisers), or if a scaled entry overflows.
    pub fn try_scaled(&self, factor: f64) -> Result<Self, ThermostatError> {
        Domain::Positive.check(COMPONENT, "factor", factor)?;
        Self::try_new(
            self.n,
            self.edges.clone(),
            self.coupling_j.iter().map(|j| j * factor).collect(),
            self.field_h.iter().map(|h| h * factor).collect(),
        )
    }

    /// `H(σ)` for spins `σ_i = ±1`.
    ///
    /// # Panics
    /// Unless `spins` has one entry per spin and each is exactly `+1.0` or `−1.0`.
    #[must_use]
    #[track_caller]
    pub fn energy(&self, spins: &[f64]) -> f64 {
        or_panic(self.check_spins(spins));
        let couplings: f64 = self
            .edges
            .iter()
            .zip(&self.coupling_j)
            .map(|(&(i, j), &jk)| jk * spins[i] * spins[j])
            .sum();
        let fields: f64 = self.field_h.iter().zip(spins).map(|(h, s)| h * s).sum();
        -couplings - fields
    }

    #[allow(clippy::float_cmp)] // a spin is exactly ±1, not close to it
    fn check_spins(&self, spins: &[f64]) -> Result<(), ThermostatError> {
        check_len(COMPONENT, "spins", spins.len(), self.n, "spin")?;
        spins
            .iter()
            .position(|&s| s != 1.0 && s != -1.0)
            .map_or(Ok(()), |i| {
                Err(ThermostatError::InvalidParameter {
                    component: COMPONENT,
                    parameter: format!("spins[{i}]"),
                    value: spins[i],
                    requirement: "+1 or -1",
                })
            })
    }

    /// `builder` with the components that put this problem on DOFs `0..n` of a coupled
    /// bistable array appended: per spin `i` a [`DoubleWellPotential`] with barrier `delta_v`
    /// and minima at `±x_0`, then a [`PairwiseCoupling`] with `J/x₀²` and an
    /// [`ExternalField`] with `h/x₀`. Add a thermostat and build.
    ///
    /// At the wells' bottoms (`x = ±x₀`) the components' energy is `H(σ)` plus a constant.
    /// At a temperature the array samples a different Ising model: at first order it has
    /// couplings `μ²·J` and fields `μ·h`, where `μ` is the mean of `|x|/x₀` in one well
    /// (`μ² ≈ 0.91` at `ΔV/kT = 3` counting only samples with every `|x| ≥ x₀/2`, `0.82`
    /// counting every sample), and at second order spins with a shared neighbour gain a
    /// coupling through it. A site's well can vanish once its tilt ratio (see
    /// [`Self::tilt_ratios`]) nears 1. `tests/ising_mapping.rs` measures all three.
    ///
    /// # Panics
    /// If [`Self::try_add_components`] refuses `delta_v` or `x_0`.
    #[must_use]
    #[track_caller]
    pub fn add_components(
        &self,
        builder: PassiveStackBuilder,
        delta_v: f64,
        x_0: f64,
    ) -> PassiveStackBuilder {
        or_panic(self.try_add_components(builder, delta_v, x_0))
    }

    /// [`Self::add_components`], returning the refusal instead of panicking.
    ///
    /// # Errors
    /// [`ThermostatError::InvalidParameter`] unless `delta_v` and `x_0` are finite and
    /// positive, or if a coupling `J/x₀²` or field `h/x₀` overflows.
    pub fn try_add_components(
        &self,
        builder: PassiveStackBuilder,
        delta_v: f64,
        x_0: f64,
    ) -> Result<PassiveStackBuilder, ThermostatError> {
        Domain::Positive.check(COMPONENT, "delta_v", delta_v)?;
        Domain::Positive.check(COMPONENT, "x_0", x_0)?;
        let mut builder = builder;
        for i in 0..self.n {
            builder = builder.with(DoubleWellPotential::try_new(delta_v, x_0, i)?);
        }
        Ok(builder
            .with(PairwiseCoupling::try_new(
                self.coupling_j.iter().map(|j| j / (x_0 * x_0)).collect(),
                self.edges.clone(),
            )?)
            .with(ExternalField::try_new(
                self.field_h.iter().map(|h| h / x_0).collect(),
            )?))
    }

    /// Each spin's tilt ratio: the largest force its neighbours and field can put on it, all
    /// neighbours sitting at a well bottom, over the force that removes one well's second
    /// minimum, `r_i = (|h_i| + Σ_{j~i} |J_ij|) / (8/(3√3)·ΔV)`. It does not depend on `x₀`.
    ///
    /// A diagnostic, not a bound: at `T = 0` a well first vanishes between `r` 0.90 and 0.95
    /// on the ferromagnetic complete graph K4, between 1.28 and 1.32 on a pair, and between
    /// 1.53 and 1.58 on the antiferromagnetic K4 (`tests/ising_mapping.rs`), so no one value
    /// of `r` marks where the mapping breaks.
    ///
    /// # Panics
    /// Unless `delta_v` is finite and positive.
    #[must_use]
    #[track_caller]
    pub fn tilt_ratios(&self, delta_v: f64) -> Vec<f64> {
        or_panic(Domain::Positive.check(COMPONENT, "delta_v", delta_v));
        let mut tilt: Vec<f64> = self.field_h.iter().map(|h| h.abs()).collect();
        for (&(i, j), &jk) in self.edges.iter().zip(&self.coupling_j) {
            tilt[i] += jk.abs();
            tilt[j] += jk.abs();
        }
        let removal = well_removal_tilt() * delta_v;
        tilt.into_iter().map(|t| t / removal).collect()
    }
}

/// The lowest-energy configuration read so far from a trajectory of a coupled bistable
/// array: an annealer's answer.
#[derive(Clone, Debug)]
pub struct SpinLatch {
    problem: IsingProblem,
    x_thresh: f64,
    best: Option<(Vec<f64>, f64)>,
    in_well_reads: u64,
}

impl SpinLatch {
    /// A latch for `problem`, reading spin `i` from DOF `i`'s position `x` as
    /// `WellState::from_position(x, x_thresh)` (see [`WellState`]): `x_thresh` is a position,
    /// and the crate's gates use `x₀/2`.
    ///
    /// # Panics
    /// If [`Self::try_new`] refuses `x_thresh`.
    #[must_use]
    #[track_caller]
    pub fn new(problem: IsingProblem, x_thresh: f64) -> Self {
        or_panic(Self::try_new(problem, x_thresh))
    }

    /// [`Self::new`], returning the refusal instead of panicking.
    ///
    /// # Errors
    /// [`ThermostatError::InvalidParameter`] unless `x_thresh` is finite and non-negative.
    pub fn try_new(problem: IsingProblem, x_thresh: f64) -> Result<Self, ThermostatError> {
        Domain::NonNegative.check("SpinLatch", "x_thresh", x_thresh)?;
        Ok(Self {
            problem,
            x_thresh,
            best: None,
            in_well_reads: 0,
        })
    }

    /// Read the configuration at `data`. If every spin is in a well, count the read and keep
    /// the configuration if its energy is below the best so far (on a tie the earlier one
    /// stays). Returns whether this read became the best. A `NaN` position reads as the
    /// barrier, so a simulation that has blown up stops being read.
    ///
    /// # Errors
    /// [`ThermostatError::DofOutOfRange`] or [`ThermostatError::NoPositionCoordinate`] if one
    /// of DOFs `0..n` is missing from `model` or has no position coordinate of its own.
    pub fn observe(&mut self, model: &Model, data: &Data) -> Result<bool, ThermostatError> {
        for dof in 0..self.problem.n {
            check_position_dof(model, dof, "SpinLatch")?;
        }
        let mut spins = Vec::with_capacity(self.problem.n);
        for dof in 0..self.problem.n {
            match WellState::from_position(data.qpos[qpos_index(model, dof)], self.x_thresh) {
                WellState::Barrier => return Ok(false),
                state => spins.push(state.spin()),
            }
        }
        self.in_well_reads += 1;
        let energy = self.problem.energy(&spins);
        let improved = self.best.as_ref().is_none_or(|&(_, best)| energy < best);
        if improved {
            self.best = Some((spins, energy));
        }
        Ok(improved)
    }

    /// The best configuration read so far, as spins `±1`, and its energy `H(σ)`.
    #[must_use]
    pub fn best(&self) -> Option<(&[f64], f64)> {
        self.best
            .as_ref()
            .map(|(spins, energy)| (spins.as_slice(), *energy))
    }

    /// How many reads found every spin in a well.
    #[must_use]
    pub const fn in_well_reads(&self) -> u64 {
        self.in_well_reads
    }

    /// The problem the latch scores against.
    #[must_use]
    pub const fn problem(&self) -> &IsingProblem {
        &self.problem
    }

    /// Forget the best configuration and the read count.
    pub fn reset(&mut self) {
        self.best = None;
        self.in_well_reads = 0;
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::PassiveStack;
    use crate::params::refused_parameter;

    /// Spins `±1` from bitmask `c`, bit `i` set ⇔ spin `i` is `+1`.
    fn spins(c: usize, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| if c >> i & 1 == 1 { 1.0 } else { -1.0 })
            .collect()
    }

    /// A QUBO's energy equals the converted problem's `H` plus the offset at every `x`.
    #[test]
    fn from_qubo_keeps_every_energy_up_to_the_offset() {
        let linear = [1.5, -2.0, 0.25, 3.0];
        let pairs = [(0, 1), (2, 1), (0, 3), (3, 2)];
        let quadratic = [-1.0, 2.5, 0.75, -4.0];
        let (problem, offset) = IsingProblem::from_qubo(&linear, &pairs, &quadratic);
        for c in 0..16 {
            let x: Vec<f64> = (0..4)
                .map(|i| f64::from(u8::from(c >> i & 1 == 1)))
                .collect();
            let qubo = linear.iter().zip(&x).map(|(a, xi)| a * xi).sum::<f64>()
                + pairs
                    .iter()
                    .zip(&quadratic)
                    .map(|(&(i, j), b)| b * x[i] * x[j])
                    .sum::<f64>();
            let ising = problem.energy(&spins(c, 4)) + offset;
            assert!(
                (qubo - ising).abs() < 1e-12,
                "x {x:?}: QUBO {qubo}, H + offset {ising}"
            );
        }
    }

    #[test]
    fn try_from_qubo_refuses_a_repeated_pair_and_a_mismatched_length() {
        assert!(matches!(
            IsingProblem::try_from_qubo(&[0.0; 3], &[(0, 1), (1, 0)], &[1.0, 1.0]),
            Err(ThermostatError::RepeatedEdge { edge: (1, 0), .. })
        ));
        assert!(matches!(
            IsingProblem::try_from_qubo(&[0.0; 3], &[(0, 1)], &[1.0, 1.0]),
            Err(ThermostatError::LengthMismatch {
                parameter: "quadratic",
                ..
            })
        ));
        assert_eq!(
            refused_parameter(IsingProblem::try_from_qubo(&[0.0, f64::NAN], &[], &[])).as_deref(),
            Some("linear[1]")
        );
    }

    /// The offset is summed term by term: two linear terms near `f64::MAX` halve to a finite
    /// offset whose raw sum would overflow.
    #[test]
    fn from_qubo_keeps_a_finite_offset_whose_raw_sum_overflows() {
        let (problem, offset) = IsingProblem::from_qubo(&[1.5e308, 1.5e308], &[], &[]);
        assert_eq!(offset, 1.5e308);
        assert_eq!(problem.field_h(), [-7.5e307, -7.5e307]);
    }

    #[test]
    fn try_new_refuses_an_edge_outside_the_spins() {
        assert!(matches!(
            IsingProblem::try_new(2, vec![(0, 2)], vec![1.0], vec![0.0; 2]),
            Err(ThermostatError::EdgeOutOfRange { n: 2, .. })
        ));
    }

    #[test]
    fn scaled_multiplies_every_energy_and_refuses_a_factor_that_is_not_positive() {
        let problem = IsingProblem::new(
            3,
            vec![(0, 1), (1, 2)],
            vec![0.5, -1.0],
            vec![0.2, 0.0, -0.3],
        );
        let double = problem.scaled(2.0);
        for c in 0..8 {
            let s = spins(c, 3);
            assert_eq!(double.energy(&s), 2.0 * problem.energy(&s));
        }
        for bad in [0.0, -1.0, f64::NAN] {
            assert_eq!(
                refused_parameter(problem.try_scaled(bad)).as_deref(),
                Some("factor"),
                "{bad}"
            );
        }
    }

    #[test]
    #[should_panic(expected = "IsingProblem: spins[1] must be +1 or -1, got 0.5")]
    fn energy_refuses_a_spin_that_is_not_plus_or_minus_one() {
        let _energy = IsingProblem::new(2, vec![], vec![], vec![0.0; 2]).energy(&[1.0, 0.5]);
    }

    #[test]
    #[should_panic(expected = "IsingProblem: spins has length 1, expected 2 (one per spin)")]
    fn energy_refuses_the_wrong_number_of_spins() {
        let _energy = IsingProblem::new(2, vec![], vec![], vec![0.0; 2]).energy(&[1.0]);
    }

    /// Spin 0 feels `|h_0| + |J|`, spin 1 `|J|`, over `8/(3√3)·ΔV`.
    #[test]
    fn tilt_ratios_sum_each_spins_field_and_couplings() {
        let problem = IsingProblem::new(
            3,
            vec![(0, 1), (0, 2)],
            vec![1.0, -0.5],
            vec![-0.5, 0.0, 0.0],
        );
        let removal = 8.0 / (3.0 * 3.0_f64.sqrt()) * 2.0;
        let r = problem.tilt_ratios(2.0);
        let expected = [2.0 / removal, 1.0 / removal, 0.5 / removal];
        for (got, want) in r.iter().zip(expected) {
            assert!((got - want).abs() < 1e-15, "{r:?} vs {expected:?}");
        }
    }

    #[test]
    #[should_panic(expected = "IsingProblem: delta_v must be finite and positive, got 0")]
    fn tilt_ratios_refuse_a_barrier_that_is_not_positive() {
        let _r = IsingProblem::new(1, vec![], vec![], vec![0.0]).tilt_ratios(0.0);
    }

    /// At the wells' bottoms the components push spin `i` with `(Σ_j J_ij·σ_j + h_i)/x₀`: the
    /// Ising local field, scaled by the `J/x₀²` and `h/x₀` the builder applies.
    #[test]
    fn add_components_puts_the_local_field_on_each_dof() {
        let problem = IsingProblem::new(
            3,
            vec![(0, 1), (1, 2)],
            vec![0.8, -0.3],
            vec![0.2, 0.0, -0.1],
        );
        let x_0 = 1.5;
        let mut model = sim_core::test_fixtures::bistable_chain(3);
        problem
            .add_components(PassiveStack::builder(), 2.0, x_0)
            .build()
            .try_install(&mut model)
            .unwrap();
        let mut data = model.make_data();
        for c in 0..8 {
            let s = spins(c, 3);
            for (i, &si) in s.iter().enumerate() {
                data.qpos[i] = si * x_0;
            }
            data.forward(&model).unwrap();
            let local = [0.8 * s[1] + 0.2, 0.8 * s[0] - 0.3 * s[2], -0.3 * s[1] - 0.1];
            for (i, field) in local.into_iter().enumerate() {
                let pushed = data.qfrc_passive[i] * x_0;
                assert!(
                    (pushed - field).abs() < 1e-12,
                    "config {c}, DOF {i}: force·x₀ {pushed}, local field {field}"
                );
            }
        }
        assert_eq!(
            refused_parameter(problem.try_add_components(PassiveStack::builder(), 2.0, 0.0))
                .as_deref(),
            Some("x_0")
        );
    }

    /// The latch keeps the lowest energy read, skips reads with a spin in the barrier, keeps
    /// the earlier of two equal energies, and forgets everything on `reset`.
    #[test]
    fn spin_latch_keeps_the_lowest_energy_read() {
        // Ferromagnetic pair with a field on spin 0: H(++) = -1.5, H(--) = -0.5, H(+-) = H(-+) = 1.
        let problem = IsingProblem::new(2, vec![(0, 1)], vec![1.0], vec![0.5, 0.0]);
        let model = sim_core::test_fixtures::bistable_chain(2);
        let mut data = model.make_data();
        let mut latch = SpinLatch::new(problem, 0.5);
        let mut read = |x: [f64; 2]| {
            data.qpos[0] = x[0];
            data.qpos[1] = x[1];
            latch.observe(&model, &data).unwrap()
        };
        assert!(read([1.0, -1.0]));
        assert!(read([-1.0, -1.0]));
        assert!(!read([-1.0, 1.0]));
        assert!(!read([0.2, 1.0]), "a spin in the barrier was read");
        assert!(read([1.0, 1.0]));
        assert!(!read([1.0, 1.0]), "an equal energy replaced the best");
        assert!(!read([f64::NAN, 1.0]));
        assert_eq!(latch.best(), Some(([1.0, 1.0].as_slice(), -1.5)));
        assert_eq!(latch.in_well_reads(), 5);
        latch.reset();
        assert_eq!((latch.best(), latch.in_well_reads()), (None, 0));
    }

    #[test]
    fn spin_latch_refuses_a_model_without_the_dofs_and_a_bad_threshold() {
        let problem = IsingProblem::new(2, vec![], vec![], vec![0.0; 2]);
        let model = sim_core::test_fixtures::bistable_chain(1);
        let data = model.make_data();
        assert_eq!(
            SpinLatch::new(problem.clone(), 0.5).observe(&model, &data),
            Err(ThermostatError::DofOutOfRange {
                component: "SpinLatch",
                dof: 1,
                nv: 1,
            })
        );
        for bad in [-0.1, f64::NAN] {
            assert_eq!(
                refused_parameter(SpinLatch::try_new(problem.clone(), bad)).as_deref(),
                Some("x_thresh")
            );
        }
    }
}
