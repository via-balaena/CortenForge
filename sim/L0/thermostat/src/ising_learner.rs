//! Boltzmann machine learning on a physical Ising sampler.
//!
//! Trains per-edge coupling constants `J_{ij}` and per-site external
//! fields `h_i` to match a target distribution using the Boltzmann
//! learning rule. The physical Langevin simulation is the generative
//! model — no software sampler, no autograd tape, no finite differences.
//!
//! Phase 5 of the thermodynamic computing initiative validates this
//! module against a known Ising target on a fully-connected 4-element
//! graph. D4 (sim-to-real on a printed device) reuses this training
//! algorithm to train the EBM before printing.

use std::sync::Arc;

use sim_core::{DVector, Model};

use crate::component::qpos_index;
use crate::error::ThermostatError;
use crate::ising::check_problem_shape;
use crate::params::{Domain, check_len, or_panic};
use crate::well_state::WellState;
use crate::{
    DoubleWellPotential, ExternalField, LangevinThermostat, PairwiseCoupling, PassiveStack,
};

const COMPONENT: &str = "IsingLearner";

/// Configuration for the Boltzmann learning loop.
///
/// Build one with [`LearnerConfig::new`] and set fields on the result: the struct is
/// `#[non_exhaustive]`, so a field added in a later release does not break callers.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct LearnerConfig {
    /// Number of elements.
    pub n: usize,
    /// Coupling topology (edge list).
    pub edges: Vec<(usize, usize)>,
    /// Double-well barrier height.
    pub delta_v: f64,
    /// Well half-separation.
    pub x_0: f64,
    /// Thermostat damping coefficient (per DOF, uniform).
    pub gamma: f64,
    /// Thermal energy kT.
    pub k_b_t: f64,
    /// Learning rate η.
    pub learning_rate: f64,
    /// Total simulation steps per trajectory (including burn-in).
    pub n_steps: usize,
    /// Burn-in steps per trajectory.
    pub n_burn_in: usize,
    /// Independent trajectories per measurement.
    pub n_trajectories: usize,
    /// Spin classification threshold.
    pub x_thresh: f64,
    /// RNG seed base.
    pub seed_base: u64,
}

impl LearnerConfig {
    /// A configuration for `n` spins coupled along `edges`, with these defaults (the
    /// settings of `tests/boltzmann_learning.rs`'s gate A, except the seed):
    ///
    /// | field | default |
    /// |---|---|
    /// | `delta_v` | 3.0 |
    /// | `x_0` | 1.0 |
    /// | `gamma` | 10.0 |
    /// | `k_b_t` | 1.0 |
    /// | `learning_rate` | 0.5 |
    /// | `n_steps` | 1 000 000 |
    /// | `n_burn_in` | 20 000 |
    /// | `n_trajectories` | 3 |
    /// | `x_thresh` | 0.5, half the default `x_0`: it does not follow `x_0`, so set both |
    /// | `seed_base` | 0 |
    ///
    /// Nothing is checked here; [`IsingLearner::try_new`] checks the whole configuration.
    #[must_use]
    pub const fn new(n: usize, edges: Vec<(usize, usize)>) -> Self {
        Self {
            n,
            edges,
            delta_v: 3.0,
            x_0: 1.0,
            gamma: 10.0,
            k_b_t: 1.0,
            learning_rate: 0.5,
            n_steps: 1_000_000,
            n_burn_in: 20_000,
            n_trajectories: 3,
            x_thresh: 0.5,
            seed_base: 0,
        }
    }
}

/// Target distribution for the learner.
///
/// Stores both the full probability distribution (for KL computation)
/// and the summary statistics (for the Boltzmann learning rule update).
#[derive(Clone, Debug)]
pub struct IsingTarget {
    /// Per-site target magnetizations `⟨σ_i⟩`.
    pub magnetizations: Vec<f64>,
    /// Per-edge target correlations `⟨σ_i σ_j⟩` (same order as edge list).
    pub correlations: Vec<f64>,
    /// Full target distribution over `2^N` configurations.
    pub distribution: Vec<(u32, f64)>,
}

impl IsingTarget {
    /// Construct from known Ising parameters via exact enumeration.
    ///
    /// # Panics
    /// If [`Self::try_from_ising_params`] refuses the parameters.
    #[must_use]
    #[track_caller]
    pub fn from_ising_params(
        n: usize,
        edges: &[(usize, usize)],
        coupling_j: &[f64],
        field_h: &[f64],
        k_b_t: f64,
    ) -> Self {
        or_panic(Self::try_from_ising_params(
            n, edges, coupling_j, field_h, k_b_t,
        ))
    }

    /// [`Self::from_ising_params`], returning the refusal instead of panicking.
    ///
    /// # Errors
    /// As [`exact_distribution`](crate::ising::exact_distribution) refuses its inputs.
    pub fn try_from_ising_params(
        n: usize,
        edges: &[(usize, usize)],
        coupling_j: &[f64],
        field_h: &[f64],
        k_b_t: f64,
    ) -> Result<Self, ThermostatError> {
        crate::ising::check_problem("IsingTarget", n, edges, coupling_j, field_h, k_b_t)?;
        let dist = crate::ising::exact_distribution(n, edges, coupling_j, field_h, k_b_t);
        let stats = crate::ising::ising_statistics(&dist, n, edges);
        Ok(Self {
            magnetizations: stats.magnetizations,
            correlations: stats.correlations,
            distribution: dist,
        })
    }
}

/// Record of a single learning iteration.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct LearningRecord {
    /// Iteration index (0-based).
    pub iteration: usize,
    /// Per-edge coupling constants at end of this iteration.
    pub coupling_j: Vec<f64>,
    /// Per-site external fields at end of this iteration.
    pub field_h: Vec<f64>,
    /// Measured per-site magnetizations from the physical sampler.
    pub measured_magnetizations: Vec<f64>,
    /// Measured per-edge correlations from the physical sampler.
    pub measured_correlations: Vec<f64>,
    /// KL divergence `KL(target ‖ exact)` at the parameters this iteration's
    /// trajectories ran with: the parameters BEFORE this iteration's update,
    /// so the first record's KL is the starting KL. `coupling_j` and
    /// `field_h` above are the parameters AFTER it.
    pub kl_divergence: f64,
}

/// Boltzmann machine learning loop on a physical Ising sampler.
///
/// The learner owns the [`Model`]. At each iteration it rebuilds and
/// re-installs the [`PassiveStack`] with updated parameters, then creates
/// fresh [`Data`](sim_core::Data) via `model.make_data()`. The caller is
/// responsible for loading the model before constructing the learner.
pub struct IsingLearner {
    config: LearnerConfig,
    target: IsingTarget,
    model: Model,
    coupling_j: Vec<f64>,
    field_h: Vec<f64>,
    iteration: usize,
}

impl IsingLearner {
    /// Create a new learner. Initial parameters: all zeros.
    ///
    /// # Panics
    /// If [`Self::try_new`] refuses the configuration, target or model.
    #[must_use]
    #[track_caller]
    pub fn new(config: LearnerConfig, target: IsingTarget, model: Model) -> Self {
        or_panic(Self::try_new(config, target, model))
    }

    /// [`Self::new`], returning the refusal instead of panicking.
    ///
    /// # Errors
    /// - [`ThermostatError::TooManySpins`] if `config.n` exceeds
    ///   [`MAX_EXACT_SPINS`](crate::ising::MAX_EXACT_SPINS).
    /// - [`ThermostatError::EdgeOutOfRange`] or [`ThermostatError::InvalidEdge`] if an edge
    ///   names a spin outside `0..n`, joins a spin to itself, or repeats a pair.
    /// - [`ThermostatError::DofOutOfRange`] if `model` has fewer than `n` DOFs.
    /// - [`ThermostatError::LengthMismatch`] unless the target has one magnetization per
    ///   spin, one correlation per edge and one probability per configuration (`2^n`).
    /// - [`ThermostatError::ConfigurationOrder`] unless `target.distribution` lists the
    ///   configurations in order (entry `k` is configuration `k`), as
    ///   [`exact_distribution`](crate::ising::exact_distribution) returns them.
    /// - [`ThermostatError::InvalidParameter`] if a target magnetization or correlation is
    ///   not finite, a target probability is not finite and non-negative, `k_b_t` is not
    ///   finite and positive (the exact distribution needs `kT > 0`), `learning_rate` is not
    ///   finite and non-negative, or `x_thresh` is not finite, non-negative and below `x_0`.
    /// - [`ThermostatError::InvalidCount`] unless `n_steps > n_burn_in` (some measured
    ///   steps) and `n_trajectories >= 1`.
    /// - The refusal of a component the learner builds: `delta_v`, `x_0`, `gamma` (see
    ///   [`DoubleWellPotential::try_new`], [`LangevinThermostat::try_new`]), or of the model
    ///   (see [`PassiveStack::validate`]): one of the first `n` DOFs has no position
    ///   coordinate of its own, or the model uses RK4.
    pub fn try_new(
        config: LearnerConfig,
        target: IsingTarget,
        model: Model,
    ) -> Result<Self, ThermostatError> {
        let n = config.n;
        let n_edges = config.edges.len();
        check_problem_shape(COMPONENT, n, &config.edges)?;
        if model.nv < n {
            return Err(ThermostatError::DofOutOfRange {
                component: COMPONENT,
                dof: n - 1,
                nv: model.nv,
            });
        }
        check_target(&target, n, n_edges)?;
        if config.n_steps <= config.n_burn_in {
            return Err(ThermostatError::InvalidCount {
                component: COMPONENT,
                parameter: "n_steps",
                value: config.n_steps,
                requirement: "greater than n_burn_in",
            });
        }
        if config.n_trajectories == 0 {
            return Err(ThermostatError::InvalidCount {
                component: COMPONENT,
                parameter: "n_trajectories",
                value: 0,
                requirement: "at least 1",
            });
        }
        Domain::Positive.check(COMPONENT, "k_b_t", config.k_b_t)?;
        Domain::NonNegative.check(COMPONENT, "learning_rate", config.learning_rate)?;
        let coupling_j = vec![0.0; n_edges];
        let field_h = vec![0.0; n];
        Self::try_build_stack(&config, &coupling_j, &field_h, 0, 0)?.validate(&model)?;
        // After the stack check, which refuses a bad `x_0`.
        if !(Domain::NonNegative.contains(config.x_thresh) && config.x_thresh < config.x_0) {
            return Err(ThermostatError::InvalidParameter {
                component: COMPONENT,
                parameter: "x_thresh".to_owned(),
                value: config.x_thresh,
                requirement: "finite, non-negative and below x_0",
            });
        }
        Ok(Self {
            config,
            target,
            model,
            coupling_j,
            field_h,
            iteration: 0,
        })
    }

    /// Create from explicit initial parameters.
    ///
    /// # Panics
    /// If [`Self::try_with_initial_params`] refuses them.
    #[must_use]
    #[track_caller]
    pub fn with_initial_params(
        config: LearnerConfig,
        target: IsingTarget,
        model: Model,
        initial_j: Vec<f64>,
        initial_h: Vec<f64>,
    ) -> Self {
        or_panic(Self::try_with_initial_params(
            config, target, model, initial_j, initial_h,
        ))
    }

    /// [`Self::with_initial_params`], returning the refusal instead of panicking.
    ///
    /// # Errors
    /// [`ThermostatError::LengthMismatch`] unless `initial_j` has one entry per edge and
    /// `initial_h` one per spin; [`ThermostatError::InvalidParameter`] if an entry is not
    /// finite; otherwise as [`Self::try_new`].
    pub fn try_with_initial_params(
        config: LearnerConfig,
        target: IsingTarget,
        model: Model,
        initial_j: Vec<f64>,
        initial_h: Vec<f64>,
    ) -> Result<Self, ThermostatError> {
        check_len(
            COMPONENT,
            "initial_j",
            initial_j.len(),
            config.edges.len(),
            "edge",
        )?;
        check_len(COMPONENT, "initial_h", initial_h.len(), config.n, "spin")?;
        Domain::Finite.check_each(COMPONENT, "initial_j", &initial_j)?;
        Domain::Finite.check_each(COMPONENT, "initial_h", &initial_h)?;
        let mut learner = Self::try_new(config, target, model)?;
        learner.coupling_j = initial_j;
        learner.field_h = initial_h;
        Ok(learner)
    }

    /// Replace the model's passive stack with one for the current parameters.
    fn install_stack(&mut self, seed: u64, traj_id: u64) {
        let stack = self.build_stack(seed, traj_id);
        self.model.clear_passive_callback();
        // `try_new` validated this stack's components on the model; learning changes only
        // `coupling_j` and `field_h`, which no component's `validate` reads.
        or_panic(stack.try_install(&mut self.model));
    }

    /// The passive stack for the current parameters.
    ///
    /// Panics if a learning update has made a coupling or field non-finite.
    fn build_stack(&self, seed: u64, traj_id: u64) -> Arc<PassiveStack> {
        or_panic(Self::try_build_stack(
            &self.config,
            &self.coupling_j,
            &self.field_h,
            seed,
            traj_id,
        ))
    }

    /// The passive stack for `config` at couplings `coupling_j` and fields `field_h`.
    fn try_build_stack(
        config: &LearnerConfig,
        coupling_j: &[f64],
        field_h: &[f64],
        seed: u64,
        traj_id: u64,
    ) -> Result<Arc<PassiveStack>, ThermostatError> {
        let n = config.n;
        let mut builder = PassiveStack::builder();
        for i in 0..n {
            builder = builder.with(DoubleWellPotential::try_new(config.delta_v, config.x_0, i)?);
        }
        Ok(builder
            .with(PairwiseCoupling::try_new(
                coupling_j.to_vec(),
                config.edges.clone(),
            )?)
            .with(ExternalField::try_new(field_h.to_vec())?)
            .with(LangevinThermostat::try_new(
                DVector::from_element(n, config.gamma),
                config.k_b_t,
                seed,
                traj_id,
            )?)
            .build())
    }

    /// Run a single trajectory and return per-site magnetization means
    /// and per-edge correlation means.
    // Precision loss is acceptable for the sample-count (`mag_count`/
    // `corr_count`) → f64 casts used to average magnetization/correlation.
    // Panics on step/forward failure are intentional — see § Panics.
    #[allow(clippy::cast_precision_loss, clippy::panic)]
    fn run_trajectory(&mut self, seed: u64, traj_id: u64) -> (Vec<f64>, Vec<f64>) {
        let n = self.config.n;
        let n_edges = self.config.edges.len();
        let n_measure = self.config.n_steps - self.config.n_burn_in;

        self.install_stack(seed, traj_id);
        let mut data = self.model.make_data();
        // Each element's position coordinate; `new` checked that DOFs 0..n each have one.
        let x_index: Vec<usize> = (0..n).map(|i| qpos_index(&self.model, i)).collect();

        // Initial condition: all elements in the right well.
        for (i, &xi) in x_index.iter().enumerate() {
            data.qpos[xi] = self.config.x_0;
            data.qvel[i] = 0.0;
        }
        // Infallible with valid MJCF — panic is an intentional safety net.
        if let Err(e) = data.forward(&self.model) {
            panic!("forward failed: {e}");
        }

        // Burn-in.
        for _ in 0..self.config.n_burn_in {
            if let Err(e) = data.step(&self.model) {
                panic!("burn-in step failed: {e}");
            }
        }

        // Measurement.
        let mut mag_sum = vec![0.0_f64; n];
        let mut mag_count = vec![0_usize; n];
        let mut corr_sum = vec![0.0_f64; n_edges];
        let mut corr_count = vec![0_usize; n_edges];

        for _ in 0..n_measure {
            // Infallible with valid MJCF — panic is an intentional safety net.
            if let Err(e) = data.step(&self.model) {
                panic!("measure step failed: {e}");
            }

            let states: Vec<WellState> = x_index
                .iter()
                .map(|&xi| WellState::from_position(data.qpos[xi], self.config.x_thresh))
                .collect();

            for (i, mag) in mag_sum.iter_mut().enumerate() {
                if states[i].is_in_well() {
                    *mag += states[i].spin();
                    mag_count[i] += 1;
                }
            }

            for (k, &(i, j)) in self.config.edges.iter().enumerate() {
                if states[i].is_in_well() && states[j].is_in_well() {
                    corr_sum[k] += states[i].spin() * states[j].spin();
                    corr_count[k] += 1;
                }
            }
        }

        let mags: Vec<f64> = mag_sum
            .iter()
            .zip(&mag_count)
            .map(|(&s, &c)| if c > 0 { s / c as f64 } else { 0.0 })
            .collect();
        let corrs: Vec<f64> = corr_sum
            .iter()
            .zip(&corr_count)
            .map(|(&s, &c)| if c > 0 { s / c as f64 } else { 0.0 })
            .collect();

        (mags, corrs)
    }

    /// Run one learning iteration.
    ///
    /// # Panics
    /// If `data.forward()` or `data.step()` fails (should not happen with valid MJCF
    /// models), or if an update makes a coupling or field non-finite (a learning rate or a
    /// target large enough to overflow `f64`).
    // Precision loss is acceptable for trajectory count / iteration index casting.
    #[allow(clippy::cast_precision_loss)]
    pub fn step(&mut self) -> LearningRecord {
        let n = self.config.n;
        let n_edges = self.config.edges.len();

        // 1. Run trajectories and collect measurements.
        let mut all_mags = vec![vec![]; n];
        let mut all_corrs = vec![vec![]; n_edges];

        for traj in 0..self.config.n_trajectories {
            let (seed, traj_id) = noise_ids(self.config.seed_base, self.iteration, traj);
            let (mags, corrs) = self.run_trajectory(seed, traj_id);
            for (i, m) in mags.into_iter().enumerate() {
                all_mags[i].push(m);
            }
            for (k, c) in corrs.into_iter().enumerate() {
                all_corrs[k].push(c);
            }
        }

        // 2. Aggregate: ensemble mean across trajectories.
        let measured_magnetizations: Vec<f64> = all_mags
            .iter()
            .map(|v| v.iter().sum::<f64>() / v.len() as f64)
            .collect();
        let measured_correlations: Vec<f64> = all_corrs
            .iter()
            .map(|v| v.iter().sum::<f64>() / v.len() as f64)
            .collect();

        // 3. KL divergence at the parameters these trajectories ran with (before the update).
        let current_dist = crate::ising::exact_distribution(
            n,
            &self.config.edges,
            &self.coupling_j,
            &self.field_h,
            self.config.k_b_t,
        );
        let kl = crate::ising::kl_divergence(&self.target.distribution, &current_dist);

        // 4. Update parameters (Boltzmann learning rule).
        for (j, (target_corr, measured_corr)) in self
            .coupling_j
            .iter_mut()
            .zip(self.target.correlations.iter().zip(&measured_correlations))
        {
            *j += self.config.learning_rate * (target_corr - measured_corr);
        }
        for (h, (target_mag, measured_mag)) in self.field_h.iter_mut().zip(
            self.target
                .magnetizations
                .iter()
                .zip(&measured_magnetizations),
        ) {
            *h += self.config.learning_rate * (target_mag - measured_mag);
        }

        let record = LearningRecord {
            iteration: self.iteration,
            coupling_j: self.coupling_j.clone(),
            field_h: self.field_h.clone(),
            measured_magnetizations,
            measured_correlations,
            kl_divergence: kl,
        };

        self.iteration += 1;
        record
    }

    /// Run multiple iterations. Returns the full learning curve.
    pub fn train(&mut self, n_iterations: usize) -> Vec<LearningRecord> {
        (0..n_iterations).map(|_| self.step()).collect()
    }

    /// Current coupling constants.
    #[must_use]
    pub fn coupling_j(&self) -> &[f64] {
        &self.coupling_j
    }

    /// Current external fields.
    #[must_use]
    pub fn field_h(&self) -> &[f64] {
        &self.field_h
    }
}

/// `Ok` if `target` has one finite magnetization per spin, one finite correlation per edge,
/// and one finite, non-negative probability per configuration, in configuration order.
fn check_target(target: &IsingTarget, n: usize, n_edges: usize) -> Result<(), ThermostatError> {
    let distribution = &target.distribution;
    check_len(
        COMPONENT,
        "target.magnetizations",
        target.magnetizations.len(),
        n,
        "spin",
    )?;
    check_len(
        COMPONENT,
        "target.correlations",
        target.correlations.len(),
        n_edges,
        "edge",
    )?;
    check_len(
        COMPONENT,
        "target.distribution",
        distribution.len(),
        1 << n,
        "configuration",
    )?;
    if let Some((entry, &(config, _))) = distribution
        .iter()
        .enumerate()
        .find(|&(k, &(c, _))| c as usize != k)
    {
        return Err(ThermostatError::ConfigurationOrder {
            component: COMPONENT,
            entry,
            config,
        });
    }
    Domain::Finite.check_each(COMPONENT, "target.magnetizations", &target.magnetizations)?;
    Domain::Finite.check_each(COMPONENT, "target.correlations", &target.correlations)?;
    let probabilities: Vec<f64> = distribution.iter().map(|&(_, p)| p).collect();
    Domain::NonNegative.check_each(COMPONENT, "target.distribution", &probabilities)
}

/// The thermostat's `(master_seed, traj_id)` for trajectory `traj` of iteration `iteration`:
/// the seed is `seed_base` itself and the trajectory id packs the iteration above the
/// trajectory, so every pair gets its own noise stream within a run, and runs with different
/// `seed_base` share none (for iteration and trajectory indices below 2^32).
const fn noise_ids(seed_base: u64, iteration: usize, traj: usize) -> (u64, u64) {
    (seed_base, ((iteration as u64) << 32) | traj as u64)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    fn minimal_config() -> LearnerConfig {
        LearnerConfig {
            n: 2,
            edges: vec![(0, 1)],
            delta_v: 5.0,
            x_0: 0.3,
            gamma: 0.1,
            k_b_t: 1.0,
            learning_rate: 0.1,
            n_steps: 20,
            n_burn_in: 5,
            n_trajectories: 1,
            x_thresh: 0.15,
            seed_base: 42,
        }
    }

    fn minimal_target() -> IsingTarget {
        IsingTarget::from_ising_params(2, &[(0, 1)], &[0.5], &[0.0, 0.0], 1.0)
    }

    fn load_model() -> Model {
        sim_core::test_fixtures::ising_pair()
    }

    // ── IsingTarget ────────────────────────────────────────────────────

    #[test]
    fn target_from_ising_params_has_correct_shape() {
        let target = minimal_target();
        assert_eq!(target.magnetizations.len(), 2);
        assert_eq!(target.correlations.len(), 1);
        assert_eq!(target.distribution.len(), 4); // 2^2 = 4 configs
        // Probabilities sum to 1.
        let sum: f64 = target.distribution.iter().map(|(_, p)| p).sum();
        assert!((sum - 1.0).abs() < 1e-12);
    }

    // ── IsingLearner::new ─────────────────────────────────���────────────

    #[test]
    fn new_initializes_zero_params() {
        let learner = IsingLearner::new(minimal_config(), minimal_target(), load_model());
        assert_eq!(learner.coupling_j(), &[0.0]);
        assert_eq!(learner.field_h(), &[0.0, 0.0]);
    }

    #[test]
    #[should_panic(expected = "model has")]
    #[allow(clippy::let_underscore_must_use)]
    fn new_panics_model_too_small() {
        // 1-DOF model with n=2 config → should panic. Body name on the
        // fixture is `p0` rather than `e0`; the test only asserts the
        // "model has" panic message which is name-agnostic.
        let model = sim_core::test_fixtures::single_slide();
        let _ = IsingLearner::new(minimal_config(), minimal_target(), model);
    }

    #[test]
    #[should_panic(
        expected = "IsingLearner: target.correlations has length 2, expected 1 (one per edge)"
    )]
    #[allow(clippy::let_underscore_must_use)]
    fn new_panics_correlations_mismatch() {
        let mut target = minimal_target();
        target.correlations = vec![0.0, 0.0]; // 2 but config has 1 edge
        let _ = IsingLearner::new(minimal_config(), target, load_model());
    }

    #[test]
    #[should_panic(
        expected = "IsingLearner: target.magnetizations has length 1, expected 2 (one per spin)"
    )]
    #[allow(clippy::let_underscore_must_use)]
    fn new_panics_magnetizations_mismatch() {
        let mut target = minimal_target();
        target.magnetizations = vec![0.0]; // 1 but config has n=2
        let _ = IsingLearner::new(minimal_config(), target, load_model());
    }

    // ── with_initial_params ────────────────────────────────────────────

    #[test]
    fn with_initial_params_sets_custom() {
        let learner = IsingLearner::with_initial_params(
            minimal_config(),
            minimal_target(),
            load_model(),
            vec![0.25],
            vec![0.1, -0.1],
        );
        assert_eq!(learner.coupling_j(), &[0.25]);
        assert_eq!(learner.field_h(), &[0.1, -0.1]);
    }

    #[test]
    #[should_panic(expected = "IsingLearner: initial_j has length 2, expected 1 (one per edge)")]
    #[allow(clippy::let_underscore_must_use)]
    fn with_initial_params_panics_j_mismatch() {
        let _ = IsingLearner::with_initial_params(
            minimal_config(),
            minimal_target(),
            load_model(),
            vec![0.1, 0.2], // 2 but 1 edge
            vec![0.0, 0.0],
        );
    }

    #[test]
    #[should_panic(expected = "IsingLearner: initial_h has length 1, expected 2 (one per spin)")]
    #[allow(clippy::let_underscore_must_use)]
    fn with_initial_params_panics_h_mismatch() {
        let _ = IsingLearner::with_initial_params(
            minimal_config(),
            minimal_target(),
            load_model(),
            vec![0.1],
            vec![0.0], // 1 but n=2
        );
    }

    // ── step + train ───────────────────────────────────────────────────

    #[test]
    fn step_returns_valid_record() {
        let mut learner = IsingLearner::new(minimal_config(), minimal_target(), load_model());
        let record = learner.step();
        assert_eq!(record.iteration, 0);
        assert_eq!(record.coupling_j.len(), 1);
        assert_eq!(record.field_h.len(), 2);
        assert_eq!(record.measured_magnetizations.len(), 2);
        assert_eq!(record.measured_correlations.len(), 1);
        assert!(record.kl_divergence >= 0.0);
    }

    #[test]
    fn step_updates_params() {
        let mut learner = IsingLearner::new(minimal_config(), minimal_target(), load_model());
        let j_before = learner.coupling_j().to_vec();
        let h_before = learner.field_h().to_vec();
        learner.step();
        // After one step, at least one parameter should have moved
        // (target is non-trivial and initial params are all zero).
        let j_moved = learner.coupling_j()[0] != j_before[0];
        let h_moved = learner.field_h().iter().zip(&h_before).any(|(a, b)| a != b);
        assert!(
            j_moved || h_moved,
            "parameters should change after step: J {:?} → {:?}, H {:?} → {:?}",
            j_before,
            learner.coupling_j(),
            h_before,
            learner.field_h(),
        );
    }

    #[test]
    fn train_returns_correct_length() {
        let mut learner = IsingLearner::new(minimal_config(), minimal_target(), load_model());
        let curve = learner.train(3);
        assert_eq!(curve.len(), 3);
        assert_eq!(curve[0].iteration, 0);
        assert_eq!(curve[1].iteration, 1);
        assert_eq!(curve[2].iteration, 2);
    }

    // ── input checks, seeds, recorded KL ──────────────────────────────

    #[test]
    #[should_panic(expected = "IsingLearner: n_steps must be greater than n_burn_in, got 5")]
    fn new_refuses_no_measured_steps() {
        let config = LearnerConfig {
            n_steps: 5,
            ..minimal_config()
        };
        let _learner = IsingLearner::new(config, minimal_target(), load_model());
    }

    #[test]
    #[should_panic(expected = "IsingLearner: n_trajectories must be at least 1, got 0")]
    fn new_refuses_zero_trajectories() {
        let config = LearnerConfig {
            n_trajectories: 0,
            ..minimal_config()
        };
        let _learner = IsingLearner::new(config, minimal_target(), load_model());
    }

    #[test]
    #[should_panic(
        expected = "IsingLearner: target.distribution has length 3, expected 4 (one per configuration)"
    )]
    fn new_refuses_a_short_target_distribution() {
        let mut target = minimal_target();
        target.distribution.pop();
        let _learner = IsingLearner::new(minimal_config(), target, load_model());
    }

    /// Trajectory 1000 of one iteration and trajectory 0 of the next used to share a seed.
    #[test]
    fn every_iteration_and_trajectory_gets_its_own_noise() {
        let mut seen = std::collections::HashSet::new();
        for iteration in 0..3 {
            for traj in [0, 1, 999, 1000, 1001] {
                assert!(
                    seen.insert(noise_ids(42, iteration, traj)),
                    "iteration {iteration}, trajectory {traj} repeats a noise stream"
                );
            }
        }
    }

    /// A record's KL is at the parameters the iteration sampled with: the first record's is
    /// the starting KL, and the second's is at the first record's (updated) parameters.
    #[test]
    fn a_record_reports_the_kl_of_the_parameters_it_sampled_with() {
        let target = minimal_target();
        let kl_at = |j: &[f64], h: &[f64]| {
            let dist = crate::ising::exact_distribution(2, &[(0, 1)], j, h, 1.0);
            crate::ising::kl_divergence(&target.distribution, &dist)
        };
        let mut learner = IsingLearner::new(minimal_config(), target.clone(), load_model());
        let first = learner.step();
        assert_eq!(
            first.kl_divergence.to_bits(),
            kl_at(&[0.0], &[0.0, 0.0]).to_bits()
        );
        let second = learner.step();
        assert_eq!(
            second.kl_divergence.to_bits(),
            kl_at(&first.coupling_j, &first.field_h).to_bits()
        );
    }

    #[test]
    #[should_panic(expected = "IsingLearner: edge (1, 0) repeats the pair of an earlier edge")]
    fn new_refuses_a_duplicate_edge() {
        let config = LearnerConfig {
            edges: vec![(0, 1), (1, 0)],
            ..minimal_config()
        };
        let target = IsingTarget {
            correlations: vec![0.0, 0.0],
            ..minimal_target()
        };
        let _learner = IsingLearner::new(config, target, load_model());
    }

    #[test]
    #[should_panic(expected = "LangevinThermostat does not support the RungeKutta4 integrator")]
    fn new_refuses_an_rk4_model() {
        let mut model = load_model();
        model.integrator = sim_core::Integrator::RungeKutta4;
        let _learner = IsingLearner::new(minimal_config(), minimal_target(), model);
    }

    /// The exact distribution needs `kT > 0`; the thermostat alone would take 0, and the
    /// first `step` used to panic after running every trajectory.
    #[test]
    fn try_new_refuses_zero_temperature() {
        let mut config = minimal_config();
        config.k_b_t = 0.0;
        assert_eq!(
            IsingLearner::try_new(config, minimal_target(), load_model()).err(),
            Some(ThermostatError::InvalidParameter {
                component: COMPONENT,
                parameter: "k_b_t".to_owned(),
                value: 0.0,
                requirement: "finite and positive",
            })
        );
    }

    /// A target listing its configurations out of order used to pass `new` and panic at the
    /// first KL computation.
    #[test]
    fn try_new_refuses_a_target_out_of_configuration_order() {
        let mut target = minimal_target();
        target.distribution.swap(1, 2);
        assert_eq!(
            IsingLearner::try_new(minimal_config(), target, load_model()).err(),
            Some(ThermostatError::ConfigurationOrder {
                component: COMPONENT,
                entry: 1,
                config: 2,
            })
        );
    }

    #[test]
    fn try_new_refuses_a_target_that_is_not_finite() {
        let refused = |target: IsingTarget| {
            crate::params::refused_parameter(IsingLearner::try_new(
                minimal_config(),
                target,
                load_model(),
            ))
        };
        let mut target = minimal_target();
        target.magnetizations[1] = f64::NAN;
        assert_eq!(refused(target).as_deref(), Some("target.magnetizations[1]"));
        let mut target = minimal_target();
        target.correlations[0] = f64::INFINITY;
        assert_eq!(refused(target).as_deref(), Some("target.correlations[0]"));
        let mut target = minimal_target();
        target.distribution[3].1 = -0.1;
        assert_eq!(refused(target).as_deref(), Some("target.distribution[3]"));
    }

    #[test]
    fn try_with_initial_params_refuses_initial_values_that_are_not_finite() {
        let refused = |initial_j: Vec<f64>, initial_h: Vec<f64>| {
            crate::params::refused_parameter(IsingLearner::try_with_initial_params(
                minimal_config(),
                minimal_target(),
                load_model(),
                initial_j,
                initial_h,
            ))
        };
        assert_eq!(
            refused(vec![f64::NAN], vec![0.0, 0.0]).as_deref(),
            Some("initial_j[0]")
        );
        assert_eq!(
            refused(vec![0.0], vec![0.0, f64::INFINITY]).as_deref(),
            Some("initial_h[1]")
        );
    }

    /// The defaults `LearnerConfig::new` documents.
    #[test]
    fn learner_config_new_has_the_documented_defaults() {
        let c = LearnerConfig::new(4, vec![(0, 1)]);
        assert_eq!((c.n, c.edges.as_slice()), (4, [(0, 1)].as_slice()));
        assert_eq!(
            (
                c.delta_v,
                c.x_0,
                c.gamma,
                c.k_b_t,
                c.learning_rate,
                c.x_thresh
            ),
            (3.0, 1.0, 10.0, 1.0, 0.5, 0.5)
        );
        assert_eq!(
            (c.n_steps, c.n_burn_in, c.n_trajectories, c.seed_base),
            (1_000_000, 20_000, 3, 0)
        );
    }

    /// `minimal_config` has `x_0 = 0.3`.
    #[test]
    fn try_new_refuses_a_bad_learning_rate_or_threshold() {
        let refused = |edit: fn(&mut LearnerConfig)| {
            let mut config = minimal_config();
            edit(&mut config);
            crate::params::refused_parameter(IsingLearner::try_new(
                config,
                minimal_target(),
                load_model(),
            ))
        };
        assert_eq!(
            refused(|c| c.learning_rate = f64::NAN).as_deref(),
            Some("learning_rate")
        );
        assert_eq!(
            refused(|c| c.learning_rate = -0.1).as_deref(),
            Some("learning_rate")
        );
        assert_eq!(refused(|c| c.x_thresh = -0.1).as_deref(), Some("x_thresh"));
        assert_eq!(refused(|c| c.x_thresh = 0.3).as_deref(), Some("x_thresh"));
        assert_eq!(
            refused(|c| c.x_thresh = f64::NAN).as_deref(),
            Some("x_thresh")
        );
        assert_eq!(refused(|c| c.x_thresh = 0.29), None);
        assert_eq!(refused(|c| c.learning_rate = 0.0), None);
    }

    /// Trajectory `traj` of iteration `iteration` runs on the thermostat stream
    /// `noise_ids(seed_base, iteration, traj)`: `step`'s measurement is the mean of exactly
    /// those trajectories, and they differ, so a stack built with one shared `traj_id`
    /// would not reproduce it.
    #[test]
    fn step_runs_each_trajectory_on_its_own_noise_stream() {
        let mut config = minimal_config();
        // A shallow barrier at kT 1, so the spins flip and the noise shows in the means.
        config.delta_v = 0.1;
        config.x_thresh = 0.01;
        config.n_steps = 20_000;
        config.n_trajectories = 2;
        let seed_base = config.seed_base;
        let learner = || IsingLearner::new(config.clone(), minimal_target(), load_model());
        let record = learner().step();
        let mut solo = learner();
        let (m0, _) =
            solo.run_trajectory(noise_ids(seed_base, 0, 0).0, noise_ids(seed_base, 0, 0).1);
        let (m1, _) =
            solo.run_trajectory(noise_ids(seed_base, 0, 1).0, noise_ids(seed_base, 0, 1).1);
        assert_ne!(m0, m1, "the two trajectories drew the same noise");
        let mean: Vec<f64> = m0.iter().zip(&m1).map(|(a, b)| (a + b) / 2.0).collect();
        assert_eq!(record.measured_magnetizations, mean);
        assert!(
            learner()
                .build_stack(5, 7)
                .components()
                .iter()
                .filter_map(|c| c.as_diagnose())
                .any(|d| d.diagnostic_summary().contains("traj_id=7")),
            "build_stack did not pass its traj_id to the thermostat"
        );
    }

    /// Runs whose `seed_base` differ by one used to share noise an iteration apart.
    #[test]
    fn runs_with_different_seed_bases_share_no_noise() {
        for traj in [0, 1, 1000] {
            assert_ne!(noise_ids(42, 1, traj), noise_ids(43, 0, traj));
        }
    }
}
