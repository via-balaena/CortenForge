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

use sim_core::{DVector, Model};

use crate::test_utils::WellState;
use crate::{
    DoubleWellPotential, ExternalField, LangevinThermostat, PairwiseCoupling, PassiveStack,
};

/// Configuration for the Boltzmann learning loop.
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

/// Target distribution for the learner.
///
/// Stores both the full probability distribution (for KL computation)
/// and the summary statistics (for the Boltzmann learning rule update).
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
    #[must_use]
    pub fn from_ising_params(
        n: usize,
        edges: &[(usize, usize)],
        coupling_j: &[f64],
        field_h: &[f64],
        k_b_t: f64,
    ) -> Self {
        let dist = crate::ising::exact_distribution(n, edges, coupling_j, field_h, k_b_t);
        let stats = crate::ising::ising_statistics(&dist, n, edges);
        Self {
            magnetizations: stats.magnetizations,
            correlations: stats.correlations,
            distribution: dist,
        }
    }
}

/// Record of a single learning iteration.
#[derive(Clone, Debug)]
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
    /// KL divergence between exact Ising at current params and target.
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
    /// - If `model.nv < config.n`.
    /// - If `target.correlations.len() != config.edges.len()`.
    /// - If `target.magnetizations.len() != config.n`.
    #[must_use]
    pub fn new(config: LearnerConfig, target: IsingTarget, model: Model) -> Self {
        assert!(
            model.nv >= config.n,
            "model has {} DOFs, need at least {}",
            model.nv,
            config.n,
        );
        assert!(
            target.correlations.len() == config.edges.len(),
            "target correlations length ({}) must match edges length ({})",
            target.correlations.len(),
            config.edges.len(),
        );
        assert!(
            target.magnetizations.len() == config.n,
            "target magnetizations length ({}) must match n ({})",
            target.magnetizations.len(),
            config.n,
        );
        let n_edges = config.edges.len();
        let n = config.n;
        Self {
            config,
            target,
            model,
            coupling_j: vec![0.0; n_edges],
            field_h: vec![0.0; n],
            iteration: 0,
        }
    }

    /// Create from explicit initial parameters.
    ///
    /// # Panics
    /// Same as [`new`](Self::new), plus length checks on initial params.
    #[must_use]
    pub fn with_initial_params(
        config: LearnerConfig,
        target: IsingTarget,
        model: Model,
        initial_j: Vec<f64>,
        initial_h: Vec<f64>,
    ) -> Self {
        assert!(
            initial_j.len() == config.edges.len(),
            "initial_j length ({}) must match edges length ({})",
            initial_j.len(),
            config.edges.len(),
        );
        assert!(
            initial_h.len() == config.n,
            "initial_h length ({}) must match n ({})",
            initial_h.len(),
            config.n,
        );
        let mut learner = Self::new(config, target, model);
        learner.coupling_j = initial_j;
        learner.field_h = initial_h;
        learner
    }

    /// Build and install a `PassiveStack` with the current parameters.
    fn install_stack(&mut self, seed: u64) {
        let n = self.config.n;
        let mut builder = PassiveStack::builder();
        for i in 0..n {
            builder = builder.with(DoubleWellPotential::new(
                self.config.delta_v,
                self.config.x_0,
                i,
            ));
        }
        builder = builder.with(PairwiseCoupling::new(
            self.coupling_j.clone(),
            self.config.edges.clone(),
        ));
        builder = builder.with(ExternalField::new(self.field_h.clone()));
        builder = builder.with(LangevinThermostat::new(
            DVector::from_element(n, self.config.gamma),
            self.config.k_b_t,
            seed,
        ));
        builder.build().install(&mut self.model);
    }

    /// Run a single trajectory and return per-site magnetization means
    /// and per-edge correlation means.
    // Precision loss is acceptable for trajectory/seed index casting.
    // Panics on step/forward failure are intentional — see § Panics.
    #[allow(clippy::cast_precision_loss, clippy::panic)]
    fn run_trajectory(&mut self, seed: u64) -> (Vec<f64>, Vec<f64>) {
        let n = self.config.n;
        let n_edges = self.config.edges.len();
        let n_measure = self.config.n_steps - self.config.n_burn_in;

        self.install_stack(seed);
        let mut data = self.model.make_data();

        // Initial condition: all elements in the right well.
        for i in 0..n {
            data.qpos[i] = self.config.x_0;
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

            let states: Vec<WellState> = (0..n)
                .map(|i| WellState::from_position(data.qpos[i], self.config.x_thresh))
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
    /// Panics if `data.forward()` or `data.step()` fails (should not
    /// happen with valid MJCF models).
    // Precision loss is acceptable for trajectory count / iteration index casting.
    #[allow(clippy::cast_precision_loss)]
    pub fn step(&mut self) -> LearningRecord {
        let n = self.config.n;
        let n_edges = self.config.edges.len();

        // 1. Run trajectories and collect measurements.
        let mut all_mags = vec![vec![]; n];
        let mut all_corrs = vec![vec![]; n_edges];

        for traj in 0..self.config.n_trajectories {
            let seed = self.config.seed_base + self.iteration as u64 * 1000 + traj as u64;
            let (mags, corrs) = self.run_trajectory(seed);
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

        // 3. Compute KL divergence.
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
    ///
    /// Prints per-iteration progress to stderr (visible with
    /// `cargo test -- --nocapture` or when run outside of test harness).
    pub fn train(&mut self, n_iterations: usize) -> Vec<LearningRecord> {
        let mut curve = Vec::with_capacity(n_iterations);
        for i in 0..n_iterations {
            let record = self.step();
            eprintln!(
                "  [IsingLearner] iter {}/{n_iterations}: KL = {:.6}",
                i + 1,
                record.kl_divergence,
            );
            curve.push(record);
        }
        curve
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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    /// Minimal 2-element MJCF model with slide joints and no gravity.
    const MINIMAL_XML: &str = r#"
    <mujoco model="ising_learner_test">
      <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
      <worldbody>
        <body name="e0">
          <joint name="x0" type="slide" axis="1 0 0" damping="0"/>
          <geom type="sphere" size="0.05" mass="1"/>
        </body>
        <body name="e1" pos="0.2 0 0">
          <joint name="x1" type="slide" axis="1 0 0" damping="0"/>
          <geom type="sphere" size="0.05" mass="1"/>
        </body>
      </worldbody>
    </mujoco>"#;

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
        sim_mjcf::load_model(MINIMAL_XML).unwrap()
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
        // 1-DOF model with n=2 config → should panic.
        let xml = r#"
        <mujoco model="small">
          <option timestep="0.001" gravity="0 0 0" integrator="Euler"/>
          <worldbody>
            <body name="e0">
              <joint name="x0" type="slide" axis="1 0 0" damping="0"/>
              <geom type="sphere" size="0.05" mass="1"/>
            </body>
          </worldbody>
        </mujoco>"#;
        let model = sim_mjcf::load_model(xml).unwrap();
        let _ = IsingLearner::new(minimal_config(), minimal_target(), model);
    }

    #[test]
    #[should_panic(expected = "target correlations length")]
    #[allow(clippy::let_underscore_must_use)]
    fn new_panics_correlations_mismatch() {
        let mut target = minimal_target();
        target.correlations = vec![0.0, 0.0]; // 2 but config has 1 edge
        let _ = IsingLearner::new(minimal_config(), target, load_model());
    }

    #[test]
    #[should_panic(expected = "target magnetizations length")]
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
    #[should_panic(expected = "initial_j length")]
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
    #[should_panic(expected = "initial_h length")]
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
}
