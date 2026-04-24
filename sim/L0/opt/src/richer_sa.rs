//! Richer-proposal Simulated Annealing — basic SA augmented with
//! Rechenberg's 1/5 success rule for proposal-width adaptation.
//!
//! This module implements Ch 53 §2.2's richer-proposal SA class
//! commitment: an SA variant whose proposal distribution is
//! *non-stationary during training*.  The non-stationarity comes
//! from a textbook 1/5 success rule (Rechenberg 1973): every
//! `adaptation_window` epochs, the proposal standard deviation is
//! multiplied by `sigma_growth_factor` if the uphill-acceptance
//! rate over the window exceeded `target_uphill_rate = 1/5`, and
//! divided by `sigma_growth_factor` if it fell below.  The target
//! rate of 1/5 is the Rechenberg value and the source of the
//! adaptation-window-based naming.
//!
//! # Class commitment qualification (Ch 53 §2.2)
//!
//! Ch 53 §2.2's boundary is "does the proposal distribution evolve
//! during training?"  The 1/5 rule evolves `current_proposal_std`
//! every `adaptation_window` epochs based on the chain's observed
//! uphill-acceptance rate — yes, the proposal distribution
//! evolves, and `RicherSa` qualifies as a Ch 53 §2.2 variant.
//! Rechenberg's rule was picked over CMA or accept-rate-tracked
//! covariance because it is the simplest member of the class and
//! its one-line update keeps the Baby-Steps-for-new-physics
//! discipline intact.  A future chapter that wants a different
//! adaptation mechanism is free to add another module under the
//! same Ch 53 §2.2 commitment.
//!
//! # Why `name()` returns `"SA"`
//!
//! `sim_opt::analysis::run_rematch` extracts per-replicate rewards
//! from `CompetitionResult` by the hardcoded algorithm names
//! `"CEM"` and `"SA"` (see `analysis.rs:580-583`).  Ch 53 §2.4's
//! measurement commitment is that follow-ups run through the
//! amended `run_rematch` driver from commit `086c04c8`
//! *unchanged*, so the driver stays bit-for-bit identical to
//! Ch 52's basic-SA pipeline.  The cleanest way to honor that
//! commitment is to have `RicherSa::name()` return `"SA"` so the
//! fixture is drop-in; the *fixture filename* and the fixture's
//! `eprintln!` header identify which SA variant actually fired.
//! See `sim/L0/opt/tests/d2c_sr_rematch_richer_sa.rs` for the
//! corresponding fixture.

use std::collections::BTreeMap;
use std::time::Instant;

use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use sim_ml_chassis::{
    Algorithm, ArtifactError, CURRENT_VERSION, EpochMetrics, Policy, PolicyArtifact,
    TrainingBudget, TrainingCheckpoint, VecEnv, collect_episodic_rollout,
};

// ── Hyperparameters ──────────────────────────────────────────────────────

/// Richer-proposal SA hyperparameters.
///
/// Shape parallels [`crate::SaHyperparams`] with four additional
/// knobs for the 1/5 adaptation rule.  The initial values match
/// basic SA's Ch 42 §4.5 defaults so that — at adaptation-window
/// boundary zero — `RicherSa` takes the same first proposal that
/// basic SA would at the same `REMATCH_MASTER_SEED`.
#[derive(Debug, Clone, Copy)]
pub struct RicherSaHyperparams {
    /// Initial Metropolis temperature.  Matches basic SA's Ch 42
    /// §4.5 default of `50.0` for the D2c SR rematch.
    pub initial_temperature: f64,
    /// Initial proposal standard deviation.  `current_proposal_std`
    /// is seeded from this and evolves over training under the
    /// 1/5 rule.  Matches basic SA's Ch 42 §4.5 default of `0.5`.
    pub initial_proposal_std: f64,
    /// Multiplicative cooling decay, geometric schedule, applied
    /// each epoch.  Matches basic SA's Ch 42 §4.4 default of
    /// `0.955`.
    pub cooling_decay: f64,
    /// Floor on the Metropolis temperature.  Matches basic SA's
    /// Ch 42 §4.5 default of `0.005`.
    pub temperature_min: f64,
    /// Maximum environment steps per episode.  Must match CEM's
    /// `max_episode_steps` and basic SA's for compute parity.
    pub max_episode_steps: usize,
    /// Number of epochs between proposal-width updates.  At 5
    /// epochs per update the 1/5 rule has its natural grain —
    /// "1 uphill move in 5 trials" is exactly the target rate.
    /// Shorter windows add noise; longer windows lose
    /// responsiveness.
    pub adaptation_window: usize,
    /// Multiplicative step for each proposal-width update.  1.22
    /// is a common practical choice near the Schumer-Steiglitz
    /// convergence-rate optimum; values below 1.1 adapt too
    /// slowly for a 100-epoch budget, values above 1.5 overshoot.
    pub sigma_growth_factor: f64,
    /// Target uphill-acceptance rate for the 1/5 rule.  The
    /// literal 1/5 = 0.2 is the Rechenberg value for Gaussian
    /// proposals and is the default.  Retained as a field so a
    /// caller can experiment with neighbouring targets without
    /// touching source.
    pub target_uphill_rate: f64,
    /// Lower clamp on `current_proposal_std`.  Protects the chain
    /// from runaway shrinkage when the landscape is locally flat
    /// and uphill moves are rare for reasons unrelated to the
    /// proposal width.
    pub proposal_std_min: f64,
    /// Upper clamp on `current_proposal_std`.  Protects against
    /// runaway growth when the chain sits in a wide basin and
    /// nearly every proposal is uphill.
    pub proposal_std_max: f64,
}

impl RicherSaHyperparams {
    /// The Ch 53 §2.2 defaults — same Ch 42 §4.5 core values as
    /// basic SA plus the 1/5 rule's defaults.
    #[must_use]
    pub const fn ch53_defaults(max_episode_steps: usize) -> Self {
        Self {
            initial_temperature: 50.0,
            initial_proposal_std: 0.5,
            cooling_decay: 0.955,
            temperature_min: 0.005,
            max_episode_steps,
            adaptation_window: 5,
            sigma_growth_factor: 1.22,
            target_uphill_rate: 0.2,
            proposal_std_min: 0.05,
            proposal_std_max: 5.0,
        }
    }
}

// ── RicherSa ─────────────────────────────────────────────────────────────

/// Simulated Annealing with a Rechenberg 1/5 rule proposal-width
/// adapter.
///
/// # Parts
///
/// - [`Policy`] — base trait only, like basic SA and CEM.
/// - Internally tracked `current_proposal_std` — the adapted
///   proposal width, distinct from the fixed-width
///   `SaHyperparams::proposal_std` of basic SA.
pub struct RicherSa {
    policy: Box<dyn Policy>,
    hyperparams: RicherSaHyperparams,
    /// Current accepted params — the Metropolis chain state.
    current_params: Vec<f64>,
    /// Current accepted fitness in the Ch 24 Decision 1 unit.
    current_fitness: f64,
    /// Current Metropolis temperature (decayed each epoch).
    temperature: f64,
    /// Current proposal standard deviation.  Adapted every
    /// `adaptation_window` epochs under the 1/5 rule.
    current_proposal_std: f64,
    /// Uphill-move count inside the current adaptation window.
    /// An "uphill move" is a `proposed_fitness` strictly greater
    /// than the pre-proposal `current_fitness`.  Reset each
    /// adaptation boundary.
    uphill_count_in_window: usize,
    /// Best-seen params (monotone; never regresses).
    best_params: Vec<f64>,
    /// Best-seen fitness.
    best_fitness: f64,
    /// Epoch at which `best_fitness` was first observed.
    best_epoch: usize,
}

impl RicherSa {
    /// Construct a new `RicherSa` with the policy's initial
    /// params as the chain's starting point.
    #[must_use]
    pub fn new(policy: Box<dyn Policy>, hyperparams: RicherSaHyperparams) -> Self {
        let current_params = policy.params().to_vec();
        let best_params = current_params.clone();
        Self {
            policy,
            hyperparams,
            current_params,
            current_fitness: f64::NEG_INFINITY,
            temperature: hyperparams.initial_temperature,
            current_proposal_std: hyperparams.initial_proposal_std,
            uphill_count_in_window: 0,
            best_params,
            best_fitness: f64::NEG_INFINITY,
            best_epoch: 0,
        }
    }

    /// Reconstruct a `RicherSa` instance from a checkpoint.
    ///
    /// Restores `temperature`, `current_fitness`, and
    /// `current_proposal_std` from `algorithm_state` so a resumed
    /// run picks up the adapted proposal width, not the initial
    /// value.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if the policy cannot be
    /// reconstructed from `checkpoint.policy_artifact`.
    pub fn from_checkpoint(
        checkpoint: &TrainingCheckpoint,
        hyperparams: RicherSaHyperparams,
    ) -> Result<Self, ArtifactError> {
        let policy = checkpoint.policy_artifact.to_policy()?;
        let temperature = checkpoint
            .algorithm_state
            .get("temperature")
            .copied()
            .unwrap_or(hyperparams.initial_temperature);
        let current_fitness = checkpoint
            .algorithm_state
            .get("current_fitness")
            .copied()
            .unwrap_or(f64::NEG_INFINITY);
        let current_proposal_std = checkpoint
            .algorithm_state
            .get("current_proposal_std")
            .copied()
            .unwrap_or(hyperparams.initial_proposal_std);
        let current_params = policy.params().to_vec();
        let best_params = checkpoint
            .best_params
            .clone()
            .unwrap_or_else(|| current_params.clone());
        let best_fitness = checkpoint.best_reward.unwrap_or(f64::NEG_INFINITY);
        let best_epoch = checkpoint.best_epoch;
        Ok(Self {
            policy,
            hyperparams,
            current_params,
            current_fitness,
            temperature,
            current_proposal_std,
            uphill_count_in_window: 0,
            best_params,
            best_fitness,
            best_epoch,
        })
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────

fn randn(rng: &mut impl rand::Rng) -> f64 {
    let u1: f64 = 1.0 - rng.random::<f64>();
    let u2: f64 = rng.random::<f64>();
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

// cast_precision_loss: rewards are averaged as `usize → f64`; the
// episode counts come from hyperparameters in the single-digit to
// low-thousands range, well within f64's exact-integer mantissa.
#[allow(clippy::cast_precision_loss)]
fn evaluate_fitness(
    env: &mut VecEnv,
    policy: &mut dyn Policy,
    params: &[f64],
    max_episode_steps: usize,
) -> f64 {
    let n_envs = env.n_envs();
    let rollout = collect_episodic_rollout(
        env,
        &mut |_env_idx, obs| {
            policy.set_params(params);
            policy.forward(obs)
        },
        max_episode_steps,
    );
    let total_reward: f64 = rollout
        .trajectories
        .iter()
        .map(|traj| traj.rewards.iter().sum::<f64>())
        .sum();
    total_reward / n_envs as f64
}

// ── Algorithm trait impl ─────────────────────────────────────────────────

impl Algorithm for RicherSa {
    fn name(&self) -> &'static str {
        // See module doc comment: intentionally "SA" so
        // run_rematch's hardcoded replicate extraction picks up
        // the richer-SA replicates without any amendment to the
        // driver.  Ch 53 §2.4 requires the driver to stay
        // unchanged, so the fixture identifies the variant.
        "SA"
    }

    // Same cast reasoning as `Sa::train`: epoch counters and step indices
    // are usize/i64 → f64 for adaptive sigma + cooling math, far below 2^52.
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
    )]
    fn train(
        &mut self,
        env: &mut VecEnv,
        budget: TrainingBudget,
        seed: u64,
        on_epoch: &dyn Fn(&EpochMetrics),
    ) -> Vec<EpochMetrics> {
        let mut rng = StdRng::seed_from_u64(seed);
        let n_envs = env.n_envs();
        let hp = self.hyperparams;
        let n_epochs = match budget {
            TrainingBudget::Epochs(n) => n,
            TrainingBudget::Steps(s) => s / (n_envs * hp.max_episode_steps).max(1),
        };

        let mut metrics = Vec::with_capacity(n_epochs);

        // Same first-proposal-forced-accept convention as basic
        // SA: self.current_fitness starts at NEG_INFINITY, so
        // epoch 0's delta is +inf and epoch 0's proposal is always
        // accepted.  Basic-SA parity on the first Metropolis step
        // is preserved (though downstream trajectories diverge
        // once the 1/5 rule starts adapting the proposal width).

        for epoch in 0..n_epochs {
            let t0 = Instant::now();

            // 1. Propose — perturb current_params with the
            //    currently-adapted proposal std.
            let sigma = self.current_proposal_std;
            let proposed_params: Vec<f64> = self
                .current_params
                .iter()
                .map(|&p| sigma.mul_add(randn(&mut rng), p))
                .collect();

            // 2. Evaluate.
            let proposed_fitness = evaluate_fitness(
                env,
                &mut *self.policy,
                &proposed_params,
                hp.max_episode_steps,
            );

            // 3. Metropolis accept/reject.  Track "uphill" as
            //    proposed_fitness strictly greater than current —
            //    this is the Rechenberg success signal, independent
            //    of whether the Metropolis RNG ended up accepting
            //    a downhill proposal.  The 1/5 rule adapts on
            //    *finding improvements*, not on raw accept rate.
            //
            //    `was_finite` gates the uphill counter past epoch
            //    0's forced-accept baseline: at epoch 0,
            //    `current_fitness` is still `NEG_INFINITY` from
            //    construction, so `delta = finite - NEG_INF = +INF`
            //    and `is_uphill` is trivially true.  Without the
            //    gate, the first adaptation window would count
            //    that non-sample as a real uphill move, inflating
            //    the rate by one unit and making the first
            //    adaptation decision unable to *shrink*
            //    `proposal_std` regardless of landscape.  Gating
            //    keeps the Metropolis accept branch unchanged (so
            //    epoch 0's forced-accept shape is preserved, same
            //    as basic SA) while excluding the non-sample from
            //    the Rechenberg rate numerator.  The denominator
            //    stays at `adaptation_window`, so the first window
            //    has at most `window - 1` countable samples — a
            //    small bias toward shrink in the first update,
            //    which is the safer failure mode if the initial
            //    proposal_std was badly sized.
            let was_finite = self.current_fitness.is_finite();
            let delta = proposed_fitness - self.current_fitness;
            let is_uphill = delta > 0.0;
            let accept = if is_uphill {
                true
            } else if self.temperature > 0.0 {
                let accept_prob = (delta / self.temperature).exp();
                rng.random::<f64>() < accept_prob
            } else {
                false
            };

            if was_finite && is_uphill {
                self.uphill_count_in_window += 1;
            }

            let accepted_count = usize::from(accept);

            if accept {
                self.current_params = proposed_params;
                self.current_fitness = proposed_fitness;
            }

            // 4. Best-tracking (same convention as basic SA).
            if self.current_fitness > self.best_fitness {
                self.best_fitness = self.current_fitness;
                self.best_params.clone_from(&self.current_params);
                self.best_epoch = epoch;
            }

            // 5. Cool the temperature (geometric schedule).
            self.temperature = (self.temperature * hp.cooling_decay).max(hp.temperature_min);

            // 6. 1/5 rule update — every `adaptation_window`
            //    epochs, re-tune `current_proposal_std`.  The
            //    boundary condition `(epoch + 1) % window == 0`
            //    fires at epoch indices `window-1`, `2*window-1`,
            //    etc., so the first update happens after the
            //    first full window of observations and never on
            //    a partial tail window.
            if hp.adaptation_window > 0 && (epoch + 1) % hp.adaptation_window == 0 {
                let rate = self.uphill_count_in_window as f64 / hp.adaptation_window as f64;
                if rate > hp.target_uphill_rate {
                    self.current_proposal_std *= hp.sigma_growth_factor;
                } else if rate < hp.target_uphill_rate {
                    self.current_proposal_std /= hp.sigma_growth_factor;
                }
                self.current_proposal_std = self
                    .current_proposal_std
                    .clamp(hp.proposal_std_min, hp.proposal_std_max);
                self.uphill_count_in_window = 0;
            }

            // 7. Emit per-epoch metrics.  `mean_reward` is the
            //    chain's current accepted fitness — same convention
            //    as basic SA.  `proposal_std` is recorded in
            //    `extra` so post-hoc diagnostics can confirm the
            //    1/5 rule actually moved the width.
            let mut extra = BTreeMap::new();
            extra.insert("temperature".into(), self.temperature);
            extra.insert("accepted".into(), accepted_count as f64);
            extra.insert("proposed_fitness".into(), proposed_fitness);
            extra.insert("proposal_std".into(), self.current_proposal_std);
            // `uphill` reports the *counted* Rechenberg sample —
            // the same boolean the 1/5 rule aggregates — so
            // post-hoc diagnostics sum this column to reconstruct
            // the counter.  Epoch 0's forced-accept baseline
            // reports 0 under this convention; see the was_finite
            // gate in step 3.
            extra.insert(
                "uphill".into(),
                f64::from(u8::from(was_finite && is_uphill)),
            );

            let em = EpochMetrics {
                epoch,
                mean_reward: self.current_fitness,
                done_count: 0,
                total_steps: n_envs * hp.max_episode_steps,
                wall_time_ms: t0.elapsed().as_millis() as u64,
                extra,
            };
            on_epoch(&em);
            metrics.push(em);

            self.policy.set_params(&self.current_params);
        }

        metrics
    }

    fn policy_artifact(&self) -> PolicyArtifact {
        PolicyArtifact::from_policy(&*self.policy)
    }

    fn best_artifact(&self) -> PolicyArtifact {
        PolicyArtifact {
            version: CURRENT_VERSION,
            descriptor: self.policy.descriptor(),
            params: self.best_params.clone(),
            provenance: None,
        }
    }

    fn checkpoint(&self) -> TrainingCheckpoint {
        TrainingCheckpoint {
            algorithm_name: "SA".into(),
            policy_artifact: self.policy_artifact(),
            critics: vec![],
            optimizer_states: vec![],
            algorithm_state: BTreeMap::from([
                ("temperature".into(), self.temperature),
                ("current_fitness".into(), self.current_fitness),
                ("current_proposal_std".into(), self.current_proposal_std),
            ]),
            best_params: Some(self.best_params.clone()),
            best_reward: if self.best_fitness.is_finite() {
                Some(self.best_fitness)
            } else {
                None
            },
            best_epoch: self.best_epoch,
        }
    }
}

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use sim_ml_chassis::LinearPolicy;
    use sim_rl::reaching_2dof;

    const TEST_N_ENVS: usize = 4;

    fn make_hp() -> RicherSaHyperparams {
        RicherSaHyperparams {
            initial_temperature: 0.5,
            initial_proposal_std: 0.3,
            cooling_decay: 0.95,
            temperature_min: 0.01,
            max_episode_steps: 50,
            adaptation_window: 5,
            sigma_growth_factor: 1.22,
            target_uphill_rate: 0.2,
            proposal_std_min: 0.05,
            proposal_std_max: 3.0,
        }
    }

    fn make_richer_sa() -> (RicherSa, sim_ml_chassis::TaskConfig) {
        let task = reaching_2dof();
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        (RicherSa::new(policy, make_hp()), task)
    }

    #[test]
    fn richer_sa_name_is_sa() {
        let (rsa, _) = make_richer_sa();
        // Intentional: see module doc comment.  The variant runs
        // through run_rematch's CEM/SA-hardcoded replicate
        // extraction by masquerading under the "SA" label; the
        // fixture file name identifies that it is the richer
        // variant.
        assert_eq!(rsa.name(), "SA");
    }

    #[test]
    fn richer_sa_smoke_2dof() {
        let (mut rsa, task) = make_richer_sa();
        let mut env = task.build_vec_env(TEST_N_ENVS, 0).unwrap();

        let metrics = rsa.train(&mut env, TrainingBudget::Epochs(10), 42, &|_| {});

        assert_eq!(metrics.len(), 10);
        for (i, m) in metrics.iter().enumerate() {
            assert_eq!(m.epoch, i);
            assert!(m.total_steps > 0);
            assert!(m.extra.contains_key("temperature"));
            assert!(m.extra.contains_key("accepted"));
            assert!(m.extra.contains_key("proposed_fitness"));
            assert!(m.extra.contains_key("proposal_std"));
            assert!(m.extra.contains_key("uphill"));
        }

        // Temperature should decay.
        assert!(metrics[9].extra["temperature"] < metrics[0].extra["temperature"]);
    }

    #[test]
    fn richer_sa_proposal_std_can_adapt() {
        // Run 10 epochs with adaptation_window = 5 — two updates
        // will fire (after epoch 4 and after epoch 9).  The test
        // doesn't assert the *direction* of the adaptation (that
        // depends on the landscape and the RNG seed, and both
        // directions are valid Rechenberg outcomes), only that
        // the proposal_std recorded in extras is consistent with
        // the 1/5 update rule actually having run: either the
        // value is unchanged (rate == target, rare with a window
        // of 5), or it moved by exactly `sigma_growth_factor` or
        // `1/sigma_growth_factor`, up to the clamp.
        let (mut rsa, task) = make_richer_sa();
        let mut env = task.build_vec_env(TEST_N_ENVS, 0).unwrap();
        let metrics = rsa.train(&mut env, TrainingBudget::Epochs(10), 123, &|_| {});

        let hp = make_hp();
        let initial = hp.initial_proposal_std;
        let c = hp.sigma_growth_factor;

        // After epoch 4 (one window boundary), the recorded
        // proposal_std in epoch 4's metrics must be one of three
        // values: `initial`, `initial * c` (clamped), or
        // `initial / c` (clamped).  Epoch 4's metric is recorded
        // *after* the 1/5 update step (§6 runs before §7 in the
        // train loop).
        let after_first = metrics[4].extra["proposal_std"];
        let candidates = [
            initial,
            (initial * c).clamp(hp.proposal_std_min, hp.proposal_std_max),
            (initial / c).clamp(hp.proposal_std_min, hp.proposal_std_max),
        ];
        assert!(
            candidates.iter().any(|x| (x - after_first).abs() < 1e-12),
            "proposal_std after first window ({after_first}) must \
             match one of the 1/5-rule post-update candidates \
             {candidates:?}"
        );

        // Proposal_std is within clamps at every epoch.
        for m in &metrics {
            let s = m.extra["proposal_std"];
            assert!(s >= hp.proposal_std_min - 1e-12);
            assert!(s <= hp.proposal_std_max + 1e-12);
        }
    }

    #[test]
    fn richer_sa_epoch_0_not_counted_as_uphill() {
        // Regression gate for the was_finite gate on the
        // Rechenberg uphill counter: epoch 0's forced-accept
        // baseline must NOT be counted as a real uphill sample,
        // even though its delta is `+INF` against the
        // `NEG_INFINITY` starting fitness.  Without the gate, the
        // first adaptation window would see uphill_count >= 1
        // trivially, and the first update could never shrink
        // proposal_std regardless of landscape.
        let (mut rsa, task) = make_richer_sa();
        let mut env = task.build_vec_env(TEST_N_ENVS, 0).unwrap();
        let metrics = rsa.train(&mut env, TrainingBudget::Epochs(3), 42, &|_| {});

        // Epoch 0's `uphill` extra reports the *counted* flag
        // (post-gate), so it must be 0.0 under the fix.
        assert_eq!(
            metrics[0].extra["uphill"], 0.0,
            "epoch 0 must not count as a Rechenberg uphill sample"
        );
        // Sanity: the Metropolis `accepted` flag at epoch 0 is
        // still 1 (force-accept preserved), confirming the fix
        // only gates the counter, not the accept branch.
        assert_eq!(
            metrics[0].extra["accepted"], 1.0,
            "epoch 0 must still be force-accepted"
        );
    }

    #[test]
    fn richer_sa_best_tracker_monotone() {
        let (mut rsa, task) = make_richer_sa();
        let mut env = task.build_vec_env(TEST_N_ENVS, 0).unwrap();

        let observed: std::cell::RefCell<Vec<f64>> = std::cell::RefCell::new(Vec::new());
        let metrics = rsa.train(&mut env, TrainingBudget::Epochs(10), 7, &|m| {
            observed.borrow_mut().push(m.mean_reward);
        });
        assert_eq!(metrics.len(), 10);

        let observed = observed.into_inner();
        let max_observed = observed.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let cp = rsa.checkpoint();
        let best = cp.best_reward.expect("best_reward must be Some");
        assert!(best.is_finite());
        assert_eq!(best, max_observed);
        assert_eq!(observed[cp.best_epoch], best);
    }

    #[test]
    fn richer_sa_checkpoint_roundtrip() {
        let (mut rsa, task) = make_richer_sa();
        let mut env = task.build_vec_env(TEST_N_ENVS, 0).unwrap();
        rsa.train(&mut env, TrainingBudget::Epochs(10), 42, &|_| {});

        let cp = rsa.checkpoint();
        assert_eq!(cp.algorithm_name, "SA");
        assert!(cp.critics.is_empty());
        assert!(cp.optimizer_states.is_empty());
        assert!(cp.algorithm_state["temperature"] < 0.5);
        assert!(cp.algorithm_state.contains_key("current_fitness"));
        assert!(cp.algorithm_state.contains_key("current_proposal_std"));

        let rsa2 = RicherSa::from_checkpoint(&cp, make_hp()).unwrap();
        let cp2 = rsa2.checkpoint();
        assert_eq!(
            cp2.algorithm_state["temperature"],
            cp.algorithm_state["temperature"]
        );
        assert_eq!(
            cp2.algorithm_state["current_fitness"],
            cp.algorithm_state["current_fitness"]
        );
        assert_eq!(
            cp2.algorithm_state["current_proposal_std"],
            cp.algorithm_state["current_proposal_std"]
        );
        assert_eq!(cp2.best_reward, cp.best_reward);
        assert_eq!(cp2.best_epoch, cp.best_epoch);
        assert_eq!(rsa2.policy_artifact().params, rsa.policy_artifact().params);
        assert_eq!(rsa2.best_artifact().params, rsa.best_artifact().params);
    }
}
