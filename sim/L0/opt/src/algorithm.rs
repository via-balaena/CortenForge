//! Simulated Annealing (SA) — gradient-free, single-chain Metropolis.
//!
//! Each epoch, SA perturbs the current policy parameters with a
//! Gaussian proposal, evaluates the proposed fitness by running
//! `n_envs` parallel episodes and averaging the per-episode total
//! rewards, and accepts or rejects the proposal under a Metropolis
//! rule with a geometrically cooling temperature. The chain's
//! accepted state is tracked as `current_params` / `current_fitness`,
//! and a separate best-tracker records the highest accepted fitness
//! seen across all epochs.
//!
//! Requires only [`Policy`] (the base trait) — no gradients.

use std::collections::BTreeMap;
use std::time::Instant;

use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use sim_ml_bridge::{
    Algorithm, ArtifactError, CURRENT_VERSION, EpochMetrics, Policy, PolicyArtifact,
    TrainingBudget, TrainingCheckpoint, VecEnv, collect_episodic_rollout,
};

// ── Hyperparameters ──────────────────────────────────────────────────────

/// Simulated Annealing hyperparameters.
///
/// SA evaluates one candidate policy per epoch (the current chain
/// state perturbed by a Gaussian proposal), averages its reward
/// across `n_envs` parallel episodes, and accepts or rejects the
/// proposal under a Metropolis criterion with a geometric cooling
/// temperature.
#[derive(Debug, Clone, Copy)]
pub struct SaHyperparams {
    /// Initial temperature for the Metropolis accept/reject.
    /// The reward scale of the task sets the right order of
    /// magnitude. The D2c SR rematch uses `50.0`; see Ch 42 §4.5
    /// for the calibration-is-a-guess defense.
    pub initial_temperature: f64,
    /// Standard deviation of the Gaussian proposal, applied
    /// element-wise to the current policy's parameter vector.
    pub proposal_std: f64,
    /// Multiplicative cooling decay applied to the temperature
    /// each epoch: `T_{k+1} = T_k * cooling_decay`. Geometric
    /// schedule. The D2c SR rematch uses `0.955`; see Ch 42 §4.4.
    pub cooling_decay: f64,
    /// Floor on the temperature (prevents degenerate
    /// accept-nothing behavior at very late epochs and guards the
    /// Metropolis denominator against divide-by-zero).
    pub temperature_min: f64,
    /// Maximum environment steps per episode. Matches CEM's
    /// `CemHyperparams::max_episode_steps` — the rollout helper
    /// and the budget-to-epochs formula are shared, so the value
    /// has to match for compute parity between SA and CEM.
    pub max_episode_steps: usize,
}

// ── SA ───────────────────────────────────────────────────────────────────

/// Simulated Annealing algorithm.
///
/// # Parts
///
/// - [`Policy`] — base trait only. SA perturbs `params()` directly
///   and applies Metropolis accept/reject; no gradients needed.
///   Same trait-bound shape as CEM.
///
/// # Constructor
///
/// ```ignore
/// let sa = Sa::new(
///     Box::new(LinearPolicy::new(obs_dim, act_dim, &obs_scale)),
///     SaHyperparams {
///         initial_temperature: 50.0,
///         proposal_std: 0.5,
///         cooling_decay: 0.955,
///         temperature_min: 0.005,
///         max_episode_steps: 5000,
///     },
/// );
/// ```
pub struct Sa {
    policy: Box<dyn Policy>,
    hyperparams: SaHyperparams,
    /// Current accepted params — the state of the Metropolis
    /// chain. Starts at the policy's initial params and moves
    /// with each accepted proposal.
    current_params: Vec<f64>,
    /// Current accepted fitness — the mean per-episode total
    /// reward across `n_envs` trajectories at `current_params`,
    /// in the Ch 24 Decision 1 unit.
    current_fitness: f64,
    /// Current temperature (decayed each epoch).
    temperature: f64,
    /// Best-seen params, separate from the Metropolis chain
    /// state. SA's best is monotone: once seen, it cannot regress.
    best_params: Vec<f64>,
    /// Best-seen fitness in the same unit as `current_fitness`.
    best_fitness: f64,
    /// Epoch index at which `best_fitness` was first observed.
    /// Strict `>` tie-breaking keeps the earlier epoch.
    best_epoch: usize,
}

impl Sa {
    /// Create a new SA instance with the policy's initial params
    /// as the Metropolis chain's starting point.
    #[must_use]
    pub fn new(policy: Box<dyn Policy>, hyperparams: SaHyperparams) -> Self {
        let current_params = policy.params().to_vec();
        let best_params = current_params.clone();
        Self {
            policy,
            hyperparams,
            current_params,
            current_fitness: f64::NEG_INFINITY,
            temperature: hyperparams.initial_temperature,
            best_params,
            best_fitness: f64::NEG_INFINITY,
            best_epoch: 0,
        }
    }

    /// Reconstruct an SA instance from a checkpoint.
    ///
    /// Restores the policy, the Metropolis chain's temperature
    /// and `current_fitness`, and the best-tracker fields.
    /// Hyperparams are provided by the caller — the caller may
    /// resume with different cooling or proposal settings.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if the policy cannot be
    /// reconstructed from the checkpoint's `policy_artifact`.
    pub fn from_checkpoint(
        checkpoint: &TrainingCheckpoint,
        hyperparams: SaHyperparams,
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
            best_params,
            best_fitness,
            best_epoch,
        })
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────

/// Box-Muller normal sample. Inlined to avoid a `rand_distr`
/// dependency. Matches the shape in `sim-ml-bridge`'s `cem.rs`.
fn randn(rng: &mut impl rand::Rng) -> f64 {
    let u1: f64 = 1.0 - rng.random::<f64>();
    let u2: f64 = rng.random::<f64>();
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

/// Evaluate a candidate parameter vector by running `n_envs`
/// parallel episodes and averaging the per-episode total reward.
///
/// The unit is the Ch 24 Decision 1 standardization — mean
/// per-episode total across `n_envs` trajectories — matching the
/// `mean_reward` formula CEM / REINFORCE / PPO / TD3 / SAC all
/// emit post-PR-2b.
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

impl Algorithm for Sa {
    fn name(&self) -> &'static str {
        "SA"
    }

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

        // Note: no baseline evaluation outside the loop. SA's
        // first iteration's Metropolis comparison is against
        // `self.current_fitness = f64::NEG_INFINITY`, which forces
        // accept on the first proposal (delta = +inf). This keeps
        // SA's total env-step count at exactly n_envs *
        // max_episode_steps * n_epochs, matching CEM's budget
        // formula. The initial policy's fitness is never evaluated
        // directly; the first accepted state is the perturbed
        // proposal, which becomes the chain's baseline.

        for epoch in 0..n_epochs {
            let t0 = Instant::now();

            // 1. Propose: perturb `current_params` with a Gaussian
            //    of standard deviation `proposal_std`, element-wise.
            let proposed_params: Vec<f64> = self
                .current_params
                .iter()
                .map(|&p| hp.proposal_std.mul_add(randn(&mut rng), p))
                .collect();

            // 2. Evaluate: run n_envs episodes with the proposed
            //    params and average the per-episode total reward.
            let proposed_fitness = evaluate_fitness(
                env,
                &mut *self.policy,
                &proposed_params,
                hp.max_episode_steps,
            );

            // 3. Metropolis accept/reject. Accept if the proposal
            //    is strictly better, or with probability
            //    exp((proposed - current) / T) if worse.
            let delta = proposed_fitness - self.current_fitness;
            let accept = if delta > 0.0 {
                true
            } else if self.temperature > 0.0 {
                let accept_prob = (delta / self.temperature).exp();
                rng.random::<f64>() < accept_prob
            } else {
                false
            };

            let accepted_count = usize::from(accept);

            if accept {
                self.current_params = proposed_params;
                self.current_fitness = proposed_fitness;
            }

            // 4. Best-tracking. Strict `>` keeps the earlier
            //    epoch on ties — matches `BestTracker`'s
            //    convention.
            if self.current_fitness > self.best_fitness {
                self.best_fitness = self.current_fitness;
                self.best_params.clone_from(&self.current_params);
                self.best_epoch = epoch;
            }

            // 5. Cool the temperature (geometric schedule).
            self.temperature = (self.temperature * hp.cooling_decay).max(hp.temperature_min);

            // 6. Emit per-epoch metrics in the Ch 24 Decision 1
            //    unit (mean per-episode total across n_envs).
            //    SA's `mean_reward` is the current accepted chain
            //    state's fitness, not the proposed fitness — the
            //    chain's trajectory is the sequence of accepted
            //    states. `proposed_fitness` and `accepted` are
            //    recorded in `extra` for post-hoc diagnostics.
            let mut extra = BTreeMap::new();
            extra.insert("temperature".into(), self.temperature);
            extra.insert("accepted".into(), accepted_count as f64);
            extra.insert("proposed_fitness".into(), proposed_fitness);

            let em = EpochMetrics {
                epoch,
                mean_reward: self.current_fitness,
                // Filled by evaluate_fitness in a future revision.
                done_count: 0,
                total_steps: n_envs * hp.max_episode_steps,
                wall_time_ms: t0.elapsed().as_millis() as u64,
                extra,
            };
            on_epoch(&em);
            metrics.push(em);

            // 7. Sync the policy to the accepted chain state so
            //    post-train `policy_artifact` calls reflect it.
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
    use sim_ml_bridge::{LinearPolicy, reaching_2dof};

    // Small debug-mode test config: n_envs=4, max_episode_steps=50.
    // The D2c SR rematch uses n_envs=32 and max_episode_steps=5000
    // under --release; these unit tests just exercise the
    // Algorithm trait surface and should stay fast in debug.
    const TEST_N_ENVS: usize = 4;

    fn make_hp() -> SaHyperparams {
        SaHyperparams {
            initial_temperature: 0.5,
            proposal_std: 0.3,
            cooling_decay: 0.95,
            temperature_min: 0.01,
            max_episode_steps: 50,
        }
    }

    fn make_sa() -> (Sa, sim_ml_bridge::TaskConfig) {
        let task = reaching_2dof();
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        (Sa::new(policy, make_hp()), task)
    }

    #[test]
    fn sa_name() {
        let (sa, _) = make_sa();
        assert_eq!(sa.name(), "SA");
    }

    #[test]
    fn sa_smoke_2dof() {
        let (mut sa, task) = make_sa();
        let mut env = task.build_vec_env(TEST_N_ENVS, 0).unwrap();

        let metrics = sa.train(&mut env, TrainingBudget::Epochs(5), 42, &|_| {});

        assert_eq!(metrics.len(), 5);
        for (i, m) in metrics.iter().enumerate() {
            assert_eq!(m.epoch, i);
            assert!(m.total_steps > 0);
            assert!(m.wall_time_ms < 60_000, "epoch took too long");
            assert!(m.extra.contains_key("temperature"));
            assert!(m.extra.contains_key("accepted"));
            assert!(m.extra.contains_key("proposed_fitness"));
        }

        // Temperature should decay monotonically under geometric
        // cooling with `cooling_decay < 1.0`.
        let t_first = metrics[0].extra["temperature"];
        let t_last = metrics[4].extra["temperature"];
        assert!(
            t_last < t_first,
            "temperature should cool: first={t_first}, last={t_last}"
        );
    }

    #[test]
    fn sa_best_tracker_monotone() {
        let (mut sa, task) = make_sa();
        let mut env = task.build_vec_env(TEST_N_ENVS, 0).unwrap();

        // Collect all chain-state rewards across 20 epochs via
        // the `on_epoch` callback.
        let observed: std::cell::RefCell<Vec<f64>> = std::cell::RefCell::new(Vec::new());
        let metrics = sa.train(&mut env, TrainingBudget::Epochs(10), 7, &|m| {
            observed.borrow_mut().push(m.mean_reward);
        });
        assert_eq!(metrics.len(), 10);

        let observed = observed.into_inner();
        let max_observed = observed.iter().copied().fold(f64::NEG_INFINITY, f64::max);

        // After training, the checkpoint's `best_reward` must
        // equal the maximum observed chain-state reward — the
        // best tracker captured the highest accepted state.
        let cp = sa.checkpoint();
        let best = cp
            .best_reward
            .expect("best_reward must be Some after training");
        assert!(best.is_finite(), "best_reward must be finite");
        assert_eq!(
            best, max_observed,
            "best tracker must match max observed chain state"
        );

        // `best_epoch` must point at an epoch with that reward.
        assert_eq!(
            observed[cp.best_epoch], best,
            "best_epoch must index the best-fitness epoch"
        );
    }

    #[test]
    fn sa_checkpoint_roundtrip() {
        let (mut sa, task) = make_sa();
        let mut env = task.build_vec_env(TEST_N_ENVS, 0).unwrap();
        sa.train(&mut env, TrainingBudget::Epochs(5), 42, &|_| {});

        let cp = sa.checkpoint();
        assert_eq!(cp.algorithm_name, "SA");
        assert!(cp.critics.is_empty());
        assert!(cp.optimizer_states.is_empty());
        // Temperature cooled from the initial value.
        assert!(cp.algorithm_state["temperature"] < 0.5);
        assert!(cp.algorithm_state.contains_key("current_fitness"));

        let sa2 = Sa::from_checkpoint(&cp, make_hp()).unwrap();
        let cp2 = sa2.checkpoint();

        // Round-tripped state matches on every public surface.
        assert_eq!(
            cp2.algorithm_state["temperature"],
            cp.algorithm_state["temperature"]
        );
        assert_eq!(
            cp2.algorithm_state["current_fitness"],
            cp.algorithm_state["current_fitness"]
        );
        assert_eq!(cp2.best_reward, cp.best_reward);
        assert_eq!(cp2.best_epoch, cp.best_epoch);
        assert_eq!(sa2.policy_artifact().params, sa.policy_artifact().params);
        assert_eq!(sa2.best_artifact().params, sa.best_artifact().params);
    }
}
