//! Cross-Entropy Method (CEM) — evolutionary, gradient-free.
//!
//! Each generation, CEM perturbs the current policy parameters with Gaussian
//! noise (one candidate per environment), evaluates fitness via episodic
//! rollout, selects the top fraction as elites, and sets the policy to the
//! elite mean.  No gradients, no optimizer — just perturbation and selection.
//!
//! Requires only [`Policy`] (the base trait).

use std::collections::BTreeMap;
use std::time::Instant;

use rand::SeedableRng;
use rand::rngs::StdRng;

use sim_ml_chassis::algorithm::{Algorithm, EpochMetrics, TrainingBudget};
use sim_ml_chassis::artifact::{ArtifactError, PolicyArtifact, TrainingCheckpoint};
use sim_ml_chassis::policy::Policy;
use sim_ml_chassis::rollout::collect_episodic_rollout;
use sim_ml_chassis::vec_env::VecEnv;

// ── Hyperparameters ──────────────────────────────────────────────────────

/// CEM hyperparameters.
///
/// Population size equals `env.n_envs()` — one candidate per environment.
#[derive(Debug, Clone, Copy)]
pub struct CemHyperparams {
    /// Fraction of population to select as elites (e.g., 0.2 = top 20%).
    pub elite_fraction: f64,
    /// Initial standard deviation for parameter perturbations.
    pub noise_std: f64,
    /// Multiplicative decay applied to `noise_std` each generation.
    pub noise_decay: f64,
    /// Minimum noise standard deviation (floor).
    pub noise_min: f64,
    /// Maximum environment steps per episode.
    pub max_episode_steps: usize,
}

// ── CEM ──────────────────────────────────────────────────────────────────

/// Cross-Entropy Method algorithm.
///
/// # Parts
///
/// - [`Policy`] — base trait only.  CEM perturbs `params()` directly;
///   no gradients needed.
///
/// # Constructor
///
/// ```ignore
/// let cem = Cem::new(
///     Box::new(LinearPolicy::new(obs_dim, act_dim, &obs_scale)),
///     CemHyperparams { elite_fraction: 0.2, noise_std: 0.3, .. },
/// );
/// ```
pub struct Cem {
    policy: Box<dyn Policy>,
    hyperparams: CemHyperparams,
    /// Current noise standard deviation (decayed each generation).
    noise_std: f64,
    /// Best-epoch policy snapshot.
    best: sim_ml_chassis::best_tracker::BestTracker,
}

impl Cem {
    /// Create a new CEM instance.
    #[must_use]
    pub fn new(policy: Box<dyn Policy>, hyperparams: CemHyperparams) -> Self {
        let noise_std = hyperparams.noise_std;
        let best = sim_ml_chassis::best_tracker::BestTracker::new(policy.params());
        Self {
            policy,
            hyperparams,
            noise_std,
            best,
        }
    }

    /// Reconstruct a CEM instance from a checkpoint.
    ///
    /// Restores the policy and `noise_std`. Hyperparams are provided by the
    /// caller — you might want to resume with different decay settings.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if the policy can't be reconstructed.
    pub fn from_checkpoint(
        checkpoint: &TrainingCheckpoint,
        hyperparams: CemHyperparams,
    ) -> Result<Self, ArtifactError> {
        let policy = checkpoint.policy_artifact.to_policy()?;
        let noise_std = checkpoint
            .algorithm_state
            .get("noise_std")
            .copied()
            .unwrap_or(hyperparams.noise_std);
        let best = sim_ml_chassis::best_tracker::BestTracker::from_checkpoint(
            checkpoint.best_params.clone(),
            checkpoint.best_reward,
            checkpoint.best_epoch,
            &checkpoint.policy_artifact.params,
        );
        Ok(Self {
            policy,
            hyperparams,
            noise_std,
            best,
        })
    }
}

/// Box-Muller normal sample.
fn randn(rng: &mut impl rand::Rng) -> f64 {
    let u1: f64 = 1.0 - rng.random::<f64>();
    let u2: f64 = rng.random::<f64>();
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

impl Algorithm for Cem {
    fn name(&self) -> &'static str {
        "CEM"
    }

    // Population size, elite count, and epoch indices are usize → f64 for
    // CEM's distribution updates; population sizes stay far below 2^52.
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
    )]
    // The training loop panics on internal invariant violations (e.g. empty
    // elite set after sorting) which represent CEM-specific bugs, not
    // recoverable runtime conditions — see `# Panics` on the trait method.
    #[allow(clippy::panic)]
    fn train(
        &mut self,
        env: &mut VecEnv,
        budget: TrainingBudget,
        seed: u64,
        on_epoch: &dyn Fn(&EpochMetrics),
    ) -> Vec<EpochMetrics> {
        let mut rng = StdRng::seed_from_u64(seed);
        let n_envs = env.n_envs();
        let n_epochs = match budget {
            TrainingBudget::Epochs(n) => n,
            TrainingBudget::Steps(s) => s / (n_envs * self.hyperparams.max_episode_steps).max(1),
        };

        let hp = self.hyperparams;
        let n_elites = ((hp.elite_fraction * n_envs as f64).floor() as usize).max(1);
        let mut metrics = Vec::with_capacity(n_epochs);

        // Current best params (the "mean" of the elite distribution).
        let mut mean_params = self.policy.params().to_vec();

        for epoch in 0..n_epochs {
            let t0 = Instant::now();

            // Generate population: one perturbation per env.
            let mut population: Vec<Vec<f64>> = Vec::with_capacity(n_envs);
            for _ in 0..n_envs {
                let candidate: Vec<f64> = mean_params
                    .iter()
                    .map(|&p| self.noise_std.mul_add(randn(&mut rng), p))
                    .collect();
                population.push(candidate);
            }

            // Evaluate: run one episode per env, each with its own candidate.
            let rollout = collect_episodic_rollout(
                env,
                &mut |env_idx, obs| {
                    self.policy.set_params(&population[env_idx]);
                    self.policy.forward(obs)
                },
                hp.max_episode_steps,
            );

            // Compute fitness = mean reward per trajectory.
            let mut fitness: Vec<(usize, f64)> = rollout
                .trajectories
                .iter()
                .enumerate()
                .map(|(i, traj)| {
                    let total: f64 = traj.rewards.iter().sum();
                    (i, total / traj.len().max(1) as f64)
                })
                .collect();

            // Select elites (highest fitness).
            fitness.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
            let elites = &fitness[..n_elites];

            let elite_mean_reward = elites.iter().map(|(_, f)| f).sum::<f64>() / n_elites as f64;

            // Update mean params = average of elite params.
            mean_params.fill(0.0);
            for &(idx, _) in elites {
                for (j, p) in population[idx].iter().enumerate() {
                    mean_params[j] += p;
                }
            }
            for p in &mut mean_params {
                *p /= n_elites as f64;
            }

            // Set policy to the new mean.
            self.policy.set_params(&mean_params);

            // Epoch stats.
            let epoch_steps: usize = rollout.trajectories.iter().map(Trajectory::len).sum();
            // Per-episode total across the n_envs trajectories (Ch 24 Decision 1).
            // The length-normalized `fitness` above is CEM's internal elite-selection
            // metric and remains in per-step units; the reported `mean_reward` unit
            // is per-episode-total to match REINFORCE/PPO/TD3/SAC.
            let total_reward: f64 = rollout
                .trajectories
                .iter()
                .map(|t| t.rewards.iter().sum::<f64>())
                .sum();
            let mean_reward = total_reward / n_envs as f64;
            let done_count = rollout.trajectories.iter().filter(|t| t.done).count();

            self.best
                .maybe_update(epoch, mean_reward, self.policy.params());

            self.noise_std = (self.noise_std * hp.noise_decay).max(hp.noise_min);

            let mut extra = BTreeMap::new();
            extra.insert("noise_std".into(), self.noise_std);
            extra.insert("elite_mean_reward_per_step".into(), elite_mean_reward);

            let em = EpochMetrics {
                epoch,
                mean_reward,
                done_count,
                total_steps: epoch_steps,
                wall_time_ms: t0.elapsed().as_millis() as u64,
                extra,
            };
            on_epoch(&em);
            metrics.push(em);
        }

        metrics
    }

    fn policy_artifact(&self) -> PolicyArtifact {
        PolicyArtifact::from_policy(&*self.policy)
    }

    fn best_artifact(&self) -> PolicyArtifact {
        self.best.to_artifact(self.policy.descriptor())
    }

    fn checkpoint(&self) -> TrainingCheckpoint {
        let (best_params, best_reward, best_epoch) = self.best.to_checkpoint();
        TrainingCheckpoint {
            algorithm_name: "CEM".into(),
            policy_artifact: self.policy_artifact(),
            critics: vec![],
            optimizer_states: vec![],
            algorithm_state: BTreeMap::from([("noise_std".into(), self.noise_std)]),
            best_params: Some(best_params),
            best_reward,
            best_epoch,
        }
    }
}

// ── use rollout::Trajectory for len() ────────────────────────────────────
use sim_ml_chassis::rollout::Trajectory;

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use sim_ml_chassis::LinearPolicy;

    use crate::tasks::reaching_2dof;

    #[test]
    fn cem_name() {
        let task = reaching_2dof();
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        let cem = Cem::new(
            policy,
            CemHyperparams {
                elite_fraction: 0.2,
                noise_std: 0.3,
                noise_decay: 0.95,
                noise_min: 0.01,
                max_episode_steps: 300,
            },
        );
        assert_eq!(cem.name(), "CEM");
    }

    #[test]
    fn cem_smoke_2dof() {
        let task = reaching_2dof();
        let mut env = task.build_vec_env(10, 0).unwrap();
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        let mut cem = Cem::new(
            policy,
            CemHyperparams {
                elite_fraction: 0.2,
                noise_std: 0.3,
                noise_decay: 0.95,
                noise_min: 0.01,
                max_episode_steps: 300,
            },
        );

        let metrics = cem.train(&mut env, TrainingBudget::Epochs(5), 42, &|_| {});

        assert_eq!(metrics.len(), 5);
        for (i, m) in metrics.iter().enumerate() {
            assert_eq!(m.epoch, i);
            assert!(m.total_steps > 0);
            assert!(m.extra.contains_key("noise_std"));
            assert!(m.extra.contains_key("elite_mean_reward_per_step"));
        }

        // Reward should not get worse over epochs (elite selection drives it up).
        let first = metrics[0].mean_reward;
        let last = metrics[4].mean_reward;
        assert!(
            last >= first - 100.0,
            "CEM should not regress catastrophically: first={first}, last={last}"
        );
    }

    #[test]
    fn cem_extra_metrics_populated() {
        let task = reaching_2dof();
        let mut env = task.build_vec_env(10, 0).unwrap();
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        let mut cem = Cem::new(
            policy,
            CemHyperparams {
                elite_fraction: 0.3,
                noise_std: 0.5,
                noise_decay: 1.0,
                noise_min: 0.5,
                max_episode_steps: 100,
            },
        );

        let metrics = cem.train(&mut env, TrainingBudget::Epochs(2), 0, &|_| {});
        for m in &metrics {
            assert!(m.extra["noise_std"] >= 0.0);
            assert!(m.extra["elite_mean_reward_per_step"].is_finite());
        }
    }

    // ── Artifact / checkpoint tests ──────────────────────────────────

    fn make_cem() -> (Cem, sim_ml_chassis::TaskConfig) {
        let task = reaching_2dof();
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        let hp = CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 0.3,
            noise_decay: 0.95,
            noise_min: 0.01,
            max_episode_steps: 300,
        };
        (Cem::new(policy, hp), task)
    }

    #[test]
    fn cem_policy_artifact_valid() {
        let (cem, _) = make_cem();
        let artifact = cem.policy_artifact();
        artifact.validate().unwrap();
    }

    #[test]
    fn cem_checkpoint_before_train() {
        let (cem, _) = make_cem();
        let cp = cem.checkpoint();
        assert_eq!(cp.algorithm_name, "CEM");
        assert_eq!(cp.algorithm_state["noise_std"], 0.3);
        assert!(cp.critics.is_empty());
        assert!(cp.optimizer_states.is_empty());
    }

    #[test]
    fn cem_checkpoint_round_trip() {
        let (mut cem, task) = make_cem();
        let mut env = task.build_vec_env(10, 0).unwrap();
        cem.train(&mut env, TrainingBudget::Epochs(3), 42, &|_| {});

        let cp = cem.checkpoint();
        assert!(cp.algorithm_state["noise_std"] < 0.3); // decayed

        let hp = CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 0.3,
            noise_decay: 0.95,
            noise_min: 0.01,
            max_episode_steps: 300,
        };
        let cem2 = Cem::from_checkpoint(&cp, hp).unwrap();
        assert_eq!(cem2.noise_std, cp.algorithm_state["noise_std"]);
        assert_eq!(cem2.policy.params(), cem.policy.params());
    }

    // ── Ch 24 Decision 1 regression guards ──────────────────────────────

    /// CEM's reported `mean_reward` is in per-episode-total units, not
    /// per-step units. A reaching task with ~-1/step rewards over
    /// ~100-300 steps produces per-episode totals with |value| in the
    /// hundreds; a per-step mean would be bounded by a small constant.
    /// The `> 10.0` gate sits in the gap between the two semantics.
    #[test]
    fn cem_mean_reward_is_per_episode_total() {
        let task = reaching_2dof();
        let mut env = task.build_vec_env(10, 0).unwrap();
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        let mut cem = Cem::new(
            policy,
            CemHyperparams {
                elite_fraction: 0.2,
                noise_std: 0.3,
                noise_decay: 0.95,
                noise_min: 0.01,
                max_episode_steps: 300,
            },
        );

        let metrics = cem.train(&mut env, TrainingBudget::Epochs(3), 42, &|_| {});
        let last = metrics.last().unwrap();
        assert!(
            last.mean_reward.abs() > 10.0,
            "CEM mean_reward should be per-episode-total (|.| > 10 for reaching_2dof at max 300 steps), got {}",
            last.mean_reward
        );
    }

    /// CEM carries two reward concepts under Decision 1: the internal
    /// length-normalized `fitness` drives elite selection in per-step
    /// units and is surfaced as `extra.elite_mean_reward_per_step`; the
    /// reported `mean_reward` is per-episode-total. The per-episode
    /// magnitude must exceed the per-step magnitude by an order of
    /// magnitude for any reasonable episode length. A future refactor
    /// that accidentally folds the reporting site back into the fitness
    /// loop would fail this gate.
    #[test]
    fn cem_dual_reward_concept_split() {
        let task = reaching_2dof();
        let mut env = task.build_vec_env(10, 0).unwrap();
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        let mut cem = Cem::new(
            policy,
            CemHyperparams {
                elite_fraction: 0.2,
                noise_std: 0.3,
                noise_decay: 0.95,
                noise_min: 0.01,
                max_episode_steps: 300,
            },
        );

        let metrics = cem.train(&mut env, TrainingBudget::Epochs(3), 42, &|_| {});
        let last = metrics.last().unwrap();
        let per_step = last.extra["elite_mean_reward_per_step"];
        assert!(
            last.mean_reward.abs() > per_step.abs() * 10.0,
            "per-episode-total |mean_reward|={} must exceed per-step |elite_mean_reward_per_step|={} by >10x",
            last.mean_reward.abs(),
            per_step.abs()
        );
    }
}
