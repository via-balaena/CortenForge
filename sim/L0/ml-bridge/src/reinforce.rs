//! REINFORCE — policy gradient from episodic returns.
//!
//! Vanilla REINFORCE with mean-return baseline and advantage normalization.
//! Collects episodic rollouts, computes discounted returns, normalizes
//! advantages, and updates the policy via the score function gradient.
//!
//! Requires [`DifferentiablePolicy`] + [`OptimizerConfig`].
//!
//! Key findings from stress-testing (see `project_reinforce_findings.md`):
//! - Adam optimizer is essential (vanilla SGD oscillates on noisy PG).
//! - Advantage normalization (zero-mean, unit-var) is critical for stability.

use std::collections::BTreeMap;
use std::time::Instant;

use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::algorithm::{Algorithm, EpochMetrics, TrainingBudget};
use crate::artifact::{ArtifactError, PolicyArtifact, TrainingCheckpoint};
use crate::optimizer::OptimizerConfig;
use crate::policy::DifferentiablePolicy;
use crate::rollout::collect_episodic_rollout;
use crate::vec_env::VecEnv;

// ── Hyperparameters ──────────────────────────────────────────────────────

/// REINFORCE hyperparameters.
#[derive(Debug, Clone, Copy)]
pub struct ReinforceHyperparams {
    /// Discount factor (typically 0.99).
    pub gamma: f64,
    /// Initial exploration noise σ.
    pub sigma_init: f64,
    /// Multiplicative decay per epoch.
    pub sigma_decay: f64,
    /// Minimum σ floor.
    pub sigma_min: f64,
    /// Maximum environment steps per episode.
    pub max_episode_steps: usize,
}

// ── REINFORCE ────────────────────────────────────────────────────────────

/// REINFORCE algorithm with mean-return baseline.
///
/// # Parts
///
/// - [`DifferentiablePolicy`] — gradient computation via `log_prob_gradient`.
/// - [`OptimizerConfig`] — optimizer (Adam recommended).
///
/// # Constructor
///
/// ```ignore
/// let reinforce = Reinforce::new(
///     Box::new(LinearPolicy::new(obs_dim, act_dim, &obs_scale)),
///     OptimizerConfig::adam(0.05),
///     ReinforceHyperparams { gamma: 0.99, sigma_init: 0.5, .. },
/// );
/// ```
pub struct Reinforce {
    policy: Box<dyn DifferentiablePolicy>,
    #[allow(dead_code)] // kept for from_checkpoint() reconstruction
    optimizer_config: OptimizerConfig,
    hyperparams: ReinforceHyperparams,
    /// Optimizer instance (momentum persists across `train()` calls).
    optimizer: Box<dyn crate::optimizer::Optimizer>,
    /// Current exploration noise σ (decayed each epoch).
    sigma: f64,
    /// Best-epoch policy snapshot.
    best: crate::best_tracker::BestTracker,
}

impl Reinforce {
    /// Create a new REINFORCE instance.
    #[must_use]
    pub fn new(
        policy: Box<dyn DifferentiablePolicy>,
        optimizer_config: OptimizerConfig,
        hyperparams: ReinforceHyperparams,
    ) -> Self {
        let optimizer = optimizer_config.build(policy.n_params());
        let sigma = hyperparams.sigma_init;
        let best = crate::best_tracker::BestTracker::new(policy.params());
        Self {
            policy,
            optimizer_config,
            hyperparams,
            optimizer,
            sigma,
            best,
        }
    }

    /// Reconstruct a REINFORCE instance from a checkpoint.
    ///
    /// Restores the policy, optimizer momentum, and sigma.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if the policy can't be reconstructed.
    pub fn from_checkpoint(
        checkpoint: &TrainingCheckpoint,
        optimizer_config: OptimizerConfig,
        hyperparams: ReinforceHyperparams,
    ) -> Result<Self, ArtifactError> {
        let policy = checkpoint.policy_artifact.to_differentiable_policy()?;
        let mut optimizer = optimizer_config.build(policy.n_params());
        if let Some(snap) = checkpoint
            .optimizer_states
            .iter()
            .find(|s| s.role == "actor")
        {
            optimizer.load_snapshot(snap);
        }
        let sigma = checkpoint
            .algorithm_state
            .get("sigma")
            .copied()
            .unwrap_or(hyperparams.sigma_init);
        let best = crate::best_tracker::BestTracker::from_checkpoint(
            checkpoint.best_params.clone(),
            checkpoint.best_reward,
            checkpoint.best_epoch,
            &checkpoint.policy_artifact.params,
        );
        Ok(Self {
            policy,
            optimizer_config,
            hyperparams,
            optimizer,
            sigma,
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

impl Algorithm for Reinforce {
    fn name(&self) -> &'static str {
        "REINFORCE"
    }

    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::too_many_lines,
        clippy::panic
    )]
    fn train(
        &mut self,
        env: &mut VecEnv,
        budget: TrainingBudget,
        seed: u64,
        on_epoch: &dyn Fn(&EpochMetrics),
    ) -> Vec<EpochMetrics> {
        let mut rng = StdRng::seed_from_u64(seed);
        let n_params = self.policy.n_params();
        let n_envs = env.n_envs();
        let hp = self.hyperparams;

        let n_epochs = match budget {
            TrainingBudget::Epochs(n) => n,
            TrainingBudget::Steps(s) => s / (n_envs * hp.max_episode_steps).max(1),
        };

        let mut metrics = Vec::with_capacity(n_epochs);

        for epoch in 0..n_epochs {
            let t0 = Instant::now();

            // Collect one episode per env with stochastic actions.
            let sigma = self.sigma; // local copy for closure
            let rollout = collect_episodic_rollout(
                env,
                &mut |_env_idx, obs| {
                    let mu = self.policy.forward(obs);
                    mu.iter()
                        .map(|&m| sigma.mul_add(randn(&mut rng), m))
                        .collect()
                },
                hp.max_episode_steps,
            );

            // Compute discounted returns per step, per trajectory.
            let mut all_returns: Vec<f64> = Vec::new();
            let mut all_obs: Vec<Vec<f32>> = Vec::new();
            let mut all_actions: Vec<Vec<f64>> = Vec::new();

            for traj in &rollout.trajectories {
                let n = traj.len();
                if n == 0 {
                    continue;
                }
                // Backward pass: G_t = r_t + γ G_{t+1}
                let mut returns = vec![0.0; n];
                returns[n - 1] = traj.rewards[n - 1];
                for t in (0..n - 1).rev() {
                    returns[t] = hp.gamma.mul_add(returns[t + 1], traj.rewards[t]);
                }
                all_returns.extend_from_slice(&returns);
                all_obs.extend_from_slice(&traj.obs);
                all_actions.extend_from_slice(&traj.actions);
            }

            let n_samples = all_returns.len();
            if n_samples == 0 {
                continue;
            }

            // Advantage = return - mean_return, then normalize.
            let mean_return = all_returns.iter().sum::<f64>() / n_samples as f64;
            let mut advantages: Vec<f64> = all_returns.iter().map(|&r| r - mean_return).collect();
            let std_adv =
                (advantages.iter().map(|&a| a * a).sum::<f64>() / n_samples as f64).sqrt();
            if std_adv > 1e-8 {
                for a in &mut advantages {
                    *a /= std_adv;
                }
            }

            // Accumulate policy gradient: (1/N) Σ advantage * ∇log π(a|s).
            let mut grad = vec![0.0; n_params];
            for i in 0..n_samples {
                let lpg = self
                    .policy
                    .log_prob_gradient(&all_obs[i], &all_actions[i], sigma);
                for (g, &lg) in grad.iter_mut().zip(&lpg) {
                    *g = advantages[i].mul_add(lg, *g);
                }
            }
            let inv_n = 1.0 / n_samples as f64;
            for g in &mut grad {
                *g *= inv_n;
            }

            let grad_norm = grad.iter().map(|g| g * g).sum::<f64>().sqrt();

            // Adam ascent step (in-place on policy params).
            let mut p = self.policy.params().to_vec();
            self.optimizer.step_in_place(&mut p, &grad, true);
            self.policy.set_params(&p);

            // Decay sigma.
            self.sigma = (self.sigma * hp.sigma_decay).max(hp.sigma_min);

            // Epoch stats.
            let epoch_steps: usize = rollout
                .trajectories
                .iter()
                .map(crate::rollout::Trajectory::len)
                .sum();
            let total_reward: f64 = rollout
                .trajectories
                .iter()
                .map(|t| t.rewards.iter().sum::<f64>())
                .sum();
            let mean_reward = total_reward / n_envs as f64;
            let done_count = rollout.trajectories.iter().filter(|t| t.done).count();

            self.best
                .maybe_update(epoch, mean_reward, self.policy.params());

            let mut extra = BTreeMap::new();
            extra.insert("sigma".into(), self.sigma);
            extra.insert("policy_grad_norm".into(), grad_norm);

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
            algorithm_name: "REINFORCE".into(),
            policy_artifact: self.policy_artifact(),
            critics: vec![],
            optimizer_states: vec![self.optimizer.snapshot("actor")],
            algorithm_state: BTreeMap::from([("sigma".into(), self.sigma)]),
            best_params: Some(best_params),
            best_reward,
            best_epoch,
        }
    }
}

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::{LinearPolicy, reaching_2dof};

    fn make_reinforce() -> (Reinforce, crate::TaskConfig) {
        let task = reaching_2dof();
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        let algo = Reinforce::new(
            policy,
            OptimizerConfig::adam(0.05),
            ReinforceHyperparams {
                gamma: 0.99,
                sigma_init: 0.5,
                sigma_decay: 0.95,
                sigma_min: 0.05,
                max_episode_steps: 300,
            },
        );
        (algo, task)
    }

    #[test]
    fn reinforce_name() {
        let (algo, _) = make_reinforce();
        assert_eq!(algo.name(), "REINFORCE");
    }

    #[test]
    fn reinforce_smoke_2dof() {
        let (mut algo, task) = make_reinforce();
        let mut env = task.build_vec_env(10, 0).unwrap();

        let metrics = algo.train(&mut env, TrainingBudget::Epochs(5), 42, &|_| {});

        assert_eq!(metrics.len(), 5);
        for (i, m) in metrics.iter().enumerate() {
            assert_eq!(m.epoch, i);
            assert!(m.total_steps > 0);
            assert!(m.extra.contains_key("sigma"));
            assert!(m.extra.contains_key("policy_grad_norm"));
        }
    }

    #[test]
    fn reinforce_reward_improves() {
        let (mut algo, task) = make_reinforce();
        let mut env = task.build_vec_env(20, 0).unwrap();

        let metrics = algo.train(&mut env, TrainingBudget::Epochs(10), 42, &|_| {});

        let first = metrics[0].mean_reward;
        let last = metrics[9].mean_reward;
        assert!(
            last > first,
            "REINFORCE should improve reward: first={first:.1}, last={last:.1}"
        );
    }

    // ── Artifact / checkpoint tests ──────────────────────────────────

    #[test]
    fn reinforce_policy_artifact_valid() {
        let (algo, _) = make_reinforce();
        algo.policy_artifact().validate().unwrap();
    }

    #[test]
    fn reinforce_checkpoint_before_train() {
        let (algo, _) = make_reinforce();
        let cp = algo.checkpoint();
        assert_eq!(cp.algorithm_name, "REINFORCE");
        assert_eq!(cp.algorithm_state["sigma"], 0.5);
        assert_eq!(cp.optimizer_states.len(), 1);
        assert_eq!(cp.optimizer_states[0].role, "actor");
        assert_eq!(cp.optimizer_states[0].t, 0);
    }

    #[test]
    fn reinforce_checkpoint_round_trip() {
        let (mut algo, task) = make_reinforce();
        let mut env = task.build_vec_env(10, 0).unwrap();
        algo.train(&mut env, TrainingBudget::Epochs(3), 42, &|_| {});

        let cp = algo.checkpoint();
        assert!(cp.algorithm_state["sigma"] < 0.5); // decayed
        assert!(cp.optimizer_states[0].t > 0); // steps taken

        let algo2 = Reinforce::from_checkpoint(
            &cp,
            OptimizerConfig::adam(0.05),
            ReinforceHyperparams {
                gamma: 0.99,
                sigma_init: 0.5,
                sigma_decay: 0.95,
                sigma_min: 0.05,
                max_episode_steps: 300,
            },
        )
        .unwrap();
        assert_eq!(algo2.policy.params(), algo.policy.params());
        assert_eq!(algo2.sigma, cp.algorithm_state["sigma"]);
    }
}
