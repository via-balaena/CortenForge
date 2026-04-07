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
    optimizer_config: OptimizerConfig,
    hyperparams: ReinforceHyperparams,
}

impl Reinforce {
    /// Create a new REINFORCE instance.
    #[must_use]
    pub fn new(
        policy: Box<dyn DifferentiablePolicy>,
        optimizer_config: OptimizerConfig,
        hyperparams: ReinforceHyperparams,
    ) -> Self {
        Self {
            policy,
            optimizer_config,
            hyperparams,
        }
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
    fn train(&mut self, env: &mut VecEnv, budget: TrainingBudget, seed: u64) -> Vec<EpochMetrics> {
        let mut rng = StdRng::seed_from_u64(seed);
        let n_params = self.policy.n_params();
        let n_envs = env.n_envs();
        let hp = self.hyperparams;

        let n_epochs = match budget {
            TrainingBudget::Epochs(n) => n,
            TrainingBudget::Steps(s) => s / (n_envs * hp.max_episode_steps).max(1),
        };

        // Build optimizer (momentum state only — no param copy needed).
        let mut optimizer = self.optimizer_config.build(n_params);

        let mut sigma = hp.sigma_init;
        let mut metrics = Vec::with_capacity(n_epochs);

        for epoch in 0..n_epochs {
            let t0 = Instant::now();

            // Collect one episode per env with stochastic actions.
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
                metrics.push(EpochMetrics {
                    epoch,
                    mean_reward: 0.0,
                    done_count: 0,
                    total_steps: 0,
                    wall_time_ms: t0.elapsed().as_millis() as u64,
                    extra: BTreeMap::new(),
                });
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
            optimizer.step_in_place(&mut p, &grad, true);
            self.policy.set_params(&p);

            // Decay sigma.
            sigma = (sigma * hp.sigma_decay).max(hp.sigma_min);

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

            let mut extra = BTreeMap::new();
            extra.insert("sigma".into(), sigma);
            extra.insert("policy_grad_norm".into(), grad_norm);

            metrics.push(EpochMetrics {
                epoch,
                mean_reward,
                done_count,
                total_steps: epoch_steps,
                wall_time_ms: t0.elapsed().as_millis() as u64,
                extra,
            });
        }

        metrics
    }
}

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used)]
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
        let mut env = task.build_vec_env(10).unwrap();

        let metrics = algo.train(&mut env, TrainingBudget::Epochs(5), 42);

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
        let mut env = task.build_vec_env(20).unwrap();

        let metrics = algo.train(&mut env, TrainingBudget::Epochs(10), 42);

        let first = metrics[0].mean_reward;
        let last = metrics[9].mean_reward;
        assert!(
            last > first,
            "REINFORCE should improve reward: first={first:.1}, last={last:.1}"
        );
    }
}
