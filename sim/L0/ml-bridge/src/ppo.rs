//! PPO — Proximal Policy Optimization with clipped surrogate + GAE.
//!
//! Collects episodic rollouts, computes GAE advantages using a learned value
//! function, then runs K optimization passes over frozen rollout data with a
//! clipped importance-weighted surrogate objective.
//!
//! Requires [`DifferentiablePolicy`] + [`ValueFn`] + [`OptimizerConfig`].
//!
//! Extracted from the working PPO example
//! (`examples/fundamentals/sim-ml/vec-env/ppo/src/main.rs`).

use std::collections::BTreeMap;
use std::time::Instant;

use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::algorithm::{Algorithm, EpochMetrics, TrainingBudget};
use crate::gae::compute_gae;
use crate::optimizer::OptimizerConfig;
use crate::policy::DifferentiablePolicy;
use crate::rollout::collect_episodic_rollout;
use crate::value::ValueFn;
use crate::vec_env::VecEnv;

// ── Hyperparameters ──────────────────────────────────────────────────────

/// PPO hyperparameters.
#[derive(Debug, Clone, Copy)]
pub struct PpoHyperparams {
    /// Clipping range for the importance ratio (typically 0.2).
    pub clip_eps: f64,
    /// Number of optimization passes over the frozen rollout data.
    pub k_passes: usize,
    /// Discount factor (typically 0.99).
    pub gamma: f64,
    /// GAE lambda (typically 0.95).
    pub gae_lambda: f64,
    /// Initial exploration noise σ.
    pub sigma_init: f64,
    /// Multiplicative decay per epoch.
    pub sigma_decay: f64,
    /// Minimum σ floor.
    pub sigma_min: f64,
    /// Maximum environment steps per episode.
    pub max_episode_steps: usize,
}

// ── PPO ──────────────────────────────────────────────────────────────────

/// PPO algorithm.
///
/// # Parts
///
/// - [`DifferentiablePolicy`] — actor with gradient computation.
/// - [`ValueFn`] — critic for advantage estimation.
/// - [`OptimizerConfig`] — builds separate actor and critic optimizers.
///
/// # Constructor
///
/// ```ignore
/// let ppo = Ppo::new(
///     Box::new(MlpPolicy::new(obs_dim, 32, act_dim, &obs_scale)),
///     Box::new(MlpValue::new(obs_dim, 32, &obs_scale)),
///     OptimizerConfig::adam(0.025),
///     PpoHyperparams { clip_eps: 0.2, k_passes: 2, gamma: 0.99, .. },
/// );
/// ```
pub struct Ppo {
    policy: Box<dyn DifferentiablePolicy>,
    value_fn: Box<dyn ValueFn>,
    optimizer_config: OptimizerConfig,
    hyperparams: PpoHyperparams,
}

impl Ppo {
    /// Create a new PPO instance.
    #[must_use]
    pub fn new(
        policy: Box<dyn DifferentiablePolicy>,
        value_fn: Box<dyn ValueFn>,
        optimizer_config: OptimizerConfig,
        hyperparams: PpoHyperparams,
    ) -> Self {
        Self {
            policy,
            value_fn,
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

/// Log probability under isotropic Gaussian: `Σ -0.5 * (a_i - μ_i)² / σ²`.
fn gaussian_log_prob(mu: &[f64], action: &[f64], sigma: f64) -> f64 {
    let sigma2 = sigma * sigma;
    mu.iter()
        .zip(action)
        .map(|(&m, &a)| -0.5 * (a - m).powi(2) / sigma2)
        .sum()
}

/// Per-step rollout data for PPO (needed for importance weighting).
struct PpoStep {
    obs: Vec<f32>,
    action: Vec<f64>,
    mu_old: Vec<f64>,
}

impl Algorithm for Ppo {
    fn name(&self) -> &'static str {
        "PPO"
    }

    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::too_many_lines,
        clippy::panic
    )]
    fn train(&mut self, env: &mut VecEnv, budget: TrainingBudget, seed: u64) -> Vec<EpochMetrics> {
        let mut rng = StdRng::seed_from_u64(seed);
        let n_envs = env.n_envs();
        let hp = self.hyperparams;
        let n_policy_params = self.policy.n_params();
        let n_value_params = self.value_fn.n_params();

        let n_epochs = match budget {
            TrainingBudget::Epochs(n) => n,
            TrainingBudget::Steps(s) => s / (n_envs * hp.max_episode_steps).max(1),
        };

        // Build separate optimizers for actor and critic.
        let mut actor_opt = self.optimizer_config.build(n_policy_params);
        actor_opt.set_params(self.policy.params());
        let mut critic_opt = self.optimizer_config.build(n_value_params);
        critic_opt.set_params(self.value_fn.params());

        let mut sigma = hp.sigma_init;
        let mut metrics = Vec::with_capacity(n_epochs);

        for epoch in 0..n_epochs {
            let t0 = Instant::now();

            // ── Phase 1: Collect rollout, recording mu_old and v_old ─────

            // We need per-step data (mu_old for importance ratio, v_old for GAE).
            // collect_episodic_rollout handles the env stepping; we record extras.
            let mut per_step_mu: Vec<Vec<Vec<f64>>> = (0..n_envs).map(|_| Vec::new()).collect();
            let mut per_step_v: Vec<Vec<f64>> = (0..n_envs).map(|_| Vec::new()).collect();
            let mut env_done: Vec<bool> = vec![false; n_envs];

            let rollout = collect_episodic_rollout(
                env,
                &mut |env_idx, obs| {
                    if env_done[env_idx] {
                        // Env already done — return zeros (won't be recorded).
                        return vec![0.0; self.policy.forward(obs).len()];
                    }
                    let mu = self.policy.forward(obs);
                    let v = self.value_fn.forward(obs);
                    per_step_mu[env_idx].push(mu.clone());
                    per_step_v[env_idx].push(v);
                    mu.iter()
                        .map(|&m| sigma.mul_add(randn(&mut rng), m))
                        .collect()
                },
                hp.max_episode_steps,
            );

            // Mark done envs (for the recording above — not used further).
            for (i, traj) in rollout.trajectories.iter().enumerate() {
                env_done[i] = traj.done;
            }

            // ── Phase 2: GAE per trajectory ──────────────────────────────

            let mut gae_results: Vec<(Vec<f64>, Vec<f64>)> = Vec::with_capacity(n_envs);
            for (i, traj) in rollout.trajectories.iter().enumerate() {
                let n = traj.len();
                if n == 0 {
                    gae_results.push((Vec::new(), Vec::new()));
                    continue;
                }

                // Bootstrap value at terminal state.
                let next_value = if traj.done {
                    0.0
                } else {
                    traj.terminal_obs
                        .as_ref()
                        .map_or(0.0, |obs| self.value_fn.forward(obs))
                };

                let values = &per_step_v[i][..n];
                let (advantages, value_targets) =
                    compute_gae(&traj.rewards, values, next_value, hp.gamma, hp.gae_lambda);
                gae_results.push((advantages, value_targets));
            }

            // ── Phase 3: Flatten and normalize advantages ────────────────

            // Build flat step list with trajectory cross-references.
            let mut steps: Vec<PpoStep> = Vec::new();
            let mut all_advantages: Vec<f64> = Vec::new();
            let mut all_value_targets: Vec<f64> = Vec::new();

            for (traj_idx, traj) in rollout.trajectories.iter().enumerate() {
                let (ref advantages, ref value_targets) = gae_results[traj_idx];
                for step_idx in 0..traj.len() {
                    steps.push(PpoStep {
                        obs: traj.obs[step_idx].clone(),
                        action: traj.actions[step_idx].clone(),
                        mu_old: per_step_mu[traj_idx][step_idx].clone(),
                    });
                    all_advantages.push(advantages[step_idx]);
                    all_value_targets.push(value_targets[step_idx]);
                }
            }

            let n_samples = steps.len();
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

            // Normalize advantages (zero-mean, unit-std).
            let mean_adv = all_advantages.iter().sum::<f64>() / n_samples as f64;
            for a in &mut all_advantages {
                *a -= mean_adv;
            }
            let std_adv =
                (all_advantages.iter().map(|a| a * a).sum::<f64>() / n_samples as f64).sqrt();
            if std_adv > 1e-8 {
                for a in &mut all_advantages {
                    *a /= std_adv;
                }
            }

            // ── Phase 4: K optimization passes ──────────────────────────

            let mut last_actor_grad_norm = 0.0;
            let mut last_critic_grad_norm = 0.0;
            let mut total_value_loss = 0.0;
            let mut total_clip_count = 0_usize;
            let mut total_sample_count = 0_usize;

            for _pass in 0..hp.k_passes {
                let mut actor_grad = vec![0.0; n_policy_params];
                let mut critic_grad = vec![0.0; n_value_params];
                let mut clip_count = 0_usize;
                let mut value_loss = 0.0;

                for (s_idx, step) in steps.iter().enumerate() {
                    let advantage = all_advantages[s_idx];
                    let value_target = all_value_targets[s_idx];

                    // Importance ratio: π_new(a|s) / π_old(a|s).
                    let mu_new = self.policy.forward(&step.obs);
                    let log_ratio = gaussian_log_prob(&mu_new, &step.action, sigma)
                        - gaussian_log_prob(&step.mu_old, &step.action, sigma);
                    let ratio = log_ratio.exp();

                    // Clipped surrogate.
                    let unclipped = ratio * advantage;
                    let clamped = ratio.clamp(1.0 - hp.clip_eps, 1.0 + hp.clip_eps) * advantage;

                    if unclipped <= clamped {
                        // Unclipped is the min — gradient flows through ratio.
                        let score = self
                            .policy
                            .log_prob_gradient(&step.obs, &step.action, sigma);
                        for (g, &s) in actor_grad.iter_mut().zip(&score) {
                            *g += advantage * ratio * s;
                        }
                    } else {
                        clip_count += 1;
                    }

                    // Critic: MSE loss against frozen value targets.
                    let v_pred = self.value_fn.forward(&step.obs);
                    value_loss += (v_pred - value_target).powi(2);
                    let vg = self.value_fn.mse_gradient(&step.obs, value_target);
                    for (g, &vgi) in critic_grad.iter_mut().zip(&vg) {
                        *g += vgi;
                    }
                }

                // Average gradients.
                let inv_n = 1.0 / n_samples as f64;
                for g in &mut actor_grad {
                    *g *= inv_n;
                }
                for g in &mut critic_grad {
                    *g *= inv_n;
                }

                last_actor_grad_norm = actor_grad.iter().map(|g| g * g).sum::<f64>().sqrt();
                last_critic_grad_norm = critic_grad.iter().map(|g| g * g).sum::<f64>().sqrt();
                total_value_loss += value_loss * inv_n;
                total_clip_count += clip_count;
                total_sample_count += n_samples;

                // Adam updates.
                actor_opt.step(&actor_grad, true); // ascent
                self.policy.set_params(actor_opt.params());
                critic_opt.step(&critic_grad, false); // descent
                self.value_fn.set_params(critic_opt.params());
            }

            // Decay sigma.
            sigma = (sigma * hp.sigma_decay).max(hp.sigma_min);

            // ── Epoch metrics ────────────────────────────────────────────

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

            let clip_fraction = if total_sample_count > 0 {
                total_clip_count as f64 / total_sample_count as f64
            } else {
                0.0
            };

            let mut extra = BTreeMap::new();
            extra.insert("sigma".into(), sigma);
            extra.insert("clip_fraction".into(), clip_fraction);
            extra.insert("value_loss".into(), total_value_loss / hp.k_passes as f64);
            extra.insert("policy_grad_norm".into(), last_actor_grad_norm);
            extra.insert("critic_grad_norm".into(), last_critic_grad_norm);

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
    use crate::{LinearPolicy, LinearValue, reaching_2dof};

    fn make_ppo() -> (Ppo, crate::TaskConfig) {
        let task = reaching_2dof();
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        let value = Box::new(LinearValue::new(task.obs_dim(), task.obs_scale()));
        let algo = Ppo::new(
            policy,
            value,
            OptimizerConfig::adam(0.025),
            PpoHyperparams {
                clip_eps: 0.2,
                k_passes: 2,
                gamma: 0.99,
                gae_lambda: 0.95,
                sigma_init: 0.5,
                sigma_decay: 0.90,
                sigma_min: 0.05,
                max_episode_steps: 300,
            },
        );
        (algo, task)
    }

    #[test]
    fn ppo_name() {
        let (algo, _) = make_ppo();
        assert_eq!(algo.name(), "PPO");
    }

    #[test]
    fn ppo_smoke_2dof() {
        let (mut algo, task) = make_ppo();
        let mut env = task.build_vec_env(10).unwrap();

        let metrics = algo.train(&mut env, TrainingBudget::Epochs(3), 42);

        assert_eq!(metrics.len(), 3);
        for (i, m) in metrics.iter().enumerate() {
            assert_eq!(m.epoch, i);
            assert!(m.total_steps > 0);
            assert!(m.extra.contains_key("sigma"));
            assert!(m.extra.contains_key("clip_fraction"));
            assert!(m.extra.contains_key("value_loss"));
            assert!(m.extra.contains_key("policy_grad_norm"));
            assert!(m.extra.contains_key("critic_grad_norm"));
        }
    }

    #[test]
    fn ppo_extra_metrics_finite() {
        let (mut algo, task) = make_ppo();
        let mut env = task.build_vec_env(10).unwrap();

        let metrics = algo.train(&mut env, TrainingBudget::Epochs(2), 0);
        for m in &metrics {
            for (k, v) in &m.extra {
                assert!(v.is_finite(), "extra[{k}] = {v} is not finite");
            }
        }
    }

    #[test]
    fn ppo_clip_fraction_in_range() {
        let (mut algo, task) = make_ppo();
        let mut env = task.build_vec_env(10).unwrap();

        let metrics = algo.train(&mut env, TrainingBudget::Epochs(3), 42);
        for m in &metrics {
            let cf = m.extra["clip_fraction"];
            assert!((0.0..=1.0).contains(&cf), "clip_fraction {cf} out of [0,1]");
        }
    }
}
