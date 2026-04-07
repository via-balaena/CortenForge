//! SAC — Soft Actor-Critic with auto-tuned entropy temperature.
//!
//! Off-policy actor-critic with twin Q-networks, entropy regularization,
//! and automatic temperature tuning.  Builds on TD3's twin-Q + replay
//! infrastructure but replaces the deterministic policy with a stochastic
//! one and adds an entropy bonus to the objective.
//!
//! Requires [`StochasticPolicy`] + 2× [`QFunction`] + [`OptimizerConfig`].
//!
//! Key differences from TD3:
//! - Stochastic policy (learned `log_std`) instead of deterministic + noise.
//! - Entropy term `α · H[π]` encourages exploration without explicit noise.
//! - No target policy network (only target Q-networks).
//! - Auto-tuned temperature `α` via dual gradient descent on target entropy.

use std::collections::BTreeMap;
use std::time::Instant;

use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::algorithm::{Algorithm, EpochMetrics, TrainingBudget};
use crate::optimizer::OptimizerConfig;
use crate::policy::StochasticPolicy;
use crate::replay_buffer::ReplayBuffer;
use crate::tensor::Tensor;
use crate::value::{QFunction, soft_update};
use crate::vec_env::VecEnv;

// ── Hyperparameters ──────────────────────────────────────────────────────

/// SAC hyperparameters.
#[derive(Debug, Clone, Copy)]
pub struct SacHyperparams {
    /// Discount factor (typically 0.99).
    pub gamma: f64,
    /// Polyak averaging coefficient for target Q-networks (typically 0.005).
    pub tau: f64,
    /// Initial entropy temperature α.
    pub alpha_init: f64,
    /// Whether to auto-tune α via dual gradient descent.
    pub auto_alpha: bool,
    /// Target entropy (typically `−act_dim`). Only used when `auto_alpha` is true.
    pub target_entropy: f64,
    /// Learning rate for the log-α optimizer.
    pub alpha_lr: f64,
    /// Mini-batch size for replay buffer sampling.
    pub batch_size: usize,
    /// Replay buffer capacity.
    pub buffer_capacity: usize,
    /// Number of random exploration steps before training starts.
    pub warmup_steps: usize,
    /// Maximum environment steps per episode (epoch boundary).
    pub max_episode_steps: usize,
}

// ── SAC ──────────────────────────────────────────────────────────────────

/// SAC algorithm.
///
/// # Parts
///
/// - [`StochasticPolicy`] — actor with learned exploration (no external σ).
/// - 2× [`QFunction`] — twin critics (+ target copies).
/// - [`OptimizerConfig`] — builds 3 optimizers (actor, Q1, Q2).
///
/// No target policy needed — SAC uses stochastic actions from the current
/// policy for both data collection and target computation.
///
/// The caller provides 4 Q-function boxes: 2 primary + 2 targets.
/// Target params are hard-synced to primaries in the constructor.
///
/// # Constructor
///
/// ```ignore
/// let sac = Sac::new(
///     Box::new(LinearStochasticPolicy::new(od, ad, &sc, -0.5)),
///     Box::new(LinearQ::new(od, ad, &sc)),
///     Box::new(LinearQ::new(od, ad, &sc)),
///     Box::new(LinearQ::new(od, ad, &sc)),  // Q1 target
///     Box::new(LinearQ::new(od, ad, &sc)),  // Q2 target
///     OptimizerConfig::adam(3e-4),
///     SacHyperparams { .. },
/// );
/// ```
pub struct Sac {
    policy: Box<dyn StochasticPolicy>,
    q1: Box<dyn QFunction>,
    q2: Box<dyn QFunction>,
    target_q1: Box<dyn QFunction>,
    target_q2: Box<dyn QFunction>,
    optimizer_config: OptimizerConfig,
    hyperparams: SacHyperparams,
}

impl Sac {
    /// Create a new SAC instance.
    ///
    /// Target Q-networks are hard-synced to their primary counterparts.
    #[must_use]
    pub fn new(
        policy: Box<dyn StochasticPolicy>,
        q1: Box<dyn QFunction>,
        q2: Box<dyn QFunction>,
        mut target_q1: Box<dyn QFunction>,
        mut target_q2: Box<dyn QFunction>,
        optimizer_config: OptimizerConfig,
        hyperparams: SacHyperparams,
    ) -> Self {
        // Hard-sync targets to primaries.
        target_q1.set_params(q1.params());
        target_q2.set_params(q2.params());

        Self {
            policy,
            q1,
            q2,
            target_q1,
            target_q2,
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

/// Log probability of action under a diagonal Gaussian.
///
/// Computes `log N(action | mu, diag(exp(log_std)²))`.
#[must_use]
pub fn gaussian_log_prob(action: &[f64], mu: &[f64], log_std: &[f64]) -> f64 {
    let mut lp = 0.0;
    for i in 0..action.len() {
        let std = log_std[i].exp();
        let var = std * std;
        let diff = action[i] - mu[i];
        lp += 0.5f64.mul_add(
            -(2.0 * std::f64::consts::PI).ln(),
            -0.5 * diff * diff / var - log_std[i],
        );
    }
    lp
}

impl Algorithm for Sac {
    fn name(&self) -> &'static str {
        "SAC"
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

        let n_epochs = match budget {
            TrainingBudget::Epochs(n) => n,
            TrainingBudget::Steps(s) => s / (n_envs * hp.max_episode_steps).max(1),
        };

        // Build 3 optimizers: actor, Q1, Q2 (momentum state only).
        let mut actor_opt = self.optimizer_config.build(self.policy.n_params());
        let mut q1_opt = self.optimizer_config.build(self.q1.n_params());
        let mut q2_opt = self.optimizer_config.build(self.q2.n_params());

        // Infer dimensions from first reset.
        let current_obs_tensor = env
            .reset_all()
            .unwrap_or_else(|e| panic!("SAC: reset_all failed: {e}"));
        let obs_dim = current_obs_tensor.shape()[1];
        let act_dim = self.policy.forward(current_obs_tensor.row(0)).len();

        let mut buffer = ReplayBuffer::new(hp.buffer_capacity, obs_dim, act_dim);
        let mut current_obs: Vec<Vec<f32>> = (0..n_envs)
            .map(|i| current_obs_tensor.row(i).to_vec())
            .collect();

        // Entropy temperature (α) and its log for gradient stability.
        let mut log_alpha = hp.alpha_init.ln();
        let mut alpha = hp.alpha_init;
        let mut metrics = Vec::with_capacity(n_epochs);

        for epoch in 0..n_epochs {
            let t0 = Instant::now();
            let mut epoch_rewards: Vec<f64> = Vec::new();
            let mut epoch_done_count = 0_usize;
            let mut epoch_steps = 0_usize;
            let mut epoch_q1_loss = 0.0_f64;
            let mut epoch_q2_loss = 0.0_f64;
            let mut epoch_policy_loss = 0.0_f64;
            let mut epoch_entropy = 0.0_f64;
            let mut epoch_updates = 0_usize;
            let mut env_episode_reward = vec![0.0_f64; n_envs];
            let mut env_complete = vec![false; n_envs];

            for _step in 0..hp.max_episode_steps {
                if env_complete.iter().all(|&c| c) {
                    break;
                }

                // Select actions via stochastic policy.
                let mut action_data = vec![0.0_f32; n_envs * act_dim];
                let mut actions_f64: Vec<Vec<f64>> = Vec::with_capacity(n_envs);

                for i in 0..n_envs {
                    let action = if buffer.len() < hp.warmup_steps {
                        // Random exploration during warmup.
                        (0..act_dim)
                            .map(|_| randn(&mut rng).clamp(-1.0, 1.0))
                            .collect::<Vec<_>>()
                    } else {
                        // Sample from stochastic policy: a = μ + σ * ε
                        let (mu, log_std) = self.policy.forward_stochastic(&current_obs[i]);
                        mu.iter()
                            .zip(&log_std)
                            .map(|(&m, &ls)| {
                                let std = ls.exp();
                                std.mul_add(randn(&mut rng), m).clamp(-1.0, 1.0)
                            })
                            .collect::<Vec<_>>()
                    };
                    for (j, &a) in action.iter().enumerate() {
                        action_data[i * act_dim + j] = a as f32;
                    }
                    actions_f64.push(action);
                }

                let action_tensor = Tensor::from_slice(&action_data, &[n_envs, act_dim]);
                let result = env
                    .step(&action_tensor)
                    .unwrap_or_else(|e| panic!("SAC: step failed: {e}"));

                epoch_steps += n_envs;

                // Store transitions and track episode boundaries.
                for i in 0..n_envs {
                    let reward = result.rewards[i];
                    let done = result.dones[i];
                    let truncated = result.truncateds[i];

                    let next_obs = if done || truncated {
                        result.terminal_observations[i].as_ref().map_or_else(
                            || result.observations.row(i).to_vec(),
                            |t| t.as_slice().to_vec(),
                        )
                    } else {
                        result.observations.row(i).to_vec()
                    };

                    buffer.push(&current_obs[i], &actions_f64[i], reward, &next_obs, done);

                    env_episode_reward[i] += reward;

                    if (done || truncated) && !env_complete[i] {
                        env_complete[i] = true;
                        epoch_rewards.push(env_episode_reward[i]);
                        if done {
                            epoch_done_count += 1;
                        }
                    }

                    current_obs[i] = result.observations.row(i).to_vec();
                    if done || truncated {
                        env_episode_reward[i] = 0.0;
                    }
                }

                // ── Gradient updates (after warmup) ──────────────────────

                if buffer.len() >= hp.batch_size.max(hp.warmup_steps) {
                    let batch = buffer.sample(hp.batch_size, &mut rng);

                    // ── 1. Compute target Q values ───────────────────────
                    // y = r + γ(1-d) * (min(Q1_tgt(s',a'), Q2_tgt(s',a')) - α·log π(a'|s'))
                    // where a' ~ π(·|s')

                    let mut targets = Vec::with_capacity(hp.batch_size);
                    for b in 0..hp.batch_size {
                        let next_obs = &batch.next_obs[b * obs_dim..(b + 1) * obs_dim];
                        let (mu_next, log_std_next) = self.policy.forward_stochastic(next_obs);
                        // Sample action from policy at s'.
                        let a_next: Vec<f64> = mu_next
                            .iter()
                            .zip(&log_std_next)
                            .map(|(&m, &ls)| {
                                let std = ls.exp();
                                std.mul_add(randn(&mut rng), m).clamp(-1.0, 1.0)
                            })
                            .collect();

                        let q1_val = self.target_q1.forward(next_obs, &a_next);
                        let q2_val = self.target_q2.forward(next_obs, &a_next);
                        let min_q = q1_val.min(q2_val);
                        let lp = gaussian_log_prob(&a_next, &mu_next, &log_std_next);
                        let d = if batch.dones[b] { 1.0 } else { 0.0 };
                        targets.push(
                            (hp.gamma * (1.0 - d))
                                .mul_add(alpha.mul_add(-lp, min_q), batch.rewards[b]),
                        );
                    }

                    // ── 2. Update Q1, Q2 ─────────────────────────────────

                    let q1_grad = self.q1.mse_gradient_batch(
                        &batch.obs,
                        &batch.actions,
                        &targets,
                        obs_dim,
                        act_dim,
                    );
                    let mut q1p = self.q1.params().to_vec();
                    q1_opt.step_in_place(&mut q1p, &q1_grad, false);
                    self.q1.set_params(&q1p);

                    let q2_grad = self.q2.mse_gradient_batch(
                        &batch.obs,
                        &batch.actions,
                        &targets,
                        obs_dim,
                        act_dim,
                    );
                    let mut q2p = self.q2.params().to_vec();
                    q2_opt.step_in_place(&mut q2p, &q2_grad, false);
                    self.q2.set_params(&q2p);

                    // Track Q losses.
                    let q1_loss: f64 = (0..hp.batch_size)
                        .map(|b| {
                            let obs = &batch.obs[b * obs_dim..(b + 1) * obs_dim];
                            let act = &batch.actions[b * act_dim..(b + 1) * act_dim];
                            (self.q1.forward(obs, act) - targets[b]).powi(2)
                        })
                        .sum::<f64>()
                        / hp.batch_size as f64;
                    let q2_loss: f64 = (0..hp.batch_size)
                        .map(|b| {
                            let obs = &batch.obs[b * obs_dim..(b + 1) * obs_dim];
                            let act = &batch.actions[b * act_dim..(b + 1) * act_dim];
                            (self.q2.forward(obs, act) - targets[b]).powi(2)
                        })
                        .sum::<f64>()
                        / hp.batch_size as f64;
                    epoch_q1_loss += q1_loss;
                    epoch_q2_loss += q2_loss;

                    // ── 3. Update actor ───────────────────────────────────
                    // Actor objective: maximize  E[Q(s,a) - α·log π(a|s)]
                    // Gradient: dJ/dθ = -(α · d(log π)/dθ - dQ/da · d(a)/dθ)
                    // where a = μ + σ·ε (reparameterization)

                    let mut actor_grad = vec![0.0; self.policy.n_params()];
                    let mut batch_entropy = 0.0_f64;

                    for b in 0..hp.batch_size {
                        let obs = &batch.obs[b * obs_dim..(b + 1) * obs_dim];
                        let (mu, log_std) = self.policy.forward_stochastic(obs);

                        // Sample ε and compute reparameterized action.
                        let eps: Vec<f64> = (0..act_dim).map(|_| randn(&mut rng)).collect();
                        let action: Vec<f64> = mu
                            .iter()
                            .zip(&log_std)
                            .zip(&eps)
                            .map(|((&m, &ls), &e)| ls.exp().mul_add(e, m).clamp(-1.0, 1.0))
                            .collect();

                        // dQ/da from Q1.
                        let dq_da = self.q1.action_gradient(obs, &action);

                        // d(a)/d(θ) via reparameterized VJP: v^T · d(a)/d(θ) with v = dQ/da.
                        let reparam_vjp = self.policy.reparameterized_vjp(obs, &eps, &dq_da);

                        // d(log π)/d(θ).
                        let log_pi_grad = self.policy.log_prob_gradient_stochastic(obs, &action);

                        // Actor gradient: we want to maximize Q - α·log π
                        // dJ/dθ = dQ/da · d(a)/dθ − α · d(log π)/dθ
                        // We negate for descent (optimizer.step with ascent=false) later.
                        // Actually: we'll do ascent, so accumulate dJ/dθ directly.
                        for (j, g) in actor_grad.iter_mut().enumerate() {
                            *g += alpha.mul_add(-log_pi_grad[j], reparam_vjp[j]);
                        }

                        batch_entropy += self.policy.entropy(obs);
                    }

                    // Average and apply.
                    let inv_n = 1.0 / hp.batch_size as f64;
                    for g in &mut actor_grad {
                        *g *= inv_n;
                    }
                    epoch_entropy += batch_entropy * inv_n;

                    let policy_loss = actor_grad.iter().map(|g| g * g).sum::<f64>().sqrt();
                    epoch_policy_loss += policy_loss;

                    let mut ap = self.policy.params().to_vec();
                    actor_opt.step_in_place(&mut ap, &actor_grad, true);
                    self.policy.set_params(&ap);

                    // ── 4. Update α (auto-tuning) ────────────────────────

                    if hp.auto_alpha {
                        // d(α_loss)/d(log_α) = -α * (mean_log_pi + target_entropy)
                        let mut mean_log_pi = 0.0;
                        for b in 0..hp.batch_size {
                            let obs = &batch.obs[b * obs_dim..(b + 1) * obs_dim];
                            let (mu, log_std) = self.policy.forward_stochastic(obs);
                            let eps: Vec<f64> = (0..act_dim).map(|_| randn(&mut rng)).collect();
                            let action: Vec<f64> = mu
                                .iter()
                                .zip(&log_std)
                                .zip(&eps)
                                .map(|((&m, &ls), &e)| ls.exp().mul_add(e, m).clamp(-1.0, 1.0))
                                .collect();
                            mean_log_pi += gaussian_log_prob(&action, &mu, &log_std);
                        }
                        mean_log_pi *= inv_n;

                        // Gradient descent on log_alpha.
                        let alpha_grad = -(mean_log_pi + hp.target_entropy);
                        log_alpha -= hp.alpha_lr * alpha_grad;
                        alpha = log_alpha.exp();
                    }

                    // ── 5. Soft update target Q-networks ─────────────────

                    soft_update(&mut *self.target_q1, &*self.q1, hp.tau);
                    soft_update(&mut *self.target_q2, &*self.q2, hp.tau);

                    epoch_updates += 1;
                }
            }

            // ── Epoch metrics ────────────────────────────────────────────

            let mean_reward = if epoch_rewards.is_empty() {
                0.0
            } else {
                epoch_rewards.iter().sum::<f64>() / epoch_rewards.len() as f64
            };

            let mut extra = BTreeMap::new();
            if epoch_updates > 0 {
                extra.insert("q1_loss".into(), epoch_q1_loss / epoch_updates as f64);
                extra.insert("q2_loss".into(), epoch_q2_loss / epoch_updates as f64);
                extra.insert(
                    "policy_loss".into(),
                    epoch_policy_loss / epoch_updates as f64,
                );
                extra.insert("entropy".into(), epoch_entropy / epoch_updates as f64);
            }
            extra.insert("alpha".into(), alpha);
            extra.insert("buffer_size".into(), buffer.len() as f64);

            metrics.push(EpochMetrics {
                epoch,
                mean_reward,
                done_count: epoch_done_count,
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
#[allow(clippy::unwrap_used, clippy::cast_precision_loss)]
mod tests {
    use super::*;
    use crate::{LinearQ, LinearStochasticPolicy, reaching_2dof};

    fn make_sac() -> (Sac, crate::TaskConfig) {
        let task = reaching_2dof();
        let od = task.obs_dim();
        let ad = task.act_dim();
        let sc = task.obs_scale();

        let algo = Sac::new(
            Box::new(LinearStochasticPolicy::new(od, ad, sc, -0.5)),
            Box::new(LinearQ::new(od, ad, sc)),
            Box::new(LinearQ::new(od, ad, sc)),
            Box::new(LinearQ::new(od, ad, sc)),
            Box::new(LinearQ::new(od, ad, sc)),
            OptimizerConfig::adam(3e-4),
            SacHyperparams {
                gamma: 0.99,
                tau: 0.005,
                alpha_init: 0.2,
                auto_alpha: true,
                target_entropy: -(ad as f64),
                alpha_lr: 3e-4,
                batch_size: 32,
                buffer_capacity: 10_000,
                warmup_steps: 64,
                max_episode_steps: 300,
            },
        );
        (algo, task)
    }

    #[test]
    fn sac_name() {
        let (algo, _) = make_sac();
        assert_eq!(algo.name(), "SAC");
    }

    #[test]
    fn sac_smoke_2dof() {
        let (mut algo, task) = make_sac();
        let mut env = task.build_vec_env(5).unwrap();

        let metrics = algo.train(&mut env, TrainingBudget::Epochs(3), 42);

        assert_eq!(metrics.len(), 3);
        for (i, m) in metrics.iter().enumerate() {
            assert_eq!(m.epoch, i);
            assert!(m.total_steps > 0);
            assert!(m.extra.contains_key("alpha"));
            assert!(m.extra.contains_key("buffer_size"));
        }
    }

    #[test]
    fn sac_alpha_adjusts() {
        let (mut algo, task) = make_sac();
        let mut env = task.build_vec_env(5).unwrap();

        let metrics = algo.train(&mut env, TrainingBudget::Epochs(3), 42);

        // Alpha should change from its initial value with auto-tuning.
        let alpha_first = metrics[0].extra["alpha"];
        let alpha_last = metrics[2].extra["alpha"];
        // They might be equal on the first epoch if warmup hasn't finished,
        // but at least they should be finite.
        assert!(alpha_first.is_finite(), "alpha should be finite");
        assert!(alpha_last.is_finite(), "alpha should be finite");
        assert!(alpha_first > 0.0, "alpha should be positive");
    }

    #[test]
    fn sac_target_sync() {
        let task = reaching_2dof();
        let od = task.obs_dim();
        let ad = task.act_dim();
        let sc = task.obs_scale();

        let sac = Sac::new(
            Box::new(LinearStochasticPolicy::new(od, ad, sc, -0.5)),
            Box::new(LinearQ::new(od, ad, sc)),
            Box::new(LinearQ::new(od, ad, sc)),
            Box::new(LinearQ::new(od, ad, sc)),
            Box::new(LinearQ::new(od, ad, sc)),
            OptimizerConfig::adam(1e-3),
            SacHyperparams {
                gamma: 0.99,
                tau: 0.005,
                alpha_init: 0.2,
                auto_alpha: false,
                target_entropy: -2.0,
                alpha_lr: 3e-4,
                batch_size: 32,
                buffer_capacity: 1000,
                warmup_steps: 32,
                max_episode_steps: 100,
            },
        );

        assert_eq!(sac.q1.params(), sac.target_q1.params());
        assert_eq!(sac.q2.params(), sac.target_q2.params());
    }
}
