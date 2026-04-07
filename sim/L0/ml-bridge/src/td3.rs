//! TD3 — Twin Delayed Deep Deterministic Policy Gradient.
//!
//! Off-policy actor-critic with twin Q-networks, delayed policy updates,
//! and target policy smoothing.  Stores transitions in a replay buffer,
//! samples mini-batches for gradient updates every step.
//!
//! Requires [`DifferentiablePolicy`] + 2× [`QFunction`] + [`OptimizerConfig`].
//!
//! TD3 vs SAC: TD3 uses deterministic policy + exploration noise and target
//! smoothing.  SAC uses a stochastic policy + entropy regularization.  TD3
//! is simpler; SAC builds on the same twin-Q + replay infrastructure.

use std::collections::BTreeMap;
use std::time::Instant;

use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::algorithm::{Algorithm, EpochMetrics, TrainingBudget};
use crate::optimizer::OptimizerConfig;
use crate::policy::DifferentiablePolicy;
use crate::replay_buffer::ReplayBuffer;
use crate::tensor::Tensor;
use crate::value::{QFunction, soft_update};
use crate::vec_env::VecEnv;

// ── Hyperparameters ──────────────────────────────────────────────────────

/// TD3 hyperparameters.
#[derive(Debug, Clone, Copy)]
pub struct Td3Hyperparams {
    /// Discount factor (typically 0.99).
    pub gamma: f64,
    /// Polyak averaging coefficient for target network updates (typically 0.005).
    pub tau: f64,
    /// Standard deviation of target policy smoothing noise.
    pub policy_noise: f64,
    /// Clipping bound for target policy smoothing noise.
    pub noise_clip: f64,
    /// Standard deviation of exploration noise during data collection.
    pub exploration_noise: f64,
    /// Update actor and targets every `policy_delay` critic updates (typically 2).
    pub policy_delay: usize,
    /// Mini-batch size for replay buffer sampling.
    pub batch_size: usize,
    /// Replay buffer capacity.
    pub buffer_capacity: usize,
    /// Number of random exploration steps before training starts.
    pub warmup_steps: usize,
    /// Maximum environment steps per episode (epoch boundary).
    pub max_episode_steps: usize,
}

// ── TD3 ──────────────────────────────────────────────────────────────────

/// TD3 algorithm.
///
/// # Parts
///
/// - [`DifferentiablePolicy`] — deterministic actor (+ target copy).
/// - 2× [`QFunction`] — twin critics (+ target copies).
/// - [`OptimizerConfig`] — builds 3 optimizers (actor, Q1, Q2).
///
/// The caller provides 6 network boxes: 3 primary + 3 targets.
/// Target params are hard-synced to primaries in the constructor.
///
/// # Constructor
///
/// ```ignore
/// let td3 = Td3::new(
///     Box::new(MlpPolicy::new(od, 32, ad, &sc)),   // actor
///     Box::new(MlpPolicy::new(od, 32, ad, &sc)),   // actor target
///     Box::new(MlpQ::new(od, 32, ad, &sc)),        // Q1
///     Box::new(MlpQ::new(od, 32, ad, &sc)),        // Q2
///     Box::new(MlpQ::new(od, 32, ad, &sc)),        // Q1 target
///     Box::new(MlpQ::new(od, 32, ad, &sc)),        // Q2 target
///     OptimizerConfig::adam(3e-4),
///     Td3Hyperparams { .. },
/// );
/// ```
pub struct Td3 {
    policy: Box<dyn DifferentiablePolicy>,
    target_policy: Box<dyn DifferentiablePolicy>,
    q1: Box<dyn QFunction>,
    q2: Box<dyn QFunction>,
    target_q1: Box<dyn QFunction>,
    target_q2: Box<dyn QFunction>,
    optimizer_config: OptimizerConfig,
    hyperparams: Td3Hyperparams,
}

impl Td3 {
    /// Create a new TD3 instance.
    ///
    /// Target networks are hard-synced to their primary counterparts.
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        policy: Box<dyn DifferentiablePolicy>,
        mut target_policy: Box<dyn DifferentiablePolicy>,
        q1: Box<dyn QFunction>,
        q2: Box<dyn QFunction>,
        mut target_q1: Box<dyn QFunction>,
        mut target_q2: Box<dyn QFunction>,
        optimizer_config: OptimizerConfig,
        hyperparams: Td3Hyperparams,
    ) -> Self {
        // Hard-sync targets to primaries.
        target_policy.set_params(policy.params());
        target_q1.set_params(q1.params());
        target_q2.set_params(q2.params());

        Self {
            policy,
            target_policy,
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

impl Algorithm for Td3 {
    fn name(&self) -> &'static str {
        "TD3"
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
            .unwrap_or_else(|e| panic!("TD3: reset_all failed: {e}"));
        let obs_dim = current_obs_tensor.shape()[1];
        let act_dim = self.policy.forward(current_obs_tensor.row(0)).len();

        let mut buffer = ReplayBuffer::new(hp.buffer_capacity, obs_dim, act_dim);
        let mut current_obs: Vec<Vec<f32>> = (0..n_envs)
            .map(|i| current_obs_tensor.row(i).to_vec())
            .collect();

        let mut total_critic_updates = 0_usize;
        let mut metrics = Vec::with_capacity(n_epochs);

        for epoch in 0..n_epochs {
            let t0 = Instant::now();
            let mut epoch_rewards: Vec<f64> = Vec::new();
            let mut epoch_done_count = 0_usize;
            let mut epoch_steps = 0_usize;
            let mut epoch_q1_loss = 0.0_f64;
            let mut epoch_q2_loss = 0.0_f64;
            let mut epoch_policy_loss = 0.0_f64;
            let mut epoch_updates = 0_usize;
            let mut env_episode_reward = vec![0.0_f64; n_envs];
            let mut env_complete = vec![false; n_envs];

            // Step until all envs complete at least one episode.
            for _step in 0..hp.max_episode_steps {
                if env_complete.iter().all(|&c| c) {
                    break;
                }

                // Select actions.
                let mut action_data = vec![0.0_f32; n_envs * act_dim];
                let mut actions_f64: Vec<Vec<f64>> = Vec::with_capacity(n_envs);

                for i in 0..n_envs {
                    let action = if buffer.len() < hp.warmup_steps {
                        // Random exploration during warmup.
                        (0..act_dim)
                            .map(|_| randn(&mut rng).clamp(-1.0, 1.0))
                            .collect::<Vec<_>>()
                    } else {
                        // Deterministic policy + exploration noise.
                        let mu = self.policy.forward(&current_obs[i]);
                        mu.iter()
                            .map(|&m| {
                                hp.exploration_noise
                                    .mul_add(randn(&mut rng), m)
                                    .clamp(-1.0, 1.0)
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
                    .unwrap_or_else(|e| panic!("TD3: step failed: {e}"));

                epoch_steps += n_envs;

                // Store transitions and track episode boundaries.
                for i in 0..n_envs {
                    let reward = result.rewards[i];
                    let done = result.dones[i];
                    let truncated = result.truncateds[i];

                    // Next obs: terminal obs if episode ended, else post-reset obs.
                    let next_obs = if done || truncated {
                        result.terminal_observations[i].as_ref().map_or_else(
                            || result.observations.row(i).to_vec(),
                            |t| t.as_slice().to_vec(),
                        )
                    } else {
                        result.observations.row(i).to_vec()
                    };

                    buffer.push(
                        &current_obs[i],
                        &actions_f64[i],
                        reward,
                        &next_obs,
                        done, // Only true terminal — truncated bootstraps.
                    );

                    env_episode_reward[i] += reward;

                    if (done || truncated) && !env_complete[i] {
                        env_complete[i] = true;
                        epoch_rewards.push(env_episode_reward[i]);
                        if done {
                            epoch_done_count += 1;
                        }
                    }

                    // Update current obs (post auto-reset for done/truncated envs).
                    current_obs[i] = result.observations.row(i).to_vec();

                    // Reset episode reward on auto-reset.
                    if done || truncated {
                        env_episode_reward[i] = 0.0;
                    }
                }

                // ── Gradient updates (after warmup) ──────────────────────

                if buffer.len() >= hp.batch_size.max(hp.warmup_steps) {
                    let batch = buffer.sample(hp.batch_size, &mut rng);

                    // Compute target Q values.
                    // a' = target_policy(s') + clip(noise, -c, c)
                    let mut target_actions = Vec::with_capacity(hp.batch_size * act_dim);
                    for b in 0..hp.batch_size {
                        let next_obs = &batch.next_obs[b * obs_dim..(b + 1) * obs_dim];
                        let mu_target = self.target_policy.forward(next_obs);
                        for &m in &mu_target {
                            let noise = (hp.policy_noise * randn(&mut rng))
                                .clamp(-hp.noise_clip, hp.noise_clip);
                            target_actions.push((m + noise).clamp(-1.0, 1.0));
                        }
                    }

                    // y = r + γ(1-d) * min(Q1_tgt(s',a'), Q2_tgt(s',a'))
                    let mut targets = Vec::with_capacity(hp.batch_size);
                    for b in 0..hp.batch_size {
                        let next_obs = &batch.next_obs[b * obs_dim..(b + 1) * obs_dim];
                        let ta = &target_actions[b * act_dim..(b + 1) * act_dim];
                        let q1_val = self.target_q1.forward(next_obs, ta);
                        let q2_val = self.target_q2.forward(next_obs, ta);
                        let min_q = q1_val.min(q2_val);
                        let d = if batch.dones[b] { 1.0 } else { 0.0 };
                        targets.push((hp.gamma * (1.0 - d)).mul_add(min_q, batch.rewards[b]));
                    }

                    // Update Q1, Q2 with batched MSE gradient (in-place).
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

                    // Track Q losses (mean batch MSE).
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

                    total_critic_updates += 1;
                    epoch_updates += 1;

                    // Delayed policy update + target soft updates.
                    if total_critic_updates % hp.policy_delay == 0 {
                        // Actor gradient: dJ/dθ = (1/N) Σ dQ1/da · dμ/dθ
                        // Compute current actions for the batch observations.
                        let mut batch_actions = Vec::with_capacity(hp.batch_size * act_dim);
                        for b in 0..hp.batch_size {
                            let obs = &batch.obs[b * obs_dim..(b + 1) * obs_dim];
                            batch_actions.extend(self.policy.forward(obs));
                        }

                        // dQ1/da for each sample.
                        let dq_da = self.q1.action_gradient_batch(
                            &batch.obs,
                            &batch_actions,
                            obs_dim,
                            act_dim,
                        );

                        // Negate dQ/da because we want to maximize Q via the policy.
                        // VJP computes dμ/dθ · v where v = -dQ/da gives ascent.
                        let neg_dq_da: Vec<f64> = dq_da.iter().map(|&g| -g).collect();

                        let actor_grad = self
                            .policy
                            .forward_vjp_batch(&batch.obs, &neg_dq_da, obs_dim);

                        // Negate the result since forward_vjp_batch computes mean(v^T · dμ/dθ)
                        // with v = -dQ/da, giving -dJ/dθ. We want ascent on J.
                        // Actually: we pass neg_dq_da as v, so VJP = mean((-dQ/da)^T · dμ/dθ)
                        // = -mean(dQ/da · dμ/dθ) = -dJ/dθ.
                        // We want to maximize J, so we do ascent on dJ/dθ = -(-dJ/dθ).
                        // Equivalently: descent on (-dJ/dθ).
                        let mut ap = self.policy.params().to_vec();
                        actor_opt.step_in_place(&mut ap, &actor_grad, false);
                        self.policy.set_params(&ap);

                        epoch_policy_loss +=
                            dq_da.iter().map(|g| g * g).sum::<f64>().sqrt() / hp.batch_size as f64;

                        // Soft update all 3 target networks.
                        soft_update(&mut *self.target_q1, &*self.q1, hp.tau);
                        soft_update(&mut *self.target_q2, &*self.q2, hp.tau);
                        // Soft update target policy via raw params.
                        let src = self.policy.params();
                        let tgt = self.target_policy.params().to_vec();
                        let updated: Vec<f64> = tgt
                            .iter()
                            .zip(src)
                            .map(|(&t, &s)| hp.tau.mul_add(s, (1.0 - hp.tau) * t))
                            .collect();
                        self.target_policy.set_params(&updated);
                    }
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
                extra.insert("policy_loss".into(), epoch_policy_loss);
            }
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
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::{LinearPolicy, LinearQ, reaching_2dof};

    fn make_td3() -> (Td3, crate::TaskConfig) {
        let task = reaching_2dof();
        let od = task.obs_dim();
        let ad = task.act_dim();
        let sc = task.obs_scale();

        let algo = Td3::new(
            Box::new(LinearPolicy::new(od, ad, sc)),
            Box::new(LinearPolicy::new(od, ad, sc)),
            Box::new(LinearQ::new(od, ad, sc)),
            Box::new(LinearQ::new(od, ad, sc)),
            Box::new(LinearQ::new(od, ad, sc)),
            Box::new(LinearQ::new(od, ad, sc)),
            OptimizerConfig::adam(3e-4),
            Td3Hyperparams {
                gamma: 0.99,
                tau: 0.005,
                policy_noise: 0.2,
                noise_clip: 0.5,
                exploration_noise: 0.1,
                policy_delay: 2,
                batch_size: 32,
                buffer_capacity: 10_000,
                warmup_steps: 64,
                max_episode_steps: 300,
            },
        );
        (algo, task)
    }

    #[test]
    fn td3_name() {
        let (algo, _) = make_td3();
        assert_eq!(algo.name(), "TD3");
    }

    #[test]
    fn td3_smoke_2dof() {
        let (mut algo, task) = make_td3();
        let mut env = task.build_vec_env(5).unwrap();

        let metrics = algo.train(&mut env, TrainingBudget::Epochs(3), 42);

        assert_eq!(metrics.len(), 3);
        for (i, m) in metrics.iter().enumerate() {
            assert_eq!(m.epoch, i);
            assert!(m.total_steps > 0);
            assert!(m.extra.contains_key("buffer_size"));
        }
    }

    #[test]
    fn td3_buffer_grows() {
        let (mut algo, task) = make_td3();
        let mut env = task.build_vec_env(5).unwrap();

        let metrics = algo.train(&mut env, TrainingBudget::Epochs(2), 42);

        let buf_epoch0 = metrics[0].extra["buffer_size"];
        let buf_epoch1 = metrics[1].extra["buffer_size"];
        assert!(
            buf_epoch1 >= buf_epoch0,
            "buffer should grow: {buf_epoch0} -> {buf_epoch1}"
        );
    }

    #[test]
    fn td3_target_sync() {
        // Verify target params match primary params after construction.
        let task = reaching_2dof();
        let od = task.obs_dim();
        let ad = task.act_dim();
        let sc = task.obs_scale();

        let policy = Box::new(LinearPolicy::new(od, ad, sc));
        let target_policy = Box::new(LinearPolicy::new(od, ad, sc));
        let q1 = Box::new(LinearQ::new(od, ad, sc));
        let q2 = Box::new(LinearQ::new(od, ad, sc));
        let tq1 = Box::new(LinearQ::new(od, ad, sc));
        let tq2 = Box::new(LinearQ::new(od, ad, sc));

        let td3 = Td3::new(
            policy,
            target_policy,
            q1,
            q2,
            tq1,
            tq2,
            OptimizerConfig::adam(1e-3),
            Td3Hyperparams {
                gamma: 0.99,
                tau: 0.005,
                policy_noise: 0.2,
                noise_clip: 0.5,
                exploration_noise: 0.1,
                policy_delay: 2,
                batch_size: 32,
                buffer_capacity: 1000,
                warmup_steps: 32,
                max_episode_steps: 100,
            },
        );

        // After construction, targets should match primaries.
        assert_eq!(td3.policy.params(), td3.target_policy.params());
        assert_eq!(td3.q1.params(), td3.target_q1.params());
        assert_eq!(td3.q2.params(), td3.target_q2.params());
    }
}
