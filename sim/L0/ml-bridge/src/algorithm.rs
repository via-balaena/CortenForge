//! Algorithm trait and training types.
//!
//! The [`Algorithm`] trait has ONE method: [`train`](Algorithm::train). Each
//! algorithm owns its entire training loop — CEM perturbs params, REINFORCE
//! estimates gradients from returns, PPO clips surrogate objectives, SAC
//! maximizes entropy. The trait doesn't know or care how they work.
//!
//! `Algorithm::train()` is for headless competition runs and batch
//! experiments. Visual Bevy examples implement their own training loops
//! using the shared components (`compute_gae`, `ReplayBuffer`, `Adam`,
//! etc.) directly — they do not go through `Algorithm::train()`.
//!
//! See the [spec](../../docs/ML_COMPETITION_SPEC.md) for the full rationale
//! behind the monolithic `train()` design.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::vec_env::VecEnv;

// ── Training types ─────────────────────────────────────────────────────────

/// Metrics for one training epoch.
///
/// Reported by [`Algorithm::train`] after each epoch. The `extra` field is
/// the diagnostics port — algorithms add keys (e.g., `"policy_loss"`,
/// `"entropy"`, `"clip_fraction"`), never remove them. Competition tests
/// that check specific keys continue working when new keys appear.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EpochMetrics {
    /// Epoch index (0-based).
    pub epoch: usize,
    /// Mean reward across all episodes this epoch.
    pub mean_reward: f64,
    /// Number of episodes that reached a true terminal state.
    pub done_count: usize,
    /// Total environment steps taken this epoch.
    pub total_steps: usize,
    /// Wall-clock time for this epoch in milliseconds.
    pub wall_time_ms: u64,
    /// Algorithm-specific metrics. Append-only — never remove keys.
    pub extra: BTreeMap<String, f64>,
}

/// How long to train.
#[derive(Debug, Clone, Copy)]
pub enum TrainingBudget {
    /// Train for a fixed number of epochs.
    Epochs(usize),
    /// Train for a fixed number of environment steps.
    Steps(usize),
}

// ── Algorithm trait ────────────────────────────────────────────────────────

/// Core algorithm trait. Each algorithm owns its entire training loop.
///
/// The algorithm receives a ready-to-use [`VecEnv`] — it never constructs
/// its own environment. This follows plumbing rule 1: algorithms accept
/// parts, never construct them.
///
/// # Examples
///
/// ```ignore
/// // PPO construction — all parts are swappable
/// let mut ppo = Ppo::new(
///     Box::new(MlpPolicy::new(obs_dim, 32, act_dim, &obs_scale)),
///     Box::new(MlpValue::new(obs_dim, 32, &obs_scale)),
///     OptimizerConfig::adam(0.025),
///     PpoHyperparams { clip_eps: 0.2, k_passes: 2, gamma: 0.99, .. },
/// );
///
/// let metrics = ppo.train(&mut env, TrainingBudget::Epochs(30), seed, &|_| {});
/// ```
pub trait Algorithm: Send {
    /// Algorithm name (e.g., `"CEM"`, `"PPO"`, `"SAC"`).
    fn name(&self) -> &'static str;

    /// Run the full training loop.
    ///
    /// The algorithm steps `env`, manages its own data structures
    /// (trajectories, replay buffers, populations), and returns
    /// standardized metrics for each epoch.
    ///
    /// `on_epoch` is called after each epoch with that epoch's metrics.
    /// Pass `&|_| {}` for silent operation, or a logging callback for
    /// real-time training visibility.
    fn train(
        &mut self,
        env: &mut VecEnv,
        budget: TrainingBudget,
        seed: u64,
        on_epoch: &dyn Fn(&EpochMetrics),
    ) -> Vec<EpochMetrics>;
}

// ── tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn boxed_algorithm_is_send() {
        fn require<T: Send>() {}
        require::<Box<dyn Algorithm>>();
    }

    #[test]
    fn epoch_metrics_debug_clone() {
        let m = EpochMetrics {
            epoch: 0,
            mean_reward: 1.5,
            done_count: 3,
            total_steps: 100,
            wall_time_ms: 42,
            extra: BTreeMap::new(),
        };
        let m2 = m.clone();
        assert_eq!(m2.epoch, 0);
        assert!(!format!("{m:?}").is_empty());
    }

    #[test]
    fn training_budget_copy() {
        let b = TrainingBudget::Epochs(10);
        let b2 = b; // Copy
        match (b, b2) {
            (TrainingBudget::Epochs(a), TrainingBudget::Epochs(c)) => assert_eq!(a, c),
            _ => panic!("expected Epochs"),
        }
    }
}
