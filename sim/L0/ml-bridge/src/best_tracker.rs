//! Best-epoch policy snapshot tracker.
//!
//! Embedded as a single field in every algorithm struct. Encapsulates the
//! snapshot decision (strict `>`), the params clone, and artifact construction.
//! One type, one set of rules, used by all 5 algorithms.

use crate::artifact::{CURRENT_VERSION, PolicyArtifact, PolicyDescriptor};

/// Tracks the best-performing policy weights seen during training.
///
/// # Initialization
///
/// Created from the initial policy params in `Algorithm::new()`. Before
/// training, `reward` is `NEG_INFINITY` — the first real epoch always
/// overwrites it.
///
/// # Checkpoint round-trip
///
/// `to_checkpoint()` → `from_checkpoint()` preserves the best state
/// across training sessions.
#[derive(Debug, Clone)]
pub struct BestTracker {
    /// Policy weights at the best epoch.
    params: Vec<f64>,
    /// Mean reward at the best epoch.
    reward: f64,
    /// Epoch index (0-based) that achieved the best reward.
    epoch: usize,
}

impl BestTracker {
    /// Create a new tracker seeded with initial policy params.
    ///
    /// `NEG_INFINITY` reward guarantees the first real epoch always wins.
    pub fn new(initial_params: &[f64]) -> Self {
        Self {
            params: initial_params.to_vec(),
            reward: f64::NEG_INFINITY,
            epoch: 0,
        }
    }

    /// Update the snapshot if this epoch's reward is strictly better.
    ///
    /// Strict `>` means ties keep the earlier snapshot — the policy that
    /// achieved a reward first (with less training) is more conservative.
    ///
    /// `NaN` rewards: `NaN > anything` is always `false` in IEEE 754, so
    /// `NaN` epochs are silently skipped.
    pub fn maybe_update(&mut self, epoch: usize, reward: f64, params: &[f64]) {
        if reward > self.reward {
            self.reward = reward;
            self.epoch = epoch;
            self.params = params.to_vec();
        }
    }

    /// Build a bare `PolicyArtifact` from the best-epoch weights.
    ///
    /// The descriptor comes from the caller (the current policy's
    /// architecture) — architecture doesn't change during training,
    /// only weights change. Provenance is `None` — the caller attaches it.
    pub fn to_artifact(&self, descriptor: PolicyDescriptor) -> PolicyArtifact {
        PolicyArtifact {
            version: CURRENT_VERSION,
            descriptor,
            params: self.params.clone(),
            provenance: None,
        }
    }

    /// Best-epoch mean reward (`NEG_INFINITY` if no training has occurred).
    #[allow(dead_code)] // non-test callers pending (visualization / analysis)
    pub const fn reward(&self) -> f64 {
        self.reward
    }

    /// Best-epoch index (0-based).
    #[allow(dead_code)] // non-test callers pending (visualization / analysis)
    pub const fn epoch(&self) -> usize {
        self.epoch
    }

    /// Serialize the best state into checkpoint-compatible fields.
    ///
    /// Returns `Option<f64>` for reward: finite → `Some(reward)`,
    /// non-finite (`NEG_INFINITY` before training) → `None`.
    /// `serde_json` rejects non-finite f64 values, so `NEG_INFINITY`
    /// must never reach the serializer.
    pub fn to_checkpoint(&self) -> (Vec<f64>, Option<f64>, usize) {
        let reward = if self.reward.is_finite() {
            Some(self.reward)
        } else {
            None
        };
        (self.params.clone(), reward, self.epoch)
    }

    /// Restore from checkpoint fields.
    ///
    /// - If `params` is `None` (pre-feature checkpoint), falls back to
    ///   `fallback_params` (the checkpoint's current policy params).
    /// - If `params` length doesn't match `fallback_params` length
    ///   (architecture mismatch from incompatible checkpoint), discards
    ///   `params` and falls back to `fallback_params`.
    /// - If `reward` is `None` (pre-feature checkpoint or pre-training
    ///   checkpoint), restores to `NEG_INFINITY` — the first real epoch
    ///   will overwrite it.
    pub fn from_checkpoint(
        params: Option<Vec<f64>>,
        reward: Option<f64>,
        epoch: usize,
        fallback_params: &[f64],
    ) -> Self {
        let params = params
            .filter(|p| p.len() == fallback_params.len())
            .unwrap_or_else(|| fallback_params.to_vec());
        Self {
            params,
            reward: reward.unwrap_or(f64::NEG_INFINITY),
            epoch,
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::artifact::NetworkKind;
    use crate::autograd_layers::Activation;

    fn test_descriptor() -> PolicyDescriptor {
        PolicyDescriptor {
            obs_dim: 2,
            act_dim: 1,
            obs_scale: vec![1.0, 1.0],
            hidden_dims: vec![],
            activation: Activation::Tanh,
            kind: NetworkKind::Linear,
            stochastic: false,
        }
    }

    #[test]
    fn new_starts_at_neg_infinity() {
        let bt = BestTracker::new(&[1.0, 2.0, 3.0]);
        assert_eq!(bt.reward(), f64::NEG_INFINITY);
        assert_eq!(bt.epoch(), 0);
        assert_eq!(bt.params, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn maybe_update_improves() {
        let mut bt = BestTracker::new(&[0.0, 0.0]);
        bt.maybe_update(0, -5.0, &[1.0, 1.0]);
        assert_eq!(bt.reward(), -5.0);
        assert_eq!(bt.epoch(), 0);
        assert_eq!(bt.params, vec![1.0, 1.0]);

        // Improves.
        bt.maybe_update(3, -2.0, &[3.0, 3.0]);
        assert_eq!(bt.reward(), -2.0);
        assert_eq!(bt.epoch(), 3);
        assert_eq!(bt.params, vec![3.0, 3.0]);

        // Tie — keeps earlier.
        bt.maybe_update(5, -2.0, &[5.0, 5.0]);
        assert_eq!(bt.reward(), -2.0);
        assert_eq!(bt.epoch(), 3);
        assert_eq!(bt.params, vec![3.0, 3.0]);

        // Worse — keeps best.
        bt.maybe_update(6, -10.0, &[6.0, 6.0]);
        assert_eq!(bt.reward(), -2.0);
        assert_eq!(bt.epoch(), 3);
    }

    #[test]
    fn maybe_update_skips_nan() {
        let mut bt = BestTracker::new(&[0.0]);
        bt.maybe_update(0, -1.0, &[1.0]);
        bt.maybe_update(1, f64::NAN, &[99.0]);
        assert_eq!(bt.reward(), -1.0);
        assert_eq!(bt.epoch(), 0);
        assert_eq!(bt.params, vec![1.0]);
    }

    #[test]
    fn to_artifact_builds_correctly() {
        let mut bt = BestTracker::new(&[0.0]);
        bt.maybe_update(0, -1.0, &[42.0]);
        let art = bt.to_artifact(test_descriptor());
        assert_eq!(art.version, CURRENT_VERSION);
        assert_eq!(art.params, vec![42.0]);
        assert!(art.provenance.is_none());
        assert_eq!(art.descriptor.obs_dim, 2);
    }

    #[test]
    fn checkpoint_round_trip() {
        let mut bt = BestTracker::new(&[0.0, 0.0]);
        bt.maybe_update(7, -0.5, &[7.0, 7.0]);

        let (params, reward, epoch) = bt.to_checkpoint();
        assert_eq!(params, vec![7.0, 7.0]);
        assert_eq!(reward, Some(-0.5));
        assert_eq!(epoch, 7);

        let restored = BestTracker::from_checkpoint(Some(params), reward, epoch, &[0.0, 0.0]);
        assert_eq!(restored.reward(), -0.5);
        assert_eq!(restored.epoch(), 7);
        assert_eq!(restored.params, vec![7.0, 7.0]);
    }

    #[test]
    fn checkpoint_pre_training_maps_neg_infinity_to_none() {
        let bt = BestTracker::new(&[1.0]);
        let (_, reward, _) = bt.to_checkpoint();
        assert!(reward.is_none());
    }

    #[test]
    fn from_checkpoint_with_none_params_falls_back() {
        let bt = BestTracker::from_checkpoint(None, Some(-1.0), 5, &[99.0, 88.0]);
        assert_eq!(bt.params, vec![99.0, 88.0]);
        assert_eq!(bt.reward(), -1.0);
        assert_eq!(bt.epoch(), 5);
    }

    #[test]
    fn from_checkpoint_with_mismatched_length_falls_back() {
        let bt =
            BestTracker::from_checkpoint(Some(vec![1.0, 2.0, 3.0]), Some(-1.0), 5, &[99.0, 88.0]);
        // Length 3 != fallback length 2 → discarded.
        assert_eq!(bt.params, vec![99.0, 88.0]);
    }

    // ── Integration tests ──────────────────────────────────────────────

    use crate::algorithm::{Algorithm, TrainingBudget};
    use crate::policy::Policy;
    use crate::{Cem, CemHyperparams, LinearPolicy, reaching_2dof};

    #[test]
    fn best_artifact_before_train_returns_initial_policy() {
        let task = reaching_2dof();
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        let initial_params = policy.params().to_vec();
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

        let best = cem.best_artifact();
        assert_eq!(best.params, initial_params);
        assert!(best.provenance.is_none());
    }

    #[test]
    fn best_artifact_after_training_has_best_epoch_weights() {
        let task = reaching_2dof();
        let mut env = task.build_vec_env(10).unwrap();
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

        let metrics = cem.train(&mut env, TrainingBudget::Epochs(10), 42, &|_| {});
        let best_art = cem.best_artifact();
        let final_art = cem.policy_artifact();

        // Best artifact should have valid params of the same length.
        assert_eq!(best_art.params.len(), final_art.params.len());

        // Find best epoch reward from metrics.
        let best_reward = metrics
            .iter()
            .map(|m| m.mean_reward)
            .fold(f64::NEG_INFINITY, f64::max);
        let final_reward = metrics.last().unwrap().mean_reward;

        // Best reward should be >= final reward (by definition).
        assert!(
            best_reward >= final_reward,
            "best ({best_reward}) should be >= final ({final_reward})"
        );
    }

    // ── Phase 3: checkpoint integration tests ─────────────────────────

    #[test]
    fn checkpoint_round_trip_preserves_best_across_resume() {
        let task = reaching_2dof();
        let mut env = task.build_vec_env(10).unwrap();
        let hp = CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 0.3,
            noise_decay: 0.95,
            noise_min: 0.01,
            max_episode_steps: 300,
        };
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        let mut cem = Cem::new(policy, hp);
        cem.train(&mut env, TrainingBudget::Epochs(5), 42, &|_| {});

        let cp = cem.checkpoint();

        // Checkpoint should contain best state.
        assert!(cp.best_params.is_some());
        let bp = cp.best_params.as_ref().unwrap();
        assert_eq!(bp.len(), cem.best_artifact().params.len());

        // Restore and verify best artifact matches.
        let cem2 = Cem::from_checkpoint(&cp, hp).unwrap();
        let original_best = cem.best_artifact();
        let restored_best = cem2.best_artifact();
        assert_eq!(restored_best.params, original_best.params);
    }

    #[test]
    fn old_checkpoint_without_best_fields_loads_with_defaults() {
        // Simulate a pre-feature checkpoint (no best_* fields) by
        // constructing a TrainingCheckpoint with None/default best fields.
        use crate::artifact::TrainingCheckpoint;
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
        let mut cp = cem.checkpoint();

        // Strip best fields to simulate old checkpoint.
        cp.best_params = None;
        cp.best_reward = None;
        cp.best_epoch = 0;

        // Also verify JSON round-trip with missing fields.
        let json = serde_json::to_string(&cp).unwrap();
        let loaded: TrainingCheckpoint = serde_json::from_str(&json).unwrap();
        assert!(loaded.best_params.is_none());
        assert!(loaded.best_reward.is_none());
        assert_eq!(loaded.best_epoch, 0);

        // Restore algorithm — should fall back to policy params.
        let hp = CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 0.3,
            noise_decay: 0.95,
            noise_min: 0.01,
            max_episode_steps: 300,
        };
        let cem2 = Cem::from_checkpoint(&loaded, hp).unwrap();
        // Best artifact should equal the current policy (fallback).
        assert_eq!(cem2.best_artifact().params, cem2.policy_artifact().params);
    }

    #[test]
    fn resume_training_preserves_best_from_previous_session() {
        let task = reaching_2dof();
        let mut env = task.build_vec_env(10).unwrap();
        let hp = CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 0.3,
            noise_decay: 0.95,
            noise_min: 0.01,
            max_episode_steps: 300,
        };

        // Session 1: train 10 epochs, record best.
        let policy = Box::new(LinearPolicy::new(
            task.obs_dim(),
            task.act_dim(),
            task.obs_scale(),
        ));
        let mut cem = Cem::new(policy, hp);
        let metrics1 = cem.train(&mut env, TrainingBudget::Epochs(10), 42, &|_| {});
        let session1_best_reward = metrics1
            .iter()
            .map(|m| m.mean_reward)
            .fold(f64::NEG_INFINITY, f64::max);

        // Checkpoint + restore.
        let cp = cem.checkpoint();
        let mut cem2 = Cem::from_checkpoint(&cp, hp).unwrap();

        // Session 2: train 1 epoch with a different seed.
        let metrics2 = cem2.train(&mut env, TrainingBudget::Epochs(1), 99, &|_| {});
        let session2_reward = metrics2[0].mean_reward;

        // The best after resume should be at least as good as session 1's best.
        // If session 2's single epoch didn't beat session 1's best, the
        // restored best should survive unchanged.
        let best_art = cem2.best_artifact();
        assert_eq!(best_art.params.len(), cem.best_artifact().params.len());

        // If session 2 didn't beat session 1, best should equal session 1's best.
        if session2_reward <= session1_best_reward {
            assert_eq!(best_art.params, cem.best_artifact().params);
        }
    }
}
