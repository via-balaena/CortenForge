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
    #[must_use]
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
    #[must_use]
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
    #[must_use]
    pub const fn reward(&self) -> f64 {
        self.reward
    }

    /// Best-epoch index (0-based).
    #[allow(dead_code)] // non-test callers pending (visualization / analysis)
    #[must_use]
    pub const fn epoch(&self) -> usize {
        self.epoch
    }

    /// Serialize the best state into checkpoint-compatible fields.
    ///
    /// Returns `Option<f64>` for reward: finite → `Some(reward)`,
    /// non-finite (`NEG_INFINITY` before training) → `None`.
    /// `serde_json` rejects non-finite f64 values, so `NEG_INFINITY`
    /// must never reach the serializer.
    #[must_use]
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
    #[must_use]
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
}
