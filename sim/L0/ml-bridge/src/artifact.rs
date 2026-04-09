//! Policy artifact types — the portable currency of the ML domain.
//!
//! An artifact captures everything needed to reconstruct and understand
//! a trained policy: architecture, weights, and provenance.
//!
//! # Key types
//!
//! - [`PolicyArtifact`] — the deployable brain. Save, load, reconstruct.
//! - [`PolicyDescriptor`] — architecture recipe for policies.
//! - [`NetworkDescriptor`] — architecture recipe for value/Q networks.
//! - [`TrainingProvenance`] — lineage: who trained it, how, and when.
//! - [`NetworkKind`] — shared enum across all network types.
//! - [`ArtifactError`] — dedicated error type for artifact operations.

use std::collections::BTreeMap;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::algorithm::EpochMetrics;
use crate::autograd_layers::Activation;
use crate::autograd_policy::{AutogradPolicy, AutogradStochasticPolicy};
use crate::autograd_value::{AutogradQ, AutogradValue};
use crate::linear::{LinearPolicy, LinearQ, LinearStochasticPolicy, LinearValue};
use crate::mlp::{MlpPolicy, MlpQ, MlpValue};
use crate::optimizer::OptimizerConfig;
use crate::policy::{DifferentiablePolicy, Policy, StochasticPolicy};
use crate::value::{QFunction, ValueFn};

// ── Version ───────────────────────────────────────────────────────────────

/// Current artifact format version.
pub const CURRENT_VERSION: u32 = 1;

// ── NetworkKind ───────────────────────────────────────────────────────────

/// Which concrete network type to construct.
///
/// Shared by policies, value functions, and Q-functions. The distinction
/// between "a Linear policy" and "a Linear value function" is in the
/// descriptor type, not the kind.
///
/// `#[non_exhaustive]` — future kinds (Cnn, Rnn, Transformer) can be
/// added without breaking existing serialized files or match arms.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum NetworkKind {
    /// Hand-coded linear (no hidden layers).
    Linear,
    /// Single-hidden-layer MLP with hand-coded backprop.
    Mlp,
    /// Arbitrary-depth network with autograd-backed gradients.
    Autograd,
}

// ── PolicyDescriptor ──────────────────────────────────────────────────────

/// Architecture recipe — enough to reconstruct an empty policy with
/// the same structure.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PolicyDescriptor {
    /// Which concrete type to construct.
    pub kind: NetworkKind,
    /// Observation dimensionality.
    pub obs_dim: usize,
    /// Action dimensionality.
    pub act_dim: usize,
    /// Hidden layer sizes. `[]` for Linear, `[H]` for Mlp, `[H1, H2, ...]` for Autograd.
    pub hidden_dims: Vec<usize>,
    /// Activation function for hidden layers.
    pub activation: Activation,
    /// Per-observation-dimension scaling factors.
    pub obs_scale: Vec<f64>,
    /// Whether this policy has learned exploration (`log_std` as parameter).
    pub stochastic: bool,
}

// ── NetworkDescriptor ─────────────────────────────────────────────────────

/// Architecture recipe for value functions and Q-functions.
///
/// Same pattern as [`PolicyDescriptor`] but for critics. No `stochastic`
/// flag — critics don't have learned exploration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NetworkDescriptor {
    /// Which concrete type to construct.
    pub kind: NetworkKind,
    /// Observation dimensionality.
    pub obs_dim: usize,
    /// Action dimensionality (`Some` for Q-functions, `None` for V-functions).
    pub act_dim: Option<usize>,
    /// Hidden layer sizes.
    pub hidden_dims: Vec<usize>,
    /// Activation function for hidden layers.
    pub activation: Activation,
    /// Per-observation-dimension scaling factors.
    pub obs_scale: Vec<f64>,
}

impl From<PolicyDescriptor> for NetworkDescriptor {
    fn from(pd: PolicyDescriptor) -> Self {
        Self {
            kind: pd.kind,
            obs_dim: pd.obs_dim,
            act_dim: Some(pd.act_dim),
            hidden_dims: pd.hidden_dims,
            activation: pd.activation,
            obs_scale: pd.obs_scale,
        }
    }
}

// ── TrainingProvenance ────────────────────────────────────────────────────

/// Lineage record — who trained the policy, how, and when.
///
/// Built by the caller, not the algorithm. The algorithm knows the policy
/// state. The caller (competition runner, Bevy example) knows the task
/// name, seed, hyperparams, and has the returned metrics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingProvenance {
    /// Algorithm that produced these weights (e.g., "CEM", "TD3").
    pub algorithm: String,
    /// Task trained on (e.g., "reaching-6dof").
    pub task: String,
    /// RNG seed used for training.
    pub seed: u64,
    /// Number of epochs completed.
    pub epochs_trained: usize,
    /// Final epoch's mean reward.
    pub final_reward: f64,
    /// Total environment steps across all epochs.
    pub total_steps: usize,
    /// Total wall-clock training time in milliseconds.
    pub wall_time_ms: u64,
    /// ISO 8601 timestamp of when training completed.
    pub timestamp: String,
    /// Algorithm-specific hyperparameters.
    pub hyperparams: BTreeMap<String, f64>,
    /// Full training curve — every epoch's metrics.
    pub metrics: Vec<EpochMetrics>,
    /// If these weights were initialized from a prior artifact (curriculum, transfer).
    #[serde(default)]
    pub parent: Option<Box<Self>>,
}

// ── ArtifactError ─────────────────────────────────────────────────────────

/// Errors from artifact operations (save, load, validate, reconstruct).
#[derive(Debug, thiserror::Error)]
pub enum ArtifactError {
    /// Artifact version is newer than this code supports.
    #[error("unsupported artifact version {found} (max supported: {max})")]
    UnsupportedVersion {
        /// Version found in the artifact.
        found: u32,
        /// Maximum version this code can read.
        max: u32,
    },

    /// `NetworkKind` variant not recognized (future version).
    #[error("unknown network kind — artifact may have been written by a newer version")]
    UnknownKind,

    /// Descriptor implies a different param count than the artifact contains.
    #[error("param count mismatch: descriptor implies {expected}, artifact has {actual}")]
    ParamCountMismatch {
        /// Param count implied by the descriptor.
        expected: usize,
        /// Actual param count in the artifact.
        actual: usize,
    },

    /// `obs_scale` length doesn't match `obs_dim`.
    #[error("obs_scale length {actual} doesn't match obs_dim {expected}")]
    ObsScaleMismatch {
        /// Expected length (`obs_dim`).
        expected: usize,
        /// Actual length of `obs_scale`.
        actual: usize,
    },

    /// Linear kind must have empty `hidden_dims`.
    #[error("hidden_dims must be empty for Linear kind, got {0} layers")]
    LinearHiddenDims(usize),

    /// Mlp/Autograd kinds require non-empty `hidden_dims`.
    #[error("hidden_dims must be non-empty for {kind:?} kind")]
    MissingHiddenDims {
        /// The network kind that requires hidden dims.
        kind: NetworkKind,
    },

    /// No concrete type exists for the requested (kind, stochastic) combination.
    #[error("unsupported combination: kind={kind:?} with stochastic={stochastic}")]
    UnsupportedCombination {
        /// The network kind.
        kind: NetworkKind,
        /// Whether stochastic mode was requested.
        stochastic: bool,
    },

    /// An `f64` field contains `NaN` or `Infinity`.
    #[error("non-finite f64 value in field `{field}`")]
    NonFiniteValue {
        /// Path to the offending field (e.g., `"params"`, `"provenance.final_reward"`).
        field: String,
    },

    /// File I/O error.
    #[error(transparent)]
    Io(#[from] std::io::Error),

    /// JSON serialization/deserialization error.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
}

// ── PolicyArtifact ────────────────────────────────────────────────────────

/// The deployable brain — architecture + weights + provenance.
///
/// This is the foundational currency of the ML domain. Every algorithm
/// produces one, every deployment consumes one.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PolicyArtifact {
    /// Format version — increment on structural changes.
    pub version: u32,
    /// Architecture — enough to reconstruct an empty policy.
    pub descriptor: PolicyDescriptor,
    /// Learned weights — length must match descriptor's implied param count.
    pub params: Vec<f64>,
    /// How these weights were produced. `None` for hand-crafted initial weights.
    #[serde(default)]
    pub provenance: Option<TrainingProvenance>,
}

/// Compute expected parameter count from a policy descriptor.
#[allow(unreachable_patterns)] // NetworkKind is #[non_exhaustive] — wildcard needed for future kinds
fn compute_param_count(d: &PolicyDescriptor) -> usize {
    match (d.kind, d.stochastic) {
        // W[act_dim × obs_dim] + b[act_dim]
        (NetworkKind::Linear, false) => d.obs_dim * d.act_dim + d.act_dim,

        // W[act_dim × obs_dim] + b[act_dim] + log_std[act_dim]
        (NetworkKind::Linear, true) => d.obs_dim * d.act_dim + 2 * d.act_dim,

        // W1[H × obs_dim] + b1[H] + W2[act_dim × H] + b2[act_dim]
        (NetworkKind::Mlp, false) => {
            let h = d.hidden_dims[0];
            d.obs_dim * h + h + h * d.act_dim + d.act_dim
        }

        // General multi-layer: sizes = [obs_dim, H1, ..., Hn, act_dim]
        (NetworkKind::Autograd, false) => {
            let mut sizes = Vec::with_capacity(d.hidden_dims.len() + 2);
            sizes.push(d.obs_dim);
            sizes.extend_from_slice(&d.hidden_dims);
            sizes.push(d.act_dim);
            sizes.windows(2).map(|w| w[0] * w[1] + w[1]).sum()
        }

        // Same as above + log_std[act_dim]
        (NetworkKind::Autograd, true) => {
            let mut sizes = Vec::with_capacity(d.hidden_dims.len() + 2);
            sizes.push(d.obs_dim);
            sizes.extend_from_slice(&d.hidden_dims);
            sizes.push(d.act_dim);
            let base: usize = sizes.windows(2).map(|w| w[0] * w[1] + w[1]).sum();
            base + d.act_dim
        }

        // Future kinds — can't compute.
        _ => 0,
    }
}

impl PolicyArtifact {
    /// Create an artifact from any live policy (no provenance).
    pub fn from_policy(policy: &dyn Policy) -> Self {
        Self {
            version: CURRENT_VERSION,
            descriptor: policy.descriptor(),
            params: policy.params().to_vec(),
            provenance: None,
        }
    }

    /// Attach provenance to this artifact. Returns self for chaining.
    #[must_use]
    pub fn with_provenance(mut self, provenance: TrainingProvenance) -> Self {
        self.provenance = Some(provenance);
        self
    }

    /// Validate internal consistency.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if the artifact has an unsupported version,
    /// mismatched dimensions, invalid kind/stochastic combination, wrong
    /// param count, or non-finite values.
    #[allow(unreachable_patterns)] // NetworkKind is #[non_exhaustive]
    pub fn validate(&self) -> Result<(), ArtifactError> {
        // 1. Version check
        if self.version > CURRENT_VERSION {
            return Err(ArtifactError::UnsupportedVersion {
                found: self.version,
                max: CURRENT_VERSION,
            });
        }
        // 2. obs_scale length matches obs_dim
        if self.descriptor.obs_scale.len() != self.descriptor.obs_dim {
            return Err(ArtifactError::ObsScaleMismatch {
                expected: self.descriptor.obs_dim,
                actual: self.descriptor.obs_scale.len(),
            });
        }
        // 3. hidden_dims consistency with kind
        match self.descriptor.kind {
            NetworkKind::Linear => {
                if !self.descriptor.hidden_dims.is_empty() {
                    return Err(ArtifactError::LinearHiddenDims(
                        self.descriptor.hidden_dims.len(),
                    ));
                }
            }
            NetworkKind::Mlp | NetworkKind::Autograd => {
                if self.descriptor.hidden_dims.is_empty() {
                    return Err(ArtifactError::MissingHiddenDims {
                        kind: self.descriptor.kind,
                    });
                }
            }
            // Future kinds — can't validate structure, but to_policy()
            // will return UnknownKind if it can't reconstruct.
            _ => {}
        }
        // 4. Unsupported (kind, stochastic) combinations
        if matches!(self.descriptor.kind, NetworkKind::Mlp) && self.descriptor.stochastic {
            return Err(ArtifactError::UnsupportedCombination {
                kind: self.descriptor.kind,
                stochastic: self.descriptor.stochastic,
            });
        }
        // 5. Param count
        let expected = compute_param_count(&self.descriptor);
        if self.params.len() != expected {
            return Err(ArtifactError::ParamCountMismatch {
                expected,
                actual: self.params.len(),
            });
        }
        // 6. Finite-value check — reject NaN/Infinity before serialization
        if self.params.iter().any(|v| !v.is_finite()) {
            return Err(ArtifactError::NonFiniteValue {
                field: "params".into(),
            });
        }
        if let Some(ref prov) = self.provenance {
            if !prov.final_reward.is_finite() {
                return Err(ArtifactError::NonFiniteValue {
                    field: "provenance.final_reward".into(),
                });
            }
            for (i, m) in prov.metrics.iter().enumerate() {
                if !m.mean_reward.is_finite() {
                    return Err(ArtifactError::NonFiniteValue {
                        field: format!("provenance.metrics[{i}].mean_reward"),
                    });
                }
            }
        }
        Ok(())
    }

    /// Reconstruct a live policy from this artifact.
    ///
    /// The returned policy is ready for `forward()` — inference/replay only.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if validation fails, the kind is unknown,
    /// or the (kind, stochastic) combination is unsupported.
    #[allow(unreachable_patterns)] // NetworkKind is #[non_exhaustive]
    pub fn to_policy(&self) -> Result<Box<dyn Policy>, ArtifactError> {
        self.validate()?;
        let d = &self.descriptor;
        let mut policy: Box<dyn Policy> = match (d.kind, d.stochastic) {
            (NetworkKind::Linear, false) => {
                Box::new(LinearPolicy::new(d.obs_dim, d.act_dim, &d.obs_scale))
            }
            (NetworkKind::Linear, true) => Box::new(LinearStochasticPolicy::new(
                d.obs_dim,
                d.act_dim,
                &d.obs_scale,
                0.0,
            )),
            (NetworkKind::Mlp, false) => Box::new(MlpPolicy::new(
                d.obs_dim,
                d.hidden_dims[0],
                d.act_dim,
                &d.obs_scale,
            )),
            (NetworkKind::Mlp, true) => {
                return Err(ArtifactError::UnsupportedCombination {
                    kind: d.kind,
                    stochastic: d.stochastic,
                });
            }
            (NetworkKind::Autograd, false) => Box::new(AutogradPolicy::new_with(
                d.obs_dim,
                &d.hidden_dims,
                d.act_dim,
                &d.obs_scale,
                d.activation,
            )),
            (NetworkKind::Autograd, true) => Box::new(AutogradStochasticPolicy::new_with(
                d.obs_dim,
                &d.hidden_dims,
                d.act_dim,
                &d.obs_scale,
                0.0,
                d.activation,
            )),
            _ => return Err(ArtifactError::UnknownKind),
        };
        policy.set_params(&self.params);
        Ok(policy)
    }

    /// Save to JSON file. Validates before serialization.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if validation fails or the file can't be written.
    pub fn save(&self, path: impl AsRef<Path>) -> Result<(), ArtifactError> {
        self.validate()?;
        let json = serde_json::to_string_pretty(self)?;
        std::fs::write(path, json)?;
        Ok(())
    }

    /// Load from JSON file. Validates version on load.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if the file can't be read, JSON is invalid,
    /// or the artifact version is unsupported.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, ArtifactError> {
        let json = std::fs::read_to_string(path)?;
        let artifact: Self = serde_json::from_str(&json)?;
        if artifact.version > CURRENT_VERSION {
            return Err(ArtifactError::UnsupportedVersion {
                found: artifact.version,
                max: CURRENT_VERSION,
            });
        }
        Ok(artifact)
    }
}

// ── Checkpoint types ─────────────────────────────────────────────────────

/// Snapshot of a single network's state (critic, target, etc.).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NetworkSnapshot {
    /// Role in the algorithm (e.g., `"value"`, `"q1"`, `"q2_target"`, `"actor_target"`).
    pub role: String,
    /// Architecture descriptor.
    pub descriptor: NetworkDescriptor,
    /// Learned weights.
    pub params: Vec<f64>,
}

impl NetworkSnapshot {
    /// Reconstruct a live value function from this snapshot.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if the kind is unknown.
    #[allow(unreachable_patterns)] // NetworkKind is #[non_exhaustive]
    pub fn to_value_fn(&self) -> Result<Box<dyn ValueFn>, ArtifactError> {
        let d = &self.descriptor;
        let mut vf: Box<dyn ValueFn> = match d.kind {
            NetworkKind::Linear => Box::new(LinearValue::new(d.obs_dim, &d.obs_scale)),
            NetworkKind::Mlp => Box::new(MlpValue::new(d.obs_dim, d.hidden_dims[0], &d.obs_scale)),
            NetworkKind::Autograd => Box::new(AutogradValue::new_with(
                d.obs_dim,
                &d.hidden_dims,
                &d.obs_scale,
                d.activation,
            )),
            _ => return Err(ArtifactError::UnknownKind),
        };
        vf.set_params(&self.params);
        Ok(vf)
    }

    /// Reconstruct a live Q-function from this snapshot.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if the kind is unknown or `act_dim` is missing.
    #[allow(unreachable_patterns)] // NetworkKind is #[non_exhaustive]
    pub fn to_q_function(&self) -> Result<Box<dyn QFunction>, ArtifactError> {
        let d = &self.descriptor;
        let act_dim = d.act_dim.ok_or(ArtifactError::ParamCountMismatch {
            expected: 1,
            actual: 0,
        })?;
        let mut qf: Box<dyn QFunction> = match d.kind {
            NetworkKind::Linear => Box::new(LinearQ::new(d.obs_dim, act_dim, &d.obs_scale)),
            NetworkKind::Mlp => Box::new(MlpQ::new(
                d.obs_dim,
                d.hidden_dims[0],
                act_dim,
                &d.obs_scale,
            )),
            NetworkKind::Autograd => Box::new(AutogradQ::new_with(
                d.obs_dim,
                &d.hidden_dims,
                act_dim,
                &d.obs_scale,
                d.activation,
            )),
            _ => return Err(ArtifactError::UnknownKind),
        };
        qf.set_params(&self.params);
        Ok(qf)
    }

    /// Reconstruct a live differentiable policy from this snapshot.
    ///
    /// Used for TD3 target policy reconstruction from checkpoint.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if the kind is unknown.
    #[allow(unreachable_patterns)] // NetworkKind is #[non_exhaustive]
    pub fn to_differentiable_policy(&self) -> Result<Box<dyn DifferentiablePolicy>, ArtifactError> {
        let d = &self.descriptor;
        let act_dim = d.act_dim.ok_or(ArtifactError::ParamCountMismatch {
            expected: 1,
            actual: 0,
        })?;
        let mut policy: Box<dyn DifferentiablePolicy> = match d.kind {
            NetworkKind::Linear => Box::new(LinearPolicy::new(d.obs_dim, act_dim, &d.obs_scale)),
            NetworkKind::Mlp => Box::new(MlpPolicy::new(
                d.obs_dim,
                d.hidden_dims[0],
                act_dim,
                &d.obs_scale,
            )),
            NetworkKind::Autograd => Box::new(AutogradPolicy::new_with(
                d.obs_dim,
                &d.hidden_dims,
                act_dim,
                &d.obs_scale,
                d.activation,
            )),
            _ => return Err(ArtifactError::UnknownKind),
        };
        policy.set_params(&self.params);
        Ok(policy)
    }
}

/// Snapshot of an optimizer's internal state (momentum, variance, step count).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimizerSnapshot {
    /// Which network this optimizer trains (e.g., "actor", "q1", "value").
    pub role: String,
    /// Optimizer configuration at time of snapshot.
    pub config: OptimizerConfig,
    /// First moment estimates (Adam m).
    pub m: Vec<f64>,
    /// Second moment estimates (Adam v).
    pub v: Vec<f64>,
    /// Step count (Adam t).
    pub t: usize,
}

/// Full training state for resuming across sessions.
///
/// Contains everything needed to reconstruct an algorithm instance and
/// continue training without regression. Each algorithm implements
/// `checkpoint()` to produce this, and has a `from_checkpoint()` constructor
/// to restore from it.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingCheckpoint {
    /// Which algorithm produced this checkpoint (e.g., "CEM", "TD3").
    pub algorithm_name: String,
    /// The trained policy at this point.
    pub policy_artifact: PolicyArtifact,
    /// Critic networks (V for PPO; Q1, Q2, targets for TD3/SAC).
    pub critics: Vec<NetworkSnapshot>,
    /// Optimizer states (one per optimizer instance).
    pub optimizer_states: Vec<OptimizerSnapshot>,
    /// Algorithm-specific scalar state (`noise_std`, `sigma`, `alpha`, etc.).
    pub algorithm_state: BTreeMap<String, f64>,
}

impl TrainingCheckpoint {
    /// Save to JSON file.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if the file can't be written or the embedded
    /// policy artifact has non-finite values.
    pub fn save(&self, path: impl AsRef<Path>) -> Result<(), ArtifactError> {
        let json = serde_json::to_string_pretty(self)?;
        std::fs::write(path, json)?;
        Ok(())
    }

    /// Load from JSON file.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if the file can't be read, JSON is invalid,
    /// or the embedded policy artifact version is unsupported.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, ArtifactError> {
        let json = std::fs::read_to_string(path)?;
        let checkpoint: Self = serde_json::from_str(&json)?;
        if checkpoint.policy_artifact.version > CURRENT_VERSION {
            return Err(ArtifactError::UnsupportedVersion {
                found: checkpoint.policy_artifact.version,
                max: CURRENT_VERSION,
            });
        }
        Ok(checkpoint)
    }
}

// ── Reconstruction helpers on PolicyArtifact ─────────────────────────────

impl PolicyArtifact {
    /// Reconstruct a live differentiable policy from this artifact.
    ///
    /// All current policy types implement `DifferentiablePolicy`, so this
    /// works for every valid `(kind, stochastic)` combination.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if validation fails, the kind is unknown,
    /// or the (kind, stochastic) combination is unsupported.
    #[allow(unreachable_patterns)] // NetworkKind is #[non_exhaustive]
    pub fn to_differentiable_policy(&self) -> Result<Box<dyn DifferentiablePolicy>, ArtifactError> {
        self.validate()?;
        let d = &self.descriptor;
        let mut policy: Box<dyn DifferentiablePolicy> = match (d.kind, d.stochastic) {
            (NetworkKind::Linear, false) => {
                Box::new(LinearPolicy::new(d.obs_dim, d.act_dim, &d.obs_scale))
            }
            (NetworkKind::Linear, true) => Box::new(LinearStochasticPolicy::new(
                d.obs_dim,
                d.act_dim,
                &d.obs_scale,
                0.0,
            )),
            (NetworkKind::Mlp, false) => Box::new(MlpPolicy::new(
                d.obs_dim,
                d.hidden_dims[0],
                d.act_dim,
                &d.obs_scale,
            )),
            (NetworkKind::Mlp, true) => {
                return Err(ArtifactError::UnsupportedCombination {
                    kind: d.kind,
                    stochastic: d.stochastic,
                });
            }
            (NetworkKind::Autograd, false) => Box::new(AutogradPolicy::new_with(
                d.obs_dim,
                &d.hidden_dims,
                d.act_dim,
                &d.obs_scale,
                d.activation,
            )),
            (NetworkKind::Autograd, true) => Box::new(AutogradStochasticPolicy::new_with(
                d.obs_dim,
                &d.hidden_dims,
                d.act_dim,
                &d.obs_scale,
                0.0,
                d.activation,
            )),
            _ => return Err(ArtifactError::UnknownKind),
        };
        policy.set_params(&self.params);
        Ok(policy)
    }

    /// Reconstruct a live stochastic policy from this artifact.
    ///
    /// Only valid for `stochastic: true` artifacts. Returns
    /// `ArtifactError::UnsupportedCombination` for non-stochastic descriptors.
    ///
    /// # Errors
    ///
    /// Returns `ArtifactError` if validation fails, the kind is unknown,
    /// or the descriptor is not stochastic.
    #[allow(unreachable_patterns)] // NetworkKind is #[non_exhaustive]
    pub fn to_stochastic_policy(&self) -> Result<Box<dyn StochasticPolicy>, ArtifactError> {
        self.validate()?;
        let d = &self.descriptor;
        if !d.stochastic {
            return Err(ArtifactError::UnsupportedCombination {
                kind: d.kind,
                stochastic: false,
            });
        }
        let mut policy: Box<dyn StochasticPolicy> = match d.kind {
            NetworkKind::Linear => Box::new(LinearStochasticPolicy::new(
                d.obs_dim,
                d.act_dim,
                &d.obs_scale,
                0.0,
            )),
            NetworkKind::Autograd => Box::new(AutogradStochasticPolicy::new_with(
                d.obs_dim,
                &d.hidden_dims,
                d.act_dim,
                &d.obs_scale,
                0.0,
                d.activation,
            )),
            _ => {
                return Err(ArtifactError::UnsupportedCombination {
                    kind: d.kind,
                    stochastic: true,
                });
            }
        };
        policy.set_params(&self.params);
        Ok(policy)
    }
}

// ── tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::suboptimal_flops,
    clippy::let_underscore_must_use
)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;
    use crate::autograd_layers::Activation;
    use crate::autograd_value::{AutogradQ, AutogradValue};
    use crate::linear::{LinearQ, LinearValue};
    use crate::mlp::{MlpQ, MlpValue};
    use crate::policy::Policy;
    use crate::value::{QFunction, ValueFn};

    const OBS_DIM: usize = 4;
    const ACT_DIM: usize = 2;
    const SCALE: [f64; 4] = [1.0, 0.5, 2.0, 0.1];

    // ── Descriptor correctness ───────────────────────────────────────────

    #[test]
    fn descriptor_linear_policy() {
        let p = LinearPolicy::new(OBS_DIM, ACT_DIM, &SCALE);
        let d = p.descriptor();
        assert_eq!(d.kind, NetworkKind::Linear);
        assert_eq!(d.obs_dim, OBS_DIM);
        assert_eq!(d.act_dim, ACT_DIM);
        assert!(d.hidden_dims.is_empty());
        assert!(!d.stochastic);
        assert_eq!(d.obs_scale, &SCALE);
    }

    #[test]
    fn descriptor_linear_stochastic_policy() {
        let p = LinearStochasticPolicy::new(OBS_DIM, ACT_DIM, &SCALE, -0.5);
        let d = p.descriptor();
        assert_eq!(d.kind, NetworkKind::Linear);
        assert!(d.stochastic);
    }

    #[test]
    fn descriptor_mlp_policy() {
        let p = MlpPolicy::new(OBS_DIM, 32, ACT_DIM, &SCALE);
        let d = p.descriptor();
        assert_eq!(d.kind, NetworkKind::Mlp);
        assert_eq!(d.hidden_dims, vec![32]);
        assert_eq!(d.activation, Activation::Tanh);
        assert!(!d.stochastic);
    }

    #[test]
    fn descriptor_autograd_policy() {
        let p = AutogradPolicy::new_with(OBS_DIM, &[32, 16], ACT_DIM, &SCALE, Activation::Relu);
        let d = p.descriptor();
        assert_eq!(d.kind, NetworkKind::Autograd);
        assert_eq!(d.hidden_dims, vec![32, 16]);
        assert_eq!(d.activation, Activation::Relu);
        assert_eq!(d.obs_dim, OBS_DIM);
        assert!(!d.stochastic);
    }

    #[test]
    fn descriptor_autograd_stochastic_policy() {
        let p = AutogradStochasticPolicy::new_with(
            OBS_DIM,
            &[64],
            ACT_DIM,
            &SCALE,
            -0.5,
            Activation::Tanh,
        );
        let d = p.descriptor();
        assert_eq!(d.kind, NetworkKind::Autograd);
        assert_eq!(d.hidden_dims, vec![64]);
        assert!(d.stochastic);
    }

    #[test]
    fn descriptor_linear_value() {
        let v = LinearValue::new(OBS_DIM, &SCALE);
        let d = v.descriptor();
        assert_eq!(d.kind, NetworkKind::Linear);
        assert_eq!(d.obs_dim, OBS_DIM);
        assert!(d.act_dim.is_none());
    }

    #[test]
    fn descriptor_mlp_value() {
        let v = MlpValue::new(OBS_DIM, 16, &SCALE);
        let d = v.descriptor();
        assert_eq!(d.kind, NetworkKind::Mlp);
        assert_eq!(d.hidden_dims, vec![16]);
        assert!(d.act_dim.is_none());
    }

    #[test]
    fn descriptor_autograd_value() {
        let v = AutogradValue::new_with(OBS_DIM, &[32], &SCALE, Activation::Tanh);
        let d = v.descriptor();
        assert_eq!(d.kind, NetworkKind::Autograd);
        assert_eq!(d.hidden_dims, vec![32]);
        assert!(d.act_dim.is_none());
    }

    #[test]
    fn descriptor_linear_q() {
        let q = LinearQ::new(OBS_DIM, ACT_DIM, &SCALE);
        let d = q.descriptor();
        assert_eq!(d.kind, NetworkKind::Linear);
        assert_eq!(d.act_dim, Some(ACT_DIM));
    }

    #[test]
    fn descriptor_mlp_q() {
        let q = MlpQ::new(OBS_DIM, 16, ACT_DIM, &SCALE);
        let d = q.descriptor();
        assert_eq!(d.kind, NetworkKind::Mlp);
        assert_eq!(d.act_dim, Some(ACT_DIM));
    }

    #[test]
    fn descriptor_autograd_q() {
        let q = AutogradQ::new_with(OBS_DIM, &[32], ACT_DIM, &SCALE, Activation::Tanh);
        let d = q.descriptor();
        assert_eq!(d.kind, NetworkKind::Autograd);
        assert_eq!(d.act_dim, Some(ACT_DIM));
        assert_eq!(d.hidden_dims, vec![32]);
    }

    // ── Round-trip tests ─────────────────────────────────────────────────

    fn test_obs() -> Vec<f32> {
        vec![0.1, -0.5, 1.0, 0.3]
    }

    /// Round-trip: construct → set params → artifact → serialize → deserialize
    /// → validate → `to_policy` → forward matches.
    fn assert_round_trip(policy: &dyn Policy) {
        let original_output = policy.forward(&test_obs());
        let artifact = PolicyArtifact::from_policy(policy);
        assert_eq!(artifact.version, CURRENT_VERSION);
        assert!(artifact.provenance.is_none());

        // Serialize → deserialize
        let json = serde_json::to_string_pretty(&artifact).unwrap();
        let loaded: PolicyArtifact = serde_json::from_str(&json).unwrap();
        loaded.validate().unwrap();

        // Reconstruct and compare
        let reconstructed = loaded.to_policy().unwrap();
        assert_eq!(reconstructed.n_params(), policy.n_params());
        let reconstructed_output = reconstructed.forward(&test_obs());
        assert_eq!(original_output, reconstructed_output);
    }

    #[test]
    fn round_trip_linear_policy() {
        let mut p = LinearPolicy::new(OBS_DIM, ACT_DIM, &SCALE);
        let params: Vec<f64> = (0..p.n_params()).map(|i| (i as f64) * 0.1 - 0.5).collect();
        p.set_params(&params);
        assert_round_trip(&p);
    }

    #[test]
    fn round_trip_linear_stochastic_policy() {
        let mut p = LinearStochasticPolicy::new(OBS_DIM, ACT_DIM, &SCALE, -0.5);
        let params: Vec<f64> = (0..p.n_params()).map(|i| (i as f64) * 0.05).collect();
        p.set_params(&params);
        assert_round_trip(&p);
    }

    #[test]
    fn round_trip_mlp_policy() {
        let mut p = MlpPolicy::new(OBS_DIM, 8, ACT_DIM, &SCALE);
        let params: Vec<f64> = (0..p.n_params()).map(|i| (i as f64) * 0.01 - 0.3).collect();
        p.set_params(&params);
        assert_round_trip(&p);
    }

    #[test]
    fn round_trip_autograd_policy() {
        let mut p = AutogradPolicy::new_with(OBS_DIM, &[8, 4], ACT_DIM, &SCALE, Activation::Tanh);
        let params: Vec<f64> = (0..p.n_params()).map(|i| (i as f64) * 0.01).collect();
        p.set_params(&params);
        assert_round_trip(&p);
    }

    #[test]
    fn round_trip_autograd_stochastic_policy() {
        let mut p = AutogradStochasticPolicy::new_with(
            OBS_DIM,
            &[8],
            ACT_DIM,
            &SCALE,
            -0.5,
            Activation::Tanh,
        );
        let params: Vec<f64> = (0..p.n_params()).map(|i| (i as f64) * 0.01).collect();
        p.set_params(&params);
        assert_round_trip(&p);
    }

    // ── File round-trip ──────────────────────────────────────────────────

    #[test]
    fn round_trip_save_load() {
        let mut p = LinearPolicy::new(OBS_DIM, ACT_DIM, &SCALE);
        let params: Vec<f64> = (0..p.n_params()).map(|i| (i as f64) * 0.1).collect();
        p.set_params(&params);
        let artifact = PolicyArtifact::from_policy(&p);

        let dir = std::env::temp_dir();
        let path = dir.join("test_artifact_round_trip.artifact.json");
        artifact.save(&path).unwrap();
        let loaded = PolicyArtifact::load(&path).unwrap();
        assert_eq!(loaded.params, artifact.params);
        assert_eq!(loaded.descriptor, artifact.descriptor);

        // Clean up.
        let _ = std::fs::remove_file(&path);
    }

    // ── Validation tests ─────────────────────────────────────────────────

    fn valid_artifact() -> PolicyArtifact {
        let p = LinearPolicy::new(OBS_DIM, ACT_DIM, &SCALE);
        PolicyArtifact::from_policy(&p)
    }

    #[test]
    fn validate_rejects_future_version() {
        let mut a = valid_artifact();
        a.version = 99;
        assert!(matches!(
            a.validate(),
            Err(ArtifactError::UnsupportedVersion { .. })
        ));
    }

    #[test]
    fn validate_rejects_param_count_mismatch() {
        let mut a = valid_artifact();
        a.params.push(42.0);
        assert!(matches!(
            a.validate(),
            Err(ArtifactError::ParamCountMismatch { .. })
        ));
    }

    #[test]
    fn validate_rejects_obs_scale_mismatch() {
        let mut a = valid_artifact();
        a.descriptor.obs_scale.push(1.0);
        assert!(matches!(
            a.validate(),
            Err(ArtifactError::ObsScaleMismatch { .. })
        ));
    }

    #[test]
    fn validate_rejects_linear_with_hidden_dims() {
        let mut a = valid_artifact();
        a.descriptor.hidden_dims = vec![32];
        assert!(matches!(
            a.validate(),
            Err(ArtifactError::LinearHiddenDims(_))
        ));
    }

    #[test]
    fn validate_rejects_mlp_empty_hidden() {
        let p = MlpPolicy::new(OBS_DIM, 8, ACT_DIM, &SCALE);
        let mut a = PolicyArtifact::from_policy(&p);
        a.descriptor.hidden_dims = vec![];
        assert!(matches!(
            a.validate(),
            Err(ArtifactError::MissingHiddenDims { .. })
        ));
    }

    #[test]
    fn validate_rejects_mlp_stochastic() {
        let mut a = valid_artifact();
        a.descriptor.kind = NetworkKind::Mlp;
        a.descriptor.hidden_dims = vec![8];
        a.descriptor.stochastic = true;
        assert!(matches!(
            a.validate(),
            Err(ArtifactError::UnsupportedCombination { .. })
        ));
    }

    #[test]
    fn validate_rejects_nan_params() {
        let mut a = valid_artifact();
        if !a.params.is_empty() {
            a.params[0] = f64::NAN;
        }
        assert!(matches!(
            a.validate(),
            Err(ArtifactError::NonFiniteValue { .. })
        ));
    }

    #[test]
    fn validate_rejects_nan_final_reward() {
        let mut a = valid_artifact();
        a.provenance = Some(TrainingProvenance {
            algorithm: "test".into(),
            task: "test".into(),
            seed: 0,
            epochs_trained: 1,
            final_reward: f64::NAN,
            total_steps: 100,
            wall_time_ms: 50,
            timestamp: "2026-01-01T00:00:00Z".into(),
            hyperparams: BTreeMap::new(),
            metrics: vec![],
            parent: None,
        });
        assert!(matches!(
            a.validate(),
            Err(ArtifactError::NonFiniteValue { .. })
        ));
    }

    // ── Provenance builder ───────────────────────────────────────────────

    #[test]
    fn with_provenance_attaches() {
        let p = LinearPolicy::new(OBS_DIM, ACT_DIM, &SCALE);
        let a = PolicyArtifact::from_policy(&p);
        assert!(a.provenance.is_none());

        let prov = TrainingProvenance {
            algorithm: "CEM".into(),
            task: "reaching-2dof".into(),
            seed: 42,
            epochs_trained: 10,
            final_reward: -1.5,
            total_steps: 5000,
            wall_time_ms: 1234,
            timestamp: "2026-04-08T12:00:00Z".into(),
            hyperparams: BTreeMap::new(),
            metrics: vec![],
            parent: None,
        };
        let a = a.with_provenance(prov);
        assert!(a.provenance.is_some());
        let p = a.provenance.as_ref().unwrap();
        assert_eq!(p.algorithm, "CEM");
        assert_eq!(p.seed, 42);
    }

    // ── Serde format ─────────────────────────────────────────────────────

    #[test]
    fn network_kind_serializes_as_string() {
        assert_eq!(
            serde_json::to_string(&NetworkKind::Linear).unwrap(),
            "\"Linear\""
        );
        assert_eq!(serde_json::to_string(&NetworkKind::Mlp).unwrap(), "\"Mlp\"");
        assert_eq!(
            serde_json::to_string(&NetworkKind::Autograd).unwrap(),
            "\"Autograd\""
        );
    }

    #[test]
    fn provenance_chain_round_trips() {
        let grandparent = TrainingProvenance {
            algorithm: "CEM".into(),
            task: "easy".into(),
            seed: 1,
            epochs_trained: 5,
            final_reward: -5.0,
            total_steps: 1000,
            wall_time_ms: 100,
            timestamp: "2026-01-01T00:00:00Z".into(),
            hyperparams: BTreeMap::new(),
            metrics: vec![],
            parent: None,
        };
        let parent = TrainingProvenance {
            algorithm: "TD3".into(),
            task: "hard".into(),
            seed: 2,
            epochs_trained: 50,
            final_reward: -1.0,
            total_steps: 50000,
            wall_time_ms: 30000,
            timestamp: "2026-02-01T00:00:00Z".into(),
            hyperparams: BTreeMap::new(),
            metrics: vec![],
            parent: Some(Box::new(grandparent)),
        };

        let json = serde_json::to_string_pretty(&parent).unwrap();
        let loaded: TrainingProvenance = serde_json::from_str(&json).unwrap();
        assert_eq!(loaded.algorithm, "TD3");
        let gp = loaded.parent.unwrap();
        assert_eq!(gp.algorithm, "CEM");
        assert!(gp.parent.is_none());
    }

    // ── Checkpoint serde ─────────────────────────────────────────────

    #[test]
    fn checkpoint_save_load_round_trip() {
        let policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &SCALE);
        let artifact = PolicyArtifact::from_policy(&policy);

        let checkpoint = TrainingCheckpoint {
            algorithm_name: "CEM".into(),
            policy_artifact: artifact,
            critics: vec![],
            optimizer_states: vec![],
            algorithm_state: BTreeMap::from([("noise_std".into(), 0.25)]),
        };

        let dir = std::env::temp_dir().join("ml_bridge_test_ckpt");
        let _ = std::fs::create_dir_all(&dir);
        let path = dir.join("test.checkpoint.json");
        checkpoint.save(&path).unwrap();

        let loaded = TrainingCheckpoint::load(&path).unwrap();
        assert_eq!(loaded.algorithm_name, "CEM");
        assert_eq!(
            loaded.policy_artifact.descriptor,
            checkpoint.policy_artifact.descriptor
        );
        assert_eq!(
            loaded.policy_artifact.params,
            checkpoint.policy_artifact.params
        );
        assert_eq!(loaded.algorithm_state["noise_std"], 0.25);
        assert!(loaded.critics.is_empty());
        assert!(loaded.optimizer_states.is_empty());

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn network_snapshot_serde_round_trip() {
        let snap = NetworkSnapshot {
            role: "q1".into(),
            descriptor: NetworkDescriptor {
                kind: NetworkKind::Linear,
                obs_dim: OBS_DIM,
                act_dim: Some(ACT_DIM),
                hidden_dims: vec![],
                activation: Activation::Tanh,
                obs_scale: SCALE.to_vec(),
            },
            params: vec![1.0, 2.0, 3.0],
        };

        let json = serde_json::to_string_pretty(&snap).unwrap();
        let loaded: NetworkSnapshot = serde_json::from_str(&json).unwrap();
        assert_eq!(loaded.role, "q1");
        assert_eq!(loaded.params, vec![1.0, 2.0, 3.0]);
        assert_eq!(loaded.descriptor.kind, NetworkKind::Linear);
    }

    #[test]
    fn optimizer_snapshot_serde_round_trip() {
        use crate::optimizer::OptimizerConfig;

        let snap = OptimizerSnapshot {
            role: "actor".into(),
            config: OptimizerConfig::adam(0.001),
            m: vec![0.1, 0.2],
            v: vec![0.01, 0.02],
            t: 42,
        };

        let json = serde_json::to_string_pretty(&snap).unwrap();
        let loaded: OptimizerSnapshot = serde_json::from_str(&json).unwrap();
        assert_eq!(loaded.role, "actor");
        assert_eq!(loaded.m, vec![0.1, 0.2]);
        assert_eq!(loaded.v, vec![0.01, 0.02]);
        assert_eq!(loaded.t, 42);
    }

    #[test]
    fn to_differentiable_policy_round_trip() {
        let policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &SCALE);
        let artifact = PolicyArtifact::from_policy(&policy);

        let dp = artifact.to_differentiable_policy().unwrap();
        assert_eq!(dp.n_params(), policy.n_params());
        assert_eq!(dp.params(), policy.params());
    }

    #[test]
    fn to_stochastic_policy_round_trip() {
        let policy = crate::linear::LinearStochasticPolicy::new(OBS_DIM, ACT_DIM, &SCALE, -0.5);
        let artifact = PolicyArtifact::from_policy(&policy);

        let sp = artifact.to_stochastic_policy().unwrap();
        assert_eq!(sp.n_params(), policy.n_params());
        assert_eq!(sp.params(), policy.params());
    }

    #[test]
    fn to_stochastic_policy_rejects_non_stochastic() {
        let policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &SCALE);
        let artifact = PolicyArtifact::from_policy(&policy);

        assert!(artifact.to_stochastic_policy().is_err());
    }

    #[test]
    fn network_snapshot_to_value_fn() {
        let vf = LinearValue::new(OBS_DIM, &SCALE);
        let snap = NetworkSnapshot {
            role: "value".into(),
            descriptor: vf.descriptor(),
            params: vf.params().to_vec(),
        };

        let restored = snap.to_value_fn().unwrap();
        assert_eq!(restored.n_params(), vf.n_params());
        assert_eq!(restored.params(), vf.params());
    }

    #[test]
    fn network_snapshot_to_q_function() {
        let qf = LinearQ::new(OBS_DIM, ACT_DIM, &SCALE);
        let snap = NetworkSnapshot {
            role: "q1".into(),
            descriptor: qf.descriptor(),
            params: qf.params().to_vec(),
        };

        let restored = snap.to_q_function().unwrap();
        assert_eq!(restored.n_params(), qf.n_params());
        assert_eq!(restored.params(), qf.params());
    }

    #[test]
    fn policy_descriptor_to_network_descriptor() {
        let policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &SCALE);
        let pd = policy.descriptor();
        let nd: NetworkDescriptor = pd.into();
        assert_eq!(nd.kind, NetworkKind::Linear);
        assert_eq!(nd.obs_dim, OBS_DIM);
        assert_eq!(nd.act_dim, Some(ACT_DIM));
    }
}
