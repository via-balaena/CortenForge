//! # sim-ml-chassis
//!
//! Algorithm chassis: traits, primitives, Competition runner — the
//! foundation every CortenForge algorithm crate bolts onto.
//!
//! This crate is **Layer 0** — zero Bevy, zero ML framework dependencies.
//! The sim stays completely pure; all ML-facing abstractions live here.
//!
//! Concrete algorithms live in separate crates (`sim-rl`, `sim-opt`) that
//! depend on this chassis for traits, primitives, and the competition
//! runner.

// Safety lint: deny unwrap/expect in library code.
#![deny(clippy::unwrap_used, clippy::expect_used)]

pub mod algorithm;
pub mod artifact;
pub mod autograd;
pub mod autograd_layers;
pub mod autograd_policy;
pub mod autograd_value;
pub mod best_tracker;
pub mod competition;
pub mod env;
pub mod error;
pub mod gae;
pub mod linear;
pub mod mlp;
pub mod optimizer;
pub mod policy;
pub mod replay_buffer;
pub mod rollout;
pub mod space;
pub mod stats;
pub mod task;
pub mod tensor;
pub mod value;
pub mod vec_env;

pub use algorithm::{Algorithm, EpochMetrics, TrainingBudget};
pub use artifact::{
    ArtifactError, CURRENT_VERSION, NetworkDescriptor, NetworkKind, NetworkSnapshot,
    OptimizerSnapshot, PolicyArtifact, PolicyDescriptor, TrainingCheckpoint, TrainingProvenance,
};
pub use autograd::{Tape, Var};
pub use autograd_layers::{
    Activation, linear_hidden, linear_raw, linear_relu, linear_tanh, mse_loss, mse_loss_batch,
};
pub use autograd_policy::{AutogradPolicy, AutogradStochasticPolicy};
pub use autograd_value::{AutogradQ, AutogradValue};
pub use best_tracker::BestTracker;
pub use competition::{Competition, CompetitionResult, RunResult, SeedSummary};
pub use env::{Environment, SimEnv, SimEnvBuilder, StepResult};
pub use error::{EnvError, ResetError, SpaceError, TensorError, VecStepError};
pub use gae::compute_gae;
pub use linear::{LinearPolicy, LinearQ, LinearStochasticPolicy, LinearValue};
pub use mlp::{MlpPolicy, MlpQ, MlpValue};
pub use optimizer::{Optimizer, OptimizerConfig};
pub use policy::{DifferentiablePolicy, Policy, StochasticPolicy};
pub use replay_buffer::{ReplayBuffer, TransitionBatch};
pub use rollout::{EpisodicRollout, Trajectory, collect_episodic_rollout};
pub use space::{
    ActionSpace, ActionSpaceBuilder, ObsSegment, ObservationSpace, ObservationSpaceBuilder,
};
pub use stats::gaussian_log_prob;
pub use task::{
    TaskConfig, TaskConfigBuilder, obstacle_reaching_6dof, reaching_2dof, reaching_6dof,
};
pub use tensor::{Tensor, TensorSpec};
pub use value::{QFunction, ValueFn, soft_update, soft_update_policy, soft_update_value};
pub use vec_env::{VecEnv, VecEnvBuilder, VecStepResult};
