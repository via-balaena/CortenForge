//! # sim-rl
//!
//! Generic RL baselines (CEM, REINFORCE, PPO, TD3, SAC) bolted onto
//! `sim-ml-chassis`. The control group for every CortenForge competition
//! against physics-aware algorithms in `sim-opt`.
//!
//! Re-exports the chassis types that every algorithm consumer also needs,
//! so `use sim_rl::{Cem, Algorithm, VecEnv}` works without reaching into
//! `sim_ml_chassis` directly.

#![deny(clippy::unwrap_used, clippy::expect_used)]

pub mod cem;
pub mod ppo;
pub mod reinforce;
pub mod sac;
pub mod td3;

// Algorithm exports.
pub use cem::{Cem, CemHyperparams};
pub use ppo::{Ppo, PpoHyperparams};
pub use reinforce::{Reinforce, ReinforceHyperparams};
pub use sac::{Sac, SacHyperparams};
pub use td3::{Td3, Td3Hyperparams};
// Note: `gaussian_log_prob` is NOT re-exported from sim-rl. It lives in
// `sim_ml_chassis::stats` post-relocation (§3.6) and is imported from
// there directly.

// Chassis re-exports — the common import set that algorithm consumers
// need alongside the algorithm structs. Keep this list aligned with
// what d1c/d1d/d2c/persistence actually import, so every site that
// used to say `sim_ml_bridge::{...}` can rewrite to `sim_rl::{...}`
// with a single find-and-replace.
pub use sim_ml_chassis::algorithm::{Algorithm, EpochMetrics, TrainingBudget};
pub use sim_ml_chassis::artifact::{PolicyArtifact, TrainingCheckpoint, TrainingProvenance};
pub use sim_ml_chassis::env::{Environment, SimEnv, StepResult};
pub use sim_ml_chassis::linear::{LinearPolicy, LinearQ, LinearStochasticPolicy, LinearValue};
pub use sim_ml_chassis::mlp::{MlpPolicy, MlpQ, MlpValue};
pub use sim_ml_chassis::optimizer::OptimizerConfig;
pub use sim_ml_chassis::policy::{DifferentiablePolicy, Policy, StochasticPolicy};
pub use sim_ml_chassis::space::{ActionSpace, ObservationSpace};
pub use sim_ml_chassis::task::{TaskConfig, reaching_2dof, reaching_6dof};
pub use sim_ml_chassis::tensor::Tensor;
pub use sim_ml_chassis::value::{QFunction, ValueFn};
pub use sim_ml_chassis::vec_env::VecEnv;
