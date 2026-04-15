//! # sim-opt
//!
//! Gradient-free optimization algorithms and rematch analysis
//! machinery.
//!
//! This crate is **Layer 0** — zero Bevy, zero ML framework
//! dependencies. It extends `sim-ml-chassis`'s `Algorithm` trait
//! with Simulated Annealing and ships the statistical-analysis
//! machinery the ml-chassis-refactor study's rematch consumes.
//!
//! ## Scope
//!
//! - [`algorithm`] — `Sa` / `SaHyperparams`: Simulated Annealing
//!   implemented as an `Algorithm` trait impl. Consumes `Policy`
//!   and `VecEnv` directly, like CEM, and emits per-epoch
//!   `EpochMetrics` in the per-episode-total unit the chassis
//!   algorithms standardized on.
//! - [`analysis`] — bootstrap CI on the difference of means and
//!   medians, bimodality coefficient, Ch 30 three-outcome
//!   classifier, and the folded-pilot driver that executes
//!   Chapter 32's rematch protocol end-to-end.
//!
//! ## What this crate does NOT do
//!
//! - **No gradient-based algorithms.** Those live in
//!   `sim-rl` alongside CEM, REINFORCE, PPO, TD3, and SAC.
//!   `sim-opt` is specifically the gradient-free branch.
//! - **No policy or network implementations.** Policies come
//!   from `sim-ml-chassis::LinearPolicy` (or `MlpPolicy`, etc.)
//!   and are passed into `Sa::new` at construction time.
//! - **No environment construction.** `VecEnv` instances come
//!   from `sim-ml-chassis::TaskConfig::build_vec_env(n_envs,
//!   seed)`, which sim-opt's analysis module calls via the
//!   rematch driver.
//! - **No Bevy dependency.** This is Layer 0.

#![deny(clippy::unwrap_used, clippy::expect_used)]

pub mod algorithm;
pub mod analysis;
pub mod parallel_tempering;
pub mod richer_sa;

pub use algorithm::{Sa, SaHyperparams};
pub use analysis::{
    BootstrapCi, N_EXPANDED, N_INITIAL, REMATCH_MASTER_SEED, REMATCH_TASK_NAME, RematchOutcome,
    TwoMetricOutcome, bimodality_coefficient, bootstrap_diff_means, bootstrap_diff_medians,
    classify_outcome, run_rematch,
};
pub use parallel_tempering::{Pt, PtHyperparams};
pub use richer_sa::{RicherSa, RicherSaHyperparams};
