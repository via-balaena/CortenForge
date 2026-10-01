//! # sim-opt
//!
//! Gradient-free optimizers (simulated annealing, parallel tempering)
//! and bootstrap statistics for comparing two training algorithms.
//!
//! This crate is **Layer 0** — zero Bevy, zero ML framework
//! dependencies. It extends `sim-ml-chassis`'s `Algorithm` trait
//! with Simulated Annealing and Parallel Tempering and ships the
//! statistical-analysis machinery behind [`run_rematch`].
//!
//! ## Scope
//!
//! - [`algorithm`] — `Sa` / `SaHyperparams`: Simulated Annealing
//!   implemented as an `Algorithm` trait impl. Consumes `Policy`
//!   and `VecEnv` directly, like CEM, and emits per-epoch
//!   `EpochMetrics` in the per-episode-total unit the chassis
//!   algorithms standardized on.
//! - [`richer_sa`] — `RicherSa` / `RicherSaHyperparams`.
//! - [`parallel_tempering`] — `Pt` / `PtHyperparams`.
//! - [`analysis`] — bootstrap CI on the difference of means and
//!   medians, bimodality coefficient, three-outcome classifier,
//!   and [`run_rematch`].
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
//!   seed)`, which sim-opt's analysis module calls via
//!   [`run_rematch`].
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
