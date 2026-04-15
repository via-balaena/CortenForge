//! # sim-ml-bridge
//!
//! Generic RL baselines (CEM, REINFORCE, PPO, TD3, SAC) bolted onto
//! `sim-ml-chassis`. Renamed to `sim-rl` in the next commit.

// Safety lint: deny unwrap/expect in library code.
#![deny(clippy::unwrap_used, clippy::expect_used)]

pub mod cem;
pub mod ppo;
pub mod reinforce;
pub mod sac;
pub mod td3;

pub use cem::{Cem, CemHyperparams};
pub use ppo::{Ppo, PpoHyperparams};
pub use reinforce::{Reinforce, ReinforceHyperparams};
pub use sac::{Sac, SacHyperparams};
pub use td3::{Td3, Td3Hyperparams};
