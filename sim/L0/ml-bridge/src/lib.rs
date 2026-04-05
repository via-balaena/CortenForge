//! # sim-ml-bridge
//!
//! A clean boundary layer that lets RL training loops consume the simulator
//! without the simulator knowing anything about ML.
//!
//! This crate is **Layer 0** — zero Bevy, zero ML framework dependencies.
//! The sim stays completely pure; all ML-facing abstractions live here.
//!
//! ## Core types
//!
//! - [`Tensor`] — flat `f32` buffer with shape metadata.
//! - [`TensorSpec`] — shape + optional bounds for observation/action spaces.
//! - [`ObservationSpace`] — configurable mapping from `Data` fields to a flat
//!   `Tensor`.
//! - [`ActionSpace`] — configurable mapping from a flat `Tensor` back into
//!   `Data` fields, with ctrl clamping.
//! - [`Environment`] trait + [`SimEnv`] — closure-based single RL environment
//!   with sub-stepping and early termination.
//! - [`VecEnv`] — vectorized environments wrapping `BatchSim` for parallel
//!   stepping with auto-reset.
//!
//! ## Design principles
//!
//! - **Sim purity** — `sim-core` and `sim-types` gain zero new dependencies or
//!   traits.  All ML-facing abstractions live here.
//! - **Composition over inheritance** — the bridge wraps `Model` + `Data` (and
//!   `BatchSim`); it doesn't subclass or modify them.
//! - **`f64→f32` is a conscious boundary** — the sim is `f64` throughout.
//!   ML is `f32`.  The bridge owns this conversion explicitly.
//! - **Vectorized from day one** — `VecEnv` produces a single `Tensor` of
//!   shape `[n_envs, obs_dim]`, not `Vec<Tensor>`.  One allocation, one
//!   memory layout.  Same for actions in.
//! - **Reward and termination are user-defined** — the bridge provides the
//!   hook; the user provides the logic via closures.
//! - No strides, no views, no autograd.  Growth path is clear but not premature.
//!
//! ## What this crate does NOT do
//!
//! - **No policy implementations** — that's the RL library's job.
//! - **No autodiff** — `Tensor` is a dumb buffer.  Autodiff is a future crate.
//! - **No GPU tensor ops** — `Tensor` lives on CPU.  GPU acceleration lives in
//!   `sim-gpu` (compute shaders) and future autodiff (GPU kernels).
//! - **No observation normalization** — running mean/std belongs in the RL
//!   library's policy preprocessing, not in the bridge.
//! - **No domain randomization primitives** — the `on_reset` hook gives users
//!   full control.
//! - **No Bevy dependency** — this is Layer 0.
//!
//! ## Growth path
//!
//! | Future capability | How it fits |
//! |---|---|
//! | Autodiff | Replace `Tensor` internals with a tape-backed tensor.  External API unchanged. |
//! | GPU tensors | `Tensor` gains a `Device` enum.  Builder API unchanged. |
//! | Custom extractors | `Extractor` enum grows new variants without breaking existing builders. |
//! | Pre-allocated buffers | `VecEnv` owns internal buffers, `step()` fills in-place, zero-alloc hot path. |

// Safety lint: deny unwrap/expect in library code.
#![deny(clippy::unwrap_used, clippy::expect_used)]

pub mod env;
pub mod error;
pub mod space;
pub mod tensor;
pub mod vec_env;

pub use env::{Environment, SimEnv, SimEnvBuilder, StepResult};
pub use error::{EnvError, ResetError, SpaceError, TensorError, VecStepError};
pub use space::{ActionSpace, ActionSpaceBuilder, ObservationSpace, ObservationSpaceBuilder};
pub use tensor::{Tensor, TensorSpec};
pub use vec_env::{VecEnv, VecEnvBuilder, VecStepResult};
