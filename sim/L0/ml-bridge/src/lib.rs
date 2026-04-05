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
//!
//! ## Design
//!
//! - `f32` throughout (ML convention). The bridge owns the `f64→f32` conversion.
//! - Dynamic shapes (`Vec<usize>`) — observation/action dimensions vary per task.
//! - No strides, no views, no autograd. Growth path is clear but not premature.

// Safety lint: deny unwrap/expect in library code.
#![deny(clippy::unwrap_used, clippy::expect_used)]

pub mod error;
pub mod space;
pub mod tensor;

pub use error::{SpaceError, TensorError};
pub use space::{ActionSpace, ActionSpaceBuilder, ObservationSpace, ObservationSpaceBuilder};
pub use tensor::{Tensor, TensorSpec};
