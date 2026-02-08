#![deny(clippy::unwrap_used, clippy::expect_used)]
#![warn(missing_docs)]

//! GPU-accelerated batched physics simulation via wgpu compute shaders.
//!
//! This is a Layer 0 crate — no Bevy, no windowing, no rendering.
//! Provides [`GpuBatchSim`] as a GPU-accelerated drop-in for
//! [`sim_core::batch::BatchSim`].
//!
//! # Usage
//!
//! ```ignore
//! use sim_core::Model;
//! use sim_gpu::GpuBatchSim;
//! use std::sync::Arc;
//!
//! let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
//! let batch = GpuBatchSim::new(model, 1024)?;
//! // ... set controls, step, read results — same API as BatchSim
//! ```

pub mod buffers;
pub mod context;
pub mod error;
pub mod pipeline;

pub use buffers::{GpuEnvBuffers, GpuParams};
pub use context::GpuSimContext;
pub use error::{GpuSimError, GpuSimResult};
pub use pipeline::GpuBatchSim;
