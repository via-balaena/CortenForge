//! GPU-accelerated physics for CortenForge.
//!
//! The **physics pipeline** (`pipeline`) runs a full GPU-resident physics step
//! — FK, CRBA, RNE, SDF collision, constraint solve, integration. See
//! `sim/docs/GPU_PHYSICS_PIPELINE_SPEC.md`.

pub mod context;
pub mod pipeline;

pub use context::{GpuContext, GpuError};
pub use pipeline::{GpuPhysicsPipeline, GpuPipelineError};
