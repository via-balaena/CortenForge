//! GPU-accelerated physics for CortenForge.
//!
//! Two subsystems:
//! - **SDF collision** (`collision`, `buffers`): GPU-accelerated SDF-SDF
//!   narrowphase. See `sim/docs/GPU_SDF_COLLISION_SPEC.md`.
//! - **Physics pipeline** (`pipeline`): Full GPU physics — FK, CRBA, RNE,
//!   solver, integration. See `sim/docs/GPU_PHYSICS_PIPELINE_SPEC.md`.

pub mod buffers;
pub mod collision;
pub mod context;
pub mod pipeline;

pub use buffers::{GpuSdfGrid, GridMeta};
pub use collision::{GpuSdfCollider, GpuTracer, enable_gpu_collision};
pub use context::{GpuContext, GpuError};
