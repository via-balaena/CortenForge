//! GPU-accelerated SDF collision detection.
//!
//! Uses wgpu compute shaders to parallelize the SDF grid evaluation
//! loops that dominate CPU time in collision detection. The grid cells
//! are independent — embarrassingly parallel on GPU.
//!
//! See `sim/docs/GPU_SDF_COLLISION_SPEC.md` for the full design.

pub mod context;

pub use context::{GpuContext, GpuError};
