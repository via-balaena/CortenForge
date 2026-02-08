//! Error types for GPU-accelerated simulation.

use sim_core::Integrator;

/// Errors from GPU-accelerated simulation.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum GpuSimError {
    /// No compatible GPU available.
    #[error("no compatible GPU available for simulation")]
    NotAvailable,

    /// Buffer exceeds `limits.max_storage_buffer_binding_size`.
    ///
    /// wgpu does not expose a "free GPU memory" API. Actual GPU OOM
    /// surfaces as [`DeviceLost`](GpuSimError::DeviceLost). This error
    /// catches the checkable limit: individual buffer size vs. the
    /// device's max storage buffer binding size.
    #[error("batch of {num_envs} envs needs {required}B buffer, limit: {limit}B")]
    BatchTooLarge {
        /// Number of environments requested.
        num_envs: usize,
        /// Required buffer size in bytes.
        required: u64,
        /// Device maximum storage buffer binding size in bytes.
        limit: u64,
    },

    /// GPU device lost during computation.
    #[error("GPU device lost during simulation step")]
    DeviceLost,

    /// Shader compilation failure.
    #[error("shader compilation failed: {0}")]
    ShaderCompilation(String),

    /// Buffer readback failure.
    #[error("GPU buffer readback failed: {0}")]
    BufferReadback(String),

    /// Unsupported integrator (GPU path supports Euler only in Phase 10a).
    #[error("unsupported integrator for GPU: {0:?} (only Euler is supported)")]
    UnsupportedIntegrator(Integrator),
}

/// Result type alias for GPU simulation operations.
pub type GpuSimResult<T> = Result<T, GpuSimError>;
