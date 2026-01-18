//! GPU error types for mesh processing.
//!
//! This module provides error types for GPU operations, including
//! device unavailability, memory limits, and shader compilation failures.

use thiserror::Error;

/// Errors that can occur during GPU operations.
///
/// # Example
///
/// ```
/// use mesh_gpu::error::{GpuError, GpuResult};
///
/// fn check_gpu() -> GpuResult<()> {
///     // Return error if GPU is not available
///     Err(GpuError::NotAvailable)
/// }
/// ```
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum GpuError {
    /// GPU device is not available on this system.
    ///
    /// This can happen when:
    /// - No compatible GPU is present
    /// - GPU drivers are not installed
    /// - The system is running in a headless environment without GPU support
    #[error("GPU not available: no compatible device found")]
    NotAvailable,

    /// GPU ran out of memory during computation.
    ///
    /// This typically occurs when allocating large buffers for mesh data
    /// or SDF grids. Consider using tiled computation for large grids.
    #[error("GPU out of memory: required {required} bytes, available {available} bytes")]
    OutOfMemory {
        /// Memory required for the operation in bytes.
        required: u64,
        /// Memory available on the GPU in bytes.
        available: u64,
    },

    /// Shader compilation failed.
    ///
    /// This usually indicates a bug in the shader code. Please report this
    /// as an issue if you encounter it.
    #[error("shader compilation failed: {0}")]
    ShaderCompilation(String),

    /// GPU device was lost during computation.
    ///
    /// This can happen due to:
    /// - GPU driver crash
    /// - GPU timeout (long-running computation)
    /// - System power management events
    #[error("GPU device lost")]
    DeviceLost,

    /// GPU execution failed.
    ///
    /// A general error for GPU command submission or execution failures.
    #[error("GPU execution failed: {0}")]
    Execution(String),

    /// Buffer mapping failed.
    ///
    /// This error occurs when reading data back from the GPU fails.
    #[error("buffer mapping failed: {0}")]
    BufferMapping(String),

    /// Grid dimensions exceed GPU limits.
    ///
    /// The requested SDF grid is too large for a single GPU buffer.
    /// Use tiled computation for grids exceeding this limit.
    #[error("grid too large for GPU: {dims:?} voxels ({total} total), max supported: {max}")]
    GridTooLarge {
        /// Requested grid dimensions [x, y, z].
        dims: [usize; 3],
        /// Total number of voxels.
        total: usize,
        /// Maximum supported voxels.
        max: usize,
    },

    /// Mesh too large for GPU.
    ///
    /// The mesh has too many triangles to fit in a single GPU buffer.
    #[error("mesh too large for GPU: {triangles} triangles, max supported: {max}")]
    MeshTooLarge {
        /// Number of triangles in the mesh.
        triangles: usize,
        /// Maximum supported triangles.
        max: usize,
    },
}

/// Result type for GPU operations.
///
/// This is a convenience alias for `Result<T, GpuError>`.
///
/// # Example
///
/// ```
/// use mesh_gpu::error::GpuResult;
///
/// fn compute_something() -> GpuResult<Vec<f32>> {
///     Ok(vec![0.0, 1.0, 2.0])
/// }
/// ```
pub type GpuResult<T> = Result<T, GpuError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display_not_available() {
        let err = GpuError::NotAvailable;
        let msg = format!("{err}");
        assert!(msg.contains("not available"));
    }

    #[test]
    fn test_error_display_out_of_memory() {
        let err = GpuError::OutOfMemory {
            required: 1024,
            available: 512,
        };
        let msg = format!("{err}");
        assert!(msg.contains("1024"));
        assert!(msg.contains("512"));
    }

    #[test]
    fn test_error_display_shader_compilation() {
        let err = GpuError::ShaderCompilation("syntax error".to_string());
        let msg = format!("{err}");
        assert!(msg.contains("syntax error"));
    }

    #[test]
    fn test_error_display_device_lost() {
        let err = GpuError::DeviceLost;
        let msg = format!("{err}");
        assert!(msg.contains("device lost"));
    }

    #[test]
    fn test_error_display_execution() {
        let err = GpuError::Execution("command failed".to_string());
        let msg = format!("{err}");
        assert!(msg.contains("command failed"));
    }

    #[test]
    fn test_error_display_buffer_mapping() {
        let err = GpuError::BufferMapping("mapping failed".to_string());
        let msg = format!("{err}");
        assert!(msg.contains("mapping failed"));
    }

    #[test]
    fn test_error_display_grid_too_large() {
        let err = GpuError::GridTooLarge {
            dims: [256, 256, 256],
            total: 16_777_216,
            max: 8_388_608,
        };
        let msg = format!("{err}");
        assert!(msg.contains("256"));
        assert!(msg.contains("16777216"));
    }

    #[test]
    fn test_error_display_mesh_too_large() {
        let err = GpuError::MeshTooLarge {
            triangles: 1_000_000,
            max: 500_000,
        };
        let msg = format!("{err}");
        assert!(msg.contains("1000000"));
        assert!(msg.contains("500000"));
    }

    #[test]
    fn test_result_type_alias() {
        fn returns_ok() -> GpuResult<i32> {
            Ok(42)
        }

        fn returns_err() -> GpuResult<i32> {
            Err(GpuError::NotAvailable)
        }

        assert_eq!(returns_ok().ok(), Some(42));
        assert!(returns_err().is_err());
    }

    #[test]
    fn test_error_is_send_sync() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<GpuError>();
    }
}
