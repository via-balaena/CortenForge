//! Error types for mesh offset operations.

use thiserror::Error;

/// Result type for offset operations.
pub type OffsetResult<T> = Result<T, OffsetError>;

/// Errors that can occur during mesh offset.
#[derive(Debug, Error)]
pub enum OffsetError {
    /// Mesh is empty (no vertices or faces).
    #[error("mesh is empty")]
    EmptyMesh,

    /// Offset distance is invalid.
    #[error("invalid offset distance: {0}")]
    InvalidDistance(String),

    /// Grid resolution is too low.
    #[error("grid resolution too low: minimum is {minimum}, got {actual}")]
    ResolutionTooLow {
        /// Minimum allowed resolution.
        minimum: usize,
        /// Actual resolution provided.
        actual: usize,
    },

    /// SDF computation failed.
    #[error("SDF computation failed: {0}")]
    SdfError(#[from] mesh_sdf::SdfError),

    /// Marching cubes failed to generate valid mesh.
    #[error("marching cubes failed: {reason}")]
    MarchingCubesFailed {
        /// Reason for failure.
        reason: String,
    },
}
