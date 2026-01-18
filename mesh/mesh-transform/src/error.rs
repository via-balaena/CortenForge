//! Error types for mesh transformation operations.

use thiserror::Error;

/// Result type for transformation operations.
pub type TransformResult<T> = Result<T, TransformError>;

/// Errors that can occur during mesh transformation.
#[derive(Debug, Error)]
pub enum TransformError {
    /// Not enough points for the operation.
    #[error("insufficient points: need at least {required}, got {actual}")]
    InsufficientPoints {
        /// Minimum number of points required.
        required: usize,
        /// Actual number of points provided.
        actual: usize,
    },

    /// Matrix is not invertible.
    #[error("matrix is not invertible")]
    NotInvertible,

    /// RANSAC failed to find a valid plane.
    #[error("RANSAC failed to find plane after {iterations} iterations")]
    RansacFailed {
        /// Number of iterations attempted.
        iterations: usize,
    },

    /// PCA computation failed.
    #[error("PCA computation failed: {reason}")]
    PcaFailed {
        /// Reason for failure.
        reason: String,
    },

    /// Empty mesh.
    #[error("mesh is empty")]
    EmptyMesh,
}
