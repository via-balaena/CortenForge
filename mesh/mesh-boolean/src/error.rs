//! Error types for boolean operations.

use thiserror::Error;

/// Errors that can occur during boolean operations.
#[derive(Debug, Error)]
pub enum BooleanError {
    /// One or both input meshes are empty.
    #[error("empty mesh: {details}")]
    EmptyMesh {
        /// Description of which mesh is empty.
        details: String,
    },

    /// Input mesh is degenerate (e.g., all coplanar faces).
    #[error("degenerate mesh: {details}")]
    DegenerateMesh {
        /// Description of the degeneracy.
        details: String,
    },

    /// Boolean operation failed due to numerical issues.
    #[error("numerical error: {details}")]
    NumericalError {
        /// Description of the numerical issue.
        details: String,
    },

    /// Self-intersection detected in input mesh.
    #[error("self-intersection detected in {mesh}: {details}")]
    SelfIntersection {
        /// Which mesh ("A" or "B").
        mesh: String,
        /// Description of the self-intersection.
        details: String,
    },

    /// Operation cancelled by progress callback.
    #[error("operation cancelled")]
    Cancelled,

    /// GPU operation failed.
    #[cfg(feature = "gpu")]
    #[error("GPU error: {details}")]
    GpuError {
        /// Description of the GPU error.
        details: String,
    },
}

/// Result type for boolean operations.
pub type BooleanResult<T> = Result<T, BooleanError>;
