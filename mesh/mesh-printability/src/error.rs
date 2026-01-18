//! Error types for printability analysis.

use thiserror::Error;

/// Result type for printability operations.
pub type PrintabilityResult<T> = Result<T, PrintabilityError>;

/// Errors that can occur during printability analysis.
#[derive(Debug, Error)]
pub enum PrintabilityError {
    /// Mesh has no vertices.
    #[error("Mesh has no vertices")]
    EmptyMesh,

    /// Mesh has no faces.
    #[error("Mesh has no faces")]
    NoFaces,

    /// Invalid configuration parameter.
    #[error("Invalid configuration: {message}")]
    InvalidConfig {
        /// Description of the configuration error.
        message: String,
    },

    /// Mesh topology error.
    #[error("Mesh topology error: {message}")]
    TopologyError {
        /// Description of the topology error.
        message: String,
    },
}
