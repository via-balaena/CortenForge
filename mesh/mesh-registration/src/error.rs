//! Error types for mesh registration operations.

use thiserror::Error;

/// Errors that can occur during mesh registration.
#[derive(Debug, Error)]
pub enum RegistrationError {
    /// Source mesh has no vertices.
    #[error("source mesh has no vertices")]
    EmptySourceMesh,

    /// Target mesh has no vertices.
    #[error("target mesh has no vertices")]
    EmptyTargetMesh,

    /// Not enough landmarks provided for registration.
    #[error("at least {required} landmarks required, got {provided}")]
    InsufficientLandmarks {
        /// Number of landmarks required.
        required: usize,
        /// Number of landmarks provided.
        provided: usize,
    },

    /// Landmark index out of bounds.
    #[error("landmark index {index} out of bounds for mesh with {vertex_count} vertices")]
    LandmarkOutOfBounds {
        /// The invalid landmark index.
        index: usize,
        /// The number of vertices in the mesh.
        vertex_count: usize,
    },

    /// SVD computation failed during transform estimation.
    #[error("SVD computation failed during transform estimation")]
    SvdFailed,

    /// Registration did not converge within the maximum iterations.
    #[error(
        "registration did not converge after {iterations} iterations (error: {final_error:.6})"
    )]
    DidNotConverge {
        /// Number of iterations performed.
        iterations: u32,
        /// Final RMS error.
        final_error: f64,
    },

    /// No valid correspondences found between meshes.
    #[error("no valid correspondences found between meshes")]
    NoCorrespondences,

    /// Invalid parameter value.
    #[error("invalid parameter: {0}")]
    InvalidParameter(String),
}

/// Result type for registration operations.
pub type RegistrationResult<T> = Result<T, RegistrationError>;
