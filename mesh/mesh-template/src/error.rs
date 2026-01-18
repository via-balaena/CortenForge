//! Error types for template fitting operations.

use thiserror::Error;

/// Result type for template fitting operations.
pub type TemplateResult<T> = Result<T, TemplateError>;

/// Errors that can occur during template fitting.
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum TemplateError {
    /// The template mesh has no vertices.
    #[error("template mesh has no vertices")]
    EmptyMesh,

    /// The target scan mesh has no vertices.
    #[error("target scan mesh has no vertices")]
    EmptyScan,

    /// A referenced control region was not found.
    #[error("control region '{name}' not found")]
    RegionNotFound {
        /// The name of the missing region.
        name: String,
    },

    /// No fitting constraints were provided.
    #[error("no fitting constraints provided (need scan, landmarks, or measurements)")]
    NoConstraints,

    /// The morphing operation failed.
    #[error("morph operation failed: {0}")]
    MorphError(#[from] mesh_morph::MorphError),

    /// The registration operation failed.
    #[error("registration operation failed: {0}")]
    RegistrationError(#[from] mesh_registration::RegistrationError),

    /// A measurement region has invalid geometry.
    #[error("measurement region '{region}' has invalid geometry: {reason}")]
    InvalidMeasurementGeometry {
        /// The name of the region with invalid geometry.
        region: String,
        /// Description of the geometry issue.
        reason: String,
    },

    /// Too few vertices in region for the requested operation.
    #[error("region '{region}' has only {count} vertices, need at least {required}")]
    InsufficientVertices {
        /// The name of the region.
        region: String,
        /// Number of vertices found.
        count: usize,
        /// Minimum number required.
        required: usize,
    },
}
