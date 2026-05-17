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

    /// Flood-fill sign build failed (only on the [`crate::SignOracle::FloodFill`] path).
    #[error("flood-fill sign build failed: {0}")]
    FloodFill(#[from] mesh_sdf::FloodFillError),

    /// Marching cubes failed to generate valid mesh.
    #[error("marching cubes failed: {reason}")]
    MarchingCubesFailed {
        /// Reason for failure.
        reason: String,
    },
}

// Decompose mesh-sdf's two-variant FloodFilledSdfBuildError into the
// two pre-existing OffsetError variants. Lets `?` work on
// `flood_filled_sdf(...)` calls inside the FloodFill path without
// introducing a third "build error" wrapper variant.
impl From<mesh_sdf::FloodFilledSdfBuildError> for OffsetError {
    fn from(err: mesh_sdf::FloodFilledSdfBuildError) -> Self {
        match err {
            mesh_sdf::FloodFilledSdfBuildError::Sdf(e) => OffsetError::SdfError(e),
            mesh_sdf::FloodFilledSdfBuildError::Flood(e) => OffsetError::FloodFill(e),
        }
    }
}
