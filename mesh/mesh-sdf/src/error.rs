//! Error types for SDF operations.

use thiserror::Error;

/// Result type for SDF operations.
pub type SdfResult<T> = Result<T, SdfError>;

/// Errors that can occur during SDF computation.
#[derive(Debug, Error)]
pub enum SdfError {
    /// Mesh is empty (no vertices or faces).
    #[error("mesh is empty")]
    EmptyMesh,

    /// Grid dimensions are invalid.
    #[error("invalid grid dimensions: {0}")]
    InvalidDimensions(String),

    /// Point is outside the SDF bounds.
    #[error("point ({x}, {y}, {z}) is outside SDF bounds")]
    OutOfBounds {
        /// X coordinate.
        x: f64,
        /// Y coordinate.
        y: f64,
        /// Z coordinate.
        z: f64,
    },
}
