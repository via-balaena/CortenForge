//! Error types for mesh decimation operations.

use thiserror::Error;

/// Errors that can occur during decimation operations.
#[derive(Debug, Error)]
pub enum DecimateError {
    /// Mesh has no vertices.
    #[error("Mesh has no vertices")]
    EmptyMesh,

    /// Mesh has no faces.
    #[error("Mesh has no faces")]
    NoFaces,

    /// Invalid target ratio.
    #[error("Invalid target ratio: {0} (must be between 0.0 and 1.0)")]
    InvalidRatio(f64),

    /// Invalid target triangle count.
    #[error("Invalid target triangle count: {0}")]
    InvalidTargetCount(usize),
}

/// Result type for decimation operations.
pub type DecimateResult<T> = std::result::Result<T, DecimateError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = DecimateError::EmptyMesh;
        assert_eq!(format!("{err}"), "Mesh has no vertices");

        let err = DecimateError::InvalidRatio(1.5);
        assert!(format!("{err}").contains("1.5"));
    }
}
