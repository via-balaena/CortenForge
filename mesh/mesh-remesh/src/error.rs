//! Error types for mesh remeshing operations.

use thiserror::Error;

/// Errors that can occur during remeshing operations.
#[derive(Debug, Error)]
pub enum RemeshError {
    /// Mesh has no vertices.
    #[error("Mesh has no vertices")]
    EmptyMesh,

    /// Mesh has no faces.
    #[error("Mesh has no faces")]
    NoFaces,

    /// Invalid target edge length.
    #[error("Invalid target edge length: {0} (must be > 0)")]
    InvalidEdgeLength(f64),

    /// Invalid iteration count.
    #[error("Invalid iteration count: {0} (must be >= 1)")]
    InvalidIterations(u32),

    /// Mesh has non-manifold geometry that cannot be remeshed.
    #[error("Mesh contains non-manifold geometry")]
    NonManifold,
}

/// Result type for remeshing operations.
pub type RemeshResult<T> = std::result::Result<T, RemeshError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = RemeshError::EmptyMesh;
        assert_eq!(format!("{err}"), "Mesh has no vertices");

        let err = RemeshError::InvalidEdgeLength(0.0);
        assert!(format!("{err}").contains("0"));

        let err = RemeshError::InvalidIterations(0);
        assert!(format!("{err}").contains("0"));
    }
}
