//! Error types for mesh slicing operations.

use std::path::PathBuf;
use thiserror::Error;

/// Errors that can occur during slicing operations.
#[derive(Debug, Error)]
pub enum SliceError {
    /// Mesh has no vertices.
    #[error("Mesh has no vertices")]
    EmptyMesh,

    /// Mesh has no faces.
    #[error("Mesh has no faces")]
    NoFaces,

    /// Invalid layer height.
    #[error("Invalid layer height: {0} (must be > 0)")]
    InvalidLayerHeight(f64),

    /// IO error during export.
    #[error("Failed to write to {path}: {source}")]
    IoWrite {
        /// The path that failed.
        path: PathBuf,
        /// The underlying IO error.
        #[source]
        source: std::io::Error,
    },
}

/// Result type for slicing operations.
pub type SliceResult<T> = std::result::Result<T, SliceError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = SliceError::EmptyMesh;
        assert_eq!(format!("{err}"), "Mesh has no vertices");

        let err = SliceError::InvalidLayerHeight(-0.1);
        assert!(format!("{err}").contains("-0.1"));
    }
}
