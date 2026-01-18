//! Error types for mesh subdivision operations.

use thiserror::Error;

/// Errors that can occur during subdivision operations.
#[derive(Debug, Error)]
pub enum SubdivideError {
    /// Mesh has no vertices.
    #[error("Mesh has no vertices")]
    EmptyMesh,

    /// Mesh has no faces.
    #[error("Mesh has no faces")]
    NoFaces,

    /// Invalid iteration count.
    #[error("Invalid iteration count: {0} (must be >= 1)")]
    InvalidIterations(u32),

    /// Mesh would exceed maximum size.
    #[error("Subdivision would exceed maximum mesh size ({current} -> {projected} faces, max {max})")]
    MeshTooLarge {
        /// Current face count.
        current: usize,
        /// Projected face count after subdivision.
        projected: usize,
        /// Maximum allowed face count.
        max: usize,
    },
}

/// Result type for subdivision operations.
pub type SubdivideResult<T> = std::result::Result<T, SubdivideError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = SubdivideError::EmptyMesh;
        assert_eq!(format!("{err}"), "Mesh has no vertices");

        let err = SubdivideError::InvalidIterations(0);
        assert!(format!("{err}").contains("0"));

        let err = SubdivideError::MeshTooLarge {
            current: 1000,
            projected: 4000,
            max: 2000,
        };
        let display = format!("{err}");
        assert!(display.contains("1000"));
        assert!(display.contains("4000"));
        assert!(display.contains("2000"));
    }
}
