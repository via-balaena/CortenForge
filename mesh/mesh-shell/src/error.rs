//! Error types for shell operations.
//!
//! This module provides comprehensive error handling for shell generation.

use thiserror::Error;

/// Result type alias for shell operations.
pub type ShellResult<T> = Result<T, ShellError>;

/// Errors that can occur during shell operations.
#[derive(Debug, Error)]
pub enum ShellError {
    /// Input mesh is empty.
    #[error("input mesh is empty")]
    EmptyMesh,

    /// Invalid shell parameters.
    #[error("invalid shell parameters: {0}")]
    InvalidParams(String),

    /// Shell generation failed.
    #[error("shell generation failed: {0}")]
    GenerationFailed(String),

    /// Rim generation failed.
    #[error("rim generation failed: {0}")]
    RimFailed(String),

    /// Offset operation failed.
    #[error("offset operation failed: {0}")]
    OffsetFailed(#[from] mesh_offset::OffsetError),

    /// Repair operation failed.
    #[error("repair operation failed: {0}")]
    RepairFailed(#[from] mesh_repair::RepairError),
}

impl ShellError {
    /// Create an empty mesh error.
    #[must_use]
    pub const fn empty_mesh() -> Self {
        Self::EmptyMesh
    }

    /// Create an invalid params error.
    #[must_use]
    pub fn invalid_params(details: impl Into<String>) -> Self {
        Self::InvalidParams(details.into())
    }

    /// Create a generation failed error.
    #[must_use]
    pub fn generation_failed(details: impl Into<String>) -> Self {
        Self::GenerationFailed(details.into())
    }

    /// Create a rim failed error.
    #[must_use]
    pub fn rim_failed(details: impl Into<String>) -> Self {
        Self::RimFailed(details.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = ShellError::empty_mesh();
        assert!(format!("{err}").contains("empty"));

        let err = ShellError::invalid_params("bad value");
        assert!(format!("{err}").contains("bad value"));

        let err = ShellError::generation_failed("no faces");
        assert!(format!("{err}").contains("no faces"));

        let err = ShellError::rim_failed("boundary issue");
        assert!(format!("{err}").contains("boundary issue"));
    }
}
