//! Error types for ml-models crate.

use thiserror::Error;

/// Errors that can occur in ml-models operations.
#[derive(Debug, Error)]
pub enum ModelError {
    /// Failed to load checkpoint.
    #[error("failed to load checkpoint from {path}: {reason}")]
    LoadCheckpoint {
        /// Path to the checkpoint file.
        path: String,
        /// Reason for failure.
        reason: String,
    },

    /// Failed to save checkpoint.
    #[error("failed to save checkpoint to {path}: {reason}")]
    SaveCheckpoint {
        /// Path to the checkpoint file.
        path: String,
        /// Reason for failure.
        reason: String,
    },

    /// Invalid model configuration.
    #[error("invalid model configuration: {0}")]
    InvalidConfig(String),

    /// Checkpoint file not found.
    #[error("checkpoint not found: {0}")]
    CheckpointNotFound(String),

    /// Unsupported checkpoint format.
    #[error("unsupported checkpoint format: {0}")]
    UnsupportedFormat(String),

    /// Shape mismatch during inference.
    #[error("shape mismatch: expected {expected}, got {actual}")]
    ShapeMismatch {
        /// Expected shape.
        expected: String,
        /// Actual shape.
        actual: String,
    },

    /// Backend initialization error.
    #[error("backend initialization failed: {0}")]
    BackendInit(String),

    /// IO error.
    #[error("IO error: {0}")]
    Io(String),

    /// Serialization error.
    #[error("serialization error: {0}")]
    Serialization(String),
}

impl ModelError {
    /// Creates a load checkpoint error.
    #[must_use]
    pub fn load_checkpoint(path: impl Into<String>, reason: impl Into<String>) -> Self {
        Self::LoadCheckpoint {
            path: path.into(),
            reason: reason.into(),
        }
    }

    /// Creates a save checkpoint error.
    #[must_use]
    pub fn save_checkpoint(path: impl Into<String>, reason: impl Into<String>) -> Self {
        Self::SaveCheckpoint {
            path: path.into(),
            reason: reason.into(),
        }
    }

    /// Creates an invalid config error.
    #[must_use]
    pub fn invalid_config(reason: impl Into<String>) -> Self {
        Self::InvalidConfig(reason.into())
    }

    /// Creates a checkpoint not found error.
    #[must_use]
    pub fn checkpoint_not_found(path: impl Into<String>) -> Self {
        Self::CheckpointNotFound(path.into())
    }

    /// Creates an unsupported format error.
    #[must_use]
    pub fn unsupported_format(format: impl Into<String>) -> Self {
        Self::UnsupportedFormat(format.into())
    }

    /// Creates a shape mismatch error.
    #[must_use]
    pub fn shape_mismatch(expected: impl Into<String>, actual: impl Into<String>) -> Self {
        Self::ShapeMismatch {
            expected: expected.into(),
            actual: actual.into(),
        }
    }

    /// Creates a backend initialization error.
    #[must_use]
    pub fn backend_init(reason: impl Into<String>) -> Self {
        Self::BackendInit(reason.into())
    }

    /// Creates an IO error.
    #[must_use]
    pub fn io(reason: impl Into<String>) -> Self {
        Self::Io(reason.into())
    }

    /// Creates a serialization error.
    #[must_use]
    pub fn serialization(reason: impl Into<String>) -> Self {
        Self::Serialization(reason.into())
    }
}

impl From<std::io::Error> for ModelError {
    fn from(err: std::io::Error) -> Self {
        Self::Io(err.to_string())
    }
}

/// Result type for ml-models operations.
pub type Result<T> = std::result::Result<T, ModelError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_load_checkpoint() {
        let err = ModelError::load_checkpoint("model.bin", "file corrupted");
        assert!(err.to_string().contains("model.bin"));
        assert!(err.to_string().contains("file corrupted"));
    }

    #[test]
    fn error_save_checkpoint() {
        let err = ModelError::save_checkpoint("output.bin", "disk full");
        assert!(err.to_string().contains("output.bin"));
        assert!(err.to_string().contains("disk full"));
    }

    #[test]
    fn error_invalid_config() {
        let err = ModelError::invalid_config("hidden must be > 0");
        assert!(err.to_string().contains("hidden must be > 0"));
    }

    #[test]
    fn error_checkpoint_not_found() {
        let err = ModelError::checkpoint_not_found("/path/to/missing.bin");
        assert!(err.to_string().contains("/path/to/missing.bin"));
    }

    #[test]
    fn error_unsupported_format() {
        let err = ModelError::unsupported_format("xml");
        assert!(err.to_string().contains("xml"));
    }

    #[test]
    fn error_shape_mismatch() {
        let err = ModelError::shape_mismatch("[1, 4]", "[1, 8]");
        assert!(err.to_string().contains("[1, 4]"));
        assert!(err.to_string().contains("[1, 8]"));
    }

    #[test]
    fn error_backend_init() {
        let err = ModelError::backend_init("GPU not available");
        assert!(err.to_string().contains("GPU not available"));
    }

    #[test]
    fn error_io() {
        let err = ModelError::io("permission denied");
        assert!(err.to_string().contains("permission denied"));
    }

    #[test]
    fn error_serialization() {
        let err = ModelError::serialization("invalid JSON");
        assert!(err.to_string().contains("invalid JSON"));
    }

    #[test]
    fn error_from_io_error() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "not found");
        let err: ModelError = io_err.into();
        assert!(matches!(err, ModelError::Io(_)));
    }
}
