//! Error types for ml-dataset crate.

use thiserror::Error;

/// Errors that can occur in ml-dataset operations.
#[derive(Debug, Error)]
pub enum DatasetError {
    /// Failed to load sample.
    #[error("failed to load sample {id}: {reason}")]
    LoadSample {
        /// Sample ID.
        id: u64,
        /// Reason for failure.
        reason: String,
    },

    /// Sample not found.
    #[error("sample not found: {0}")]
    SampleNotFound(String),

    /// Invalid image dimensions.
    #[error("invalid image dimensions: {width}x{height}")]
    InvalidDimensions {
        /// Width in pixels.
        width: u32,
        /// Height in pixels.
        height: u32,
    },

    /// Invalid split ratio.
    #[error("invalid split ratio: {0} (must be in (0, 1))")]
    InvalidSplitRatio(f32),

    /// Empty dataset.
    #[error("dataset is empty")]
    EmptyDataset,

    /// Shard not found.
    #[error("shard not found: {0}")]
    ShardNotFound(String),

    /// Invalid manifest.
    #[error("invalid manifest: {0}")]
    InvalidManifest(String),

    /// IO error.
    #[error("IO error: {0}")]
    Io(String),

    /// Serialization error.
    #[error("serialization error: {0}")]
    Serialization(String),

    /// Validation error.
    #[error("validation error: {0}")]
    Validation(String),
}

impl DatasetError {
    /// Creates a load sample error.
    #[must_use]
    pub fn load_sample(id: u64, reason: impl Into<String>) -> Self {
        Self::LoadSample {
            id,
            reason: reason.into(),
        }
    }

    /// Creates a sample not found error.
    #[must_use]
    pub fn sample_not_found(path: impl Into<String>) -> Self {
        Self::SampleNotFound(path.into())
    }

    /// Creates an invalid dimensions error.
    #[must_use]
    pub const fn invalid_dimensions(width: u32, height: u32) -> Self {
        Self::InvalidDimensions { width, height }
    }

    /// Creates an invalid split ratio error.
    #[must_use]
    pub const fn invalid_split_ratio(ratio: f32) -> Self {
        Self::InvalidSplitRatio(ratio)
    }

    /// Creates a shard not found error.
    #[must_use]
    pub fn shard_not_found(id: impl Into<String>) -> Self {
        Self::ShardNotFound(id.into())
    }

    /// Creates an invalid manifest error.
    #[must_use]
    pub fn invalid_manifest(reason: impl Into<String>) -> Self {
        Self::InvalidManifest(reason.into())
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

    /// Creates a validation error.
    #[must_use]
    pub fn validation(reason: impl Into<String>) -> Self {
        Self::Validation(reason.into())
    }
}

impl From<std::io::Error> for DatasetError {
    fn from(err: std::io::Error) -> Self {
        Self::Io(err.to_string())
    }
}

impl From<serde_json::Error> for DatasetError {
    fn from(err: serde_json::Error) -> Self {
        Self::Serialization(err.to_string())
    }
}

/// Result type for ml-dataset operations.
pub type Result<T> = std::result::Result<T, DatasetError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_load_sample() {
        let err = DatasetError::load_sample(42, "file corrupted");
        assert!(err.to_string().contains("42"));
        assert!(err.to_string().contains("file corrupted"));
    }

    #[test]
    fn error_sample_not_found() {
        let err = DatasetError::sample_not_found("/path/to/sample");
        assert!(err.to_string().contains("/path/to/sample"));
    }

    #[test]
    fn error_invalid_dimensions() {
        let err = DatasetError::invalid_dimensions(0, 480);
        assert!(err.to_string().contains("0x480"));
    }

    #[test]
    fn error_invalid_split_ratio() {
        let err = DatasetError::invalid_split_ratio(1.5);
        assert!(err.to_string().contains("1.5"));
    }

    #[test]
    fn error_shard_not_found() {
        let err = DatasetError::shard_not_found("shard_001");
        assert!(err.to_string().contains("shard_001"));
    }

    #[test]
    fn error_invalid_manifest() {
        let err = DatasetError::invalid_manifest("missing version");
        assert!(err.to_string().contains("missing version"));
    }

    #[test]
    fn error_io() {
        let err = DatasetError::io("permission denied");
        assert!(err.to_string().contains("permission denied"));
    }

    #[test]
    fn error_serialization() {
        let err = DatasetError::serialization("invalid JSON");
        assert!(err.to_string().contains("invalid JSON"));
    }

    #[test]
    fn error_validation() {
        let err = DatasetError::validation("missing labels");
        assert!(err.to_string().contains("missing labels"));
    }

    #[test]
    fn error_from_io_error() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "not found");
        let err: DatasetError = io_err.into();
        assert!(matches!(err, DatasetError::Io(_)));
    }

    #[test]
    fn error_from_serde_error() {
        let json_err = serde_json::from_str::<i32>("invalid").unwrap_err();
        let err: DatasetError = json_err.into();
        assert!(matches!(err, DatasetError::Serialization(_)));
    }
}
