//! Error types for ml-types crate.

use thiserror::Error;

/// Errors that can occur in ml-types operations.
#[derive(Debug, Error)]
pub enum MlTypesError {
    /// Invalid bounding box coordinates.
    #[error("invalid bounding box: {0}")]
    InvalidBoundingBox(String),

    /// Invalid image dimensions.
    #[error("invalid dimensions: {width}x{height}")]
    InvalidDimensions {
        /// Width in pixels.
        width: u32,
        /// Height in pixels.
        height: u32,
    },

    /// Invalid confidence value.
    #[error("invalid confidence {value}: must be in [0, 1]")]
    InvalidConfidence {
        /// The invalid confidence value.
        value: f32,
    },

    /// Invalid class ID.
    #[error("invalid class ID {id}: expected < {max}")]
    InvalidClassId {
        /// The invalid class ID.
        id: u32,
        /// Maximum valid class ID.
        max: u32,
    },

    /// Data size mismatch.
    #[error("data size mismatch: expected {expected}, got {actual}")]
    DataSizeMismatch {
        /// Expected size.
        expected: usize,
        /// Actual size.
        actual: usize,
    },

    /// Invalid frame ID.
    #[error("invalid frame ID: {0}")]
    InvalidFrameId(String),

    /// Missing required field.
    #[error("missing required field: {0}")]
    MissingField(String),

    /// Schema version mismatch.
    #[error("schema version mismatch: expected {expected}, got {actual}")]
    SchemaVersionMismatch {
        /// Expected version.
        expected: String,
        /// Actual version.
        actual: String,
    },

    /// Invalid manifest.
    #[error("invalid manifest: {0}")]
    InvalidManifest(String),

    /// Serialization error.
    #[error("serialization error: {0}")]
    Serialization(String),

    /// Deserialization error.
    #[error("deserialization error: {0}")]
    Deserialization(String),

    /// Validation error.
    #[error("validation error: {0}")]
    Validation(String),

    /// IO error.
    #[error("IO error: {0}")]
    Io(String),
}

impl MlTypesError {
    /// Creates an invalid bounding box error.
    #[must_use]
    pub fn invalid_bbox(reason: impl Into<String>) -> Self {
        Self::InvalidBoundingBox(reason.into())
    }

    /// Creates an invalid dimensions error.
    #[must_use]
    pub const fn invalid_dimensions(width: u32, height: u32) -> Self {
        Self::InvalidDimensions { width, height }
    }

    /// Creates an invalid confidence error.
    #[must_use]
    pub const fn invalid_confidence(value: f32) -> Self {
        Self::InvalidConfidence { value }
    }

    /// Creates an invalid class ID error.
    #[must_use]
    pub const fn invalid_class_id(id: u32, max: u32) -> Self {
        Self::InvalidClassId { id, max }
    }

    /// Creates a data size mismatch error.
    #[must_use]
    pub const fn data_size_mismatch(expected: usize, actual: usize) -> Self {
        Self::DataSizeMismatch { expected, actual }
    }

    /// Creates an invalid frame ID error.
    #[must_use]
    pub fn invalid_frame_id(id: impl Into<String>) -> Self {
        Self::InvalidFrameId(id.into())
    }

    /// Creates a missing field error.
    #[must_use]
    pub fn missing_field(field: impl Into<String>) -> Self {
        Self::MissingField(field.into())
    }

    /// Creates a schema version mismatch error.
    #[must_use]
    pub fn schema_mismatch(expected: impl Into<String>, actual: impl Into<String>) -> Self {
        Self::SchemaVersionMismatch {
            expected: expected.into(),
            actual: actual.into(),
        }
    }

    /// Creates an invalid manifest error.
    #[must_use]
    pub fn invalid_manifest(reason: impl Into<String>) -> Self {
        Self::InvalidManifest(reason.into())
    }

    /// Creates a serialization error.
    #[must_use]
    pub fn serialization(reason: impl Into<String>) -> Self {
        Self::Serialization(reason.into())
    }

    /// Creates a deserialization error.
    #[must_use]
    pub fn deserialization(reason: impl Into<String>) -> Self {
        Self::Deserialization(reason.into())
    }

    /// Creates a validation error.
    #[must_use]
    pub fn validation(reason: impl Into<String>) -> Self {
        Self::Validation(reason.into())
    }

    /// Creates an IO error.
    #[must_use]
    pub fn io(reason: impl Into<String>) -> Self {
        Self::Io(reason.into())
    }
}

impl From<std::io::Error> for MlTypesError {
    fn from(err: std::io::Error) -> Self {
        Self::Io(err.to_string())
    }
}

impl From<serde_json::Error> for MlTypesError {
    fn from(err: serde_json::Error) -> Self {
        if err.is_io() {
            Self::Io(err.to_string())
        } else if err.is_syntax() || err.is_data() || err.is_eof() {
            Self::Deserialization(err.to_string())
        } else {
            Self::Serialization(err.to_string())
        }
    }
}

/// Result type for ml-types operations.
pub type Result<T> = std::result::Result<T, MlTypesError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_invalid_bbox() {
        let err = MlTypesError::invalid_bbox("x_min > x_max");
        assert!(err.to_string().contains("invalid bounding box"));
        assert!(err.to_string().contains("x_min > x_max"));
    }

    #[test]
    fn error_invalid_dimensions() {
        let err = MlTypesError::invalid_dimensions(0, 480);
        assert!(err.to_string().contains("0x480"));
    }

    #[test]
    fn error_invalid_confidence() {
        let err = MlTypesError::invalid_confidence(1.5);
        assert!(err.to_string().contains("1.5"));
    }

    #[test]
    fn error_invalid_class_id() {
        let err = MlTypesError::invalid_class_id(100, 80);
        assert!(err.to_string().contains("100"));
        assert!(err.to_string().contains("80"));
    }

    #[test]
    fn error_data_size_mismatch() {
        let err = MlTypesError::data_size_mismatch(1000, 500);
        assert!(err.to_string().contains("1000"));
        assert!(err.to_string().contains("500"));
    }

    #[test]
    fn error_invalid_frame_id() {
        let err = MlTypesError::invalid_frame_id("bad_id");
        assert!(err.to_string().contains("bad_id"));
    }

    #[test]
    fn error_missing_field() {
        let err = MlTypesError::missing_field("timestamp");
        assert!(err.to_string().contains("timestamp"));
    }

    #[test]
    fn error_schema_mismatch() {
        let err = MlTypesError::schema_mismatch("1.0", "2.0");
        assert!(err.to_string().contains("1.0"));
        assert!(err.to_string().contains("2.0"));
    }

    #[test]
    fn error_invalid_manifest() {
        let err = MlTypesError::invalid_manifest("empty runs");
        assert!(err.to_string().contains("empty runs"));
    }

    #[test]
    fn error_serialization() {
        let err = MlTypesError::serialization("failed to serialize");
        assert!(err.to_string().contains("serialization"));
    }

    #[test]
    fn error_deserialization() {
        let err = MlTypesError::deserialization("invalid JSON");
        assert!(err.to_string().contains("deserialization"));
    }

    #[test]
    fn error_validation() {
        let err = MlTypesError::validation("constraints not met");
        assert!(err.to_string().contains("validation"));
    }

    #[test]
    fn error_io() {
        let err = MlTypesError::io("file not found");
        assert!(err.to_string().contains("IO error"));
    }

    #[test]
    fn error_from_io_error() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "test");
        let err: MlTypesError = io_err.into();
        assert!(matches!(err, MlTypesError::Io(_)));
    }

    #[test]
    fn error_from_serde_error() {
        let json_err = serde_json::from_str::<i32>("invalid").unwrap_err();
        let err: MlTypesError = json_err.into();
        assert!(matches!(err, MlTypesError::Deserialization(_)));
    }
}
