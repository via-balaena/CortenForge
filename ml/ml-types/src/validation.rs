//! Validation error types.

use thiserror::Error;

/// Validation errors for ML types.
#[derive(Debug, Error, Clone, PartialEq)]
pub enum ValidationError {
    /// Pixel bounding box has invalid order or negative values.
    #[error("invalid pixel bbox (order or negative): {0:?}")]
    InvalidBboxPx([f32; 4]),

    /// Normalized bounding box is out of range `[0, 1]`.
    #[error("normalized bbox out of range [0, 1]: {0:?}")]
    InvalidBboxNorm([f32; 4]),

    /// Source confidence is out of range `[0, 1]`.
    #[error("source confidence out of range [0, 1]: {0}")]
    InvalidSourceConfidence(f32),

    /// Missing required image path.
    #[error("missing image path for present frame")]
    MissingImage,

    /// Invalid timestamp (negative or `NaN`).
    #[error("invalid timestamp: {0}")]
    InvalidTimestamp(String),

    /// Invalid frame ID.
    #[error("invalid frame ID: {0}")]
    InvalidFrameId(String),

    /// Duplicate frame ID in manifest.
    #[error("duplicate frame ID: {0}")]
    DuplicateFrameId(u64),

    /// Missing required field.
    #[error("missing required field: {0}")]
    MissingField(String),

    /// Invalid class ID.
    #[error("invalid class ID: {0}")]
    InvalidClassId(u32),

    /// Schema version mismatch.
    #[error("schema version mismatch: expected {expected}, got {actual}")]
    SchemaVersionMismatch {
        /// Expected version.
        expected: String,
        /// Actual version.
        actual: String,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_display() {
        let err = ValidationError::InvalidBboxPx([0.5, 0.6, 0.1, 0.2]);
        let msg = format!("{err}");
        assert!(msg.contains("invalid pixel bbox"));

        let err = ValidationError::InvalidSourceConfidence(1.5);
        let msg = format!("{err}");
        assert!(msg.contains("1.5"));

        let err = ValidationError::MissingImage;
        let msg = format!("{err}");
        assert!(msg.contains("missing image"));
    }

    #[test]
    fn error_schema_mismatch() {
        let err = ValidationError::SchemaVersionMismatch {
            expected: "1.0".to_string(),
            actual: "2.0".to_string(),
        };
        let msg = format!("{err}");
        assert!(msg.contains("1.0"));
        assert!(msg.contains("2.0"));
    }
}
