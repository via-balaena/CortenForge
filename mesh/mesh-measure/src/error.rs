//! Error types for measurement operations.

use thiserror::Error;

/// Result type alias for measurement operations.
pub type MeasureResult<T> = Result<T, MeasureError>;

/// Errors that can occur during measurement operations.
#[derive(Debug, Error)]
pub enum MeasureError {
    /// Input mesh is empty.
    #[error("input mesh is empty")]
    EmptyMesh,

    /// Invalid measurement parameters.
    #[error("invalid parameters: {0}")]
    InvalidParams(String),

    /// Measurement operation failed.
    #[error("measurement failed: {0}")]
    MeasurementFailed(String),
}

impl MeasureError {
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

    /// Create a measurement failed error.
    #[must_use]
    pub fn measurement_failed(details: impl Into<String>) -> Self {
        Self::MeasurementFailed(details.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = MeasureError::empty_mesh();
        assert!(format!("{err}").contains("empty"));

        let err = MeasureError::invalid_params("bad value");
        assert!(format!("{err}").contains("bad value"));

        let err = MeasureError::measurement_failed("computation error");
        assert!(format!("{err}").contains("computation error"));
    }
}
