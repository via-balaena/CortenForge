//! Error types for thickness analysis.

use thiserror::Error;

/// Result type alias for thickness operations.
pub type ThicknessResult<T> = Result<T, ThicknessError>;

/// Errors that can occur during thickness analysis.
#[derive(Debug, Error)]
pub enum ThicknessError {
    /// Input mesh is empty.
    #[error("input mesh is empty")]
    EmptyMesh,

    /// Invalid analysis parameters.
    #[error("invalid parameters: {0}")]
    InvalidParams(String),

    /// Analysis operation failed.
    #[error("analysis failed: {0}")]
    AnalysisFailed(String),
}

impl ThicknessError {
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

    /// Create an analysis failed error.
    #[must_use]
    pub fn analysis_failed(details: impl Into<String>) -> Self {
        Self::AnalysisFailed(details.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = ThicknessError::empty_mesh();
        assert!(format!("{err}").contains("empty"));

        let err = ThicknessError::invalid_params("bad threshold");
        assert!(format!("{err}").contains("bad threshold"));

        let err = ThicknessError::analysis_failed("no hits");
        assert!(format!("{err}").contains("no hits"));
    }
}
