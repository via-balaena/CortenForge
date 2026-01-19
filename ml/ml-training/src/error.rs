//! Error types for ml-training crate.

use thiserror::Error;

/// Errors that can occur during training.
#[derive(Debug, Error)]
pub enum TrainingError {
    /// Invalid training configuration.
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),

    /// Dataset error.
    #[error("dataset error: {0}")]
    Dataset(String),

    /// Model error.
    #[error("model error: {0}")]
    Model(String),

    /// Checkpoint error.
    #[error("checkpoint error: {0}")]
    Checkpoint(String),

    /// Loss computation error.
    #[error("loss error: {0}")]
    Loss(String),

    /// Optimizer error.
    #[error("optimizer error: {0}")]
    Optimizer(String),

    /// Training was interrupted.
    #[error("training interrupted: {0}")]
    Interrupted(String),

    /// Numerical instability detected.
    #[error("numerical instability: {0}")]
    NumericalInstability(String),

    /// IO error.
    #[error("IO error: {0}")]
    Io(String),
}

impl TrainingError {
    /// Creates an invalid configuration error.
    #[must_use]
    pub fn invalid_config(reason: impl Into<String>) -> Self {
        Self::InvalidConfig(reason.into())
    }

    /// Creates a dataset error.
    #[must_use]
    pub fn dataset(reason: impl Into<String>) -> Self {
        Self::Dataset(reason.into())
    }

    /// Creates a model error.
    #[must_use]
    pub fn model(reason: impl Into<String>) -> Self {
        Self::Model(reason.into())
    }

    /// Creates a checkpoint error.
    #[must_use]
    pub fn checkpoint(reason: impl Into<String>) -> Self {
        Self::Checkpoint(reason.into())
    }

    /// Creates a loss error.
    #[must_use]
    pub fn loss(reason: impl Into<String>) -> Self {
        Self::Loss(reason.into())
    }

    /// Creates an optimizer error.
    #[must_use]
    pub fn optimizer(reason: impl Into<String>) -> Self {
        Self::Optimizer(reason.into())
    }

    /// Creates an interrupted error.
    #[must_use]
    pub fn interrupted(reason: impl Into<String>) -> Self {
        Self::Interrupted(reason.into())
    }

    /// Creates a numerical instability error.
    #[must_use]
    pub fn numerical_instability(reason: impl Into<String>) -> Self {
        Self::NumericalInstability(reason.into())
    }

    /// Creates an IO error.
    #[must_use]
    pub fn io(reason: impl Into<String>) -> Self {
        Self::Io(reason.into())
    }
}

impl From<std::io::Error> for TrainingError {
    fn from(err: std::io::Error) -> Self {
        Self::Io(err.to_string())
    }
}

impl From<serde_json::Error> for TrainingError {
    fn from(err: serde_json::Error) -> Self {
        Self::Io(err.to_string())
    }
}

impl From<ml_dataset::DatasetError> for TrainingError {
    fn from(err: ml_dataset::DatasetError) -> Self {
        Self::Dataset(err.to_string())
    }
}

impl From<ml_models::ModelError> for TrainingError {
    fn from(err: ml_models::ModelError) -> Self {
        Self::Model(err.to_string())
    }
}

/// Result type for training operations.
pub type Result<T> = std::result::Result<T, TrainingError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_invalid_config() {
        let err = TrainingError::invalid_config("batch size must be > 0");
        assert!(err.to_string().contains("invalid configuration"));
        assert!(err.to_string().contains("batch size"));
    }

    #[test]
    fn error_dataset() {
        let err = TrainingError::dataset("empty dataset");
        assert!(err.to_string().contains("dataset error"));
    }

    #[test]
    fn error_model() {
        let err = TrainingError::model("dimension mismatch");
        assert!(err.to_string().contains("model error"));
    }

    #[test]
    fn error_checkpoint() {
        let err = TrainingError::checkpoint("file not found");
        assert!(err.to_string().contains("checkpoint error"));
    }

    #[test]
    fn error_loss() {
        let err = TrainingError::loss("NaN detected");
        assert!(err.to_string().contains("loss error"));
    }

    #[test]
    fn error_optimizer() {
        let err = TrainingError::optimizer("invalid learning rate");
        assert!(err.to_string().contains("optimizer error"));
    }

    #[test]
    fn error_interrupted() {
        let err = TrainingError::interrupted("user cancelled");
        assert!(err.to_string().contains("training interrupted"));
    }

    #[test]
    fn error_numerical_instability() {
        let err = TrainingError::numerical_instability("gradient explosion");
        assert!(err.to_string().contains("numerical instability"));
    }

    #[test]
    fn error_io() {
        let err = TrainingError::io("permission denied");
        assert!(err.to_string().contains("IO error"));
    }

    #[test]
    fn error_from_io_error() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "test");
        let err: TrainingError = io_err.into();
        assert!(matches!(err, TrainingError::Io(_)));
    }
}
