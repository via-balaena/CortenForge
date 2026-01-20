//! Error types for sensor operations.

use thiserror::Error;

/// Errors that can occur during sensor operations.
#[derive(Debug, Error, Clone)]
pub enum SensorError {
    /// The specified body was not found.
    #[error("body not found: {0}")]
    BodyNotFound(String),

    /// The specified joint was not found.
    #[error("joint not found: {0}")]
    JointNotFound(String),

    /// Invalid sensor configuration.
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),

    /// Sensor reading failed.
    #[error("reading failed: {0}")]
    ReadingFailed(String),
}

impl SensorError {
    /// Create a body not found error.
    pub fn body_not_found(msg: impl Into<String>) -> Self {
        Self::BodyNotFound(msg.into())
    }

    /// Create a joint not found error.
    pub fn joint_not_found(msg: impl Into<String>) -> Self {
        Self::JointNotFound(msg.into())
    }

    /// Create an invalid config error.
    pub fn invalid_config(msg: impl Into<String>) -> Self {
        Self::InvalidConfig(msg.into())
    }

    /// Create a reading failed error.
    pub fn reading_failed(msg: impl Into<String>) -> Self {
        Self::ReadingFailed(msg.into())
    }
}
