//! Error types for tendon simulation.

use thiserror::Error;

/// Errors that can occur in tendon simulation.
#[derive(Debug, Error)]
pub enum TendonError {
    /// Invalid tendon configuration.
    #[error("Invalid tendon configuration: {0}")]
    InvalidConfig(String),

    /// Joint index out of bounds.
    #[error("Joint index {index} out of bounds (max {max})")]
    JointIndexOutOfBounds {
        /// The invalid index.
        index: usize,
        /// Maximum valid index.
        max: usize,
    },

    /// Tendon length became negative (impossible for cables).
    #[error("Negative tendon length: {length}")]
    NegativeLength {
        /// The computed length.
        length: f64,
    },

    /// Wrapping geometry failed to compute valid path.
    #[error("Wrapping computation failed: {0}")]
    WrappingFailed(String),

    /// Invalid attachment point.
    #[error("Invalid attachment point: {0}")]
    InvalidAttachment(String),

    /// Cable exceeded tension limit.
    #[error("Cable tension {tension} exceeded limit {limit}")]
    TensionLimitExceeded {
        /// Current tension.
        tension: f64,
        /// Maximum allowed tension.
        limit: f64,
    },
}

impl TendonError {
    /// Create an invalid configuration error.
    pub fn invalid_config(msg: impl Into<String>) -> Self {
        Self::InvalidConfig(msg.into())
    }

    /// Create a joint index out of bounds error.
    pub fn joint_out_of_bounds(index: usize, max: usize) -> Self {
        Self::JointIndexOutOfBounds { index, max }
    }

    /// Create a negative length error.
    pub fn negative_length(length: f64) -> Self {
        Self::NegativeLength { length }
    }

    /// Create a wrapping failed error.
    pub fn wrapping_failed(msg: impl Into<String>) -> Self {
        Self::WrappingFailed(msg.into())
    }

    /// Create an invalid attachment error.
    pub fn invalid_attachment(msg: impl Into<String>) -> Self {
        Self::InvalidAttachment(msg.into())
    }

    /// Create a tension limit exceeded error.
    pub fn tension_exceeded(tension: f64, limit: f64) -> Self {
        Self::TensionLimitExceeded { tension, limit }
    }
}

/// Result type for tendon operations.
pub type Result<T> = std::result::Result<T, TendonError>;
