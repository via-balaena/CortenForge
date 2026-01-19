//! Error types for sensor-fusion crate.

use thiserror::Error;

/// Errors that can occur in sensor fusion operations.
#[derive(Debug, Error)]
pub enum FusionError {
    /// Insufficient data for operation.
    #[error("insufficient data: {0}")]
    InsufficientData(String),

    /// Timestamp out of range.
    #[error("timestamp out of range: {timestamp} not in [{min}, {max}]")]
    TimestampOutOfRange {
        /// The requested timestamp.
        timestamp: f64,
        /// Minimum available timestamp.
        min: f64,
        /// Maximum available timestamp.
        max: f64,
    },

    /// Invalid configuration.
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),

    /// Transform error.
    #[error("transform error: {0}")]
    Transform(String),

    /// Synchronization error.
    #[error("synchronization error: {0}")]
    Sync(String),

    /// Buffer overflow.
    #[error("buffer overflow: capacity {capacity}, attempted to add {attempted}")]
    BufferOverflow {
        /// Buffer capacity.
        capacity: usize,
        /// Attempted size.
        attempted: usize,
    },

    /// Stream not found.
    #[error("stream not found: {0}")]
    StreamNotFound(String),
}

impl FusionError {
    /// Creates an insufficient data error.
    #[must_use]
    pub fn insufficient_data(reason: impl Into<String>) -> Self {
        Self::InsufficientData(reason.into())
    }

    /// Creates a timestamp out of range error.
    #[must_use]
    pub const fn timestamp_out_of_range(timestamp: f64, min: f64, max: f64) -> Self {
        Self::TimestampOutOfRange {
            timestamp,
            min,
            max,
        }
    }

    /// Creates an invalid configuration error.
    #[must_use]
    pub fn invalid_config(reason: impl Into<String>) -> Self {
        Self::InvalidConfig(reason.into())
    }

    /// Creates a transform error.
    #[must_use]
    pub fn transform(reason: impl Into<String>) -> Self {
        Self::Transform(reason.into())
    }

    /// Creates a synchronization error.
    #[must_use]
    pub fn sync(reason: impl Into<String>) -> Self {
        Self::Sync(reason.into())
    }

    /// Creates a buffer overflow error.
    #[must_use]
    pub const fn buffer_overflow(capacity: usize, attempted: usize) -> Self {
        Self::BufferOverflow {
            capacity,
            attempted,
        }
    }

    /// Creates a stream not found error.
    #[must_use]
    pub fn stream_not_found(name: impl Into<String>) -> Self {
        Self::StreamNotFound(name.into())
    }
}

/// Result type for sensor fusion operations.
pub type Result<T> = std::result::Result<T, FusionError>;

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names
)]
mod tests {
    use super::*;

    #[test]
    fn error_insufficient_data() {
        let err = FusionError::insufficient_data("need at least 2 samples");
        assert!(err.to_string().contains("insufficient data"));
    }

    #[test]
    fn error_timestamp_out_of_range() {
        let err = FusionError::timestamp_out_of_range(5.0, 0.0, 3.0);
        assert!(err.to_string().contains("timestamp out of range"));
        assert!(err.to_string().contains('5'));
    }

    #[test]
    fn error_invalid_config() {
        let err = FusionError::invalid_config("window size must be positive");
        assert!(err.to_string().contains("invalid configuration"));
    }

    #[test]
    fn error_transform() {
        let err = FusionError::transform("singular matrix");
        assert!(err.to_string().contains("transform error"));
    }

    #[test]
    fn error_sync() {
        let err = FusionError::sync("streams out of phase");
        assert!(err.to_string().contains("synchronization error"));
    }

    #[test]
    fn error_buffer_overflow() {
        let err = FusionError::buffer_overflow(100, 150);
        assert!(err.to_string().contains("buffer overflow"));
        assert!(err.to_string().contains("100"));
        assert!(err.to_string().contains("150"));
    }

    #[test]
    fn error_stream_not_found() {
        let err = FusionError::stream_not_found("lidar_front");
        assert!(err.to_string().contains("stream not found"));
        assert!(err.to_string().contains("lidar_front"));
    }
}
