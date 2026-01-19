//! Error types for sensor operations.

use thiserror::Error;

/// Errors that can occur when working with sensor data.
#[derive(Debug, Error)]
pub enum SensorError {
    /// Invalid timestamp (e.g., negative duration).
    #[error("invalid timestamp: {0}")]
    InvalidTimestamp(String),

    /// Invalid coordinate value (e.g., `NaN` or out of range).
    #[error("invalid coordinate: {0}")]
    InvalidCoordinate(String),

    /// Buffer size mismatch (e.g., image buffer wrong size).
    #[error("buffer size mismatch: expected {expected}, got {actual}")]
    BufferSizeMismatch {
        /// Expected buffer size.
        expected: usize,
        /// Actual buffer size.
        actual: usize,
    },

    /// Sensor data is inconsistent (e.g., mismatched vector lengths).
    #[error("inconsistent data: {0}")]
    InconsistentData(String),

    /// Required calibration data is missing.
    #[error("missing calibration: {0}")]
    MissingCalibration(String),

    /// Coordinate frame mismatch.
    #[error("frame mismatch: expected {expected}, got {actual}")]
    FrameMismatch {
        /// Expected coordinate frame.
        expected: String,
        /// Actual coordinate frame.
        actual: String,
    },

    /// Sensor data is stale (too old).
    #[error("stale data: timestamp {timestamp_ns} ns is older than threshold {max_age_ns} ns")]
    StaleData {
        /// Data timestamp in nanoseconds.
        timestamp_ns: u64,
        /// Maximum allowed age in nanoseconds.
        max_age_ns: u64,
    },

    /// Division by zero or near-zero value.
    #[error("division by zero or near-zero: {context}")]
    DivisionByZero {
        /// Context where the division occurred.
        context: String,
    },
}

impl SensorError {
    /// Creates a buffer size mismatch error.
    #[must_use]
    pub const fn buffer_mismatch(expected: usize, actual: usize) -> Self {
        Self::BufferSizeMismatch { expected, actual }
    }

    /// Creates a frame mismatch error.
    #[must_use]
    pub fn frame_mismatch(expected: impl Into<String>, actual: impl Into<String>) -> Self {
        Self::FrameMismatch {
            expected: expected.into(),
            actual: actual.into(),
        }
    }

    /// Creates a stale data error.
    #[must_use]
    pub const fn stale(timestamp_ns: u64, max_age_ns: u64) -> Self {
        Self::StaleData {
            timestamp_ns,
            max_age_ns,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_display() {
        let err = SensorError::InvalidTimestamp("negative".to_string());
        let msg = format!("{err}");
        assert!(msg.contains("invalid timestamp"));
        assert!(msg.contains("negative"));
    }

    #[test]
    fn error_buffer_mismatch() {
        let err = SensorError::buffer_mismatch(100, 50);
        let msg = format!("{err}");
        assert!(msg.contains("100"));
        assert!(msg.contains("50"));
    }

    #[test]
    fn error_frame_mismatch() {
        let err = SensorError::frame_mismatch("body", "world");
        let msg = format!("{err}");
        assert!(msg.contains("body"));
        assert!(msg.contains("world"));
    }

    #[test]
    fn error_stale() {
        let err = SensorError::stale(1000, 500);
        let msg = format!("{err}");
        assert!(msg.contains("1000"));
        assert!(msg.contains("500"));
    }
}
