//! Error types for mesh-from-curves operations.

use thiserror::Error;

/// Result type for mesh-from-curves operations.
pub type CurveResult<T> = Result<T, CurveError>;

/// Errors that can occur during curve-to-mesh operations.
#[derive(Debug, Error)]
pub enum CurveError {
    /// Curve has too few points.
    #[error("curve needs at least {min} points, got {actual}")]
    TooFewPoints {
        /// Minimum required points.
        min: usize,
        /// Actual point count.
        actual: usize,
    },

    /// Profile has too few points.
    #[error("profile needs at least {min} points, got {actual}")]
    TooFewProfilePoints {
        /// Minimum required points.
        min: usize,
        /// Actual point count.
        actual: usize,
    },

    /// Radius is invalid (zero or negative).
    #[error("invalid radius: {0}")]
    InvalidRadius(f64),

    /// Segments count is too low.
    #[error("segments must be at least {min}, got {actual}")]
    TooFewSegments {
        /// Minimum required segments.
        min: usize,
        /// Actual segment count.
        actual: usize,
    },

    /// Degenerate curve segment (zero-length).
    #[error("degenerate curve segment at index {index}")]
    DegenerateSegment {
        /// Index of the degenerate segment.
        index: usize,
    },
}
