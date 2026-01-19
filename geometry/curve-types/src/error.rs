//! Error types for curve operations.

use thiserror::Error;

/// Errors that can occur during curve operations.
#[derive(Debug, Error, Clone, PartialEq)]
pub enum CurveError {
    /// Insufficient points to define the curve.
    #[error("insufficient points: need at least {required}, got {actual}")]
    InsufficientPoints {
        /// Minimum required points.
        required: usize,
        /// Actual number of points provided.
        actual: usize,
    },

    /// Parameter is outside the valid range [0, 1].
    #[error("parameter {0} is outside valid range [0, 1]")]
    ParameterOutOfRange(f64),

    /// Invalid degree for the curve type.
    #[error("invalid degree {degree}: must be between {min} and {max}")]
    InvalidDegree {
        /// Specified degree.
        degree: usize,
        /// Minimum allowed degree.
        min: usize,
        /// Maximum allowed degree.
        max: usize,
    },

    /// Knot vector is invalid for the given curve parameters.
    #[error("invalid knot vector: {reason}")]
    InvalidKnotVector {
        /// Description of what's wrong with the knot vector.
        reason: String,
    },

    /// Weight values are invalid (e.g., negative or zero).
    #[error("invalid weight at index {index}: {value} (must be positive)")]
    InvalidWeight {
        /// Index of the invalid weight.
        index: usize,
        /// The invalid weight value.
        value: f64,
    },

    /// Radius must be positive.
    #[error("invalid radius: {0} (must be positive)")]
    InvalidRadius(f64),

    /// Angle values are invalid.
    #[error("invalid angle: {reason}")]
    InvalidAngle {
        /// Description of what's wrong with the angle.
        reason: String,
    },

    /// Curves cannot be joined (endpoints don't match).
    #[error("curves cannot be joined: endpoint gap of {gap}")]
    CannotJoin {
        /// Distance between curve endpoints.
        gap: f64,
    },

    /// Continuity requirement cannot be satisfied.
    #[error("cannot achieve {requested:?} continuity: {reason}")]
    ContinuityViolation {
        /// Requested continuity level.
        requested: super::Continuity,
        /// Reason why continuity cannot be achieved.
        reason: String,
    },

    /// Degenerate curve (e.g., zero length).
    #[error("degenerate curve: {reason}")]
    Degenerate {
        /// Description of the degeneracy.
        reason: String,
    },

    /// Numerical computation failed to converge.
    #[error("numerical computation failed: {reason}")]
    NumericalError {
        /// Description of the numerical issue.
        reason: String,
    },

    /// Arc length parameterization failed.
    #[error("arc length computation failed: {reason}")]
    ArcLengthError {
        /// Description of the error.
        reason: String,
    },

    /// Split parameter is invalid.
    #[error("cannot split at t={t}: {reason}")]
    InvalidSplit {
        /// The split parameter.
        t: f64,
        /// Reason why split is invalid.
        reason: String,
    },
}

impl CurveError {
    /// Create an insufficient points error.
    #[must_use]
    pub fn insufficient_points(required: usize, actual: usize) -> Self {
        Self::InsufficientPoints { required, actual }
    }

    /// Create an invalid knot vector error.
    #[must_use]
    pub fn invalid_knot_vector(reason: impl Into<String>) -> Self {
        Self::InvalidKnotVector {
            reason: reason.into(),
        }
    }

    /// Create a degenerate curve error.
    #[must_use]
    pub fn degenerate(reason: impl Into<String>) -> Self {
        Self::Degenerate {
            reason: reason.into(),
        }
    }

    /// Create a numerical error.
    #[must_use]
    pub fn numerical(reason: impl Into<String>) -> Self {
        Self::NumericalError {
            reason: reason.into(),
        }
    }

    /// Create an arc length error.
    #[must_use]
    pub fn arc_length(reason: impl Into<String>) -> Self {
        Self::ArcLengthError {
            reason: reason.into(),
        }
    }

    /// Create an invalid split error.
    #[must_use]
    pub fn invalid_split(t: f64, reason: impl Into<String>) -> Self {
        Self::InvalidSplit {
            t,
            reason: reason.into(),
        }
    }

    /// Check if this is an insufficient points error.
    #[must_use]
    pub fn is_insufficient_points(&self) -> bool {
        matches!(self, Self::InsufficientPoints { .. })
    }

    /// Check if this is a parameter out of range error.
    #[must_use]
    pub fn is_parameter_out_of_range(&self) -> bool {
        matches!(self, Self::ParameterOutOfRange(_))
    }

    /// Check if this is a numerical error.
    #[must_use]
    pub fn is_numerical(&self) -> bool {
        matches!(self, Self::NumericalError { .. })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = CurveError::insufficient_points(4, 2);
        assert!(err.to_string().contains("need at least 4"));
        assert!(err.to_string().contains("got 2"));

        let err = CurveError::ParameterOutOfRange(1.5);
        assert!(err.to_string().contains("1.5"));

        let err = CurveError::InvalidRadius(-1.0);
        assert!(err.to_string().contains("-1"));
    }

    #[test]
    fn test_error_predicates() {
        let err = CurveError::insufficient_points(3, 1);
        assert!(err.is_insufficient_points());
        assert!(!err.is_parameter_out_of_range());

        let err = CurveError::ParameterOutOfRange(2.0);
        assert!(err.is_parameter_out_of_range());
        assert!(!err.is_insufficient_points());

        let err = CurveError::numerical("convergence failed");
        assert!(err.is_numerical());
    }

    #[test]
    fn test_error_constructors() {
        let err = CurveError::degenerate("zero length");
        assert!(matches!(err, CurveError::Degenerate { reason } if reason == "zero length"));

        let err = CurveError::invalid_knot_vector("not monotonic");
        assert!(
            matches!(err, CurveError::InvalidKnotVector { reason } if reason == "not monotonic")
        );
    }
}
