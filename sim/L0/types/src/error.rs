//! Error types for simulation operations.

use thiserror::Error;

/// Errors that can occur during simulation.
#[derive(Debug, Error, Clone, PartialEq)]
pub enum SimError {
    /// Invalid body ID referenced.
    #[error("invalid body ID: {0}")]
    InvalidBodyId(u64),

    /// Invalid joint ID referenced.
    #[error("invalid joint ID: {0}")]
    InvalidJointId(u64),

    /// Joint limit violated.
    #[error("joint {joint_id} limit violated: {value} not in [{min}, {max}]")]
    JointLimitViolation {
        /// The joint that violated limits.
        joint_id: u64,
        /// The violating value.
        value: f64,
        /// Minimum allowed value.
        min: f64,
        /// Maximum allowed value.
        max: f64,
    },

    /// Invalid timestep.
    #[error("invalid timestep: {0} (must be positive and finite)")]
    InvalidTimestep(f64),

    /// Simulation diverged (`NaN` or `Inf` detected).
    #[error("simulation diverged: {reason}")]
    Diverged {
        /// Description of what went wrong.
        reason: String,
    },

    /// Invalid configuration.
    #[error("invalid configuration: {reason}")]
    InvalidConfig {
        /// Description of the configuration error.
        reason: String,
    },

    /// Action type mismatch.
    #[error("action type mismatch: expected {expected}, got {actual}")]
    ActionTypeMismatch {
        /// Expected action type.
        expected: String,
        /// Actual action type provided.
        actual: String,
    },

    /// Body not found in world.
    #[error("body not found: {name}")]
    BodyNotFound {
        /// Name of the missing body.
        name: String,
    },

    /// Joint not found in world.
    #[error("joint not found: {name}")]
    JointNotFound {
        /// Name of the missing joint.
        name: String,
    },

    /// Parent body required but not set.
    #[error("parent body required for joint {joint_name}")]
    ParentRequired {
        /// Name of the joint missing a parent.
        joint_name: String,
    },

    /// Invalid mass properties.
    #[error("invalid mass properties: {reason}")]
    InvalidMassProperties {
        /// Description of what's wrong.
        reason: String,
    },
}

impl SimError {
    /// Create a diverged error.
    #[must_use]
    pub fn diverged(reason: impl Into<String>) -> Self {
        Self::Diverged {
            reason: reason.into(),
        }
    }

    /// Create an invalid configuration error.
    #[must_use]
    pub fn invalid_config(reason: impl Into<String>) -> Self {
        Self::InvalidConfig {
            reason: reason.into(),
        }
    }

    /// Create an invalid mass properties error.
    #[must_use]
    pub fn invalid_mass(reason: impl Into<String>) -> Self {
        Self::InvalidMassProperties {
            reason: reason.into(),
        }
    }

    /// Check if this is a divergence error.
    #[must_use]
    pub fn is_diverged(&self) -> bool {
        matches!(self, Self::Diverged { .. })
    }

    /// Check if this is a configuration error.
    #[must_use]
    pub fn is_config_error(&self) -> bool {
        matches!(self, Self::InvalidConfig { .. })
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::approx_constant
)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = SimError::InvalidBodyId(42);
        assert!(err.to_string().contains("42"));

        let err = SimError::JointLimitViolation {
            joint_id: 1,
            value: 3.5,
            min: 0.0,
            max: 3.14,
        };
        assert!(err.to_string().contains("3.5"));

        let err = SimError::diverged("NaN in velocity");
        assert!(err.to_string().contains("NaN"));
    }

    #[test]
    fn test_error_predicates() {
        let err = SimError::diverged("test");
        assert!(err.is_diverged());
        assert!(!err.is_config_error());

        let err = SimError::invalid_config("bad value");
        assert!(err.is_config_error());
        assert!(!err.is_diverged());
    }
}
