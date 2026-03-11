//! Error types for simulation configuration.

use thiserror::Error;

/// Errors that can occur during simulation configuration.
#[derive(Debug, Error, Clone, PartialEq)]
pub enum SimError {
    /// Invalid timestep.
    #[error("invalid timestep: {0} (must be positive and finite)")]
    InvalidTimestep(f64),

    /// Invalid configuration.
    #[error("invalid configuration: {reason}")]
    InvalidConfig {
        /// Description of the configuration error.
        reason: String,
    },
}

impl SimError {
    /// Create an invalid configuration error.
    #[must_use]
    pub fn invalid_config(reason: impl Into<String>) -> Self {
        Self::InvalidConfig {
            reason: reason.into(),
        }
    }

    /// Check if this is a configuration error.
    #[must_use]
    pub fn is_config_error(&self) -> bool {
        matches!(self, Self::InvalidConfig { .. })
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn test_invalid_timestep() {
        let err = SimError::InvalidTimestep(-0.01);
        assert!(err.to_string().contains("-0.01"));
    }

    #[test]
    fn test_invalid_config() {
        let err = SimError::invalid_config("bad value");
        assert!(err.is_config_error());
        assert!(err.to_string().contains("bad value"));
    }

    #[test]
    fn test_is_config_error() {
        let config_err = SimError::invalid_config("test");
        assert!(config_err.is_config_error());

        let timestep_err = SimError::InvalidTimestep(0.0);
        assert!(!timestep_err.is_config_error());
    }
}
