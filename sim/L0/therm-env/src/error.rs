//! Error types for `sim-therm-env`.

use sim_ml_chassis::{EnvError, SpaceError};

/// Errors from [`ThermCircuitEnvBuilder::build`](crate::ThermCircuitEnvBuilder::build).
#[derive(Debug, thiserror::Error)]
pub enum ThermCircuitError {
    /// A required builder field was not set.
    #[error("missing required field: {field}")]
    MissingField {
        /// Name of the missing field.
        field: &'static str,
    },

    /// `n_particles` was zero.
    #[error("n_particles must be >= 1")]
    ZeroParticles,

    /// A builder f64 parameter was `NaN` or infinite.
    #[error("non-finite parameter `{field}`: {value}")]
    NonFiniteParameter {
        /// Name of the offending field.
        field: &'static str,
        /// The non-finite value that was supplied.
        value: f64,
    },

    /// Propagated from `SimEnv::builder().build()`.
    #[error(transparent)]
    Env(#[from] EnvError),

    /// Propagated from observation/action space construction.
    #[error(transparent)]
    Space(#[from] SpaceError),

    /// Propagated from `sim_mjcf::load_model`.
    #[error(transparent)]
    Mjcf(#[from] sim_mjcf::MjcfError),
}
