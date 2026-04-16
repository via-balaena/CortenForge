//! [`ThermCircuitEnv`] — typed wrapper around [`SimEnv`] for thermodynamic circuits.

use std::fmt;

use sim_core::{Data, Model, StepError};
use sim_ml_chassis::{
    ActionSpace, Environment, ObservationSpace, ResetError, SimEnv, StepResult, Tensor,
};

use crate::builder::ThermCircuitEnvBuilder;

/// A thermodynamic-circuit environment implementing [`Environment`].
///
/// Wraps a [`SimEnv`] with domain-specific accessors and metadata.
/// Construct via [`ThermCircuitEnv::builder`].
pub struct ThermCircuitEnv {
    pub(crate) inner: SimEnv,
    pub(crate) n_particles: usize,
    pub(crate) k_b_t: f64,
    pub(crate) ctrl_temperature_idx: Option<usize>,
}

impl fmt::Debug for ThermCircuitEnv {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ThermCircuitEnv")
            .field("n_particles", &self.n_particles)
            .field("k_b_t", &self.k_b_t)
            .field("ctrl_temperature_idx", &self.ctrl_temperature_idx)
            .field("inner", &self.inner)
            .finish()
    }
}

impl ThermCircuitEnv {
    /// Start building a `ThermCircuitEnv` for `n_particles` particles.
    #[must_use]
    pub fn builder(n_particles: usize) -> ThermCircuitEnvBuilder {
        ThermCircuitEnvBuilder::new(n_particles)
    }

    /// Number of particles in the circuit.
    #[must_use]
    pub const fn n_particles(&self) -> usize {
        self.n_particles
    }

    /// Configured base bath temperature `k_B·T`.
    #[must_use]
    pub const fn config_k_b_t(&self) -> f64 {
        self.k_b_t
    }

    /// Effective temperature: `k_B·T * ctrl_multiplier` if ctrl-temperature
    /// is enabled, otherwise just `k_B·T`.
    #[must_use]
    pub fn effective_temperature(&self) -> f64 {
        self.ctrl_temperature_idx.map_or(self.k_b_t, |idx| {
            self.k_b_t * self.inner.data().ctrl[idx].clamp(0.0, 10.0)
        })
    }

    /// Immutable access to the inner `SimEnv`.
    #[must_use]
    pub const fn inner(&self) -> &SimEnv {
        &self.inner
    }

    /// Mutable access to the inner `SimEnv`.
    pub const fn inner_mut(&mut self) -> &mut SimEnv {
        &mut self.inner
    }
}

impl Environment for ThermCircuitEnv {
    fn observation_space(&self) -> &ObservationSpace {
        self.inner.observation_space()
    }

    fn action_space(&self) -> &ActionSpace {
        self.inner.action_space()
    }

    fn observe(&self) -> Tensor {
        self.inner.observe()
    }

    fn step(&mut self, action: &Tensor) -> Result<StepResult, StepError> {
        self.inner.step(action)
    }

    fn reset(&mut self) -> Result<Tensor, ResetError> {
        self.inner.reset()
    }

    fn model(&self) -> &Model {
        self.inner.model()
    }

    fn data(&self) -> &Data {
        self.inner.data()
    }
}
