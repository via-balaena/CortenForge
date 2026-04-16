//! `ThermCircuitEnv`: typed builder producing `SimEnv`/`VecEnv` from
//! thermodynamic circuit descriptions.
//!
//! [`ThermCircuitEnv::builder`] encapsulates the boilerplate that every
//! thermodynamic-computing experiment repeats: MJCF generation,
//! thermostat construction, passive-stack wiring, and obs/act space
//! setup.  All 7 biological navigation experiments use this builder,
//! configured with different passive components and reward functions.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

pub mod builder;
pub mod env;
pub mod error;

pub use builder::{ThermCircuitEnvBuilder, generate_mjcf};
pub use env::ThermCircuitEnv;
pub use error::ThermCircuitError;
