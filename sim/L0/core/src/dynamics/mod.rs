//! Dynamics computations: spatial algebra, CRBA, RNE, sparse factorization.
//!
//! This module groups the core dynamics algorithms that operate on the
//! articulated rigid body tree: spatial algebra utilities, the Composite
//! Rigid Body Algorithm (mass matrix), Recursive Newton-Euler (bias forces),
//! sparse LDL factorization, and flex vertex synchronization.

pub(crate) mod crba;
pub(crate) mod factor;
pub(crate) mod flex;
pub(crate) mod rne;
pub(crate) mod spatial;

pub(crate) use crba::DEFAULT_MASS_FALLBACK;
pub use spatial::SpatialVector;
pub(crate) use spatial::{compute_body_spatial_inertia, shift_spatial_inertia};
