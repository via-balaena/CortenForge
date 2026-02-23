//! Dynamics computations: spatial algebra, CRBA, RNE, sparse factorization.
//!
//! This module groups the core dynamics algorithms that operate on the
//! articulated rigid body tree. Currently contains spatial algebra utilities;
//! CRBA, RNE, factorization, and flex sync arrive in Phase 7.

pub(crate) mod spatial;

pub use spatial::SpatialVector;
pub(crate) use spatial::{
    compute_body_spatial_inertia, shift_spatial_inertia, spatial_cross_force, spatial_cross_motion,
};
