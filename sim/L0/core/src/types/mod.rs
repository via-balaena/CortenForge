//! Core type definitions for the MuJoCo-aligned physics pipeline.
//!
//! This module contains the fundamental types that flow through every stage of
//! the simulation: enums, Model, Data, contacts, and keyframes.

pub(crate) mod contact_types;
pub(crate) mod data;
pub(crate) mod enums;
pub(crate) mod keyframe;
pub(crate) mod model;
pub(crate) mod model_factories;
pub(crate) mod model_init;

pub use contact_types::{Contact, ContactPair};
pub use data::*;
pub use enums::*;
pub use keyframe::Keyframe;
pub use model::*;
pub use model_init::compute_dof_lengths;
