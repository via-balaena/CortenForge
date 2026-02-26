//! Core type definitions for the MuJoCo-aligned physics pipeline.
//!
//! This module contains the fundamental types that flow through every stage of
//! the simulation: enums, Model, Data, contacts, and keyframes.

pub mod callbacks;
pub(crate) mod contact_types;
pub(crate) mod data;
pub(crate) mod enums;
pub mod flags;
pub(crate) mod keyframe;
pub(crate) mod model;
pub(crate) mod model_factories;
pub(crate) mod model_init;
pub mod validation;
pub mod warning;

pub(crate) use contact_types::compute_tangent_frame;
pub use contact_types::{Contact, ContactPair};
pub use data::*;
pub use enums::*;
pub use flags::{actuator_disabled, disabled, enabled};
pub use keyframe::Keyframe;
pub use model::*;
pub use model_init::compute_dof_lengths;
pub use validation::{MAX_VAL, MIN_VAL, is_bad};
pub use warning::{NUM_WARNINGS, Warning, WarningStat, mj_warning};
