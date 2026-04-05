//! Error types for the sim-ml-bridge crate.

use std::ops::Range;

/// Errors arising from tensor construction or validation.
#[derive(Debug, Clone, thiserror::Error)]
pub enum TensorError {
    /// The number of data elements does not match the product of the shape.
    #[error(
        "shape mismatch: data has {data_len} elements but shape {shape:?} \
         requires {expected}"
    )]
    ShapeMismatch {
        /// Actual number of elements in the data.
        data_len: usize,
        /// Declared shape.
        shape: Vec<usize>,
        /// Expected number of elements (product of shape).
        expected: usize,
    },
}

/// Errors arising from observation/action space construction.
#[derive(Debug, Clone, thiserror::Error)]
pub enum SpaceError {
    /// An element-index range exceeds the field's length.
    #[error("{field} range {range:?} out of bounds (field length {field_len})")]
    RangeOutOfBounds {
        /// Name of the Data field (e.g. "qpos", "qvel").
        field: &'static str,
        /// The requested range.
        range: Range<usize>,
        /// Actual length of the field.
        field_len: usize,
    },

    /// A body-index range exceeds `nbody`.
    #[error("{field} body range {range:?} out of bounds (nbody = {nbody})")]
    BodyRangeOutOfBounds {
        /// Name of the per-body field (e.g. "xpos", "xquat").
        field: &'static str,
        /// The requested body range.
        range: Range<usize>,
        /// Total number of bodies.
        nbody: usize,
    },

    /// A mocap-body range exceeds `nmocap`.
    #[error("{field} mocap range {range:?} out of bounds (nmocap = {nmocap})")]
    MocapRangeOutOfBounds {
        /// Name of the mocap field (e.g. `mocap_pos`, `mocap_quat`).
        field: &'static str,
        /// The requested mocap-body range.
        range: Range<usize>,
        /// Total number of mocap bodies.
        nmocap: usize,
    },

    /// A sensor name was not found in the model.
    #[error("sensor \"{name}\" not found in model")]
    SensorNotFound {
        /// The sensor name that could not be resolved.
        name: String,
    },
}

/// Errors arising from environment builder validation.
#[derive(Debug, Clone, thiserror::Error)]
pub enum EnvError {
    /// A required field was not set on the builder.
    #[error("missing required field: {field}")]
    MissingField {
        /// Name of the missing builder field.
        field: &'static str,
    },

    /// `sub_steps` was set to zero.
    #[error("sub_steps must be >= 1, got 0")]
    ZeroSubSteps,
}

/// Bridge-level error from [`VecEnv::step()`](crate::VecEnv::step).
///
/// Only for shape/dimension mismatches — never for per-env physics errors
/// (those are reported in `VecStepResult::errors`).
#[derive(Debug, Clone, thiserror::Error)]
pub enum VecStepError {
    /// Action tensor has the wrong shape.
    #[error("action shape mismatch: expected [{n_envs}, {act_dim}], got {actual:?}")]
    ActionShapeMismatch {
        /// Expected number of environments (rows).
        n_envs: usize,
        /// Expected action dimension (columns).
        act_dim: usize,
        /// Actual shape provided.
        actual: Vec<usize>,
    },
}

/// Error from `reset()` / `reset_all()`.
///
/// Indicates that `forward()` failed after reset + `on_reset` hook —
/// the `on_reset` hook likely put `Data` into an invalid state.
#[derive(Debug, thiserror::Error)]
#[error("forward() failed after reset: {source}")]
pub struct ResetError {
    /// The underlying physics error from `forward()`.
    #[from]
    pub source: sim_core::StepError,
}
