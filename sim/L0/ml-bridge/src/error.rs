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
