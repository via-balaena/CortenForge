//! Error types for the sim-ml-bridge crate.

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
