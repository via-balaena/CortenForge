//! Tensor and `TensorSpec` — the core data types for the ML bridge.
//!
//! [`Tensor`] is a flat `f32` buffer with shape metadata. No autograd, no
//! device placement, no strides. The simplest possible tensor that knows
//! its shape.
//!
//! [`TensorSpec`] describes the shape and bounds of an observation or action
//! tensor, so that policies can be constructed with matching dimensions.

use crate::error::TensorError;

// ─── helpers ────────────────────────────────────────────────────────────────

/// Product of all dimensions.  Returns 1 for an empty shape (scalar).
fn shape_numel(shape: &[usize]) -> usize {
    shape.iter().copied().product::<usize>()
}

// ─── Tensor ─────────────────────────────────────────────────────────────────

/// A flat `f32` buffer with shape metadata.
///
/// No autograd, no device placement, no strides.  This is the simplest
/// possible tensor — a typed `Vec<f32>` that knows its shape.  We will
/// grow it later (autodiff, GPU buffers) but the interface stays stable.
#[derive(Debug, Clone, PartialEq)]
pub struct Tensor {
    data: Vec<f32>,
    shape: Vec<usize>,
}

impl Tensor {
    // ── constructors (panicking — for internal / known-good paths) ───────

    /// Create a tensor filled with zeros.
    #[must_use]
    pub fn zeros(shape: &[usize]) -> Self {
        let n = shape_numel(shape);
        Self {
            data: vec![0.0; n],
            shape: shape.to_vec(),
        }
    }

    /// Create a tensor from an `f32` slice.
    ///
    /// # Panics
    ///
    /// Panics if `data.len() != product(shape)`.  This is always a
    /// programming error — use [`try_from_slice`](Self::try_from_slice)
    /// when validating external input.
    #[must_use]
    pub fn from_slice(data: &[f32], shape: &[usize]) -> Self {
        let expected = shape_numel(shape);
        assert!(
            data.len() == expected,
            "Tensor::from_slice: data has {} elements but shape {shape:?} requires {expected}",
            data.len(),
        );
        Self {
            data: data.to_vec(),
            shape: shape.to_vec(),
        }
    }

    /// Create a tensor from an `f64` slice, casting each element to `f32`.
    ///
    /// The `f64→f32` truncation is intentional — the sim is `f64` throughout,
    /// but ML convention is `f32`.  The bridge owns this conversion.
    ///
    /// # Panics
    ///
    /// Panics if `data.len() != product(shape)`.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn from_f64_slice(data: &[f64], shape: &[usize]) -> Self {
        let expected = shape_numel(shape);
        assert!(
            data.len() == expected,
            "Tensor::from_f64_slice: data has {} elements but shape {shape:?} requires {expected}",
            data.len(),
        );
        Self {
            data: data.iter().map(|&v| v as f32).collect(),
            shape: shape.to_vec(),
        }
    }

    // ── constructors (fallible — for external / untrusted input) ─────────

    /// Try to create a tensor from an `f32` slice.
    ///
    /// # Errors
    ///
    /// Returns [`TensorError::ShapeMismatch`] if `data.len() != product(shape)`.
    pub fn try_from_slice(data: &[f32], shape: &[usize]) -> Result<Self, TensorError> {
        let expected = shape_numel(shape);
        if data.len() != expected {
            return Err(TensorError::ShapeMismatch {
                data_len: data.len(),
                shape: shape.to_vec(),
                expected,
            });
        }
        Ok(Self {
            data: data.to_vec(),
            shape: shape.to_vec(),
        })
    }

    /// Try to create a tensor from an `f64` slice, casting each element to `f32`.
    ///
    /// # Errors
    ///
    /// Returns [`TensorError::ShapeMismatch`] if `data.len() != product(shape)`.
    #[allow(clippy::cast_possible_truncation)]
    pub fn try_from_f64_slice(data: &[f64], shape: &[usize]) -> Result<Self, TensorError> {
        let expected = shape_numel(shape);
        if data.len() != expected {
            return Err(TensorError::ShapeMismatch {
                data_len: data.len(),
                shape: shape.to_vec(),
                expected,
            });
        }
        Ok(Self {
            data: data.iter().map(|&v| v as f32).collect(),
            shape: shape.to_vec(),
        })
    }

    // ── accessors ────────────────────────────────────────────────────────

    /// The shape of this tensor.
    #[must_use]
    pub fn shape(&self) -> &[usize] {
        &self.shape
    }

    /// Number of dimensions.
    #[must_use]
    pub fn ndim(&self) -> usize {
        self.shape.len()
    }

    /// Total number of elements (product of shape).
    #[must_use]
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Whether the tensor has zero elements.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// View the raw data as a slice.
    #[must_use]
    pub fn as_slice(&self) -> &[f32] {
        &self.data
    }

    /// View the raw data as a mutable slice.
    pub fn as_mut_slice(&mut self) -> &mut [f32] {
        &mut self.data
    }

    // ── batched row access ───────────────────────────────────────────────

    /// View row `i` of a 2-D tensor (shape `[N, D]`).
    ///
    /// Returns a slice of length `D`.
    ///
    /// # Panics
    ///
    /// Panics if `ndim != 2` or `i >= shape[0]`.
    #[must_use]
    pub fn row(&self, i: usize) -> &[f32] {
        assert!(
            self.shape.len() == 2,
            "Tensor::row requires ndim == 2, got {}",
            self.shape.len(),
        );
        assert!(
            i < self.shape[0],
            "Tensor::row: index {i} out of bounds for shape {:?}",
            self.shape,
        );
        let cols = self.shape[1];
        let start = i * cols;
        &self.data[start..start + cols]
    }

    /// View row `i` of a 2-D tensor (shape `[N, D]`) mutably.
    ///
    /// Returns a mutable slice of length `D`.
    ///
    /// # Panics
    ///
    /// Panics if `ndim != 2` or `i >= shape[0]`.
    pub fn row_mut(&mut self, i: usize) -> &mut [f32] {
        assert!(
            self.shape.len() == 2,
            "Tensor::row_mut requires ndim == 2, got {}",
            self.shape.len(),
        );
        assert!(
            i < self.shape[0],
            "Tensor::row_mut: index {i} out of bounds for shape {:?}",
            self.shape,
        );
        let cols = self.shape[1];
        let start = i * cols;
        &mut self.data[start..start + cols]
    }
}

// ─── TensorSpec ─────────────────────────────────────────────────────────────

/// Describes the shape and bounds of an observation or action tensor.
///
/// Used by environments to declare their observation/action spaces
/// so that policies can be constructed with matching dimensions.
#[derive(Debug, Clone)]
pub struct TensorSpec {
    /// Shape of the tensor (e.g., `[obs_dim]` or `[n_envs, obs_dim]`).
    pub shape: Vec<usize>,

    /// Per-element lower bounds.  `None` means unbounded below.
    pub low: Option<Vec<f32>>,

    /// Per-element upper bounds.  `None` means unbounded above.
    pub high: Option<Vec<f32>>,
}

// ─── tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use
)]
mod tests {
    use super::*;

    // ── shape arithmetic ─────────────────────────────────────────────────

    #[test]
    fn zeros_creates_correct_shape_and_data() {
        let t = Tensor::zeros(&[3, 4]);
        assert_eq!(t.shape(), &[3, 4]);
        assert_eq!(t.len(), 12);
        assert_eq!(t.ndim(), 2);
        assert!(!t.is_empty());
        assert!(t.as_slice().iter().all(|&v| v == 0.0));
    }

    #[test]
    fn zeros_scalar_shape() {
        let t = Tensor::zeros(&[]);
        assert_eq!(t.shape(), &[] as &[usize]);
        assert_eq!(t.len(), 1); // product of empty shape = 1
        assert_eq!(t.ndim(), 0);
    }

    #[test]
    fn zeros_with_zero_dim() {
        let t = Tensor::zeros(&[3, 0, 4]);
        assert_eq!(t.len(), 0);
        assert!(t.is_empty());
    }

    // ── from_slice ───────────────────────────────────────────────────────

    #[test]
    fn from_slice_roundtrip() {
        let data = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0];
        let t = Tensor::from_slice(&data, &[2, 3]);
        assert_eq!(t.shape(), &[2, 3]);
        assert_eq!(t.as_slice(), &data);
    }

    #[test]
    #[should_panic(expected = "shape")]
    fn from_slice_panics_on_mismatch() {
        let data = [1.0_f32, 2.0, 3.0];
        let _ = Tensor::from_slice(&data, &[2, 3]);
    }

    // ── from_f64_slice ───────────────────────────────────────────────────

    #[test]
    fn from_f64_slice_casts_correctly() {
        let data = [1.0_f64, 2.5, 3.75];
        let t = Tensor::from_f64_slice(&data, &[3]);
        assert_eq!(t.as_slice(), &[1.0_f32, 2.5, 3.75]);
    }

    #[test]
    fn from_f64_slice_precision() {
        // A value that differs at f64 vs f32 precision
        let val: f64 = 1.0 + 1e-10;
        let t = Tensor::from_f64_slice(&[val], &[1]);
        // f32 should lose the 1e-10 precision
        assert_eq!(t.as_slice()[0], val as f32);
    }

    #[test]
    #[should_panic(expected = "shape")]
    fn from_f64_slice_panics_on_mismatch() {
        let data = [1.0_f64, 2.0];
        let _ = Tensor::from_f64_slice(&data, &[3]);
    }

    // ── try_from_slice ───────────────────────────────────────────────────

    #[test]
    fn try_from_slice_ok() {
        let data = [1.0_f32, 2.0, 3.0];
        let t = Tensor::try_from_slice(&data, &[3]).expect("should succeed");
        assert_eq!(t.as_slice(), &data);
    }

    #[test]
    fn try_from_slice_err() {
        let data = [1.0_f32, 2.0, 3.0];
        let err = Tensor::try_from_slice(&data, &[2, 3]).unwrap_err();
        assert!(matches!(
            err,
            TensorError::ShapeMismatch {
                data_len: 3,
                expected: 6,
                ..
            }
        ));
    }

    // ── try_from_f64_slice ───────────────────────────────────────────────

    #[test]
    fn try_from_f64_slice_ok() {
        let data = [1.0_f64, 2.0];
        let t = Tensor::try_from_f64_slice(&data, &[2]).expect("should succeed");
        assert_eq!(t.as_slice(), &[1.0_f32, 2.0]);
    }

    #[test]
    fn try_from_f64_slice_err() {
        let data = [1.0_f64];
        let err = Tensor::try_from_f64_slice(&data, &[2]).unwrap_err();
        assert!(matches!(err, TensorError::ShapeMismatch { .. }));
    }

    // ── row / row_mut ────────────────────────────────────────────────────

    #[test]
    fn row_access() {
        let data = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0];
        let t = Tensor::from_slice(&data, &[2, 3]);
        assert_eq!(t.row(0), &[1.0, 2.0, 3.0]);
        assert_eq!(t.row(1), &[4.0, 5.0, 6.0]);
    }

    #[test]
    fn row_mut_access() {
        let data = [0.0_f32; 6];
        let mut t = Tensor::from_slice(&data, &[3, 2]);
        t.row_mut(1).copy_from_slice(&[7.0, 8.0]);
        assert_eq!(t.row(0), &[0.0, 0.0]);
        assert_eq!(t.row(1), &[7.0, 8.0]);
        assert_eq!(t.row(2), &[0.0, 0.0]);
    }

    #[test]
    #[should_panic(expected = "ndim == 2")]
    fn row_panics_on_1d() {
        let t = Tensor::zeros(&[5]);
        let _ = t.row(0);
    }

    #[test]
    #[should_panic(expected = "out of bounds")]
    fn row_panics_on_oob() {
        let t = Tensor::zeros(&[2, 3]);
        let _ = t.row(2);
    }

    #[test]
    #[should_panic(expected = "ndim == 2")]
    fn row_mut_panics_on_3d() {
        let mut t = Tensor::zeros(&[2, 3, 4]);
        let _ = t.row_mut(0);
    }

    // ── as_mut_slice ─────────────────────────────────────────────────────

    #[test]
    fn as_mut_slice_modifies_data() {
        let mut t = Tensor::zeros(&[3]);
        t.as_mut_slice()[1] = 42.0;
        assert_eq!(t.as_slice(), &[0.0, 42.0, 0.0]);
    }

    // ── clone / eq ───────────────────────────────────────────────────────

    #[test]
    fn clone_and_eq() {
        let t1 = Tensor::from_slice(&[1.0, 2.0], &[2]);
        let t2 = t1.clone();
        assert_eq!(t1, t2);
    }

    #[test]
    fn ne_different_data() {
        let t1 = Tensor::from_slice(&[1.0, 2.0], &[2]);
        let t2 = Tensor::from_slice(&[1.0, 3.0], &[2]);
        assert_ne!(t1, t2);
    }

    #[test]
    fn ne_different_shape() {
        let t1 = Tensor::from_slice(&[1.0, 2.0], &[2]);
        let t2 = Tensor::from_slice(&[1.0, 2.0], &[1, 2]);
        assert_ne!(t1, t2);
    }

    // ── TensorSpec ───────────────────────────────────────────────────────

    #[test]
    fn tensor_spec_unbounded() {
        let spec = TensorSpec {
            shape: vec![4],
            low: None,
            high: None,
        };
        assert_eq!(spec.shape, vec![4]);
        assert!(spec.low.is_none());
        assert!(spec.high.is_none());
    }

    #[test]
    fn tensor_spec_bounded() {
        let spec = TensorSpec {
            shape: vec![2],
            low: Some(vec![-1.0, -2.0]),
            high: Some(vec![1.0, 2.0]),
        };
        assert_eq!(spec.low.as_deref(), Some([-1.0_f32, -2.0].as_slice()));
        assert_eq!(spec.high.as_deref(), Some([1.0_f32, 2.0].as_slice()));
    }
}
