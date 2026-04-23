//! Tensor and `TensorSpec` — the core data types for the ML bridge.
//!
//! [`Tensor`] is a flat buffer over `T` with shape metadata. No autograd, no
//! device placement, no strides. The simplest possible tensor that knows
//! its shape.
//!
//! The `T = f32` default matches the RL contract (observations, actions,
//! policies); `Tensor<f64>` is available for sim-soft physics consumers.
//!
//! [`TensorSpec`] describes the shape and bounds of an observation or action
//! tensor, so that policies can be constructed with matching dimensions.
//! `TensorSpec` is f32-only — it is an RL-environment-declaration concept.

use std::fmt::Debug;

use num_traits::{Float, One, Zero, cast::NumCast};

use crate::error::TensorError;

// ─── helpers ────────────────────────────────────────────────────────────────

/// Product of all dimensions.  Returns 1 for an empty shape (scalar).
fn shape_numel(shape: &[usize]) -> usize {
    shape.iter().copied().product::<usize>()
}

/// Cast `f64` → `T` via `NumCast`.
///
/// # Panics
///
/// Panics if `NumCast::from(v)` returns `None`.  Unreachable for stdlib
/// `f32`/`f64`; reachable only if a custom `Float` impl has a partial
/// `NumCast` from `f64`.  Single localized `expect` escape for the whole
/// module — all f64→T casts route through here.
#[allow(clippy::expect_used)]
fn numcast_f64<T: NumCast>(v: f64) -> T {
    <T as NumCast>::from(v).expect("NumCast::from::<f64> returned None")
}

// ─── Tensor ─────────────────────────────────────────────────────────────────

/// A flat buffer over `T` with shape metadata.
///
/// No autograd, no device placement, no strides.  This is the simplest
/// possible tensor — a typed `Vec<T>` that knows its shape.  We will
/// grow it later (autodiff, GPU buffers) but the interface stays stable.
///
/// The default `T = f32` matches the RL contract; `Tensor<f64>` is used by
/// sim-soft physics code per the workspace precision split.
// Float bound precludes `Eq` — f32/f64 have NaN and do not impl `Eq`.
#[allow(clippy::derive_partial_eq_without_eq)]
#[derive(Debug, Clone, PartialEq)]
pub struct Tensor<T = f32>
where
    T: Copy + Default + PartialEq + Debug + Send + Sync + Zero + One + Float,
{
    data: Vec<T>,
    shape: Vec<usize>,
}

impl<T> Tensor<T>
where
    T: Copy + Default + PartialEq + Debug + Send + Sync + Zero + One + Float,
{
    // ── constructors (panicking — for internal / known-good paths) ───────

    /// Create a tensor filled with zeros.
    #[must_use]
    pub fn zeros(shape: &[usize]) -> Self {
        let n = shape_numel(shape);
        Self {
            data: vec![T::zero(); n],
            shape: shape.to_vec(),
        }
    }

    /// Create a tensor from a `T` slice.
    ///
    /// # Panics
    ///
    /// Panics if `data.len() != product(shape)`.  This is always a
    /// programming error — use [`try_from_slice`](Self::try_from_slice)
    /// when validating external input.
    #[must_use]
    pub fn from_slice(data: &[T], shape: &[usize]) -> Self {
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

    /// Create a tensor from an `f64` slice, casting each element to `T`.
    ///
    /// For `T = f32` this is the explicit sim/ML boundary cast (the sim is
    /// `f64` throughout, RL convention is `f32`). For `T = f64` it is an
    /// identity conversion — a trivial generic-monomorphization artifact
    /// per A.1 sub-decision.
    ///
    /// # Panics
    ///
    /// Panics if `data.len() != product(shape)`, or if `NumCast::from(v)`
    /// fails for any element (unreachable for stdlib `f32`/`f64`; reachable
    /// only if a custom `Float` impl has a partial `NumCast` from `f64`).
    #[must_use]
    pub fn from_f64_slice(data: &[f64], shape: &[usize]) -> Self {
        let expected = shape_numel(shape);
        assert!(
            data.len() == expected,
            "Tensor::from_f64_slice: data has {} elements but shape {shape:?} requires {expected}",
            data.len(),
        );
        Self {
            data: data.iter().map(|&v| numcast_f64::<T>(v)).collect(),
            shape: shape.to_vec(),
        }
    }

    // ── constructors (fallible — for external / untrusted input) ─────────

    /// Try to create a tensor from a `T` slice.
    ///
    /// # Errors
    ///
    /// Returns [`TensorError::ShapeMismatch`] if `data.len() != product(shape)`.
    pub fn try_from_slice(data: &[T], shape: &[usize]) -> Result<Self, TensorError> {
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

    /// Try to create a tensor from an `f64` slice, casting each element to `T`.
    ///
    /// # Errors
    ///
    /// Returns [`TensorError::ShapeMismatch`] if `data.len() != product(shape)`.
    ///
    /// # Panics
    ///
    /// Panics if `NumCast::from(v)` fails for any element (unreachable for
    /// stdlib `f32`/`f64`; reachable only if a custom `Float` impl has a
    /// partial `NumCast` from `f64`).
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
            data: data.iter().map(|&v| numcast_f64::<T>(v)).collect(),
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
    pub const fn ndim(&self) -> usize {
        self.shape.len()
    }

    /// Total number of elements (product of shape).
    #[must_use]
    pub const fn len(&self) -> usize {
        self.data.len()
    }

    /// Whether the tensor has zero elements.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// View the raw data as a slice.
    #[must_use]
    pub fn as_slice(&self) -> &[T] {
        &self.data
    }

    /// View the raw data as a mutable slice.
    pub fn as_mut_slice(&mut self) -> &mut [T] {
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
    pub fn row(&self, i: usize) -> &[T] {
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
    pub fn row_mut(&mut self, i: usize) -> &mut [T] {
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
    use super::{Tensor as GenericTensor, TensorError, TensorSpec};

    // These tests exercise the f32 monomorphization — the RL contract per A.1.
    // Shadows the generic import so test bodies can use bare `Tensor::...`.
    type Tensor = GenericTensor<f32>;

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
