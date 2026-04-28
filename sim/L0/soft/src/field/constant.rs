//! `ConstantField<T>` — same value at every reference-space point.
//!
//! The degenerate-uniform [`Field`] implementation. A `MaterialField`
//! whose every per-parameter slot holds a `ConstantField` describes a
//! uniform material — the Phase 2/3 default. Bit-equality of pre-Phase-4
//! invariant tests through this path is the IV-1 regression-net gate
//! (scope memo §1).
//!
//! `T: Clone` is the minimal bound: [`Field::sample`] must return `T` by
//! value, so the stored value is cloned per call. For Phase 4's `T = f64`
//! the clone is a register copy; for Phase H `T = Vec3` it is a 3-word
//! copy. No allocation either way.

use super::Field;
use crate::Vec3;

/// Field that returns the same value at every reference-space point.
///
/// Construct via [`ConstantField::new`]. A `ConstantField<f64>` produced
/// by `MaterialField::uniform(mu, lambda)` is the Phase 4 default for
/// every existing scene; Phase 2/3 invariant tests exercise this path
/// unchanged per scope memo Decision P.
#[derive(Clone, Debug)]
pub struct ConstantField<T> {
    value: T,
}

impl<T> ConstantField<T> {
    /// Construct a field whose [`Field::sample`] returns `value` at
    /// every input.
    #[must_use]
    pub const fn new(value: T) -> Self {
        Self { value }
    }
}

impl<T> Field<T> for ConstantField<T>
where
    T: Clone + Send + Sync,
{
    fn sample(&self, _x_ref: Vec3) -> T {
        self.value.clone()
    }
}
