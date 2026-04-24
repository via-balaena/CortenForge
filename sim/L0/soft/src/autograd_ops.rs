//! Small custom `VjpOp`s for sim-soft's reward-composition path.
//!
//! Chassis ships `add`, `sub`, `mul`, `neg`, `square`, `sum` elementwise —
//! all requiring matched shapes and no `div` / `recip`. Step 6's on-tape
//! reward scalar needs two sim-soft-local ops:
//!
//! - [`IndexOp`] — extract a single element of a flat `[N]` tensor as a
//!   shape-`[1]` scalar, so free-DOF picks can chain into chassis `mul` /
//!   `add` against the `[1]` θ tensor.
//! - [`DivOp`] — elementwise division on shape-`[1]` scalars, needed for
//!   `stiffness_bound = θ / x_final[11]`.
//!
//! Both stash the primal values they need for their gradient rules; both
//! emit shape `[1]` so composition with the existing chassis primitives
//! is homogeneous.
//!
//! [`IndexOp`] started life as a test fixture inside
//! `tests/invariant_4_5_gradcheck.rs` at step 5; step 6 promotes it to
//! production as the γ-locked reward composition starts consuming
//! `x_final` on-tape.

use sim_ml_chassis::{Tensor, autograd::VjpOp};

/// Extract element `idx` of a flat parent `[N]` tensor as a shape-`[1]`
/// scalar.
///
/// VJP rule: `parent_cot[idx] += cot[0]`. All other parent slots are
/// untouched.
#[derive(Debug)]
pub struct IndexOp {
    /// Index into the parent `[N]` tensor.
    pub idx: usize,
    /// Parent length — used to validate the incoming parent shape at
    /// backward time. Panics (not recoverable) on mismatch.
    pub parent_len: usize,
}

impl IndexOp {
    /// Construct an `IndexOp` that picks `idx` out of a length-`parent_len`
    /// parent tensor.
    #[must_use]
    pub const fn new(idx: usize, parent_len: usize) -> Self {
        Self { idx, parent_len }
    }
}

impl VjpOp for IndexOp {
    fn op_id(&self) -> &'static str {
        "sim_soft::IndexOp"
    }

    // Shape mismatches are programmer bugs, not runtime-recoverable state.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [1],
            "IndexOp: cotangent must be shape [1], got {:?}",
            cotangent.shape(),
        );
        assert!(
            parent_cotans.len() == 1,
            "IndexOp: expected 1 parent, got {}",
            parent_cotans.len(),
        );
        assert!(
            parent_cotans[0].shape() == [self.parent_len],
            "IndexOp: parent must be shape [{}], got {:?}",
            self.parent_len,
            parent_cotans[0].shape(),
        );
        assert!(
            self.idx < self.parent_len,
            "IndexOp: idx {} out of bounds for parent length {}",
            self.idx,
            self.parent_len,
        );

        let scalar_cot = cotangent.as_slice()[0];
        parent_cotans[0].as_mut_slice()[self.idx] += scalar_cot;
    }
}

/// Elementwise scalar division `z = a / b` on shape-`[1]` tensors.
///
/// VJP rule for `z = a / b`:
///
/// ```text
///     dz/da = 1 / b      →     cot_a += cot · (1 / b)
///     dz/db = -a / b²    →     cot_b += cot · (-a / b²)
/// ```
///
/// Stashes the primal `a` and `b` at forward time; `b` must be non-zero
/// both at forward (value) and backward (division by zero is a
/// programmer bug here, not a recoverable condition).
#[derive(Clone, Copy, Debug)]
pub struct DivOp {
    a_val: f64,
    b_val: f64,
}

impl DivOp {
    /// Construct a `DivOp` with the stashed primal values from the
    /// forward pass.
    #[must_use]
    pub const fn new(a_val: f64, b_val: f64) -> Self {
        Self { a_val, b_val }
    }
}

impl VjpOp for DivOp {
    fn op_id(&self) -> &'static str {
        "sim_soft::DivOp"
    }

    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [1],
            "DivOp: cotangent must be shape [1], got {:?}",
            cotangent.shape(),
        );
        assert!(
            parent_cotans.len() == 2,
            "DivOp: expected 2 parents (a, b), got {}",
            parent_cotans.len(),
        );
        assert!(
            parent_cotans[0].shape() == [1] && parent_cotans[1].shape() == [1],
            "DivOp: parent shapes must be [1], got {:?} and {:?}",
            parent_cotans[0].shape(),
            parent_cotans[1].shape(),
        );
        assert!(
            self.b_val != 0.0,
            "DivOp: stashed b_val is zero — forward should have panicked",
        );

        let cot = cotangent.as_slice()[0];
        let inv_b = 1.0 / self.b_val;
        parent_cotans[0].as_mut_slice()[0] += cot * inv_b;
        parent_cotans[1].as_mut_slice()[0] += cot * (-self.a_val * inv_b * inv_b);
    }
}

#[cfg(test)]
mod tests {
    // Math-style pair names (`a_var`/`b_var`, `fd_da`/`fd_db`) mirror the
    // two-argument VJP rule and the two-component FD; renaming for
    // uniqueness would obscure the math. Matches the precedent set by
    // `tests/invariant_4_5_gradcheck.rs`.
    #![allow(clippy::similar_names)]

    use sim_ml_chassis::{Tape, Tensor};

    use super::*;

    // ── IndexOp ──────────────────────────────────────────────────────────

    #[test]
    fn index_op_forward_and_backward_single_element() {
        // L = parent[2] where parent = [1.0, 2.0, 3.0, 4.0]
        let mut tape = Tape::new();
        let parent = tape.param_tensor(Tensor::from_slice(&[1.0, 2.0, 3.0, 4.0], &[4]));

        let val = Tensor::from_slice(&[3.0], &[1]);
        let l = tape.push_custom(&[parent], val, Box::new(IndexOp::new(2, 4)));

        tape.backward(l);
        let grad = tape.grad_tensor(parent);
        assert_eq!(grad.as_slice(), &[0.0, 0.0, 1.0, 0.0]);
    }

    #[test]
    fn index_op_accumulates_into_nonzero_parent_cotan() {
        // Two IndexOp children of one parent: L = parent[0] + parent[2].
        // Backward must see grad = [1, 0, 1, 0], i.e. accumulated not
        // overwritten.
        let mut tape = Tape::new();
        let parent = tape.param_tensor(Tensor::from_slice(&[5.0, 6.0, 7.0, 8.0], &[4]));

        let l0 = tape.push_custom(
            &[parent],
            Tensor::from_slice(&[5.0], &[1]),
            Box::new(IndexOp::new(0, 4)),
        );
        let l2 = tape.push_custom(
            &[parent],
            Tensor::from_slice(&[7.0], &[1]),
            Box::new(IndexOp::new(2, 4)),
        );
        let sum = tape.add(l0, l2);

        tape.backward(sum);
        assert_eq!(tape.grad_tensor(parent).as_slice(), &[1.0, 0.0, 1.0, 0.0]);
    }

    // ── DivOp ────────────────────────────────────────────────────────────

    #[test]
    fn div_op_forward_and_backward_matches_closed_form() {
        // z = a / b at (a=6, b=2) → z=3; dz/da = 1/b = 0.5, dz/db = -a/b² = -1.5
        let mut tape = Tape::new();
        let a = tape.param_tensor(Tensor::from_slice(&[6.0], &[1]));
        let b = tape.param_tensor(Tensor::from_slice(&[2.0], &[1]));

        let z_val = 6.0 / 2.0;
        let z = tape.push_custom(
            &[a, b],
            Tensor::from_slice(&[z_val], &[1]),
            Box::new(DivOp::new(6.0, 2.0)),
        );

        tape.backward(z);
        assert!((tape.grad_tensor(a).as_slice()[0] - 0.5).abs() < 1e-12);
        assert!((tape.grad_tensor(b).as_slice()[0] - (-1.5)).abs() < 1e-12);
    }

    #[test]
    fn div_op_gradcheck_via_central_fd() {
        // Central FD against analytic DivOp vjp at (a=7, b=3).
        const H: f64 = 1e-6;
        let forward = |a: f64, b: f64| a / b;

        let a0 = 7.0;
        let b0 = 3.0;

        let mut tape = Tape::new();
        let a_var = tape.param_tensor(Tensor::from_slice(&[a0], &[1]));
        let b_var = tape.param_tensor(Tensor::from_slice(&[b0], &[1]));
        let z = tape.push_custom(
            &[a_var, b_var],
            Tensor::from_slice(&[forward(a0, b0)], &[1]),
            Box::new(DivOp::new(a0, b0)),
        );
        tape.backward(z);
        let da = tape.grad_tensor(a_var).as_slice()[0];
        let db = tape.grad_tensor(b_var).as_slice()[0];

        let fd_da = (forward(a0 + H, b0) - forward(a0 - H, b0)) / (2.0 * H);
        let fd_db = (forward(a0, b0 + H) - forward(a0, b0 - H)) / (2.0 * H);

        assert!(
            (da - fd_da).abs() / fd_da.abs() < 1e-6,
            "da: {da} vs {fd_da}"
        );
        assert!(
            (db - fd_db).abs() / fd_db.abs() < 1e-6,
            "db: {db} vs {fd_db}"
        );
    }
}
