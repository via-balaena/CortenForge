//! `CpuDifferentiable` — CPU-backend `Differentiable` impl — plus the
//! `NewtonStepVjp: VjpOp` type that plugs into chassis `Tape::push_custom`.
//!
//! **Math (scope §5, Part 6 Ch 02:13–15).** At equilibrium `r(x*; θ) = 0`,
//! the implicit function theorem gives
//!
//! ```text
//!     ∂x*/∂θ  =  -A⁻¹ · ∂r/∂θ          where  A = ∂r/∂x |_(x*, θ)
//! ```
//!
//! For a scalar loss `L(x*)` with upstream cotangent `g = ∂L/∂x*`, the
//! reverse-mode VJP is
//!
//! ```text
//!     ∂L/∂θ  =  g^T · ∂x*/∂θ  =  -g^T · A⁻¹ · ∂r/∂θ
//! ```
//!
//! Two-step factor-reuse form (avoids materializing `A⁻¹`):
//!
//! 1. Solve  `A · λ = g_free`  (A is symmetric for `NeoHookean` — the same
//!    factor the forward Newton step converged with).
//! 2. Contract  `∂L/∂θ = -λ^T · (∂r/∂θ)_free`.
//!
//! **Stage-1 `∂r/∂θ`.** For the skeleton scene (scope §2 R-6), θ is a
//! length-1 scalar = +ẑ traction magnitude at `v_3`. The external force
//! is `f_ext[11] = θ`, else `0`, so
//!
//! ```text
//!     ∂f_ext/∂θ  =  e_11     →     ∂r/∂θ  =  -e_11     →     (∂r/∂θ)_free = [0, 0, -1]
//! ```
//!
//! Substituting: `grad_θ = -λ^T · [0, 0, -1] = +λ[2]`. Stage-2 (full force
//! vector) widens `∂r/∂θ` to `-I_3` on free DOFs; deferred per SQ2.
//!
//! **Factor reuse.** The `Llt<usize, f64>` stashed here comes from
//! `CpuNewtonSolver::factor_at_position` at `x_final`, not from the last
//! Newton-iter factor (which was at the pre-convergence iterate). For
//! `NeoHookean`, `A = A^T` — the same factor serves both forward-solve
//! and adjoint-solve. A future asymmetric material would need
//! `solve_transpose_in_place` here. I-3 (factor ownership) verified
//! operationally in `tests/invariant_3_factor.rs`.

use std::fmt;

use faer::linalg::solvers::SolveCore;
use faer::sparse::linalg::solvers::Llt;
use faer::{Conj, MatMut};
use sim_ml_chassis::{Tensor, autograd::VjpOp};

use super::{Differentiable, TapeNodeKey};
use crate::readout::GradientEstimate;
use crate::solver::{CpuTape, NewtonStep};

/// Number of DOFs in the skeleton system (4 vertices × 3 spatial axes).
/// Local alias — duplicated from `solver::backward_euler` to keep the
/// VJP self-contained without a cross-module constant export.
const N_DOF: usize = 12;

/// Number of free DOFs after Dirichlet condensation (only `v_3`'s xyz).
const N_FREE: usize = 3;

/// Offset of the first free DOF in the 12-vector layout — `v_3.x` at
/// DOF 9 under vertex-major + xyz-inner ordering.
const FREE_OFFSET: usize = 9;

/// CPU-backend `Differentiable` impl. Stateless at type level;
/// `NewtonStepVjp` holds primal data per tape node.
#[derive(Clone, Copy, Debug, Default)]
pub struct CpuDifferentiable;

impl Differentiable for CpuDifferentiable {
    type Tape = CpuTape;

    fn register_vjp(&mut self, _forward_key: TapeNodeKey, _vjp: Box<dyn VjpOp>) {
        unimplemented!(
            "skeleton phase 2 — BF-4 stub: chassis uses push_custom, not a key-indexed registry"
        )
    }

    fn ift_adjoint(
        &self,
        _tape: &Self::Tape,
        _step: &NewtonStep<Self::Tape>,
        _upstream: &Tensor<f64>,
    ) -> Tensor<f64> {
        unimplemented!("skeleton phase 2 — step 6 wraps tape.backward + theta-grad readback")
    }

    fn time_adjoint(
        &self,
        _tape: &Self::Tape,
        _rollout: &[NewtonStep<Self::Tape>],
        _upstream: &Tensor<f64>,
    ) -> Tensor<f64> {
        unimplemented!("skeleton phase 2 — time-adjoint is Phase E+")
    }

    fn fd_wrapper(
        &self,
        _forward: &dyn Fn(&Tensor<f64>) -> Tensor<f64>,
        _theta: &Tensor<f64>,
    ) -> (Tensor<f64>, GradientEstimate) {
        unimplemented!("skeleton phase 2 — fd_wrapper is Phase G stochastic-adjoint")
    }
}

// ── NewtonStepVjp ─────────────────────────────────────────────────────────

/// Custom VJP for one converged Newton step.
///
/// Stashes the faer `Llt` factor of `A = ∂r/∂x` at `x_final`; `vjp`
/// solves the adjoint system `A · λ = g_free` and contracts against
/// Stage-1 `∂r/∂θ` in closed form.
///
/// Stage-1 θ (length-1 scalar, +ẑ traction) → `∂r/∂θ` restricted to free
/// DOFs is `[0, 0, -1]`; the closed-form accumulation lives in `vjp`.
pub struct NewtonStepVjp {
    factor: Llt<usize, f64>,
}

impl NewtonStepVjp {
    /// Construct a VJP for one converged Newton step.
    #[must_use]
    pub const fn new(factor: Llt<usize, f64>) -> Self {
        Self { factor }
    }
}

// Llt<usize, f64> does not derive Debug; hand-roll a minimal impl so the
// VjpOp `Debug` supertrait bound is satisfied without leaking factor
// internals.
impl fmt::Debug for NewtonStepVjp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("NewtonStepVjp")
            .field("op_id", &"sim_soft::NewtonStepVjp")
            .field("factor", &"<Llt<usize, f64>>")
            .finish()
    }
}

impl VjpOp for NewtonStepVjp {
    fn op_id(&self) -> &'static str {
        "sim_soft::NewtonStepVjp"
    }

    // The hot-path VJP body. Assertions panic on shape mismatches because
    // those are programmer bugs (wrong output shape was pushed, wrong
    // parent shape registered) — not runtime-recoverable conditions.
    #[allow(clippy::panic)]
    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        assert!(
            cotangent.shape() == [N_DOF],
            "NewtonStepVjp: cotangent must have shape [{N_DOF}], got {:?}",
            cotangent.shape(),
        );
        assert!(
            parent_cotans.len() == 1,
            "NewtonStepVjp: expected 1 parent (theta_var), got {}",
            parent_cotans.len(),
        );
        assert!(
            parent_cotans[0].shape() == [1],
            "NewtonStepVjp: Stage-1 θ cotangent must have shape [1], got {:?}",
            parent_cotans[0].shape(),
        );

        // Slice the upstream cotangent to free DOFs. Pinned DOFs contribute
        // nothing to ∂L/∂θ (x_0, x_1, x_2 are Dirichlet-constant in θ).
        let cot = cotangent.as_slice();
        let mut rhs: [f64; N_FREE] = [cot[FREE_OFFSET], cot[FREE_OFFSET + 1], cot[FREE_OFFSET + 2]];

        // Solve `A · λ = g_free` in place via the stashed Llt. `A` is
        // symmetric for `NeoHookean`, so we use the same factor shape
        // the forward Newton step produced — no transpose needed today.
        // (Asymmetric materials would need `solve_transpose_in_place`.)
        let rhs_mat: MatMut<'_, f64> = MatMut::from_column_major_slice_mut(&mut rhs, N_FREE, 1);
        self.factor.solve_in_place_with_conj(Conj::No, rhs_mat);
        // `rhs` now holds λ.

        // Contract against (∂r/∂θ)_free = [0, 0, -1]:
        //   grad_θ  =  -λ^T · (∂r/∂θ)_free
        //          =  -( λ[0]·0 + λ[1]·0 + λ[2]·(-1) )
        //          =  +λ[2]
        //
        // Accumulate (`+=` per VjpOp contract), not overwrite.
        parent_cotans[0].as_mut_slice()[0] += rhs[2];
    }
}
