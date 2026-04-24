//! `SkeletonForwardMap` — concrete γ-locked `ForwardMap` for the 1-tet scene.
//!
//! Threads the three seven-trait pieces [`Solver`], [`Observable`], and
//! the reward-composition path together to satisfy invariant I-2 (a
//! populated `RewardBreakdown` + `EditResult::ParameterOnly` from one
//! forward pass) and I-4 (a full-stack gradcheck through
//! `Observable::reward_breakdown` + `RewardWeights::score_with` +
//! `tape.backward`).
//!
//! ## Autodiff chain
//!
//! ```text
//!     theta (Tensor<f64>, shape [1])
//!       → tape.param_tensor             → theta_var
//!       → solver.step                   → x_final_var (via push_custom / NewtonStepVjp)
//!       → build_reward_on_tape          → reward_var (IndexOp + DivOp + mul + add)
//!       → tape.backward(reward_var)     → cotangents populated
//!     gradient() reads tape.grad_tensor(theta_var)
//! ```
//!
//! ## Evaluate/gradient split
//!
//! The γ-locked `ForwardMap` trait puts the forward pass in `evaluate`
//! and the gradient readback in `gradient`. `gradient`'s `tape: &Tape`
//! is immutable, so it cannot drive `tape.backward`. Consequently
//! `evaluate` does both forward + backward, stashes `theta_var` on the
//! struct, and `gradient` only reads `tape.grad_tensor(theta_var)`.
//! Caller contract: pass the same tape to `gradient` that `evaluate`
//! wrote to, and call `evaluate` before `gradient`.
//!
//! ## Scope-§14 deltas
//!
//! Spec §14's composition sketch omits `weights` (needed to build the
//! on-tape reward) and `stashed_theta_var` (needed for the
//! evaluate/gradient split above). Also logs BF-6: `Differentiable::
//! ift_adjoint`'s `&Tape` + manual-upstream signature doesn't fit the
//! chassis `push_custom` + `tape.backward(scalar)` model (same root
//! cause as BF-4 `register_vjp`); this impl drives `tape.backward`
//! directly inside `evaluate` instead of routing through
//! `ift_adjoint`, which stays stubbed.

// `theta_val`/`theta_var`, `peak_val`/`peak_var`, `w_peak`/`term_peak`
// etc. are meaningful math-mirroring distinctions in the reward-
// composition path (f64 primal vs tape `Var`, weight vs weighted term);
// suppressing here matches the precedent set by
// `tests/invariant_4_5_gradcheck.rs`.
#![allow(clippy::similar_names)]

use sim_ml_chassis::{Tape, Tensor, Var};

use super::{
    EditResult, ForwardMap, GradientEstimate, RewardBreakdown, RewardWeights, SceneInitial,
};
use crate::autograd_ops::{DivOp, IndexOp};
use crate::observable::{BasicObservable, Observable};
use crate::solver::{CpuTape, Solver};

/// Number of DOFs in the skeleton 1-tet scene — `x_final` shape.
const N_DOF: usize = 12;

/// Free-DOF index for `v_3.z`; the only DOF that moves on the skeleton
/// scene and the one `BasicObservable` reports as `peak_bound`.
const FREE_DOF_Z: usize = 11;

/// γ-locked `ForwardMap` impl for the 1-tet skeleton scene.
///
/// Owns the dyn-boxed `Solver`, the `BasicObservable`, the scene initial
/// state, the reward-composition `weights`, and the stashed `theta_var`
/// from the most recent `evaluate` call.
pub struct SkeletonForwardMap {
    solver: Box<dyn Solver<Tape = CpuTape>>,
    observable: BasicObservable,
    initial: SceneInitial,
    weights: RewardWeights,
    stashed_theta_var: Option<Var>,
}

impl SkeletonForwardMap {
    /// Assemble a `SkeletonForwardMap` from its solver, observable,
    /// initial-state bundle, and reward-composition weights.
    ///
    /// The observable is usually `BasicObservable` (the only concrete
    /// impl). Weights must zero the NaN-sentinel fields
    /// (`pressure_uniformity`, `coverage`) to stay semantically
    /// consistent — the NaN-skip in both `score_with` and
    /// `build_reward_on_tape` makes non-zero weights there inert, but
    /// zeroing them is the contract.
    #[must_use]
    pub const fn new(
        solver: Box<dyn Solver<Tape = CpuTape>>,
        observable: BasicObservable,
        initial: SceneInitial,
        weights: RewardWeights,
    ) -> Self {
        Self {
            solver,
            observable,
            initial,
            weights,
            stashed_theta_var: None,
        }
    }

    /// Mirror of `RewardBreakdown::score_with` built on the tape so
    /// `tape.backward` has a differentiable scalar root.
    ///
    /// Hardcoded to the skeleton scene's two finite fields: `peak_bound
    /// = x_final[11]` (`IndexOp`) and `stiffness_bound = θ / x_final[11]`
    /// (`DivOp`). `pressure_uniformity` + `coverage` are `NaN` on this
    /// scene (scope §2) — the same `NaN`-skip branch that `score_with`
    /// takes; they have no on-tape expression here.
    ///
    /// Returns the shape-`[1]` reward `Var` ready for `tape.backward`.
    fn build_reward_on_tape(
        &self,
        tape: &mut Tape,
        x_final_var: Var,
        theta_var: Var,
        reward_breakdown: &RewardBreakdown,
    ) -> Var {
        // peak_bound = x_final[FREE_DOF_Z]
        let peak_val = reward_breakdown.peak_bound;
        let peak_var = tape.push_custom(
            &[x_final_var],
            Tensor::from_slice(&[peak_val], &[1]),
            Box::new(IndexOp::new(FREE_DOF_Z, N_DOF)),
        );

        // stiffness_bound = θ[0] / peak_bound
        // DivOp stashes (a_val, b_val) = (θ[0], peak_bound). Both primal
        // values are already known off-tape — snapshot them here rather
        // than re-reading via tape.value_tensor, since reward_breakdown
        // is the authoritative primal at this point.
        let theta_val = tape.value_tensor(theta_var).as_slice()[0];
        let stiff_val = reward_breakdown.stiffness_bound;
        let stiff_var = tape.push_custom(
            &[theta_var, peak_var],
            Tensor::from_slice(&[stiff_val], &[1]),
            Box::new(DivOp::new(theta_val, peak_val)),
        );

        // reward = w_peak · peak_var + w_stiff · stiff_var
        let w_peak = tape.constant_tensor(Tensor::from_slice(&[self.weights.peak_bound], &[1]));
        let term_peak = tape.mul(w_peak, peak_var);

        let w_stiff =
            tape.constant_tensor(Tensor::from_slice(&[self.weights.stiffness_bound], &[1]));
        let term_stiff = tape.mul(w_stiff, stiff_var);

        tape.add(term_peak, term_stiff)
    }
}

impl ForwardMap for SkeletonForwardMap {
    #[allow(clippy::expect_used)]
    fn evaluate(&mut self, theta: &Tensor<f64>, tape: &mut Tape) -> (RewardBreakdown, EditResult) {
        let theta_var = tape.param_tensor(theta.clone());
        // Borrow-checker: dt and step can't hold &self.solver simultaneously,
        // so snapshot dt first.
        let dt = self.solver.current_dt();
        let step = self.solver.step(
            tape,
            &self.initial.x_prev,
            &self.initial.v_prev,
            theta_var,
            dt,
        );

        let reward_breakdown = self.observable.reward_breakdown(&step, theta);

        let x_final_var = step
            .x_final_var
            .expect("Solver::step must populate x_final_var via push_custom");

        let reward_var = self.build_reward_on_tape(tape, x_final_var, theta_var, &reward_breakdown);

        tape.backward(reward_var);
        self.stashed_theta_var = Some(theta_var);

        (reward_breakdown, EditResult::ParameterOnly)
    }

    #[allow(clippy::expect_used)]
    fn gradient(&mut self, _theta: &Tensor<f64>, tape: &Tape) -> (Tensor<f64>, GradientEstimate) {
        let theta_var = self
            .stashed_theta_var
            .expect("SkeletonForwardMap::gradient called before evaluate");
        (tape.grad_tensor(theta_var).clone(), GradientEstimate::Exact)
    }
}
