//! Step-6 full-stack gradcheck — `SkeletonForwardMap` composition.
//!
//! Invariant I-4 (composition). `NewtonStepVjp` in isolation is already
//! covered by `invariant_4_5_gradcheck.rs` via a direct `tape.backward`
//! off a single `IndexOp` scalar. This file exercises the γ-locked
//! `ForwardMap::{evaluate, gradient}` path end-to-end:
//!
//! ```text
//!     theta → tape.param_tensor → theta_var
//!           → solver.step          → x_final_var (NewtonStepVjp)
//!           → build_reward_on_tape → reward_var  (IndexOp + DivOp + mul + add)
//!           → tape.backward        → grad_tensor(theta_var)
//! ```
//!
//! Complements — does not replace — step-5's `stage_1_gradcheck_central_fd`.
//! Step-5 catches `NewtonStepVjp` bugs in isolation; step-6 catches
//! composition bugs in `Observable::reward_breakdown`, on-tape reward
//! assembly, and the evaluate/gradient split.

#![allow(
    // Test asserts that evaluate was called before gradient via `.expect`
    // (the stashed_theta_var contract); a `None` here is a caller-order
    // violation, not a runtime input condition.
    clippy::expect_used,
    // `theta_val` (f64) vs `theta` (Tensor) is a meaningful distinction
    // in the gradcheck; the similar naming mirrors how they appear
    // together in the math.
    clippy::similar_names
)]

use sim_ml_chassis::{Tape, Tensor};
use sim_soft::{
    BasicObservable, CpuNewtonSolver, CpuTape, ForwardMap, GradientEstimate, NeoHookean,
    NullContact, RewardWeights, SkeletonForwardMap, SoftScene, Solver, SolverConfig, Tet4,
};

// ── Shared fixtures ──────────────────────────────────────────────────────

/// Reward weights used in the gradcheck and determinism tests.
///
/// `w_peak = 1` keeps the gradient in the same ballpark as step-5's
/// ~9.59e-5; `w_stiff = 1e-6` adds a measurable `DivOp`-chain
/// contribution without swamping the peak term (near-cancellation from
/// `d(θ/x)/dθ` under near-linear response is second-order). `NaN`
/// fields (`pressure_uniformity`, `coverage`) are zero-weighted as
/// scope §2 prescribes.
const fn gradcheck_weights() -> RewardWeights {
    RewardWeights {
        pressure_uniformity: 0.0,
        coverage: 0.0,
        peak_bound: 1.0,
        stiffness_bound: 1.0e-6,
    }
}

/// Build a fresh `SkeletonForwardMap` against the canonical 1-tet scene.
/// A fresh instance per call keeps `stashed_theta_var` clean and lets
/// the FD loop reuse the same builder without state leakage.
fn build_forward_map() -> SkeletonForwardMap {
    let cfg = SolverConfig::skeleton();
    let (mesh, initial) = SoftScene::one_tet_cube();
    let solver: Box<dyn Solver<Tape = CpuTape>> = Box::new(CpuNewtonSolver::new(
        NeoHookean::from_lame(1e5, 4e5),
        Tet4,
        mesh,
        NullContact,
        cfg,
    ));
    SkeletonForwardMap::new(solver, BasicObservable, initial, gradcheck_weights())
}

/// Forward + backward, returning the analytic gradient of θ plus the
/// primal reward scalar for determinism checks.
fn evaluate_with_gradient(theta_val: f64) -> (f64, f64) {
    let mut fm = build_forward_map();
    let mut tape = Tape::new();
    let theta = Tensor::from_slice(&[theta_val], &[1]);

    let (rb, _edit) = fm.evaluate(&theta, &mut tape);
    let reward_scalar = rb.score_with(&gradcheck_weights());

    let (grad, estimate) = fm.gradient(&theta, &tape);
    assert!(matches!(estimate, GradientEstimate::Exact));
    assert!(grad.shape() == [1]);

    (grad.as_slice()[0], reward_scalar)
}

/// Primal-only reward (forward pass + `score_with`, skip gradient).
/// Used inside the FD loop. Reuses `evaluate` (which drives
/// `tape.backward` internally); the wasted backward is fine for a
/// handful of FD probes.
fn reward_scalar(theta_val: f64) -> f64 {
    let mut fm = build_forward_map();
    let mut tape = Tape::new();
    let theta = Tensor::from_slice(&[theta_val], &[1]);
    let (rb, _edit) = fm.evaluate(&theta, &mut tape);
    rb.score_with(&gradcheck_weights())
}

// ── Full-stack gradcheck ─────────────────────────────────────────────────

#[test]
fn forward_map_full_stack_gradcheck() {
    const THETA_0: f64 = 10.0;
    // Scope §2 h = √ε for f64 — central FD sweet spot for smooth functions.
    const H: f64 = 1.5e-8;
    // Scope §6 5-digit relative-error bar.
    const REL_ERR_BOUND: f64 = 1e-5;

    let (analytic_grad, reward_at_theta) = evaluate_with_gradient(THETA_0);
    assert!(
        reward_at_theta.is_finite(),
        "reward scalar must be finite, got {reward_at_theta:e}"
    );

    let l_plus = reward_scalar(THETA_0 + H);
    let l_minus = reward_scalar(THETA_0 - H);
    let fd_grad = (l_plus - l_minus) / (2.0 * H);

    let denom = fd_grad.abs().max(1e-12);
    let rel_err = (analytic_grad - fd_grad).abs() / denom;

    assert!(
        rel_err <= REL_ERR_BOUND,
        "Full-stack gradcheck failed: analytic = {analytic_grad:.6e}, \
         FD = {fd_grad:.6e}, rel_err = {rel_err:.3e} > {REL_ERR_BOUND:.0e}. \
         Diagnose in this order (prompt): (1) build_reward_on_tape formula \
         mismatch with score_with primal, (2) DivOp a_val/b_val stash \
         ordering, (3) IndexOp shape-[1] output, (4) NewtonStepVjp \
         regression (step-5 test should catch this independently), \
         (5) RewardBreakdown NaN-skip asymmetry between primal and tape.",
    );

    // Sanity band around step-5's ~9.59e-5 (peak term dominates at these
    // weights; the stiff term's second-order contribution is ~1e-6
    // under near-linear NH response at θ=10 N).
    assert!(
        (5.0e-5..=5.0e-4).contains(&analytic_grad),
        "Analytic grad {analytic_grad:.3e} outside sanity band [5e-5, 5e-4] \
         with weights w_peak=1, w_stiff=1e-6 at θ=10 N — check observable \
         or on-tape reward composition."
    );
}

// ── Determinism across the full forward+backward stack ───────────────────

#[test]
fn forward_map_determinism() {
    const THETA_0: f64 = 10.0;

    let (grad_a, reward_a) = evaluate_with_gradient(THETA_0);
    let (grad_b, reward_b) = evaluate_with_gradient(THETA_0);

    assert_eq!(
        grad_a.to_bits(),
        grad_b.to_bits(),
        "grad_θ not bit-equal across full-stack runs: {grad_a:e} vs {grad_b:e}"
    );
    assert_eq!(
        reward_a.to_bits(),
        reward_b.to_bits(),
        "reward scalar not bit-equal: {reward_a:e} vs {reward_b:e}"
    );
}

// ── NaN-sentinel contract on the skeleton scene ──────────────────────────

#[test]
fn reward_breakdown_1tet_nan_sentinels() {
    // BasicObservable populates peak_bound + stiffness_bound as finite
    // and leaves pressure_uniformity + coverage as NaN per scope §2.
    // Full-stack verification that score_with's NaN-skip handles the
    // sentinel without poisoning the reward scalar.
    const THETA_0: f64 = 10.0;
    let mut fm = build_forward_map();
    let mut tape = Tape::new();
    let theta = Tensor::from_slice(&[THETA_0], &[1]);
    let (rb, edit) = fm.evaluate(&theta, &mut tape);

    assert_eq!(edit, sim_soft::EditResult::ParameterOnly);
    assert!(
        rb.pressure_uniformity.is_nan(),
        "pressure_uniformity must be NaN on 1-tet, got {}",
        rb.pressure_uniformity
    );
    assert!(
        rb.coverage.is_nan(),
        "coverage must be NaN on 1-tet, got {}",
        rb.coverage
    );
    assert!(
        rb.peak_bound.is_finite(),
        "peak_bound must be finite, got {}",
        rb.peak_bound
    );
    assert!(
        rb.stiffness_bound.is_finite(),
        "stiffness_bound must be finite, got {}",
        rb.stiffness_bound
    );

    // NaN + finite weights must not poison the primal scalar. Use
    // intentionally non-zero NaN-field weights to prove the is_nan
    // branch drops them (IEEE 754: NaN × 0 = NaN, so multiply-by-zero
    // alone wouldn't rescue).
    let poison_weights = RewardWeights {
        pressure_uniformity: 42.0,
        coverage: 42.0,
        peak_bound: 1.0,
        stiffness_bound: 0.0,
    };
    let scalar = rb.score_with(&poison_weights);
    assert!(
        scalar.is_finite(),
        "score_with must silently drop NaN fields, got {scalar}"
    );
    // With only w_peak=1 effective, scalar must equal peak_bound.
    assert_eq!(
        scalar.to_bits(),
        rb.peak_bound.to_bits(),
        "NaN-skip score_with must equal w_peak · peak_bound"
    );
}
