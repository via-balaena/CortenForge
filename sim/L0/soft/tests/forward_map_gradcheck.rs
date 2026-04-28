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
    BasicObservable, BoundaryConditions, CpuNewtonSolver, CpuTape, ForwardMap, GradientEstimate,
    HandBuiltTetMesh, LoadAxis, MaterialField, Mesh, NeoHookean, NullContact, RewardWeights,
    SceneInitial, SkeletonForwardMap, SoftScene, Solver, SolverConfig, Tet4,
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
    let (mesh, bc, initial) = SoftScene::one_tet_cube();
    let solver: Box<dyn Solver<Tape = CpuTape>> = Box::new(CpuNewtonSolver::new(
        NeoHookean::from_lame(1e5, 4e5),
        Tet4,
        mesh,
        NullContact,
        cfg,
        bc,
    ));
    // 1-tet skeleton: peak_bound = x_final[11] (single DOF). Single
    // entry chain reduces to one IndexOp, no tape.add — bit-equal to
    // pre-Phase-2-commit-9 form.
    let observable = BasicObservable::new(vec![11]);
    SkeletonForwardMap::new(solver, observable, initial, gradcheck_weights())
}

/// Build a `SkeletonForwardMap` against the 2-isolated-tet scene
/// (Phase 2 commit 9 multi-vertex full-stack gradcheck — gate per
/// scope §8 commit 9).
fn build_forward_map_two_isolated() -> SkeletonForwardMap {
    let cfg = SolverConfig::skeleton();
    let mesh = HandBuiltTetMesh::two_isolated_tets(&MaterialField::uniform(1.0e5, 4.0e5));

    let positions = mesh.positions();
    let n_dof = 3 * positions.len();
    let mut x_prev_flat = vec![0.0; n_dof];
    for (v, pos) in positions.iter().enumerate() {
        x_prev_flat[3 * v] = pos.x;
        x_prev_flat[3 * v + 1] = pos.y;
        x_prev_flat[3 * v + 2] = pos.z;
    }
    let initial = SceneInitial {
        x_prev: Tensor::from_slice(&x_prev_flat, &[n_dof]),
        v_prev: Tensor::zeros(&[n_dof]),
    };
    let bc = BoundaryConditions {
        pinned_vertices: vec![0, 1, 2, 4, 5, 6],
        loaded_vertices: vec![(3, LoadAxis::AxisZ), (7, LoadAxis::AxisZ)],
    };
    let solver: Box<dyn Solver<Tape = CpuTape>> = Box::new(CpuNewtonSolver::new(
        NeoHookean::from_lame(1e5, 4e5),
        Tet4,
        mesh,
        NullContact,
        cfg,
        bc,
    ));
    // 2-isolated-tet: peak_bound = x_final[11] + x_final[23] (sum of
    // each tet's v_3-equivalent z-DOF). Multi-DOF chain exercises
    // commit 9's IndexOp + tape.add composition path.
    let observable = BasicObservable::new(vec![11, 23]);
    SkeletonForwardMap::new(solver, observable, initial, gradcheck_weights())
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

// ── Phase 2 commit 9: multi-vertex full-stack gradcheck ──────────────────

/// 2-isolated-tet full-stack gradcheck per scope §8 commit 9 gate.
/// Exercises `BasicObservable::new(vec![11, 23])` summing both
/// tets' free-vertex z-DOFs into `peak_bound`, threaded through
/// `SkeletonForwardMap::build_reward_on_tape`'s chained `IndexOp` +
/// `tape.add` composition. Backward through `tape.backward(reward_var)`
/// then `NewtonStepVjp`'s Stage-1 multi-loaded-vertex sum branch.
fn evaluate_two_isolated_with_gradient(theta_val: f64) -> (f64, f64) {
    let mut fm = build_forward_map_two_isolated();
    let mut tape = Tape::new();
    let theta = Tensor::from_slice(&[theta_val], &[1]);

    let (rb, _edit) = fm.evaluate(&theta, &mut tape);
    let reward_scalar = rb.score_with(&gradcheck_weights());

    let (grad, estimate) = fm.gradient(&theta, &tape);
    assert!(matches!(estimate, GradientEstimate::Exact));
    assert!(grad.shape() == [1]);

    (grad.as_slice()[0], reward_scalar)
}

fn two_isolated_reward_scalar(theta_val: f64) -> f64 {
    let mut fm = build_forward_map_two_isolated();
    let mut tape = Tape::new();
    let theta = Tensor::from_slice(&[theta_val], &[1]);
    let (rb, _edit) = fm.evaluate(&theta, &mut tape);
    rb.score_with(&gradcheck_weights())
}

#[test]
fn forward_map_two_isolated_full_stack_gradcheck() {
    const THETA_0: f64 = 10.0;
    const H: f64 = 1.5e-8;
    const REL_ERR_BOUND: f64 = 1e-5;

    let (analytic_grad, reward_at_theta) = evaluate_two_isolated_with_gradient(THETA_0);
    assert!(
        reward_at_theta.is_finite(),
        "2-tet reward scalar must be finite, got {reward_at_theta:e}"
    );

    let l_plus = two_isolated_reward_scalar(THETA_0 + H);
    let l_minus = two_isolated_reward_scalar(THETA_0 - H);
    let fd_grad = (l_plus - l_minus) / (2.0 * H);

    let denom = fd_grad.abs().max(1e-12);
    let rel_err = (analytic_grad - fd_grad).abs() / denom;
    assert!(
        rel_err <= REL_ERR_BOUND,
        "2-tet full-stack gradcheck failed: analytic = {analytic_grad:.6e}, \
         FD = {fd_grad:.6e}, rel_err = {rel_err:.3e} > {REL_ERR_BOUND:.0e}. \
         Diagnose in this order: (1) BasicObservable::reward_breakdown \
         peak_bound sum across [11, 23], (2) build_reward_on_tape's \
         IndexOp + tape.add chain (must mirror the primal sum), (3) DivOp \
         primal stash uses the summed peak_bound, (4) NewtonStepVjp's \
         Stage-1 multi-vertex λ accumulation (commit 6).",
    );

    // Sanity: with w_peak=1, w_stiff=1e-6, the reward scalar is dominated
    // by peak_bound ≈ 2 × 1-tet's peak ≈ 2 × 0.1009596 ≈ 0.20192. Its
    // gradient w.r.t. θ ≈ 2 × 9.59e-5 ≈ 1.92e-4 (dominated by w_peak
    // term; w_stiff·d(θ/peak)/dθ is second-order at near-linear NH).
    assert!(
        (1.0e-4..=1.0e-3).contains(&analytic_grad),
        "2-tet full-stack analytic grad {analytic_grad:.3e} outside sanity \
         band [1e-4, 1e-3] m/N at θ=10 N — check observable peak_bound \
         sum or on-tape composition."
    );
}

#[test]
fn forward_map_two_isolated_determinism() {
    const THETA_0: f64 = 10.0;
    let (grad_a, reward_a) = evaluate_two_isolated_with_gradient(THETA_0);
    let (grad_b, reward_b) = evaluate_two_isolated_with_gradient(THETA_0);
    assert_eq!(
        grad_a.to_bits(),
        grad_b.to_bits(),
        "2-tet full-stack grad_θ not bit-equal: {grad_a:e} vs {grad_b:e}"
    );
    assert_eq!(
        reward_a.to_bits(),
        reward_b.to_bits(),
        "2-tet full-stack reward scalar not bit-equal: {reward_a:e} vs {reward_b:e}"
    );
}
