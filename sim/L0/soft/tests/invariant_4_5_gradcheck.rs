//! Invariants I-4 + I-5 — joint gradcheck + determinism-in-θ test.
//!
//! **I-4 (gradcheck).** The IFT adjoint in `NewtonStepVjp::vjp` matches
//! central finite differences of the Stage-1 scene at `h = 1.5e-8` to at
//! least 5 decimal digits of relative accuracy. Scope §6 bars:
//!
//! - `h = 1.5e-8` (≈ √ε for `f64`, scope §2).
//! - Relative-error threshold `≤ 1e-5` per-component.
//! - Scalar loss `L(x*) = x_final[11]` (SQ1 option α — direct free-DOF
//!   pick; analytic grad reduces to `+λ[2]` of the adjoint solve).
//!
//! **I-5 (determinism in θ).** Two forward+backward runs with identical
//! θ must produce bit-equal `grad_θ`, `x_final`, `iter_count`, and
//! `final_residual_norm`. Validates scope §15 D-9 (no RNG on forward or
//! adjoint) and cements the "same inputs → same bits" reproducibility
//! contract the `ForwardMap` (step 6) will lean on.
//!
//! Single scene per scope §2: canonical decimeter NH tet,
//! `SolverConfig::skeleton()` defaults, θ = 10 N on +ẑ.

#![allow(
    // Test asserts that `step.x_final_var` is `Some`; a `None` here is a
    // `Solver::step` contract violation, not a runtime input condition.
    clippy::expect_used,
    // `theta_val` (the f64 input) vs `theta_var` (the tape Var) is a
    // meaningful distinction in the gradcheck; the similar naming mirrors
    // how they appear together in the math.
    clippy::similar_names
)]

use sim_ml_chassis::autograd::VjpOp;
use sim_ml_chassis::{Tape, Tensor, Var};
use sim_soft::{
    CpuNewtonSolver, NeoHookean, NullContact, SkeletonSolver, SoftScene, Solver, SolverConfig, Tet4,
};

// ── Test fixture: IndexOp ────────────────────────────────────────────────
//
// Scalarizes a 12-vector `Tensor<f64>` down to `x[idx]` — a shape-`[]`
// `Tensor` — so `tape.backward` can seed with `1.0` at the scalar and
// propagate back through `NewtonStepVjp`. Lives here because it is a
// test-only concern (the production path composes via `Observable::
// reward_breakdown` + `RewardWeights::score_with` in step 6).

const PARENT_LEN: usize = 12;

#[derive(Debug)]
struct IndexOp {
    idx: usize,
}

impl VjpOp for IndexOp {
    fn op_id(&self) -> &'static str {
        "test::IndexOp"
    }

    fn vjp(&self, cotangent: &Tensor<f64>, parent_cotans: &mut [Tensor<f64>]) {
        // Output is shape [] scalar; `tape.backward` seeds `ones(shape)`
        // which is a length-1 buffer at shape [].
        assert!(
            cotangent.shape().is_empty(),
            "IndexOp: cotangent must be shape [] scalar, got {:?}",
            cotangent.shape()
        );
        assert!(parent_cotans.len() == 1);
        assert!(
            parent_cotans[0].shape() == [PARENT_LEN],
            "IndexOp: parent must be shape [{PARENT_LEN}], got {:?}",
            parent_cotans[0].shape()
        );
        let scalar_cot = cotangent.as_slice()[0];
        parent_cotans[0].as_mut_slice()[self.idx] += scalar_cot;
    }
}

// ── Shared scene builder ─────────────────────────────────────────────────

/// Build the Stage-1 skeleton scene. Pinned by the scope's §2 defaults —
/// no knobs so every test runs against the same geometry and tuning.
fn build_solver() -> (SkeletonSolver, sim_soft::SceneInitial, SolverConfig) {
    let cfg = SolverConfig::skeleton();
    let (mesh, initial) = SoftScene::one_tet_cube();
    let solver: SkeletonSolver = CpuNewtonSolver::new(
        NeoHookean::from_lame(1e5, 4e5),
        Tet4,
        mesh,
        NullContact,
        cfg,
    );
    (solver, initial, cfg)
}

/// Run one forward + backward pass, returning the IFT analytic gradient
/// of `L = x_final[11]` w.r.t. Stage-1 θ along with all primal artifacts
/// needed for the determinism check.
fn run_forward_backward(theta_val: f64) -> (f64, Vec<f64>, usize, f64) {
    let (mut solver, initial, cfg) = build_solver();

    let mut tape = Tape::new();
    let theta_var: Var = tape.param_tensor(Tensor::from_slice(&[theta_val], &[1]));

    let step = solver.step(
        &mut tape,
        &initial.x_prev,
        &initial.v_prev,
        theta_var,
        cfg.dt,
    );

    let x_final_var = step
        .x_final_var
        .expect("Solver::step must populate x_final_var via push_custom");

    // L = x_final[11]. Push an IndexOp node that reduces the 12-vector
    // x_final to the scalar `x_final[11]`; backward from the scalar seeds
    // `1.0` at that node and propagates through IndexOp (→ one-hot [12])
    // then NewtonStepVjp (→ scalar θ cotangent).
    let l_val_scalar = tape.value_tensor(x_final_var).as_slice()[11];
    let l_tensor = Tensor::from_slice(&[l_val_scalar], &[]);
    let l_var = tape.push_custom(&[x_final_var], l_tensor, Box::new(IndexOp { idx: 11 }));

    tape.backward(l_var);
    let grad_theta = tape.grad_tensor(theta_var).as_slice()[0];

    (
        grad_theta,
        step.x_final,
        step.iter_count,
        step.final_residual_norm,
    )
}

/// Primal-only forward (no tape, no backward). Used inside the FD loop
/// where `replay_step` gives us a clean `NewtonStep` without any backward
/// bookkeeping.
fn forward_only(theta_val: f64) -> f64 {
    let (solver, initial, cfg) = build_solver();
    let theta_tensor = Tensor::from_slice(&[theta_val], &[1]);
    let step = solver.replay_step(&initial.x_prev, &initial.v_prev, &theta_tensor, cfg.dt);
    step.x_final[11]
}

// ── I-4: gradcheck at h = 1.5e-8 ─────────────────────────────────────────

#[test]
fn stage_1_gradcheck_central_fd() {
    const THETA_0: f64 = 10.0;
    // Scope §2: h = √ε for f64 — the standard spot for central FD of a
    // smooth function.
    const H: f64 = 1.5e-8;
    // Scope §6: 5-digit relative-error bar.
    const REL_ERR_BOUND: f64 = 1e-5;

    let (analytic_grad, _x_final, _iter, _resid) = run_forward_backward(THETA_0);
    let l_plus = forward_only(THETA_0 + H);
    let l_minus = forward_only(THETA_0 - H);
    let fd_grad = (l_plus - l_minus) / (2.0 * H);

    let denom = fd_grad.abs().max(1e-12);
    let rel_err = (analytic_grad - fd_grad).abs() / denom;

    assert!(
        rel_err <= REL_ERR_BOUND,
        "Stage-1 gradcheck failed: analytic = {analytic_grad:.6e}, \
         FD = {fd_grad:.6e}, rel_err = {rel_err:.3e} > {REL_ERR_BOUND:.0e}. \
         Diagnose in this order (prompt): (1) IFT minus sign, (2) free-DOF \
         slicing, (3) stale factor at non-converged iter, (4) ∂r/∂θ sign, \
         (5) S-2 line-search non-smoothness.",
    );

    // Sanity: analytic grad in the right ballpark — dimensional analysis
    // at θ=10 N gives ~1/A_33[2,2] ≈ 9.6e-5 m/N (scope solver_convergence
    // module doc; SD-3 scaling). Widen to ±3× for NH nonlinearity at ~1%
    // strain.
    assert!(
        (3.0e-5..=3.0e-4).contains(&analytic_grad),
        "Analytic grad {analytic_grad:.3e} outside sanity band [3e-5, 3e-4] m/N \
         at θ=10 N — check factor ownership or ∂r/∂θ pattern."
    );
}

// ── I-5: determinism in θ ────────────────────────────────────────────────

#[test]
fn determinism_in_theta() {
    const THETA_0: f64 = 10.0;

    let (grad_a, x_final_a, iter_a, resid_a) = run_forward_backward(THETA_0);
    let (grad_b, x_final_b, iter_b, resid_b) = run_forward_backward(THETA_0);

    assert_eq!(
        grad_a.to_bits(),
        grad_b.to_bits(),
        "grad_θ not bit-equal: {grad_a:e} vs {grad_b:e}"
    );
    assert_eq!(iter_a, iter_b, "iter_count not equal: {iter_a} vs {iter_b}");
    assert_eq!(
        resid_a.to_bits(),
        resid_b.to_bits(),
        "final_residual_norm not bit-equal: {resid_a:e} vs {resid_b:e}"
    );
    assert!(
        x_final_a.len() == x_final_b.len(),
        "x_final length mismatch: {} vs {}",
        x_final_a.len(),
        x_final_b.len(),
    );
    for (i, (&a, &b)) in x_final_a.iter().zip(x_final_b.iter()).enumerate() {
        assert_eq!(
            a.to_bits(),
            b.to_bits(),
            "x_final[{i}] not bit-equal: {a:e} vs {b:e}"
        );
    }
}
