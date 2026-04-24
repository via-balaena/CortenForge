//! Invariants I-4 + I-5 — joint gradcheck + determinism-in-θ test.
//!
//! **I-4 (gradcheck).** The IFT adjoint in `NewtonStepVjp::vjp` matches
//! central finite differences to at least 5 decimal digits of relative
//! accuracy on both θ stages committed by scope §6:
//!
//! - Stage 1 — θ ∈ ℝ¹, 2 FD evaluations, `grad_θ = +λ[2]`.
//! - Stage 2 — θ ∈ ℝ³, 6 FD evaluations (per-component central FD),
//!   `grad_θ = +λ` full 3-vector.
//!
//! Bars:
//!
//! - `h = 1.5e-8` (≈ √ε for `f64`, scope §2).
//! - Relative-error threshold `≤ 1e-5` per-component.
//! - Scalar loss `L(x*) = x_final[11]` (SQ1 option α — direct free-DOF
//!   pick; Stage-1 analytic grad reduces to `+λ[2]`, Stage-2 widens to
//!   `+λ` on the three free DOFs).
//!
//! **I-5 (determinism in θ).** Two forward+backward runs with identical
//! θ must produce bit-equal `grad_θ`, `x_final`, `iter_count`, and
//! `final_residual_norm` on both stages. Validates scope §15 D-9 (no RNG
//! on forward or adjoint) and cements the "same inputs → same bits"
//! reproducibility contract the `ForwardMap` (step 6) leans on.
//!
//! Single scene per scope §2: canonical decimeter NH tet,
//! `SolverConfig::skeleton()` defaults. Stage 1 at θ = 10 N along +ẑ;
//! Stage 2 at θ = [3, 4, 10] N (all-nonzero mix keeps each per-component
//! rel-err denominator distinctly non-zero).

#![allow(
    // Test asserts that `step.x_final_var` is `Some`; a `None` here is a
    // `Solver::step` contract violation, not a runtime input condition.
    clippy::expect_used,
    // `theta_val` (the f64 input) vs `theta_var` (the tape Var) is a
    // meaningful distinction in the gradcheck; the similar naming mirrors
    // how they appear together in the math.
    clippy::similar_names
)]

use sim_ml_chassis::{Tape, Tensor, Var};
use sim_soft::{
    CpuNewtonSolver, IndexOp, NeoHookean, NullContact, SkeletonSolver, SoftScene, Solver,
    SolverConfig, Tet4,
};

// Step-6 promoted `IndexOp` from a test-only fixture into production
// (`sim_soft::autograd_ops::IndexOp`), so this test now imports the same
// type that `SkeletonForwardMap::build_reward_on_tape` uses. Production
// emits shape `[1]` (not `[]`) so it composes with chassis `mul` / `add`
// against Stage-1 θ; `tape.backward` on a shape-`[1]` root still seeds
// a single `1.0`, so the adjoint computed here is identical to the
// step-5 baseline.

const PARENT_LEN: usize = 12;

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
    // x_final to the scalar `x_final[11]` (as a shape-`[1]` tensor);
    // backward from the scalar seeds `1.0` at that node and propagates
    // through IndexOp (→ one-hot [12]) then NewtonStepVjp (→ scalar θ
    // cotangent).
    let l_val_scalar = tape.value_tensor(x_final_var).as_slice()[11];
    let l_tensor = Tensor::from_slice(&[l_val_scalar], &[1]);
    let l_var = tape.push_custom(
        &[x_final_var],
        l_tensor,
        Box::new(IndexOp::new(11, PARENT_LEN)),
    );

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

// ── I-4 Stage 2: θ ∈ ℝ³ full-vector gradcheck ────────────────────────────
//
// Scope §6 commits `invariant_4_5_gradcheck.rs` to run both Stage 1 (above)
// and Stage 2 (here). Stage 2 widens θ to the full traction vector
// `(t_x, t_y, t_z)` — exercising vector-cotangent accumulation into a
// shape-[3] parent through `NewtonStepVjp::vjp`'s Stage-2 branch.

/// Forward + backward with Stage-2 θ. Returns the 3-component analytic
/// gradient `∂L/∂θ` for `L = x_final[11]`.
fn run_stage_2_forward_backward(theta_val: &[f64; 3]) -> [f64; 3] {
    let (mut solver, initial, cfg) = build_solver();

    let mut tape = Tape::new();
    let theta_var: Var = tape.param_tensor(Tensor::from_slice(theta_val, &[3]));

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

    // Same loss as Stage 1: L = x_final[11]. Stage-2 widens the θ
    // cotangent shape but leaves the x_final-side cotangent flow identical.
    let l_val_scalar = tape.value_tensor(x_final_var).as_slice()[11];
    let l_tensor = Tensor::from_slice(&[l_val_scalar], &[1]);
    let l_var = tape.push_custom(
        &[x_final_var],
        l_tensor,
        Box::new(IndexOp::new(11, PARENT_LEN)),
    );

    tape.backward(l_var);
    let grad = tape.grad_tensor(theta_var);
    let g = grad.as_slice();
    [g[0], g[1], g[2]]
}

/// Primal-only Stage-2 forward. Uses `replay_step` to avoid allocating
/// tape nodes for FD loops.
fn stage_2_forward_only(theta_val: &[f64; 3]) -> f64 {
    let (solver, initial, cfg) = build_solver();
    let theta_tensor = Tensor::from_slice(theta_val, &[3]);
    let step = solver.replay_step(&initial.x_prev, &initial.v_prev, &theta_tensor, cfg.dt);
    step.x_final[11]
}

#[test]
fn stage_2_gradcheck_central_fd() {
    // θ_0 picked to have all three components distinctly nonzero (so each
    // per-component grad has a non-trivial magnitude for a rel-err
    // denominator that isn't near zero). Magnitude |θ_0| ≈ 11.6 N keeps
    // us inside the well-conditioned band Stage 1 validated (scope §2).
    const THETA_0: [f64; 3] = [3.0, 4.0, 10.0];
    const H: f64 = 1.5e-8;
    const REL_ERR_BOUND: f64 = 1e-5;

    let analytic = run_stage_2_forward_backward(&THETA_0);

    // Per-component central FD — six forward evaluations (two per axis).
    for i in 0..3 {
        let mut theta_plus = THETA_0;
        let mut theta_minus = THETA_0;
        theta_plus[i] += H;
        theta_minus[i] -= H;

        let l_plus = stage_2_forward_only(&theta_plus);
        let l_minus = stage_2_forward_only(&theta_minus);
        let fd_grad_i = (l_plus - l_minus) / (2.0 * H);

        let analytic_i = analytic[i];
        let denom = fd_grad_i.abs().max(1e-12);
        let rel_err = (analytic_i - fd_grad_i).abs() / denom;

        assert!(
            rel_err <= REL_ERR_BOUND,
            "Stage-2 gradcheck failed on θ[{i}]: analytic = {analytic_i:.6e}, \
             FD = {fd_grad_i:.6e}, rel_err = {rel_err:.3e} > {REL_ERR_BOUND:.0e}. \
             Diagnose in this order (prompt): (1) NewtonStepVjp Stage-2 \
             branch sign, (2) external_force Stage-2 index mapping, \
             (3) ∂r/∂θ = -I₃ derivation, (4) FREE_OFFSET drift, \
             (5) Stage-1 regression (should still pass)."
        );
    }
}

#[test]
fn stage_2_determinism_in_theta() {
    // Vector-cotangent determinism: two same-process Stage-2 runs must
    // return bit-equal 3-vectors of grad_θ. Catches non-determinism in
    // the shape-[3] accumulation path that shape-[1] Stage 1 cannot.
    const THETA_0: [f64; 3] = [3.0, 4.0, 10.0];

    let grad_a = run_stage_2_forward_backward(&THETA_0);
    let grad_b = run_stage_2_forward_backward(&THETA_0);

    for i in 0..3 {
        assert_eq!(
            grad_a[i].to_bits(),
            grad_b[i].to_bits(),
            "Stage-2 grad_θ[{i}] not bit-equal: {a:e} vs {b:e}",
            a = grad_a[i],
            b = grad_b[i],
        );
    }
}
