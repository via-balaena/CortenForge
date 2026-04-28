//! II-2 — gradient aggregation across N elements is correct.
//!
//! Phase 2 scope memo §1: "Same isolated-N-tet scene with one shared
//! Stage-1 θ driving identical `+ẑ` traction on every tet's free
//! vertex. The aggregate gradient `∂(Σ_i v_3_i.z)/∂θ` equals the
//! deterministic-order sum of N copies of the 1-tet skeleton's
//! analytic grad ... per-tet contribution bit-equals baseline."
//!
//! Four asserts:
//! 1. `aggregate_gradcheck_central_fd` — FD-vs-analytic at 5-digit
//!    relative error on `L = x_final[11] + x_final[23]` (sum of both
//!    isolated tets' free-vertex z-displacements).
//! 2. `aggregate_grad_determinism` — bit-equality of `∂L/∂θ` across
//!    two same-process runs at the same θ.
//! 3. `aggregate_grad_decomposes_to_per_tet_sum` — linearity of the
//!    IFT adjoint: `∂(L_tet0 + L_tet1)/∂θ ≈ ∂L_tet0/∂θ + ∂L_tet1/∂θ`
//!    within FP precision (1e-12 relative). Demonstrates that the
//!    multi-loaded-vertex Stage-1 closed form in `NewtonStepVjp::vjp`
//!    correctly accumulates per-loaded-vertex λ contributions.
//! 4. `per_tet_grad_close_to_one_tet_baseline` — each per-tet
//!    contribution `∂(x_final[per_tet_z_dof])/∂θ` matches the 1-tet
//!    skeleton's baseline gradient within 1e-12 relative. Verifies
//!    that the multi-element machinery doesn't perturb the per-tet
//!    IFT adjoint solve materially.
//!
//! NOTE on bit-equality (per
//! [`project_faer_block_diagonal_fp_drift.md`](../../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_faer_block_diagonal_fp_drift.md)):
//! the scope memo's strict bit-equality claims for per-tet
//! contribution and aggregate-as-sum-of-N-baselines were revised
//! after the commit-5 finding that faer's sparse solve on a
//! block-diagonal SPD matrix takes a different per-column FP path
//! than a standalone same-block solve. Tests 3 and 4 use 1e-12
//! relative tolerance instead of `to_bits` equality. The
//! determinism claim within a single system size (test 2) IS
//! bit-equal — same code path, deterministic FP operations.

#![allow(
    // `to_bits()` comparisons in test 2 are intentional.
    clippy::float_cmp,
    // Test fixtures `.expect` on Var-population guarantees that hold by
    // Solver::step's contract — same pattern as
    // invariant_4_5_gradcheck.rs and forward_map_gradcheck.rs.
    clippy::expect_used,
    // `theta_val` (the f64 input) vs `theta_var` (the tape Var) is a
    // meaningful distinction in the gradcheck; same pattern as
    // invariant_4_5_gradcheck.rs.
    clippy::similar_names
)]

use sim_ml_chassis::{Tape, Tensor, Var};
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, HandBuiltTetMesh, IndexOp, LoadAxis,
    MaterialField, Mesh, NullContact, SkeletonSolver, SoftScene, Solver, SolverConfig, Tet4,
};

/// Stage-1 θ magnitude shared by all helpers in this file.
const THETA: f64 = 10.0;

/// 2-tet system DOF count.
const N_DOF_2TET: usize = 24;

/// 1-tet baseline DOF count.
const N_DOF_1TET: usize = 12;

/// Free-DOF index of `v_3.z` in the 1-tet system.
const ONE_TET_FREE_Z_DOF: usize = 11;

/// Free-DOF indices of each tet's `v_3`-equivalent z-component in
/// the 2-isolated-tet system. Tet 0's `v_3` is vertex ID 3 → DOF 11;
/// tet 1's `v_7` is vertex ID 7 → DOF 23.
const TET_0_Z_DOF: usize = 11;
const TET_1_Z_DOF: usize = 23;

// ── Scene runners ────────────────────────────────────────────────────────

/// Build a 2-isolated-tet solver + scene initial state. Mirrors
/// `multi_element_isolation::run_two_isolated_tets`'s setup.
fn build_two_tet_solver() -> (
    CpuTet4NHSolver<HandBuiltTetMesh>,
    Tensor<f64>,
    Tensor<f64>,
    SolverConfig,
) {
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
    let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
    let v_prev = Tensor::zeros(&[n_dof]);

    let bc = BoundaryConditions {
        pinned_vertices: vec![0, 1, 2, 4, 5, 6],
        loaded_vertices: vec![(3, LoadAxis::AxisZ), (7, LoadAxis::AxisZ)],
    };

    let solver: CpuTet4NHSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);

    (solver, x_prev, v_prev, cfg)
}

/// Run forward + backward on the 2-tet system at `theta_val`,
/// returning `∂L/∂θ` where `L = Σ x_final[dof]` over `loss_dofs`.
/// Loss is built via chained `IndexOp` + `tape.add`.
fn two_tet_grad_at_loss_dofs(theta_val: f64, loss_dofs: &[usize]) -> f64 {
    let (mut solver, x_prev, v_prev, cfg) = build_two_tet_solver();
    let mut tape = Tape::new();
    let theta_var = tape.param_tensor(Tensor::from_slice(&[theta_val], &[1]));
    let step = solver.step(&mut tape, &x_prev, &v_prev, theta_var, cfg.dt);
    let x_final_var = step
        .x_final_var
        .expect("Solver::step must populate x_final_var via push_custom");

    let mut loss_var: Option<Var> = None;
    for &dof in loss_dofs {
        let val = tape.value_tensor(x_final_var).as_slice()[dof];
        let val_tensor = Tensor::from_slice(&[val], &[1]);
        let dof_var = tape.push_custom(
            &[x_final_var],
            val_tensor,
            Box::new(IndexOp::new(dof, N_DOF_2TET)),
        );
        loss_var = Some(loss_var.map_or(dof_var, |prev| tape.add(prev, dof_var)));
    }
    tape.backward(loss_var.expect("at least one loss DOF must be provided"));
    tape.grad_tensor(theta_var).as_slice()[0]
}

/// Primal-only forward on the 2-tet system at `theta_val`, returning
/// the loss value `L = Σ x_final[dof]` over `loss_dofs`. Uses
/// `replay_step` to avoid tape allocations inside the FD loop.
fn two_tet_loss_only(theta_val: f64, loss_dofs: &[usize]) -> f64 {
    let (solver, x_prev, v_prev, cfg) = build_two_tet_solver();
    let theta_tensor = Tensor::from_slice(&[theta_val], &[1]);
    let step = solver.replay_step(&x_prev, &v_prev, &theta_tensor, cfg.dt);
    loss_dofs.iter().map(|&dof| step.x_final[dof]).sum()
}

/// Run forward + backward on the 1-tet baseline at `THETA`, returning
/// the analytic gradient `∂(x_final[11])/∂θ`. Mirrors
/// `invariant_4_5_gradcheck::stage_1_gradcheck_central_fd`'s
/// analytic-grad path so test 4 has a directly-comparable baseline.
fn one_tet_baseline_grad() -> f64 {
    let cfg = SolverConfig::skeleton();
    let (mesh, bc, initial) = SoftScene::one_tet_cube();
    let mut solver: SkeletonSolver = CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);
    let mut tape = Tape::new();
    let theta_var = tape.param_tensor(Tensor::from_slice(&[THETA], &[1]));
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
    let val = tape.value_tensor(x_final_var).as_slice()[ONE_TET_FREE_Z_DOF];
    let l_var = tape.push_custom(
        &[x_final_var],
        Tensor::from_slice(&[val], &[1]),
        Box::new(IndexOp::new(ONE_TET_FREE_Z_DOF, N_DOF_1TET)),
    );
    tape.backward(l_var);
    tape.grad_tensor(theta_var).as_slice()[0]
}

// ── Tests ────────────────────────────────────────────────────────────────

#[test]
fn aggregate_gradcheck_central_fd() {
    // Scope §2 h = √ε for f64; scope §6 5-digit relative-error bar.
    const H: f64 = 1.5e-8;
    const REL_ERR_BOUND: f64 = 1e-5;

    let analytic = two_tet_grad_at_loss_dofs(THETA, &[TET_0_Z_DOF, TET_1_Z_DOF]);
    let l_plus = two_tet_loss_only(THETA + H, &[TET_0_Z_DOF, TET_1_Z_DOF]);
    let l_minus = two_tet_loss_only(THETA - H, &[TET_0_Z_DOF, TET_1_Z_DOF]);
    let fd = (l_plus - l_minus) / (2.0 * H);

    let denom = fd.abs().max(1e-12);
    let rel_err = (analytic - fd).abs() / denom;
    assert!(
        rel_err <= REL_ERR_BOUND,
        "Aggregate gradcheck failed: analytic = {analytic:.6e}, FD = {fd:.6e}, \
         rel_err = {rel_err:.3e} > {REL_ERR_BOUND:.0e}. Diagnose in this order: \
         (1) NewtonStepVjp Stage-1 multi-vertex sum, (2) IndexOp + tape.add \
         chain in two_tet_grad_at_loss_dofs, (3) BC's broadcast-θ semantics \
         in assemble_external_force.",
    );

    // Sanity band — aggregate is ~2× the per-tet grad (~9.59e-5), so
    // expect ~1.9e-4 at θ=10 N. Widen 3× for NH nonlinearity margin.
    assert!(
        (6.0e-5..=6.0e-4).contains(&analytic),
        "Aggregate analytic grad {analytic:.3e} outside sanity band [6e-5, 6e-4]"
    );
}

#[test]
fn aggregate_grad_determinism() {
    let g_a = two_tet_grad_at_loss_dofs(THETA, &[TET_0_Z_DOF, TET_1_Z_DOF]);
    let g_b = two_tet_grad_at_loss_dofs(THETA, &[TET_0_Z_DOF, TET_1_Z_DOF]);
    assert_eq!(
        g_a.to_bits(),
        g_b.to_bits(),
        "Aggregate grad_θ not bit-equal across same-process runs: {g_a:e} vs {g_b:e}"
    );
}

#[test]
fn aggregate_grad_decomposes_to_per_tet_sum() {
    // IFT linearity: ∂(L_tet0 + L_tet1)/∂θ should equal
    // ∂L_tet0/∂θ + ∂L_tet1/∂θ. The aggregate is computed from one
    // sparse solve of A · λ = g_free with g_free seeded at both
    // tets' z-DOFs; the per-tet versions seed only one. Linearity
    // of A^{-1} gives equality up to FP rounding (~1-2 ULP).
    let g_aggregate = two_tet_grad_at_loss_dofs(THETA, &[TET_0_Z_DOF, TET_1_Z_DOF]);
    let g_tet0 = two_tet_grad_at_loss_dofs(THETA, &[TET_0_Z_DOF]);
    let g_tet1 = two_tet_grad_at_loss_dofs(THETA, &[TET_1_Z_DOF]);
    let sum = g_tet0 + g_tet1;

    let denom = sum.abs().max(1e-12);
    let rel_err = (g_aggregate - sum).abs() / denom;
    assert!(
        rel_err < 1e-12,
        "Aggregate-vs-per-tet-sum decomposition: g_aggregate = {g_aggregate:.6e}, \
         g_tet0 + g_tet1 = {sum:.6e} (rel_err {rel_err:.3e} > 1e-12). \
         Failure indicates non-linearity in the IFT adjoint accumulation \
         — likely a NewtonStepVjp Stage-1 sum-order bug, OR faer's \
         sparse solve linearity-under-RHS-superposition broken at the \
         block-diagonal level."
    );
}

#[test]
fn per_tet_grad_close_to_one_tet_baseline() {
    // Each per-tet contribution should match the 1-tet baseline grad
    // within FP precision. Bit-equality is NOT asserted per the
    // faer-block-diagonal FP-drift finding (project memory:
    // project_faer_block_diagonal_fp_drift.md) — faer's sparse
    // factor on a 6×6 block-diagonal matrix takes a slightly
    // different per-column FP path than a standalone 3×3, so the
    // adjoint solve λ differs at the last few bits. Tolerance:
    // 1e-12 relative, ~13 digits of agreement.
    let g_baseline = one_tet_baseline_grad();
    let g_tet0 = two_tet_grad_at_loss_dofs(THETA, &[TET_0_Z_DOF]);
    let g_tet1 = two_tet_grad_at_loss_dofs(THETA, &[TET_1_Z_DOF]);

    let denom = g_baseline.abs().max(1e-12);

    let rel_err_tet0 = (g_tet0 - g_baseline).abs() / denom;
    assert!(
        rel_err_tet0 < 1e-12,
        "Per-tet 0 grad {g_tet0:.6e} differs from 1-tet baseline {g_baseline:.6e} \
         by rel_err {rel_err_tet0:.3e} > 1e-12 — multi-loaded-vertex VJP \
         perturbing per-tet contribution beyond expected faer FP-drift."
    );

    let rel_err_tet1 = (g_tet1 - g_baseline).abs() / denom;
    assert!(
        rel_err_tet1 < 1e-12,
        "Per-tet 1 grad {g_tet1:.6e} differs from 1-tet baseline {g_baseline:.6e} \
         by rel_err {rel_err_tet1:.3e} > 1e-12 — multi-loaded-vertex VJP \
         perturbing per-tet contribution beyond expected faer FP-drift."
    );
}
