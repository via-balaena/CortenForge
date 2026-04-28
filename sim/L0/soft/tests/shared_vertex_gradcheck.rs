//! II-3 — shared-vertex tangent assembly is correct.
//!
//! Phase 2 scope memo §1: "A 2-tet mesh with one shared face (and
//! therefore three shared vertices, one of which is free), Stage-1
//! θ on the shared free vertex, FD-vs-analytic gradcheck holds at
//! 5-digit relative-error bar. Catches double-counted Hessian
//! blocks, wrong sparsity pattern, sign errors in shared-vertex
//! accumulation."
//!
//! This is the actual new physics of multi-element FEM. The shared
//! free vertex `v_3` belongs to BOTH tets (position `a=3` in tet 0
//! `[0,1,2,3]`, position `a=2` in tet 1 `[1,2,3,4]`). Both tets
//! contribute a 3×3 block to `v_3`'s position in the global free
//! Hessian; the sum is the actual `A_free` for that vertex's DOFs.
//!
//! The asymmetric apex of tet 1 at `(0.08, 0.08, 0.08)` (NOT a
//! mirror reflection of tet 0's `v_0` per
//! `mesh::HandBuiltTetMesh::two_tet_shared_face`'s docstring)
//! ensures the two per-element Hessian contributions are distinct,
//! and the shared-vertex coupling has non-zero off-diagonal terms
//! that a buggy assembly (e.g., dropping shared-vertex coupling
//! entirely, double-counting, or sign-flipping one tet's
//! contribution) would not produce. A symmetric mirror placement
//! would let an assembly bug pass the gradcheck on cancellation
//! alone — the asymmetry is load-bearing for II-3's coverage.
//!
//! Two asserts:
//! 1. `shared_vertex_gradcheck_central_fd` — FD-vs-analytic at
//!    5-digit relative error on `L = x_final[v_3.z]`. Catches
//!    double-counted Hessian blocks, wrong sparsity pattern, sign
//!    errors in shared-vertex accumulation.
//! 2. `shared_vertex_grad_determinism` — bit-equality of `∂L/∂θ`
//!    across two same-process runs at the same θ. I-5 carry-forward
//!    for the shared-vertex backward path.
//!
//! BC layout:
//! - pinned: `[0, 1, 2, 4]` — apex `v_0` (tet 0 only), two of three
//!   shared vertices `v_1` `v_2`, apex `v_4` (tet 1 only). 4 pinned
//!   × 3 = 12 pinned DOFs.
//! - loaded: `[(3, AxisZ)]` — the third shared vertex `v_3` with
//!   Stage-1 `+ẑ` traction. 1 free × 3 = 3 free DOFs.
//! - Total: 5 vertices × 3 = 15 DOFs (12 pinned + 3 free).

#![allow(
    // `to_bits()` comparisons in test 2 are intentional.
    clippy::float_cmp,
    // `.expect` on Var-population guarantees that hold by Solver::step
    // contract — same pattern as multi_element_grad_scaling.rs et al.
    clippy::expect_used,
    // `theta_val` (f64) vs `theta_var` (Var) is a meaningful
    // distinction in the gradcheck.
    clippy::similar_names
)]

use sim_ml_chassis::{Tape, Tensor};
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, HandBuiltTetMesh, IndexOp, LoadAxis,
    MaterialField, Mesh, NullContact, Solver, SolverConfig, Tet4,
};

/// Stage-1 θ magnitude shared by all helpers.
const THETA: f64 = 10.0;

/// 2-tet shared-face system: 5 vertices × 3 = 15 DOFs.
const N_DOF: usize = 15;

/// Full-DOF index of `v_3.z` (the only free vertex's z-component).
/// `v_3` has vertex ID 3; full DOFs 9, 10, 11.
const FREE_Z_DOF: usize = 11;

// ── Scene runners ────────────────────────────────────────────────────────

/// Build the shared-face 2-tet solver + scene initial state. The
/// shared free vertex `v_3` belongs to both tets (position 3 in tet
/// 0, position 2 in tet 1) — its global free Hessian block sums
/// contributions from both elements.
fn build_shared_face_solver() -> (
    CpuTet4NHSolver<HandBuiltTetMesh>,
    Tensor<f64>,
    Tensor<f64>,
    SolverConfig,
) {
    let cfg = SolverConfig::skeleton();
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&MaterialField::uniform(1.0e5, 4.0e5));

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
        pinned_vertices: vec![0, 1, 2, 4],
        loaded_vertices: vec![(3, LoadAxis::AxisZ)],
    };

    let solver: CpuTet4NHSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);

    (solver, x_prev, v_prev, cfg)
}

/// Run forward + backward at `theta_val`, returning `∂(x_final[v_3.z])/∂θ`.
fn shared_face_grad(theta_val: f64) -> f64 {
    let (mut solver, x_prev, v_prev, cfg) = build_shared_face_solver();
    let mut tape = Tape::new();
    let theta_var = tape.param_tensor(Tensor::from_slice(&[theta_val], &[1]));
    let step = solver.step(&mut tape, &x_prev, &v_prev, theta_var, cfg.dt);
    let x_final_var = step
        .x_final_var
        .expect("Solver::step must populate x_final_var via push_custom");
    let val = tape.value_tensor(x_final_var).as_slice()[FREE_Z_DOF];
    let l_var = tape.push_custom(
        &[x_final_var],
        Tensor::from_slice(&[val], &[1]),
        Box::new(IndexOp::new(FREE_Z_DOF, N_DOF)),
    );
    tape.backward(l_var);
    tape.grad_tensor(theta_var).as_slice()[0]
}

/// Primal-only forward at `theta_val`, returning `L = x_final[v_3.z]`.
/// Uses `replay_step` for tape-free FD evaluation.
fn shared_face_loss_only(theta_val: f64) -> f64 {
    let (solver, x_prev, v_prev, cfg) = build_shared_face_solver();
    let theta_tensor = Tensor::from_slice(&[theta_val], &[1]);
    let step = solver.replay_step(&x_prev, &v_prev, &theta_tensor, cfg.dt);
    step.x_final[FREE_Z_DOF]
}

// ── Tests ────────────────────────────────────────────────────────────────

#[test]
fn shared_vertex_gradcheck_central_fd() {
    // Scope §2 h = √ε for f64; scope §6 5-digit relative-error bar.
    const H: f64 = 1.5e-8;
    const REL_ERR_BOUND: f64 = 1e-5;

    let analytic = shared_face_grad(THETA);
    let l_plus = shared_face_loss_only(THETA + H);
    let l_minus = shared_face_loss_only(THETA - H);
    let fd = (l_plus - l_minus) / (2.0 * H);

    let denom = fd.abs().max(1e-12);
    let rel_err = (analytic - fd).abs() / denom;
    assert!(
        rel_err <= REL_ERR_BOUND,
        "Shared-vertex gradcheck failed: analytic = {analytic:.6e}, \
         FD = {fd:.6e}, rel_err = {rel_err:.3e} > {REL_ERR_BOUND:.0e}. \
         Diagnose in this order: (1) shared-vertex Hessian accumulation \
         in assemble_free_hessian_triplets (BTreeMap += per-element \
         contribution at the same (col, row) key), (2) sparsity pattern \
         dedup in `new()` (BTreeSet should keep one entry per (col, row) \
         even when both tets contribute), (3) per-element grad_x_n / \
         volume sign for tet 1's asymmetric geometry, (4) NewtonStepVjp's \
         single-loaded-vertex Stage-1 path on the shared vertex.",
    );

    // Sanity band — shared free vertex's compliance is smaller than
    // 1-tet baseline (~9.6e-5) because both tets contribute mass +
    // stiffness at v_3. Loose bound to allow for asymmetric tet 1
    // geometry; tightens on real-world data.
    assert!(
        (1.0e-6..=1.0e-3).contains(&analytic),
        "Shared-vertex analytic grad {analytic:.3e} outside sanity band \
         [1e-6, 1e-3] m/N — check shared-vertex assembly."
    );
}

#[test]
fn shared_vertex_grad_determinism() {
    let g_a = shared_face_grad(THETA);
    let g_b = shared_face_grad(THETA);
    assert_eq!(
        g_a.to_bits(),
        g_b.to_bits(),
        "Shared-vertex grad_θ not bit-equal across same-process runs: \
         {g_a:e} vs {g_b:e}"
    );
}
