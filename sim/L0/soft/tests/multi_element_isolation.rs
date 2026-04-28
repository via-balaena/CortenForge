//! II-1 — multi-element assembly determinism scales 1×N to N×1.
//!
//! Phase 2 scope memo §1: "An N-tet mesh whose tets are mechanically
//! isolated (each tet's three corners Dirichlet-pinned, only its
//! `v_3`-equivalent free) produces bit-equal per-tet `x_final` across
//! runs AND bit-equal per-tet `x_final` to the 1-tet skeleton's
//! `x_final` from `tests/solver_convergence.rs::stage_1_traction_converges`."
//!
//! Three asserts:
//! 1. `isolated_tets_run_to_run_determinism` — bit-equality across two
//!    runs of the 2-isolated-tet scene at the same θ. Catches any
//!    `HashMap` leak or rayon enablement in the multi-element path
//!    (Decision M / scope §15 D-3).
//! 2. `isolated_tets_per_tet_matches_one_tet_baseline` — bit-equality
//!    of per-tet `x_final` against a fresh 1-tet skeleton run at the
//!    same θ. Tet 0 (untranslated, vertex IDs 0..3): direct
//!    component-by-component bit-equality. Tet 1 (translated by
//!    `(0.5, 0, 0)`, vertex IDs 4..7): Y/Z direct bit-equality;
//!    X-component compared as displacement (after subtracting the
//!    0.5-m translation) at FP-tolerance bound — the subtraction
//!    `(0.5 + tiny) - 0.5` may differ at the last bit from
//!    `0 + tiny` even when the underlying Newton trajectory is
//!    bit-equal.
//! 3. `isolated_tets_iter_count_matches_one_tet` — both systems
//!    converge in the same number of Newton iterations (within ±1
//!    under quadratic convergence's fast tail; in practice exactly
//!    equal at the skeleton's well-conditioned θ). Block-diagonal
//!    sparse Cholesky solves each block at the same per-iter rate
//!    as a standalone 3×3.
//!
//! NOTE on per-iter residual values: per-tet `x_final` is bit-equal
//! (test 2) but per-iter residuals are NOT — faer's sparse solve on
//! the 6×6 block-diagonal matrix takes a slightly different
//! FP-operation path per column than a standalone 3×3 sparse solve,
//! even though both blocks are mathematically independent. Newton's
//! quadratic convergence tail pushes both systems' per-tet
//! `x_final` to the same machine-precision answer despite the
//! per-iter FP drift. A naïve "per-tet residual = √2 × baseline"
//! sanity check would fail; we instead assert iter-count parity,
//! which is the load-bearing claim for block-diagonal convergence
//! rate.
//!
//! Together these gate Phase 2 commit 4's substitution: failure of
//! (1) means a determinism leak; failure of (2) means the multi-
//! element machinery perturbs per-tet behavior; failure of (3) means
//! the block-diagonal sparse-pattern construction is broken.

#![allow(
    // Bit-equality assertions on f64 are intentional throughout this
    // file — `to_bits()` comparisons document the contract.
    clippy::float_cmp
)]

use sim_ml_chassis::{Tape, Tensor};
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, HandBuiltTetMesh, LoadAxis,
    MaterialField, Mesh, NeoHookean, NullContact, SkeletonSolver, SoftScene, Solver, SolverConfig,
    Tet4,
};

/// Stage-1 θ magnitude shared by all baseline + multi-tet runs in
/// this file. Same value as
/// `tests/solver_convergence.rs::stage_1_traction_converges`
/// (θ = 10 N along `+ẑ`).
const THETA: f64 = 10.0;

/// Tet 1's translation in `two_isolated_tets`: 5× edge length along
/// `+x̂` per `mesh::HandBuiltTetMesh::two_isolated_tets` doc.
const TET_1_TRANSLATION_X: f64 = 0.5;

// ── Scene runners ────────────────────────────────────────────────────────

/// Run the 1-tet skeleton scene at `THETA` and return the converged
/// `x_final` + iter count + residual norm. Mirrors the construction
/// in `tests/solver_convergence.rs::stage_1_traction_converges` so
/// per-tet comparisons see the exact same baseline.
fn run_one_tet() -> (Vec<f64>, usize, f64) {
    let cfg = SolverConfig::skeleton();
    let (mesh, bc, initial) = SoftScene::one_tet_cube();
    let mut solver: SkeletonSolver = CpuNewtonSolver::new(
        NeoHookean::from_lame(1e5, 4e5),
        Tet4,
        mesh,
        NullContact,
        cfg,
        bc,
    );
    let mut tape = Tape::new();
    let theta_var = tape.param_tensor(Tensor::from_slice(&[THETA], &[1]));
    let step = solver.step(
        &mut tape,
        &initial.x_prev,
        &initial.v_prev,
        theta_var,
        cfg.dt,
    );
    (step.x_final, step.iter_count, step.final_residual_norm)
}

/// Run the 2-isolated-tet scene at `THETA` and return the converged
/// `x_final` + iter count + residual norm. Pinned: each tet's three
/// corners (vertex IDs 0,1,2 for tet 0; 4,5,6 for tet 1). Loaded:
/// each tet's `v_3`-equivalent (3 and 7) with `+ẑ` traction; θ
/// broadcasts (Decision L all-AxisZ branch).
fn run_two_isolated_tets() -> (Vec<f64>, usize, f64) {
    let cfg = SolverConfig::skeleton();
    let mesh = HandBuiltTetMesh::two_isolated_tets(&MaterialField::uniform(1.0e5, 4.0e5));

    // Build SceneInitial inline — `SoftScene` only ships the
    // `one_tet_cube()` constructor; multi-tet scenes drive their
    // own initial-state construction here and in
    // `multi_element_grad_scaling.rs` / `shared_vertex_gradcheck.rs`
    // (a `SoftScene::n_isolated_tets()` etc. would dedup ~10 lines
    // across the 3 multi-tet test files; deferred — minor cleanup).
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

    let mut solver: CpuTet4NHSolver<HandBuiltTetMesh> = CpuNewtonSolver::new(
        NeoHookean::from_lame(1e5, 4e5),
        Tet4,
        mesh,
        NullContact,
        cfg,
        bc,
    );

    let mut tape = Tape::new();
    let theta_var = tape.param_tensor(Tensor::from_slice(&[THETA], &[1]));
    let step = solver.step(&mut tape, &x_prev, &v_prev, theta_var, cfg.dt);
    (step.x_final, step.iter_count, step.final_residual_norm)
}

// ── Tests ────────────────────────────────────────────────────────────────

#[test]
fn isolated_tets_run_to_run_determinism() {
    let (xa, iter_a, resid_a) = run_two_isolated_tets();
    let (xb, iter_b, resid_b) = run_two_isolated_tets();

    assert_eq!(xa.len(), 24, "2-tet x_final should be 24 DOFs");
    assert_eq!(xb.len(), 24);
    assert_eq!(iter_a, iter_b, "iter_count not equal: {iter_a} vs {iter_b}");
    assert_eq!(
        resid_a.to_bits(),
        resid_b.to_bits(),
        "final_residual_norm not bit-equal: {resid_a:e} vs {resid_b:e}"
    );

    for i in 0..24 {
        assert_eq!(
            xa[i].to_bits(),
            xb[i].to_bits(),
            "x_final[{i}] not bit-equal across runs: {} vs {}",
            xa[i],
            xb[i],
        );
    }
}

#[test]
fn isolated_tets_per_tet_matches_one_tet_baseline() {
    let (one_tet, _, _) = run_one_tet();
    let (two_tet, _, _) = run_two_isolated_tets();

    assert_eq!(one_tet.len(), 12);
    assert_eq!(two_tet.len(), 24);

    // Tet 0 (vertex IDs 0..3): untranslated, occupies the same global
    // positions as the 1-tet skeleton. Bit-equality holds component-
    // by-component because the per-tet Newton trajectory is identical
    // (same x_curr inputs, same f_int / f_ext / r contributions, same
    // 3×3 Hessian block in the lower-left corner of the 2-tet
    // block-diagonal A_free).
    for i in 0..12 {
        assert_eq!(
            two_tet[i].to_bits(),
            one_tet[i].to_bits(),
            "tet 0 DOF {i} not bit-equal to one-tet baseline: {} vs {}",
            two_tet[i],
            one_tet[i],
        );
    }

    // Tet 1 (vertex IDs 4..7): translated by `(0.5, 0, 0)`. Y and Z
    // components of every vertex match the 1-tet baseline directly
    // (no translation along those axes). X components are translated;
    // we compare as displacements after subtracting `0.5`.
    for v in 4..8 {
        // Y/Z bit-equality.
        for axis in 1..3 {
            let two_tet_dof = 3 * v + axis;
            let one_tet_dof = 3 * (v - 4) + axis;
            assert_eq!(
                two_tet[two_tet_dof].to_bits(),
                one_tet[one_tet_dof].to_bits(),
                "tet 1 vertex {v} axis {axis} (DOF {two_tet_dof} = {}) ≠ \
                 one-tet baseline (DOF {one_tet_dof} = {}) bit-equal",
                two_tet[two_tet_dof],
                one_tet[one_tet_dof],
            );
        }
        // X displacement comparison. The Newton iteration produces
        // x_final[3v] ≈ 0.5 + dx (tet 1) and one_tet[3(v-4)] ≈ 0 + dx
        // for the SAME displacement dx. Subtracting the 0.5
        // translation should recover dx, but the subtraction
        // `(0.5 + dx) - 0.5` may differ from `0 + dx` by at most a
        // few ULPs near 0 (cancellation magnitude). Allowance: 1e-13
        // absolute, which is ~16-17 digits of agreement.
        let two_tet_dof_x = 3 * v;
        let one_tet_dof_x = 3 * (v - 4);
        let two_tet_dx = two_tet[two_tet_dof_x] - TET_1_TRANSLATION_X;
        let one_tet_dx = one_tet[one_tet_dof_x];
        let diff = (two_tet_dx - one_tet_dx).abs();
        assert!(
            diff < 1e-13,
            "tet 1 vertex {v} X-displacement: 2-tet (post-subtract) \
             {two_tet_dx} vs 1-tet baseline {one_tet_dx} \
             (|diff| {diff} > 1e-13)",
        );
    }
}

#[test]
fn isolated_tets_iter_count_matches_one_tet() {
    let (_, iter_one, resid_one) = run_one_tet();
    let (_, iter_two, resid_two) = run_two_isolated_tets();

    let cfg = SolverConfig::skeleton();
    assert!(
        iter_one < cfg.max_newton_iter,
        "1-tet exceeded Newton budget: {iter_one}"
    );
    assert!(
        iter_two < cfg.max_newton_iter,
        "2-tet exceeded Newton budget: {iter_two}"
    );

    // Block-diagonal sparse Cholesky converges each block at the same
    // per-iter rate as a standalone 3×3 (mathematically; FP path
    // differs per the file-level NOTE). 2-tet's tighter global tol
    // threshold (the global r_norm aggregates two blocks) is at most
    // 1 iter stricter under quadratic convergence; in practice both
    // converge in the same iter when 1-tet is comfortably below
    // threshold at θ=10 N.
    assert!(
        iter_two == iter_one || iter_two == iter_one + 1,
        "2-tet iter count {iter_two} should equal 1-tet baseline {iter_one} \
         or be at most 1 higher (block-diagonal solve)",
    );
    assert!(
        resid_one < cfg.tol,
        "1-tet final residual {resid_one:e} not below tol {:e}",
        cfg.tol,
    );
    assert!(
        resid_two < cfg.tol,
        "2-tet final residual {resid_two:e} not below tol {:e}",
        cfg.tol,
    );
}
