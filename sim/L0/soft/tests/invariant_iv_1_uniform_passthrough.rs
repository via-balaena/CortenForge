//! IV-1 — uniform `MaterialField` passthrough is bit-equal to
//! pre-Phase-4 baseline.
//!
//! Phase 4 scope memo §6 + §8 commit 6: post-Phase-4 code threading
//! `MaterialField::uniform(1.0e5, 4.0e5)` through every canonical scene
//! produces bit-equal numerical outputs to the pre-Phase-4 baseline
//! captured at main `c3729d4a` (the Phase 3 tip — pre-Phase-4 commit
//! immediately before this branch was cut).
//!
//! **Cross-Phase-4-boundary regression net.** The formal, named
//! version of the implicit regression net post-commit-5 already
//! satisfies (II-1 multi-element-isolation determinism, III-1 SDF-
//! pipeline mesh-equality, every gradcheck across
//! `forward_map_gradcheck` / `multi_element_grad_scaling` /
//! `shared_vertex_gradcheck` / `invariant_4_5_gradcheck` /
//! `sdf_forward_map_gradcheck` would have surfaced any drift). IV-1
//! differs from those *within-Phase-4* determinism nets by anchoring
//! against frozen pre-Phase-4 reference constants captured before this
//! branch existed — making the f64-passthrough contract from Decision P
//! ("backward compatibility — preserved") explicit and discoverable.
//!
//! **Reference capture provenance.** Bit-patterns below were captured
//! by running the canonical scene fixtures on c3729d4a with
//! `cargo test --release -p sim-soft -- --nocapture` on rustc 1.95.0
//! (2026-04-14) on macOS arm64, on 2026-04-27. Capture used inline
//! eprintln on the existing tests
//! (`solver_convergence::stage_1_traction_converges`,
//! `multi_element_isolation::run_two_isolated_tets`, the
//! `shared_vertex_gradcheck_central_fd` test body's added fresh
//! `replay_step` call,
//! `sdf_forward_map_gradcheck::sdf_forward_map_full_stack_gradcheck`'s
//! `evaluate_with_gradient(THETA_0)`); changes were never committed on
//! the detached HEAD.
//!
//! **Two-tier contract — bit-equal on dense, relative-tol on sparse-
//! solver-at-scale.** Tests 1–3 (1-tet skeleton, 2-isolated-tets,
//! 2-tet shared-face) are dense small-FEM scenes whose post-solve
//! `x_final` is bit-equal across rustc/LLVM minor versions AND across
//! `(macOS arm64, Linux x86_64)` SIMD architectures, because the
//! 12-/24-/15-DOF assembly path uses explicit `nalgebra::Matrix3`
//! arithmetic that compiles to scalar-equivalent IEEE-754 ops on
//! every supported target. Test 4 (SDF-meshed sphere) goes through
//! `faer`'s sparse Cholesky at ~3 k tets, where SIMD lane width
//! differences (NEON 128-bit vs SSE/AVX 128/256/512-bit) and
//! per-column FMA-fusion choices produce ULP-level differences in the
//! gradient/reward scalars. Empirically observed: 3-ULP drift on
//! `sdf_sphere_grad` between macOS arm64 (capture platform) and
//! Linux `x86_64` (CI runner). Tests 1–3 keep bit-equality (the
//! contract Decision P actually wants); test 4 uses 1e-12 relative
//! tolerance — twelve digits of agreement, three orders of magnitude
//! above the observed `~6.7e-16` cross-platform noise floor and many
//! orders below the relative drift any real
//! `MaterialField`-passthrough regression would produce (`>= 1e-3`
//! for a wrong-direction Lamé pair). Same-platform failure on test 4
//! signals a real regression; cross-platform 1e-12-bar agreement
//! signals the value is correct within FP precision.
//!
//! **Aligned with [`project_faer_block_diagonal_fp_drift`](../../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/project_faer_block_diagonal_fp_drift.md).**
//! The Phase 2 commit-5 finding — `faer`'s sparse solve takes a
//! per-column FP path that drifts at the last few bits even on a
//! block-diagonal SPD matrix — extends here cross-platform: the same
//! drift is amplified by SIMD-lane and FMA-fusion differences across
//! architectures. Test 4's relative-tol bar respects this finding;
//! tests 1–3's dense path doesn't trigger it.
//!
//! **Toolchain fragility.** Bit-equality on tests 1–3 is still
//! sensitive to rustc / LLVM / libm minor version differences. A
//! same-platform same-toolchain failure on tests 1–3 signals a genuine
//! Phase-4-boundary regression; cross-toolchain failure on tests 1–3
//! is the FP-stability tripwire Phase H would otherwise meet head-on.
//! Do NOT relax tests 1–3 to relative-error — defeats the contract on
//! the path where bit-equality IS achievable.
//!
//! **Tests:**
//! 1. `iv_1_one_tet_skeleton_x_final` — 1-tet skeleton `x_final` (12 DOFs).
//! 2. `iv_1_two_isolated_tets_x_final` — 2-isolated-tets `x_final` (24 DOFs).
//! 3. `iv_1_two_tet_shared_face_x_final` — 2-shared-face `x_final` via
//!    `replay_step` (15 DOFs).
//! 4. `iv_1_sdf_meshed_sphere_grad_and_reward` — SDF→FEM→autograd
//!    `(grad, reward)` via `SkeletonForwardMap::{evaluate, gradient}`.
//! 5. `iv_1_reference_constant_lengths` — meta-smoke catching
//!    accidental truncation of the reference arrays.
//!
//! Failure-mode protocol: if a test fails, do NOT re-capture the
//! reference values without first ruling out a real Phase-4 regression
//! and a real toolchain delta. Spurious re-capture hollows the
//! contract to a tautology. Toolchain-environment drift is the only
//! sanctioned re-capture trigger, and that re-capture must include a
//! memo entry naming the rustc-version delta.

#![allow(
    // Bit-equality assertions on f64 are the entire point of IV-1 —
    // `to_bits()` comparisons document the contract.
    clippy::float_cmp,
    // `.expect()` on `SdfMeshedTetMesh::from_sdf` and on the load-
    // vertex argmax surface canonical-scene contract violations as
    // test failures, matching the existing pattern in
    // `sdf_forward_map_gradcheck.rs`.
    clippy::expect_used
)]

use sim_ml_chassis::{Tape, Tensor};
use sim_soft::mesh::referenced_vertices;
use sim_soft::readout::scene::pick_vertices_by_predicate;
use sim_soft::sdf_bridge::{Aabb3, MeshingHints, SdfMeshedTetMesh, SphereSdf};
use sim_soft::{
    BasicObservable, BoundaryConditions, CpuNewtonSolver, CpuTape, CpuTet4NHSolver, ForwardMap,
    GradientEstimate, HandBuiltTetMesh, LoadAxis, MaterialField, Mesh, NullContact, RewardWeights,
    SceneInitial, SkeletonForwardMap, SkeletonSolver, SoftScene, Solver, SolverConfig, Tet4, Vec3,
    VertexId,
};

// ── Shared scene constants ───────────────────────────────────────────────

/// Stage-1 θ magnitude — same value as
/// `solver_convergence::stage_1_traction_converges`,
/// `multi_element_isolation::run_two_isolated_tets`, and
/// `sdf_forward_map_gradcheck::THETA_0`.
const THETA: f64 = 10.0;

/// Ecoflex-class Lamé parameters threaded through every scene's
/// `MaterialField::uniform`. Same values the pre-Phase-4 solver's
/// deleted `material: M` field hardcoded via
/// `NeoHookean::from_lame(1e5, 4e5)`.
const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0e5;

/// SDF sphere scene constants — mirror
/// `sdf_forward_map_gradcheck.rs::{RADIUS, CELL_SIZE, BBOX_HALF_EXTENT}`.
const SDF_RADIUS: f64 = 0.1;
const SDF_CELL_SIZE: f64 = 0.02;
const SDF_BBOX_HALF: f64 = 0.12;

// ── Pre-Phase-4 reference bit-patterns (frozen at c3729d4a) ──────────────

/// 1-tet skeleton: 4 vertices × 3 DOFs. Captured from
/// `solver_convergence::stage_1_traction_converges` at c3729d4a.
const ONE_TET_X_FINAL: [u64; 12] = [
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_d91e_b89f_fc81,
];

/// 2-isolated-tets: 8 vertices × 3 DOFs. Captured from
/// `multi_element_isolation::run_two_isolated_tets` at c3729d4a.
const TWO_ISOLATED_TETS_X_FINAL: [u64; 24] = [
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_d91e_b89f_fc81,
    0x3fe0_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fe3_3333_3333_3333,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fe0_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x3fe0_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_d91e_b89f_fc81,
];

/// 2-tet shared-face: 5 vertices × 3 DOFs, captured via a fresh
/// `replay_step(THETA, ...)` call inserted into the
/// `shared_vertex_gradcheck_central_fd` test body at c3729d4a.
const SHARED_FACE_X_FINAL: [u64; 15] = [
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x0000_0000_0000_0000,
    0x3fb9_9999_9999_999a,
    0x0000_0000_0000_0000,
    0x3f26_4080_b691_bfb7,
    0x3f26_4080_b691_bfb8,
    0x3fb9_c967_368a_3cc5,
    0x3fb4_7ae1_47ae_147b,
    0x3fb4_7ae1_47ae_147b,
    0x3fb4_7ae1_47ae_147b,
];

/// SDF-meshed sphere `evaluate_with_gradient(THETA)` outputs:
/// `(analytic_grad, reward_at_theta)`. Captured from
/// `sdf_forward_map_gradcheck::sdf_forward_map_full_stack_gradcheck`
/// at c3729d4a.
const SDF_SPHERE_GRAD: u64 = 0x3f2b_d552_2067_431f;
const SDF_SPHERE_REWARD: u64 = 0x3fba_2250_2768_c330;

// ── Helpers ──────────────────────────────────────────────────────────────

fn assert_x_final_bit_equal(actual: &[f64], expected_bits: &[u64], scene: &str) {
    assert_eq!(
        actual.len(),
        expected_bits.len(),
        "{scene}: x_final length drift — got {} entries, expected {}",
        actual.len(),
        expected_bits.len(),
    );
    for (i, (val, &exp)) in actual.iter().zip(expected_bits.iter()).enumerate() {
        let got = val.to_bits();
        assert_eq!(
            got,
            exp,
            "{scene}: x_final[{i}] bit drift across the Phase-4 boundary — \
             got {got:#018x} ({val:e}), expected {exp:#018x} ({:e}). \
             Diagnose in this order: (1) rule out toolchain drift \
             (rustc / LLVM / libm minor version delta vs the capture's \
             rustc 1.95.0); (2) if same toolchain, real regression — \
             identify which Phase-4 commit (4 mesh material-cache, 5 \
             solver `mesh.materials()` migration) altered the post-\
             Phase-4 numerics on the uniform-field path; (3) NEVER \
             re-bake the reference values to make the test green.",
            f64::from_bits(exp),
        );
    }
}

/// Relative-tolerance comparison for sparse-solver-at-scale outputs.
///
/// Used by the SDF-meshed sphere block (test 4). Cross-platform SIMD
/// lane differences (NEON 128-bit vs SSE/AVX 128/256/512-bit) and
/// per-column FMA-fusion choices produce ULP-level drift in `faer`'s
/// sparse Cholesky outputs at scale; bit-equality is too tight a
/// contract there. The 1e-12 relative bar provides 12 digits of
/// agreement — three orders above the observed `~6.7e-16`
/// cross-platform noise floor, many orders below any real
/// `MaterialField`-passthrough regression's relative drift. See
/// module docstring's "Two-tier contract" section.
fn assert_scalar_close_within_relative_tol(
    actual: f64,
    expected_bits: u64,
    max_relative: f64,
    label: &str,
) {
    let expected = f64::from_bits(expected_bits);
    let denom = expected.abs().max(f64::MIN_POSITIVE);
    let rel_err = (actual - expected).abs() / denom;
    assert!(
        rel_err <= max_relative,
        "{label}: scalar drift exceeds 1e-12 relative bar across the \
         Phase-4 boundary — got {actual:e} (bits {got:#018x}), expected \
         {expected:e} (bits {expected_bits:#018x}), rel_err = \
         {rel_err:.3e} > {max_relative:e}. \
         A drift this large is NOT cross-platform sparse-solver SIMD \
         noise (~6.7e-16 floor); it signals a real Phase-4-boundary \
         regression. Diagnose in this order: (1) verify the small-FEM \
         IV-1 sub-tests still pass bit-equal — if they fail too, \
         regression touches every `mesh.materials()` path; (2) if only \
         this test fails, regression is in the SDF→FEM→autograd \
         pipeline (Phase 4 commit 9 `MeshingHints::material_field` \
         threading is a likely candidate); (3) NEVER re-bake the \
         reference values to make the test green.",
        got = actual.to_bits(),
    );
}

// ── Scene runners (mirror pre-Phase-4 capture paths exactly) ─────────────

fn run_one_tet_x_final() -> Vec<f64> {
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
    step.x_final
}

fn run_two_isolated_tets_x_final() -> Vec<f64> {
    let cfg = SolverConfig::skeleton();
    let mesh = HandBuiltTetMesh::two_isolated_tets(&MaterialField::uniform(MU, LAMBDA));
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
    let mut solver: CpuTet4NHSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);
    let mut tape = Tape::new();
    let theta_var = tape.param_tensor(Tensor::from_slice(&[THETA], &[1]));
    let step = solver.step(&mut tape, &x_prev, &v_prev, theta_var, cfg.dt);
    step.x_final
}

fn run_shared_face_x_final() -> Vec<f64> {
    let cfg = SolverConfig::skeleton();
    let mesh = HandBuiltTetMesh::two_tet_shared_face(&MaterialField::uniform(MU, LAMBDA));
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
    let theta_tensor = Tensor::from_slice(&[THETA], &[1]);
    let step = solver.replay_step(&x_prev, &v_prev, &theta_tensor, cfg.dt);
    step.x_final
}

fn run_sdf_sphere_evaluate() -> (f64, f64) {
    // Mirrors `sdf_forward_map_gradcheck::evaluate_with_gradient(THETA_0)`
    // exactly: same hints, same boundary setup (bottom-hemisphere pin
    // + max-z referenced-vertex load), same `BasicObservable` /
    // `RewardWeights` shape, same solver path, same `(grad, reward)`
    // return.
    let mesh = SdfMeshedTetMesh::from_sdf(
        &SphereSdf { radius: SDF_RADIUS },
        &MeshingHints {
            bbox: Aabb3::new(
                Vec3::new(-SDF_BBOX_HALF, -SDF_BBOX_HALF, -SDF_BBOX_HALF),
                Vec3::new(SDF_BBOX_HALF, SDF_BBOX_HALF, SDF_BBOX_HALF),
            ),
            cell_size: SDF_CELL_SIZE,
            material_field: Some(MaterialField::uniform(MU, LAMBDA)),
        },
    )
    .expect("canonical sphere scene should mesh successfully");

    let pinned = pick_vertices_by_predicate(&mesh, |p| p.z < 0.0);
    let positions = mesh.positions();
    let load: VertexId = referenced_vertices(&mesh)
        .into_iter()
        .filter(|&v| positions[v as usize].z > 0.0)
        .max_by(|&a, &b| {
            positions[a as usize]
                .z
                .partial_cmp(&positions[b as usize].z)
                .expect("BCC lattice positions are finite by construction")
        })
        .expect("at least one referenced vertex with positive z must exist");

    let cfg = SolverConfig::skeleton();
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
        pinned_vertices: pinned,
        loaded_vertices: vec![(load, LoadAxis::AxisZ)],
    };

    let solver: Box<dyn Solver<Tape = CpuTape>> =
        Box::new(CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc));
    let load_z_dof = 3 * (load as usize) + 2;
    let observable = BasicObservable::new(vec![load_z_dof]);
    let weights = RewardWeights {
        pressure_uniformity: 0.0,
        coverage: 0.0,
        peak_bound: 1.0,
        stiffness_bound: 1.0e-6,
    };
    let mut fm = SkeletonForwardMap::new(solver, observable, initial, weights);

    let mut tape = Tape::new();
    let theta = Tensor::from_slice(&[THETA], &[1]);
    let (rb, _edit) = fm.evaluate(&theta, &mut tape);
    let reward = rb.score_with(&weights);

    let (grad, estimate) = fm.gradient(&theta, &tape);
    assert!(matches!(estimate, GradientEstimate::Exact));
    assert_eq!(grad.shape(), &[1]);
    (grad.as_slice()[0], reward)
}

// ── Tests ────────────────────────────────────────────────────────────────

#[test]
fn iv_1_one_tet_skeleton_x_final() {
    let actual = run_one_tet_x_final();
    assert_x_final_bit_equal(&actual, &ONE_TET_X_FINAL, "one_tet_skeleton");
}

#[test]
fn iv_1_two_isolated_tets_x_final() {
    let actual = run_two_isolated_tets_x_final();
    assert_x_final_bit_equal(&actual, &TWO_ISOLATED_TETS_X_FINAL, "two_isolated_tets");
}

#[test]
fn iv_1_two_tet_shared_face_x_final() {
    let actual = run_shared_face_x_final();
    assert_x_final_bit_equal(&actual, &SHARED_FACE_X_FINAL, "two_tet_shared_face");
}

#[test]
fn iv_1_sdf_meshed_sphere_grad_and_reward() {
    // 1e-12 relative tolerance per the two-tier contract — see module
    // docstring's "Two-tier contract — bit-equal on dense, relative-tol
    // on sparse-solver-at-scale" section. Cross-platform faer sparse
    // Cholesky drifts at the last few bits (NEON vs SSE/AVX SIMD lane
    // width + FMA fusion); the 1e-12 bar admits that noise while
    // catching any real Phase-4 regression at relative `>= 1e-3`.
    const REL_TOL: f64 = 1.0e-12;
    let (grad, reward) = run_sdf_sphere_evaluate();
    assert_scalar_close_within_relative_tol(grad, SDF_SPHERE_GRAD, REL_TOL, "sdf_sphere_grad");
    assert_scalar_close_within_relative_tol(
        reward,
        SDF_SPHERE_REWARD,
        REL_TOL,
        "sdf_sphere_reward",
    );
}

#[test]
fn iv_1_reference_constant_lengths() {
    // Meta-smoke: catches accidental truncation of the reference
    // arrays during edits. If a future contributor partially
    // regenerates the captures and pastes a shorter array, this test
    // surfaces the mismatch immediately rather than letting the
    // corresponding scene assertion fail with a confusing length-drift
    // diagnostic.
    assert_eq!(
        ONE_TET_X_FINAL.len(),
        12,
        "one_tet captures should be 4 vertices × 3 DOFs",
    );
    assert_eq!(
        TWO_ISOLATED_TETS_X_FINAL.len(),
        24,
        "two_isolated_tets captures should be 8 vertices × 3 DOFs",
    );
    assert_eq!(
        SHARED_FACE_X_FINAL.len(),
        15,
        "two_tet_shared_face captures should be 5 vertices × 3 DOFs",
    );
}
