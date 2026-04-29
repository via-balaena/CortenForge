//! V-1 — `NullContact` real-zero-stub passthrough is bit-equal to
//! pre-Phase-5 baseline.
//!
//! Phase 5 scope memo §1 V-1 + §8 commit 7: post-Phase-5 code threading
//! the new contact-dispatch hot path (commit 5's edits to
//! `backward_euler.rs`'s `assemble_global_int_force` and
//! `assemble_free_hessian_triplets` — `self.contact.active_pairs(...)`
//! per Newton iter + gradient/Hessian scatter via `contributions` Vecs)
//! through every representative pre-Phase-5 invariant scene produces
//! bit-equal numerical outputs to the pre-Phase-5 baseline captured at
//! main `c3729d4a` (the IV-1 baseline-capture point — pre-Phase-5 by
//! transitivity through Phase 4, see "Reference provenance" below).
//!
//! **Cross-Phase-5-boundary regression net.** Distinct from Phase 4 IV-1
//! (`invariant_iv_1_uniform_passthrough.rs`) in *what* it pins:
//! - IV-1 verifies that Phase 4's per-tet `MaterialField` aggregator
//!   passthrough produces bit-equal outputs against the pre-Phase-4
//!   baseline. Contact dispatch did not exist when IV-1 was authored —
//!   `_contact: C` carried an underscore prefix on `CpuNewtonSolver`
//!   and not a single `ContactModel` method was reached on any code
//!   path (scope memo §0.5 architectural finding).
//! - V-1 verifies that Phase 5's contact-dispatch hot path through
//!   `NullContact`'s real zero-stubs (commit 3:
//!   `Vec::new()` / `0.0` / `Default::default()` / `f64::INFINITY`
//!   replacing the unreachable `unimplemented!("skeleton phase 2")`)
//!   produces bit-equal outputs against the pre-Phase-5 baseline.
//!   This is the *first end-to-end exercise* of `NullContact`'s
//!   methods through the live solver dispatch path.
//!
//! Both tests assert bit-equality on the same canonical scenes today;
//! the contracts diverge if (e.g.) a future Phase 6 change touches the
//! solver's contact-summation path without touching the material path,
//! or vice versa.
//!
//! **Reference provenance.** The bit-pattern constants below are the
//! IV-1 captures from `c3729d4a` (Phase 3 tip, pre-Phase-4) re-pasted
//! verbatim. Their validity as pre-Phase-5 baselines rests on
//! transitivity: IV-1 still passes at HEAD post-commit-5/6 (215/215
//! sim-soft tests green pre-V-1, including IV-1's four sub-tests on
//! these scenes), so the bit-equal `x_final` at `c3729d4a` carries
//! through `c3729d4a` → main `96d7d679` (Phase 4 merge tip =
//! pre-Phase-5 branch-cut point) → HEAD via the new contact-dispatch
//! path.
//! See `invariant_iv_1_uniform_passthrough.rs` module docstring for
//! the full capture protocol (rustc 1.95.0, macOS arm64, 2026-04-27,
//! detached HEAD), the toolchain-fragility tripwires, and the
//! re-capture failure-mode protocol — NEVER re-bake reference values
//! to make a test green.
//!
//! **Two-tier contract — bit-equal on dense, relative-tol on sparse-
//! solver-at-scale.** Mirrors the IV-1 contract (whose two-tier rationale
//! and Phase-4 cross-platform CI provenance live there in detail). Tests
//! 1–3 (1-tet skeleton, 2-isolated-tets, 2-tet shared-face) are dense
//! small-FEM scenes whose post-solve `x_final` is bit-equal across
//! `(rustc/LLVM minor versions, macOS arm64, Linux x86_64)`. Test 4
//! (SDF-meshed sphere) goes through `faer`'s sparse Cholesky at ~3 k
//! tets, where SIMD lane width differences and per-column FMA-fusion
//! choices produce ULP-level drift; the 1e-12 relative bar (twelve
//! digits of agreement, three orders above the observed ~6.7e-16
//! cross-platform noise floor, many orders below any real
//! contact-dispatch regression's drift) is the right tolerance there.
//! See `project_faer_block_diagonal_fp_drift.md` cross-platform
//! extension for the full rationale.
//!
//! **Determinism-across-runs gate (R-2 carry-forward, dispatch-wiring
//! tier).** Each scene runs twice within its test fn; the second run's
//! output is asserted bit-equal to the first run's via `to_bits()` on
//! every scalar. Scope memo §6 R-2 names a failure mode that lives
//! inside `PenaltyRigidContact::active_pairs` (e.g., `HashSet`-based
//! primitive iteration); V-1 exercises `NullContact` specifically, so
//! it cannot directly surface non-determinism that lives inside the
//! penalty impl — that's V-3a/V-3/V-5's beat (commits 8/9/10). What V-1
//! *does* catch is non-determinism in the *dispatch wiring* introduced
//! at commit 5: the new for-loops over `gradient.contributions` /
//! `hessian.contributions`, the `slice_to_vec3s` helper, and the
//! ordering of `self.contact.active_pairs` relative to elastic
//! assembly inside `assemble_global_int_force` /
//! `assemble_free_hessian_triplets`. With `NullContact` the
//! `contributions` Vecs are empty so those for-loops are 0-iter; any
//! non-determinism in their setup or in the surrounding solver
//! arithmetic still surfaces as a run-vs-run drift. Bit-equality
//! between runs holds even on the SDF-sphere case because the
//! within-process faer execution is deterministic by construction
//! (`sim-soft/Cargo.toml` disables faer's rayon feature via
//! `default-features = false`, per the I-5 within-process
//! determinism rationale documented in that file). Distinct from
//! II-1's within-process-determinism gate
//! (`multi_element_isolation.rs`), which only covers hand-built scenes
//! — V-1 generalizes the gate to the SDF-meshed sparse-solver path,
//! the path where any solver-side allocation-order regression on the
//! contact-dispatch boundary would land first.
//!
//! **Tests:**
//! 1. `v_1_one_tet_skeleton_passthrough` — 1-tet skeleton `x_final`
//!    (12 DOFs) bit-equal vs baseline AND between two runs.
//! 2. `v_1_two_isolated_tets_passthrough` — 2-isolated-tets `x_final`
//!    (24 DOFs) bit-equal vs baseline AND between two runs.
//! 3. `v_1_two_tet_shared_face_passthrough` — 2-tet shared-face
//!    `x_final` (15 DOFs) bit-equal vs baseline AND between two runs.
//! 4. `v_1_sdf_sphere_grad_and_reward_passthrough` — SDF-meshed sphere
//!    `(grad, reward)` 1e-12 relative vs baseline AND bit-equal
//!    between two runs (within-process faer determinism).
//! 5. `v_1_reference_constant_lengths` — meta-smoke catching accidental
//!    truncation of the reference arrays.
//!
//! **Why V-1 lands before V-3a / V-3 / V-5 (commits 8 / 9 / 10).**
//! Failure of V-1 means the Phase 5 contact-dispatch wiring is
//! silently corrupting elastic dynamics on non-contact scenes, and any
//! V-3a / V-3 / V-5 failure would otherwise be entangled with that
//! root cause. V-1 must pin the zero-regression baseline before
//! contact-active scientific gates can be diagnosed in isolation.

#![allow(
    // Bit-equality assertions on f64 are the entire point of V-1 —
    // `to_bits()` comparisons document the contract.
    clippy::float_cmp,
    // `.expect()` on `SdfMeshedTetMesh::from_sdf` and on the load-
    // vertex argmax surface canonical-scene contract violations as
    // test failures, matching the existing pattern in
    // `invariant_iv_1_uniform_passthrough.rs` and
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

/// Stage-1 θ magnitude — same value as IV-1 + the upstream
/// `solver_convergence::stage_1_traction_converges` capture path.
const THETA: f64 = 10.0;

/// Ecoflex-class Lamé parameters threaded through every scene's
/// `MaterialField::uniform`. Matches IV-1.
const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0e5;

/// SDF sphere scene constants — mirror IV-1 +
/// `sdf_forward_map_gradcheck.rs::{RADIUS, CELL_SIZE, BBOX_HALF_EXTENT}`.
const SDF_RADIUS: f64 = 0.1;
const SDF_CELL_SIZE: f64 = 0.02;
const SDF_BBOX_HALF: f64 = 0.12;

// ── Pre-Phase-5 reference bit-patterns (from c3729d4a — IV-1 captures) ───
//
// These are the IV-1 captures re-pasted verbatim. Their validity as
// pre-Phase-5 baselines rests on transitivity: IV-1 still passes at
// HEAD post-commit-5/6, so the bit-equal x_final at c3729d4a carries
// through Phase 4 merge tip 96d7d679 (= pre-Phase-5 branch-cut point)
// and the new contact-dispatch path. See module docstring "Reference
// provenance" section + `invariant_iv_1_uniform_passthrough.rs` for
// the full capture protocol and re-capture failure-mode protocol.

/// 1-tet skeleton: 4 vertices × 3 DOFs.
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

/// 2-isolated-tets: 8 vertices × 3 DOFs.
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

/// 2-tet shared-face: 5 vertices × 3 DOFs.
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
/// `(analytic_grad, reward_at_theta)`.
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
            "{scene}: x_final[{i}] bit drift across the Phase-5 boundary — \
             got {got:#018x} ({val:e}), expected {exp:#018x} ({:e}). \
             Diagnose in this order: (1) rule out toolchain drift \
             (rustc / LLVM / libm minor version delta vs the IV-1 \
             capture's rustc 1.95.0); (2) if same toolchain, real \
             regression — identify which Phase-5 commit (3 NullContact \
             zero-stubs, 5 solver contact-dispatch wiring, 6 alias + \
             scene helpers) altered the post-Phase-5 numerics on the \
             NullContact-flavor path; (3) NEVER re-bake the reference \
             values to make the test green.",
            f64::from_bits(exp),
        );
    }
}

/// Within-process determinism gate — second run's `x_final` is
/// bit-equal to first run's. Catches non-determinism in the
/// `active_pairs` walk + `gradient`/`hessian` `contributions` Vec
/// allocation pattern through faer (scope memo §6 R-2 + Decision M).
fn assert_x_final_runs_bit_equal(run_a: &[f64], run_b: &[f64], scene: &str) {
    assert_eq!(
        run_a.len(),
        run_b.len(),
        "{scene}: x_final length drift between runs — run A has {} \
         entries, run B has {} entries (impossible barring a structural \
         scene-construction non-determinism)",
        run_a.len(),
        run_b.len(),
    );
    for (i, (&a, &b)) in run_a.iter().zip(run_b.iter()).enumerate() {
        assert_eq!(
            a.to_bits(),
            b.to_bits(),
            "{scene}: x_final[{i}] bit drift between two within-process \
             runs — run A {:#018x} ({a:e}) vs run B {:#018x} ({b:e}). \
             V-1's run-vs-run gate covers the dispatch-wiring tier of \
             scope memo R-2 + Decision M (the in-impl-`active_pairs` \
             non-determinism tier is V-3a/V-3/V-5's beat — V-1 uses \
             `NullContact` whose `active_pairs` returns `Vec::new()`). \
             Faer is bit-deterministic within-process by construction \
             (`sim-soft/Cargo.toml` disables faer's rayon feature for \
             this purpose); a run-vs-run drift here implicates Phase-5 \
             dispatch wiring (commit 5's edits to `backward_euler.rs`'s \
             assembly methods or any subsequent regression in the \
             surrounding solver arithmetic), not faer.",
            a.to_bits(),
            b.to_bits(),
        );
    }
}

/// Relative-tolerance comparison for sparse-solver-at-scale outputs.
///
/// Used by the SDF-meshed sphere block (test 4). Mirrors IV-1's helper
/// of the same name. Cross-platform SIMD lane differences (NEON vs
/// SSE/AVX) and per-column FMA-fusion choices produce ULP-level drift
/// in `faer`'s sparse Cholesky outputs at scale; bit-equality is too
/// tight a contract there for the *baseline-comparison* tier (the
/// run-vs-run within-process tier still uses `to_bits()` because faer
/// is deterministic same-process). The 1e-12 relative bar provides 12
/// digits of agreement — three orders above the observed ~6.7e-16
/// cross-platform noise floor, many orders below any real
/// contact-dispatch-passthrough regression's relative drift. See module
/// docstring's "Two-tier contract" section.
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
         Phase-5 boundary — got {actual:e} (bits {got:#018x}), expected \
         {expected:e} (bits {expected_bits:#018x}), rel_err = \
         {rel_err:.3e} > {max_relative:e}. \
         A drift this large is NOT cross-platform sparse-solver SIMD \
         noise (~6.7e-16 floor); it signals a real Phase-5-boundary \
         regression on the SDF-meshed-sphere contact-dispatch path. \
         Diagnose in this order: (1) verify the small-FEM V-1 \
         sub-tests still pass bit-equal — if they fail too, regression \
         touches every contact-dispatch path through `NullContact`'s \
         zero-stubs (commit 5 wiring is the prime candidate); (2) if \
         only this test fails, regression is in the SDF→FEM→autograd \
         contact-dispatch composition; (3) NEVER re-bake the reference \
         values to make the test green.",
        got = actual.to_bits(),
    );
}

// ── Scene runners (mirror IV-1 capture paths exactly) ────────────────────

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
    // Mirrors IV-1's `run_sdf_sphere_evaluate` exactly: same hints,
    // same boundary setup (bottom-hemisphere pin + max-z referenced-
    // vertex load), same `BasicObservable` / `RewardWeights` shape,
    // same solver path, same `(grad, reward)` return.
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
fn v_1_one_tet_skeleton_passthrough() {
    let run_a = run_one_tet_x_final();
    let run_b = run_one_tet_x_final();
    assert_x_final_bit_equal(&run_a, &ONE_TET_X_FINAL, "one_tet_skeleton (run A)");
    assert_x_final_runs_bit_equal(&run_a, &run_b, "one_tet_skeleton");
}

#[test]
fn v_1_two_isolated_tets_passthrough() {
    let run_a = run_two_isolated_tets_x_final();
    let run_b = run_two_isolated_tets_x_final();
    assert_x_final_bit_equal(
        &run_a,
        &TWO_ISOLATED_TETS_X_FINAL,
        "two_isolated_tets (run A)",
    );
    assert_x_final_runs_bit_equal(&run_a, &run_b, "two_isolated_tets");
}

#[test]
fn v_1_two_tet_shared_face_passthrough() {
    let run_a = run_shared_face_x_final();
    let run_b = run_shared_face_x_final();
    assert_x_final_bit_equal(&run_a, &SHARED_FACE_X_FINAL, "two_tet_shared_face (run A)");
    assert_x_final_runs_bit_equal(&run_a, &run_b, "two_tet_shared_face");
}

#[test]
fn v_1_sdf_sphere_grad_and_reward_passthrough() {
    // 1e-12 relative tolerance for baseline comparison per the
    // two-tier contract — see module docstring's "Two-tier contract"
    // section. Cross-platform faer sparse Cholesky drifts at the last
    // few bits (NEON vs SSE/AVX SIMD lane width + FMA fusion); the
    // 1e-12 bar admits that noise while catching any real Phase-5
    // contact-dispatch regression at relative >= 1e-3. The
    // run-vs-run within-process determinism gate uses `to_bits()`
    // because faer is bit-deterministic within a single process.
    const REL_TOL: f64 = 1.0e-12;
    let (grad_a, reward_a) = run_sdf_sphere_evaluate();
    let (grad_b, reward_b) = run_sdf_sphere_evaluate();
    assert_scalar_close_within_relative_tol(
        grad_a,
        SDF_SPHERE_GRAD,
        REL_TOL,
        "sdf_sphere_grad (run A)",
    );
    assert_scalar_close_within_relative_tol(
        reward_a,
        SDF_SPHERE_REWARD,
        REL_TOL,
        "sdf_sphere_reward (run A)",
    );
    assert_eq!(
        grad_a.to_bits(),
        grad_b.to_bits(),
        "sdf_sphere_grad: bit drift between two within-process runs — \
         run A {:#018x} ({grad_a:e}) vs run B {:#018x} ({grad_b:e}). \
         Faer is bit-deterministic within-process; a drift here \
         implicates Phase-5 dispatch-wiring non-determinism (commit \
         5's edits to `backward_euler.rs`'s assembly methods). \
         `NullContact`'s `active_pairs` returns `Vec::new()` and \
         `contributions` Vecs are empty, so the in-impl-`active_pairs` \
         non-determinism mode named at R-2 cannot surface here — that \
         tier is V-3a/V-3/V-5's beat.",
        grad_a.to_bits(),
        grad_b.to_bits(),
    );
    assert_eq!(
        reward_a.to_bits(),
        reward_b.to_bits(),
        "sdf_sphere_reward: bit drift between two within-process runs \
         — run A {:#018x} ({reward_a:e}) vs run B {:#018x} \
         ({reward_b:e}). Faer is bit-deterministic within-process; a \
         drift here implicates Phase-5 dispatch-wiring non-determinism \
         (commit 5). See `sdf_sphere_grad`'s assertion for the full \
         R-2-tier diagnostic.",
        reward_a.to_bits(),
        reward_b.to_bits(),
    );
}

#[test]
fn v_1_reference_constant_lengths() {
    // Meta-smoke: catches accidental truncation of the reference
    // arrays during edits (mirror of IV-1's
    // `iv_1_reference_constant_lengths`).
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
