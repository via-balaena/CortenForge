//! IV-7 — Validity-domain composition with first-violator-wins
//! structured panic.
//!
//! Phase 4 scope memo §1 IV-7 + §8 commit 12. Verifies the per-tet
//! [`Material::validity`] check the solver runs at step start
//! (Decision Q "at step start") fires on the first violator with a
//! structured panic message naming the tet id and the violated
//! [`ValidityDomain`] slot. Three independent gates:
//!
//! - **Baseline non-violation** — at the rest configuration `F = I`
//!   the validity check is satisfied (zero stretch deviation,
//!   det F = 1) and the solver step runs cleanly. Pins the
//!   "happy path is unchanged" contract — Decision Q's check
//!   does not introduce false positives on canonical scenes.
//! - **Stretch violation panic** — `x_prev` constructed so the
//!   single tet's deformation gradient is `F = diag(2.5, 1, 1)`
//!   gives `max_i |σ_i − 1| = 1.5`, violating
//!   [`NeoHookean`]'s declared `max_stretch_deviation = 1.0`.
//!   Solver step panics with a message containing both the slot
//!   name (`max_stretch_deviation`) and the tet id (`tet 0`).
//! - **First-violator-wins** — multi-tet scene with both tets
//!   violating; the panic message names the lowest-id tet (`tet
//!   0`), not the second violator (`tet 1`). Pins the
//!   ascending-`tet_id` ordering of the validity walk per
//!   Decision Q first-violator-wins semantics + Decision N
//!   determinism (same scene → same panic message bit-by-bit).
//!
//! ## Why message-substring contracts
//!
//! Per Decision Q "no new `SolverError` enum, no new `Result`-returning
//! surface, no γ-locked API change" — the panic message is the API
//! contract. Tests pin on the named slot identifier (one of
//! `max_stretch_deviation` or `inversion`) plus the tet id (`tet
//! N`), not the numeric formatting. Future drift in the numeric
//! portion (e.g., changing `{:.3}` to `{:.4}`) is harmless;
//! drift in the slot name is a contract break.
//!
//! ## Why `replay_step` not `step`
//!
//! `step` mutates the solver and pushes a `NewtonStepVjp` onto a
//! tape. `replay_step` is the pure-function counterpart, calls the
//! same `solve_impl` (which contains the Decision Q validity check),
//! and is the ergonomic test path when no autograd interaction is
//! needed. Mirrors the IV-2 (`multi_material_continuity.rs`)
//! `replay_step` pattern.
//!
//! ## `ValidityDomain` slots not exercised here
//!
//! Per [`Part 2 §00 §02`][v]: six slots — `max_stretch_deviation`
//! (exercised), `max_rotation` (NH = `INFINITY`, never violated),
//! `poisson_range` (construction-time on the material constants,
//! not on `F`; not a runtime check), `temperature_range` (None for
//! NH; thermal decorator is Phase H), `strain_rate_range` (None for
//! NH; viscoelastic decorator is Phase H), `inversion` (NH =
//! `RequireOrientation`).
//!
//! `inversion` is checked by the solver (`check_validity_at_step_start`
//! tests `det F ≤ 0` against `RequireOrientation`) but not exercised
//! here as a triggered panic — `det F < 0` requires a
//! left-handed reference tet, which `signed_volume()` assertions in
//! [`HandBuiltTetMesh`] prevent at construction. A test that
//! constructs `x_prev` with one vertex permutation that flips
//! orientation is a valid follow-on; for Phase 4 the
//! stretch-deviation gate is the discriminating panic test for the
//! mechanism, and inversion is a structurally-prevented case (NH's
//! own `first_piola` `expect`s on invertibility, but that fires
//! _inside_ NH not in the solver-side check). Tracking as a commit
//! 13 grader-fixup or Phase H follow-on candidate.
//!
//! [v]: ../../../../docs/studies/soft_body_architecture/src/20-materials/00-trait-hierarchy/02-validity.md

use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, HandBuiltTetMesh, LoadAxis,
    MaterialField, Mesh, NullContact, SingleTetMesh, SkeletonSolver, Solver, SolverConfig, Tet4,
};

// ── Helpers ──────────────────────────────────────────────────────────────

fn canonical_field() -> MaterialField {
    MaterialField::uniform(1.0e5, 4.0e5)
}

/// Pinned-base BC for `SingleTetMesh`: pin `v_0..=v_2`, leave `v_3`
/// free. Mirrors the IV-1 `solver_convergence::stage_1_traction_converges`
/// fixture; load is omitted for IV-7 (validity panic must fire before
/// θ matters).
fn single_tet_bc() -> BoundaryConditions {
    BoundaryConditions {
        pinned_vertices: vec![0, 1, 2],
        loaded_vertices: vec![(3, LoadAxis::AxisZ)],
    }
}

fn rest_x_prev_single_tet() -> Tensor<f64> {
    let mesh = SingleTetMesh::new(&canonical_field());
    let positions = mesh.positions();
    let n_dof = 3 * positions.len();
    let mut x_prev_flat = vec![0.0; n_dof];
    for (v, pos) in positions.iter().enumerate() {
        x_prev_flat[3 * v] = pos.x;
        x_prev_flat[3 * v + 1] = pos.y;
        x_prev_flat[3 * v + 2] = pos.z;
    }
    Tensor::from_slice(&x_prev_flat, &[n_dof])
}

fn build_skeleton_solver() -> SkeletonSolver {
    CpuNewtonSolver::new(
        Tet4,
        SingleTetMesh::new(&canonical_field()),
        NullContact,
        SolverConfig::skeleton(),
        single_tet_bc(),
    )
}

// ── Tests ────────────────────────────────────────────────────────────────

#[test]
fn iv_7_baseline_rest_config_does_not_panic() {
    // Happy-path gate. The canonical decimeter `SingleTetMesh` at its
    // rest configuration produces `F = I` for every tet (zero stretch
    // deviation, `det F = 1.0`); the Decision Q validity check fires
    // and is satisfied, the Newton step runs to convergence. Pins the
    // "happy path is unchanged" contract — IV-7's panic gate must
    // not introduce false positives on canonical scenes.
    //
    // Loaded vertex `v_3` carries θ = 1.0 along `+ẑ` (Stage-1 axis-z
    // traction); the small-strain step displaces v_3 by O(θ / k_eff)
    // — well within the NH stretch-deviation bound (= 1.0). If this
    // test starts panicking, either the validity check is over-
    // reading the stretch (e.g., reading `F` post-step instead of
    // pre-step), or the NH stretch bound was tightened below 1.0.
    let solver = build_skeleton_solver();
    let cfg = SolverConfig::skeleton();
    let x_prev = rest_x_prev_single_tet();
    let v_prev = Tensor::zeros(&[3 * 4]);
    let theta = Tensor::from_slice(&[1.0], &[1]);

    // No `#[should_panic]` — this call must complete normally.
    let _ = solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt);
}

#[test]
#[should_panic(expected = "max_stretch_deviation")]
fn iv_7_excessive_stretch_panics_with_slot_name() {
    // Stretch-violation gate. Construct `x_prev` with `v_1`
    // translated 2.5× along `+x̂` (canonical decimeter edge length
    // is 0.1 m; `v_1 = (0.1, 0, 0)` rest, `v_1 = (0.25, 0, 0)`
    // here). The deformation gradient at the single tet becomes
    //
    //     F = diag(2.5, 1, 1)
    //
    // (for the canonical Tet4 with `v_0 = origin` + axis-aligned
    // `v_1`/`v_2`/`v_3`). Singular values `σ = (2.5, 1.0, 1.0)`,
    // `max |σ_i - 1| = 1.5`, which exceeds `NeoHookean`'s declared
    // `max_stretch_deviation = 1.0`. Decision Q validity check at
    // step start panics with the slot identifier in the message;
    // `#[should_panic(expected = "max_stretch_deviation")]` pins
    // that slot identifier.
    //
    // This test deliberately does NOT pin the `tet 0` identifier
    // (the next test does that explicitly via the multi-tet scene);
    // the slot name alone is the contract this test exercises.
    let solver = build_skeleton_solver();
    let cfg = SolverConfig::skeleton();
    let n_dof = 3 * 4;
    let mut x_prev_flat = vec![0.0; n_dof];
    // v_0 at origin (rest).
    // v_1 at (0.25, 0, 0) — stretched 2.5× from rest.
    x_prev_flat[3] = 0.25;
    // v_2 at (0, 0.1, 0) (rest).
    x_prev_flat[7] = 0.1;
    // v_3 at (0, 0, 0.1) (rest).
    x_prev_flat[11] = 0.1;
    let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
    let v_prev = Tensor::zeros(&[n_dof]);
    let theta = Tensor::from_slice(&[1.0], &[1]);

    let _ = solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt);
}

#[test]
#[should_panic(expected = "tet 0")]
fn iv_7_first_violator_wins_names_lowest_tet_id() {
    // First-violator-wins gate per Decision Q. Multi-tet scene
    // ([`HandBuiltTetMesh::two_isolated_tets`] — 8 vertices, 2
    // mechanically isolated tets, vertices `[0..4]` ∈ tet 0,
    // `[4..8]` ∈ tet 1) with BOTH tets driven into stretch
    // violation simultaneously: the validity walk visits tet 0
    // first (ascending `tet_id` order per Decision Q), panics
    // there, and never reaches tet 1.
    //
    // Construction: stretch tet 0's `v_1` to 2.5×, AND stretch
    // tet 1's `v_5` to 4× the canonical edge. Both tets violate
    // `max_stretch_deviation = 1.0`. Decision N determinism
    // guarantees the walk order is deterministic, so the first
    // violator is always tet 0; `#[should_panic(expected = "tet
    // 0")]` pins that ordering. A regression that visits tet 1
    // first (e.g., a future parallelism switch on the validity
    // walk) would produce `"tet 1"` and fail this test loud.
    //
    // The slot identifier is implicitly `max_stretch_deviation`
    // (since both tets exceed that bound); pinning `tet 0`
    // alone is the discriminator for first-violator-wins.
    let cfg = SolverConfig::skeleton();
    let mesh = HandBuiltTetMesh::two_isolated_tets(&canonical_field());
    // Pin all of tet 0's vertices, leave tet 1's free; load v_4.
    // (BC layout follows IV-1's `two_isolated_tets` pattern.)
    let bc = BoundaryConditions {
        pinned_vertices: vec![0, 1, 2, 3, 5, 6, 7],
        loaded_vertices: vec![(4, LoadAxis::AxisZ)],
    };
    let solver: CpuTet4NHSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, NullContact, cfg, bc);

    // Build `x_prev` from rest, then mutate two specific vertices
    // to trigger violations in BOTH tets.
    let positions = SingleTetMesh::new(&canonical_field()).positions().to_vec();
    // Above is just for canonical edge length reference (0.1 m).
    let _ = positions;

    let n_dof = 3 * 8;
    // Reconstruct the rest config of `two_isolated_tets`:
    //   tet 0: v_0=(0,0,0), v_1=(0.1,0,0), v_2=(0,0.1,0), v_3=(0,0,0.1)
    //   tet 1: v_4=(0.5,0,0), v_5=(0.6,0,0), v_6=(0.5,0.1,0), v_7=(0.5,0,0.1)
    let mut x_prev_flat = vec![0.0; n_dof];
    // Tet 0 rest: v_1, v_2, v_3 at canonical offsets
    x_prev_flat[3] = 0.25; // v_1 stretched 2.5× along +x (tet 0 violation)
    x_prev_flat[7] = 0.1; // v_2.y
    x_prev_flat[11] = 0.1; // v_3.z
    // Tet 1: v_4 base, v_5 stretched 4× from base, v_6 + v_7 at rest offsets
    x_prev_flat[12] = 0.5; // v_4.x (rest base)
    x_prev_flat[15] = 0.5 + 0.4; // v_5.x = base + 4× edge (tet 1 violation, also exceeds 1.0)
    x_prev_flat[18] = 0.5; // v_6.x (rest)
    x_prev_flat[19] = 0.1; // v_6.y
    x_prev_flat[21] = 0.5; // v_7.x (rest)
    x_prev_flat[23] = 0.1; // v_7.z

    let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
    let v_prev = Tensor::zeros(&[n_dof]);
    let theta = Tensor::from_slice(&[1.0], &[1]);

    let _ = solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt);
}
