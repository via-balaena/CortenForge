//! Phase 2 commit 4a.1 — `BoundaryConditions` validation tests
//! plus A2 LU-fallback unit fixtures.
//!
//! `CpuNewtonSolver::new` validates BC against mesh dimensions
//! and the no-overlap-between-pinned-and-loaded contract before
//! the cache build runs. Three `#[should_panic]` cases here cover
//! the three validation branches; happy-path BC is covered by the
//! existing seven integration tests in `tests/`.
//!
//! The A2 fixtures exercise `factor_free_tangent` on
//! hand-constructed 3×3 triplets against a 1-tet `SkeletonSolver`
//! (`n_free = 3`). The SPD case verifies the happy path returns
//! `Llt` and round-trips a solve; the indefinite case verifies the
//! Lu fallback engages on `LltError::Numeric` and the Lu solve
//! still recovers `Ax = b`. The fallback `eprintln!` is a
//! deliberate stderr side-effect — it surfaces the rare event in
//! the test runner output as it would in a live run.

#![allow(
    // Tests are allowed to panic / expect / strict-compare-f64 — these
    // are the inherent vocabulary of "fail loudly with a clear message"
    // assertions. Production code carries per-method allows; the test
    // module hoists them once at the module level.
    clippy::panic,
    clippy::expect_used,
    clippy::float_cmp,
    // `count as VertexId` is the Mesh-trait API tax (usize -> u32 id): the
    // hand-built test meshes hold far fewer than u32::MAX vertices.
    clippy::cast_possible_truncation
)]

use faer::sparse::Triplet;

use sim_ml_chassis::Tensor;

use crate::contact::NullContact;
use crate::material::MaterialField;
use crate::mesh::{HandBuiltTetMesh, Mesh, SingleTetMesh, Tet10Mesh, VertexId};
use crate::readout::{BoundaryConditions, LoadAxis};
use crate::solver::SolverFailure;
use crate::solver::lm::LmState;
use crate::solver::{CpuNewtonSolver, LmConfig, Solver, SolverConfig};
use crate::{
    CpuTet10NHSolver, SkeletonSolver,
    element::{Tet4, Tet10},
};

fn build(bc: BoundaryConditions) -> SkeletonSolver {
    CpuNewtonSolver::new(
        Tet4,
        SingleTetMesh::new(&MaterialField::uniform(1.0e5, 4.0e5)),
        NullContact,
        SolverConfig::skeleton(),
        bc,
    )
}

/// Build a 1-tet skeleton solver with pinned 0/1/2, free vertex 3.
/// `n_free = 3` and the cached symbolic factor covers the dense
/// lower-tri 3×3 pattern (6 entries: every (col, row) with
/// `row ≥ col`).
fn build_3free_solver() -> SkeletonSolver {
    build(BoundaryConditions {
        pinned_vertices: vec![0, 1, 2],
        roller_vertices: Vec::new(),
        loaded_vertices: vec![(3, LoadAxis::AxisZ)],
    })
}

#[test]
#[should_panic(expected = "pinned_vertices contains vertex ID 99")]
fn pinned_vertex_out_of_range_panics() {
    let bc = BoundaryConditions {
        pinned_vertices: vec![0, 1, 2, 99],
        roller_vertices: Vec::new(),
        loaded_vertices: vec![(3, LoadAxis::AxisZ)],
    };
    let _ = build(bc);
}

#[test]
#[should_panic(expected = "loaded_vertices contains vertex ID 42")]
fn loaded_vertex_out_of_range_panics() {
    let bc = BoundaryConditions {
        pinned_vertices: vec![0, 1, 2],
        roller_vertices: Vec::new(),
        loaded_vertices: vec![(42, LoadAxis::AxisZ)],
    };
    let _ = build(bc);
}

#[test]
#[should_panic(expected = "which is also in pinned_vertices")]
fn loaded_vertex_overlapping_pinned_panics() {
    let bc = BoundaryConditions {
        pinned_vertices: vec![0, 1, 2, 3],
        roller_vertices: Vec::new(),
        loaded_vertices: vec![(3, LoadAxis::AxisZ)],
    };
    let _ = build(bc);
}

// --- M2-S1: roller / per-axis Dirichlet BCs ---

/// A roller frees exactly the unconstrained axes of its vertex while
/// pinning the `true` axes. Pin 0/1 fully, roller vertex 2 on x only
/// (`[true,false,false]`) so its y,z stay free, vertex 3 fully free.
/// Free DOFs (ascending) = {2y, 2z, 3x, 3y, 3z}.
#[test]
fn roller_frees_only_unconstrained_axes() {
    let solver = build(BoundaryConditions {
        pinned_vertices: vec![0, 1],
        roller_vertices: vec![(2, [true, false, false])],
        loaded_vertices: vec![],
    });
    assert_eq!(
        solver.free_dof_indices,
        vec![3 * 2 + 1, 3 * 2 + 2, 3 * 3, 3 * 3 + 1, 3 * 3 + 2],
        "roller on x must free only y,z of v2 plus all DOFs of v3"
    );
    assert_eq!(solver.n_free, 5);
    // The pinned axis maps to None; the free axes map to Some.
    assert!(
        solver.full_to_free_idx[3 * 2].is_none(),
        "v2 x is roller-pinned → not a free DOF"
    );
    assert!(
        solver.full_to_free_idx[3 * 2 + 1].is_some(),
        "v2 y is free → a free DOF"
    );
}

/// No-roller scenes yield a BIT-IDENTICAL free-DOF map to the
/// pre-roller per-vertex construction: pinning 0/1/2 leaves exactly
/// vertex 3's three DOFs free, in order.
#[test]
fn no_rollers_free_dof_map_is_unchanged() {
    let solver = build_3free_solver();
    assert_eq!(solver.free_dof_indices, vec![9, 10, 11]);
}

/// An all-`true` roller is equivalent to a full pin — the two
/// constructions produce identical free-DOF maps. (Full pins still
/// belong in `pinned_vertices`; this pins the equivalence.)
#[test]
fn all_true_roller_equals_full_pin() {
    let via_pin = build(BoundaryConditions {
        pinned_vertices: vec![0, 1, 2],
        roller_vertices: Vec::new(),
        loaded_vertices: vec![],
    });
    let via_roller = build(BoundaryConditions {
        pinned_vertices: vec![0, 1],
        roller_vertices: vec![(2, [true, true, true])],
        loaded_vertices: vec![],
    });
    assert_eq!(via_pin.free_dof_indices, via_roller.free_dof_indices);
}

/// The ergonomic `BoundaryConditions::new` constructor leaves the
/// roller list empty (the common full-pin case).
#[test]
fn new_constructor_has_no_rollers() {
    let bc = BoundaryConditions::new(vec![0, 1, 2], vec![(3, LoadAxis::AxisZ)]);
    assert!(bc.roller_vertices.is_empty());
}

#[test]
#[should_panic(expected = "roller_vertices contains vertex ID 99")]
fn roller_vertex_out_of_range_panics() {
    let _ = build(BoundaryConditions {
        pinned_vertices: vec![0, 1, 2],
        roller_vertices: vec![(99, [true, false, false])],
        loaded_vertices: vec![],
    });
}

#[test]
#[should_panic(expected = "all-false mask")]
fn roller_all_false_mask_panics() {
    let _ = build(BoundaryConditions {
        pinned_vertices: vec![0, 1, 2],
        roller_vertices: vec![(3, [false, false, false])],
        loaded_vertices: vec![],
    });
}

#[test]
#[should_panic(expected = "lists vertex ID 3 twice")]
fn roller_duplicate_vertex_panics() {
    let _ = build(BoundaryConditions {
        pinned_vertices: vec![0, 1, 2],
        roller_vertices: vec![(3, [true, false, false]), (3, [false, true, false])],
        loaded_vertices: vec![],
    });
}

#[test]
#[should_panic(expected = "in both pinned_vertices and roller_vertices")]
fn roller_overlapping_pinned_panics() {
    let _ = build(BoundaryConditions {
        pinned_vertices: vec![0, 1, 2],
        roller_vertices: vec![(2, [true, false, false])],
        loaded_vertices: vec![],
    });
}

#[test]
#[should_panic(expected = "also roller-constrained")]
fn loaded_overlapping_roller_panics() {
    let _ = build(BoundaryConditions {
        pinned_vertices: vec![0, 1],
        roller_vertices: vec![(3, [true, false, false])],
        loaded_vertices: vec![(3, LoadAxis::AxisZ)],
    });
}

/// A roller is a Dirichlet constraint held at the vertex's **initial
/// `x_prev`**, not at rest: a roller DOF driven to a nonzero offset must
/// stay exactly there through the solve. Anchors v0/v1/v3 fully pinned
/// (removes all rigid-body modes); v2 is a roller on x only, driven to
/// `rest.x + offset`, with y,z free. This is the displacement-driven
/// roller the M2 free-lateral coupon relies on to drive its `x=L` face.
#[test]
fn driven_roller_holds_dof_at_nonzero_initial_offset() {
    let offset = 0.01_f64;
    let solver = build(BoundaryConditions {
        pinned_vertices: vec![0, 1, 3],
        roller_vertices: vec![(2, [true, false, false])],
        loaded_vertices: vec![],
    });
    // SingleTetMesh rest: v0=(0,0,0), v1=(.1,0,0), v2=(0,.1,0), v3=(0,0,.1).
    let rest = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1];
    let mut x_prev = rest;
    x_prev[3 * 2] = offset; // drive v2.x off rest (rest v2.x = 0)
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = 1.0e3; // static: large dt makes M/dt² negligible
    let step = solver.replay_step(
        &Tensor::from_slice(&x_prev, &[12]),
        &Tensor::zeros(&[12]),
        &Tensor::zeros(&[0]),
        cfg.dt,
    );
    let xf = step.x_final;
    assert!(
        (xf[3 * 2] - offset).abs() < 1.0e-12,
        "driven roller DOF must stay at its x_prev offset {offset}, got {}",
        xf[3 * 2]
    );
    // The full-pin anchors stay at rest; v2's free y,z are solved (the
    // roller did not pin them — proven by the free-DOF count test above).
    assert!(
        (xf[0]).abs() < 1.0e-12
            && (xf[3] - 0.1).abs() < 1.0e-12
            && (xf[9 + 2] - 0.1).abs() < 1.0e-12,
        "fully-pinned anchors must stay at rest"
    );
}

/// SPD case: `A = [[2, 1, 0], [1, 3, 1], [0, 1, 4]]` (Sylvester:
/// det of every leading minor positive). `factor_free_tangent`
/// must return the `Llt` variant; the in-place solve must recover
/// `x` such that `A · x ≈ b = [1, 1, 1]` to f64 noise.
#[test]
fn factor_free_tangent_llt_path_on_spd_matrix() {
    let solver = build_3free_solver();
    assert_eq!(
        solver.n_free, 3,
        "1-tet skeleton with vertices 0/1/2 pinned must have n_free=3"
    );

    let triplets: Vec<Triplet<usize, usize, f64>> = vec![
        Triplet::new(0, 0, 2.0),
        Triplet::new(1, 0, 1.0),
        Triplet::new(2, 0, 0.0),
        Triplet::new(1, 1, 3.0),
        Triplet::new(2, 1, 1.0),
        Triplet::new(2, 2, 4.0),
    ];

    let mut lm_state = LmState::disabled();
    let factor = solver.factor_free_tangent(&triplets, &mut lm_state, "SPD test fixture");
    assert!(
        factor.is_llt(),
        "SPD case must take the Llt happy path, got Lu fallback"
    );

    let mut rhs = [1.0_f64, 1.0, 1.0];
    factor.solve_base_in_place(&mut rhs);

    // A · x verification — full matrix from the symmetric pattern.
    let ax0 = 2.0_f64.mul_add(rhs[0], rhs[1]);
    let ax1 = 1.0_f64.mul_add(rhs[0], 3.0_f64.mul_add(rhs[1], rhs[2]));
    let ax2 = 1.0_f64.mul_add(rhs[1], 4.0 * rhs[2]);
    let tol = 1e-12;
    assert!((ax0 - 1.0).abs() < tol, "A·x[0] = {ax0}, expected 1.0");
    assert!((ax1 - 1.0).abs() < tol, "A·x[1] = {ax1}, expected 1.0");
    assert!((ax2 - 1.0).abs() < tol, "A·x[2] = {ax2}, expected 1.0");
}

/// Indefinite case: `A = [[1, 2, 0], [2, 1, 0], [0, 0, 1]]` —
/// top-left 2×2 has eigenvalues `{3, -1}` so Llt trips
/// `NonPositivePivot { index: 1 }` (the second-row Cholesky pivot
/// `A[1][1] - L[1][0]^2 = 1 - 4 = -3 < 0`).
/// `factor_free_tangent` must engage the A2 LU fallback (printing
/// a `sim-soft: faer LU fallback ...` line to stderr) and return
/// the `Lu` variant; the in-place solve must still recover `x`
/// with `A · x ≈ b = [1, 1, 1]`.
#[test]
fn factor_free_tangent_falls_through_to_lu_on_non_pd() {
    let solver = build_3free_solver();

    let triplets: Vec<Triplet<usize, usize, f64>> = vec![
        Triplet::new(0, 0, 1.0),
        Triplet::new(1, 0, 2.0),
        Triplet::new(2, 0, 0.0),
        Triplet::new(1, 1, 1.0),
        Triplet::new(2, 1, 0.0),
        Triplet::new(2, 2, 1.0),
    ];

    let mut lm_state = LmState::disabled();
    let factor = solver.factor_free_tangent(&triplets, &mut lm_state, "indefinite test fixture");
    assert!(
        factor.is_lu(),
        "indefinite case must engage the Lu fallback"
    );

    let mut rhs = [1.0_f64, 1.0, 1.0];
    factor.solve_base_in_place(&mut rhs);

    // A = [[1, 2, 0], [2, 1, 0], [0, 0, 1]]; symmetric so the
    // lower-tri-derived "full" pattern matches A exactly.
    let ax0 = 2.0_f64.mul_add(rhs[1], rhs[0]);
    let ax1 = 2.0_f64.mul_add(rhs[0], rhs[1]);
    let ax2 = rhs[2];
    let tol = 1e-12;
    assert!((ax0 - 1.0).abs() < tol, "A·x[0] = {ax0}, expected 1.0");
    assert!((ax1 - 1.0).abs() < tol, "A·x[1] = {ax1}, expected 1.0");
    assert!((ax2 - 1.0).abs() < tol, "A·x[2] = {ax2}, expected 1.0");
}

/// F3.2 LM rescue: same indefinite fixture as
/// `factor_free_tangent_falls_through_to_lu_on_non_pd` but with
/// LM enabled (`fork_b` defaults). The Marquardt retry loop must
/// bump λ until the regularized tangent `A + λI` becomes SPD,
/// returning the `Llt` variant instead of falling through to the
/// LU fallback.
///
/// Eigenvalue arithmetic: `A = [[1, 2, 0], [2, 1, 0], [0, 0, 1]]`
/// has top-left 2×2 eigenvalues `{3, -1}`. To dominate the
/// negative mode requires `λ > 1`. With `max_diag = 1.0`,
/// `seed_relative = 1e-6`, `up_factor = 10.0`,
/// `max_retries_per_iter = 8`: retry 1 seeds at λ = 1e-6, then
/// retries 2..8 multiply by 10 each. Retry 8 reaches λ = 1e-6 ×
/// 10⁷ = 10.0, which dominates the negative mode and rescues to
/// SPD. Headroom is tight (1 retry above design-target — see spec
/// §2.2 numerical-sanity para); a future seed-relative
/// retune would loosen this.
///
/// Also confirms the LM seed log fires on stderr (visible in test
/// runner output) and the success-summary log fires once the
/// retry loop converges.
#[test]
fn factor_free_tangent_lm_bump_rescues_non_pd() {
    let mut cfg = SolverConfig::skeleton();
    cfg.lm_regularization = Some(LmConfig::fork_b());
    let solver = CpuNewtonSolver::new(
        Tet4,
        SingleTetMesh::new(&MaterialField::uniform(1.0e5, 4.0e5)),
        NullContact,
        cfg,
        BoundaryConditions {
            pinned_vertices: vec![0, 1, 2],
            roller_vertices: Vec::new(),
            loaded_vertices: vec![(3, LoadAxis::AxisZ)],
        },
    );

    // Same indefinite fixture as the LU-fallback test above; LM
    // must beat it to SPD before saturation.
    let triplets: Vec<Triplet<usize, usize, f64>> = vec![
        Triplet::new(0, 0, 1.0),
        Triplet::new(1, 0, 2.0),
        Triplet::new(2, 0, 0.0),
        Triplet::new(1, 1, 1.0),
        Triplet::new(2, 1, 0.0),
        Triplet::new(2, 2, 1.0),
    ];

    let mut lm_state = LmState::from_config(LmConfig::fork_b());
    let factor = solver.factor_free_tangent(&triplets, &mut lm_state, "LM-rescue test fixture");
    assert!(
        factor.is_llt(),
        "LM bump must rescue the non-PD case to Llt before LU fallback \
         fires (got Lu — LM exhausted retry budget before reaching SPD)"
    );
    // λ exposed post-call is the post-decay value (the
    // `on_llt_success` decay runs before return) — empirically
    // observed at this fixture: faer's Llt accepts at λ = 1.0
    // (the negative-eigenvalue mode reaches a "good enough" pivot
    // threshold), retry count = 7, then decay halves λ to 0.5.
    // Pin retry_count rather than λ directly so the test stays
    // robust to future fork_b tunables that change the seed /
    // up_factor / down_factor — the load-bearing invariant is
    // "LM activated and rescued before saturation," not the exact
    // λ value.
    assert!(
        lm_state.retry_count() > 0,
        "LM must have activated (retry_count > 0), got {}",
        lm_state.retry_count()
    );
    assert!(
        lm_state.retry_count() < LmConfig::fork_b().max_retries_per_iter,
        "LM rescued before saturation: retry_count = {} < {}",
        lm_state.retry_count(),
        LmConfig::fork_b().max_retries_per_iter,
    );
}

// ── F3.3 graceful-failure tests ────────────────────────────────────

/// F3.3 end-to-end `try_step` `NewtonIterCap` dispatch: same
/// iter-cap scene as
/// `try_solve_impl_returns_err_on_newton_iter_cap` but invoked
/// through the public `Solver::try_step` API. Verifies that
/// (a) the trait method is reachable +
/// (b) the `try_solve_impl` → `try_step` Result-bubbling chain
/// works + (c) `NewtonIterCap` returns `Err` regardless of
/// saturation policy per spec §2.5 dispatch table.
///
/// NOTE on `DoublyFailedFactor` end-to-end: constructing a
/// runtime-reachable `DoublyFailedFactor` at this fixture scale
/// is hard because faer's `SymbolicLu` pivot search accepts
/// zero-valued candidates within the structural sparsity pattern
/// (only `SymbolicSingular { index }` fires on pattern-level
/// rank deficiency, unreachable via our cached `symbolic_lu`).
/// `try_factor_free_tangent`'s direct `DoublyFailedFactor` `Err`
/// path is exercised at runtime via contact-induced rank
/// deficiency (e.g., row 21 capsule-cap-apex pre-A2 panics);
/// a permanent regression net for it is the F3.5
/// `contact_stability` augment.
#[test]
fn try_step_returns_err_on_newton_iter_cap() {
    use sim_ml_chassis::Tape;
    let mut cfg = SolverConfig::skeleton();
    cfg.max_newton_iter = 1;
    // LM enabled — NewtonIterCap is an unconditional `Err` on the
    // `try_` path (it was never gated on a stall policy).
    cfg.lm_regularization = Some(LmConfig::fork_b());
    let mut solver = CpuNewtonSolver::new(
        Tet4,
        SingleTetMesh::new(&MaterialField::uniform(1.0e5, 4.0e5)),
        NullContact,
        cfg,
        BoundaryConditions {
            pinned_vertices: vec![0, 1, 2],
            roller_vertices: Vec::new(),
            loaded_vertices: vec![(3, LoadAxis::AxisZ)],
        },
    );

    let rest = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1];
    let zero = [0.0_f64; 12];
    let x_prev = sim_ml_chassis::Tensor::from_slice(&rest, &[12]);
    let v_prev = sim_ml_chassis::Tensor::from_slice(&zero, &[12]);
    let mut tape = Tape::new();
    let theta_var = tape.param_tensor(sim_ml_chassis::Tensor::from_slice(&[1.0_f64], &[1]));

    match solver.try_step(&mut tape, &x_prev, &v_prev, theta_var, 1e-2) {
        Err(SolverFailure::NewtonIterCap { max_iter, .. }) => {
            assert_eq!(max_iter, 1);
        }
        Err(other) => panic!("expected NewtonIterCap, got {other:?}"),
        Ok(_) => panic!("expected NewtonIterCap, got Ok"),
    }
}

/// F-bar forward-only contract: the differentiable `step` (which factors the
/// adjoint tangent via `factor_at_position`) fails loudly under `config.fbar`
/// rather than return a silently-wrong gradient — the adjoint RHS does not yet
/// carry `F*` (PR2). The forward-only `replay_step` stays usable with F-bar.
#[test]
#[should_panic(expected = "F-bar differentiable gradients are not yet supported")]
fn fbar_differentiable_step_panics() {
    use sim_ml_chassis::Tape;
    let mut cfg = SolverConfig::skeleton();
    cfg.fbar = true;
    let mut solver = CpuNewtonSolver::new(
        Tet4,
        SingleTetMesh::new(&MaterialField::uniform(1.0e5, 4.0e5)),
        NullContact,
        cfg,
        BoundaryConditions {
            pinned_vertices: vec![0, 1, 2],
            roller_vertices: Vec::new(),
            loaded_vertices: vec![(3, LoadAxis::AxisZ)],
        },
    );

    let rest = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1];
    let zero = [0.0_f64; 12];
    let x_prev = sim_ml_chassis::Tensor::from_slice(&rest, &[12]);
    let v_prev = sim_ml_chassis::Tensor::from_slice(&zero, &[12]);
    let mut tape = Tape::new();
    let theta_var = tape.param_tensor(sim_ml_chassis::Tensor::from_slice(&[1.0_f64], &[1]));

    // Differentiable path → factors the adjoint → guard fires.
    let _ = solver.step(&mut tape, &x_prev, &v_prev, theta_var, 1e-2);
}

/// F3.3 `ArmijoStall` (local): direct call to `armijo_backtrack`
/// with `max_line_search_backtracks = 0` AND a synthetic
/// `r_norm = 0` at `x_curr` AND a non-zero `delta_free`. Any
/// nonzero step produces `trial_norm > 0`, and Armijo requires
/// `trial_norm <= (1 - c1·α) · 0 = 0` — guaranteed to fail. The
/// single-iteration loop (cap = 0 → range `0..=0`) exhausts
/// without an accept → `Err(ArmijoStallInfo)`.
///
/// Synthetic in that `solve_impl` never calls armijo when
/// `r_norm < tol` would return early — but the direct call here
/// pins the local `Err` mechanism, which is what
/// `try_solve_impl`'s `.map_err(|stall| SolverFailure::ArmijoStall
/// { x_partial: stall.x_curr, last_iter: stall.iter, last_r_norm:
/// stall.r_norm })` wraps. The mapping is trivial 3-line shape;
/// the load-bearing test is that `armijo_backtrack` DOES return
/// `Err` when its budget exhausts.
#[test]
fn armijo_backtrack_returns_err_on_budget_exhaustion() {
    let mut cfg = SolverConfig::skeleton();
    cfg.max_line_search_backtracks = 0;
    let solver = CpuNewtonSolver::new(
        Tet4,
        SingleTetMesh::new(&MaterialField::uniform(1.0e5, 4.0e5)),
        NullContact,
        cfg,
        BoundaryConditions {
            pinned_vertices: vec![0, 1, 2],
            roller_vertices: Vec::new(),
            loaded_vertices: vec![(3, LoadAxis::AxisZ)],
        },
    );

    // SingleTetMesh rest positions (per `mesh/single_tet.rs::build`):
    // v0=(0,0,0), v1=(0.1,0,0), v2=(0,0.1,0), v3=(0,0,0.1).
    let rest = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1];
    let zero = [0.0_f64; 12];
    let dt = 1e-2;
    // Nonzero delta on v3's 3 free DOFs → trial_x != rest →
    // trial_norm > 0 → fails Armijo against r_norm = 0.
    let delta_free = [0.05_f64, 0.05, 0.05];

    let result = solver.armijo_backtrack(&rest, &rest, &zero, &zero, dt, &delta_free, 0.0, 0);
    let stall = result
        .expect_err("max_backtracks=0 + r_norm=0 + nonzero delta must exhaust the line search");
    assert_eq!(stall.iter, 0);
    assert_eq!(stall.r_norm, 0.0);
    assert_eq!(stall.x_curr, rest.to_vec());
}

/// Build a compressive-block scene that Armijo-stalls through the
/// FULL solve (not the synthetic direct `armijo_backtrack` call
/// above): a 2×2×2 `NeoHookean` cube, bottom face pinned, top face
/// driven by a strong downward `AxisZ` load. With the line-search
/// budget pinned to a single α = 1 trial, the strong load's full
/// Newton step overshoots the nonlinear equilibrium on the first
/// iter → `armijo_backtrack` exhausts → `ArmijoStall` — a real
/// `solve_impl` path, deterministic with a large overshoot margin.
/// Returns the `(rest, v_prev, config, bc)` build inputs so each
/// test constructs a fresh solver (the cache is mutated per solve).
// Test-fixture casts: tiny usize node counts/indices -> u32 VertexId and
// f64 coordinates. Values are small loop bounds, so no precision concern.
#[allow(clippy::cast_precision_loss, clippy::cast_possible_truncation)]
fn compressive_block_stall_scene() -> (Vec<f64>, Vec<f64>, SolverConfig, BoundaryConditions) {
    let n = 2usize;
    let edge = 0.01;
    let dx = edge / n as f64;
    let sy = n + 1;
    let sz = (n + 1) * (n + 1);
    let vid = |i: usize, j: usize, k: usize| (i + j * sy + k * sz) as u32;
    let (mut pinned, mut loaded) = (Vec::new(), Vec::new());
    for i in 0..=n {
        for j in 0..=n {
            pinned.push(vid(i, j, 0));
            loaded.push((vid(i, j, n), LoadAxis::AxisZ));
        }
    }
    let n_dof = (n + 1) * (n + 1) * (n + 1) * 3;
    let mut rest = Vec::with_capacity(n_dof);
    for k in 0..=n {
        for j in 0..=n {
            for i in 0..=n {
                rest.extend_from_slice(&[i as f64 * dx, j as f64 * dx, k as f64 * dx]);
            }
        }
    }
    let mut cfg = SolverConfig::skeleton();
    // One α = 1 trial only: a strong load's full Newton step
    // overshoots → guaranteed stall on the first iter.
    cfg.max_line_search_backtracks = 0;
    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: loaded,
    };
    (rest, vec![0.0; n_dof], cfg, bc)
}

/// Wrapper contract (the graceful API never panics): on a scene that
/// Armijo-stalls through the full solve, `try_step` returns
/// `Err(SolverFailure::ArmijoStall)` — NOT a panic — under the
/// default LM-disabled config (the coupling's config). This is the
/// fix that closed the coupling's last panic-path residual; the
/// sister `step_panics_on_armijo_stall` pins the mirror.
#[test]
fn try_step_returns_err_on_armijo_stall() {
    use crate::mesh::HandBuiltTetMesh;
    use sim_ml_chassis::Tape;
    let (rest, zerov, cfg, bc) = compressive_block_stall_scene();
    let n = 2usize;
    let mut solver = CpuNewtonSolver::new(
        Tet4,
        HandBuiltTetMesh::uniform_block(n, 0.01, &MaterialField::uniform(1.0e5, 4.0e5)),
        NullContact,
        cfg,
        bc,
    );
    assert!(
        cfg.lm_regularization.is_none(),
        "scene must use the LM-disabled default (the coupling's config)"
    );
    let x_prev = Tensor::from_slice(&rest, &[rest.len()]);
    let v_prev = Tensor::from_slice(&zerov, &[zerov.len()]);
    let mut tape = Tape::new();
    let tv = tape.param_tensor(Tensor::from_slice(&[-1.0e3], &[1]));
    match solver.try_step(&mut tape, &x_prev, &v_prev, tv, 1e-3) {
        Ok(_) => panic!("expected Err(ArmijoStall), got Ok"),
        Err(SolverFailure::ArmijoStall { last_iter, .. }) => {
            assert_eq!(
                last_iter, 0,
                "single-α stall fires on the first Newton iter"
            );
        }
        Err(other) => panic!("expected Err(ArmijoStall), got {other:?}"),
    }
}

/// Mirror of `try_step_returns_err_on_armijo_stall`: the SAME
/// stalling scene driven through `Solver::step` STILL panics with
/// the pre-F3 Armijo-stall message — the panic-on-fail-close
/// contract is preserved (only the graceful `try_` API changed).
#[test]
#[should_panic(expected = "Armijo line-search stalled")]
fn step_panics_on_armijo_stall() {
    use crate::mesh::HandBuiltTetMesh;
    use sim_ml_chassis::Tape;
    let (rest, zerov, cfg, bc) = compressive_block_stall_scene();
    let n = 2usize;
    let mut solver = CpuNewtonSolver::new(
        Tet4,
        HandBuiltTetMesh::uniform_block(n, 0.01, &MaterialField::uniform(1.0e5, 4.0e5)),
        NullContact,
        cfg,
        bc,
    );
    let x_prev = Tensor::from_slice(&rest, &[rest.len()]);
    let v_prev = Tensor::from_slice(&zerov, &[zerov.len()]);
    let mut tape = Tape::new();
    let tv = tape.param_tensor(Tensor::from_slice(&[-1.0e3], &[1]));
    let _ = solver.step(&mut tape, &x_prev, &v_prev, tv, 1e-3);
}

/// F3.3 `NewtonIterCap` via `try_solve_impl`: set `max_newton_iter
/// = 1` and a θ that requires more than 1 iter to converge. Iter
/// 0 runs (factor + armijo accept); loop exits via cap; returns
/// `Err(SolverFailure::NewtonIterCap)` with `x_partial`
/// containing the post-armijo iterate.
#[test]
fn try_solve_impl_returns_err_on_newton_iter_cap() {
    let mut cfg = SolverConfig::skeleton();
    cfg.max_newton_iter = 1;
    let solver = CpuNewtonSolver::new(
        Tet4,
        SingleTetMesh::new(&MaterialField::uniform(1.0e5, 4.0e5)),
        NullContact,
        cfg,
        BoundaryConditions {
            pinned_vertices: vec![0, 1, 2],
            roller_vertices: Vec::new(),
            loaded_vertices: vec![(3, LoadAxis::AxisZ)],
        },
    );

    let rest = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1];
    let zero = [0.0_f64; 12];
    let x_prev = sim_ml_chassis::Tensor::from_slice(&rest, &[12]);
    let v_prev = sim_ml_chassis::Tensor::from_slice(&zero, &[12]);
    // Non-zero θ → r > 0 at iter 0 → must do real Newton work →
    // iter 1 needed for convergence → cap forces Err.
    let theta = sim_ml_chassis::Tensor::from_slice(&[1.0_f64], &[1]);

    match solver.try_solve_impl(&x_prev, &v_prev, &theta, 1e-2) {
        Err(SolverFailure::NewtonIterCap {
            max_iter,
            last_r_norm,
            x_partial,
        }) => {
            assert_eq!(max_iter, 1, "max_iter field must echo SolverConfig");
            assert!(
                last_r_norm > 0.0,
                "last_r_norm should be the iter-0 starting residual (>0), got {last_r_norm}",
            );
            assert_eq!(
                x_partial.len(),
                12,
                "x_partial must carry the full n_dof iterate"
            );
        }
        Err(other) => panic!("expected NewtonIterCap, got {other:?}"),
        Ok(_) => panic!("expected NewtonIterCap, got Ok"),
    }
}

/// F3.3 panic-translation: same scene as
/// `try_solve_impl_returns_err_on_newton_iter_cap` but called via
/// `Solver::step` — verifies `solve_impl`'s panic-translation arm
/// (per F3.3 spec §2.5: `step` ALWAYS panics on `NewtonIterCap`).
/// Pins the existing panic message text so a regression in
/// `solve_impl`'s `match` arm would surface as test failure not
/// silent message drift.
#[test]
#[should_panic(expected = "Newton failed to converge within 1 iterations")]
fn solver_step_panics_on_newton_iter_cap_via_solve_impl_translation() {
    use sim_ml_chassis::Tape;
    let mut cfg = SolverConfig::skeleton();
    cfg.max_newton_iter = 1;
    let mut solver = CpuNewtonSolver::new(
        Tet4,
        SingleTetMesh::new(&MaterialField::uniform(1.0e5, 4.0e5)),
        NullContact,
        cfg,
        BoundaryConditions {
            pinned_vertices: vec![0, 1, 2],
            roller_vertices: Vec::new(),
            loaded_vertices: vec![(3, LoadAxis::AxisZ)],
        },
    );

    let rest = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1];
    let zero = [0.0_f64; 12];
    let x_prev = sim_ml_chassis::Tensor::from_slice(&rest, &[12]);
    let v_prev = sim_ml_chassis::Tensor::from_slice(&zero, &[12]);
    let mut tape = Tape::new();
    let theta_var = tape.param_tensor(sim_ml_chassis::Tensor::from_slice(&[1.0_f64], &[1]));

    // Should panic with the pre-F3 NewtonIterCap message preserved.
    let _ = solver.step(&mut tape, &x_prev, &v_prev, theta_var, 1e-2);
}

// ── F3 recon candidate A (gated LM) tests ─────────────────────────

/// F3 recon A: with LM ENABLED (Fork-B preset), the gated
/// `try_solve_impl` must still converge normally on a happy-path
/// fixture where pre-F3 LU + Armijo succeeds at every iter — no
/// regression from F3.4's always-on LM behavior, no regression
/// from pre-F3.
///
/// The gated mechanism's intent is "LM stays dormant when the
/// first-pass LU + Armijo succeeds." Hard to assert directly
/// without internal introspection, but the observable signature
/// is: a fixture that converges pre-F3 in N iters MUST still
/// converge with gated-LM-enabled in N iters (no extra iter
/// cost, no different `x_final` at the converged tol). This is
/// the LM-enabled half of the §3 bit-equal-when-dormant
/// contract (the LM-disabled half is covered by the existing
/// test suite passing bit-equal).
///
/// Companion: the gated short-circuit branch (`!lm_state.
/// is_active()`) is exercised by every existing LM-disabled
/// test that completes normally; the gated escalation branch
/// is exercised end-to-end by the cf-sim-research insertion
/// suite (integration test).
#[test]
fn gated_lm_enabled_happy_path_converges_same_as_pre_f3() {
    // Baseline: LM-disabled (default skeleton), converges normally.
    let baseline_cfg = SolverConfig::skeleton();
    let baseline_solver = CpuNewtonSolver::new(
        Tet4,
        SingleTetMesh::new(&MaterialField::uniform(1.0e5, 4.0e5)),
        NullContact,
        baseline_cfg,
        BoundaryConditions {
            pinned_vertices: vec![0, 1, 2],
            roller_vertices: Vec::new(),
            loaded_vertices: vec![(3, LoadAxis::AxisZ)],
        },
    );

    let rest = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1];
    let zero = [0.0_f64; 12];
    let x_prev = sim_ml_chassis::Tensor::from_slice(&rest, &[12]);
    let v_prev = sim_ml_chassis::Tensor::from_slice(&zero, &[12]);
    let theta = sim_ml_chassis::Tensor::from_slice(&[1.0_f64], &[1]);
    let dt = 1e-2;

    let (baseline_step, _baseline_lambda) = baseline_solver
        .try_solve_impl(&x_prev, &v_prev, &theta, dt)
        .expect("baseline (LM-disabled) must converge on the happy-path fixture");

    // Same scene, but LM enabled with Fork-B preset. Gated A's
    // intent: LM stays dormant because LU + Armijo succeeds at
    // every iter on this fixture. Observable: iter_count +
    // x_final must match baseline bit-equal (no LM activity =
    // no trajectory difference).
    let mut lm_cfg = SolverConfig::skeleton();
    lm_cfg.lm_regularization = Some(LmConfig::fork_b());
    let lm_solver = CpuNewtonSolver::new(
        Tet4,
        SingleTetMesh::new(&MaterialField::uniform(1.0e5, 4.0e5)),
        NullContact,
        lm_cfg,
        BoundaryConditions {
            pinned_vertices: vec![0, 1, 2],
            roller_vertices: Vec::new(),
            loaded_vertices: vec![(3, LoadAxis::AxisZ)],
        },
    );

    let (lm_step, lm_lambda) = lm_solver
        .try_solve_impl(&x_prev, &v_prev, &theta, dt)
        .expect("LM-enabled gated A must converge on the happy-path fixture");

    assert_eq!(
        lm_step.iter_count, baseline_step.iter_count,
        "gated LM-enabled must converge in the same iter count as LM-disabled \
         baseline (no escalation should fire on this happy-path fixture)"
    );
    assert_eq!(
        lm_step.x_final, baseline_step.x_final,
        "gated LM-enabled must reach the same x_final as LM-disabled baseline \
         (bit-equal trajectory — proves LM stayed dormant)"
    );
    assert_eq!(
        lm_lambda, 0.0,
        "outer lm_state.lambda must stay 0.0 throughout (no escalation fired \
         on happy-path), got {lm_lambda}"
    );
}

// --- Rung 6: nodal reaction-force readout (`nodal_reaction_forces`) ---

/// The reaction readout returns correct NEWTONS, validated against the analytic
/// uniaxial solution (an INDEPENDENT oracle, not the FEM assembly).
///
/// Impose the homogeneous uniaxial-tension affine field `diag(λ, λt, λt)` (λt from
/// [`free_transverse_uniaxial`]) as full Dirichlet BCs on every boundary node of a
/// `Tet4` coupon; solve one static step. The reaction summed over the driven `+x`
/// face is the total axial force transmitted, which must equal the analytic
/// `cauchy_stress · deformed_area` (`σ · L² · λt²`), and its transverse components
/// must vanish (the free-transverse condition). This proves the public readout
/// (which the rung-6 soft↔rigid bond consumes) integrates the internal stress into
/// the right force, not just a self-consistent `−f_int`.
#[test]
#[allow(clippy::cast_possible_truncation)] // vertex indices sit far inside u32 range
fn nodal_reaction_matches_analytic_uniaxial_traction() {
    use crate::CpuTet4NHSolver;
    use crate::Vec3;
    use crate::material::{NeoHookean, free_transverse_uniaxial};
    use crate::mesh::{HandBuiltTetMesh, Mesh, VertexId};

    const L: f64 = 0.1;
    const N: usize = 4; // nz must be even
    const MU: f64 = 1.69e4;
    let mat = NeoHookean::from_lame(MU, 4.0 * MU);
    let field = MaterialField::uniform(MU, 4.0 * MU);

    let lam = 1.15_f64;
    let lat = free_transverse_uniaxial(&mat, lam).transverse_stretch;
    let affine = |p: Vec3| Vec3::new(p.x * lam, p.y * lat, p.z * lat);
    let on_boundary = |p: Vec3| {
        let e = 1e-9;
        p.x.abs() < e
            || (p.x - L).abs() < e
            || p.y.abs() < e
            || (p.y - L).abs() < e
            || p.z.abs() < e
            || (p.z - L).abs() < e
    };

    let mesh = HandBuiltTetMesh::uniform_block(N, L, &field);
    let rest: Vec<Vec3> = mesh.positions().to_vec();
    let n = mesh.n_vertices();
    let mut x_prev = vec![0.0_f64; 3 * n];
    let mut pinned = Vec::new();
    let mut plus_x_face = Vec::new();
    for (v, &p) in rest.iter().enumerate() {
        let target = affine(p);
        if on_boundary(p) {
            pinned.push(v as VertexId);
            if (p.x - L).abs() < 1e-9 {
                plus_x_face.push(v);
            }
        }
        x_prev[3 * v] = target.x;
        x_prev[3 * v + 1] = target.y;
        x_prev[3 * v + 2] = target.z;
    }

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = 1.0e3; // static (inertia negligible)
    let solver: CpuTet4NHSolver<HandBuiltTetMesh> = CpuNewtonSolver::new(
        Tet4,
        mesh,
        NullContact,
        cfg,
        BoundaryConditions::new(pinned, vec![]),
    );
    let step = solver.replay_step(
        &Tensor::from_slice(&x_prev, &[3 * n]),
        &Tensor::zeros(&[3 * n]),
        &Tensor::zeros(&[0]),
        cfg.dt,
    );
    assert!(
        step.final_residual_norm < cfg.tol,
        "static uniaxial solve must converge, residual {:.2e}",
        step.final_residual_norm
    );

    let react = solver.nodal_reaction_forces(&step.x_final, &x_prev, cfg.dt);
    // Reaction the constraint supplies on the +x face = the transmitted axial force.
    let f_face = plus_x_face.iter().fold(Vec3::zeros(), |acc, &v| {
        acc + Vec3::new(react[3 * v], react[3 * v + 1], react[3 * v + 2])
    });

    // Independent oracle: analytic Cauchy stress on the DEFORMED cross-section
    // (rest area L² scaled by the two transverse stretches λt²).
    let sigma = free_transverse_uniaxial(&mat, lam).cauchy_stress;
    let f_analytic = sigma * L * L * lat * lat;
    assert!(
        (f_face.x.abs() - f_analytic).abs() / f_analytic < 1e-6,
        "axial reaction {:.6} N ≠ analytic uniaxial force {f_analytic:.6} N",
        f_face.x.abs()
    );
    // Free-transverse: the driven face carries no shear/transverse reaction.
    assert!(
        f_face.y.abs().max(f_face.z.abs()) / f_analytic < 1e-6,
        "transverse reaction not free: ({:.3e}, {:.3e}) N",
        f_face.y,
        f_face.z
    );

    // The readout is exactly `−f_int` (the documented contract).
    let mut f_int = vec![0.0; 3 * n];
    solver.assemble_global_int_force(&step.x_final, &x_prev, cfg.dt, &mut f_int);
    for (r, fi) in react.iter().zip(f_int.iter()) {
        assert_eq!(*r, -*fi, "nodal_reaction_forces must return −f_int exactly");
    }
}

// --- Rung 6d: constrained (Dirichlet) reaction sensitivity (lib smoke) ---

/// Lib-level check of the rung-6d building block `internal_force_tangent_matvec`
/// (`K·v = ∂f_int/∂x · v`) against a directional FD of `assemble_global_int_force`, plus
/// a finite/live smoke of `equilibrium_dirichlet_reaction_sensitivity`. The full
/// re-solve FD gate lives in `tests/dirichlet_reaction_sensitivity.rs`; this keeps the
/// new methods in the `--lib` coverage set and validates the matvec primitive in
/// isolation (FD-the-forward-intermediate).
#[test]
// Test-only lints: small usize loop indices/counts cast to f64 (lossless at these sizes)
// and exact-value asserts on analytically-known quantities (`rel`, `live`).
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::float_cmp
)]
fn dirichlet_reaction_matvec_and_sensitivity_lib_smoke() {
    use crate::mesh::{HandBuiltTetMesh, Mesh, VertexId};

    const L: f64 = 0.1;
    const N: usize = 2;
    const MU: f64 = 3.0e4;
    let field = MaterialField::uniform(MU, 4.0 * MU);
    let mesh = HandBuiltTetMesh::uniform_block(N, L, &field);
    let n = mesh.n_vertices();

    // Both z-faces pinned (the two-endplate bond), free interior.
    let pinned: Vec<VertexId> = mesh
        .positions()
        .iter()
        .enumerate()
        .filter(|(_, p)| p.z.abs() < 1e-9 || (p.z - L).abs() < 1e-9)
        .map(|(v, _)| v as VertexId)
        .collect();
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = 1.0e3;
    cfg.max_newton_iter = 50;
    let solver: crate::CpuTet4NHSolver<HandBuiltTetMesh> = CpuNewtonSolver::new(
        Tet4,
        mesh.clone(),
        NullContact,
        cfg,
        BoundaryConditions::new(pinned, vec![]),
    );

    // Compress the top face 5% and solve to equilibrium (a non-trivial K).
    let mut x_prev = vec![0.0_f64; 3 * n];
    for (v, p) in mesh.positions().iter().enumerate() {
        x_prev[3 * v] = p.x;
        x_prev[3 * v + 1] = p.y;
        x_prev[3 * v + 2] = if (p.z - L).abs() < 1e-9 {
            p.z - 0.05 * L
        } else {
            p.z
        };
    }
    let x_final = solver
        .replay_step(
            &Tensor::from_slice(&x_prev, &[3 * n]),
            &Tensor::zeros(&[3 * n]),
            &Tensor::zeros(&[0]),
            cfg.dt,
        )
        .x_final;

    // (1) K·v vs a directional FD of f_int — the matvec primitive in isolation.
    let mut v = vec![0.0_f64; 3 * n];
    for (k, vi) in v.iter_mut().enumerate() {
        *vi = ((k % 7) as f64 - 3.0) * 0.1;
    }
    let kv = solver.internal_force_tangent_matvec(&x_final, &v);
    let eps = 1e-7;
    let xp: Vec<f64> = x_final.iter().zip(&v).map(|(a, d)| a + eps * d).collect();
    let xm: Vec<f64> = x_final.iter().zip(&v).map(|(a, d)| a - eps * d).collect();
    let (mut fp, mut fm) = (vec![0.0; 3 * n], vec![0.0; 3 * n]);
    solver.assemble_global_int_force(&xp, &x_prev, cfg.dt, &mut fp);
    solver.assemble_global_int_force(&xm, &x_prev, cfg.dt, &mut fm);
    let mut num = 0.0;
    let mut den = 0.0;
    for i in 0..3 * n {
        let fd = (fp[i] - fm[i]) / (2.0 * eps);
        num += (kv[i] - fd).powi(2);
        den += fd * fd;
    }
    let rel = (num / den.max(1e-300)).sqrt();
    assert!(rel < 1e-6, "K·v must match ∂f_int/∂x·v FD; rel = {rel:.3e}");

    // (2) reaction sensitivity: finite, live, and conserved (Σ dR ≈ 0 per axis).
    // Perturb the upper (top-face) pinned targets straight down.
    let mut dir = vec![0.0_f64; 3 * n];
    for (pv, p) in mesh.positions().iter().enumerate() {
        if (p.z - L).abs() < 1e-9 {
            dir[3 * pv + 2] = -1.0;
        }
    }
    let dr = solver.equilibrium_dirichlet_reaction_sensitivity(&x_final, cfg.dt, &dir);
    assert!(dr.iter().all(|x| x.is_finite()), "dR must be finite");
    let live = dr.iter().map(|x| x.abs()).fold(0.0, f64::max);
    assert!(live > 1.0, "dR must be live (max |dR| = {live:.3e})");
}

// ── Rung 3b: the Tet10 unpin-trio dynamics gate ────────────────────────────
//
// {free-DOF unpin + HRZ mass + Hessian-incidence widening} lands atomically:
// the moment a midside DOF is freed, its mass-diagonal `(k, k)` needs both a
// POSITIVE value (else an indefinite tangent) and a symbolic-pattern SLOT
// (else a factor pattern-mismatch). These gates prove all three on a uniform
// Tet10 mesh — midsides FREE, mass diagonal strictly POSITIVE (the HRZ
// landmine: naive row-sum lumping gives NEGATIVE Tet10 corner masses), and the
// free tangent factors PD (Llt, non-singular) at a small DYNAMIC dt. STATIC_DT
// would be mass-blind (`M/Δt² ≈ 0` swamps a bad mass), so the factor gate MUST
// run dynamically.

/// A small uniform Tet10 solver: enrich a `uniform_block` cube into a
/// [`Tet10Mesh`], pin its bottom face, and free everything above — corners
/// AND the (unpinned) midside nodes. Returns the solver plus the corner count
/// (every midside `VertexId` is `>= n_corners`). `config` lets a caller drive
/// a dynamic solve (e.g. gravity) since the config is fixed at construction.
fn build_tet10_block(config: SolverConfig) -> (CpuTet10NHSolver<Tet10Mesh>, usize) {
    let edge = 0.1;
    let field = MaterialField::uniform(1.0e5, 4.0e5);
    let cube = HandBuiltTetMesh::uniform_block(2, edge, &field);
    let n_corners = cube.n_vertices();
    let positions = cube.positions().to_vec();
    // Pin the bottom layer (z ≈ 0); the rest — corners and midsides — is free.
    let pinned: Vec<VertexId> = (0..n_corners as VertexId)
        .filter(|&v| positions[v as usize].z < 0.25 * edge)
        .collect();
    assert!(!pinned.is_empty(), "fixture must clamp a bottom face");

    let tet10 = Tet10Mesh::from_tet4(&cube);
    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: Vec::new(),
    };
    let solver = CpuNewtonSolver::new(Tet10, tet10, NullContact, config, bc);
    (solver, n_corners)
}

/// Flatten a solver's rest positions into a `3·n_vertices` DOF vector.
fn tet10_rest_dofs(solver: &CpuTet10NHSolver<Tet10Mesh>) -> Vec<f64> {
    let positions = solver.mesh.positions();
    let mut x = vec![0.0_f64; 3 * positions.len()];
    for (v, pos) in positions.iter().enumerate() {
        x[3 * v] = pos.x;
        x[3 * v + 1] = pos.y;
        x[3 * v + 2] = pos.z;
    }
    x
}

/// The unpin: a Tet10 solve must FREE the midside DOFs (rung 3a auto-pinned
/// them). At least one free DOF must belong to a midside vertex, and every
/// free DOF must round-trip through `full_to_free_idx`.
#[test]
fn tet10_midside_dofs_are_freed() {
    let (solver, n_corners) = build_tet10_block(SolverConfig::skeleton());
    let free_midside_dofs = solver
        .free_dof_indices
        .iter()
        .filter(|&&d| d / 3 >= n_corners)
        .count();
    assert!(
        free_midside_dofs > 0,
        "rung 3b must FREE the midside DOFs, but none appear in free_dof_indices \
         (n_corners = {n_corners}, n_free = {})",
        solver.n_free,
    );
    for &d in &solver.free_dof_indices {
        assert!(
            solver.full_to_free_idx[d].is_some(),
            "free DOF {d} must round-trip through full_to_free_idx",
        );
    }
}

/// The HRZ landmine: naive Tet10 row-sum lumping gives NEGATIVE corner masses
/// → indefinite tangent → Cholesky failure. This uniform block has no orphan
/// vertices, so every DOF's lumped mass must be strictly positive.
#[test]
fn tet10_mass_diagonal_is_strictly_positive() {
    let (solver, _) = build_tet10_block(SolverConfig::skeleton());
    for (i, &m) in solver.mass_per_dof.iter().enumerate() {
        assert!(
            m > 0.0,
            "mass_per_dof[{i}] = {m} is not strictly positive — HRZ lumping must \
             keep every Tet10 nodal mass positive (naive row-sum goes negative on \
             corners)",
        );
    }
}

/// The construct + FACTOR gate: the free-DOF tangent (corner constant-strain
/// stiffness + HRZ mass on the freed midsides) must factor PD — the `Llt`
/// happy path, NOT the `Lu` rescue — at a small DYNAMIC dt. The mass term
/// `M/Δt²` is what keeps the stiffness-free midside DOFs non-singular; at
/// `STATIC_DT` it is ≈ 0 and this gate would be mass-blind.
#[test]
fn tet10_free_tangent_factors_pd_at_dynamic_dt() {
    // Small + dynamic: `M/Δt²` is significant (STATIC_DT ≈ 1.0 would swamp it).
    let dt = 1.0e-3;
    let (solver, _) = build_tet10_block(SolverConfig::skeleton());
    let x_rest = tet10_rest_dofs(&solver);

    let triplets = solver.assemble_free_hessian_triplets(&x_rest, None, dt);
    let mut lm_state = LmState::disabled();
    let factor = solver.factor_free_tangent(&triplets, &mut lm_state, "rung 3b dynamics gate");
    assert!(
        factor.is_llt(),
        "the Tet10 free tangent must factor PD (Llt) at a small dynamic dt — an Lu \
         fallback means the freed midside mass diagonal is non-positive or the widened \
         symbolic incidence is missing a free-DOF (k, k) slot",
    );
}

/// End-to-end forward path: a dynamic `replay_step` under gravity must
/// converge (non-singular through every Newton factor) on the Tet10 mesh,
/// with the freed, stiffness-free midsides carried as floating masses. A
/// singular midside would trip the LU-doubly-failed panic instead.
#[test]
fn tet10_dynamic_replay_step_converges() {
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = 1.0e-3; // small dynamic dt (mass-significant)
    cfg.gravity_z = -9.81; // non-zero residual at rest → the solve actually factors
    let (solver, _) = build_tet10_block(cfg);

    let rest = tet10_rest_dofs(&solver);
    let n_dof = rest.len();
    let x_prev = Tensor::from_slice(&rest, &[n_dof]);
    let v_prev = Tensor::zeros(&[n_dof]);
    let theta = Tensor::zeros(&[0]);

    // Forward-only: `replay_step` never touches the (rung-7-guarded) adjoint.
    let step = solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt);
    assert_eq!(step.x_final.len(), n_dof);
    assert!(
        step.x_final.iter().all(|x| x.is_finite()),
        "Tet10 dynamic solve must produce a finite converged state",
    );
}
