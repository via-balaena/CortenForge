//! Tet10 ladder rung 6 — the **locking-sensitive** half of the ν → 0.5 verdict.
//!
//! ## Why this file exists
//!
//! `tet10_lame_decision.rs` decides rung 6c on a thick-walled sphere under
//! internal pressure, and returned ACCEPT. A gating cold-read then falsified
//! the *warrant* for that verdict: extending the sweep one step showed that
//! the Lamé oracle **cannot express progressive volumetric locking at all**.
//! Going ν = 0.49 → 0.499 raises `λ/μ` from 49 to 499 — a 10× stiffer
//! volumetric constraint — and Tet4's error does not grow; it saturates and
//! slightly improves (0.1083 → 0.1062 under `Continuum`, 0.1677 → 0.1653 under
//! `Facet`; see `oracle_locking_sensitivity_saturates_beyond_nu_0_49`).
//!
//! That is physically reasonable rather than a bug. The sphere problem is
//! radially symmetric, and its exact incompressible solution is the pure
//! volume-preserving mode `u_r ∝ 1/r²`. A displacement field with one spatial
//! degree of freedom never sets the deviatoric and volumetric responses
//! against each other, which is what makes locking severe. Corroborating: the
//! Lamé oracle's own docstring predicts "convergence collapse" as ν → 0.5, and
//! Newton converges in 3 iterations there at every ν tried, including 0.499.
//!
//! So the sphere legitimately certifies *accuracy in mean displacement at
//! ν = 0.49*, and cannot certify *"Tet10 cures locking"* — an oracle in which
//! the control does not lock cannot show that a candidate does not lock
//! either. **This file supplies the missing evidence on an oracle that
//! provably does lock.**
//!
//! ## The oracle: a cantilever, and the offset-free ratio metric
//!
//! Bending is the canonical locking signature (Part 2 Ch 05 `00-locking.md`):
//! a cantilever's tip deflection **collapses** as ν → 0.5 even though the
//! physical deflection is nearly ν-independent, because the per-element
//! volumetric constraints suppress exactly the isochoric bending modes.
//! `fbar_locking.rs` already establishes this beam as a working locking
//! detector — plain Tet4's ν-ratio collapses below 0.45 there, and F-bar
//! restores it above 0.60. This file reuses that scene and adds a Tet10 arm.
//!
//! **The metric is the ratio `δ(ν_hi)/δ(ν_lo)` on one element, never a
//! cross-element deflection comparison.** Constant-strain Tet4 under-resolves
//! bending by a large multiplicative factor, and Tet10 does not — comparing
//! their raw deflections would measure element order, not locking. The ratio
//! divides out any factor that is ν-independent, which includes the
//! discretisation factor, the mesh, **and the tip-load distribution**. That
//! last point matters here: an equal per-node split of the tip force is
//! inconsistent for a quadratic face (the Lamé file's `LoadRule` work is all
//! about this), but it is applied identically at both ν, so it cancels in the
//! ratio exactly as the discretisation factor does. No consistent-load
//! machinery is needed for a locking measurement.
//!
//! Physical target: `δ ∝ 1/E` with `E = 2μ(1+ν)`, so the ideal ratio is
//! `(1+ν_lo)/(1+ν_hi) = 1.30/1.49 ≈ 0.872` — the beam should barely stiffen.
//!
//! ## Result
//!
//! Tet4 **0.2341** (collapsed — the control locks, as `fbar_locking` also
//! finds), Tet10 **0.8058** against the physical 0.8725. The quadratic element
//! retains its isochoric bending modes where the constant-strain element loses
//! them, which is the claim the Lamé oracle structurally could not test.
//!
//! ★ **The cost is real and is not visible on the sphere.** Tet10 at ν = 0.49
//! needs **272 Newton iterations** here against Tet4's 7, and a first run of
//! this gate hit `fbar_locking`'s 150-iteration budget and read as a stall. It
//! is not a stall — the solve converges to `1.7e-10` — but plan §6's warning
//! that "ν = 0.49 is a convergence risk before it is an accuracy risk" is
//! *partly* vindicated here even though the sphere converged in 3 iterations at
//! every ν. Anything that drives near-incompressible Tet10 in bending must
//! budget for it. The shipped absolute `tol = 1e-10` is also unusable at this
//! ν (it sits at the residual floor), exactly as in the Lamé harness.
//!
//! ## Profile cost
//!
//! Measured, not inherited: **4.49 s debug against 3.83 s release (1.17×)** —
//! the workspace pins `opt-level = 2` on `[profile.dev]` and `[profile.test]`,
//! so the "~30× slower in debug" rule of thumb does not apply. This file is
//! therefore NOT release-gated and needs no hand-registration in
//! `quality-gate.yml`; it is auto-discovered and run by `tests-debug`, which
//! cannot silently go dark the way an unregistered release-only test can.

// Saint-Venant-averaged tip deflection (`Σ z / N`) and the per-vertex load
// split (`F / N`) cast the loaded-vertex count to `f64` — the canonical FEM
// end-load idiom, mirroring `fbar_locking.rs`.
#![allow(clippy::cast_precision_loss)]

use sim_ml_chassis::Tensor;
use sim_soft::element::Tet10;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, CpuTet10NHSolver, HandBuiltTetMesh,
    LoadAxis, MaterialField, Mesh, NullContact, Solver, SolverConfig, Tet4, Tet10Mesh, VertexId,
    pick_vertices_by_predicate,
};

// ── Scene constants, mirroring `fbar_locking.rs` exactly ─────────────────

const MU: f64 = 1.0e5;
const LENGTH: f64 = 0.5;
const BREADTH: f64 = 0.1;
const HEIGHT: f64 = 0.1;
const TIP_FORCE_TOTAL: f64 = 1.0;
const STATIC_DT: f64 = 1.0;

/// Low Poisson ratio — both elements are (nearly) unlocked here; the reference.
const NU_LO: f64 = 0.30;
/// Ecoflex 00-30's real Poisson ratio — deep in the locked regime for plain
/// Tet4, and the ν rung 6c decides.
const NU_HI: f64 = 0.49;

/// Newton budget. **★ Measured, and the headline robustness finding of this
/// file: Tet10 at ν = 0.49 needs 272 iterations here, against Tet4's 7.**
/// `fbar_locking`'s 150 — enough for every Tet4 configuration — is NOT enough
/// for near-incompressible Tet10 in bending, and a first run of this gate hit
/// that cap and read as a stall. It is not a stall: the solve converges, it is
/// just ~40× more expensive in iterations. 500 leaves ~1.8× headroom.
/// [`MAX_EXPECTED_NEWTON_ITER`] is the bound that actually discriminates.
const MAX_NEWTON_ITER: usize = 500;

/// Iteration count above which the convergence regime has changed. Measured:
/// 7 (Tet4, both ν), 19 (Tet10 ν = 0.30), 272 (Tet10 ν = 0.49).
const MAX_EXPECTED_NEWTON_ITER: usize = 350;

/// Convergence tolerance as a fraction of the applied load's L2 norm.
///
/// The shipped absolute `SolverConfig::skeleton` tolerance (`1e-10`) is the
/// same trap `tet10_lame_decision.rs` documents: it sits at or below the
/// ν = 0.49 Tet10 residual floor, so the solve cannot reach it. Scaling to the
/// load makes the criterion mean the same thing for every configuration.
const TOL_RELATIVE: f64 = 1e-9;

// ── Committed measurements ───────────────────────────────────────────────

/// Ideal ν-ratio `(1+ν_lo)/(1+ν_hi)` — the beam barely stiffens with ν.
const PHYSICAL_RATIO: f64 = 0.8725;

/// **Plain Tet4's ν-ratio — the locking signature this oracle must show for
/// the Tet10 reading to mean anything.** A control that does not collapse
/// would make this file as uninformative as the Lamé oracle turned out to be.
const TET4_NU_RATIO: f64 = 0.2341;

/// **Tet10's ν-ratio — the rung-6 bending evidence.** Committed with the same
/// ±5 % relative band convention as the Lamé file.
const TET10_NU_RATIO: f64 = 0.8058;

/// Band half-width as a fraction of the committed value.
const BAND_FRACTION: f64 = 0.05;

// ── Harness ──────────────────────────────────────────────────────────────

/// Which element to solve the cantilever with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ElementOrder {
    Tet4,
    Tet10,
}

fn lambda_from_nu(nu: f64) -> f64 {
    2.0 * MU * nu / (1.0 - 2.0 * nu)
}

/// Single static `replay_step` on the uniform `(8,2,2)` cantilever at Poisson
/// ratio `nu`. Returns the Saint-Venant-averaged tip `z`-deflection over the
/// `x = LENGTH` face.
///
/// The clamped and loaded bands are selected by an exact-plane predicate, which
/// generalises to Tet10 without change: a midside node on the `x = 0` face is
/// the midpoint of two `x = 0` corners and so lies exactly on the plane, while
/// a midside on an edge running into the beam does not. So the quadratic mesh
/// gets its face midsides clamped and loaded, and its interior midsides free,
/// with no Tet10-specific band logic.
fn tip_deflection(order: ElementOrder, nu: f64) -> f64 {
    let field = MaterialField::uniform(MU, lambda_from_nu(nu));
    let tet4_mesh =
        HandBuiltTetMesh::cantilever_bilayer_beam(8, 2, 2, LENGTH, BREADTH, HEIGHT, &field);
    let tet10_mesh = Tet10Mesh::from_tet4(&tet4_mesh);

    let mesh: &dyn Mesh = match order {
        ElementOrder::Tet4 => &tet4_mesh,
        ElementOrder::Tet10 => &tet10_mesh,
    };
    let pinned: Vec<VertexId> = pick_vertices_by_predicate(mesh, |p| p.x.abs() < 1e-9);
    let loaded: Vec<VertexId> = pick_vertices_by_predicate(mesh, |p| (p.x - LENGTH).abs() < 1e-9);
    assert!(
        !pinned.is_empty() && !loaded.is_empty(),
        "the clamped and tip bands must be non-empty"
    );

    let positions = mesh.positions();
    let n_dof = 3 * positions.len();
    let mut x_prev_flat = vec![0.0; n_dof];
    for (v, pos) in positions.iter().enumerate() {
        x_prev_flat[3 * v] = pos.x;
        x_prev_flat[3 * v + 1] = pos.y;
        x_prev_flat[3 * v + 2] = pos.z;
    }
    let loaded_rest_z: Vec<(VertexId, f64)> = loaded
        .iter()
        .map(|&v| (v, positions[v as usize].z))
        .collect();

    let x_prev = Tensor::from_slice(&x_prev_flat, &[n_dof]);
    let v_prev = Tensor::zeros(&[n_dof]);
    // Equal per-node split of the tip force: inconsistent for a quadratic
    // face, but ν-independent, so it cancels in the ratio (module docs).
    let theta = Tensor::from_slice(&[TIP_FORCE_TOTAL / loaded.len() as f64], &[1]);

    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::AxisZ)).collect(),
    };
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = MAX_NEWTON_ITER;
    cfg.tol = TOL_RELATIVE * TIP_FORCE_TOTAL / (loaded_rest_z.len() as f64).sqrt();

    let step = match order {
        ElementOrder::Tet4 => {
            let solver: CpuTet4NHSolver<HandBuiltTetMesh> =
                CpuNewtonSolver::new(Tet4, tet4_mesh, NullContact, cfg, bc);
            solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt)
        }
        ElementOrder::Tet10 => {
            let solver: CpuTet10NHSolver<Tet10Mesh> =
                CpuNewtonSolver::new(Tet10, tet10_mesh, NullContact, cfg, bc);
            solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt)
        }
    };

    eprintln!(
        "rung-6 bending {order:?} ν={nu}: newton iters = {i}, residual = {r:e}, tol = {t:e}",
        i = step.iter_count,
        r = step.final_residual_norm,
        t = cfg.tol,
    );
    assert!(
        step.iter_count <= MAX_EXPECTED_NEWTON_ITER,
        "{order:?} at ν={nu} took {i} Newton iterations against an expected 272 worst case — the \
         solve succeeded, but the convergence regime changed; investigate before widening",
        i = step.iter_count,
    );
    let sum: f64 = loaded_rest_z
        .iter()
        .map(|&(v, rest_z)| step.x_final[3 * v as usize + 2] - rest_z)
        .sum();
    sum / loaded_rest_z.len() as f64
}

/// ν-ratio `δ(ν_hi)/δ(ν_lo)` for one element — the offset-free locking metric.
fn nu_ratio(order: ElementOrder) -> f64 {
    let lo = tip_deflection(order, NU_LO);
    let hi = tip_deflection(order, NU_HI);
    assert!(
        lo > 0.0 && hi > 0.0,
        "{order:?}: the tip must deflect along the applied +z load at both ν (got lo = {lo:e}, \
         hi = {hi:e})"
    );
    eprintln!(
        "rung-6 bending {order:?}: δ_lo = {lo:.4e}, δ_hi = {hi:.4e}, ratio = {r:.4}",
        r = hi / lo
    );
    hi / lo
}

fn assert_committed(label: &str, measured: f64, committed: f64) {
    let band = BAND_FRACTION * committed;
    assert!(
        (measured - committed).abs() < band,
        "{label}: committed {committed:.4} but measured {measured:.4} (band ±{band:.4})"
    );
}

// ── The gate ─────────────────────────────────────────────────────────────

#[test]
fn tet10_does_not_lock_in_bending_where_tet4_does() {
    // The locking-sensitive companion to the rung-6c Lamé verdict, on an
    // oracle whose control provably DOES lock.
    let tet4 = nu_ratio(ElementOrder::Tet4);
    let tet10 = nu_ratio(ElementOrder::Tet10);
    eprintln!("rung-6 bending: physical ≈ {PHYSICAL_RATIO:.4}, Tet4 {tet4:.4}, Tet10 {tet10:.4}");

    // 1. THE CONTROL MUST LOCK. Without this the oracle is as uninformative as
    //    the sphere turned out to be, and the Tet10 reading means nothing.
    //    `fbar_locking.rs` pins the same collapse at the same threshold.
    assert!(
        tet4 < 0.45,
        "the control did not lock: plain Tet4's ν-ratio {tet4:.4} did not collapse below 0.45 \
         against a physical {PHYSICAL_RATIO:.4}. This oracle is then NOT locking-sensitive and \
         cannot certify anything about Tet10 — do not weaken this bound, find out why the beam \
         stopped locking."
    );

    // 2. Tet10 must NOT collapse. This is the claim the Lamé oracle could not
    //    support: at ν = 0.49, in bending, the quadratic element retains its
    //    isochoric modes where the constant-strain element loses them.
    assert!(
        tet10 > 0.60,
        "Tet10 shows volumetric locking in bending: ν-ratio {tet10:.4} collapsed below 0.60 \
         against a physical {PHYSICAL_RATIO:.4}. Rung 6c's ACCEPT was decided on a \
         locking-INSENSITIVE oracle; if Tet10 locks here, that verdict does not generalise and \
         Taylor-Hood P2-P1 is back on the table."
    );

    // 3. And it must be a large margin over the control, not a nudge.
    assert!(
        tet10 > 2.0 * tet4,
        "Tet10's ν-ratio {tet10:.4} is not decisively better than Tet4's {tet4:.4}"
    );

    assert_committed("Tet4 ν-ratio", tet4, TET4_NU_RATIO);
    assert_committed("Tet10 ν-ratio", tet10, TET10_NU_RATIO);
}
