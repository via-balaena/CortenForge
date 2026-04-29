//! V-6 — empirical κ-ceiling stability scan.
//!
//! Phase 5 scope memo §1 V-6 + §8 commit 10 (`phase_5_penalty_contact_scope.md`).
//! Documents the empirically-determined `κ_pen` ceiling above which Newton
//! diverges for a representative contact-active scene at the Phase-5 dev
//! machine, and verifies that the production default `PENALTY_KAPPA_DEFAULT
//! = 1e4 N/m` sits at least one order of magnitude below the ceiling.
//!
//! ## Material plan change vs scope memo §1 V-6
//!
//! Scope memo §1 V-6 prescribes a Hessian condition-number gate
//! (`cond < 1e10` at every Newton iter for V-3 / V-3a / V-5 scenes).
//! `CpuNewtonSolver` does not expose tangent eigenvalues today —
//! adding eigenvalue extraction is non-trivial plumbing (faer's
//! `SymbolicLlt` doesn't surface a condition-number estimate; a
//! separate eigensolver call is needed) and lives outside Phase 5's
//! "first-time wiring" axis. Instead V-6 ships an **empirical
//! Newton-convergence + monotonic-residual proxy gate**:
//!
//! - Run a single quasi-static Newton step at each `κ` in a logarithmic
//!   scan from `1e3` to `1e7` against a fixed reference scene.
//! - Treat the run as **convergent** iff `replay_step` does not panic
//!   AND `final_residual_norm < cfg.tol`.
//! - Treat the run as **failed** iff Newton exceeds `max_newton_iter`,
//!   Armijo line-search stalls, the tangent fails Cholesky
//!   (`NonPositivePivot`), or any other panic surfaces.
//! - Define the empirical ceiling as the largest `κ` that converged in
//!   monotonic-residual fashion (the solver's Armijo backtracking
//!   enforces residual decrease per iter — failure manifests as
//!   panic, not as "converged at high residual").
//!
//! Phase H IPC tightens this to a proper condition-number gate when the
//! solver gains tangent-eigenvalue instrumentation; the proxy gate here
//! is a Phase 5 hygiene posture, not a substitute for the long-term
//! observability story.
//!
//! ## Scene choice — V-3a override regime
//!
//! V-6 runs the scan against the V-3a compressive-block scene at its
//! commit-8 working override regime (`d̂ = 1e-5 m`, `δ = 5e-5 m`,
//! `n_per_edge = 4`). Two reasons:
//!
//! 1. **Cheapest contact-active scene** — quasi-static Newton on a
//!    `~384`-tet cube at `STATIC_DT = 1.0` converges in `~3-5` iters
//!    release-mode in `~25 ms`. A 5-point `κ` scan completes in well
//!    under a second per `feedback_release_mode_heavy_tests`.
//! 2. **Well-understood failure regime** — V-3a commit-8 surfaced
//!    cold-start tet inversion at `(κ = 1e4, d̂ = 1e-3, δ = 5e-4)`
//!    where `κ · (d̂ + δ) ≈ 15 N` per top-face vertex. The override
//!    `(d̂, δ)` keeps cold-start residual `κ · (d̂ + δ) ≈ κ · 6e-5 N`
//!    per vertex; ceiling onset is dominated by `κ · (d̂ + δ)`
//!    reaching a tet-inversion threshold scaled by element stiffness
//!    `E·h`. At `μ = 1e5, λ = 4e5, E ≈ 2.8e5 Pa, h = EDGE_LEN/n =
//!    2.5 mm`, that threshold scale is `~E·h ≈ 700 N/m`. Empirically
//!    the ceiling sits between `1e7` and `1e8 N/m` at this scene
//!    (`κ ≤ 1e7` converges in 3 Newton iters; `κ ≥ 1e8` Armijo-stalls).
//!
//! V-3 `sphere_on_plane` and V-5 `dropping_sphere` are also scope-memo-
//! named V-6 targets, but at `~22 s` (V-5) and `~7 s` (V-3 coarsest)
//! per scan point they would push V-6 into the multi-minute regime
//! without diagnostic gain. The V-3a-only choice is documented here
//! and is consistent with the proxy framing.
//!
//! ## Headroom assertion
//!
//! The scope memo's load-bearing claim is *"default κ stays one order of
//! magnitude below the ceiling."* V-6 asserts this directly by walking
//! the κ scan and pinning `highest_converged_kappa ≥ 10 × κ_default`.
//! Failure here surfaces a `(κ_default, h, material)` retune as a
//! Decision-J adjustment (the same authority that allowed V-3a's `d̂`
//! override and V-3's local `κ` override).

#![allow(
    // Helper destructures a 4-tuple (no Result); `expect_used` left
    // enabled for future test additions. Mirrors V-3a precedent.
    clippy::expect_used
)]

use std::panic::{AssertUnwindSafe, catch_unwind};

use sim_ml_chassis::Tensor;
use sim_soft::{
    CpuNewtonSolver, HandBuiltTetMesh, MaterialField, PenaltyRigidContact,
    PenaltyRigidContactSolver, RigidPlane, SceneInitial, SoftScene, Solver, SolverConfig, Tet4,
    Vec3,
};

// ── Reference V-3a scene constants — must track penalty_compressive_block.rs

const EDGE_LEN: f64 = 0.01;
const DISPLACEMENT: f64 = 5.0e-5;
const MU: f64 = 1.0e5;
const LAMBDA: f64 = 4.0e5;
const D_HAT_OVERRIDE: f64 = 1.0e-5;
const STATIC_DT: f64 = 1.0;
const MAX_NEWTON_ITER: usize = 50;
const N_PER_EDGE: usize = 4;

/// Production default penalty stiffness — re-pinned here because the
/// upstream `sim_soft::contact::penalty::PENALTY_KAPPA_DEFAULT` is
/// `pub(crate)`. Phase 5 commit 4 baseline.
const KAPPA_DEFAULT: f64 = 1.0e4;

/// Logarithmic κ scan from `1e3` to `1e10` (one OOM steps). Eight
/// points spans seven decades around the production default to
/// bracket the empirical ceiling at the V-3a regime per the module
/// docstring "Scene choice" section. Empirical result at the Phase-5
/// dev machine: `1e3..=1e7` all converge in 3 Newton iters; `1e8..=1e10`
/// Armijo-stall, locating the ceiling at exactly `κ = 1e7`. Production
/// default `κ = 1e4` therefore sits 1000× (3 OOM) below ceiling — well
/// past the scope memo's 10× headroom requirement.
const KAPPA_SCAN: &[f64] = &[1.0e3, 1.0e4, 1.0e5, 1.0e6, 1.0e7, 1.0e8, 1.0e9, 1.0e10];

/// Required ceiling-headroom multiplier — scope memo §1 V-6's *"default
/// κ stays one order of magnitude below the ceiling"*. Asserted as
/// `highest_converged_kappa >= HEADROOM_MULTIPLIER · KAPPA_DEFAULT`.
const HEADROOM_MULTIPLIER: f64 = 10.0;

#[derive(Debug)]
enum ScanResult {
    Converged { iters: usize, residual: f64 },
    Failed { reason: String },
}

fn material_field() -> MaterialField {
    MaterialField::uniform(MU, LAMBDA)
}

/// Single quasi-static Newton step against the V-3a override scene at
/// `kappa`. Catches solver panics (Newton non-convergence, Armijo
/// stall, `NonPositivePivot`) and returns a structured `ScanResult`
/// rather than propagating. `replay_step` is the tape-free path so
/// `AssertUnwindSafe` is sufficient — the solver's internal state is
/// constructed fresh per call and dropped on panic.
// Cast safe: `N_PER_EDGE` is a small literal; the f64 conversion is
// loss-free. Helper function applies the `clippy::cast_precision_loss`
// allow at one site, used both by the per-κ test runner and the
// diagnostic eprintln after the scan loop.
#[allow(clippy::cast_precision_loss)]
fn cell_size() -> f64 {
    EDGE_LEN / (N_PER_EDGE as f64)
}

fn try_run_at_kappa(kappa: f64) -> ScanResult {
    let cell_size = cell_size();
    let (mesh, bc, initial, _default_contact) =
        SoftScene::compressive_block_on_plane(EDGE_LEN, cell_size, DISPLACEMENT, &material_field());
    let plane = RigidPlane::new(Vec3::new(0.0, 0.0, -1.0), DISPLACEMENT - EDGE_LEN);
    let contact = PenaltyRigidContact::with_params(vec![plane], kappa, D_HAT_OVERRIDE);

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.max_newton_iter = MAX_NEWTON_ITER;

    let SceneInitial { x_prev, v_prev } = initial;
    let solver: PenaltyRigidContactSolver<HandBuiltTetMesh> =
        CpuNewtonSolver::new(Tet4, mesh, contact, cfg, bc);
    let theta = Tensor::from_slice(&[], &[0]);

    let result = catch_unwind(AssertUnwindSafe(|| {
        solver.replay_step(&x_prev, &v_prev, &theta, cfg.dt)
    }));
    match result {
        Ok(step) if step.final_residual_norm < cfg.tol => ScanResult::Converged {
            iters: step.iter_count,
            residual: step.final_residual_norm,
        },
        Ok(step) => ScanResult::Failed {
            reason: format!(
                "residual {res:e} above tol {tol:e} after {iters} iters",
                res = step.final_residual_norm,
                tol = cfg.tol,
                iters = step.iter_count,
            ),
        },
        Err(panic_payload) => {
            let msg = panic_payload
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| {
                    panic_payload
                        .downcast_ref::<&str>()
                        .map(|s| (*s).to_string())
                })
                .unwrap_or_else(|| "<non-string panic payload>".to_string());
            // Trim panic message to first line — Newton stall / Armijo
            // stall / NonPositivePivot panics include multi-line
            // diagnostic prose that adds noise to the eprintln summary.
            let first_line = msg.lines().next().unwrap_or("").to_string();
            ScanResult::Failed { reason: first_line }
        }
    }
}

#[test]
fn v_6_kappa_ceiling_scan_v_3a_regime() {
    let mut results: Vec<(f64, ScanResult)> = Vec::with_capacity(KAPPA_SCAN.len());
    let mut highest_converged: f64 = 0.0;

    for &kappa in KAPPA_SCAN {
        let result = try_run_at_kappa(kappa);
        if matches!(result, ScanResult::Converged { .. }) {
            highest_converged = kappa;
        }
        results.push((kappa, result));
    }

    eprintln!(
        "v_6 κ ceiling scan (V-3a override regime, d̂ = {dhat:e} m, δ = {disp:e} m, n = {n}, \
         μ = {mu:e} Pa, λ = {lam:e} Pa, h = {h:e} m):",
        dhat = D_HAT_OVERRIDE,
        disp = DISPLACEMENT,
        n = N_PER_EDGE,
        mu = MU,
        lam = LAMBDA,
        h = cell_size(),
    );
    for (kappa, result) in &results {
        match result {
            ScanResult::Converged { iters, residual } => {
                eprintln!(
                    "  κ = {kappa:e} N/m  →  CONVERGED in {iters} iters, residual {residual:.3e}",
                );
            }
            ScanResult::Failed { reason } => {
                eprintln!("  κ = {kappa:e} N/m  →  FAILED ({reason})");
            }
        }
    }
    eprintln!(
        "  Empirical ceiling: highest converged κ = {hc:e} N/m \
         (default κ = {kd:e} N/m, headroom = {hm:.1}×)",
        hc = highest_converged,
        kd = KAPPA_DEFAULT,
        hm = if KAPPA_DEFAULT > 0.0 {
            highest_converged / KAPPA_DEFAULT
        } else {
            f64::INFINITY
        },
    );

    // ── Default-κ converges ────────────────────────────────────────────
    //
    // Sanity: the production default (`PENALTY_KAPPA_DEFAULT = 1e4`)
    // must be in the convergent regime. If it isn't, the entire Phase
    // 5 default-κ contract is broken (V-1 / V-3a / V-3 / V-4 / V-5 all
    // assume default κ converges on at least *some* contact-active
    // scene).
    let default_converged = results.iter().any(|(k, r)| {
        (k - KAPPA_DEFAULT).abs() < f64::EPSILON && matches!(r, ScanResult::Converged { .. })
    });
    assert!(
        default_converged,
        "Production default κ = {KAPPA_DEFAULT:e} N/m did not converge on the V-3a override \
         regime. The Phase 5 contract assumes default κ is in the convergent regime — \
         investigate Decision-J retune.",
    );

    // ── One-OOM headroom ──────────────────────────────────────────────
    //
    // Scope memo §1 V-6 load-bearing claim. Failure here means the
    // production default sits at the very top of the convergent
    // range — a small parameter perturbation (cell size, material
    // change, slightly tighter d̂) could push it over. Decision-J
    // retune authority applies; surface as a material plan change at
    // commit time rather than a fix-on-the-fly.
    let required_ceiling = HEADROOM_MULTIPLIER * KAPPA_DEFAULT;
    assert!(
        highest_converged >= required_ceiling,
        "Empirical κ ceiling {highest_converged:e} N/m is below the required \
         {required_ceiling:e} N/m ({HEADROOM_MULTIPLIER:.1}× default = {KAPPA_DEFAULT:e} N/m). \
         The default has less than one OOM of stability headroom on the V-3a regime — \
         Decision-J retune candidate.",
    );
}
