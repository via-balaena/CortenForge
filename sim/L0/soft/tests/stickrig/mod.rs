//! **The stick.** One definition of the fixture's physical identity, shared by
//! every stick measurement so the numbers can be read together.
//!
//! Per Rust integration-test convention each `tests/<name>.rs` is its own crate,
//! so this module is compiled into each file that declares `mod stickrig;` and
//! into no other. It holds the stick's *identity* — the two free inputs, what
//! derives from them, and the mesh/BC/config construction — and deliberately
//! **not** the measurement loops, which differ per fixture.
//!
//! ★ **Why shared rather than duplicated.** `stick_flex.rs` measures
//! discretisation error and per-frame cost; `stick_impact.rs` measures
//! convergence and `p99` under a puck strike. Those two are only readable
//! against each other if they are the *same stick* — the same `EI`, the same
//! section, the same clamped and loaded bands. Duplicated constants can drift
//! silently and would make a cross-fixture claim unfalsifiable. This is the
//! reasoning `tests/reduced_report/` was extracted under, applied to geometry
//! instead of arithmetic.
//!
//! ⛔ **`slender_bending_matches_analytic.rs` is deliberately NOT a consumer.**
//! Its beam is a different section at a different modulus, and folding it in
//! means showing its numbers do not move — a separate change with its own
//! verification burden.

// Loaded-vertex counts cast to `f64` for the per-vertex load split and the
// analytic comparison — the FEM idiom shared with
// `slender_bending_matches_analytic.rs` and `fbar_locking.rs`, not a
// precision-sensitive path.
#![allow(clippy::cast_precision_loss)]
// `expect` on a value just established to be present. The convention across
// this crate's tests: in a test, a violated invariant should abort loudly with
// its message rather than be threaded through a `Result` no caller can act on.
#![allow(clippy::expect_used)]
// Each declaring crate compiles this whole module but uses only the helpers it
// needs, so every helper is dead code in some binary — the standard
// `tests/common` friction, and the same allowance that module carries.
#![allow(dead_code)]

use sim_ml_chassis::Tensor;
use sim_soft::mesh::HandBuiltTetMesh;
use sim_soft::{
    BoundaryConditions, LoadAxis, MaterialField, Mesh, NewtonStep, Solver, SolverConfig,
    SolverFailure, Tet10Mesh, VertexId, pick_vertices_by_predicate,
};

/// Nearest-rank order statistic: the `q`-quantile of `values`, taken as
/// `values_sorted[⌈q·n⌉ − 1]`, saturating so `q = 1.0` is the maximum.
///
/// ★★ **Shared so instruments cannot drift on the arithmetic that decides a
/// rung** — the reason `tests/reduced_report/` exists, applied to summary
/// statistics. `stick_flex.rs` reads a `p50` off a 20-frame cost sample and
/// `stick_impact.rs` a `p99` off a 300-frame one, and `InitialGuess`'s docs
/// quote both beside `predictor_spike.rs`. If those disagreed about what a
/// percentile *is*, none of the three could be quoted next to the others.
///
/// ⚠⚠ **This definition is not free-standing — it is `predictor_spike.rs`'s
/// `rank_index`, deliberately.** That file is the sibling instrument on the same
/// subject (the `InitialGuess` predictor, per-step iterations and `ms`), and it
/// got here first. An earlier version of this function used `⌊q·n⌋`, which
/// differs by exactly one order statistic at every intermediate quantile
/// (`n = 20, q = 0.5` → index 10 against 9; `n = 300, q = 0.99` → 297 against
/// 296) — so the extraction that was justified by "two fixtures cannot drift"
/// shipped already drifting against the nearest fixture of all. If either
/// definition changes, change both.
///
/// ⛔ The `⌊q·n⌋` form carried a rationale that it "biases the reported cost up,
/// the conservative direction for a claim that something FITS a budget". That
/// argument does not survive: this file's headline claim is that a `p99` BUSTS a
/// budget, for which the same bias is anti-conservative. A percentile should be
/// the standard one and the conservatism argued separately.
///
/// ⚠ Panics on an empty sample rather than returning a zero that would read as
/// "instant", and rejects a `q` outside `[0, 1]` rather than saturating — a
/// quantile silently clamped is how a `p99` stops being a `p99`.
pub fn percentile(values: &[f64], q: f64) -> f64 {
    assert!(
        !values.is_empty(),
        "no samples: there is nothing to summarise"
    );
    assert!(
        (0.0..=1.0).contains(&q),
        "quantile {q} is outside [0, 1] — clamping it would silently report a different statistic"
    );
    let mut v = values.to_vec();
    v.sort_by(|a, b| a.partial_cmp(b).expect("timings are never NaN"));
    v[rank_index(v.len(), q)]
}

/// Nearest-rank index into a length-`n` sorted sample for quantile `q`.
///
/// Byte-for-byte the rule in `predictor_spike.rs::rank_index`; see
/// [`percentile`] for why that matters.
fn rank_index(n: usize, q: f64) -> usize {
    // `q` is in `[0, 1]` and `n >= 1` (both asserted by the sole caller), so
    // `⌈q·n⌉` is in `[0, n]` — inside `usize` and inside `f64`'s integer range.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let rank = (q * n as f64).ceil() as usize;
    rank.max(1).min(n) - 1
}

// ---------------------------------------------------------------------------
// The fixture. Two free inputs (`EI_TARGET`, `MASS_PER_LENGTH`); the rest derives.
// ---------------------------------------------------------------------------

/// Free cantilever span (m) — the shaft below the lower hand.
pub const SPAN: f64 = 1.20;
/// Section width (m), side-to-side. Bending is *not* about this axis.
pub const WIDTH: f64 = 0.019;
/// Section depth (m), front-to-back. **This is the flex direction**, and the
/// mesh axis whose resolution decides whether bending is captured.
pub const DEPTH: f64 = 0.029;

/// Bending stiffness (N·m²) of an 85-flex senior shaft. See the module docs for
/// the reading this comes from, and for the 2.8× spread across readings.
pub const EI_TARGET: f64 = 310.0;
/// Mass per unit length (kg/m) — a ~400 g stick over ~1.6 m.
pub const MASS_PER_LENGTH: f64 = 0.25;

/// Poisson ratio of a composite layup.
pub const NU: f64 = 0.30;

/// Driven tip deflection as a fraction of span. Small enough that
/// Euler–Bernoulli is exact to well under a percent — see the module docs.
pub const TARGET_RATIO: f64 = 1.0e-3;

/// Quasi-static timestep. ⚠ Inert on its own: what makes the step static is
/// [`STATIC_DENSITY`] being zero, which removes `M(x − x₀)/dt²` outright. A `dt`
/// alone does not make a step static — that mistake cost
/// `slender_bending_matches_analytic.rs` a whole false ceiling.
const STATIC_DT: f64 = 1.0;

/// ★ Zero, because this is a static benchmark against a static closed form.
pub const STATIC_DENSITY: f64 = 0.0;

/// Second moment of area of the **solid** twin (m⁴), about the bending axis.
fn i_solid() -> f64 {
    WIDTH * DEPTH * DEPTH * DEPTH / 12.0
}

/// Cross-sectional area of the solid twin (m²).
fn area() -> f64 {
    WIDTH * DEPTH
}

/// Effective Young's modulus (Pa) that lands the solid twin on [`EI_TARGET`].
pub fn e_eff_for(ei: f64) -> f64 {
    ei / i_solid()
}

/// Effective density (kg/m³) that lands the solid twin on [`MASS_PER_LENGTH`].
pub fn rho_eff() -> f64 {
    MASS_PER_LENGTH / area()
}

/// Lamé pair for a given effective modulus.
pub fn lame_for(e: f64) -> (f64, f64) {
    let mu = e / (2.0 * (1.0 + NU));
    let lambda = 2.0 * mu * NU / (1.0 - 2.0 * NU);
    (mu, lambda)
}

/// Tip load (N) that lands a cantilever of stiffness `ei` on [`TARGET_RATIO`].
///
/// Inverts `δ = P L³ / (3 E I)`. Loading to a fixed *deflection ratio* rather
/// than a fixed force keeps rows comparable across stiffnesses.
pub fn load_for(ei: f64) -> f64 {
    TARGET_RATIO * SPAN * 3.0 * ei / (SPAN * SPAN * SPAN)
}

/// Analytic first bending frequency (Hz) of the fixture, `f₁ =
/// (1.875²/2π)·√(EI / (m′L⁴))`. Reported for scale, and as the quantity the
/// per-row stiffness error propagates into.
pub fn f1_analytic(ei: f64) -> f64 {
    const BETA_L_SQ: f64 = 1.875_104_068_711_961 * 1.875_104_068_711_961;
    let l4 = SPAN * SPAN * SPAN * SPAN;
    BETA_L_SQ / (2.0 * std::f64::consts::PI) * (ei / (MASS_PER_LENGTH * l4)).sqrt()
}

/// Which element (and cure) an arm runs.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Arm {
    Tet4,
    Tet4Fbar,
    Tet10,
}

impl Arm {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Tet4 => "Tet4",
            Self::Tet4Fbar => "Tet4+F-bar",
            Self::Tet10 => "Tet10",
        }
    }
}

/// Boundary conditions, initial state and config for one cantilever solve.
///
/// ⚠⚠ **The tolerance is piloted, and the first pilot was unreachable.** At
/// `1e-6·load/√n` — the factor `slender_bending_matches_analytic.rs` uses —
/// **17 of 30 cells stalled Armijo**, every one of them at the right answer:
/// Tet10 read `0.978–0.997` of analytic and then failed, with `r/tol` between
/// `1.1` and `9.1`. That rig's beam is `E = 2.7e5 Pa`; this stick is
/// `E_eff = 8.0e9`, **~30 000× stiffer**, so its assembled internal forces are
/// ~`10²  N` while `1e-6·load/√n` asks for a residual of `1.7e-7 N` — a relative
/// precision of `~1e-9` on a summed quantity, which is the f64 floor, not a
/// convergence criterion. `1e-4` puts the demand at `~1e-7` relative, reachable
/// with three orders of margin over the `1e-4`-relative accuracy this file
/// reports. ★ The relaxation is **not** taken on trust — see
/// `stick_flex.rs::the_verdict_does_not_depend_on_the_tolerance`.
///
/// Provenance: re-derived from `slender_bending_matches_analytic.rs` for this
/// section and load. It lives here — rather than duplicated per fixture —
/// because `stick_flex.rs` and `stick_impact.rs` are only readable against each
/// other if they clamp and load the *same* stick; see the module docs.
pub fn rig<M: Mesh>(
    mesh: &M,
    load: f64,
    density: f64,
    fbar: bool,
    tol_rel: f64,
) -> (Vec<f64>, Vec<f64>, BoundaryConditions, SolverConfig) {
    let n_dof = 3 * mesh.n_vertices();
    let pinned: Vec<VertexId> = pick_vertices_by_predicate(mesh, |p| p.x.abs() < 1e-9);
    let loaded: Vec<VertexId> = pick_vertices_by_predicate(mesh, |p| (p.x - SPAN).abs() < 1e-9);
    assert!(
        !pinned.is_empty() && !loaded.is_empty(),
        "clamped and tip bands must both be non-empty or the rig measures nothing"
    );

    let positions = mesh.positions();
    let mut x_flat = vec![0.0; n_dof];
    for (v, p) in positions.iter().enumerate() {
        x_flat[3 * v] = p.x;
        x_flat[3 * v + 1] = p.y;
        x_flat[3 * v + 2] = p.z;
    }
    let rest_z: Vec<f64> = loaded.iter().map(|&v| positions[v as usize].z).collect();

    // ⚠⚠ `gravity_z` is left at `skeleton()`'s `0.0`, so this "physical
    // identity" rig carries NO self-weight — the stick is weightless and only
    // moves when something drives it. Deliberate, and load-bearing twice over:
    // every deflection reported against the analytic cantilever is a response to
    // the applied load alone, and `stick_impact.rs`'s unstruck control asserts
    // the quiet run deflects less than `1e-12`, which self-weight would break
    // outright (a horizontal 1.2 m shaft sags to `d/L ~ 1.7e-3` under its own
    // mass). Enabling gravity here is not a knob — it changes what both fixtures
    // measure.
    let mut cfg = SolverConfig::skeleton();
    cfg.dt = STATIC_DT;
    cfg.density = density;
    cfg.fbar = fbar;
    cfg.max_newton_iter = 500;
    cfg.tol = tol_rel * load / (loaded.len() as f64).sqrt();

    let bc = BoundaryConditions {
        pinned_vertices: pinned,
        roller_vertices: Vec::new(),
        loaded_vertices: loaded.iter().map(|&v| (v, LoadAxis::AxisZ)).collect(),
    };
    (x_flat, rest_z, bc, cfg)
}

/// Peak tip deflection of the loaded band, relative to rest.
pub fn tip_of(x_final: &[f64], loaded: &[(VertexId, LoadAxis)], rest_z: &[f64]) -> f64 {
    loaded
        .iter()
        .zip(rest_z)
        .map(|(&(v, _), &z0)| (x_final[3 * v as usize + 2] - z0).abs())
        .fold(0.0f64, f64::max)
}

/// Relative Newton tolerance, as a multiple of `load/√n_loaded`. Piloted — see
/// [`rig`] for the reading that set it and for the one that did not work.
pub const TOL_REL: f64 = 1.0e-4;

/// Unwrap a step into `(position, failure diagnostics)`.
///
/// ★ The failure arm keeps `x_partial` rather than discarding it. Reading the
/// position a failed solve reached is what separates "the tolerance was
/// unreachable" from "the solve went somewhere wrong", and those two demand
/// opposite fixes.
pub fn outcome_of<T>(
    step: Result<NewtonStep<T>, SolverFailure>,
    tol: f64,
    rest: &[f64],
) -> (Vec<f64>, usize, Option<(&'static str, f64, f64, usize)>) {
    match step {
        Ok(s) => (s.x_final, s.iter_count, None),
        Err(SolverFailure::NewtonIterCap {
            x_partial,
            max_iter,
            last_r_norm,
        }) => (
            x_partial,
            max_iter,
            Some(("iter-cap", last_r_norm, tol, max_iter)),
        ),
        Err(SolverFailure::ArmijoStall {
            x_partial,
            last_iter,
            last_r_norm,
        }) => (
            x_partial,
            last_iter,
            Some(("armijo-stall", last_r_norm, tol, last_iter)),
        ),
        // No residual is meaningful once the factor has failed, and
        // `ValidityViolation` carries no position at all, so both report a
        // `NaN` residual rather than a `0.0` that would read as "converged".
        Err(SolverFailure::DoublyFailedFactor {
            x_partial,
            last_iter,
            ..
        }) => (
            x_partial,
            last_iter,
            Some(("factor-failed", f64::NAN, tol, last_iter)),
        ),
        Err(SolverFailure::ValidityViolation { tet_id, .. }) => {
            (rest.to_vec(), 0, Some(("validity", f64::NAN, tol, tet_id)))
        }
    }
}

/// Free DOF a `(arm, nx, ny, nz)` cell would carry, **without solving it**.
///
/// The sizing instrument: a Tet10 cell carries roughly `8×` a Tet4 cell's nodes,
/// and the sweep has to be designed around that before any solve time is spent.
pub fn free_dof_of(arm: Arm, nx: usize, ny: usize, nz: usize) -> usize {
    let field = MaterialField::uniform(1.0, 1.0);
    let tet4 = HandBuiltTetMesh::cantilever_bilayer_beam(nx, ny, nz, SPAN, WIDTH, DEPTH, &field);
    match arm {
        Arm::Tet10 => {
            let mesh = Tet10Mesh::from_tet4(&tet4);
            let pinned = pick_vertices_by_predicate(&mesh, |p| p.x.abs() < 1e-9).len();
            3 * (mesh.n_vertices() - pinned)
        }
        Arm::Tet4 | Arm::Tet4Fbar => {
            let pinned = pick_vertices_by_predicate(&tet4, |p| p.x.abs() < 1e-9).len();
            3 * (tet4.n_vertices() - pinned)
        }
    }
}

/// Tip deflection a slapshot drives (m) — `δ/L ≈ 0.083`, deeply nonlinear, and
/// the regime that sets the Newton iteration count. ⚠ The accuracy sweep runs
/// at `δ/L = 1e-3` because that is where a closed form exists; **this is the
/// other regime**, and the two must not be read as one measurement.
pub const SLAPSHOT_DEFLECTION: f64 = 0.10;

/// Frames over which the slapshot load is ramped in before measuring.
///
/// ⚠⚠ **Piloted, and the pilot is a finding.** Applied as a *step* at frame 0,
/// the full load does not converge at all — 500 iterations with `r_norm` stuck
/// at a fixed ~25 % of the load, for either element, at any `dt` and either
/// predictor (`stick_flex.rs::where_the_dynamic_rig_loses_convergence`). Ramped,
/// it clears:
/// **Tet4 needs 6 frames, Tet10 needs 60** — the quadratic element is `10×` less
/// tolerant of an abrupt load change
/// (`stick_flex.rs::whether_load_continuation_clears_the_wall`). `60` is the
/// value both arms
/// clear, so both are measured in the same regime.
///
/// It doubles as warmup: first-touch page faults and the symbolic factorisation
/// happen inside it and no measured frame repeats them.
pub const RAMP_FRAMES: usize = 60;
/// Tip load (N) for the slapshot regime.
pub fn slapshot_load(ei: f64) -> f64 {
    3.0 * ei * SLAPSHOT_DEFLECTION / (SPAN * SPAN * SPAN)
}

/// One frame, timed. `Err` **carries why** — a failed frame ends the run rather
/// than contributing a fast-because-it-gave-up sample, and a run that ends
/// without saying why costs another build cycle to diagnose.
pub fn one_frame<S: Solver>(
    solver: &S,
    x: &mut Vec<f64>,
    v: &mut Vec<f64>,
    theta: &Tensor<f64>,
    dt: f64,
) -> Result<(usize, f64), String> {
    let n_dof = x.len();
    let x_in = Tensor::from_slice(x, &[n_dof]);
    let v_in = Tensor::from_slice(v, &[n_dof]);
    let t0 = std::time::Instant::now();
    let step = solver
        .try_replay_step(&x_in, &v_in, theta, dt)
        .map_err(|e| describe_failure(&e))?;
    let ms = t0.elapsed().as_secs_f64() * 1e3;
    if !step.x_final.iter().all(|f| f.is_finite()) {
        return Err("non-finite x_final".to_owned());
    }
    *v = step
        .x_final
        .iter()
        .zip(x.iter())
        .map(|(xf, xp)| (xf - xp) / dt)
        .collect();
    x.clone_from(&step.x_final);
    Ok((step.iter_count, ms))
}

/// Stable leading tokens of [`describe_failure`]'s output, one per
/// [`SolverFailure`] variant.
///
/// ★ Shared with the classifier below rather than written twice: a gate that
/// wants "did this fail for the reason I claim?" must not re-spell the prefix
/// its producer emits, or the two drift and the classifier silently answers
/// `false` forever.
pub const FAIL_ITER_CAP: &str = "iter-cap";
pub const FAIL_ARMIJO: &str = "armijo-stall";
pub const FAIL_FACTOR: &str = "factor-failed";
pub const FAIL_VALIDITY: &str = "validity-violation";

/// Whether a failure reason from [`describe_failure`] is Newton failing to
/// CONVERGE, as opposed to the step dying some other way.
///
/// ⚠⚠ **The distinction is the whole point of several gates.** `Run::completed()`
/// is `false` for an iteration cap, an Armijo stall, a failed factorisation, an
/// inverted element and a non-finite state alike. A gate asserting only
/// `!completed()` to prove "the convergence wall is intact" therefore reports
/// success when an element inverts instead — a different defect entirely,
/// demanding the opposite fix.
pub fn is_convergence_failure(reason: &str) -> bool {
    // `describe_failure` prefixes the token; the frame loops prepend "frame N: ".
    let body = reason.split_once(": ").map_or(reason, |(_, rest)| rest);
    body.starts_with(FAIL_ITER_CAP) || body.starts_with(FAIL_ARMIJO)
}

/// A one-line reason a step failed, for the run-ended-early report.
pub fn describe_failure(e: &SolverFailure) -> String {
    match e {
        SolverFailure::NewtonIterCap {
            max_iter,
            last_r_norm,
            ..
        } => format!("{FAIL_ITER_CAP} at {max_iter}, r_norm {last_r_norm:.3e}"),
        SolverFailure::ArmijoStall {
            last_iter,
            last_r_norm,
            ..
        } => format!("{FAIL_ARMIJO} at iter {last_iter}, r_norm {last_r_norm:.3e}"),
        SolverFailure::DoublyFailedFactor {
            last_iter, context, ..
        } => format!("{FAIL_FACTOR} at iter {last_iter}: {context}"),
        SolverFailure::ValidityViolation { tet_id, message } => {
            format!("{FAIL_VALIDITY}, tet {tet_id}: {message}")
        }
    }
}
