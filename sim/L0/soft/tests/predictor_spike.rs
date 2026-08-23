//! Newton initial-guess (predictor) spike — does extrapolating `x⁰` off
//! `x_prev` cut ITERATION COUNT, and does it ever cost convergence?
//!
//! ## Why this exists
//!
//! `docs/SIM_SOFT_REALTIME_RECON.md` §2a measures a frame as
//! `iterations × per-iteration cost`, and the two factors behave nothing
//! alike: iteration count spans **74×** across fixtures (`block_sag` 0.5–0.6,
//! IPC indentation 6.5, `cantilever 80×8` 37.0) while the whole R0–R6
//! reduced-order ladder moves only the *per-iteration* factor — R1.1 measured
//! the reduced solve taking IDENTICAL iteration counts to the oracle, and
//! substepping measured 1.4× *worse*. So nothing shipped or planned attacks
//! the first factor at all.
//!
//! A velocity-extrapolated initial guess is the standard lever on that factor
//! and had never been tried here. It runs ahead of R3 because it is cheap, it
//! benefits the **full-order oracle too** (R3 helps only the fast path), and
//! its result **rescopes R3**: whatever iteration-count gain lands here comes
//! straight off the 12–16× per-iteration gain R3 would otherwise have to find.
//!
//! ## The matrix (five cells fixed before the first run; a sixth added after review)
//!
//! | knob | value | why |
//! |---|---|---|
//! | fixture | `cantilever` · IPC `dynamic_indentation` · `block_sag` | largest expected win · the representative workload · **negative control** |
//! | size | cantilever 3 000 + 19 440 free DOF; IPC 5 202 + 18 750; block 1 944 | two per subject — iteration count grows with refinement (recon §3b.4), so the gain may be size-dependent |
//! | guess | [`InitialGuess::PreviousState`] · [`InitialGuess::Inertial`] · [`InitialGuess::InertialWithLoad`] | baseline · the memo's `x + Δt·v` · the incremental-potential predictive position |
//! | `dt` | `1/60` | the frame budget being priced |
//! | steps | cantilever + block 60 (1 s) · IPC 71 (the full ramp) | past the startup transient |
//! | [`SolverConfig::max_newton_iter`] | 120 | recon's 60-cap aborted a case needing 65 (§3d); ~2× the observed max of 56, so a predictor blowup is VISIBLE rather than truncated |
//! | [`SolverConfig::tol`] | `1e-10`, unchanged | a predictor must not be allowed to buy iterations by converging less |
//! | repeats | none | see "on repeats" below |
//! | **body load** | **`0` · `±9.81` on IPC** | ⚠ **ADDED AFTER THE FIRST RUN, and the omission was the matrix's worst defect** — see below |
//!
//! ⚠ **The `load × contact` cell was missing, and the harness's own self-check
//! hid it.** With `gravity_z = 0` and an empty θ the IPC fixture has `f_ext ≡ 0`,
//! so `Inertial` and `InertialWithLoad` are the same point there — designed in as
//! a self-check, and duly passed. What that identity *also* means is that the only
//! contact fixture in the matrix **could not test the term separating the two
//! arms**. Every "these must agree exactly" self-check is simultaneously a
//! coverage-hole report; this one went unread until review. Measuring the missing
//! cell killed `InertialWithLoad` outright (step-0 failure in both load
//! directions), so the omission was not cosmetic — it sat under the recommendation.
//!
//! **On the third arm.** The memo scoped this spike as `x + Δt·v`
//! ([`InitialGuess::Inertial`]). That variant does nothing at all on the first
//! step of a body released from rest, and the incremental-potential literature
//! starts from `x̂ + Δt²·f_ext/m` instead. Measuring only the first risks
//! concluding "predictors do not help" from the one nobody uses, so both run.
//! On the IPC fixture the two are IDENTICAL by construction (`gravity_z = 0`,
//! empty θ ⇒ `f_ext ≡ 0`), which makes that pair a free self-check on the
//! harness: they must agree exactly there.
//!
//! **On interleaving.** The arms are stepped in LOCKSTEP from a common initial
//! state — arm 0 step k, arm 1 step k, arm 2 step k, … — never as three
//! block-ordered runs, so machine drift lands on all three equally. Step-by-step
//! interleaving *inside* one trajectory is impossible here: the arms converge to
//! the same root each step but not to the same bits, so they are genuinely three
//! trajectories after step 1.
//!
//! **On repeats.** Iteration count is deterministic given the state, so it needs
//! none. Wall-clock gets 60–71 paired samples per arm from a single interleaved
//! run, which is what the recon's min-of-3 was substituting for when it timed 10
//! steps of a fixed 2-iteration workload. Absolute times are still upper bounds
//! (contended box, house rule); the RATIO between arms is the trustworthy part.
//!
//! **On the reported quantiles.** p50 / p95 / max. VR wants p99 — a dropped
//! frame is nausea, not a slow frame — but 60 samples cannot resolve a 99th
//! percentile, and padding the run with settled steps would dilute the tail
//! rather than measure it. `max` is the tail proxy here; certifying p99 is a
//! longer, separate run and is not what this spike answers.
//!
//! ## Pre-registered decision rule
//!
//! - **WIN** — ≥20 % fewer total iterations on the cantilever **and** ≥10 % on
//!   IPC, with **zero** failures (Armijo stall / iter cap / validity violation /
//!   non-finite state) on either.
//! - **KILL** — IPC loses convergence, or iteration count rises.
//! - **`block_sag` MUST NOT IMPROVE.** ⚠ Written as "must not MOVE" before the first
//!   run, and that was wrong: the fixture was chosen for having no UPSIDE (it
//!   converges at iterate 0 already — measured 0.18 iters/step, not the 0.5–0.6 it
//!   was picked on), and that got silently recorded as having no DOWNSIDE. Its
//!   downside is unbounded, and it duly degraded 32.7×. **An improvement here would
//!   indict the harness; a degradation is a fact about the technique.** State a
//!   control's licence in both directions, and name which direction is the alarm.
//! - Also reported: `‖x_arm − x_base‖ / ‖u_base‖` at the final step, where the
//!   denominator is the baseline's DISPLACEMENT from rest, not its absolute
//!   position — dividing by `‖x_base‖` on a 20 cm beam whose tip moved 4 cm
//!   reads reassuringly tiny whatever the arms did (see `rel_drift`). Two
//!   trajectories each converged to `1e-10` still separate over 60 steps, so
//!   this is REPORTED, not gated — but a large value means the predictor changed
//!   the answer rather than the path, which would be a finding, not a win.
//!
//! **VERDICT, scored against the above:** `Inertial` **WINS** (cantilever −78.7 %,
//! IPC −48.8 %, zero failures anywhere). `InertialWithLoad` is **KILLED** by the
//! KILL clause verbatim — *"IPC loses convergence"* — on the contact-plus-load
//! cell, where it dies at step 0 in both load directions. The rule predates the
//! data and discriminates the arms on its own terms, which is the point of writing
//! one down first.
//!
//! ## Running it
//!
//! Every case is `#[ignore]`d: this is an instrument, not a gate, and it must
//! cost CI nothing while still being compiled (the `gpu-probe` job builds every
//! sim-soft test target, so it cannot bit-rot silently).
//!
//! ```text
//! cargo test --release -p sim-soft --test predictor_spike -- --ignored --nocapture --test-threads=1
//! ```
//!
//! `--test-threads=1` is load-bearing twice over: it keeps the printed tables
//! from interleaving, and it keeps two cases from competing for cores while one
//! of them is being timed.

#![allow(
    // A broken fixture or a solver failure is a broken instrument, not a runtime
    // condition to recover from — the harness stops and says which arm died.
    clippy::panic,
    clippy::expect_used,
    // Step counters and iteration tallies convert small integer counts to f64 for
    // means and percentages. Every value is a step index or an iteration count,
    // orders of magnitude below f64's 2^53 exact-integer range.
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    // Each case body is one linear narrative — build the scene, run the arms,
    // print the comparison. Splitting them would scatter what a reader of the
    // results needs to hold together.
    clippy::too_many_lines
)]

use std::time::Instant;

use sim_ml_chassis::Tensor;
use sim_soft::{
    BoundaryConditions, CpuNewtonSolver, CpuTet4NHSolver, HandBuiltTetMesh, InitialGuess,
    IpcRigidContact, IpcRigidContactSolver, MaterialField, Mesh, NullContact, Solver, SolverConfig,
    SolverFailure, SphereSdf, Tet4, TranslatedSdf, Vec3, pick_vertices_by_predicate,
};

// ── Shared knobs ───────────────────────────────────────────────────────────

/// Frame time being priced (60 Hz). The recon's whole frame-budget argument is
/// denominated in this.
const DT: f64 = 1.0 / 60.0;
/// Steps for the gravity-driven fixtures — 1 s of simulated time, long enough
/// to cover the release transient AND the settled regime after it.
const STEPS: usize = 60;
/// Newton cap. Recon §3d's 60 aborted a case that needed 65 and the failure was
/// read as a solver defect for a week; 120 is ~2× the largest count ever
/// observed here (56), so a predictor that blows up shows its true cost.
const MAX_NEWTON_ITER: usize = 120;
/// Silicone-class density, shared by every fixture (recon §1b).
const DENSITY: f64 = 1030.0;
/// Downward gravity for the two body-force fixtures.
const GRAVITY: f64 = -9.81;

/// The three arms, in the order every table prints them. Arm 0 is the baseline
/// every ratio is taken against.
const ARMS: [(&str, InitialGuess); 3] = [
    ("prev-state", InitialGuess::PreviousState),
    ("inertial", InitialGuess::Inertial),
    ("inert+load", InitialGuess::InertialWithLoad),
];

// ── Per-arm result ─────────────────────────────────────────────────────────

/// One arm's trajectory record. `iters[k]` and `ms[k]` are step `k`; a dead arm
/// has fewer entries than the others and carries the reason in `failure`.
struct ArmResult {
    label: &'static str,
    iters: Vec<usize>,
    ms: Vec<f64>,
    /// Compact failure description, or `None` if the arm ran the full ramp.
    /// The `SolverFailure` variants each carry an `x_partial` of length `n_dof`,
    /// so they are summarised rather than `Debug`-printed.
    failure: Option<String>,
    /// Final position, for the cross-arm drift readout.
    x_final: Vec<f64>,
}

impl ArmResult {
    fn total_iters(&self) -> usize {
        self.iters.iter().sum()
    }

    /// Nearest-rank percentile over the per-step iteration counts.
    fn iter_pct(&self, p: f64) -> usize {
        percentile(&mut self.iters.clone(), p)
    }

    fn ms_pct(&self, p: f64) -> f64 {
        let mut v = self.ms.clone();
        v.sort_by(f64::total_cmp);
        if v.is_empty() {
            return f64::NAN;
        }
        v[rank_index(v.len(), p)]
    }

    fn ms_total(&self) -> f64 {
        self.ms.iter().sum()
    }
}

/// Nearest-rank index into a length-`n` sorted sample for percentile `p`
/// (`0.0..=1.0`). Saturates at the last element, so `p = 1.0` is the max.
fn rank_index(n: usize, p: f64) -> usize {
    let rank = (p * n as f64).ceil() as usize;
    rank.max(1).min(n) - 1
}

fn percentile(v: &mut [usize], p: f64) -> usize {
    v.sort_unstable();
    if v.is_empty() {
        0
    } else {
        v[rank_index(v.len(), p)]
    }
}

/// One-line summary of a failure. `SolverFailure`'s variants each hold an
/// `x_partial` of length `n_dof`, which must not reach a printed table.
fn describe(fail: &SolverFailure) -> String {
    match fail {
        SolverFailure::ArmijoStall {
            last_iter,
            last_r_norm,
            ..
        } => format!("ArmijoStall @ iter {last_iter}, ‖r‖ = {last_r_norm:.3e}"),
        SolverFailure::NewtonIterCap {
            max_iter,
            last_r_norm,
            ..
        } => format!("NewtonIterCap @ {max_iter}, ‖r‖ = {last_r_norm:.3e}"),
        SolverFailure::DoublyFailedFactor { last_iter, .. } => {
            format!("DoublyFailedFactor @ iter {last_iter}")
        }
        SolverFailure::ValidityViolation { tet_id, .. } => {
            format!("ValidityViolation @ tet {tet_id}")
        }
    }
}

// ── The driver ─────────────────────────────────────────────────────────────

/// Step all three arms in lockstep and record per-step iterations + wall time.
///
/// `pre_step(&mut solver, k)` runs OUTSIDE the timer and is where a
/// displacement-controlled fixture advances its collider — every arm sees the
/// identical pose at step `k`, so the arms differ only in where Newton starts.
///
/// An arm that fails, or that returns a non-finite state, stops there and keeps
/// the reason; the others carry on. That is deliberate — a predictor that kills
/// one fixture must not take the measurement of the others with it.
fn run_arms<S: Solver>(
    mut solvers: [S; 3],
    x0: &[f64],
    n_steps: usize,
    mut pre_step: impl FnMut(&mut S, usize),
) -> Vec<ArmResult> {
    let n_dof = x0.len();
    let theta = Tensor::from_slice(&[], &[0]);

    let mut x: Vec<Vec<f64>> = vec![x0.to_vec(); 3];
    let mut v: Vec<Vec<f64>> = vec![vec![0.0; n_dof]; 3];
    let mut out: Vec<ArmResult> = ARMS
        .iter()
        .map(|(label, _)| ArmResult {
            label,
            iters: Vec::with_capacity(n_steps),
            ms: Vec::with_capacity(n_steps),
            failure: None,
            x_final: x0.to_vec(),
        })
        .collect();

    for k in 0..n_steps {
        for (a, solver) in solvers.iter_mut().enumerate() {
            if out[a].failure.is_some() {
                continue;
            }
            pre_step(solver, k);

            let x_in = Tensor::from_slice(&x[a], &[n_dof]);
            let v_in = Tensor::from_slice(&v[a], &[n_dof]);
            let t0 = Instant::now();
            let step = solver.try_replay_step(&x_in, &v_in, &theta, DT);
            let elapsed_ms = t0.elapsed().as_secs_f64() * 1e3;

            match step {
                Ok(s) => {
                    if !s.x_final.iter().all(|f| f.is_finite()) {
                        out[a].failure =
                            Some(format!("non-finite x_final at step {k} (Newton diverged)"));
                        continue;
                    }
                    out[a].iters.push(s.iter_count);
                    out[a].ms.push(elapsed_ms);
                    // Backward-Euler velocity update, same form as
                    // `tests/contact_drop_rest.rs`.
                    v[a] = s
                        .x_final
                        .iter()
                        .zip(x[a].iter())
                        .map(|(xf, xp)| (xf - xp) / DT)
                        .collect();
                    x[a].clone_from(&s.x_final);
                    out[a].x_final = s.x_final;
                }
                Err(fail) => {
                    out[a].failure = Some(format!("step {k}: {}", describe(&fail)));
                }
            }
        }
    }
    out
}

/// Relative L2 separation of two arms' final states, measured on DISPLACEMENT
/// from rest rather than on absolute position.
///
/// ⚠ The absolute-position form is the trap: a 20 cm beam whose tip has moved
/// 4 cm carries a position norm dominated by the rest configuration, so any
/// separation divided by it reads as reassuringly tiny whatever the arms
/// actually did. Displacement is the quantity that is zero when nothing moved,
/// and therefore the only one whose ratio means anything.
/// ⚠ Returns `None` when the number would be unreadable rather than a number
/// nobody can interpret: either arm having died means the two `x_final`s are
/// from DIFFERENT steps (an arm that stopped at step 20 against a base that ran
/// to 71), and a base that died at step 0 leaves the denominator exactly zero.
/// The drift line is the "did the predictor change the ANSWER rather than the
/// path" readout, so it must go quiet exactly where it would otherwise mislead.
fn rel_drift(arm: &ArmResult, base: &ArmResult, x_rest: &[f64]) -> Option<f64> {
    if arm.failure.is_some() || base.failure.is_some() {
        return None;
    }
    let num: f64 = arm
        .x_final
        .iter()
        .zip(base.x_final.iter())
        .map(|(a, b)| (a - b) * (a - b))
        .sum::<f64>()
        .sqrt();
    let den: f64 = base
        .x_final
        .iter()
        .zip(x_rest.iter())
        .map(|(b, r)| (b - r) * (b - r))
        .sum::<f64>()
        .sqrt();
    (den > 0.0).then(|| num / den)
}

/// Print one case's comparison table. Everything the decision rule reads is
/// here; nothing is asserted, because the spike's job is to produce numbers,
/// not to pass.
fn report(case: &str, n_free: usize, n_steps: usize, x_rest: &[f64], arms: &[ArmResult]) {
    println!("\n╔═ {case}  ({n_free} free DOF, {n_steps} steps @ dt = 1/60) ");
    println!(
        "║ {:<11} {:>7} {:>8} {:>6} {:>6} {:>6} {:>10} {:>10} {:>10}",
        "arm", "ΣIters", "vs base", "p50", "p95", "max", "ms p50", "ms max", "ms total"
    );
    let base_total = arms[0].total_iters();
    let base_complete = arms[0].failure.is_none();
    for arm in arms {
        // A ratio against a truncated run compares different amounts of work,
        // and a dead arm's `0.000×` reads as "infinitely better" — print a
        // dash instead and let the FAILED line below carry the story.
        let ratio = if base_total == 0 || !base_complete || arm.failure.is_some() {
            "       —".to_string()
        } else {
            format!("{:>7.3}×", arm.total_iters() as f64 / base_total as f64)
        };
        if arm.iters.is_empty() {
            // A dead arm has no samples; printing `NaN` quantiles next to a
            // `0` total invites reading it as a spectacular result.
            println!(
                "║ {:<11} {:>7} {} {:>6} {:>6} {:>6} {:>10} {:>10} {:>10}",
                arm.label, "—", ratio, "—", "—", "—", "—", "—", "—",
            );
            continue;
        }
        println!(
            "║ {:<11} {:>7} {} {:>6} {:>6} {:>6} {:>10.2} {:>10.2} {:>10.1}",
            arm.label,
            arm.total_iters(),
            ratio,
            arm.iter_pct(0.50),
            arm.iter_pct(0.95),
            arm.iter_pct(1.00),
            arm.ms_pct(0.50),
            arm.ms_pct(1.00),
            arm.ms_total(),
        );
    }
    for arm in &arms[1..] {
        match rel_drift(arm, &arms[0], x_rest) {
            Some(d) => println!("║ drift {:<9} ‖Δu‖/‖u_base‖ = {d:.3e}", arm.label),
            None => println!(
                "║ drift {:<9} — not comparable (an arm died, or the base never moved)",
                arm.label
            ),
        }
    }
    for arm in arms {
        if let Some(reason) = &arm.failure {
            println!(
                "║ ⚠ FAILED {:<10} after {} steps — {reason}",
                arm.label,
                arm.iters.len()
            );
        }
    }
    println!("╚═\n");
}

// ── Fixtures ───────────────────────────────────────────────────────────────

/// Rest positions of a mesh, flattened to the solver's DOF layout.
fn rest_positions(mesh: &HandBuiltTetMesh) -> Vec<f64> {
    let mut x = vec![0.0_f64; 3 * mesh.n_vertices()];
    for (c, p) in x.chunks_exact_mut(3).zip(mesh.positions().iter()) {
        c[0] = p.x;
        c[1] = p.y;
        c[2] = p.z;
    }
    x
}

/// Recon §1b's `cantilever`: a 20 × 2 × 2 cm Neo-Hookean beam, `x = 0` face
/// pinned, released horizontal from rest under gravity. THE representative
/// large-deflection fixture — the tip droops 3.7–8.6 cm on a 20 cm span, so
/// Newton genuinely works (6–37 iterations per step).
///
/// `n_cross` sets both cross-section counts, matching the recon's `nx × ny`
/// naming: `(80, 8)` is its `cantilever 80×8` at exactly 19 440 free DOF.
fn cantilever(nx: usize, n_cross: usize, guess: InitialGuess) -> CpuTet4NHSolver<HandBuiltTetMesh> {
    /// `E ≈ 1 MPa, ν = 0.4` — the Tet4-safe ratio (volumetric locking precludes
    /// `ν → 0.5` without F-bar).
    const MU: f64 = 3.5e5;
    const LAMBDA: f64 = 1.4e6;
    const LENGTH: f64 = 0.20;
    const CROSS: f64 = 0.02;

    let field = MaterialField::uniform(MU, LAMBDA);
    let mesh = HandBuiltTetMesh::cantilever_bilayer_beam(
        nx, n_cross, n_cross, LENGTH, CROSS, CROSS, &field,
    );
    let pins = pick_vertices_by_predicate(&mesh, |p| p.x.abs() < 1e-9);
    assert!(!pins.is_empty(), "cantilever root face has no vertices");

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.density = DENSITY;
    cfg.gravity_z = GRAVITY;
    cfg.max_newton_iter = MAX_NEWTON_ITER;
    cfg.initial_guess = guess;

    CpuNewtonSolver::new(
        Tet4,
        mesh,
        NullContact,
        cfg,
        BoundaryConditions::new(pins, Vec::new()),
    )
}

/// Recon §1b's `block_sag`: a 5 cm Neo-Hookean cube, bottom face pinned,
/// released from rest under gravity. **The negative control.** Stiff and
/// small-strain — it converges in 0.5–2 Newton iterations against a floor of
/// ~1, so a predictor has nothing to win here. If this fixture MOVES, the
/// instrument is wrong.
fn block_sag(n: usize, guess: InitialGuess) -> CpuTet4NHSolver<HandBuiltTetMesh> {
    /// Matches `contact_drop_rest` / `hertz_sphere_plane`.
    const MU: f64 = 2.0e5;
    const LAMBDA: f64 = 8.0e5;
    const EDGE: f64 = 0.05;

    let field = MaterialField::uniform(MU, LAMBDA);
    let mesh = HandBuiltTetMesh::uniform_block(n, EDGE, &field);
    let pins = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    assert!(!pins.is_empty(), "block bottom face has no vertices");

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.density = DENSITY;
    cfg.gravity_z = GRAVITY;
    cfg.max_newton_iter = MAX_NEWTON_ITER;
    cfg.initial_guess = guess;

    CpuNewtonSolver::new(
        Tet4,
        mesh,
        NullContact,
        cfg,
        BoundaryConditions::new(pins, Vec::new()),
    )
}

// ── IPC `dynamic_indentation` ──────────────────────────────────────────────
//
// Recon §1b's contact fixture: `bonded_layer_indentation`'s geometry (a bonded
// elastic layer with a rigid sphere lowered through an IPC log barrier), but
// driven at `dt = 1/60` with velocity carried step to step instead of that
// gate's `STATIC_DT = 1.0`. It is the fixture closest to VR "grab and deform",
// and the one whose 2.2×-over-budget residual the ladder has to close.

/// Sphere radius (10 mm).
const RADIUS: f64 = 1.0e-2;
/// Final indentation depth.
const DELTA: f64 = 5.0e-4;
/// IPC barrier stiffness.
const KAPPA: f64 = 1.0e4;
/// Barrier band as a fraction of `δ`.
const BAND_FRAC: f64 = 0.05;
/// Layer material (μ, λ) — `ν = 0.4`.
const LAYER_MU: f64 = 1.0e5;
const LAYER_LAMBDA: f64 = 4.0 * LAYER_MU;
/// Lateral extent as a multiple of the contact patch `a`.
const LATERAL_FACTOR: f64 = 8.0;

/// Layer aspect `χ = a/h` and mesh resolution `a/cell`, held at the shared
/// central case of `bonded_layer_indentation`'s sweep. `a/cell = 2` gives
/// 5 202 free DOF and `= 3` gives 18 750 — the recon's two IPC rows.
const CHI: f64 = 0.35;

/// `(n_lat, nz, lateral, h)` for a given `a/cell`, holding the contact patch
/// fixed. Reproduces `bonded_layer_indentation::dims_for` exactly, so the DOF
/// counts line up with the recon's table rather than approximately matching it.
fn dims_for(a_over_cell: f64) -> (usize, usize, f64, f64) {
    let a = (RADIUS * DELTA).sqrt();
    let cell = a / a_over_cell;
    let h = a / CHI;
    let lateral = LATERAL_FACTOR * a;
    let even = |n: usize| if n.is_multiple_of(2) { n } else { n + 1 };
    let n_lat = even(((lateral / cell).round() as usize).max(2));
    let nz = even(((h / cell).round() as usize).max(2));
    (n_lat, nz, lateral, h)
}

fn indenter(lx: f64, ly: f64, z_center: f64) -> TranslatedSdf<SphereSdf> {
    TranslatedSdf {
        inner: SphereSdf { radius: RADIUS },
        offset: Vec3::new(lx / 2.0, ly / 2.0, z_center),
    }
}

/// The indentation scene's geometry, resolved once by [`dims_for`] and shared by
/// all three arms. Grouped because the arms must differ in the guess and NOTHING
/// else — passing six loose numbers three times invites one of them drifting.
#[derive(Clone, Copy)]
struct LayerScene {
    n_lat: usize,
    nz: usize,
    lateral: f64,
    h: f64,
    z_start: f64,
    d_hat: f64,
}

fn ipc_solver(
    scene: LayerScene,
    gravity: f64,
    guess: InitialGuess,
) -> IpcRigidContactSolver<HandBuiltTetMesh> {
    let LayerScene {
        n_lat,
        nz,
        lateral,
        h,
        z_start,
        d_hat,
    } = scene;
    let field = MaterialField::uniform(LAYER_MU, LAYER_LAMBDA);
    let mesh =
        HandBuiltTetMesh::cantilever_bilayer_beam(n_lat, n_lat, nz, lateral, lateral, h, &field);
    let pins = pick_vertices_by_predicate(&mesh, |p| p.z.abs() < 1e-9);
    assert!(!pins.is_empty(), "bonded bottom face has no vertices");

    let mut cfg = SolverConfig::skeleton();
    cfg.dt = DT;
    cfg.density = DENSITY;
    cfg.max_newton_iter = MAX_NEWTON_ITER;
    cfg.initial_guess = guess;
    cfg.gravity_z = gravity;

    CpuNewtonSolver::new(
        Tet4,
        mesh,
        IpcRigidContact::with_params(vec![indenter(lateral, lateral, z_start)], KAPPA, d_hat),
        cfg,
        BoundaryConditions::new(pins, Vec::new()),
    )
}

/// Drive the indentation ramp across all three arms.
fn run_indentation(a_over_cell: f64, gravity: f64) -> (usize, usize, Vec<f64>, Vec<ArmResult>) {
    let (n_lat, nz, lateral, h) = dims_for(a_over_cell);
    let d_hat = BAND_FRAC * DELTA;
    // South pole starts a full `1.2·d̂` above the top face (barrier inactive)
    // and is lowered in `0.3·d̂` increments to `δ` below it — the gentler start
    // `bonded_layer_indentation`'s module docs require.
    let z_start = h + RADIUS + 1.2 * d_hat;
    let z_end = h + RADIUS - DELTA;
    let increment = 0.3 * d_hat;
    let n_steps = ((z_start - z_end) / increment).ceil() as usize;

    let field = MaterialField::uniform(LAYER_MU, LAYER_LAMBDA);
    let probe =
        HandBuiltTetMesh::cantilever_bilayer_beam(n_lat, n_lat, nz, lateral, lateral, h, &field);
    let x0 = rest_positions(&probe);
    let n_free =
        3 * probe.n_vertices() - 3 * pick_vertices_by_predicate(&probe, |p| p.z.abs() < 1e-9).len();

    let scene = LayerScene {
        n_lat,
        nz,
        lateral,
        h,
        z_start,
        d_hat,
    };
    let solvers = [
        ipc_solver(scene, gravity, ARMS[0].1),
        ipc_solver(scene, gravity, ARMS[1].1),
        ipc_solver(scene, gravity, ARMS[2].1),
    ];

    // Every arm sees the identical sphere pose at step `k`; only the Newton
    // start point differs.
    let arms = run_arms(solvers, &x0, n_steps, |solver, k| {
        let z = (z_start - (k + 1) as f64 * increment).max(z_end);
        solver.replace_contact(IpcRigidContact::with_params(
            vec![indenter(lateral, lateral, z)],
            KAPPA,
            d_hat,
        ));
    });
    (n_free, n_steps, x0, arms)
}

// ── Cases ──────────────────────────────────────────────────────────────────
//
// Ordered cheapest-first so a harness defect surfaces in seconds rather than
// after the expensive case has run.

/// **Negative control.** 1 944 free DOF. Picked against recon §2a's `0.5–0.6
/// iterations/step`; it actually measures **0.18** (11 iterations over 60 steps,
/// `p50 = 0`), so its floor is even closer to zero than the premise assumed.
///
/// ⚠ The pre-registered rule in the module docs says "must not move", and that rule
/// is MIS-SPECIFIED: this fixture was chosen for having no UPSIDE, which is not the
/// same as having no downside. Its downside is wide open and it degrades hard. What
/// would indict the harness is an IMPROVEMENT here, not a degradation.
#[test]
#[ignore = "predictor spike instrument — run explicitly, see module docs"]
fn spike_block_sag_1944() {
    let solvers = [
        block_sag(8, ARMS[0].1),
        block_sag(8, ARMS[1].1),
        block_sag(8, ARMS[2].1),
    ];
    let probe = HandBuiltTetMesh::uniform_block(8, 0.05, &MaterialField::uniform(2.0e5, 8.0e5));
    let x0 = rest_positions(&probe);
    let n_free =
        3 * probe.n_vertices() - 3 * pick_vertices_by_predicate(&probe, |p| p.z.abs() < 1e-9).len();
    let arms = run_arms(solvers, &x0, STEPS, |_, _| {});
    report("block_sag (NEGATIVE CONTROL)", n_free, STEPS, &x0, &arms);
}

/// IPC indentation at 5 202 free DOF — the small representative case.
#[test]
#[ignore = "predictor spike instrument — run explicitly, see module docs"]
fn spike_ipc_5202() {
    let (n_free, n_steps, x_rest, arms) = run_indentation(2.0, 0.0);
    report("IPC dynamic_indentation", n_free, n_steps, &x_rest, &arms);
}

/// **The cell the original matrix missed: contact AND a body load.**
///
/// ⚠ Both IPC rows above run at `gravity_z = 0` with an empty θ, so `f_ext ≡ 0`
/// and `Inertial` and `InertialWithLoad` are the *same point* by construction.
/// That was designed in as a harness self-check — and it has a consequence the
/// self-check hides: **`Δt²·f_ext/m`, the only term distinguishing the arm the
/// tail result recommends, was never exercised against a contact barrier at
/// all.** The scales say that is exactly where it should be tested: one frame
/// of free fall is `Δt²g` = 2.7 mm against an IPC band `d_hat` = 2.5e-5 m —
/// **109× the length scale the barrier operates on**, and 43 % of the layer
/// thickness in a single iterate.
///
/// Run in BOTH directions, because only one of them is adversarial:
/// `-z` loads the layer AWAY from the indenter sitting above it (benign), while
/// `+z` drives it INTO the collider (the hazard). Testing only the benign
/// direction would be the same vacuous-fixture mistake as zeroing the pinned
/// velocities — see `feedback_negative_controls_need_two_sided_rules`.
#[test]
#[ignore = "predictor spike instrument — run explicitly, see module docs"]
fn spike_ipc_gravity_5202() {
    for (label, g) in [
        ("−z, away from collider", -9.81),
        ("+z, INTO collider", 9.81),
    ] {
        let (n_free, n_steps, x_rest, arms) = run_indentation(2.0, g);
        report(
            &format!("IPC + body load ({label})"),
            n_free,
            n_steps,
            &x_rest,
            &arms,
        );
    }
}

/// Cantilever at 3 000 free DOF — 24.2 iterations/step in the recon.
#[test]
#[ignore = "predictor spike instrument — run explicitly, see module docs"]
fn spike_cantilever_3000() {
    let solvers = [
        cantilever(40, 4, ARMS[0].1),
        cantilever(40, 4, ARMS[1].1),
        cantilever(40, 4, ARMS[2].1),
    ];
    let probe = HandBuiltTetMesh::cantilever_bilayer_beam(
        40,
        4,
        4,
        0.20,
        0.02,
        0.02,
        &MaterialField::uniform(3.5e5, 1.4e6),
    );
    let x0 = rest_positions(&probe);
    let n_free =
        3 * probe.n_vertices() - 3 * pick_vertices_by_predicate(&probe, |p| p.x.abs() < 1e-9).len();
    let arms = run_arms(solvers, &x0, STEPS, |_, _| {});
    report("cantilever 40×4", n_free, STEPS, &x0, &arms);
}

/// IPC indentation at 18 750 free DOF — **the representative workload**. This is
/// the fixture carrying the 2.2×-over-budget residual after R1 × R3.
#[test]
#[ignore = "predictor spike instrument — run explicitly, see module docs"]
fn spike_ipc_18750() {
    let (n_free, n_steps, x_rest, arms) = run_indentation(3.0, 0.0);
    report("IPC dynamic_indentation", n_free, n_steps, &x_rest, &arms);
}

/// Cantilever at 19 440 free DOF — **where the win should be largest**. 37.0
/// iterations/step in the recon, the number the whole 12× headline rests on.
#[test]
#[ignore = "predictor spike instrument — run explicitly, see module docs"]
fn spike_cantilever_19440() {
    let solvers = [
        cantilever(80, 8, ARMS[0].1),
        cantilever(80, 8, ARMS[1].1),
        cantilever(80, 8, ARMS[2].1),
    ];
    let probe = HandBuiltTetMesh::cantilever_bilayer_beam(
        80,
        8,
        8,
        0.20,
        0.02,
        0.02,
        &MaterialField::uniform(3.5e5, 1.4e6),
    );
    let x0 = rest_positions(&probe);
    let n_free =
        3 * probe.n_vertices() - 3 * pick_vertices_by_predicate(&probe, |p| p.x.abs() < 1e-9).len();
    let arms = run_arms(solvers, &x0, STEPS, |_, _| {});
    report("cantilever 80×8", n_free, STEPS, &x0, &arms);
}
