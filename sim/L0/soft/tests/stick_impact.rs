//! **A puck strike is a step load, and a step load is the one thing this stick
//! does not survive. Is that the physics, or the way the load was posed?**
//!
//! `stick_flex.rs` answered the sizing question — 360 free DOF of Tet10 is a
//! convincing stick at `1.14 ms/frame`, no reduction needed — and then turned up
//! something sharper on the way past:
//!
//! > A step load's residual pins at a fixed ~25 % of the load and the solve
//! > caps out — for either element, at any `dt`, under either predictor — and
//! > **even 2 % of a slapshot needs 279 iterations**.
//!
//! ⚠ Quoted carefully, because an earlier version of this file did not. It read
//! that as "a step load does not converge above ~2 % of a slapshot", which is a
//! different and stronger claim: at 2 % it *does* converge, expensively. On this
//! rig the held-force wall sits between **5 % and 10 %** of the slapshot load —
//! `a_held_force_strike_fails_above_the_wall_and_completes_below_it` hard-codes
//! `0.05` as its must-complete arm, so the gate and the misquote contradicted
//! each other.
//!
//! A puck strike *is* an abrupt load, so on its face that reads as "VR hockey is
//! not reachable", and it reads that way on the side of the budget no rung of
//! the reduced-order ladder can move: **reduction cuts `ms/iteration`, never
//! iteration count.**
//!
//! # ★ The knob the discriminator held fixed
//!
//! `stick_flex.rs::where_the_dynamic_rig_loses_convergence` swept 24 cells over
//! **density × timestep × predictor × element**. All 24 pose the load the same
//! way: a **tip force**, applied instantaneously and held. That is one posing of
//! a strike, and it is not the posing a game uses.
//!
//! A puck does not apply a held force to a blade. It transfers **momentum** over
//! a contact of `~1 ms` and leaves. The distinction is not cosmetic here,
//! because of what the predictor does with it:
//!
//! ```text
//!   force step      θ jumps;  v is unchanged  ->  x + dt·v is the OLD state,
//!                                                 so Newton starts a full
//!                                                 impact away from the answer
//!   momentum impulse   θ = 0;  v jumps        ->  x + dt·v already carries the
//!                                                 strike, so Newton starts
//!                                                 NEXT TO the answer
//! ```
//!
//! `InitialGuess::Inertial` is exactly `x + dt·v`. So the same physical strike,
//! posed the way a game poses it, hands Newton a *qualitatively different*
//! starting point. Whether that is enough to clear the wall is the question this
//! file exists to answer, and it is not answerable from the 24 cells already
//! run. → `stick_flex.rs`, and the ladder's memory of it.
//!
//! # The three arms, and why the comparison is made at MEASURED deflection
//!
//! | arm | what it does | why it is here |
//! |---|---|---|
//! | `ForceHeld` | tip force applied at the strike frame and **held** | the discriminator's posing — the **positive control** that this rig can still see the known wall |
//! | `ForcePulse` | the same force, for **one frame**, then released | separates "abrupt" from "sustained": it is a momentum transfer, but delivered as a force |
//! | `VelocityImpulse` | **no external force at all**; a velocity jump on the blade band | what a puck actually does |
//!
//! ⚠⚠ **`ForceHeld`'s `p99` is not comparable to the other two arms', by
//! construction.** Because the force is already standing when the second strike
//! frame arrives, strikes 2–10 are no-ops: the run contains **one** impact event,
//! not ten, so its impact frames are `0.3 %` of the sample and the `99th`
//! percentile falls outside them. Its `max` shows what its `p99` hides: the
//! producer reads `p99 = 8.5-9.3 ms` against `max = 365-395 ms` on the same row
//! — a `~43×` gap between two statistics of one sample, because only one frame
//! of the three hundred is an impact. (Quoted as ranges over six quiet reps: the
//! single-run values this replaced were below every subsequent sample, which is
//! what a figure frozen at run 1 does.) Only `ForceHeld`'s **convergence verdict** and its
//! **`max`** are quotable; nothing in this file asserts on its `p99`.
//!
//! ⚠⚠ **The arms are not momentum-matched analytically, and that is deliberate.**
//! Matching `F·dt` against `m·Δv` needs the lumped mass of the loaded band,
//! which is a property of the mass matrix, not of the fixture. Asserting a match
//! that the rig cannot verify is how a comparison becomes unfalsifiable. So each
//! arm sweeps its own magnitude, and every cell **reports the peak tip
//! deflection it actually reached**. The comparison is then made at matched
//! *achieved* `δ/L` — a measured quantity — not at matched drive.
//!
//! ★ **This is also the trap the sweep has to avoid.** If the impulse arm only
//! ever reaches `δ/L = 1e-4`, then "the impulse arm converges" is a tautology:
//! it converges because it barely moved the stick, not because the posing
//! helped. A claim from this file counts **only** at an achieved deflection that
//! reaches the regime where `ForceHeld` fails. That is why the magnitude grid is
//! piloted rather than guessed.
//!
//! # Why the strike REPEATS, and why that is a `p99` requirement
//!
//! VR punishes a dropped frame with nausea, so the requirement is `p99`, not
//! `p50`. A `p99` reads the impact regime only if impacts are a large enough
//! share of the sample — so the stick is struck every [`STRIKE_PERIOD`] frames
//! across [`RUN_FRAMES`]: `10` strikes, `3` per second of sim time. That is at
//! the aggressive end of a real possession (a game's stick-puck contacts run
//! nearer `1–2 Hz`), which is the conservative direction for a claim that
//! something **fits** a budget.
//!
//! ⚠ **What that buys is measured, not argued** —
//! [`striking_once_leaves_the_p99_blind_to_the_impact`]. An earlier draft of
//! these docs claimed a single strike would make the `p99` "meaningless"; it does
//! not. Backward Euler at `dt = 1/90` is strongly dissipative on a `13.7 Hz`
//! first mode (`~6.6 steps per period`), but the ring-down still leaves several
//! elevated frames, so a one-strike `p99` lands **on the ring-down**, at `38 %`
//! of the impact's iteration count — it understates by `2.6×` rather than
//! reporting nothing. The wall-time dilution is `2.7×`. Ten strikes put the
//! `p99` exactly on the worst frame (`37` of `37` iterations).
//!
//! ⚠⚠ Those figures read `43 %` / `2.3×` until 2026-08-25, and the story of how
//! is worth more than the numbers. They were measured when `stickrig::percentile`
//! used `⌊q·n⌋`; changing it to the standard `⌈q·n⌉` moved the `n = 300,
//! q = 0.99` order statistic by exactly ONE rank, which on a run with a single
//! elevated event is the difference between `16` iterations and `14`. Nothing
//! re-derived them. A later pass then "corrected" `2.4×` to `2.3×` by taking the
//! reciprocal of the stale `43 %` — **a correction computed on top of an
//! un-re-derived number.** ⇒ Changing a summary statistic's definition
//! invalidates every figure ever measured with it, including the ones in prose.
//!
//! ⚠⚠ **And `max` is not usable as a statistic here.** A `max` over 300 frames is
//! a single sample, and on this box the worst frame read `39.92 ms` on one run
//! and `58.29 ms` on the next — a `1.46×` swing from one OS hiccup. Newton
//! iteration counts are deterministic for a given trajectory and do not move at
//! all, so every claim in this file is asserted on **iterations**, with wall
//! times printed beside them and gated only where the threshold is an order of
//! magnitude clear. ★ That is a finding in its own right: **VR cares about the
//! worst frame, and the worst frame is the quantity this box measures least
//! reliably.**
//!
//! # The budget this is scored against
//!
//! The target is **PCVR** (stated 2026-08-25), i.e. `90 Hz = 11.1 ms` per frame
//! with render, AI, audio and gameplay taking their share. Physics gets
//! `~2 ms`.
//!
//! ⚠ **The struck stick busts that on BOTH statistics** — `p50 ~1.1×`,
//! `p99 ~20×`. An earlier draft said the stick "sits at `0.57×` on a `p50`",
//! which is `stick_flex.rs`'s **60 Hz smooth-ramp** figure (`1.14 ms` against
//! `2 ms`): a different regime, at a different `dt`, from a different fixture.
//! Struck at 90 Hz this file measures `~2.2 ms`. The finding is that `p50` was
//! never the *binding* number, not that it fits.
//!
//! # ⛔ Deliberately not measured here
//!
//! - **Hand-tracking jitter on the clamped band** — the other half of the VR
//!   robustness question, and a separate rung.
//! - **Sub-frame substepping through the contact** — a fourth posing, worth
//!   measuring only if these three leave a gap.
//! - **Tet4.** `stick_flex.rs` established that the held-force wall holds for
//!   *both* elements, and this file does not re-check it. The scope cut is
//!   principled rather than defensive: Tet4 **locks at this stick's `41:1`
//!   slenderness** and cannot reach the accuracy bar at any swept size, so a Tet4
//!   convergence result — whichever way it came out — cannot change what ships.
//!   ⚠ It would still be *informative*: "posing matters" is claimed here on one
//!   element only, and that is a narrower claim than the wall it is attacking.
//! - **Whether the impulse arm's cost is monotone in drive.** It is not, and
//!   [`whether_the_impulse_arm_costs_more_as_the_strike_grows`] prints the shape
//!   without explaining it. An unexplained non-monotonicity is reported as open,
//!   not smoothed over — the `8×` row is quotable as a measurement and not yet as
//!   an understanding.

// Loaded-vertex counts and frame indices cast to `f64` for load splits and
// percentile arithmetic — the idiom shared with `stick_flex.rs`, not a
// precision-sensitive path.
#![allow(clippy::cast_precision_loss)]
// `expect` on a value this file has just established to be present. In a test, a
// violated invariant should abort loudly rather than be threaded through a
// `Result` no caller can act on — the convention across this crate's tests.
#![allow(clippy::expect_used)]

use sim_ml_chassis::Tensor;
use sim_soft::element::Tet10;
use sim_soft::mesh::HandBuiltTetMesh;
use sim_soft::solver::CpuNewtonSolver;
use sim_soft::solver::backward_euler::reduced::{Inner, PodBasis, SnapshotSet};
use sim_soft::{
    CpuTet10NHSolver, InitialGuess, LoadAxis, MaterialField, Mesh, NullContact, Solver, Tet10Mesh,
    VertexId,
};

mod refbox;
mod stickrig;

use stickrig::{
    DEPTH, EI_TARGET, MASS_PER_LENGTH, RAMP_FRAMES, SLAPSHOT_DEFLECTION, SPAN, TOL_REL, WIDTH,
    describe_failure, e_eff_for, f1_analytic, is_convergence_failure, lame_for, one_frame,
    outcome_of, percentile, rho_eff, rig, slapshot_load, tip_of,
};

// ---------------------------------------------------------------------------
// The regime.
// ---------------------------------------------------------------------------

/// VR frame period (s). `90 Hz`, not the `60 Hz` every cost figure in
/// `stick_flex.rs` is measured at.
const VR_DT: f64 = 1.0 / 90.0;

/// Frames per run.
///
/// ★ Set by the `p99` it has to support, not by taste: a `p99` over
/// `stick_flex.rs`'s 20 samples is the 20th-of-20 order statistic, i.e. the
/// maximum wearing a percentile's name. `300` puts the `99th` percentile at
/// index `296` under [`percentile`]'s `⌈q·n⌉ − 1` rule, three samples deep, and
/// spans `3.33 s` of sim time — about `46` periods of the `13.7 Hz` first mode.
const RUN_FRAMES: usize = 300;

/// Frames between strikes. See the module docs: this is a `p99` **requirement**,
/// not a scenario preference. At `30` the stick is struck `10` times per run and
/// impact frames are ~`3 %` of the sample, so the `99th` percentile lands inside
/// the impacts rather than in the quiet ring-down between them.
const STRIKE_PERIOD: usize = 30;

/// The grid that carries `360` free Tet10 DOF — the stick `stick_flex.rs` says
/// is a `5 %`-flex-error stick, and the one a PCVR budget affords.
const GRID: (usize, usize, usize) = (4, 1, 2);

/// Tip stiffness (N/m) of the fixture, `k = 3EI/L³`.
fn tip_stiffness(ei: f64) -> f64 {
    3.0 * ei / (SPAN * SPAN * SPAN)
}

/// Effective tip mass (kg) of the cantilever's first bending mode,
/// `0.2427·m′·L` — the standard Rayleigh lumping for a clamped-free beam.
///
/// Used only to set the *scale* of the velocity sweep. It is not used to match
/// the arms against each other; see the module docs for why that matching is
/// refused.
fn effective_tip_mass() -> f64 {
    0.242_7 * MASS_PER_LENGTH * SPAN
}

/// Tip velocity (m/s) whose kinetic energy equals the strain energy of a
/// slapshot-scale deflection: `½m_eff·v² = ½k·δ²`, so `v = δ·√(k/m_eff)`.
///
/// Lands at `~8.6 m/s`. ★ Worth reading against a real puck: the momentum that
/// carries is `m_eff·v ≈ 0.63 N·s`, which a `170 g` puck delivers by losing
/// `~3.7 m/s`. That is a firm pass, not a slapshot — so the sweep must reach
/// **above** `1.0×` before it can claim to have covered game loads.
fn slapshot_equivalent_tip_velocity() -> f64 {
    SLAPSHOT_DEFLECTION * (tip_stiffness(EI_TARGET) / effective_tip_mass()).sqrt()
}

/// Tip displacement (m) a suddenly-applied tip load `f` produces in ONE backward
/// -Euler frame from rest.
///
/// From `M(x − x₀)/dt² + K x = f` with `v₀ = 0`, lumping the beam onto its first
/// mode: `x = f / (m_eff/dt² + k)`. At `dt = 1/90` the two terms are comparable
/// (`590` against `538 N/m`), so inertia carries about half the load — which is
/// why the answer for a force step is *small*, and why failing to find it is
/// surprising.
fn one_frame_response(f: f64) -> f64 {
    f / (effective_tip_mass() / (VR_DT * VR_DT) + tip_stiffness(EI_TARGET))
}

// ---------------------------------------------------------------------------
// The arms.
// ---------------------------------------------------------------------------

/// How a strike is delivered to the blade band.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Strike {
    /// Tip force applied at the strike frame and held for the rest of the run.
    /// The 24-cell discriminator's posing, and this file's positive control.
    ForceHeld,
    /// The same tip force, applied for exactly one frame and then released.
    ForcePulse,
    /// No external force at all: a velocity jump on the blade band, which is
    /// what a momentum transfer is.
    VelocityImpulse,
}

impl Strike {
    const fn label(self) -> &'static str {
        match self {
            Self::ForceHeld => "force-held",
            Self::ForcePulse => "force-pulse",
            Self::VelocityImpulse => "impulse",
        }
    }
}

/// What one `(strike, magnitude)` cell produced.
struct Run {
    free_dof: usize,
    /// Per-frame Newton iteration counts, in frame order.
    iters: Vec<usize>,
    /// Per-frame wall time (ms), in frame order.
    ms: Vec<f64>,
    /// ★ Peak tip deflection reached over the run, as a fraction of span. **The
    /// axis the arms are compared on**, because it is measured rather than
    /// assumed — see the module docs.
    peak_ratio: f64,
    /// `Some(reason)` if the run ended before [`RUN_FRAMES`].
    ended_early: Option<String>,
}

impl Run {
    /// Whether the run completed. ⚠ Every summary below is meaningless on a
    /// truncated sample, so callers gate on this rather than reading a `p99`
    /// over however many frames happened before the solver gave up.
    const fn completed(&self) -> bool {
        self.ended_early.is_none()
    }

    /// Frames whose Newton iteration count lies in `band`, as
    /// `(iterations, ms per iteration)` pairs.
    ///
    /// ⚠⚠ The ratio is `ms/iters` **per frame**, not `median(ms)/median(iters)`.
    /// The two diverge as soon as iteration counts spread — which is exactly the
    /// regime this file measures, `1` iteration in the quiet frames against `37`
    /// in the impact frames — and the second form silently divides two order
    /// statistics that need not come from the same frame at all.
    /// `stick_flex.rs`'s `Cost::ms_per_iter` carries the same warning; here it
    /// would be live.
    ///
    /// ★★ **The iteration count is returned alongside the cost, from this one
    /// filter, deliberately.** A caller that wants to check its two bands really
    /// are far apart must read that from the *same* rows it took the costs from.
    /// Computing it from a second, parallel filter is how a guard ends up unable
    /// to see the band it is guarding — which is what mutation testing found
    /// here, in the version that did exactly that.
    fn frames_in(&self, band: std::ops::RangeInclusive<usize>) -> Vec<(usize, f64)> {
        self.iters
            .iter()
            .zip(&self.ms)
            // ⚠ No `&& it > 0` here. It was redundant against every caller's
            // band, and it silently REPAIRED a mutated lower bound of `0` — so
            // the one mutation able to catch a bad band was swallowed.
            .filter(|&(&it, _)| band.contains(&it))
            .map(|(&it, &ms)| (it, ms / it as f64))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// The runner.
// ---------------------------------------------------------------------------

/// Run one `(strike, magnitude)` cell: [`RUN_FRAMES`] frames at [`VR_DT`], struck
/// every [`STRIKE_PERIOD`] frames.
///
/// `magnitude` is a fraction of the arm's own reference drive — of
/// [`slapshot_load`] for the two force arms, of
/// [`slapshot_equivalent_tip_velocity`] for the impulse arm. The arms are
/// **not** comparable at equal `magnitude`; they are compared at equal
/// [`Run::peak_ratio`], which this returns.
///
/// ★★ **Every arm is held to the same absolute Newton tolerance**, taken from
/// the slapshot load regardless of which arm is running. Letting each arm scale
/// its own tolerance by its own drive would mean the impulse arm could "clear
/// the wall" by being asked for less, which is the failure mode this whole file
/// is built to avoid. → `stickrig::rig`, which derives `tol` from the load it is
/// handed.
fn run_cell(strike: Strike, magnitude: f64, grid: (usize, usize, usize)) -> Run {
    run_cell_every(strike, magnitude, grid, STRIKE_PERIOD)
}

/// [`run_cell`], with the strike interval exposed.
///
/// ★ Exposed because [`STRIKE_PERIOD`] is a load-bearing design choice and this
/// file asserts what it buys rather than arguing for it in prose —
/// see [`striking_once_leaves_the_p99_blind_to_the_impact`].
fn run_cell_every(
    strike: Strike,
    magnitude: f64,
    grid: (usize, usize, usize),
    strike_period: usize,
) -> Run {
    let (nx, ny, nz) = grid;
    let (mu, lambda) = lame_for(e_eff_for(EI_TARGET));
    let field = MaterialField::uniform(mu, lambda);
    let tet4 = HandBuiltTetMesh::cantilever_bilayer_beam(nx, ny, nz, SPAN, WIDTH, DEPTH, &field);
    let mesh = Tet10Mesh::from_tet4(&tet4);
    let n_dof = 3 * mesh.n_vertices();

    // ★ The tolerance reference is the slapshot load for EVERY arm — see the
    // doc comment. `rig`'s other uses of `load` are confined to that.
    let tol_reference = slapshot_load(EI_TARGET);
    let (x_flat, rest_z, bc, mut cfg) = rig(&mesh, tol_reference, rho_eff(), false, TOL_REL);
    cfg.dt = VR_DT;
    cfg.initial_guess = InitialGuess::Inertial;

    let free_dof = n_dof - 3 * bc.pinned_vertices.len();
    let loaded = bc.loaded_vertices.clone();
    let n_loaded = loaded.len();
    let solver: CpuTet10NHSolver<Tet10Mesh> =
        CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);

    let force_per_vertex = slapshot_load(EI_TARGET) * magnitude / n_loaded as f64;
    let strike_velocity = slapshot_equivalent_tip_velocity() * magnitude;

    let (mut x, mut v) = (x_flat, vec![0.0; n_dof]);
    let mut iters = Vec::with_capacity(RUN_FRAMES);
    let mut ms = Vec::with_capacity(RUN_FRAMES);
    let mut peak_ratio = 0.0f64;
    let mut ended_early = None;
    // Held only by `ForceHeld`; the other two arms drive through `v` or through
    // a single frame's `theta` and leave this at zero.
    let mut standing_force = 0.0f64;

    for k in 0..RUN_FRAMES {
        let striking = k % strike_period == 0;
        let theta_value = match strike {
            Strike::ForceHeld => {
                if striking {
                    standing_force = force_per_vertex;
                }
                standing_force
            }
            Strike::ForcePulse if striking => force_per_vertex,
            Strike::ForcePulse | Strike::VelocityImpulse => 0.0,
        };
        if strike == Strike::VelocityImpulse && striking {
            // A momentum transfer, not a force: the blade band gains velocity
            // along the flex axis and nothing else changes. `Inertial` reads
            // this straight into `x + dt·v`, which is the whole hypothesis.
            for &(vertex, _) in &loaded {
                v[3 * vertex as usize + 2] += strike_velocity;
            }
        }

        let theta = Tensor::from_slice(&[theta_value], &[1]);
        match one_frame(&solver, &mut x, &mut v, &theta, VR_DT) {
            Ok((it, t)) => {
                iters.push(it);
                ms.push(t);
                peak_ratio = peak_ratio.max(tip_of(&x, &loaded, &rest_z) / SPAN);
            }
            Err(why) => {
                ended_early = Some(format!("frame {k}: {why}"));
                break;
            }
        }
    }

    Run {
        free_dof,
        iters,
        ms,
        peak_ratio,
        ended_early,
    }
}

/// One row of the report, or the reason there is no row.
fn report_row(strike: Strike, magnitude: f64, run: &Run) {
    let verdict = run.ended_early.as_deref().unwrap_or("ok");
    if run.completed() {
        let ms_f: Vec<f64> = run.ms.clone();
        let it_f: Vec<f64> = run.iters.iter().map(|&i| i as f64).collect();
        // A run whose predictor already meets tol takes ZERO iterations, so
        // there is no per-iteration cost to report. Printing the `inf` that
        // division gives reads as a defect; `n/a` reads as what it is.
        let it99 = percentile(&it_f, 0.99);
        let per_frame: Vec<f64> = run
            .frames_in(1..=usize::MAX)
            .into_iter()
            .map(|(_, r)| r)
            .collect();
        let per_iter = if per_frame.is_empty() {
            "n/a".to_owned()
        } else {
            format!("{:.3}", percentile(&per_frame, 0.5))
        };
        println!(
            "{:>13} {magnitude:>7.3} {:>7} {:>10.2e} {:>8.2} {:>8.2} {:>8.2} {:>7.1} {:>7.1} \
             {:>7} {:>8} {:>9}",
            strike.label(),
            run.free_dof,
            run.peak_ratio,
            percentile(&ms_f, 0.5),
            percentile(&ms_f, 0.99),
            ms_f.iter().copied().fold(0.0f64, f64::max),
            percentile(&it_f, 0.5),
            it99,
            run.iters.iter().copied().max().unwrap_or(0),
            per_iter,
            verdict,
        );
    } else {
        // ⚠ No summary at all on a truncated run. A `p99` over the handful of
        // frames that happened before the solver gave up is not a `p99` of
        // anything, and printing one beside the completed rows invites it to be
        // read as if it were.
        println!(
            "{:>13} {magnitude:>7.3} {:>7} {:>10.2e} {:>8} {:>8} {:>8} {:>7} {:>7} {:>7} \
             {:>8} {:>9}",
            strike.label(),
            run.free_dof,
            run.peak_ratio,
            "-",
            "-",
            "-",
            "-",
            "-",
            "-",
            "-",
            verdict,
        );
    }
}

/// Column header for [`report_row`].
fn report_header() {
    println!(
        "{:>13} {:>7} {:>7} {:>10} {:>8} {:>8} {:>8} {:>7} {:>7} {:>7} {:>8} {:>9}",
        "arm",
        "mag",
        "DOF",
        "peak d/L",
        "p50 ms",
        "p99 ms",
        "max ms",
        "p50 it",
        "p99 it",
        "max it",
        "ms/it",
        "outcome"
    );
}

// ---------------------------------------------------------------------------
// Producer. Run this before touching a gate number — the grid below is piloted.
// ---------------------------------------------------------------------------

/// Magnitudes swept, as a fraction of each arm's reference drive.
///
/// Brackets the held-force wall — measured on this rig between `5 %` and `10 %`
/// of the slapshot load — by an order of magnitude below and nearly two above,
/// and **reaches `8.0×`** because the impulse arm's reference velocity
/// corresponds to only a firm pass, not a slapshot — see
/// [`slapshot_equivalent_tip_velocity`].
const MAGNITUDES: [f64; 10] = [0.01, 0.02, 0.05, 0.10, 0.25, 0.50, 1.00, 2.00, 4.00, 8.00];

/// **The producer: three postings of the same strike, swept.**
///
/// Read `peak d/L` first and the outcome second. A row that converged at a
/// deflection three decades below the row it is being compared with has not
/// cleared any wall; it has avoided one.
#[test]
#[ignore = "producer — reference box only, run explicitly"]
fn how_a_strike_is_posed_against_whether_it_converges() {
    println!(
        "\n=== strike posing ladder — Tet10 {GRID:?}, dt = 1/90, {RUN_FRAMES} frames, \
         struck every {STRIKE_PERIOD} ==="
    );
    println!(
        "  reference force {:.1} N (slapshot), reference tip velocity {:.2} m/s",
        slapshot_load(EI_TARGET),
        slapshot_equivalent_tip_velocity()
    );
    report_header();

    for strike in [
        Strike::ForceHeld,
        Strike::ForcePulse,
        Strike::VelocityImpulse,
    ] {
        for magnitude in MAGNITUDES {
            let run = run_cell(strike, magnitude, GRID);
            report_row(strike, magnitude, &run);
        }
    }
}

/// Magnitudes for the impulse arm's own ladder, refining the range where the
/// producer's iteration count stops being monotone.
const IMPULSE_FINE: [f64; 7] = [2.00, 3.00, 4.00, 5.00, 6.00, 7.00, 8.00];

/// **Diagnostic: the impulse arm's cost is NOT monotone in strike magnitude.**
///
/// The producer reads `136 → 190 → 75` iterations at `2× → 4× → 8×`. A cost that
/// falls as the drive rises is either real Newton-basin geometry or a rig
/// artefact, and quoting the `8×` figure without saying which would be quoting a
/// number nobody has looked at. This prints the shape at `1×` steps so the
/// non-monotonicity can be seen rather than inferred from three points.
///
/// ⚠ It resolves nothing on its own; it is the instrument that says what to
/// attack next. Peak deflection is printed beside cost because if `δ/L` is
/// **linear** in magnitude across the whole range — as the producer's
/// `1.08e-2 / 2.16e-2 / 4.31e-2 / 8.57e-2` suggests — then the response is
/// essentially linear there and a *non-monotone iteration count over a linear
/// response* is the part that wants explaining.
#[test]
#[ignore = "diagnostic — run explicitly"]
fn whether_the_impulse_arm_costs_more_as_the_strike_grows() {
    println!("\n=== impulse arm, fine ladder — is cost monotone in drive? ===");
    report_header();
    let mut prev: Option<(f64, f64)> = None;
    let mut inversions = 0usize;
    for magnitude in IMPULSE_FINE {
        let run = run_cell(Strike::VelocityImpulse, magnitude, GRID);
        report_row(Strike::VelocityImpulse, magnitude, &run);
        if run.completed() {
            let it99 = percentile(
                &run.iters.iter().map(|&i| i as f64).collect::<Vec<_>>(),
                0.99,
            );
            if let Some((_, prev_it)) = prev
                && it99 < prev_it
            {
                inversions += 1;
            }
            prev = Some((run.peak_ratio, it99));
        }
    }
    println!(
        "  {inversions} magnitude step(s) COST LESS than the step below them, over \
         {} rungs",
        IMPULSE_FINE.len()
    );
    // ⚠ Deliberately not a pass/fail threshold. This is a producer whose job is
    // to make the shape visible; asserting a monotonicity nobody has explained
    // would turn an open question into a gate that fails for a reason its
    // message cannot name.
    assert!(
        prev.is_some(),
        "no rung of the fine ladder completed, so it shows nothing — the rig is \
         wrong, not the ladder"
    );
}

// ---------------------------------------------------------------------------
// Gates.
// ---------------------------------------------------------------------------

/// PCVR physics budget (ms) inside a `90 Hz / 11.1 ms` frame.
///
/// ⚠ **An estimate, not a figure the user stated.** Render (stereo), AI, audio,
/// broadphase and gameplay take the rest. It is quoted here so the verdict below
/// is legible, and the verdict does not turn on it: the measured `p99` is more
/// than an order of magnitude over this, so it also busts `4 ms`, `6 ms`, and
/// the entire `11.1 ms` frame.
const PCVR_PHYSICS_BUDGET_MS: f64 = 2.0;

/// **Positive control: this rig still sees the wall `stick_flex.rs` found.**
///
/// Two-sided, and it has to be. A rig where everything converges would report
/// "momentum posing clears the wall" without there being a wall, and a rig where
/// nothing converges would report it just as loudly. So both arms of the
/// discriminator are asserted: below the wall the held force completes, above it
/// the solve caps out.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn a_held_force_strike_fails_above_the_wall_and_completes_below_it() {
    let below = run_cell(Strike::ForceHeld, 0.05, GRID);
    let above = run_cell(Strike::ForceHeld, 0.25, GRID);
    report_header();
    report_row(Strike::ForceHeld, 0.05, &below);
    report_row(Strike::ForceHeld, 0.25, &above);

    assert!(
        below.completed(),
        "a held force at 5 % of a slapshot did NOT complete ({:?}) — the wall has \
         moved DOWN, and every comparison in this file is scored against where it is",
        below.ended_early
    );
    assert!(
        !above.completed(),
        "a held force at 25 % of a slapshot COMPLETED, reaching d/L = {:.3e}. The \
         wall this file exists to attack is gone, so the momentum comparison below \
         no longer demonstrates anything and its verdict must be re-derived",
        above.peak_ratio
    );
    // ★★ ...and it must fail for the reason this gate NAMES. `completed()` is
    // false for an inverted element or a failed factorisation too, so without
    // this the gate reports "the convergence wall is intact" when the cell died
    // of something else entirely — a different defect demanding the opposite fix.
    let why = above.ended_early.as_deref().unwrap_or_default();
    assert!(
        is_convergence_failure(why),
        "the above-wall cell did fail, but NOT by failing to converge: {why:?}. \
         This gate certifies a Newton convergence wall; anything else here is a \
         separate defect wearing its clothes"
    );
}

/// ★★★ **The headline: posing a strike as momentum clears the wall that posing
/// it as force hits — and it clears it at a DEEPER deflection, not a shallower
/// one.**
///
/// The guard is the whole gate. "The impulse arm converged" is worth nothing on
/// its own, because an arm that barely moves the stick converges trivially. So
/// the deepest deflection *any* force arm reached while completing is measured
/// here rather than hard-coded, and the impulse arm has to beat it.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn a_momentum_strike_converges_deeper_than_any_force_strike_that_converged() {
    report_header();
    let mut deepest_force = 0.0f64;
    let mut any_force_failed = false;
    for strike in [Strike::ForceHeld, Strike::ForcePulse] {
        for magnitude in [0.05, 0.10, 0.25] {
            let run = run_cell(strike, magnitude, GRID);
            report_row(strike, magnitude, &run);
            if run.completed() {
                deepest_force = deepest_force.max(run.peak_ratio);
            } else {
                let why = run.ended_early.as_deref().unwrap_or_default();
                assert!(
                    is_convergence_failure(why),
                    "{} at {magnitude}x failed, but not by failing to converge: \
                     {why:?}. This file's central claim rests on the force posing \
                     hitting a CONVERGENCE wall, not on it dying some other way",
                    strike.label()
                );
                any_force_failed = true;
            }
        }
    }
    let impulse = run_cell(Strike::VelocityImpulse, 8.0, GRID);
    report_row(Strike::VelocityImpulse, 8.0, &impulse);

    assert!(
        any_force_failed,
        "no force-posed cell failed, so there is no wall here to clear and this \
         gate is vacuous"
    );
    // ⚠⚠ A floor, not `> 0.0`. The bar is the deepest deflection a force arm
    // reached WHILE CONVERGING, so it moves DOWN as the wall gets stronger: if a
    // change left only the shallowest cells converging, `deepest_force` would
    // collapse, `any_force_failed` would still hold, and the impulse arm would
    // clear a tiny bar — the gate passing LOUDER on weaker evidence. In the limit
    // `> 0.0` accepts a bar of `1e-30`.
    //
    // ⚠ `1e-3` sits BETWEEN two adjacent rungs of the sweep, not comfortably
    // above one: `0.05` reaches `4.96e-3` and the next magnitude down (`0.01`)
    // reaches `9.92e-4`, just under. That is the useful placement — it
    // discriminates one rung from the next — but it is not the "~5x margin" an
    // earlier comment claimed. ⚠⚠ And within the magnitudes this gate actually
    // sweeps, the floor is equivalent to the `> 0.0` arm below: the reachable
    // values are `{0, r(0.05), r(0.10), r(0.25)}`, so it only becomes
    // independent if the deflection RESPONSE shrinks ~5x.
    // ⚠ Two different defects, so two different messages. The `> 0.0` form this
    // replaced said "the rig is broken, not the finding" — right for the
    // nothing-completed case and wrong for a merely shallow bar. Collapsing them
    // into one threshold lost that, and a diagnosis is most of what an assert is
    // worth when it fires.
    assert!(
        deepest_force > 0.0,
        "NO force-posed cell completed, so there is nothing to compare the impulse \
         arm against. The rig is broken, not the finding"
    );
    assert!(
        deepest_force > 1e-3,
        "the deepest CONVERGING force cell reached only d/L = {deepest_force:.3e}. \
         That is not a deflection worth beating, so the impulse arm clearing it \
         demonstrates nothing about posing"
    );
    assert!(
        impulse.completed(),
        "the momentum-posed strike did NOT complete ({:?}) — the file's central \
         claim is false as measured",
        impulse.ended_early
    );
    assert!(
        impulse.peak_ratio > deepest_force,
        "the impulse arm converged at d/L = {:.3e}, which is NOT deeper than the \
         {:.3e} the force arms reached while converging. It therefore converged by \
         moving the stick LESS, and this gate must not be read as evidence that \
         posing matters",
        impulse.peak_ratio,
        deepest_force
    );
}

/// **The `p99` Newton iteration count spans the magnitude sweep — the one half of
/// the cost story that is deterministic enough for CI.**
///
/// Asserts exactly one thing: `p99` iterations differ by more than `4×` between
/// the shallowest and deepest strike. That is a *precondition* for attributing
/// frame cost to iteration count — if iterations did not vary, there would be
/// nothing to attribute to — and it is not the attribution.
///
/// ⚠⚠ **This gate has been renamed TWICE for over-claiming, so read what it
/// asserts, not what it is called.** It reads only `run.iters` and never touches
/// `run.ms`. It was `..._is_iteration_count_not_cost_per_iteration` while
/// enforcing nothing about cost per iteration, then
/// `the_frame_cost_spike_tracks_iteration_count` while establishing no frame-cost
/// spike at all — mutation proved that one by pinning every frame's cost to a
/// constant `1.00 ms`, which leaves no spike anywhere and still passes.
///
/// ⇒ **The attribution lives in
/// [`the_cost_per_iteration_is_flat_across_the_spike`], is reference-box gated,
/// and is NOT a CI guarantee.** Keeping a clock off CI's path cost that, and the
/// cost is real rather than a naming detail.
///
/// ⚠ Splitting them is not cosmetic. `quality-gate.yml` runs eleven binaries in
/// one `cargo test --release` with no `--test-threads=1`, so every release-only
/// gate here executes concurrently, each driving a rayon-parallel solver. A `ms`
/// threshold under that contention fails for a reason its message cannot name,
/// on PRs that changed nothing — and `stick_flex.rs` already keeps all three of
/// its timing instruments behind `refbox::require_quiet_box()` for exactly this.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn the_p99_iteration_count_spans_the_magnitude_sweep() {
    report_header();
    let mut it99 = Vec::new();
    for magnitude in [0.05, 0.50, 2.00] {
        let run = run_cell(Strike::VelocityImpulse, magnitude, GRID);
        report_row(Strike::VelocityImpulse, magnitude, &run);
        assert!(
            run.completed(),
            "impulse at {magnitude}x did not complete ({:?}), so this gate has no \
             sample to attribute",
            run.ended_early
        );
        it99.push(percentile(
            &run.iters.iter().map(|&i| i as f64).collect::<Vec<_>>(),
            0.99,
        ));
    }
    let (lo, hi) = (
        it99.iter().copied().fold(f64::MAX, f64::min),
        it99.iter().copied().fold(0.0f64, f64::max),
    );
    println!(
        "  p99 Newton iterations span {:.1}x ({lo:.0}-{hi:.0})",
        hi / lo
    );
    assert!(
        hi / lo > 4.0,
        "p99 iteration count only spanned {:.1}x ({lo:.0}-{hi:.0}) across these \
         magnitudes, so there is no spike here to attribute to anything",
        hi / lo
    );
}

/// **The wall-time half: cost per Newton iteration is flat across the spike.**
///
/// Two POPULATIONS of frames, not two order statistics — cheap frames (1-2
/// iterations, the stick coasting) against costly ones (10+, the stick being
/// struck), each frame's own `ms/iters`. If cost were linear in iterations those
/// populations agree; if a fixed per-frame overhead or a super-linear term drove
/// the spike, they do not.
///
/// ★ It doubles as the control on this file's own instrument: the solver prints
/// to stdout whenever the `faer` LU fallback fires, and that printing happens
/// INSIDE the timed region, on impact frames only. If it were inflating them,
/// the costly population would show a higher cost per iteration. It does not.
///
/// ⛔ `#[ignore]`d and quiet-box gated, because it is the only claim in this file
/// that irreducibly needs a clock. → `stick_flex.rs`, same treatment.
#[test]
#[ignore = "timing instrument — reference box only, run explicitly"]
fn the_cost_per_iteration_is_flat_across_the_spike() {
    refbox::require_quiet_box();
    report_header();
    let mut cheap: Vec<(usize, f64)> = Vec::new();
    let mut costly: Vec<(usize, f64)> = Vec::new();
    for magnitude in [0.05, 0.50, 2.00] {
        let run = run_cell(Strike::VelocityImpulse, magnitude, GRID);
        report_row(Strike::VelocityImpulse, magnitude, &run);
        assert!(run.completed(), "impulse at {magnitude}x did not complete");
        cheap.extend(run.frames_in(1..=2));
        costly.extend(run.frames_in(10..=usize::MAX));
    }
    // ⚠⚠ A population FLOOR, not `!is_empty()`. The two bands are disjoint
    // literals (`1..=2` against `10..`), so any assertion that they are far apart
    // in iteration count is a tautology — it holds for every possible sample. What
    // genuinely needs asserting is that each population is big enough for a median
    // to be a rate rather than noise: `!is_empty()` accepted `n = 1`, letting a
    // `1.00x` agreement be read off one frame against 290. Piloted at 440 and 80.
    assert!(
        cheap.len() >= 100 && costly.len() >= 20,
        "the sample holds {} frames at 1-2 Newton iterations and {} at 10+. A median \
         over a handful of frames is not a rate, so the flatness below would be read \
         off noise",
        cheap.len(),
        costly.len()
    );
    let mean = |rows: &[(usize, f64)]| {
        rows.iter().map(|&(it, _)| it).sum::<usize>() as f64 / rows.len() as f64
    };
    let (cheap_mean, costly_mean) = (mean(&cheap), mean(&costly));
    let cheap_rate = percentile(&cheap.iter().map(|&(_, r)| r).collect::<Vec<_>>(), 0.5);
    let costly_rate = percentile(&costly.iter().map(|&(_, r)| r).collect::<Vec<_>>(), 0.5);
    let drift = (costly_rate / cheap_rate).max(cheap_rate / costly_rate);
    println!(
        "  {cheap_rate:.3} ms/iter on {} frames averaging {cheap_mean:.1} iterations, \
         {costly_rate:.3} on {} averaging {costly_mean:.1} — {drift:.2}x apart",
        cheap.len(),
        costly.len(),
    );
    assert!(
        drift < 1.5,
        "cost per Newton iteration differs {drift:.2}x between frames taking 1-2 \
         iterations ({cheap_rate:.3} ms) and frames taking 10+ ({costly_rate:.3} ms). \
         The spike is then NOT purely iteration count, and the claim that reduction \
         cannot touch it does not follow"
    );
}

/// ★★★ **The verdict: a game strike busts the PCVR physics budget on BOTH
/// statistics — marginally on `p50`, catastrophically on `p99`.**
///
/// Every cost figure this stick had been scored on was a `p50` in a smooth ramp,
/// where `stick_flex.rs` reads `1.14 ms/frame` at 60 Hz — `0.57×` a ~2 ms budget.
/// Struck at 90 Hz the same stick at the same DOF on the same box reads `~2.2 ms`
/// (`~1.1×`, over) and a `p99` above `40 ms` (`~20×`, far over).
///
/// ⚠⚠ **This test used to be called `..._fits_the_pcvr_budget_on_p50_...` and
/// asserted `p50 < 2 × budget`.** It printed `p50 2.22 ms = 1.11x budget` — its
/// own refutation — and passed, because the threshold was set at twice the number
/// the name claimed. The `p50` does not fit and never did; the honest contrast is
/// `1.1×` against `20×`, not "fits" against "busts". ★ The `0.57×` figure the old
/// doc leant on is `stick_flex.rs`'s **60 Hz smooth-ramp** number — a different
/// regime at a different `dt` — so the comparison was also across two fixtures.
///
/// ⛔ `#[ignore]`d and quiet-box gated. This claim is irreducibly about
/// milliseconds, and `quality-gate.yml` runs eleven binaries in one
/// `cargo test --release` with no `--test-threads=1`. Under that contention a
/// `ms` threshold fails for a reason its message cannot name, on PRs that changed
/// nothing. `stick_flex.rs` ships only accuracy gates to CI for the same reason.
#[test]
#[ignore = "timing instrument — reference box only, run explicitly"]
fn a_game_strike_busts_the_pcvr_budget_on_both_p50_and_p99() {
    refbox::require_quiet_box();
    let run = run_cell(Strike::VelocityImpulse, 1.0, GRID);
    report_header();
    report_row(Strike::VelocityImpulse, 1.0, &run);
    assert!(
        run.completed(),
        "the strike did not complete ({:?}), so there is no cost distribution to \
         judge",
        run.ended_early
    );

    let p50 = percentile(&run.ms, 0.5);
    let p99 = percentile(&run.ms, 0.99);
    println!(
        "  p50 {p50:.2} ms = {:.2}x budget; p99 {p99:.2} ms = {:.1}x budget; \
         p99/p50 = {:.1}x",
        p50 / PCVR_PHYSICS_BUDGET_MS,
        p99 / PCVR_PHYSICS_BUDGET_MS,
        p99 / p50,
    );
    // ★ Both arms now assert what the name says: over the budget, on both
    // statistics. Piloted at 2.20-2.35 ms and 38-43 ms across runs.
    //
    // ⚠⚠ The `p50` arm is the THINNEST margin on this branch — 13-14 % over its
    // threshold, on a quantity this file's own docs put at ~5 % run-to-run drift.
    //
    // ⚠⚠⚠ And `require_quiet_box()` above is NOT a guarantee that this reading is
    // clean. Measured: a `stick_flex` timing instrument read `3.150 ms/frame`
    // against a true `1.14` — 2.8x wrong — on a run where the quiet-box probe
    // PASSED (`p50 22.76 ms, burst 1.02x`), because the contending work started
    // after the probe sampled. An interleaved A/B then showed no regression at
    // all. ⇒ The gate catches a box that is busy WHEN PROBED; it cannot catch one
    // that gets busy afterwards. Never quote a single reading from behind it.
    // Every other threshold here has >=3x margin or sits on deterministic
    // iteration counts. It is tolerable only because this test is `#[ignore]`d
    // and quiet-box gated, so a flake costs a re-run rather than a red CI check.
    // If it ever fires, re-measure before believing it.
    assert!(
        p50 > PCVR_PHYSICS_BUDGET_MS,
        "p50 was {p50:.2} ms, INSIDE the {PCVR_PHYSICS_BUDGET_MS} ms budget. The \
         smooth-regime cost has improved past what this file reports, so the \
         verdict needs re-deriving rather than silently passing"
    );
    assert!(
        p99 > 7.5 * PCVR_PHYSICS_BUDGET_MS,
        "p99 was {p99:.2} ms, inside 7.5x the physics budget. The impact spike has \
         been fixed or the strike is no longer reaching the stick; either way this \
         file's verdict needs re-deriving"
    );
}

/// **Negative control, with the companion positive arm that makes it mean
/// something — both on Newton ITERATIONS.**
///
/// An unstruck run must be flat: the `Inertial` guess already meets tol, so
/// every frame returns `iter_count = 0`. That alone is worth little, so the
/// struck run's iteration `p99` is asserted beside it.
///
/// ⚠⚠ **This used to compare wall time, and could not fail for its stated
/// reason.** The quiet arm never enters the solver at all — 300 frames of
/// `iter_count = 0`, ~0.07 ms of predictor return — so `struck99 > 5 × quiet99`
/// was a real solve measured against a no-op, cleared by ~600× by any frame
/// taking a single iteration. Mutation confirmed it: scaling the strike to
/// `0.1 %` (880× shallower, no impact regime, at most 2 iterations) left this
/// gate green while every other gate in the file failed. Both arms below now
/// catch it — though the deflection arm fires FIRST, at `d/L = 1.23e-5`, so the
/// iteration arm is never evaluated on that particular mutation. Each is
/// independently reachable; neither is load-bearing alone.
///
/// ★ It also takes wall time off CI's path. `quality-gate.yml` runs eleven
/// binaries in one `cargo test --release` with no `--test-threads=1`, so these
/// gates execute concurrently, each driving a rayon-parallel solver — and
/// `stick_flex.rs` keeps every timing instrument behind `require_quiet_box()`
/// for exactly that reason.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn an_unstruck_run_is_flat_and_a_struck_one_is_not() {
    let quiet = run_cell(Strike::VelocityImpulse, 0.0, GRID);
    let struck = run_cell(Strike::VelocityImpulse, 1.0, GRID);
    report_header();
    report_row(Strike::VelocityImpulse, 0.0, &quiet);
    report_row(Strike::VelocityImpulse, 1.0, &struck);

    assert!(
        quiet.completed() && struck.completed(),
        "both runs must complete: {:?} / {:?}",
        quiet.ended_early,
        struck.ended_early
    );
    assert!(
        quiet.peak_ratio < 1e-12,
        "the unstruck run deflected to d/L = {:.3e} — it is being driven by \
         something this file does not model, so it is not a control",
        quiet.peak_ratio
    );
    // ★ The companion arm the negative control needs: the strike must actually
    // bend the stick. Without it, a strike applied to the wrong axis still costs
    // iterations and still passes everything below.
    assert!(
        struck.peak_ratio > 1e-3,
        "the struck run only reached d/L = {:.3e}. There is no impact regime here, \
         so nothing below certifies that this file's p99 reads an impact",
        struck.peak_ratio
    );

    let quiet_max = quiet.iters.iter().copied().max().unwrap_or(0);
    let struck_it99 = percentile(
        &struck.iters.iter().map(|&i| i as f64).collect::<Vec<_>>(),
        0.99,
    );
    println!("  quiet max {quiet_max} iters vs struck p99 {struck_it99:.0} iters");
    assert!(
        quiet_max <= 1,
        "the unstruck run took up to {quiet_max} Newton iterations in a frame, so \
         \"flat\" is not what this control is measuring"
    );
    // Piloted at 37. `30` leaves room for the impact frame to get cheaper without
    // letting the strike vanish: the mutation that scaled it to 0.1 % read 2.
    assert!(
        struck_it99 >= 30.0,
        "a struck run's iteration p99 is {struck_it99:.0}, not the tens an impact \
         costs. The p99 this file reports is then not reading an impact"
    );
}

/// **What repeated striking actually buys — measured, because the first version
/// of this file only asserted it.**
///
/// Mutation testing set [`STRIKE_PERIOD`] past [`RUN_FRAMES`] — one strike per
/// run instead of ten — and every other gate in this file kept passing. So the
/// design choice was undefended, and the prose defending it was untested.
///
/// What the interval changes is **which frame the `p99` lands on**:
///
/// ```text
///   10 strikes   p99 = 37 iters, worst = 37 iters   ratio 1.00  <- p99 IS an impact frame
///    1 strike    p99 = 14 iters, worst = 37 iters   ratio 0.38  <- p99 is a RING-DOWN frame
/// ```
///
/// ★★ **The worst frame is the same in both runs, which is the control that
/// makes the rest readable** — the strike is identical and only the sampling of
/// it differs. Without it, a higher `p99` under repeated striking could equally
/// mean the stick was being driven harder.
///
/// ⚠⚠ **Everything here is asserted on Newton ITERATIONS, not milliseconds, and
/// that is not a stylistic choice.** The first version of this gate controlled
/// on `max` wall time and failed on its own control: the worst frame read
/// `39.92 ms` on one run and `58.29 ms` on the next — a `1.46x` swing, because a
/// `max` over 300 frames is a **single sample** and one OS hiccup owns it.
/// Iteration counts are deterministic for a given trajectory, so they carry the
/// claim; the wall times are printed beside them as the human-facing number and
/// asserted on nowhere.
///
/// ★ That is worth carrying out of this file: **VR cares about the worst frame,
/// and the worst frame is the statistic this box measures least reliably.** A
/// `p99` over a sample designed to contain impacts is the strongest honest
/// upper statistic available here.
///
/// ⚠ The `p50` also roughly doubles (`~1.15 -> ~2.27 ms` across quiet reps), a
/// real and separate
/// effect: struck three times a second the stick is never fully quiet, so the
/// median frame is doing work too. That is duty cycle, not tail sampling, and it
/// is reported rather than folded into the claim.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn striking_once_leaves_the_p99_blind_to_the_impact() {
    let many = run_cell_every(Strike::VelocityImpulse, 1.0, GRID, STRIKE_PERIOD);
    let once = run_cell_every(Strike::VelocityImpulse, 1.0, GRID, RUN_FRAMES * 2);
    report_header();
    report_row(Strike::VelocityImpulse, 1.0, &many);
    report_row(Strike::VelocityImpulse, 1.0, &once);
    assert!(
        many.completed() && once.completed(),
        "both runs must complete: {:?} / {:?}",
        many.ended_early,
        once.ended_early
    );

    let it99 = |r: &Run| percentile(&r.iters.iter().map(|&i| i as f64).collect::<Vec<_>>(), 0.99);
    let worst = |r: &Run| r.iters.iter().copied().max().unwrap_or(0) as f64;
    let (many99, once99) = (it99(&many), it99(&once));
    let (many_worst, once_worst) = (worst(&many), worst(&once));
    println!(
        "  {} strikes: p99 {many99:.0} of {many_worst:.0} iters = {:.2} | 1 strike: \
         p99 {once99:.0} of {once_worst:.0} = {:.2}. Wall time p50 {:.2} -> {:.2} ms \
         (duty cycle), p99 {:.2} -> {:.2} ms",
        RUN_FRAMES / STRIKE_PERIOD,
        many99 / many_worst,
        once99 / once_worst,
        percentile(&once.ms, 0.5),
        percentile(&many.ms, 0.5),
        percentile(&once.ms, 0.99),
        percentile(&many.ms, 0.99),
    );

    // ★ The control first: same strike, so the worst frame must take the same
    // number of iterations. If it does not, the two runs are different
    // experiments and nothing below compares.
    assert!(
        (many_worst - once_worst).abs() < 1.0,
        "the worst frame took {many_worst:.0} Newton iterations under repeated \
         striking and {once_worst:.0} under one. The two runs are not delivering \
         the same strike, so the p99 comparison below is confounded"
    );
    assert!(
        many99 > 0.9 * many_worst,
        "with {} strikes the p99 ({many99:.0} iterations) is not within 10 % of the \
         worst frame ({many_worst:.0}), so it is not reading the impact regime and \
         STRIKE_PERIOD is not doing the job this file claims for it",
        RUN_FRAMES / STRIKE_PERIOD
    );
    assert!(
        once99 < 0.6 * once_worst,
        "with ONE strike the p99 ({once99:.0} iterations) already reaches {:.0} % of \
         the worst frame ({once_worst:.0}). Repeated striking then buys nothing, and \
         the module docs — and STRIKE_PERIOD itself — should say so",
        100.0 * once99 / once_worst
    );
}

// ---------------------------------------------------------------------------
// Probe 1 — the load path, or the starting point?
// One BE step at a time, so nothing accumulates across frames.
// ---------------------------------------------------------------------------

/// One backward-Euler step from rest, with the blade band's velocity seeded.
struct StepProbe {
    /// Where `x + dt·v` puts the tip, in metres.
    guess_tip: f64,
    converged: bool,
    iters: usize,
    r_norm: f64,
    /// Tip displacement of the state the solve actually reached.
    final_tip: f64,
    /// `‖x_guess − x_final‖ / ‖x_final − x₀‖` — how far the initial guess sits
    /// from the answer, as a fraction of how far the answer sits from rest.
    /// `0` is a perfect guess, `1` is starting at rest.
    guess_err: f64,
}

/// Take exactly one BE step from rest under `load`, starting from `x + dt·v`
/// with `v` seeded on the blade band.
///
/// ⚠ Deliberately a **single step**, not a 300-frame run. The question here is
/// about one solve's basin, and a multi-frame run re-seeds at every strike and
/// carries state between them, so its "peak deflection" is a trajectory
/// property rather than this equation's answer.
fn probe_step(load: f64, seed_tip_m: f64, guess_mode: InitialGuess) -> StepProbe {
    let (mu, lambda) = lame_for(e_eff_for(EI_TARGET));
    let field = MaterialField::uniform(mu, lambda);
    let (nx, ny, nz) = GRID;
    let tet4 = HandBuiltTetMesh::cantilever_bilayer_beam(nx, ny, nz, SPAN, WIDTH, DEPTH, &field);
    let mesh = Tet10Mesh::from_tet4(&tet4);
    let n_dof = 3 * mesh.n_vertices();

    let (x0, rest_z, bc, mut cfg) = rig(&mesh, slapshot_load(EI_TARGET), rho_eff(), false, TOL_REL);
    cfg.dt = VR_DT;
    cfg.initial_guess = guess_mode;
    let tol = cfg.tol;
    let loaded = bc.loaded_vertices.clone();
    let n_loaded = loaded.len();
    let solver: CpuTet10NHSolver<Tet10Mesh> =
        CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);

    let mut v = vec![0.0; n_dof];
    for &(vertex, _) in &loaded {
        v[3 * vertex as usize + 2] += seed_tip_m / VR_DT;
    }
    let theta = Tensor::from_slice(&[load / n_loaded as f64], &[1]);
    let step = solver.try_replay_step(
        &Tensor::from_slice(&x0, &[n_dof]),
        &Tensor::from_slice(&v, &[n_dof]),
        &theta,
        VR_DT,
    );
    let r_norm = step.as_ref().map_or(f64::NAN, |s| s.final_residual_norm);
    let (x_final, iters, failure) = outcome_of(step, tol, &x0);

    // The guess Newton actually starts from, which is what `guess_mode` selects.
    let guess: Vec<f64> = match guess_mode {
        InitialGuess::Inertial => x0
            .iter()
            .zip(&v)
            .map(|(&xi, &vi)| vi.mul_add(VR_DT, xi))
            .collect(),
        // `InitialGuess` is `#[non_exhaustive]`; anything added later starts
        // somewhere this probe does not model, so it must not be reported as if
        // it did.
        InitialGuess::PreviousState => x0.clone(),
        // Same convention as this file's `expect_used` allowance: in a test, a
        // case the probe cannot model must abort loudly rather than be reported
        // as if it had been handled.
        #[allow(clippy::panic)]
        _ => panic!("probe_step models PreviousState and Inertial only, got {guess_mode:?}"),
    };
    let norm = |a: &[f64], b: &[f64]| {
        a.iter()
            .zip(b)
            .map(|(p, q)| (p - q) * (p - q))
            .sum::<f64>()
            .sqrt()
    };
    let travel = norm(&x_final, &x0);
    StepProbe {
        guess_tip: seed_tip_m,
        converged: failure.is_none(),
        iters,
        r_norm: failure.map_or(r_norm, |(_, r, _, _)| r),
        final_tip: tip_of(&x_final, &loaded, &rest_z),
        guess_err: if travel > 0.0 {
            norm(&guess, &x_final) / travel
        } else {
            f64::NAN
        },
    }
}

/// ★★★ **Seeding velocity does NOT hold the problem fixed — it changes the
/// equation. So the previous diagnostic could not have shown what it claimed.**
///
/// A backward-Euler step solves
///
/// ```text
///   M (x − x₀ − dt·v₀) / dt²  +  f_int(x)  =  F
/// ```
///
/// and **`v₀` is in it**. Seeding the blade band's velocity therefore adds
/// momentum to the *problem*, not just to the starting point — so a seed sweep
/// cannot be described as "the load held fixed, only the guess moved". An
/// earlier producer in this file did exactly that and has been deleted; this
/// ladder is what falsified it. The answer moves with the seed, which
/// this ladder shows directly: at `5.4 N` the converged tip runs `4.90 → 6.07 mm`
/// **monotonically** across the swept seeds, `0.2 → 2.0`. (An earlier draft said
/// `4.84` at `seed 0.1` — a value from a finer grid this loop no longer walks;
/// it was also inconsistent with the `+23.8 %` the same code prints.)
///
/// ⇒ What that experiment really established is narrower and still useful: **a
/// BE step from REST under a suddenly applied load is the hard case, and any
/// initial momentum makes it tractable.** Whether the help comes from the easier
/// equation or from the better starting point, it could not separate — because
/// `InitialGuess::Inertial` ties them together.
///
/// [`whether_the_starting_point_or_the_momentum_does_the_work`] separates them.
#[test]
#[ignore = "diagnostic — run explicitly"]
fn seeding_velocity_moves_the_answer_not_just_the_guess() {
    for magnitude in [0.10, 1.00] {
        let load = slapshot_load(EI_TARGET) * magnitude;
        let unit = one_frame_response(load);
        println!("\n=== ONE BE step from rest, load {load:.1} N. Does the ANSWER move? ===");
        println!(
            "{:>6} {:>11} {:>11} {:>10} {:>7} {:>11} {:>9}",
            "seed", "guess (mm)", "answer (mm)", "guess err", "iters", "r_norm", "outcome"
        );
        let mut answers: Vec<f64> = Vec::new();
        for i in 0..=10 {
            let seed = f64::from(i) * 0.2;
            let p = probe_step(load, seed * unit, InitialGuess::Inertial);
            println!(
                "{seed:>6.2} {:>11.2} {:>11.2} {:>10.3} {:>7} {:>11.3e} {:>9}",
                1e3 * p.guess_tip,
                1e3 * p.final_tip,
                p.guess_err,
                p.iters,
                p.r_norm,
                if p.converged { "ok" } else { "FAILED" },
            );
            if p.converged {
                answers.push(p.final_tip);
            }
        }
        assert!(
            answers.len() >= 3,
            "only {} seeds converged at {load:.1} N — too few to say the answer moves",
            answers.len()
        );
        // ★ The claim is that the answer moves WITH the seed, monotonically. A
        // rise that is not monotone would mean something other than the added
        // momentum is driving it.
        let rises = answers.windows(2).filter(|w| w[1] > w[0]).count();
        let spread = (answers[answers.len() - 1] - answers[0]) / answers[0];
        println!(
            "  answer rose on {rises} of {} steps, total {:.1} % — the seed is in the \
             EQUATION, not only in the guess",
            answers.len() - 1,
            100.0 * spread
        );
        assert!(
            rises == answers.len() - 1 && spread > 0.05,
            "the converged answer did not rise monotonically with the seed ({rises} of \
             {} steps, {:.1} % total). Seeding would then NOT be changing the \
             equation, and the previous diagnostic's framing would have been right \
             after all",
            answers.len() - 1,
            100.0 * spread
        );
    }
}

/// ★★★ **The separator: same equation, two starting points.**
///
/// `InitialGuess::PreviousState` starts Newton at `x₀`; `Inertial` starts it at
/// `x₀ + dt·v₀`. Run both at the **same `v₀`** and the equation is identical —
/// only the starting point differs. That is the comparison
/// [`seeding_velocity_moves_the_answer_not_just_the_guess`] shows the seed sweep
/// could not make.
///
/// - If both converge to the same answer and `Inertial` merely takes fewer
///   iterations, the starting point is a **speed** lever, and what rescues the
///   force wall is the added momentum.
/// - If `Inertial` converges where `PreviousState` does not, the starting point
///   is a **correctness** lever and the original framing was right.
///
/// ★★ The control is that both reach the **same answer** — they are solving one
/// equation, so a disagreement means Newton's landing spot depends on where it
/// set off, and no deflection in this file would mean anything.
///
/// ⚠ At `seed = 0` the two modes give *identical* guesses (`x₀ + dt·0 = x₀`), so
/// that row is a built-in sanity check: it must read the same on both sides.
#[test]
#[ignore = "diagnostic — run explicitly"]
fn whether_the_starting_point_or_the_momentum_does_the_work() {
    for magnitude in [0.10, 1.00] {
        let load = slapshot_load(EI_TARGET) * magnitude;
        let unit = one_frame_response(load);
        println!(
            "\n=== ONE BE step, load {load:.1} N, SAME v0 both sides — only the \
             starting point differs ==="
        );
        println!(
            "{:>6} {:>26} {:>26} {:>12}",
            "seed", "PreviousState (x0)", "Inertial (x0 + dt*v0)", "answers"
        );
        println!(
            "{:>6} {:>9} {:>7} {:>9} {:>9} {:>7} {:>9} {:>12}",
            "", "answer", "iters", "outcome", "answer", "iters", "outcome", "agree?"
        );
        let mut both = 0usize;
        for i in 0..=10 {
            let seed = f64::from(i) * 0.2;
            let prev = probe_step(load, seed * unit, InitialGuess::PreviousState);
            let iner = probe_step(load, seed * unit, InitialGuess::Inertial);
            let agree = if prev.converged && iner.converged {
                format!(
                    "{:.2e}",
                    (prev.final_tip - iner.final_tip).abs() / iner.final_tip
                )
            } else {
                "-".to_owned()
            };
            println!(
                "{seed:>6.2} {:>9.2} {:>7} {:>9} {:>9.2} {:>7} {:>9} {agree:>12}",
                1e3 * prev.final_tip,
                prev.iters,
                if prev.converged { "ok" } else { "FAILED" },
                1e3 * iner.final_tip,
                iner.iters,
                if iner.converged { "ok" } else { "FAILED" },
            );
            if prev.converged && iner.converged {
                both += 1;
                let rel = (prev.final_tip - iner.final_tip).abs() / iner.final_tip;
                assert!(
                    rel < 1e-3,
                    "at seed {seed:.2} the two starting points converged to DIFFERENT \
                     answers ({:.4} vs {:.4} mm, {rel:.2e} apart) while solving the \
                     same equation. Newton's landing spot then depends on where it \
                     set off, and no deflection in this file means anything",
                    1e3 * prev.final_tip,
                    1e3 * iner.final_tip
                );
            }
        }
        // ⚠ Say so when the agreement control never ran. It is conditional on
        // both sides converging, and if `PreviousState` fails everywhere the
        // control is vacuous — which is a result, not a pass.
        println!(
            "  both sides converged on {both} of 11 rows{}",
            if both == 0 {
                " — the same-answer control did NOT run"
            } else {
                ""
            }
        );
    }
}

// ---------------------------------------------------------------------------
// Probe 2 — same distance from the answer, different direction.
// Holds `p` fixed so the equation cannot move while the start does.
// ---------------------------------------------------------------------------

/// The axial profile a guess-error is given, as a function of rest `x/L`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Shape {
    /// Non-zero on the blade band only — the field a velocity seed imposes, and
    /// a discontinuous one.
    Band,
    /// Linear in `x`: a rigid rotation about the clamp. Smooth, and wrong in
    /// curvature everywhere.
    Rotation,
    /// `3(x/L)² − (x/L)³`, the static deflection curve of a tip-loaded
    /// cantilever — **the shape the answer itself has**.
    TipLoadCurve,
    /// The first free-vibration mode of a clamped-free beam. Smooth, and the
    /// shape a struck stick actually rings in.
    FirstMode,
}

impl Shape {
    const fn label(self) -> &'static str {
        match self {
            Self::Band => "band",
            Self::Rotation => "rotation",
            Self::TipLoadCurve => "tip-load",
            Self::FirstMode => "mode-1",
        }
    }

    /// Profile value at rest coordinate `s = x/L`, unnormalised.
    fn at(self, s: f64) -> f64 {
        match self {
            Self::Band => f64::from(s > 1.0 - 1e-9),
            Self::Rotation => s,
            Self::TipLoadCurve => 3.0 * s * s - s * s * s,
            Self::FirstMode => {
                // Clamped-free mode 1: cosh−cos − σ(sinh−sin), βL = 1.8751,
                // σ = 0.734_096. Standard Euler–Bernoulli eigenfunction.
                let b = 1.875_104_068_711_961 * s;
                (b.cosh() - b.cos()) - 0.734_096 * (b.sinh() - b.sin())
            }
        }
    }
}

/// The fixed-equation rig the shape probe runs on: one `p`, one answer, and a
/// solver that starts Newton wherever it is handed.
struct ShapeRig {
    n_dof: usize,
    /// Rest `x/L` per vertex, for evaluating a [`Shape`] profile.
    axial: Vec<f64>,
    x_rest: Vec<f64>,
    /// The answer this equation has, found by starting at `p`.
    x_star: Vec<f64>,
    /// `p = x₀ + dt·v₀`. The equation depends on **only** this.
    p: Vec<f64>,
    theta_value: f64,
    tol: f64,
    solver: CpuTet10NHSolver<Tet10Mesh>,
    star_iters: usize,
    /// `‖x* − x_rest‖`, the yardstick every error is scaled against.
    travel: f64,
}

/// Euclidean distance between two DOF vectors.
fn dof_distance(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(u, w)| (u - w) * (u - w))
        .sum::<f64>()
        .sqrt()
}

impl ShapeRig {
    fn build() -> Self {
        let (mu, lambda) = lame_for(e_eff_for(EI_TARGET));
        let field = MaterialField::uniform(mu, lambda);
        let (nx, ny, nz) = GRID;
        let tet4 =
            HandBuiltTetMesh::cantilever_bilayer_beam(nx, ny, nz, SPAN, WIDTH, DEPTH, &field);
        let mesh = Tet10Mesh::from_tet4(&tet4);
        let n_dof = 3 * mesh.n_vertices();
        let axial: Vec<f64> = mesh.positions().iter().map(|q| q.x / SPAN).collect();

        let load = slapshot_load(EI_TARGET);
        let (x_rest, _rest_z, bc, mut cfg) = rig(&mesh, load, rho_eff(), false, TOL_REL);
        cfg.dt = VR_DT;
        let loaded = bc.loaded_vertices.clone();
        let theta_value = load / loaded.len() as f64;

        // `p` for a strike known to converge, so there is an answer to aim at.
        let seed_tip = 0.2 * one_frame_response(load);
        let mut p = x_rest.clone();
        for &(vertex, _) in &loaded {
            p[3 * vertex as usize + 2] += seed_tip;
        }

        let mut cfg_a = cfg;
        cfg_a.initial_guess = InitialGuess::Inertial;
        // ⚠⚠ Enriched ONCE and cloned. Every `VertexId` in the boundary
        // conditions and every index in `axial` is derived from this instance, so
        // handing the solvers separately-enriched meshes would apply those
        // indices to different objects. It is correct today only because
        // `from_tet4` numbers midside nodes deterministically — asserted nowhere,
        // and `Tet10Mesh` carries curvature/projection constructors whose future
        // use would break it. If the numbering ever diverged, the clamp and the
        // load would land on the wrong nodes and every reported deflection would
        // be wrong while every gate stayed green.
        let solver_a: CpuTet10NHSolver<Tet10Mesh> =
            CpuNewtonSolver::new(Tet10, mesh.clone(), NullContact, cfg_a, bc.clone());
        let v_star: Vec<f64> = p
            .iter()
            .zip(&x_rest)
            .map(|(pi, xi)| (pi - xi) / VR_DT)
            .collect();
        let star = solver_a
            .try_replay_step(
                &Tensor::from_slice(&x_rest, &[n_dof]),
                &Tensor::from_slice(&v_star, &[n_dof]),
                &Tensor::from_slice(&[theta_value], &[1]),
                VR_DT,
            )
            .expect("the reference solve must converge or there is no answer to aim at");

        let mut cfg_b = cfg;
        cfg_b.initial_guess = InitialGuess::PreviousState;
        let solver: CpuTet10NHSolver<Tet10Mesh> =
            CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg_b, bc);

        let travel = dof_distance(&star.x_final, &x_rest);
        Self {
            n_dof,
            axial,
            x_rest,
            x_star: star.x_final,
            p,
            theta_value,
            tol: cfg_b.tol,
            solver,
            star_iters: star.iter_count,
            travel,
        }
    }

    /// Solve the fixed equation starting from `start`.
    ///
    /// ★ `v_prev` is chosen as `(p − start)/dt` so that `x_prev + dt·v_prev = p`
    /// no matter where `start` is — the equation is untouched, only Newton's
    /// starting point moves.
    fn run_from(&self, start: &[f64]) -> (bool, usize, Vec<f64>) {
        let v: Vec<f64> = self
            .p
            .iter()
            .zip(start)
            .map(|(pi, si)| (pi - si) / VR_DT)
            .collect();
        let step = self.solver.try_replay_step(
            &Tensor::from_slice(start, &[self.n_dof]),
            &Tensor::from_slice(&v, &[self.n_dof]),
            &Tensor::from_slice(&[self.theta_value], &[1]),
            VR_DT,
        );
        let (x_final, iters, failure) = outcome_of(step, self.tol, &self.x_rest);
        (failure.is_none(), iters, x_final)
    }

    /// A start displaced from `x*` by `err_frac · travel`, along `shape`.
    ///
    /// Returns the start and the cosine between the error and the answer's own
    /// displacement from rest — "how much does this error look like the thing
    /// Newton is trying to find", with magnitude divided out.
    fn start_along(&self, shape: Shape, err_frac: f64) -> (Vec<f64>, f64) {
        let mut dir = vec![0.0; self.n_dof];
        for (v, &s) in self.axial.iter().enumerate() {
            dir[3 * v + 2] = shape.at(s);
        }
        let mag = dir.iter().map(|d| d * d).sum::<f64>().sqrt();
        let scale = err_frac * self.travel / mag;
        let start: Vec<f64> = self
            .x_star
            .iter()
            .zip(&dir)
            .map(|(xs, d)| d.mul_add(scale, *xs))
            .collect();
        let (mut dot, mut na, mut nb) = (0.0, 0.0, 0.0);
        for ((si, xs), xr) in start.iter().zip(&self.x_star).zip(&self.x_rest) {
            let (a, b) = (si - xs, xs - xr);
            dot += a * b;
            na += a * a;
            nb += b * b;
        }
        (start, dot / (na.sqrt() * nb.sqrt()))
    }
}

/// Guess-error magnitudes swept, as a fraction of `‖x* − x_rest‖`.
const ERR_FRACTIONS: [f64; 4] = [0.25, 0.50, 1.00, 2.00];

/// The rung the published shape ratio is quoted at — the smallest, where the
/// band/smooth gap is widest and least confounded by the error's own amplitude.
const RATIO_ERR_FRAC: f64 = ERR_FRACTIONS[0];

/// ★★★ **Same distance from the answer, different DIRECTION. Does the shape of
/// the error set the iteration count?**
///
/// The previous diagnostic left a sharp anomaly: a guess sitting at `19 %` of the
/// answer converged in `6` iterations while one at `88 %` took `51`, and the
/// full-vector distance agreed the second was *closer*. So distance is not what
/// Newton is paying for. The hypothesis is **shape** — a velocity seed puts its
/// error on the blade band alone, a discontinuous field, so a larger seed buys a
/// righter tip on a more wrongly-shaped guess.
///
/// ★★ **The design this rests on.** A backward-Euler step solves
/// `M(x − p)/dt² + f_int(x) = F` with `p = x₀ + dt·v₀`, so the equation — and
/// therefore the answer — **depends only on `p`**. Fixing `p` and choosing `x₀'`
/// freely with `v₀' = (p − x₀')/dt` puts Newton's starting point anywhere while
/// leaving the problem untouched. `InitialGuess::Inertial` cannot do this: it
/// always starts *at* `p`, the very point that defines the equation, so under
/// `Inertial` the guess and the problem are one knob. That is why the earlier
/// seed sweep could not separate them.
///
/// Two controls make the reading safe:
///
/// - **`err = 0` must converge immediately.** Needing real iterations from `x*`
///   would mean `x*` is not the answer and every distance is measured from the
///   wrong point.
/// - **every start must land on the same `x*`.** They share one equation; a
///   disagreement means `x_prev` does more than feed the inertia term and the
///   whole construction is invalid.
#[test]
#[ignore = "diagnostic — run explicitly"]
fn whether_the_shape_of_the_guess_error_sets_the_iteration_count() {
    let r = ShapeRig::build();
    println!(
        "\n=== SAME equation (p fixed), SAME |error|, different error SHAPE ===\n  \
         load {:.1} N, answer reached from p in {} iters, |x* − x_rest| = {:.3} mm",
        slapshot_load(EI_TARGET),
        r.star_iters,
        1e3 * r.travel
    );

    let (ok0, iters0, _) = r.run_from(&r.x_star);
    println!(
        "  control: starting AT x* -> {iters0} iters, {}",
        if ok0 { "ok" } else { "FAILED" }
    );
    assert!(
        ok0 && iters0 <= 1,
        "starting at x* took {iters0} iterations, so x* is not the answer and every \
         distance below is measured from the wrong point"
    );

    println!(
        "{:>10} {:>8} {:>9} {:>8} {:>10} {:>9}",
        "shape", "|err|", "cos", "iters", "answer err", "outcome"
    );
    let mut converged_shapes: std::collections::BTreeSet<&'static str> =
        std::collections::BTreeSet::new();
    let (mut band_iters, mut smooth_iters): (Option<usize>, Option<usize>) = (None, None);
    for err_frac in ERR_FRACTIONS {
        for shape in [
            Shape::Band,
            Shape::Rotation,
            Shape::TipLoadCurve,
            Shape::FirstMode,
        ] {
            let (start, cos) = r.start_along(shape, err_frac);
            let (ok, iters, x_final) = r.run_from(&start);
            let answer_err = dof_distance(&x_final, &r.x_star) / r.travel;
            println!(
                "{:>10} {err_frac:>8.2} {cos:>9.3} {iters:>8} {answer_err:>10.2e} {:>9}",
                shape.label(),
                if ok { "ok" } else { "FAILED" }
            );
            if ok {
                converged_shapes.insert(shape.label());
                if (err_frac - RATIO_ERR_FRAC).abs() < 1e-12 {
                    if shape == Shape::Band {
                        band_iters = Some(iters);
                    } else {
                        smooth_iters = Some(smooth_iters.map_or(iters, |b: usize| b.min(iters)));
                    }
                }
                assert!(
                    answer_err < 1e-3,
                    "{} at |err| {err_frac} converged to a DIFFERENT answer \
                     ({answer_err:.2e} away). `x_prev` does more than feed the inertia \
                     term, so holding `p` fixed does not hold the equation fixed and \
                     this construction is invalid",
                    shape.label()
                );
            }
        }
    }
    // ⚠ Counts distinct SHAPES, not rows. `converged >= 4` counted rows, and 4 is
    // exactly what ONE shape contributes when it converges at all four `err_frac`
    // values and the other three fail everywhere — so the guard passed on a table
    // with no cross-shape comparison left in it, which is the only thing this
    // diagnostic exists to provide.
    assert!(
        converged_shapes.len() >= 2,
        "only {} distinct shape(s) converged ({converged_shapes:?}) — a comparison \
         BETWEEN shapes needs at least two, and this table is the sole backing for \
         the 47x figure quoted elsewhere",
        converged_shapes.len()
    );

    // ★★★ **The `47x` this file's headline rests on, COMPUTED and gated.**
    //
    // It used to be read off the printed table by eye, with nothing asserting it
    // — and the iterations column cannot be read safely by eye either, because a
    // row that hits `max_newton_iter` reports the CAP as its iteration count and
    // prints indistinguishably from a converged one. `run_from` returns whether
    // the row converged, and both rows feeding the ratio are required to have.
    let (band, smooth) = (
        band_iters.expect("the Band row at the ratio's err_frac must converge"),
        smooth_iters.expect("at least one smooth row at the ratio's err_frac must converge"),
    );
    let ratio = band as f64 / smooth as f64;
    println!(
        "  at |err| {RATIO_ERR_FRAC}: band {band} iterations against {smooth} for the \
         best smooth shape — {ratio:.1}x"
    );
    // Piloted at 234/5 = 46.8x. `10x` keeps a wide margin while refusing to let
    // the headline survive a collapse of the effect it names.
    assert!(
        ratio > 10.0,
        "a band-localised guess error cost {ratio:.1}x a smooth one at matched \
         distance ({band} against {smooth} iterations), not the order of magnitude \
         this file and `InitialGuess`'s docs claim. The shape finding does not hold \
         as measured"
    );
}

// ---------------------------------------------------------------------------
// Probe 3 — a real impact frame, and the same intervention end to end.
// Mid-trajectory rather than from rest; this is the claim that ships.
// ---------------------------------------------------------------------------

/// Where Newton is started on a real impact frame.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Start {
    /// `x₀ + dt·v₀` — exactly what `InitialGuess::Inertial` does today, and
    /// band-shaped whenever the strike is.
    AtP,
    /// `x₀` — what `InitialGuess::PreviousState` does. The control.
    AtPrevious,
    /// The predicted tip displacement, redistributed along the beam on a smooth
    /// profile instead of dumped on the blade band.
    Smoothed(Shape),
    /// `x₀ + Φ_S Φ_Sᵀ M (p − x₀)` — the predicted increment with everything
    /// outside a band of the stick's own modes removed. See Probe 4.
    Modal(ModalArm),
}

impl Start {
    fn label(self) -> String {
        match self {
            Self::AtP => "p (Inertial)".to_owned(),
            Self::AtPrevious => "x0 (Previous)".to_owned(),
            Self::Smoothed(s) => format!("smooth:{}", s.label()),
            Self::Modal(a) => format!("modal:{} r={}", a.band.label(), a.rank),
        }
    }
}

/// ★★★ **On a REAL impact frame, does a smoothed start cut the iteration count?**
///
/// [`whether_the_shape_of_the_guess_error_sets_the_iteration_count`] showed a
/// band-localised error costs `47×` a smooth one at matched distance, on a
/// *synthetic* error. This is the claim that matters: the same intervention on an
/// impact frame the game would actually produce.
///
/// The stick is advanced through a full strike and its ring-down under the normal
/// `Inertial` path, and then — at the next strike — `p = x₀ + dt·v₀` is held
/// **exactly** and only Newton's starting point is varied. The physics is
/// untouched: same load, same momentum, same answer. Only the guess moves.
///
/// ★★ The control is that every start reaches the **same** `x_final`. They share
/// one equation. A disagreement means this is changing the simulation rather
/// than the way it is solved, and any speedup would be worthless.
#[test]
#[ignore = "diagnostic — run explicitly"]
fn whether_a_smoothed_start_cuts_iterations_on_a_real_impact_frame() {
    for magnitude in [0.25, 1.00, 2.00, 8.00] {
        let probe = ImpactFrame::at_strike(magnitude, STRIKE_PERIOD);
        println!(
            "\n=== REAL impact frame: impulse {magnitude:.2}x, struck at frame {}, \
             after one prior strike ===\n  p is {:.1} mm from x0 at the tip; the \
             answer is {:.1} mm from x0. **Inertial baseline: {} iters**",
            STRIKE_PERIOD,
            1e3 * probe.p_tip,
            1e3 * probe.answer_tip,
            probe.star_iters,
        );
        println!(
            "{:>15} {:>8} {:>11} {:>10} {:>9}",
            "start", "iters", "start->x*", "answer err", "outcome"
        );
        // ★ The baseline is the REAL `Inertial` path, not the `AtP` row. `AtP`
        // reproduces it exactly whenever `p` is a valid configuration — and when
        // it is not, `AtP` dies on a validity check that `Inertial` never
        // performs, because `Inertial` keeps `x_prev = x0`.
        let baseline = probe.star_iters;
        let mut at_p: Option<usize> = None;
        let mut best = usize::MAX;
        for start in [
            Start::AtP,
            Start::AtPrevious,
            Start::Smoothed(Shape::TipLoadCurve),
            Start::Smoothed(Shape::FirstMode),
            Start::Smoothed(Shape::Rotation),
        ] {
            let (ok, iters, dist, answer_err) = probe.run(start, None);
            println!(
                "{:>15} {iters:>8} {dist:>11.3} {answer_err:>10.2e} {:>9}",
                start.label(),
                if ok { "ok" } else { "FAILED" }
            );
            if ok {
                assert!(
                    answer_err < 1e-3,
                    "{} reached a DIFFERENT answer ({answer_err:.2e} away) on the same \
                     equation — this is changing the simulation, not the way it is \
                     solved",
                    start.label()
                );
                if start == Start::AtP {
                    at_p = Some(iters);
                } else if !matches!(start, Start::AtPrevious) {
                    best = best.min(iters);
                }
            }
        }
        // Cross-check: where `p` is valid, the fixed-`p` construction must
        // reproduce the real path exactly. If it ever disagrees while both
        // converge, the construction is not neutral and every row is suspect.
        match at_p {
            Some(i) => assert!(
                i == baseline,
                "the fixed-p construction took {i} iterations from `p` where the real \
                 Inertial path took {baseline}. Holding `p` fixed is then NOT \
                 equivalent to what the solver does, and this whole table is invalid"
            ),
            None => println!(
                "  note: starting AT p failed a validity check — `p` is an inverted \
                 configuration here, which the real Inertial path never has to hold as \
                 `x_prev`. Baseline stays the real path's {baseline} iters."
            ),
        }
        if best == usize::MAX {
            println!("  no smoothed start converged — smoothing does NOT transfer here");
        } else {
            println!(
                "  best smoothed start: {best} iters against {baseline} for Inertial \
                 — {:.2}x",
                baseline as f64 / best as f64
            );
        }
    }
}

/// A real impact frame, captured mid-trajectory: the state just after the strike
/// velocity lands, with `p` pinned so only the starting point can vary.
struct ImpactFrame {
    n_dof: usize,
    axial: Vec<f64>,
    x0: Vec<f64>,
    /// `x₀ + dt·v₀`. Fixed — it defines the equation.
    p: Vec<f64>,
    /// Tip displacement of `p` relative to `x₀`.
    p_tip: f64,
    /// Tip displacement of the answer relative to `x₀`.
    answer_tip: f64,
    x_star: Vec<f64>,
    /// Iterations the REAL `Inertial` path took on this frame — `x_prev = x₀`,
    /// `v_prev = v₀`, guess at `p`. ★ This is the baseline, not the `AtP` row:
    /// `AtP` hands the solver `x_prev = p`, and when `p` is an inverted
    /// configuration that trips a validity check the real path never sees.
    star_iters: usize,
    /// `‖x* − x₀‖`, the yardstick distances are reported against.
    travel: f64,
    theta_value: f64,
    tol: f64,
    solver: CpuTet10NHSolver<Tet10Mesh>,
    loaded: Vec<(VertexId, LoadAxis)>,
}

impl ImpactFrame {
    /// Advance the impulse arm through `warmup` frames under the normal
    /// `Inertial` path, then land the next strike and freeze the frame.
    fn at_strike(magnitude: f64, warmup: usize) -> Self {
        let (mu, lambda) = lame_for(e_eff_for(EI_TARGET));
        let field = MaterialField::uniform(mu, lambda);
        let (nx, ny, nz) = GRID;
        let tet4 =
            HandBuiltTetMesh::cantilever_bilayer_beam(nx, ny, nz, SPAN, WIDTH, DEPTH, &field);
        let mesh = Tet10Mesh::from_tet4(&tet4);
        let n_dof = 3 * mesh.n_vertices();
        let axial: Vec<f64> = mesh.positions().iter().map(|q| q.x / SPAN).collect();

        let (x_rest, rest_z, bc, mut cfg) =
            rig(&mesh, slapshot_load(EI_TARGET), rho_eff(), false, TOL_REL);
        cfg.dt = VR_DT;
        let loaded = bc.loaded_vertices.clone();

        let mut cfg_a = cfg;
        cfg_a.initial_guess = InitialGuess::Inertial;
        // ⚠⚠ Enriched ONCE and cloned. Every `VertexId` in the boundary
        // conditions and every index in `axial` is derived from this instance, so
        // handing the solvers separately-enriched meshes would apply those
        // indices to different objects. It is correct today only because
        // `from_tet4` numbers midside nodes deterministically — asserted nowhere,
        // and `Tet10Mesh` carries curvature/projection constructors whose future
        // use would break it. If the numbering ever diverged, the clamp and the
        // load would land on the wrong nodes and every reported deflection would
        // be wrong while every gate stayed green.
        let advancer: CpuTet10NHSolver<Tet10Mesh> =
            CpuNewtonSolver::new(Tet10, mesh.clone(), NullContact, cfg_a, bc.clone());
        let mut cfg_b = cfg;
        cfg_b.initial_guess = InitialGuess::PreviousState;
        let solver: CpuTet10NHSolver<Tet10Mesh> =
            CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg_b, bc);

        // The impulse arm carries no external load at all.
        let theta = Tensor::from_slice(&[0.0], &[1]);
        let strike_velocity = slapshot_equivalent_tip_velocity() * magnitude;
        let (mut x, mut v) = (x_rest, vec![0.0; n_dof]);
        for k in 0..warmup {
            if k % STRIKE_PERIOD == 0 {
                for &(vertex, _) in &loaded {
                    v[3 * vertex as usize + 2] += strike_velocity;
                }
            }
            one_frame(&advancer, &mut x, &mut v, &theta, VR_DT)
                .expect("the warm-up trajectory must complete or there is no impact frame");
        }
        // The strike under test.
        for &(vertex, _) in &loaded {
            v[3 * vertex as usize + 2] += strike_velocity;
        }
        let x0 = x.clone();
        let p: Vec<f64> = x0
            .iter()
            .zip(&v)
            .map(|(xi, vi)| vi.mul_add(VR_DT, *xi))
            .collect();

        let star = advancer
            .try_replay_step(
                &Tensor::from_slice(&x0, &[n_dof]),
                &Tensor::from_slice(&v, &[n_dof]),
                &theta,
                VR_DT,
            )
            .expect("the Inertial baseline must converge or there is nothing to improve");
        let x_star = star.x_final;
        let tip_of_rel = |a: &[f64]| tip_of(a, &loaded, &rest_z) - tip_of(&x0, &loaded, &rest_z);
        Self {
            n_dof,
            axial,
            p_tip: tip_of_rel(&p),
            answer_tip: tip_of_rel(&x_star),
            star_iters: star.iter_count,
            travel: dof_distance(&x_star, &x0),
            x0,
            p,
            x_star,
            theta_value: 0.0,
            tol: cfg_b.tol,
            solver,
            loaded,
        }
    }

    /// The starting point a [`Start`] selects, at this frame.
    fn start_point(&self, start: Start, modal: Option<&ModalRig>) -> Vec<f64> {
        smoothed_start(start, &self.x0, &self.p, &self.axial, &self.loaded, modal)
    }

    /// Solve the frozen frame from `start`. Returns
    /// `(converged, iterations, ‖start − x*‖/travel, ‖x_final − x*‖/travel)`.
    fn run(&self, start: Start, modal: Option<&ModalRig>) -> (bool, usize, f64, f64) {
        let s = self.start_point(start, modal);
        // ★ `v_prev` is chosen so `x_prev + dt·v_prev = p` regardless of `start`,
        // which is what keeps the equation — and the answer — fixed.
        let v: Vec<f64> = self
            .p
            .iter()
            .zip(&s)
            .map(|(pi, si)| (pi - si) / VR_DT)
            .collect();
        let step = self.solver.try_replay_step(
            &Tensor::from_slice(&s, &[self.n_dof]),
            &Tensor::from_slice(&v, &[self.n_dof]),
            &Tensor::from_slice(&[self.theta_value], &[1]),
            VR_DT,
        );
        let (x_final, iters, failure) = outcome_of(step, self.tol, &self.x0);
        (
            failure.is_none(),
            iters,
            dof_distance(&s, &self.x_star) / self.travel,
            dof_distance(&x_final, &self.x_star) / self.travel,
        )
    }
}

/// One frame stepped from an arbitrary starting point, holding the physics fixed.
///
/// ⚠⚠ **The velocity update uses the TRUE previous position, not the start.**
/// `x_prev` is being used here as a lever on where Newton begins, not as a claim
/// about where the stick was; updating `v` from it would make the trajectory
/// diverge from the one this is supposed to reproduce exactly.
fn one_frame_from<S: Solver>(
    solver: &S,
    x: &mut Vec<f64>,
    v: &mut [f64],
    theta: &Tensor<f64>,
    start: &[f64],
) -> Result<(usize, f64), String> {
    let n_dof = x.len();
    // `p` is what the physics depends on, and it is computed from the true state.
    let p: Vec<f64> = x
        .iter()
        .zip(&*v)
        .map(|(xi, vi)| vi.mul_add(VR_DT, *xi))
        .collect();
    let v_eff: Vec<f64> = p
        .iter()
        .zip(start)
        .map(|(pi, si)| (pi - si) / VR_DT)
        .collect();
    // ⚠⚠ Built BEFORE the clock starts, exactly as `stickrig::one_frame` does.
    // `Tensor::from_slice` is two `to_vec`s, so leaving these inside the timed
    // region charged this path ~4 allocations and ~6.5 KB of memcpy per frame
    // that the shipped path is not charged — a systematic bias, and one pointing
    // the WRONG way, since it inflated the arm being compared against.
    let x_in = Tensor::from_slice(start, &[n_dof]);
    let v_in = Tensor::from_slice(&v_eff, &[n_dof]);
    let t0 = std::time::Instant::now();
    let step = solver
        .try_replay_step(&x_in, &v_in, theta, VR_DT)
        .map_err(|e| describe_failure(&e))?;
    let ms = t0.elapsed().as_secs_f64() * 1e3;
    if !step.x_final.iter().all(|f| f.is_finite()) {
        return Err("non-finite x_final".to_owned());
    }
    for (vi, (xf, xp)) in v.iter_mut().zip(step.x_final.iter().zip(x.iter())) {
        *vi = (xf - xp) / VR_DT;
    }
    x.clone_from(&step.x_final);
    Ok((step.iter_count, ms))
}

/// ★★★ **End to end: does a smoothed start move the `p99` the whole arc is
/// about — without moving the simulation?**
///
/// Per-frame the intervention is worth `1.4–6.3×`
/// ([`whether_a_smoothed_start_cuts_iterations_on_a_real_impact_frame`]). This
/// runs the full `300`-frame trajectory under each strategy and reads the
/// distribution, which is the quantity a VR budget is actually judged on.
///
/// ⚠⚠ **The trajectories DO move, and an earlier version of this file was wrong
/// to say otherwise.** Stepped in lockstep against the baseline, a smoothed run
/// reads:
///
/// ```text
///   frame   1     ~7e-9      solver tolerance — the same equation, the same answer
///   frame  10     ~5e-8
///   frame 300      5.6e-1    56 % apart
/// ```
///
/// So the *equation* is identical — frame 1 is measured before anything can
/// accumulate — but this fixture amplifies a `1e-8` difference to `O(1)` over
/// 300 frames, about `6 %` growth per frame, which is ordinary for a nonlinear
/// oscillator struck ten times. The claim "the trajectory is unchanged to
/// `1.8e-9`" came from comparing `peak_ratio`, a scalar extremum reached right
/// after the FIRST strike, before divergence sets in. It certified nothing — the
/// trace above is what the code actually produces, and `5.6e-1` is the figure to
/// quote.
///
/// ⇒ **The accumulation-free evidence is
/// [`whether_a_smoothed_start_cuts_iterations_on_a_real_impact_frame`]**, which
/// compares ONE frame with `p` pinned and reads `25 → 4` iterations with the
/// answers `2.6e-8` apart. This producer corroborates over a trajectory; it is
/// not the primary evidence, and its two runs are not the same trajectory by the
/// end.
#[test]
#[ignore = "producer — reference box only, run explicitly"]
fn whether_a_smoothed_start_moves_the_p99_end_to_end() {
    let magnitude = 1.0;
    println!(
        "\n=== 300 frames, impulse {magnitude:.2}x, struck every {STRIKE_PERIOD} — \
         SAME physics, different Newton start ==="
    );
    println!(
        "{:>15} {:>8} {:>8} {:>8} {:>7} {:>7} {:>11} {:>9}",
        "start", "p50 ms", "p99 ms", "max ms", "p50 it", "p99 it", "peak d/L", "outcome"
    );
    let report = |label: &str, run: &Run| {
        let it: Vec<f64> = run.iters.iter().map(|&i| i as f64).collect();
        let ok = run.completed();
        println!(
            "{label:>15} {:>8.2} {:>8.2} {:>8.2} {:>7.1} {:>7.1} {:>11.4e} {:>9}",
            if ok {
                percentile(&run.ms, 0.5)
            } else {
                f64::NAN
            },
            if ok {
                percentile(&run.ms, 0.99)
            } else {
                f64::NAN
            },
            run.ms.iter().copied().fold(0.0f64, f64::max),
            if ok { percentile(&it, 0.5) } else { f64::NAN },
            if ok { percentile(&it, 0.99) } else { f64::NAN },
            run.peak_ratio,
            if ok { "ok" } else { "FAILED" },
        );
        if !ok {
            println!("      ended early: {:?}", run.ended_early);
        }
    };

    // ★ The baseline is `AtP` EXPLICITLY, run outside the loop and required to
    // complete. Anchoring it to "the first strategy that completes" let a
    // smoothed run silently become the baseline whenever `AtP` failed — and this
    // file documents `AtP` failing as reachable (it hands the solver
    // `x_prev = p`, which trips a validity check on an inverted `p` that the
    // shipped `Inertial` path never performs). Every ratio below would then have
    // been smoothed-against-smoothed, under a closing assert saying otherwise.
    let base = trajectory_with(Start::AtP, magnitude);
    report(&Start::AtP.label(), &base);
    assert!(
        base.completed(),
        "the AtP baseline did not complete ({:?}). There is nothing to measure a \
         speedup against, and no other strategy may silently become the reference",
        base.ended_early
    );
    let base99 = percentile(&base.ms, 0.99);

    for start in [
        Start::Smoothed(Shape::TipLoadCurve),
        Start::Smoothed(Shape::FirstMode),
        Start::Smoothed(Shape::Rotation),
    ] {
        let run = trajectory_with(start, magnitude);
        report(&start.label(), &run);
        assert!(
            run.completed(),
            "{} did not complete ({:?}) — it steps the same `p` every frame as the \
             baseline, so a failure here is a defect in the start, not a result",
            start.label(),
            run.ended_early
        );
        // ★★★ The control, run in LOCKSTEP rather than on end states. Both
        // strategies step the same `p` every frame, so at frame 1 — before
        // anything can accumulate — they must agree to solver tolerance. Later
        // frames are allowed to drift: this is a nonlinear oscillator struck ten
        // times, and tolerance-level differences amplify in one.
        //
        // ⚠⚠ The end-state comparison this replaced could not tell those apart,
        // and neither could the `peak_ratio` scalar before it — which reported
        // near-perfect agreement on runs this trace ends `5.6e-1` apart.
        let trace = divergence_trace(Start::AtP, start, magnitude);
        assert!(
            trace.len() == RUN_FRAMES,
            "{} diverged into a failed frame at {} of {RUN_FRAMES}",
            start.label(),
            trace.len()
        );
        let (first, last) = (trace[0], trace[trace.len() - 1]);
        println!(
            "      lockstep vs baseline: frame 1 {first:.2e}, frame 10 {:.2e}, \
             frame {RUN_FRAMES} {last:.2e}",
            trace[9],
        );
        assert!(
            first < 1e-6,
            "{} differs from the baseline by {first:.2e} at FRAME 1, before anything \
             can accumulate. It is solving a different equation, so its speedup is \
             bought on a different simulation",
            start.label()
        );
        assert!(
            last > first,
            "{} shows divergence {last:.2e} at frame {RUN_FRAMES} against {first:.2e} \
             at frame 1 — it did not grow. Then the frame-1 agreement is not \
             evidence of anything, because this comparison is not sensitive enough \
             to see a difference at all",
            start.label()
        );
        let p99 = percentile(&run.ms, 0.99);
        println!(
            "      p99 {base99:.2} -> {p99:.2} ms = {:.2}x",
            base99 / p99
        );
    }
}

/// Run the full trajectory with Newton started per `start` each frame.
fn trajectory_with(start: Start, magnitude: f64) -> Run {
    let (mu, lambda) = lame_for(e_eff_for(EI_TARGET));
    let field = MaterialField::uniform(mu, lambda);
    let (nx, ny, nz) = GRID;
    let tet4 = HandBuiltTetMesh::cantilever_bilayer_beam(nx, ny, nz, SPAN, WIDTH, DEPTH, &field);
    let mesh = Tet10Mesh::from_tet4(&tet4);
    let n_dof = 3 * mesh.n_vertices();
    let axial: Vec<f64> = mesh.positions().iter().map(|q| q.x / SPAN).collect();

    let (x_rest, rest_z, bc, mut cfg) =
        rig(&mesh, slapshot_load(EI_TARGET), rho_eff(), false, TOL_REL);
    cfg.dt = VR_DT;
    cfg.initial_guess = InitialGuess::PreviousState;
    let free_dof = n_dof - 3 * bc.pinned_vertices.len();
    let loaded = bc.loaded_vertices.clone();
    let solver: CpuTet10NHSolver<Tet10Mesh> =
        CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);

    let theta = Tensor::from_slice(&[0.0], &[1]);
    let strike_velocity = slapshot_equivalent_tip_velocity() * magnitude;
    let (mut pos, mut vel) = (x_rest, vec![0.0; n_dof]);
    let (mut iters, mut ms) = (Vec::new(), Vec::new());
    let mut peak_ratio = 0.0f64;
    let mut ended_early = None;

    for k in 0..RUN_FRAMES {
        if k % STRIKE_PERIOD == 0 {
            for &(vertex, _) in &loaded {
                vel[3 * vertex as usize + 2] += strike_velocity;
            }
        }
        let p: Vec<f64> = pos
            .iter()
            .zip(&vel)
            .map(|(xi, vi)| vi.mul_add(VR_DT, *xi))
            .collect();
        let s = smoothed_start(start, &pos, &p, &axial, &loaded, None);
        match one_frame_from(&solver, &mut pos, &mut vel, &theta, &s) {
            Ok((it, t)) => {
                iters.push(it);
                ms.push(t);
                peak_ratio = peak_ratio.max(tip_of(&pos, &loaded, &rest_z) / SPAN);
            }
            Err(why) => {
                ended_early = Some(format!("frame {k}: {why}"));
                break;
            }
        }
    }
    Run {
        free_dof,
        iters,
        ms,
        peak_ratio,
        ended_early,
    }
}

/// The point Newton is started from, for a given [`Start`].
///
/// ★ One definition, used by the per-frame probe, the end-to-end producer and
/// the lockstep trace. It was duplicated across two of them, which would have
/// let the per-frame and end-to-end headlines silently measure *different*
/// interventions while both kept passing.
fn smoothed_start(
    start: Start,
    x0: &[f64],
    p: &[f64],
    axial: &[f64],
    loaded: &[(VertexId, LoadAxis)],
    modal: Option<&ModalRig>,
) -> Vec<f64> {
    match start {
        Start::AtP => p.to_vec(),
        Start::AtPrevious => x0.to_vec(),
        Start::Modal(arm) => {
            modal_start(
                modal.expect("a Start::Modal arm needs the basis it projects onto"),
                arm,
                x0,
                p,
                axial,
                loaded,
            )
            .0
        }
        Start::Smoothed(shape) => {
            // Keep the tip where the impulse puts it; spread the rest of the
            // displacement along a smooth profile instead of the blade band.
            let predicted: Vec<f64> = p.iter().zip(x0).map(|(a, b)| a - b).collect();
            let tip = tip_displacement(&predicted, loaded);
            let unit = shape.at(1.0);
            let mut out = x0.to_vec();
            for (v, &s) in axial.iter().enumerate() {
                out[3 * v + 2] += tip * shape.at(s) / unit;
            }
            out
        }
    }
}

/// Step two starting strategies in **lockstep** from the same initial state,
/// returning their per-frame relative state divergence.
///
/// ★★★ This is what separates the two readings of an end-to-end difference.
/// Both strategies step the same `p` every frame, so if they are solving the
/// same equation they must agree at frame 1 to solver tolerance, and any larger
/// end-state gap is **accumulation** — tolerance-level differences amplified by
/// a nonlinear oscillator over hundreds of frames. If instead they disagree at
/// frame 1, the start is changing the problem and every speedup is bought on a
/// different simulation.
///
/// Comparing only the final states cannot tell those apart, which is how the
/// `peak_ratio` control this replaced reported near-perfect agreement on runs
/// this trace ends `5.6e-1` apart.
fn divergence_trace(a: Start, b: Start, magnitude: f64) -> Vec<f64> {
    let (mu, lambda) = lame_for(e_eff_for(EI_TARGET));
    let field = MaterialField::uniform(mu, lambda);
    let (nx, ny, nz) = GRID;
    let tet4 = HandBuiltTetMesh::cantilever_bilayer_beam(nx, ny, nz, SPAN, WIDTH, DEPTH, &field);
    let mesh = Tet10Mesh::from_tet4(&tet4);
    let n_dof = 3 * mesh.n_vertices();
    let axial: Vec<f64> = mesh.positions().iter().map(|q| q.x / SPAN).collect();
    let (x_rest, _rest_z, bc, mut cfg) =
        rig(&mesh, slapshot_load(EI_TARGET), rho_eff(), false, TOL_REL);
    cfg.dt = VR_DT;
    cfg.initial_guess = InitialGuess::PreviousState;
    let loaded = bc.loaded_vertices.clone();
    let solver: CpuTet10NHSolver<Tet10Mesh> =
        CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);

    let theta = Tensor::from_slice(&[0.0], &[1]);
    let strike_velocity = slapshot_equivalent_tip_velocity() * magnitude;
    let mut state = [
        (x_rest.clone(), vec![0.0; n_dof]),
        (x_rest.clone(), vec![0.0; n_dof]),
    ];
    let mut trace = Vec::with_capacity(RUN_FRAMES);

    for k in 0..RUN_FRAMES {
        for (which, start) in [a, b].into_iter().enumerate() {
            let (pos, vel) = &mut state[which];
            if k % STRIKE_PERIOD == 0 {
                for &(vertex, _) in &loaded {
                    vel[3 * vertex as usize + 2] += strike_velocity;
                }
            }
            let p: Vec<f64> = pos
                .iter()
                .zip(&*vel)
                .map(|(xi, vi)| vi.mul_add(VR_DT, *xi))
                .collect();
            let s = smoothed_start(start, pos, &p, &axial, &loaded, None);
            if one_frame_from(&solver, pos, vel, &theta, &s).is_err() {
                return trace;
            }
        }
        let scale = dof_distance(&state[0].0, &x_rest).max(f64::MIN_POSITIVE);
        trace.push(dof_distance(&state[0].0, &state[1].0) / scale);
    }
    trace
}

// ---------------------------------------------------------------------------
// Probe 4 — the modal predictor: the mesh-general form of "a low-strain shape".
// The basis is fitted on a ring-down, never on a strike.
// ---------------------------------------------------------------------------

// ★ `Shape::TipLoadCurve` and `Shape::FirstMode` win Probe 3 by imposing a
// low-strain GLOBAL shape, and both of them know they are describing a
// cantilever — the profile is written down analytically. Modal projection is
// the same idea without the analytic beam: keep the part of `dt·v₀` that lies
// in the span of the structure's own low modes, and discard the part that does
// not.
//
//     x⁰ = x₀ + Φ_S Φ_Sᵀ M (p − x₀)          on the free DOFs
//
// ⚠ **Rank 0 IS `InitialGuess::PreviousState`, exactly** — the projector is the
// zero map, so the sweep starts at a variant that already ships. The other
// endpoint is NOT reachable here and this file does not claim it: `ΦΦᵀM = I`
// needs `r = n_free = 360`, and this ring-down supplies a single-digit number
// of sound modes. `AtP` therefore stays a separate row rather than the top of
// the sweep.
//
// ★★ Why this is worth measuring even though R1 does not generalise across
// contact positions: **a predictor cannot be wrong, only expensive.** The
// full-order residual still governs the answer, so a basis that fails to span
// the deformation costs iterations and nothing else. The generalisation
// failure that disqualifies the reduced *solve* is not disqualifying here, and
// that asymmetry is the whole reason this candidate outlived `SmoothedInertial`.

/// Modes the first, diagnostic fit is allowed to return.
///
/// A cap on what gets *looked at*, not on what gets used: the arms run against
/// the leading [`ModalRig::sound_rank`] modes, which is a much smaller number
/// and is derived from the spectrum rather than chosen.
const MODAL_MAX_MODES: usize = 32;

/// Periods of the first bending mode the ring-down is sampled over.
///
/// Backward Euler is numerically dissipative at `ω·Δt ≈ 0.96 rad/frame`, so the
/// tail of a long ring-down carries almost no amplitude and contributes almost
/// no content. Eight periods is long enough that the low modes are resolved
/// many times over and short enough that the samples are not all noise.
const RINGDOWN_PERIODS: f64 = 8.0;

/// How far from `ΦᵀMΦ = I` the retained modes are allowed to sit.
///
/// ★★★ **This number chooses the rank — the rank does not choose it.** A POD
/// fitted through the Gram matrix `G = UᵀU` squares the spectrum, so mode `k`
/// carries a relative eigenvalue `(σ_k/σ_max)²` and an f64 eigensolve resolves
/// it to about
///
/// ```text
///     mode error  ≈  ε · (σ_max/σ_k)²
/// ```
///
/// which inverts to a floor on the spectrum: a mode is worth keeping only while
/// `σ_k/σ_max ≥ √(ε/target)`. At `1e-8` that is `σ_k/σ_max ≥ 1.5e-4`.
///
/// ⚠ **The crate's own `SIGMA_FLOOR_REL = 1e-8` is a different threshold and
/// does not serve this purpose.** It marks where a mode becomes
/// *distinguishable from* round-off, not where it becomes *accurate*, and the
/// gap between the two is a square root. Fitted with `max_modes = 32` this
/// ring-down clears that floor with 9 modes, of which the last few are noise:
/// the pilot read `ΦᵀMΦ − I` at `1.8e-3` on the diagonal and `6.3e-4` off it,
/// four orders worse than an orthogonal projector, which is what
/// [`whether_the_band_projector_is_a_projector`] failed on before this constant
/// existed. That is not a defect in `PodBasis` — it is `SIGMA_FLOOR_REL` being
/// asked a question it does not answer.
const MODAL_ORTHONORMALITY_TARGET: f64 = 1.0e-8;

/// Relative singular-value floor implied by [`MODAL_ORTHONORMALITY_TARGET`].
fn modal_sigma_floor() -> f64 {
    (f64::EPSILON / MODAL_ORTHONORMALITY_TARGET).sqrt()
}

/// Frames of free vibration to snapshot, DERIVED from the stick's own `f₁`
/// rather than hard-coded, so it cannot rot if the section or `EI` moves.
fn ringdown_frames() -> usize {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let n = (RINGDOWN_PERIODS / (f1_analytic(EI_TARGET) * VR_DT)).ceil() as usize;
    n
}

/// `Φ_S Φ_Sᵀ M u` for an M-orthonormal column set `modes`.
///
/// Free rather than a method because the two subspaces under test come from
/// different places — the POD fit and a deterministic random draw — and
/// projecting them with the same code is what makes the control comparable.
fn project_onto(modes: &[Vec<f64>], mass: &[f64], u: &[f64]) -> Vec<f64> {
    let mut out = vec![0.0; u.len()];
    for phi in modes {
        let q: f64 = phi
            .iter()
            .zip(u)
            .zip(mass)
            .map(|((a, b), m)| a * b * m)
            .sum();
        for (o, p) in out.iter_mut().zip(phi) {
            *o += q * p;
        }
    }
    out
}

/// Worst `|ΦᵀMΦ − I|` over a column set, as `(diagonal, off-diagonal)`.
///
/// The instrument [`MODAL_ORTHONORMALITY_TARGET`] is read against, and the one
/// the random control is held to as well — a control compared against an
/// orthogonal projector has to be one.
fn gram_error(modes: &[Vec<f64>], mass: &[f64]) -> (f64, f64) {
    let (mut diag, mut off) = (0.0f64, 0.0f64);
    for (j, a) in modes.iter().enumerate() {
        for (k, b) in modes.iter().enumerate() {
            let g: f64 = a.iter().zip(b).zip(mass).map(|((x, y), m)| x * y * m).sum();
            if j == k {
                diag = diag.max((g - 1.0).abs());
            } else {
                off = off.max(g.abs());
            }
        }
    }
    (diag, off)
}

/// Largest signed `z` displacement over the driven band — what both the ramp
/// and every tip-matched modal arm mean by "the tip".
///
/// ★ ONE definition, shared by [`smoothed_start`] and [`modal_start`]. The 2×2
/// in this section compares their rows directly; if the two folded the band
/// differently they would be reading different tips and every row would still
/// print.
fn tip_displacement(disp: &[f64], loaded: &[(VertexId, LoadAxis)]) -> f64 {
    loaded
        .iter()
        .map(|&(vtx, _)| disp[3 * vtx as usize + 2])
        .fold(0.0f64, |a, b| if b.abs() > a.abs() { b } else { a })
}

/// `‖u‖_M = √(uᵀMu)` — the norm the basis is orthonormal in, and the one
/// [`Band::ScaledLikeLow`] matches against.
fn m_norm(mass: &[f64], u: &[f64]) -> f64 {
    u.iter()
        .zip(mass)
        .map(|(a, m)| a * a * m)
        .sum::<f64>()
        .sqrt()
}

/// `count` M-orthonormal directions drawn from a fixed seed.
///
/// ★ Deterministic on purpose. A control that redraws every run cannot be
/// re-read, and a row that moves between runs is indistinguishable from a row
/// that responds to the change under test.
fn random_orthonormal(n: usize, count: usize, mass: &[f64]) -> Vec<Vec<f64>> {
    let mut seed: u64 = 0x2545_f491_4f6c_dd1d;
    let mut next = || {
        seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((seed >> 11) as f64 / (1u64 << 53) as f64).mul_add(2.0, -1.0)
    };
    let mut out: Vec<Vec<f64>> = Vec::with_capacity(count);
    for _ in 0..count {
        let mut v: Vec<f64> = (0..n).map(|_| next()).collect();
        // Modified Gram–Schmidt in the mass product, twice: one pass loses
        // orthogonality to the earlier columns at the level of the condition
        // number, and this control is compared against a basis held to 1e-8.
        for _ in 0..2 {
            for q in &out {
                let d: f64 = q
                    .iter()
                    .zip(&v)
                    .zip(mass)
                    .map(|((a, b), m)| a * b * m)
                    .sum();
                for (vi, qi) in v.iter_mut().zip(q) {
                    *vi -= d * qi;
                }
            }
        }
        let nrm = m_norm(mass, &v);
        assert!(
            nrm > 0.0,
            "a random draw collapsed onto the span of the earlier controls"
        );
        for vi in &mut v {
            *vi /= nrm;
        }
        out.push(v);
    }
    out
}

/// A basis of the stick's own low-strain shapes, plus the metric it is
/// orthonormal in, the DOF map it is written in, and the rank-matched random
/// subspace it is judged against.
///
/// ⚠⚠ **Fitted on a ring-down, never on a strike.** The stick is bent by the
/// static tip load, the load is released, and the free vibration is
/// snapshotted. Nothing in the training data is impulsive or band-shaped, so
/// the basis has not been shown the field it is being asked to repair.
///
/// ⚠ It HAS been shown the tip band, because `stickrig::rig` loads the stick
/// there and that is also where the puck lands — this fixture has no second
/// place to push. So the basis is independent of the *time signature* of the
/// strike but not of its *location*, and a claim that modal projection
/// transfers across contact positions cannot be made from this rig. That is the
/// same axis R1 failed on, and it is the first thing to attack if this wins.
struct ModalRig {
    basis: PodBasis,
    /// The solver's free-DOF map. Full-DOF index per free unknown.
    free: Vec<usize>,
    /// Lumped mass per free DOF, in `free` order — the metric `Inner::Mass`
    /// makes the basis orthonormal in.
    mass: Vec<f64>,
    /// Leading modes that clear [`modal_sigma_floor`]. The arms use these; the
    /// rest are kept only so the spectrum can be printed.
    sound: usize,
    /// `sound` M-orthonormal random directions — the rank-matched control.
    random: Vec<Vec<f64>>,
    /// Rest `x/L` per VERTEX — the coordinate a [`Shape`] profile is a function
    /// of, kept so a mode can be asked whether it is one too.
    axial: Vec<f64>,
}

impl ModalRig {
    /// Bend the stick with the static tip load, release it, and fit a POD basis
    /// to the free vibration that follows.
    fn from_ringdown() -> Self {
        let (mu, lambda) = lame_for(e_eff_for(EI_TARGET));
        let field = MaterialField::uniform(mu, lambda);
        let (nx, ny, nz) = GRID;
        let tet4 =
            HandBuiltTetMesh::cantilever_bilayer_beam(nx, ny, nz, SPAN, WIDTH, DEPTH, &field);
        let mesh = Tet10Mesh::from_tet4(&tet4);
        let n_dof = 3 * mesh.n_vertices();
        let axial: Vec<f64> = mesh.positions().iter().map(|q| q.x / SPAN).collect();
        let (x_rest, _rest_z, bc, mut cfg) =
            rig(&mesh, slapshot_load(EI_TARGET), rho_eff(), false, TOL_REL);
        cfg.dt = VR_DT;
        cfg.initial_guess = InitialGuess::Inertial;
        let n_loaded = bc.loaded_vertices.len();
        let solver: CpuTet10NHSolver<Tet10Mesh> =
            CpuNewtonSolver::new(Tet10, mesh, NullContact, cfg, bc);

        let free: Vec<usize> = solver.free_dof_indices().to_vec();
        let mass = solver.mass_per_free_dof();

        let (mut x, mut v) = (x_rest.clone(), vec![0.0; n_dof]);
        // Bend. The same continuous ramp `stick_flex.rs` loads this stick with;
        // `theta` is the PER-VERTEX force, not a 0..1 scale.
        let load = slapshot_load(EI_TARGET);
        for k in 0..RAMP_FRAMES {
            let frac = (k + 1) as f64 / RAMP_FRAMES as f64;
            let theta = Tensor::from_slice(&[load * frac / n_loaded as f64], &[1]);
            one_frame(&solver, &mut x, &mut v, &theta, VR_DT)
                .expect("the bend must complete or there is no ring-down to fit");
        }
        // Release, and snapshot the free vibration.
        let theta = Tensor::from_slice(&[0.0], &[1]);
        let mut snaps = SnapshotSet::new(free.len());
        for _ in 0..ringdown_frames() {
            one_frame(&solver, &mut x, &mut v, &theta, VR_DT)
                .expect("the ring-down must complete or the basis is fitted on a stub");
            snaps.push(&SnapshotSet::free_displacement(&x, &x_rest, &free));
        }
        // `energy_fraction = 1.0`: the size is controlled by `max_modes` alone,
        // as `PodBasis::fit`'s docs require — retained energy is measured to
        // mislead about held-out error.
        let basis = PodBasis::fit(&snaps, Inner::Mass, &mass, 1.0, MODAL_MAX_MODES)
            .expect("the ring-down must carry resolvable content");

        let sv = basis.singular_values();
        let floor = modal_sigma_floor() * sv.first().copied().unwrap_or(0.0);
        let sound = sv
            .iter()
            .take(basis.modes().len())
            .take_while(|s| **s >= floor)
            .count();
        assert!(
            sound > 0,
            "no mode cleared the {:.1e} relative spectrum floor, so there is no \
             sound subspace to project onto",
            modal_sigma_floor()
        );
        let random = random_orthonormal(free.len(), sound, &mass);
        Self {
            basis,
            free,
            mass,
            sound,
            random,
            axial,
        }
    }

    /// Modes the fit returned, sound or not.
    fn retained(&self) -> usize {
        self.basis.modes().len()
    }

    /// Modes the arms are allowed to use — see [`MODAL_ORTHONORMALITY_TARGET`].
    const fn sound_rank(&self) -> usize {
        self.sound
    }
}

/// Which subspace a [`Start::Modal`] arm keeps — and the two controls that
/// decide whether keeping the stick's own LOW modes is what does the work.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Band {
    /// The leading `rank` POD modes of the ring-down. The low-strain subspace
    /// under test.
    Low,
    /// `rank` deterministic M-orthonormal RANDOM directions.
    ///
    /// ★★ Rank-matched and structure-blind: the same number of directions, drawn
    /// with no knowledge of the stick. Separates "the structure's own low modes"
    /// from "any `rank`-dimensional subspace". In 360 dimensions a random
    /// direction is essentially all high-strain, so this also subsumes the
    /// high-modes control it replaced — which could not be formed at every rank,
    /// because the ring-down supplies only a handful of sound modes.
    Random,
    /// The leading `rank` modes, then RESCALED so the driven nodes land where
    /// `p` puts them.
    ///
    /// ★★★ The one difference between `Low` and the ramp that wins Probe 3.
    /// `Shape::TipLoadCurve` keeps the full predicted tip displacement and only
    /// redistributes it — it starts `7.541` travel from `x₀`, further out than
    /// `p` — whereas `Low` fixes the shape and throws the amplitude away with
    /// it: mode 1 carries about `40 %` of the mass-weighted increment, so
    /// `Low r=1` moves `1.942`. This arm separates the two: same low-strain
    /// direction, full amplitude.
    ///
    /// ⚠ It needs the driven set, which is a real dependency and not a free
    /// one — but it is the SAME information a contact solve already has, not
    /// the analytic beam profile `Shape` needs, so an arm that wins here is
    /// still mesh-general.
    TipMatched,
    /// [`Self::TipMatched`] with its `x` and `y` components zeroed — the
    /// tip-matched POD `z` profile alone.
    ///
    /// ★★★ Cell (POD-z, no in-plane) of the 2×2 that splits the `36×`. If this
    /// reads like `smooth:mode-1`, the in-plane components carry the whole cost
    /// and a mesh-general predictor can be built from transverse fields.
    TipMatchedZOnly,
    /// [`Self::TipMatched`] with the transverse part scaled by `β` and the
    /// in-plane part by `β²`.
    ///
    /// ★★★ The falsifiable prediction of the axial-strain mechanism. A mode's
    /// in-plane field cancels `½(∂w/∂x)²` at the amplitude it was fitted at;
    /// rescale `w` by `β` and that term grows as `β²` while a linearly scaled
    /// `∂u/∂x` grows as `β`. Scaling the in-plane part by `β²` instead should
    /// cancel the mismatch — and `β²` is arithmetic on a number the arm already
    /// has, not knowledge about beams, so a win here is still mesh-general.
    ///
    /// ⚠ It is a prediction about the LEADING term only. The cancellation is
    /// exact for a kinematic that is quadratic in the rotation and nothing
    /// more; a Neo-Hookean Tet10 at `0.6 rad` is not that, so a partial
    /// recovery would still support the mechanism.
    QuadraticInPlane,
    /// The analytic first-mode `z` profile plus [`Self::TipMatched`]'s `x`/`y`.
    ///
    /// ★★★ Cell (analytic-z, POD in-plane). The other half of the crossover,
    /// and the two decompose exactly: `TipMatched` is this arm's in-plane field
    /// plus `TipMatchedZOnly`'s `z` field, because zeroing `x`/`y` cannot change
    /// a fold that reads only `z`, so both inherit the SAME `β`.
    RampPlusModalXy,
    /// **Not a projection.** `x₀ + α(p − x₀)` with `α = ‖P_low u‖_M / ‖u‖_M`.
    ///
    /// ★★★ The load-bearing control. Projection can only SHRINK the increment
    /// in the mass norm, so every `Low` arm sits somewhere between `AtPrevious`
    /// and `AtP` in magnitude. If moving toward `x₀` is worth iterations by
    /// itself, `Low` wins for a reason that has nothing to do with modes. This
    /// arm travels the same mass-norm distance along the RAW direction, so a
    /// `Low` that beats it is winning on SHAPE — the same question
    /// [`whether_the_shape_of_the_guess_error_sets_the_iteration_count`] asks
    /// of a synthetic error, asked of the real intervention.
    ScaledLikeLow,
}

impl Band {
    const fn label(self) -> &'static str {
        match self {
            Self::Low => "low",
            Self::Random => "rand",
            Self::TipMatched => "tipmatch",
            Self::TipMatchedZOnly => "tip-z",
            Self::QuadraticInPlane => "tip-b2",
            Self::RampPlusModalXy => "ramp+xy",
            Self::ScaledLikeLow => "scaled",
        }
    }
}

/// One modal arm: how many directions, and which ones.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct ModalArm {
    rank: usize,
    band: Band,
}

/// The starting point a modal arm selects, and the number of directions it
/// really used.
///
/// ★ The width is returned rather than clipped silently: an arm asking for rank
/// 8 against a basis with 4 sound modes is projecting onto a narrower subspace
/// than its label claims, and a caller that cannot tell reports a rank it never
/// ran.
fn modal_start(
    rig: &ModalRig,
    arm: ModalArm,
    x0: &[f64],
    p: &[f64],
    axial: &[f64],
    loaded: &[(VertexId, LoadAxis)],
) -> (Vec<f64>, usize) {
    let u: Vec<f64> = rig.free.iter().map(|&i| p[i] - x0[i]).collect();
    let low = arm.rank.min(rig.sound_rank());
    let (d, width) = match arm.band {
        Band::Random => {
            let k = arm.rank.min(rig.random.len());
            (project_onto(&rig.random[..k], &rig.mass, &u), k)
        }
        Band::ScaledLikeLow => {
            let proj = project_onto(&rig.basis.modes()[..low], &rig.mass, &u);
            let den = m_norm(&rig.mass, &u);
            let alpha = if den > 0.0 {
                m_norm(&rig.mass, &proj) / den
            } else {
                0.0
            };
            (u.iter().map(|ui| alpha * ui).collect(), low)
        }
        Band::Low
        | Band::TipMatched
        | Band::TipMatchedZOnly
        | Band::QuadraticInPlane
        | Band::RampPlusModalXy => (project_onto(&rig.basis.modes()[..low], &rig.mass, &u), low),
    };
    // ⚠ Only the FREE DOFs enter the field. A constrained DOF keeps `x₀`,
    // matching what `Start::AtPrevious` and every `Start::Smoothed` profile
    // already do (their profiles vanish at the clamp), and matching
    // `InitialGuess`'s own note that a constrained row enters neither the
    // residual norm nor the solve.
    let mut field = vec![0.0; x0.len()];
    for (&i, di) in rig.free.iter().zip(&d) {
        field[i] = *di;
    }
    let predicted: Vec<f64> = p.iter().zip(x0).map(|(a, b)| a - b).collect();
    let want = tip_displacement(&predicted, loaded);
    // `β` rescales a projection so the driven band lands where `p` puts it. It
    // is read off the UNSCALED field, and it is the SAME number for every
    // tip-matched arm below — zeroing, replacing or separately scaling the
    // in-plane components cannot move a fold that reads only `z`. That is what
    // makes the 2×2 a decomposition rather than four unrelated rows.
    //
    // A projection with no amplitude at the driven band cannot be rescaled to
    // have some; `β = 1` leaves it as the plain projection, which shows up as a
    // `Low`-sized row rather than hiding behind a division.
    let have = tip_displacement(&field, loaded);
    let beta = if have == 0.0 { 1.0 } else { want / have };
    match arm.band {
        // A projection, used as one. At `β = 1` there is no amplitude mismatch
        // to create — which is why these are the arms that do well.
        Band::Low | Band::Random | Band::ScaledLikeLow => {}
        Band::TipMatched => {
            for f in &mut field {
                *f *= beta;
            }
        }
        Band::TipMatchedZOnly => {
            for (i, f) in field.iter_mut().enumerate() {
                *f = if i % 3 == 2 { *f * beta } else { 0.0 };
            }
        }
        Band::QuadraticInPlane => {
            for (i, f) in field.iter_mut().enumerate() {
                *f *= if i % 3 == 2 { beta } else { beta * beta };
            }
        }
        Band::RampPlusModalXy => {
            for (i, f) in field.iter_mut().enumerate() {
                if i % 3 != 2 {
                    *f *= beta;
                }
            }
            // Overwrite `z` with the analytic profile, keeping the projection's
            // in-plane field. Written at EVERY vertex, exactly as
            // `smoothed_start` writes it, and it vanishes at the clamp because
            // the profile does.
            let unit = Shape::FirstMode.at(1.0);
            for (v, &s) in axial.iter().enumerate() {
                field[3 * v + 2] = want * Shape::FirstMode.at(s) / unit;
            }
        }
    }
    let out: Vec<f64> = x0.iter().zip(&field).map(|(a, b)| a + b).collect();
    (out, width)
}

/// ★★ **Is the projector actually a projector, and how many modes is that true
/// of?**
///
/// Four properties, and every modal row is meaningless without them:
///
/// 1. `ΦᵀMΦ = I` on the SOUND modes, to [`MODAL_ORTHONORMALITY_TARGET`].
///    Without it `Φ_SΦ_SᵀM` is not an orthogonal projection, `‖Pu‖_M ≤ ‖u‖_M`
///    need not hold, and [`Band::ScaledLikeLow`]'s `α` matches against nothing.
/// 2. The same, for the random control — which is compared against the basis
///    and so has to be held to the same standard.
/// 3. The full retained band agrees with the crate's own
///    `reconstruct(project(u))`. ★ The oracle DISAGREES with the subject:
///    `project_onto` is hand-rolled here over a subspace, `PodBasis::project`
///    is the shipped path, and they were written by different code. Agreement
///    is evidence; a re-implementation checked against itself would be none.
/// 4. Rank 0 reproduces `Start::AtPrevious` **exactly**, bit for bit — the
///    endpoint claim the section header makes.
///
/// ⚠ It also PRINTS the orthonormality error against retained rank, because
/// that curve is where [`MODAL_ORTHONORMALITY_TARGET`] gets its floor. Read it
/// before changing the constant.
#[test]
#[ignore = "diagnostic — run explicitly"]
fn whether_the_band_projector_is_a_projector() {
    let rig = ModalRig::from_ringdown();
    println!(
        "\n=== ring-down basis: {} snapshots over {} periods of f1={:.2} Hz ===\n  \
         {} modes cleared PodBasis's own SIGMA_FLOOR_REL (cap {MODAL_MAX_MODES}); \
         {} clear this file's {:.1e} spectrum floor and are used",
        ringdown_frames(),
        RINGDOWN_PERIODS,
        f1_analytic(EI_TARGET),
        rig.retained(),
        rig.sound_rank(),
        modal_sigma_floor(),
    );
    let sv = rig.basis.singular_values();
    let s0 = sv[0];
    print!("  sigma/sigma_0:");
    for s in sv.iter().take(12) {
        print!(" {:.2e}", s / s0);
    }
    println!("{}", if sv.len() > 12 { " ..." } else { "" });

    // The curve the floor is read off.
    println!(
        "  {:>5} {:>11} {:>11} {:>7}",
        "rank", "|diag−1|", "|off|", "sound"
    );
    for r in 1..=rig.retained() {
        let (diag, off) = gram_error(&rig.basis.modes()[..r], &rig.mass);
        println!(
            "  {r:>5} {diag:>11.2e} {off:>11.2e} {:>7}",
            if r <= rig.sound_rank() { "yes" } else { "no" }
        );
    }

    // 1 — mass-orthonormality of the sound modes.
    let (diag, off) = gram_error(&rig.basis.modes()[..rig.sound_rank()], &rig.mass);
    assert!(
        diag < MODAL_ORTHONORMALITY_TARGET && off < MODAL_ORTHONORMALITY_TARGET,
        "the {} sound modes are not M-orthonormal (diag {diag:.2e}, off {off:.2e}) \
         against a target of {MODAL_ORTHONORMALITY_TARGET:.1e}, so Φ_SΦ_SᵀM is not an \
         orthogonal projection and the ScaledLikeLow control is matching against a \
         norm that does not mean what it says",
        rig.sound_rank(),
    );

    // 2 — and of the random control it is judged against.
    let (rdiag, roff) = gram_error(&rig.random, &rig.mass);
    println!("  random control: |diag−1| {rdiag:.2e}, |off| {roff:.2e}");
    assert!(
        rdiag < MODAL_ORTHONORMALITY_TARGET && roff < MODAL_ORTHONORMALITY_TARGET,
        "the random control is not M-orthonormal (diag {rdiag:.2e}, off {roff:.2e}), so \
         it is not a rank-matched projection and cannot be compared to one"
    );

    // 3 — the full band must agree with the shipped projection path.
    let u: Vec<f64> = (0..rig.free.len())
        .map(|i| ((i % 17) as f64 - 8.0) * 1e-3)
        .collect();
    let mine = project_onto(rig.basis.modes(), &rig.mass, &u);
    let theirs = rig.basis.reconstruct(&rig.basis.project(&u));
    let gap = mine
        .iter()
        .zip(&theirs)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0f64, f64::max);
    let scale = mine.iter().fold(0.0f64, |m, a| m.max(a.abs()));
    println!("  full band vs PodBasis::reconstruct(project(u)): {gap:.2e} on a {scale:.2e} field");
    assert!(
        gap <= 1e-12 * scale.max(1e-12),
        "the hand-rolled projector disagrees with the crate's own projection by \
         {gap:.2e} on a {scale:.2e} field — every modal row is computed by code that \
         does not do what PodBasis does"
    );

    // 4 — rank 0 is `AtPrevious`, exactly.
    let n = 3 * (rig.free.iter().copied().max().unwrap_or(0) / 3 + 1);
    let x0: Vec<f64> = (0..n).map(|i| (i as f64) * 1e-4).collect();
    let p: Vec<f64> = x0.iter().map(|a| a + 1e-3).collect();
    let (zero, width) = modal_start(
        &rig,
        ModalArm {
            rank: 0,
            band: Band::Low,
        },
        &x0,
        &p,
        &[],
        &[],
    );
    assert!(width == 0, "a rank-0 subspace cannot have width {width}");
    assert!(
        zero == x0,
        "rank 0 must reproduce Start::AtPrevious bit for bit; it did not"
    );
    println!("  rank 0 == AtPrevious: exact");
}

/// ★★★ **Does starting Newton in the span of the stick's own low modes cut the
/// iteration count on a real impact frame?**
///
/// The same frozen-frame rig Probe 3 uses: the stick is advanced through a
/// strike and its ring-down, the next strike lands, and then `p = x₀ + dt·v₀`
/// is held **exactly** while only the starting point varies. Same load, same
/// momentum, same answer — only the guess moves.
///
/// ★ This run is a **ceiling test, and deliberately generous**: the basis is
/// fitted on this very stick. If a fixture-trained basis cannot beat plain
/// `Inertial` here, no cheaper or more general basis will, and the candidate
/// dies for the price of one run. Only if it wins is it worth asking where the
/// basis could honestly come from.
///
/// ★★ Read the `low` row against the `scaled` row of the SAME rank, not against
/// `p (Inertial)`. Projection shrinks the increment, so a `low` arm that beats
/// `Inertial` may only be discovering that `x₀` is a better start than `p` —
/// see [`Band::ScaledLikeLow`].
///
/// # What it read
///
/// Newton iterations, `4` sound modes, one frozen impact frame per column:
///
/// ```text
///   impulse  Inertial     x₀   ramp tip/mode1   low r=1  r=2  r=3  r=4   scaled  rand
///    0.25x          7    449            5 / 5         8    6    6    8        5    41
///    1.00x         25   FAIL            4 / 4         6    8    8   14       41    45
///    2.00x         12   FAIL            6 / 5        18    9    9   26      107    70
///    8.00x     32 [*]   FAIL          37 / 25        37   48   54  219       14  FAIL
/// ```
///
/// `[*]` at `8.00x` the `AtP` row itself fails a validity check — `p` is an
/// inverted configuration there, which the real `Inertial` path never has to
/// hold as `x_prev`. `32` is the real path's count. `scaled`/`rand` columns are
/// rank 1.
///
/// ★★★ **The mechanism is real, and it is the modes.** At the game strike a
/// SINGLE mode takes `25 → 6` iterations, `4.17×`, and it beats both controls:
/// `6.83×` over the norm-matched scalar shrink and `7.50×` over a rank-matched
/// random subspace. The two distance columns are what make that a statement
/// about shape — `low r=1` and `scaled r=1` start `1.942` and `2.445` travel
/// from `x₀`, and `0.944` and `1.939` from `x*`, so they are comparable in
/// distance and `6.8×` apart in cost. The modes beat the random subspace at
/// **every** magnitude and **every** rank (`1.07×` to `15.83×`), and at
/// `8.00x` every random arm fails outright while every modal arm converges.
///
/// ⚠ **Distance is not the variable at all, and the ramp rows prove it.**
/// `smooth:tip-load` starts `7.541` travel from `x₀` and `6.542` from `x*` —
/// FURTHER from the answer than `p` is — and converges in `4` iterations, while
/// `x₀`, which sits at `1.000`, hits the `500` cap and fails.
///
/// ⛔ **But modal projection does NOT beat the beam-specific ramp it was meant
/// to generalise**, and that is the finding that decides the candidate. The
/// analytic profiles take `4` at `1.00x` against the best modal arm's `6`, and
/// `25` at `8.00x` against `37`. A mesh-general basis reproduced most of the
/// win and none of the margin.
///
/// ⚠ Three more things this read that are not yet understood:
///
/// - **Rank is not monotone, and rank 4 is the worst arm at three of four
///   magnitudes** (`14`, `26`, `219`). The 4th mode is where `x0->start` jumps
///   from `1.947` to `2.998`: it adds a large component that overshoots.
/// - **Both ends of the sweep invert.** At `0.25x` `Inertial` is already cheap
///   and the scalar shrink wins; at `8.00x` `scaled r=1` takes `14` against
///   `low r=1`'s `37` and `mode-1`'s `25`.
/// - **A `0.040`-travel random nudge off `x₀` turns a `500`-iteration failure
///   into `45` iterations** at `1.00x`. Whatever traps Newton exactly at `x₀`
///   is not distance and not shape.
///
/// ★ One thing the random control DOES explain, consistently with the previous
/// attempt: at `8.00x` it fails at iteration 0 on a validity check while moving
/// only `0.04` travel. A random field displaces midside nodes off the edges
/// they bisect and folds the quadratic element — the same failure mode
/// `SmoothedInertial`'s graph-Laplacian arm died of, and the reason
/// `InitialGuess`'s docs say a midside node is not an ordinary graph vertex.
///
/// # ★★★ The amplitude hypothesis was wrong, and the way it failed is the
/// sharpest thing in this file
///
/// [`Band::TipMatched`] was added to test the one difference between `Low` and
/// the ramp: the ramp keeps the full predicted tip displacement, `Low` keeps
/// only the `~40 %` that projects onto mode 1. Rescale the mode to the same
/// tip and the two should meet.
///
/// They do meet — in position. At `1.00x`:
///
/// ```text
///   start                    iters   x0->start   start->x*
///   smooth:mode-1                4       7.713       6.715
///   modal:tipmatch r=1         146       7.643       6.644
///   modal:low r=1                6       1.942       0.944
/// ```
///
/// **`1 %` apart in both distances, `36×` apart in cost.** Rescaling made the
/// modal arm `24×` WORSE than not rescaling it, and left it `36×` behind an
/// analytic profile sitting essentially on top of it.
///
/// ⇒ So the winning property is **narrower than "a low-strain global shape"**,
/// and narrower than "the right shape at the right amplitude". Two fields with
/// the same driven-node displacement and the same first-mode z-profile are not
/// interchangeable. What separates them is what `Shape` does NOT have:
/// `smooth:mode-1` is a pure transverse `z` field, while a POD mode is a full
/// 3-D finite-element deformation carrying the axial and Poisson components
/// that belong to the amplitude it was FITTED at. Scaling those linearly is
/// exactly what large-deflection beam kinematics says you may not do — axial
/// shortening goes as the square of the rotation, not the first power.
///
/// ⚠ That reading is a hypothesis, not a measurement. What is measured is the
/// `36×`. Two observations constrain it, and neither is explained:
///
/// - **More modes rescue the rescale**: `tipmatch` reads `146 → 41 → 39` over
///   ranks 1–3 at `1.00x`, so whatever rank 1 gets wrong, ranks 2 and 3 partly
///   correct.
/// - **It is amplitude-dependent, not `β`-dependent.** `β ≈ 3.9` at every
///   magnitude, because `want` and `have` both scale with the strike — yet
///   `tipmatch r=1` costs only `1.6×` `Low` at `0.25x` and `24×` at `1.00x`.
///
/// # ★★★ The component split: the in-plane field carries it
///
/// The 2×2, iterations at `1.00x`, rank 1:
///
/// ```text
///                     xy: none      xy: POD
///    z: analytic             4           82
///    z: POD                 10          146
/// ```
///
/// **Adding the POD mode's in-plane field to the analytic profile costs `20×`
/// (`4 → 82`). Swapping the analytic `z` profile for the POD one costs `2.5×`
/// (`4 → 10`).** The in-plane components carry the gap, and it holds across the
/// sweep: at `8.00x` rank 3 the POD `z` profile alone reads `29` against the
/// hand-written Euler–Bernoulli mode's `25`, within `1.16×`, while the in-plane
/// field takes the same start to `156`.
///
/// ⇒ **POD finds the right shape. The damage is entirely in what rides along
/// with it.** Zeroing `x`/`y` turns `tipmatch`'s `146` into `10`, and unlike
/// every other modal arm that one is stable in rank (`10, 11, 12, 13` over
/// ranks 1–4, against `tipmatch`'s `146, 41, 39, 36`).
///
/// ## Why, and the two things it retro-explains
///
/// A slender beam's axial strain is `ε = ∂u/∂x + ½(∂w/∂x)²`. A POD mode's
/// in-plane field `u` is whatever cancels that quadratic term **at the
/// amplitude the mode was fitted at**. Rescaling the mode by `β` grows `∂u/∂x`
/// linearly and `½(∂w/∂x)²` quadratically, leaving `(β² − β)·½(∂w/∂x)²` of
/// spurious AXIAL strain — and this section's axial stiffness is about
/// `(L/r)² ≈ 2·10⁴` times its bending stiffness, so a strain error far too
/// small to see in the position columns is an enormous force error.
///
/// That is a hypothesis, but it is the one the numbers already committed to
/// were waiting for, and it closes two of the three open items above:
///
/// - **Why the damage is amplitude-dependent rather than `β`-dependent.** The
///   spurious term scales with `(∂w/∂x)²`, not with `β` alone. `β ≈ 3.9` at
///   every magnitude, and `tipmatch` costs `1.6×` `Low` at `0.25x` and `24×` at
///   `1.00x` — because the rotation, not the ratio, is what grew.
/// - **Why plain `Low` is the best modal arm despite throwing amplitude away.**
///   It never rescales. At `β = 1` there is no mismatch to create.
///
/// ## ⛔ The mechanism made a prediction, and the prediction is dead
///
/// If the in-plane damage were an axial-strain mismatch, scaling the in-plane
/// part by `β²` instead of `β` should cancel it. It does not. Iterations by
/// in-plane treatment, ranks 1–4:
///
/// ```text
///   impulse   xy: β                xy: β²                xy: none        analytic z
///    0.25x    13, 10,  9,  10      12,  13,  16,  17     11, 46, 28, 16           5
///    1.00x   146, 41, 39,  36     194, 174, 174, 299     10, 11, 12, 13           4
///    2.00x    66, 71, 95, 329      38,  39,  39,  57     45, 16, 19, 39           5
///    8.00x   153, 45, 65, FAIL    FAIL at every rank     50, 37, 29, FAIL        25
/// ```
///
/// `β²` helps at `2.00x`, hurts at `0.25x` and `1.00x` — where it is `4×` worse
/// than `β` at rank 1 and `8×` worse at rank 4 — and fails outright at every
/// rank at `8.00x`. **The quadratic correction is not a correction.**
///
/// ⇒ So the axial-strain story is wrong, or at least the leading-order
/// cancellation is not what governs, and it is struck rather than kept beside
/// its own refutation. What survives it is stronger and simpler: **the in-plane
/// field is harmful at every scaling tried, and REMOVING it is what works.**
/// `xy: none` is the only in-plane treatment that is stable in rank and it is
/// `14×` better than either rescaling at `1.00x`.
///
/// ⚠ Two of the three items the earlier section closed have to be re-opened
/// with it: the amplitude dependence and `Low`'s advantage no longer have an
/// explanation, only a description.
///
/// # ⛔ Verdict on the candidate
///
/// Best mesh-general modal arm against the two references, iterations:
///
/// ```text
///   impulse   Inertial   best modal arm        analytic ramp
///    0.25x           7   6   (low r=2)                     5
///    1.00x          25   6   (low r=1)                     4
///    2.00x          12   9   (low r=2)                     5
///    8.00x          32   29  (tip-z r=3)                  25
/// ```
///
/// **The ramp beats the best modal arm at every magnitude**, and the modal
/// arm's margin over `Inertial` is large only at `1.00x`. Modal projection is
/// real, mesh-general, and reproducible — and it does not clear the bar the
/// beam-specific profile already set. It is recorded here rather than deleted
/// so that the next attempt starts from measurements it can re-run, which is
/// exactly what `SmoothedInertial` could not offer.
///
/// ▶ The last factor — whether a POD mode's `z` varies across the section — was
/// then measured directly and is NOT the answer:
/// [`whether_a_pod_mode_is_a_function_of_the_axial_coordinate`] reads a
/// cross-sectional spread of `0.001` on mode 0. What it found instead is where
/// to look next, and it is not about `z` at all.
#[test]
#[ignore = "diagnostic — run explicitly"]
fn whether_a_modal_start_cuts_iterations_on_a_real_impact_frame() {
    let rig = ModalRig::from_ringdown();
    println!(
        "\n=== ring-down basis: {} of {} retained modes are sound and usable ===",
        rig.sound_rank(),
        rig.retained()
    );
    for magnitude in [0.25, 1.00, 2.00, 8.00] {
        modal_sweep_at(&rig, magnitude);
    }
}

/// One frozen impact frame, every arm.
///
/// Split out of the test only because the per-magnitude body IS the
/// instrument and the test around it is a loop over four magnitudes.
fn modal_sweep_at(rig: &ModalRig, magnitude: f64) {
    let probe = ImpactFrame::at_strike(magnitude, STRIKE_PERIOD);
    // ⚠ The basis and the frame are built from separately enriched meshes.
    // They agree only because both go through `rig()` on the same GRID; if
    // the free-DOF map ever diverged, `modal_start` would scatter the
    // projection onto the wrong DOFs and every row would still print.
    assert!(
        rig.free == probe.solver.free_dof_indices(),
        "the basis and the impact frame disagree about which DOFs are free \
         ({} vs {}), so the projection would land on the wrong unknowns",
        rig.free.len(),
        probe.solver.free_dof_indices().len()
    );
    println!(
        "\n=== REAL impact frame: impulse {magnitude:.2}x, struck at frame {}, \
         after one prior strike ===\n  p is {:.1} mm from x0 at the tip; the \
         answer is {:.1} mm from x0. **Inertial baseline: {} iters**",
        STRIKE_PERIOD,
        1e3 * probe.p_tip,
        1e3 * probe.answer_tip,
        probe.star_iters,
    );
    println!(
        "{:>16} {:>8} {:>6} {:>10} {:>11} {:>10} {:>9}",
        "start", "iters", "width", "x0->start", "start->x*", "answer err", "outcome"
    );
    let baseline = probe.star_iters;

    let row = |start: Start, width: usize| -> Option<usize> {
        let (ok, iters, dist, answer_err) = probe.run(start, Some(rig));
        // ★ Both distances, in units of `travel = ‖x* − x₀‖`. Without the
        // first column the table cannot support its own headline: "the same
        // distance from the answer, a different cost" is a claim ABOUT
        // distance, and one distance is not enough to make it.
        let moved = dof_distance(&probe.start_point(start, Some(rig)), &probe.x0) / probe.travel;
        println!(
            "{:>16} {iters:>8} {width:>6} {moved:>10.3} {dist:>11.3} {answer_err:>10.2e} {:>9}",
            start.label(),
            if ok { "ok" } else { "FAILED" }
        );
        if ok {
            assert!(
                answer_err < 1e-3,
                "{} reached a DIFFERENT answer ({answer_err:.2e} away) on the same \
                 equation — this is changing the simulation, not the way it is solved",
                start.label()
            );
            Some(iters)
        } else {
            None
        }
    };

    if row(Start::AtP, 0).is_none() {
        // The same asymmetry Probe 3 documents: `AtP` hands the solver
        // `x_prev = p`, and an inverted `p` trips a validity check the real
        // `Inertial` path never performs, because it keeps `x_prev = x₀`.
        // The baseline stays the real path's count.
        println!(
            "  note: starting AT p failed a validity check — `p` is inverted here, \
             which the real Inertial path never has to hold as `x_prev`. Baseline \
             stays the real path's {baseline} iters."
        );
    }
    row(Start::AtPrevious, 0);
    row(Start::Smoothed(Shape::TipLoadCurve), 0);
    let ramp = row(Start::Smoothed(Shape::FirstMode), 0);

    let mut verdict: Vec<String> = Vec::new();
    for rank in 1..=rig.sound_rank() {
        let mut got: Vec<(Band, Option<usize>)> = Vec::new();
        for band in [
            Band::Low,
            Band::TipMatched,
            Band::TipMatchedZOnly,
            Band::QuadraticInPlane,
            Band::RampPlusModalXy,
            Band::Random,
            Band::ScaledLikeLow,
        ] {
            let arm = ModalArm { rank, band };
            let width = modal_start(rig, arm, &probe.x0, &probe.p, &probe.axial, &probe.loaded).1;
            got.push((band, row(Start::Modal(arm), width)));
        }
        verdict.extend(modal_verdict(rank, baseline, ramp, &got));
    }
    for v in &verdict {
        println!("  {v}");
    }
}

/// The two summary lines one rank produces: the control comparison, and the
/// 2×2 that splits the tip-matched gap.
///
/// `low` is the subject; each control is reported as its own count and the
/// factor it costs OVER `low`, so `>1.00x` always means the modes won.
fn modal_verdict(
    rank: usize,
    baseline: usize,
    ramp: Option<usize>,
    got: &[(Band, Option<usize>)],
) -> Vec<String> {
    let find = |b: Band| got.iter().find(|g| g.0 == b).and_then(|g| g.1);
    let against = |other: Option<usize>| -> String {
        match (other, find(Band::Low)) {
            (Some(o), Some(low)) => format!("{o} ({:.2}x)", o as f64 / low as f64),
            (Some(o), None) => format!("{o} (--)"),
            (None, _) => "FAILED".to_owned(),
        }
    };
    let headline = find(Band::Low).map_or_else(
        || format!("r={rank}: low FAILED to converge"),
        |low| {
            format!(
                "r={rank}: low {low} vs Inertial {baseline} ({:.2}x); \
                 vs scaled {} = SHAPE; vs rand {} = THESE modes; \
                 vs tipmatch {} = AMPLITUDE",
                baseline as f64 / low as f64,
                against(find(Band::ScaledLikeLow)),
                against(find(Band::Random)),
                against(find(Band::TipMatched)),
            )
        },
    );
    // ★ The four cells share one `β` and decompose exactly: (POD-z, POD-xy) is
    // (POD-z, none) plus the in-plane half of (analytic-z, POD-xy).
    let cell = |v: Option<usize>| v.map_or_else(|| "FAIL".to_owned(), |i| i.to_string());
    let split = format!(
        "      2x2 r={rank}  z:analytic|xy:none {}   z:POD|xy:none {}   \
         z:analytic|xy:POD {}   z:POD|xy:POD {}",
        cell(ramp),
        cell(find(Band::TipMatchedZOnly)),
        cell(find(Band::RampPlusModalXy)),
        cell(find(Band::TipMatched)),
    );
    // ★ The prediction the axial-strain mechanism makes, read against the two
    // rescales that bracket it and against the profile that still wins.
    let quad = format!(
        "      b2  r={rank}  xy:beta {}   xy:beta^2 {}   xy:none {}   analytic-z {}",
        cell(find(Band::TipMatched)),
        cell(find(Band::QuadraticInPlane)),
        cell(find(Band::TipMatchedZOnly)),
        cell(ramp),
    );
    vec![headline, split, quad]
}

/// ★★ **Is a POD mode a function of `x/L`, the way a [`Shape`] profile is?**
///
/// The 2×2 left one factor unseparated. With the in-plane field gone, the POD
/// `z` profile still costs `2.5×` the analytic one at `1.00x` (`10` vs `4`)
/// while sitting within `1.16×` at `8.00x`. Both analytic curves — the static
/// tip-load shape and the vibration mode — read `4`, so it is not WHICH curve.
///
/// Exactly two things can differ. A `Shape` profile is a function of the axial
/// coordinate alone: every vertex at a given `x` gets the same `z`. A POD mode
/// is a finite-element field with no such constraint, so it may carry
/// **cross-sectional variation** — different `z` at the same `x`, depending on
/// where in the section the vertex sits.
///
/// ⚠ **This measures the mode field. It runs no solve and proves nothing about
/// iteration counts** — it decides whether an averaging arm would be fixing a
/// real feature or a phantom, and the honest order is to look before building.
///
/// Reported per mode, all relative to the mode's peak `|z|`:
///
/// - `spread` — `(max z − min z)` within one axial station. This is the whole
///   question: `0` means the mode already IS a function of `x/L`.
/// - `in-plane` — RMS `√(x²+y²)`, the field the 2×2 showed carries `20×`.
/// - `dev(mode-1)` / `dev(tip-load)` — how far the station-mean profile is from
///   each analytic curve, both normalised at the driven end.
///
/// # What it read — the question was wrong, and the answer is better
///
/// ```text
///   mode   spread   in-plane   dev(mode-1)   dev(tip-load)   sigma/sigma_0
///      0    0.001      0.017         0.012           0.018          1.00e0
///      1    0.933     22.338         1.491           1.462          1.49e-2
///      2    0.005     18.343         1.015           0.986          5.21e-3
///      3    0.003      0.088         1.074           1.044          7.50e-4
/// ```
///
/// ⛔ **Cross-sectional variation is not the answer.** Mode 0's spread is
/// `0.001` — it already IS a function of `x/L`, to a tenth of a percent. Had
/// the averaging arm been built first, it would have averaged a field that was
/// already flat.
///
/// ★★★ **What the table shows instead: modes 1 and 2 are AXIAL modes.** Their
/// in-plane RMS is `22×` and `18×` their own transverse peak — they are
/// stretching motions with a little bending along for the ride, not bending
/// modes. Mode 3 is a second bending mode (`8.8 %` in-plane). Mode 0 is the
/// analytic first bending mode to `1.2 %`, and `98.3 %` transverse.
///
/// ⇒ **This is the source of the in-plane finding, and of the rank
/// non-monotonicity the sweep recorded as unexplained.** Raising the rank past
/// 1 does not add finer bending detail; it adds STRETCHING. `low r=1` is the
/// best modal arm at the game strike because it is the only arm made of nothing
/// but the bending mode.
///
/// ★★ **And it says why POD is the wrong ordering for a predictor.** POD ranks
/// by energy IN THE SNAPSHOTS. A modest axial oscillation can outrank a second
/// bending mode on that measure while being orders stiffer, which is exactly
/// what `σ/σ₀` shows here: the two axial modes sit at ranks 1–2, above the
/// bending mode at rank 3. **A predictor wants the SOFTEST modes, not the most
/// energetic ones** — those are different orderings, and nothing in the fit
/// knows the difference.
///
/// ★ It also sharpens how violent the in-plane sensitivity is. Mode 0 carries
/// only `1.7 %` in-plane, and scaling THAT by `β ≈ 3.9` is the whole difference
/// between `tipmatch r=1`'s `146` iterations and `tip-z r=1`'s `10` — `14.6×`
/// from a `1.7 %` component. The STIFFNESS half of the struck mechanism (a
/// slender section's axial stiffness is `~(L/r)² ≈ 2·10⁴` times its bending
/// stiffness, so a small axial error is a huge force error) is independently
/// evidenced by that. The KINEMATIC half — the `β²` cancellation — stays dead;
/// it was refuted on its own prediction.
///
/// ⚠ **The residual `2.5×` is still not explained, only bounded.** `tip-z r=1`
/// IS mode 0 with the in-plane zeroed, and its profile agrees with the analytic
/// curve to `1.2 %` — yet the two cost `10` and `4`. Two curves agreeing to one
/// percent are worth `2.5×`, which is the same order of sensitivity this whole
/// file keeps running into (`46.8×` on shape at matched distance, `36×` at `1 %`
/// apart in position). **The start's shape matters far below the level any
/// position metric here can resolve**, and that is the finding to carry, not the
/// individual ratios.
///
/// ▶ The lever this points at: order the basis by **frequency**, not by
/// singular value. Each POD mode has a time coefficient over the ring-down, so
/// its dominant frequency is measurable from the fit that already ran — no
/// stiffness matrix, no beam knowledge. Sorting by it would put the bending
/// modes first and the stretching modes last, mesh-generally.
#[test]
#[ignore = "diagnostic — run explicitly"]
fn whether_a_pod_mode_is_a_function_of_the_axial_coordinate() {
    let rig = ModalRig::from_ringdown();
    // Axial stations, in rest order. Tet10 midside nodes put vertices at every
    // half-element, so this is 2·nx+1 stations for a (4,1,2) grid.
    let mut stations: Vec<f64> = rig.axial.clone();
    stations.sort_by(f64::total_cmp);
    stations.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    println!(
        "\n=== POD mode structure: {} vertices over {} axial stations, \
         {} sound modes ===",
        rig.axial.len(),
        stations.len(),
        rig.sound_rank()
    );

    for k in 0..rig.sound_rank() {
        // Scatter the mode onto full DOFs. A constrained vertex has no free DOF
        // and stays zero, which is what the clamp does anyway.
        let mut full = vec![0.0; 3 * rig.axial.len()];
        for (&i, &v) in rig.free.iter().zip(&rig.basis.modes()[k]) {
            full[i] = v;
        }
        let peak = full
            .iter()
            .skip(2)
            .step_by(3)
            .fold(0.0f64, |m, a| m.max(a.abs()));
        assert!(
            peak > 0.0,
            "mode {k} has no transverse content at all, so every ratio below \
             would be a division by zero dressed as a measurement"
        );

        let mut rows: Vec<(f64, f64, f64, f64)> = Vec::new();
        for &s in &stations {
            let at: Vec<usize> = (0..rig.axial.len())
                .filter(|&v| (rig.axial[v] - s).abs() < 1e-9)
                .collect();
            let zs: Vec<f64> = at.iter().map(|&v| full[3 * v + 2]).collect();
            let mean = zs.iter().sum::<f64>() / zs.len() as f64;
            let lo = zs.iter().fold(f64::INFINITY, |m, a| m.min(*a));
            let hi = zs.iter().fold(f64::NEG_INFINITY, |m, a| m.max(*a));
            let inplane = (at
                .iter()
                .map(|&v| full[3 * v].powi(2) + full[3 * v + 1].powi(2))
                .sum::<f64>()
                / at.len() as f64)
                .sqrt();
            rows.push((s, mean / peak, (hi - lo) / peak, inplane / peak));
        }

        // Normalise the station-mean profile at the driven end, exactly as
        // `smoothed_start` normalises a `Shape` at `s = 1`, so the deviation is
        // between two curves that agree there by construction.
        let tip_mean = rows.last().expect("stations are non-empty").1;
        let dev = |shape: Shape| -> f64 {
            let unit = shape.at(1.0);
            rows.iter()
                .map(|&(s, mean, _, _)| (mean / tip_mean - shape.at(s) / unit).abs())
                .fold(0.0f64, f64::max)
        };
        let worst_spread = rows.iter().fold(0.0f64, |m, r| m.max(r.2));
        let worst_inplane = rows.iter().fold(0.0f64, |m, r| m.max(r.3));
        println!(
            "\n  mode {k}: worst spread {worst_spread:.3}, worst in-plane \
             {worst_inplane:.3}, dev(mode-1) {:.3}, dev(tip-load) {:.3}",
            dev(Shape::FirstMode),
            dev(Shape::TipLoadCurve),
        );
        println!(
            "  {:>7} {:>9} {:>9} {:>9}",
            "s", "mean z", "spread", "in-plane"
        );
        for &(s, mean, spread, inplane) in &rows {
            println!("  {s:>7.3} {mean:>9.3} {spread:>9.3} {inplane:>9.3}");
        }
    }
}
