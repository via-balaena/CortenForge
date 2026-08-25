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
//! percentile falls outside them. Its `max` shows what its `p99` hides — the
//! producer reads `p99 = 21.9 ms` against `max = 366 ms` on the same row. Only
//! `ForceHeld`'s **convergence verdict** and its **`max`** are quotable; nothing
//! in this file asserts on its `p99`.
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
//! elevated frames, so a one-strike `p99` lands **on the ring-down**, at `43 %`
//! of the impact's iteration count — it understates by `2.3×` rather than
//! reporting nothing. (An earlier draft said `2.4×`, which is the *wall-time*
//! dilution; sitting beside the `43 %` iteration ratio it read as that ratio's
//! reciprocal, which is `2.3`.) Ten strikes put the `p99` exactly on the worst frame
//! (`37` of `37` iterations).
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
use sim_soft::{
    CpuTet10NHSolver, InitialGuess, LoadAxis, MaterialField, Mesh, NullContact, Solver, Tet10Mesh,
    VertexId,
};

mod refbox;
mod stickrig;

use stickrig::{
    DEPTH, EI_TARGET, MASS_PER_LENGTH, SLAPSHOT_DEFLECTION, SPAN, TOL_REL, WIDTH, describe_failure,
    e_eff_for, is_convergence_failure, lame_for, one_frame, outcome_of, percentile, rho_eff, rig,
    slapshot_load, tip_of,
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
/// index `297`, three samples deep, and spans `3.33 s` of sim time — about
/// `46` periods of the `13.7 Hz` first mode.
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
    // `> 0.0` accepts a bar of `1e-30`. Piloted at `4.96e-3`, so `1e-3` keeps
    // ~5x margin while refusing a bar that represents no real deflection.
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

/// ★★ **The spike is iteration count — asserted on iterations, so it can run in
/// CI.**
///
/// If frame cost were driven by something other than how many Newton iterations
/// a frame takes, the iteration count would NOT span the range the frame cost
/// does. This gate holds the iteration half, which is deterministic; the
/// wall-time half — that cost per iteration stays flat — needs a clock and lives
/// in [`the_cost_per_iteration_is_flat_across_the_spike`] behind a quiet-box
/// gate.
///
/// ⚠ Splitting them is not cosmetic. `quality-gate.yml` runs eleven binaries in
/// one `cargo test --release` with no `--test-threads=1`, so every release-only
/// gate here executes concurrently, each driving a rayon-parallel solver. A `ms`
/// threshold under that contention fails for a reason its message cannot name,
/// on PRs that changed nothing — and `stick_flex.rs` already keeps all three of
/// its timing instruments behind `refbox::require_quiet_box()` for exactly this.
#[test]
#[cfg_attr(debug_assertions, ignore = "release-only measurement")]
fn the_frame_cost_spike_is_iteration_count_not_cost_per_iteration() {
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
    // statistics. Piloted at 2.1-2.4 ms and 38-43 ms across runs.
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
/// gate green while every other gate in the file failed. On iterations the same
/// mutation is caught, because 2 iterations is not 30.
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
///    1 strike    p99 = 16 iters, worst = 37 iters   ratio 0.43  <- p99 is a RING-DOWN frame
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
/// ⚠ The `p50` also roughly doubles (`1.07 -> 2.16 ms`), a real and separate
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
/// momentum to the *problem*, not just to the starting point, and
/// `whether_the_force_wall_is_the_load_path_or_the_initial_guess` was wrong to
/// describe it as holding the load fixed. The answer moves with the seed, which
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
}

impl Start {
    fn label(self) -> String {
        match self {
            Self::AtP => "p (Inertial)".to_owned(),
            Self::AtPrevious => "x0 (Previous)".to_owned(),
            Self::Smoothed(s) => format!("smooth:{}", s.label()),
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
            let (ok, iters, dist, answer_err) = probe.run(start);
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
    fn start_point(&self, start: Start) -> Vec<f64> {
        smoothed_start(start, &self.x0, &self.p, &self.axial, &self.loaded)
    }

    /// Solve the frozen frame from `start`. Returns
    /// `(converged, iterations, ‖start − x*‖/travel, ‖x_final − x*‖/travel)`.
    fn run(&self, start: Start) -> (bool, usize, f64, f64) {
        let s = self.start_point(start);
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
/// after the FIRST strike, before divergence sets in. It certified nothing.
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
        // `1.8e-9` agreement on runs whose full states end `6.5e-4` apart.
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
        let s = smoothed_start(start, &pos, &p, &axial, &loaded);
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
) -> Vec<f64> {
    match start {
        Start::AtP => p.to_vec(),
        Start::AtPrevious => x0.to_vec(),
        Start::Smoothed(shape) => {
            // Keep the tip where the impulse puts it; spread the rest of the
            // displacement along a smooth profile instead of the blade band.
            let tip = loaded
                .iter()
                .map(|&(vtx, _)| p[3 * vtx as usize + 2] - x0[3 * vtx as usize + 2])
                .fold(0.0f64, |a, b| if b.abs() > a.abs() { b } else { a });
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
/// `peak_ratio` control this replaced reported `1.8e-9` on runs whose full
/// states were `6.5e-4` apart.
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
            let s = smoothed_start(start, pos, &p, &axial, &loaded);
            if one_frame_from(&solver, pos, vel, &theta, &s).is_err() {
                return trace;
            }
        }
        let scale = dof_distance(&state[0].0, &x_rest).max(f64::MIN_POSITIVE);
        trace.push(dof_distance(&state[0].0, &state[1].0) / scale);
        let _ = k;
    }
    trace
}
