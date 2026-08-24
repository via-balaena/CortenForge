//! The reduced-frame share table, the ECSW ceiling bracket, and the `I ≤ B`
//! verdict — shared by every fixture that measures a reduced frame.
//!
//! Shared rather than copied because of what went wrong the last time this
//! arithmetic had one home and one consumer: the verdict compared a freshly
//! measured ceiling against a HARDCODED requirement, and announced "BRACKET
//! STRADDLES the requirement" for a change that had moved the margin by nothing
//! (recon §2j). The requirement is derived from the same run as the ceiling
//! here, and a second fixture must not be able to drift away from that.
//!
//! Consumers: `reduced_phase_shares.rs` (contact-free, R1.1's operating point)
//! and `reduced_contact.rs` (IPC indentation, where R3's requirement lives).

// Each declaring binary compiles this module in full but consumes only what it
// needs — `reduced_phase_shares` reads none of [`Verdict`]'s fields because it
// reports one arm and the verdict block already printed them, while
// `reduced_contact` tabulates three arms and reads them all. The same
// `tests/common` friction `common/mod.rs` and `refbox/mod.rs` carry, for the
// same reason. Scoped to this shared module; production `src/` has no such
// blanket.
#![allow(dead_code)]

use sim_soft::profile::{self, Phase, Reducible};

/// A 60 Hz frame. What the IRREDUCIBLE mass has to fit inside for R3 to clear —
/// recon §2j's restated gate, which replaced a `≥10×` ratio over a baseline the
/// rest of the ladder keeps moving.
pub const FRAME_BUDGET_MS: f64 = 16.7;

/// Below this instrumented-over-wall fraction the slots are not measuring the
/// run at all. Piloted: a correctly built run reads `90 %+`; a run built WITHOUT
/// `--features phase-timing` reads exactly `0`, and every share, the ceiling and
/// the verdict then come out of an empty snapshot — `I` equals the whole frame
/// and the harness reports a confident, wrong "cannot clear".
const MIN_COVERAGE_PCT: f64 = 20.0;

/// What the verdict block printed, so a caller comparing several arms can
/// tabulate them without a second copy of the arithmetic — the failure this
/// module exists to prevent.
#[derive(Clone, Copy, Debug)]
pub struct Verdict {
    /// Wall ms per step over the measured window.
    pub per_step_ms: f64,
    /// `I` — the part of a frame ECSW cannot remove, in ms/step. The only
    /// quantity that decides a rung, since `C/R = B/I`.
    pub irreducible_ms: f64,
    /// `B / I`. `≥ 1` is a pass.
    pub margin: f64,
    /// `T / B` — what this fixture would need overall, for context only.
    pub requirement: f64,
    /// `1/(1 − f)` at the certain and the R3-contingent reducible shares.
    pub ceiling_lo: f64,
    pub ceiling_hi: f64,
}

pub fn main_report(label: &str, wall_ms: f64, steps: usize, iters: usize) -> Verdict {
    let p = profile::snapshot();
    let total_instr: f64 = Phase::ALL
        .iter()
        .filter(|ph| !ph.is_nested())
        .map(|ph| p.millis(*ph))
        .sum();
    assert!(
        100.0 * total_instr / wall_ms >= MIN_COVERAGE_PCT,
        "{label}: instrumented time is {:.1} % of wall — the timers recorded \
         essentially nothing, which is what running WITHOUT `--features \
         phase-timing` looks like. Every number below would be derived from an \
         empty snapshot.",
        100.0 * total_instr / wall_ms,
    );
    println!(
        "\n╔═ {label} — {steps} steps, {:.2} iters/step",
        iters as f64 / steps as f64
    );
    println!("║ {:.2} ms/step wall", wall_ms / steps as f64);
    println!("║");
    println!("║ {:<22} {:>10} {:>9}  ECSW?", "phase", "ms/step", "share");
    let (mut sure, mut planned) = (0.0, 0.0);
    for ph in Phase::ALL {
        let ms = p.millis(ph);
        if ms <= 0.0 {
            continue;
        }
        let share = 100.0 * ms / wall_ms;
        // ⚠ A NESTED slot's time is already in the totals via its parent, so it
        // must not be added again — but it must be SUBTRACTED when its own ECSW
        // class differs from the parent that carried it. `Contact` is the case
        // that bites: irreducible, inside two reducible parents. Missing it
        // overstates the ceiling — when #822 first ran, 90.0 % read as 10.0x
        // where the honest lower bound was 88.0 % / 8.3x. The two
        // `ReducedProjectTangent*` children are reducible inside a reducible
        // parent and correctly need no adjustment.
        if ph.is_nested() {
            if ph.ecsw_reducible() != Reducible::Yes {
                sure -= share;
            }
        } else {
            match ph.ecsw_reducible() {
                Reducible::Yes => sure += share,
                Reducible::PlannedByR3 => planned += share,
                // `Reducible::No` and anything added later: counted as NOT reducible,
                // which lowers the ceiling. `Phase` is `#[non_exhaustive]`, so a new
                // variant reaches here silently — failing toward the pessimistic bound
                // is the safe direction, and the label below makes it visible.
                _ => {}
            }
        }
        println!(
            "║ {:<22} {:10.3} {:8.1} %  {}",
            ph.label(),
            ms / steps as f64,
            share,
            match ph.ecsw_reducible() {
                Reducible::Yes => "yes",
                Reducible::No => "NO",
                Reducible::PlannedByR3 => "if R3's own design works",
                _ => "?? UNCLASSIFIED — counted as NOT reducible",
            }
        );
    }
    println!("║");
    // ★ POSITIVE CONTROL for the knob-0 split. The two children bracket every
    // statement of the parent except `basis.modes()`, so they must very nearly
    // exhaust it. A low figure means one of the two timers is not where its
    // name says, and the A:B ratio below it is then meaningless.
    let parent = p.millis(Phase::ReducedProjectTangent);
    if parent > 0.0 {
        let (g, c) = (
            p.millis(Phase::ReducedProjectTangentGather),
            p.millis(Phase::ReducedProjectTangentContract),
        );
        println!(
            "║ ★ split control: Y=AΦ + ΦᵀY = {:.1} % of red proj K   (A:B = {:.2}:1)",
            100.0 * (g + c) / parent,
            if c > 0.0 { g / c } else { f64::NAN }
        );
    }
    println!(
        "║ instrumented / wall = {:.1} %",
        100.0 * total_instr / wall_ms
    );
    let unaccounted = 100.0 - 100.0 * total_instr / wall_ms;
    println!("║ reducible: {sure:.1} % certain + {planned:.1} % if R3's own design works");
    println!("║ unaccounted: {unaccounted:.1} % — counted as NOT reducible in BOTH bounds");
    let lo = 100.0 / (100.0 - sure);
    let hi = 100.0 / (100.0 - sure - planned);
    println!("║");
    // ⚠ The upper bound is ILL-CONDITIONED: at f ≈ 0.97 a 0.1 pp shift in the
    // reducible share moves it by ~2×, and two identical runs gave 37.1 and 38.9.
    // Print it as an approximate lower limit so it is not read to three digits.
    let hi_txt = if sure + planned > 95.0 {
        format!("≳{hi:.0}×  ⚠ ill-conditioned, do not quote precisely")
    } else {
        format!("{hi:.1}×")
    };
    println!("║ ⇒ R3 Amdahl ceiling = {lo:.1}× … {hi_txt}");

    // ★★ recon §2j. The ceiling ALONE decides nothing, and this harness used to
    // print it against a hardcoded `needs 13.5–15.8×, floor 10×`. Both halves of
    // that were wrong to print here: the requirement is `T/B` and falls whenever
    // the frame does, so a run that speeds up a REDUCIBLE phase lowers the
    // ceiling and the requirement by the same factor — `C/R = B/I` — and the
    // comparison reads as a regression that did not happen. Measured: the
    // `project_tangent` layout change took the ceiling `20.0× → 11.3×` and this
    // line printed "BRACKET STRADDLES the requirement" for a change that left
    // the margin at `5.1× → 5.3×`.
    //
    // What decides is whether the IRREDUCIBLE time fits the frame budget.
    let per_step = wall_ms / steps as f64;
    let irreducible = per_step * (100.0 - sure) / 100.0;
    let margin = FRAME_BUDGET_MS / irreducible;
    println!(
        "║ ⇒ GATE (§2j): irreducible {irreducible:.3} ms/step vs {FRAME_BUDGET_MS} ms ⇒ margin {margin:.2}×"
    );
    println!(
        "║ ⇒ {}",
        if margin >= 1.0 {
            "CLEARS — R3 can reach the budget on this fixture, with that much room"
        } else {
            "⛔ CANNOT clear: the irreducible work alone overruns the frame budget"
        }
    );
    println!(
        "║ ⚠ this fixture's OWN requirement is {:.2}× (T/B). §2f's 13.5–15.8× is IPC 18 750",
        per_step / FRAME_BUDGET_MS
    );
    println!("║   WITH contact — a different fixture. Do not read the ceiling against it.");
    println!("╚═");
    Verdict {
        per_step_ms: per_step,
        irreducible_ms: irreducible,
        margin,
        requirement: per_step / FRAME_BUDGET_MS,
        ceiling_lo: lo,
        ceiling_hi: hi,
    }
}
