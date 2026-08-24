//! Pilot for the contention gate's threshold. NOT a gate itself.
//!
//! Thresholds are LEARNED, not guessed — a number picked from intuition is how
//! R1.0 shipped a gate that read FAIL at 2.7 % on its first honest run. This
//! prints the probe's natural spread over repeated runs so the threshold can be
//! set above what a quiet box actually does, and then negative-controlled under
//! deliberate load.
//!
//! ```text
//! cargo test --release -p sim-soft --test refbox_pilot -- --ignored --nocapture
//! ```

#![allow(clippy::print_stdout, clippy::panic, clippy::expect_used)]

mod refbox;

#[test]
#[ignore = "pilot, ~1 min — run explicitly when re-tuning the contention gate"]
fn pilot_probe_spread() {
    let id = refbox::BoxIdentity::detect();
    id.stamp();
    println!("│ (pilot does NOT require the reference box — it characterises whatever it runs on)");
    let mut bursts = Vec::new();
    let mut p50s = Vec::new();
    for i in 0..10 {
        let p = refbox::probe();
        println!(
            "run {i:2}  p50 {:8.3} ms   min {:8.3}   max {:8.3}   burst {:.3}x   iters {}",
            p.p50_ms,
            p.min_ms,
            p.max_ms,
            p.burst(),
            p.iters
        );
        bursts.push(p.burst());
        p50s.push(p.p50_ms);
    }
    bursts.sort_by(|a, b| a.partial_cmp(b).expect("no NaN"));
    p50s.sort_by(|a, b| a.partial_cmp(b).expect("no NaN"));
    println!(
        "\nburst  min {:.3}x  median {:.3}x  MAX {:.3}x",
        bursts[0], bursts[5], bursts[9]
    );
    println!(
        "p50    min {:.3} ms  median {:.3} ms  max {:.3} ms",
        p50s[0], p50s[5], p50s[9]
    );
    println!(
        "\n⇒ set BURST_MAX above {:.3}x, and the p50 band around {:.3} ms",
        bursts[9], p50s[5]
    );
}

/// What phase does the probe actually stress?
///
/// The probe's design rationale is that it loads the same phase the real
/// measurements are bottlenecked on. §2d's `57.5 % numeric factor` at 3 000 DOF
/// is NOT evidence for that — that row is `dt = 1/60` with many Newton
/// iterations, and the probe runs `dt = 1e-3` with three. Different regime, so
/// it needs measuring rather than citing.
///
/// ```text
/// cargo test --release -p sim-soft --features phase-timing \
///   --test refbox_pilot -- --ignored --nocapture probe_phase_character
/// ```
#[test]
#[ignore = "pilot, ~1 s — characterises the probe's phase mix"]
#[cfg(feature = "phase-timing")]
fn probe_phase_character() {
    use sim_soft::profile::{self, Phase};
    profile::reset();
    let p = refbox::probe();
    // `probe` resets on the way out, so re-run one under a fresh window. The
    // returned stats are irrelevant here — only the counters it left behind are.
    profile::reset();
    let _unused = refbox::probe_inner_for_profiling();
    let ph = profile::snapshot();
    println!(
        "\nprobe phase mix (p50 {:.2} ms, {} iters/step):",
        p.p50_ms, p.iters
    );
    for phase in Phase::ALL {
        println!("  {:<16} {:6.1} %", phase.label(), 100.0 * ph.share(phase));
    }
}
