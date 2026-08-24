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
