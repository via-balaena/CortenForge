//! A3 scorecard — grade the dial-able thigh–knee–shank–ankle twin in one report.
//!
//!   cargo run -p cf-osim --example a3_scorecard
//!
//! Consolidates the three-tier validation pyramid (recon `03_phases/leg_region`)
//! for a *generated* body into one runnable scorecard, the way `cf-msk-fit`'s
//! `g1_scorecard` did for the *placed/scanned* knee:
//!   [T1] differential-oracle COVERAGE — are the body's per-axis factors inside the
//!        OpenSim-graded grid envelope? (the grid result is cited, not recomputed)
//!   [T2] internal consistency — axial lengths realize exactly; deterministic.
//!   [T3] plausibility + shape-corr vs canonical — gated ≥0.95 for the coupled
//!        regime, reported (≥0.80 floor) for the decoupled tail.
//! Honesty: T1+T2 prove the machinery matches OpenSim's scaling; T3 proves
//! plausibility, NOT personhood.

use cf_msk_lib::anthro::{AnthroSource, Sex};
use cf_msk_lib::{BodyParams, ParamSource};
use cf_osim::parse_leg_chain;
use cf_osim::scorecard::{DiffOracleEnvelope, Regime, Scorecard};

fn assets() -> String {
    format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392",
        env!("CARGO_MANIFEST_DIR")
    )
}

fn yn(b: bool) -> &'static str {
    if b { "PASS" } else { "FAIL" }
}

fn main() {
    let template =
        parse_leg_chain(&std::fs::read_to_string(format!("{}/gait2392.osim", assets())).unwrap());
    let grid: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(format!("{}/scaled_moment_arms_opensim.json", assets())).unwrap(),
    )
    .unwrap();
    let sc = Scorecard::new(&template, DiffOracleEnvelope::from_grid_json(&grid));

    // The dial set: the canonical reference, a coupled percentile sweep (both
    // sexes, girth tracks stature), and two extreme decoupled builds (the boundary).
    let mut bodies: Vec<(String, BodyParams, Regime)> = vec![(
        "canonical (50th-M)".to_string(),
        BodyParams::IDENTITY,
        Regime::Coupled,
    )];
    for sex in [Sex::Male, Sex::Female] {
        for &p in &[0.05, 0.5, 0.95] {
            let src = AnthroSource::new(sex, p);
            bodies.push((
                format!("{sex:?} {:.0}th", p * 100.0),
                src.params(&template),
                Regime::Coupled,
            ));
        }
    }
    bodies.push((
        "M 90th + lean (girth 5th)".to_string(),
        AnthroSource::new(Sex::Male, 0.90)
            .with_girth_percentile(0.05)
            .params(&template),
        Regime::Decoupled,
    ));
    bodies.push((
        "F 10th + stocky (girth 90th)".to_string(),
        AnthroSource::new(Sex::Female, 0.10)
            .with_girth_percentile(0.90)
            .params(&template),
        Regime::Decoupled,
    ));

    let pop = sc.grade_population(&bodies);

    println!("=== A3 SCORECARD — dial-able thigh–knee–shank–ankle twin (scan-free) ===");
    println!("T1 cited: {}\n", cf_osim::scorecard::T1_CITATION);
    println!(
        "{:<28} {:>6} {:>6} {:>9} {:>5} {:>8} {:>5}",
        "body", "femur", "tibia", "T2 resid", "T1", "corr", "verd"
    );
    for c in &pop.cards {
        let t2_ok = c.t2.exact();
        let resid = c.t2.femur_axial_residual_m.max(c.t2.tibia_axial_residual_m);
        println!(
            "{:<28} {:>5.3}m {:>5.3}m {:>9.0e} {:>5} {:>8.4} {:>5}",
            c.label,
            c.t3.femur_axial_m,
            c.t3.tibia_axial_m,
            resid,
            if c.t1.in_envelope { "in" } else { "OUT" },
            c.t3.worst_corr,
            yn(t2_ok && c.t3.passes()),
        );
        for flag in &c.t1.extrapolated {
            println!("    ⚠ T1 extrapolated: {flag}");
        }
    }

    println!("\n--- population aggregate ({} bodies) ---", pop.n);
    println!(
        "  in differential-oracle envelope : {}/{}",
        pop.n_in_envelope, pop.n
    );
    println!(
        "  lengths plausible               : {}/{}",
        pop.n_lengths_plausible, pop.n
    );
    println!("  Tier-2 morph exact (all)        : {}", pop.all_t2_exact);
    println!(
        "  coupled bodies ({})  worst corr  : {:.4}  (gate ≥0.95)",
        pop.n_coupled, pop.worst_corr_coupled
    );
    println!(
        "  decoupled tail ({}) worst corr   : {:.4}  (reported, floor ≥0.80)",
        pop.n_decoupled, pop.worst_corr_decoupled
    );
    println!(
        "  median worst-muscle corr        : {:.4}",
        pop.median_worst_corr
    );
    println!(
        "\n=== A3 GATE: {} (T2 exact + plausible + coupled shape-corr ≥0.95; tail reported) ===",
        yn(pop.passes())
    );
}
