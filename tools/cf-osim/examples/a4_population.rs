//! A4 population — sample a population of leg twins and grade it with the A3
//! scorecard in one report.
//!
//!   cargo run -p cf-osim --example a4_population
//!
//! `RandomizerSource` (cf-msk-lib) samples a reproducible population over the
//! validated `AnthroSource` family — coupled-by-default with a bounded decoupled
//! tail — and `cf_osim::scorecard::grade_population` grades it (T2 exact + T3
//! plausibility/shape-corr + T1 coverage). This validates **coverage + the
//! machinery, not personhood**: the population is free training data whose every
//! member is a plausible, OpenSim-scaling-faithful body, with the harder
//! anisotropic regime reported (not hidden).

use cf_msk_lib::{ParamSource, RandomizerSource};
use cf_osim::parse_leg_chain;
use cf_osim::scorecard::{BodyEntry, DiffOracleEnvelope, Regime, Scorecard};

const SEED: u64 = 20_260_608;
const N: usize = 500;

fn assets() -> String {
    format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392",
        env!("CARGO_MANIFEST_DIR")
    )
}

fn main() {
    let template =
        parse_leg_chain(&std::fs::read_to_string(format!("{}/gait2392.osim", assets())).unwrap());
    let grid: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(format!("{}/scaled_moment_arms_opensim.json", assets())).unwrap(),
    )
    .unwrap();
    let sc = Scorecard::new(&template, DiffOracleEnvelope::from_grid_json(&grid));

    // Sample a reproducible population; each body keeps its coupled/decoupled
    // provenance (never inferred).
    let randomizer = RandomizerSource::new();
    let bodies: Vec<BodyEntry> = randomizer
        .population(SEED, N)
        .iter()
        .enumerate()
        .map(|(i, src)| {
            BodyEntry::new(
                format!("r{i:03}"),
                src.params(&template),
                Regime::from_coupled(src.is_coupled()),
            )
        })
        .collect();

    let pop = sc.grade_population(&bodies);

    println!("=== A4 POPULATION SCORECARD — {N} randomized leg twins (seed {SEED}) ===");
    println!("config: {:?}\n", randomizer.config());
    println!(
        "  coupled / decoupled             : {} / {}",
        pop.n_coupled, pop.n_decoupled
    );
    println!("  Tier-2 morph exact (all)        : {}", pop.all_t2_exact);
    println!(
        "  lengths plausible               : {}/{}",
        pop.n_lengths_plausible, pop.n
    );
    println!(
        "  in differential-oracle envelope : {}/{}  (per-axis coverage)",
        pop.n_in_envelope, pop.n
    );
    println!(
        "  coupled worst shape-corr        : {:.4}  (gate ≥0.95)",
        pop.worst_corr_coupled
    );
    println!(
        "  decoupled worst shape-corr      : {:.4}  (reported, floor ≥0.80)",
        pop.worst_corr_decoupled
    );
    println!(
        "  median worst-muscle shape-corr  : {:.4}",
        pop.median_worst_corr
    );

    let yn = if pop.passes() { "PASS" } else { "FAIL" };
    println!("\n=== A4 GATE: {yn} (every body T2-exact + plausible + above its regime floor) ===");
    println!("    coverage + machinery validated — NOT personhood.");
}
