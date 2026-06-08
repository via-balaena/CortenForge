//! A4 gate — a sampled population of leg twins grades green through the A3
//! scorecard. Mirrors `examples/a4_population`.
//!
//! This is the A4 deliverable's formal check: `RandomizerSource` (cf-msk-lib) draws
//! a reproducible population over the validated `AnthroSource` family, and the
//! consolidated `cf_osim::scorecard` grades the whole population. It pins that the
//! randomizer produces a **valid, plausible, machinery-correct** population covering
//! BOTH regimes — coverage + machinery, not personhood.

use cf_msk_lib::{ParamSource, RandomizerConfig, RandomizerSource};
use cf_osim::parse_leg_chain;
use cf_osim::scorecard::{BodyEntry, DiffOracleEnvelope, PopulationScorecard, Regime, Scorecard};

const SEED: u64 = 20_260_608;
const N: usize = 300;

fn assets() -> String {
    format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392",
        env!("CARGO_MANIFEST_DIR")
    )
}

fn template() -> cf_msk_lib::Model {
    parse_leg_chain(&std::fs::read_to_string(format!("{}/gait2392.osim", assets())).unwrap())
}

fn envelope() -> DiffOracleEnvelope {
    let grid: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(format!("{}/scaled_moment_arms_opensim.json", assets())).unwrap(),
    )
    .unwrap();
    DiffOracleEnvelope::from_grid_json(&grid)
}

/// Grade a population sampled from `randomizer` at `seed`.
fn grade(
    sc: &Scorecard,
    template: &cf_msk_lib::Model,
    randomizer: &RandomizerSource,
    seed: u64,
    n: usize,
) -> PopulationScorecard {
    let bodies: Vec<BodyEntry> = randomizer
        .population(seed, n)
        .iter()
        .enumerate()
        .map(|(i, src)| {
            BodyEntry::new(
                format!("r{i}"),
                src.params(template),
                Regime::from_coupled(src.is_coupled()),
            )
        })
        .collect();
    sc.grade_population(&bodies)
}

/// The headline A4 gate: a default-config population is fully valid — every body
/// Tier-2 exact + plausible, the coupled majority clears ≥0.95, and the bounded
/// decoupled tail clears its reported ≥0.80 floor. The population gate passes.
#[test]
fn a4_gate_default_population_passes() {
    let t = template();
    let sc = Scorecard::new(&t, envelope());
    let pop = grade(&sc, &t, &RandomizerSource::new(), SEED, N);

    assert_eq!(pop.n, N);
    assert!(
        pop.all_t2_exact,
        "every sampled body's morph must be Tier-2 exact"
    );
    assert_eq!(
        pop.n_lengths_plausible, N,
        "every sampled body must be plausible ({} of {N})",
        pop.n_lengths_plausible
    );
    // Both regimes are actually exercised (the tail isn't empty, the majority isn't).
    assert!(
        pop.n_coupled > 0 && pop.n_decoupled > 0,
        "both regimes must appear"
    );
    assert!(
        pop.worst_corr_coupled >= 0.95,
        "coupled worst shape-corr {:.4} < 0.95",
        pop.worst_corr_coupled
    );
    assert!(
        pop.worst_corr_decoupled >= 0.80,
        "decoupled worst shape-corr {:.4} < 0.80 floor",
        pop.worst_corr_decoupled
    );
    // The whole coupled-by-default family lands in the differential-oracle envelope.
    assert_eq!(
        pop.n_in_envelope, N,
        "{} of {N} bodies extrapolated beyond the grid",
        pop.n_in_envelope
    );
    assert!(pop.passes(), "A4 population gate failed");
}

/// Grading is reproducible: the same seed yields the same aggregate verdict and
/// counts (the training-data regeneration guarantee, end to end through the morph
/// + oracle).
#[test]
fn a4_gate_population_is_reproducible() {
    let t = template();
    let sc = Scorecard::new(&t, envelope());
    let r = RandomizerSource::new();
    let a = grade(&sc, &t, &r, SEED, 128);
    let b = grade(&sc, &t, &r, SEED, 128);
    assert_eq!(a.n_coupled, b.n_coupled);
    assert_eq!(a.n_decoupled, b.n_decoupled);
    assert_eq!(a.n_in_envelope, b.n_in_envelope);
    assert!((a.worst_corr_coupled - b.worst_corr_coupled).abs() < 1e-12);
    assert!((a.median_worst_corr - b.median_worst_corr).abs() < 1e-12);
}

/// A coupled-only population (no decoupled tail) is entirely held to — and clears —
/// the strict ≥0.95 gate.
#[test]
fn a4_gate_coupled_only_population_all_strict() {
    let t = template();
    let sc = Scorecard::new(&t, envelope());
    let r = RandomizerSource::with_config(RandomizerConfig {
        decoupled_fraction: 0.0,
        ..RandomizerConfig::default()
    });
    let pop = grade(&sc, &t, &r, SEED, N);
    assert_eq!(
        pop.n_decoupled, 0,
        "coupled-only config must yield no decoupled bodies"
    );
    assert!(pop.worst_corr_coupled >= 0.95);
    assert!(pop.passes());
}
