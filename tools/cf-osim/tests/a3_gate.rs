//! A3 gate — the formal validation tolerances for the dial-able thigh–knee–shank–
//! ankle twin, asserted through the consolidated three-tier `cf_osim::scorecard`.
//! Mirrors `examples/a3_scorecard`, the way `cf-msk-fit`'s `g1_gate` mirrors its
//! `g1_scorecard`.
//!
//! This is the ONE place the A3 validation lives now: it supersedes the bespoke
//! assertions previously split across `anthro_validation.rs` (plausibility +
//! shape-corr) and routes them through the scorecard, plus the family-level
//! ordering checks that are generator properties (not per-body). The exact
//! morph round-trips stay unit-tested in `cf-msk-lib`; here they are re-asserted
//! per body as Tier-2 exactness.
//!
//! Honesty (recon's two-claims rule): T1+T2 prove the machinery matches OpenSim's
//! scaling; T3 proves plausibility, NOT personhood.

use cf_msk_lib::anthro::{AnthroSource, Sex};
use cf_msk_lib::{BodyParams, ParamSource, realize};
use cf_osim::parse_leg_chain;
use cf_osim::scorecard::{DiffOracleEnvelope, Regime, Scorecard};

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

/// The representative dial set: the canonical reference + a coupled percentile
/// sweep (both sexes) + the two extreme decoupled boundary builds.
fn dial_set(t: &cf_msk_lib::Model) -> Vec<(String, BodyParams, Regime)> {
    let mut bodies = vec![(
        "canonical".to_string(),
        BodyParams::IDENTITY,
        Regime::Coupled,
    )];
    for sex in [Sex::Male, Sex::Female] {
        for &p in &[0.05, 0.5, 0.95] {
            bodies.push((
                format!("{sex:?}_{p}"),
                AnthroSource::new(sex, p).params(t),
                Regime::Coupled,
            ));
        }
    }
    bodies.push((
        "tall_lean".to_string(),
        AnthroSource::new(Sex::Male, 0.90)
            .with_girth_percentile(0.05)
            .params(t),
        Regime::Decoupled,
    ));
    bodies.push((
        "short_stocky".to_string(),
        AnthroSource::new(Sex::Female, 0.10)
            .with_girth_percentile(0.90)
            .params(t),
        Regime::Decoupled,
    ));
    bodies
}

/// The headline gate: over the representative dial set, every Tier-2 morph is exact,
/// every body is plausible, and the coupled family clears the §7 ≥0.95 shape-corr
/// bar. The decoupled tail is reported, not gated at 0.95 (asserted separately).
#[test]
fn a3_gate_population_passes() {
    let t = template();
    let sc = Scorecard::new(&t, envelope());
    let pop = sc.grade_population(&dial_set(&t));

    for c in &pop.cards {
        assert!(
            c.t2.exact(),
            "{}: Tier-2 morph not exact (femur resid {:.1e}, tibia {:.1e}, det {})",
            c.label,
            c.t2.femur_axial_residual_m,
            c.t2.tibia_axial_residual_m,
            c.t2.deterministic
        );
        assert!(
            c.t3.lengths_plausible,
            "{}: implausible lengths (femur {:.3}, tibia {:.3})",
            c.label, c.t3.femur_axial_m, c.t3.tibia_axial_m
        );
    }
    assert!(
        pop.worst_corr_coupled >= 0.95,
        "coupled family broke the §7 shape-corr bar: {:.4} < 0.95",
        pop.worst_corr_coupled
    );
    assert!(pop.passes(), "A3 population gate failed");
}

/// The whole coupled generated family lands INSIDE the differential-oracle envelope
/// — the purpose of the A3-PR4 grid extension (the grid now carries the generator's
/// own coupled extremes for both sexes, so coupled bodies are interpolated within an
/// OpenSim-graded region, not extrapolated).
#[test]
fn a3_gate_coupled_family_in_envelope() {
    let t = template();
    let sc = Scorecard::new(&t, envelope());
    for sex in [Sex::Male, Sex::Female] {
        for &p in &[0.01, 0.05, 0.5, 0.95, 0.99] {
            let card = sc.grade(
                &format!("{sex:?}_{p}"),
                &AnthroSource::new(sex, p).params(&t),
                Regime::Coupled,
            );
            assert!(
                card.t1.in_envelope,
                "{sex:?} {p}: coupled body extrapolated beyond the grid: {:?}",
                card.t1.extrapolated
            );
        }
    }
}

/// The decoupled tail is the documented boundary: below the tight 0.95 bar but above
/// the loose 0.80 sanity floor — REPORTED, not failed. (Pins PR1's "girth is
/// ~20–30%, not second-order": an extreme transverse-only offset moves curve shape.)
#[test]
fn a3_gate_decoupled_tail_reported_within_floor() {
    let t = template();
    let sc = Scorecard::new(&t, envelope());
    let extreme = AnthroSource::new(Sex::Male, 0.90).with_girth_percentile(0.05);
    let card = sc.grade("tall_lean", &extreme.params(&t), Regime::Decoupled);
    assert!(
        card.t3.worst_corr < 0.95,
        "expected the extreme tail below 0.95 (got {:.4}) — if this rises, the boundary moved",
        card.t3.worst_corr
    );
    assert!(
        card.t3.worst_corr >= 0.80,
        "extreme decoupled fell below the sanity floor: {:.4} < 0.80",
        card.t3.worst_corr
    );
    // It still "passes" — its honest gate is the loose floor, not 0.95.
    assert!(card.passes(), "decoupled body should pass its regime floor");
}

/// Family-level ordering (a generator property, not a single body's): taller
/// percentile → longer segments; female 50th shorter than male 50th; the reference
/// (male 50th) reproduces the template exactly. Carried over from the retired
/// `anthro_validation::dialed_lengths_are_plausible_and_ordered`.
#[test]
fn a3_gate_family_lengths_ordered() {
    let t = template();
    let femur = |s: Sex, p: f64| {
        realize(&t, &AnthroSource::new(s, p).params(&t)).segment_axial_length("femur_r", "tibia_r")
    };
    assert!(femur(Sex::Male, 0.95) > femur(Sex::Male, 0.05));
    assert!(femur(Sex::Female, 0.5) < femur(Sex::Male, 0.5));
    assert!(
        (femur(Sex::Male, 0.5) - t.segment_axial_length("femur_r", "tibia_r")).abs() < 1e-9,
        "male 50th must reproduce the template femur length"
    );
}
