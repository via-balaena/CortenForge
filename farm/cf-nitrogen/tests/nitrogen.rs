//! Gates for stage 5 — the nitrogen leg.
//!
//! Pre-registered from three rosters, in the order the previous stage's
//! defects say to try them:
//!
//! 1. **The classes PR #944 produced** — a number stated in prose with no
//!    producer, a comparison whose two sides use different denominators, an
//!    unsourced empirical claim the conclusion rests on, and a digest nothing
//!    computes.
//! 2. **This crate's own new surfaces** — NASS's withheld `(D)` cell, the
//!    production-practice key, and the survey's three-way self-checksum.
//! 3. **Standing invariants** — an absence claim asserts its collection, every
//!    constant has a gate, and every gate was made to fail.

#![allow(
    clippy::panic,
    reason = "assert! in a test is the failure mechanism, not an escape hatch"
)]
#![allow(
    clippy::expect_used,
    reason = "a gate that cannot name what it could not build is not much of a gate"
)]
#![allow(
    clippy::cast_precision_loss,
    reason = "sample counts here are ~10^5 and exact in f64"
)]

use cf_electrolysis::{Electrolyser, FixedSpecificEnergy, current_distributed};
use cf_nitrogen::{
    ACRES_FERTILIZED_PER_TURBINE_YEAR, ACRES_PLANTED_SERVED_PER_TURBINE_YEAR, ALL_PRACTICES,
    CHECKSUM_DISAGREEMENTS, CHECKSUM_INDETERMINATE, CHECKSUM_RECONCILES, CIAAW_ATOMIC_WEIGHTS,
    CIRCULATED_HYDROGEN_FRACTION, CORN_BELT_RATE_OVERSTATEMENT_PERCENT, CORN_HYDROGEN_KG_PER_ACRE,
    CORN_NITROGEN_LB_PER_ACRE, CORN_REFERENCE_YEAR, Cell, HYDROGEN_ATOMIC_WEIGHT, LB_KG, Measure,
    NASS_CHEMICAL_USE, NASS_TSV_SHA256, NITROGEN_ATOMIC_WEIGHT, NitrogenDemand, Printed,
    UNMEASURED_HERE, ammonia_kg_per_acre, ammonia_molar_mass, hydrogen_kg_per_acre,
    hydrogen_mass_fraction, nitrogen_mass_fraction, observations, parse_line, planted_acres,
    record, records, unparsed_lines,
};
use cf_storage::{Demand, debit_compression};
use cf_tillage::{FallYear, NOMINAL_RULE, NOMINAL_YEAR};
use cf_wind::{Air, EWT_DW54X, FOSTER_COUNTY_ND_2012 as YEAR, Machine};

const AIR: Air = Air {
    density_kg_m3: 1.16,
};
const RATED_W: f64 = 1.0e6;
const TURNDOWN: f64 = 0.10;
const OUTLET_BAR: f64 = 20.68;
const TANK_BAR: f64 = 350.0;
const STATION_EFFICIENCY: f64 = 0.52;

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

/// Stages 1-3: delivered kilograms at 350 bar, recomputed on the real series.
fn delivered_series() -> Vec<f64> {
    let plant = FixedSpecificEnergy::from_case(&current_distributed(), RATED_W, TURNDOWN);
    let floor = plant.rated_power_w() * plant.min_load_fraction();
    let dt = YEAR.interval_seconds();
    let tank = cf_storage::Tank::new(TANK_BAR, cf_storage::CARRINGTON_FALL.mean_k);
    let work = tank
        .compression_kwh_per_kg(OUTLET_BAR, &cf_storage::Covolume, STATION_EFFICIENCY)
        .expect("compression work");
    let specific = current_distributed().total_kwh_per_kg.value();
    YEAR.speeds()
        .map(|v| EWT_DW54X.power_w(v, AIR))
        .map(|p| {
            if p < floor {
                return 0.0;
            }
            let used = p.min(plant.rated_power_w());
            let se = plant.specific_energy_kwh_per_kg(used / plant.rated_power_w());
            if se > 0.0 {
                used * dt / 3.6e6 / se
            } else {
                0.0
            }
        })
        .map(|kg| debit_compression(kg, specific, work).map_or(0.0, |d| d.kg))
        .collect()
}

// ═══════════════════════════════════ roster 2: the committed survey extract

#[test]
fn the_extract_parses_completely() {
    assert_eq!(unparsed_lines(), 0, "every data line must parse");
    assert_eq!(observations().len(), 356, "the committed row count");
    assert!(
        observations()
            .iter()
            .all(|o| o.cell.published().is_none_or(|p| p.value().is_finite())),
        "no non-finite value survives the parser"
    );
}

#[test]
fn the_committed_extract_matches_its_digest() {
    use sha2::{Digest, Sha256};
    use std::fmt::Write as _;
    let bytes = include_bytes!("../data/nd_nitrogen.tsv");
    let mut hex = String::with_capacity(64);
    for b in Sha256::digest(bytes) {
        write!(hex, "{b:02x}").expect("writing to a String cannot fail");
    }
    assert_eq!(
        hex, NASS_TSV_SHA256,
        "the extract no longer matches its digest"
    );
    assert_eq!(NASS_TSV_SHA256.len(), 64);
    assert!(NASS_TSV_SHA256.chars().all(|c| c.is_ascii_hexdigit()));
}

#[test]
fn the_line_parser_rejects_every_way_a_line_can_be_wrong() {
    assert!(
        parse_line("CORN\tALL CLASSES\tALL PRODUCTION PRACTICES\t2021\tPER_YEAR\t118").is_some()
    );
    assert!(
        parse_line("CORN\tALL CLASSES\tALL PRODUCTION PRACTICES\t2009\tPER_YEAR\t(D)").is_some()
    );
    for bad in [
        "",
        "CORN",
        "CORN\tALL CLASSES\tALL PRODUCTION PRACTICES\t2021",
        "CORN\tALL CLASSES\tALL PRODUCTION PRACTICES\tnotayear\tPER_YEAR\t118",
        "CORN\tALL CLASSES\tALL PRODUCTION PRACTICES\t2021\tNITROGEN\t118",
        "CORN\tALL CLASSES\tALL PRODUCTION PRACTICES\t2021\tPER_YEAR\tnope",
        "CORN\tALL CLASSES\tALL PRODUCTION PRACTICES\t2021\tPER_YEAR\t118\textra",
        "\tALL CLASSES\tALL PRODUCTION PRACTICES\t2021\tPER_YEAR\t118",
        "CORN\tALL CLASSES\t\t2021\tPER_YEAR\t118",
    ] {
        assert!(parse_line(bad).is_none(), "{bad:?} should not parse");
    }
}

#[test]
fn a_withheld_cell_is_not_a_missing_one() {
    // ⛔ NASS suppresses a cell when publishing it would disclose an individual
    // operation. The quantity EXISTS and the survey knows it. Collapsing that
    // into a missing value loses the difference between "not surveyed" and
    // "surveyed and withheld".
    let withheld: Vec<_> = observations()
        .iter()
        .filter(|o| o.cell.is_withheld())
        .collect();
    assert_eq!(withheld.len(), 4, "the extract carries four withheld cells");
    for o in &withheld {
        assert!(
            o.cell.published().is_none(),
            "a withheld cell has no figure"
        );
        assert_eq!(
            o.practice, "ORGANIC",
            "every withheld cell here is organic, which is why NASS suppressed it"
        );
    }
    // And they are not silently zero.
    assert!(
        withheld.iter().all(|o| !close(
            o.cell.published().map_or(f64::NAN, Printed::value),
            0.0,
            0.0
        )),
        "a withheld cell must never read as zero"
    );
}

#[test]
fn the_production_practice_is_part_of_the_key() {
    // ⛔⛔ The defect this crate shipped and caught: without `practice` in the
    // key, organic spring wheat 2009 and conventional spring wheat 2009 collide,
    // and `records()` silently returns whichever the search reached first.
    let mut keys: Vec<_> = observations()
        .iter()
        .map(|o| (o.crop, o.class, o.practice, o.year, o.measure))
        .collect();
    let before = keys.len();
    keys.sort_unstable_by_key(|k| (k.0, k.1, k.2, k.3, k.4.code()));
    keys.dedup();
    assert_eq!(before, keys.len(), "the full key must be unique");

    // Dropping the practice really does collide — the gate is not vacuous.
    let mut without: Vec<_> = observations()
        .iter()
        .map(|o| (o.crop, o.class, o.year, o.measure))
        .collect();
    let n = without.len();
    without.sort_unstable_by_key(|k| (k.0, k.1, k.2, k.3.code()));
    without.dedup();
    assert!(
        without.len() < n,
        "without the practice the key collides; that is why it is in the key"
    );
}

#[test]
fn organic_is_carried_and_excluded() {
    // An organic acre receives no synthetic anhydrous ammonia, so it must not
    // enter a Haber-Bosch hydrogen model — but it must stay visible.
    let organic: Vec<_> = records()
        .into_iter()
        .filter(|r| r.practice != ALL_PRACTICES)
        .collect();
    assert_eq!(organic.len(), 1, "one organic record exists in the extract");
    assert_eq!(organic[0].practice, "ORGANIC");
    assert!(
        organic[0].reconciles().is_none(),
        "it is entirely withheld, so the checksum cannot judge it either way"
    );
    // Every figure this crate calculates with comes from ALL PRODUCTION PRACTICES.
    let reference = record("CORN", "ALL CLASSES", ALL_PRACTICES, CORN_REFERENCE_YEAR)
        .expect("the reference record");
    assert_eq!(reference.practice, ALL_PRACTICES);
}

// ══════════════════════════════ roster 2: the survey's own three-way checksum

#[test]
fn the_survey_reconciles_with_itself() {
    // ★★ per-application × applications must reproduce per-year, by
    // ROUNDING-INTERVAL overlap — NASS prints the count to one decimal.
    let recs = records();
    let ok = recs.iter().filter(|r| r.reconciles() == Some(true)).count();
    let bad = recs
        .iter()
        .filter(|r| r.reconciles() == Some(false))
        .count();
    let unknown = recs.iter().filter(|r| r.reconciles().is_none()).count();
    assert_eq!(ok, CHECKSUM_RECONCILES, "records that reconcile");
    assert_eq!(bad, CHECKSUM_DISAGREEMENTS.len(), "records that do not");
    assert_eq!(unknown, CHECKSUM_INDETERMINATE, "records it cannot judge");
    assert_eq!(
        ok + bad + unknown,
        recs.len(),
        "every record is accounted for"
    );
    assert!(
        ok > 80,
        "the collection is populated, so this is not passing by being empty"
    );
}

#[test]
fn the_disagreements_are_the_recorded_ones() {
    // ⚠ Recorded, not repaired. Both are ND corn and both miss by about a pound.
    let bad: Vec<(&str, &str, u16)> = records()
        .iter()
        .filter(|r| r.reconciles() == Some(false))
        .map(|r| (r.crop, r.class, r.year))
        .collect();
    assert_eq!(bad.len(), CHECKSUM_DISAGREEMENTS.len());
    for entry in CHECKSUM_DISAGREEMENTS {
        assert!(
            bad.contains(entry),
            "{entry:?} should be a recorded disagreement"
        );
    }
}

#[test]
fn a_mistyped_rate_breaks_the_checksum() {
    // The gate above made to fail. `Record` is built from the extract, so this
    // exercises the arithmetic directly on a record with known-good figures.
    let r = record("CORN", "ALL CLASSES", ALL_PRACTICES, CORN_REFERENCE_YEAR).expect("record");
    assert_eq!(
        r.reconciles(),
        Some(true),
        "2021 corn reconciles as published"
    );
    let per_app = r
        .per_application
        .and_then(Cell::published)
        .expect("per app");
    let apps = r.applications.and_then(Cell::published).expect("apps");
    let per_year = r.per_year.and_then(Cell::published).expect("per year");
    // The product of the intervals must NOT admit a transposed year figure.
    let transposed = per_year.value() * 2.0;
    let low = per_app.low() * apps.low();
    let high = per_app.high() * apps.high();
    assert!(
        !(low <= transposed && transposed <= high),
        "doubling the annual rate must fall outside [{low}, {high}]"
    );
}

#[test]
fn the_percent_of_area_is_a_different_denominator() {
    // ⚠⚠ The per-acre rates are per TREATED acre. In 1990 only 80% of North
    // Dakota corn was treated, so nitrogen per PLANTED acre was a fifth lower.
    let early = record("CORN", "ALL CLASSES", ALL_PRACTICES, 1990).expect("1990");
    let late = record("CORN", "ALL CLASSES", ALL_PRACTICES, CORN_REFERENCE_YEAR).expect("2021");
    let pct = |r: &cf_nitrogen::Record| {
        r.pct_of_area_planted
            .and_then(Cell::published)
            .map(Printed::value)
    };
    let (e, l) = (
        pct(&early).expect("1990 pct"),
        pct(&late).expect("2021 pct"),
    );
    assert!(close(e, 80.0, 0.5), "1990 treated share is {e}%");
    assert!(close(l, 99.0, 0.5), "2021 treated share is {l}%");
    assert!(
        e < l,
        "the treated share rose, so the gap between treated and planted acres shrank"
    );
    assert_eq!(
        Measure::PctOfAreaPlanted.nass_unit(),
        "PCT OF AREA PLANTED, AVG",
        "the measure must name its own denominator"
    );
}

// ═══════════════════════════════ roster 2: stoichiometry, derived not quoted

#[test]
fn ammonia_is_built_from_its_two_atomic_weights() {
    let m = ammonia_molar_mass();
    assert!(
        close(
            m,
            NITROGEN_ATOMIC_WEIGHT.value() + 3.0 * HYDROGEN_ATOMIC_WEIGHT.value(),
            0.0
        ),
        "the molar mass is N + 3H and nothing else"
    );
    assert!(close(m, 17.031, 1e-12), "NH3 is {m}");
    assert!(
        close(
            nitrogen_mass_fraction() + hydrogen_mass_fraction(),
            1.0,
            1e-12
        ),
        "the two fractions are the whole molecule"
    );
    // The hydrogen used here is the hydrogen the rest of the chain uses.
    assert!(
        close(
            2.0 * HYDROGEN_ATOMIC_WEIGHT.value() / 1000.0,
            2.016e-3,
            1e-12
        ),
        "twice this atomic weight is cf_storage's molar mass of H2"
    );
}

#[test]
fn the_circulated_hydrogen_fraction_is_not_the_stoichiometric_one() {
    // ⛔ The figure in circulation is 17.6%. Stoichiometry gives 17.756%.
    let derived = hydrogen_mass_fraction();
    assert!(
        close(derived, 0.177_559, 1e-6),
        "derived fraction is {derived}"
    );
    assert!(
        !CIRCULATED_HYDROGEN_FRACTION.admits(derived),
        "the quoted 0.176 does not admit the stoichiometric {derived}"
    );
    let gap = (CIRCULATED_HYDROGEN_FRACTION.value() / derived - 1.0) * 100.0;
    assert!(close(gap, -0.88, 0.02), "the quoted figure is {gap}% off");
    // ⛔ And it is never used in a calculation: the conversion must follow the
    // derived fraction, not the quoted one.
    let by_derived = hydrogen_kg_per_acre(118.0).expect("kg");
    let by_quoted = ammonia_kg_per_acre(118.0).expect("nh3") * CIRCULATED_HYDROGEN_FRACTION.value();
    assert!(
        !close(by_derived, by_quoted, 1e-6),
        "the two routes must differ, or the gate proves nothing"
    );
}

#[test]
fn the_pound_is_the_defined_one() {
    assert!(
        close(LB_KG, 0.453_592_37, 0.0),
        "the international avoirdupois pound is exact"
    );
    let kg_n = 118.0 * LB_KG;
    assert!(
        close(kg_n, 53.524_3, 1e-3),
        "118 lb of nitrogen is {kg_n} kg"
    );
}

#[test]
fn the_corn_acre_hydrogen_is_the_measured_one() {
    let rate = record("CORN", "ALL CLASSES", ALL_PRACTICES, CORN_REFERENCE_YEAR)
        .expect("the reference record")
        .per_year
        .and_then(Cell::published)
        .expect("a published rate");
    assert!(
        CORN_NITROGEN_LB_PER_ACRE.admits(rate.value()),
        "the pinned rate must be the surveyed one: {} vs {}",
        CORN_NITROGEN_LB_PER_ACRE.value(),
        rate.value()
    );
    let h2 = hydrogen_kg_per_acre(rate.value()).expect("kg");
    assert!(
        CORN_HYDROGEN_KG_PER_ACRE.admits(h2),
        "{h2} kg H2/acre against the pinned {}",
        CORN_HYDROGEN_KG_PER_ACRE.value()
    );
    let nh3 = ammonia_kg_per_acre(rate.value()).expect("nh3");
    assert!(close(nh3, 65.079, 1e-2), "{nh3} kg NH3/acre");
    assert!(close(h2 / nh3, hydrogen_mass_fraction(), 1e-12));
}

#[test]
fn the_circulated_rate_overstates_north_dakota() {
    // ⛔⛔ 150-200 lb N/acre is a Corn Belt figure. Measured ND corn runs
    // 118-160, and the most recent reading is the bottom of that.
    let measured = CORN_NITROGEN_LB_PER_ACRE.value();
    let (lo, hi) = CORN_BELT_RATE_OVERSTATEMENT_PERCENT;
    assert!(close((150.0 / measured - 1.0) * 100.0, lo, 0.02), "at 150");
    assert!(close((200.0 / measured - 1.0) * 100.0, hi, 0.02), "at 200");
    // And no year of the survey reaches 200.
    let corn: Vec<f64> = records()
        .iter()
        .filter(|r| r.crop == "CORN" && r.practice == ALL_PRACTICES)
        .filter_map(|r| r.per_year.and_then(Cell::published).map(Printed::value))
        .collect();
    assert!(
        corn.len() >= 8,
        "the collection is populated: {} years",
        corn.len()
    );
    let max = corn.iter().copied().fold(f64::MIN, f64::max);
    assert!(
        max < 200.0,
        "the highest ND corn year on record is {max} lb N/acre"
    );
    assert!(
        corn.iter().filter(|&&v| v >= 150.0).count() <= 1,
        "at most one surveyed year even reaches the bottom of the quoted range"
    );
}

// ═════════════════════════════════════ roster 1: the headline has a producer

#[test]
fn the_acres_fertilized_headline() {
    // ★★★ Recomputed from stages 1-3 on the real series, never quoted.
    let delivered = delivered_series();
    let annual: f64 = delivered.iter().fold(0.0, |a, b| a + b);
    assert!(
        (49_000.0..51_000.0).contains(&annual),
        "the chain delivers {annual} kg, which is not what stages 1-3 measure"
    );
    let per_acre = hydrogen_kg_per_acre(CORN_NITROGEN_LB_PER_ACRE.value()).expect("kg");
    let acres = annual / per_acre;
    assert!(
        ACRES_FERTILIZED_PER_TURBINE_YEAR.admits(acres),
        "{acres} acres against the pinned {}",
        ACRES_FERTILIZED_PER_TURBINE_YEAR.value()
    );
    // ⚠ A CEILING: it counts the hydrogen in the molecule, not what making the
    // molecule costs. The unknown that says so must be present.
    assert!(
        UNMEASURED_HERE
            .iter()
            .any(|u| u.what.contains("synthesis costs on top")),
        "the headline is a ceiling and the crate must say why"
    );
}

#[test]
fn the_nitrogen_leg_outweighs_the_fuel_leg_by_less_than_advertised() {
    // ⚠⚠ The documents lead with 6-8x. That rests on 150-200 lb N/acre. At
    // North Dakota's MEASURED rate the multiple is smaller — and the fuel side
    // here is still an EXTERNAL figure, not one this chain has measured, which
    // is why this gate states both bounds rather than a single number.
    const EXTERNAL_FUEL_KG_PER_ACRE: (f64, f64) = (2.0, 3.0); // Iowa State / USDA ERS, via the research doc
    let nitrogen = hydrogen_kg_per_acre(CORN_NITROGEN_LB_PER_ACRE.value()).expect("kg");
    let lo = nitrogen / EXTERNAL_FUEL_KG_PER_ACRE.1;
    let hi = nitrogen / EXTERNAL_FUEL_KG_PER_ACRE.0;
    assert!(close(lo, 3.85, 0.02), "low multiple is {lo}");
    assert!(close(hi, 5.78, 0.02), "high multiple is {hi}");
    assert!(
        hi < 6.0,
        "at ND's measured rate the multiple does not reach the quoted 6-8x"
    );
    // At the quoted rate it does, which shows the difference is the RATE.
    let at_quoted = hydrogen_kg_per_acre(200.0).expect("kg") / EXTERNAL_FUEL_KG_PER_ACRE.1;
    assert!(
        at_quoted > 6.0,
        "at 200 lb N/acre the quoted range is reached: {at_quoted}"
    );
}

// ══════════════════════════════════════════ the demand, and the shared window

#[test]
fn the_demand_draws_exactly_what_it_was_asked_for() {
    let w = FallYear::get(NOMINAL_YEAR)
        .expect("2012")
        .window(NOMINAL_RULE)
        .expect("window");
    let acres = 1_000.0;
    let d = NitrogenDemand::new(
        NOMINAL_YEAR,
        &w,
        acres,
        CORN_NITROGEN_LB_PER_ACRE.value(),
        YEAR.len(),
        YEAR.interval_seconds(),
    )
    .expect("a demand");
    let want = hydrogen_kg_per_acre(CORN_NITROGEN_LB_PER_ACRE.value()).expect("kg") * acres;
    assert!(
        close(d.total_kg(), want, 1e-6),
        "asked {want}, got {}",
        d.total_kg()
    );
    assert!(close(d.acres(), acres, 0.0));
    assert_eq!(d.samples(), YEAR.len());
    assert!(
        d.name().contains("nitrogen"),
        "the profile must not inherit the tillage name: {}",
        d.name()
    );
}

#[test]
fn the_demand_shares_the_tillage_window() {
    // ⛔ The window is a DECISION with a citation, not a measurement: NASS
    // publishes no fertilizer timing, and puts anhydrous in the same sentence
    // as fall tillage. The two profiles must therefore have the same SHAPE.
    let w = FallYear::get(NOMINAL_YEAR)
        .expect("2012")
        .window(NOMINAL_RULE)
        .expect("window");
    let total = 1_000.0;
    let nitrogen = NitrogenDemand::new(
        NOMINAL_YEAR,
        &w,
        total / hydrogen_kg_per_acre(CORN_NITROGEN_LB_PER_ACRE.value()).expect("kg"),
        CORN_NITROGEN_LB_PER_ACRE.value(),
        YEAR.len(),
        YEAR.interval_seconds(),
    )
    .expect("nitrogen");
    let tillage = cf_tillage::TillageDemand::new(
        NOMINAL_YEAR,
        &w,
        total,
        YEAR.len(),
        YEAR.interval_seconds(),
    )
    .expect("tillage");
    assert!(
        close(nitrogen.total_kg(), tillage.total_kg(), 1e-6),
        "same total for the test"
    );
    for i in (0..YEAR.len()).step_by(997) {
        assert!(
            close(nitrogen.kg_at(i), tillage.kg_at(i), 1e-9),
            "sample {i}: the two profiles must have the same shape"
        );
    }
    assert!(
        UNMEASURED_HERE
            .iter()
            .any(|u| u.what.contains("when nitrogen is actually applied")),
        "and the crate must record that the window is decided, not measured"
    );
}

#[test]
fn the_demand_refuses_what_it_cannot_shape() {
    let w = FallYear::get(NOMINAL_YEAR)
        .expect("2012")
        .window(NOMINAL_RULE)
        .expect("window");
    let n = YEAR.len();
    let dt = YEAR.interval_seconds();
    let r = CORN_NITROGEN_LB_PER_ACRE.value();
    for bad in [f64::NAN, f64::INFINITY, -1.0] {
        assert!(
            NitrogenDemand::new(NOMINAL_YEAR, &w, bad, r, n, dt).is_none(),
            "{bad} acres"
        );
        assert!(
            NitrogenDemand::new(NOMINAL_YEAR, &w, 100.0, bad, n, dt).is_none(),
            "{bad} lb N"
        );
    }
    assert!(NitrogenDemand::new(NOMINAL_YEAR, &w, 100.0, r, n, 0.0).is_none());
    assert!(NitrogenDemand::new(NOMINAL_YEAR, &w, 100.0, r, 10, dt).is_none());
    // ⚠ Zero acres is ACCEPTED, not refused. A farm that plants no corn has a
    // nitrogen demand of nothing, and that composes; refusing it would push a
    // special case onto every caller. `cf_tillage::TillageDemand` takes a zero
    // total for the same reason. An earlier version of this gate asserted the
    // opposite from preference rather than from anything.
    let none_planted =
        NitrogenDemand::new(NOMINAL_YEAR, &w, 0.0, r, n, dt).expect("zero acres is a profile");
    assert!(close(none_planted.total_kg(), 0.0, 0.0));
    assert!(
        !none_planted.total_kg().is_sign_negative(),
        "and positively zero"
    );
}

#[test]
fn the_conversions_refuse_what_they_cannot_answer() {
    for bad in [f64::NAN, f64::INFINITY, -1.0] {
        assert!(ammonia_kg_per_acre(bad).is_none(), "{bad}");
        assert!(hydrogen_kg_per_acre(bad).is_none(), "{bad}");
    }
    assert!(
        ammonia_kg_per_acre(0.0).is_some(),
        "zero is a rate, just a boring one"
    );
    assert!(close(hydrogen_kg_per_acre(0.0).expect("zero"), 0.0, 0.0));
}

// ══════════════════════════════════════════════════════ terms and unknowns

#[test]
fn every_source_states_its_own_terms() {
    for s in [NASS_CHEMICAL_USE, CIAAW_ATOMIC_WEIGHTS] {
        assert!(!s.document.is_empty() && !s.url.is_empty());
        assert_eq!(s.retrieved.len(), 10, "{}: an ISO date", s.document);
        assert!(!s.terms.is_empty(), "{}: terms must be stated", s.document);
    }
    assert!(
        NASS_CHEMICAL_USE.terms.contains("105"),
        "the federal source cites the section that makes it free"
    );
    assert!(
        CIAAW_ATOMIC_WEIGHTS.terms.contains("facts"),
        "⛔ IUPAC is not a U.S. federal body; the determination is that the VALUES are facts"
    );
    assert!(
        !CIAAW_ATOMIC_WEIGHTS.terms.contains("17 U.S.C."),
        "and it must NOT borrow the federal reasoning it has no claim to"
    );
}

#[test]
fn the_unknowns_say_what_would_close_them() {
    assert!(UNMEASURED_HERE.len() >= 4);
    for u in UNMEASURED_HERE {
        assert!(!u.what.is_empty() && !u.why_unmeasured.is_empty());
        assert!(
            u.what_would_measure_it.len() > 20,
            "{}: an unknown with no route to closing it is a shrug",
            u.what
        );
    }
    assert!(
        UNMEASURED_HERE
            .iter()
            .any(|u| u.what.contains("fall and spring")),
        "the survey gives an annual rate and no split; that must be recorded"
    );
}

#[test]
fn every_measure_names_its_own_denominator() {
    // ⛔ This crate carries FOUR printings of one quantity and two of them are
    // per-acre with different denominators. A measure that cannot say what it
    // is a measure of is how those get mixed.
    let all = [
        Measure::PerApplication,
        Measure::Applications,
        Measure::PerYear,
        Measure::PctOfAreaPlanted,
    ];
    for m in all {
        assert!(!m.code().is_empty());
        assert!(
            m.nass_unit().len() > m.code().len(),
            "{}: the NASS unit is what says what the rate is a rate of",
            m.code()
        );
        assert_eq!(
            Measure::from_code(m.code()),
            Some(m),
            "the code round-trips"
        );
    }
    // The two per-acre measures must be distinguishable by their unit alone.
    assert!(Measure::PerApplication.nass_unit().contains("APPLICATION"));
    assert!(Measure::PerYear.nass_unit().contains("YEAR"));
    assert_ne!(
        Measure::PerApplication.nass_unit(),
        Measure::PerYear.nass_unit()
    );
    assert_eq!(Measure::from_code("NITROGEN"), None);
    assert_eq!(Measure::from_code(""), None);
}

#[test]
fn a_printing_carries_the_precision_it_was_printed_to() {
    // The rounding-interval contract: a figure printed to one decimal admits a
    // tenth of slack, one printed to none admits a half.
    let whole = Printed::new(118.0, 0);
    let tenth = Printed::new(1.9, 1);
    assert_eq!(whole.decimals(), 0);
    assert_eq!(tenth.decimals(), 1);
    assert!(close(whole.half_ulp(), 0.5, 1e-12));
    assert!(close(tenth.half_ulp(), 0.05, 1e-12));
    assert!(whole.admits(117.6) && whole.admits(118.4));
    assert!(
        !whole.admits(118.6),
        "beyond half a unit it is a different figure"
    );
    // And the extract's own precisions are read, not assumed.
    let apps = record("CORN", "ALL CLASSES", ALL_PRACTICES, CORN_REFERENCE_YEAR)
        .expect("record")
        .applications
        .and_then(Cell::published)
        .expect("apps");
    assert_eq!(apps.decimals(), 1, "NASS prints the count to one decimal");
}

#[test]
fn the_demand_reports_the_rate_it_was_built_at() {
    let w = FallYear::get(NOMINAL_YEAR)
        .expect("2012")
        .window(NOMINAL_RULE)
        .expect("window");
    let rate = CORN_NITROGEN_LB_PER_ACRE.value();
    let d = NitrogenDemand::new(
        NOMINAL_YEAR,
        &w,
        640.0,
        rate,
        YEAR.len(),
        YEAR.interval_seconds(),
    )
    .expect("a demand");
    assert!(
        close(d.lb_n_per_acre(), rate, 0.0),
        "a draw must say what rate produced it"
    );
    assert!(close(d.acres(), 640.0, 0.0));
    // The two together reproduce the total, so neither is decoration.
    //
    // ⚠ Compared RELATIVELY. `total_kg` sums 105,408 per-sample terms, so the
    // absolute error grows with the total while the relative error stays at
    // about 1e-13. An absolute tolerance here is a tolerance on the wrong
    // denominator, and it fails for a big enough farm.
    let want = hydrogen_kg_per_acre(d.lb_n_per_acre()).expect("kg") * d.acres();
    let relative = (d.total_kg() - want).abs() / want;
    assert!(
        relative < 1e-12,
        "relative error {relative} between {} and {want}",
        d.total_kg()
    );
}

// ═════════════════════════ gates added by the review of this PR

#[test]
fn the_planted_acre_headline_uses_the_surveyed_treated_share() {
    // ⛔⛔ The survey's rate is per TREATED acre, so dividing hydrogen by it
    // yields treated acres. An earlier version of this crate documented that
    // distinction in three places and then reported the treated figure while
    // calling it "corn acres".
    let delivered = delivered_series();
    let annual: f64 = delivered.iter().fold(0.0, |a, b| a + b);
    let per_acre = hydrogen_kg_per_acre(CORN_NITROGEN_LB_PER_ACRE.value()).expect("kg");
    let treated = annual / per_acre;
    assert!(
        ACRES_FERTILIZED_PER_TURBINE_YEAR.admits(treated),
        "treated = {treated}"
    );

    let share = record("CORN", "ALL CLASSES", ALL_PRACTICES, CORN_REFERENCE_YEAR)
        .expect("record")
        .pct_of_area_planted
        .and_then(Cell::published)
        .expect("a published treated share");
    let planted = planted_acres(treated, share.value()).expect("planted");
    assert!(
        ACRES_PLANTED_SERVED_PER_TURBINE_YEAR.admits(planted),
        "planted = {planted} against the pinned {}",
        ACRES_PLANTED_SERVED_PER_TURBINE_YEAR.value()
    );
    assert!(planted > treated, "a farm plants more than it treats");

    // And the gap is the survey's, not a constant: 1990's 80% share is 25%.
    let early = record("CORN", "ALL CLASSES", ALL_PRACTICES, 1990)
        .expect("1990")
        .pct_of_area_planted
        .and_then(Cell::published)
        .expect("1990 share");
    let then = planted_acres(treated, early.value()).expect("1990 planted");
    let gap = (then / treated - 1.0) * 100.0;
    assert!(close(gap, 25.0, 0.1), "at 1990's share the gap is {gap}%");
}

#[test]
fn the_planted_conversion_refuses_what_it_cannot_answer() {
    assert!(
        planted_acres(100.0, 100.0).is_some(),
        "a fully treated crop is the limit"
    );
    assert!(close(
        planted_acres(100.0, 100.0).expect("full"),
        100.0,
        1e-12
    ));
    for bad in [0.0, -1.0, 100.1, f64::NAN, f64::INFINITY] {
        assert!(
            planted_acres(100.0, bad).is_none(),
            "{bad} is not a percentage"
        );
    }
    for bad in [-1.0, f64::NAN, f64::INFINITY] {
        assert!(planted_acres(bad, 99.0).is_none(), "{bad} acres");
    }
}

#[test]
fn the_absence_claim_names_a_collection_this_crate_does_not_hold() {
    // ⛔⛔ The window decision rests on "no fertilizer-timing series", checked
    // against 106,596 rows that are NOT committed anywhere. The claim is
    // reproducible, not verifiable in place, and every surface must say so.
    const SURFACES: [(&str, &str); 2] = [
        ("src/lib.rs", include_str!("../src/lib.rs")),
        ("NASS_FERTILIZER.md", include_str!("../NASS_FERTILIZER.md")),
    ];
    for (name, src) in SURFACES {
        for (n, _) in src.match_indices("106,596") {
            let window = &src[n..(n + 700).min(src.len())];
            // ⚠ Requires the EXPLICIT caveat, not merely the word
            // "intermediate". Mutation testing showed an either-or accepted
            // text with the caveat deleted, because the weaker word survived.
            assert!(
                window.to_lowercase().contains("not committed"),
                "{name}: a 106,596-row absence claim must say the collection is NOT COMMITTED"
            );
        }
    }
    // And the committed collections really are smaller, so the caveat is not idle.
    assert_eq!(observations().len(), 356, "this crate commits 356 rows");
    assert!(
        observations().len() < 106_596,
        "the committed extract is not the collection the absence was checked against"
    );
}
