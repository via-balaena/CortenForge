//! Gates for stage 6 — the ceiling model.
//!
//! Pre-registered from three rosters, previous defects first:
//!
//! 1. **The classes PR #945 produced** — a rate whose denominator is mislabelled,
//!    an absence resting on an uncommitted collection, an unattributed "widely
//!    quoted", a figure living only in a test, an **invented corroboration**,
//!    and gates that grep for a sentence instead of the claim.
//! 2. **This crate's own new surfaces** — two committed extracts, a source with
//!    LAYERED terms (federal tool, state data), the lumped drawbar term, and a
//!    binding ceiling that changes hands.
//! 3. **Standing invariants** — absence claims assert their collection, every
//!    constant has a gate, every gate was made to fail, and the crate does not
//!    overclaim its scope.

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

use cf_acres::{
    ARS_TRACTOR_POWER_WORKSHEET, BINDING_CEILING_FLIPS_AT_HOURS_PER_DAY, CHISEL_TSV_SHA256,
    Ceiling, Ceilings, ENERGY_NEVER_BINDS, LOAM_SHARE_PERCENT, NOMINAL_HOURS_PER_DAY,
    NRCS_SOIL_DATA_ACCESS, REFERENCE_PLOW, TEXTURE_TSV_SHA256,
    TIME_AND_NITROGEN_CONVERGE_WITHIN_PERCENT, UNMEASURED_HERE, chisel_plows, dominant_texture,
    drawbar_kwh_per_acre, energy_ceiling, nitrogen_ceiling, parse_chisel_line, parse_texture_line,
    reference_plow, surface_textures, surveyed_acres, time_ceiling, unparsed_lines,
};
use cf_electrolysis::{Electrolyser, FixedSpecificEnergy, current_distributed};
use cf_nitrogen::CORN_NITROGEN_LB_PER_ACRE;
use cf_storage::{debit_compression, produced_during_demand};
use cf_tillage::{
    FallYear, HYDROGEN_LHV_KWH_PER_KG, NOMINAL_RULE, NOMINAL_YEAR, TillageDemand, john_deere_8245r,
    load_factor_band,
};
use cf_wind::{Air, EWT_DW54X, FOSTER_COUNTY_ND_2012 as YEAR, Machine};

const AIR: Air = Air {
    density_kg_m3: 1.16,
};

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

/// Stages 1–3: delivered kilograms at 350 bar, recomputed on the real series.
fn delivered_series() -> Vec<f64> {
    let plant = FixedSpecificEnergy::from_case(&current_distributed(), 1.0e6, 0.10);
    let floor = plant.rated_power_w() * plant.min_load_fraction();
    let dt = YEAR.interval_seconds();
    let tank = cf_storage::Tank::new(350.0, cf_storage::CARRINGTON_FALL.mean_k);
    let work = tank
        .compression_kwh_per_kg(20.68, &cf_storage::Covolume, 0.52)
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

/// `(annual kg, in-window kg, operable days)` — every one recomputed.
fn chain() -> (f64, f64, f64) {
    let delivered = delivered_series();
    let annual = delivered.iter().fold(0.0, |a, b| a + b);
    let w = FallYear::get(NOMINAL_YEAR)
        .expect("2012")
        .window(NOMINAL_RULE)
        .expect("window");
    let probe = TillageDemand::new(
        NOMINAL_YEAR,
        &w,
        1.0,
        delivered.len(),
        YEAR.interval_seconds(),
    )
    .expect("a probe demand");
    (
        annual,
        produced_during_demand(&delivered, &probe),
        w.operable_days,
    )
}

/// The three ceilings at a stated operating point.
fn ceilings_at(hours_per_day: f64, load_factor: f64, engine: f64) -> Ceilings {
    let (annual, in_window, days) = chain();
    let t = john_deere_8245r().expect("tractor");
    let plow = reference_plow().expect("the reference row");
    let cap = plow.field_capacity_acres_per_hr;
    let drivetrain = t.drivetrain_from_power();
    let kwh = drawbar_kwh_per_acre(t.pto_kw, load_factor, drivetrain, cap).expect("kWh/acre");
    Ceilings {
        time_acres: time_ceiling(days, hours_per_day, cap).expect("time"),
        energy_acres: energy_ceiling(
            in_window,
            HYDROGEN_LHV_KWH_PER_KG.value(),
            engine,
            drivetrain,
            kwh,
        )
        .expect("energy"),
        nitrogen_acres: nitrogen_ceiling(annual, CORN_NITROGEN_LB_PER_ACRE.value())
            .expect("nitrogen"),
    }
}

// ══════════════════════════════════ roster 2: the two committed extracts

#[test]
fn both_extracts_parse_completely() {
    assert_eq!(unparsed_lines(), 0, "every data line must parse");
    assert_eq!(surface_textures().len(), 8, "texture classes");
    assert_eq!(chisel_plows().len(), 8, "chisel plow rows");
    assert!(
        surface_textures()
            .iter()
            .all(|t| t.acres.is_finite() && t.acres > 0.0)
    );
    assert!(chisel_plows().iter().all(|p| p.width_ft > 0.0));
}

#[test]
fn both_committed_extracts_match_their_digests() {
    use sha2::{Digest, Sha256};
    use std::fmt::Write as _;
    for (name, bytes, pinned) in [
        (
            "nd031_surface_texture.tsv",
            include_bytes!("../data/nd031_surface_texture.tsv").as_slice(),
            TEXTURE_TSV_SHA256,
        ),
        (
            "ars_chisel_plow.tsv",
            include_bytes!("../data/ars_chisel_plow.tsv").as_slice(),
            CHISEL_TSV_SHA256,
        ),
    ] {
        let mut hex = String::with_capacity(64);
        for b in Sha256::digest(bytes) {
            write!(hex, "{b:02x}").expect("writing to a String cannot fail");
        }
        assert_eq!(hex, pinned, "{name} no longer matches its digest");
        assert_eq!(pinned.len(), 64);
        assert!(pinned.chars().all(|c| c.is_ascii_hexdigit()));
    }
}

#[test]
fn both_parsers_reject_every_way_a_line_can_be_wrong() {
    assert!(parse_texture_line("Loam\t233889.00").is_some());
    for bad in [
        "",
        "Loam",
        "Loam\tnope",
        "\t100",
        "Loam\t100\textra",
        "Loam\t-1",
    ] {
        assert!(parse_texture_line(bad).is_none(), "texture {bad:?}");
    }
    assert!(parse_chisel_line("Chisel Plow Folding 24'\t24\tMFWD\t0.076\t13.16").is_some());
    for bad in [
        "",
        "Chisel",
        "Chisel\tnope\tMFWD\t0.076\t13.16",
        "Chisel\t24\tMFWD\t0.076",
        "\t24\tMFWD\t0.076\t13.16",
        "Chisel\t24\t\t0.076\t13.16",
        "Chisel\t24\tMFWD\t0.076\t13.16\textra",
    ] {
        assert!(parse_chisel_line(bad).is_none(), "chisel {bad:?}");
    }
}

// ════════════════════════════ roster 2: the worksheet's own reciprocal checksum

#[test]
fn the_worksheet_reconciles_with_itself() {
    // ★★ It prints hours-per-acre AND acres-per-hour, so a mistranscribed rate
    // stops being the reciprocal of its neighbour.
    assert!(!chisel_plows().is_empty(), "the collection is populated");
    for p in chisel_plows() {
        assert!(
            p.reciprocal_holds(),
            "{}: {} x {} is not 1",
            p.equipment,
            p.perf_rate_hr_per_acre,
            p.field_capacity_acres_per_hr
        );
    }
}

#[test]
fn a_mistyped_rate_breaks_the_reciprocal() {
    // The gate above made to fail.
    let broken =
        parse_chisel_line("Chisel Plow Folding 24'\t24\tMFWD\t0.076\t99.9").expect("it parses");
    assert!(!broken.reciprocal_holds(), "99.9 is not 1/0.076");
    let zero = parse_chisel_line("Chisel Plow Folding 24'\t24\tMFWD\t0\t13.16").expect("parses");
    assert!(!zero.reciprocal_holds(), "a zero rate has no reciprocal");
}

#[test]
fn the_anchor_implies_a_speed_the_tractor_could_actually_drive() {
    // ⚠ A DERIVED sanity check, not a figure the worksheet prints. If the rate
    // were wrong by a factor, the implied speed would be absurd.
    let plow = reference_plow().expect("reference row");
    let t = john_deere_8245r().expect("tractor");
    let ideal = plow.implied_speed_mph(1.0).expect("speed");
    assert!(
        close(ideal, 4.52, 0.01),
        "at 100% field efficiency, {ideal} mph"
    );
    // Nebraska MEASURED 4.85 mph at 75% drawbar load. A real chisel plough runs
    // at 70-90% field efficiency, which brackets that.
    let slow = plow.implied_speed_mph(0.90).expect("speed");
    let fast = plow.implied_speed_mph(0.70).expect("speed");
    assert!(slow < 5.1 && fast > 6.0, "{slow} to {fast} mph");
    assert!(
        (4.0..7.0).contains(&ideal) && (4.0..7.0).contains(&fast),
        "outside anything a chisel plough is driven at"
    );
    let _ = t;
    for bad in [0.0, -1.0, 1.01, f64::NAN] {
        assert!(plow.implied_speed_mph(bad).is_none(), "{bad}");
    }
}

// ════════════════════════════════════ roster 2: the soil, and whether it says so

#[test]
fn the_soil_under_this_farm_is_loam() {
    // ⚠ Committed so a reader can judge whether a Mississippi-derived rate
    // transfers — not because this crate computes with it.
    let dom = dominant_texture().expect("a dominant class");
    assert_eq!(dom.class, "Loam");
    let pct = 100.0 * dom.acres / surveyed_acres();
    assert!(
        LOAM_SHARE_PERCENT.admits(pct),
        "loam is {pct}% against the pinned {}",
        LOAM_SHARE_PERCENT.value()
    );
    assert!(
        close(surveyed_acres(), 286_447.0, 1.0),
        "{} acres",
        surveyed_acres()
    );
    // And it is a real distribution, not one class.
    assert!(surface_textures().len() >= 5, "several classes are present");
    assert!(
        surface_textures().iter().any(|t| t.class.contains("clay")
            || t.class.contains("Clay")
            || t.class.contains("sand")
            || t.class.contains("Sand")),
        "including some that are not loam"
    );
}

// ═══════════════════════════ roster 1: the headline is recomputed, not quoted

#[test]
fn the_energy_ceiling_never_binds() {
    // ★★★ THE FINDING, and the sweep is the gate. Four of the chain's six
    // crates exist to measure this ceiling; having measured it, it is not the
    // constraint.
    let (lf_lo, lf_hi) = load_factor_band();
    let mut worst_slack = f64::MAX;
    let mut checked = 0;
    let mut energy_ever_bound = false;
    for hours in [10.0_f64, 11.0, 12.0, 14.0, 16.0] {
        for lf in [lf_lo, lf_hi] {
            for engine in [0.25_f64, 0.30, 0.40, 0.50] {
                let c = ceilings_at(hours, lf, engine);
                checked += 1;
                if c.binding() == Ceiling::Energy {
                    energy_ever_bound = true;
                }
                let margin = c.energy_acres / c.acres();
                worst_slack = worst_slack.min(margin);
            }
        }
    }
    assert_eq!(checked, 40, "the sweep is not empty");
    // ⚠ Compared against the SWEEP, not asserted on its own. `assert!(CONST)`
    // is a constant assertion — it cannot fail, and a gate that cannot fail is
    // not a gate.
    assert_eq!(
        !energy_ever_bound, ENERGY_NEVER_BINDS,
        "the constant must say what the sweep found"
    );
    let (want_lo, want_hi) = cf_acres::ENERGY_SLACK_RANGE;
    assert!(
        close(worst_slack, want_lo, 0.01),
        "tightest slack {worst_slack}x against the pinned {want_lo}x"
    );
    assert!(
        worst_slack > 1.0,
        "and 'never binds' means the slack never reaches 1"
    );
    let best = {
        let mut m = f64::MIN;
        for hours in [10.0_f64, 11.0, 12.0, 14.0, 16.0] {
            for lf in [lf_lo, lf_hi] {
                for engine in [0.25_f64, 0.30, 0.40, 0.50] {
                    let c = ceilings_at(hours, lf, engine);
                    m = m.max(c.energy_acres / c.acres());
                }
            }
        }
        m
    };
    assert!(
        close(best, want_hi, 0.01),
        "loosest slack {best}x against {want_hi}x"
    );
}

#[test]
fn the_binding_ceiling_flips_with_the_working_day() {
    // ★★★ Below the crossover the WINDOW binds; above it NITROGEN does.
    let flip = BINDING_CEILING_FLIPS_AT_HOURS_PER_DAY.value();
    let (lf_lo, _) = load_factor_band();
    let below = ceilings_at(flip - 1.0, lf_lo, 0.40);
    let above = ceilings_at(flip + 1.0, lf_lo, 0.40);
    assert_eq!(below.binding(), Ceiling::Time, "an hour short: {below:?}");
    assert_eq!(
        above.binding(),
        Ceiling::Nitrogen,
        "an hour long: {above:?}"
    );

    // And the crossover really is where the two cross.
    let at = ceilings_at(flip, lf_lo, 0.40);
    let gap = (at.time_acres / at.nitrogen_acres - 1.0).abs() * 100.0;
    assert!(
        gap < 0.2,
        "at the pinned crossover the two differ by {gap}%"
    );

    // ⚠ It does NOT depend on the load factor or the engine: neither enters
    // the time or nitrogen ceiling. If that ever changes, this fails.
    for lf in [lf_lo, load_factor_band().1] {
        for engine in [0.25_f64, 0.50] {
            let c = ceilings_at(flip + 1.0, lf, engine);
            assert_eq!(c.binding(), Ceiling::Nitrogen, "lf {lf}, engine {engine}");
        }
    }
}

#[test]
fn the_time_and_nitrogen_ceilings_converge() {
    // ★★ Two ceilings from different crates, different federal surveys, no
    // shared input.
    let (lf_lo, _) = load_factor_band();
    let c = ceilings_at(NOMINAL_HOURS_PER_DAY, lf_lo, 0.40);
    let gap = (c.time_acres / c.nitrogen_acres - 1.0).abs() * 100.0;
    assert!(
        TIME_AND_NITROGEN_CONVERGE_WITHIN_PERCENT.admits(gap),
        "they differ by {gap}% against the pinned {}",
        TIME_AND_NITROGEN_CONVERGE_WITHIN_PERCENT.value()
    );
    assert!(
        (4_000.0..5_000.0).contains(&c.time_acres)
            && (4_000.0..5_000.0).contains(&c.nitrogen_acres),
        "both land near 4,300-4,700 acres: {c:?}"
    );
    assert_eq!(
        c.binding(),
        Ceiling::Nitrogen,
        "at the nominal day nitrogen binds"
    );
    assert!(
        close(c.acres(), c.nitrogen_acres, 0.0),
        "and acres() takes the smallest"
    );
}

#[test]
fn the_lumped_drawbar_term_is_what_the_measurements_imply() {
    // ⛔ Draft, depth, speed, slip and field efficiency all live in here,
    // because this chain can measure none of them separately.
    let t = john_deere_8245r().expect("tractor");
    let plow = reference_plow().expect("plow");
    let (lo, hi) = load_factor_band();
    let at = |lf: f64| {
        drawbar_kwh_per_acre(
            t.pto_kw,
            lf,
            t.drivetrain_from_power(),
            plow.field_capacity_acres_per_hr,
        )
        .expect("kWh/acre")
    };
    assert!(close(at(lo), 6.64, 0.02), "at lf {lo}: {} kWh/acre", at(lo));
    assert!(close(at(hi), 8.78, 0.02), "at lf {hi}: {} kWh/acre", at(hi));
    assert!(at(lo) < at(hi), "a harder duty cycle spends more per acre");
    // It is proportional to the load factor and inverse in the rate — the two
    // relationships that make it a lumped OBSERVATION rather than a model.
    assert!(close(at(hi) / at(lo), hi / lo, 1e-9));
    let half_rate = drawbar_kwh_per_acre(
        t.pto_kw,
        lo,
        t.drivetrain_from_power(),
        plow.field_capacity_acres_per_hr / 2.0,
    )
    .expect("kWh/acre");
    assert!(
        close(half_rate, at(lo) * 2.0, 1e-9),
        "halving the rate doubles the energy"
    );
}

// ══════════════════════════════════════════════════════ refusals and boundaries

#[test]
fn every_ceiling_refuses_what_it_cannot_answer() {
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(time_ceiling(bad, 12.0, 13.0).is_none(), "days {bad}");
        assert!(time_ceiling(29.9, bad, 13.0).is_none(), "hours {bad}");
        assert!(time_ceiling(29.9, 12.0, bad).is_none(), "rate {bad}");
        assert!(nitrogen_ceiling(bad, 118.0).is_none(), "kg {bad}");
        assert!(
            energy_ceiling(bad, 33.3, 0.4, 0.92, 7.0).is_none(),
            "available {bad}"
        );
        assert!(
            energy_ceiling(7000.0, bad, 0.4, 0.92, 7.0).is_none(),
            "lhv {bad}"
        );
        assert!(
            energy_ceiling(7000.0, 33.3, 0.4, 0.92, bad).is_none(),
            "kWh/acre {bad}"
        );
        assert!(
            drawbar_kwh_per_acre(bad, 0.6, 0.92, 13.0).is_none(),
            "pto {bad}"
        );
    }
    // Fractions must be fractions.
    for bad in [0.0, -0.1, 1.01, f64::NAN] {
        assert!(
            energy_ceiling(7000.0, 33.3, bad, 0.92, 7.0).is_none(),
            "engine {bad}"
        );
        assert!(
            energy_ceiling(7000.0, 33.3, 0.4, bad, 7.0).is_none(),
            "drivetrain {bad}"
        );
        assert!(
            drawbar_kwh_per_acre(160.0, bad, 0.92, 13.0).is_none(),
            "lf {bad}"
        );
    }
    assert!(
        energy_ceiling(7000.0, 33.3, 1.0, 1.0, 7.0).is_some(),
        "exactly 1.0 is the limit and is allowed"
    );
    assert!(nitrogen_ceiling(7000.0, -1.0).is_none(), "a negative rate");
}

#[test]
fn the_ceilings_report_which_one_binds_and_what_sets_it() {
    let c = Ceilings {
        time_acres: 100.0,
        energy_acres: 200.0,
        nitrogen_acres: 300.0,
    };
    assert_eq!(c.binding(), Ceiling::Time);
    assert!(close(c.acres(), 100.0, 0.0));
    assert!(close(c.slack(), 3.0, 1e-12));
    let c = Ceilings {
        time_acres: 300.0,
        energy_acres: 100.0,
        nitrogen_acres: 200.0,
    };
    assert_eq!(c.binding(), Ceiling::Energy);
    let c = Ceilings {
        time_acres: 300.0,
        energy_acres: 200.0,
        nitrogen_acres: 100.0,
    };
    assert_eq!(c.binding(), Ceiling::Nitrogen);
    for ceiling in [Ceiling::Time, Ceiling::Energy, Ceiling::Nitrogen] {
        assert!(
            ceiling.what_sets_it().len() > 20,
            "a ceiling that cannot say what sets it produces orphan numbers"
        );
    }
}

// ═══════════════════════════════════════════════════ terms, scope and unknowns

#[test]
fn the_worksheets_terms_are_layered_and_say_so() {
    // ⛔⛔ A federal tool over a STATE source. The federal wrapper does not
    // launder the provenance, and this row must not read as a plain §105 work.
    assert!(
        ARS_TRACTOR_POWER_WORKSHEET.terms.contains("NOT DETERMINED"),
        "the state half must be recorded as undetermined"
    );
    assert!(
        ARS_TRACTOR_POWER_WORKSHEET
            .terms
            .contains("Mississippi State"),
        "and must name where the data actually came from"
    );
    assert!(
        !ARS_TRACTOR_POWER_WORKSHEET
            .terms
            .starts_with("U.S. Government work,"),
        "⛔ it must NOT read as a plain federal work"
    );
    // The soil source, by contrast, is federal through and through.
    assert!(NRCS_SOIL_DATA_ACCESS.terms.contains("105"));
    for s in [ARS_TRACTOR_POWER_WORKSHEET, NRCS_SOIL_DATA_ACCESS] {
        assert_eq!(s.retrieved.len(), 10, "{}: an ISO date", s.document);
        assert!(!s.url.is_empty() && !s.document.is_empty());
    }
}

#[test]
fn the_crate_says_what_it_does_not_model() {
    // ⛔ A ceiling model presented as a soil model is the lie this arc's
    // pre-registered traps warn about. Checks the CLAIM: the docs must name
    // draft, slip and the surface as unmeasured, and the reference plow's
    // provenance must be stated wherever the rate is anchored.
    const LIB: &str = include_str!("../src/lib.rs");
    assert!(
        LIB.contains("It is a ceiling model. It is not a soil model."),
        "the scope must be stated outright"
    );
    for term in ["draft", "slip", "surface"] {
        assert!(
            UNMEASURED_HERE.iter().any(|u| u.what.contains(term)),
            "{term} must be recorded as unmeasured"
        );
    }
    assert!(
        UNMEASURED_HERE
            .iter()
            .any(|u| u.what.contains("transfers to North Dakota")),
        "and so must the anchor's region"
    );
    assert!(
        LIB.contains("Mississippi State University 2026 Crop Planning Budgets"),
        "the anchor's provenance must be quoted where it is used"
    );
    assert_eq!(REFERENCE_PLOW, "Chisel Plow Folding 24'");
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
            .any(|u| u.why_unmeasured.contains("ANSI")),
        "the paywall is named, not hinted at"
    );
}

#[test]
fn a_degenerate_row_yields_no_speed_and_no_slack() {
    // ⛔ Both guards exercised. A worksheet row with no width, or a set of
    // ceilings that bind at zero, must refuse rather than return a number that
    // looks like an answer.
    let zero_width = parse_chisel_line("Bad\t0\tMFWD\t0.076\t13.16").expect("it parses");
    assert!(
        zero_width.implied_speed_mph(1.0).is_none(),
        "a zero-width implement has no ground speed"
    );
    let zero_cap = parse_chisel_line("Bad\t24\tMFWD\t0.076\t0").expect("it parses");
    assert!(zero_cap.implied_speed_mph(1.0).is_none(), "nor a zero rate");
    // And the real row does yield one, so the guard is not swallowing everything.
    assert!(
        reference_plow()
            .expect("plow")
            .implied_speed_mph(1.0)
            .is_some()
    );

    let nowhere = Ceilings {
        time_acres: 0.0,
        energy_acres: 0.0,
        nitrogen_acres: 0.0,
    };
    assert!(
        nowhere.slack().is_nan(),
        "slack over a zero ceiling is not a number, and must not read as one"
    );
    let real = Ceilings {
        time_acres: 100.0,
        energy_acres: 200.0,
        nitrogen_acres: 300.0,
    };
    assert!(
        real.slack().is_finite(),
        "a real set of ceilings has finite slack"
    );
}
