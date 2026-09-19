//! Gates for stage 4 — the tillage window and what it demands.
//!
//! Pre-registered before the crate existed, from three external rosters, in the
//! order the previous stage's defects said to try them:
//!
//! 1. **The classes PR #943 actually produced** — a figure printed rather than
//!    pinned, a retrieval grid that did not contain the values its own field
//!    names claimed, a constant with no producer, and a comparison whose two
//!    sides shared a denominator.
//! 2. **EPA NR-005c's own arithmetic** — the bins are stated averages, so a
//!    mistranscribed cycle stops reconciling.
//! 3. **The NASS extract's structure** — row counts, completeness, and the two
//!    years where the window never opens at all.
//!
//! ★★★ Every gate here was made to FAIL before it was kept. One that could not
//! be made to fail was deleted rather than left in reading like a check.

#![allow(
    clippy::panic,
    reason = "assert! in a test is the failure mechanism, not an escape hatch"
)]
#![allow(
    clippy::cast_precision_loss,
    reason = "sample counts here are ~10^5 and exact in f64"
)]
#![allow(
    clippy::expect_used,
    reason = "a gate that cannot name what it could not build is not much of a gate"
)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "sample indices here are computed from dates the parser already validated"
)]

use cf_electrolysis::{Electrolyser, FixedSpecificEnergy, current_distributed};
use cf_storage::{Demand, IdealGas, debit_compression, produced_during_demand};
use cf_tillage::{
    AFTER_CORN, AFTER_SOYBEANS, AGRICULTURAL_TRACTOR_ANNUAL_HOURS, AGRICULTURAL_TRACTOR_APPLIED_LF,
    AGRICULTURAL_TRACTOR_MEASURED_LF, AVERAGED_BINS, BREAK_EVEN_CAVEATS, COMPLETE_YEAR_MIN_WEEKS,
    DIESEL_LHV_BTU_PER_GAL, DIESEL_PTO_THERMAL_EFFICIENCY_PERCENT,
    DRIVETRAIN_ROUTES_AGREE_WITHIN_PP, FIXED_OCT_NOV, FallYear, HIGH_LF_BIN,
    HYDROGEN_ENGINE_EFFICIENCY_NEEDED, HYDROGEN_LHV_KWH_PER_KG, LOAD_FACTOR_HAIRCUT_PERCENT,
    LOW_LF_BIN, NASS_CROP_PROGRESS, NEEDED_ROWS, NOMINAL_HOURS_PER_OPERABLE_DAY, NOMINAL_RULE,
    NOMINAL_YEAR, SEASON_MARGIN_AT_FORTY_PERCENT, SWEPT_RULES, Season, Series, TillageDemand,
    UNMEASURED_HERE, WINDOW_RULE_CANCELS_WITHIN_PP, WINDOW_RULE_REVERSES_THE_RANKING, WindowRule,
    break_even_engine_efficiency, btu_per_hp_hr, climatology, day_of_year, diesel_lhv_kwh_per_l,
    john_deere_8245r, load_factor_band, observations, season_diesel_litres, season_hydrogen_kg,
    unparsed_lines, years,
};
use cf_wind::{Air, EWT_DW54X, FOSTER_COUNTY_ND_2012 as YEAR, Machine};

/// Air at the site's 484 m elevation, matching every earlier stage's tests.
const AIR: Air = Air {
    density_kg_m3: 1.16,
};
/// The plant the chain's figures are pinned at: 1 MW, 10% turndown.
const RATED_W: f64 = 1.0e6;
const TURNDOWN: f64 = 0.10;
/// The electrolyser outlet, bar.
const OUTLET_BAR: f64 = 20.68;
/// A tractor tank's working pressure, bar.
const TANK_BAR: f64 = 350.0;
/// The record's own station efficiency to 440 bar, as a fraction.
const STATION_EFFICIENCY: f64 = 0.52;

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

/// Stage 1 into stage 2: the kilogram series at the electrolyser outlet.
///
/// The same reconstruction `cf-storage`'s own tests use, so any drift in stages
/// 1 or 2 reddens this crate rather than quietly moving its headline.
fn hydrogen_series() -> Vec<f64> {
    let plant = FixedSpecificEnergy::from_case(&current_distributed(), RATED_W, TURNDOWN);
    let floor = plant.rated_power_w() * plant.min_load_fraction();
    let dt = YEAR.interval_seconds();
    YEAR.speeds()
        .map(|v| EWT_DW54X.power_w(v, AIR))
        .map(|p| {
            if p < floor {
                return 0.0;
            }
            let used = p.min(plant.rated_power_w());
            let specific = plant.specific_energy_kwh_per_kg(used / plant.rated_power_w());
            if specific > 0.0 {
                used * dt / 3.6e6 / specific
            } else {
                0.0
            }
        })
        .collect()
}

/// Stage 1 into stage 2, before the compression debit.
///
/// ⛔ Kept separate from [`delivered_series`] because stage 3's published
/// figures are on THIS side of the debit and this crate's are on the other. A
/// review of this PR found the two silently mixed.
fn outlet_series() -> Vec<f64> {
    hydrogen_series()
}

/// Stage 3: the same series, compressed into a 350 bar tank.
fn delivered_series() -> Vec<f64> {
    let tank = cf_storage::Tank::new(TANK_BAR, cf_storage::CARRINGTON_FALL.mean_k);
    let work = tank
        .compression_kwh_per_kg(OUTLET_BAR, &cf_storage::Covolume, STATION_EFFICIENCY)
        .expect("compression work");
    let case = current_distributed();
    let specific = case.total_kwh_per_kg.value();
    hydrogen_series()
        .into_iter()
        .map(|kg| debit_compression(kg, specific, work).map_or(0.0, |d| d.kg))
        .collect()
}

/// The hydrogen produced *inside* a rule's own window, recomputed from stages
/// 1-3 on the real series.
///
/// ⛔⛔ Takes the rule, because the cliff is a function of the window. Stage 3
/// publishes 2,695 kg for its own 21-day illustration; using that figure for a
/// seven-week window compares demand from one window against production from
/// another, and an earlier version of these gates did exactly that.
fn in_window_kg(delivered: &[f64], rule: WindowRule) -> (f64, f64) {
    let w = FallYear::get(NOMINAL_YEAR)
        .expect("2012")
        .window(rule)
        .expect("the rule opens in 2012");
    let probe = TillageDemand::new(
        NOMINAL_YEAR,
        &w,
        1.0,
        delivered.len(),
        YEAR.interval_seconds(),
    )
    .expect("a probe demand");
    (w.operable_days, produced_during_demand(delivered, &probe))
}

/// The nominal season for `year` under `rule` at `load_factor`.
fn nominal_season(year: u16, rule: WindowRule, load_factor: f64) -> Season {
    let w = FallYear::get(year)
        .expect("the year is in the extract")
        .window(rule)
        .expect("the rule opens in this year");
    Season {
        operable_days: w.operable_days,
        hours_per_operable_day: NOMINAL_HOURS_PER_OPERABLE_DAY,
        load_factor,
    }
}

// ══════════════════════════════════════════════ roster 3: the committed extract

#[test]
fn the_extract_parses_completely() {
    // ⛔⛔ EMPTY is not evidence. Assert the collection, not the absence: a
    // parser that dropped every row would also report zero failures.
    assert_eq!(unparsed_lines(), 0, "every data line must parse");
    assert_eq!(observations().len(), 921, "the committed row count");
    assert!(
        observations().iter().all(|o| o.value.is_finite()),
        "no non-finite value survives the parser"
    );
}

#[test]
fn the_committed_extract_matches_its_digest() {
    // ⛔ Without this the digest is a 64-character string nothing produces —
    // the defect class #943 shipped four times. Mutating the constant must
    // redden a gate, not merely change a doc comment.
    use sha2::{Digest, Sha256};
    use std::fmt::Write as _;
    let bytes = include_bytes!("../data/nd_fall_fieldwork.tsv");
    let digest = Sha256::digest(bytes);
    let mut hex = String::with_capacity(64);
    for b in digest {
        write!(hex, "{b:02x}").expect("writing to a String cannot fail");
    }
    assert_eq!(
        hex,
        cf_tillage::NASS_TSV_SHA256,
        "the committed extract no longer matches its recorded digest"
    );
    assert_eq!(cf_tillage::NASS_TSV_SHA256.len(), 64);
    assert!(
        cf_tillage::NASS_TSV_SHA256
            .chars()
            .all(|c| c.is_ascii_hexdigit()),
        "and it is a hex digest"
    );
}

#[test]
fn the_harvest_threshold_is_inclusive() {
    // The `>=` in `harvest_reached`, made observable. Three weeks in the
    // extract sit EXACTLY on 90%, and only an inclusive comparison opens the
    // window on them. Without this gate `>=` and `>` are indistinguishable.
    let fy = FallYear::get(2010).expect("2010 is in the extract");
    let exact = fy
        .series(Series::Soybeans)
        .into_iter()
        .find(|o| close(o.value, 90.0, 0.0))
        .expect("2010 has a week printed at exactly 90%");
    assert_eq!(
        fy.harvest_reached(Series::Soybeans, 90.0),
        Some(exact.week_ending),
        "a week printed at exactly the threshold must open the window"
    );
    // And the boundary is the only thing separating the two comparisons.
    let strictly_after = fy
        .series(Series::Soybeans)
        .into_iter()
        .find(|o| o.value > 90.0)
        .expect("a later week exceeds 90%");
    assert!(
        strictly_after.week_ending > exact.week_ending,
        "an exclusive comparison would have opened at {} instead of {}",
        strictly_after.week_ending,
        exact.week_ending
    );
}

#[test]
fn the_extract_carries_all_three_series() {
    for series in [Series::DaysSuitable, Series::Soybeans, Series::Corn] {
        let n = observations().iter().filter(|o| o.series == series).count();
        assert!(n > 200, "{} has only {n} observations", series.nass_name());
    }
}

#[test]
fn the_extract_is_in_week_order() {
    let rows = observations();
    for pair in rows.windows(2) {
        assert!(
            pair[0].week_ending <= pair[1].week_ending,
            "{} then {}",
            pair[0].week_ending,
            pair[1].week_ending
        );
    }
}

#[test]
fn every_week_ending_is_a_real_date() {
    for o in observations() {
        let doy =
            day_of_year(o.week_ending).unwrap_or_else(|| panic!("{} is not a date", o.week_ending));
        assert!((1..=366).contains(&doy), "{} -> {doy}", o.week_ending);
    }
}

#[test]
fn the_date_parser_rejects_what_is_not_a_date() {
    // Made to fail first: without the month-length table, "2013-02-30" parsed.
    for bad in [
        "2013-02-30",
        "2013-13-01",
        "2013-00-10",
        "2013-01-00",
        "not-a-date",
        "2013-1-1-1",
        "",
        "2013-01",
    ] {
        assert!(day_of_year(bad).is_none(), "{bad} should not parse");
    }
    assert_eq!(day_of_year("2012-02-29"), Some(60), "2012 is a leap year");
    assert_eq!(day_of_year("2013-03-01"), Some(60), "2013 is not");
    assert_eq!(day_of_year("2012-12-31"), Some(366));
}

#[test]
fn the_series_codes_round_trip() {
    for series in [Series::DaysSuitable, Series::Soybeans, Series::Corn] {
        assert_eq!(Series::from_code(series.code()), Some(series));
        assert!(
            series.nass_name().len() > series.code().len(),
            "the full NASS name is what says what the rate is a rate of"
        );
    }
    assert_eq!(Series::from_code("TILLAGE"), None);
    assert_eq!(Series::from_code(""), None);
}

#[test]
fn no_fall_tillage_series_exists_and_that_is_recorded() {
    // ⛔ The absence that shaped the crate, asserted against a NON-EMPTY
    // collection so it cannot pass by the extract being broken.
    assert!(observations().len() > 900, "the collection is populated");
    assert!(
        Series::from_code("TILLAGE").is_none(),
        "NASS publishes no tillage-progress series; days suitable replaces it"
    );
    assert!(
        UNMEASURED_HERE
            .iter()
            .any(|u| u.what.contains("fieldwork days")),
        "and the state-level limit that follows from it is recorded"
    );
}

// ═══════════════════════════════════════════════════ roster 3: the window rules

#[test]
fn an_incomplete_year_is_rejected_not_averaged() {
    // ⛔⛔ 2025 publishes fieldwork into late November but is missing every week
    // from 28 September to 16 November. Summed, it reads as a catastrophic
    // season rather than an absent one.
    let y2025 = FallYear::get(2025).expect("2025 is in the extract");
    assert!(!y2025.is_complete(), "2025 must not be treated as a season");
    assert!(
        y2025.weeks_in_completeness_span() < COMPLETE_YEAR_MIN_WEEKS,
        "and the reason is the week count, not a special case"
    );
    let clim = climatology(FIXED_OCT_NOV).expect("a climatology");
    assert!(
        clim.rejected_incomplete.contains(&2025),
        "the rejection is reported, not silent"
    );
    assert!(
        !clim.years.contains(&2025),
        "and 2025 contributes to no statistic"
    );
}

#[test]
fn a_complete_year_is_not_rejected() {
    // The other side of the same gate: without it, a too-strict threshold would
    // reject everything and every statistic would still "pass".
    let y2012 = FallYear::get(NOMINAL_YEAR).expect("2012");
    assert!(y2012.is_complete());
    assert!(y2012.weeks_in_completeness_span() >= COMPLETE_YEAR_MIN_WEEKS);
    let clim = climatology(FIXED_OCT_NOV).expect("a climatology");
    assert_eq!(clim.years.len(), 19, "nineteen complete years");
    assert!(clim.years.contains(&NOMINAL_YEAR));
}

#[test]
fn an_empty_window_is_positive_zero() {
    // ⚠ `sum::<f64>()` over an empty iterator returns NEGATIVE zero in Rust,
    // which prints as `-0.00` and sorts below `+0.0` under `total_cmp`. Two
    // real years reach here with no weeks at all.
    let mut empties = 0;
    for y in years() {
        let Some(fy) = FallYear::get(y) else { continue };
        let Some(w) = fy.window(AFTER_CORN) else {
            continue;
        };
        if w.weeks_counted == 0 {
            empties += 1;
            assert!(
                !w.operable_days.is_sign_negative(),
                "{y}: an empty window reported negative zero"
            );
            assert!(close(w.operable_days, 0.0, 0.0), "{y}: and it is zero");
        }
    }
    assert!(
        empties >= 2,
        "two years really do have an empty corn window"
    );
}

#[test]
fn a_rule_that_never_opens_says_so_rather_than_returning_zero() {
    // ⛔ Corn does not reach 90% statewide in every North Dakota year. That is
    // an outcome, not an error, and it must not be reported as "no days".
    let never = climatology(AFTER_CORN)
        .expect("corn opens in some years")
        .rejected_never_opened;
    assert!(
        never.len() >= 6,
        "corn never clears 90% in at least six complete years, got {never:?}"
    );
    for y in &never {
        let fy = FallYear::get(*y).expect("the year exists");
        assert!(fy.window(AFTER_CORN).is_none(), "{y} must return None");
        assert!(
            fy.window(AFTER_SOYBEANS).is_some(),
            "{y}: but soybeans do clear, so this is about corn and not about the year"
        );
    }
}

#[test]
fn a_harvest_gated_window_opens_after_the_threshold_week() {
    let fy = FallYear::get(NOMINAL_YEAR).expect("2012");
    let opens = fy
        .harvest_reached(Series::Soybeans, 90.0)
        .expect("soybeans clear 90% in 2012");
    let w = fy.window(AFTER_SOYBEANS).expect("a window");
    assert_eq!(w.opens_after, Some(opens));
    let before = fy
        .series(Series::Soybeans)
        .into_iter()
        .filter(|o| o.week_ending < opens)
        .all(|o| o.value < 90.0);
    assert!(before, "no earlier week had already reached the threshold");
}

#[test]
fn a_higher_threshold_never_opens_the_window_earlier() {
    // Monotonicity: the gate that would catch a comparison written backwards.
    for y in years() {
        let Some(fy) = FallYear::get(y) else { continue };
        let early = fy.harvest_reached(Series::Soybeans, 50.0);
        let late = fy.harvest_reached(Series::Soybeans, 90.0);
        if let (Some(e), Some(l)) = (early, late) {
            assert!(e <= l, "{y}: 50% cleared at {e}, 90% at {l}");
        }
    }
}

#[test]
fn the_fixed_calendar_rule_ignores_the_harvest_entirely() {
    let fy = FallYear::get(NOMINAL_YEAR).expect("2012");
    let w = fy.window(FIXED_OCT_NOV).expect("a window");
    assert_eq!(
        w.opens_after, None,
        "a fixed calendar has no opening week - that is the whole point of it"
    );
    assert!(w.weeks_counted >= COMPLETE_YEAR_MIN_WEEKS);
}

#[test]
fn every_rule_says_what_it_is() {
    for rule in SWEPT_RULES {
        let name = rule.name();
        assert!(
            !name.is_empty(),
            "a rule with no name produces orphan numbers"
        );
        assert!(
            name.contains("calendar") || name.contains("harvested"),
            "the name must say which kind: {name}"
        );
    }
}

// ═════════════════════════════ roster 1: the class #943 produced — a comparison
//                               whose two sides do not hold the same thing fixed

#[test]
fn the_window_rule_reverses_the_ranking() {
    // ★★★ The finding. Same year, same data, opposite conclusions — because a
    // fixed calendar holds the START DATE constant when the start date is what
    // varies most between years.
    let fixed = climatology(FIXED_OCT_NOV).expect("fixed");
    let gated = climatology(AFTER_SOYBEANS).expect("gated");
    let (want_fixed, want_gated) = WINDOW_RULE_REVERSES_THE_RANKING;
    assert_eq!(
        fixed.rank_of(NOMINAL_YEAR),
        Some(want_fixed),
        "2012 under a fixed calendar"
    );
    assert_eq!(
        gated.rank_of(NOMINAL_YEAR),
        Some(want_gated),
        "2012 once the window may open when harvest clears"
    );
    // And the reversal is real, not a tie read two ways.
    assert!(
        want_fixed.0 * 2 < want_fixed.1,
        "the fixed rank is in the bottom half"
    );
    assert!(
        want_gated.0 * 2 > want_gated.1,
        "the gated rank is in the top half"
    );
    assert_eq!(want_fixed.1, want_gated.1, "and both rank against 19 years");
}

#[test]
fn the_two_rules_disagree_about_the_year_not_about_the_data() {
    // ⚠ The mechanism, gated: 2012 has FEWER operable days than average on a
    // fixed calendar and MORE once the window opens with the harvest. Both
    // statements are about the same weekly numbers.
    let fixed = climatology(FIXED_OCT_NOV).expect("fixed");
    let gated = climatology(AFTER_SOYBEANS).expect("gated");
    let f = FallYear::get(NOMINAL_YEAR)
        .expect("2012")
        .window(FIXED_OCT_NOV)
        .expect("w")
        .operable_days;
    let g = FallYear::get(NOMINAL_YEAR)
        .expect("2012")
        .window(AFTER_SOYBEANS)
        .expect("w")
        .operable_days;
    assert!(
        f < fixed.mean_days,
        "{f} should be below {}",
        fixed.mean_days
    );
    assert!(
        g > gated.mean_days,
        "{g} should be above {}",
        gated.mean_days
    );
    assert!(
        g < f,
        "and the gated window is the SHORTER one in absolute days - the \
         reversal is about rank, not about length"
    );
}

#[test]
fn a_climatology_reports_its_own_exclusions() {
    for rule in SWEPT_RULES {
        let c = climatology(*rule).expect("a climatology");
        let accounted = c.years.len() + c.rejected_incomplete.len() + c.rejected_never_opened.len();
        assert_eq!(
            accounted,
            years().len(),
            "{}: every year must be either used or explained",
            rule.name()
        );
        assert!(c.min_days <= c.median_days && c.median_days <= c.max_days);
        assert!(c.mean_days >= c.min_days && c.mean_days <= c.max_days);
    }
}

#[test]
fn the_median_is_the_middle_value() {
    // ⚠ Added after the odd/even branches were written the wrong way round
    // while quieting a lint. `min <= median <= max` passed throughout.
    let c = climatology(FIXED_OCT_NOV).expect("a climatology");
    let mut days: Vec<f64> = c
        .years
        .iter()
        .map(|&y| {
            FallYear::get(y)
                .expect("a complete year")
                .window(FIXED_OCT_NOV)
                .expect("a window")
                .operable_days
        })
        .collect();
    days.sort_by(f64::total_cmp);
    assert_eq!(
        days.len(),
        19,
        "an ODD number of years, so no midpoint is taken"
    );
    assert!(
        close(c.median_days, days[9], 0.0),
        "median {} must be the 10th of 19, {}",
        c.median_days,
        days[9]
    );
    // And the even branch, on a roster of known length.
    let even = climatology(AFTER_CORN).expect("corn opens somewhere");
    assert!(
        even.years.len() < 19,
        "the corn rule drops years, giving a different parity to exercise"
    );
}

#[test]
fn the_climatology_refuses_what_it_cannot_answer() {
    let impossible = WindowRule::HarvestGated {
        crop: Series::Corn,
        percent: 1000.0,
    };
    assert!(
        climatology(impossible).is_none(),
        "a threshold no year reaches yields no climatology, not an empty mean"
    );
    let empty_span = WindowRule::FixedCalendar {
        from: "12-30",
        to: "12-31",
    };
    let c = climatology(empty_span).expect("the rule still opens");
    assert!(
        close(c.mean_days, 0.0, 0.0) && !c.mean_days.is_sign_negative(),
        "a span containing no weeks is zero days, positively"
    );
}

// ════════════════════════════════════ roster 2: EPA NR-005c's own arithmetic

#[test]
fn every_averaged_bin_reproduces_its_printed_composite() {
    // ★★ The document's own checksum. A mistranscribed cycle stops reconciling.
    for bin in AVERAGED_BINS {
        let mean = bin.mean_of_cycles().expect("a non-empty roster");
        assert!(
            bin.composite.admits(mean),
            "{}: cycles mean to {mean}, document prints {}",
            bin.name,
            bin.composite.value()
        );
    }
    assert!(HIGH_LF_BIN.reconciles() && LOW_LF_BIN.reconciles());
}

#[test]
fn a_mistyped_cycle_breaks_the_bin_check() {
    // The gate above made to fail: without it, any cycle could be wrong.
    use cf_tillage::{Bin, Cycle, Printed};
    const MISTYPED: &[Cycle] = &[
        Cycle {
            equipment: "agricultural tractor",
            load_factor: Printed::new(0.87, 2),
        },
        Cycle {
            equipment: "crawler dozer",
            load_factor: Printed::new(0.58, 2),
        },
        Cycle {
            equipment: "rubber-tire loader",
            load_factor: Printed::new(0.48, 2),
        },
        Cycle {
            equipment: "excavator",
            load_factor: Printed::new(0.53, 2),
        },
    ];
    const BROKEN: Bin = Bin {
        name: "High",
        composite: Printed::new(0.59, 2),
        cycles: MISTYPED,
    };
    assert!(
        !BROKEN.reconciles(),
        "transposing 0.78 to 0.87 must break the composite"
    );
}

#[test]
fn a_bin_with_no_cycles_refuses_rather_than_dividing_by_zero() {
    use cf_tillage::{Bin, Printed};
    const EMPTY: Bin = Bin {
        name: "empty",
        composite: Printed::new(0.59, 2),
        cycles: &[],
    };
    assert!(EMPTY.mean_of_cycles().is_none());
    assert!(
        !EMPTY.reconciles(),
        "an empty roster must not read as agreement"
    );
}

#[test]
fn the_source_prints_the_tractor_two_ways_and_both_are_carried() {
    // ⛔⛔ Carry the spread, do not resolve it.
    let (lo, hi) = load_factor_band();
    assert!(close(lo, AGRICULTURAL_TRACTOR_APPLIED_LF.value(), 0.0));
    assert!(close(hi, AGRICULTURAL_TRACTOR_MEASURED_LF.value(), 0.0));
    assert!(lo < hi, "the applied value is the lower of the two");
    assert!(
        !AGRICULTURAL_TRACTOR_APPLIED_LF.overlaps(AGRICULTURAL_TRACTOR_MEASURED_LF),
        "and the two printings do not even overlap at their printed precision"
    );
}

#[test]
fn the_binning_haircut_is_the_measured_one() {
    let (lo, hi) = load_factor_band();
    let haircut = (lo / hi - 1.0) * 100.0;
    assert!(
        close(haircut, LOAD_FACTOR_HAIRCUT_PERCENT, 0.05),
        "{haircut} against the pinned {LOAD_FACTOR_HAIRCUT_PERCENT}"
    );
    // And the tractor is the cycle the binning costs most, which is why it
    // matters here rather than being a curiosity about EPA's method.
    let worst = HIGH_LF_BIN
        .cycles
        .iter()
        .map(|c| c.load_factor.value())
        .fold(f64::MIN, f64::max);
    assert!(
        close(worst, hi, 0.0),
        "the agricultural tractor is the highest cycle in its bin"
    );
}

#[test]
fn the_annual_hours_are_carried_but_never_used_to_build_a_window() {
    // ⚠ A national figure over a 50-150 hp population, against a 215.88 hp
    // tractor at one site. Present for provenance; the window comes from NASS.
    assert_eq!(AGRICULTURAL_TRACTOR_ANNUAL_HOURS, 475);
    let season = nominal_season(NOMINAL_YEAR, NOMINAL_RULE, 0.59);
    assert!(
        season.hours() < f64::from(AGRICULTURAL_TRACTOR_ANNUAL_HOURS),
        "one fall window is a fraction of a year's work, not all of it"
    );
}

// ═══════════════════════════════════════════════ the tractor, read not retyped

#[test]
fn the_tractor_is_read_live_from_the_reconciling_crate() {
    let t = john_deere_8245r().expect("every needed row reconciles");
    assert_eq!(t.summary_no, 963);
    assert!(t.make_model.contains("8245R"));
    for row in NEEDED_ROWS {
        let reading = cf_nebraska::JOHN_DEERE_8245R
            .get(row)
            .unwrap_or_else(|| panic!("{row} is absent from the source crate"));
        assert!(
            reading.agreed().is_some(),
            "{row} must reconcile across editions for this crate to use it"
        );
    }
}

#[test]
fn the_diesel_thermal_efficiency_is_where_a_tier_four_engine_belongs() {
    // ★ The chain's sanity check: a unit slip anywhere between Nebraska's
    // hp·hr/gal, AFDC's Btu/gal and SI shows up here as an absurd number.
    let t = john_deere_8245r().expect("tractor");
    let pct = t.pto_thermal_efficiency() * 100.0;
    assert!(
        DIESEL_PTO_THERMAL_EFFICIENCY_PERCENT.admits(pct),
        "{pct} against the pinned {}",
        DIESEL_PTO_THERMAL_EFFICIENCY_PERCENT.value()
    );
    assert!((30.0..45.0).contains(&pct), "{pct}% is not a diesel");
    assert!(
        t.drawbar_thermal_efficiency() < t.pto_thermal_efficiency(),
        "the drawbar is downstream of the PTO, so it must be less efficient"
    );
}

#[test]
fn the_two_drivetrain_routes_agree() {
    // ★★ Independent: one route never touches a fuel figure.
    let t = john_deere_8245r().expect("tractor");
    let gap = (t.drivetrain_from_fuel() - t.drivetrain_from_power()).abs() * 100.0;
    assert!(
        gap < DRIVETRAIN_ROUTES_AGREE_WITHIN_PP,
        "routes differ by {gap} pp"
    );
    for route in [t.drivetrain_from_fuel(), t.drivetrain_from_power()] {
        assert!(
            (0.85..1.0).contains(&route),
            "{route} is not a drivetrain efficiency"
        );
    }
}

#[test]
fn the_unit_bridge_is_derived_and_not_transcribed() {
    // A third document's rounding must not sit between the two printings.
    let btu = btu_per_hp_hr();
    assert!(close(btu, 2544.43, 0.01), "{btu}");
    let lhv = diesel_lhv_kwh_per_l();
    assert!((9.0..11.0).contains(&lhv), "{lhv} kWh/L is not diesel");
    // And the AFDC figure it comes from is the one actually printed.
    assert!(close(DIESEL_LHV_BTU_PER_GAL.value(), 128_488.0, 0.0));
    assert_eq!(DIESEL_LHV_BTU_PER_GAL.decimals(), 0);
}

#[test]
fn the_heating_value_matches_stage_two() {
    // Cross-crate: both read the same AFDC table, so a drift means one of them
    // re-transcribed it.
    let here = HYDROGEN_LHV_KWH_PER_KG;
    let there = cf_electrolysis::AFDC_2026.lhv_kwh_per_kg();
    assert!(
        here.admits(there),
        "stage 4 prints {} and stage 2 computes {there}",
        here.value()
    );
}

// ═══════════════════════════════════════ the demand, and the seam it plugs into

#[test]
fn the_demand_draws_exactly_what_it_was_asked_for() {
    let w = FallYear::get(NOMINAL_YEAR)
        .expect("2012")
        .window(NOMINAL_RULE)
        .expect("window");
    let samples = YEAR.len();
    let want = 3_000.0;
    let d = TillageDemand::new(NOMINAL_YEAR, &w, want, samples, YEAR.interval_seconds())
        .expect("a demand");
    assert!(
        close(d.total_kg(), want, 1e-6),
        "asked for {want}, profile totals {}",
        d.total_kg()
    );
    assert!(close(d.requested_kg(), want, 0.0));
    assert_eq!(d.samples(), samples);
}

#[test]
fn the_demand_follows_the_days_the_ground_was_workable() {
    // ★★ The reason this is not `SeasonalDemand`: a real season is not even.
    let fy = FallYear::get(NOMINAL_YEAR).expect("2012");
    let w = fy.window(NOMINAL_RULE).expect("window");
    let samples = YEAR.len();
    let d = TillageDemand::new(NOMINAL_YEAR, &w, 3_000.0, samples, YEAR.interval_seconds())
        .expect("a demand");
    let weeks: Vec<_> = fy
        .series(Series::DaysSuitable)
        .into_iter()
        .filter(|o| w.opens_after.is_some_and(|open| o.week_ending > open))
        .collect();
    assert_eq!(d.weeks(), weeks.len(), "one segment per published week");

    // The heaviest week must be the one with the most days suitable.
    let per_week: Vec<(f64, f64)> = weeks
        .iter()
        .map(|o| {
            let end = day_of_year(o.week_ending).expect("date");
            let per_day = 86_400.0 / YEAR.interval_seconds();
            let first = ((f64::from(end) - 7.0) * per_day) as usize;
            let last = (f64::from(end) * per_day) as usize;
            (o.value, (first..last).map(|i| d.kg_at(i)).sum::<f64>())
        })
        .collect();
    let best_days = per_week
        .iter()
        .max_by(|a, b| a.0.total_cmp(&b.0))
        .expect("weeks");
    let best_kg = per_week
        .iter()
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .expect("weeks");
    assert!(
        close(best_days.0, best_kg.0, 0.0),
        "the week with the most operable days must carry the most kilograms"
    );
    // And a week with fewer days must carry strictly less.
    let (lo, hi) = (
        per_week
            .iter()
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .expect("w"),
        best_days,
    );
    if lo.0 < hi.0 {
        assert!(lo.1 < hi.1, "{lo:?} should draw less than {hi:?}");
    }
}

#[test]
fn the_demand_is_zero_outside_its_window() {
    let w = FallYear::get(NOMINAL_YEAR)
        .expect("2012")
        .window(NOMINAL_RULE)
        .expect("window");
    let samples = YEAR.len();
    let d = TillageDemand::new(NOMINAL_YEAR, &w, 3_000.0, samples, YEAR.interval_seconds())
        .expect("a demand");
    let opens = day_of_year(w.opens_after.expect("gated")).expect("date");
    let per_day = 86_400.0 / YEAR.interval_seconds();
    // A sample well before the window opens, and one on 1 January.
    for i in [0usize, ((f64::from(opens) - 14.0) * per_day) as usize] {
        assert!(
            close(d.kg_at(i), 0.0, 0.0),
            "sample {i} is outside the window and must draw nothing"
        );
    }
    assert!(
        close(d.kg_at(samples + 1_000), 0.0, 0.0),
        "past the end draws nothing"
    );
}

#[test]
fn the_demand_refuses_what_it_cannot_shape() {
    let w = FallYear::get(NOMINAL_YEAR)
        .expect("2012")
        .window(NOMINAL_RULE)
        .expect("window");
    let samples = YEAR.len();
    let dt = YEAR.interval_seconds();
    assert!(TillageDemand::new(NOMINAL_YEAR, &w, 3_000.0, samples, 0.0).is_none());
    assert!(TillageDemand::new(NOMINAL_YEAR, &w, 3_000.0, samples, -1.0).is_none());
    assert!(TillageDemand::new(NOMINAL_YEAR, &w, 3_000.0, samples, f64::NAN).is_none());
    assert!(TillageDemand::new(NOMINAL_YEAR, &w, f64::NAN, samples, dt).is_none());
    assert!(TillageDemand::new(NOMINAL_YEAR, &w, -1.0, samples, dt).is_none());
    assert!(
        TillageDemand::new(NOMINAL_YEAR, &w, 3_000.0, 10, dt).is_none(),
        "a window that does not fit inside the series must be refused, not clipped"
    );
    assert!(
        TillageDemand::new(1800, &w, 3_000.0, samples, dt).is_none(),
        "a year the extract does not carry"
    );
}

// ══════════════════════════════════ roster 1: the headline is pinned, not printed

#[test]
fn the_break_even_band_is_the_measured_one() {
    // ★★★ THE ANSWER. The cliff is RECOMPUTED for this window from stages 1-3
    // on the real series, so an upstream change reddens this gate.
    let delivered = delivered_series();
    let (days, cliff) = in_window_kg(&delivered, NOMINAL_RULE);
    assert!(
        (5_000.0..9_000.0).contains(&cliff),
        "the in-window production is {cliff} kg over {days} operable days"
    );
    let t = john_deere_8245r().expect("tractor");
    let (lo, hi) = load_factor_band();
    let h2 = HYDROGEN_LHV_KWH_PER_KG.value();
    let at = |lf: f64| {
        break_even_engine_efficiency(
            &t,
            &nominal_season(NOMINAL_YEAR, NOMINAL_RULE, lf),
            cliff,
            h2,
        )
        .expect("a break-even below 100%")
            * 100.0
    };
    let (want_lo, want_hi) = HYDROGEN_ENGINE_EFFICIENCY_NEEDED;
    assert!(close(at(lo), want_lo, 0.05), "applied 0.59 -> {}%", at(lo));
    assert!(close(at(hi), want_hi, 0.05), "measured 0.78 -> {}%", at(hi));
    assert!(
        at(hi) < 25.0,
        "even the pessimistic load factor needs only {}% - and what that is \
         under is stated by the_break_even_is_a_fraction_of_the_measured_diesel, \
         not by an unsourced claim about hydrogen engines",
        at(hi)
    );
}

#[test]
fn the_window_rule_nearly_cancels_against_its_own_production() {
    // ⛔⛔ The correction, gated. A longer window buys operable days AND
    // in-window hydrogen together. Holding the cliff fixed across rules - which
    // an earlier version did - inflates this term fiftyfold.
    let delivered = delivered_series();
    let t = john_deere_8245r().expect("tractor");
    let h2 = HYDROGEN_LHV_KWH_PER_KG.value();
    let (lo, _) = load_factor_band();
    let consistent: Vec<f64> = SWEPT_RULES
        .iter()
        .map(|r| {
            let (days, cliff) = in_window_kg(&delivered, *r);
            break_even_engine_efficiency(
                &t,
                &Season {
                    operable_days: days,
                    hours_per_operable_day: NOMINAL_HOURS_PER_OPERABLE_DAY,
                    load_factor: lo,
                },
                cliff,
                h2,
            )
            .expect("a break-even")
                * 100.0
        })
        .collect();
    let spread = consistent.iter().copied().fold(f64::MIN, f64::max)
        - consistent.iter().copied().fold(f64::MAX, f64::min);
    assert!(
        spread < WINDOW_RULE_CANCELS_WITHIN_PP,
        "recomputed consistently the rules spread {spread} pp: {consistent:?}"
    );

    // And the inconsistent comparison really is the thing that inflates it.
    let (_, fixed_cliff) = in_window_kg(&delivered, NOMINAL_RULE);
    let inconsistent: Vec<f64> = SWEPT_RULES
        .iter()
        .map(|r| {
            let (days, _) = in_window_kg(&delivered, *r);
            break_even_engine_efficiency(
                &t,
                &Season {
                    operable_days: days,
                    hours_per_operable_day: NOMINAL_HOURS_PER_OPERABLE_DAY,
                    load_factor: lo,
                },
                fixed_cliff,
                h2,
            )
            .expect("a break-even")
                * 100.0
        })
        .collect();
    let bad_spread = inconsistent.iter().copied().fold(f64::MIN, f64::max)
        - inconsistent.iter().copied().fold(f64::MAX, f64::min);
    assert!(
        bad_spread > spread * 10.0,
        "holding the cliff fixed must visibly inflate the term: {bad_spread} \
         against {spread}"
    );
}

#[test]
fn the_season_closes_with_margin() {
    let delivered = delivered_series();
    let (days, cliff) = in_window_kg(&delivered, NOMINAL_RULE);
    let t = john_deere_8245r().expect("tractor");
    let (lo, hi) = load_factor_band();
    let h2 = HYDROGEN_LHV_KWH_PER_KG.value();
    let margin = |lf: f64| {
        let need = season_hydrogen_kg(
            &t,
            &Season {
                operable_days: days,
                hours_per_operable_day: NOMINAL_HOURS_PER_OPERABLE_DAY,
                load_factor: lf,
            },
            0.40,
            h2,
        )
        .expect("kilograms");
        cliff / need
    };
    let (want_tight, want_loose) = SEASON_MARGIN_AT_FORTY_PERCENT;
    assert!(
        close(margin(hi), want_tight, 0.01),
        "at 0.78 -> {}x",
        margin(hi)
    );
    assert!(
        close(margin(lo), want_loose, 0.01),
        "at 0.59 -> {}x",
        margin(lo)
    );
    assert!(
        margin(hi) > 2.0,
        "even the pessimistic load factor leaves more than double"
    );
}

#[test]
fn the_annual_hydrogen_is_never_the_binding_constraint() {
    // ⚠ Worth stating because it is the opposite of the intuition: the farm's
    // YEAR of hydrogen dwarfs the tillage season. What binds is the WINDOW.
    let delivered = delivered_series();
    let annual: f64 = delivered.iter().fold(0.0, |a, b| a + b);
    let t = john_deere_8245r().expect("tractor");
    let (_, hi) = load_factor_band();
    let season = nominal_season(NOMINAL_YEAR, NOMINAL_RULE, hi);
    let needed =
        season_hydrogen_kg(&t, &season, 0.40, HYDROGEN_LHV_KWH_PER_KG.value()).expect("kilograms");
    assert!(
        needed * 10.0 < annual,
        "the season needs {needed} kg against {annual} kg produced - more than \
         tenfold headroom, so the constraint is storage and timing"
    );
    let vs_year =
        break_even_engine_efficiency(&t, &season, annual, HYDROGEN_LHV_KWH_PER_KG.value())
            .expect("a break-even");
    assert!(
        vs_year < 0.05,
        "against the whole year the required efficiency is {vs_year}, which is \
         not an engineering constraint at all"
    );
}

#[test]
fn a_bigger_window_needs_a_better_engine() {
    // The monotonicity the headline depends on: more operable days means more
    // work, so more hydrogen, so a higher break-even.
    let t = john_deere_8245r().expect("tractor");
    let h2 = HYDROGEN_LHV_KWH_PER_KG.value();
    let mk = |days: f64| Season {
        operable_days: days,
        hours_per_operable_day: NOMINAL_HOURS_PER_OPERABLE_DAY,
        load_factor: 0.59,
    };
    let a = break_even_engine_efficiency(&t, &mk(20.0), 2_695.0, h2).expect("a");
    let b = break_even_engine_efficiency(&t, &mk(30.0), 2_695.0, h2).expect("b");
    assert!(a < b, "{a} should be below {b}");
    // And more available hydrogen means a lower bar.
    let c = break_even_engine_efficiency(&t, &mk(30.0), 5_390.0, h2).expect("c");
    assert!(
        c < b,
        "doubling the hydrogen must halve the required efficiency"
    );
    assert!(close(c, b / 2.0, 1e-9));
}

#[test]
fn the_season_arithmetic_reproduces_the_measured_fuel_rate() {
    // ★ The formula checked against the instrument that produced its inputs:
    // at full load and the measured efficiency it must reproduce Nebraska's
    // own printed fuel rate.
    let t = john_deere_8245r().expect("tractor");
    let one_hour = Season {
        operable_days: 1.0,
        hours_per_operable_day: 1.0,
        load_factor: 1.0,
    };
    let litres = season_diesel_litres(&t, &one_hour).expect("litres");
    assert!(
        close(litres, t.pto_fuel_l_per_h, 1e-9),
        "one hour at rated power should burn {} L, got {litres}",
        t.pto_fuel_l_per_h
    );
}

#[test]
fn the_hydrogen_and_diesel_routes_agree_at_equal_efficiency() {
    // A hydrogen engine as efficient as the diesel must need the same shaft
    // energy, so the two fuels differ only by their heating values.
    let t = john_deere_8245r().expect("tractor");
    let s = nominal_season(NOMINAL_YEAR, NOMINAL_RULE, 0.59);
    let litres = season_diesel_litres(&t, &s).expect("litres");
    let kg = season_hydrogen_kg(
        &t,
        &s,
        t.pto_thermal_efficiency(),
        HYDROGEN_LHV_KWH_PER_KG.value(),
    )
    .expect("kg");
    let diesel_kwh = litres * diesel_lhv_kwh_per_l();
    let hydrogen_kwh = kg * HYDROGEN_LHV_KWH_PER_KG.value();
    assert!(
        close(diesel_kwh, hydrogen_kwh, 1e-6),
        "{diesel_kwh} kWh of diesel against {hydrogen_kwh} kWh of hydrogen"
    );
}

#[test]
fn the_break_even_refuses_what_it_cannot_answer() {
    let t = john_deere_8245r().expect("tractor");
    let s = nominal_season(NOMINAL_YEAR, NOMINAL_RULE, 0.59);
    let h2 = HYDROGEN_LHV_KWH_PER_KG.value();
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(
            break_even_engine_efficiency(&t, &s, bad, h2).is_none(),
            "{bad} kg"
        );
        assert!(
            break_even_engine_efficiency(&t, &s, 2_695.0, bad).is_none(),
            "{bad} LHV"
        );
    }
    // ⛔ Too little hydrogen is not a small efficiency - it is no answer.
    assert!(
        break_even_engine_efficiency(&t, &s, 1.0, h2).is_none(),
        "one kilogram cannot close a season at any efficiency"
    );
    let empty = Season {
        operable_days: 0.0,
        hours_per_operable_day: 12.0,
        load_factor: 0.59,
    };
    assert!(break_even_engine_efficiency(&t, &empty, 2_695.0, h2).is_none());
}

#[test]
fn the_hydrogen_demand_refuses_an_impossible_engine() {
    let t = john_deere_8245r().expect("tractor");
    let s = nominal_season(NOMINAL_YEAR, NOMINAL_RULE, 0.59);
    let h2 = HYDROGEN_LHV_KWH_PER_KG.value();
    for bad in [0.0, -0.1, 1.01, f64::NAN, f64::INFINITY] {
        assert!(
            season_hydrogen_kg(&t, &s, bad, h2).is_none(),
            "{bad} is not a thermal efficiency"
        );
    }
    assert!(
        season_hydrogen_kg(&t, &s, 1.0, h2).is_some(),
        "exactly 1.0 is the limit and is allowed, so the bound is closed"
    );
}

// ═══════════════════════════════════════════════════════ caveats and unknowns

#[test]
fn the_caveats_are_ranked_by_measured_effect() {
    let mut last = f64::INFINITY;
    for c in BREAK_EVEN_CAVEATS {
        assert!(
            c.break_even_swing_pp <= last,
            "{} is out of order at {}",
            c.what,
            c.break_even_swing_pp
        );
        last = c.break_even_swing_pp;
        assert!(c.break_even_swing_pp > 0.0);
        assert!(!c.range.is_empty(), "{}: a swing without its range", c.what);
        assert!(!c.why.is_empty());
    }
    assert_eq!(BREAK_EVEN_CAVEATS.len(), 3);
}

#[test]
fn each_caveat_says_whose_range_it_moved_over() {
    // ⚠ Without this the ranking compares a measurement against a modelling
    // choice and presents both as findings.
    let sourced = BREAK_EVEN_CAVEATS
        .iter()
        .filter(|c| c.range.contains("source"))
        .count();
    let chosen = BREAK_EVEN_CAVEATS
        .iter()
        .filter(|c| c.range.contains("chosen by this crate"))
        .count();
    assert_eq!(sourced, 1, "exactly one range is the source's own");
    assert_eq!(chosen, 2, "the other two are this crate's");
    assert_eq!(sourced + chosen, BREAK_EVEN_CAVEATS.len());
}

#[test]
fn the_caveat_magnitudes_are_the_measured_ones() {
    // ⚠ The cliff is recomputed at EVERY point. Holding it fixed is what put
    // the window rule at the top of this list instead of the bottom.
    let delivered = delivered_series();
    let t = john_deere_8245r().expect("tractor");
    let h2 = HYDROGEN_LHV_KWH_PER_KG.value();
    let be = |rule: WindowRule, hpd: f64, lf: f64| {
        let (days, cliff) = in_window_kg(&delivered, rule);
        break_even_engine_efficiency(
            &t,
            &Season {
                operable_days: days,
                hours_per_operable_day: hpd,
                load_factor: lf,
            },
            cliff,
            h2,
        )
        .expect("a break-even")
            * 100.0
    };
    let (lo, hi) = load_factor_band();
    let lf_swing = be(NOMINAL_RULE, 12.0, hi) - be(NOMINAL_RULE, 12.0, lo);
    let hours_swing = be(NOMINAL_RULE, 14.0, lo) - be(NOMINAL_RULE, 10.0, lo);
    let rule_values: Vec<f64> = SWEPT_RULES.iter().map(|r| be(*r, 12.0, lo)).collect();
    let rule_swing = rule_values.iter().copied().fold(f64::MIN, f64::max)
        - rule_values.iter().copied().fold(f64::MAX, f64::min);

    for (name, measured) in [
        ("hours worked per operable day", hours_swing),
        (
            "the load factor, 0.59 applied against 0.78 measured",
            lf_swing,
        ),
        ("the window rule", rule_swing),
    ] {
        let pinned = BREAK_EVEN_CAVEATS
            .iter()
            .find(|c| c.what == name)
            .unwrap_or_else(|| panic!("{name} is not in the caveat list"))
            .break_even_swing_pp;
        assert!(
            close(measured, pinned, 0.02),
            "{name}: measured {measured} pp against pinned {pinned} pp"
        );
    }
    // And the ranking the list asserts is the ranking the measurements produce.
    assert!(hours_swing > lf_swing && lf_swing > rule_swing);
}

#[test]
fn the_unknowns_say_what_would_close_them() {
    assert!(UNMEASURED_HERE.len() >= 4);
    for u in UNMEASURED_HERE {
        assert!(!u.what.is_empty());
        assert!(!u.why_unmeasured.is_empty());
        assert!(
            u.what_would_measure_it.len() > 20,
            "{}: an unknown with no route to closing it is a shrug",
            u.what
        );
    }
    assert!(
        UNMEASURED_HERE
            .iter()
            .any(|u| u.what.contains("acres per hour")),
        "the term this crate deliberately does not produce"
    );
    assert!(
        UNMEASURED_HERE.iter().any(|u| u.what.contains("WIND year")),
        "and the one the chain cannot yet answer at all"
    );
}

#[test]
fn every_source_states_its_own_terms() {
    // ⛔ Stage 3 found that "a government document, therefore free" is false.
    use cf_tillage::{AFDC_FUEL_PROPERTIES, EPA_NR005C, NEBRASKA_TRACTOR_TEST};
    for s in [
        NASS_CROP_PROGRESS,
        EPA_NR005C,
        AFDC_FUEL_PROPERTIES,
        NEBRASKA_TRACTOR_TEST,
    ] {
        assert!(!s.document.is_empty() && !s.url.is_empty());
        assert_eq!(s.retrieved.len(), 10, "{}: an ISO date", s.document);
        assert!(!s.terms.is_empty(), "{}: terms must be stated", s.document);
    }
    assert!(
        NEBRASKA_TRACTOR_TEST.terms.contains("NOT DETERMINED"),
        "the state laboratory's status is undetermined and must say so"
    );
    assert!(
        NASS_CROP_PROGRESS.terms.contains("105"),
        "the federal sources cite the section that makes them free"
    );
}

#[test]
fn the_ideal_gas_would_not_change_this_stage() {
    // ⚠ Stage 3 measured the equation of state as worth 0.21% on delivered
    // kilograms and 19.2% on tank VOLUME. This stage consumes kilograms, so it
    // should inherit the small end. Gated so the claim is not just repeated.
    let tank = cf_storage::Tank::new(TANK_BAR, cf_storage::CARRINGTON_FALL.mean_k);
    let real = tank
        .compression_kwh_per_kg(OUTLET_BAR, &cf_storage::Covolume, STATION_EFFICIENCY)
        .expect("real");
    let ideal = tank
        .compression_kwh_per_kg(OUTLET_BAR, &IdealGas, STATION_EFFICIENCY)
        .expect("ideal");
    let case = current_distributed();
    let specific = case.total_kwh_per_kg.value();
    let a = debit_compression(1_000.0, specific, real).expect("a").kg;
    let b = debit_compression(1_000.0, specific, ideal).expect("b").kg;
    let moved = (b / a - 1.0).abs() * 100.0;
    assert!(
        moved < 0.5,
        "the equation of state moves this stage's input by {moved}%, which \
         should be the diluted end of stage 3's leverage"
    );
}

// ════════════════════════════ the refusal paths, each one actually exercised

#[test]
fn the_line_parser_rejects_every_way_a_line_can_be_wrong() {
    // ⛔ The committed extract parses cleanly by construction, so without this
    // every rejection branch in the parser is unexercised.
    assert!(cf_tillage::parse_line("2012\t2012-10-07\tDS\t5.2").is_some());
    for bad in [
        "",                                 // empty
        "notayear\t2012-10-07\tDS\t5.2",    // year
        "2012",                             // truncated after year
        "2012\t2012-10-07",                 // truncated after date
        "2012\t2012-10-07\tDS",             // truncated after series
        "2012\t2012-10-07\tTILLAGE\t5.2",   // unknown series
        "2012\t2012-10-07\tDS\tnope",       // value
        "2012\t2012-10-7\tDS\t5.2",         // date not ten characters
        "2012\t2012-10-07\tDS\t5.2\textra", // trailing field
    ] {
        assert!(
            cf_tillage::parse_line(bad).is_none(),
            "{bad:?} should not parse"
        );
    }
}

#[test]
fn the_median_takes_a_midpoint_when_the_count_is_even() {
    // The other branch of `the_median_is_the_middle_value`. Soybeans at 95%
    // drops three years, giving an EVEN roster.
    let rule = WindowRule::HarvestGated {
        crop: Series::Soybeans,
        percent: 95.0,
    };
    let c = climatology(rule).expect("a climatology");
    assert_eq!(
        c.years.len(),
        16,
        "an even roster is what exercises the midpoint"
    );
    let mut days: Vec<f64> = c
        .years
        .iter()
        .map(|&y| {
            FallYear::get(y)
                .expect("year")
                .window(rule)
                .expect("window")
                .operable_days
        })
        .collect();
    days.sort_by(f64::total_cmp);
    assert!(
        close(c.median_days, f64::midpoint(days[7], days[8]), 1e-12),
        "median {} must be the midpoint of {} and {}",
        c.median_days,
        days[7],
        days[8]
    );
}

#[test]
fn the_diesel_baseline_refuses_what_it_cannot_answer() {
    let t = john_deere_8245r().expect("tractor");
    for bad in [f64::NAN, f64::INFINITY, -1.0] {
        let s = Season {
            operable_days: bad,
            hours_per_operable_day: 12.0,
            load_factor: 0.59,
        };
        assert!(
            season_diesel_litres(&t, &s).is_none(),
            "{bad} operable days must not yield litres"
        );
    }
    // And a tractor whose measured efficiency cannot be computed.
    let broken = cf_tillage::Tractor {
        pto_fuel_l_per_h: 0.0,
        ..t
    };
    assert!(
        season_diesel_litres(&broken, &nominal_season(NOMINAL_YEAR, NOMINAL_RULE, 0.59)).is_none(),
        "a zero fuel rate implies an infinite efficiency, which is not an answer"
    );
}

#[test]
fn the_hydrogen_demand_refuses_a_broken_season_or_heating_value() {
    let t = john_deere_8245r().expect("tractor");
    let ok = nominal_season(NOMINAL_YEAR, NOMINAL_RULE, 0.59);
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(
            season_hydrogen_kg(&t, &ok, 0.40, bad).is_none(),
            "{bad} is not a heating value"
        );
    }
    for bad in [f64::NAN, f64::INFINITY, -1.0] {
        let s = Season {
            operable_days: bad,
            hours_per_operable_day: 12.0,
            load_factor: 0.59,
        };
        assert!(
            season_hydrogen_kg(&t, &s, 0.40, HYDROGEN_LHV_KWH_PER_KG.value()).is_none(),
            "{bad} operable days must not yield kilograms"
        );
    }
}

#[test]
fn a_window_with_no_operable_days_shapes_no_demand() {
    // 2017's corn-gated window opens in the last published week, so it contains
    // no days at all. The profile must refuse rather than divide by zero.
    let fy = FallYear::get(2017).expect("2017");
    let w = fy.window(AFTER_CORN).expect("the rule opens");
    assert_eq!(w.weeks_counted, 0);
    assert!(
        TillageDemand::new(2017, &w, 3_000.0, YEAR.len(), YEAR.interval_seconds()).is_none(),
        "no operable days means no profile"
    );
}

#[test]
fn an_absurd_sampling_interval_is_refused_rather_than_overflowing() {
    // The `to_index` guard, reached: a denormal interval makes samples-per-day
    // infinite, so the index is not a number.
    let w = FallYear::get(NOMINAL_YEAR)
        .expect("2012")
        .window(NOMINAL_RULE)
        .expect("window");
    assert!(
        TillageDemand::new(NOMINAL_YEAR, &w, 3_000.0, YEAR.len(), 1e-320).is_none(),
        "an interval that overflows samples-per-day must be refused"
    );
    assert!(TillageDemand::new(NOMINAL_YEAR, &w, 3_000.0, YEAR.len(), f64::MIN_POSITIVE).is_none());
}

#[test]
fn a_sampling_interval_coarser_than_a_week_is_refused() {
    // ⛔ The OTHER half of the window bound, and it fires for a different
    // reason than the infinite case: at one sample per fortnight a seven-day
    // reporting week maps to zero samples, so `last == first`. Mutation
    // testing found this half unexercised — `last > samples` was catching
    // everything else.
    let w = FallYear::get(NOMINAL_YEAR)
        .expect("2012")
        .window(NOMINAL_RULE)
        .expect("window");
    let fortnight = 14.0 * 86_400.0;
    assert!(
        TillageDemand::new(NOMINAL_YEAR, &w, 3_000.0, YEAR.len(), fortnight).is_none(),
        "a week of window cannot be spread across zero samples"
    );
    // And the boundary: one sample per day still works.
    assert!(
        TillageDemand::new(NOMINAL_YEAR, &w, 3_000.0, 400, 86_400.0).is_some(),
        "daily sampling is coarse but well defined"
    );
}

#[test]
fn the_demand_profile_says_what_it_is() {
    let w = FallYear::get(NOMINAL_YEAR)
        .expect("2012")
        .window(NOMINAL_RULE)
        .expect("window");
    let d = TillageDemand::new(
        NOMINAL_YEAR,
        &w,
        3_000.0,
        YEAR.len(),
        YEAR.interval_seconds(),
    )
    .expect("a demand");
    assert!(
        d.name().contains("tillage") && d.name().contains("days suitable"),
        "a profile that does not name itself produces orphan numbers: {}",
        d.name()
    );
}

// ══════════════════════════════════ gates added by the review of this PR

#[test]
fn the_break_even_is_a_fraction_of_the_measured_diesel() {
    // Hoisted above the statements: an item declared after them reads as if
    // it were scoped later than it is.
    const BANNED: &[&str] = &[
        "engines run well above",
        "comfortably above",
        "engines are above",
    ];
    // ⛔ The ONLY engine comparison this crate is entitled to make. It divides
    // one measured number by another: the break-even against the brake thermal
    // efficiency Nebraska measured for this tractor's own diesel. A claim about
    // what hydrogen engines reach would need an oracle that does not exist in
    // `farm/`, and `UNMEASURED_HERE` says so.
    let t = john_deere_8245r().expect("tractor");
    let diesel = t.pto_thermal_efficiency() * 100.0;
    let (lo, hi) = HYDROGEN_ENGINE_EFFICIENCY_NEEDED;
    let (want_lo, want_hi) = cf_tillage::BREAK_EVEN_AS_SHARE_OF_MEASURED_DIESEL_PERCENT;
    assert!(
        close(lo / diesel * 100.0, want_lo, 0.05),
        "{lo}% of {diesel}% is {}%",
        lo / diesel * 100.0
    );
    assert!(
        close(hi / diesel * 100.0, want_hi, 0.05),
        "{hi}% of {diesel}% is {}%",
        hi / diesel * 100.0
    );
    assert!(
        want_hi < 60.0 && want_hi > 50.0,
        "the headline phrase is `a little over half` - {want_hi}% must support it"
    );
    // ⚠ And no surface may claim where real hydrogen engines sit.
    // ⛔⛔ Scans the PROSE surfaces only. Including this file makes the check
    // match its own banned list and fail always - the producer-matching-itself
    // bug a previous PR shipped, reproduced here on the first run.
    for (name, src) in [
        ("src/lib.rs", include_str!("../src/lib.rs")),
        ("NASS_VALIDATION.md", include_str!("../NASS_VALIDATION.md")),
    ] {
        for banned in BANNED {
            assert!(
                !src.contains(banned),
                "{name}: an unsourced claim about hydrogen engines came back: {banned:?}"
            );
        }
    }
    // And the scan is not vacuous: it finds the phrase when it is there.
    assert!(
        BANNED
            .iter()
            .any(|b| "engines run well above that".contains(b)),
        "the banned-phrase scan cannot detect its own target"
    );
}

#[test]
fn stage_threes_own_illustration_is_what_the_correction_says_it_is() {
    // ⛔ The module doc attributes 2,695 kg over a 21-day window to stage 3, and
    // the whole correction narrative turns on that being a DIFFERENT window from
    // the ones measured here. Recomputed, so it cannot rot when stage 3 moves.
    let outlet = outlet_series();
    let delivered = delivered_series();
    let twenty_one_days =
        cf_storage::SeasonalDemand::new(1.0, 289, 21, outlet.len(), YEAR.interval_seconds())
            .expect("21 days from 15 October, stage 3's illustration");
    // ⛔ On the OUTLET series. Stage 3's cliff is production before its own
    // compression debit; this crate's windows are measured after it. Asserting
    // 2,695 against `delivered` gives 2,610 — which is how the second half of
    // the mismatch was found.
    let cliff = produced_during_demand(&outlet, &twenty_one_days);
    assert!(
        close(cliff, 2_695.0, 0.5),
        "stage 3's 21-day cliff recomputes to {cliff} kg on the outlet series"
    );
    let after_debit = produced_during_demand(&delivered, &twenty_one_days);
    assert!(
        after_debit < cliff,
        "and the same window after compression is smaller: {after_debit} kg"
    );
    // The point of the correction: this crate's windows are longer, so they
    // carry far more in-window production and are not comparable to it.
    let (_, mine) = in_window_kg(&delivered, NOMINAL_RULE);
    assert!(
        mine > cliff * 2.0,
        "a seven-week window holds {mine} kg against the 21-day {cliff} kg - \
         comparing a demand across them is the error this crate documents"
    );
}

#[test]
fn the_committed_extract_reproduces_the_published_reports() {
    // ⛔ NASS_VALIDATION.md's oracle check, made executable. It is the only
    // thing tying the committed extract to NASS's own published weekly reports,
    // and it was prose.
    //
    // ⚠ Validates TRANSCRIPTION, not measurement: both renderings come from the
    // same field office, so a survey error is invisible to it.
    const PUBLISHED: &[(&str, f64)] = &[
        ("2012-09-30", 6.8),
        ("2012-10-14", 6.2),
        ("2012-10-28", 4.2),
        ("2012-11-04", 4.2),
        ("2012-11-18", 4.1),
    ];
    let fy = FallYear::get(2012).expect("2012");
    let days = fy.series(Series::DaysSuitable);
    for &(week, printed) in PUBLISHED {
        let got = days
            .iter()
            .find(|o| o.week_ending == week)
            .unwrap_or_else(|| panic!("{week} is absent from the committed extract"));
        assert!(
            close(got.value, printed, 1e-9),
            "{week}: the PDF prints {printed}, the extract holds {}",
            got.value
        );
    }
    assert_eq!(
        PUBLISHED.len(),
        5,
        "five reports were read, and five are checked"
    );
}

#[test]
fn the_early_harvest_that_drives_the_reversal() {
    // ⛔ The mechanism behind WINDOW_RULE_REVERSES_THE_RANKING. Only the half
    // that is in the committed extract is gated; the five-year average the same
    // report prints is quoted in the docs and deliberately not relied on.
    let fy = FallYear::get(NOMINAL_YEAR).expect("2012");
    let corn = fy.series(Series::Corn);
    let late_october = corn
        .iter()
        .find(|o| o.week_ending == "2012-10-28")
        .expect("the week the docs quote");
    assert!(
        close(late_october.value, 94.0, 1e-9),
        "corn stood at {} on 28 October 2012",
        late_october.value
    );
    // And it really is early: no other complete year is this far along by then.
    let others: Vec<f64> = climatology(FIXED_OCT_NOV)
        .expect("a climatology")
        .years
        .iter()
        .filter(|&&y| y != NOMINAL_YEAR)
        .filter_map(|&y| {
            FallYear::get(y)?
                .series(Series::Corn)
                .into_iter()
                .find(|o| o.month_day() >= "10-25" && o.month_day() <= "10-31")
                .map(|o| o.value)
        })
        .collect();
    assert!(!others.is_empty(), "the comparison set is populated");
    let beaten = others.iter().filter(|&&v| v >= 94.0).count();
    assert_eq!(
        beaten, 0,
        "2012 should lead every other complete year at that week, but {beaten} match it"
    );
}

#[test]
fn the_inconsistent_comparison_inflates_the_window_rule() {
    // ⛔ The magnitude the docs used to quote as 19.81 pp, measured instead of
    // written down. Holding the cliff at one window's value while varying the
    // window is the error; this shows what it costs.
    let delivered = delivered_series();
    let t = john_deere_8245r().expect("tractor");
    let h2 = HYDROGEN_LHV_KWH_PER_KG.value();
    let (lo, _) = load_factor_band();
    let spread = |fixed_cliff: Option<f64>| {
        let v: Vec<f64> = SWEPT_RULES
            .iter()
            .map(|r| {
                let (days, own) = in_window_kg(&delivered, *r);
                break_even_engine_efficiency(
                    &t,
                    &Season {
                        operable_days: days,
                        hours_per_operable_day: NOMINAL_HOURS_PER_OPERABLE_DAY,
                        load_factor: lo,
                    },
                    fixed_cliff.unwrap_or(own),
                    h2,
                )
                .expect("a break-even")
                    * 100.0
            })
            .collect();
        v.iter().copied().fold(f64::MIN, f64::max) - v.iter().copied().fold(f64::MAX, f64::min)
    };
    let consistent = spread(None);
    // ⛔ The historical error held the cliff at stage 3's published 2,695 kg —
    // a figure from a 21-day window on the pre-compression series. Reproduced
    // exactly, because the magnitude depends on WHICH wrong value was held.
    let inflated = spread(Some(2_695.0));
    assert!(
        close(inflated, 19.81, 0.05),
        "the inconsistent comparison spreads {inflated} pp"
    );
    assert!(
        close(consistent, 0.39, 0.02),
        "the consistent one spreads {consistent} pp"
    );
    assert!(
        inflated > consistent * 40.0,
        "{inflated} against {consistent} is the whole finding"
    );
}
