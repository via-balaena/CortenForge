//! Gates for stage 3 — compression and storage.
//!
//! Pre-registered before the crate existed, from three external rosters: DOE
//! Program Record 9013's printed figures, Table 2 of Goodwin et al. 1964, and
//! four NIST isotherms that are **retrieved and never committed**.
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

use cf_electrolysis::{
    AFDC_2026, Electrolyser, FixedSpecificEnergy, annual_hydrogen, current_distributed,
};
use cf_storage::{
    CARRINGTON_FALL, CARRINGTON_YEAR, Covolume, DELIVERED_KILOGRAMS_CAVEATS, Demand,
    EquationOfState, FlatDemand, GOODWIN_TABLE_2, IdealGas, NIST_DENSITY_COMPARISON, RECORD_9013,
    RECORD_DISAGREEMENT_WITH_CURRENT_NIST_PERCENT, SEVEN_HUNDRED_BAR_AS_ABSTRACTED,
    SEVEN_HUNDRED_BAR_AS_TABLED, SeasonalDemand, TANK_SIZE_CAVEATS, Tank, UNMEASURED_AT_FARM_SCALE,
    debit_compression, minimum_tank_kg, produced_during_demand, run_tank, second_virial_2a,
    second_virial_2a_band, second_virial_2b,
};
use cf_wind::{Air, EWT_DW54X, FOSTER_COUNTY_ND_2012 as YEAR, Machine};

/// Air at the site's 484 m elevation, matching both earlier stages' tests.
const AIR: Air = Air {
    density_kg_m3: 1.16,
};
/// The plant the chain's figures are pinned at: 1 MW, 10% turndown.
const RATED_W: f64 = 1.0e6;
const TURNDOWN: f64 = 0.10;
/// The electrolyser outlet, bar — `cf_electrolysis`'s boundary, carried forward.
const OUTLET_BAR: f64 = 20.68;
/// A tractor tank's working pressure, bar.
const TANK_BAR: f64 = 350.0;
/// The record's own station efficiency to 440 bar, as a fraction.
const STATION_EFFICIENCY: f64 = 0.52;

/// Exactly zero, as in: the code under test returns the literal `0.0`.
///
/// ⚠ Not an approximate comparison wearing a disguise. These are the paths that
/// return a zero constant — an empty profile, a sample past the end, a tank that
/// is not a number — and asserting "close to zero" there would pass on a bug
/// that returned 1e-300.
#[allow(
    clippy::float_cmp,
    reason = "the values compared are literals the code returns, not computed quantities"
)]
fn exactly_zero(x: f64) -> bool {
    x == 0.0
}

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs().max(1.0)
}

/// The real wind year, turned into hydrogen at the outlet, sample by sample.
///
/// ⚠ This reconstructs what `cf_electrolysis::annual_hydrogen` sums, from that
/// crate's **public trait** rather than from a copy of its constants — and
/// `the_reconstructed_series_is_the_one_stage_two_reports` proves the two agree
/// on the real year. That gate is the seam: if either crate's arithmetic moves,
/// it reddens here instead of the chain quietly disagreeing with itself.
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

// ══════════════════════════════════════ layer 1: differential, table vs prose

/// Table 1 with exactly 1.00 kWh/kg added to every figure — the mutation the
/// differential layer is, by construction, unable to notice.
const EVERY_FIGURE_PLUS_ONE: &[cf_storage::TheoreticalWork] = &[
    cf_storage::TheoreticalWork {
        to_bar: 350.0,
        printed: cf_storage::Printed::new(2.05, 2),
    },
    cf_storage::TheoreticalWork {
        to_bar: 440.0,
        printed: cf_storage::Printed::new(2.15, 2),
    },
    cf_storage::TheoreticalWork {
        to_bar: 700.0,
        printed: cf_storage::Printed::new(2.35, 2),
    },
    cf_storage::TheoreticalWork {
        to_bar: 880.0,
        printed: cf_storage::Printed::new(2.47, 2),
    },
];

#[test]
fn the_table_increments_reproduce_the_prose() {
    assert!(
        RECORD_9013.increments_reconcile(),
        "Table 1's own differences must reproduce the increments the prose states"
    );
}

#[test]
fn the_differential_layer_cannot_resolve_the_last_digit() {
    // ⚠ A second blind spot, found by mutation rather than by reading the code.
    // Differencing two two-decimal figures doubles the rounding slack: 1.15−1.05
    // is only pinned to ±0.015, so the prose could have printed 0.11 and this
    // layer would still reconcile. Measured here rather than described, with the
    // threshold shown from both sides.
    let with_increment = |v: f64| cf_storage::Record9013 {
        increment_350_to_440: cf_storage::Printed::new(v, 2),
        ..RECORD_9013
    };
    assert!(
        with_increment(0.11).increments_reconcile(),
        "0.11 is inside the resolution of a difference of two rounded figures — \
         this layer genuinely cannot see it"
    );
    assert!(
        !with_increment(0.13).increments_reconcile(),
        "but 0.13 must be caught, or the layer resolves nothing at all"
    );
    assert!(
        !with_increment(0.07).increments_reconcile(),
        "and the limit must bite in both directions"
    );
}

#[test]
fn the_differential_layer_is_blind_to_an_error_in_both_places() {
    // ⚠ Naming a blind spot is cheap; showing it is not. Shift the table and the
    // prose together and the layer stays green — which is exactly why it is not
    // the only layer.
    let shifted = cf_storage::Record9013 {
        theoretical: EVERY_FIGURE_PLUS_ONE,
        ..RECORD_9013
    };
    assert!(
        shifted.increments_reconcile(),
        "a whole kWh/kg added to every figure leaves the differences identical — \
         the differential layer cannot see it, and that is the property being pinned"
    );
    // …and the physical layer can.
    let cov = Covolume;
    let w = cov
        .isothermal_work_kwh_per_kg(RECORD_9013.inlet_bar, 350.0, RECORD_9013.temperature_k)
        .expect("a state the model answers for");
    assert!(
        !shifted.theoretical[0].printed.admits(w),
        "the model must reject the shifted figure the differential layer accepted"
    );
}

// ═══════════════════════════════════ layer 2: multiplicative, theory vs plant

#[test]
fn the_printed_efficiencies_reconcile_theory_with_practice() {
    assert!(
        RECORD_9013.efficiencies_reconcile(),
        "theoretical / actual has to reproduce the record's own printed efficiency"
    );
}

/// Table 1 and the station figures scaled by the SAME factor — the mutation the
/// multiplicative layer is, by construction, unable to notice.
const EVERYTHING_SCALED_BY_TWO: (&[cf_storage::TheoreticalWork], &[cf_storage::StationFigure]) = (
    &[
        cf_storage::TheoreticalWork {
            to_bar: 350.0,
            printed: cf_storage::Printed::new(2.10, 2),
        },
        cf_storage::TheoreticalWork {
            to_bar: 440.0,
            printed: cf_storage::Printed::new(2.30, 2),
        },
        cf_storage::TheoreticalWork {
            to_bar: 700.0,
            printed: cf_storage::Printed::new(2.70, 2),
        },
        cf_storage::TheoreticalWork {
            to_bar: 880.0,
            printed: cf_storage::Printed::new(2.94, 2),
        },
    ],
    &[
        cf_storage::StationFigure {
            who: "HDSAM, doubled",
            to_bar: 440.0,
            printed: cf_storage::Printed::new(4.46, 2),
            efficiency_percent: Some(cf_storage::Printed::new(52.0, 0)),
        },
        cf_storage::StationFigure {
            who: "HDSAM, doubled",
            to_bar: 880.0,
            printed: cf_storage::Printed::new(6.0, 1),
            efficiency_percent: Some(cf_storage::Printed::new(49.0, 0)),
        },
    ],
);

#[test]
fn the_multiplicative_layer_is_blind_to_a_common_scale_factor() {
    // ⚠ The layer's stated blind spot, SHOWN rather than described. It divides
    // theory by practice, so a factor applied to both cancels exactly. This gate
    // existed only as a sentence in a doc comment until review asked for it.
    let (theoretical, station) = EVERYTHING_SCALED_BY_TWO;
    let doubled = cf_storage::Record9013 {
        theoretical,
        station,
        ..RECORD_9013
    };
    assert!(
        doubled.efficiencies_reconcile(),
        "doubling theory AND practice leaves every ratio identical — the \
         multiplicative layer cannot see it, and that is the property being pinned"
    );
    // …and the physical layer can, which is why there is more than one.
    let w = Covolume
        .isothermal_work_kwh_per_kg(RECORD_9013.inlet_bar, 350.0, RECORD_9013.temperature_k)
        .expect("a state the model answers for");
    assert!(
        !doubled.theoretical[0].printed.admits(w),
        "the model must reject the doubled figure the ratio check accepted"
    );
}

#[test]
fn the_multiplicative_layer_refuses_an_empty_roster() {
    // ⛔⛔ EMPTY is not evidence. A layer with nothing left to check must fail,
    // not report success — this is the shape that let a vacuous gate ship in
    // stage 2.
    let gutted = cf_storage::Record9013 {
        station: &[],
        ..RECORD_9013
    };
    assert!(
        !gutted.efficiencies_reconcile(),
        "a roster with no efficiencies in it must FAIL the check, not pass it vacuously"
    );
}

// ════════════════════════════════════ layer 3: cross-document heating value

#[test]
fn the_record_agrees_with_the_other_doe_document_about_the_heating_value() {
    // Two DOE publications, transcribed independently by two crates. The record
    // prints 33.3; AFDC prints 51,585 Btu/lb, which converts to four figures.
    let afdc = AFDC_2026.lhv_kwh_per_kg();
    assert!(
        RECORD_9013.lhv_agrees_with(afdc),
        "the record's 33.3 kWh/kg must admit AFDC's {afdc:.4}"
    );
    // ⚠ The check is only as sharp as the coarser printing, and MEASURED rather
    // than described: 33.3 admits [33.25, 33.35], so against AFDC's 33.3296 the
    // band is +0.06% / −0.24%. A drift inside that is invisible here.
    assert!(
        RECORD_9013.lhv_agrees_with(afdc * 1.0005),
        "+0.05% sits inside the coarser printing — that is this layer's blind spot"
    );
    assert!(
        !RECORD_9013.lhv_agrees_with(afdc * 1.002),
        "+0.2% must NOT be invisible, or the layer checks nothing at all"
    );
}

#[test]
fn the_share_of_heating_value_band_contains_its_own_measurements() {
    // ⚠ A finding, found by this gate failing first: the record's "5–20% of LHV"
    // is NOT its measured range rounded to nearest. 1.7 and 6.4 kWh/kg over
    // 33.3 are 5.11% and 19.22%, so the printed band is rounded OUTWARD — it is
    // a containing statement, not an arithmetic one. Checking it as arithmetic
    // would have reported a source error that is not there.
    let (low, high) = RECORD_9013.share_of_lhv_from_measurements();
    let (printed_low, printed_high) = RECORD_9013.share_of_lhv_percent;
    assert!(close(low, 5.11, 2e-3), "low share drifted to {low:.3}%");
    assert!(close(high, 19.22, 2e-3), "high share drifted to {high:.3}%");
    assert!(
        low >= printed_low.value() && high <= printed_high.value(),
        "the measured range {low:.2}–{high:.2}% must sit inside the printed          {}–{}% band",
        printed_low.value(),
        printed_high.value()
    );
    // …and containment is not free: a tenth on either measured endpoint breaks it.
    let lhv = RECORD_9013.lhv_kwh_per_kg.value();
    assert!(
        100.0 * RECORD_9013.tech_val_range_kwh_per_kg.0.value() * 0.9 / lhv < printed_low.value(),
        "a 10% lower measurement must fall outside the printed band"
    );
    assert!(
        100.0 * RECORD_9013.tech_val_range_kwh_per_kg.1.value() * 1.1 / lhv > printed_high.value(),
        "a 10% higher measurement must fall outside the printed band"
    );
}

// ══════════════════════════════════════════ layer 4: the source against itself

#[test]
fn the_record_disagrees_with_itself_about_700_bar() {
    assert!(
        !SEVEN_HUNDRED_BAR_AS_TABLED.overlaps(&SEVEN_HUNDRED_BAR_AS_ABSTRACTED),
        "1.35 and 1.36 are printed for the same quantity and their roundings do \
         not overlap — if these are ever made equal, the finding has been smoothed away"
    );
    assert!(
        (SEVEN_HUNDRED_BAR_AS_ABSTRACTED.value() - SEVEN_HUNDRED_BAR_AS_TABLED.value()).abs() > 0.0,
        "the two printings must stay distinct"
    );
}

// ════════════════════════════ the model, from a source outside every check

#[test]
fn the_paper_reproduces_its_own_calculated_columns() {
    // ★★ The strongest transcription check in this crate: the paper prints the
    // OUTPUT of its own equation in its own column, so an implementation can be
    // compared with the page it came from rather than with a derived quantity.
    // This is the equation the model uses, and it comes back clean on all nine.
    assert_eq!(
        GOODWIN_TABLE_2.len(),
        9,
        "the roster must not shrink silently"
    );
    let mut signs = (0usize, 0usize);
    for r in GOODWIN_TABLE_2 {
        let b = second_virial_2b(r.temperature_k).expect("a tabulated temperature");
        assert!(
            r.calculated_2b.admits(b),
            "eq (2b) at {} K gives {b:.4}, outside the printed {:.2}",
            r.temperature_k,
            r.calculated_2b.value()
        );
        if b > r.calculated_2b.value() {
            signs.0 += 1;
        } else {
            signs.1 += 1;
        }
    }
    // ★ And the residual changes sign, which a systematic transcription error
    // would not. Contrast the one-term fit, below.
    assert!(
        signs.0 >= 3 && signs.1 >= 3,
        "the residual must scatter, not drift: {} high, {} low",
        signs.0,
        signs.1
    );
}

#[test]
fn the_one_term_fit_is_systematically_low_against_its_own_column() {
    // ⛔⛔ A finding, recorded rather than repaired. The paper's OTHER equation
    // does not reproduce its own printed column: three of nine rows fall outside
    // the last digit, and every residual but one is negative.
    let residuals: Vec<f64> = GOODWIN_TABLE_2
        .iter()
        .map(|r| second_virial_2a(r.temperature_k).expect("tabulated") - r.calculated_2a.value())
        .collect();
    let misses = GOODWIN_TABLE_2
        .iter()
        .filter(|r| {
            !r.calculated_2a
                .admits(second_virial_2a(r.temperature_k).expect("tabulated"))
        })
        .count();
    assert_eq!(misses, 3, "three rows must fall outside the printed digit");
    let mean = residuals.iter().sum::<f64>() / residuals.len() as f64;
    assert!(
        mean < -0.003 && mean > -0.005,
        "the bias must be about -0.004 cm³/mol, measured {mean:.5}"
    );
    assert!(
        residuals.iter().filter(|d| **d > 0.0).count() <= 1,
        "the residual must drift one way, unlike (2b)'s"
    );

    // ⛔ And the printed precision of the equation's own parameters explains
    // well under half of it, so the rest stays an explicit unknown. Closing it
    // by nudging T0 would author an input from the check that validates it.
    let t = 248.15;
    let (lo, hi) = second_virial_2a_band(t).expect("in range");
    let explained = (hi - lo) / 2.0;
    let printed = GOODWIN_TABLE_2
        .iter()
        .find(|r| (r.temperature_k - t).abs() < f64::EPSILON)
        .expect("248.15 K is in the roster")
        .calculated_2a
        .value();
    let gap = (second_virial_2a(t).expect("in range") - printed).abs();
    assert!(
        explained < 0.5 * gap,
        "the parameter band explains ±{explained:.5} of a {gap:.5} cm³/mol gap"
    );

    // ★ And it is affordable because (2a) is not the model. ⚠ The comparison has
    // to be carried into DENSITY to mean anything: the gap is 0.04% of the
    // coefficient, but the coefficient is only a sixth of the specific volume at
    // tank pressure, so it is under 0.01% of the quantity that sizes a tank.
    // Comparing a percent-of-coefficient against a percent-of-density would be
    // two different denominators wearing the same unit.
    let v_ideal = IdealGas
        .specific_volume_m3_per_kg(TANK_BAR, t)
        .expect("a usable state");
    let b = Covolume::covolume_m3_per_kg(t).expect("in range");
    let gap_m3_per_kg = gap * 1e-6 / 2.016e-3;
    let on_density = 100.0 * gap_m3_per_kg / (v_ideal + b);
    assert!(
        on_density < 0.01,
        "carried into density at {TANK_BAR} bar the gap is {on_density:.5}%"
    );
    assert!(
        on_density * 100.0 < NIST_DENSITY_COMPARISON.covolume_at_350_bar_273k_percent,
        "and must stay two orders of magnitude below the model's own measured          {:.2}% error at the tank's temperature",
        NIST_DENSITY_COMPARISON.covolume_at_350_bar_273k_percent
    );
}

#[test]
fn a_mistyped_coefficient_breaks_the_column_check() {
    // ⛔ The gate above is only worth having if it can fail. Shifting the
    // reference temperature by 1% must put at least one row outside its printed
    // value — otherwise the check is decorative.
    let mistyped = |t: f64| 19.866 * (1.0 - (109.83_f64 * 1.01 / t).powf(1.25));
    let broken = GOODWIN_TABLE_2
        .iter()
        .filter(|r| !r.calculated_2a.admits(mistyped(r.temperature_k)))
        .count();
    assert!(
        broken >= 7,
        "a 1% error in T0 must break most rows, broke only {broken}"
    );
}

#[test]
fn the_four_term_fit_is_closer_than_the_one_term_fit() {
    let worst = |f: fn(f64) -> Option<f64>, pick: fn(&cf_storage::VirialRow) -> f64| {
        GOODWIN_TABLE_2
            .iter()
            .map(|r| (f(r.temperature_k).expect("tabulated") - pick(r)).abs())
            .fold(0.0_f64, f64::max)
    };
    let one_term = worst(second_virial_2a, |r| r.derived.value());
    let four_term = worst(second_virial_2b, |r| r.derived.value());
    assert!(
        four_term < one_term,
        "eq (2b) must be the closer fit: {four_term:.3} against {one_term:.3} cm³/mol"
    );
    assert!(
        four_term < 0.05 && one_term > 0.25,
        "measured, not asserted: (2b) worst {four_term:.3}, (2a) worst {one_term:.3} cm³/mol"
    );
}

#[test]
fn only_the_real_gas_model_reproduces_the_record() {
    // ★★★ The discriminating gate. Both models are asked the record's own
    // question at the record's own inlet and temperature, and only one of them
    // can answer it. This is the check the ideal-gas model exists to fail.
    let (ideal, cov) = (IdealGas, Covolume);
    let (mut ideal_in, mut cov_in, mut cov_missed) = (0usize, 0usize, Vec::new());
    for w in RECORD_9013.theoretical {
        let arg = |e: &dyn EquationOfState| {
            e.isothermal_work_kwh_per_kg(RECORD_9013.inlet_bar, w.to_bar, RECORD_9013.temperature_k)
                .expect("a state both models answer for")
        };
        if w.printed.admits(arg(&ideal)) {
            ideal_in += 1;
        }
        if w.printed.admits(arg(&cov)) {
            cov_in += 1;
        } else {
            cov_missed.push(w.to_bar);
        }
    }
    assert_eq!(
        ideal_in, 0,
        "the ideal gas must reproduce NONE of the record's four theoretical figures"
    );
    assert_eq!(
        cov_in, 3,
        "the covolume model must reproduce three of the four; it got {cov_in}"
    );
    assert_eq!(
        cov_missed,
        vec![700.0],
        "and the one it misses must be 700 bar — the single figure the record \
         itself prints two different ways"
    );
    // ★ And on that figure it agrees with the OTHER printing. Two independent
    // computations both landing above 1.355 is evidence about which of the
    // record's two numbers was the one computed.
    let w700 = cov
        .isothermal_work_kwh_per_kg(RECORD_9013.inlet_bar, 700.0, RECORD_9013.temperature_k)
        .expect("a state the model answers for");
    assert!(
        SEVEN_HUNDRED_BAR_AS_ABSTRACTED.admits(w700),
        "the model gives {w700:.4}, which must sit inside the Item section's 1.36"
    );
    assert!(
        !SEVEN_HUNDRED_BAR_AS_TABLED.admits(w700),
        "and outside Table 1's 1.35 — if both admitted it there would be no finding"
    );
}

#[test]
fn the_ideal_gas_understates_the_compression_work() {
    // ★ The other half of this crate's opening claim, and it had no producer:
    // the module doc asserted "~7%" and nothing measured it. It is 6.3–6.6%
    // across the temperature range the farm's tank actually spans, and the
    // contrast with the DENSITY error at the same pressure is the whole point.
    let understatement = |t: f64| {
        let ideal = IdealGas
            .isothermal_work_kwh_per_kg(OUTLET_BAR, TANK_BAR, t)
            .expect("a usable state");
        let cov = Covolume
            .isothermal_work_kwh_per_kg(OUTLET_BAR, TANK_BAR, t)
            .expect("a usable state");
        100.0 * (1.0 - ideal / cov)
    };
    let (low, high) = cf_storage::IDEAL_GAS_WORK_UNDERSTATEMENT_PERCENT;
    let hot = understatement(RECORD_9013.temperature_k);
    let cold = understatement(CARRINGTON_FALL.min_k);
    assert!(
        close(hot, low, 5e-3),
        "at 300 K measured {hot:.3}%, published {low}%"
    );
    assert!(
        close(cold, high, 5e-3),
        "at the window's coldest hour measured {cold:.3}%, published {high}%"
    );
    // ⚠ The pair must actually BOUND every temperature the crate evaluates
    // at. The earlier pair was (300 K, window MEAN) while describing itself as
    // spanning the window — the coldest hour fell outside it, at 6.65%.
    let ladder = [
        CARRINGTON_FALL.min_k,
        CARRINGTON_FALL.mean_k,
        CARRINGTON_FALL.max_k,
        RECORD_9013.temperature_k,
    ];
    let mut previous = f64::INFINITY;
    for t in ladder {
        let u = understatement(t);
        assert!(
            u >= low - 1e-9 && u <= high + 1e-9,
            "{t} K gives {u:.3}%, outside the published {low}–{high}%"
        );
        // …and "grows as the gas cools" is executable, not an adjective.
        assert!(
            u < previous,
            "{t} K gives {u:.3}%, not less than the colder point's {previous:.3}%"
        );
        previous = u;
    }
}

#[test]
fn the_equation_of_state_moves_the_tank_far_more_than_the_energy() {
    // ⛔⛔ **Both sides as a fraction of THE ANSWER, at ONE temperature.** An
    // earlier version compared a percent-of-work against a percent-of-volume —
    // two denominators wearing the same unit — and produced a "twenty times"
    // that was really 3.4. The version after that fixed the denominators and
    // then blended three temperatures, which is the same mistake in the other
    // axis. Everything here is at 273.15 K: the isotherm the density error was
    // measured on, and within 0.01 K of what the farm's window actually averages.
    const T: f64 = 273.15;

    // Energy: diluted, because compression is only a ~3% debit on the chain.
    let work = |eos: &dyn EquationOfState| {
        Tank::new(TANK_BAR, T)
            .compression_kwh_per_kg(OUTLET_BAR, eos, STATION_EFFICIENCY)
            .expect("a usable state")
    };
    let kg = |w: f64| {
        debit_compression(51_721.008, 55.8, w)
            .expect("a usable case")
            .kg
    };
    let on_kilograms = 100.0 * (kg(work(&IdealGas)) / kg(work(&Covolume)) - 1.0);

    // Volume: undiluted. A density overstated by d understates volume by d/(1+d).
    let d = NIST_DENSITY_COMPARISON.ideal_at_350_bar_273k_percent / 100.0;
    let on_volume = 100.0 * (1.0 - 1.0 / (1.0 + d));

    assert!(
        close(on_kilograms, 0.208, 5e-3),
        "ideal gas moves the delivered kilograms by {on_kilograms:.3}%"
    );
    assert!(
        close(on_volume, 19.225, 5e-3),
        "and the tank volume by {on_volume:.3}%"
    );
    // ★ The crate's thesis, as one measured ratio rather than an adjective.
    let leverage = on_volume / on_kilograms;
    assert!(
        close(leverage, 92.4, 2e-2),
        "the equation of state is worth {leverage:.0}x more on the tank than on \
         the energy — if that ratio ever approaches 1, this crate's opening \
         argument is wrong"
    );
}

#[test]
fn the_closed_form_work_is_the_integral_of_the_volume() {
    // ★ An independent physical consistency layer: the work each model reports
    // must be ∫v dP over the volume that same model reports. A closed form that
    // has drifted from its own equation of state would pass every transcription
    // check in this file.
    for eos in [&IdealGas as &dyn EquationOfState, &Covolume] {
        for &t in &[250.0, 273.15, 300.0, 330.0] {
            let (p1, p2) = (20.0_f64, 350.0_f64);
            let steps = 100_000;
            let h = (p2 - p1) / f64::from(steps);
            let v = |p: f64| {
                eos.specific_volume_m3_per_kg(p, t)
                    .expect("a state the model answers for")
            };
            let mut sum = 0.5 * (v(p1) + v(p2));
            for i in 1..steps {
                sum += v(p1 + h * f64::from(i));
            }
            let numeric = sum * h * 1e5 / 3.6e6;
            let closed = eos
                .isothermal_work_kwh_per_kg(p1, p2, t)
                .expect("a state the model answers for");
            assert!(
                close(numeric, closed, 1e-9),
                "{} at {t} K: closed form {closed:.9} against ∫v dP {numeric:.9}",
                eos.name()
            );
        }
    }
}

#[test]
fn the_covolume_is_the_only_difference_between_the_two_models() {
    // ★ Isolating a cause beats measuring a gap. These two models differ by one
    // term and nothing else, so their specific volumes differ by EXACTLY b(T) at
    // every pressure — not approximately, and not only where it was checked.
    for &t in &[250.0, 273.15, 300.0, 330.0] {
        let b = Covolume::covolume_m3_per_kg(t).expect("a temperature in range");
        for &p in &[1.0, 20.68, 350.0, 700.0, 880.0] {
            let vi = IdealGas
                .specific_volume_m3_per_kg(p, t)
                .expect("a usable state");
            let vc = Covolume
                .specific_volume_m3_per_kg(p, t)
                .expect("a usable state");
            assert!(
                close(vc - vi, b, 1e-12),
                "at {p} bar, {t} K the difference must be exactly b(T)"
            );
        }
        // …so at low enough pressure the two are indistinguishable, which is why
        // the ideal gas is fine for a pipeline and wrong for a tank.
        let gap = |p: f64| {
            (Covolume.density_kg_per_m3(p, t).expect("usable")
                / IdealGas.density_kg_per_m3(p, t).expect("usable")
                - 1.0)
                .abs()
        };
        assert!(gap(0.01) < 1e-5, "at 0.01 bar the models must converge");
        assert!(
            gap(350.0) > 1e4 * gap(0.01),
            "and the gap must be four orders of magnitude larger at tank pressure:              {:.3e} against {:.3e}",
            gap(350.0),
            gap(0.01)
        );
    }
}

#[test]
fn the_molar_mass_difference_is_below_the_model_error() {
    // ⚠ This crate uses cf-electrolysis's 2.016e-3 kg/mol so the chain has ONE
    // molar mass; NIST's equation of state uses 2.015 88e-3. Specific volume
    // scales as 1/M in both terms, so the density difference is exactly the
    // molar mass difference — measured here against the model's own residual
    // rather than dismissed in a comment.
    let (chain_m, nist_m): (f64, f64) = (2.016e-3, 2.015_88e-3);
    let relative = (chain_m - nist_m).abs() / nist_m * 100.0;
    assert!(
        close(relative, 0.005_95, 1e-2),
        "the molar mass difference is {relative:.5}%"
    );
    assert!(
        relative * 100.0 < NIST_DENSITY_COMPARISON.covolume_at_350_bar_273k_percent,
        "it must be more than two orders of magnitude below the model's own \
         measured {:.2}% error at the tank's temperature",
        NIST_DENSITY_COMPARISON.covolume_at_350_bar_273k_percent
    );
}

#[test]
fn the_models_refuse_states_they_cannot_answer_for() {
    for eos in [&IdealGas as &dyn EquationOfState, &Covolume] {
        for &(p, t) in &[
            (0.0, 300.0),
            (-1.0, 300.0),
            (f64::NAN, 300.0),
            (f64::INFINITY, 300.0),
            (350.0, 0.0),
            (350.0, -5.0),
            (350.0, f64::NAN),
        ] {
            assert!(
                eos.specific_volume_m3_per_kg(p, t).is_none(),
                "{} accepted P={p}, T={t}",
                eos.name()
            );
            assert!(
                eos.density_kg_per_m3(p, t).is_none(),
                "{} gave a density for P={p}, T={t}",
                eos.name()
            );
            assert!(
                eos.isothermal_work_kwh_per_kg(p, 350.0, t).is_none()
                    || eos.isothermal_work_kwh_per_kg(20.0, p, t).is_none(),
                "{} gave work involving P={p}, T={t}",
                eos.name()
            );
        }
    }
}

// ═══════════════════════════════════════════ the seam back to stage 2

#[test]
fn the_reconstructed_series_is_the_one_stage_two_reports() {
    // ★★★ The seam. This crate takes kilograms per interval and knows nothing
    // about electrolysis, so the per-sample series is rebuilt here from
    // `cf-electrolysis`'s public trait. That rebuild is a second implementation
    // of the same arithmetic, and this is the gate that stops the two drifting:
    // it must sum to exactly what stage 2 reports for the same year.
    let series = hydrogen_series();
    let plant = FixedSpecificEnergy::from_case(&current_distributed(), RATED_W, TURNDOWN);
    let stage_two = annual_hydrogen(
        YEAR.speeds().map(|v| EWT_DW54X.power_w(v, AIR)),
        &plant,
        YEAR.interval_seconds(),
    );
    assert_eq!(
        series.len(),
        105_408,
        "the whole 2012 series must be consumed"
    );
    assert_eq!(series.len(), stage_two.samples);
    let total: f64 = series.iter().sum();
    assert!(
        close(total, stage_two.kg, 1e-12),
        "the reconstructed series sums to {total:.6} against stage 2's {:.6}",
        stage_two.kg
    );
    assert!(
        close(total, 51_721.008, 1e-6),
        "and to the figure the chain is pinned at; got {total:.3}"
    );
}

// ═══════════════════════════════════════════════════ the tank, as a reservoir

/// The pinned window: 21 days from 15 October, on the real series.
fn fall_window(total_kg: f64) -> SeasonalDemand {
    SeasonalDemand::new(total_kg, 289, 21, 105_408, YEAR.interval_seconds())
        .expect("the window fits inside 2012")
}

#[test]
fn a_tank_below_the_minimum_does_not_meet_the_demand() {
    // ★★ "Minimum" is a claim, so it is tested as one: take the answer, shave a
    // kilogram off it, and watch the demand go unmet.
    let series = hydrogen_series();
    let demand = fall_window(2_000.0);
    let size = minimum_tank_kg(&series, &demand, 0.0).expect("the demand is meetable");
    assert!(
        size > 1.0,
        "a sub-kilogram tank would make the shave meaningless"
    );
    assert!(
        run_tank(series.iter().copied(), &demand, size, 0.0).meets_demand(),
        "the reported minimum must itself meet the demand"
    );
    assert!(
        !run_tank(series.iter().copied(), &demand, size - 1.0, 0.0).meets_demand(),
        "a tank one kilogram smaller must NOT"
    );
}

#[test]
fn the_mass_balance_is_exhaustive() {
    // ⛔⛔ Stage 2 shipped a three-term energy identity tested where one term was
    // always zero, so deleting that term entirely still passed. This runs at a
    // size where production is curtailed, demand is met AND hydrogen is left
    // over, and asserts every term is LIVE before asserting the sum.
    let series = hydrogen_series();
    let demand = fall_window(2_000.0);
    let r = run_tank(series.iter().copied(), &demand, 4_000.0, 250.0);
    assert!(r.produced_kg > 0.0, "production must be live");
    assert!(r.delivered_kg > 0.0, "delivery must be live");
    assert!(r.curtailed_kg > 0.0, "curtailment must be live");
    assert!(r.final_kg > 0.0, "the leftover term must be live");
    assert!(r.initial_kg > 0.0, "the starting term must be live");
    assert!(
        r.balance_residual_kg().abs() < 1e-6,
        "in + produced - delivered - curtailed - left = {:.9}",
        r.balance_residual_kg()
    );
}

#[test]
fn the_storage_cliff_is_at_in_window_production() {
    // ★★★ The finding of this stage, measured rather than asserted. Below the
    // hydrogen the turbine makes DURING the window the tank is a rounding error;
    // above it the tank carries the shortfall in from months earlier.
    let series = hydrogen_series();
    let cliff = produced_during_demand(&series, &fall_window(1.0));
    assert!(
        close(cliff, 2_695.0, 1e-3),
        "in-window production drifted to {cliff:.1} kg"
    );

    let size = |d: f64| minimum_tank_kg(&series, &fall_window(d), 0.0).expect("meetable");

    // ★★★ The mechanism, not a threshold picked by eye. Every kilogram of demand
    // ABOVE what the window itself produces has to be carried in from earlier, so
    // far above the cliff dTank/dDemand goes to 1. Far below it, the tank only
    // has to smooth hours against each other, and the slope is a fifth of that.
    let slope = |a: f64, b: f64| (size(b) - size(a)) / (b - a);
    let below = slope(0.2 * cliff, 0.5 * cliff);
    let above = slope(4.0 * cliff, 8.0 * cliff);
    assert!(
        below < 0.25,
        "below the cliff the tank must grow far slower than the demand: {below:.3}"
    );
    assert!(
        close(above, 1.0, 0.05),
        "above it, every extra kilogram must be stored: {above:.3}"
    );
    assert!(
        above > 4.0 * below,
        "and the two regimes must be plainly different: {below:.3} against {above:.3}"
    );

    // …with the tank at the cliff itself a quarter of the demand, pinned as the
    // measurement it is rather than as an inequality chosen to pass.
    let at_cliff = size(cliff) / cliff;
    assert!(
        close(at_cliff, 0.253, 0.02),
        "at the cliff the tank is {:.1}% of the demand",
        100.0 * at_cliff
    );
}

#[test]
fn a_seasonal_demand_needs_a_far_bigger_tank_than_a_flat_one() {
    let series = hydrogen_series();
    let total = 4_000.0;
    let flat = minimum_tank_kg(&series, &FlatDemand::new(total, 105_408), 0.0).expect("meetable");
    let seasonal = minimum_tank_kg(&series, &fall_window(total), 0.0).expect("meetable");
    assert!(
        seasonal > 4.0 * flat,
        "the same kilograms drawn in three weeks need {seasonal:.0} kg of tank \
         against {flat:.0} spread over the year — WHEN is the question"
    );
}

#[test]
fn a_full_tank_curtails_rather_than_overflowing() {
    let series = hydrogen_series();
    let demand = fall_window(100.0);
    let r = run_tank(series.iter().copied(), &demand, 50.0, 0.0);
    assert!(
        r.samples_full > 1_000,
        "a tiny tank must spend the year full"
    );
    assert!(
        r.curtailed_kg > 0.9 * r.produced_kg,
        "and almost everything made must be curtailed, not vanish"
    );
    assert!(
        r.peak_kg <= r.capacity_kg,
        "the tank must never exceed itself"
    );
    assert!(r.balance_residual_kg().abs() < 1e-6, "and still balance");
}

#[test]
fn a_damaged_production_sample_cannot_reach_the_headline() {
    // ⚠ `cf-electrolysis` shipped without this guard and one infinity produced a
    // finite-looking mass with a broken balance.
    let mut series = hydrogen_series();
    series[5_000] = f64::NAN;
    series[6_000] = f64::INFINITY;
    series[7_000] = -1.0;
    let demand = fall_window(2_000.0);
    let r = run_tank(series.iter().copied(), &demand, 4_000.0, 0.0);
    assert_eq!(
        r.samples_not_finite, 3,
        "all three must be counted, not absorbed"
    );
    assert!(r.produced_kg.is_finite(), "production must stay finite");
    assert!(
        r.balance_residual_kg().abs() < 1e-6,
        "and the balance must still close: {:.9}",
        r.balance_residual_kg()
    );
}

#[test]
fn the_sized_tank_survives_a_second_identical_year() {
    // ⚠ Sizing starts the tank empty on 1 January, which is a first-year
    // question. A farm does this every year, so the honest check is that the
    // same tank, carrying over whatever the first year left, still meets the
    // second — measured rather than assumed to be equivalent.
    let series = hydrogen_series();
    let demand = fall_window(2_000.0);
    let size = minimum_tank_kg(&series, &demand, 0.0).expect("meetable");
    let first = run_tank(series.iter().copied(), &demand, size, 0.0);
    let second = run_tank(series.iter().copied(), &demand, size, first.final_kg);
    assert!(first.meets_demand(), "year one must be met");
    assert!(
        second.meets_demand(),
        "year two, starting from year one's leftover, must be met too"
    );
}

#[test]
fn a_charged_tank_never_needs_to_be_a_smaller_tank() {
    // ⚠ Sizing starts the tank empty, and whether that was conservative looked
    // like an open question. It measured as making NO difference at either end
    // of the year — because what binds is **capacity to hold a surplus**, not
    // charge carried in. By the time the shortfall arrives the tank has been
    // filled by production anyway.
    let series = hydrogen_series();
    for (label, demand) in [
        ("October", fall_window(4_000.0)),
        (
            "January",
            SeasonalDemand::new(4_000.0, 1, 21, 105_408, YEAR.interval_seconds())
                .expect("a January window fits"),
        ),
    ] {
        let empty = minimum_tank_kg(&series, &demand, 0.0).expect("meetable");
        let full = minimum_tank_kg(&series, &demand, 1.0).expect("meetable");
        assert!(
            close(full, empty, 1e-9),
            "{label}: the starting charge changed the size, {full:.3} against {empty:.3}"
        );
    }

    // ⛔ And the mechanism, because a null result is worth nothing without one.
    // Charge an UNDERSIZED October tank and it still delivers exactly the same:
    // the borrowed hydrogen is curtailed away in the spring, months before the
    // window needs it, because the tank fills from production regardless. The
    // balance shows it as curtailment rather than absorbing it.
    let demand = fall_window(4_000.0);
    let undersized = 0.5 * minimum_tank_kg(&series, &demand, 0.0).expect("meetable");
    let from_empty = run_tank(series.iter().copied(), &demand, undersized, 0.0);
    let from_full = run_tank(series.iter().copied(), &demand, undersized, undersized);
    assert!(
        !from_empty.meets_demand(),
        "the undersized tank must fall short"
    );
    assert!(
        close(from_full.delivered_kg, from_empty.delivered_kg, 1e-12),
        "the charge must change nothing it delivers: {:.3} against {:.3}",
        from_full.delivered_kg,
        from_empty.delivered_kg
    );
    assert!(
        from_full.curtailed_kg > from_empty.curtailed_kg,
        "because it is curtailed away, not lost: {:.1} against {:.1} kg",
        from_full.curtailed_kg,
        from_empty.curtailed_kg
    );
    assert!(
        from_full.initial_kg > 0.0,
        "and the borrowed hydrogen must be visible"
    );
    assert!(
        from_full.balance_residual_kg().abs() < 1e-6,
        "with the balance still closing: {:.9}",
        from_full.balance_residual_kg()
    );
}

#[test]
fn the_sizing_refuses_what_it_cannot_answer() {
    let series = hydrogen_series();
    assert!(
        minimum_tank_kg(&series, &fall_window(1.0), -0.1).is_none(),
        "a negative starting fraction is not a state"
    );
    assert!(
        minimum_tank_kg(&series, &fall_window(1.5), 1.5).is_none(),
        "nor is a tank more than full"
    );
    // ⚠ The API accepts the whole interval, so the interval is exercised — only
    // the two endpoints were, and a bisection that assumed monotonicity was
    // being trusted at values no test had ever passed it.
    for f in [0.25_f64, 0.5, 0.75] {
        assert!(
            minimum_tank_kg(&series, &fall_window(2_000.0), f).is_some(),
            "a starting fraction of {f} is inside the accepted range"
        );
    }
    assert!(
        minimum_tank_kg(&series, &fall_window(f64::NAN), 0.0).is_none(),
        "nor is a demand that is not a number"
    );
    // A demand larger than the whole year's production cannot be met by any tank.
    assert!(
        minimum_tank_kg(&series, &fall_window(1e9), 0.0).is_none(),
        "no tank meets a demand the resource cannot supply — that must be None, \
         not a very large number"
    );
    // And a window that does not fit in the year is refused at construction.
    assert!(
        SeasonalDemand::new(100.0, 360, 21, 105_408, YEAR.interval_seconds()).is_none(),
        "a window running past 31 December must be refused"
    );
    assert!(
        SeasonalDemand::new(100.0, 289, 0, 105_408, YEAR.interval_seconds()).is_none(),
        "a zero-day window must be refused"
    );
}

// ═══════════════════════════════════════ compression, at the measured temperature

/// ★ A **compile-time** gate: the fall window's observed range must sit inside
/// the year's. Editing either constant so it does not stops the build, which no
/// runtime test can be skipped past.
const FALL_SITS_INSIDE_THE_YEAR: () = assert!(
    CARRINGTON_YEAR.min_k < CARRINGTON_FALL.min_k && CARRINGTON_FALL.max_k < CARRINGTON_YEAR.max_k
);

#[test]
fn the_measured_window_is_not_the_records_convention() {
    // ⛔ 27 kelvin, and it moves both outputs in opposite directions. This is the
    // gate that would redden if anyone replaced the measurement with 300 K.
    let doe = RECORD_9013.temperature_k;
    let farm = CARRINGTON_FALL.mean_k;
    assert!(
        doe - farm > 25.0,
        "the gap must be real: {doe} K against {farm:.2} K"
    );
    assert!(
        CARRINGTON_FALL.min_k < farm && farm < CARRINGTON_FALL.max_k,
        "the mean must lie inside the observed range"
    );
    // ★ The containment of the two windows is checked at BUILD time instead —
    // see `FALL_SITS_INSIDE_THE_YEAR`. clippy spotted that as a runtime assert
    // it was constant-valued, which is a better gate, not a worse one.
    let () = FALL_SITS_INSIDE_THE_YEAR;

    let cold = Tank::new(TANK_BAR, farm);
    let doe_convention = Tank::new(TANK_BAR, doe);
    let rho = |t: &Tank| {
        Covolume
            .density_kg_per_m3(t.working_pressure_bar, t.temperature_k)
            .expect("a usable state")
    };
    let denser = 100.0 * (rho(&cold) / rho(&doe_convention) - 1.0);
    assert!(
        close(denser, 8.96, 5e-3),
        "the cold tank must hold about 9% more: measured {denser:.2}%"
    );

    let work = |t: &Tank| {
        t.compression_kwh_per_kg(OUTLET_BAR, &Covolume, STATION_EFFICIENCY)
            .expect("a usable state")
    };
    let cheaper = 100.0 * (work(&cold) / work(&doe_convention) - 1.0);
    assert!(
        close(cheaper, -8.68, 5e-3),
        "and cost about 9% less to fill: measured {cheaper:.2}%"
    );
}

#[test]
fn the_compression_debit_is_paid_out_of_the_same_wind() {
    // ★★ The electricity is fixed by stage 1, so a compressor does not raise the
    // bill — it lowers the kilograms. The identity is exact, not iterative.
    let outlet = 51_721.008;
    let electrolysis = 55.8;
    let compression = 1.8;
    let d = debit_compression(outlet, electrolysis, compression).expect("a usable case");
    assert!(d.kg < outlet, "compression must cost kilograms");
    // Total electricity is unchanged: what made `outlet` at `e` makes `kg` at `e + w`.
    assert!(
        close(
            d.kg * (electrolysis + compression),
            outlet * electrolysis,
            1e-12
        ),
        "the same kilowatt-hours must be spent either way"
    );
    assert!(
        close(d.lost_percent, 3.13, 5e-3),
        "lost {:.3}%",
        d.lost_percent
    );
    assert!(
        close(d.compression_kwh, d.kg * compression, 1e-12),
        "the compression energy must be the kilograms actually compressed"
    );
    // ⚠ Zero compression must be the identity, or the debit has a bias in it.
    let none = debit_compression(outlet, electrolysis, 0.0).expect("a usable case");
    assert!(
        close(none.kg, outlet, 1e-12),
        "no compression must cost nothing"
    );
    // And the guards.
    assert!(debit_compression(outlet, 0.0, 1.8).is_none());
    assert!(debit_compression(outlet, electrolysis, -1.0).is_none());
    assert!(debit_compression(f64::NAN, electrolysis, 1.8).is_none());
}

#[test]
fn the_tank_refuses_what_it_cannot_answer() {
    let t = Tank::new(TANK_BAR, CARRINGTON_FALL.mean_k);
    assert!(t.capacity_kg(0.0, &Covolume).is_none());
    assert!(t.capacity_kg(-1.0, &Covolume).is_none());
    assert!(t.volume_m3(f64::NAN, &Covolume).is_none());
    assert!(
        t.compression_kwh_per_kg(OUTLET_BAR, &Covolume, 0.0)
            .is_none()
    );
    assert!(
        t.compression_kwh_per_kg(OUTLET_BAR, &Covolume, 1.5)
            .is_none()
    );
    assert!(
        t.compression_kwh_per_kg(OUTLET_BAR, &Covolume, 1.0)
            .is_some(),
        "an efficiency of exactly 1 is the reversible floor, and must be allowed"
    );
    // Volume and capacity must be inverses.
    let v = t.volume_m3(1_000.0, &Covolume).expect("a usable state");
    let back = t.capacity_kg(v, &Covolume).expect("a usable state");
    assert!(close(back, 1_000.0, 1e-12), "round trip gave {back:.6} kg");
}

// ═════════════════════════════════════════════════════════ the ranked caveats

#[test]
fn the_caveats_are_ranked_by_measured_effect() {
    for (list, label) in [
        (DELIVERED_KILOGRAMS_CAVEATS, "delivered kilograms"),
        (TANK_SIZE_CAVEATS, "tank size"),
    ] {
        assert!(!list.is_empty(), "{label}: the list must not be empty");
        let mut previous = f64::INFINITY;
        for c in list {
            let size = c.headline_swing_percent.abs();
            assert!(
                size > 0.0,
                "{label}: \u{201c}{}\u{201d} has no measured magnitude — an \
                 unmeasured risk belongs in UNMEASURED_AT_FARM_SCALE",
                c.what
            );
            assert!(
                size <= previous,
                "{label}: \u{201c}{}\u{201d} at {size:.2}% is larger than the entry \
                 above it at {previous:.2}% — the list must be sorted by measured effect",
                c.what
            );
            previous = size;
        }
    }
}

#[test]
fn the_unknowns_carry_no_magnitude_and_say_what_would_settle_them() {
    assert!(!UNMEASURED_AT_FARM_SCALE.is_empty());
    for u in UNMEASURED_AT_FARM_SCALE {
        assert!(!u.what.is_empty());
        assert!(
            u.why_unmeasured.len() > 80,
            "\u{201c}{}\u{201d} must say why, not just that",
            u.what
        );
        assert!(
            u.what_would_measure_it.len() > 40,
            "\u{201c}{}\u{201d} must name what would settle it",
            u.what
        );
    }
}

// ══════════════════════════════════════════════════════════════ the headline

#[test]
fn the_headline() {
    let series = hydrogen_series();
    let outlet: f64 = series.iter().sum();
    let tank_sizing = Tank::new(TANK_BAR, CARRINGTON_FALL.max_k);
    let tank_mean = Tank::new(TANK_BAR, CARRINGTON_FALL.mean_k);
    let w = tank_mean
        .compression_kwh_per_kg(OUTLET_BAR, &Covolume, STATION_EFFICIENCY)
        .expect("a usable state");
    let d = debit_compression(outlet, 55.8, w).expect("a usable case");
    let cliff = produced_during_demand(&series, &fall_window(1.0));

    println!("\n── stage 3, {} ──", Covolume.name());
    println!("  outlet                {outlet:>12.0} kg at {OUTLET_BAR} bar");
    println!(
        "  compression           {w:>12.4} kWh/kg at {:.2} K, {:.0}% station",
        tank_mean.temperature_k,
        STATION_EFFICIENCY * 100.0
    );
    println!(
        "  delivered             {:>12.0} kg at {TANK_BAR} bar  ({:.2}% lost)",
        d.kg, d.lost_percent
    );
    println!("  in-window production  {cliff:>12.0} kg  ← the cliff");
    for demand in [0.5 * cliff, cliff, 2.0 * cliff, 4.0 * cliff] {
        let size = minimum_tank_kg(&series, &fall_window(demand), 0.0).expect("meetable");
        let vol = tank_sizing
            .volume_m3(size, &Covolume)
            .expect("a usable state");
        let ideal = tank_sizing
            .volume_m3(size, &IdealGas)
            .expect("a usable state");
        println!(
            "    demand {demand:>7.0} kg  → tank {size:>7.0} kg = {vol:>6.1} m³  \
             (ideal gas would say {ideal:>6.1} m³ — {:>4.1}% too small)",
            100.0 * (1.0 - ideal / vol)
        );
    }
    // ★★ PINNED, not merely printed. Every figure this crate publishes lives
    // here, so a drift in the wind series, the electrolyser case, the equation
    // of state or the station efficiency reddens instead of silently rewriting
    // the headline. `cf-electrolysis` pins 51_721.008 for the same reason; this
    // test asserted only `kg > 0` until review pointed out it was a reporting
    // function wearing a test's name.
    assert!(
        close(outlet, 51_721.008, 1e-6),
        "outlet drifted to {outlet:.3} kg"
    );
    assert!(
        close(w, 1.821_987, 1e-5),
        "compression drifted to {w:.6} kWh/kg"
    );
    assert!(
        close(d.kg, 50_085.61, 1e-5),
        "delivered drifted to {:.2} kg",
        d.kg
    );
    assert!(
        close(d.lost_percent, 3.161, 1e-3),
        "lost {:.3}%",
        d.lost_percent
    );
    assert!(
        close(cliff, 2_695.0, 1e-3),
        "the cliff drifted to {cliff:.1} kg"
    );
    assert!(d.kg < outlet, "compression must always cost kilograms");

    // ⚠ And the tank figures the headline quotes, at the cliff itself.
    let at_cliff = minimum_tank_kg(&series, &fall_window(cliff), 0.0).expect("meetable");
    assert!(
        close(at_cliff, 683.0, 2e-3),
        "tank at the cliff drifted to {at_cliff:.1} kg"
    );
    let vol = tank_sizing
        .volume_m3(at_cliff, &Covolume)
        .expect("a usable state");
    assert!(close(vol, 27.8, 2e-3), "and its volume to {vol:.2} m³");
}

#[test]
fn the_caveat_magnitudes_are_the_measured_ones() {
    let series = hydrogen_series();
    let outlet: f64 = series.iter().sum();
    let e = 55.8;
    let w = |t_k: f64, eff: f64, scale: f64| {
        Tank::new(TANK_BAR, t_k)
            .compression_kwh_per_kg(OUTLET_BAR, &Covolume, eff)
            .expect("a usable state")
            * scale
    };
    let kg = |work: f64| {
        debit_compression(outlet, e, work)
            .expect("a usable case")
            .kg
    };
    let swing = |a: f64, b: f64| 100.0 * (a / b - 1.0);

    let base_w = w(CARRINGTON_FALL.mean_k, STATION_EFFICIENCY, 1.0);
    let base_kg = kg(base_w);

    // 1. the record's own two sources at 440 bar imply two efficiencies
    let apci = RECORD_9013
        .theoretical_at(440.0)
        .expect("440 bar is in the roster")
        .printed
        .value()
        / 2.05;
    let s_compressor = swing(kg(w(CARRINGTON_FALL.mean_k, apci, 1.0)), base_kg);
    // 2. the window's warmest hour instead of its mean
    let s_temperature = swing(
        kg(w(CARRINGTON_FALL.max_k, STATION_EFFICIENCY, 1.0)),
        base_kg,
    );
    // 3. the record's 1.2% disagreement with the current NIST EOS
    let scale = 1.0 - RECORD_DISAGREEMENT_WITH_CURRENT_NIST_PERCENT / 100.0;
    let s_record = swing(
        kg(w(CARRINGTON_FALL.mean_k, STATION_EFFICIENCY, scale)),
        base_kg,
    );

    // tank size: volume, at the temperature it is sized at
    let sizing = Tank::new(TANK_BAR, CARRINGTON_FALL.max_k);
    let demand_kg = 4.0 * produced_during_demand(&series, &fall_window(1.0));
    let tank_21 = minimum_tank_kg(&series, &fall_window(demand_kg), 0.0).expect("meetable");
    let twelve = SeasonalDemand::new(demand_kg, 289, 12, 105_408, YEAR.interval_seconds())
        .expect("a 12-day window fits");
    let tank_12 = minimum_tank_kg(&series, &twelve, 0.0).expect("meetable");
    let vol = |t: &Tank, m: f64| t.volume_m3(m, &Covolume).expect("a usable state");
    let s_burst = swing(vol(&sizing, tank_12), vol(&sizing, tank_21));
    let s_eos = NIST_DENSITY_COMPARISON.covolume_at_350_bar_273k_percent;
    let s_sizing_t = swing(
        vol(&Tank::new(TANK_BAR, CARRINGTON_FALL.mean_k), tank_21),
        vol(&sizing, tank_21),
    );

    println!("\n── measured caveat swings ──");
    println!(
        "  delivered kg: compressor {s_compressor:+.2}%  temperature {s_temperature:+.2}%  record {s_record:+.2}%"
    );
    println!("  tank size:    burst {s_burst:+.2}%  eos {s_eos:+.2}%  sizing-T {s_sizing_t:+.2}%");

    let pinned = |list: &[cf_storage::Caveat], what: &str, measured: f64| {
        let c = list
            .iter()
            .find(|c| c.what == what)
            .unwrap_or_else(|| panic!("no caveat named {what}"));
        assert!(
            close(c.headline_swing_percent, measured, 5e-2),
            "\u{201c}{what}\u{201d} is published as {:+.2}% but measures {measured:+.2}%",
            c.headline_swing_percent
        );
    };
    pinned(
        DELIVERED_KILOGRAMS_CAVEATS,
        "the compressor is a refuelling station's, not a farm's",
        s_compressor,
    );
    pinned(
        DELIVERED_KILOGRAMS_CAVEATS,
        "the tank is at the window's mean temperature, not its warmest hour",
        s_temperature,
    );
    pinned(
        DELIVERED_KILOGRAMS_CAVEATS,
        "the source's own figures disagree with the current NIST EOS",
        s_record,
    );
    pinned(
        TANK_SIZE_CAVEATS,
        "the demand is drawn evenly across the window",
        s_burst,
    );
    pinned(
        TANK_SIZE_CAVEATS,
        "the equation of state is two-term",
        s_eos,
    );
    pinned(
        TANK_SIZE_CAVEATS,
        "the tank is sized at the window's warmest hour",
        s_sizing_t,
    );
}

#[test]
fn the_published_oracle_figures_are_internally_consistent() {
    // ⛔⛔ The oracle test is #[ignore]d — correctly, since the data cannot be
    // committed — so NOTHING in CI guarded these six figures and a typo would
    // ship green. This cannot verify them against NIST, and does not claim to.
    // It catches a fat finger, which is the failure mode that was unguarded.
    let c = NIST_DENSITY_COMPARISON;
    for (ideal, cov, where_) in [
        (
            c.ideal_at_350_bar_300k_percent,
            c.covolume_at_350_bar_300k_percent,
            "350/300",
        ),
        (
            c.ideal_at_350_bar_273k_percent,
            c.covolume_at_350_bar_273k_percent,
            "350/273",
        ),
        (
            c.ideal_at_700_bar_300k_percent,
            c.covolume_at_700_bar_300k_percent,
            "700/300",
        ),
        (
            c.ideal_at_700_bar_273k_percent,
            c.covolume_at_700_bar_273k_percent,
            "700/273",
        ),
    ] {
        assert!(
            ideal > 0.0 && cov > 0.0,
            "{where_}: both models overstate density"
        );
        assert!(
            ideal > 5.0 * cov,
            "{where_}: one parameter from 1964 must remove most of the error, \
             {ideal:.2}% against {cov:.2}%"
        );
        assert!(
            c.worst_ideal_percent >= ideal && c.worst_covolume_percent >= cov,
            "{where_}: the worst case must bound every specific case"
        );
    }
    // Colder and denser is harder for a truncated virial, at both pressures.
    assert!(
        c.covolume_at_350_bar_273k_percent > c.covolume_at_350_bar_300k_percent
            && c.covolume_at_700_bar_273k_percent > c.covolume_at_700_bar_300k_percent,
        "the residual must grow as the gas cools"
    );
    // And higher pressure is harder than lower, at both temperatures.
    assert!(
        c.covolume_at_700_bar_300k_percent > c.covolume_at_350_bar_300k_percent
            && c.covolume_at_700_bar_273k_percent > c.covolume_at_350_bar_273k_percent,
        "the residual must grow with pressure — that is the third virial term"
    );
    // The grid must contain every pressure the crate names.
    for bar in [350.0, 440.0, 700.0, 880.0] {
        assert!(
            c.pressure_low_bar <= bar && bar <= c.pressure_high_bar,
            "{bar} bar must lie inside the comparison's own range"
        );
    }
    assert_eq!(c.states, 348, "four isotherms of 87 pressures");
}

// ══════════════════════════════════ the oracle: retrieved, never committed

/// One state from a retrieved NIST isotherm.
struct NistState {
    pressure_bar: f64,
    density: f64,
    enthalpy_kj_kg: f64,
    entropy_j_g_k: f64,
    volume: f64,
    internal_energy_kj_kg: f64,
}

/// Parse one isotherm, keeping the FIRST row for each distinct pressure.
///
/// ⛔ Keyed by the printed pressure, never by row position. Two measured traps
/// make position meaningless: the CGI emits **duplicate rows** at the
/// vapour→supercritical label boundary (73 rows came back for 71 requested),
/// and it silently **overrides the requested increment** once the point count
/// would exceed roughly six hundred — a request for 1 bar steps over 1–901 bar
/// came back at 1.5 bar steps. `PInc` is a request, not a contract.
fn parse_isotherm(text: &str) -> Vec<NistState> {
    let mut header = std::collections::HashMap::new();
    let mut out: Vec<NistState> = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let cells: Vec<&str> = line.split('\t').collect();
        if n == 0 {
            for (i, c) in cells.iter().enumerate() {
                header.insert(c.trim().to_owned(), i);
            }
            continue;
        }
        let get =
            |name: &str| -> Option<f64> { cells.get(*header.get(name)?)?.trim().parse().ok() };
        let Some(p) = get("Pressure (bar)") else {
            continue;
        };
        if out.iter().any(|s| (s.pressure_bar - p).abs() < 1e-9) {
            continue;
        }
        let (Some(density), Some(volume), Some(h), Some(entropy), Some(u)) = (
            get("Density (kg/m3)"),
            get("Volume (m3/kg)"),
            get("Enthalpy (kJ/kg)"),
            get("Entropy (J/g*K)"),
            get("Internal Energy (kJ/kg)"),
        ) else {
            continue;
        };
        out.push(NistState {
            pressure_bar: p,
            density,
            volume,
            enthalpy_kj_kg: h,
            entropy_j_g_k: entropy,
            internal_energy_kj_kg: u,
        });
    }
    out
}

/// The residual's SHAPE, which is the claim [`Covolume`]'s documentation makes.
///
/// ★ A dropped ρ² term leaves a residual proportional to ρ². Checked as a ratio
/// that stays put along the isotherm rather than by fitting a value — isolating
/// a cause beats measuring a gap, and a fitted coefficient would be a number
/// where a mechanism belongs.
fn the_residual_looks_like_a_missing_third_virial_term(t: f64, rows: &[NistState]) {
    let r_specific = 8.314_462_618_153_24 / 2.016e-3;
    let implied: Vec<f64> = rows
        .iter()
        .filter(|r| r.pressure_bar >= 100.0)
        .map(|r| {
            let z_nist = r.pressure_bar * 1e5 * r.volume / (r_specific * t);
            let v = Covolume
                .specific_volume_m3_per_kg(r.pressure_bar, t)
                .expect("a usable state");
            let z_cov = r.pressure_bar * 1e5 * v / (r_specific * t);
            (z_nist - z_cov) / r.density.powi(2)
        })
        .collect();
    let lo = implied.iter().copied().fold(f64::INFINITY, f64::min);
    let hi = implied.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    assert!(
        lo > 0.0 && hi < 3.0 * lo,
        "at {t} K the residual/ρ² must stay within a factor of three — a missing \
         third virial term looks like this; something else would not. Got {lo:.3e}–{hi:.3e}"
    );
}

/// Refuse to trust a retrieval before checking it is the one that was requested.
///
/// ⛔⛔ EMPTY is not evidence, and neither is "some rows". Two things are asserted
/// before any comparison runs: the returned **grid** is the requested grid, and
/// each row satisfies the file's own internal identities ρ·v = 1 and U + Pv = H.
fn trust_the_retrieval(file: &str, rows: &[NistState]) {
    assert_eq!(
        rows.len(),
        87,
        "{file}: expected 87 distinct pressures from PLow=20 PHigh=880 PInc=10, got {}",
        rows.len()
    );
    for (i, r) in rows.iter().enumerate() {
        let expected = 20.0 + 10.0 * i as f64;
        assert!(
            (r.pressure_bar - expected).abs() < 1e-9,
            "{file}: row {i} is at {} bar, not the requested {expected} — the CGI \
             silently rescales PInc, so the returned grid must be checked",
            r.pressure_bar
        );
    }
    for r in rows {
        assert!(
            (r.density * r.volume - 1.0).abs() < 1e-6,
            "{file}: ρ·v = {} at {} bar",
            r.density * r.volume,
            r.pressure_bar
        );
        let pv_kj = r.pressure_bar * 1e5 * r.volume / 1e3;
        assert!(
            ((r.internal_energy_kj_kg + pv_kj - r.enthalpy_kj_kg) / r.enthalpy_kj_kg).abs() < 1e-6,
            "{file}: U + Pv ≠ H at {} bar",
            r.pressure_bar
        );
    }
}

#[test]
#[ignore = "needs CF_NIST_H2_ISOTHERMS — the oracle is copyright-asserted and not committed"]
fn the_nist_comparison_reproduces() {
    let Ok(dir) = std::env::var("CF_NIST_H2_ISOTHERMS") else {
        panic!(
            "set CF_NIST_H2_ISOTHERMS to a directory of retrieved isotherms; \
                see farm/cf-storage/NIST_VALIDATION.md"
        );
    };
    let isotherms = [
        (250.0_f64, "nist_T250.tsv"),
        (273.15, "nist_T273.15.tsv"),
        (300.0, "nist_T300.tsv"),
        (330.0, "nist_T330.tsv"),
    ];

    let mut states = 0usize;
    let (mut worst_ideal, mut worst_cov) = (0.0_f64, 0.0_f64);
    let (mut at350_ideal_300, mut at350_cov_300) = (f64::NAN, f64::NAN);
    let (mut at350_ideal_273, mut at350_cov_273) = (f64::NAN, f64::NAN);
    let (mut at700_ideal_300, mut at700_cov_300) = (f64::NAN, f64::NAN);
    let (mut at700_ideal_273, mut at700_cov_273) = (f64::NAN, f64::NAN);

    for (t, file) in isotherms {
        let path = std::path::Path::new(&dir).join(file);
        let text = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
        let rows = parse_isotherm(&text);
        trust_the_retrieval(file, &rows);

        for r in &rows {
            states += 1;
            let err = |e: &dyn EquationOfState| {
                100.0
                    * (e.density_kg_per_m3(r.pressure_bar, t)
                        .expect("a usable state")
                        / r.density
                        - 1.0)
            };
            let (ei, ec) = (err(&IdealGas), err(&Covolume));
            worst_ideal = worst_ideal.max(ei.abs());
            worst_cov = worst_cov.max(ec.abs());
            let (hot, cold) = ((t - 300.0).abs() < 1e-9, (t - 273.15).abs() < 1e-9);
            if (r.pressure_bar - 350.0).abs() < 1e-9 {
                if hot {
                    at350_ideal_300 = ei;
                    at350_cov_300 = ec;
                } else if cold {
                    at350_ideal_273 = ei;
                    at350_cov_273 = ec;
                }
            } else if (r.pressure_bar - 700.0).abs() < 1e-9 {
                if hot {
                    at700_ideal_300 = ei;
                    at700_cov_300 = ec;
                } else if cold {
                    at700_ideal_273 = ei;
                    at700_cov_273 = ec;
                }
            }
        }

        the_residual_looks_like_a_missing_third_virial_term(t, &rows);
    }

    pin_the_published_figures(
        states,
        [
            worst_ideal,
            at350_ideal_300,
            at350_ideal_273,
            at700_ideal_300,
            at700_ideal_273,
        ],
        [
            worst_cov,
            at350_cov_300,
            at350_cov_273,
            at700_cov_300,
            at700_cov_273,
        ],
    );
    the_record_sits_below_the_current_nist_eos(&std::path::Path::new(&dir).join("nist_T300.tsv"));
}

/// Report the measured errors and pin every published figure against them.
///
/// Split out of the oracle test only so that test stays readable; the ordering
/// is `[worst, 350/300, 350/273, 700/300, 700/273]` for each model.
fn pin_the_published_figures(states: usize, ideal: [f64; 5], covolume: [f64; 5]) {
    let labels = [
        "worst",
        "350 bar 300 K",
        "350 bar 273 K",
        "700 bar 300 K",
        "700 bar 273 K",
    ];
    println!("\n── NIST comparison, {states} states ──");
    for i in 0..5 {
        println!(
            "  {:<14} ideal {:>+7.2}%   covolume {:>+6.2}%",
            labels[i], ideal[i], covolume[i]
        );
    }
    let c = NIST_DENSITY_COMPARISON;
    assert_eq!(
        states, c.states,
        "the comparison must cover the published count"
    );
    let published_ideal = [
        c.worst_ideal_percent,
        c.ideal_at_350_bar_300k_percent,
        c.ideal_at_350_bar_273k_percent,
        c.ideal_at_700_bar_300k_percent,
        c.ideal_at_700_bar_273k_percent,
    ];
    let published_covolume = [
        c.worst_covolume_percent,
        c.covolume_at_350_bar_300k_percent,
        c.covolume_at_350_bar_273k_percent,
        c.covolume_at_700_bar_300k_percent,
        c.covolume_at_700_bar_273k_percent,
    ];
    for i in 0..5 {
        assert!(
            close(ideal[i], published_ideal[i], 5e-3),
            "{} ideal: published {:.2}%, measured {:.2}%",
            labels[i],
            published_ideal[i],
            ideal[i]
        );
        assert!(
            close(covolume[i], published_covolume[i], 5e-3),
            "{} covolume: published {:.2}%, measured {:.2}%",
            labels[i],
            published_covolume[i],
            covolume[i]
        );
    }
}

/// The record's Table 1 against the equation of state it says it used.
///
/// ⛔⛔ Called from the oracle test rather than standing alone, because it needs
/// the retrieval. [`RECORD_DISAGREEMENT_WITH_CURRENT_NIST_PERCENT`] previously
/// had **no producer at all** while its documentation read as a measurement.
///
/// ⚠ Not an independent check of the record. Record 9013 says its figures came
/// from NIST, so this compares the record against a *later version of its own
/// source* — which is why the finding is a disagreement with a date on it and
/// not a verdict.
fn the_record_sits_below_the_current_nist_eos(path: &std::path::Path) {
    let text = std::fs::read_to_string(path).expect("the 300 K isotherm");
    let rows = parse_isotherm(&text);
    let at = |bar: f64| {
        rows.iter()
            .find(|r| (r.pressure_bar - bar).abs() < 1e-9)
            .unwrap_or_else(|| panic!("{bar} bar must be ON the retrieved grid"))
    };
    let inlet = at(RECORD_9013.inlet_bar);
    let t = RECORD_9013.temperature_k;
    let mut disagreeing = Vec::new();
    for w in RECORD_9013.theoretical {
        let state = at(w.to_bar);
        // Reversible isothermal work is the exergy difference, h − T·s.
        let nist = ((state.enthalpy_kj_kg - inlet.enthalpy_kj_kg) * 1e3
            - t * (state.entropy_j_g_k - inlet.entropy_j_g_k) * 1e3)
            / 3.6e6;
        if !w.printed.admits(nist) {
            disagreeing.push((w.to_bar, 100.0 * (nist / w.printed.value() - 1.0)));
        }
    }
    assert_eq!(
        disagreeing.iter().map(|d| d.0).collect::<Vec<_>>(),
        vec![700.0, 880.0],
        "exactly the two highest-pressure figures must disagree; got {disagreeing:?}"
    );
    for (bar, rel) in &disagreeing {
        assert!(
            *rel > 0.0,
            "the record must be LOW at {bar} bar, not high: {rel:+.2}%"
        );
    }
    let mean = disagreeing.iter().map(|d| d.1).sum::<f64>() / 2.0;
    assert!(
        close(mean, RECORD_DISAGREEMENT_WITH_CURRENT_NIST_PERCENT, 2e-2),
        "the published {RECORD_DISAGREEMENT_WITH_CURRENT_NIST_PERCENT}% must be the \
         mean of the two; measured {mean:.3}%"
    );
}

// ═════════════════════════════════════════════ the guards, and the boundaries

/// A station figure that cannot be right: dividing by it would give infinity.
const ZERO_ACTUAL: &[cf_storage::StationFigure] = &[cf_storage::StationFigure {
    who: "a figure that cannot be right",
    to_bar: 440.0,
    printed: cf_storage::Printed::new(0.0, 2),
    efficiency_percent: Some(cf_storage::Printed::new(52.0, 0)),
}];

#[test]
fn the_virial_equations_refuse_impossible_temperatures() {
    for t in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(second_virial_2a(t).is_none(), "eq (2a) accepted {t}");
        assert!(second_virial_2b(t).is_none(), "eq (2b) accepted {t}");
        assert!(second_virial_2a_band(t).is_none(), "the band accepted {t}");
        assert!(
            Covolume::covolume_m3_per_kg(t).is_none(),
            "the covolume accepted {t}"
        );
    }
}

#[test]
fn every_model_and_profile_says_what_it_is() {
    // ⚠ Names are what a sweep reports, so an unnamed model produces a number
    // nobody can attribute. Cheap to check, and it is the only thing standing
    // between a comparison table and two identical rows.
    let names = [
        IdealGas.name(),
        Covolume.name(),
        FlatDemand::new(1.0, 10).name(),
        fall_window(1.0).name(),
    ];
    for n in names {
        assert!(n.len() > 5, "\u{201c}{n}\u{201d} is not a name");
    }
    assert_ne!(
        IdealGas.name(),
        Covolume.name(),
        "two equations of state must not report the same name"
    );
}

#[test]
fn the_reconciliation_layers_fail_on_a_broken_roster() {
    // ⛔ Each layer's failure path, exercised. A check whose only tested outcome
    // is "true" has not been shown to be a check at all.
    let no_table = cf_storage::Record9013 {
        theoretical: &[],
        ..RECORD_9013
    };
    assert!(
        !no_table.increments_reconcile(),
        "with no table there is nothing to difference — that must be a failure"
    );
    assert!(
        !no_table.efficiencies_reconcile(),
        "an efficiency with no theoretical figure to divide must fail"
    );
    assert!(
        no_table.theoretical_at(350.0).is_none(),
        "and the lookup must come back empty rather than guessing"
    );
    assert!(
        RECORD_9013.theoretical_at(123.0).is_none(),
        "a pressure the record does not print must not resolve to a neighbour"
    );

    // A zero actual figure would divide by zero; it must be refused, not infinite.
    let zeroed = cf_storage::Record9013 {
        station: ZERO_ACTUAL,
        ..RECORD_9013
    };
    assert!(
        !zeroed.efficiencies_reconcile(),
        "a zero actual figure must fail the ratio, not divide by zero"
    );
}

#[test]
fn the_demand_profiles_are_bounded_and_say_where_they_are() {
    let d = fall_window(2_100.0);
    // ★ 289th day, 5-minute samples: the window must start where it says it does.
    assert_eq!(d.start_index(), 288 * 288, "day 289 at 288 samples/day");
    assert_eq!(d.window_samples(), 21 * 288, "21 days at 288 samples/day");
    assert!(
        close(d.total_kg(), 2_100.0, 1e-9),
        "the total must be preserved"
    );
    assert!(exactly_zero(d.kg_at(0)), "nothing is drawn in January");
    assert!(
        exactly_zero(d.kg_at(usize::MAX)),
        "nor past the end of the year"
    );
    assert!(
        d.kg_at(d.start_index()) > 0.0,
        "and something is drawn inside it"
    );

    let flat = FlatDemand::new(366.0, 105_408);
    assert!(close(flat.total_kg(), 366.0, 1e-9));
    assert!(flat.kg_at(0) > 0.0, "a flat draw starts on day one");
    assert!(
        exactly_zero(flat.kg_at(105_408)),
        "and stops at the end of the series"
    );
    assert!(
        exactly_zero(FlatDemand::new(1.0, 0).kg_at(0)),
        "an empty profile must divide by nothing"
    );

    // A demand of nothing needs no tank, and must say so rather than bisecting.
    let series = hydrogen_series();
    let nothing = minimum_tank_kg(&series, &FlatDemand::new(0.0, 105_408), 0.0);
    assert!(nothing.is_some_and(exactly_zero), "no demand, no tank");
    assert!(
        exactly_zero(produced_during_demand(
            &series,
            &FlatDemand::new(0.0, 105_408)
        )),
        "and no window to produce into"
    );
}

#[test]
fn the_debit_and_the_tank_hold_at_their_boundaries() {
    // Zero kilograms in must be zero kilograms out, without a division by zero
    // reaching the percentage.
    let none = debit_compression(0.0, 55.8, 1.8).expect("zero is a usable case");
    assert!(exactly_zero(none.kg));
    assert!(
        exactly_zero(none.lost_percent),
        "no hydrogen cannot lose a percentage of itself"
    );
    assert!(exactly_zero(none.compression_kwh));

    // A tank the equation of state will not answer for must refuse, not guess.
    let impossible = Tank::new(TANK_BAR, -1.0);
    assert!(impossible.capacity_kg(1.0, &Covolume).is_none());
    assert!(impossible.volume_m3(1.0, &Covolume).is_none());
    assert!(
        impossible
            .compression_kwh_per_kg(OUTLET_BAR, &Covolume, 0.52)
            .is_none(),
        "a compression figure at an impossible temperature must be None"
    );

    // And a run against a tank that is not a tank must still balance.
    let r = run_tank([1.0, 2.0, 3.0], &FlatDemand::new(0.0, 3), f64::NAN, 5.0);
    assert!(
        exactly_zero(r.capacity_kg),
        "a tank that is not a number holds nothing"
    );
    assert!(
        exactly_zero(r.initial_kg),
        "and cannot start with anything in it"
    );
    assert!(r.balance_residual_kg().abs() < 1e-12, "and still balances");
}
