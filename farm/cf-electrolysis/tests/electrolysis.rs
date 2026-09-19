//! Gates on the electrolysis stage, pre-registered before the crate was written.
//!
//! Every pinned number here was measured by running the code, never guessed.
//! The roster these came from names, for each transcription check, what that
//! check is structurally unable to see — because three checks that share a
//! blind spot are one check wearing three hats.
#![allow(
    clippy::panic,
    reason = "a gate that cannot name what it rejected is not much of a gate"
)]
#![allow(
    clippy::cast_precision_loss,
    reason = "sample counts here are ~10^5 and exact in f64"
)]

use cf_electrolysis::{
    AFDC_2026, CAVEAT_FARM_SCALE_KG_PER_DAY, Electrolyser, FixedSpecificEnergy, H2A_CASES,
    H2A_RECORD_19009, HHV_SOURCE_SPREAD_PERCENT, NOT_A_FARM_PLANT, Printed, TABLE_5_COLUMN_OF_CASE,
    TABLE_5_EFFICIENCY_ROW, ThermodynamicBound, UNMEASURED_AT_FARM_SCALE, annual_hydrogen,
    bop_scaling, current_distributed, table_5_restated_total,
};
use cf_wind::{Air, EWT_DW54X, FOSTER_COUNTY_ND_2012 as YEAR, Machine};

/// Air at the site's 484 m elevation, matching the wind stage's own tests.
const AIR: Air = Air {
    density_kg_m3: 1.16,
};
/// The plant these figures are pinned at: 1 MW, 10% turndown.
const RATED_W: f64 = 1.0e6;
const TURNDOWN: f64 = 0.10;

/// The real wind year turned into electrical power by the real power curve.
fn real_power_series() -> Vec<f64> {
    YEAR.speeds().map(|v| EWT_DW54X.power_w(v, AIR)).collect()
}

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs().max(1.0)
}

// ------------------------------------------------ layer 1: additive, in-column

#[test]
fn stack_plus_bop_reproduces_every_printed_total() {
    for case in H2A_CASES {
        assert!(
            case.components_reconcile(),
            "{}: stack {} + BoP {} does not reach printed total {}",
            case.name,
            case.stack_kwh_per_kg.value(),
            case.bop_kwh_per_kg.value(),
            case.total_kwh_per_kg.value()
        );
    }
}

#[test]
fn the_additive_check_is_not_vacuous() {
    // A case whose total is wrong by more than the components' combined slack
    // must be rejected. Without this, `components_reconcile` could return true
    // unconditionally and every case above would still pass.
    let mut broken = H2A_CASES[0];
    broken.total_kwh_per_kg = Printed::new(57.0, 1);
    assert!(
        !broken.components_reconcile(),
        "a total 1.2 kWh/kg off its own components was accepted"
    );
}

// -------------------------------------------- layer 2: multiplicative, in-row

#[test]
fn every_case_recovers_the_heating_values_from_its_own_ratio_rows() {
    // Eight independent cells. These bracket the values AFDC prints, which is
    // the point: the ratio rows are an INDEPENDENT check, not their source.
    for case in H2A_CASES {
        let (lhv, hhv) = case.implied_heating_values();
        assert!(
            (33.30..=33.36).contains(&lhv),
            "{}: implied LHV {lhv:.3} outside 33.30..33.36",
            case.name
        );
        assert!(
            (39.37..=39.41).contains(&hhv),
            "{}: implied HHV {hhv:.3} outside 39.37..39.41",
            case.name
        );
    }
}

// ------------------------- layer 3: cross-table, DIFFERENT column order

#[test]
fn table_five_restates_table_two_totals_in_a_different_column_order() {
    for (i, case) in H2A_CASES.iter().enumerate() {
        let Some(restated) = table_5_restated_total(i) else {
            panic!("case {i} has no Table 5 column")
        };
        assert!(
            close(case.total_kwh_per_kg.value(), restated, 1e-12),
            "{}: Table 2 prints {} and Table 5 prints {restated}",
            case.name,
            case.total_kwh_per_kg.value()
        );
    }
    assert!(table_5_restated_total(H2A_CASES.len()).is_none());
}

#[test]
fn the_two_tables_really_do_order_their_columns_differently() {
    // The machine-checkable form of the claim that justifies carrying a third
    // layer at all. If this permutation were sorted, the two sources would be
    // in the same order and the cross-table check would be a self-comparison.
    let sorted = TABLE_5_COLUMN_OF_CASE.windows(2).all(|w| w[0] < w[1]);
    assert!(
        !sorted,
        "TABLE_5_COLUMN_OF_CASE is monotonic, so the orders no longer differ: \
         {TABLE_5_COLUMN_OF_CASE:?}"
    );
    assert_eq!(TABLE_5_COLUMN_OF_CASE.len(), H2A_CASES.len());

    // And the row really is Table 5's eight columns, not four of Table 2's
    // padded out: the 2014 study's figures are different numbers entirely, so a
    // copy of Table 2 could not produce them.
    assert_eq!(TABLE_5_EFFICIENCY_ROW.len(), 8);
    let totals_2019: Vec<f64> = H2A_CASES
        .iter()
        .map(|c| c.total_kwh_per_kg.value())
        .collect();
    for (i, v) in TABLE_5_EFFICIENCY_ROW.iter().enumerate() {
        if !TABLE_5_COLUMN_OF_CASE.contains(&i) {
            assert!(
                !totals_2019.iter().any(|t| close(*t, *v, 1e-12)),
                "column {i} ({v}) is a 2014 figure but matches a 2019 total"
            );
        }
    }
}

#[test]
fn the_cross_table_check_catches_what_the_other_two_cannot() {
    // Swap two whole cases. Both other layers are computed INSIDE a column, so
    // both still pass; only the differently ordered table notices. This is the
    // gate that justifies carrying a third source at all.
    let mut swapped: Vec<_> = H2A_CASES.to_vec();
    swapped.swap(1, 2);
    for case in &swapped {
        assert!(
            case.components_reconcile(),
            "layer 1 should be blind to a whole-column swap"
        );
        let (lhv, _) = case.implied_heating_values();
        assert!(
            (33.30..=33.36).contains(&lhv),
            "layer 2 should be blind to a whole-column swap"
        );
    }
    let caught = swapped.iter().enumerate().any(|(i, c)| {
        !close(
            c.total_kwh_per_kg.value(),
            table_5_restated_total(i).unwrap_or(f64::NAN),
            1e-12,
        )
    });
    assert!(caught, "layer 3 failed to catch a whole-column swap");
}

// ------------------------------- layer 4: PHYSICS, not the printed page

#[test]
fn every_case_admits_a_physically_possible_electrolyser() {
    // Faradaic efficiency cannot exceed 1.0. The cell voltage fixes the
    // electricity per kilogram exactly, so a stack figure below what the voltage
    // requires would describe more hydrogen than the charge can make -- and all
    // three transcription layers pass on it, because they only ever compare
    // printed numbers with other printed numbers.
    for case in H2A_CASES {
        let (low, high) = case.faradaic_efficiency_band();
        assert!(
            case.physically_possible(0.90),
            "{}: implied Faradaic efficiency {low:.4}..{high:.4} admits nothing \
             between 0.90 and 1.0",
            case.name
        );
        assert!(
            low <= 1.0,
            "{}: even the most generous reading needs {low:.4} Faradaic efficiency",
            case.name
        );
    }
}

#[test]
fn the_physics_check_rejects_an_impossible_stack_figure() {
    // Non-vacuous: a stack drawing far less than its own cell voltage requires.
    // Both other in-column layers are untouched by this -- only physics objects.
    let mut impossible = H2A_CASES[0];
    impossible.stack_kwh_per_kg = Printed::new(40.0, 1);
    assert!(
        !impossible.physically_possible(0.90),
        "a stack drawing 40 kWh/kg at 1.9 V was accepted; that is {:.0}% Faradaic",
        100.0 * impossible.faradaic_efficiency_band().0
    );
    // ...and the other direction: an absurdly thirsty stack is not "impossible",
    // only inefficient, so the check must NOT reject it. A gate that rejects
    // everything is as useless as one that accepts everything.
    let mut thirsty = H2A_CASES[0];
    thirsty.stack_kwh_per_kg = Printed::new(70.0, 1);
    assert!(
        thirsty.physically_possible(0.70),
        "a thirsty but possible stack was rejected"
    );
}

#[test]
fn the_faraday_constant_is_physics_and_not_a_fitted_number() {
    // 2 electrons per H2, one kilogram is 1/M(H2) moles. Pinned so a silent
    // edit to either constant reddens rather than quietly rescaling the check.
    let case = current_distributed();
    let (low, high) = case.stack_energy_band_from_voltage();
    let midpoint = f64::midpoint(low, high);
    assert!(
        close(midpoint / case.cell_voltage_v.value(), 26.588_8, 1e-4),
        "kWh/kg per volt drifted to {:.4}",
        midpoint / case.cell_voltage_v.value()
    );
    assert!(
        low < case.stack_kwh_per_kg.value() && case.stack_kwh_per_kg.value() < high,
        "the printed stack figure {} falls outside the voltage band {low:.2}..{high:.2}",
        case.stack_kwh_per_kg.value()
    );
}

// ----------------------------------------------- the heating values themselves

#[test]
fn the_lower_heating_value_reconciles_across_its_two_printed_units() {
    assert!(
        AFDC_2026.lhv_printings_agree(),
        "51,585 Btu/lb converts to {:.4} kWh/kg, outside the printed 33.3",
        AFDC_2026.lhv_kwh_per_kg()
    );
    assert!(close(AFDC_2026.lhv_kwh_per_kg(), 33.329_642, 1e-6));
    assert!(close(AFDC_2026.hhv_kwh_per_kg(), 39.421_177, 1e-6));
}

#[test]
fn the_dual_unit_check_is_not_vacuous() {
    let mut misread = AFDC_2026;
    misread.lhv_btu_per_lb = Printed::new(52_585.0, 0); // one digit wrong
    assert!(
        !misread.lhv_printings_agree(),
        "a 1000 Btu/lb misreading survived the conversion check"
    );
}

#[test]
fn the_higher_heating_value_is_the_weakly_checked_figure_of_the_pair() {
    // Recorded asymmetry, asserted so it cannot quietly stop being true:
    // AFDC's LHV lands inside the band H2A's ratio rows imply; its HHV does not.
    let implied: Vec<(f64, f64)> = H2A_CASES
        .iter()
        .map(cf_electrolysis::H2aCase::implied_heating_values)
        .collect();
    let lhv_lo = implied.iter().map(|x| x.0).fold(f64::MAX, f64::min);
    let lhv_hi = implied.iter().map(|x| x.0).fold(f64::MIN, f64::max);
    let hhv_lo = implied.iter().map(|x| x.1).fold(f64::MAX, f64::min);
    let hhv_hi = implied.iter().map(|x| x.1).fold(f64::MIN, f64::max);

    let lhv = AFDC_2026.lhv_kwh_per_kg();
    let hhv = AFDC_2026.hhv_kwh_per_kg();
    assert!(
        lhv >= lhv_lo && lhv <= lhv_hi,
        "AFDC LHV {lhv:.4} should sit inside the implied {lhv_lo:.3}..{lhv_hi:.3}"
    );
    assert!(
        hhv > hhv_hi,
        "AFDC HHV {hhv:.4} should sit ABOVE the implied {hhv_lo:.3}..{hhv_hi:.3}; \
         if this stopped being true, HHV_SOURCE_SPREAD_PERCENT is now fiction"
    );
}

#[test]
fn the_heating_values_come_from_a_different_document_than_the_rows_that_check_them() {
    // The structural guarantee behind layer 2 being able to fail at all.
    assert_ne!(
        AFDC_2026.source.url, H2A_RECORD_19009.url,
        "the heating values and the rows that check them must not share a source"
    );
}

#[test]
fn the_recorded_source_spread_is_exactly_what_closes_the_efficiency_gap() {
    // Every case's printed HHV efficiency must be reproducible from AFDC's HHV
    // within its rounding interval WIDENED BY the recorded spread -- and one
    // case genuinely needs the widening, which is what makes the constant real
    // rather than decorative.
    let mut needed_widening = 0;
    for case in H2A_CASES {
        let computed = case.efficiency_hhv(&AFDC_2026) * 100.0;
        let printed = case.total_percent_hhv;
        let widen = HHV_SOURCE_SPREAD_PERCENT / 100.0 * printed.value();
        assert!(
            computed >= printed.low() - widen && computed <= printed.high() + widen,
            "{}: computed {computed:.4}% outside widened interval",
            case.name
        );
        if computed > printed.high() || computed < printed.low() {
            needed_widening += 1;
        }
    }
    assert_eq!(
        needed_widening, 1,
        "exactly one case should need the spread; if none does the constant is \
         unfalsifiable, and if more do the sources disagree by more than recorded"
    );
}

#[test]
fn the_lower_heating_value_efficiencies_need_no_widening_at_all() {
    for case in H2A_CASES {
        let computed = case.efficiency_lhv(&AFDC_2026) * 100.0;
        let printed = case.total_percent_lhv;
        assert!(
            computed >= printed.low() && computed <= printed.high(),
            "{}: LHV efficiency {computed:.4}% outside printed {}±{}",
            case.name,
            printed.value(),
            printed.half_ulp()
        );
    }
}

#[test]
fn the_heating_value_ratio_is_pinned() {
    let ratio = AFDC_2026.lhv_kwh_per_kg() / AFDC_2026.hhv_kwh_per_kg();
    assert!(
        close(ratio, 0.845_476, 1e-6),
        "LHV/HHV drifted to {ratio:.6}; an efficiency quoted on the wrong one is \
         out by 18%"
    );
}

// ------------------------------------------------------------ physics bounds

#[test]
fn hydrogen_never_carries_more_energy_than_the_electricity_that_made_it() {
    let case = current_distributed();
    let plant = FixedSpecificEnergy::from_case(&case, RATED_W, TURNDOWN);
    let r = annual_hydrogen(real_power_series(), &plant, YEAR.interval_seconds());
    let in_hydrogen = r.kg * AFDC_2026.lhv_kwh_per_kg();
    assert!(
        in_hydrogen < r.energy_converted_kwh,
        "{in_hydrogen:.0} kWh of hydrogen from {:.0} kWh of electricity",
        r.energy_converted_kwh
    );
    // ⚠ The inequality alone is far too loose to fail -- 57% against a 100%
    // ceiling. Pin the ratio, which is the case's own LHV efficiency and moves
    // the moment any term in the conversion does.
    let recovered = in_hydrogen / r.energy_converted_kwh;
    assert!(
        close(
            recovered,
            AFDC_2026.lhv_kwh_per_kg() / case.total_kwh_per_kg.value(),
            1e-12
        ),
        "energy recovered as hydrogen drifted to {recovered:.6}"
    );
}

#[test]
fn nothing_beats_the_thermodynamic_bound() {
    let case = current_distributed();
    let series = real_power_series();
    let real = FixedSpecificEnergy::from_case(&case, RATED_W, TURNDOWN);
    let bound = ThermodynamicBound::new(RATED_W, &AFDC_2026, case.outlet_pressure_bar);
    let r = annual_hydrogen(series.iter().copied(), &real, YEAR.interval_seconds());
    let b = annual_hydrogen(series.iter().copied(), &bound, YEAR.interval_seconds());
    assert!(b.kg > r.kg, "the bound produced less than a real machine");
    assert!(
        close(b.kg / r.kg, 1.727_596, 1e-5),
        "headroom to the bound drifted to {:.6}",
        b.kg / r.kg
    );
}

// ----------------------------------------------------- chain coupling gates

#[test]
fn the_energy_balance_is_exhaustive() {
    // ★★ Run at 400 kW, NOT at the pinned 1 MW. A plant matched to its turbine
    // curtails nothing, so at 1 MW this gate asserted a three-term identity with
    // one term pinned at zero -- deleting the curtailed term from the sum
    // entirely still passed it. All three must be live for the claim to mean
    // anything.
    let case = current_distributed();
    let plant = FixedSpecificEnergy::from_case(&case, 400e3, TURNDOWN);
    let series = real_power_series();
    let offered: f64 = series
        .iter()
        .map(|p| p * YEAR.interval_seconds() / 3.6e6)
        .sum();
    let r = annual_hydrogen(series.iter().copied(), &plant, YEAR.interval_seconds());

    assert!(r.energy_converted_kwh > 0.0, "conversion term must be live");
    assert!(
        r.energy_curtailed_kwh > 0.0,
        "curtailment term must be live"
    );
    assert!(
        r.energy_below_turndown_kwh > 0.0,
        "below-turndown term must be live"
    );
    assert!(
        close(r.energy_available_kwh(), offered, 1e-9),
        "converted + curtailed + below-turndown = {:.6} but {offered:.6} was offered",
        r.energy_available_kwh()
    );
}

#[test]
fn the_chain_runs_on_the_real_wind_year() {
    let case = current_distributed();
    let plant = FixedSpecificEnergy::from_case(&case, RATED_W, TURNDOWN);
    let r = annual_hydrogen(real_power_series(), &plant, YEAR.interval_seconds());
    assert_eq!(r.samples, 105_408, "the whole 2012 series must be consumed");
    assert_eq!(
        r.samples_not_finite, 0,
        "the real series must be clean, or the pinned figures below cover only part of it"
    );
    assert!(close(r.kg, 51_721.008, 1e-6), "kg drifted to {:.3}", r.kg);
    assert!(close(r.energy_converted_kwh, 2_886_032.246, 1e-6));
    assert!(close(r.energy_below_turndown_kwh, 92_071.231, 1e-6));
    assert_eq!(r.samples_below_turndown, 32_310);
    assert!(close(r.utilisation(), 0.969_084, 1e-6));
}

#[test]
fn a_plant_matched_to_its_turbine_never_curtails_and_that_is_not_a_dead_branch() {
    // Recorded rather than hidden: at 1 MW against a 1 MW turbine the clipping
    // branch never fires on the real series, so the pinned figures above do not
    // exercise it. `clipping_bites_once_the_plant_is_smaller_than_the_turbine`
    // is what proves that branch works.
    let case = current_distributed();
    let plant = FixedSpecificEnergy::from_case(&case, RATED_W, TURNDOWN);
    let r = annual_hydrogen(real_power_series(), &plant, YEAR.interval_seconds());
    assert_eq!(r.samples_at_rated, 0);
    assert!(close(r.energy_curtailed_kwh, 0.0, 1e-9));
}

#[test]
fn clipping_bites_once_the_plant_is_smaller_than_the_turbine() {
    let case = current_distributed();
    let plant = FixedSpecificEnergy::from_case(&case, 400e3, TURNDOWN);
    let r = annual_hydrogen(real_power_series(), &plant, YEAR.interval_seconds());
    assert_eq!(r.samples_at_rated, 39_513);
    assert!(r.energy_curtailed_kwh > 900_000.0);
    assert!(close(r.kg, 36_422.1, 1e-4), "kg drifted to {:.3}", r.kg);
}

#[test]
fn shrinking_the_plant_trades_curtailment_for_utilisation() {
    // The sweep the harness exists to run: every term moves monotonically and
    // in the direction the physics requires, across an order of magnitude.
    let case = current_distributed();
    let series = real_power_series();
    let mut last_curtailed = -1.0;
    let mut last_below = f64::MAX;
    let mut last_util = f64::MAX;
    for kw in [1000.0_f64, 800.0, 600.0, 400.0, 300.0, 200.0, 100.0, 50.0] {
        let plant = FixedSpecificEnergy::from_case(&case, kw * 1000.0, TURNDOWN);
        let r = annual_hydrogen(series.iter().copied(), &plant, YEAR.interval_seconds());
        assert!(
            r.energy_curtailed_kwh > last_curtailed,
            "curtailment must rise as the plant shrinks, at {kw} kW"
        );
        assert!(
            r.energy_below_turndown_kwh <= last_below,
            "below-turndown loss must fall as the plant shrinks, at {kw} kW"
        );
        assert!(
            r.utilisation() < last_util,
            "utilisation must fall as the plant shrinks, at {kw} kW"
        );
        last_curtailed = r.energy_curtailed_kwh;
        last_below = r.energy_below_turndown_kwh;
        last_util = r.utilisation();
    }
}

// ------------------------------------------- this stage's own nonlinearity

#[test]
fn with_both_clips_inactive_the_plant_is_exactly_linear() {
    // ★★★ The isolation that makes the next gate mean something. With no
    // turndown floor and a plant no smaller than its turbine, hydrogen is a
    // LINEAR function of power, so a flat series at the mean gives exactly the
    // same answer as the real one. Any difference is therefore caused by the
    // clipping and by nothing else.
    let case = current_distributed();
    let plant = FixedSpecificEnergy::from_case(&case, RATED_W, 0.0);
    let series = real_power_series();
    let mean = series.iter().sum::<f64>() / series.len() as f64;
    let flat = vec![mean; series.len()];

    let real = annual_hydrogen(series.iter().copied(), &plant, YEAR.interval_seconds());
    let flattened = annual_hydrogen(flat, &plant, YEAR.interval_seconds());
    assert_eq!(real.samples_at_rated, 0, "the upper clip must be inactive");
    assert_eq!(
        real.samples_below_turndown, 0,
        "the lower clip must be inactive"
    );
    assert!(
        close(flattened.kg, real.kg, 1e-9),
        "linear case disagreed: {:.6} vs {:.6}",
        flattened.kg,
        real.kg
    );
}

#[test]
fn a_flat_series_at_the_mean_overstates_a_clipped_plant() {
    // Turn one clip back on and the equality above breaks by a measured 3.19%,
    // in the optimistic direction. Small next to the wind stage's 40% cube
    // trap -- which is the sweep reporting that this stage's nonlinearity is
    // not the chain's weak term, rather than anyone deciding so in advance.
    let case = current_distributed();
    let plant = FixedSpecificEnergy::from_case(&case, RATED_W, TURNDOWN);
    let series = real_power_series();
    let mean = series.iter().sum::<f64>() / series.len() as f64;
    let flat = vec![mean; series.len()];

    let real = annual_hydrogen(series.iter().copied(), &plant, YEAR.interval_seconds());
    let flattened = annual_hydrogen(flat, &plant, YEAR.interval_seconds());
    let ratio = flattened.kg / real.kg;
    assert!(ratio > 1.0, "the flat estimate should be optimistic");
    assert!(
        close(ratio, 1.031_902, 1e-6),
        "the nonlinearity gap drifted to {ratio:.6}"
    );
}

// -------------------------------------------------------------- seam gates

#[test]
fn every_plant_parameter_changes_the_answer() {
    // ONE parameter at a time. Varying whole objects lets a pair of parameters
    // mask each other, which is exactly how three turbine parameters in the
    // wind stage ended up wired to nothing any gate could see.
    let base_case = current_distributed();
    let series = real_power_series();
    let run = |case: &_, rated, td| {
        annual_hydrogen(
            series.iter().copied(),
            &FixedSpecificEnergy::from_case(case, rated, td),
            YEAR.interval_seconds(),
        )
    };
    let base = run(&base_case, RATED_W, TURNDOWN);

    assert!(
        !close(run(&base_case, 600e3, TURNDOWN).kg, base.kg, 1e-9),
        "rated power is wired to nothing"
    );
    assert!(
        !close(run(&base_case, RATED_W, 0.25).kg, base.kg, 1e-9),
        "turndown is wired to nothing"
    );

    let mut thirstier = base_case;
    thirstier.total_kwh_per_kg = Printed::new(60.0, 1);
    assert!(
        !close(run(&thirstier, RATED_W, TURNDOWN).kg, base.kg, 1e-9),
        "specific energy is wired to nothing"
    );

    // Outlet pressure must NOT move the kilograms -- it is the boundary the
    // kilograms are true at. Asserting it changed `kg` would be asserting a bug.
    let mut higher = base_case;
    higher.outlet_pressure_bar = 700.0;
    let moved = run(&higher, RATED_W, TURNDOWN);
    assert!(
        close(moved.kg, base.kg, 1e-12),
        "pressure must not alter mass"
    );
    assert!(
        !close(moved.outlet_pressure_bar, base.outlet_pressure_bar, 1e-9),
        "outlet pressure is wired to nothing"
    );
}

#[test]
fn a_second_implementation_through_a_trait_object_gives_a_different_answer() {
    let case = current_distributed();
    let series = real_power_series();
    let fixed = FixedSpecificEnergy::from_case(&case, RATED_W, TURNDOWN);
    let bound = ThermodynamicBound::new(RATED_W, &AFDC_2026, case.outlet_pressure_bar);
    let plants: [&dyn Electrolyser; 2] = [&fixed, &bound];
    let kgs: Vec<f64> = plants
        .iter()
        .map(|p| annual_hydrogen(series.iter().copied(), *p, YEAR.interval_seconds()).kg)
        .collect();
    assert!(
        !close(kgs[0], kgs[1], 1e-9),
        "both implementations produced {:.3} kg -- the seam is decorative",
        kgs[0]
    );
    assert_ne!(plants[0].name(), plants[1].name());

    // ⚠ "They differ" is satisfied by dispatching to the wrong one. Each dyn
    // call must match that implementation's own direct call.
    let direct_fixed = annual_hydrogen(series.iter().copied(), &fixed, YEAR.interval_seconds()).kg;
    let direct_bound = annual_hydrogen(series.iter().copied(), &bound, YEAR.interval_seconds()).kg;
    assert!(
        close(kgs[0], direct_fixed, 1e-12),
        "dyn dispatch missed FixedSpecificEnergy"
    );
    assert!(
        close(kgs[1], direct_bound, 1e-12),
        "dyn dispatch missed ThermodynamicBound"
    );
}

// ------------------------------------------------------------- hygiene

#[test]
fn the_outlet_pressure_travels_with_the_kilograms() {
    // ★ Two different cases through the SAME code path. Asserting one literal
    // let a hardcoded `20.6843` in the constructor pass; two cases with
    // different outlet pressures cannot both be satisfied by any one constant.
    let series = real_power_series();
    let run = |case: &_| {
        annual_hydrogen(
            series.iter().copied(),
            &FixedSpecificEnergy::from_case(case, RATED_W, TURNDOWN),
            YEAR.interval_seconds(),
        )
        .outlet_pressure_bar
    };
    let current = run(&H2A_CASES[0]); // 300 psi
    let future = run(&H2A_CASES[1]); // 700 psi
    assert!(
        close(current, 20.684_3, 1e-4),
        "current drifted to {current:.4}"
    );
    assert!(
        close(future, 48.263_0, 1e-4),
        "future drifted to {future:.4}"
    );
    assert!(
        future > current * 2.0,
        "the two cases must be far apart, or one constant satisfies both"
    );
    assert!(
        current < 50.0 && future < 100.0,
        "both must stay far below storage pressure, or the caveat is wrong"
    );
}

#[test]
fn a_damaged_sample_cannot_reach_the_headline() {
    // ⛔⛔ A NaN used to propagate straight into the annual kilograms with no
    // diagnostic. An infinite sample was worse: the upper clip bounded it, so
    // the mass came back finite and plausible while the energy balance broke.
    let case = current_distributed();
    let plant = FixedSpecificEnergy::from_case(&case, RATED_W, TURNDOWN);
    let good = [400e3, 600e3, 900e3];

    let clean = annual_hydrogen(good, &plant, 300.0);
    assert_eq!(clean.samples_not_finite, 0);

    for (label, bad) in [
        ("NaN", f64::NAN),
        ("+inf", f64::INFINITY),
        ("-inf", f64::NEG_INFINITY),
    ] {
        let mut series = good.to_vec();
        series.push(bad);
        let r = annual_hydrogen(series, &plant, 300.0);
        assert_eq!(r.samples_not_finite, 1, "{label} was not counted");
        assert_eq!(
            r.samples, 4,
            "{label}: every sample is still counted in total"
        );
        assert!(r.kg.is_finite(), "{label} reached the kilograms");
        assert!(
            close(r.kg, clean.kg, 1e-12),
            "{label} changed the mass: {:.6} vs {:.6}",
            r.kg,
            clean.kg
        );
        assert!(
            close(
                r.energy_available_kwh(),
                clean.energy_available_kwh(),
                1e-12
            ),
            "{label} broke the energy balance"
        );
    }
}

#[test]
fn an_empty_series_produces_nothing_and_does_not_divide_by_zero() {
    let case = current_distributed();
    let plant = FixedSpecificEnergy::from_case(&case, RATED_W, TURNDOWN);
    let r = annual_hydrogen(std::iter::empty(), &plant, 300.0);
    assert_eq!(r.samples, 0);
    assert!(close(r.kg, 0.0, 1e-12));
    assert!(close(r.utilisation(), 0.0, 1e-12));
}

#[test]
fn printed_intervals_overlap_by_rounding_not_by_percentage() {
    let coarse = Printed::new(55.5, 1);
    let fine = Printed::new(55.46, 2);
    assert!(
        coarse.overlaps(&fine),
        "55.46 could have been printed as 55.5"
    );
    let far = Printed::new(55.3, 1);
    assert!(!coarse.overlaps(&far), "55.3 and 55.5 cannot be one number");
    assert!(close(coarse.half_ulp(), 0.05, 1e-12));
    assert!(close(fine.half_ulp(), 0.005, 1e-12));
}

#[test]
fn the_caveats_are_ranked_by_measured_effect() {
    // A flat list of caveats invites weighting by prose emphasis. These carry
    // their measured effect and are ordered by it, and the numbers are
    // recomputed here from the real chain rather than trusted as written.
    let base_case = current_distributed();
    let series = real_power_series();
    let kg_at = |specific: f64| {
        let mut c = base_case;
        c.total_kwh_per_kg = Printed::new(specific, 3);
        annual_hydrogen(
            series.iter().copied(),
            &FixedSpecificEnergy::from_case(&c, RATED_W, TURNDOWN),
            YEAR.interval_seconds(),
        )
        .kg
    };
    let base = kg_at(55.8);

    // Each alternative is the one named in that caveat's `why`, in the same
    // order. ★ The balance-of-plant figure is DERIVED from the record's own two
    // scales, not typed in, so it cannot drift back into being an assumption.
    let Some(scaling) = bop_scaling(&H2A_CASES[0], &H2A_CASES[2]) else {
        panic!("the two 2019 cases must be comparable")
    };
    let farm_bop = scaling.extrapolate_bop(CAVEAT_FARM_SCALE_KG_PER_DAY);
    let alternatives = [
        55.8 * 1.10,                                   // stack degradation over life
        55.8 + 1.923,                                  // compression to 700 bar
        55.8 * 0.97,                                   // part-load gain
        base_case.stack_kwh_per_kg.value() + farm_bop, // balance of plant at farm scale
    ];
    assert_eq!(alternatives.len(), NOT_A_FARM_PLANT.len());

    for (caveat, specific) in NOT_A_FARM_PLANT.iter().zip(alternatives) {
        let measured = 100.0 * (kg_at(specific) - base) / base;
        assert!(
            (measured - caveat.headline_swing_percent).abs() < 0.1,
            "{}: recorded {:+.1}% but recomputes to {measured:+.1}%",
            caveat.what,
            caveat.headline_swing_percent
        );
    }

    let mut previous = f64::MAX;
    for caveat in NOT_A_FARM_PLANT {
        let size = caveat.headline_swing_percent.abs();
        assert!(
            size <= previous,
            "{} breaks the largest-first ordering",
            caveat.what
        );
        previous = size;
    }
}

#[test]
fn the_balance_of_plant_swing_is_measured_from_the_record_not_assumed() {
    // ★★★ The record publishes two plant scales at one technology year with an
    // IDENTICAL stack figure, which is what makes balance of plant the only term
    // responding to size. This entry once read -16.2% from an assumed tripling
    // while the crate called the question uncheckable.
    let Some(scaling) = bop_scaling(&H2A_CASES[0], &H2A_CASES[2]) else {
        panic!("the two 2019 cases must be comparable")
    };
    assert!(close(scaling.decades(), 1.522_879, 1e-5));
    assert!(
        scaling.kwh_per_kg_per_decade() > 0.0,
        "balance of plant must RISE as the plant shrinks"
    );
    assert!(
        close(scaling.kwh_per_kg_per_decade(), 0.236_394, 1e-5),
        "measured slope drifted to {:.6}",
        scaling.kwh_per_kg_per_decade()
    );
    // Extrapolated two decades below the smallest published case.
    let farm_bop = scaling.extrapolate_bop(CAVEAT_FARM_SCALE_KG_PER_DAY);
    assert!(
        close(farm_bop, 5.872_789, 1e-5),
        "farm BoP drifted to {farm_bop:.6}"
    );
    assert!(
        farm_bop < 3.0 * H2A_CASES[0].bop_kwh_per_kg.value(),
        "the measured trend must stay far below the tripling this once assumed"
    );
}

#[test]
fn the_scaling_measurement_refuses_incomparable_cases() {
    // The guard is the whole validity of the slope: if the stack figure moved
    // too, balance of plant would not be the only term responding to scale.
    assert!(
        bop_scaling(&H2A_CASES[0], &H2A_CASES[1]).is_none(),
        "cases from different technology years must not be compared"
    );
    assert!(
        bop_scaling(&H2A_CASES[2], &H2A_CASES[0]).is_none(),
        "the larger plant must not be passed as the smaller"
    );
    let mut tampered = H2A_CASES[2];
    tampered.stack_kwh_per_kg = Printed::new(49.0, 1);
    assert!(
        bop_scaling(&H2A_CASES[0], &tampered).is_none(),
        "a moving stack figure must void the measurement"
    );

    // ⚠ No REAL pair isolates the technology-year guard: every cross-year pair
    // in the record also differs in stack figure, so the stack check rejects it
    // first and the year check never runs. Removing the year guard entirely left
    // every gate green. This synthetic pair is the only thing that exercises it.
    let mut future_twin = H2A_CASES[2];
    future_twin.technology_year = 2035;
    assert!(
        close(
            future_twin.stack_kwh_per_kg.value(),
            H2A_CASES[0].stack_kwh_per_kg.value(),
            1e-12
        ),
        "the twin must differ ONLY in year, or this proves nothing"
    );
    assert!(
        bop_scaling(&H2A_CASES[0], &future_twin).is_none(),
        "cases from different technology years must not be compared even when \
         their stack figures agree"
    );
}

#[test]
fn risks_without_a_measured_size_are_kept_out_of_the_ranked_list() {
    // ⛔ A ranked list places whatever you put in it. An unmeasured risk given a
    // zero would sort last, which is a claim about its size that nothing
    // supports -- the exact error this whole ranking exists to prevent.
    assert!(!UNMEASURED_AT_FARM_SCALE.is_empty());
    for u in UNMEASURED_AT_FARM_SCALE {
        assert!(!u.what.is_empty());
        assert!(
            !u.why_unmeasured.is_empty() && !u.what_would_measure_it.is_empty(),
            "{}: an unknown must say why, and what would settle it",
            u.what
        );
    }
    let knee = UNMEASURED_AT_FARM_SCALE
        .iter()
        .any(|u| u.what.contains("knee"));
    assert!(
        knee,
        "the balance-of-plant knee must be recorded as unmeasured"
    );
    let ranked: String = NOT_A_FARM_PLANT.iter().map(|c| c.what).collect();
    assert!(
        !ranked.contains("knee"),
        "the knee has no measured magnitude and must not appear in the ranked list"
    );
}

#[test]
fn every_source_records_its_terms_as_a_determination() {
    // ⛔ "It is a government document" is not a determination. NIST is a federal
    // agency whose Standard Reference Data IS copyright-asserted, so the general
    // rule has a loud exception and leaning on it silently is indistinguishable
    // from not having checked.
    for source in [AFDC_2026.source, H2A_RECORD_19009] {
        assert!(
            !source.terms.is_empty(),
            "{}: no terms recorded",
            source.document
        );
        assert!(
            source.terms.contains("17 U.S.C."),
            "{}: terms cite no authority, so they are an assumption: {}",
            source.document,
            source.terms
        );
        assert!(
            source.terms.contains("Checked"),
            "{}: terms carry no date of determination",
            source.document
        );
    }
}

#[test]
fn the_caveats_name_the_boundary_and_the_scale() {
    assert!(NOT_A_FARM_PLANT.len() >= 4);
    let joined: String = NOT_A_FARM_PLANT.iter().map(|c| c.what).collect();
    assert!(joined.contains("1,500 kg/day"), "plant scale must be named");
    assert!(
        joined.contains("bar"),
        "the pressure boundary must be named"
    );
    for c in NOT_A_FARM_PLANT {
        assert!(!c.what.is_empty() && !c.why.is_empty());
    }
}
