//! Electricity into hydrogen, at one farm, from one public record.
//!
//! This is stage 2 of the acres-per-season chain. Stage 1 (`cf-wind`) says how
//! many kilowatt-hours the wind carries at one real site; this crate says how
//! many kilograms of hydrogen those kilowatt-hours become.
//!
//! # ⛔⛔ The error this crate exists to make impossible
//!
//! **A kilogram of hydrogen is not a kilogram of hydrogen.** The figure every
//! electrolysis study publishes is energy per kilogram *at the electrolyser's
//! own outlet pressure* — here **300 psi, about 21 bar**. A tractor's tank is
//! at 350 or 700 bar. The compression between the two is real electricity that
//! this stage does **not** debit, and quoting this crate's output as "hydrogen
//! the farm can burn" overstates the farm by that margin.
//!
//! So the boundary is carried in the type. [`AnnualHydrogen`] has an
//! `outlet_pressure_bar` field and no constructor that omits it: the number
//! cannot be moved around without the pressure it is true at.
//!
//! # ⛔ The second error: which heating value
//!
//! An electrolyser quoted at "70% efficient" is 70% on the **higher** heating
//! value and about 60% on the **lower** one. The two differ by 18%, both are in
//! common use, and neither is wrong — but a chain that mixes them is. See
//! [`HeatingValues`], which carries both and states which one each efficiency
//! figure is on.
//!
//! # ★★ How the transcribed figures are checked
//!
//! Everything here is read off two scanned-or-rendered public documents, so the
//! same discipline `cf-nebraska` needed applies: **three structurally different
//! checks, because each is blind to what the others catch.**
//!
//! | layer | what it is | what it CANNOT see |
//! |---|---|---|
//! | [`H2aCase::components_reconcile`] | additive, within a row group: stack + `BoP` == total | a whole column read from the wrong case |
//! | [`H2aCase::implied_heating_values`] | multiplicative, within one row: value × %LHV recovers the LHV | the same wrong column, again |
//! | [`TABLE_5_RESTATED_TOTALS`] | the same four totals reprinted on a different page in a **different column order** | a figure mis-printed identically in both tables |
//!
//! ⚠ The third layer is the load-bearing one and the easiest to skip. A value
//! taken consistently from the wrong column satisfies the first two perfectly —
//! they are both computed *within* the column. Only a source that orders its
//! columns differently can catch it.
//!
//! ⛔ **The heating values are not derived from layer two.** They come from a
//! separate document ([`AFDC_2026`]); layer two is then an independent check
//! that can actually fail. Deriving them from the ratio rows and then checking
//! the ratio rows against them would be a check that is unable to fail, which
//! is worse than no check because it reads as one.

/// One International Table Btu, in joules. Exact by definition.
const BTU_IT_J: f64 = 1_055.055_852_62;
/// One avoirdupois pound, in kilograms. Exact by definition.
const LB_KG: f64 = 0.453_592_37;
/// Joules in a kilowatt-hour. Exact.
const KWH_J: f64 = 3.6e6;

/// Where a figure was read, precisely enough to read it again.
#[derive(Clone, Copy, Debug)]
pub struct Source {
    /// Publishing body and document title.
    pub document: &'static str,
    /// Where it lives.
    pub url: &'static str,
    /// ISO date it was retrieved.
    pub retrieved: &'static str,
}

/// A figure as the source printed it, with the precision it was printed to.
///
/// The decimals matter: two figures agree if their **rounding intervals
/// overlap**, which is not the same set as "within 0.5%". A value printed to
/// one decimal place carries ±0.05 of slack whatever its magnitude, so a
/// percentage tolerance is too tight on small figures and too loose on large
/// ones. `cf-nebraska` learned this on a table whose adjacent rows differed by
/// 0.17%.
#[derive(Clone, Copy, Debug)]
pub struct Printed {
    value: f64,
    decimals: u8,
}

impl Printed {
    /// A figure and the number of decimal places it was printed to.
    #[must_use]
    pub const fn new(value: f64, decimals: u8) -> Self {
        Self { value, decimals }
    }

    /// The figure as printed.
    #[must_use]
    pub const fn value(&self) -> f64 {
        self.value
    }

    /// Half of the last printed unit — how far the true value may lie either
    /// side of what is on the page.
    #[must_use]
    pub fn half_ulp(&self) -> f64 {
        0.5 * 10f64.powi(-i32::from(self.decimals))
    }

    /// Lowest value that would have been printed this way.
    #[must_use]
    pub fn low(&self) -> f64 {
        self.value - self.half_ulp()
    }

    /// Highest value that would have been printed this way.
    #[must_use]
    pub fn high(&self) -> f64 {
        self.value + self.half_ulp()
    }

    /// Whether two printed figures could be the same underlying number.
    #[must_use]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.low() <= other.high() && other.low() <= self.high()
    }
}

/// Hydrogen's two heating values, and the fact that sources disagree slightly.
///
/// ⚠ The disagreement recorded in [`HHV_SOURCE_SPREAD_PERCENT`] is real and
/// small. It is carried rather than resolved because resolving it would mean
/// picking a side on no evidence, and because a later silent edit to either
/// constant is exactly the drift a recorded spread catches.
#[derive(Clone, Copy, Debug)]
pub struct HeatingValues {
    /// Lower heating value as printed, Btu/lb — the higher-resolution figure.
    pub lhv_btu_per_lb: Printed,
    /// Higher heating value as printed, Btu/lb.
    pub hhv_btu_per_lb: Printed,
    /// The LHV *also* printed in kWh/kg by the same table, to fewer figures.
    ///
    /// ★ This is the within-document checksum: a misread digit in the Btu/lb
    /// figure does not survive conversion into agreement with this one.
    /// ⚠ The HHV has no such second printing, which makes it the weakly
    /// checked figure of the pair — and it is also the one that disagrees
    /// across sources. Both facts are recorded rather than smoothed.
    pub lhv_kwh_per_kg_printed: Printed,
    /// Where these came from.
    pub source: Source,
}

impl HeatingValues {
    /// Lower heating value, kWh/kg, converted from the Btu/lb figure.
    #[must_use]
    pub fn lhv_kwh_per_kg(&self) -> f64 {
        self.lhv_btu_per_lb.value * BTU_IT_J / LB_KG / KWH_J
    }

    /// Higher heating value, kWh/kg, converted from the Btu/lb figure.
    #[must_use]
    pub fn hhv_kwh_per_kg(&self) -> f64 {
        self.hhv_btu_per_lb.value * BTU_IT_J / LB_KG / KWH_J
    }

    /// Whether the converted LHV agrees with the same table's kWh/kg printing.
    ///
    /// ⚠ Asymmetric on purpose: the Btu/lb figure has five significant digits
    /// and the kWh/kg figure three, so the interval that must contain the
    /// converted value is the *coarse* one.
    #[must_use]
    pub fn lhv_printings_agree(&self) -> bool {
        let converted = self.lhv_kwh_per_kg();
        converted >= self.lhv_kwh_per_kg_printed.low()
            && converted <= self.lhv_kwh_per_kg_printed.high()
    }
}

/// DOE Alternative Fuels Data Center fuel property comparison.
///
/// Chosen over the H2A record's own ratio rows deliberately: see the crate
/// documentation on why a check must not validate its own input.
pub const AFDC_2026: HeatingValues = HeatingValues {
    lhv_btu_per_lb: Printed::new(51_585.0, 0),
    hhv_btu_per_lb: Printed::new(61_013.0, 0),
    lhv_kwh_per_kg_printed: Printed::new(33.3, 1),
    source: Source {
        document: "U.S. DOE Alternative Fuels Data Center, Fuel Properties Comparison",
        url: "https://afdc.energy.gov/fuels/properties",
        retrieved: "2026-09-18",
    },
};

/// How far the two sources' higher heating values sit apart, in percent.
///
/// Measured against the **Current Distributed** case's own ratio row, which is
/// the case this chain uses: 55.8 kWh/kg × 70.6% implies 39.395 kWh/kg, while
/// AFDC's 61,013 Btu/lb converts to 39.421. Across all four cases the implied
/// values span 39.372–39.405, so AFDC sits just above the whole band.
///
/// The physical reason is a convention difference in the condensation reference
/// state. The reason to record it rather than pick a side is that 0.07% is far
/// too small to notice by eye and far too large to be a typo.
///
/// ★ **This constant does real work.** The higher-heating-value efficiencies
/// cannot all be reproduced from AFDC's HHV within their printed rounding
/// intervals — Future Distributed misses by 0.045 percentage points — and this
/// spread is exactly what closes the gap. Shrink it and a test reddens; that is
/// the difference between a recorded disagreement and a fudge factor.
///
/// ⚠ The lower heating value has no such problem: AFDC's 33.330 sits inside
/// the implied 33.307–33.355 band. HHV is the weakly checked figure of the
/// pair, on both axes at once — it is also the one AFDC prints in only one unit.
pub const HHV_SOURCE_SPREAD_PERCENT: f64 = 0.067;

/// One case of the DOE H2A PEM electrolysis cost record.
///
/// ⚠ Fields are the figures **as printed**, in the table's own order. They were
/// read by structural position, never by matching magnitudes to expectations:
/// `cf-nebraska` had four figures silently taken from neighbouring rows that
/// way, every one of which then reconciled perfectly because each was a real
/// number from a real row.
#[derive(Clone, Copy, Debug)]
pub struct H2aCase {
    /// The case as the record names it.
    pub name: &'static str,
    /// Technology year the case represents.
    pub technology_year: u16,
    /// Plant scale, kg H2 per day.
    pub scale_kg_per_day: f64,
    /// Total electrical usage, kWh/kg — stack plus balance of plant.
    pub total_kwh_per_kg: Printed,
    /// Stack electrical usage, kWh/kg.
    pub stack_kwh_per_kg: Printed,
    /// Balance-of-plant electrical usage, kWh/kg.
    pub bop_kwh_per_kg: Printed,
    /// System efficiency on the lower heating value, as the record prints it.
    pub total_percent_lhv: Printed,
    /// System efficiency on the higher heating value, as the record prints it.
    pub total_percent_hhv: Printed,
    /// Pressure the hydrogen leaves the electrolyser at, bar.
    ///
    /// ⛔ **Not storage pressure.** See the crate documentation.
    pub outlet_pressure_bar: f64,
}

impl H2aCase {
    /// Layer one: does stack plus balance of plant reproduce the printed total?
    ///
    /// Compared as **rounding intervals**, since all three figures are rounded
    /// independently and their slacks add.
    #[must_use]
    pub fn components_reconcile(&self) -> bool {
        let sum = self.stack_kwh_per_kg.value + self.bop_kwh_per_kg.value;
        let slack = self.stack_kwh_per_kg.half_ulp() + self.bop_kwh_per_kg.half_ulp();
        sum - slack <= self.total_kwh_per_kg.high() && self.total_kwh_per_kg.low() <= sum + slack
    }

    /// Layer two: the heating values this row implies, kWh/kg, as (LHV, HHV).
    ///
    /// An efficiency is `heating_value / specific_energy`, so multiplying the
    /// printed efficiency back by the printed specific energy must return the
    /// heating value — a check that uses only figures from this one row.
    #[must_use]
    pub fn implied_heating_values(&self) -> (f64, f64) {
        (
            self.total_kwh_per_kg.value * self.total_percent_lhv.value / 100.0,
            self.total_kwh_per_kg.value * self.total_percent_hhv.value / 100.0,
        )
    }

    /// Efficiency on the lower heating value, computed rather than read.
    #[must_use]
    pub fn efficiency_lhv(&self, hv: &HeatingValues) -> f64 {
        hv.lhv_kwh_per_kg() / self.total_kwh_per_kg.value
    }

    /// Efficiency on the higher heating value, computed rather than read.
    #[must_use]
    pub fn efficiency_hhv(&self, hv: &HeatingValues) -> f64 {
        hv.hhv_kwh_per_kg() / self.total_kwh_per_kg.value
    }
}

/// Where the H2A figures were read.
pub const H2A_RECORD_19009: Source = Source {
    document: "DOE Hydrogen Program Record 19009, Hydrogen Production Cost From PEM Electrolysis (2019)",
    url: "https://www.hydrogen.energy.gov/docs/hydrogenprogramlibraries/pdfs/19009_h2_production_cost_pem_electrolysis_2019.pdf",
    retrieved: "2026-09-18",
};

/// 300 psi, the current cases' outlet pressure, in bar.
const PSI_300_BAR: f64 = 300.0 * 6_894.757_293_168_361 / 1e5;
/// 700 psi, the future cases' outlet pressure, in bar.
const PSI_700_BAR: f64 = 700.0 * 6_894.757_293_168_361 / 1e5;

/// The four cases of Table 2, in the record's own column order.
///
/// ⚠ Order is load-bearing: [`TABLE_5_RESTATED_TOTALS`] checks these against a
/// table that orders its columns differently, and that check only works if this
/// array preserves the order it was read in.
pub const H2A_CASES: &[H2aCase] = &[
    H2aCase {
        name: "Current Distributed",
        technology_year: 2019,
        scale_kg_per_day: 1_500.0,
        total_kwh_per_kg: Printed::new(55.8, 1),
        stack_kwh_per_kg: Printed::new(50.4, 1),
        bop_kwh_per_kg: Printed::new(5.4, 1),
        total_percent_lhv: Printed::new(59.7, 1),
        total_percent_hhv: Printed::new(70.6, 1),
        outlet_pressure_bar: PSI_300_BAR,
    },
    H2aCase {
        name: "Future Distributed",
        technology_year: 2035,
        scale_kg_per_day: 1_500.0,
        total_kwh_per_kg: Printed::new(51.4, 1),
        stack_kwh_per_kg: Printed::new(47.8, 1),
        bop_kwh_per_kg: Printed::new(3.66, 2),
        total_percent_lhv: Printed::new(64.8, 1),
        total_percent_hhv: Printed::new(76.6, 1),
        outlet_pressure_bar: PSI_700_BAR,
    },
    H2aCase {
        name: "Current Central",
        technology_year: 2019,
        scale_kg_per_day: 50_000.0,
        total_kwh_per_kg: Printed::new(55.5, 1),
        stack_kwh_per_kg: Printed::new(50.4, 1),
        bop_kwh_per_kg: Printed::new(5.04, 2),
        total_percent_lhv: Printed::new(60.1, 1),
        total_percent_hhv: Printed::new(71.0, 1),
        outlet_pressure_bar: PSI_300_BAR,
    },
    H2aCase {
        name: "Future Central",
        technology_year: 2035,
        scale_kg_per_day: 50_000.0,
        total_kwh_per_kg: Printed::new(51.3, 1),
        stack_kwh_per_kg: Printed::new(47.8, 1),
        bop_kwh_per_kg: Printed::new(3.54, 2),
        total_percent_lhv: Printed::new(65.0, 1),
        total_percent_hhv: Printed::new(76.8, 1),
        outlet_pressure_bar: PSI_700_BAR,
    },
];

/// The same four totals, reprinted by Table 5 of the same record.
///
/// ★★★ **This is the only layer that catches a consistently wrong column.**
/// Table 5 compares the 2014 and 2019 studies and therefore interleaves its
/// columns as Distributed-2014, Distributed-2019, Central-2014, Central-2019
/// within each of Current and Future — a different order from Table 2. A figure
/// taken from the wrong Table 2 column reconciles under both other layers,
/// because both are computed inside that column; it lands on a different case
/// here and is caught.
///
/// Indexed to match [`H2A_CASES`].
pub const TABLE_5_RESTATED_TOTALS: &[f64] = &[55.8, 51.4, 55.5, 51.3];

/// The Current Distributed case: 2019 technology, the honest "today" figure.
///
/// ⚠ Chosen over the Future cases because a chain built on 2035 projections
/// answers a question nobody asked. Its limits are in [`NOT_A_FARM_PLANT`].
#[must_use]
pub fn current_distributed() -> H2aCase {
    H2A_CASES[0]
}

/// Something that turns electrical power into hydrogen.
///
/// The seam. Every stage of this chain gets one, including the stages nobody
/// currently suspects — a seam placed only at the term you already believe
/// dominates can do nothing but confirm you.
pub trait Electrolyser {
    /// Nameplate electrical input, watts.
    fn rated_power_w(&self) -> f64;
    /// Lowest fraction of rated power the stack will run at.
    ///
    /// Below this it shuts down, and the energy is unusable rather than merely
    /// inefficient — which is why it is a separate term in [`AnnualHydrogen`].
    fn min_load_fraction(&self) -> f64;
    /// Electricity per kilogram at a given fraction of rated power, kWh/kg.
    fn specific_energy_kwh_per_kg(&self, load_fraction: f64) -> f64;
    /// Pressure the hydrogen leaves at, bar.
    fn outlet_pressure_bar(&self) -> f64;
    /// What to call it.
    fn name(&self) -> &'static str;
}

/// An electrolyser whose specific energy does not vary with load.
///
/// ⚠ **A deliberate simplification, and a flat one.** A real stack is *more*
/// efficient at part load, because cell voltage falls with current density —
/// the record's own 1.9 V at 2.0 A/cm² is one point on a curve this type
/// replaces with a horizontal line. Modelling the curve needs a polarisation
/// dataset this crate does not have, so the flat line is used and named, rather
/// than a shape being invented to look more sophisticated.
///
/// The direction of the resulting error is known even though its size is not:
/// a wind-driven plant spends much of its year at part load, where the real
/// machine does better than this one. **This type therefore understates
/// production.** It is the conservative side, which is the correct side to be
/// wrong on for a feasibility claim.
#[derive(Clone, Copy, Debug)]
pub struct FixedSpecificEnergy {
    name: &'static str,
    rated_power_w: f64,
    min_load_fraction: f64,
    specific_energy_kwh_per_kg: f64,
    outlet_pressure_bar: f64,
}

impl FixedSpecificEnergy {
    /// Build one from a published case, at a chosen plant size.
    ///
    /// `min_load_fraction` is a plant-integration choice, not a figure from the
    /// record — the record sizes for steady operation and never states a
    /// turndown limit. It is an argument rather than a constant so that it
    /// shows up in a sweep instead of hiding in the arithmetic.
    #[must_use]
    pub const fn from_case(case: &H2aCase, rated_power_w: f64, min_load_fraction: f64) -> Self {
        Self {
            name: case.name,
            rated_power_w,
            min_load_fraction,
            specific_energy_kwh_per_kg: case.total_kwh_per_kg.value,
            outlet_pressure_bar: case.outlet_pressure_bar,
        }
    }
}

impl Electrolyser for FixedSpecificEnergy {
    fn rated_power_w(&self) -> f64 {
        self.rated_power_w
    }
    fn min_load_fraction(&self) -> f64 {
        self.min_load_fraction
    }
    fn specific_energy_kwh_per_kg(&self, _load_fraction: f64) -> f64 {
        self.specific_energy_kwh_per_kg
    }
    fn outlet_pressure_bar(&self) -> f64 {
        self.outlet_pressure_bar
    }
    fn name(&self) -> &'static str {
        self.name
    }
}

/// The thermodynamic ceiling: every kilowatt-hour becomes hydrogen at its LHV.
///
/// ⛔ **Not a machine, and not a target.** No electrolyser reaches this and none
/// ever will; it exists so the chain has a bound it cannot cross, and so a
/// second implementation exercises the [`Electrolyser`] seam with different
/// code rather than the same code holding different numbers.
///
/// It runs from zero load and clips only at rated power.
#[derive(Clone, Copy, Debug)]
pub struct ThermodynamicBound {
    rated_power_w: f64,
    lhv_kwh_per_kg: f64,
    outlet_pressure_bar: f64,
}

impl ThermodynamicBound {
    /// The bound at a chosen plant size, on a stated heating value.
    #[must_use]
    pub fn new(rated_power_w: f64, hv: &HeatingValues, outlet_pressure_bar: f64) -> Self {
        Self {
            rated_power_w,
            lhv_kwh_per_kg: hv.lhv_kwh_per_kg(),
            outlet_pressure_bar,
        }
    }
}

impl Electrolyser for ThermodynamicBound {
    fn rated_power_w(&self) -> f64 {
        self.rated_power_w
    }
    fn min_load_fraction(&self) -> f64 {
        0.0
    }
    fn specific_energy_kwh_per_kg(&self, _load_fraction: f64) -> f64 {
        self.lhv_kwh_per_kg
    }
    fn outlet_pressure_bar(&self) -> f64 {
        self.outlet_pressure_bar
    }
    fn name(&self) -> &'static str {
        "thermodynamic bound (LHV, not a machine)"
    }
}

/// What a plant made of a year's electricity, and what it did not.
///
/// ⚠ The three energy terms are exhaustive and are asserted to sum to the
/// energy offered. A loss that is not one of these three is a loss the model
/// cannot represent, and the balance failing is how that would announce itself.
#[derive(Clone, Copy, Debug)]
pub struct AnnualHydrogen {
    /// Hydrogen produced over the year, kg.
    pub kg: f64,
    /// ⛔ Pressure that hydrogen is at, bar. **Not storage pressure.**
    pub outlet_pressure_bar: f64,
    /// Energy that became hydrogen, kWh.
    pub energy_converted_kwh: f64,
    /// Energy offered above rated power and therefore spilled, kWh.
    pub energy_curtailed_kwh: f64,
    /// Energy offered below the turndown limit, with the stack off, kWh.
    pub energy_below_turndown_kwh: f64,
    /// Intervals the stack spent shut down for want of power.
    pub samples_below_turndown: usize,
    /// Intervals the plant spent clipped at rated power.
    pub samples_at_rated: usize,
    /// Intervals counted in total.
    pub samples: usize,
}

impl AnnualHydrogen {
    /// Energy offered by the source over the year, kWh.
    #[must_use]
    pub fn energy_available_kwh(&self) -> f64 {
        self.energy_converted_kwh + self.energy_curtailed_kwh + self.energy_below_turndown_kwh
    }

    /// Fraction of offered energy that became hydrogen.
    #[must_use]
    pub fn utilisation(&self) -> f64 {
        let available = self.energy_available_kwh();
        if available > 0.0 {
            self.energy_converted_kwh / available
        } else {
            0.0
        }
    }
}

/// Run a plant against a series of electrical power samples.
///
/// `power_w` is whatever the stage upstream produced, sampled at a fixed
/// interval; this crate does not know or care that it came from wind.
///
/// ⚠⚠ **This is not `mean(power)` put through the same arithmetic**, and the
/// difference is this stage's version of the trap that sinks stage 1. The plant
/// is clipped at both ends — nothing below turndown, nothing above rated — so
/// it is a **nonlinear** function of the input, and a nonlinear function of an
/// average is not the average of the function. The size *and sign* of that gap
/// depend on the distribution and must be measured, never assumed.
#[must_use]
pub fn annual_hydrogen<E, I>(power_w: I, plant: &E, interval_seconds: f64) -> AnnualHydrogen
where
    E: Electrolyser + ?Sized,
    I: IntoIterator<Item = f64>,
{
    let rated = plant.rated_power_w();
    let floor = rated * plant.min_load_fraction();

    let mut kg = 0.0;
    let mut converted_kwh = 0.0;
    let mut curtailed_kwh = 0.0;
    let mut below_kwh = 0.0;
    let (mut n_below, mut n_rated, mut n) = (0usize, 0usize, 0usize);

    for p in power_w {
        n += 1;
        let offered_kwh = p * interval_seconds / KWH_J;

        if p < floor {
            n_below += 1;
            below_kwh += offered_kwh;
            continue;
        }

        let used_w = if p > rated {
            n_rated += 1;
            rated
        } else {
            p
        };
        let used_kwh = used_w * interval_seconds / KWH_J;
        curtailed_kwh += offered_kwh - used_kwh;
        converted_kwh += used_kwh;

        let load_fraction = if rated > 0.0 { used_w / rated } else { 0.0 };
        let specific = plant.specific_energy_kwh_per_kg(load_fraction);
        if specific > 0.0 {
            kg += used_kwh / specific;
        }
    }

    AnnualHydrogen {
        kg,
        outlet_pressure_bar: plant.outlet_pressure_bar(),
        energy_converted_kwh: converted_kwh,
        energy_curtailed_kwh: curtailed_kwh,
        energy_below_turndown_kwh: below_kwh,
        samples_below_turndown: n_below,
        samples_at_rated: n_rated,
        samples: n,
    }
}

/// Something true about these numbers that the numbers themselves do not say.
///
/// ★★ **Every caveat carries how far it moves the headline**, because a flat
/// list of worries is unreadable and invites the reader to weight them by the
/// emphasis of their prose rather than their size. The figures are measured by
/// `the_caveats_are_ranked_by_measured_effect`, not estimated here.
#[derive(Clone, Copy, Debug)]
pub struct Caveat {
    /// Short name.
    pub what: &'static str,
    /// Why it matters to the chain.
    pub why: &'static str,
    /// How far this term moves the annual kilograms, percent, signed.
    ///
    /// ⚠ Not an error bar. It is the effect of one plausible alternative
    /// assumption, stated in `why`, so that terms can be ranked against each
    /// other instead of all reading as equally alarming.
    pub headline_swing_percent: f64,
}

/// ⚠ What this stage's figures are not, **largest effect first**.
///
/// ⛔ **Do not confuse these with the transcription findings.** The
/// disagreement recorded in [`HHV_SOURCE_SPREAD_PERCENT`] is a question about
/// whether the source says what this crate thinks it says; it moves the annual
/// kilograms by **exactly zero**, because the kilograms depend on specific
/// energy and never on a heating value. Integrity checks and sensitivity terms
/// are different categories and a reader who sees them in one list will weight
/// them wrongly.
///
/// ★ The largest term overall is **upstream**: inter-annual wind variability
/// swings the headline about ±10.6%, which is more than everything here except
/// plant scale. See `cf_wind::NOT_MEASURED_HERE`.
pub const NOT_A_FARM_PLANT: &[Caveat] = &[
    Caveat {
        what: "the plant is 1,500 kg/day; a farm needs single-digit kg/day",
        why: "H2A's smallest published PEM case is a fuelling station roughly two \
              orders of magnitude larger than one farm. Balance of plant does not \
              scale down linearly, so the 5.4 kWh/kg BoP term is the figure most \
              likely to be optimistic here; the stack term is far less sensitive. \
              Swing shown is BoP tripling to 16.2 kWh/kg.",
        headline_swing_percent: -16.2,
    },
    Caveat {
        what: "no degradation over stack life",
        why: "The record carries a stack degradation rate of 1.5 mV/khr, which \
              raises specific energy as the stack ages. A single year at beginning \
              of life is the best case of the plant's service life. Swing shown is \
              a 10% rise in specific energy.",
        headline_swing_percent: -9.1,
    },
    Caveat {
        what: "outlet is about 21 bar, not storage pressure",
        why: "Compression from the outlet to a tractor's tank is unmodelled \
              electricity that will reduce these kilograms. Smaller than it looks: \
              an ideal-gas multi-stage estimate is 1.6 kWh/kg to 350 bar and 1.9 to \
              700, against a 55.8 kWh/kg input. The real-gas figure is the next \
              increment; this is an order of magnitude, not a result.",
        headline_swing_percent: -3.3,
    },
    Caveat {
        what: "specific energy does not vary with load",
        why: "A real stack does better at part load, which is where a wind-driven \
              plant spends most of its year. This is the one term that moves the \
              answer UP, so the model is conservative in direction and unmeasured \
              in size. Swing shown is a 3% fall in specific energy.",
        headline_swing_percent: 3.1,
    },
];
