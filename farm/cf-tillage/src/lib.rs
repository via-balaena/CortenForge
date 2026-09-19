//! What one farm's fall tillage demands of its own hydrogen.
//!
//! This is stage 4 of the acres-per-season chain. Stage 1 (`cf-wind`) says how
//! many kilowatt-hours the wind carries at one real site; stage 2
//! (`cf-electrolysis`) turns those into kilograms; stage 3 (`cf-storage`)
//! compresses them into a tank and locates the **storage cliff** — the point
//! where a seasonal draw stops being servable out of production during the
//! window itself. This crate builds the draw, and so decides which side of that
//! cliff the farm lands on.
//!
//! # ★★★ The answer: the bar the season sets, and what it is half of
//!
//! The headline of this crate is not a kilogram count. It is
//! [`break_even_engine_efficiency`]: **what thermal efficiency a hydrogen
//! engine would have to reach** for fall tillage to run on the hydrogen the
//! farm makes *during the tillage window itself*. That inverts the one term
//! nobody can measure yet into the one term the answer is stated in, so no
//! headline here rests on an invented engine.
//!
//! [`HYDROGEN_ENGINE_EFFICIENCY_NEEDED`] is **14.4% to 19.1%**.
//!
//! ⛔⛔ **This crate does not say whether an engine reaches that**, and the
//! temptation to add a sentence claiming it does is the reason this paragraph
//! is written the way it is. There is no hydrogen-engine oracle anywhere in
//! `farm/`; [`UNMEASURED_HERE`] records its absence, and an unsourced sentence
//! asserting that real engines clear the bar would be exactly the transcribed
//! figure the swept-parameter discipline exists to refuse.
//!
//! What can be said with a measurement behind it is a **ratio to the engine
//! this tractor actually has**: the break-even is
//! [`BREAK_EVEN_AS_SHARE_OF_MEASURED_DIESEL_PERCENT`] — **39.9% to 52.8%** of
//! the brake thermal efficiency Nebraska measured for the 8245R's own diesel.
//! So the season closes if a hydrogen engine reaches a little over half what
//! the diesel beside it was measured doing. Whether one does is the reader's
//! call, on evidence this crate does not hold.
//!
//! [`SEASON_MARGIN_AT_FORTY_PERCENT`] gives the same result as a surplus,
//! **2.1× to 2.8×**, at an efficiency that is *stated as an illustration*
//! rather than claimed.
//!
//! # ⛔⛔ The correction that produced that number
//!
//! The first version of this crate compared the window's demand against stage
//! 3's published cliff of 2,695 kg. That comparison was wrong **twice over**,
//! and the second one only surfaced when a review of this PR forced the figure
//! to be recomputed instead of quoted:
//!
//! 1. **A different window.** 2,695 kg is production across stage 3's own
//!    illustrative **21-day** window; the windows measured here run five to
//!    eight weeks.
//! 2. **A different side of the compression debit.** It is production at the
//!    electrolyser *outlet*, before stage 3 charges compression against it.
//!    This crate measures delivered kilograms. The same 21 days after the debit
//!    hold about 2,610 kg — a 3.3% gap that a quoted number hides completely.
//!
//! **Demand from one window against production from another** is the
//! denominator error this chain keeps generating, and it made the load factor
//! look as though it straddled the cliff when it does not. Both halves are now
//! recomputed by `stage_threes_own_illustration_is_what_the_correction_says_it_is`,
//! so the narrative cannot rot when stage 3 moves.
//!
//! Recomputed consistently — the cliff re-derived for *every* window — the
//! picture inverts: see [`WINDOW_RULE_CANCELS_WITHIN_PP`]. Demand and in-window
//! production both scale with the window, so they very nearly cancel.
//!
//! # ★★ The window is a count of operable days, not a span of calendar
//!
//! USDA NASS publishes no fall-tillage progress series — checked, not assumed
//! (see `NASS_VALIDATION.md`). What it publishes is
//! **`FIELDWORK - DAYS SUITABLE`**, weekly, which is the better input: a farm
//! tills on days it can, and acres per season scales with that count directly.
//!
//! ⛔⛔ **And the window rule reverses which years look good.** Measured over
//! the complete years this crate carries, 2012 — the year `cf-wind` holds — is
//! one of the worst falls on a fixed Oct 1 – Nov 25 calendar and one of the
//! best once the window may open when harvest clears. The calendar view holds
//! the start date constant when the start date is exactly what varies. So
//! [`WindowRule`] is a parameter this crate sweeps, never a constant it picks —
//! see [`WINDOW_RULE_REVERSES_THE_RANKING`].
//!
//! ⚠⚠ **That reversal barely moves the break-even.** Which years had the most
//! workable days, and how good an engine the season needs, are two different
//! questions; reading the first as evidence about the second is exactly the
//! error described above. Both results are kept because the pair is the
//! lesson.
//!
//! # What is measured, what is swept
//!
//! | term | source | status |
//! |---|---|---|
//! | PTO and drawbar power, fuel economy | Nebraska summary 963, via `cf_nebraska` | **measured** |
//! | drivetrain and slip | the same test, **two independent routes** | **measured**, and they agree |
//! | load factor | [`EPA_NR005C`] | **a band**, because the source prints two |
//! | operable days | [`NASS_CROP_PROGRESS`] | **measured** |
//! | hydrogen and diesel heating values | [`AFDC_FUEL_PROPERTIES`] | **measured** |
//! | hydrogen engine thermal efficiency | — | ⚠ **swept, never transcribed** |
//! | hours worked per operable day | — | ⚠ **swept, never transcribed** |
//!
//! ⚠ The last two rows are the discipline this crate is most likely to be
//! judged on. NONROAD's brake-specific fuel consumption lives in a **draft**
//! companion built on 1988 Tier 0 engines, and hydrogen combustion efficiency
//! is genuinely unsettled. A swept parameter with a stated range is honest; a
//! transcribed draft figure would look like a measurement and would not be one.
//! Neither has a default anywhere in this crate — the caller must supply both,
//! so no headline can quietly acquire an invented input.

use std::sync::OnceLock;

/// Where a figure was read, precisely enough to read it again — and on what
/// terms.
///
/// The same shape `cf_electrolysis` and `cf_storage` carry. ⚠ [`Source::terms`]
/// is **determined, not assumed**: stage 3 found that "a government document,
/// therefore free" is false for NIST, so every source in the chain states its
/// own status rather than inheriting one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Source {
    /// Title as the publisher prints it.
    pub document: &'static str,
    /// Where it was retrieved.
    pub url: &'static str,
    /// ISO date of retrieval.
    pub retrieved: &'static str,
    /// What the publisher's terms permit.
    pub terms: &'static str,
}

/// A work of the U.S. federal government: no copyright, 17 U.S.C. §105.
pub const US_GOV_PUBLIC_DOMAIN: &str =
    "U.S. Government work, not subject to copyright in the United States (17 U.S.C. \u{a7}105)";

/// USDA NASS crop progress and fieldwork, the operating window.
pub const NASS_CROP_PROGRESS: Source = Source {
    document: "USDA NASS Quick Stats, crops: North Dakota weekly fieldwork and harvest progress",
    url: "https://www.nass.usda.gov/datasets/qs.crops_20260919.txt.gz",
    retrieved: "2026-09-19",
    terms: US_GOV_PUBLIC_DOMAIN,
};

/// EPA NR-005c, the load factor and its two printings.
pub const EPA_NR005C: Source = Source {
    document: "EPA420-P-04-005, Median Life, Annual Activity, and Load Factor Values for \
               Nonroad Engine Emissions Modeling (NR-005c), April 2004",
    url: "https://usace.contentdm.oclc.org/digital/api/collection/p16021coll7/id/558/download",
    retrieved: "2026-09-19",
    terms: US_GOV_PUBLIC_DOMAIN,
};

/// DOE AFDC fuel properties, the heating values on both sides of the swap.
///
/// ⚠ The same document `cf_electrolysis::AFDC_2026` reads. Deliberately: the
/// diesel a tractor burns and the hydrogen replacing it are then compared
/// **inside one table**, so a heating-value convention cannot differ between
/// the two halves of the comparison.
pub const AFDC_FUEL_PROPERTIES: Source = Source {
    document: "U.S. DOE Alternative Fuels Data Center, Fuel Properties Comparison",
    url: "https://afdc.energy.gov/fuels/properties",
    retrieved: "2026-09-19",
    terms: US_GOV_PUBLIC_DOMAIN,
};

/// The Nebraska OECD tractor test, reached through `cf_nebraska`.
///
/// ⚠ Its terms are **NOT DETERMINED**, and the determination is not repeated
/// here — it is [`cf_nebraska::TERMS_NOT_DETERMINED`], stated once at the
/// source that carries the documents. A second copy would be a second thing to
/// keep true.
pub const NEBRASKA_TRACTOR_TEST: Source = Source {
    document: "Nebraska Tractor Test Laboratory, OECD summary 963, John Deere 8245R (2014)",
    url: "https://tractortestlab.unl.edu/",
    retrieved: "2026-09-18",
    terms: cf_nebraska::TERMS_NOT_DETERMINED,
};

/// A figure as a document printed it, with the precision it was printed to.
///
/// The primitive `cf_storage` established: comparison is by **rounding-interval
/// overlap**, never by a percentage tolerance invented at the call site. A
/// figure printed `0.78` asserts only that the quantity lies in
/// `[0.775, 0.785]`.
#[derive(Clone, Copy, Debug)]
pub struct Printed {
    value: f64,
    decimals: u8,
}

impl Printed {
    /// A figure printed as `value` to `decimals` places.
    #[must_use]
    pub const fn new(value: f64, decimals: u8) -> Self {
        Self { value, decimals }
    }

    /// The figure as printed.
    #[must_use]
    pub const fn value(self) -> f64 {
        self.value
    }

    /// How many decimals it was printed to.
    #[must_use]
    pub const fn decimals(self) -> u8 {
        self.decimals
    }

    /// Half the last printed digit: the radius of the rounding interval.
    #[must_use]
    pub fn half_ulp(self) -> f64 {
        0.5 * 10f64.powi(-i32::from(self.decimals))
    }

    /// Lower edge of the interval this printing admits.
    #[must_use]
    pub fn low(self) -> f64 {
        self.value - self.half_ulp()
    }

    /// Upper edge of the interval this printing admits.
    #[must_use]
    pub fn high(self) -> f64 {
        self.value + self.half_ulp()
    }

    /// Whether `other` could have been rounded to this printing.
    #[must_use]
    pub fn admits(self, other: f64) -> bool {
        other >= self.low() && other <= self.high()
    }

    /// Whether two printings could describe the same quantity.
    #[must_use]
    pub fn overlaps(self, other: Self) -> bool {
        self.low() <= other.high() && other.low() <= self.high()
    }
}

/// One British thermal unit (IT) in joules.
const BTU_IT_J: f64 = 1_055.055_852_62;
/// One horsepower-hour in joules: 745.699 871 582 27 W × 3600 s.
const HP_HR_J: f64 = 745.699_871_582_270_2 * 3600.0;
/// One kilowatt-hour in joules.
const KWH_J: f64 = 3.6e6;
/// One US liquid gallon in litres.
const GAL_L: f64 = 3.785_411_784;

/// One horsepower-hour expressed in Btu (IT).
///
/// Not transcribed: derived from the two SI definitions above, so the Nebraska
/// `hp·hr/gal` printings and the AFDC `Btu/gal` printing meet without a third
/// document's rounding in between.
#[must_use]
pub fn btu_per_hp_hr() -> f64 {
    HP_HR_J / BTU_IT_J
}

/// Diesel lower heating value, Btu per US gallon, as AFDC prints it.
pub const DIESEL_LHV_BTU_PER_GAL: Printed = Printed::new(128_488.0, 0);

/// Hydrogen lower heating value, kWh/kg, as AFDC prints it.
///
/// ⚠ The same figure `cf_electrolysis::AFDC_2026` carries, to the same one
/// decimal. It is repeated rather than imported because this crate does not
/// depend on stage 2 — and `the_heating_value_matches_stage_two` fails if the
/// two ever drift apart.
pub const HYDROGEN_LHV_KWH_PER_KG: Printed = Printed::new(33.3, 1);

/// Diesel lower heating value in kWh per litre, from the AFDC printing.
#[must_use]
pub fn diesel_lhv_kwh_per_l() -> f64 {
    DIESEL_LHV_BTU_PER_GAL.value() * BTU_IT_J / GAL_L / KWH_J
}

/// One `SwRI` transient cycle and the load factor it measured.
#[derive(Clone, Copy, Debug)]
pub struct Cycle {
    /// Equipment type the cycle was developed for, as NR-005c names it.
    pub equipment: &'static str,
    /// The load factor the cycle measured.
    pub load_factor: Printed,
}

/// The four cycles NR-005c puts in the High bin.
///
/// ★ The agricultural tractor is the **highest of all seven cycles EPA ran**,
/// which is exactly why binning costs it so much.
pub const HIGH_LF_CYCLES: &[Cycle] = &[
    Cycle {
        equipment: "agricultural tractor",
        load_factor: Printed::new(0.78, 2),
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

/// The three cycles NR-005c puts in the Low bin.
pub const LOW_LF_CYCLES: &[Cycle] = &[
    Cycle {
        equipment: "backhoe/loader",
        load_factor: Printed::new(0.21, 2),
    },
    Cycle {
        equipment: "skid-steer loader",
        load_factor: Printed::new(0.23, 2),
    },
    Cycle {
        equipment: "arc welder",
        load_factor: Printed::new(0.19, 2),
    },
];

/// A bin of cycles and the composite load factor NR-005c prints for it.
#[derive(Clone, Copy, Debug)]
pub struct Bin {
    /// The bin's name in the document.
    pub name: &'static str,
    /// The composite the document prints.
    pub composite: Printed,
    /// The cycles averaged to obtain it.
    pub cycles: &'static [Cycle],
}

impl Bin {
    /// The mean of this bin's own cycle load factors.
    ///
    /// Returns `None` for an empty roster rather than dividing by zero — the
    /// blind spot `cf_storage` found in its multiplicative layer, closed here
    /// at the type level instead of being rediscovered.
    #[must_use]
    pub fn mean_of_cycles(&self) -> Option<f64> {
        if self.cycles.is_empty() {
            return None;
        }
        let n = self.cycles.len();
        // NR-005c defines at most seven cycles, so the count is exact in f64.
        #[allow(
            clippy::cast_precision_loss,
            reason = "a roster of at most seven cycles is exact in f64"
        )]
        let n = n as f64;
        Some(
            self.cycles
                .iter()
                .fold(0.0, |acc, c| acc + c.load_factor.value())
                / n,
        )
    }

    /// Whether the mean of the cycles reproduces the printed composite.
    ///
    /// ★★ This is the document's **own checksum**, and it is what makes
    /// NR-005c usable the way Nebraska was: the binning is arithmetic stated in
    /// the text, so a mistranscribed cycle stops reconciling.
    #[must_use]
    pub fn reconciles(&self) -> bool {
        self.mean_of_cycles()
            .is_some_and(|mean| self.composite.admits(mean))
    }
}

/// The High bin: composite 0.59, from four measured cycles.
pub const HIGH_LF_BIN: Bin = Bin {
    name: "High",
    composite: Printed::new(0.59, 2),
    cycles: HIGH_LF_CYCLES,
};

/// The Low bin: composite 0.21, from three measured cycles.
pub const LOW_LF_BIN: Bin = Bin {
    name: "Low",
    composite: Printed::new(0.21, 2),
    cycles: LOW_LF_CYCLES,
};

/// Every bin NR-005c defines by averaging, for the roster checks.
///
/// ⚠ The seven-cycle "steady-state" composite (0.43) is deliberately **not**
/// here: its roster is the union of the other two, so including it would let
/// one check pass twice and look like two.
pub const AVERAGED_BINS: &[Bin] = &[HIGH_LF_BIN, LOW_LF_BIN];

/// The agricultural tractor's load factor as the `SwRI` cycle **measured** it.
pub const AGRICULTURAL_TRACTOR_MEASURED_LF: Printed = Printed::new(0.78, 2);

/// The load factor NONROAD **applies** to diesel agricultural tractors.
///
/// ⛔ Not a different tractor and not a correction — the same machine, binned.
/// SCC 2270005015, `Diesel Agricultural Tractors`, in the table of values the
/// model actually runs on.
pub const AGRICULTURAL_TRACTOR_APPLIED_LF: Printed = Printed::new(0.59, 2);

/// Annual hours NR-005c assigns to diesel agricultural tractors, SCC 2270005015.
///
/// ⛔ **Carried for provenance, and deliberately not used to build the window.**
/// It is a national figure over a population NR-005c describes as *"typically
/// in the 50 to 150 HP range"*; this farm's tractor is 215.88 PTO horsepower,
/// and its window is measured at its own site by [`NASS_CROP_PROGRESS`]. Using
/// 475 here would swap a site measurement for a national average of the wrong
/// size class.
pub const AGRICULTURAL_TRACTOR_ANNUAL_HOURS: u32 = 475;

/// What binning costs the agricultural tractor, percent.
///
/// `0.59 / 0.78 - 1`, the largest such gap in the document: the tractor
/// measured highest of the seven and is applied at its bin's mean. Pinned by
/// `the_binning_haircut_is_the_measured_one`.
pub const LOAD_FACTOR_HAIRCUT_PERCENT: f64 = -24.4;

/// The load factor band this crate carries: `(applied, measured)`.
///
/// ⛔⛔ **Carry the spread, do not resolve it.** The document prints both and
/// justifies both; picking one and calling it the load factor would turn an
/// editorial choice into an apparent measurement.
#[must_use]
pub const fn load_factor_band() -> (f64, f64) {
    (
        AGRICULTURAL_TRACTOR_APPLIED_LF.value(),
        AGRICULTURAL_TRACTOR_MEASURED_LF.value(),
    )
}

/// The committed NASS extract: North Dakota, state level, weeks from 25 August.
///
/// 17 U.S.C. §105 material, so unlike stage 3's NIST isotherms it **is**
/// committed rather than retrieved. `NASS_VALIDATION.md` records the query that
/// produced it and the digest below.
const NASS_TSV: &str = include_str!("../data/nd_fall_fieldwork.tsv");

/// SHA-256 of the committed extract.
///
/// ⚠ Pinned because a data file crossing a squash-merge is exactly what stage 1
/// learned to check on `main` rather than on a branch.
pub const NASS_TSV_SHA256: &str =
    "99375f514fd18c09b53c8933e5bbfe71347bb64a767cd4b548b59c57e11d5615";

/// Which weekly series an observation belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Series {
    /// `FIELDWORK - DAYS SUITABLE, MEASURED IN DAYS / WEEK`.
    DaysSuitable,
    /// `SOYBEANS - PROGRESS, MEASURED IN PCT HARVESTED`.
    Soybeans,
    /// `CORN - PROGRESS, MEASURED IN PCT HARVESTED`.
    Corn,
}

impl Series {
    /// The code this series carries in the committed extract.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::DaysSuitable => "DS",
            Self::Soybeans => "SOY",
            Self::Corn => "CORN",
        }
    }

    /// The series' full NASS name, for reporting what a number is a number of.
    #[must_use]
    pub const fn nass_name(self) -> &'static str {
        match self {
            Self::DaysSuitable => "FIELDWORK - DAYS SUITABLE, MEASURED IN DAYS / WEEK",
            Self::Soybeans => "SOYBEANS - PROGRESS, MEASURED IN PCT HARVESTED",
            Self::Corn => "CORN - PROGRESS, MEASURED IN PCT HARVESTED",
        }
    }

    /// The code as the extract spells it, back to a series.
    #[must_use]
    pub fn from_code(code: &str) -> Option<Self> {
        match code {
            "DS" => Some(Self::DaysSuitable),
            "SOY" => Some(Self::Soybeans),
            "CORN" => Some(Self::Corn),
            _ => None,
        }
    }
}

/// One published weekly figure.
#[derive(Clone, Copy, Debug)]
pub struct Observation {
    /// Marketing year NASS files the week under.
    pub year: u16,
    /// Week ending date, `YYYY-MM-DD`, as NASS prints it.
    pub week_ending: &'static str,
    /// Which series.
    pub series: Series,
    /// The published value.
    pub value: f64,
}

impl Observation {
    /// The `MM-DD` part of the week-ending date, for comparing across years.
    #[must_use]
    pub fn month_day(&self) -> &'static str {
        self.week_ending.get(5..).unwrap_or("")
    }
}

/// One extract line to an observation, or `None` if it cannot be read.
///
/// ⛔ Public so the **rejection** paths are reachable from a test. The
/// committed extract parses cleanly by construction, so every failure branch
/// here would otherwise be unexercised — and an unexercised guard is a guard
/// nobody has seen work.
#[must_use]
pub fn parse_line(line: &'static str) -> Option<Observation> {
    let mut f = line.split('\t');
    let year: u16 = f.next()?.parse().ok()?;
    let week_ending = f.next()?;
    let series = Series::from_code(f.next()?)?;
    let value: f64 = f.next()?.parse().ok()?;
    if f.next().is_some() || week_ending.len() != 10 {
        return None;
    }
    Some(Observation {
        year,
        week_ending,
        series,
        value,
    })
}

struct Extract {
    rows: Vec<Observation>,
    unparsed: usize,
}

fn extract() -> &'static Extract {
    static EXTRACT: OnceLock<Extract> = OnceLock::new();
    EXTRACT.get_or_init(|| {
        let mut rows = Vec::new();
        let mut unparsed = 0usize;
        for line in NASS_TSV.lines() {
            if line.starts_with('#') || line.starts_with("year\t") || line.is_empty() {
                continue;
            }
            match parse_line(line) {
                Some(o) => rows.push(o),
                None => unparsed += 1,
            }
        }
        rows.sort_by(|a, b| {
            a.week_ending
                .cmp(b.week_ending)
                .then(a.series.code().cmp(b.series.code()))
        });
        Extract { rows, unparsed }
    })
}

/// Every observation in the committed extract, in week order.
#[must_use]
pub fn observations() -> &'static [Observation] {
    &extract().rows
}

/// Lines the parser could not read.
///
/// ⛔⛔ **An absence claim is a hypothesis.** A parser that silently drops rows
/// reports a clean, short, wrong series; this counts what it dropped so
/// `the_extract_parses_completely` can assert zero against a known row count
/// rather than against "it looked fine".
#[must_use]
pub fn unparsed_lines() -> usize {
    extract().unparsed
}

/// Every year present in the extract, ascending.
#[must_use]
pub fn years() -> Vec<u16> {
    let mut ys: Vec<u16> = observations().iter().map(|o| o.year).collect();
    ys.sort_unstable();
    ys.dedup();
    ys
}

/// The first `MM-DD` of the comparison span used to judge a year complete.
pub const COMPLETENESS_FROM: &str = "10-01";
/// The last `MM-DD` of that span.
pub const COMPLETENESS_TO: &str = "11-25";
/// How many fieldwork weeks a year must publish inside it to be usable.
///
/// ⛔⛔ **2025 is why this exists.** It publishes fieldwork through late
/// November, but is missing every week from 28 September to 16 November — the
/// whole fall. Summed without this gate it reads as a catastrophic season
/// rather than as an absent one, which is the difference between a measurement
/// and an artefact. `an_incomplete_year_is_rejected_not_averaged` fires on it.
pub const COMPLETE_YEAR_MIN_WEEKS: usize = 8;

/// One year's observations, grouped.
#[derive(Clone, Debug)]
pub struct FallYear {
    /// The year.
    pub year: u16,
    rows: Vec<Observation>,
}

impl FallYear {
    /// Every observation for `year`, or `None` if the extract has none.
    #[must_use]
    pub fn get(year: u16) -> Option<Self> {
        let rows: Vec<Observation> = observations()
            .iter()
            .filter(|o| o.year == year)
            .copied()
            .collect();
        if rows.is_empty() {
            return None;
        }
        Some(Self { year, rows })
    }

    /// This year's observations for one series, in week order.
    #[must_use]
    pub fn series(&self, series: Series) -> Vec<Observation> {
        self.rows
            .iter()
            .filter(|o| o.series == series)
            .copied()
            .collect()
    }

    /// Fieldwork weeks published inside the completeness span.
    #[must_use]
    pub fn weeks_in_completeness_span(&self) -> usize {
        self.series(Series::DaysSuitable)
            .iter()
            .filter(|o| o.month_day() >= COMPLETENESS_FROM && o.month_day() <= COMPLETENESS_TO)
            .count()
    }

    /// Whether this year published enough of the fall to be used.
    #[must_use]
    pub fn is_complete(&self) -> bool {
        self.weeks_in_completeness_span() >= COMPLETE_YEAR_MIN_WEEKS
    }

    /// The first week in which `crop` reached `percent` harvested.
    #[must_use]
    pub fn harvest_reached(&self, crop: Series, percent: f64) -> Option<&'static str> {
        self.series(crop)
            .iter()
            .find(|o| o.value >= percent)
            .map(|o| o.week_ending)
    }

    /// Apply a window rule to this year.
    ///
    /// Returns `None` if the rule never opens — which is a real outcome, not an
    /// error: corn does not reach 90% statewide in every North Dakota year.
    #[must_use]
    pub fn window(&self, rule: WindowRule) -> Option<Window> {
        let days = self.series(Series::DaysSuitable);
        let (opens_after, counted): (Option<&'static str>, Vec<&Observation>) = match rule {
            WindowRule::FixedCalendar { from, to } => (
                None,
                days.iter()
                    .filter(|o| o.month_day() >= from && o.month_day() <= to)
                    .collect(),
            ),
            WindowRule::HarvestGated { crop, percent } => {
                let opens = self.harvest_reached(crop, percent)?;
                (
                    Some(opens),
                    days.iter().filter(|o| o.week_ending > opens).collect(),
                )
            }
        };
        Some(Window {
            year: self.year,
            rule,
            opens_after,
            // ⚠ NOT `.sum()`. Rust's `Sum for f64` folds from `-0.0`, so an
            // empty window returns NEGATIVE zero: it prints as `-0.00` and
            // `total_cmp` sorts it below `+0.0`. Two real years reach here
            // with no weeks at all. See `an_empty_window_is_positive_zero`.
            operable_days: counted.iter().fold(0.0, |acc, o| acc + o.value),
            weeks_counted: counted.len(),
            complete: self.is_complete(),
        })
    }
}

/// How the operating window is allowed to open.
///
/// ⛔⛔ **Not a detail.** Which of these is used reverses 2012's rank among its
/// own neighbours — see [`WINDOW_RULE_REVERSES_THE_RANKING`]. A fixed calendar
/// holds the start date constant across years when the start date is the thing
/// that varies most.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WindowRule {
    /// A fixed `MM-DD` span, the same every year.
    FixedCalendar {
        /// First `MM-DD` counted.
        from: &'static str,
        /// Last `MM-DD` counted.
        to: &'static str,
    },
    /// Opens the week after `crop` first reaches `percent` harvested, and runs
    /// to the last week NASS publishes.
    HarvestGated {
        /// Which harvest clears the ground.
        crop: Series,
        /// The percent-harvested threshold.
        percent: f64,
    },
}

impl WindowRule {
    /// A short name, so a number always says which rule produced it.
    #[must_use]
    pub fn name(&self) -> String {
        match self {
            Self::FixedCalendar { from, to } => format!("fixed calendar {from} to {to}"),
            Self::HarvestGated { crop, percent } => {
                format!("opens at {} {percent:.0}% harvested", crop.code())
            }
        }
    }
}

/// The fixed-calendar rule used for the completeness span and the comparison.
pub const FIXED_OCT_NOV: WindowRule = WindowRule::FixedCalendar {
    from: COMPLETENESS_FROM,
    to: COMPLETENESS_TO,
};

/// Opens when soybeans are 90% harvested: the rule a corn-soybean rotation
/// tilling bean stubble would actually experience.
pub const AFTER_SOYBEANS: WindowRule = WindowRule::HarvestGated {
    crop: Series::Soybeans,
    percent: 90.0,
};

/// Opens when corn is 90% harvested: the latest-opening rule, and the one that
/// does not open at all in some years.
pub const AFTER_CORN: WindowRule = WindowRule::HarvestGated {
    crop: Series::Corn,
    percent: 90.0,
};

/// The three rules this crate sweeps.
pub const SWEPT_RULES: &[WindowRule] = &[FIXED_OCT_NOV, AFTER_SOYBEANS, AFTER_CORN];

/// One year's operating window under one rule.
#[derive(Clone, Copy, Debug)]
pub struct Window {
    /// The year.
    pub year: u16,
    /// The rule that produced it.
    pub rule: WindowRule,
    /// The week after which the window opened, for a harvest-gated rule.
    pub opens_after: Option<&'static str>,
    /// Days suitable for fieldwork inside the window.
    pub operable_days: f64,
    /// How many published weeks contributed.
    pub weeks_counted: usize,
    /// Whether the year published enough of the fall to be trusted.
    pub complete: bool,
}

/// Operable days across the complete years, under one rule.
#[derive(Clone, Debug)]
pub struct Climatology {
    /// The rule.
    pub rule: WindowRule,
    /// Years that contributed, ascending.
    pub years: Vec<u16>,
    /// Years rejected as incomplete.
    pub rejected_incomplete: Vec<u16>,
    /// Years where the rule never opened.
    pub rejected_never_opened: Vec<u16>,
    /// Mean operable days.
    pub mean_days: f64,
    /// Median operable days.
    pub median_days: f64,
    /// Fewest operable days in any contributing year.
    pub min_days: f64,
    /// Most operable days in any contributing year.
    pub max_days: f64,
}

impl Climatology {
    /// Where `year` ranks among the contributing years, 1 = fewest days.
    #[must_use]
    pub fn rank_of(&self, year: u16) -> Option<(usize, usize)> {
        let mut days: Vec<(u16, f64)> = self
            .years
            .iter()
            .filter_map(|&y| {
                FallYear::get(y)?
                    .window(self.rule)
                    .map(|w| (y, w.operable_days))
            })
            .collect();
        days.sort_by(|a, b| a.1.total_cmp(&b.1));
        let n = days.len();
        days.iter()
            .position(|&(y, _)| y == year)
            .map(|i| (i + 1, n))
    }
}

/// Operable days across every complete year, under one rule.
///
/// Returns `None` if no year survives both filters.
#[must_use]
pub fn climatology(rule: WindowRule) -> Option<Climatology> {
    let mut contributing: Vec<(u16, f64)> = Vec::new();
    let mut rejected_incomplete = Vec::new();
    let mut rejected_never_opened = Vec::new();
    for y in years() {
        let Some(fy) = FallYear::get(y) else { continue };
        if !fy.is_complete() {
            rejected_incomplete.push(y);
            continue;
        }
        match fy.window(rule) {
            Some(w) => contributing.push((y, w.operable_days)),
            None => rejected_never_opened.push(y),
        }
    }
    if contributing.is_empty() {
        return None;
    }
    let mut vals: Vec<f64> = contributing.iter().map(|&(_, d)| d).collect();
    vals.sort_by(f64::total_cmp);
    let n = vals.len();
    // NASS has published this series since 1981, so the year count is small
    // and exact in f64.
    #[allow(
        clippy::cast_precision_loss,
        reason = "a few tens of years is exact in f64"
    )]
    let count = n as f64;
    // ⚠ Odd count takes the middle element; even takes the midpoint of the two
    // middle ones. Written the other way round once, and min <= median <= max
    // still passed - `the_median_is_the_middle_value` is the gate that does not.
    let median = if n.is_multiple_of(2) {
        f64::midpoint(vals[n / 2 - 1], vals[n / 2])
    } else {
        vals[n / 2]
    };
    Some(Climatology {
        rule,
        years: contributing.iter().map(|&(y, _)| y).collect(),
        rejected_incomplete,
        rejected_never_opened,
        mean_days: vals.iter().fold(0.0, |acc, v| acc + v) / count,
        median_days: median,
        min_days: vals[0],
        max_days: vals[n - 1],
    })
}

/// The tractor, read live from the crate that reconciles the scans.
#[derive(Clone, Copy, Debug)]
pub struct Tractor {
    /// Make and model as Nebraska prints it.
    pub make_model: &'static str,
    /// Nebraska summary number.
    pub summary_no: u32,
    /// Maximum PTO power, kW.
    pub pto_kw: f64,
    /// Fuel rate at maximum PTO power, L/h.
    pub pto_fuel_l_per_h: f64,
    /// Drawbar power at 100% load, kW.
    pub drawbar_kw: f64,
    /// Drawbar fuel economy at 100% load, kW·h/L.
    pub drawbar_kwh_per_l: f64,
}

/// Row labels this crate reads out of `cf_nebraska`.
///
/// ⚠ Named as constants so a typo is a compile error at one site rather than a
/// `None` at four.
pub const NEEDED_ROWS: &[&str] = &[
    "pto max power",
    "pto max power fuel rate",
    "drawbar 100pct load power",
    "drawbar 100pct load fuel economy",
];

/// The John Deere 8245R, as Nebraska summary 963 measured it.
///
/// Returns `None` if any needed row fails to reconcile across the editions —
/// so a corrupt scan propagates as an absence here instead of a plausible
/// number. All values are SI, which is what `cf_nebraska::Reading` yields.
#[must_use]
pub fn john_deere_8245r() -> Option<Tractor> {
    let t = &cf_nebraska::JOHN_DEERE_8245R;
    Some(Tractor {
        make_model: t.make_model,
        summary_no: t.summary_no,
        pto_kw: t.get("pto max power")?.agreed()?,
        pto_fuel_l_per_h: t.get("pto max power fuel rate")?.agreed()?,
        drawbar_kw: t.get("drawbar 100pct load power")?.agreed()?,
        drawbar_kwh_per_l: t.get("drawbar 100pct load fuel economy")?.agreed()?,
    })
}

impl Tractor {
    /// Fuel economy at maximum PTO power, kW·h/L.
    #[must_use]
    pub fn pto_kwh_per_l(&self) -> f64 {
        self.pto_kw / self.pto_fuel_l_per_h
    }

    /// Measured brake thermal efficiency at the PTO, as a fraction.
    ///
    /// ★ The chain's sanity check. A 2014 Tier-4 agricultural diesel belongs in
    /// the mid-thirties; a conversion error anywhere between Nebraska's
    /// `hp·hr/gal`, AFDC's `Btu/gal` and SI shows up here as a number that is
    /// obviously wrong rather than as a headline that is quietly wrong.
    #[must_use]
    pub fn pto_thermal_efficiency(&self) -> f64 {
        self.pto_kwh_per_l() / diesel_lhv_kwh_per_l()
    }

    /// Measured brake thermal efficiency at the drawbar, as a fraction.
    #[must_use]
    pub fn drawbar_thermal_efficiency(&self) -> f64 {
        self.drawbar_kwh_per_l / diesel_lhv_kwh_per_l()
    }

    /// Drivetrain and slip, from the two fuel economies.
    #[must_use]
    pub fn drivetrain_from_fuel(&self) -> f64 {
        self.drawbar_kwh_per_l / self.pto_kwh_per_l()
    }

    /// Drivetrain and slip, from the two powers.
    ///
    /// ★★ A genuinely independent route to the same quantity: this one never
    /// touches a fuel figure, so a mistranscribed fuel economy moves
    /// [`Tractor::drivetrain_from_fuel`] and leaves this one alone.
    #[must_use]
    pub fn drivetrain_from_power(&self) -> f64 {
        self.drawbar_kw / self.pto_kw
    }
}

/// How far apart the two drivetrain routes land, percentage points.
///
/// Measured, and pinned by `the_two_drivetrain_routes_agree`. ⚠ They are not
/// expected to be identical: the two Nebraska runs sit at different operating
/// points. What matters is that they agree far more closely than any error this
/// crate would care about.
///
/// ⚠ The measured gap is 0.175 pp, and most of it is **printing, not physics**:
/// `cf_nebraska` yields SI, and Nebraska prints drawbar fuel economy as
/// `3.31 kW·h/L` — three significant figures where the US column's
/// `16.82 hp·hr/gal` carries four. The SI route therefore inherits the coarser
/// rounding. Below anything this crate resolves, and recorded rather than
/// tuned away.
pub const DRIVETRAIN_ROUTES_AGREE_WITHIN_PP: f64 = 0.18;

/// The tractor's measured PTO thermal efficiency, percent.
pub const DIESEL_PTO_THERMAL_EFFICIENCY_PERCENT: Printed = Printed::new(36.11, 2);

/// A season of work: how long the tractor can run, and how hard.
#[derive(Clone, Copy, Debug)]
pub struct Season {
    /// Days suitable for fieldwork in the window, from NASS.
    pub operable_days: f64,
    /// Hours worked per operable day. ⚠ **Swept — no default exists.**
    pub hours_per_operable_day: f64,
    /// Engine load factor over the cycle. From [`load_factor_band`].
    pub load_factor: f64,
}

impl Season {
    /// Total engine hours.
    #[must_use]
    pub fn hours(&self) -> f64 {
        self.operable_days * self.hours_per_operable_day
    }

    /// Shaft energy delivered over the season, kWh.
    ///
    /// Rated PTO power × load factor × hours. ⚠ **Check what the fraction is a
    /// fraction of**: NR-005c's load factor is a fraction of *rated engine
    /// power averaged over a duty cycle*, which is why it multiplies the PTO
    /// rating here. Nebraska's own "75% load" rows are a fraction of *maximum
    /// drawbar pull* — a different fraction of a different thing, and not
    /// interchangeable with this one.
    #[must_use]
    pub fn shaft_kwh(&self, tractor: &Tractor) -> f64 {
        tractor.pto_kw * self.load_factor * self.hours()
    }
}

/// Diesel the season would burn, litres, at the tractor's measured efficiency.
///
/// The baseline the hydrogen replaces. Returns `None` on a non-finite or
/// non-positive season.
#[must_use]
pub fn season_diesel_litres(tractor: &Tractor, season: &Season) -> Option<f64> {
    let shaft = season.shaft_kwh(tractor);
    if !shaft.is_finite() || shaft < 0.0 {
        return None;
    }
    let eff = tractor.pto_thermal_efficiency();
    if !eff.is_finite() || eff <= 0.0 {
        return None;
    }
    Some(shaft / eff / diesel_lhv_kwh_per_l())
}

/// Hydrogen the season would need, kilograms.
///
/// `engine_efficiency` is the hydrogen engine's brake thermal efficiency as a
/// fraction. ⚠ **It has no default anywhere in this crate.** Returns `None`
/// unless it is a finite fraction strictly inside `(0, 1]`.
#[must_use]
pub fn season_hydrogen_kg(
    tractor: &Tractor,
    season: &Season,
    engine_efficiency: f64,
    h2_lhv_kwh_per_kg: f64,
) -> Option<f64> {
    if !engine_efficiency.is_finite() || engine_efficiency <= 0.0 || engine_efficiency > 1.0 {
        return None;
    }
    if !h2_lhv_kwh_per_kg.is_finite() || h2_lhv_kwh_per_kg <= 0.0 {
        return None;
    }
    let shaft = season.shaft_kwh(tractor);
    if !shaft.is_finite() || shaft < 0.0 {
        return None;
    }
    Some(shaft / engine_efficiency / h2_lhv_kwh_per_kg)
}

/// ★★★ The headline: what a hydrogen engine would have to reach.
///
/// Solves [`season_hydrogen_kg`] for the efficiency at which the season's
/// demand exactly equals `available_kg`. Above this number the season closes on
/// that hydrogen; below it, it does not.
///
/// ⇒ This is the crate's answer **because it is the one form that needs no
/// invented input**. Every other output would require assuming the very engine
/// efficiency that nobody can yet measure; this one reports what that
/// efficiency would have to be, and leaves the judgement where it belongs.
///
/// Returns `None` if the inputs are not finite and positive, or if the required
/// efficiency exceeds 1 — which is itself an answer, and a decisive one: no
/// engine closes that season.
#[must_use]
pub fn break_even_engine_efficiency(
    tractor: &Tractor,
    season: &Season,
    available_kg: f64,
    h2_lhv_kwh_per_kg: f64,
) -> Option<f64> {
    if !available_kg.is_finite() || available_kg <= 0.0 {
        return None;
    }
    if !h2_lhv_kwh_per_kg.is_finite() || h2_lhv_kwh_per_kg <= 0.0 {
        return None;
    }
    let shaft = season.shaft_kwh(tractor);
    if !shaft.is_finite() || shaft <= 0.0 {
        return None;
    }
    let needed = shaft / (available_kg * h2_lhv_kwh_per_kg);
    (needed.is_finite() && needed > 0.0 && needed <= 1.0).then_some(needed)
}

/// Day of the year, 1-based, for a `YYYY-MM-DD` date.
///
/// Returns `None` on anything it cannot read, including a date that does not
/// exist. Leap years are handled by the full Gregorian rule.
#[must_use]
pub fn day_of_year(date: &str) -> Option<u16> {
    let mut parts = date.split('-');
    let y: u16 = parts.next()?.parse().ok()?;
    let m: u8 = parts.next()?.parse().ok()?;
    let d: u8 = parts.next()?.parse().ok()?;
    if parts.next().is_some() || m == 0 || m > 12 || d == 0 {
        return None;
    }
    let leap = (y.is_multiple_of(4) && !y.is_multiple_of(100)) || y.is_multiple_of(400);
    let lengths = [
        31u16,
        if leap { 29 } else { 28 },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    let month = usize::from(m) - 1;
    if u16::from(d) > lengths[month] {
        return None;
    }
    Some(lengths[..month].iter().sum::<u16>() + u16::from(d))
}

/// How many days one NASS reporting week covers.
pub const NASS_WEEK_DAYS: u16 = 7;

/// The season's draw, shaped by when the ground is actually workable.
///
/// ★★ Why this is not [`cf_storage::SeasonalDemand`]. That profile spreads a
/// total evenly across a window, which is the right foil. A real tillage season
/// is not even: NASS publishes days suitable **per week**, and the tractor runs
/// when they occur. So this distributes the season's kilograms in proportion to
/// each week's operable days — the buffer is then sized against the draw the
/// weather actually shapes, rather than against its average.
#[derive(Clone, Debug)]
pub struct TillageDemand {
    samples: usize,
    total_kg: f64,
    /// `(first_sample, last_sample_exclusive, kg_per_sample)`, one per week.
    weeks: Vec<(usize, usize, f64)>,
}

impl TillageDemand {
    /// Build the draw for one year's window.
    ///
    /// `total_kg` is spread across the window's weeks in proportion to each
    /// week's days suitable, then evenly across the samples of the seven days
    /// that week covers.
    ///
    /// Returns `None` if the interval is not positive and finite, if the window
    /// has no operable days, or if any week falls outside `samples`.
    #[must_use]
    pub fn new(
        year: u16,
        window: &Window,
        total_kg: f64,
        samples: usize,
        interval_seconds: f64,
    ) -> Option<Self> {
        if !interval_seconds.is_finite() || interval_seconds <= 0.0 {
            return None;
        }
        if !total_kg.is_finite() || total_kg < 0.0 {
            return None;
        }
        let fy = FallYear::get(year)?;
        let days = fy.series(Series::DaysSuitable);
        let in_window: Vec<&Observation> = days
            .iter()
            .filter(|o| match window.rule {
                WindowRule::FixedCalendar { from, to } => {
                    o.month_day() >= from && o.month_day() <= to
                }
                WindowRule::HarvestGated { .. } => window
                    .opens_after
                    .is_some_and(|opens| o.week_ending > opens),
            })
            .collect();
        // Folded from `+0.0` for the same reason as [`FallYear::window`].
        let total_days: f64 = in_window.iter().fold(0.0, |acc, o| acc + o.value);
        if !total_days.is_finite() || total_days <= 0.0 {
            return None;
        }
        // ⚠ `per_day` may be infinite, when a denormal interval overflows the
        // division. That is deliberately NOT guarded here: the sample indices
        // then saturate to `usize::MAX` and the window bound below rejects
        // them. Two earlier versions guarded it twice over, and mutation
        // testing showed neither guard changed any outcome — a check that
        // cannot fail reads like protection and is not.
        let per_day = 86_400.0 / interval_seconds;
        let mut weeks = Vec::with_capacity(in_window.len());
        for o in in_window {
            let end_doy = day_of_year(o.week_ending)?;
            let start_doy = end_doy.checked_sub(NASS_WEEK_DAYS)?;
            let first = to_index(f64::from(start_doy) * per_day);
            let last = to_index(f64::from(end_doy) * per_day);
            if last > samples || last <= first {
                return None;
            }
            let week_kg = total_kg * o.value / total_days;
            // A week is 2016 samples at five minutes, 168 at an hour; either
            // is exact in f64.
            #[allow(
                clippy::cast_precision_loss,
                reason = "a week of samples at any sane interval is exact in f64"
            )]
            let n = (last - first) as f64;
            weeks.push((first, last, week_kg / n));
        }
        // ⚠ No `weeks.is_empty()` guard: `total_days > 0` above already implies
        // `in_window` is non-empty, and every element of it either pushes a week
        // or returns. A guard that cannot fire reads like a check and is not one.
        Some(Self {
            samples,
            total_kg,
            weeks,
        })
    }

    /// The kilograms this profile was built to draw.
    #[must_use]
    pub const fn requested_kg(&self) -> f64 {
        self.total_kg
    }

    /// How many weeks carry draw.
    #[must_use]
    pub const fn weeks(&self) -> usize {
        self.weeks.len()
    }
}

/// A day-of-year multiple of samples-per-day to a sample index.
///
/// ⚠ **Total, and deliberately saturating.** The argument is a small positive
/// day number times a positive samples-per-day, so it is never negative and
/// never `NaN`; it can be infinite, and then the cast saturates to
/// `usize::MAX`. [`TillageDemand::new`]'s window bound is what rejects that —
/// and it is the ONLY check that does, which mutation testing established by
/// removing two others and seeing nothing change.
const fn to_index(x: f64) -> usize {
    // Non-negative and non-NaN by construction, so the floor-then-cast cannot
    // lose a sign. Saturation on an infinite input is intended and is caught
    // by the caller's bound.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "non-negative by construction; saturation is intended and caught by the caller"
    )]
    let i = x as usize;
    i
}

impl cf_storage::Demand for TillageDemand {
    fn name(&self) -> &'static str {
        "fall tillage, drawn in proportion to days suitable"
    }

    fn samples(&self) -> usize {
        self.samples
    }

    fn kg_at(&self, index: usize) -> f64 {
        self.weeks
            .iter()
            .find(|&&(first, last, _)| index >= first && index < last)
            .map_or(0.0, |&(_, _, kg)| kg)
    }
}

/// The year this crate reports its nominal figures for.
///
/// ⚠ 2012 because that is the year `cf-wind` carries, so the demand and the
/// production come from the same year. It is **not** a claim that 2012 is
/// typical — [`WINDOW_RULE_REVERSES_THE_RANKING`] exists precisely because
/// whether it is typical depends on the question asked.
pub const NOMINAL_YEAR: u16 = 2012;

/// Hours per operable day used when reporting a nominal figure.
///
/// ⛔ A **reporting convention, not a measurement**, and deliberately not a
/// default: no function in this crate reads it. Every nominal figure derived
/// from it names it, and [`BREAK_EVEN_CAVEATS`] measures what moving it does.
pub const NOMINAL_HOURS_PER_OPERABLE_DAY: f64 = 12.0;

/// The window rule used when reporting a nominal figure.
pub const NOMINAL_RULE: WindowRule = AFTER_SOYBEANS;

/// ★★★ The answer: the break-even hydrogen engine efficiency, percent.
///
/// `(at the applied load factor 0.59, at the measured 0.78)`, for
/// [`NOMINAL_YEAR`] under [`NOMINAL_RULE`] at
/// [`NOMINAL_HOURS_PER_OPERABLE_DAY`], against **the production inside that
/// same window** — recomputed from stages 1–3, never carried over from another
/// window's figure.
///
/// A hydrogen engine reaching 19% closes the season without touching hydrogen
/// made at any other time of year. ⛔ Whether one reaches it is **not asserted
/// here** — see [`BREAK_EVEN_AS_SHARE_OF_MEASURED_DIESEL_PERCENT`] for the one
/// comparison this crate can make with a measurement behind it.
///
/// Pinned by `the_break_even_band_is_the_measured_one`.
pub const HYDROGEN_ENGINE_EFFICIENCY_NEEDED: (f64, f64) = (14.41, 19.05);

/// The break-even as a share of the diesel's **measured** efficiency, percent.
///
/// `(at the applied load factor 0.59, at the measured 0.78)`:
/// [`HYDROGEN_ENGINE_EFFICIENCY_NEEDED`] divided by
/// [`DIESEL_PTO_THERMAL_EFFICIENCY_PERCENT`].
///
/// ★★ **The only engine comparison in this crate that rests on a measurement.**
/// Nebraska ran the 8245R's diesel and this crate reads its fuel rate and power
/// live, so "a little over half what the diesel does" is a statement about two
/// numbers that both exist. A claim about where real hydrogen engines sit is
/// not, and is deliberately absent: a gate scans the prose surfaces of this
/// crate and fails if such a claim comes back.
///
/// Pinned by `the_break_even_is_a_fraction_of_the_measured_diesel`.
pub const BREAK_EVEN_AS_SHARE_OF_MEASURED_DIESEL_PERCENT: (f64, f64) = (39.90, 52.76);

/// How much more hydrogen the window makes than the season needs, at a 40%
/// engine: `(at load factor 0.78, at 0.59)`.
///
/// ⚠ 40% is a **stated illustration, not a measurement** — the whole point of
/// [`HYDROGEN_ENGINE_EFFICIENCY_NEEDED`] is that this crate does not need to
/// assume an engine. This constant exists to say how much room there is, in a
/// unit that is easier to feel than a percentage.
///
/// Pinned by `the_season_closes_with_margin`.
pub const SEASON_MARGIN_AT_FORTY_PERCENT: (f64, f64) = (2.10, 2.78);

/// How little the window rule moves the break-even, percentage points.
///
/// ⛔⛔ **The correction, as a number.** Across all of [`SWEPT_RULES`] — windows
/// of five, seven and eight weeks — the required engine efficiency varies by
/// less than half a percentage point, because a longer window buys more
/// operable days *and* more hydrogen in roughly equal measure.
///
/// Compare [`WINDOW_RULE_REVERSES_THE_RANKING`], where the same choice
/// completely reorders the years. One question is about operable days, the
/// other about the ratio of demand to supply; an early version of this crate
/// answered the second with evidence from the first.
///
/// Pinned by `the_window_rule_nearly_cancels_against_its_own_production`.
pub const WINDOW_RULE_CANCELS_WITHIN_PP: f64 = 0.4;

/// 2012's rank among complete years under two rules: `(fixed, harvest-gated)`.
///
/// Each is `(rank, of)` with 1 = fewest operable days. Under
/// [`FIXED_OCT_NOV`] 2012 is 4th worst of 19; under [`AFTER_SOYBEANS`] it is
/// 17th of 19, i.e. 3rd best. **Same year, same data, opposite conclusions.**
///
/// The mechanism is measurable rather than arguable: 2012's harvest was
/// extraordinarily early. Corn stood at **94% harvested on 28 October**, which
/// is in this crate's own extract and gated by
/// `the_early_harvest_that_drives_the_reversal`. ⚠ The same NASS report prints
/// a 2007–2011 average of 42% beside it — *North Dakota Crop, Livestock &
/// Weather Report*, week ending 28 October 2012 — but that five-year column is
/// **not** in the committed extract and cannot be checked from this repository.
/// It is quoted, not relied on: the gate uses the 94% alone. A fixed calendar
/// holds the start date constant, and the start date is what varies.
///
/// ⇒ [`WindowRule`] is swept, never picked. Pinned by
/// `the_window_rule_reverses_the_ranking`.
pub const WINDOW_RULE_REVERSES_THE_RANKING: ((usize, usize), (usize, usize)) = ((4, 19), (17, 19));

/// A term that moves the headline, with the size of the move measured.
#[derive(Clone, Copy, Debug)]
pub struct Caveat {
    /// What the term is.
    pub what: &'static str,
    /// The range it was moved over, and where that range came from.
    ///
    /// ⚠ Carried because the swings below are **not commensurable without it**.
    /// One range is the source's own two printings; two are spans this crate
    /// chose. Ranking them without saying so would compare a measurement
    /// against a modelling choice and present both as findings.
    pub range: &'static str,
    /// Why it is uncertain.
    pub why: &'static str,
    /// How far it moves the break-even efficiency, percentage points.
    pub break_even_swing_pp: f64,
}

/// What moves the answer, ranked by measured effect.
///
/// Each swing holds the other two terms at their nominal and moves one across
/// the stated range, measuring the break-even efficiency at each end.
///
/// ⚠ Ranked by what each term does **to the break-even efficiency**, not by the
/// size of the input's own uncertainty. Stage 3 shipped that mistake once: raw
/// error against raw error answers a question nobody asked.
///
/// ⚠ Every swing below recomputes the in-window production at each point. An
/// earlier version held it fixed at a figure belonging to a different window,
/// which put the window rule at the **top** of this list instead of the bottom
/// — the wrong ordering entirely. Both numbers are measured by
/// `the_inconsistent_comparison_inflates_the_window_rule`, which computes the
/// inflated spread rather than quoting it.
pub const BREAK_EVEN_CAVEATS: &[Caveat] = &[
    Caveat {
        what: "hours worked per operable day",
        range: "chosen by this crate: 10 to 14 hours",
        why: "swept; a NASS day suitable for fieldwork is a day the ground can be worked, \
              not a count of hours anyone worked it, and nothing in the window cancels it",
        break_even_swing_pp: 4.80,
    },
    Caveat {
        what: "the load factor, 0.59 applied against 0.78 measured",
        range: "the source's own: NR-005c prints both and justifies both",
        why: "an editorial choice inside EPA NR-005c, not a measurement error - the SwRI \
              cycle measured 0.78 and the model applies its bin composite 0.59",
        break_even_swing_pp: 4.64,
    },
    Caveat {
        what: "the window rule",
        range: "chosen by this crate: the three rules in SWEPT_RULES",
        why: "nearly cancels - a longer window buys operable days and in-window hydrogen \
              together, so it reorders the YEARS without moving the ANSWER",
        break_even_swing_pp: 0.39,
    },
];

/// A term with no magnitude at all, and what would give it one.
#[derive(Clone, Copy, Debug)]
pub struct Unknown {
    /// What is not known.
    pub what: &'static str,
    /// Why this crate does not measure it.
    pub why_unmeasured: &'static str,
    /// What would.
    pub what_would_measure_it: &'static str,
}

/// What this crate cannot answer, stated rather than smoothed.
pub const UNMEASURED_HERE: &[Unknown] = &[
    Unknown {
        what: "acres per hour",
        why_unmeasured: "it runs through draft and wheel slip on deformable soil, which is \
                         the chain's weakest term and has its own stage",
        what_would_measure_it: "ASABE D497 draft for a chisel plow at a stated depth and \
                                soil, against the drawbar surface Nebraska does not state",
    },
    Unknown {
        what: "whether 2012 was also an unusual WIND year",
        why_unmeasured: "cf-wind carries one year, so this crate can compare 2012's fall \
                         against 19 others but its wind against none",
        what_would_measure_it: "a second WTK year at the same grid point; the retrieval \
                                already works and takes about 142 seconds",
    },
    Unknown {
        what: "the farm's own fieldwork days",
        why_unmeasured: "NASS publishes days suitable at STATE level only - checked, no \
                         district breakdown exists in the series",
        what_would_measure_it: "a soil moisture and trafficability model at the grid point, \
                                which would then need its own oracle",
    },
    Unknown {
        what: "hydrogen engine brake thermal efficiency",
        why_unmeasured: "genuinely unsettled, and NONROAD's fuel consumption sits in a DRAFT \
                         companion built on 1988 Tier 0 engines",
        what_would_measure_it: "a dynamometer curve for a specific hydrogen engine; until \
                                one exists this crate reports the efficiency REQUIRED rather \
                                than assuming one",
    },
];
