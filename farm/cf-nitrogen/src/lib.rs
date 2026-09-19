//! The other hydrogen an acre needs.
//!
//! Stage 5 of the acres-per-season chain. Stage 4 asked what the **tractor**
//! demands of the farm's hydrogen. This crate asks what the **crop** does:
//! nitrogen fertilizer is ammonia, ammonia is 17.76% hydrogen by mass, and that
//! hydrogen comes off the same electrolyser.
//!
//! # ★★★ The finding: the crop outweighs the machine
//!
//! At North Dakota's **measured** nitrogen rate for corn, an acre needs
//! [`CORN_HYDROGEN_KG_PER_ACRE`] — about **11.56 kg of hydrogen** for its
//! nitrogen. Stage 4's whole tillage season, run flat out for every operable
//! day the weather allowed, needs a few thousand kilograms for the entire farm.
//!
//! ⇒ One 1 MW turbine's delivered hydrogen covers
//! [`ACRES_FERTILIZED_PER_TURBINE_YEAR`] — **4,334 acres that get treated**,
//! or [`ACRES_PLANTED_SERVED_PER_TURBINE_YEAR`] **4,378 acres planted**, since
//! the survey's rate is per treated acre and not every planted acre is treated.
//! The fuel leg is not what sizes this farm's electrolyser; the nitrogen leg is.
//!
//! ⚠ For scale against stage 4: at the external field-operation figure this
//! chain has **not** measured — 2 to 3 kg of hydrogen per corn acre for all
//! passes — the nitrogen leg is **3.9× to 5.8×** the fuel leg here. That is
//! below the 6–8× in circulation, and [`CORN_BELT_RATE_OVERSTATEMENT_PERCENT`]
//! is why: the multiple is carried by the nitrogen rate, and North Dakota's is
//! lower. Gated by
//! `the_nitrogen_leg_outweighs_the_fuel_leg_by_less_than_advertised`.
//!
//! # ⛔⛔ The rate is a state's rate, and it is not the one in circulation
//!
//! The chain's own research context puts corn nitrogen at 150–200 lb N/acre and
//! attributes it to *"USDA NASS / state extension rates"*. **North Dakota is
//! not that** — and the attribution is the sharp part: read from the NASS
//! Agricultural Chemical Use survey itself, ND corn runs **118 to 160
//! lb N/acre/year** across every year surveyed, and the most recent reading is
//! the low end:
//!
//! | year | lb N/acre/yr |
//! |---|---|
//! | 2010 | 160 |
//! | 2014 | 128 |
//! | 2016 | 133 |
//! | 2018 | 146 |
//! | 2021 | **118** |
//!
//! Using 150–200 here would overstate the nitrogen leg by
//! [`CORN_BELT_RATE_OVERSTATEMENT_PERCENT`]. ⇒ The rate is read per crop and
//! per state from the survey, never from a remembered headline.
//!
//! # ★★ The survey carries its own checksum
//!
//! For every crop and year NASS prints the nitrogen rate **three ways**:
//! pounds per acre per *application*, the *number* of applications, and pounds
//! per acre per *year*. The first two multiply to the third, so a mistranscribed
//! figure stops reconciling — the same structure `cf-nebraska` found in the
//! dual-unit tractor tables.
//!
//! ⚠ Checked by **rounding-interval overlap**, not by a tolerance invented
//! here: NASS rounds the count to one decimal, so `65 × 1.9` must be allowed to
//! mean anything in `[64.5, 65.5] × [1.85, 1.95]`. Under that rule
//! [`CHECKSUM_RECONCILES`] of the records agree and
//! [`CHECKSUM_DISAGREEMENTS`] do not. The disagreements are **recorded, not
//! repaired**.
//!
//! # ⛔ What is measured, what is derived, what is decided
//!
//! | term | source | status |
//! |---|---|---|
//! | nitrogen rate, lb N/acre/yr | [`NASS_CHEMICAL_USE`] | **measured**, per crop and year |
//! | share of planted area treated | the same survey | **measured** — and it is a *different denominator* |
//! | ammonia composition | [`CIAAW_ATOMIC_WEIGHTS`] | **derived** from two atomic weights, not transcribed |
//! | the application window | — | ⛔ **decided**, and stated: see below |
//!
//! ⛔⛔ **NASS publishes no fertilizer-application timing series.** Checked
//! across the 106,596 North Dakota weekly rows of the intermediate file
//! `cf-tillage/NASS_VALIDATION.md` gives the command for: zero mention
//! fertilizer, anhydrous or application.
//!
//! ⚠ **That collection is not committed** — this crate commits 356 fertilizer
//! rows and `cf-tillage` commits 921 fall-week rows, neither of which is the
//! set the absence was checked against. Reproduce it from that page before
//! relying on the claim. The second instrument is recorded there too: the 2012
//! weekly PDFs carry anhydrous in prose and no fertilizer table.
//!
//! What NASS does publish is the sentence this crate takes its window from —
//! for the week ending 14 October 2012, *"anhydrous application and fall
//! tillage occurred in areas of the state with adequate levels of soil
//! moisture."*
//!
//! ⇒ Fall nitrogen shares the operable-days window `cf-tillage` measures,
//! because both need ground a machine can drive on and NASS reports them
//! together. That is a **modelling decision with a citation**, not a
//! measurement, and [`UNMEASURED_HERE`] says what would replace it.

use std::sync::OnceLock;

/// Where a figure was read, precisely enough to read it again — and on what
/// terms.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Source {
    /// Title as the publisher prints it.
    pub document: &'static str,
    /// Where it was retrieved.
    pub url: &'static str,
    /// ISO date of retrieval.
    pub retrieved: &'static str,
    /// What the publisher's terms permit, determined and not assumed.
    pub terms: &'static str,
}

/// A work of the U.S. federal government: no copyright, 17 U.S.C. §105.
pub const US_GOV_PUBLIC_DOMAIN: &str =
    "U.S. Government work, not subject to copyright in the United States (17 U.S.C. \u{a7}105)";

/// The nitrogen rates.
pub const NASS_CHEMICAL_USE: Source = Source {
    document: "USDA NASS Quick Stats, environmental: North Dakota fertilizer application, \
               Agricultural Chemical Use Program",
    url: "https://www.nass.usda.gov/datasets/qs.environmental_20260919.txt.gz",
    retrieved: "2026-09-19",
    terms: US_GOV_PUBLIC_DOMAIN,
};

/// The two atomic weights, and the only figures this crate transcribes.
///
/// ⚠ Terms follow the reasoning `farm/PROVENANCE.md` already applies to the
/// Nebraska tables: a standard atomic weight is a **fact**, and a fact is not
/// the expression of it. IUPAC's tabulation is IUPAC's; the two numbers are
/// not. Nothing else here is transcribed — every other quantity in this
/// module is stoichiometry.
pub const CIAAW_ATOMIC_WEIGHTS: Source = Source {
    document: "IUPAC Commission on Isotopic Abundances and Atomic Weights, \
               standard atomic weights",
    url: "https://ciaaw.org/atomic-weights.htm",
    retrieved: "2026-09-19",
    terms: "the VALUES are facts and not copyrightable; the tabulation is IUPAC's and is \
            not reproduced here beyond the two weights ammonia is made of",
};

/// A figure as a document printed it, with the precision it was printed to.
///
/// The primitive the chain uses everywhere: comparison is by **rounding-interval
/// overlap**, never by a percentage tolerance invented at the call site.
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
}

/// Standard atomic weight of nitrogen.
pub const NITROGEN_ATOMIC_WEIGHT: Printed = Printed::new(14.007, 3);

/// Standard atomic weight of hydrogen.
///
/// ⚠ Consistent with `cf_storage`'s molar mass of H₂, `2.016e-3 kg/mol`, which
/// is exactly twice this. One hydrogen for the whole chain.
pub const HYDROGEN_ATOMIC_WEIGHT: Printed = Printed::new(1.008, 3);

/// One international avoirdupois pound in kilograms. Exact by definition.
pub const LB_KG: f64 = 0.453_592_37;

/// Molar mass of ammonia, NH₃.
#[must_use]
pub fn ammonia_molar_mass() -> f64 {
    NITROGEN_ATOMIC_WEIGHT.value() + 3.0 * HYDROGEN_ATOMIC_WEIGHT.value()
}

/// Nitrogen's share of ammonia by mass.
///
/// ⚠ **Derived, not transcribed.** Widely quoted as "82%"; the ratio of the two
/// atomic weights gives 82.244%.
#[must_use]
pub fn nitrogen_mass_fraction() -> f64 {
    NITROGEN_ATOMIC_WEIGHT.value() / ammonia_molar_mass()
}

/// Hydrogen's share of ammonia by mass.
///
/// ⚠ **Derived, not transcribed**, and this one matters: the figure in
/// circulation is 17.6%, and the stoichiometric value is **17.756%** — see
/// [`CIRCULATED_HYDROGEN_FRACTION`]. Every kilogram of ammonia this crate
/// converts runs through it.
#[must_use]
pub fn hydrogen_mass_fraction() -> f64 {
    3.0 * HYDROGEN_ATOMIC_WEIGHT.value() / ammonia_molar_mass()
}

/// The hydrogen fraction as commonly quoted, for the gate that rejects it.
///
/// Carried so `the_circulated_hydrogen_fraction_is_not_the_stoichiometric_one`
/// can measure the gap rather than assert it. ⛔ Never used in a calculation.
pub const CIRCULATED_HYDROGEN_FRACTION: Printed = Printed::new(0.176, 3);

/// The committed NASS extract: North Dakota nitrogen fertilizer, state level.
const NASS_TSV: &str = include_str!("../data/nd_nitrogen.tsv");

/// SHA-256 of the committed extract.
///
/// Gated by `the_committed_extract_matches_its_digest`, which computes it
/// in-process — a digest no test computes is a 64-character string with no
/// producer.
pub const NASS_TSV_SHA256: &str =
    "b589e55b320f6eb3c3ab76a8978e2929919b0db16a3e5265bcaf50d5eddb93b3";

/// Which of the survey's four printings an observation is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Measure {
    /// `LB / ACRE / APPLICATION, AVG`.
    PerApplication,
    /// `NUMBER, AVG` — applications per year.
    Applications,
    /// `LB / ACRE / YEAR, AVG`.
    PerYear,
    /// `PCT OF AREA PLANTED, AVG`.
    ///
    /// ⚠⚠ **A different denominator.** The per-acre rates are per *treated*
    /// acre; this says what share of the planted area was treated at all. In
    /// 1990 that was 80% of North Dakota corn, so nitrogen per *planted* acre
    /// was a fifth lower than the printed rate.
    PctOfAreaPlanted,
}

impl Measure {
    /// The code this measure carries in the committed extract.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::PerApplication => "PER_APP",
            Self::Applications => "APPS",
            Self::PerYear => "PER_YEAR",
            Self::PctOfAreaPlanted => "PCT_AREA",
        }
    }

    /// The survey's own unit string, so a number always says what it is of.
    #[must_use]
    pub const fn nass_unit(self) -> &'static str {
        match self {
            Self::PerApplication => "LB / ACRE / APPLICATION, AVG",
            Self::Applications => "NUMBER, AVG",
            Self::PerYear => "LB / ACRE / YEAR, AVG",
            Self::PctOfAreaPlanted => "PCT OF AREA PLANTED, AVG",
        }
    }

    /// The code as the extract spells it, back to a measure.
    #[must_use]
    pub fn from_code(code: &str) -> Option<Self> {
        match code {
            "PER_APP" => Some(Self::PerApplication),
            "APPS" => Some(Self::Applications),
            "PER_YEAR" => Some(Self::PerYear),
            "PCT_AREA" => Some(Self::PctOfAreaPlanted),
            _ => None,
        }
    }
}

/// A published cell, or NASS's statement that it will not publish one.
///
/// ⛔ Withholding is **not** a missing value. NASS suppresses a cell when
/// publishing it would disclose an individual operation, so the quantity exists
/// and the survey knows it. Collapsing that into `None` loses the distinction
/// between "not surveyed" and "surveyed and withheld".
#[derive(Clone, Copy, Debug)]
pub enum Cell {
    /// A figure, with the precision NASS printed it to.
    Published(Printed),
    /// NASS's `(D)` flag: withheld to avoid disclosing individual operations.
    Withheld,
}

impl Cell {
    /// The figure, or `None` if withheld.
    #[must_use]
    pub const fn published(self) -> Option<Printed> {
        match self {
            Self::Published(p) => Some(p),
            Self::Withheld => None,
        }
    }

    /// Whether NASS withheld this cell.
    #[must_use]
    pub const fn is_withheld(self) -> bool {
        matches!(self, Self::Withheld)
    }
}

/// One published figure from the survey.
#[derive(Clone, Copy, Debug)]
pub struct Observation {
    /// Commodity, as NASS names it.
    pub crop: &'static str,
    /// Class within the commodity — wheat is surveyed by class.
    pub class: &'static str,
    /// Production practice.
    ///
    /// ⛔⛔ **Part of the key, and dropping it collapsed two distinct records.**
    /// NASS surveys `ORGANIC` separately from `ALL PRODUCTION PRACTICES`, and
    /// an organic acre does not receive synthetic anhydrous ammonia at all — so
    /// mixing the two would put acres that need no Haber-Bosch hydrogen into a
    /// Haber-Bosch model. See [`ALL_PRACTICES`].
    pub practice: &'static str,
    /// Survey year.
    pub year: u16,
    /// Which of the four printings.
    pub measure: Measure,
    /// The cell.
    pub cell: Cell,
}

struct Extract {
    rows: Vec<Observation>,
    unparsed: usize,
}

/// One line of the extract to an observation, or `None` if it cannot be read.
///
/// ⛔ Public so the rejection paths are reachable from a test; the committed
/// extract parses cleanly by construction.
#[must_use]
pub fn parse_line(line: &'static str) -> Option<Observation> {
    let mut f = line.split('\t');
    let crop = f.next()?;
    let class = f.next()?;
    let practice = f.next()?;
    let year: u16 = f.next()?.parse().ok()?;
    let measure = Measure::from_code(f.next()?)?;
    let raw = f.next()?;
    if f.next().is_some() || crop.is_empty() || class.is_empty() || practice.is_empty() {
        return None;
    }
    let cell = if raw == "(D)" {
        Cell::Withheld
    } else {
        let value: f64 = raw.parse().ok()?;
        let decimals = raw.split_once('.').map_or(0, |(_, frac)| frac.len());
        Cell::Published(Printed::new(value, u8::try_from(decimals).ok()?))
    };
    Some(Observation {
        crop,
        class,
        practice,
        year,
        measure,
        cell,
    })
}

fn extract() -> &'static Extract {
    static EXTRACT: OnceLock<Extract> = OnceLock::new();
    EXTRACT.get_or_init(|| {
        let mut rows = Vec::new();
        let mut unparsed = 0usize;
        for line in NASS_TSV.lines() {
            if line.starts_with('#') || line.starts_with("crop\t") || line.is_empty() {
                continue;
            }
            match parse_line(line) {
                Some(o) => rows.push(o),
                None => unparsed += 1,
            }
        }
        Extract { rows, unparsed }
    })
}

/// Every observation in the committed extract.
#[must_use]
pub fn observations() -> &'static [Observation] {
    &extract().rows
}

/// Lines the parser could not read.
///
/// ⛔⛔ An absence claim is a hypothesis: a parser that dropped every row would
/// also report zero failures. Asserted against a known row count.
#[must_use]
pub fn unparsed_lines() -> usize {
    extract().unparsed
}

/// One crop, class and year's complete nitrogen record.
#[derive(Clone, Copy, Debug)]
pub struct Record {
    /// Commodity.
    pub crop: &'static str,
    /// Class.
    pub class: &'static str,
    /// Production practice.
    pub practice: &'static str,
    /// Survey year.
    pub year: u16,
    /// Pounds of nitrogen per acre per application.
    pub per_application: Option<Cell>,
    /// Applications per year.
    pub applications: Option<Cell>,
    /// Pounds of nitrogen per acre per year.
    pub per_year: Option<Cell>,
    /// Share of planted area that was treated at all.
    pub pct_of_area_planted: Option<Cell>,
}

impl Record {
    /// Whether the survey's three printings reconcile with each other.
    ///
    /// ★★ **The source's own checksum**: per-application × applications must
    /// reproduce per-year. Compared by **rounding-interval overlap**, because
    /// NASS prints the count to one decimal and the rates to whole pounds —
    /// `65 × 1.9` must be allowed to mean anything in
    /// `[64.5, 65.5] × [1.85, 1.95]`.
    ///
    /// Returns `None` when any of the three is absent or withheld, which is not
    /// the same as disagreeing.
    #[must_use]
    pub fn reconciles(&self) -> Option<bool> {
        let a = self.per_application?.published()?;
        let n = self.applications?.published()?;
        let y = self.per_year?.published()?;
        let low = a.low() * n.low();
        let high = a.high() * n.high();
        Some(low <= y.high() && y.low() <= high)
    }
}

/// Every complete record in the extract, in crop then year order.
#[must_use]
pub fn records() -> Vec<Record> {
    let mut keys: Vec<(&'static str, &'static str, &'static str, u16)> = observations()
        .iter()
        .map(|o| (o.crop, o.class, o.practice, o.year))
        .collect();
    keys.sort_unstable();
    keys.dedup();
    keys.into_iter()
        .map(|(crop, class, practice, year)| {
            let cell = |m: Measure| {
                observations()
                    .iter()
                    .find(|o| {
                        o.crop == crop
                            && o.class == class
                            && o.practice == practice
                            && o.year == year
                            && o.measure == m
                    })
                    .map(|o| o.cell)
            };
            Record {
                crop,
                class,
                practice,
                year,
                per_application: cell(Measure::PerApplication),
                applications: cell(Measure::Applications),
                per_year: cell(Measure::PerYear),
                pct_of_area_planted: cell(Measure::PctOfAreaPlanted),
            }
        })
        .collect()
}

/// The production practice every figure this crate calculates with comes from.
///
/// ⛔ Anything else — currently only `ORGANIC` — is carried in the extract so
/// it is visible, and excluded from every calculation on purpose.
pub const ALL_PRACTICES: &str = "ALL PRODUCTION PRACTICES";

/// The record for one crop, class, practice and year.
#[must_use]
pub fn record(crop: &str, class: &str, practice: &str, year: u16) -> Option<Record> {
    records()
        .into_iter()
        .find(|r| r.crop == crop && r.class == class && r.practice == practice && r.year == year)
}

/// How many records reconcile under the rounding-interval rule.
pub const CHECKSUM_RECONCILES: usize = 86;

/// How many records the checksum cannot judge either way.
///
/// ⛔ **Not the same as agreeing.** One record — organic spring wheat, 2009 —
/// has all four of its cells withheld, so there is nothing to reconcile. A
/// checksum that counted it as a pass would be counting an absence as evidence.
pub const CHECKSUM_INDETERMINATE: usize = 1;

/// The records whose three printings do **not** reconcile.
///
/// ⚠ **Recorded, not repaired.** Both are North Dakota corn, and both miss by
/// about a pound. Why is **NOT ISOLATED**: it could be a revision between
/// printings, a suppressed application class, or rounding applied to a figure
/// this crate cannot see. Nudging any of the three to close it would be
/// authoring an input from the check that validates it.
pub const CHECKSUM_DISAGREEMENTS: &[(&str, &str, u16)] =
    &[("CORN", "ALL CLASSES", 2001), ("CORN", "ALL CLASSES", 2003)];

/// Ammonia needed to deliver `lb_n_per_acre` pounds of nitrogen, kg per acre.
///
/// Returns `None` unless the rate is finite and non-negative.
#[must_use]
pub fn ammonia_kg_per_acre(lb_n_per_acre: f64) -> Option<f64> {
    if !lb_n_per_acre.is_finite() || lb_n_per_acre < 0.0 {
        return None;
    }
    Some(lb_n_per_acre * LB_KG / nitrogen_mass_fraction())
}

/// Hydrogen bound up in that ammonia, kg per acre.
///
/// ⚠ This is the hydrogen the **molecule** contains. It is not the hydrogen a
/// plant would consume: synthesis losses, purge and the energy to run the loop
/// sit on top, and this crate deliberately models none of them — see
/// [`UNMEASURED_HERE`]. It is therefore a **floor**.
#[must_use]
pub fn hydrogen_kg_per_acre(lb_n_per_acre: f64) -> Option<f64> {
    Some(ammonia_kg_per_acre(lb_n_per_acre)? * hydrogen_mass_fraction())
}

/// The most recent year the survey covers North Dakota corn.
pub const CORN_REFERENCE_YEAR: u16 = 2021;

/// North Dakota corn's measured nitrogen rate in [`CORN_REFERENCE_YEAR`].
pub const CORN_NITROGEN_LB_PER_ACRE: Printed = Printed::new(118.0, 0);

/// Hydrogen bound up in one corn acre's nitrogen, kg.
///
/// At [`CORN_NITROGEN_LB_PER_ACRE`]. Pinned by
/// `the_corn_acre_hydrogen_is_the_measured_one`.
pub const CORN_HYDROGEN_KG_PER_ACRE: Printed = Printed::new(11.555, 3);

/// **Treated** corn acres one 1 MW turbine's delivered hydrogen covers in a year.
///
/// ★★★ The headline. Against the **50,086 kg** stages 1–3 measure at 350 bar
/// for the Foster County 2012 wind year, at the rate above.
///
/// ⛔⛔ **Treated acres, not planted acres**, and the distinction is this
/// crate's own: [`Measure::PerYear`] is pounds per *treated* acre, so dividing
/// hydrogen by it yields the acres that get treated. A farm plants more than it
/// treats — see [`ACRES_PLANTED_SERVED_PER_TURBINE_YEAR`], which applies the
/// survey's own [`Measure::PctOfAreaPlanted`]. At 2021's 99% the two differ by
/// 1%; at 1990's 80% they differ by **25%**.
///
/// ⚠ A **ceiling**, for the reason [`hydrogen_kg_per_acre`] gives: it counts
/// the hydrogen in the molecule and nothing that making the molecule costs.
///
/// Pinned by `the_acres_fertilized_headline`, which recomputes the delivered
/// kilograms from stages 1–3 rather than quoting them.
pub const ACRES_FERTILIZED_PER_TURBINE_YEAR: Printed = Printed::new(4_334.0, 0);

/// **Planted** corn acres the same hydrogen serves, at the surveyed treated share.
///
/// [`ACRES_FERTILIZED_PER_TURBINE_YEAR`] divided by the share of planted area
/// the survey says was treated at all in [`CORN_REFERENCE_YEAR`].
///
/// ⚠ The two headlines answer different questions and neither is wrong. This
/// one is what a farmer plants; the other is what gets a pass of anhydrous.
/// Reporting one while naming the other is the denominator error this chain
/// keeps finding, and an earlier version of this crate did exactly that —
/// documenting the distinction in three places and then not applying it.
///
/// Pinned by `the_planted_acre_headline_uses_the_surveyed_treated_share`.
pub const ACRES_PLANTED_SERVED_PER_TURBINE_YEAR: Printed = Printed::new(4_378.0, 0);

/// Planted acres implied by `treated` acres at a `pct_of_area_planted` share.
///
/// Returns `None` unless the share is a finite percentage in `(0, 100]`.
#[must_use]
pub fn planted_acres(treated: f64, pct_of_area_planted: f64) -> Option<f64> {
    if !treated.is_finite() || treated < 0.0 {
        return None;
    }
    if !pct_of_area_planted.is_finite() || pct_of_area_planted <= 0.0 || pct_of_area_planted > 100.0
    {
        return None;
    }
    Some(treated / (pct_of_area_planted / 100.0))
}

/// How far the circulated 150–200 lb N/acre overstates North Dakota, percent.
///
/// `(at 150, at 200)` against the measured [`CORN_NITROGEN_LB_PER_ACRE`].
/// Pinned by `the_circulated_rate_overstates_north_dakota`.
pub const CORN_BELT_RATE_OVERSTATEMENT_PERCENT: (f64, f64) = (27.12, 69.49);

/// The season's nitrogen, drawn on the window the fuel is drawn on.
///
/// ⛔ Delegates its shape to [`cf_tillage::TillageDemand`] rather than
/// re-deriving it. NASS reports anhydrous application and fall tillage in the
/// same sentence and the same week, so they share the operable-days profile;
/// duplicating the distribution logic would let the two drift apart while
/// looking independent.
#[derive(Clone, Debug)]
pub struct NitrogenDemand {
    inner: cf_tillage::TillageDemand,
    acres: f64,
    lb_n_per_acre: f64,
}

impl NitrogenDemand {
    /// The nitrogen draw for `acres` at `lb_n_per_acre`, over one year's window.
    ///
    /// Returns `None` if the rate is not finite and non-negative, if the
    /// acreage makes the total unusable, or if the window cannot be shaped —
    /// see [`cf_tillage::TillageDemand::new`], which is the single place the
    /// total is checked.
    #[must_use]
    pub fn new(
        year: u16,
        window: &cf_tillage::Window,
        acres: f64,
        lb_n_per_acre: f64,
        samples: usize,
        interval_seconds: f64,
    ) -> Option<Self> {
        // ⚠ No guard on `acres`. A non-finite or negative acreage makes the
        // total non-finite or negative, and `TillageDemand::new` refuses both —
        // mutation testing showed a guard here could not be made to fail. The
        // rate IS guarded, because `hydrogen_kg_per_acre` is where a bad rate
        // would otherwise pass through as a plausible number.
        let total = hydrogen_kg_per_acre(lb_n_per_acre)? * acres;
        let inner = cf_tillage::TillageDemand::new(year, window, total, samples, interval_seconds)?;
        Some(Self {
            inner,
            acres,
            lb_n_per_acre,
        })
    }

    /// Acres this draw fertilizes.
    #[must_use]
    pub const fn acres(&self) -> f64 {
        self.acres
    }

    /// The nitrogen rate it was built at.
    #[must_use]
    pub const fn lb_n_per_acre(&self) -> f64 {
        self.lb_n_per_acre
    }
}

impl cf_storage::Demand for NitrogenDemand {
    fn name(&self) -> &'static str {
        "fall nitrogen, drawn in proportion to days suitable"
    }

    fn samples(&self) -> usize {
        cf_storage::Demand::samples(&self.inner)
    }

    fn kg_at(&self, index: usize) -> f64 {
        cf_storage::Demand::kg_at(&self.inner, index)
    }

    fn total_kg(&self) -> f64 {
        cf_storage::Demand::total_kg(&self.inner)
    }
}

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
        what: "everything synthesis costs on top of the hydrogen in the molecule",
        why_unmeasured: "Haber-Bosch conversion is incomplete and the loop purges; the \
                         synthesis and air-separation energy are electricity this crate does \
                         not model, and the loop has a minimum turndown it can trip below",
        what_would_measure_it: "a loop model with turndown and ramp against a published \
                                electric Haber-Bosch study - deliberately a later stage, and \
                                not started until this one is gated",
    },
    Unknown {
        what: "when nitrogen is actually applied",
        why_unmeasured: "NASS publishes NO fertilizer-timing series - checked across the \
                         106,596 weekly rows of the intermediate file cf-tillage/NASS_VALIDATION.md \
                         gives the command for, which is NOT committed in this repository",
        what_would_measure_it: "a state extension survey of application timing, or a \
                                soil-temperature rule; until then the window is the one NASS \
                                puts anhydrous in the same sentence as tillage",
    },
    Unknown {
        what: "the split between fall and spring application",
        why_unmeasured: "the same absence: the survey gives an annual rate and no timing, so \
                         this crate draws the whole rate on the fall window",
        what_would_measure_it: "extension survey data on fall versus spring anhydrous share \
                                in eastern North Dakota",
    },
    Unknown {
        what: "whether a farm would buy ammonia rather than make it",
        why_unmeasured: "economics are deliberately outside this model",
        what_would_measure_it: "nothing here; it belongs in a separate layer that reads these \
                                outputs and never feeds back into them",
    },
];
