//! MISSION's headline: acres per season, as the minimum of three ceilings.
//!
//! The last stage of the acres-per-season chain. Stages 1–3 measured the
//! hydrogen a farm's wind makes and what it costs to keep; stage 4 measured the
//! window the tractor can work in; stage 5 measured what the crop's nitrogen
//! takes out of the same hydrogen. This crate puts them together.
//!
//! # ⛔⛔ What this is, and what it is NOT
//!
//! **It is a ceiling model. It is not a soil model.** Acres per season here is
//! the smallest of three independently measured limits:
//!
//! | ceiling | set by | measured from |
//! |---|---|---|
//! | **time** | days the ground is workable × acres per hour | USDA NASS + [`ARS_TRACTOR_POWER_WORKSHEET`] |
//! | **energy** | in-window hydrogen ÷ drawbar energy per acre | stages 1–3 + `cf_nebraska` |
//! | **nitrogen** | annual hydrogen ÷ the hydrogen an acre's fertilizer holds | `cf_nitrogen` |
//!
//! ⛔ It does **not** predict draft for an implement or a soil, and it models
//! **no wheel slip**. Those need the measured→field conversion that this chain
//! has twice found to be unavailable:
//!
//! - **ASABE D497's draft coefficients are paywalled** (sold through the ANSI
//!   webstore). The model's *structure* is in the open literature; the
//!   coefficient table is the standard's content, so it is not reproduced here.
//! - **No Nebraska edition carries per-load wheel slip** — and that is not
//!   asserted here, it is *read*: `cf_nebraska::ABSENT` records it as data, and
//!   `the_slip_absence_is_read_from_the_source_not_asserted` fails if that
//!   entry ever leaves. The individual reports that would carry it are behind
//!   an endpoint that refuses automated retrieval, re-confirmed on a second
//!   route in 2026-09.
//! - **The Nebraska drawbar surface is not stated in any edition.**
//!
//! ⇒ Every one of those collapses into a single lumped term,
//! [`drawbar_kwh_per_acre`], which this crate **derives from measurements it
//! has** rather than from coefficients it does not. See
//! [`UNMEASURED_HERE`] for what would replace it.
//!
//! # ★★★ The finding: fuel is never the answer, and the other two trade places
//!
//! **The energy ceiling never binds** — not at any load factor, engine
//! efficiency or working day this crate sweeps. Its slack runs
//! [`ENERGY_SLACK_RANGE`], and even at the tightest corner of the sweep — a 25%
//! engine at the pessimistic load factor — fuel still has 43% headroom.
//! See [`ENERGY_NEVER_BINDS`].
//!
//! ⇒ That is the most useful thing here, because **the whole hydrogen half of
//! the chain — `cf-wind`, `cf-electrolysis` and `cf-storage` — exists to
//! measure that ceiling.** Having measured it properly, the answer is that it
//! is not the constraint.
//!
//! ★★★ What binds instead **changes hands at
//! [`BINDING_CEILING_FLIPS_AT_HOURS_PER_DAY`]**: work a shorter day and the
//! *window* binds; work a longer one and *nitrogen* does. The farm sits almost
//! exactly on that crossover — the two ceilings agree to
//! [`TIME_AND_NITROGEN_CONVERGE_WITHIN_PERCENT`] at the nominal day.
//!
//! ⚠ And those two were measured by **different crates, from different federal
//! surveys, with no shared input**: one from NASS fieldwork days and an ARS
//! machinery rate, the other from a NASS chemical-use survey and stoichiometry.
//! Their landing this close is not a construction of this crate.
//!
//! # ⚠ The soft input, named up front
//!
//! [`REFERENCE_PLOW`]'s **acres per hour** sets the time ceiling directly, and
//! it is the weakest number here. The worksheet it comes from states its own
//! provenance: *"derived from the published Mississippi State University 2026
//! Crop Planning Budgets"* — a **state** source inside a federal tool, for a
//! region that is not this farm's.
//!
//! ⇒ It is **swept, never asserted**, and [`surface_textures`] is committed so
//! a reader can judge whether it transfers: Foster County's surface is
//! [`LOAM_SHARE_PERCENT`] loam, where the Mississippi Delta is not.

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

/// The soil under this farm.
pub const NRCS_SOIL_DATA_ACCESS: Source = Source {
    document: "USDA NRCS Soil Data Access (SSURGO), survey area ND031 — Foster County, \
               North Dakota, surface horizon texture by major component",
    url: "https://sdmdataaccess.nrcs.usda.gov/Tabular/post.rest",
    retrieved: "2026-09-19",
    terms: US_GOV_PUBLIC_DOMAIN,
};

/// The acres-per-hour anchor — and the one source here whose terms are layered.
///
/// ⛔⛔ **A federal tool over a state source.** The worksheet is published by
/// USDA-ARS, but it states plainly that its implement data is *"derived from
/// the published Mississippi State University 2026 Crop Planning Budgets"*.
/// The federal wrapper does not change where the numbers came from, and
/// `farm/PROVENANCE.md` records it as its own case rather than folding it in
/// with the §105 sources.
pub const ARS_TRACTOR_POWER_WORKSHEET: Source = Source {
    document: "USDA-ARS National Soil Dynamics Laboratory, Auburn AL: \
               Tractor Power Requirement Recommendation Worksheet",
    url: "https://www.ars.usda.gov/ARSUserFiles/60100500/Worksheets/",
    retrieved: "2026-09-19",
    terms: "the TOOL is a U.S. Government work; its implement data is stated by the tool \
            itself to derive from Mississippi State University crop budgets, a STATE source \
            whose status this repository has NOT DETERMINED",
};

/// A figure as a document printed it, with the precision it was printed to.
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

/// The committed SSURGO extract.
const TEXTURE_TSV: &str = include_str!("../data/nd031_surface_texture.tsv");
/// The committed ARS worksheet extract.
const CHISEL_TSV: &str = include_str!("../data/ars_chisel_plow.tsv");

/// SHA-256 of the committed soil extract.
pub const TEXTURE_TSV_SHA256: &str =
    "012e99b4be513e210ea16248af2ae90465f5ac0abf460a867261796a683fe2f7";
/// SHA-256 of the committed implement extract.
pub const CHISEL_TSV_SHA256: &str =
    "e0989091d079669d6a756cbb0ebe9acc0078cb310a37f62b8e6398a0964aa748";

/// One surface texture class and the area it covers.
#[derive(Clone, Copy, Debug)]
pub struct TextureShare {
    /// SSURGO texture class, as the survey names it.
    pub class: &'static str,
    /// Map-unit acres weighted by component percent.
    pub acres: f64,
}

/// One chisel plow row from the ARS worksheet.
#[derive(Clone, Copy, Debug)]
pub struct ChiselPlow {
    /// Equipment name as the worksheet spells it.
    pub equipment: &'static str,
    /// Working width, feet.
    pub width_ft: f64,
    /// The power unit the worksheet recommends for it.
    pub power_unit: &'static str,
    /// Hours per acre.
    pub perf_rate_hr_per_acre: f64,
    /// Acres per hour — the worksheet prints this too.
    pub field_capacity_acres_per_hr: f64,
}

impl ChiselPlow {
    /// Whether the worksheet's two rate printings are reciprocals.
    ///
    /// ★★ **The worksheet's own checksum.** It prints hours-per-acre and
    /// acres-per-hour as separate columns, so a mistranscribed rate stops being
    /// the reciprocal of its neighbour — the same structure `cf-nebraska` found
    /// in dual-unit tables and `cf-nitrogen` found in NASS's three printings.
    #[must_use]
    pub fn reciprocal_holds(&self) -> bool {
        let p = self.perf_rate_hr_per_acre;
        let c = self.field_capacity_acres_per_hr;
        if !p.is_finite() || p <= 0.0 || !c.is_finite() || c <= 0.0 {
            return false;
        }
        (p * c - 1.0).abs() < 1e-9
    }

    /// Ground speed the worksheet's rate implies, mph, at a stated field
    /// efficiency.
    ///
    /// ⚠ A **derived** figure, not one the worksheet prints. It exists so the
    /// anchor can be sanity-checked against a speed `cf_nebraska` measured:
    /// if the implied speed were absurd, the rate would be too.
    #[must_use]
    pub fn implied_speed_mph(&self, field_efficiency: f64) -> Option<f64> {
        const SQ_FT_PER_ACRE: f64 = 43_560.0;
        const FT_PER_MILE: f64 = 5_280.0;
        if !field_efficiency.is_finite() || field_efficiency <= 0.0 || field_efficiency > 1.0 {
            return None;
        }
        if self.width_ft <= 0.0 || self.field_capacity_acres_per_hr <= 0.0 {
            return None;
        }
        let ft_per_hr = self.field_capacity_acres_per_hr * SQ_FT_PER_ACRE / self.width_ft;
        Some(ft_per_hr / FT_PER_MILE / field_efficiency)
    }
}

struct Extract {
    textures: Vec<TextureShare>,
    plows: Vec<ChiselPlow>,
    unparsed: usize,
}

/// One texture line to a share, or `None` if it cannot be read.
#[must_use]
pub fn parse_texture_line(line: &'static str) -> Option<TextureShare> {
    let mut f = line.split('\t');
    let class = f.next()?;
    let acres: f64 = f.next()?.parse().ok()?;
    if f.next().is_some() || class.is_empty() || !acres.is_finite() || acres < 0.0 {
        return None;
    }
    Some(TextureShare { class, acres })
}

/// One implement line to a plow, or `None` if it cannot be read.
#[must_use]
pub fn parse_chisel_line(line: &'static str) -> Option<ChiselPlow> {
    let mut f = line.split('\t');
    let equipment = f.next()?;
    let width_ft: f64 = f.next()?.parse().ok()?;
    let power_unit = f.next()?;
    let perf_rate_hr_per_acre: f64 = f.next()?.parse().ok()?;
    let field_capacity_acres_per_hr: f64 = f.next()?.parse().ok()?;
    if f.next().is_some() || equipment.is_empty() || power_unit.is_empty() {
        return None;
    }
    Some(ChiselPlow {
        equipment,
        width_ft,
        power_unit,
        perf_rate_hr_per_acre,
        field_capacity_acres_per_hr,
    })
}

fn extract() -> &'static Extract {
    static EXTRACT: OnceLock<Extract> = OnceLock::new();
    EXTRACT.get_or_init(|| {
        let mut textures = Vec::new();
        let mut plows = Vec::new();
        let mut unparsed = 0usize;
        for line in TEXTURE_TSV.lines() {
            if line.starts_with('#') || line.starts_with("texture_class\t") || line.is_empty() {
                continue;
            }
            match parse_texture_line(line) {
                Some(t) => textures.push(t),
                None => unparsed += 1,
            }
        }
        for line in CHISEL_TSV.lines() {
            if line.starts_with('#') || line.starts_with("equipment\t") || line.is_empty() {
                continue;
            }
            match parse_chisel_line(line) {
                Some(p) => plows.push(p),
                None => unparsed += 1,
            }
        }
        Extract {
            textures,
            plows,
            unparsed,
        }
    })
}

/// Foster County's surface texture classes, largest area first.
#[must_use]
pub fn surface_textures() -> &'static [TextureShare] {
    &extract().textures
}

/// The chisel plows the ARS worksheet carries.
#[must_use]
pub fn chisel_plows() -> &'static [ChiselPlow] {
    &extract().plows
}

/// Lines neither parser could read.
///
/// ⛔⛔ An absence claim is a hypothesis: a parser that dropped every row would
/// also report zero failures. Asserted against known row counts.
#[must_use]
pub fn unparsed_lines() -> usize {
    extract().unparsed
}

/// Total component-weighted acres in the soil extract.
#[must_use]
pub fn surveyed_acres() -> f64 {
    surface_textures().iter().fold(0.0, |a, t| a + t.acres)
}

/// The texture class covering the most ground.
#[must_use]
pub fn dominant_texture() -> Option<&'static TextureShare> {
    surface_textures()
        .iter()
        .max_by(|a, b| a.acres.total_cmp(&b.acres))
}

/// Share of Foster County's surveyed surface that is loam, percent.
///
/// ⚠ Carried because it is the evidence a reader needs to judge whether the
/// Mississippi-derived acres-per-hour anchor transfers to this farm. Pinned by
/// `the_soil_under_this_farm_is_loam`.
pub const LOAM_SHARE_PERCENT: Printed = Printed::new(81.65, 2);

/// The worksheet row this crate anchors on.
///
/// ★ Chosen because its recommended power unit is the class `cf_nebraska`'s
/// tractor falls in — a 190 hp MFWD against this tractor's measured PTO power —
/// and because the arc locked a ~200 hp row-crop MFWD before any of this was
/// known. ⚠ The comparison is **recomputed** from `cf_nebraska` by
/// `the_reference_plow_matches_this_tractors_power_class`, not quoted: a figure
/// retyped from another crate goes stale without anything going red.
pub const REFERENCE_PLOW: &str = "Chisel Plow Folding 24'";

/// The reference row, or `None` if the extract no longer carries it.
#[must_use]
pub fn reference_plow() -> Option<&'static ChiselPlow> {
    chisel_plows()
        .iter()
        .find(|p| p.equipment == REFERENCE_PLOW)
}

/// Which limit is actually doing the binding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ceiling {
    /// Days the ground is workable × acres per hour.
    Time,
    /// In-window hydrogen ÷ drawbar energy per acre.
    Energy,
    /// Annual hydrogen ÷ the hydrogen an acre's fertilizer holds.
    Nitrogen,
}

impl Ceiling {
    /// What sets it, for reporting a number that says what it is of.
    #[must_use]
    pub const fn what_sets_it(self) -> &'static str {
        match self {
            Self::Time => "days the ground is workable, times acres per hour",
            Self::Energy => "hydrogen made during the window, over drawbar energy per acre",
            Self::Nitrogen => {
                "hydrogen made across the year, over the hydrogen in an acre's ammonia"
            }
        }
    }
}

/// The three ceilings, and which one binds.
#[derive(Clone, Copy, Debug)]
pub struct Ceilings {
    /// Acres the window allows.
    pub time_acres: f64,
    /// Acres the in-window hydrogen allows as fuel.
    pub energy_acres: f64,
    /// Acres the year's hydrogen allows as nitrogen.
    pub nitrogen_acres: f64,
}

impl Ceilings {
    /// The binding ceiling — the smallest of the three.
    #[must_use]
    pub fn binding(&self) -> Ceiling {
        let mut best = (Ceiling::Time, self.time_acres);
        for (c, v) in [
            (Ceiling::Energy, self.energy_acres),
            (Ceiling::Nitrogen, self.nitrogen_acres),
        ] {
            if v < best.1 {
                best = (c, v);
            }
        }
        best.0
    }

    /// Acres per season: the smallest ceiling.
    #[must_use]
    pub const fn acres(&self) -> f64 {
        self.time_acres
            .min(self.energy_acres)
            .min(self.nitrogen_acres)
    }

    /// How much slack the loosest ceiling has over the binding one, as a
    /// multiple.
    ///
    /// ⚠ Read this against the binding ceiling, not on its own: a ceiling with
    /// 3× slack is not a constraint, and saying so is most of what this crate
    /// has to report.
    #[must_use]
    pub fn slack(&self) -> f64 {
        let lo = self.acres();
        if !lo.is_finite() || lo <= 0.0 {
            return f64::NAN;
        }
        self.time_acres
            .max(self.energy_acres)
            .max(self.nitrogen_acres)
            / lo
    }
}

/// Acres the operating window allows.
///
/// Returns `None` unless every input is finite and positive.
#[must_use]
pub fn time_ceiling(operable_days: f64, hours_per_day: f64, acres_per_hour: f64) -> Option<f64> {
    let all = [operable_days, hours_per_day, acres_per_hour];
    if all.iter().any(|v| !v.is_finite() || *v <= 0.0) {
        return None;
    }
    Some(operable_days * hours_per_day * acres_per_hour)
}

/// Drawbar energy one acre of tillage takes, kWh.
///
/// ⛔⛔ **The lumped term.** Draft, depth, ground speed, wheel slip and field
/// efficiency all live inside this one number, because this chain can measure
/// none of them separately — see the module docs. It is *derived* from things
/// that were measured: the tractor's rated PTO power (`cf_nebraska`), the load
/// factor EPA measured, the drivetrain ratio Nebraska gives two ways, and the
/// acres-per-hour anchor.
///
/// ⚠ That makes it a **lumped observation, not a model**. It cannot be pushed
/// to another implement, another depth or another soil, and
/// [`UNMEASURED_HERE`] says what would let it be.
#[must_use]
pub fn drawbar_kwh_per_acre(
    pto_kw: f64,
    load_factor: f64,
    drivetrain: f64,
    acres_per_hour: f64,
) -> Option<f64> {
    if !pto_kw.is_finite() || pto_kw <= 0.0 || !acres_per_hour.is_finite() || acres_per_hour <= 0.0
    {
        return None;
    }
    for fraction in [load_factor, drivetrain] {
        if !fraction.is_finite() || fraction <= 0.0 || fraction > 1.0 {
            return None;
        }
    }
    Some(pto_kw * load_factor * drivetrain / acres_per_hour)
}

/// Acres the available hydrogen allows as tractor fuel.
///
/// `engine_efficiency` is the hydrogen engine's brake thermal efficiency as a
/// fraction. ⚠ It has **no default** here, for the reason `cf_tillage` gives:
/// nothing in `farm/` holds an oracle for it.
#[must_use]
pub fn energy_ceiling(
    available_kg: f64,
    h2_lhv_kwh_per_kg: f64,
    engine_efficiency: f64,
    drivetrain: f64,
    drawbar_kwh_per_acre: f64,
) -> Option<f64> {
    if !available_kg.is_finite() || available_kg <= 0.0 {
        return None;
    }
    if !h2_lhv_kwh_per_kg.is_finite() || h2_lhv_kwh_per_kg <= 0.0 {
        return None;
    }
    if !drawbar_kwh_per_acre.is_finite() || drawbar_kwh_per_acre <= 0.0 {
        return None;
    }
    for fraction in [engine_efficiency, drivetrain] {
        if !fraction.is_finite() || fraction <= 0.0 || fraction > 1.0 {
            return None;
        }
    }
    Some(available_kg * h2_lhv_kwh_per_kg * engine_efficiency * drivetrain / drawbar_kwh_per_acre)
}

/// Acres the available hydrogen allows as nitrogen fertilizer.
///
/// ⚠ **Treated acres**, because the NASS rate is per treated acre — the
/// distinction `cf_nitrogen` documents and this crate inherits rather than
/// rediscovers.
#[must_use]
pub fn nitrogen_ceiling(available_kg: f64, lb_n_per_acre: f64) -> Option<f64> {
    if !available_kg.is_finite() || available_kg <= 0.0 {
        return None;
    }
    let per_acre = cf_nitrogen::hydrogen_kg_per_acre(lb_n_per_acre)?;
    (per_acre > 0.0).then_some(available_kg / per_acre)
}

/// Whether the fuel hydrogen is ever the binding limit.
///
/// ★★★ **It is not — in any combination this crate sweeps.** Across the load
/// factor's two printings, four engine efficiencies and three working days, the
/// energy ceiling never falls below the other two; its slack runs 2.3× to 3.0×.
/// Pinned by `the_energy_ceiling_never_binds`.
///
/// ⇒ For this farm, at this scale, **fuel is not the question.** That is the
/// single most useful thing this crate reports, because the chain's whole
/// hydrogen half — `cf-wind`, `cf-electrolysis`, `cf-storage` — exists to
/// measure it.
pub const ENERGY_NEVER_BINDS: bool = true;

/// How much slack the energy ceiling has over the binding one: `(min, max)`.
///
/// Measured across the whole 40-point sweep. ⚠ The minimum sits at a **25%**
/// engine — below anything a real hydrogen engine does — at the pessimistic
/// load factor, which is why the finding survives: fuel is not close to binding
/// even when the engine is assumed worse than it could be.
///
/// ⛔ An earlier draft of the module docs said "2.3× to 3.0×". That was the
/// engine-0.40 slice, not the sweep, and the gate caught it.
pub const ENERGY_SLACK_RANGE: (f64, f64) = (1.43, 4.16);

/// Working hours per day at which the binding ceiling changes hands.
///
/// ★★★ Below this the **window** binds; above it **nitrogen** does. The farm
/// sits almost exactly on the crossover — an hour either way changes which
/// constraint is real, and fuel is neither.
///
/// ⚠ This is the answer to MISSION's question having a *shape* rather than a
/// value: acres per season is not set by one thing, and which thing it is set
/// by depends on an operating choice nobody has measured. Pinned by
/// `the_binding_ceiling_flips_with_the_working_day`.
pub const BINDING_CEILING_FLIPS_AT_HOURS_PER_DAY: Printed = Printed::new(11.02, 2);

/// How close the time and nitrogen ceilings sit at [`NOMINAL_HOURS_PER_DAY`],
/// percent of the smaller.
///
/// ★★ The more interesting half: two ceilings measured by **different crates
/// from different federal surveys with no shared input** land within this much
/// of each other. Pinned by `the_time_and_nitrogen_ceilings_converge`.
pub const TIME_AND_NITROGEN_CONVERGE_WITHIN_PERCENT: Printed = Printed::new(8.92, 2);

/// Hours per day this crate reports its nominal figures at.
///
/// ⛔ A **reporting convention, not a measurement**, carried over from
/// `cf_tillage` so the two stages report on the same footing. No function here
/// reads it.
pub const NOMINAL_HOURS_PER_DAY: f64 = 12.0;

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
        what: "draft force for an implement, a depth or a soil",
        why_unmeasured: "ASABE D497's coefficient table is sold through the ANSI webstore, so \
                         it is not reproduced here; only the model's published STRUCTURE is open",
        what_would_measure_it: "the standard itself, used as a retrieved oracle the way \
                                cf-storage uses NIST - validate against it, commit the \
                                measurement and the command, never the table",
    },
    Unknown {
        what: "wheel slip at any load",
        why_unmeasured: "NO Nebraska edition carries it - half the oracle MISSION names by \
                         name - and the individual reports that would are behind an endpoint \
                         that refuses automated retrieval, re-confirmed on a second route",
        what_would_measure_it: "one individual OECD report obtained by hand, which would also \
                                close the drawbar-surface question below",
    },
    Unknown {
        what: "the surface Nebraska measured drawbar on",
        why_unmeasured: "it is not stated in any edition read, and asserting concrete from \
                         recall would prejudge the measured-to-field conversion that is \
                         exactly the term being modelled",
        what_would_measure_it: "the individual report, or the laboratory's test procedure \
                                document",
    },
    Unknown {
        what: "whether the acres-per-hour anchor transfers to North Dakota",
        why_unmeasured: "the ARS worksheet's implement data derives from Mississippi State \
                         crop budgets; this crate sweeps the rate rather than asserting it, \
                         and commits the local soil texture so a reader can judge",
        what_would_measure_it: "a North Dakota extension machinery-rate publication, or a \
                                field measurement on this farm",
    },
];
