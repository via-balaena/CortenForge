//! Hydrogen from the electrolyser outlet into a farm tank.
//!
//! This is stage 3 of the acres-per-season chain. Stage 1 (`cf-wind`) says how
//! many kilowatt-hours the wind carries at one real site; stage 2
//! (`cf-electrolysis`) turns those into kilograms at the electrolyser's outlet,
//! about **21 bar**. This crate answers the two questions that stand between
//! those kilograms and a tractor: **what does it cost to compress them**, and
//! **how big a tank does the farm need**.
//!
//! # ★★★ The finding: the two questions are not the same size
//!
//! Both run through one number — the real gas's compressibility — and that
//! number has wildly different leverage on each:
//!
//! | quantity | ideal gas is wrong by | this crate's model is wrong by |
//! |---|---|---|
//! | compression **work**, 20.68 → 350 bar | **6.29–6.65%** | below the source's own printing |
//! | tank **volume** at 350 bar, 300 K | **21.96%** | **1.48%** |
//! | tank **volume** at 700 bar, 300 K | **44.87%** | **3.21%** |
//!
//! Every figure in that table is a constant a gate pins — the work row by
//! [`IDEAL_GAS_WORK_UNDERSTATEMENT_PERCENT`], the volume rows by
//! [`NIST_DENSITY_COMPARISON`]. ⚠ The pressures are exact: the oracle is
//! retrieved on a grid containing 350, 440, 700 and 880 bar, because an earlier
//! grid of `1 + 10k` put every measurement one bar away from the pressure its
//! own field name claimed.
//!
//! ⚠ **Read those two rows against the answer, not against each other.** A work
//! error is diluted, because compression is only a ~3% debit; a density error
//! passes straight into the vessel. Using ideal gas moves the delivered
//! kilograms by **0.20%** and the tank volume by **18.0%** — both measured by
//! `the_equation_of_state_moves_the_tank_far_more_than_the_energy`.
//! ⇒ **Model the gas properly because of the tank, not because of the energy.**
//!
//! # ⛔⛔ The second finding: storage is a cliff, not a slope
//!
//! A seasonal operation drawing on an all-year resource has a threshold at
//! **the hydrogen produced during the operating window itself**. Below it the
//! buffer is a rounding error; above it the tank must carry the shortfall from
//! months earlier and grows without any useful bound. [`minimum_tank_kg`]
//! locates the threshold; it does not assume where the farm sits relative to
//! it, because that needs the tractor's demand and the tillage window, which
//! are stage 4.
//!
//! # ⛔ The temperature is measured, not conventional
//!
//! Every figure in the source document is at **300 K**, which is a laboratory
//! convention. This tank stands in a North Dakota field, and over the candidate
//! tillage window the measured mean is **273.14 K** — see [`CARRINGTON_FALL`].
//! Twenty-seven kelvin is an ~8% term on both outputs, in opposite directions:
//! the cold tank holds **more** and costs **less** to fill. A default of 300 K
//! would have put a convention inside a number that reads as measured.
//!
//! # ★★ How the transcribed figures are checked
//!
//! Four structurally different layers, because each is blind to what the others
//! catch. Three are about the page; the fourth is about the world.
//!
//! | layer | what it is | what it CANNOT see |
//! |---|---|---|
//! | [`Record9013::increments_reconcile`] | **differential**: the table's differences reproduce the prose's stated increments | an error common to table and prose |
//! | [`Record9013::efficiencies_reconcile`] | **multiplicative**: theoretical ÷ actual reproduces the record's own printed station efficiencies | an error that scales both together |
//! | [`Record9013::lhv_agrees_with`] | **cross-document**: the record's heating value against `cf_electrolysis::AFDC_2026` | anything the two documents got wrong alike |
//! | [`Covolume`] against [`NIST_DENSITY_COMPARISON`] | **physics**, from a source outside the record | an error in the equation of state itself |
//!
//! ⛔⛔ **The fourth layer is weaker than it looks, and the reason is worth
//! stating.** Record 9013 says its theoretical figures were *"determined from
//! exergy differences using the standard properties … as reported by NIST"*.
//! So the record and the NIST oracle **share a source**. Agreeing with both is
//! one check, not two. The genuinely independent leg is [`GOODWIN_1964`], a
//! 1964 NBS paper that supplies this model's only parameter and touches neither
//! — see [`Covolume`].
//!
//! # ⛔ What is committed here and what is not
//!
//! NIST Standard Reference Data is **copyright-asserted** under the Standard
//! Reference Data Act, unlike the public-domain DOE and NBS documents beside
//! it. So no NIST value is stored in this crate. What is stored is the
//! **measurement** of this model against it ([`NIST_DENSITY_COMPARISON`]) and
//! the command that reproduces the comparison. The test that performs it is
//! `#[ignore]`d and reads a locally fetched file, the pattern
//! `design/cf-fsu-geometry/BODYPARTS3D.md` established for licensed assets.

/// How far [`IdealGas`] understates reversible compression work, percent.
///
/// From the electrolyser outlet (20.68 bar) to a 350 bar tank, as
/// `(smallest, largest)` over every temperature this crate evaluates at: the
/// source document's 300 K convention at one end and [`CARRINGTON_FALL`]'s
/// **coldest observed hour**, 265.15 K, at the other. The understatement grows
/// as the gas cools.
///
/// ⚠ An earlier version paired 300 K with the window *mean* and described the
/// two as spanning the window. They did not — the coldest hour sits outside
/// them both, at 6.65%.
///
/// Pinned by `the_ideal_gas_understates_the_compression_work`. ⚠ Unlike the
/// density figures this needs no oracle — one model against another, both in
/// this crate, so it runs in CI.
pub const IDEAL_GAS_WORK_UNDERSTATEMENT_PERCENT: (f64, f64) = (6.29, 6.65);

/// Molar gas constant, J/(mol·K). Exact by the 2019 SI redefinition.
const R_J_PER_MOL_K: f64 = 8.314_462_618_153_24;
/// Molar mass of hydrogen gas, kg/mol.
///
/// ⚠ The same value `cf-electrolysis` uses, deliberately: one molar mass for
/// the chain. NIST's hydrogen equation of state uses 2.015 88 g/mol, which is
/// 0.006% away — three orders of magnitude below this model's own measured
/// error, and pinned by `the_molar_mass_difference_is_below_the_model_error`
/// rather than waved at.
const H2_MOLAR_MASS_KG: f64 = 2.016e-3;
/// Specific gas constant for hydrogen, J/(kg·K).
const R_SPECIFIC: f64 = R_J_PER_MOL_K / H2_MOLAR_MASS_KG;
/// Joules in a kilowatt-hour. Exact.
const KWH_J: f64 = 3.6e6;
/// Pascals in a bar. Exact.
const BAR_PA: f64 = 1e5;
/// Cubic centimetres per mole, in m³/mol. The unit the 1964 paper prints B in.
const CM3_PER_MOL_M3: f64 = 1e-6;

/// Where a figure was read, precisely enough to read it again — and on what terms.
///
/// ⚠ Same shape as `cf_electrolysis::Source`, and the `terms` field is there
/// for the same reason: this crate is the one where *"it is a government
/// document, therefore free"* finally breaks. Two of its three sources are
/// public-domain federal works and the third is a federal database that
/// asserts copyright.
#[derive(Clone, Copy, Debug)]
pub struct Source {
    /// Publishing body and document title.
    pub document: &'static str,
    /// Where it lives.
    pub url: &'static str,
    /// ISO date it was retrieved.
    pub retrieved: &'static str,
    /// What the publisher's terms permit, **determined and not assumed**.
    pub terms: &'static str,
}

/// The determination for a work of the United States Government.
const US_GOV_PUBLIC_DOMAIN: &str = concat!(
    "U.S. Government work, 17 U.S.C. §105 — no copyright, public domain; ",
    "no attribution or redistribution obligation. Checked 2026-09-18."
);

/// The determination for NIST Standard Reference Data, which is **not** that.
///
/// ⛔⛔ The loud exception to the rule above, and the reason this crate keeps a
/// `terms` field on every source rather than a comment somewhere.
const NIST_SRD_COPYRIGHT_ASSERTED: &str = concat!(
    "⛔ NIST Standard Reference Data — copyright ASSERTED under the Standard ",
    "Reference Data Act, 15 U.S.C. §290e, notwithstanding 17 U.S.C. §105. ",
    "\u{201c}© 2026 by the U.S. Secretary of Commerce on behalf of the United ",
    "States of America. All rights reserved.\u{201d} ⇒ retrieved oracle only, ",
    "never committed to this repository. Checked 2026-09-18."
);

/// A figure as the source printed it, with the precision it was printed to.
///
/// ⚠ The same primitive as `cf_electrolysis::Printed`, not yet shared. Two
/// figures agree if their **rounding intervals overlap**, which is not the same
/// set as "within some percent": a value printed to two decimals carries ±0.005
/// of slack whatever its magnitude. Extracting this into one crate is a real
/// and deferred job — see the arc note; it waits on a third distinct user so
/// the shape is chosen from evidence rather than symmetry.
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

    /// Half of the last printed unit — how far the true value may lie either side.
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

    /// Whether a computed number could have been printed this way.
    #[must_use]
    pub fn admits(&self, computed: f64) -> bool {
        computed >= self.low() && computed <= self.high()
    }

    /// Whether two printed figures could be the same underlying number.
    #[must_use]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.low() <= other.high() && other.low() <= self.high()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// The model's one parameter, from a source outside every check it is used in
// ─────────────────────────────────────────────────────────────────────────────

/// The 1964 NBS paper that supplies B(T).
///
/// ★★★ **This is the independent leg of the whole crate.** Record 9013's
/// figures come from NIST, so checking this model against NIST *and* against
/// the record is one check computed twice. The second virial coefficient comes
/// from neither: a 1964 reduction of Michels' experimental compressibility
/// data, published in the NBS Journal of Research, which states in terms that
/// it is a public-domain U.S. Government work.
///
/// ⚠ **Not fully independent, and the difference matters.** NIST's current
/// equation of state was itself fitted to experimental PVT data that overlaps
/// what this paper reduced. These are two reductions of overlapping
/// measurements forty-five years apart, not two unrelated observations. What
/// *is* independent is the arithmetic: nothing NIST publishes enters this
/// model, so a NIST comparison can fail.
pub const GOODWIN_1964: Source = Source {
    document: "R. D. Goodwin, D. E. Diller, H. M. Roder & L. A. Weber, \
               \u{201c}Second and Third Virial Coefficients for Hydrogen\u{201d}, \
               Journal of Research of the National Bureau of Standards \
               Section A: Physics and Chemistry, 68A(1), 1964, pp. 121–126",
    url: "https://doi.org/10.6028/jres.068A.011",
    retrieved: "2026-09-18",
    terms: US_GOV_PUBLIC_DOMAIN,
};

/// Reference temperature of the paper's equation (2a), kelvin, as printed.
pub const GOODWIN_2A_T0_K: Printed = Printed::new(109.83, 2);
/// Amplitude of the paper's equation (2a), cm³/mol, as printed.
pub const GOODWIN_2A_B0: Printed = Printed::new(19.866, 3);
/// Reference temperature of the paper's equation (2b), kelvin, as printed.
pub const GOODWIN_2B_T0_K: Printed = Printed::new(109.781, 3);

/// Equation (2a) with its two parameters supplied, cm³/mol.
fn eq_2a(t0: f64, b0: f64, temperature_k: f64) -> f64 {
    b0 * (1.0 - (t0 / temperature_k).powf(1.25))
}

/// Second virial coefficient from the paper's equation (2a), cm³/mol.
///
/// `B = B₀ · [1 − x^(5/4)]`, `x ≡ T₀/T`, with `T₀ = 109.83 K` and
/// `B₀ = 19.866 cm³/mol`. The paper quotes a fit deviation of 0.125 cm³/mol.
///
/// ★ Kept alongside [`second_virial_2b`] as a **cross-check, not a spare**:
/// the paper prints the output of each equation in its own column of Table 2,
/// so an implementation of either can be checked against the page it came from.
/// ⚠ This one does **not** come back clean — see [`second_virial_2a_band`].
///
/// Returns `None` for a temperature that is not positive and finite.
#[must_use]
pub fn second_virial_2a(temperature_k: f64) -> Option<f64> {
    if !temperature_k.is_finite() || temperature_k <= 0.0 {
        return None;
    }
    Some(eq_2a(
        GOODWIN_2A_T0_K.value(),
        GOODWIN_2A_B0.value(),
        temperature_k,
    ))
}

/// Equation (2a) across the printed precision of its own two parameters, cm³/mol.
///
/// ⛔⛔ **This exists to measure an unexplained gap, not to close one.** This
/// implementation of (2a) is systematically **about 0.004 cm³/mol below** the
/// paper's own calculated column — at 248.15 K it gives 12.6944 against a
/// printed 12.70 — and three of the nine rows fall outside the last printed
/// digit. This function evaluates (2a) across the printed precision of `T₀` and
/// `B₀`, and that precision accounts for well under half of the gap.
///
/// ⛔ **The rest has not been isolated**, and it is left that way. Nudging `T₀`
/// by 0.008 K would close it exactly and would be authoring an input from the
/// check that validates it — the error `cf-nebraska` exists to make impossible.
/// A plausible story about 1964 arithmetic would be worse than the measurement.
///
/// ★ Which is affordable, because **(2a) is not the model**. [`second_virial_2b`]
/// is, and it reproduces its own column at face value on all nine rows with a
/// centred residual. Carried into the covolume this gap would be 0.03% of it,
/// two orders of magnitude below the model's measured error against NIST —
/// `the_one_term_fit_is_systematically_low_against_its_own_column` measures
/// both halves of that.
///
/// Returns the `(low, high)` bound, or `None` for a temperature that is not
/// positive and finite.
#[must_use]
pub fn second_virial_2a_band(temperature_k: f64) -> Option<(f64, f64)> {
    if !temperature_k.is_finite() || temperature_k <= 0.0 {
        return None;
    }
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for t0 in [GOODWIN_2A_T0_K.low(), GOODWIN_2A_T0_K.high()] {
        for b0 in [GOODWIN_2A_B0.low(), GOODWIN_2A_B0.high()] {
            let v = eq_2a(t0, b0, temperature_k);
            lo = lo.min(v);
            hi = hi.max(v);
        }
    }
    Some((lo, hi))
}

/// Second virial coefficient from the paper's equation (2b), cm³/mol.
///
/// `B = Σᵢ₌₁⁴ Bᵢ · x^((2i−1)/4)`, `x ≡ T₀/T`, `T₀ = 109.781 K`, with
/// `B₁ = +42.464`, `B₂ = −37.1172`, `B₃ = −2.2982`, `B₄ = −3.0484` cm³/mol.
///
/// ★ **This is the one the model uses**, and the one that comes back clean: it
/// reproduces the paper's own calculated column at face value on all nine
/// tabulated temperatures at or above 223 K, with a residual that changes sign
/// rather than drifting one way. It is also the closer fit to the derived
/// values — within 0.04 cm³/mol where (2a) reaches 0.30.
///
/// Returns `None` for a temperature that is not positive and finite.
#[must_use]
pub fn second_virial_2b(temperature_k: f64) -> Option<f64> {
    if !temperature_k.is_finite() || temperature_k <= 0.0 {
        return None;
    }
    Some(eq_2b(GOODWIN_2B_T0_K.value(), temperature_k))
}

/// Equation (2b) with its reference temperature supplied, cm³/mol.
fn eq_2b(t0: f64, temperature_k: f64) -> f64 {
    let x = t0 / temperature_k;
    42.464 * x.powf(0.25)
        - 37.117_2 * x.powf(0.75)
        - 2.298_2 * x.powf(1.25)
        - 3.048_4 * x.powf(1.75)
}

/// One row of the paper's Table 2, at or above 223.15 K.
///
/// ★ The **roster** these figures are checked against, transcribed as printed.
/// The paper gives both what it derived from measurement and what each of its
/// own equations calculates, which is what makes a transcription check possible
/// at all: an implementation of (2a) can be compared with the column the paper
/// printed *from* (2a).
#[derive(Clone, Copy, Debug)]
pub struct VirialRow {
    /// Temperature, kelvin.
    pub temperature_k: f64,
    /// The paper's value derived from measurement, cm³/mol.
    pub derived: Printed,
    /// The paper's own equation (2a) output, cm³/mol.
    pub calculated_2a: Printed,
    /// The paper's own equation (2b) output, cm³/mol.
    pub calculated_2b: Printed,
}

/// Table 2 of [`GOODWIN_1964`], restricted to the range this crate works in.
///
/// ⚠ Restricted deliberately. The paper covers 15–423 K; a hydrogen tank in a
/// field does not go below 223 K, and a two-term virial has no business near
/// the critical point (33.145 K) anyway. Transcribing rows the crate cannot use
/// would add checkable surface without adding a check.
pub const GOODWIN_TABLE_2: &[VirialRow] = &[
    row(223.15, 11.98, 11.68, 11.93),
    row(248.15, 12.97, 12.70, 12.94),
    row(273.15, 13.76, 13.51, 13.72),
    row(298.15, 14.38, 14.17, 14.34),
    row(323.15, 14.87, 14.71, 14.85),
    row(348.15, 15.27, 15.17, 15.25),
    row(373.15, 15.60, 15.56, 15.59),
    row(398.15, 15.86, 15.90, 15.87),
    row(423.15, 16.08, 16.19, 16.10),
];

/// Build a Table 2 row; every figure there is printed to two decimals.
/// `simple_fit` is the paper's one-term equation (2a);
/// `series_fit` is its four-term equation (2b).
const fn row(temperature_k: f64, derived: f64, simple_fit: f64, series_fit: f64) -> VirialRow {
    VirialRow {
        temperature_k,
        derived: Printed::new(derived, 2),
        calculated_2a: Printed::new(simple_fit, 2),
        calculated_2b: Printed::new(series_fit, 2),
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// The equation of state, as a seam
// ─────────────────────────────────────────────────────────────────────────────

/// How this crate believes hydrogen behaves under pressure.
///
/// ★★★ A trait rather than a function because **every stage of this chain gets
/// a seam**. The sweep has to be able to say how much the equation of state
/// moves the answer, and it can only do that if a worse one can be substituted.
/// [`IdealGas`] is that worse one, and it is not a straw man: it is what a
/// back-of-envelope estimate uses, and
/// `only_the_real_gas_model_reproduces_the_record` measures what it costs.
pub trait EquationOfState {
    /// Name, for reporting which model produced a number.
    fn name(&self) -> &'static str;

    /// Specific volume, m³/kg.
    ///
    /// Returns `None` for inputs that are not positive and finite, or for a
    /// state the model cannot represent.
    fn specific_volume_m3_per_kg(&self, pressure_bar: f64, temperature_k: f64) -> Option<f64>;

    /// Reversible isothermal compression work, kWh/kg.
    ///
    /// The minimum electricity a perfect machine with perfect intercooling
    /// would need. Real compressors take more; [`Record9013`] carries measured
    /// station efficiencies for turning this into a plant figure.
    ///
    /// Returns `None` for inputs that are not positive and finite.
    fn isothermal_work_kwh_per_kg(
        &self,
        from_bar: f64,
        to_bar: f64,
        temperature_k: f64,
    ) -> Option<f64>;

    /// Density, kg/m³ — the reciprocal of the specific volume.
    ///
    /// Returns `None` whenever [`EquationOfState::specific_volume_m3_per_kg`]
    /// does, or if that volume is not strictly positive.
    fn density_kg_per_m3(&self, pressure_bar: f64, temperature_k: f64) -> Option<f64> {
        let v = self.specific_volume_m3_per_kg(pressure_bar, temperature_k)?;
        (v > 0.0).then_some(1.0 / v)
    }
}

/// Reject a pressure/temperature pair that no model can be asked about.
fn state_is_usable(pressure_bar: f64, temperature_k: f64) -> bool {
    pressure_bar.is_finite()
        && pressure_bar > 0.0
        && temperature_k.is_finite()
        && temperature_k > 0.0
}

/// `Pv = RT`. The model to beat, and the one most estimates actually use.
///
/// ⛔ Kept because being wrong is its job. At 350 bar it puts **22% more**
/// hydrogen in a tank than is really there, and at 700 bar **45% more** — an
/// error that sizes steel, not an academic one.
#[derive(Clone, Copy, Debug, Default)]
pub struct IdealGas;

impl EquationOfState for IdealGas {
    fn name(&self) -> &'static str {
        "ideal gas"
    }

    fn specific_volume_m3_per_kg(&self, pressure_bar: f64, temperature_k: f64) -> Option<f64> {
        state_is_usable(pressure_bar, temperature_k)
            .then(|| R_SPECIFIC * temperature_k / (pressure_bar * BAR_PA))
    }

    fn isothermal_work_kwh_per_kg(
        &self,
        from_bar: f64,
        to_bar: f64,
        temperature_k: f64,
    ) -> Option<f64> {
        if !state_is_usable(from_bar, temperature_k) || !state_is_usable(to_bar, temperature_k) {
            return None;
        }
        Some(R_SPECIFIC * temperature_k * (to_bar / from_bar).ln() / KWH_J)
    }
}

/// `v = R·T/P + b(T)` — the virial expansion truncated after the second
/// coefficient, written pressure-explicitly.
///
/// ★★ **The covolume is not a fitted parameter.** `b(T) = B(T)/M` with B from
/// [`second_virial_2b`], so the single number that makes this model better than
/// [`IdealGas`] was published in 1964 by people who had never heard of this
/// repository. Nothing here is tuned to the oracle it is measured against —
/// which is the only reason [`NIST_DENSITY_COMPARISON`] is a measurement rather
/// than a restatement of a fit.
///
/// ⚠ **Where it degrades, and why.** The residual against NIST grows with
/// pressure and shrinks with temperature, which is the signature of the *third*
/// virial coefficient — the term this model drops. That is checkable rather
/// than plausible, and it is checked by its **shape**: a missing ρ² term leaves
/// a residual proportional to ρ², which
/// `the_residual_looks_like_a_missing_third_virial_term` measures along each
/// isotherm. Restoring the term would roughly halve the error at 700 bar, and
/// the chain's headline cannot see the difference, so it is not in the model.
#[derive(Clone, Copy, Debug, Default)]
pub struct Covolume;

impl Covolume {
    /// The covolume at a temperature, m³/kg.
    ///
    /// Returns `None` for a temperature that is not positive and finite.
    #[must_use]
    pub fn covolume_m3_per_kg(temperature_k: f64) -> Option<f64> {
        Some(second_virial_2b(temperature_k)? * CM3_PER_MOL_M3 / H2_MOLAR_MASS_KG)
    }
}

impl EquationOfState for Covolume {
    fn name(&self) -> &'static str {
        "second-virial covolume, B(T) from Goodwin et al. 1964 eq. (2b)"
    }

    fn specific_volume_m3_per_kg(&self, pressure_bar: f64, temperature_k: f64) -> Option<f64> {
        if !state_is_usable(pressure_bar, temperature_k) {
            return None;
        }
        let v = R_SPECIFIC * temperature_k / (pressure_bar * BAR_PA)
            + Self::covolume_m3_per_kg(temperature_k)?;
        (v > 0.0).then_some(v)
    }

    fn isothermal_work_kwh_per_kg(
        &self,
        from_bar: f64,
        to_bar: f64,
        temperature_k: f64,
    ) -> Option<f64> {
        if !state_is_usable(from_bar, temperature_k) || !state_is_usable(to_bar, temperature_k) {
            return None;
        }
        // ∫v dP with v = R·T/P + b is R·T·ln(P₂/P₁) + b·(P₂−P₁): the covolume
        // contributes a term LINEAR in the pressure difference, which is why it
        // is a small correction to the work and a large one to the density.
        let b = Self::covolume_m3_per_kg(temperature_k)?;
        Some(
            (R_SPECIFIC * temperature_k * (to_bar / from_bar).ln()
                + b * (to_bar - from_bar) * BAR_PA)
                / KWH_J,
        )
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// The oracle: measured, committed as a measurement, and not committed as data
// ─────────────────────────────────────────────────────────────────────────────

/// The NIST `WebBook`, as an oracle this repository may use but not store.
pub const NIST_WEBBOOK: Source = Source {
    document: "NIST Chemistry WebBook, SRD 69 — Thermophysical Properties of \
               Fluid Systems, hydrogen (CAS 1333-74-0), isothermal property tables",
    url: "https://webbook.nist.gov/chemistry/fluid/",
    retrieved: "2026-09-18",
    terms: NIST_SRD_COPYRIGHT_ASSERTED,
};

/// What this crate's two models cost, measured against [`NIST_WEBBOOK`].
///
/// ★★ **A measurement about a database, not a copy of one.** The numbers here
/// are errors of *this crate's* models. Reproducing them needs the oracle,
/// which is why [`NistDensityComparison::reproduce`] is a command rather than a
/// data file — see the crate's `NIST_VALIDATION.md`.
#[derive(Clone, Copy, Debug)]
pub struct NistDensityComparison {
    /// Where the reference values came from.
    pub source: Source,
    /// Number of distinct (pressure, temperature) states compared.
    pub states: usize,
    /// Lowest temperature of the comparison, kelvin.
    pub temperature_low_k: f64,
    /// Highest temperature of the comparison, kelvin.
    pub temperature_high_k: f64,
    /// Lowest pressure of the comparison, bar.
    ///
    /// ⚠ 20 bar, not 1. The grid starts at the record's own inlet so every
    /// pressure this crate names — 350, 440, 700, 880 — is **on** it. An earlier
    /// grid of `1 + 10k` contained none of them, so each field named
    /// `_at_350_bar_` was in fact measured at 351.
    pub pressure_low_bar: f64,
    /// Highest pressure of the comparison, bar.
    pub pressure_high_bar: f64,
    /// Worst density error of [`IdealGas`] over those states, percent.
    pub worst_ideal_percent: f64,
    /// Worst density error of [`Covolume`] over those states, percent.
    pub worst_covolume_percent: f64,
    /// [`IdealGas`] density error at 350 bar and 300 K, percent — the tank case.
    pub ideal_at_350_bar_300k_percent: f64,
    /// [`Covolume`] density error at 350 bar and 300 K, percent.
    pub covolume_at_350_bar_300k_percent: f64,
    /// [`IdealGas`] density error at 350 bar and 273.15 K, percent.
    ///
    /// ★ The row that matters for **this** farm. 300 K is the source document's
    /// convention; [`CARRINGTON_FALL`] is 273 K, and the residual of a two-term
    /// virial grows as the gas gets colder and denser.
    pub ideal_at_350_bar_273k_percent: f64,
    /// [`Covolume`] density error at 350 bar and 273.15 K, percent.
    pub covolume_at_350_bar_273k_percent: f64,
    /// [`IdealGas`] density error at 700 bar and 300 K, percent.
    pub ideal_at_700_bar_300k_percent: f64,
    /// [`Covolume`] density error at 700 bar and 300 K, percent.
    pub covolume_at_700_bar_300k_percent: f64,
    /// [`IdealGas`] density error at 700 bar and 273.15 K, percent.
    pub ideal_at_700_bar_273k_percent: f64,
    /// [`Covolume`] density error at 700 bar and 273.15 K, percent.
    ///
    /// ⚠ The worst case this farm could actually meet: a cold tank at the higher
    /// service pressure. Every other row in this struct is a gentler condition.
    pub covolume_at_700_bar_273k_percent: f64,
    /// How to obtain the reference values again.
    pub reproduce: &'static str,
}

/// Measured 2026-09-18 over four isotherms.
///
/// ⛔⛔ **Read the two rows against each other, not each alone.** The point is
/// not that [`Covolume`] is good; it is that the model a back-of-envelope
/// estimate reaches for is wrong by **half the answer** at the pressure a
/// tractor's tank runs at, and that one parameter from a 1964 paper removes
/// nine tenths of that.
pub const NIST_DENSITY_COMPARISON: NistDensityComparison = NistDensityComparison {
    source: NIST_WEBBOOK,
    states: 348,
    temperature_low_k: 250.0,
    temperature_high_k: 330.0,
    pressure_low_bar: 20.0,
    pressure_high_bar: 880.0,
    worst_ideal_percent: 67.44,
    worst_covolume_percent: 8.00,
    ideal_at_350_bar_300k_percent: 21.96,
    covolume_at_350_bar_300k_percent: 1.48,
    ideal_at_350_bar_273k_percent: 23.80,
    covolume_at_350_bar_273k_percent: 2.19,
    ideal_at_700_bar_300k_percent: 44.87,
    covolume_at_700_bar_300k_percent: 3.21,
    ideal_at_700_bar_273k_percent: 49.06,
    covolume_at_700_bar_273k_percent: 4.75,
    reproduce: "For each T in 250, 273.15, 300, 330 K: GET \
        https://webbook.nist.gov/cgi/fluid.cgi with Action=Data, Wide=on, \
        ID=C1333740, Type=IsoTherm, Digits=8, PLow=20, PHigh=880, PInc=10, T=<T>, \
        TUnit=K, PUnit=bar, DUnit=kg/m3, HUnit=kJ/kg, RefState=DEF. Save the four \
        files and point CF_NIST_H2_ISOTHERMS at the directory, then \
        `cargo test -p cf-storage --test storage -- --ignored --nocapture`. \
        ⛔ Do NOT commit the files: see NIST_VALIDATION.md.",
};

// ─────────────────────────────────────────────────────────────────────────────
// DOE Hydrogen and Fuel Cells Program Record 9013
// ─────────────────────────────────────────────────────────────────────────────

/// One theoretical isothermal figure from the record's Table 1.
#[derive(Clone, Copy, Debug)]
pub struct TheoreticalWork {
    /// Outlet pressure the figure is to, bar.
    pub to_bar: f64,
    /// The figure as printed, kWh/kg.
    pub printed: Printed,
}

/// A real machine doing the same compression, as the record reports it.
#[derive(Clone, Copy, Debug)]
pub struct StationFigure {
    /// Who the record attributes the figure to.
    pub who: &'static str,
    /// Outlet pressure, bar.
    pub to_bar: f64,
    /// Compression energy as printed, kWh/kg — compression only, no pre-cooling.
    pub printed: Printed,
    /// The record's own printed efficiency for this figure, percent, if it gives one.
    ///
    /// ★ Where present this is the **multiplicative transcription layer**: the
    /// theoretical figure divided by this one has to reproduce it.
    pub efficiency_percent: Option<Printed>,
}

/// The record itself, and everything transcribed from it.
#[derive(Clone, Copy, Debug)]
pub struct Record9013 {
    /// Provenance and terms.
    pub source: Source,
    /// Pressure the record assumes hydrogen is generated at, bar.
    ///
    /// ⚠ 20 bar against this chain's 20.68 — near, not equal. Every comparison
    /// with the record therefore computes **at the record's inlet**, and the
    /// chain's own figures compute at the chain's. Mixing the two would make a
    /// transcription check pass or fail for a reason that is not transcription.
    pub inlet_bar: f64,
    /// Temperature every theoretical figure is at, kelvin.
    ///
    /// ⛔ A laboratory convention, not this farm's tank. See [`CARRINGTON_FALL`].
    pub temperature_k: f64,
    /// Table 1's theoretical isothermal figures.
    pub theoretical: &'static [TheoreticalWork],
    /// The prose's stated increment from 350 to 440 bar, kWh/kg.
    pub increment_350_to_440: Printed,
    /// The prose's stated increment from 700 to 880 bar, kWh/kg.
    pub increment_700_to_880: Printed,
    /// Real-machine figures.
    pub station: &'static [StationFigure],
    /// The lower heating value the record uses, kWh/kg.
    pub lhv_kwh_per_kg: Printed,
    /// The record's stated band for on-site compression as a share of LHV, percent.
    pub share_of_lhv_percent: (Printed, Printed),
    /// The record's stated range for measured on-site compression energy, kWh/kg.
    pub tech_val_range_kwh_per_kg: (Printed, Printed),
}

impl Record9013 {
    /// The theoretical figure for an outlet pressure, if the record prints one.
    #[must_use]
    pub fn theoretical_at(&self, to_bar: f64) -> Option<&TheoreticalWork> {
        self.theoretical
            .iter()
            .find(|w| (w.to_bar - to_bar).abs() < f64::EPSILON)
    }

    /// **Layer 1, differential**: the table's own differences reproduce the
    /// prose's stated increments, as rounding intervals.
    ///
    /// ⚠ **Two blind spots, and the second was found by mutation, not by
    /// reading.** The first is the obvious one: a figure wrong in both table and
    /// prose by the same amount passes perfectly. The second is a resolution
    /// limit — differencing two figures printed to two decimals **doubles** the
    /// rounding slack, so a 0.10 increment is only pinned to about ±0.015. The
    /// prose could print 0.11 and this layer could not tell. It catches a
    /// transposed or dropped digit; it does not catch a last-digit slip, and
    /// `the_differential_layer_cannot_resolve_the_last_digit` measures exactly
    /// where the line falls.
    ///
    /// ⛔ That is a property of differencing rounded figures, not a tolerance
    /// chosen here. Tightening it would make the layer reject increments the
    /// source is entitled to print.
    #[must_use]
    pub fn increments_reconcile(&self) -> bool {
        let pair = |lo: f64, hi: f64, stated: &Printed| {
            match (self.theoretical_at(lo), self.theoretical_at(hi)) {
                (Some(a), Some(b)) => {
                    // The difference of two two-decimal figures carries both
                    // half-ulps, so it is compared as an interval, not a value.
                    let slack = a.printed.half_ulp() + b.printed.half_ulp();
                    let diff = b.printed.value() - a.printed.value();
                    (diff - stated.value()).abs() <= slack + stated.half_ulp()
                }
                _ => false,
            }
        };
        pair(350.0, 440.0, &self.increment_350_to_440)
            && pair(700.0, 880.0, &self.increment_700_to_880)
    }

    /// **Layer 2, multiplicative**: theoretical ÷ actual reproduces the
    /// record's own printed station efficiencies.
    ///
    /// ⚠ Blind to an error that scales both figures together — the ratio is
    /// unchanged if theoretical and actual are both wrong by a common factor.
    #[must_use]
    pub fn efficiencies_reconcile(&self) -> bool {
        let mut checked = 0usize;
        for s in self.station {
            let Some(eff) = s.efficiency_percent else {
                continue;
            };
            let Some(theory) = self.theoretical_at(s.to_bar) else {
                return false;
            };
            if s.printed.value() <= 0.0 {
                return false;
            }
            if !eff.admits(100.0 * theory.printed.value() / s.printed.value()) {
                return false;
            }
            checked += 1;
        }
        // ⛔ A layer that checked nothing would otherwise report success. The
        // record prints two efficiencies; fewer than two means the roster was
        // edited out from under the check.
        checked >= 2
    }

    /// **Layer 3, cross-document**: the record's heating value against another
    /// document's.
    ///
    /// The record prints 33.3 kWh/kg in passing; `cf_electrolysis::AFDC_2026`
    /// carries a four-figure value from a different DOE publication. They are
    /// independent transcriptions of the same physical constant, so they are a
    /// check on each other — but only within the coarser one's precision.
    ///
    /// ⚠ Blind to anything both documents got wrong alike, which for a widely
    /// reprinted constant is not a remote possibility.
    #[must_use]
    pub fn lhv_agrees_with(&self, kwh_per_kg: f64) -> bool {
        self.lhv_kwh_per_kg.admits(kwh_per_kg)
    }

    /// The record's own "5–20% of LHV" band, recomputed from its own measured range.
    ///
    /// Returns the low and high shares as percentages.
    #[must_use]
    pub fn share_of_lhv_from_measurements(&self) -> (f64, f64) {
        let lhv = self.lhv_kwh_per_kg.value();
        (
            100.0 * self.tech_val_range_kwh_per_kg.0.value() / lhv,
            100.0 * self.tech_val_range_kwh_per_kg.1.value() / lhv,
        )
    }
}

/// DOE Hydrogen and Fuel Cells Program Record 9013.
///
/// *"Energy requirements for hydrogen gas compression and liquefaction as
/// related to vehicle storage needs"*, Monterey Gardiner, 7 July 2009, approved
/// 26 October 2009. Only the compression half is transcribed; the liquefaction
/// half is a different storage technology and nothing in this chain uses it.
pub const RECORD_9013: Record9013 = Record9013 {
    source: Source {
        document: "DOE Hydrogen and Fuel Cells Program Record 9013, \
                   \u{201c}Energy requirements for hydrogen gas compression and \
                   liquefaction as related to vehicle storage needs\u{201d}, \
                   Monterey Gardiner, 7 July 2009",
        url: "https://www.hydrogen.energy.gov/pdfs/9013_energy_requirements_for_hydrogen_gas_compression.pdf",
        retrieved: "2026-09-18",
        terms: US_GOV_PUBLIC_DOMAIN,
    },
    inlet_bar: 20.0,
    temperature_k: 300.0,
    theoretical: &[
        TheoreticalWork {
            to_bar: 350.0,
            printed: Printed::new(1.05, 2),
        },
        TheoreticalWork {
            to_bar: 440.0,
            printed: Printed::new(1.15, 2),
        },
        TheoreticalWork {
            to_bar: 700.0,
            printed: Printed::new(1.35, 2),
        },
        TheoreticalWork {
            to_bar: 880.0,
            printed: Printed::new(1.47, 2),
        },
    ],
    increment_350_to_440: Printed::new(0.10, 2),
    increment_700_to_880: Printed::new(0.12, 2),
    station: &[
        StationFigure {
            who: "H2A Delivery Scenario Model (HDSAM v2.0.6)",
            to_bar: 440.0,
            printed: Printed::new(2.23, 2),
            efficiency_percent: Some(Printed::new(52.0, 0)),
        },
        StationFigure {
            who: "H2A Delivery Scenario Model (HDSAM v2.0.6)",
            to_bar: 880.0,
            printed: Printed::new(3.0, 1),
            efficiency_percent: Some(Printed::new(49.0, 0)),
        },
        StationFigure {
            who: "Air Products and Chemicals, personal communication",
            to_bar: 440.0,
            printed: Printed::new(2.05, 2),
            efficiency_percent: None,
        },
        StationFigure {
            who: "Air Products and Chemicals, personal communication",
            to_bar: 880.0,
            printed: Printed::new(2.67, 2),
            efficiency_percent: None,
        },
    ],
    lhv_kwh_per_kg: Printed::new(33.3, 1),
    share_of_lhv_percent: (Printed::new(5.0, 0), Printed::new(20.0, 0)),
    tech_val_range_kwh_per_kg: (Printed::new(1.7, 1), Printed::new(6.4, 1)),
};

/// ⚠ The record prints its 700 bar theoretical figure **twice, differently**.
///
/// The opening Item section says *"only 1.36 kWh/kg H2 for 700 bar"*; Table 1
/// says **1.35**. The two roundings do not overlap, so at most one of them is
/// what was computed.
///
/// ★ Carried rather than resolved, exactly as `cf_electrolysis`'s
/// `HHV_SOURCE_SPREAD_PERCENT` is. Picking one would be a determination on no
/// evidence, and a later silent edit to either constant is precisely the drift
/// a recorded spread catches. `the_record_disagrees_with_itself_about_700_bar`
/// fails if they are ever made equal.
///
/// ⚠⚠ It is not an isolated blemish. **Two of Table 1's four** theoretical
/// figures — 700 and 880 bar — sit below the current NIST equation of state in
/// the same direction, while 350 and 440 agree. See
/// [`RECORD_DISAGREEMENT_WITH_CURRENT_NIST_PERCENT`].
pub const SEVEN_HUNDRED_BAR_AS_TABLED: Printed = Printed::new(1.35, 2);
/// The same quantity as the record's Item section prints it, kWh/kg.
pub const SEVEN_HUNDRED_BAR_AS_ABSTRACTED: Printed = Printed::new(1.36, 2);

/// How far the record's theoretical figures sit from the current NIST EOS, percent.
///
/// ⛔ **An explicit unknown, not an explanation.** The record cites "NIST" in
/// 2009 and names no equation-of-state version; the `WebBook`'s hydrogen EOS has
/// been revised since. Whether that is the cause **has not been isolated**, and
/// a plausible story about it would be a permanent liability rather than a
/// finding. What is measured is the size and the direction: **two of Table 1's
/// four** figures disagree — 700 bar low by 1.30% and 880 bar low by 1.08%,
/// mean 1.19% — while 350 and 440 bar agree within their own printed rounding.
///
/// ★ It does not move the chain. Compression is a ~3% debit, so 1.2% inside it
/// is 0.04% of the headline — which is itself the useful conclusion.
///
/// ⚠ **Now measured**: `the_record_sits_below_the_current_nist_eos` reproduces
/// both figures against the oracle. This constant previously had no producer at
/// all while its documentation read as a measurement.
pub const RECORD_DISAGREEMENT_WITH_CURRENT_NIST_PERCENT: f64 = 1.2;

// ─────────────────────────────────────────────────────────────────────────────
// The temperature the tank is actually at
// ─────────────────────────────────────────────────────────────────────────────

/// Measured air temperature over a span of the year at one station.
///
/// ⛔ This type exists because the alternative is a literal. Every figure in
/// [`RECORD_9013`] is at 300 K, and adopting that would have made the farm's
/// tank 27 K warmer than it is — an 8% error on both outputs, hidden inside a
/// number that reads as measured. `cf_wind::Air` exists for the same reason and
/// caught the same class of thing at 484 m of elevation.
#[derive(Clone, Copy, Debug)]
pub struct AmbientWindow {
    /// NOAA/NCEI station identifier.
    pub station_id: &'static str,
    /// Station name as NCEI records it.
    pub station_name: &'static str,
    /// Calendar year observed.
    pub year: u16,
    /// What span this covers, in words.
    pub span: &'static str,
    /// Quality-passed hourly observations in the span.
    pub observations: usize,
    /// Mean temperature over the span, kelvin.
    pub mean_k: f64,
    /// Coldest observation in the span, kelvin.
    pub min_k: f64,
    /// Warmest observation in the span, kelvin.
    ///
    /// ★★ **This is the one a tank is sized at, not the mean.** Hydrogen
    /// expands as it warms, so the warmest hour is when the least mass fits in
    /// a fixed volume. Sizing at the mean would build a tank that is too small
    /// on the afternoons it is most needed.
    pub max_k: f64,
    /// Where the observations came from.
    pub source: Source,
    /// How to obtain them again.
    pub reproduce: &'static str,
}

/// How to pull either window again.
const NOAA_TMP_REPRODUCE: &str = concat!(
    "GET https://www.ncei.noaa.gov/access/services/data/v1 with ",
    "dataset=global-hourly, stations=72073700266, startDate=2012-01-01, ",
    "endDate=2012-12-31, dataTypes=TMP, format=csv; the TMP field is ",
    "\u{201c}tenths of °C, quality code\u{201d} — keep quality codes 1 and 5, ",
    "drop the 9999 missing sentinel, and restrict to the days of the span. ",
    "⚠ Same station and same quality gate cf_wind::CARRINGTON_AIRPORT uses for wind."
);

/// The NOAA station, as a source.
const NOAA_GLOBAL_HOURLY: Source = Source {
    document: "NOAA/NCEI Integrated Surface Database, global-hourly, station \
               72073700266 CARRINGTON MUNICIPAL AIRPORT, ND US",
    url: "https://www.ncei.noaa.gov/access/services/data/v1",
    retrieved: "2026-09-18",
    terms: US_GOV_PUBLIC_DOMAIN,
};

/// 15 October – 4 November 2012 at Carrington: the candidate tillage window.
///
/// ⚠⚠ **The dates are a candidate, not a finding.** NDSU has primary tillage in
/// eastern North Dakota as a *fall* operation bounded by harvest completion and
/// soil freeze-up; the actual window has to be pinned from USDA NASS
/// crop-progress and that is stage 4. What is measured here is the temperature
/// *given* those dates. ⛔ How much a different fall window would shift it is
/// **not measured here** — that needs the observation series, which this crate
/// does not carry, and guessing would put a number where a retrieval belongs.
pub const CARRINGTON_FALL: AmbientWindow = AmbientWindow {
    station_id: "72073700266",
    station_name: "CARRINGTON MUNICIPAL AIRPORT, ND US",
    year: 2012,
    span: "15 October – 4 November (days 289–309), 21 days",
    observations: 876,
    mean_k: 273.143_150_684_931_47,
    min_k: 265.15,
    max_k: 286.15,
    source: NOAA_GLOBAL_HOURLY,
    reproduce: NOAA_TMP_REPRODUCE,
};

/// All of 2012 at Carrington — for a buffer that stands through the year.
pub const CARRINGTON_YEAR: AmbientWindow = AmbientWindow {
    station_id: "72073700266",
    station_name: "CARRINGTON MUNICIPAL AIRPORT, ND US",
    year: 2012,
    span: "1 January – 31 December, the whole year",
    observations: 22_253,
    mean_k: 278.634_698_692_311_13,
    min_k: 245.15,
    max_k: 307.15,
    source: NOAA_GLOBAL_HOURLY,
    reproduce: NOAA_TMP_REPRODUCE,
};

// ─────────────────────────────────────────────────────────────────────────────
// The tank
// ─────────────────────────────────────────────────────────────────────────────

/// A vessel at a working pressure, at a temperature.
///
/// ⛔ **Both, always.** A pressure alone does not say how much hydrogen is in a
/// tank — over this farm's year the same vessel at the same pressure holds 25%
/// more in January than in July. `cf_electrolysis::AnnualHydrogen` carries an
/// outlet pressure in the type for the same reason: the boundary condition
/// travels with the number, because prose is what gets dropped when a number is
/// quoted.
#[derive(Clone, Copy, Debug)]
pub struct Tank {
    /// Working pressure, bar.
    pub working_pressure_bar: f64,
    /// Gas temperature, kelvin.
    pub temperature_k: f64,
}

impl Tank {
    /// A tank at a pressure and temperature.
    #[must_use]
    pub const fn new(working_pressure_bar: f64, temperature_k: f64) -> Self {
        Self {
            working_pressure_bar,
            temperature_k,
        }
    }

    /// Hydrogen a given internal volume holds, kg.
    ///
    /// Returns `None` for a volume that is not positive and finite, or a state
    /// the equation of state will not answer for.
    #[must_use]
    pub fn capacity_kg<E: EquationOfState + ?Sized>(&self, volume_m3: f64, eos: &E) -> Option<f64> {
        if !volume_m3.is_finite() || volume_m3 <= 0.0 {
            return None;
        }
        Some(volume_m3 * eos.density_kg_per_m3(self.working_pressure_bar, self.temperature_k)?)
    }

    /// Internal volume needed to hold a given mass, m³.
    ///
    /// Returns `None` for a mass that is not positive and finite, or a state
    /// the equation of state will not answer for.
    #[must_use]
    pub fn volume_m3<E: EquationOfState + ?Sized>(&self, kg: f64, eos: &E) -> Option<f64> {
        if !kg.is_finite() || kg <= 0.0 {
            return None;
        }
        Some(kg / eos.density_kg_per_m3(self.working_pressure_bar, self.temperature_k)?)
    }

    /// Electricity to fill this tank from `from_bar`, kWh/kg.
    ///
    /// `station_efficiency` is the fraction of a real machine's electricity that
    /// ends up as reversible compression work — the record publishes this
    /// directly, as 52% to 440 bar and 49% to 880 bar, so it is transcribed
    /// rather than invented. Pass `1.0` for the reversible floor.
    ///
    /// ⚠ **Isothermal, so the floor is a real floor.** A perfectly intercooled
    /// machine approaches it from above and no machine goes below it. The
    /// efficiency is what carries every real loss, which is why it must come
    /// from a measurement and not from a plausible number.
    ///
    /// Returns `None` for an efficiency outside `(0, 1]` or a state the
    /// equation of state will not answer for.
    #[must_use]
    pub fn compression_kwh_per_kg<E: EquationOfState + ?Sized>(
        &self,
        from_bar: f64,
        eos: &E,
        station_efficiency: f64,
    ) -> Option<f64> {
        if !station_efficiency.is_finite() || station_efficiency <= 0.0 || station_efficiency > 1.0
        {
            return None;
        }
        let w = eos.isothermal_work_kwh_per_kg(
            from_bar,
            self.working_pressure_bar,
            self.temperature_k,
        )?;
        Some(w / station_efficiency)
    }
}

/// Kilograms that survive paying for their own compression.
///
/// ★★ **The compression is paid out of the same wind.** The electricity is
/// fixed by stage 1, so spending some of it on a compressor does not raise the
/// bill — it lowers the kilograms. With `e` kWh/kg of electrolysis and `w`
/// kWh/kg of compression, the same energy yields `outlet · e/(e + w)`, which is
/// exact rather than iterative.
#[derive(Clone, Copy, Debug)]
pub struct Delivered {
    /// Kilograms at tank pressure, after compression is paid for.
    pub kg: f64,
    /// Kilograms at the electrolyser outlet, before it is.
    pub outlet_kg: f64,
    /// Electrolysis energy, kWh/kg.
    pub electrolysis_kwh_per_kg: f64,
    /// Compression energy, kWh/kg of hydrogen compressed.
    pub compression_kwh_per_kg: f64,
    /// Total electricity that went into compression, kWh.
    pub compression_kwh: f64,
    /// Share of the outlet kilograms lost to compression, percent.
    pub lost_percent: f64,
}

/// Debit compression from an outlet figure.
///
/// Returns `None` unless every input is finite, the masses and energies are
/// positive, and the electrolysis figure is non-zero.
#[must_use]
pub fn debit_compression(
    outlet_kg: f64,
    electrolysis_kwh_per_kg: f64,
    compression_kwh_per_kg: f64,
) -> Option<Delivered> {
    let finite = outlet_kg.is_finite()
        && electrolysis_kwh_per_kg.is_finite()
        && compression_kwh_per_kg.is_finite();
    if !finite || outlet_kg < 0.0 || electrolysis_kwh_per_kg <= 0.0 || compression_kwh_per_kg < 0.0
    {
        return None;
    }
    let total = electrolysis_kwh_per_kg + compression_kwh_per_kg;
    let kg = outlet_kg * electrolysis_kwh_per_kg / total;
    Some(Delivered {
        kg,
        outlet_kg,
        electrolysis_kwh_per_kg,
        compression_kwh_per_kg,
        compression_kwh: kg * compression_kwh_per_kg,
        lost_percent: if outlet_kg > 0.0 {
            100.0 * (outlet_kg - kg) / outlet_kg
        } else {
            0.0
        },
    })
}

// ─────────────────────────────────────────────────────────────────────────────
// Demand, and the buffer it implies
// ─────────────────────────────────────────────────────────────────────────────

/// When the farm wants hydrogen, sample by sample.
///
/// ★ A trait, because **when** is the whole question. Hydrogen accumulates all
/// year and tillage happens in three weeks; a buffer sized against a flat draw
/// answers a question nobody asked. The shape of demand is stage 4's to supply,
/// so this crate takes it as a seam rather than deciding it.
pub trait Demand {
    /// Name, for reporting which profile produced a number.
    fn name(&self) -> &'static str;

    /// Number of samples the profile covers.
    fn samples(&self) -> usize;

    /// Kilograms wanted in the sample at `index`. Zero past the end.
    fn kg_at(&self, index: usize) -> f64;

    /// Total kilograms over the whole profile.
    fn total_kg(&self) -> f64 {
        (0..self.samples()).map(|i| self.kg_at(i)).sum()
    }
}

/// The same draw in every sample of the year.
///
/// ⚠ Kept as the **foil**, not as a candidate. A farm does not till in January,
/// and the difference between this and [`SeasonalDemand`] is most of what this
/// crate has to say about storage.
#[derive(Clone, Copy, Debug)]
pub struct FlatDemand {
    total_kg: f64,
    samples: usize,
}

impl FlatDemand {
    /// A flat draw of `total_kg` spread over `samples` samples.
    #[must_use]
    pub const fn new(total_kg: f64, samples: usize) -> Self {
        Self { total_kg, samples }
    }
}

impl Demand for FlatDemand {
    fn name(&self) -> &'static str {
        "flat, every sample of the year"
    }

    fn samples(&self) -> usize {
        self.samples
    }

    fn kg_at(&self, index: usize) -> f64 {
        if index >= self.samples || self.samples == 0 {
            return 0.0;
        }
        self.total_kg / lossless_count(self.samples)
    }
}

/// A single operating window, drawn evenly across it.
///
/// ⚠ Evenly is a simplification with a stated direction: a real tillage day is
/// a burst and a rained-out day is nothing, so this understates the intra-window
/// peak and therefore the buffer. Recorded in [`TANK_SIZE_CAVEATS`] with a
/// measured size rather than left implicit.
#[derive(Clone, Copy, Debug)]
pub struct SeasonalDemand {
    total_kg: f64,
    start_index: usize,
    window_samples: usize,
    samples: usize,
}

impl SeasonalDemand {
    /// `total_kg` drawn evenly over `window_days` from `start_day_of_year`.
    ///
    /// `start_day_of_year` is 1-based, as crop-progress reporting is.
    ///
    /// Returns `None` if the interval is not positive and finite, if the window
    /// is empty, or if the window does not fit inside `samples`.
    #[must_use]
    pub fn new(
        total_kg: f64,
        start_day_of_year: u16,
        window_days: u16,
        samples: usize,
        interval_seconds: f64,
    ) -> Option<Self> {
        if !interval_seconds.is_finite() || interval_seconds <= 0.0 || window_days == 0 {
            return None;
        }
        let per_day = 86_400.0 / interval_seconds;
        let start_index = to_index((f64::from(start_day_of_year) - 1.0) * per_day)?;
        let window_samples = to_index(f64::from(window_days) * per_day)?;
        if window_samples == 0 || start_index.checked_add(window_samples)? > samples {
            return None;
        }
        Some(Self {
            total_kg,
            start_index,
            window_samples,
            samples,
        })
    }

    /// First sample of the window.
    #[must_use]
    pub const fn start_index(&self) -> usize {
        self.start_index
    }

    /// Number of samples the window spans.
    #[must_use]
    pub const fn window_samples(&self) -> usize {
        self.window_samples
    }
}

impl Demand for SeasonalDemand {
    fn name(&self) -> &'static str {
        "one operating window, drawn evenly"
    }

    fn samples(&self) -> usize {
        self.samples
    }

    fn kg_at(&self, index: usize) -> f64 {
        let inside = index >= self.start_index && index < self.start_index + self.window_samples;
        if inside {
            self.total_kg / lossless_count(self.window_samples)
        } else {
            0.0
        }
    }
}

/// A sample count as an `f64`.
///
/// Sample counts here are of order 10⁵ — exactly representable, so this is a
/// widening rather than a rounding.
#[expect(
    clippy::cast_precision_loss,
    reason = "a year of 5-minute samples is ~10^5 and exact in f64"
)]
const fn lossless_count(n: usize) -> f64 {
    n as f64
}

/// A non-negative count of samples, from a float, or `None` if it is not one.
#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "guarded above: finite, non-negative, below 2^53; truncation to a \
              whole sample index is the intent"
)]
fn to_index(x: f64) -> Option<usize> {
    // NaN and the infinities fail `contains`, so this is the finiteness check too.
    (0.0..9_007_199_254_740_992.0)
        .contains(&x)
        .then_some(x.trunc() as usize)
}

/// What a tank of a given size did over a year.
#[derive(Clone, Copy, Debug)]
pub struct Reservoir {
    /// Size of the tank, kg.
    pub capacity_kg: f64,
    /// Hydrogen in it at the first sample, kg.
    pub initial_kg: f64,
    /// Hydrogen left in it at the last, kg.
    pub final_kg: f64,
    /// Fullest it ever got, kg.
    pub peak_kg: f64,
    /// Hydrogen offered by production over the year, kg.
    pub produced_kg: f64,
    /// Hydrogen the demand asked for, kg.
    pub demanded_kg: f64,
    /// Hydrogen the demand actually got, kg.
    pub delivered_kg: f64,
    /// Production discarded because the tank was already full, kg.
    pub curtailed_kg: f64,
    /// Demand that went unmet, kg.
    pub shortfall_kg: f64,
    /// Samples seen.
    pub samples: usize,
    /// Samples whose production was not a finite number, and were skipped.
    pub samples_not_finite: usize,
    /// Samples that ended with the tank full.
    pub samples_full: usize,
    /// Samples that ended with the tank empty.
    pub samples_empty: usize,
}

impl Reservoir {
    /// Whether every kilogram asked for was delivered.
    #[must_use]
    pub fn meets_demand(&self) -> bool {
        self.shortfall_kg <= 0.0
    }

    /// Mass that the balance fails to account for, kg.
    ///
    /// ★★ **Exhaustive, and every term has to be live for that to mean
    /// anything.** What went in is what came out plus what is still there:
    /// `initial + produced − delivered − curtailed − final`. A three-term
    /// identity tested where one term is always zero passes with that term
    /// deleted, which is how stage 2 shipped an energy balance that could not
    /// fail; `the_mass_balance_is_exhaustive` runs where all of them move.
    #[must_use]
    pub fn balance_residual_kg(&self) -> f64 {
        (self.initial_kg + self.produced_kg)
            - (self.delivered_kg + self.curtailed_kg + self.final_kg)
    }
}

/// Run a tank of a fixed size against a year of production and demand.
///
/// Production arrives first in each sample, then demand is served — a tank that
/// is filled and emptied within one interval does not overflow. Anything that
/// will not fit is **curtailed**: the electrolyser stops rather than the tank
/// bursting, which is what a real plant does and what makes curtailment a
/// reportable term instead of a silent loss.
///
/// ⚠ A non-finite production sample is counted and skipped rather than allowed
/// to poison the totals. `cf-electrolysis` shipped without that guard and a
/// single infinity produced a finite-looking mass with a broken balance.
pub fn run_tank<I, D>(production_kg: I, demand: &D, capacity_kg: f64, initial_kg: f64) -> Reservoir
where
    I: IntoIterator<Item = f64>,
    D: Demand + ?Sized,
{
    let capacity = if capacity_kg.is_finite() && capacity_kg > 0.0 {
        capacity_kg
    } else {
        0.0
    };
    let start = if initial_kg.is_finite() {
        initial_kg.clamp(0.0, capacity)
    } else {
        0.0
    };

    let mut level = start;
    let (mut produced, mut demanded, mut delivered) = (0.0, 0.0, 0.0);
    let (mut curtailed, mut shortfall, mut peak) = (0.0, 0.0, start);
    let (mut n, mut not_finite, mut full, mut empty) = (0usize, 0usize, 0usize, 0usize);

    for p in production_kg {
        let index = n;
        n += 1;
        if p.is_finite() && p >= 0.0 {
            produced += p;
            level += p;
            if level > capacity {
                curtailed += level - capacity;
                level = capacity;
            }
        } else {
            not_finite += 1;
        }
        if level >= capacity {
            full += 1;
        }
        peak = peak.max(level);

        let want = demand.kg_at(index);
        if want.is_finite() && want > 0.0 {
            demanded += want;
            if want > level {
                shortfall += want - level;
                delivered += level;
                level = 0.0;
            } else {
                delivered += want;
                level -= want;
            }
        }
        if level <= 0.0 {
            empty += 1;
        }
    }

    Reservoir {
        capacity_kg: capacity,
        initial_kg: start,
        final_kg: level,
        peak_kg: peak,
        produced_kg: produced,
        demanded_kg: demanded,
        delivered_kg: delivered,
        curtailed_kg: curtailed,
        shortfall_kg: shortfall,
        samples: n,
        samples_not_finite: not_finite,
        samples_full: full,
        samples_empty: empty,
    }
}

/// Smallest tank that meets every kilogram of a demand, kg.
///
/// `initial_fraction` is how full the tank is at the first sample, as a
/// fraction of its own size. ⛔ **Anything above zero is hydrogen from outside
/// the series**, and it shows up in [`Reservoir::balance_residual_kg`] as an
/// `initial_kg` term rather than vanishing — pass `0.0` for a tank that starts
/// empty and has to earn everything it delivers.
///
/// Returns `None` if the inputs are not usable, or if no tank meets the demand
/// because the production simply is not there in time.
///
/// ★★ The answer is a **minimum**, and that is a testable claim rather than a
/// description: `a_tank_below_the_minimum_does_not_meet_the_demand` shrinks the
/// result and watches it fail.
#[must_use]
pub fn minimum_tank_kg<D: Demand + ?Sized>(
    production_kg: &[f64],
    demand: &D,
    initial_fraction: f64,
) -> Option<f64> {
    if !initial_fraction.is_finite() || !(0.0..=1.0).contains(&initial_fraction) {
        return None;
    }
    let total = demand.total_kg();
    if !total.is_finite() || total < 0.0 {
        return None;
    }
    let meets = |size: f64| {
        run_tank(
            production_kg.iter().copied(),
            demand,
            size,
            size * initial_fraction,
        )
        .meets_demand()
    };
    if total == 0.0 || meets(0.0) {
        return Some(0.0);
    }
    // A tank larger than the whole demand cannot help: it can hold no more than
    // the demand needs, so if this fails nothing succeeds.
    let mut hi = total;
    if !meets(hi) {
        return None;
    }
    let mut lo = 0.0;
    // Fixed iteration count: 64 halvings take any starting interval below the
    // resolution of an f64, so this terminates by construction rather than by a
    // tolerance that could be met differently on another target.
    for _ in 0..64 {
        let mid = 0.5 * (lo + hi);
        if mid <= lo || mid >= hi {
            break;
        }
        if meets(mid) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    Some(hi)
}

/// Production that arrives while the demand is actually drawing, kg.
///
/// ★★★ **This is where the cliff is.** Below it the tank is a rounding error;
/// above it the tank has to carry the shortfall in from months earlier and
/// grows without a useful bound. The number is a property of the resource and
/// the window, and it is computed rather than asserted:
/// `the_storage_cliff_is_at_in_window_production` measures the tank on both
/// sides of it.
#[must_use]
pub fn produced_during_demand<D: Demand + ?Sized>(production_kg: &[f64], demand: &D) -> f64 {
    production_kg
        .iter()
        .enumerate()
        .filter(|&(i, p)| p.is_finite() && *p > 0.0 && demand.kg_at(i) > 0.0)
        .map(|(_, p)| *p)
        .sum()
}

// ─────────────────────────────────────────────────────────────────────────────
// What these numbers are not
// ─────────────────────────────────────────────────────────────────────────────

/// Something true about these numbers that the numbers themselves do not say.
///
/// ★★ Every caveat carries how far it moves its headline, because a flat list
/// of worries gets weighted by the emphasis of its prose rather than by its
/// size. The figures are measured by `the_caveats_are_ranked_by_measured_effect`.
#[derive(Clone, Copy, Debug)]
pub struct Caveat {
    /// Short name.
    pub what: &'static str,
    /// Why it matters, and what alternative assumption the swing is the effect of.
    pub why: &'static str,
    /// How far this term moves **its own list's headline**, percent, signed.
    pub headline_swing_percent: f64,
}

/// ⚠ What the delivered kilograms are not, **largest effect first**.
///
/// ⛔⛔ **Deliberately not one list with [`TANK_SIZE_CAVEATS`].** These two
/// rank against different denominators: compression is a ~3% debit on the
/// kilograms, so a 10% error inside it is 0.3% of that headline, while the same
/// equation of state is a first-order term on tank volume. A single sorted list
/// would put a 10%-of-a-small-thing above a 2%-of-a-big-thing and read as if it
/// mattered more. `cf-electrolysis` learned the neighbouring version of this
/// when integrity findings and sensitivity terms shared a list.
pub const DELIVERED_KILOGRAMS_CAVEATS: &[Caveat] = &[
    Caveat {
        what: "the compressor is a refuelling station's, not a farm's",
        why: "The efficiency that turns reversible work into electricity — 52% to \
              440 bar — is the record's figure for a 1,000 kg/day station with a \
              reciprocating compressor. A farm is orders of magnitude smaller. \
              Swing shown is the record's own spread between its two sources at \
              440 bar, HDSAM 2.23 against Air Products 2.05 kWh/kg.",
        headline_swing_percent: 0.23,
    },
    Caveat {
        what: "the tank is at the window's mean temperature, not its warmest hour",
        why: "Compression work rises with temperature. Swing shown is the measured \
              window mean against the window maximum, 273.14 K against 286.15 K.",
        headline_swing_percent: -0.15,
    },
    Caveat {
        what: "the source's own figures disagree with the current NIST EOS",
        why: "Three of Record 9013's five theoretical figures sit about 1.2% below \
              both the current NIST equation of state and this crate's model. \
              Swing shown is that disagreement carried through the compression term. \
              See RECORD_DISAGREEMENT_WITH_CURRENT_NIST_PERCENT.",
        headline_swing_percent: 0.04,
    },
];

/// ⚠ What the tank size is not, **largest effect first**.
pub const TANK_SIZE_CAVEATS: &[Caveat] = &[
    Caveat {
        what: "the demand is drawn evenly across the window",
        why: "A real tillage window is working days and rained-out days, not a \
              constant trickle. Swing shown is the same total drawn over 12 days \
              instead of 21 — the same work, concentrated.",
        headline_swing_percent: 5.97,
    },
    Caveat {
        what: "the tank is sized at the window's warmest hour",
        why: "Hydrogen expands as it warms, so the warmest hour is when the least \
              mass fits and the tank has to be biggest. Swing shown is what sizing \
              at the window MEAN instead would save — 273.14 K against 286.15 K. \
              ⚠ It is a saving that fails on the afternoons the tank is most needed, \
              which is why the larger figure is the one used.",
        headline_swing_percent: -4.18,
    },
    Caveat {
        what: "the equation of state is two-term",
        why: "The covolume model overstates density, so it understates the vessel. \
              Swing shown is the measured error against NIST at 350 bar and 273.15 K \
              — the tank's own temperature, not the source document's 300 K — \
              carried straight into volume. See NIST_DENSITY_COMPARISON.",
        headline_swing_percent: 2.20,
    },
];

/// Something that matters and has **no measured magnitude**.
///
/// ⛔⛔ A separate type from [`Caveat`] and a separate list, because a ranked
/// list places whatever you put in it. Giving an unmeasured risk a zero would
/// sort it last, which is a claim about its size that nothing supports.
#[derive(Clone, Copy, Debug)]
pub struct Unknown {
    /// Short name.
    pub what: &'static str,
    /// Why no magnitude is given.
    pub why_unmeasured: &'static str,
    /// What would actually settle it.
    pub what_would_measure_it: &'static str,
}

/// ⚠ Risks at farm scale with no measured size.
pub const UNMEASURED_AT_FARM_SCALE: &[Unknown] = &[
    Unknown {
        what: "months of standing storage, rather than a station's hours",
        why_unmeasured: "Every figure in Record 9013 is about a refuelling station, \
             where hydrogen is compressed and dispensed within a day. This farm's \
             tank holds gas from spring to October. Permeation through a polymer \
             liner, seal creep and venting on temperature swings are all \
             time-integrated, and none of them appear in a source written about a \
             different duty cycle.",
        what_would_measure_it: "A published permeation or boil-off rate for a Type \
             III or Type IV vessel held at working pressure for months, or a \
             standing-loss measurement from a seasonal storage installation.",
    },
    Unknown {
        what: "the transfer into the tractor's own tank",
        why_unmeasured: "Filling a vessel heats the gas in it, which is why SAE \
             J2601 has cars pre-cooled to −40 °C. How much that costs here depends \
             on the tractor's tank type, its size and how fast the operator expects \
             to fill it — none of which exist yet. The record carries 0.15–0.45 \
             kWh/kg for automotive pre-cooling, which is an order of magnitude and \
             not this machine's number.",
        what_would_measure_it: "Stage 4's tractor: a tank type, a volume and a fill \
             time, after which J2601's thermal limits give a pre-cooling duty.",
    },
];
