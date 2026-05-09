//! Silicone material reference — Yeoh-calibrated `pub const` anchors.
//!
//! Covers every Smooth-On platinum-cure silicone the
//! layered-silicone-device arc currently considers as a body-layer or
//! rigid-mold material.
//!
//! Engineering-grade lookup, not "guess your Lamé params from a search
//! bar." Every value here is sourced from Smooth-On's published
//! technical data sheets ([www.smooth-on.com](https://www.smooth-on.com))
//! under the assumptions documented in §Conversion below; consumers
//! that need calibrated post-cast values overwrite per Fork B
//! (sim-soft is a relative-comparison tool, not an absolute predictor;
//! ν = 0.40 locking is absorbed by the post-cast modulus fit).
//!
//! # Table
//!
//! | Material        | Shore | μ (kPa) | λ (kPa) | C₂ (kPa) | ρ (kg/m³) | `λ_break` | max σ | `σ_100` |
//! |-----------------|-------|--------:|--------:|---------:|----------:|--------:|------:|---------|
//! | Ecoflex 00-10   | 00-10 |    18.0 |    72.0 |     1.69 |      1040 |    9.00 |  7.20 |  8 PSI  |
//! | Ecoflex 00-20   | 00-20 |    18.0 |    72.0 |     1.69 |      1070 |    9.45 |  7.56 |  8 PSI  |
//! | Ecoflex 00-30   | 00-30 |    23.0 |    92.0 |     2.05 |      1070 |   10.00 |  8.00 | 10 PSI  |
//! | Ecoflex 00-50   | 00-50 |    28.0 |   112.0 |     2.41 |      1070 |   10.80 |  8.64 | 12 PSI  |
//! | Dragon Skin 10A | 10A   |    51.0 |   204.0 |     4.46 |      1070 |   11.00 |  8.80 | 22 PSI  |
//! | Dragon Skin 15  | 15A   |    92.0 |   368.0 |     8.20 |      1070 |    8.71 |  6.97 | 40 PSI  |
//! | Dragon Skin 20A | 20A   |   113.0 |   452.0 |    10.00 |      1080 |    7.20 |  5.76 | 49 PSI  |
//! | Dragon Skin 30A | 30A   |   198.0 |   792.0 |    17.60 |      1080 |    4.64 |  3.71 | 86 PSI  |
//!
//! `min σ` (compressive validity bound) is family-uniform at `0.30`
//! (70 % compression cap; engineering-aggressive default per Yeoh arc
//! memo D8 — no universal Yeoh-on-silicone literature bound exists,
//! replace with anchor-specific measured value once compression-set
//! test data lands).
//!
//! Ecoflex 00-10 and 00-20 share a 100% modulus on Smooth-On's data
//! sheet (~8 PSI) — Shore 00 hardness differentiates them by
//! indentation resistance more than by elongation modulus, and the
//! shared `σ_100` is faithful to the published sheets rather than
//! interpolated to force a strict ordering. Density differs by 30
//! kg/m³ (1040 vs 1070) per the `00-10` and `00-20` data sheets'
//! specific-gravity rows. Their Yeoh `(C₁, C₂)` pairs are therefore
//! identical; their validity bounds differ via `λ_break`.
//!
//! # Conversion
//!
//! Smooth-On data sheets report **100 % modulus** `σ_100` — engineering
//! stress at 100 % uniaxial strain — alongside tensile strength,
//! elongation at break, and specific gravity. Conversion to
//! compressible Lamé parameters at ν = 0.40 (Fork B framing) uses the
//! small-strain incompressible-NH approximation `σ_eng ≈ 3 μ` at
//! ε = 1.0:
//!
//! ```text
//! μ = σ_100 / 3                              (Pa)
//! λ = 2μν / (1 − 2ν) = 4μ      at ν = 0.40   (Pa)
//! E = 2μ(1 + ν)     = 2.8μ     at ν = 0.40   (Pa)
//! ρ = (specific gravity) · 1000              (kg/m³)
//! ```
//!
//! `σ_100 ≈ 3 μ` is the small-strain identity for the incompressible NH
//! material (`σ_eng = μ(λ_axial − 1/λ_axial²)` with the small-strain
//! linearization `λ_axial − 1 = ε`); at ε = 1.0 the linearized form
//! gives `σ_100 = 3μ` exactly. The finite-strain form
//! `σ_eng = μ(λ − 1/λ²)` evaluated at λ = 2 (ε = 1.0) gives
//! `σ_100 = 1.75μ` — but the linearized small-strain identity matches
//! how data-sheet "100 % modulus" is conventionally interpreted in the
//! soft-robotics literature (Marechal et al. 2021, Polygerinos et al.
//! 2017). The factor of ~1.7 discrepancy between 1.75μ and 3μ is the
//! kind of catalog-value uncertainty Fork B's calibration loop is
//! designed to absorb at post-cast time.
//!
//! Yeoh `C₁` and `C₂` derive from incompressible-uniaxial Yeoh at
//! λ = 2:
//!
//! ```text
//! C₁ = μ / 2                                  (small-strain consistency)
//! C₂ = (σ_100 / 3.5 − C₁) / 4                 (uniaxial at λ = 2)
//! ```
//!
//! Yeoh arc memo §"Math derivations" derives both. C₂ is positive for
//! every published Smooth-On silicone. Mooney-Rivlin's I₂ contribution
//! gives Drucker-unstable `C₀₁ < 0` on the same data and is rejected
//! per arc memo D1.
//!
//! Tensile validity cap `max_principal_stretch = 0.8 · λ_break` (80 %
//! safety margin to rupture; calibrated per anchor from TDS elongation
//! at break). Compressive cap is family-uniform at `0.30` per arc memo
//! D8.
//!
//! # Validity
//!
//! ν = 0.40 sits comfortably inside [`NeoHookean::from_young_poisson`]'s
//! ν < 0.45 standalone-compressible-law cap. Real silicones are
//! near-incompressible (ν ≈ 0.49); the 0.40 framing deliberately
//! introduces volumetric locking error that calibration absorbs into
//! the effective μ at post-cast time. Tet10 + F-bar at Phase H recovers
//! the near-incompressible regime without the ν shift.

use super::{NeoHookean, Yeoh};

/// Hardness reading on one of Smooth-On's published Shore scales.
///
/// The scale is part of the type so callers cannot conflate Shore 00
/// and Shore A readings. Wrapped value is the dimensionless durometer
/// number (e.g. `DoubleZero(30.0)` for Shore 00-30, `A(15.0)` for Shore
/// A 15).
///
/// Shore D is reserved for harder elastomers outside the current
/// silicone arc; no anchors live in that family yet. Adding a Shore D
/// anchor in a future PR is non-breaking thanks to `#[non_exhaustive]`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum ShoreReading {
    /// Shore 00 scale (Ecoflex family).
    DoubleZero(f64),
    /// Shore A scale (Dragon Skin family).
    A(f64),
    /// Shore D scale (reserved; no anchors today).
    D(f64),
}

/// Provenance tag carried alongside each [`SiliconeMaterial`].
///
/// Lets downstream artifacts (JSON capture, row prose) record whether
/// a material came from a published anchor, an anchor-bounded
/// interpolation, or a post-cast measurement, so `Path 1 / Path 2 /
/// Path 3` (Yeoh arc memo D5) is preserved through the pipeline.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum ConstructionSource {
    /// Path 1: published Smooth-On anchor (one of the `pub const`
    /// entries below). `name` is the anchor's identifier (e.g.
    /// `"ECOFLEX_00_30"`).
    Anchor {
        /// Anchor identifier (matches the `pub const` symbol).
        name: &'static str,
    },
    /// Path 2: linear interpolation in Shore space between two anchors
    /// of the same family. `weight` is `t ∈ [0, 1]` with 0 = `low_anchor`
    /// and 1 = `high_anchor`; `user_description` records the user's
    /// physical recipe (e.g. `"Ecoflex 00-30 + 50 % Slacker"`).
    Interpolated {
        /// Lower-Shore bracketing anchor.
        low_anchor: &'static str,
        /// Higher-Shore bracketing anchor.
        high_anchor: &'static str,
        /// Interpolation weight in Shore space.
        weight: f64,
        /// User-supplied recipe description, if any.
        user_description: Option<&'static str>,
    },
    /// Path 3: post-cast measurement of Shore + 100 % modulus.
    /// `user_description` is mandatory: it explains the measurement
    /// basis (cast batch, durometer + tensile setup, date).
    Measured {
        /// User-supplied measurement description.
        user_description: &'static str,
    },
}

/// Reference data for one silicone — Lamé pair, Yeoh `C₂`, density,
/// validity bounds, and provenance.
///
/// Construction paths (Yeoh arc memo D5):
///
/// 1. Use one of the `pub const` anchor entries below
///    (`ECOFLEX_00_30`, `DRAGON_SKIN_10A`, …). Source tag
///    `ConstructionSource::Anchor`.
/// 2. Build via `from_effective_shore` (anchor-bounded interpolation,
///    F2.2). Source tag `ConstructionSource::Interpolated`.
/// 3. Build via `from_measured` (post-cast measurement, F2.2). Source
///    tag `ConstructionSource::Measured`.
///
/// `#[non_exhaustive]` so future fields (3-param Yeoh `C₃`, post-cast
/// curve handles) land without breaking downstream pattern matches.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct SiliconeMaterial {
    /// First Lamé parameter (shear modulus) in pascals.
    pub mu: f64,
    /// Second Lamé parameter in pascals. At ν = 0.40 every entry
    /// satisfies `lambda == 4 * mu` (verified by the
    /// `lambda_is_four_times_mu_at_nu_0_40` unit test below).
    pub lambda: f64,
    /// Yeoh polynomial coefficient `C₂` in pascals. `C₁ = μ / 2` is
    /// derived; the table doesn't store `C₁` separately.
    pub c2: f64,
    /// Bulk density in kilograms per cubic meter.
    pub density: f64,
    /// Tensile principal-stretch cap (Yeoh validity gate, memo D8).
    /// Calibrated as `0.8 · λ_break` per anchor.
    pub validity_max_principal_stretch: f64,
    /// Compressive principal-stretch cap (Yeoh validity gate, memo D8).
    /// Family-uniform at 0.30 (engineering-aggressive default; replace
    /// with measured value once compression-set test data lands).
    pub validity_min_principal_stretch: f64,
    /// Anchor's nominal Shore reading. Drives family identification
    /// for `from_effective_shore` interpolation.
    pub shore: ShoreReading,
    /// Provenance — anchor / interpolated / measured.
    pub source: ConstructionSource,
}

impl SiliconeMaterial {
    /// Internal anchor constructor. `const fn` so the table entries
    /// below are compile-time values. Not pub: external callers go
    /// through `from_effective_shore` / `from_measured` (F2.2).
    #[allow(clippy::too_many_arguments)]
    const fn from_anchor(
        name: &'static str,
        mu: f64,
        lambda: f64,
        c2: f64,
        density: f64,
        validity_max_principal_stretch: f64,
        validity_min_principal_stretch: f64,
        shore: ShoreReading,
    ) -> Self {
        Self {
            mu,
            lambda,
            c2,
            density,
            validity_max_principal_stretch,
            validity_min_principal_stretch,
            shore,
            source: ConstructionSource::Anchor { name },
        }
    }

    /// Build a [`NeoHookean`] from the material's Lamé pair. `const`
    /// so consumer scenes can construct per-tet materials at compile
    /// time without losing the silicone-table provenance. Drops the
    /// Yeoh `C₂` and validity bounds — use `to_yeoh` for the richer
    /// material model.
    #[must_use]
    pub const fn to_neo_hookean(&self) -> NeoHookean {
        NeoHookean::from_lame(self.mu, self.lambda)
    }

    /// Build a [`Yeoh`] from the material's `(μ, λ, C₂)` and asymmetric
    /// validity bounds. `const` so consumer scenes can construct
    /// per-tet materials at compile time.
    #[must_use]
    pub const fn to_yeoh(&self) -> Yeoh {
        Yeoh::from_lame_and_c2(self.mu, self.lambda, self.c2).with_principal_stretch_bounds(
            self.validity_max_principal_stretch,
            self.validity_min_principal_stretch,
        )
    }

    /// Path 2 — anchor-bounded interpolation in Shore space within
    /// family. Bracket `shore` between two adjacent published anchors
    /// of the same scale and linearly interpolate the Lamé pair, Yeoh
    /// `C₂`, density, and tensile validity bound. Compressive bound
    /// stays family-uniform at `0.30` (no interpolation).
    ///
    /// Cross-family interpolation is rejected (memo D3): Ecoflex and
    /// Dragon Skin are different chemistries and pretending Shore
    /// 00-50 sits on a continuous curve with Shore A 5 hides this.
    ///
    /// `user_description` carries the recipe prose
    /// (`"Ecoflex 00-30 + 50 % Slacker"`) into
    /// [`ConstructionSource::Interpolated`] for downstream JSON
    /// capture and row prose.
    ///
    /// # Errors
    ///
    /// - [`ShoreInterpolationError::NoAnchorsInFamily`] when no
    ///   anchors are published for the supplied scale (Shore D today).
    /// - [`ShoreInterpolationError::OutOfRange`] when `shore`'s value
    ///   is outside the bracketing anchors' span.
    pub fn from_effective_shore(
        shore: ShoreReading,
        user_description: Option<&'static str>,
    ) -> Result<Self, ShoreInterpolationError> {
        let anchors = anchors_for_family(&shore)
            .ok_or(ShoreInterpolationError::NoAnchorsInFamily { shore })?;
        let target = shore_value(&shore);
        let (low, high, weight) =
            bracket(anchors, target).ok_or_else(|| ShoreInterpolationError::OutOfRange {
                shore,
                low_bound: shore_value(&anchors[0].shore),
                high_bound: shore_value(&anchors[anchors.len() - 1].shore),
            })?;
        let low_name = anchor_name(low);
        let high_name = anchor_name(high);
        Ok(Self {
            mu: lerp(low.mu, high.mu, weight),
            lambda: lerp(low.lambda, high.lambda, weight),
            c2: lerp(low.c2, high.c2, weight),
            density: lerp(low.density, high.density, weight),
            validity_max_principal_stretch: lerp(
                low.validity_max_principal_stretch,
                high.validity_max_principal_stretch,
                weight,
            ),
            validity_min_principal_stretch: YEOH_MIN_PRINCIPAL_STRETCH,
            shore,
            source: ConstructionSource::Interpolated {
                low_anchor: low_name,
                high_anchor: high_name,
                weight,
                user_description,
            },
        })
    }

    /// Path 3 — post-cast measurement. Derive `(μ, λ, C₂)` from the
    /// user's measured Shore reading + 100 % modulus per the F4
    /// `σ_100 / 3` correlation and the Yeoh calibration formula:
    ///
    /// ```text
    /// μ = σ_100 / 3
    /// λ = 4 μ                           (ν = 0.40 convention)
    /// C₂ = (σ_100 / 3.5 − μ / 2) / 4    (uniaxial Yeoh at λ = 2)
    /// ```
    ///
    /// Density defaults per [`silicone_default_density`] (anchor-family
    /// average). Validity bounds are open on the tensile side
    /// (`f64::INFINITY`) — the user must clamp via Fork-B compression
    /// or rupture testing if a tighter bound is needed; the
    /// compressive bound stays at `0.30`.
    ///
    /// `user_description` is mandatory: it documents the cast batch,
    /// durometer + tensile setup, and date.
    ///
    /// Substituting `μ = σ_100 / 3` into the C₂ formula collapses to
    /// `C₂ = σ_100 / 33.6`, so any positive `σ_100` yields a positive
    /// (Drucker-stable) C₂ — no negative-C₂ branch needs guarding here.
    /// The 2-parameter Yeoh model's calibration shape is the load-
    /// bearing reason this is safe; switching to 3-param Yeoh or a
    /// different correlation would require revisiting.
    ///
    /// # Errors
    ///
    /// - [`MeasuredMaterialError::NonpositiveModulus`] when
    ///   `modulus_100_pct_pa` is non-finite or `≤ 0`.
    pub fn from_measured(
        shore: ShoreReading,
        modulus_100_pct_pa: f64,
        user_description: &'static str,
    ) -> Result<Self, MeasuredMaterialError> {
        if !modulus_100_pct_pa.is_finite() || modulus_100_pct_pa <= 0.0 {
            return Err(MeasuredMaterialError::NonpositiveModulus { modulus_100_pct_pa });
        }
        let mu = modulus_100_pct_pa / 3.0;
        let lambda = 4.0 * mu;
        let c1 = 0.5 * mu;
        let c2 = (modulus_100_pct_pa / 3.5 - c1) / 4.0;
        Ok(Self {
            mu,
            lambda,
            c2,
            density: silicone_default_density(&shore),
            validity_max_principal_stretch: f64::INFINITY,
            validity_min_principal_stretch: YEOH_MIN_PRINCIPAL_STRETCH,
            shore,
            source: ConstructionSource::Measured { user_description },
        })
    }
}

/// Family default density for materials constructed via
/// [`SiliconeMaterial::from_measured`] when the user doesn't have a
/// specific-gravity reading.
///
/// Drawn from the anchor table averages: Shore 00 → 1070 kg/m³
/// (Ecoflex 00-20/30/50); Shore A → 1075 kg/m³ (`DS_10A`/15 at 1070,
/// `DS_20A`/`DS_30A` at 1080); Shore D not yet anchored.
#[must_use]
pub const fn silicone_default_density(shore: &ShoreReading) -> f64 {
    match shore {
        ShoreReading::DoubleZero(_) => 1070.0,
        ShoreReading::A(_) => 1075.0,
        ShoreReading::D(_) => 1100.0,
    }
}

/// Errors from [`SiliconeMaterial::from_effective_shore`].
#[derive(Clone, Debug)]
pub enum ShoreInterpolationError {
    /// Supplied Shore reading falls outside the bracketing anchors'
    /// span for its family.
    OutOfRange {
        /// User's Shore reading.
        shore: ShoreReading,
        /// Lowest published anchor on this scale.
        low_bound: f64,
        /// Highest published anchor on this scale.
        high_bound: f64,
    },
    /// No anchors are published for the supplied Shore scale (Shore D
    /// today). Add an anchor const to `silicone_table.rs` to unblock.
    NoAnchorsInFamily {
        /// User's Shore reading whose scale has no anchors.
        shore: ShoreReading,
    },
}

/// Errors from [`SiliconeMaterial::from_measured`].
#[derive(Clone, Debug)]
#[non_exhaustive]
pub enum MeasuredMaterialError {
    /// `modulus_100_pct_pa` was non-finite or `≤ 0`. Remeasure.
    NonpositiveModulus {
        /// As supplied by the caller.
        modulus_100_pct_pa: f64,
    },
}

// --- Internal interpolation helpers ------------------------------------

const ECOFLEX_FAMILY: &[SiliconeMaterial] =
    &[ECOFLEX_00_10, ECOFLEX_00_20, ECOFLEX_00_30, ECOFLEX_00_50];

const DRAGON_SKIN_FAMILY: &[SiliconeMaterial] = &[
    DRAGON_SKIN_10A,
    DRAGON_SKIN_15,
    DRAGON_SKIN_20A,
    DRAGON_SKIN_30A,
];

const fn anchors_for_family(shore: &ShoreReading) -> Option<&'static [SiliconeMaterial]> {
    match shore {
        ShoreReading::DoubleZero(_) => Some(ECOFLEX_FAMILY),
        ShoreReading::A(_) => Some(DRAGON_SKIN_FAMILY),
        ShoreReading::D(_) => None,
    }
}

const fn shore_value(shore: &ShoreReading) -> f64 {
    match shore {
        ShoreReading::DoubleZero(v) | ShoreReading::A(v) | ShoreReading::D(v) => *v,
    }
}

fn anchor_name(mat: &SiliconeMaterial) -> &'static str {
    match mat.source {
        ConstructionSource::Anchor { name } => name,
        // Internal helper called only on the family-anchor slices, all
        // of which are constructed via `from_anchor`.
        _ => unreachable!("family slice must contain Anchor-sourced entries"),
    }
}

fn bracket(
    anchors: &[SiliconeMaterial],
    target: f64,
) -> Option<(&SiliconeMaterial, &SiliconeMaterial, f64)> {
    let first = anchors.first()?;
    let last = anchors.last()?;
    let lo_bound = shore_value(&first.shore);
    let hi_bound = shore_value(&last.shore);
    if target < lo_bound || target > hi_bound {
        return None;
    }
    for window in anchors.windows(2) {
        let lo = shore_value(&window[0].shore);
        let hi = shore_value(&window[1].shore);
        if target >= lo && target <= hi {
            let span = hi - lo;
            // Adjacent anchors at identical Shore would zero-divide;
            // the published table has strictly increasing Shore within
            // each family, so this branch is defensive only.
            let weight = if span < f64::EPSILON {
                0.0
            } else {
                (target - lo) / span
            };
            return Some((&window[0], &window[1], weight));
        }
    }
    None
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    // `(1 - t) · a + t · b` ordering preserves bit-equality with `a`
    // at `t = 0` and with `b` at `t = 1`: at t=0 the FMA reduces to
    // `1·a + 0 = a`; at t=1 it reduces to `0·a + b = b`. The naive
    // `(b - a) · t + a` form rounds `b - a` first and may drift by
    // 1 ULP at `t = 1`.
    (1.0 - t).mul_add(a, t * b)
}

// Per-anchor Yeoh validity bounds: tensile cap = 0.8 · λ_break,
// compressive cap = 0.30 (family-uniform). λ_break per Smooth-On TDS
// elongation-at-break rows; recon table at Yeoh arc memo §"Recon
// findings".
const YEOH_MIN_PRINCIPAL_STRETCH: f64 = 0.30;

/// Ecoflex 00-10 — softest in the Ecoflex line; Shore 00-10. `λ_break`
/// = 9.0 (800 % elongation at break per TDS).
pub const ECOFLEX_00_10: SiliconeMaterial = SiliconeMaterial::from_anchor(
    "ECOFLEX_00_10",
    18_000.0,
    72_000.0,
    1_690.0,
    1040.0,
    7.20,
    YEOH_MIN_PRINCIPAL_STRETCH,
    ShoreReading::DoubleZero(10.0),
);

/// Ecoflex 00-20 — Shore 00-20. Shares 100 % modulus with 00-10 per
/// Smooth-On data sheet; density and `λ_break` differ. `λ_break` = 9.45
/// (845 % elongation at break per TDS).
pub const ECOFLEX_00_20: SiliconeMaterial = SiliconeMaterial::from_anchor(
    "ECOFLEX_00_20",
    18_000.0,
    72_000.0,
    1_690.0,
    1070.0,
    7.56,
    YEOH_MIN_PRINCIPAL_STRETCH,
    ShoreReading::DoubleZero(20.0),
);

/// Ecoflex 00-30 — Shore 00-30. Most-cited Ecoflex grade in the
/// soft-robotics literature. `λ_break` = 10.0 (900 % elongation).
pub const ECOFLEX_00_30: SiliconeMaterial = SiliconeMaterial::from_anchor(
    "ECOFLEX_00_30",
    23_000.0,
    92_000.0,
    2_050.0,
    1070.0,
    8.00,
    YEOH_MIN_PRINCIPAL_STRETCH,
    ShoreReading::DoubleZero(30.0),
);

/// Ecoflex 00-50 — Shore 00-50; firmest in the Ecoflex line. `λ_break`
/// = 10.80 (980 % elongation per TDS — `λ = 1 + ε`).
pub const ECOFLEX_00_50: SiliconeMaterial = SiliconeMaterial::from_anchor(
    "ECOFLEX_00_50",
    28_000.0,
    112_000.0,
    2_410.0,
    1070.0,
    8.64,
    YEOH_MIN_PRINCIPAL_STRETCH,
    ShoreReading::DoubleZero(50.0),
);

/// Dragon Skin 10 — Shore 10A; softest in the Dragon Skin line.
///
/// Cure-speed variants (Fast / Medium / NV / Slow) share these
/// mechanical properties on Smooth-On's data sheets and differ only
/// in pot life and cure time. `λ_break` = 11.0 (1000 % elongation).
pub const DRAGON_SKIN_10A: SiliconeMaterial = SiliconeMaterial::from_anchor(
    "DRAGON_SKIN_10A",
    51_000.0,
    204_000.0,
    4_460.0,
    1070.0,
    8.80,
    YEOH_MIN_PRINCIPAL_STRETCH,
    ShoreReading::A(10.0),
);

/// Dragon Skin 15 — Shore 15A. `λ_break` = 8.71 (771 % elongation per
/// TDS — `λ = 1 + ε`).
///
/// Added for Yeoh arc 2026-05-08 — sits between [`DRAGON_SKIN_10A`] and
/// [`DRAGON_SKIN_20A`] in the hardness sequence.
pub const DRAGON_SKIN_15: SiliconeMaterial = SiliconeMaterial::from_anchor(
    "DRAGON_SKIN_15",
    92_000.0,
    368_000.0,
    8_200.0,
    1070.0,
    6.97,
    YEOH_MIN_PRINCIPAL_STRETCH,
    ShoreReading::A(15.0),
);

/// Dragon Skin 20 — Shore 20A. `λ_break` = 7.20 (620 % elongation).
pub const DRAGON_SKIN_20A: SiliconeMaterial = SiliconeMaterial::from_anchor(
    "DRAGON_SKIN_20A",
    113_000.0,
    452_000.0,
    10_000.0,
    1080.0,
    5.76,
    YEOH_MIN_PRINCIPAL_STRETCH,
    ShoreReading::A(20.0),
);

/// Dragon Skin 30 — Shore 30A; firmest in the Dragon Skin line.
/// `λ_break` = 4.64 (364 % elongation — significantly less ductile than
/// the softer Dragon Skin grades).
pub const DRAGON_SKIN_30A: SiliconeMaterial = SiliconeMaterial::from_anchor(
    "DRAGON_SKIN_30A",
    198_000.0,
    792_000.0,
    17_600.0,
    1080.0,
    3.71,
    YEOH_MIN_PRINCIPAL_STRETCH,
    ShoreReading::A(30.0),
);

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Every entry in the table, in source-PSI order. Used by every
    /// per-row contract test below so a new entry tripped by adding it
    /// to the table also runs through the contract.
    const ALL: &[(&str, SiliconeMaterial)] = &[
        ("Ecoflex 00-10", ECOFLEX_00_10),
        ("Ecoflex 00-20", ECOFLEX_00_20),
        ("Ecoflex 00-30", ECOFLEX_00_30),
        ("Ecoflex 00-50", ECOFLEX_00_50),
        ("Dragon Skin 10A", DRAGON_SKIN_10A),
        ("Dragon Skin 15", DRAGON_SKIN_15),
        ("Dragon Skin 20A", DRAGON_SKIN_20A),
        ("Dragon Skin 30A", DRAGON_SKIN_30A),
    ];

    /// At ν = 0.40 the conversion `λ = 2μν / (1 − 2ν) = 4μ` is exact.
    /// Every entry must satisfy this identity bit-equally — pinning ν
    /// = 0.40 across the whole table.
    #[test]
    fn lambda_is_four_times_mu_at_nu_0_40() {
        for (name, mat) in ALL {
            assert_relative_eq!(mat.lambda, 4.0 * mat.mu, epsilon = 0.0,);
            assert!(
                mat.lambda > 0.0,
                "{name}: lambda must be strictly positive, got {}",
                mat.lambda
            );
        }
    }

    /// Every Lamé and density value must be strictly positive and
    /// finite. Catches accidental zero / `NaN` / negative typos in the
    /// table at compile-test time.
    #[test]
    fn all_values_strictly_positive_and_finite() {
        for (name, mat) in ALL {
            assert!(
                mat.mu > 0.0 && mat.mu.is_finite(),
                "{name}: mu must be strictly positive and finite, got {}",
                mat.mu
            );
            assert!(
                mat.lambda > 0.0 && mat.lambda.is_finite(),
                "{name}: lambda must be strictly positive and finite, got {}",
                mat.lambda
            );
            assert!(
                mat.c2 > 0.0 && mat.c2.is_finite(),
                "{name}: c2 must be strictly positive and finite, got {}",
                mat.c2
            );
            assert!(
                mat.density > 0.0 && mat.density.is_finite(),
                "{name}: density must be strictly positive and finite, got {}",
                mat.density
            );
        }
    }

    /// μ is non-decreasing along the table in source-PSI order, with
    /// the Ecoflex 00-10 / 00-20 tie acceptable (both sit at 8 PSI on
    /// the data sheet — Shore 00 hardness differentiates them by
    /// indentation more than by elongation modulus). Every other
    /// adjacent pair is strictly increasing, including the
    /// 10A → 15 → 20A → 30A Dragon Skin sequence.
    #[test]
    fn mu_is_non_decreasing_along_hardness_order() {
        for window in ALL.windows(2) {
            let (lo_name, lo) = &window[0];
            let (hi_name, hi) = &window[1];
            assert!(
                hi.mu >= lo.mu,
                "{hi_name}'s mu ({}) must be ≥ {lo_name}'s mu ({})",
                hi.mu,
                lo.mu,
            );
        }
    }

    /// Density values fall inside the expected range for platinum-cure
    /// silicones. Smooth-On's specific-gravity rows for these grades
    /// land in [1.04, 1.08], so density (kg/m³) lies in [1040, 1080].
    #[test]
    fn densities_within_silicone_range() {
        for (name, mat) in ALL {
            assert!(
                (1040.0..=1080.0).contains(&mat.density),
                "{name}: density {} kg/m³ outside expected silicone range [1040, 1080]",
                mat.density,
            );
        }
    }

    /// Validity bounds: `0 < min < 1 < max`. `min` is family-uniform
    /// at 0.30. Catches ordering typos. The `max = 0.8 · λ_break`
    /// invariant is locked separately by
    /// `validity_max_pins_to_80_pct_of_one_plus_elongation_at_break`
    /// so a typo in either `λ_break` or the multiplier surfaces with a
    /// targeted failure rather than this gross-shape check.
    #[test]
    fn validity_bounds_well_ordered() {
        for (name, mat) in ALL {
            assert!(
                mat.validity_min_principal_stretch > 0.0
                    && mat.validity_min_principal_stretch < 1.0,
                "{name}: min stretch {} must be in (0, 1)",
                mat.validity_min_principal_stretch,
            );
            assert!(
                mat.validity_max_principal_stretch > 1.0,
                "{name}: max stretch {} must be > 1",
                mat.validity_max_principal_stretch,
            );
            assert_relative_eq!(
                mat.validity_min_principal_stretch,
                YEOH_MIN_PRINCIPAL_STRETCH,
                epsilon = 0.0
            );
        }
    }

    /// Each anchor's `validity_max_principal_stretch` equals
    /// `0.8 · (1 + ε_break(%) / 100)` where `ε_break` is the
    /// elongation-at-break published on the Smooth-On TDS. Pins both
    /// the `λ = 1 + ε` ASTM-D412 conversion and the 80 %-rupture-margin
    /// rule from arc memo D8. A typo in either `λ_break` or the 0.8
    /// multiplier surfaces here.
    #[test]
    fn validity_max_pins_to_80_pct_of_one_plus_elongation_at_break() {
        // (anchor, ε_break in %, from Smooth-On TDS via arc memo
        // §"Recon findings" lines 26-31, 35-40).
        const PUBLISHED_ELONGATION_PCT: &[(SiliconeMaterial, f64)] = &[
            (ECOFLEX_00_10, 800.0),
            (ECOFLEX_00_20, 845.0),
            (ECOFLEX_00_30, 900.0),
            (ECOFLEX_00_50, 980.0),
            (DRAGON_SKIN_10A, 1000.0),
            (DRAGON_SKIN_15, 771.0),
            (DRAGON_SKIN_20A, 620.0),
            (DRAGON_SKIN_30A, 364.0),
        ];
        for (mat, eb_pct) in PUBLISHED_ELONGATION_PCT {
            let lambda_break = 1.0 + eb_pct / 100.0;
            let expected_max = 0.8 * lambda_break;
            assert_relative_eq!(
                mat.validity_max_principal_stretch,
                expected_max,
                max_relative = 1e-3
            );
        }
    }

    /// Every const anchor's `source` is `Anchor { name }` with `name`
    /// matching the symbol identifier. Verifies the F2.1 plumbing
    /// preserves provenance from `from_anchor` → const entry.
    #[test]
    fn provenance_is_anchor_for_const_entries() {
        const EXPECTED: &[(&str, SiliconeMaterial)] = &[
            ("ECOFLEX_00_10", ECOFLEX_00_10),
            ("ECOFLEX_00_20", ECOFLEX_00_20),
            ("ECOFLEX_00_30", ECOFLEX_00_30),
            ("ECOFLEX_00_50", ECOFLEX_00_50),
            ("DRAGON_SKIN_10A", DRAGON_SKIN_10A),
            ("DRAGON_SKIN_15", DRAGON_SKIN_15),
            ("DRAGON_SKIN_20A", DRAGON_SKIN_20A),
            ("DRAGON_SKIN_30A", DRAGON_SKIN_30A),
        ];
        for (expected_name, mat) in EXPECTED {
            assert!(
                matches!(mat.source, ConstructionSource::Anchor { name } if name == *expected_name),
                "{expected_name}: expected Anchor {{ name = {expected_name} }}, got {:?}",
                mat.source,
            );
        }
    }

    /// `to_neo_hookean()` builds a `NeoHookean` whose Lamé pair matches
    /// the source [`SiliconeMaterial`] bit-equally at the small
    /// uniaxial probe `F = diag(1.01, 1, 1)`. Pinned because the
    /// const-fn conversion is the load-bearing bridge from this table
    /// into [`crate::Material`] consumers.
    #[test]
    fn to_neo_hookean_round_trips_lame_pair() {
        use crate::material::Material;
        use nalgebra::Matrix3;
        for (_, mat) in ALL {
            let nh = mat.to_neo_hookean();
            let id = Matrix3::<f64>::identity();
            assert_relative_eq!(nh.energy(&id), 0.0, epsilon = 0.0,);

            let mut f = Matrix3::<f64>::identity();
            f[(0, 0)] = 1.01;
            let i1 = 1.01_f64.mul_add(1.01, 2.0);
            let j_ln = 1.01_f64.ln();
            let half_mu = 0.5 * mat.mu;
            let half_lambda = 0.5 * mat.lambda;
            let expected =
                half_lambda.mul_add(j_ln * j_ln, half_mu.mul_add(i1 - 3.0, -mat.mu * j_ln));
            assert_relative_eq!(nh.energy(&f), expected, epsilon = 0.0,);
        }
    }

    /// `to_yeoh()` builds a `Yeoh` whose `(μ, λ, C₂)` and validity
    /// bounds round-trip the source [`SiliconeMaterial`] exactly. At
    /// the small uniaxial probe `F = diag(1.01, 1, 1)` the Yeoh energy
    /// equals NH-energy + `C₂(I₁−3)²` (the additive-decomposition
    /// pattern from the Yeoh struct's module doc).
    #[test]
    fn to_yeoh_round_trips_yeoh_fields_for_each_anchor() {
        use crate::material::Material;
        use nalgebra::Matrix3;
        for (_name, mat) in ALL {
            let yeoh = mat.to_yeoh();
            assert_relative_eq!(yeoh.mu(), mat.mu, epsilon = 0.0);
            assert_relative_eq!(yeoh.lambda(), mat.lambda, epsilon = 0.0);
            assert_relative_eq!(yeoh.c2(), mat.c2, epsilon = 0.0);

            let validity = yeoh.validity();
            assert_eq!(
                validity.max_principal_stretch,
                Some(mat.validity_max_principal_stretch)
            );
            assert_eq!(
                validity.min_principal_stretch,
                Some(mat.validity_min_principal_stretch)
            );

            let mut f = Matrix3::<f64>::identity();
            f[(0, 0)] = 1.01;
            let i1 = 1.01_f64.mul_add(1.01, 2.0);
            let i1m3 = i1 - 3.0;
            let j_ln = 1.01_f64.ln();
            let half_mu = 0.5 * mat.mu;
            let half_lambda = 0.5 * mat.lambda;
            let nh_part =
                half_lambda.mul_add(j_ln * j_ln, half_mu.mul_add(i1 - 3.0, -mat.mu * j_ln));
            let yeoh_extra = mat.c2 * i1m3 * i1m3;
            assert_relative_eq!(yeoh.energy(&f), nh_part + yeoh_extra, epsilon = 0.0);
        }
    }

    /// Every anchor's `(μ, C₂)` pair reproduces the published 100 %
    /// modulus via the Yeoh calibration formula
    /// `M_100 = 3.5·C₁ + 14·C₂` (incompressible-uniaxial Yeoh at
    /// λ = 2; arc memo line 72). Catches transcription typos in the
    /// anchor table.
    ///
    /// Tolerance covers `c2` rounding to nearest 10 Pa; worst case
    /// observed: `DS_20A` at `rel_err` ≈ 2.76e-4 (calibrated `c2 =
    /// 10006.65 Pa`, table value 10000).
    #[test]
    fn c2_calibration_reproduces_published_100_pct_modulus() {
        // psi → Pa (NIST: 1 psi = 6894.757293168361 Pa).
        const PSI_TO_PA: f64 = 6_894.757_293_168_361;
        const REL_TOL: f64 = 1e-3;

        // (anchor, M_100 in psi from Smooth-On TDS).
        const PUBLISHED: &[(SiliconeMaterial, f64)] = &[
            (ECOFLEX_00_10, 8.0),
            (ECOFLEX_00_20, 8.0),
            (ECOFLEX_00_30, 10.0),
            (ECOFLEX_00_50, 12.0),
            (DRAGON_SKIN_10A, 22.0),
            (DRAGON_SKIN_15, 40.0),
            (DRAGON_SKIN_20A, 49.0),
            (DRAGON_SKIN_30A, 86.0),
        ];

        for (mat, m100_psi) in PUBLISHED {
            let c1 = 0.5 * mat.mu;
            let predicted = 3.5_f64.mul_add(c1, 14.0 * mat.c2);
            let published = m100_psi * PSI_TO_PA;
            let rel_err = (predicted - published).abs() / published;
            assert!(
                rel_err < REL_TOL,
                "predicted M_100={predicted} Pa, published={published} Pa, rel_err={rel_err}",
            );
        }
    }

    // ---- F2.2 — from_effective_shore (Path 2) ------------------------

    /// At `DoubleZero(20)` the bracket is `(00-10, 00-20)` with
    /// `weight = 1`, so every interpolated field equals 00-20's
    /// (within FMA-rounding f64 epsilon). Source carries the
    /// bracketing anchor names + `weight = 1`.
    #[test]
    fn from_effective_shore_at_anchor_position_returns_anchor_data() {
        let result = SiliconeMaterial::from_effective_shore(
            ShoreReading::DoubleZero(20.0),
            Some("test: Ecoflex 00-20 anchor position"),
        )
        .expect("Shore 00-20 sits at a published anchor");
        assert_relative_eq!(result.mu, ECOFLEX_00_20.mu, epsilon = 0.0);
        assert_relative_eq!(result.lambda, ECOFLEX_00_20.lambda, epsilon = 0.0);
        assert_relative_eq!(result.c2, ECOFLEX_00_20.c2, epsilon = 0.0);
        assert_relative_eq!(result.density, ECOFLEX_00_20.density, epsilon = 0.0);
        assert_relative_eq!(
            result.validity_max_principal_stretch,
            ECOFLEX_00_20.validity_max_principal_stretch,
            epsilon = 0.0
        );
        assert!(matches!(
            result.source,
            ConstructionSource::Interpolated {
                low_anchor: "ECOFLEX_00_10",
                high_anchor: "ECOFLEX_00_20",
                weight,
                user_description: Some("test: Ecoflex 00-20 anchor position"),
            } if (weight - 1.0).abs() < f64::EPSILON
        ));
    }

    /// At `DoubleZero(25)` (midpoint between 00-20 and 00-30) every
    /// scalar field is the linear average of the bracketing anchors.
    #[test]
    fn from_effective_shore_at_midpoint_interpolates_linearly() {
        let result = SiliconeMaterial::from_effective_shore(ShoreReading::DoubleZero(25.0), None)
            .expect("Shore 00-25 lies between 00-20 and 00-30");
        assert_relative_eq!(
            result.mu,
            0.5 * (ECOFLEX_00_20.mu + ECOFLEX_00_30.mu),
            epsilon = 0.0
        );
        assert_relative_eq!(
            result.c2,
            0.5 * (ECOFLEX_00_20.c2 + ECOFLEX_00_30.c2),
            epsilon = 0.0
        );
        assert_relative_eq!(
            result.density,
            0.5 * (ECOFLEX_00_20.density + ECOFLEX_00_30.density),
            epsilon = 0.0
        );
        assert!(matches!(
            result.source,
            ConstructionSource::Interpolated {
                low_anchor: "ECOFLEX_00_20",
                high_anchor: "ECOFLEX_00_30",
                weight,
                user_description: None,
            } if (weight - 0.5).abs() < f64::EPSILON
        ));
    }

    /// Below the lowest published anchor returns `OutOfRange`.
    #[test]
    #[allow(clippy::float_cmp)]
    fn from_effective_shore_below_lowest_returns_out_of_range() {
        let err = SiliconeMaterial::from_effective_shore(ShoreReading::DoubleZero(5.0), None)
            .expect_err("Shore 00-5 is below the lowest Ecoflex anchor (00-10)");
        // Integer-valued f64s; exact == is correct.
        assert!(matches!(
            err,
            ShoreInterpolationError::OutOfRange {
                shore: ShoreReading::DoubleZero(v),
                low_bound,
                high_bound,
            } if v == 5.0 && low_bound == 10.0 && high_bound == 50.0
        ));
    }

    /// Above the highest published anchor returns `OutOfRange`.
    #[test]
    #[allow(clippy::float_cmp)]
    fn from_effective_shore_above_highest_returns_out_of_range() {
        let err = SiliconeMaterial::from_effective_shore(ShoreReading::A(40.0), None)
            .expect_err("Shore A 40 is above the highest Dragon Skin anchor (30A)");
        assert!(matches!(
            err,
            ShoreInterpolationError::OutOfRange {
                shore: ShoreReading::A(v),
                low_bound,
                high_bound,
            } if v == 40.0 && low_bound == 10.0 && high_bound == 30.0
        ));
    }

    /// Shore D has no published anchors today; supplying it returns
    /// `NoAnchorsInFamily`.
    #[test]
    #[allow(clippy::float_cmp)]
    fn from_effective_shore_d_scale_returns_no_anchors_error() {
        let err = SiliconeMaterial::from_effective_shore(ShoreReading::D(50.0), None)
            .expect_err("Shore D has no anchors yet");
        assert!(matches!(
            err,
            ShoreInterpolationError::NoAnchorsInFamily {
                shore: ShoreReading::D(v),
            } if v == 50.0
        ));
    }

    // ---- F2.2 — from_measured (Path 3) -------------------------------

    /// `from_measured` applies the F4 `μ = σ_100 / 3` correlation +
    /// Yeoh calibration formula in isolation. Probes the formula
    /// directly (not against the rounded anchor table — the table
    /// rounds μ to nearest kPa and C₂ to nearest 10 Pa, propagating
    /// 1 % errors that this test would surface as flakiness if it
    /// compared anchor values).
    #[test]
    fn from_measured_applies_calibration_formula_directly() {
        for m100_pa in [10_000.0_f64, 68_947.57, 151_684.66, 592_949.13] {
            let measured = SiliconeMaterial::from_measured(
                ShoreReading::A(20.0),
                m100_pa,
                "test: formula probe",
            )
            .expect("positive σ_100 yields positive C₂");
            assert_relative_eq!(measured.mu, m100_pa / 3.0, max_relative = 1e-12);
            assert_relative_eq!(measured.lambda, 4.0 * m100_pa / 3.0, max_relative = 1e-12);
            // c2 collapses to σ_100 / 33.6 after μ = σ_100 / 3
            // substitution; see from_measured doc.
            assert_relative_eq!(measured.c2, m100_pa / 33.6, max_relative = 1e-12);
            assert!(matches!(
                measured.source,
                ConstructionSource::Measured {
                    user_description: "test: formula probe",
                }
            ));
        }
    }

    /// Non-positive modulus (zero, negative, `NaN`, `±inf`) is rejected.
    #[test]
    fn from_measured_with_nonpositive_modulus_returns_error() {
        for bad in [0.0, -100.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let err = SiliconeMaterial::from_measured(ShoreReading::A(20.0), bad, "test: bad")
                .expect_err("non-positive modulus must error");
            assert!(matches!(
                err,
                MeasuredMaterialError::NonpositiveModulus { .. }
            ));
        }
    }

    /// `from_measured` C₂ is positive for any positive `σ_100`
    /// (collapses to `σ_100 / 33.6` after substituting `μ = σ_100/3`).
    /// Pinned: a future calibration formula change that breaks this
    /// invariant must also reintroduce the negative-C₂ guard.
    #[test]
    fn from_measured_c2_positive_for_any_positive_modulus() {
        for m100_pa in [1.0, 100.0, 10_000.0, 1_000_000.0] {
            let mat = SiliconeMaterial::from_measured(
                ShoreReading::A(20.0),
                m100_pa,
                "test: positive C₂ regime",
            )
            .expect("positive σ_100 always yields positive C₂");
            assert!(
                mat.c2 > 0.0,
                "σ_100={m100_pa} produced non-positive C₂={}",
                mat.c2
            );
        }
    }
}
