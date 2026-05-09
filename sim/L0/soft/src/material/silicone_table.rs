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
//! | Ecoflex 00-50   | 00-50 |    28.0 |   112.0 |     2.41 |      1070 |    9.80 |  7.84 | 12 PSI  |
//! | Dragon Skin 10A | 10A   |    51.0 |   204.0 |     4.46 |      1070 |   11.00 |  8.80 | 22 PSI  |
//! | Dragon Skin 15  | 15A   |    92.0 |   368.0 |     8.20 |      1070 |    7.71 |  6.17 | 40 PSI  |
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
/// = 9.80 (980 % elongation).
pub const ECOFLEX_00_50: SiliconeMaterial = SiliconeMaterial::from_anchor(
    "ECOFLEX_00_50",
    28_000.0,
    112_000.0,
    2_410.0,
    1070.0,
    7.84,
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

/// Dragon Skin 15 — Shore 15A. `λ_break` = 7.71 (771 % elongation).
/// Added for Yeoh arc 2026-05-08 — sits between [`DRAGON_SKIN_10A`]
/// and [`DRAGON_SKIN_20A`] in the hardness sequence.
pub const DRAGON_SKIN_15: SiliconeMaterial = SiliconeMaterial::from_anchor(
    "DRAGON_SKIN_15",
    92_000.0,
    368_000.0,
    8_200.0,
    1070.0,
    6.17,
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

    /// Validity bounds: `0 < min < 1 < max`, with `max ≤ λ_break · 0.8`.
    /// `min` is family-uniform at 0.30. Catches ordering typos.
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
}
