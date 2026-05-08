//! Silicone material reference — `pub const` Lamé pairs + density for
//! every Smooth-On platinum-cure silicone the layered-silicone-device
//! arc currently considers as a body-layer or rigid-mold material.
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
//! | Material        | μ (kPa) | λ (kPa) | ρ (kg/m³) | E (kPa) | `σ_100` source |
//! |-----------------|--------:|--------:|----------:|--------:|----------------|
//! | Ecoflex 00-10   |    18.0 |    72.0 |      1040 |    50.4 |  8 PSI         |
//! | Ecoflex 00-20   |    18.0 |    72.0 |      1070 |    50.4 |  8 PSI         |
//! | Ecoflex 00-30   |    23.0 |    92.0 |      1070 |    64.4 | 10 PSI         |
//! | Ecoflex 00-50   |    28.0 |   112.0 |      1070 |    78.4 | 12 PSI         |
//! | Dragon Skin 10A |    51.0 |   204.0 |      1070 |   142.8 | 22 PSI         |
//! | Dragon Skin 20A |   113.0 |   452.0 |      1080 |   316.4 | 49 PSI         |
//! | Dragon Skin 30A |   198.0 |   792.0 |      1080 |   554.4 | 86 PSI         |
//!
//! Ecoflex 00-10 and 00-20 share a 100% modulus on Smooth-On's data
//! sheet (~8 PSI) — Shore 00 hardness differentiates them by
//! indentation resistance more than by elongation modulus, and the
//! shared `σ_100` is faithful to the published sheets rather than
//! interpolated to force a strict ordering. Density differs by 30
//! kg/m³ (1040 vs 1070) per the `00-10` and `00-20` data sheets'
//! specific-gravity rows.
//!
//! # Conversion
//!
//! Smooth-On data sheets report **100% modulus** `σ_100` — engineering
//! stress at 100% uniaxial strain — alongside tensile strength,
//! elongation at break, and specific gravity. Conversion to compressible
//! Neo-Hookean Lamé parameters at ν = 0.40 (Fork B compressible-NH
//! framing) follows the small-strain incompressible-NH approximation
//! `σ_eng ≈ 3μ` at ε = 1.0:
//!
//! ```text
//! μ = σ_100 / 3                              (Pa)
//! λ = 2μν / (1 − 2ν) = 4μ      at ν = 0.40   (Pa)
//! E = 2μ(1 + ν)     = 2.8μ     at ν = 0.40   (Pa)
//! ρ = (specific gravity) · 1000              (kg/m³)
//! ```
//!
//! `σ_100 ≈ 3μ` is a small-strain identity for the incompressible NH
//! material (`σ_eng = μ(λ_axial − 1/λ_axial²)` with the small-strain
//! linearization `λ_axial − 1 = ε`); at ε = 1.0 the linearized form
//! gives `σ_100 = 3μ` exactly. The finite-strain form
//! `σ_eng = μ(λ − 1/λ²)` evaluated at λ = 2 (i.e. ε = 1.0) gives
//! `σ_100 = 1.75μ` — but the linearized small-strain identity matches
//! how data-sheet "100% modulus" is conventionally interpreted in the
//! soft-robotics literature (Marechal et al. 2021, Polygerinos et al.
//! 2017). The factor of ~1.7 discrepancy between 1.75μ and 3μ is the
//! kind of catalog-value uncertainty Fork B's calibration loop is
//! designed to absorb at post-cast time.
//!
//! # Validity
//!
//! ν = 0.40 sits comfortably inside [`NeoHookean::from_young_poisson`]'s
//! ν < 0.45 standalone-compressible-law cap (`super::neo_hookean.rs`).
//! Real silicones are near-incompressible (ν ≈ 0.49); the 0.40 framing
//! deliberately introduces volumetric locking error that calibration
//! absorbs into the effective μ at post-cast time. Tet10 + F-bar at
//! Phase H recovers the near-incompressible regime without the ν
//! shift.

use super::NeoHookean;

/// Reference data for one platinum-cure silicone — Lamé pair and bulk
/// density at ν = 0.40 compressible-NH framing.
///
/// Values are starting points; production calibration per Fork B
/// overwrites μ and λ from the post-cast modulus fit.
#[derive(Clone, Copy, Debug)]
pub struct SiliconeMaterial {
    /// First Lamé parameter (shear modulus) in pascals.
    pub mu: f64,
    /// Second Lamé parameter in pascals. At ν = 0.40 every entry
    /// satisfies `lambda == 4 * mu` (verified by the
    /// `lambda_is_four_times_mu_at_nu_0_40` unit test below).
    pub lambda: f64,
    /// Bulk density in kilograms per cubic meter.
    pub density: f64,
}

impl SiliconeMaterial {
    /// Construct from `(μ, λ, ρ)`. `const` so the table entries
    /// below can use it directly.
    #[must_use]
    pub const fn new(mu: f64, lambda: f64, density: f64) -> Self {
        Self {
            mu,
            lambda,
            density,
        }
    }

    /// Build a [`NeoHookean`] from the material's Lamé pair. `const`
    /// so consumer scenes can construct per-tet materials at compile
    /// time without losing the silicone-table provenance.
    #[must_use]
    pub const fn to_neo_hookean(&self) -> NeoHookean {
        NeoHookean::from_lame(self.mu, self.lambda)
    }
}

/// Ecoflex 00-10 — softest in the Ecoflex line; Shore 00-10.
pub const ECOFLEX_00_10: SiliconeMaterial = SiliconeMaterial::new(18_000.0, 72_000.0, 1040.0);

/// Ecoflex 00-20 — Shore 00-20. Shares 100% modulus with 00-10 per
/// Smooth-On data sheet; density differs.
pub const ECOFLEX_00_20: SiliconeMaterial = SiliconeMaterial::new(18_000.0, 72_000.0, 1070.0);

/// Ecoflex 00-30 — Shore 00-30. Most-cited Ecoflex grade in the
/// soft-robotics literature.
pub const ECOFLEX_00_30: SiliconeMaterial = SiliconeMaterial::new(23_000.0, 92_000.0, 1070.0);

/// Ecoflex 00-50 — Shore 00-50; firmest in the Ecoflex line.
pub const ECOFLEX_00_50: SiliconeMaterial = SiliconeMaterial::new(28_000.0, 112_000.0, 1070.0);

/// Dragon Skin 10 — Shore 10A; softest in the Dragon Skin line.
///
/// Cure-speed variants (Fast / Medium / NV / Slow) share these
/// mechanical properties on Smooth-On's data sheets and differ only
/// in pot life and cure time.
pub const DRAGON_SKIN_10A: SiliconeMaterial = SiliconeMaterial::new(51_000.0, 204_000.0, 1070.0);

/// Dragon Skin 20 — Shore 20A.
pub const DRAGON_SKIN_20A: SiliconeMaterial = SiliconeMaterial::new(113_000.0, 452_000.0, 1080.0);

/// Dragon Skin 30 — Shore 30A; firmest in the Dragon Skin line.
pub const DRAGON_SKIN_30A: SiliconeMaterial = SiliconeMaterial::new(198_000.0, 792_000.0, 1080.0);

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
    /// adjacent pair is strictly increasing.
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

    /// `to_neo_hookean()` builds a `NeoHookean` whose Lamé pair matches
    /// the source [`SiliconeMaterial`] bit-equally. Pinned because the
    /// const-fn conversion is the load-bearing bridge from this table
    /// into [`crate::Material`] consumers (per-tet `MaterialField`,
    /// `Element::stiffness`, etc.).
    #[test]
    fn to_neo_hookean_round_trips_lame_pair() {
        // The NeoHookean fields are private; `to_neo_hookean` returns
        // an opaque NeoHookean. Probe it indirectly via Material::energy
        // at F = I (where ψ = 0 for both compressible NH parameters),
        // and at a small uniaxial stretch where the energy depends on
        // the specific (μ, λ).
        use crate::material::Material;
        use nalgebra::Matrix3;
        for (_, mat) in ALL {
            let nh = mat.to_neo_hookean();
            let id = Matrix3::<f64>::identity();
            assert_relative_eq!(nh.energy(&id), 0.0, epsilon = 0.0,);

            // Small uniaxial stretch F = diag(1.01, 1, 1). For NH
            //   ψ = (μ/2)(I₁ − 3) − μ ln J + (λ/2)(ln J)²
            // at I₁ = 1.01² + 1² + 1² = 1.0201 + 2.0 = 3.0201,
            // J = 1.01, ln J = ln(1.01). The mul_add chain rewrites
            // ψ as (λ/2)(ln J)² + ((μ/2)(I₁ − 3) − μ ln J) so each
            // FMA matches NeoHookean::energy's internal FMA exactly
            // (single rounding per term, bit-equal at this F).
            let mut f = Matrix3::<f64>::identity();
            f[(0, 0)] = 1.01;
            let i1 = 1.01_f64.mul_add(1.01, 2.0);
            let j_ln = 1.01_f64.ln();
            let half_mu = 0.5 * mat.mu;
            let half_lambda = 0.5 * mat.lambda;
            let expected =
                half_lambda.mul_add(j_ln * j_ln, half_mu.mul_add(i1 - 3.0, -mat.mu * j_ln));
            // Bit-equal: the mul_add chain matches NeoHookean::energy
            // exactly; no rounding slack is needed at this F.
            assert_relative_eq!(nh.energy(&f), expected, epsilon = 0.0,);
        }
    }
}
