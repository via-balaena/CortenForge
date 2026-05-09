//! Compressible 2-parameter Yeoh material — closed-form `P` and tangent.
//!
//! Closed forms from the Yeoh arc memo (`§Math derivations`):
//!
//! - `ψ(F) = C₁(I₁ − 3) + C₂(I₁ − 3)² − μ ln J + (λ/2)(ln J)²`
//! - `P(F) = μ(F − F⁻ᵀ) + λ (ln J) F⁻ᵀ + 4 C₂ (I₁ − 3) F`
//! - `C_ijkl = C_NH_ijkl + 4 C₂ (I₁ − 3) δ_ik δ_jl + 8 C₂ F_ij F_kl`
//!
//! with `C₁ = μ/2` (small-strain consistency).
//!
//! ## NH-bit-exact contract (spec D2)
//!
//! At `C₂ = 0`, every output equals [`NeoHookean`]'s output bit-exactly.
//! This is **structurally fragile**: Spike 1 surfaced two requirements
//! that a naive consolidated-coefficient form (`2F·[C₁ + 2 C₂(I₁−3)]`)
//! violates by 1–12 ULP.
//!
//! 1. **Additive decomposition.** Compute the NH-form expression first,
//!    then add the Yeoh-extra term. At `C₂ = 0` the extra collapses to
//!    `0.0` (or the zero matrix); `x + 0.0 = x` is bit-exact in f64.
//! 2. **Match NH's FMA pattern.** The tangent's `cross_coeff` uses
//!    `lambda.mul_add(-ln_j, mu)`, not `mu - lambda*ln_j`. The latter
//!    rounds the product before subtracting and drifts 1 ULP.
//!
//! Validation: `tests/yeoh_contract.rs`.
//!
//! Inversion handling matches `NeoHookean::RequireOrientation`: panic on
//! non-positive `det F`, expected to be prevented upstream by the IPC
//! barrier at contact time.
//!
//! ## Validity domain
//!
//! Yeoh uses the asymmetric `max_principal_stretch` /
//! `min_principal_stretch` bounds (memo D8). Default constructors leave
//! these `None`, falling through to the legacy NH symmetric bound at
//! `max_stretch_deviation = 1.0`. Per-anchor calibrated bounds are set
//! by the [`crate::SiliconeMaterial::to_yeoh`] path (Yeoh arc F2 work).

use nalgebra::{Matrix3, SMatrix};

use super::{InversionHandling, Material, NeoHookean, ValidityDomain};

/// Compressible 2-parameter Yeoh energy and its derivatives. `c2 = 0`
/// reduces bit-exactly to [`NeoHookean`] at the same `(μ, λ)`.
#[derive(Clone, Debug)]
pub struct Yeoh {
    mu: f64,
    lambda: f64,
    c2: f64,
    max_principal_stretch: Option<f64>,
    min_principal_stretch: Option<f64>,
}

impl Yeoh {
    /// Construct from Lamé parameters `(μ, λ)` and Yeoh `C₂` directly.
    /// Validity bounds default to `None` (legacy NH symmetric gate).
    #[must_use]
    pub const fn from_lame_and_c2(mu: f64, lambda: f64, c2: f64) -> Self {
        Self {
            mu,
            lambda,
            c2,
            max_principal_stretch: None,
            min_principal_stretch: None,
        }
    }

    /// Set per-material asymmetric principal-stretch bounds (memo D8).
    /// `max` caps tensile stretch (e.g. `0.8 · λ_break`); `min` caps
    /// compression (e.g. `0.30` engineering-aggressive default).
    #[must_use]
    pub const fn with_principal_stretch_bounds(mut self, max: f64, min: f64) -> Self {
        self.max_principal_stretch = Some(max);
        self.min_principal_stretch = Some(min);
        self
    }

    /// Lift a [`NeoHookean`] to a Yeoh with `C₂ = 0`. Validity bounds
    /// inherit `None` so the solver falls back to the same legacy gate
    /// the source NH used; `energy`/`first_piola`/`tangent` are bit-exactly
    /// equal to the source NH at every `F` (D2 contract).
    #[must_use]
    pub const fn from_neo_hookean(nh: &NeoHookean) -> Self {
        Self::from_lame_and_c2(nh.mu(), nh.lambda(), 0.0)
    }

    /// Construct from Young's modulus, Poisson ratio, and `C₂` via the
    /// isotropic-elasticity conversion `μ = E / 2(1+ν)`,
    /// `λ = E ν / [(1+ν)(1−2ν)]`.
    ///
    /// # Panics
    /// On `ν ≥ 0.45` (matches [`NeoHookean::from_young_poisson`]'s
    /// validity bound; the Ch 05 mixed-u-p / F-bar decorator widens to
    /// 0.499).
    #[must_use]
    pub fn from_young_poisson_and_c2(young: f64, nu: f64, c2: f64) -> Self {
        assert!(
            nu < 0.45,
            "standalone Yeoh requires nu < 0.45; use the Ch 05 locking-fix decorator for higher Poisson ratios"
        );
        let mu = young / (2.0 * (1.0 + nu));
        let lambda = young * nu / ((1.0 + nu) * 2.0_f64.mul_add(-nu, 1.0));
        Self::from_lame_and_c2(mu, lambda, c2)
    }

    /// First Lamé parameter `μ` in pascals.
    #[must_use]
    pub const fn mu(&self) -> f64 {
        self.mu
    }

    /// Second Lamé parameter `λ` in pascals.
    #[must_use]
    pub const fn lambda(&self) -> f64 {
        self.lambda
    }

    /// Yeoh polynomial coefficient `C₂` in pascals.
    #[must_use]
    pub const fn c2(&self) -> f64 {
        self.c2
    }
}

impl Material for Yeoh {
    fn energy(&self, f: &Matrix3<f64>) -> f64 {
        let i1 = f.norm_squared();
        let ln_j = f.determinant().ln();
        let i1m3 = i1 - 3.0;
        // NH part — bit-exact with `neo_hookean::energy`.
        let half_mu = 0.5 * self.mu;
        let half_lambda = 0.5 * self.lambda;
        let nh_part = half_lambda.mul_add(ln_j * ln_j, half_mu.mul_add(i1 - 3.0, -self.mu * ln_j));
        // Yeoh extra: C₂(I₁−3)². Vanishes at C₂ = 0 (additive
        // decomposition preserves NH bit-exactness, see module doc).
        let yeoh_extra = self.c2 * i1m3 * i1m3;
        nh_part + yeoh_extra
    }

    fn first_piola(&self, f: &Matrix3<f64>) -> Matrix3<f64> {
        let f_inv_t = invert_transpose(f);
        let ln_j = f.determinant().ln();
        let i1m3 = f.norm_squared() - 3.0;
        // NH part — bit-exact with `neo_hookean::first_piola`.
        let nh_part = self.mu * (f - f_inv_t) + self.lambda * ln_j * f_inv_t;
        // Yeoh extra: 4·C₂·(I₁−3)·F (∂/∂F of C₂(I₁−3)²). Vanishes at
        // C₂ = 0.
        let yeoh_extra = (4.0 * self.c2 * i1m3) * f;
        nh_part + yeoh_extra
    }

    fn tangent(&self, f: &Matrix3<f64>) -> SMatrix<f64, 9, 9> {
        let f_inv_t = invert_transpose(f);
        let ln_j = f.determinant().ln();
        let i1m3 = f.norm_squared() - 3.0;
        // NH part — bit-exact reproduction of `neo_hookean::tangent`'s
        // 4-loop assembly. `lambda.mul_add(-ln_j, mu)` matches NH's FMA
        // call site (plain ops drift 1 ULP for nontrivial `ln_j`).
        let cross_coeff = self.lambda.mul_add(-ln_j, self.mu);
        let mut t = SMatrix::<f64, 9, 9>::zeros();
        for i in 0..3 {
            for j in 0..3 {
                let row = i + 3 * j;
                for k in 0..3 {
                    for l in 0..3 {
                        let col = k + 3 * l;
                        let cross = cross_coeff * f_inv_t[(i, l)] * f_inv_t[(k, j)];
                        let direct = self.lambda * f_inv_t[(i, j)] * f_inv_t[(k, l)];
                        let identity = if i == k && j == l { self.mu } else { 0.0 };
                        t[(row, col)] = cross + direct + identity;
                    }
                }
            }
        }
        // Yeoh extra: 4 C₂ (I₁−3) δ_ik δ_jl + 8 C₂ F_ij F_kl. Both
        // contributions vanish at C₂ = 0 (additive decomposition; see
        // module doc).
        let yeoh_id_coeff = 4.0 * self.c2 * i1m3;
        let yeoh_outer_coeff = 8.0 * self.c2;
        for i in 0..3 {
            for j in 0..3 {
                let row = i + 3 * j;
                for k in 0..3 {
                    for l in 0..3 {
                        let col = k + 3 * l;
                        let yeoh_identity = if i == k && j == l { yeoh_id_coeff } else { 0.0 };
                        let yeoh_outer = yeoh_outer_coeff * f[(i, j)] * f[(k, l)];
                        t[(row, col)] += yeoh_identity + yeoh_outer;
                    }
                }
            }
        }
        t
    }

    fn validity(&self) -> ValidityDomain {
        ValidityDomain {
            max_stretch_deviation: 1.0,
            max_principal_stretch: self.max_principal_stretch,
            min_principal_stretch: self.min_principal_stretch,
            max_rotation: f64::INFINITY,
            poisson_range: (-1.0, 0.45),
            temperature_range: None,
            strain_rate_range: None,
            inversion: InversionHandling::RequireOrientation,
        }
    }
}

// Yeoh declares `InversionHandling::RequireOrientation`; non-invertible
// F is an IPC-barrier failure upstream, not a constitutive branch.
#[allow(clippy::expect_used)]
fn invert_transpose(f: &Matrix3<f64>) -> Matrix3<f64> {
    f.try_inverse()
        .expect("non-invertible F in Yeoh; IPC barrier should prevent this")
        .transpose()
}
