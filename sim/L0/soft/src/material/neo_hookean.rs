//! Compressible Neo-Hookean material — closed-form `P` and tangent.
//!
//! Closed forms from Part 2 Ch 04 [00-energy.md][e] + [01-tangent.md][t]:
//!
//! - `ψ(F) = (μ/2)(I₁ − 3) − μ ln J + (λ/2)(ln J)²`
//! - `P(F) = μ(F − F⁻ᵀ) + λ (ln J) F⁻ᵀ`
//! - `C_ijkl = μ δ_ik δ_jl + (μ − λ ln J) F⁻ᵀ_il F⁻ᵀ_kj + λ F⁻ᵀ_ij F⁻ᵀ_kl`
//!
//! Inversion handling is `RequireOrientation` (spec §3; book 03-impl.md):
//! `first_piola` / `tangent` panic on non-positive `det F`, which the
//! IPC barrier is expected to prevent at contact time. The small-strain
//! cancellation-safe branch (log1p on `J`, Frobenius-decomposed `I₁ − 3`)
//! from Part 2 Ch 04 03-impl.md §88 is deferred to the gradcheck session
//! that first exercises `F ≈ I` at six-digit agreement with linear.
//!
//! [e]: docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/00-energy.md
//! [t]: docs/studies/soft_body_architecture/src/20-materials/04-hyperelastic/00-neo-hookean/01-tangent.md

use nalgebra::{Matrix3, SMatrix};

use super::{InversionHandling, Material, ValidityDomain};

/// Compressible Neo-Hookean energy and its derivatives. Skeleton scene
/// parameters per spec §2: `mu = 1e5`, `lambda = 4e5` (Ecoflex-class,
/// `ν ≈ 0.40`).
#[derive(Clone, Debug)]
pub struct NeoHookean {
    mu: f64,
    lambda: f64,
}

impl NeoHookean {
    /// Construct from Lamé parameters `(μ, λ)` directly.
    #[must_use]
    pub const fn from_lame(mu: f64, lambda: f64) -> Self {
        Self { mu, lambda }
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

    /// Construct from Young's modulus and Poisson ratio `(E, ν)` via the
    /// isotropic-elasticity conversion `μ = E / 2(1+ν)`,
    /// `λ = Eν / [(1+ν)(1−2ν)]`.
    ///
    /// # Panics
    /// On `ν ≥ 0.45`. The standalone compressible law's validity domain
    /// caps Poisson at 0.45 (Part 2 Ch 04 03-impl.md §67); higher ratios
    /// require wrapping in the Ch 05 mixed-u-p or F-bar locking-fix
    /// decorator, which widens the bound to 0.499.
    #[must_use]
    pub fn from_young_poisson(young: f64, nu: f64) -> Self {
        assert!(
            nu < 0.45,
            "standalone NeoHookean requires nu < 0.45; use the Ch 05 locking-fix decorator for higher Poisson ratios"
        );
        let mu = young / (2.0 * (1.0 + nu));
        let lambda = young * nu / ((1.0 + nu) * 2.0_f64.mul_add(-nu, 1.0));
        Self { mu, lambda }
    }
}

impl Material for NeoHookean {
    fn energy(&self, f: &Matrix3<f64>) -> f64 {
        let j = f.determinant();
        let i1 = f.norm_squared();
        let ln_j = j.ln();
        let half_mu = 0.5 * self.mu;
        let half_lambda = 0.5 * self.lambda;
        // W = (μ/2)(I₁ − 3) + (λ/2)(ln J)² − μ ln J
        half_lambda.mul_add(ln_j * ln_j, half_mu.mul_add(i1 - 3.0, -self.mu * ln_j))
    }

    fn first_piola(&self, f: &Matrix3<f64>) -> Matrix3<f64> {
        let f_inv_t = invert_transpose(f);
        let ln_j = f.determinant().ln();
        self.mu * (f - f_inv_t) + self.lambda * ln_j * f_inv_t
    }

    fn tangent(&self, f: &Matrix3<f64>) -> SMatrix<f64, 9, 9> {
        let f_inv_t = invert_transpose(f);
        let ln_j = f.determinant().ln();
        assemble_tangent_9x9(self.mu, self.lambda, &f_inv_t, ln_j)
    }

    fn validity(&self) -> ValidityDomain {
        ValidityDomain {
            max_stretch_deviation: 1.0,
            max_principal_stretch: None,
            min_principal_stretch: None,
            max_rotation: f64::INFINITY,
            poisson_range: (-1.0, 0.45),
            temperature_range: None,
            strain_rate_range: None,
            inversion: InversionHandling::RequireOrientation,
        }
    }

    /// `[∂P/∂μ, ∂P/∂λ]` in `(μ, λ)` order. From `P = μ(F − F⁻ᵀ) + λ ln(J) F⁻ᵀ`:
    /// `∂P/∂μ = F − F⁻ᵀ`, `∂P/∂λ = ln(J) · F⁻ᵀ` (both independent of `μ, λ`).
    fn first_piola_param_grad(&self, f: &Matrix3<f64>) -> Vec<Matrix3<f64>> {
        let f_inv_t = invert_transpose(f);
        let ln_j = f.determinant().ln();
        vec![f - f_inv_t, ln_j * f_inv_t]
    }
}

// NH declares `InversionHandling::RequireOrientation`; non-invertible F
// is an IPC-barrier failure upstream, not a constitutive branch.
#[allow(clippy::expect_used)]
fn invert_transpose(f: &Matrix3<f64>) -> Matrix3<f64> {
    f.try_inverse()
        .expect("non-invertible F in NeoHookean; IPC barrier should prevent this")
        .transpose()
}

// 4-nested index loop matches the C_ijkl closed form; `i j k l` are the
// tensor indices from Part 2 Ch 04 01-tangent.md, so renaming loses the
// one-to-one mapping between the math and the code.
#[allow(clippy::many_single_char_names)]
fn assemble_tangent_9x9(
    mu: f64,
    lambda: f64,
    f_inv_t: &Matrix3<f64>,
    ln_j: f64,
) -> SMatrix<f64, 9, 9> {
    // cross_coeff = μ − λ ln J, the coefficient of F⁻ᵀ_il F⁻ᵀ_kj in the
    // closed form.
    let cross_coeff = lambda.mul_add(-ln_j, mu);
    let mut t = SMatrix::<f64, 9, 9>::zeros();
    for i in 0..3 {
        for j in 0..3 {
            let row = i + 3 * j;
            for k in 0..3 {
                for l in 0..3 {
                    let col = k + 3 * l;
                    let cross = cross_coeff * f_inv_t[(i, l)] * f_inv_t[(k, j)];
                    let direct = lambda * f_inv_t[(i, j)] * f_inv_t[(k, l)];
                    let identity = if i == k && j == l { mu } else { 0.0 };
                    t[(row, col)] = cross + direct + identity;
                }
            }
        }
    }
    t
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use nalgebra::{Matrix3, Vector3};

    use super::*;

    /// Skeleton Ecoflex-class Lamé pair (`ν = 0.40`, `λ = 4μ`) per spec §2.
    const MU: f64 = 1.0e5;
    const LAMBDA: f64 = 4.0e5;

    /// `F = diag(s, 1, 1)` — simple uniaxial stretch (transverse stretches
    /// pinned at 1, so `J = s`); the configuration whose closed form is
    /// scalar arithmetic with no inner Newton solve.
    fn diag_stretch(s: f64) -> Matrix3<f64> {
        Matrix3::from_diagonal(&Vector3::new(s, 1.0, 1.0))
    }

    #[test]
    fn first_piola_matches_closed_form_at_uniaxial_stretch() {
        // At F = diag(s, 1, 1): J = s, F⁻ᵀ = diag(1/s, 1, 1), so
        // P = μ(F − F⁻ᵀ) + Λ ln(J) F⁻ᵀ is diagonal with
        //   P_11 = μ(s − 1/s) + Λ ln(s) / s
        //   P_22 = P_33 = Λ ln(s)
        // and vanishing off-diagonals. The expected values use straight
        // scalar arithmetic (no `mul_add`), independent of the matrix
        // implementation — a genuine cross-check, not a re-run of the
        // impl's own FMA chain.
        let nh = NeoHookean::from_lame(MU, LAMBDA);
        for &s in &[2.0_f64, 1.5, 0.8] {
            let p = nh.first_piola(&diag_stretch(s));
            let ln_s = s.ln();
            let p11 = MU * (s - 1.0 / s) + LAMBDA * ln_s / s;
            let p22 = LAMBDA * ln_s;
            assert_relative_eq!(p[(0, 0)], p11, max_relative = 1.0e-12);
            // P_22 matches the closed form Λ ln(s)...
            assert_relative_eq!(p[(1, 1)], p22, max_relative = 1.0e-12);
            // ...and P_33 == P_22 by transverse symmetry (F leaves the
            // (2, 3)-plane symmetric). Asserting P_33 against P_22 rather
            // than re-asserting it against `p22` keeps this a distinct
            // symmetry check — the direction the sibling test in
            // `yeoh_contract.rs` covers only for `Yeoh`, not `NeoHookean`.
            assert_relative_eq!(p[(2, 2)], p[(1, 1)], max_relative = 1.0e-12);
            for i in 0..3 {
                for j in 0..3 {
                    if i != j {
                        assert_relative_eq!(p[(i, j)], 0.0, epsilon = 1.0e-9);
                    }
                }
            }
        }
    }

    #[test]
    fn energy_matches_closed_form_at_uniaxial_stretch() {
        // ψ(diag(s, 1, 1)) = (μ/2)(I₁ − 3) − μ ln J + (Λ/2)(ln J)²
        //                  = (μ/2)(s² − 1) − μ ln(s) + (Λ/2)(ln s)²
        let nh = NeoHookean::from_lame(MU, LAMBDA);
        for &s in &[2.0_f64, 1.5, 0.8] {
            let ln_s = s.ln();
            let expected = 0.5 * MU * (s * s - 1.0) - MU * ln_s + 0.5 * LAMBDA * ln_s * ln_s;
            assert_relative_eq!(
                nh.energy(&diag_stretch(s)),
                expected,
                max_relative = 1.0e-12
            );
        }
    }

    #[test]
    fn rest_configuration_is_stress_and_energy_free() {
        // At F = I: F − F⁻ᵀ = 0 and ln(det I) = 0, so both P and ψ vanish.
        let nh = NeoHookean::from_lame(MU, LAMBDA);
        let p = nh.first_piola(&Matrix3::identity());
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(p[(i, j)], 0.0, epsilon = 1.0e-12);
            }
        }
        assert_relative_eq!(nh.energy(&Matrix3::identity()), 0.0, epsilon = 1.0e-12);
    }
}
