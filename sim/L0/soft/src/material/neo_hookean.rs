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
    /// Shear modulus `μ` (Pa).
    pub mu: f64,
    /// First Lamé parameter `λ` (Pa).
    pub lambda: f64,
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
            max_rotation: f64::INFINITY,
            poisson_range: (-1.0, 0.45),
            temperature_range: None,
            strain_rate_range: None,
            inversion: InversionHandling::RequireOrientation,
        }
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
