// The constitutive law, per element: `sim-soft`'s compressible Yeoh, which is
// its neo-Hookean when C₂ = 0 (`sim/L0/soft/src/material/yeoh.rs`,
// `neo_hookean.rs`):
//
//     Ψ(F) = μ/2 (I₁ − 3) − μ ln J + C₂ (I₁ − 3)² + λ/2 (ln J)²
//     P(F) = μ (F − F⁻ᵀ) + 4 C₂ (I₁ − 3) F + λ ln J F⁻ᵀ
//
// Selective averaged nodal pressure (plan §15c) evaluates the terms carrying
// μ and C₂ per element and averages only the λ term over nodes, so the two
// parts are written separately: "the μ terms" and "the λ term".

/// One element's material. `#[repr(C)]` with four scalars (16 bytes at
/// `f32`), so it can sit in a GPU buffer as is.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Material {
    /// Shear modulus μ (Pa).
    pub mu: R,
    /// Lamé's first parameter λ (Pa): the volumetric stiffness.
    pub lambda: R,
    /// Yeoh's C₂ (Pa). Zero gives neo-Hookean.
    pub c2: R,
    /// Mass density ρ (kg/m³).
    pub density: R,
}

/// Whether a volume ratio `J` is inverted or degenerate, `J ≤ 0`.
///
/// An explicit comparison, never a `NaN` test: WGSL lets an implementation
/// assume `NaN` and infinity never occur (plan §13d rule 2), so an inversion
/// must be caught before it reaches `ln J`.
#[must_use]
pub const fn is_inverted(j: R) -> bool {
    j <= 0.0
}

/// `J`, or 1 where `J ≤ 0`.
///
/// The guard for the operand of `ln J` and `1 / J`: both arms of a WGSL `select` are evaluated, so the operand is guarded,
/// not the result. An inverted element's stress is then finite and
/// meaningless; [`is_inverted`] is what reports it.
#[must_use]
pub const fn guarded_volume_ratio(j: R) -> R {
    if j > 0.0 { j } else { 1.0 }
}

/// The μ terms of the first Piola–Kirchhoff stress:
/// `μ (F − F⁻ᵀ) + 4 C₂ (I₁ − 3) F`.
///
/// The Yeoh term is added to the neo-Hookean one, not folded into a combined
/// coefficient, so C₂ = 0 reproduces neo-Hookean exactly (the order
/// `sim-soft`'s Yeoh keeps for the same reason).
#[must_use]
pub const fn first_piola_mu_terms(f: [R; 9], material: Material) -> [R; 9] {
    let j = guarded_volume_ratio(mat3_det(f));
    let f_inverse_transpose = mat3_scale(mat3_cofactor(f), 1.0 / j);
    let neo_hookean = mat3_scale(mat3_sub(f, f_inverse_transpose), material.mu);
    let i1_minus_3 = mat3_frobenius_squared(f) - 3.0;
    let yeoh = mat3_scale(f, 4.0 * material.c2 * i1_minus_3);
    mat3_add(neo_hookean, yeoh)
}

/// The full first Piola–Kirchhoff stress, `P(F)`.
///
/// The μ terms plus the λ term `λ ln J F⁻ᵀ`. The explicit solver does not use it (it averages the
/// λ term over nodes); it is the definition the split must add up to.
#[must_use]
pub fn first_piola(f: [R; 9], material: Material) -> [R; 9] {
    let j = guarded_volume_ratio(mat3_det(f));
    let f_inverse_transpose = mat3_scale(mat3_cofactor(f), 1.0 / j);
    let lambda_term = mat3_scale(f_inverse_transpose, material.lambda * j.ln());
    mat3_add(first_piola_mu_terms(f, material), lambda_term)
}

/// The μ terms' energy density: `μ/2 (I₁ − 3) − μ ln J + C₂ (I₁ − 3)²`.
#[must_use]
pub fn energy_density_mu_terms(f: [R; 9], material: Material) -> R {
    let ln_j = guarded_volume_ratio(mat3_det(f)).ln();
    let i1_minus_3 = mat3_frobenius_squared(f) - 3.0;
    0.5 * material.mu * i1_minus_3 - material.mu * ln_j + material.c2 * i1_minus_3 * i1_minus_3
}

/// The λ term's energy density at volume ratio `j`: `λ/2 (ln J)²`.
#[must_use]
pub fn energy_density_lambda_term(j: R, lambda: R) -> R {
    let ln_j = guarded_volume_ratio(j).ln();
    0.5 * lambda * ln_j * ln_j
}

/// The full energy density `Ψ(F)`.
#[must_use]
pub fn energy_density(f: [R; 9], material: Material) -> R {
    energy_density_mu_terms(f, material) + energy_density_lambda_term(mat3_det(f), material.lambda)
}

/// The λ term's pressure per unit λ at volume ratio `j`:
/// `U′(J) / λ = ln J / J`, where `U = λ/2 (ln J)²`.
///
/// It depends on no material, so a node can carry it for the elements of
/// every material around it (see `element_pressure`).
#[must_use]
pub fn pressure_per_lambda(j: R) -> R {
    let guarded = guarded_volume_ratio(j);
    guarded.ln() / guarded
}
