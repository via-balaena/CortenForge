// The constitutive law, per element: `sim-soft`'s compressible Yeoh, which is
// its neo-Hookean when C₂ = 0 (`sim/L0/soft/src/material/yeoh.rs`,
// `neo_hookean.rs`):
//
//     Ψ(F) = μ/2 (I₁ − 3) − μ ln J + C₂ (I₁ − 3)² + λ/2 (ln J)²
//     P(F) = μ (F − F⁻ᵀ) + 4 C₂ (I₁ − 3) F + λ ln J F⁻ᵀ
//
// plus a deviatoric Kelvin–Voigt viscosity η, the material's own loss:
//
//     σ_v = 2η dev D,   D = sym(Ḟ F⁻¹),   P_v = σ_v cof F
//
// It is stress from the rate of deformation, so it vanishes at rest. It does
// not move a frictionless seated reading; a frictional one depends on the path
// it took, which the viscosity changes (plan §16p).
//
// It is written in the displacement gradient `H = F − I` (plan §6): each
// quantity that is zero at rest (`J − 1`, `I₁ − 3`, `F − F⁻ᵀ`, `ln J`) is built
// from `H` directly, never as a difference of two numbers near 1.
//
// Selective averaged nodal pressure (plan §15c) evaluates the terms carrying
// μ and C₂ per element and averages only the λ term over nodes, so the two
// parts are written separately: "the μ terms" and "the λ term".

/// One element's material. `#[repr(C)]` with five scalars (20 bytes at
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
    /// The deviatoric Kelvin–Voigt viscosity η (Pa·s). Zero gives an
    /// elastic material.
    pub viscosity: R,
    /// Mass density ρ (kg/m³).
    pub density: R,
}

/// Whether a dilation `J − 1` means an inverted or degenerate volume,
/// `J ≤ 0`.
///
/// An explicit comparison, never a `NaN` test: WGSL lets an implementation
/// assume `NaN` and infinity never occur (plan §13d rule 2), so an inversion
/// must be caught before it reaches `ln J`.
#[must_use]
pub const fn is_inverted(dilation: R) -> bool {
    dilation <= -1.0
}

/// The dilation `J − 1`, or 0 where `J ≤ 0`.
///
/// The guard for the operand of `ln J` and `1 / J`: both arms of a WGSL
/// `select` are evaluated, so the operand is guarded, not the result. An
/// inverted element's stress is then finite and meaningless; [`is_inverted`]
/// is what reports it.
#[must_use]
pub const fn guarded_dilation(dilation: R) -> R {
    if dilation > -1.0 { dilation } else { 0.0 }
}

/// `ln(1 + x)` for `x > −1`, without forming `1 + x` where `x` is small.
///
/// For `|x| ≤ 0.25` it sums `2 atanh(y)`, `y = x / (2 + x)`, to `y²¹`;
/// elsewhere it is `ln(1 + x)`. Written here rather than left to each
/// backend's `ln`, so the CPU and the GPU evaluate the same expression.
// The coefficients are 1/3, 1/5, …, 1/21 to f64 precision, more than f32
// holds; the partial sums are named for the power each starts at; `ln_1p` is
// not in the subset, and `(1 + x).ln()` is used only where |x| > 0.25.
#[allow(
    clippy::excessive_precision,
    clippy::similar_names,
    clippy::imprecise_flops
)]
#[must_use]
pub fn ln_1p(x: R) -> R {
    let y = x / (2.0 + x);
    let y2 = y * y;
    // Horner's rule, from the y²¹ term down: `tail_k` sums the terms from
    // `yᵏ / k` on, divided by `yᵏ`.
    let tail_19 = 0.047_619_047_619_047_616 * y2 + 0.052_631_578_947_368_42;
    let tail_17 = tail_19 * y2 + 0.058_823_529_411_764_705;
    let tail_15 = tail_17 * y2 + 0.066_666_666_666_666_67;
    let tail_13 = tail_15 * y2 + 0.076_923_076_923_076_93;
    let tail_11 = tail_13 * y2 + 0.090_909_090_909_090_91;
    let tail_9 = tail_11 * y2 + 0.111_111_111_111_111_11;
    let tail_7 = tail_9 * y2 + 0.142_857_142_857_142_85;
    let tail_5 = tail_7 * y2 + 0.2;
    let tail_3 = tail_5 * y2 + 0.333_333_333_333_333_3;
    let tail_1 = tail_3 * y2 + 1.0;
    let series = 2.0 * y * tail_1;
    let direct = (1.0 + x).ln();
    if x.abs() <= 0.25 { series } else { direct }
}

/// `J − 1 = det(I + H) − 1`, by expansion: `tr H + m₂(H) + det H`, with
/// `m₂` the sum of `H`'s principal 2×2 minors.
#[must_use]
pub const fn gradient_dilation(h: [R; 9]) -> R {
    mat3_trace(h) + (mat3_principal_minors(h) + mat3_det(h))
}

/// `cof F = cof(I + H) = (1 + tr H) I − Hᵀ + cof H`, the cofactor of the
/// deformation gradient, which is `J F⁻ᵀ`.
#[must_use]
pub const fn deformation_cofactor(h: [R; 9]) -> [R; 9] {
    mat3_add_scaled_identity(
        mat3_sub(mat3_cofactor(h), mat3_transpose(h)),
        1.0 + mat3_trace(h),
    )
}

/// The μ terms of the first Piola–Kirchhoff stress:
/// `μ (F − F⁻ᵀ) + 4 C₂ (I₁ − 3) F`.
///
/// `F − F⁻ᵀ = (J F − cof F) / J`, and
/// `J F − cof F = H + Hᵀ + (m₂ + det H) I + (J − 1) H − cof H`, every term of
/// which vanishes at rest. `I₁ − 3 = 2 tr H + ‖H‖²`. The Yeoh term is added
/// to the neo-Hookean one, not folded into a combined coefficient, so C₂ = 0
/// adds exactly zero.
#[must_use]
pub const fn first_piola_mu_terms(h: [R; 9], material: Material) -> [R; 9] {
    let beyond_trace = mat3_principal_minors(h) + mat3_det(h);
    let dilation = guarded_dilation(mat3_trace(h) + beyond_trace);
    let numerator = mat3_add_scaled_identity(
        mat3_sub(
            mat3_add(mat3_add(h, mat3_transpose(h)), mat3_scale(h, dilation)),
            mat3_cofactor(h),
        ),
        beyond_trace,
    );
    let neo_hookean = mat3_scale(numerator, material.mu / (1.0 + dilation));
    let i1_minus_3 = 2.0 * mat3_trace(h) + mat3_frobenius_squared(h);
    let yeoh = mat3_scale(
        mat3_add_scaled_identity(h, 1.0),
        4.0 * material.c2 * i1_minus_3,
    );
    mat3_add(neo_hookean, yeoh)
}

/// The viscous first Piola–Kirchhoff stress, `P_v = 2η dev(D) cof F`, from the
/// displacement gradient `h = F − I` and its rate `rate = Ḟ`.
///
/// `D = sym(L)` with `L = Ḟ F⁻¹ = Ḟ (cof F)ᵀ / J`, and `P_v = J σ_v F⁻ᵀ =
/// σ_v cof F`. Its power, `P_v : Ḟ = 2η J |dev D|²`, is never negative, and it
/// vanishes for a rigid motion (`D = 0`) and for a pure change of volume
/// (`dev D = 0`).
#[must_use]
pub const fn first_piola_viscous(h: [R; 9], rate: [R; 9], viscosity: R) -> [R; 9] {
    let cofactor = deformation_cofactor(h);
    let jacobian = 1.0 + guarded_dilation(gradient_dilation(h));
    let velocity_gradient = mat3_scale(mat3_mul(rate, mat3_transpose(cofactor)), 1.0 / jacobian);
    let stretching = mat3_scale(
        mat3_add(velocity_gradient, mat3_transpose(velocity_gradient)),
        0.5,
    );
    let deviator = mat3_add_scaled_identity(stretching, -mat3_trace(stretching) / 3.0);
    mat3_mul(mat3_scale(deviator, 2.0 * viscosity), cofactor)
}

/// The full first Piola–Kirchhoff stress, `P(F)`.
///
/// The μ terms plus the λ term `λ ln J F⁻ᵀ`. The explicit solver does not
/// use it (it averages the λ term over nodes); it is the definition the split
/// must add up to.
#[must_use]
pub fn first_piola(h: [R; 9], material: Material) -> [R; 9] {
    let dilation = guarded_dilation(gradient_dilation(h));
    let lambda_term = mat3_scale(
        deformation_cofactor(h),
        material.lambda * ln_1p(dilation) / (1.0 + dilation),
    );
    mat3_add(first_piola_mu_terms(h, material), lambda_term)
}

/// The μ terms' energy density, `μ/2 (I₁ − 3) − μ ln J + C₂ (I₁ − 3)²`,
/// as `μ/2 ‖H‖² − μ (m₂ + det H) + μ ((J − 1) − ln J) + C₂ (I₁ − 3)²`.
#[must_use]
pub fn energy_density_mu_terms(h: [R; 9], material: Material) -> R {
    let beyond_trace = mat3_principal_minors(h) + mat3_det(h);
    let dilation = guarded_dilation(mat3_trace(h) + beyond_trace);
    let squared = mat3_frobenius_squared(h);
    let i1_minus_3 = 2.0 * mat3_trace(h) + squared;
    0.5 * material.mu * squared - material.mu * beyond_trace
        + material.mu * (dilation - ln_1p(dilation))
        + material.c2 * i1_minus_3 * i1_minus_3
}

/// The λ term's energy density at dilation `J − 1`: `λ/2 (ln J)²`.
#[must_use]
pub fn energy_density_lambda_term(dilation: R, lambda: R) -> R {
    let ln_j = ln_1p(guarded_dilation(dilation));
    0.5 * lambda * ln_j * ln_j
}

/// The full energy density `Ψ(F)`.
#[must_use]
pub fn energy_density(h: [R; 9], material: Material) -> R {
    energy_density_mu_terms(h, material)
        + energy_density_lambda_term(gradient_dilation(h), material.lambda)
}

/// The λ term's pressure at dilation `J − 1`: `U′(J) = λ ln J / J`, where
/// `U = λ/2 (ln J)²`.
#[must_use]
pub fn pressure_lambda_term(dilation: R, lambda: R) -> R {
    let guarded = guarded_dilation(dilation);
    lambda * ln_1p(guarded) / (1.0 + guarded)
}
