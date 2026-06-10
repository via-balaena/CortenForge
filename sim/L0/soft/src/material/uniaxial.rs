//! Homogeneous uniaxial-tension response and calibration for hyperelastic
//! materials — the analytical primitive behind the soft-FEM fidelity arc
//! (Mission Layer-1; see `docs/soft_fidelity/`).
//!
//! Under uniaxial tension with **traction-free lateral faces**, an
//! isotropic hyperelastic body deforms homogeneously as
//! `F = diag(λ, λ_t, λ_t)`: the axial stretch `λ` is prescribed and the
//! transverse stretch `λ_t` is whatever makes the lateral first-Piola
//! traction vanish (`P₁₁ = P₂₂ = 0`). [`free_transverse_uniaxial`] solves
//! that one-dimensional condition on the real [`Material`] surface, so it
//! exercises the *same* `first_piola` the FEM solver uses — there is no
//! parallel closed form to drift from. This is the analytical, locking-free
//! reference a single-element FEM uniaxial coupon must reproduce, and the
//! curve we grade against measured tensile data.
//!
//! [`fit_yeoh_uniaxial`] is the inverse: given a measured
//! (stretch, true-stress) curve, calibrate a compressible 2-parameter
//! [`Yeoh`] to it over a stretch window. Used to replace the
//! single-datasheet-point `silicone_table` params with measured-curve fits.

use nalgebra::Matrix3;

use super::{Material, Yeoh};

/// Homogeneous response of an isotropic hyperelastic material under
/// uniaxial tension with traction-free lateral faces.
///
/// All stresses are axial (the lateral components are zero by
/// construction). `cauchy_stress` is the true stress
/// `σ = P₀₀ · λ / J`; `nominal_stress` is the engineering / first-Piola
/// stress `P₀₀` (force per *undeformed* area).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UniaxialResponse {
    /// Prescribed axial stretch `λ = l/l₀`.
    pub axial_stretch: f64,
    /// Solved transverse stretch `λ_t` (lateral contraction; `< 1` in
    /// tension).
    pub transverse_stretch: f64,
    /// Axial true (Cauchy) stress in pascals.
    pub cauchy_stress: f64,
    /// Axial nominal (engineering / first-Piola) stress in pascals.
    pub nominal_stress: f64,
}

/// Solve the free-transverse uniaxial-tension response of `material` at
/// the given `axial_stretch` (≥ 1).
///
/// Root-finds the transverse stretch `λ_t ∈ (0, 1]` that zeroes the
/// lateral first-Piola component `P₁₁` (traction-free lateral faces) by
/// bisection — `P₁₁(λ_t)` is monotone increasing, negative as `λ_t → 0`
/// and `≥ 0` at `λ_t = 1` under axial tension, so the bracket `[ε, 1]`
/// always contains the root.
///
/// # Panics
/// Panics if `axial_stretch < 1.0` (this is the tension-only primitive;
/// compression flips the transverse bracket) or if the lateral bracket
/// does not straddle zero (a non-physical material, e.g. one that expands
/// laterally under axial tension).
#[must_use]
pub fn free_transverse_uniaxial<M: Material>(material: &M, axial_stretch: f64) -> UniaxialResponse {
    assert!(
        axial_stretch >= 1.0,
        "free_transverse_uniaxial is tension-only (axial_stretch ≥ 1); got {axial_stretch}"
    );
    let lateral_traction = |lt: f64| -> f64 {
        let f = Matrix3::new(axial_stretch, 0.0, 0.0, 0.0, lt, 0.0, 0.0, 0.0, lt);
        material.first_piola(&f)[(1, 1)]
    };

    let (mut lo, mut hi) = (1.0e-3_f64, 1.0_f64);
    let (flo, fhi) = (lateral_traction(lo), lateral_traction(hi));
    // Bisection precondition; `P₁₁` is monotone in `λ_t` for an isotropic
    // hyperelastic material under axial tension.
    assert!(
        flo <= 0.0 && fhi >= 0.0,
        "no transverse bracket at axial_stretch={axial_stretch}: P11({lo})={flo}, P11({hi})={fhi}"
    );
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if lateral_traction(mid) > 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
        if hi - lo < 1.0e-13 {
            break;
        }
    }
    let lt = 0.5 * (lo + hi);
    let f = Matrix3::new(axial_stretch, 0.0, 0.0, 0.0, lt, 0.0, 0.0, 0.0, lt);
    let p = material.first_piola(&f);
    let j = axial_stretch * lt * lt;
    let nominal = p[(0, 0)];
    UniaxialResponse {
        axial_stretch,
        transverse_stretch: lt,
        cauchy_stress: nominal * axial_stretch / j,
        nominal_stress: nominal,
    }
}

/// Calibrate a compressible 2-parameter [`Yeoh`] to a measured uniaxial
/// true-stress curve over a stretch window.
///
/// `curve` is `(stretch, cauchy_stress_pa)` samples; `window = (lo, hi)`
/// selects the stretch range to fit (M1 fits the **device** window, not
/// the to-failure tail — a to-failure fit trades away low-stretch
/// accuracy). Poisson ratio is held at the `silicone_table` convention
/// `ν = 0.40` (`λ = 4 μ`); only `(μ, C₂)` are free. Minimises the RMS
/// true-stress residual via a two-stage grid (coarse box → local refine),
/// evaluating each candidate through [`free_transverse_uniaxial`] so the
/// fit targets the *same* compressible response the solver produces.
///
/// # Panics
/// Panics if no `curve` samples fall inside `window`.
// The `f64` casts are small grid indices (`i`, `k`, `n` ≤ a few dozen)
// and the sample count (`pts.len()`), all far inside f64's exact-integer
// range — no precision is lost.
#[allow(clippy::cast_precision_loss)]
#[must_use]
pub fn fit_yeoh_uniaxial(curve: &[(f64, f64)], window: (f64, f64)) -> Yeoh {
    let pts: Vec<(f64, f64)> = curve
        .iter()
        .copied()
        .filter(|&(l, _)| l >= window.0 && l <= window.1)
        .collect();
    assert!(
        !pts.is_empty(),
        "fit_yeoh_uniaxial: no curve samples in window {window:?}"
    );

    let rms = |mu: f64, c2: f64| -> f64 {
        let mat = Yeoh::from_lame_and_c2(mu, 4.0 * mu, c2);
        let sse: f64 = pts
            .iter()
            .map(|&(l, sig)| {
                let r = free_transverse_uniaxial(&mat, l).cauchy_stress - sig;
                r * r
            })
            .sum();
        (sse / pts.len() as f64).sqrt()
    };

    // Scale guess: small-strain secant μ ≈ σ/(λ²−1/λ) at the first point.
    let (l0, s0) = pts[0];
    let mu_guess = (s0 / (l0 * l0 - 1.0 / l0)).abs().max(1.0);

    // Stage 1 — coarse grid over a wide box around the guess.
    let grid = |mu_lo: f64, mu_hi: f64, c2_lo: f64, c2_hi: f64, n: usize| {
        let mut best = (f64::INFINITY, mu_guess, 0.0_f64);
        for i in 0..=n {
            let mu = mu_lo + (mu_hi - mu_lo) * i as f64 / n as f64;
            for k in 0..=n {
                let c2 = c2_lo + (c2_hi - c2_lo) * k as f64 / n as f64;
                let e = rms(mu, c2);
                if e < best.0 {
                    best = (e, mu, c2);
                }
            }
        }
        best
    };
    let coarse = grid(0.2 * mu_guess, 2.0 * mu_guess, 0.0, 2.0 * mu_guess, 40);
    // Stage 2 — refine in a local box around the coarse optimum.
    let mu_span = 0.1 * mu_guess;
    let c2_span = 0.1 * mu_guess;
    let fine = grid(
        (coarse.1 - mu_span).max(1.0),
        coarse.1 + mu_span,
        (coarse.2 - c2_span).max(0.0),
        coarse.2 + c2_span,
        40,
    );
    let (mu, c2) = (fine.1, fine.2);
    Yeoh::from_lame_and_c2(mu, 4.0 * mu, c2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::silicone_table::ECOFLEX_00_30;

    /// The defining condition: at the solved `λ_t`, the lateral traction
    /// `P₁₁` must vanish across the whole tension sweep.
    #[test]
    fn free_transverse_zeroes_lateral_traction() {
        let mat = ECOFLEX_00_30.to_yeoh();
        for &lam in &[1.0, 1.1, 1.5, 2.0, 3.0] {
            let r = free_transverse_uniaxial(&mat, lam);
            let f = Matrix3::new(
                r.axial_stretch,
                0.0,
                0.0,
                0.0,
                r.transverse_stretch,
                0.0,
                0.0,
                0.0,
                r.transverse_stretch,
            );
            let p11 = mat.first_piola(&f)[(1, 1)];
            // Stress scale ~kPa; bisection to 1e-13 in λ_t gives sub-Pa residual.
            assert!(
                p11.abs() < 1.0,
                "lateral traction not zeroed at λ={lam}: P11={p11}"
            );
        }
    }

    /// Small-strain limit must match isotropic linear elasticity:
    /// axial Cauchy ≈ E·ε with `E = μ(3λ_L+2μ)/(λ_L+μ)`, `ν = 0.40`.
    /// Tested on a C₂ = 0 (Neo-Hookean) material so the linear-elastic
    /// limit is isolated from the Yeoh nonlinear term (which adds a small
    /// finite stiffening even near ε = 0).
    #[test]
    fn free_transverse_matches_linear_elastic_at_small_strain() {
        let e = ECOFLEX_00_30.to_yeoh();
        let mat = Yeoh::from_lame_and_c2(e.mu(), e.lambda(), 0.0);
        let (mu, lam_l) = (mat.mu(), mat.lambda());
        let young = mu * (3.0 * lam_l + 2.0 * mu) / (lam_l + mu);
        let eps = 1.0e-4;
        let r = free_transverse_uniaxial(&mat, 1.0 + eps);
        let expected = young * eps;
        assert!(
            (r.cauchy_stress - expected).abs() / expected < 5.0e-3,
            "small-strain σ {} vs Eε {expected}",
            r.cauchy_stress
        );
        // Poisson check: λ_t ≈ 1 − ν·ε, ν = 0.40.
        let nu = lam_l / (2.0 * (lam_l + mu));
        assert!((r.transverse_stretch - (1.0 - nu * eps)).abs() < 1.0e-6);
    }

    /// The fitter must recover the parameters of a synthetic curve
    /// generated from a known `Yeoh` (round-trip identifiability).
    #[test]
    fn fit_recovers_known_yeoh() {
        let truth = Yeoh::from_lame_and_c2(15_000.0, 60_000.0, 800.0); // ν=0.40
        let curve: Vec<(f64, f64)> = (0..=40)
            .map(|i| {
                let lam = 0.05f64.mul_add(f64::from(i), 1.0); // 1.0 .. 3.0
                (lam, free_transverse_uniaxial(&truth, lam).cauchy_stress)
            })
            .collect();
        let fit = fit_yeoh_uniaxial(&curve, (1.1, 2.0));
        assert!(
            (fit.mu() - 15_000.0).abs() / 15_000.0 < 0.05,
            "μ recovered {}",
            fit.mu()
        );
        assert!(
            (fit.c2() - 800.0).abs() < 250.0,
            "C₂ recovered {}",
            fit.c2()
        );
    }
}
