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

/// Homogeneous tension deformation mode for [`free_transverse`].
///
/// Each drives one or two in-plane axes by `λ` and leaves the out-of-plane
/// (thickness) direction traction-free:
/// - `Uniaxial`: `F = diag(λ, λ_t, λ_t)` — both lateral directions free, equal.
/// - `Planar` (pure shear): `F = diag(λ, 1, λ_t)` — width held at 1, thickness
///   free. The constrained-width direction carries a non-zero reaction stress.
/// - `Equibiaxial`: `F = diag(λ, λ, λ_t)` — two axes driven equally, thickness
///   free.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeformationMode {
    /// Uniaxial tension.
    Uniaxial,
    /// Planar tension (pure shear): in-plane width fixed.
    Planar,
    /// Equibiaxial tension: two in-plane axes driven equally.
    Equibiaxial,
}

/// Solve the homogeneous free-transverse response of `material` in `mode` at
/// the driven `axial_stretch` (≥ 1).
///
/// Root-finds the free (out-of-plane / thickness) stretch `λ_t ∈ (0, 1]` that
/// zeroes its first-Piola traction `P₃₃` (traction-free thickness face) by
/// bisection — `P₃₃(λ_t)` is monotone increasing, negative as `λ_t → 0` and
/// `≥ 0` at `λ_t = 1` under tension, so the bracket `[ε, 1]` contains the root.
/// (For `Uniaxial`, `P₃₃ ≡ P₂₂` by symmetry.) Evaluated on the real
/// [`Material::first_piola`], so it exercises the *same* constitutive surface
/// the FEM solver uses. The reported stresses are along the driven axis;
/// `UniaxialResponse::transverse_stretch` is the free (thickness) stretch.
///
/// # Panics
/// Panics if `axial_stretch < 1.0` (tension-only) or if the thickness bracket
/// does not straddle zero (a non-physical material that expands under tension).
#[must_use]
pub fn free_transverse<M: Material>(
    material: &M,
    mode: DeformationMode,
    axial_stretch: f64,
) -> UniaxialResponse {
    assert!(
        axial_stretch >= 1.0,
        "free_transverse is tension-only (axial_stretch ≥ 1); got {axial_stretch}"
    );
    let f_of = |t: f64| -> Matrix3<f64> {
        let driven2 = match mode {
            DeformationMode::Equibiaxial => axial_stretch,
            DeformationMode::Uniaxial => t,
            DeformationMode::Planar => 1.0,
        };
        Matrix3::new(axial_stretch, 0.0, 0.0, 0.0, driven2, 0.0, 0.0, 0.0, t)
    };
    // Thickness (index 2) is the traction-free direction in every mode.
    let thickness_traction = |t: f64| material.first_piola(&f_of(t))[(2, 2)];

    let (mut lo, mut hi) = (1.0e-3_f64, 1.0_f64);
    let (flo, fhi) = (thickness_traction(lo), thickness_traction(hi));
    assert!(
        flo <= 0.0 && fhi >= 0.0,
        "no thickness bracket for {mode:?} at axial_stretch={axial_stretch}: P33({lo})={flo}, P33({hi})={fhi}"
    );
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if thickness_traction(mid) > 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
        if hi - lo < 1.0e-13 {
            break;
        }
    }
    let lt = 0.5 * (lo + hi);
    let f = f_of(lt);
    let p = material.first_piola(&f);
    let j = f.determinant();
    let nominal = p[(0, 0)];
    UniaxialResponse {
        axial_stretch,
        transverse_stretch: lt,
        cauchy_stress: nominal * axial_stretch / j,
        nominal_stress: nominal,
    }
}

/// Uniaxial-tension special case of [`free_transverse`] (`F = diag(λ, λ_t,
/// λ_t)`, both lateral faces traction-free). Retained as the canonical M1/M2
/// entry point.
#[must_use]
pub fn free_transverse_uniaxial<M: Material>(material: &M, axial_stretch: f64) -> UniaxialResponse {
    free_transverse(material, DeformationMode::Uniaxial, axial_stretch)
}

/// Calibrate a compressible 2-parameter [`Yeoh`] to a measured uniaxial
/// true-stress curve over a stretch window.
///
/// `curve` is `(stretch, cauchy_stress_pa)` samples; `window = (lo, hi)`
/// selects the stretch range to fit (M1 fits the **device** window, not
/// the to-failure tail — a to-failure fit trades away low-stretch
/// accuracy). Poisson ratio is held at the `silicone_table` convention
/// `ν = 0.40` (`λ = 4 μ`); only `(μ, C₂)` are free. Minimises the
/// **absolute** RMS true-stress residual (a *relative* objective would
/// over-weight the near-zero toe) via a two-stage grid — a coarse box, then
/// a local refine centred on the coarse optimum — evaluating each candidate
/// through [`free_transverse_uniaxial`] so the fit targets the *same*
/// compressible response the solver produces. The grid is scaled for the
/// kPa-class silicones in [`crate::material::silicone_table`].
///
/// Returns a bare [`Yeoh`] with **no validity bounds**; attach
/// [`Yeoh::with_principal_stretch_bounds`] if the solver's stretch gate is
/// required.
///
/// # Panics
/// Panics if no `curve` samples fall inside `window`. Expects `window.0 > 1`
/// (uniaxial tension — the small-strain scale estimate is singular at λ = 1).
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

    // Scale guess: small-strain secant μ ≈ σ/(λ²−1/λ), taken as the MEDIAN
    // over the in-window points (robust to single-point toe noise). The
    // denominator is > 0 for λ > 1; points with a near-singular denominator
    // (λ ≈ 1) are skipped, and `.abs()` tolerates near-zero toe stress whose
    // sign is measurement noise.
    let mut secants: Vec<f64> = pts
        .iter()
        .filter(|&&(l, _)| (l * l - 1.0 / l) > 1.0e-3)
        .map(|&(l, s)| (s / (l * l - 1.0 / l)).abs())
        .collect();
    secants.sort_by(f64::total_cmp);
    let mu_guess = secants
        .get(secants.len() / 2)
        .copied()
        .unwrap_or(1.0)
        .max(1.0);

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
    // Stage 2 — refine in a local box *centred on the coarse optimum*. Bounds
    // are relative to that optimum so the box always brackets it; the μ floor
    // is a tiny positive (NOT 1.0, which could invert the box for a soft fit),
    // and the C₂ span scales with the found C₂ so it isn't starved when C₂ is
    // large relative to μ.
    let mu_span = 0.1 * mu_guess;
    let c2_span = (0.1 * mu_guess).max(0.25 * coarse.2);
    let fine = grid(
        (coarse.1 - mu_span).max(1.0e-9),
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
        // Nominal (engineering) ≈ Cauchy (true) at small strain since J ≈ 1.
        assert!((r.nominal_stress - r.cauchy_stress).abs() / r.cauchy_stress < 1.0e-3);
        // Poisson check: λ_t ≈ 1 − ν·ε, ν = 0.40.
        let nu = lam_l / (2.0 * (lam_l + mu));
        assert!((r.transverse_stretch - (1.0 - nu * eps)).abs() < 1.0e-6);
    }

    /// Defining condition for the other modes: at the solved `λ_t`, the
    /// thickness traction `P₃₃` vanishes across the sweep (planar holds the
    /// width at 1; equibiaxial drives two axes by λ).
    #[test]
    fn free_transverse_zeroes_thickness_traction_planar_equibiaxial() {
        let mat = ECOFLEX_00_30.to_yeoh();
        for mode in [DeformationMode::Planar, DeformationMode::Equibiaxial] {
            for &lam in &[1.1, 1.5, 2.0, 3.0] {
                let r = free_transverse(&mat, mode, lam);
                let driven2 = match mode {
                    DeformationMode::Equibiaxial => lam,
                    _ => 1.0,
                };
                let f = Matrix3::new(
                    lam,
                    0.0,
                    0.0,
                    0.0,
                    driven2,
                    0.0,
                    0.0,
                    0.0,
                    r.transverse_stretch,
                );
                let p33 = mat.first_piola(&f)[(2, 2)];
                assert!(
                    p33.abs() < 1.0,
                    "thickness traction not zeroed for {mode:?} at λ={lam}: P33={p33}"
                );
            }
        }
    }

    /// Physical cross-mode structure: at a fixed stretch the axial Cauchy stress
    /// orders `equibiaxial > planar > uniaxial` (stiffer modes), and planar's
    /// held width (`λ₂ = 1`) carries a non-zero reaction while its thickness is
    /// traction-free.
    #[test]
    fn mode_stress_ordering_and_planar_width_reaction() {
        let mat = ECOFLEX_00_30.to_yeoh();
        let lam = 1.8;
        let uniaxial_stress = free_transverse(&mat, DeformationMode::Uniaxial, lam).cauchy_stress;
        let planar_stress = free_transverse(&mat, DeformationMode::Planar, lam).cauchy_stress;
        let equibiaxial_stress =
            free_transverse(&mat, DeformationMode::Equibiaxial, lam).cauchy_stress;
        assert!(
            equibiaxial_stress > planar_stress && planar_stress > uniaxial_stress,
            "expected ET>PT>UT axial stress; got UT={uniaxial_stress} PT={planar_stress} ET={equibiaxial_stress}"
        );
        let planar = free_transverse(&mat, DeformationMode::Planar, lam);
        let fgrad = Matrix3::new(
            lam,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            planar.transverse_stretch,
        );
        let pk = mat.first_piola(&fgrad);
        assert!(
            pk[(1, 1)].abs() > 1.0,
            "planar held-width reaction should be non-zero"
        );
        assert!(
            pk[(2, 2)].abs() < 1.0,
            "planar thickness should be traction-free"
        );
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

#[cfg(test)]
mod edge_case_tests {
    use super::*;

    #[test]
    fn fit_with_very_small_stretch() {
        // Synthetic curve starting very close to λ = 1
        let curve = vec![
            (1.001, -100.0),
            (1.01, 500.0),
            (1.1, 2000.0),
            (1.5, 5000.0),
            (2.0, 8000.0),
        ];

        // Window that includes the problematic first point
        let window = (1.0, 2.0);

        // This will call fit with the bad window
        let fit = fit_yeoh_uniaxial(&curve, window);
        eprintln!(
            "Fit with window (1.0, 2.0) succeeded: μ={}, C₂={}",
            fit.mu(),
            fit.c2()
        );
    }
}
