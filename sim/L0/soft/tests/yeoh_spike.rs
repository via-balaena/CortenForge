//! Yeoh contract — math + calibration cross-checks.
//!
//! Originally written as the pre-F1 "Spike 1" file (single-tet analytic
//! validation, throwaway). Since lifted to call the production
//! [`sim_soft::Yeoh`] struct after F1 landed; renames to
//! `yeoh_contract.rs` in F3. The closed forms are documented in
//! `material/yeoh.rs` and the Yeoh arc memo §"Math derivations":
//!
//! - `ψ = C₁(I₁ − 3) + C₂(I₁ − 3)² − μ ln J + (λ/2)(ln J)²`
//! - `P = μ(F − F⁻ᵀ) + λ (ln J) F⁻ᵀ + 4 C₂ (I₁ − 3) F`
//! - `C_ijkl = C_NH_ijkl + 4 C₂ (I₁ − 3) δ_ik δ_jl + 8 C₂ F_ij F_kl`
//!
//! Cross-checks:
//!
//! 1. **NH bit-exact at C₂=0** — every Yeoh output equals NH on a
//!    battery of `F`. Validates the locked D2 contract; the production
//!    impl uses the additive-decomposition + FMA pattern that Spike 1
//!    surfaced as load-bearing for bit-exactness.
//! 2. **Hand-derived scalar uniaxial closed-form** — for `F = diag(s,1,1)`,
//!    matrix-form `ψ`, `P_11`, `P_22` match the scalar closed-form
//!    derived by hand-substituting the diagonal `F` into the matrix
//!    formulas. Catches transcription bugs.
//! 3. **FD ψ → P** — central FD of `energy` matches `first_piola` at
//!    the nontrivial off-diagonal-rich `F` from `material_fd.rs`.
//! 4. **FD P → tangent** — central FD of `first_piola` matches
//!    `tangent` at the same point.
//! 5. **Per-anchor calibration arithmetic** — `M_100 = 3.5·C₁ + 14·C₂`
//!    holds for every anchor in the Yeoh arc memo's calibration table.
//!    Catches typos in the F2 `silicone_table` when it lands.

use approx::assert_relative_eq;
use nalgebra::{Matrix3, Vector3};

use sim_soft::{Material, NeoHookean, Yeoh};

// ECOFLEX_00_30 calibration per Yeoh arc memo line 81.
const MU: f64 = 23_000.0; // Pa
const LAMBDA: f64 = 92_000.0; // Pa (ν=0.40 convention: λ = 4μ)
const C2: f64 = 2_050.0; // Pa (Yeoh C₂)

// FD step + tolerances match `material_fd.rs` precedent (square-root of
// f64 eps, 5-digit relative bar with absolute slack covering FD noise at
// μ ≈ 10⁵).
const H: f64 = 1.5e-8;
const MAX_RELATIVE: f64 = 1.0e-5;
const EPSILON: f64 = 1.0e-3;

// Hand-derived scalar closed-forms for F = diag(s, 1, 1):
//
//   I₁ = s² + 2,  J = s,  ln J = ln s,  I₁ − 3 = s² − 1,  F⁻ᵀ = diag(1/s, 1, 1)
//
// substituted into the matrix formulas. Plain ops (no `mul_add`) so the
// test code mirrors the derivation literally — the rel-tol used in
// suite (b) is 1e-12, which absorbs any FMA/non-FMA ULP drift.
#[allow(clippy::suboptimal_flops)]
fn analytic_uniaxial_energy(s: f64, mu: f64, lambda: f64, c2: f64) -> f64 {
    let c1 = 0.5 * mu;
    let i1m3 = s * s - 1.0;
    let ln_s = s.ln();
    c1 * i1m3 + c2 * i1m3 * i1m3 - mu * ln_s + 0.5 * lambda * ln_s * ln_s
}

#[allow(clippy::suboptimal_flops)]
fn analytic_uniaxial_p11(s: f64, mu: f64, lambda: f64, c2: f64) -> f64 {
    let c1 = 0.5 * mu;
    let i1m3 = s * s - 1.0;
    let ln_s = s.ln();
    2.0 * s * (c1 + 2.0 * c2 * i1m3) + (lambda * ln_s - mu) / s
}

#[allow(clippy::suboptimal_flops)]
fn analytic_uniaxial_p22(s: f64, mu: f64, lambda: f64, c2: f64) -> f64 {
    let c1 = 0.5 * mu;
    let i1m3 = s * s - 1.0;
    let ln_s = s.ln();
    2.0 * (c1 + 2.0 * c2 * i1m3) + (lambda * ln_s - mu)
}

#[allow(clippy::missing_const_for_fn)]
fn nontrivial_f() -> Matrix3<f64> {
    // Same pose as `material_fd.rs::nontrivial_f` so the spike reuses the
    // off-diagonal-rich F that exercises every term of the closed-form
    // tangent. det F ≈ 0.987 > 0.
    Matrix3::new(
        1.10, 0.05, 0.02, //
        0.04, 0.90, 0.01, //
        0.03, 0.02, 1.00, //
    )
}

fn diag(s1: f64, s2: f64, s3: f64) -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(s1, s2, s3))
}

// ---- Suite (a): NH bit-exact at C₂=0 -----------------------------------

#[test]
fn yeoh_at_c2_zero_matches_neo_hookean_bit_exactly_on_battery() {
    let nh = NeoHookean::from_lame(MU, LAMBDA);
    let yeoh = Yeoh::from_lame_and_c2(MU, LAMBDA, 0.0);

    let battery = [
        Matrix3::<f64>::identity(),
        diag(1.01, 1.0, 1.0),
        diag(1.5, 1.0, 1.0),
        diag(1.0, 1.0, 1.0 / 1.05),
        nontrivial_f(),
    ];

    for f in &battery {
        let nh_e = nh.energy(f);
        let y_e = yeoh.energy(f);
        assert_eq!(
            nh_e.to_bits(),
            y_e.to_bits(),
            "energy mismatch at C₂=0: NH={nh_e}, Yeoh={y_e}, F={f:?}"
        );

        let nh_p = nh.first_piola(f);
        let y_p = yeoh.first_piola(f);
        for i in 0..3 {
            for j in 0..3 {
                assert_eq!(
                    nh_p[(i, j)].to_bits(),
                    y_p[(i, j)].to_bits(),
                    "first_piola[{i},{j}] mismatch at C₂=0"
                );
            }
        }

        let nh_t = nh.tangent(f);
        let y_t = yeoh.tangent(f);
        for r in 0..9 {
            for c in 0..9 {
                assert_eq!(
                    nh_t[(r, c)].to_bits(),
                    y_t[(r, c)].to_bits(),
                    "tangent[{r},{c}] mismatch at C₂=0"
                );
            }
        }
    }
}

// ---- Suite (b): hand-derived scalar uniaxial closed-form ---------------

#[test]
fn yeoh_uniaxial_matrix_form_matches_scalar_closed_form() {
    let yeoh = Yeoh::from_lame_and_c2(MU, LAMBDA, C2);

    // Stretches span tensile + compressive within Yeoh validity for
    // ECOFLEX_00_30 (max_principal_stretch = 8.0, min = 0.3).
    for s in [0.5, 0.8, 1.0, 1.2, 1.5, 2.0] {
        let f = diag(s, 1.0, 1.0);

        let e_matrix = yeoh.energy(&f);
        let e_scalar = analytic_uniaxial_energy(s, MU, LAMBDA, C2);
        assert_relative_eq!(e_matrix, e_scalar, max_relative = 1e-12, epsilon = 1e-9);

        let p_matrix = yeoh.first_piola(&f);
        let p11 = analytic_uniaxial_p11(s, MU, LAMBDA, C2);
        let p22 = analytic_uniaxial_p22(s, MU, LAMBDA, C2);
        assert_relative_eq!(p_matrix[(0, 0)], p11, max_relative = 1e-12, epsilon = 1e-9);
        assert_relative_eq!(p_matrix[(1, 1)], p22, max_relative = 1e-12, epsilon = 1e-9);
        assert_relative_eq!(p_matrix[(2, 2)], p22, max_relative = 1e-12, epsilon = 1e-9);

        for i in 0..3 {
            for j in 0..3 {
                if i != j {
                    assert!(
                        p_matrix[(i, j)].abs() < 1e-9,
                        "off-diagonal P[{i},{j}]={} at s={s}",
                        p_matrix[(i, j)]
                    );
                }
            }
        }
    }
}

// ---- Suite (c): FD ψ → P at nontrivial F -------------------------------

#[test]
fn yeoh_first_piola_matches_central_fd_of_energy() {
    let yeoh = Yeoh::from_lame_and_c2(MU, LAMBDA, C2);
    let f0 = nontrivial_f();
    let p = yeoh.first_piola(&f0);

    for i in 0..3 {
        for j in 0..3 {
            let mut f_pos = f0;
            f_pos[(i, j)] += H;
            let mut f_neg = f0;
            f_neg[(i, j)] -= H;
            let fd = (yeoh.energy(&f_pos) - yeoh.energy(&f_neg)) / (2.0 * H);
            assert_relative_eq!(
                fd,
                p[(i, j)],
                max_relative = MAX_RELATIVE,
                epsilon = EPSILON
            );
        }
    }
}

// ---- Suite (d): FD P → tangent at nontrivial F -------------------------

#[test]
fn yeoh_tangent_matches_central_fd_of_first_piola() {
    let yeoh = Yeoh::from_lame_and_c2(MU, LAMBDA, C2);
    let f0 = nontrivial_f();
    let tangent = yeoh.tangent(&f0);

    for k in 0..3 {
        for l in 0..3 {
            let mut f_pos = f0;
            f_pos[(k, l)] += H;
            let mut f_neg = f0;
            f_neg[(k, l)] -= H;
            let p_pos = yeoh.first_piola(&f_pos);
            let p_neg = yeoh.first_piola(&f_neg);
            let dp_dfkl = (p_pos - p_neg) / (2.0 * H);
            for i in 0..3 {
                for j in 0..3 {
                    let row = i + 3 * j;
                    let col = k + 3 * l;
                    assert_relative_eq!(
                        dp_dfkl[(i, j)],
                        tangent[(row, col)],
                        max_relative = MAX_RELATIVE,
                        epsilon = EPSILON
                    );
                }
            }
        }
    }
}

// ---- Suite (e): per-anchor calibration arithmetic ---------------------
//
// Validates that each anchor's `(μ, c2)` reproduces the published M_100
// via the calibration formula M_100 = 3.5·C₁ + 14·C₂ (Yeoh arc memo
// line 72, derived from incompressible-uniaxial Yeoh at λ=2). Catches
// transcription typos in the F2 anchor table when it lands.
//
// Note: this is a calibration-arithmetic check, not a compressible-Yeoh-
// struct-vs-published-modulus check. The arc memo's calibration assumes
// incompressible-uniaxial kinematics F=diag(λ, 1/√λ, 1/√λ); compressible
// Yeoh evaluated at the same F with λ_lame=4μ (ν=0.40) gives P_11 that
// differs from the incompressible σ_eng by O(2-3 %) due to the
// non-vanishing lateral stress. That compressibility error is the cost
// of ν=0.40 vs the silicone's true ν≈0.495, accepted per the memo's
// Fork-B "relative comparison, not absolute prediction" framing.

#[test]
fn yeoh_calibration_table_reproduces_published_100_pct_modulus() {
    // Anchor calibration values from project_yeoh_hyperelastic_arc.md
    // lines 76-85. (name, μ_Pa, c2_Pa, M_100_psi)
    const ANCHORS: &[(&str, f64, f64, f64)] = &[
        ("ECOFLEX_00_10", 18_000.0, 1_690.0, 8.0),
        ("ECOFLEX_00_20", 18_000.0, 1_690.0, 8.0),
        ("ECOFLEX_00_30", 23_000.0, 2_050.0, 10.0),
        ("ECOFLEX_00_50", 28_000.0, 2_410.0, 12.0),
        ("DRAGON_SKIN_10A", 51_000.0, 4_460.0, 22.0),
        ("DRAGON_SKIN_15", 92_000.0, 8_200.0, 40.0),
        ("DRAGON_SKIN_20A", 113_000.0, 10_000.0, 49.0),
        ("DRAGON_SKIN_30A", 198_000.0, 17_600.0, 86.0),
    ];

    // psi → Pa (NIST: 1 psi = 6894.757293168361 Pa).
    const PSI_TO_PA: f64 = 6_894.757_293_168_361;

    // Tolerance covers c2 rounding to nearest 10 Pa in the memo's table.
    // Worst case observed: DS_20A at rel_err ≈ 2.76e-4 (c2 calibrated
    // value 10006.65 Pa, rounded to 10000).
    const REL_TOL: f64 = 1e-3;

    for (name, mu, c2, m100_psi) in ANCHORS {
        let c1 = 0.5 * mu;
        // M_100 = 3.5·C₁ + 14·C₂  (incompressible uniaxial Yeoh at λ=2).
        let predicted_m100 = 3.5_f64.mul_add(c1, 14.0 * c2);
        let published_m100 = m100_psi * PSI_TO_PA;
        let rel_err = (predicted_m100 - published_m100).abs() / published_m100;
        assert!(
            rel_err < REL_TOL,
            "{name}: predicted M_100={predicted_m100} Pa, published={published_m100} Pa, rel_err={rel_err}"
        );
    }
}
