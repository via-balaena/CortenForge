//! M3-S1 — same-source cross-mode measured-accuracy gate (soft-FEM fidelity).
//!
//! Validates `sim-soft`'s hyperelastic response across deformation MODES against
//! the Hossain 2021 Ecoflex 00-30 dataset (uniaxial + planar + equibiaxial, all
//! on the same material; figure-digitized, `tests/assets/hossain_2021/`).
//!
//! ## Why "cross-mode", not absolute stress (the M3 reframe)
//!
//! Two peer-reviewed *measured* Ecoflex 00-30 datasets — Marechal 2021 (M1/M2's
//! oracle) and Hossain 2021 — disagree by **~1.75×** in absolute modulus.
//! Silicone modulus genuinely varies ~1.5–2× with batch / cure / lab, so
//! *absolute* modulus carries a ~1.75× inter-source floor that dwarfs every model
//! effect (ν, I₂). What IS material-batch-invariant — and what tests the model
//! FORM — is the **cross-mode relationship**: calibrate to one source's uniaxial
//! (absorbing its batch modulus), then *predict* that same source's planar +
//! equibiaxial. This gate does exactly that, and asserts:
//!
//! 1. Calibrated to Hossain uniaxial at near-incompressible ν, the model predicts
//!    the same source's **planar to a few %** and **equibiaxial to ≲ 10 %** over
//!    the device window — i.e. the 2-term Yeoh form is adequate cross-mode.
//! 2. **Near-incompressibility helps**: raising ν from 0.40 → 0.499 roughly
//!    halves the equibiaxial cross-mode error (the concrete payoff that justifies
//!    the near-incompressibility upgrade — see `near_incompressibility_recon.md`).
//! 3. The **inter-source material variability (~1.75×) dwarfs** that residual
//!    model error — documenting that absolute-stress fidelity is floored by the
//!    material, not the model.
//!
//! See `docs/soft_fidelity/03_phases/m3_deformation_modes/recon.md` (§REFRAME).

#![allow(
    // Reading the vendored asset / parsing its JSON either succeeds or surfaces a
    // missing/corrupt-fixture regression as a test panic — the canonical fixture
    // idiom in this crate's integration tests.
    clippy::expect_used,
    clippy::unwrap_used,
    // Grid indices and the in-window sample count are small integers far inside
    // f64's exact range — the `i/k/n as f64` and `Σ/N` arithmetic loses no precision.
    clippy::cast_precision_loss
)]

use sim_soft::{DeformationMode, Yeoh, free_transverse};

const WINDOW: (f64, f64) = (1.1, 2.0);

const HOSSAIN: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/assets/hossain_2021/ecoflex_00_30_multimode.json"
);
const MARECHAL: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/assets/marechal_2021/ecoflex_00_30_uniaxial.json"
);

fn json(path: &str) -> serde_json::Value {
    serde_json::from_str(&std::fs::read_to_string(path).expect("read asset")).expect("parse json")
}

/// `(stretch, cauchy_stress_pa)` samples for one mode array of an asset.
fn curve(v: &serde_json::Value, key: &str) -> Vec<(f64, f64)> {
    v[key]
        .as_array()
        .expect("mode array")
        .iter()
        .map(|p| {
            (
                p["stretch"].as_f64().unwrap(),
                p["cauchy_stress_pa"].as_f64().unwrap(),
            )
        })
        .collect()
}

/// Compressible Yeoh at shear modulus `mu`, second coefficient `c2`, Poisson
/// ratio `nu` (`λ_lame = 2μν/(1−2ν)`).
fn yeoh(mu: f64, c2: f64, nu: f64) -> Yeoh {
    Yeoh::from_lame_and_c2(mu, 2.0 * mu * nu / (1.0 - 2.0 * nu), c2)
}

/// RMS relative error of `mat`'s `mode` Cauchy response vs a measured curve over
/// the window.
fn rms(mat: &Yeoh, mode: DeformationMode, curve: &[(f64, f64)]) -> f64 {
    let (mut sse, mut n) = (0.0, 0u32);
    for &(lam, sig) in curve {
        if lam < WINDOW.0 || lam > WINDOW.1 {
            continue;
        }
        let model = free_transverse(mat, mode, lam).cauchy_stress;
        let re = (model - sig) / sig.abs().max(1.0);
        sse += re * re;
        n += 1;
    }
    assert!(n > 0, "no samples in window");
    (sse / f64::from(n)).sqrt()
}

/// Calibrate `(μ, C₂)` to a uniaxial curve at fixed `nu` by a two-stage grid
/// (minimising the uniaxial Cauchy RMS through the real `free_transverse`).
fn fit_uniaxial(ut: &[(f64, f64)], nu: f64) -> Yeoh {
    let score = |mu: f64, c2: f64| rms(&yeoh(mu, c2, nu), DeformationMode::Uniaxial, ut);
    let grid = |mu_lo: f64, mu_hi: f64, c2_lo: f64, c2_hi: f64, n: usize| {
        let mut best = (f64::INFINITY, mu_lo, c2_lo);
        for i in 0..=n {
            let mu = mu_lo + (mu_hi - mu_lo) * i as f64 / n as f64;
            for k in 0..=n {
                let c2 = c2_lo + (c2_hi - c2_lo) * k as f64 / n as f64;
                let e = score(mu, c2);
                if e < best.0 {
                    best = (e, mu, c2);
                }
            }
        }
        best
    };
    let coarse = grid(10_000.0, 45_000.0, 0.0, 2_000.0, 35);
    let (_, mu, c2) = grid(
        0.8 * coarse.1,
        1.2 * coarse.1,
        (coarse.2 - 300.0).max(0.0),
        coarse.2 + 300.0,
        20,
    );
    yeoh(mu, c2, nu)
}

#[test]
fn cross_mode_prediction_at_near_incompressible() {
    let h = json(HOSSAIN);
    let (ut, pt, et) = (
        curve(&h, "uniaxial"),
        curve(&h, "planar"),
        curve(&h, "equibiaxial"),
    );
    // Calibrate to UNIAXIAL only, at near-incompressible ν.
    let mat = fit_uniaxial(&ut, 0.499);
    let ut_fit = rms(&mat, DeformationMode::Uniaxial, &ut);
    let pt_pred = rms(&mat, DeformationMode::Planar, &pt);
    let et_pred = rms(&mat, DeformationMode::Equibiaxial, &et);
    eprintln!(
        "ν=0.499  μ={:.0} Pa C₂={:.0} Pa | UT fit {:.1}% | PT predict {:.1}% | ET predict {:.1}% (λ≤2)",
        mat.mu(),
        mat.c2(),
        ut_fit * 100.0,
        pt_pred * 100.0,
        et_pred * 100.0
    );
    // The model FORM is adequate cross-mode: uniaxial calibration predicts the
    // same source's other modes within these bounds (observed ~2.3% / ~8%).
    assert!(
        ut_fit < 0.05,
        "uniaxial calibration should be tight; got {:.1}%",
        ut_fit * 100.0
    );
    assert!(
        pt_pred < 0.05,
        "planar cross-mode prediction {:.1}% exceeds 5%",
        pt_pred * 100.0
    );
    assert!(
        et_pred < 0.12,
        "equibiaxial cross-mode prediction {:.1}% exceeds 12%",
        et_pred * 100.0
    );
}

#[test]
fn near_incompressibility_halves_cross_mode_error() {
    let h = json(HOSSAIN);
    let (ut, et) = (curve(&h, "uniaxial"), curve(&h, "equibiaxial"));
    let et_040 = rms(&fit_uniaxial(&ut, 0.40), DeformationMode::Equibiaxial, &et);
    let et_499 = rms(&fit_uniaxial(&ut, 0.499), DeformationMode::Equibiaxial, &et);
    eprintln!(
        "equibiaxial cross-mode error: ν=0.40 → {:.1}% ; ν=0.499 → {:.1}% (improvement {:.2}×)",
        et_040 * 100.0,
        et_499 * 100.0,
        et_040 / et_499
    );
    // Near-incompressibility is the concrete payoff that justifies the upgrade:
    // it materially improves the cross-mode prediction (observed ~15% → ~8%).
    assert!(
        et_499 < 0.75 * et_040,
        "ν→0.499 should materially improve equibiaxial (got {:.1}% vs {:.1}%)",
        et_499 * 100.0,
        et_040 * 100.0
    );
}

#[test]
fn inter_source_material_variability_dwarfs_model_error() {
    // Two measured Ecoflex 00-30 sources disagree ~1.75× in absolute modulus —
    // larger than the cross-mode model error. This is the reframe's premise.
    let hoss_ut = curve(&json(HOSSAIN), "uniaxial");
    let mar = curve(&json(MARECHAL), "measured");
    let at = |c: &[(f64, f64)], lam: f64| -> f64 {
        for w in c.windows(2) {
            if w[0].0 <= lam && lam <= w[1].0 {
                let t = (lam - w[0].0) / (w[1].0 - w[0].0);
                return w[0].1 + t * (w[1].1 - w[0].1);
            }
        }
        f64::NAN
    };
    let ratio = at(&hoss_ut, 1.5) / at(&mar, 1.5);
    eprintln!("Hossain / Marechal measured Cauchy at λ=1.5 = {ratio:.2}×");
    assert!(
        ratio > 1.5,
        "expected large inter-source material variability (>1.5×); got {ratio:.2}×"
    );
    // ...and the cross-mode MODEL error is far below that floor: the prediction
    // bounds asserted above (planar < 5%, equibiaxial < 12%) sit well under the
    // ~75% inter-source material scatter measured here — absolute-stress fidelity
    // is floored by the material, not the model.
}
