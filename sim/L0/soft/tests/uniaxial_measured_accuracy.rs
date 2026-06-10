//! M1 uniaxial measured-accuracy gate (soft-FEM fidelity, Mission Layer-1).
//!
//! Grades `sim-soft`'s compressible 2-parameter `Yeoh` uniaxial response
//! against Marechal et al. 2021 MEASURED Ecoflex 00-30 true-stress–stretch
//! data (vendored under `tests/assets/marechal_2021/`; ASTM D412, true
//! stress). Two claims, both asserted:
//!
//! 1. The **datasheet-derived** params this arc replaces (a single
//!    Smooth-On TDS 100 %-modulus point; vendored as a snapshot in the
//!    asset) over-predict the measured curve grossly — the gap this arc
//!    exists to close. The TDS 100 %-modulus is ~2.3× the measured value.
//! 2. A measured-curve fit of the *same* model reaches measured accuracy
//!    (≤ 10 % RMS) over the device window λ ≤ 2.0 — closing the gap is a
//!    parameter problem, not a solver problem (the ν = 0.40 compressible
//!    form is adequate here; near-incompressibility is not required).
//!
//! The measured fit is published as the Path-3
//! [`ECOFLEX_00_30_MEASURED`](sim_soft::material::silicone_table::ECOFLEX_00_30_MEASURED)
//! const and graded by `published_measured_table_entry_matches_measurement`
//! below. The datasheet `ECOFLEX_00_30` anchor is intentionally left
//! unchanged (it anchors the Shore-interpolation family) — the gap-gate
//! keys off the vendored datasheet *snapshot* and the un-asserted
//! live-table readout shows it still at ~85 %.
//!
//! See `docs/soft_fidelity/03_phases/m1_silicone_uniaxial/recon.md`.

#![allow(
    // Reading the vendored reference asset either succeeds or surfaces a
    // missing/corrupt-asset regression as a test panic — the canonical
    // fixture-load idiom in this crate's integration tests.
    clippy::expect_used,
    clippy::unwrap_used,
    // `n` is the in-window sample count (≤ a few hundred), far inside
    // f64's exact-integer range — the RMS `Σ/N` divide loses no precision.
    clippy::cast_precision_loss
)]

use sim_soft::material::silicone_table::{ECOFLEX_00_30, ECOFLEX_00_30_MEASURED};
use sim_soft::{ConstructionSource, Yeoh, fit_yeoh_uniaxial, free_transverse_uniaxial};

// Device window λ ≤ 2; toe (λ<1.1) excluded — σ there is ~1–4 kPa, low signal
// that inflates relative error. S0 found ~6% achievable, so the gate sits at
// 10% (≈4 pp margin). See recon.md §"Progress · S0".
const WINDOW: (f64, f64) = (1.1, 2.0);
const GATE_RMS: f64 = 0.10;

const ASSET: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/assets/marechal_2021/ecoflex_00_30_uniaxial.json"
);

fn load_asset() -> serde_json::Value {
    let text = std::fs::read_to_string(ASSET).expect("read vendored Marechal asset");
    serde_json::from_str(&text).expect("parse asset JSON")
}

/// The vendored measured curve as `(stretch, cauchy_stress_pa)`.
fn measured_curve() -> Vec<(f64, f64)> {
    load_asset()["measured"]
        .as_array()
        .expect("measured array")
        .iter()
        .map(|p| {
            (
                p["stretch"].as_f64().unwrap(),
                p["cauchy_stress_pa"].as_f64().unwrap(),
            )
        })
        .collect()
}

/// The datasheet-derived params this arc replaces, read from the asset
/// snapshot — stable across the S3 `silicone_table` re-fit (unlike the
/// live `ECOFLEX_00_30` table).
fn datasheet_baseline() -> Yeoh {
    let p = load_asset()["current_silicone_table_params"].clone();
    Yeoh::from_lame_and_c2(
        p["mu_pa"].as_f64().unwrap(),
        p["lambda_pa"].as_f64().unwrap(),
        p["c2_pa"].as_f64().unwrap(),
    )
}

/// RMS relative error of `mat`'s uniaxial Cauchy stress vs the measured
/// curve over stretch `[lo, hi]`.
fn rms_rel_err(mat: &Yeoh, curve: &[(f64, f64)], lo: f64, hi: f64) -> (f64, usize) {
    let mut sse = 0.0;
    let mut n = 0;
    for &(lam, sig) in curve {
        if lam < lo || lam > hi {
            continue;
        }
        let model = free_transverse_uniaxial(mat, lam).cauchy_stress;
        // Floor the denominator at 1 Pa so relative error stays well-defined if
        // the window ever dips into near-zero toe stress; inert for λ≥1.1 here
        // (σ ∈ ~[4, 59] kPa).
        let re = (model - sig) / sig.abs().max(1.0);
        sse += re * re;
        n += 1;
    }
    assert!(n > 0, "no measured points in [{lo}, {hi}]");
    ((sse / n as f64).sqrt(), n)
}

#[test]
fn datasheet_params_grossly_overpredict() {
    let curve = measured_curve();
    let (rms, n) = rms_rel_err(&datasheet_baseline(), &curve, WINDOW.0, WINDOW.1);
    let (live, _) = rms_rel_err(&ECOFLEX_00_30.to_yeoh(), &curve, WINDOW.0, WINDOW.1);
    eprintln!(
        "datasheet snapshot: RMS {:.0}% | live silicone_table: RMS {:.0}% over λ≤2 (n={n})",
        rms * 100.0,
        live * 100.0
    );
    // The documented gap (S0 measured ~85%): assert the datasheet method is
    // unambiguously gross. Keyed off the snapshot so the S3 table re-fit
    // does not silently erase this evidence.
    assert!(
        rms > 0.5,
        "expected a gross datasheet gap (>50% RMS); got {:.1}%",
        rms * 100.0
    );
}

#[test]
fn measured_fit_reaches_measured_accuracy() {
    let curve = measured_curve();
    let fit = fit_yeoh_uniaxial(&curve, WINDOW);
    let (rms, n) = rms_rel_err(&fit, &curve, WINDOW.0, WINDOW.1);
    eprintln!(
        "measured-fit Yeoh: μ={:.0} Pa, C₂={:.0} Pa → RMS rel err {:.1}% over λ≤2 (n={n})",
        fit.mu(),
        fit.c2(),
        rms * 100.0
    );
    assert!(
        rms <= GATE_RMS,
        "measured-fit RMS {:.1}% exceeds {:.0}% gate",
        rms * 100.0,
        GATE_RMS * 100.0
    );
    // The fit must materially soften μ relative to the datasheet value
    // (datasheet μ ~1.4× the fit; its σ₁₀₀ input is ~2.3× measured): a
    // sanity check that it is a real measured fit, not datasheet in disguise.
    assert!(
        fit.mu() < 0.85 * datasheet_baseline().mu(),
        "fit μ should drop well below the datasheet μ"
    );
}

#[test]
fn published_measured_table_entry_matches_measurement() {
    // The Path-3 ECOFLEX_00_30_MEASURED const must reproduce the measured
    // curve within the M1 gate, and carry Measured provenance.
    let curve = measured_curve();
    let (rms, n) = rms_rel_err(
        &ECOFLEX_00_30_MEASURED.to_yeoh(),
        &curve,
        WINDOW.0,
        WINDOW.1,
    );
    eprintln!(
        "ECOFLEX_00_30_MEASURED: RMS rel err {:.1}% over λ≤2 (n={n})",
        rms * 100.0
    );
    assert!(
        rms <= GATE_RMS,
        "measured table entry RMS {:.1}% exceeds {:.0}% gate",
        rms * 100.0,
        GATE_RMS * 100.0
    );
    assert!(
        matches!(
            ECOFLEX_00_30_MEASURED.source,
            ConstructionSource::Measured { .. }
        ),
        "the measured entry must carry Measured provenance"
    );
    // ...and it must be materially softer than the datasheet anchor
    // (compile-time invariant — both are consts).
    const { assert!(ECOFLEX_00_30_MEASURED.mu < 0.85 * ECOFLEX_00_30.mu) };
}
