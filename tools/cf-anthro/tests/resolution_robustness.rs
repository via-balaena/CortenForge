//! Regression guard for the flat-minimum / mesh-aliasing fragility.
//!
//! The knee is a flat area minimum, so a bare argmin is walked by periodic ripple
//! in the profile. A regular mesh whose ring count equals the detector's
//! `N_SAMPLES` once phase-locked the two grids and shifted the detected knee ~27 mm
//! (found while building `cf-msk-fit::ScanSource`). The fix is a robust two-stage
//! minimum-finder (`robust_min_z`: locate the broad bowl on a heavy low-pass, then
//! refine locally). This test pins detection accurate across a resolution sweep —
//! including the historical failure / lock value (`n_rings == N_SAMPLES == 240`).

use cf_anthro::detect_landmarks;
use cf_anthro::synthetic::LegSpec;

#[test]
fn knee_detection_robust_across_resolutions() {
    let spec = LegSpec::default_leg();
    let gt = spec.ground_truth();

    // Round resolutions a caller might pick, plus 240 — which equals N_SAMPLES and
    // was the pre-fix ~27 mm failure. With the robust finder every resolution lands
    // ~1.4 mm; assert a tight 3 mm bound so a regression (the lock returning, or the
    // finder biasing) is caught long before the detector's 10 mm gate.
    for &n_rings in &[
        120usize, 160, 200, 220, 238, 240, 242, 250, 256, 300, 360, 480,
    ] {
        let (mesh, _) = spec.build(n_rings, 64);
        let lm = detect_landmarks(&mesh)
            .unwrap_or_else(|e| panic!("detect failed at n_rings={n_rings}: {e:?}"));

        let knee_err_mm = ((lm.knee_z - gt.knee_z) * 1000.0).abs();
        assert!(
            knee_err_mm < 3.0,
            "n_rings={n_rings}: knee off by {knee_err_mm:.2} mm (aliasing/flat-min regressed?)"
        );
    }
}
