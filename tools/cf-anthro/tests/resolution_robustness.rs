//! Regression guard for the `n_rings == N_SAMPLES` aliasing lock.
//!
//! The knee is a flat area minimum, so the argmin is sensitive to periodic ripple
//! in the profile. A regular mesh whose ring count equals the detector's
//! `N_SAMPLES` phase-locks the two grids and walked the detected knee ~27 mm on a
//! synthetic leg (found while building `cf-msk-fit::ScanSource`). The fix made
//! `N_SAMPLES` a non-round prime (401); this test pins that detection stays in
//! gate across a resolution sweep — including the *old* danger value (240) and a
//! span of round resolutions a caller might plausibly pick.

use cf_anthro::detect_landmarks;
use cf_anthro::synthetic::LegSpec;

#[test]
fn knee_detection_robust_across_resolutions() {
    let spec = LegSpec::default_leg();
    let gt = spec.ground_truth();

    // Round resolutions a caller might pick, plus the historical failure (240).
    for &n_rings in &[120usize, 160, 200, 220, 240, 250, 256, 300, 360, 480] {
        let (mesh, _) = spec.build(n_rings, 64);
        let lm = detect_landmarks(&mesh)
            .unwrap_or_else(|e| panic!("detect failed at n_rings={n_rings}: {e:?}"));

        let knee_err_mm = ((lm.knee_z - gt.knee_z) * 1000.0).abs();
        let thigh_pct = (lm.thigh_length_m - gt.thigh_length_m).abs() / gt.thigh_length_m * 100.0;

        // Comfortably inside the detector's 10 mm / 5% gate at every resolution —
        // a real lock (pre-fix) blew past this by ~3×.
        assert!(
            knee_err_mm < 5.0,
            "n_rings={n_rings}: knee off by {knee_err_mm:.2} mm (aliasing lock regressed?)"
        );
        assert!(
            thigh_pct < 2.0,
            "n_rings={n_rings}: thigh length off by {thigh_pct:.2}%"
        );
    }
}
