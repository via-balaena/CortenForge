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

fn knee_err_mm(spec: &LegSpec, n_rings: usize) -> f64 {
    let (mesh, _) = spec.build(n_rings, 64);
    let lm = detect_landmarks(&mesh)
        .unwrap_or_else(|e| panic!("detect failed at n_rings={n_rings}: {e:?}"));
    ((lm.knee_z - spec.ground_truth().knee_z) * 1000.0).abs()
}

/// Round resolutions a caller might plausibly pick — including the historical
/// failure (240) — must detect the knee FAR inside the 10 mm gate (they land
/// ~0.8 mm). A regression (e.g. the aliasing lock returning) would blow past this.
#[test]
fn knee_detection_tight_at_common_resolutions() {
    let spec = LegSpec::default_leg();
    for &n_rings in &[120usize, 160, 200, 220, 240, 250, 256, 300, 360, 480] {
        let err = knee_err_mm(&spec, n_rings);
        assert!(
            err < 3.0,
            "n_rings={n_rings}: knee off by {err:.2} mm (aliasing lock regressed?)"
        );
    }
}

/// The residual lock — `n_rings == N_SAMPLES` (401) — is the fix's worst case
/// (~7 mm). It must still clear the detector's actual 10 mm gate: the fix relies
/// on the coincidence being implausible *and* non-catastrophic when it does hit,
/// not merely on avoiding it. (Pre-fix, the equal-grid lock was ~27 mm — a fail.)
#[test]
fn knee_detection_in_gate_at_residual_lock() {
    let spec = LegSpec::default_leg();
    let err = knee_err_mm(&spec, 401);
    assert!(
        err < 10.0,
        "residual lock n_rings=401: knee off by {err:.2} mm"
    );
}
