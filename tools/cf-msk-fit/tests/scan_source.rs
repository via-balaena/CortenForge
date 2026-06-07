//! `ScanSource` — the scan as a `ParamSource`, validated on the synthetic fixture.
//!
//! No real scan is needed (none exists yet, and sub-$1000 scanners are the whole
//! reason for builder-first): `cf-anthro::synthetic` generates a watertight leg
//! with known ground truth, and `detect_landmarks` recovers the landmarks
//! `ScanSource` consumes. These run in CI (not `#[ignore]`).
//!
//! Coverage:
//! * `scan_source_scale_matches_fitter` — the refactor-safety checkpoint: the
//!   scale `ScanSource` puts into `BodyParams` is *exactly* the scale the
//!   validated `Fitter` (`place_knee`) computes. The seam reproduces the prior
//!   behavior.
//! * `scan_source_is_uniform_v1` — v1 is a single uniform scale (all three
//!   per-segment factors equal); pins the documented v1 approximation.
//! * `synthetic_scan_builds_loadable_model` — the full path
//!   `synthetic leg → detect → ScanSource → realize → emit` yields an MJCF that
//!   loads, with the detected thigh size flowing into the model scale.
//!
//! The morph *physics* (uniform scale = exact moment-arm dilation) is proven in
//! `cf-msk-lib`'s `uniform_scale_is_exact_dilation`; here we prove the **seam**
//! (landmarks → the right scale) and that the pipeline wires up end to end.

use cf_anthro::synthetic::LegSpec;
use cf_anthro::{Landmarks, detect_landmarks};
use cf_msk_fit::{Fitter, ScanSource};
use cf_msk_lib::{ParamSource, build};
use cf_osim::osim::{Subgraph, parse_knee_subgraph};
use sim_mjcf::load_model;

fn template() -> Subgraph {
    let path = format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    );
    parse_knee_subgraph(&std::fs::read_to_string(path).expect("read gait2392.osim"))
}

/// Detect landmarks on a clean default synthetic leg — the fixture, no real scan.
/// Uses the same mesh resolution cf-anthro's `synthetic_validation` validates at
/// (detector accuracy is cf-anthro's concern; here it just feeds the seam).
fn synthetic_landmarks() -> (Landmarks, f64) {
    let (mesh, gt) = LegSpec::default_leg().build(220, 96);
    let lm = detect_landmarks(&mesh).expect("detect on clean synthetic leg");
    (lm, gt.thigh_length_m)
}

#[test]
fn scan_source_scale_matches_fitter() {
    let t = template();
    let (lm, _) = synthetic_landmarks();

    // The seam (ScanSource) and the validated placement (Fitter) must compute the
    // SAME scale from the same landmarks — the no-behavior-change checkpoint.
    let via_source = ScanSource::new(lm.clone()).params(&t).femur_scale;
    let via_fitter = Fitter::new(&t, &lm).scale();

    assert!(
        (via_source - via_fitter).abs() < 1e-12,
        "ScanSource scale {via_source} != Fitter scale {via_fitter}"
    );
}

#[test]
fn scan_source_is_uniform_v1() {
    let t = template();
    let (lm, _) = synthetic_landmarks();
    let p = ScanSource::new(lm).params(&t);

    // v1 single-scale: every segment factor equal (shank inherits the femur scale).
    assert_eq!(p.femur_scale, p.tibia_scale);
    assert_eq!(p.femur_scale, p.pelvis_scale);
    assert!(
        p.femur_scale > 0.5 && p.femur_scale < 2.0,
        "scale {} insane",
        p.femur_scale
    );
}

#[test]
fn synthetic_scan_builds_loadable_model() {
    let t = template();
    let (lm, gt_thigh) = synthetic_landmarks();

    // Fixture sanity only — the detector feeds the seam with a believable size.
    // (Detector accuracy itself is cf-anthro's `synthetic_validation` job; its
    // gate is 5%. We assert that loose bound, not a tight one.)
    let thigh_pct = (lm.thigh_length_m - gt_thigh).abs() / gt_thigh * 100.0;
    assert!(
        thigh_pct < 5.0,
        "detected thigh {} vs ground truth {gt_thigh} ({thigh_pct:.1}% > 5%)",
        lm.thigh_length_m
    );

    // Full path: scan → ParamSource → realize → emit.
    let source = ScanSource::new(lm);
    let emitted = build(&t, &source);
    let model = load_model(&emitted.mjcf).expect("scan-morphed MJCF must load");

    assert_eq!(emitted.muscles.len(), 4, "all four muscles emitted");
    assert!(model.ntendon >= 4, "tendons present in the loaded model");
}
