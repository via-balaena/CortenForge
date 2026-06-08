//! `ScanSource` — the scan as a `ParamSource`, validated on the synthetic fixture.
//!
//! No real scan is needed (none exists yet, and sub-$1000 scanners are the whole
//! reason for builder-first): `cf-anthro::synthetic` generates a watertight leg
//! with known ground truth, and `detect_landmarks` recovers the landmarks
//! `ScanSource` consumes. These run in CI (not `#[ignore]`).
//!
//! Coverage:
//! * `scan_source_drives_per_segment_lengths` — A3: the femur axial scale is
//!   `thigh_length / template_femur_axial` and the tibia's is `shank_length /
//!   template_tibia_axial` (shank finally drives the tibia, now that the ankle
//!   exists); girth (transverse) stays at the template default.
//! * `scan_source_femur_close_to_fitter` — sanity that the per-segment refactor
//!   didn't move femur sizing far from the validated `Fitter` (axial ≈ euclidean).
//! * `synthetic_scan_builds_loadable_model` — the full path
//!   `synthetic leg → detect → ScanSource → realize → emit` yields an MJCF that
//!   loads, with the detected thigh size flowing into the model scale.
//!
//! The morph *physics* (uniform scale = exact moment-arm dilation; length
//! round-trip) is proven in `cf-msk-lib`; here we prove the **seam** (landmarks →
//! the right per-segment scale) and that the pipeline wires up end to end.

use cf_anthro::synthetic::LegSpec;
use cf_anthro::{Landmarks, detect_landmarks};
use cf_mjcf_emit::build;
use cf_msk_fit::{Fitter, ScanSource};
use cf_msk_lib::{Model, ParamSource};
use cf_osim::parse_leg_chain;
use sim_mjcf::load_model;

fn template() -> Model {
    let path = format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    );
    parse_leg_chain(&std::fs::read_to_string(path).expect("read gait2392.osim"))
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
fn scan_source_drives_per_segment_lengths() {
    let t = template();
    let (lm, _) = synthetic_landmarks();
    let p = ScanSource::new(lm.clone()).params(&t);

    // Femur axial scale = thigh / template femur axial length; tibia axial scale =
    // shank / template tibia axial length. The shank now drives the tibia
    // INDEPENDENTLY (the documented v1 block, lifted by the ankle).
    let femur_axial = t.segment_axial_length("femur_r", "tibia_r");
    let tibia_axial = t.segment_axial_length("tibia_r", "talus_r");
    assert!((p.femur.axial - lm.thigh_length_m / femur_axial).abs() < 1e-12);
    assert!((p.tibia.axial - lm.shank_length_m / tibia_axial).abs() < 1e-12);

    // Girth (transverse) stays at the template default until the generator/scan
    // supplies an anthropometric reference (A3-PR3); the pelvis is untouched.
    assert_eq!(p.femur.transverse, 1.0);
    assert_eq!(p.tibia.transverse, 1.0);
    assert_eq!(p.pelvis, cf_msk_lib::SegmentScale::IDENTITY);

    // Both per-segment scales are physiological.
    assert!(
        p.femur.axial > 0.5 && p.femur.axial < 2.0,
        "femur {}",
        p.femur.axial
    );
    assert!(
        p.tibia.axial > 0.5 && p.tibia.axial < 2.0,
        "tibia {}",
        p.tibia.axial
    );
}

#[test]
fn scan_source_femur_close_to_fitter() {
    let t = template();
    let (lm, _) = synthetic_landmarks();

    // The per-segment femur scale (axial) should stay close to the validated
    // `Fitter`'s single euclidean scale — they differ only by the small condylar
    // transverse offset in the hip→knee vector, not by a refactor bug.
    let via_source = ScanSource::new(lm.clone()).params(&t).femur.axial;
    let via_fitter = Fitter::new(&t, &lm).scale();
    let rel = (via_source - via_fitter).abs() / via_fitter;
    assert!(
        rel < 0.02,
        "femur axial scale {via_source} vs Fitter euclidean {via_fitter} (rel {rel:.3})"
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

#[test]
fn scan_source_scale_tracks_leg_size() {
    let t = template();

    // A longer leg (axis stretched 1.3×) has a longer thigh → a larger scale.
    let scale_of = |length_scale: f64| {
        let (mesh, _) = LegSpec::default_leg()
            .scaled(1.0, length_scale)
            .build(220, 96);
        let lm = detect_landmarks(&mesh).expect("detect on scaled synthetic leg");
        ScanSource::new(lm).params(&t).femur.axial
    };

    let base = scale_of(1.0);
    let longer = scale_of(1.3);

    // Detected size flows into the model scale: a 1.3× leg scales markedly more,
    // and roughly in proportion (the thigh length stretches ~1.3×).
    assert!(
        longer > base * 1.2,
        "longer leg should scale more: base {base:.3}, longer {longer:.3}"
    );
}
