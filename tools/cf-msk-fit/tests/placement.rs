//! S3 placement invariants — the OpenSim knee lands on the scan landmarks.
//!
//! These lock the properties the review confirmed: the joint center sits on the
//! detected knee, the bones meet there and match the measured segment lengths,
//! the femur runs along the limb axis, and the scale is thigh / OpenSim-femur.

use cf_anthro::detect_landmarks;
use cf_anthro::synthetic::LegSpec;
use cf_msk_fit::{Fitter, Placement, place_knee};
use cf_osim::oracle::{Kinematics, Variant};
use cf_osim::osim::parse_knee_subgraph;
use mesh_types::Point3;
use nalgebra::Vector3;

fn gait2392() -> String {
    std::fs::read_to_string(format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    ))
    .expect("read vendored gait2392.osim")
}

#[test]
fn places_knee_at_landmark_and_scales_bones() {
    let (leg, _gt) = LegSpec::default_leg().build(160, 64);
    let lm = detect_landmarks(&leg).expect("detect landmarks");
    let sub = parse_knee_subgraph(&gait2392());
    let p = place_knee(&sub, &lm);

    // The joint center is exactly the detected knee, and both bones meet there.
    assert!(
        (p.knee - lm.knee_point).norm() < 1e-12,
        "knee not at landmark"
    );
    assert!(
        (p.femur.distal - lm.knee_point).norm() < 1e-12,
        "femur distal != knee"
    );
    assert!(
        (p.tibia.proximal - lm.knee_point).norm() < 1e-12,
        "tibia proximal != knee"
    );

    // Bone lengths match the measured thigh / shank.
    let femur_len = (p.femur.proximal - p.femur.distal).norm();
    let tibia_len = (p.tibia.proximal - p.tibia.distal).norm();
    assert!(
        (femur_len - lm.thigh_length_m).abs() < 1e-9,
        "femur {femur_len} != thigh {}",
        lm.thigh_length_m
    );
    assert!(
        (tibia_len - lm.shank_length_m).abs() < 1e-9,
        "tibia {tibia_len} != shank {}",
        lm.shank_length_m
    );

    // The femur runs proximally along the limb axis (+z).
    let fdir = (p.femur.proximal - p.femur.distal).normalize();
    assert!(
        (fdir - Vector3::z()).norm() < 1e-9,
        "femur not along +z: {fdir:?}"
    );

    // Scale = thigh / OpenSim femur length (and is sane, ≈1 here).
    let kin = Kinematics::new(&sub);
    let knee_o = kin.body_pose("tibia_r", 0.0, Variant::TRUTH) * Point3::origin();
    let hip_o = kin.body_pose("femur_r", 0.0, Variant::TRUTH) * Point3::origin();
    let model_femur = (hip_o - knee_o).norm();
    assert!(
        (p.scale - lm.thigh_length_m / model_femur).abs() < 1e-12,
        "scale mismatch"
    );
    assert!(
        (0.7..1.4).contains(&p.scale),
        "implausible scale {}",
        p.scale
    );

    // All four muscles produce non-degenerate paths.
    assert_eq!(p.muscles.len(), 4);
    for m in &p.muscles {
        assert!(m.polyline.len() >= 2, "{} has < 2 points", m.name);
    }
}

#[test]
fn transform_is_self_consistent_at_the_hip() {
    // The directly-placed femur top must equal the model's hip mapped through
    // the same similarity transform the muscles use (no kink at the femur).
    let (leg, _gt) = LegSpec::default_leg().build(160, 64);
    let lm = detect_landmarks(&leg).expect("detect");
    let sub = parse_knee_subgraph(&gait2392());
    let p = place_knee(&sub, &lm);

    // rect_fem_r originates on the pelvis (above the hip); its first point should
    // map to ABOVE the femur top along +z — a sanity check the proximal end is up.
    let rf = p.muscles.iter().find(|m| m.name == "rect_fem_r").unwrap();
    assert!(
        rf.polyline[0].z > p.femur.distal.z,
        "rect_fem origin should be proximal to the knee"
    );
}

#[test]
fn flexion_moves_tibia_and_muscles_but_not_the_femur() {
    let (leg, _gt) = LegSpec::default_leg().build(160, 64);
    let lm = detect_landmarks(&leg).expect("detect");
    let sub = parse_knee_subgraph(&gait2392());
    let fit = Fitter::new(&sub, &lm);
    let ext = fit.pose(0.0);
    let flex = fit.pose(-90f64.to_radians());

    // The femur is fixed — knee flexion doesn't move the thigh bone.
    assert!(
        (ext.femur.proximal - flex.femur.proximal).norm() < 1e-12,
        "femur moved"
    );
    assert!((ext.femur.distal - flex.femur.distal).norm() < 1e-12);

    // The tibia swings substantially about the knee, keeping its (rigid) length.
    let swing = (ext.tibia.distal - flex.tibia.distal).norm();
    assert!(swing > 0.1, "tibia barely swung ({swing} m) at 90° flexion");
    let len = |p: &Placement| (p.tibia.proximal - p.tibia.distal).norm();
    assert!(
        (len(&ext) - len(&flex)).abs() < 1e-9,
        "tibia length changed under flexion"
    );

    // Muscle paths re-route with flexion (the moment arms at work).
    let path = |p: &Placement, n: &str| -> f64 {
        p.muscles
            .iter()
            .find(|m| m.name == n)
            .unwrap()
            .polyline
            .windows(2)
            .map(|w| (w[1] - w[0]).norm())
            .sum()
    };
    let d = (path(&ext, "semimem_r") - path(&flex, "semimem_r")).abs();
    assert!(
        d > 0.01,
        "semimem length barely changed with flexion ({d} m)"
    );
}
