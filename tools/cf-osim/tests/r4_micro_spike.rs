//! R4 micro-spike — prove the emit→import→moment-arm loop closes.
//!
//! The first sub-step of the S0 bridge spike (recon risk R4): before any
//! `.osim` parsing exists, hand-author the smallest meaningful MJCF — one hinge
//! joint plus one spatial tendon wrapping a cylinder centered on the hinge axis
//! — load it through the engine, and confirm we can pull a correct knee-DOF
//! moment arm out of `ten_J`.
//!
//! Why a cylinder centered on the rotation axis: a tendon wrapping such a
//! cylinder is the textbook ideal pulley — its moment arm equals the cylinder
//! radius exactly, for every angle in the wrapping range (both straight
//! segments are tangent to the cylinder, so their perpendicular distance from
//! the axis is the radius). That gives a closed-form oracle with no external
//! data: **moment arm ≡ r**.
//!
//! Checks: the model loads + `ten_J` is populated; `-ten_J` matches a finite
//! difference of the engine's own length curve (extraction + sign correct);
//! the moment arm equals the cylinder radius in the wrap regime (the loop is
//! sane); and it *unpins* from the radius once the wrap disengages (the solver
//! isn't trivially returning `r`).

use approx::assert_relative_eq;
use cf_osim::{joint_id, moment_arm_finite_diff, moment_arm_sweep, tendon_id};
use sim_mjcf::load_model;

/// Hinge about +z at the origin; a radius-0.05 wrap cylinder (its long axis is
/// local +z, i.e. coincident with the hinge axis) centered on the joint; the
/// tendon runs proximal-site → cylinder → distal-site. The proximal site sits
/// on the welded root (`femur`); the distal site rides the rotating `tibia`.
const PULLEY_KNEE: &str = r#"
<mujoco>
  <worldbody>
    <body name="femur" pos="0 0 0">
      <geom name="knee_pulley" type="cylinder" size="0.05 0.1"/>
      <site name="origin" pos="0 0.2 0"/>
      <body name="tibia" pos="0 0 0">
        <joint name="knee" type="hinge" axis="0 0 1"/>
        <site name="insertion" pos="0 -0.2 0"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <spatial name="quad">
      <site site="origin"/>
      <geom geom="knee_pulley"/>
      <site site="insertion"/>
    </spatial>
  </tendon>
</mujoco>
"#;

/// Radius of the wrap cylinder in `PULLEY_KNEE` — the analytic moment arm.
const PULLEY_RADIUS: f64 = 0.05;

/// Angles (rad) where the straight chord between the two sites passes within
/// the cylinder radius, so the tendon genuinely wraps. For both sites at radius
/// R=0.2, wrapping holds while 0.2·sin(θ/2) < r=0.05, i.e. θ < ~0.505 rad. We
/// start at 0.1 to stay clear of the exactly-collinear singularity at θ=0.
const WRAP_ANGLES: &[f64] = &[0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40];

#[test]
fn loop_closes_model_loads_and_forward_populates_tendon() {
    let model = load_model(PULLEY_KNEE).expect("minimal knee MJCF should load");
    assert_eq!(model.ntendon, 1, "one spatial tendon");
    assert_eq!(model.nv, 1, "one hinge DOF");

    let t = tendon_id(&model, "quad");
    let mut data = model.make_data();
    data.qpos[0] = 0.25;
    data.forward(&model).expect("forward pass failed");

    assert!(
        data.ten_length[t].is_finite() && data.ten_length[t] > 0.0,
        "tendon length must be finite and positive, got {}",
        data.ten_length[t]
    );
    assert_eq!(data.ten_J[t].len(), model.nv, "Jacobian spans all DOFs");
    assert!(
        data.ten_J[t][0].abs() > 1e-6,
        "wrapping tendon must have a non-zero moment arm about the knee"
    );
}

#[test]
fn moment_arm_matches_finite_difference_of_length() {
    let model = load_model(PULLEY_KNEE).expect("load");
    let knee = joint_id(&model, "knee");
    let t = tendon_id(&model, "quad");

    let samples = moment_arm_sweep(&model, t, knee, WRAP_ANGLES);
    for s in &samples {
        let fd = moment_arm_finite_diff(&model, t, knee, s.angle, 1e-6);
        // The analytic -ten_J and a length-only central difference are two
        // independent paths to the same number; agreement proves the
        // extraction + sign are right (and re-confirms ten_J end to end).
        assert_relative_eq!(s.moment_arm, fd, epsilon = 1e-4);
    }
}

#[test]
fn moment_arm_equals_cylinder_radius() {
    let model = load_model(PULLEY_KNEE).expect("load");
    let knee = joint_id(&model, "knee");
    let t = tendon_id(&model, "quad");

    let samples = moment_arm_sweep(&model, t, knee, WRAP_ANGLES);
    for s in &samples {
        // Ideal-pulley closed form: |moment arm| = cylinder radius, every angle.
        assert_relative_eq!(s.moment_arm.abs(), PULLEY_RADIUS, epsilon = 1e-3);
    }
}

/// Past the disengage boundary (θ ≳ 0.5 rad here, where the straight chord
/// clears the cylinder) the tendon stops wrapping, so the moment arm must
/// *unpin* from the radius and vary with angle. Without this, a wrap solver that
/// trivially always returned `r` would pass the in-regime radius check above.
#[test]
fn moment_arm_unpins_when_wrap_disengages() {
    let model = load_model(PULLEY_KNEE).expect("load");
    let knee = joint_id(&model, "knee");
    let t = tendon_id(&model, "quad");

    let disengaged = moment_arm_sweep(&model, t, knee, &[0.7, 0.8, 0.9, 1.0]);
    // Not pinned to the radius any more...
    assert!(
        disengaged
            .iter()
            .any(|s| (s.moment_arm.abs() - PULLEY_RADIUS).abs() > 2e-3),
        "moment arm stayed pinned to the radius after the wrap should have disengaged"
    );
    // ...and actually varies across the disengaged range (constant in-wrap).
    let span = disengaged
        .iter()
        .map(|s| s.moment_arm)
        .fold(f64::MIN, f64::max)
        - disengaged
            .iter()
            .map(|s| s.moment_arm)
            .fold(f64::MAX, f64::min);
    assert!(
        span > 1e-3,
        "moment arm should vary once unwrapped, span={span:.4}"
    );
}
