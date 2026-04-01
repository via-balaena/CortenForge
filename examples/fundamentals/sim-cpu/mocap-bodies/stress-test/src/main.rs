//! Stress test — headless validation of all mocap body invariants.
//!
//! 12 checks covering: position/quaternion tracking, world-child invariant,
//! contact generation, gravity immunity, contact force immunity, multi-mocap
//! independence, keyframe restore, weld-to-mocap force/tracking, zero-mass FK,
//! and child-follows-parent.
//!
//! Run: `cargo run -p example-mocap-bodies-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::cast_sign_loss,
    clippy::cast_lossless
)]

use nalgebra::{UnitQuaternion, Vector3};

// ── MJCF Models ────────────────────────────────────────────────────────────

/// Model A — Single mocap body (wall) + dynamic ball, gravity, contact-capable.
/// Used by checks 1–6.
const MODEL_A: &str = r#"
<mujoco model="mocap-contact">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="wall" mocap="true" pos="2 0 0.5">
      <geom type="box" size="0.05 1 1" mass="10.0"/>
    </body>
    <body name="ball" pos="0 0 0.5">
      <freejoint name="ball_free"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model B — Two independent mocap bodies + dynamic body.
/// Used by check 7.
const MODEL_B: &str = r#"
<mujoco model="mocap-two">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="target_a" mocap="true" pos="1 0 0">
      <geom type="sphere" size="0.05" mass="0.1"/>
    </body>
    <body name="target_b" mocap="true" pos="0 1 0">
      <geom type="sphere" size="0.05" mass="0.2"/>
    </body>
    <body name="link" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model C — Mocap body with keyframe storing mpos/mquat.
/// Used by check 8.
const MODEL_C: &str = r#"
<mujoco model="mocap-keyframe">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="target" mocap="true" pos="1 0 0">
      <geom type="sphere" size="0.05" mass="0.1"/>
    </body>
    <body name="link" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
  <keyframe>
    <key name="home" qpos="0.0" mpos="5 6 7" mquat="0.707 0.707 0 0"/>
  </keyframe>
</mujoco>
"#;

/// Model D — Mocap body + free-floating box connected by soft weld.
/// Used by checks 9–10.
const MODEL_D: &str = r#"
<mujoco model="mocap-weld">
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>
  <worldbody>
    <body name="target" mocap="true" pos="0 0 1">
      <geom type="sphere" size="0.05" mass="0.1"/>
    </body>
    <body name="follower" pos="0 0 1">
      <freejoint name="free"/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
    </body>
  </worldbody>
  <equality>
    <weld body1="follower" body2="target" solref="0.02 1.0"/>
  </equality>
</mujoco>
"#;

/// Model E — Mocap body with zero-mass geom.
/// Used by check 11.
const MODEL_E: &str = r#"
<mujoco model="mocap-zero-mass">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="ghost" mocap="true" pos="1 2 3">
      <geom type="sphere" size="0.05" mass="0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model F — Mocap body with a child body (hinge joint on child).
/// Used by check 12.
const MODEL_F: &str = r#"
<mujoco model="mocap-child">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="platform" mocap="true" pos="0 0 2">
      <geom type="box" size="0.5 0.5 0.05" mass="1.0"/>
      <body name="arm" pos="0 0 0.05">
        <joint name="hinge" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 0.4" mass="0.5"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Helpers ────────────────────────────────────────────────────────────────

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

// ── Check 1: Position tracks mocap_pos ────────────────────────────────────

fn check_1_position_tracks() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_A).expect("parse");
    let mut data = model.make_data();

    let wall_id = 1; // first non-world body
    let mocap_idx = model.body_mocapid[wall_id].expect("wall is mocap");

    let new_pos = Vector3::new(3.0, 4.0, 5.0);
    data.mocap_pos[mocap_idx] = new_pos;
    data.forward(&model).expect("forward");

    let err = (data.xpos[wall_id] - new_pos).norm();
    let p = check(
        "Position tracks mocap_pos",
        err < 1e-12,
        &format!("error = {err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 2: Quaternion tracks mocap_quat ─────────────────────────────────

fn check_2_quaternion_tracks() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_A).expect("parse");
    let mut data = model.make_data();

    let wall_id = 1;
    let mocap_idx = model.body_mocapid[wall_id].expect("wall is mocap");

    // 45° about Z
    let new_quat = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_4);
    data.mocap_quat[mocap_idx] = new_quat;
    data.forward(&model).expect("forward");

    let angle_err = data.xquat[wall_id].angle_to(&new_quat);
    let p = check(
        "Quaternion tracks mocap_quat",
        angle_err < 1e-12,
        &format!("angle error = {angle_err:.2e} rad"),
    );
    (u32::from(p), 1)
}

// ── Check 3: World-child invariant ────────────────────────────────────────

fn check_3_world_child() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_A).expect("parse");

    let wall_id = 1;
    let parent = model.body_parent[wall_id];
    let p = check(
        "World-child invariant",
        parent == 0,
        &format!("body_parent[{wall_id}] = {parent}"),
    );
    (u32::from(p), 1)
}

// ── Check 4: Contact generation ───────────────────────────────────────────

fn check_4_contact_generation() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_A).expect("parse");
    let mut data = model.make_data();

    let wall_id = 1;
    let mocap_idx = model.body_mocapid[wall_id].expect("wall is mocap");

    // Move wall right next to ball (ball is at x=0, wall geom half-size=0.05)
    data.mocap_pos[mocap_idx] = Vector3::new(0.15, 0.0, 0.5);

    for _ in 0..10 {
        data.step(&model).expect("step");
    }

    let p = check(
        "Contact generation",
        data.ncon > 0,
        &format!("ncon = {}", data.ncon),
    );
    (u32::from(p), 1)
}

// ── Check 5: Immune to gravity ────────────────────────────────────────────

fn check_5_gravity_immune() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_A).expect("parse");
    let mut data = model.make_data();

    let wall_id = 1;
    data.forward(&model).expect("forward");
    let pos_before = data.xpos[wall_id];

    for _ in 0..1000 {
        data.step(&model).expect("step");
    }

    let drift = (data.xpos[wall_id] - pos_before).norm();
    let p = check(
        "Immune to gravity",
        drift < 1e-12,
        &format!("position drift = {drift:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 6: Immune to contact forces ─────────────────────────────────────

fn check_6_contact_immune() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_A).expect("parse");
    let mut data = model.make_data();

    let wall_id = 1;
    let mocap_idx = model.body_mocapid[wall_id].expect("wall is mocap");

    // Place wall in contact with ball
    let wall_pos = Vector3::new(0.15, 0.0, 0.5);
    data.mocap_pos[mocap_idx] = wall_pos;

    for _ in 0..100 {
        data.step(&model).expect("step");
    }

    let drift = (data.xpos[wall_id] - wall_pos).norm();
    let p = check(
        "Immune to contact forces",
        drift < 1e-12,
        &format!("position drift after contact = {drift:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 7: Multiple mocap independent ───────────────────────────────────

fn check_7_multi_mocap() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_B).expect("parse");
    let mut data = model.make_data();

    let a_id = 1;
    let b_id = 2;
    let idx_a = model.body_mocapid[a_id].expect("target_a is mocap");
    let _idx_b = model.body_mocapid[b_id].expect("target_b is mocap");

    data.forward(&model).expect("forward");
    let b_pos_before = data.xpos[b_id];

    // Move only A
    let new_a = Vector3::new(100.0, 0.0, 0.0);
    data.mocap_pos[idx_a] = new_a;
    data.forward(&model).expect("forward");

    let a_ok = (data.xpos[a_id] - new_a).norm() < 1e-12;
    let b_drift = (data.xpos[b_id] - b_pos_before).norm();
    let b_ok = b_drift < 1e-12;

    let p = check(
        "Multiple mocap independent",
        a_ok && b_ok,
        &format!("A at target: {a_ok}, B drift = {b_drift:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 8: Keyframe restores mocap state ────────────────────────────────

fn check_8_keyframe_restore() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_C).expect("parse");
    let mut data = model.make_data();

    let target_id = 1;
    let mocap_idx = model.body_mocapid[target_id].expect("target is mocap");

    // Scramble mocap to arbitrary values
    data.mocap_pos[mocap_idx] = Vector3::new(99.0, 88.0, 77.0);
    data.mocap_quat[mocap_idx] = UnitQuaternion::identity();

    // Reset to keyframe "home" (mpos="5 6 7", mquat="0.707 0.707 0 0")
    let kf_id = model.keyframe_id("home").expect("keyframe 'home' exists");
    data.reset_to_keyframe(&model, kf_id).expect("reset");

    let pos_err = (data.mocap_pos[mocap_idx] - Vector3::new(5.0, 6.0, 7.0)).norm();
    let expected_quat =
        UnitQuaternion::new_normalize(nalgebra::Quaternion::new(0.707, 0.707, 0.0, 0.0));
    let quat_err = data.mocap_quat[mocap_idx].angle_to(&expected_quat);

    let p = check(
        "Keyframe restores mocap state",
        pos_err < 1e-6 && quat_err < 1e-6,
        &format!("pos_err = {pos_err:.2e}, quat_err = {quat_err:.2e} rad"),
    );
    (u32::from(p), 1)
}

// ── Check 9: Weld-to-mocap: force generated ───────────────────────────────

fn check_9_weld_force() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_D).expect("parse");
    let mut data = model.make_data();

    let target_id = 1;
    let mocap_idx = model.body_mocapid[target_id].expect("target is mocap");

    // Displace mocap to create violation
    data.mocap_pos[mocap_idx] = Vector3::new(1.0, 0.0, 1.0);
    data.forward(&model).expect("forward");

    let max_frc = data
        .qfrc_constraint
        .iter()
        .map(|x| x.abs())
        .fold(0.0_f64, f64::max);

    let p = check(
        "Weld-to-mocap: force generated",
        max_frc > 1e-6,
        &format!("max |qfrc_constraint| = {max_frc:.4e}"),
    );
    (u32::from(p), 1)
}

// ── Check 10: Weld-to-mocap: follower tracks ──────────────────────────────

fn check_10_weld_tracking() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_D).expect("parse");
    let mut data = model.make_data();

    let target_id = 1;
    let follower_id = 2;
    let mocap_idx = model.body_mocapid[target_id].expect("target is mocap");

    // Displace mocap 0.3 m along X
    let displacement = 0.3;
    let target_pos = Vector3::new(displacement, 0.0, 1.0);
    data.mocap_pos[mocap_idx] = target_pos;

    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    let final_sep = (data.xpos[follower_id] - target_pos).norm();
    let threshold = displacement * 0.5;

    let p = check(
        "Weld-to-mocap: follower tracks",
        final_sep < threshold,
        &format!("separation = {final_sep:.4}, threshold = {threshold:.4}"),
    );
    (u32::from(p), 1)
}

// ── Check 11: Zero-mass FK override ───────────────────────────────────────

fn check_11_zero_mass_fk() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_E).expect("parse");
    let mut data = model.make_data();

    let ghost_id = 1;
    let mocap_idx = model.body_mocapid[ghost_id].expect("ghost is mocap");

    // Verify mass is actually zero
    let mass = model.body_mass[ghost_id];

    let new_pos = Vector3::new(10.0, 20.0, 30.0);
    data.mocap_pos[mocap_idx] = new_pos;
    data.forward(&model).expect("forward");

    let err = (data.xpos[ghost_id] - new_pos).norm();
    let p = check(
        "Zero-mass FK override",
        err < 1e-12 && mass == 0.0,
        &format!("body_mass = {mass}, position error = {err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 12: Child body follows mocap parent ─────────────────────────────

fn check_12_child_follows() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_F).expect("parse");
    let mut data = model.make_data();

    let platform_id = 1;
    let arm_id = 2;
    let mocap_idx = model.body_mocapid[platform_id].expect("platform is mocap");

    // Get initial positions
    data.forward(&model).expect("forward");
    let arm_pos_before = data.xpos[arm_id];
    let plat_pos_before = data.xpos[platform_id];

    // Move platform +5 along X
    let shift = Vector3::new(5.0, 0.0, 0.0);
    data.mocap_pos[mocap_idx] = plat_pos_before + shift;
    data.forward(&model).expect("forward");

    let arm_shift = data.xpos[arm_id] - arm_pos_before;
    let shift_err = (arm_shift - shift).norm();

    let p = check(
        "Child body follows mocap parent",
        shift_err < 1e-10,
        &format!(
            "arm shift = [{:.4}, {:.4}, {:.4}], error = {shift_err:.2e}",
            arm_shift.x, arm_shift.y, arm_shift.z
        ),
    );
    (u32::from(p), 1)
}

// ── Main ───────────────────────────────────────────────────────────────────

fn main() {
    println!("=== Mocap Bodies — Stress Test ===\n");

    let checks: Vec<(&str, fn() -> (u32, u32))> = vec![
        ("Position tracks mocap_pos", check_1_position_tracks),
        ("Quaternion tracks mocap_quat", check_2_quaternion_tracks),
        ("World-child invariant", check_3_world_child),
        ("Contact generation", check_4_contact_generation),
        ("Immune to gravity", check_5_gravity_immune),
        ("Immune to contact forces", check_6_contact_immune),
        ("Multiple mocap independent", check_7_multi_mocap),
        ("Keyframe restores mocap state", check_8_keyframe_restore),
        ("Weld-to-mocap: force generated", check_9_weld_force),
        ("Weld-to-mocap: follower tracks", check_10_weld_tracking),
        ("Zero-mass FK override", check_11_zero_mass_fk),
        ("Child follows mocap parent", check_12_child_follows),
    ];

    let mut total = 0u32;
    let mut passed = 0u32;

    for (i, (label, func)) in checks.iter().enumerate() {
        println!("-- {}. {} --", i + 1, label);
        let (p, t) = func();
        passed += p;
        total += t;
        println!();
    }

    println!("============================================================");
    println!("  TOTAL: {passed}/{total} checks passed");
    if passed == total {
        println!("  ALL PASS");
    } else {
        println!("  {} FAILED", total - passed);
        std::process::exit(1);
    }
}
