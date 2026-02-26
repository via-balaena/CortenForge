//! DT-103: Spatial transport helper tests.
//!
//! T5–T14 verify the composed helpers (`object_velocity`, `object_acceleration`,
//! `object_force`) and backward compatibility of `object_velocity_local`.
//!
//! T1–T4 (pure transport kernels) live as unit tests in
//! `sim-core/src/dynamics/spatial.rs`.

use nalgebra::Vector3;
use sim_core::dynamics::{
    object_acceleration, object_force, object_velocity, object_velocity_local,
};
use sim_mjcf::load_model;

const TOL: f64 = 1e-10;
const TOL_TIGHT: f64 = 1e-15;

// ============================================================================
// T5: object_velocity vs object_velocity_local consistency
// ============================================================================

/// T5: object_velocity(Some(rot)) matches object_velocity_local for a rotating body.
#[test]
fn t05_object_velocity_matches_local() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 0 1"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="s1" pos="0.3 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <velocimeter site="s1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 5.0;
    data.forward(&model).expect("forward");

    let body_id = 1;
    let site_pos = data.site_xpos[0];
    let site_mat = data.site_xmat[0];

    let (omega_new, v_new) = object_velocity(&data, body_id, &site_pos, Some(&site_mat));
    let old = object_velocity_local(&model, &data, body_id, &site_pos, &site_mat);

    assert!((omega_new.x - old[0]).abs() < TOL_TIGHT, "omega.x mismatch");
    assert!((omega_new.y - old[1]).abs() < TOL_TIGHT, "omega.y mismatch");
    assert!((omega_new.z - old[2]).abs() < TOL_TIGHT, "omega.z mismatch");
    assert!((v_new.x - old[3]).abs() < TOL_TIGHT, "v.x mismatch");
    assert!((v_new.y - old[4]).abs() < TOL_TIGHT, "v.y mismatch");
    assert!((v_new.z - old[5]).abs() < TOL_TIGHT, "v.z mismatch");
}

// ============================================================================
// T6: object_velocity world frame (None rotation)
// ============================================================================

/// T6: object_velocity(None) matches world-frame sensor reading.
#[test]
fn t06_object_velocity_world_frame() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 0 1"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="s1" pos="0.3 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <framelinvel site="s1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 5.0;
    data.forward(&model).expect("forward");

    let body_id = 1;
    let site_pos = data.site_xpos[0];
    let (_omega, v) = object_velocity(&data, body_id, &site_pos, None);

    let adr = model.sensor_adr[0];
    assert!(
        (v.x - data.sensordata[adr]).abs() < TOL_TIGHT,
        "v.x mismatch"
    );
    assert!(
        (v.y - data.sensordata[adr + 1]).abs() < TOL_TIGHT,
        "v.y mismatch"
    );
    assert!(
        (v.z - data.sensordata[adr + 2]).abs() < TOL_TIGHT,
        "v.z mismatch"
    );
}

// ============================================================================
// T7: object_acceleration — static body under gravity
// ============================================================================

/// T7: Static body reads (alpha=0, a_linear=[0,0,+9.81]) from object_acceleration.
#[test]
fn t07_object_acceleration_static_gravity() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="imu" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <accelerometer site="imu"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let body_id = 1;
    let pos = data.xpos[body_id];
    let (alpha, a_lin) = object_acceleration(&data, body_id, &pos, None);

    assert!(alpha.x.abs() < TOL, "alpha.x should be ~0, got {}", alpha.x);
    assert!(alpha.y.abs() < TOL, "alpha.y should be ~0, got {}", alpha.y);
    assert!(alpha.z.abs() < TOL, "alpha.z should be ~0, got {}", alpha.z);
    assert!(a_lin.x.abs() < TOL, "a.x should be ~0, got {}", a_lin.x);
    assert!(a_lin.y.abs() < TOL, "a.y should be ~0, got {}", a_lin.y);
    assert!(
        (a_lin.z - 9.81).abs() < TOL,
        "a.z should be +9.81, got {}",
        a_lin.z
    );
}

// ============================================================================
// T8: object_acceleration — centripetal
// ============================================================================

/// T8: Rotating body (omega=10, site at [0.5,0,0]) → centripetal [-50,0,0].
#[test]
fn t08_object_acceleration_centripetal() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 0 1"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="imu" pos="0.5 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <accelerometer site="imu"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 10.0;
    data.forward(&model).expect("forward");

    let body_id = 1;
    let site_pos = data.site_xpos[0];
    let (_alpha, a_lin) = object_acceleration(&data, body_id, &site_pos, None);

    assert!(
        (a_lin.x - (-50.0)).abs() < 1e-8,
        "a.x should be -50, got {}",
        a_lin.x
    );
    assert!(a_lin.y.abs() < 1e-8, "a.y should be ~0, got {}", a_lin.y);
    assert!(a_lin.z.abs() < 1e-8, "a.z should be ~0, got {}", a_lin.z);
}

// ============================================================================
// T9: object_force — force translation invariance
// ============================================================================

/// T9: Force component is identical at body origin and at offset point.
#[test]
fn t09_object_force_translation_invariance() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="s1" pos="0 0 0"/>
          <site name="s2" pos="0.5 0.3 -0.2"/>
        </body>
      </worldbody>
      <sensor>
        <force site="s1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let body_id = 1;
    let pos_origin = data.xpos[body_id];
    let pos_offset = data.site_xpos[1];

    let (_torque_a, force_a) = object_force(&data, body_id, &pos_origin, None);
    let (_torque_b, force_b) = object_force(&data, body_id, &pos_offset, None);

    assert_eq!(force_a, force_b, "Force should be identical at both points");
}

// ============================================================================
// T10: object_force — torque transport
// ============================================================================

/// T10: Torque at offset point = torque_at_origin − r × force.
#[test]
fn t10_object_force_torque_transport() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="s1" pos="0 0 0"/>
          <site name="s2" pos="0.5 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <force site="s1"/>
        <torque site="s1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let body_id = 1;
    let pos_origin = data.xpos[body_id];
    let pos_offset = data.site_xpos[1]; // s2 at [0.5, 0, 0] offset

    let (torque_origin, force_origin) = object_force(&data, body_id, &pos_origin, None);
    let (torque_offset, _force_offset) = object_force(&data, body_id, &pos_offset, None);

    // Expected: torque_offset = torque_origin − r × force
    let r = pos_offset - pos_origin;
    let expected_torque = torque_origin - r.cross(&force_origin);

    for i in 0..3 {
        assert!(
            (torque_offset[i] - expected_torque[i]).abs() < 1e-12,
            "Torque[{i}] mismatch: expected {}, got {}",
            expected_torque[i],
            torque_offset[i]
        );
    }
}

// ============================================================================
// T13: World body (body_id = 0)
// ============================================================================

/// T13: World body produces valid (finite, zero velocity) output.
#[test]
fn t13_world_body() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="imu" pos="0 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <accelerometer site="imu"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let point = Vector3::new(1.0, 0.0, 0.0);

    // Velocity: world body is static → zero
    let (omega, v) = object_velocity(&data, 0, &point, None);
    assert!(omega.norm() < TOL_TIGHT, "World body omega should be 0");
    assert!(v.norm() < TOL_TIGHT, "World body v should be 0");
    assert!(omega.x.is_finite() && omega.y.is_finite() && omega.z.is_finite());
    assert!(v.x.is_finite() && v.y.is_finite() && v.z.is_finite());

    // Acceleration: world body has gravity pseudo-acceleration
    let (alpha, a_lin) = object_acceleration(&data, 0, &point, None);
    assert!(alpha.norm() < TOL, "World body alpha should be ~0");
    assert!(
        (a_lin.z - 9.81).abs() < TOL,
        "World body a.z should be +9.81, got {}",
        a_lin.z
    );
    assert!(a_lin.x.is_finite() && a_lin.y.is_finite() && a_lin.z.is_finite());
}

// ============================================================================
// T14: object_velocity_local backward compatibility
// ============================================================================

/// T14: object_velocity_local returns identical results to the pre-refactor
/// implementation (bit-identical).
#[test]
fn t14_object_velocity_local_backward_compat() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 0 1"/>
          <geom type="sphere" size="0.1" mass="1.0"/>
          <site name="s1" pos="0.3 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <velocimeter site="s1"/>
      </sensor>
    </mujoco>"#;

    let model = load_model(xml).expect("load");
    let mut data = model.make_data();
    data.qvel[0] = 5.0;
    data.forward(&model).expect("forward");

    let body_id = 1;
    let site_pos = data.site_xpos[0];
    let site_mat = data.site_xmat[0];

    let result = object_velocity_local(&model, &data, body_id, &site_pos, &site_mat);

    // Manually compute expected (pre-refactor logic):
    let cvel = &data.cvel[body_id];
    let omega = Vector3::new(cvel[0], cvel[1], cvel[2]);
    let v_origin = Vector3::new(cvel[3], cvel[4], cvel[5]);
    let dif = site_pos - data.xpos[body_id];
    let v_point = v_origin + omega.cross(&dif);
    let omega_local = site_mat.transpose() * omega;
    let v_local = site_mat.transpose() * v_point;

    assert!((result[0] - omega_local.x).abs() < TOL_TIGHT, "omega.x");
    assert!((result[1] - omega_local.y).abs() < TOL_TIGHT, "omega.y");
    assert!((result[2] - omega_local.z).abs() < TOL_TIGHT, "omega.z");
    assert!((result[3] - v_local.x).abs() < TOL_TIGHT, "v.x");
    assert!((result[4] - v_local.y).abs() < TOL_TIGHT, "v.y");
    assert!((result[5] - v_local.z).abs() < TOL_TIGHT, "v.z");

    // Also test body_id == 0 returns zeros
    let world_result = object_velocity_local(&model, &data, 0, &site_pos, &site_mat);
    assert_eq!(world_result, [0.0; 6], "World body should return zeros");
}
