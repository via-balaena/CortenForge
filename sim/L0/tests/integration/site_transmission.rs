//! Integration tests for Site-Transmission Actuators (§5 of future_work_2.md).
//!
//! Covers acceptance criteria from the spec:
//! - Mode A (no refsite): length=0, velocity=moment·qvel, force via moment
//! - Mode B (with refsite): translational/rotational length, diff Jacobian
//! - Common-ancestor zeroing
//! - ActuatorFrc sensor fix
//! - Backward compatibility (Joint/Tendon unchanged)
//! - Validation (unknown site/refsite, mutual exclusivity)

use approx::assert_relative_eq;
use sim_mjcf::{load_model, parse_mjcf_str, validate};

// ============================================================================
// Mode A — no refsite (jets / propellers)
// ============================================================================

/// Criterion 1: Mode A length is always zero.
#[test]
fn test_mode_a_length_is_zero() {
    let mjcf = r#"
        <mujoco model="mode_a_length">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="thrust" pos="0 0 0.1"/>
                </body>
            </worldbody>
            <actuator>
                <general name="jet" site="thrust" gear="0 0 1 0 0 0"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    // Set non-zero joint angle
    data.qpos[0] = 0.5;
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.actuator_length[0], 0.0, epsilon = 1e-14);
}

/// Criterion 2: Mode A velocity = moment · qvel (generally nonzero).
#[test]
fn test_mode_a_velocity_nonzero() {
    let mjcf = r#"
        <mujoco model="mode_a_velocity">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="thrust" pos="0.2 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <general name="jet" site="thrust" gear="0 0 1 0 0 0"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.qvel[0] = 1.0;
    data.forward(&model).expect("forward failed");

    // velocity = moment · qvel; moment is nonzero because site moves with hinge
    assert!(
        data.actuator_velocity[0].abs() > 1e-10,
        "Mode A velocity should be nonzero when site moves: got {}",
        data.actuator_velocity[0]
    );

    // Verify velocity = moment · qvel
    let expected_vel = data.actuator_moment[0].dot(&data.qvel);
    assert_relative_eq!(data.actuator_velocity[0], expected_vel, epsilon = 1e-14);
}

/// Criterion 3: Translational gear produces correct qfrc via Jacobian.
/// gear="0 0 1 0 0 0" → force along site's z-axis.
#[test]
fn test_mode_a_translational_force() {
    let mjcf = r#"
        <mujoco model="mode_a_trans">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="thrust" pos="0 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <general name="jet" site="thrust" gear="0 0 1 0 0 0" ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    // At qpos=0, site z-axis = world z-axis, hinge axis = world y.
    // Jacobian: cross(y_axis, lever_arm) · z_site.
    // Lever arm = site_pos - joint_anchor = (0,0,1) - (0,0,1) = (0,0,0).
    // So translational Jacobian contribution is zero (site at joint center).
    // qfrc_actuator should be 0.
    assert_relative_eq!(data.qfrc_actuator[0], 0.0, epsilon = 1e-10);

    // Now with offset site — lever arm is nonzero.
    let mjcf_offset = r#"
        <mujoco model="mode_a_trans_offset">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="thrust" pos="0.5 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <general name="jet" site="thrust" gear="0 0 1 0 0 0" ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;

    let model2 = load_model(mjcf_offset).expect("should load");
    let mut data2 = model2.make_data();
    data2.ctrl[0] = 1.0;
    data2.forward(&model2).expect("forward failed");

    // Site at (0.5, 0, 0) in body frame, body at (0, 0, 1).
    // World site pos = (0.5, 0, 1). Joint anchor = (0, 0, 1).
    // Lever arm r = (0.5, 0, 0). Hinge axis = (0, 1, 0).
    // cross(y, r) = cross((0,1,0), (0.5,0,0)) = (0*0 - 0*0, 0*0.5 - 1*0, 1*0 - 0*0.5) = (0, 0, -0.5)
    // Wait: cross((0,1,0), (0.5,0,0)) = det = (1*0-0*0, 0*0.5-0*0, 0*0-1*0.5) = (0, 0, -0.5)
    // Dot with gear direction z=(0,0,1): -0.5
    // Force = ctrl * gear[0] (gear=1 via general) = 1.0
    // qfrc = moment * force = -0.5 * 1.0 = -0.5
    assert_relative_eq!(data2.qfrc_actuator[0], -0.5, epsilon = 1e-10);
}

/// Criterion 4: Rotational gear produces pure torque.
/// gear="0 0 0 0 1 0" → torque about site's y-axis.
#[test]
fn test_mode_a_rotational_torque() {
    let mjcf = r#"
        <mujoco model="mode_a_rot">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="thrust" pos="0 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <general name="torquer" site="thrust" gear="0 0 0 0 1 0" ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    // At qpos=0, site y-axis = world y-axis, hinge axis = world y.
    // Rotational Jacobian: dot(hinge_axis, site_y) = dot((0,1,0), (0,1,0)) = 1.0
    // qfrc = 1.0 * 1.0 = 1.0
    assert_relative_eq!(data.qfrc_actuator[0], 1.0, epsilon = 1e-10);
}

/// Criterion 5: Mixed translational + rotational gear superposition.
#[test]
fn test_mode_a_mixed_gear() {
    // Pure translational gear
    let mjcf_t = r#"
        <mujoco model="trans_only">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="s1" pos="0.5 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <general name="a1" site="s1" gear="0 0 1 0 0 0" ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;
    // Pure rotational gear
    let mjcf_r = r#"
        <mujoco model="rot_only">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="s1" pos="0.5 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <general name="a1" site="s1" gear="0 0 0 0 1 0" ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;
    // Mixed gear
    let mjcf_m = r#"
        <mujoco model="mixed">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="s1" pos="0.5 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <general name="a1" site="s1" gear="0 0 1 0 1 0" ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;

    let model_t = load_model(mjcf_t).expect("load trans");
    let model_r = load_model(mjcf_r).expect("load rot");
    let model_m = load_model(mjcf_m).expect("load mixed");

    let mut data_t = model_t.make_data();
    let mut data_r = model_r.make_data();
    let mut data_m = model_m.make_data();

    data_t.ctrl[0] = 1.0;
    data_r.ctrl[0] = 1.0;
    data_m.ctrl[0] = 1.0;

    data_t.forward(&model_t).expect("fwd");
    data_r.forward(&model_r).expect("fwd");
    data_m.forward(&model_m).expect("fwd");

    // Mixed gear force = translational + rotational (superposition)
    let expected = data_t.qfrc_actuator[0] + data_r.qfrc_actuator[0];
    assert_relative_eq!(data_m.qfrc_actuator[0], expected, epsilon = 1e-10);
}

/// Criterion 7: Rotational length — 90° rotation about refsite's x-axis.
#[test]
fn test_mode_b_rotational_length() {
    // Two-link arm: b1 has hinge about x-axis, ee site on b1, target site on worldbody.
    // When j1 rotates 90° about x, the quaternion difference = π/2 about x.
    // gear = "0 0 0 1 0 0" → length = gear[3] * (π/2) = π/2.
    let mjcf = r#"
        <mujoco model="mode_b_rot_len">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="1 0 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="ee" pos="0 0 0"/>
                </body>
                <site name="target" pos="0 0 0"/>
            </worldbody>
            <actuator>
                <general name="a1" site="ee" refsite="target" gear="0 0 0 1 0 0"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    // Rotate hinge 90° about x
    data.qpos[0] = std::f64::consts::FRAC_PI_2;
    data.forward(&model).expect("forward failed");

    // subquat(q_ee, q_target) = rotation by π/2 about x → expmap = (π/2, 0, 0)
    // length = gear[3] * π/2 = 1.0 * π/2
    assert_relative_eq!(
        data.actuator_length[0],
        std::f64::consts::FRAC_PI_2,
        epsilon = 1e-10
    );
}

// ============================================================================
// Mode B — with refsite (Cartesian control)
// ============================================================================

/// Criterion 6: Translational length in refsite frame.
#[test]
fn test_mode_b_translational_length() {
    let mjcf = r#"
        <mujoco model="mode_b_trans_len">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="ee" pos="0 0 -0.5"/>
                </body>
                <site name="target" pos="0 0 0"/>
            </worldbody>
            <actuator>
                <general name="a1" site="ee" refsite="target" gear="0 0 1 0 0 0"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // ee is at (0, 0, 0.5) in world (body at (0,0,1), offset (0,0,-0.5)).
    // target is at (0, 0, 0) in world.
    // dp = ee - target = (0, 0, 0.5) in world.
    // R_ref = identity (worldbody site).
    // dp_ref = R_ref^T * dp = (0, 0, 0.5).
    // gear = [0, 0, 1, 0, 0, 0] → length = 0*0 + 0*0 + 1*0.5 = 0.5
    assert_relative_eq!(data.actuator_length[0], 0.5, epsilon = 1e-10);
}

/// Criterion 8: Mode B velocity = moment · qvel.
#[test]
fn test_mode_b_velocity_from_moment() {
    let mjcf = r#"
        <mujoco model="mode_b_vel">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="ee" pos="0 0 -0.5"/>
                </body>
                <site name="target" pos="0 0 0"/>
            </worldbody>
            <actuator>
                <general name="a1" site="ee" refsite="target" gear="0 0 1 0 0 0"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.qvel[0] = 1.0;
    data.forward(&model).expect("forward failed");

    let expected_vel = data.actuator_moment[0].dot(&data.qvel);
    assert_relative_eq!(data.actuator_velocity[0], expected_vel, epsilon = 1e-14);
}

/// Criterion 9: Energy conservation: force * velocity == qfrc · qvel.
#[test]
fn test_mode_b_energy_conservation() {
    let mjcf = r#"
        <mujoco model="mode_b_energy">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="ee" pos="0 0 -0.5"/>
                </body>
                <site name="target" pos="0 0 0"/>
            </worldbody>
            <actuator>
                <general name="a1" site="ee" refsite="target" gear="0 0 1 0 0 0"
                         ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.qvel[0] = 1.5;
    data.ctrl[0] = 3.0;
    data.forward(&model).expect("forward failed");

    let power_actuator = data.actuator_force[0] * data.actuator_velocity[0];
    let power_generalized: f64 = data
        .qfrc_actuator
        .iter()
        .zip(data.qvel.iter())
        .map(|(f, v)| f * v)
        .sum();

    assert_relative_eq!(power_actuator, power_generalized, epsilon = 1e-10);
}

/// Criterion 10: Common-ancestor zeroing — site and refsite on same body.
#[test]
fn test_common_ancestor_same_body_zero_moment() {
    let mjcf = r#"
        <mujoco model="same_body">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="s1" pos="0.1 0 0"/>
                    <site name="s2" pos="-0.1 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <general name="a1" site="s1" refsite="s2" gear="1 0 0 0 0 0"
                         ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.ctrl[0] = 10.0;
    data.forward(&model).expect("forward failed");

    // Both sites on same body — all DOFs are common-ancestor DOFs.
    // Moment should be identically zero → no generalized force.
    for dof in 0..model.nv {
        assert_relative_eq!(data.actuator_moment[0][dof], 0.0, epsilon = 1e-14);
    }
    assert_relative_eq!(data.qfrc_actuator[0], 0.0, epsilon = 1e-14);
}

/// Criterion 11: Partial chain sharing — only shared DOFs zeroed.
#[test]
fn test_common_ancestor_partial_chain() {
    let mjcf = r#"
        <mujoco model="partial_chain">
            <worldbody>
                <body name="base" pos="0 0 1">
                    <joint name="j_base" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="left" pos="0.5 0 0">
                        <joint name="j_left" type="hinge" axis="0 1 0"/>
                        <geom type="sphere" size="0.05" mass="0.5"/>
                        <site name="s_left" pos="0 0 -0.3"/>
                    </body>
                    <body name="right" pos="-0.5 0 0">
                        <joint name="j_right" type="hinge" axis="0 1 0"/>
                        <geom type="sphere" size="0.05" mass="0.5"/>
                        <site name="s_right" pos="0 0 -0.3"/>
                    </body>
                </body>
            </worldbody>
            <actuator>
                <general name="a1" site="s_left" refsite="s_right" gear="1 0 0 0 0 0"
                         ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    // j_base is common ancestor → its DOF (index 0) should be zeroed in moment.
    // j_left and j_right are unique branches → their DOFs should be nonzero.
    // j_base (DOF 0) is common ancestor → zeroed
    assert_relative_eq!(data.actuator_moment[0][0], 0.0, epsilon = 1e-14);

    // The branch DOFs (1 and 2) should have nonzero moment.
    let branch_moment_sum = data.actuator_moment[0][1].abs() + data.actuator_moment[0][2].abs();
    assert!(
        branch_moment_sum > 1e-10,
        "branch DOFs should have nonzero moment: got {}",
        branch_moment_sum
    );
}

// ============================================================================
// Backward compatibility
// ============================================================================

/// Criterion 12: Joint transmission unchanged — gear[0] works as before.
#[test]
fn test_joint_transmission_backward_compat() {
    let mjcf = r#"
        <mujoco model="joint_compat">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <motor name="m1" joint="j1" gear="50"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qvel[0] = 1.0;
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.actuator_length[0], 50.0 * 0.3, epsilon = 1e-10);
    assert_relative_eq!(data.actuator_velocity[0], 50.0 * 1.0, epsilon = 1e-10);
    assert_relative_eq!(data.qfrc_actuator[0], 50.0 * 1.0, epsilon = 1e-10);
}

/// Criterion 13: Scalar gear parses as 6D with zeros.
#[test]
fn test_scalar_gear_parses_as_6d() {
    let mjcf = r#"
        <mujoco model="scalar_gear">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <motor name="m1" joint="j1" gear="50"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_relative_eq!(model.actuator_gear[0][0], 50.0, epsilon = 1e-10);
    for i in 1..6 {
        assert_relative_eq!(model.actuator_gear[0][i], 0.0, epsilon = 1e-10);
    }
}

// ============================================================================
// Sensor correctness
// ============================================================================

/// Criterion 16: ActuatorFrc sensor reports scalar actuator force (not qfrc).
#[test]
fn test_actuator_frc_sensor_reports_scalar_force() {
    let mjcf = r#"
        <mujoco model="frc_sensor">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <motor name="m1" joint="j1" gear="50"/>
            </actuator>
            <sensor>
                <actuatorfrc actuator="m1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.ctrl[0] = 2.0;
    data.forward(&model).expect("forward failed");

    // Motor: force = gain * ctrl = 1.0 * 2.0 = 2.0 (gain=1 for motor type).
    // ActuatorFrc sensor should report 2.0 (scalar actuator force).
    // The generalized force is gear * force = 50 * 2.0 = 100.0.
    assert_relative_eq!(data.actuator_force[0], 2.0, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[0], 2.0, epsilon = 1e-10);
    // Verify generalized force = gear * scalar_force
    assert_relative_eq!(data.qfrc_actuator[0], 100.0, epsilon = 1e-10);
}

/// Criterion 17: ActuatorPos sensor for Site transmissions.
#[test]
fn test_actuator_pos_sensor_site() {
    let mjcf = r#"
        <mujoco model="pos_sensor_site">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="ee" pos="0 0 -0.5"/>
                </body>
                <site name="target" pos="0 0 0"/>
            </worldbody>
            <actuator>
                <general name="a1" site="ee" refsite="target" gear="0 0 1 0 0 0"/>
            </actuator>
            <sensor>
                <actuatorpos actuator="a1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Sensor should report same value as actuator_length
    assert_relative_eq!(data.sensordata[0], data.actuator_length[0], epsilon = 1e-14);
}

/// Criterion 18: ActuatorVel sensor for site transmissions.
#[test]
fn test_actuator_vel_sensor_site() {
    let mjcf = r#"
        <mujoco model="vel_sensor_site">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="ee" pos="0.3 0 0"/>
                </body>
                <site name="target" pos="0 0 0"/>
            </worldbody>
            <actuator>
                <general name="a1" site="ee" refsite="target" gear="0 0 1 0 0 0"/>
            </actuator>
            <sensor>
                <actuatorvel actuator="a1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.qvel[0] = 2.5;
    data.forward(&model).expect("forward failed");

    // Sensor should report same value as actuator_velocity
    assert_relative_eq!(
        data.sensordata[0],
        data.actuator_velocity[0],
        epsilon = 1e-14
    );
    // Also verify it's nonzero (site offset gives nonzero moment)
    assert!(
        data.actuator_velocity[0].abs() > 1e-10,
        "velocity should be nonzero for site with offset: got {}",
        data.actuator_velocity[0]
    );
}

// ============================================================================
// Gain/bias compatibility
// ============================================================================

/// Criterion 14: Position actuator (kp servo) with site transmission.
/// MuJoCo position actuator: gain = kp, bias = -kp*length - kv*velocity.
/// Force = kp * (ctrl - length) - kv * velocity.
#[test]
fn test_position_actuator_site_transmission() {
    let mjcf = r#"
        <mujoco model="pos_act_site">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="ee" pos="0 0 -0.5"/>
                </body>
                <site name="target" pos="0 0 0"/>
            </worldbody>
            <actuator>
                <position name="a1" site="ee" refsite="target" kp="100" gear="0 0 1 0 0 0"/>
            </actuator>
            <sensor>
                <actuatorfrc actuator="a1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    // Set control to desired position
    data.ctrl[0] = 0.0;
    data.forward(&model).expect("forward failed");

    // At rest: length = gear · dp_ref = 0.5 (from test_mode_b_translational_length).
    // Force = kp * (ctrl - length) = 100 * (0.0 - 0.5) = -50.
    let length = data.actuator_length[0];
    assert_relative_eq!(length, 0.5, epsilon = 1e-10);

    let expected_force = 100.0 * (0.0 - length);
    assert_relative_eq!(data.actuator_force[0], expected_force, epsilon = 1e-10);
    // Sensor should match
    assert_relative_eq!(data.sensordata[0], expected_force, epsilon = 1e-10);
}

/// Criterion 15: Verify site actuator force application is consistent
/// with the gain/bias model and produces correct generalized forces.
#[test]
fn test_gain_bias_force_application() {
    let mjcf = r#"
        <mujoco model="gain_bias_force">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="ee" pos="0.5 0 0"/>
                </body>
                <site name="target" pos="0 0 0"/>
            </worldbody>
            <actuator>
                <position name="a1" site="ee" refsite="target" kp="50" gear="0 0 1 0 0 0"
                          ctrlrange="-10 10"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.qvel[0] = 0.5;
    data.forward(&model).expect("forward failed");

    let length = data.actuator_length[0];
    let velocity = data.actuator_velocity[0];

    // Position actuator: gain = kp = 50, bias = -kp*length - kv*velocity.
    // Default kv = 0 for position actuators, so bias = -50 * length.
    // Force = 50 * ctrl + (-50 * length) = 50 * (ctrl - length).
    let expected_force = 50.0 * (data.ctrl[0] - length);
    assert_relative_eq!(data.actuator_force[0], expected_force, epsilon = 1e-10);

    // Verify energy conservation: force * velocity == qfrc · qvel
    let power_actuator = data.actuator_force[0] * velocity;
    let power_generalized: f64 = data
        .qfrc_actuator
        .iter()
        .zip(data.qvel.iter())
        .map(|(f, v)| f * v)
        .sum();
    assert_relative_eq!(power_actuator, power_generalized, epsilon = 1e-10);
}

// ============================================================================
// Validation
// ============================================================================

/// Criterion 19: Unknown site reference produces error.
#[test]
fn test_unknown_site_error() {
    let mjcf = r#"
        <mujoco model="bad_site">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <general name="a1" site="nonexistent" gear="0 0 1 0 0 0"/>
            </actuator>
        </mujoco>
    "#;

    let result = load_model(mjcf);
    assert!(result.is_err(), "should fail for unknown site");
}

/// Criterion 20: Unknown refsite reference produces error.
#[test]
fn test_unknown_refsite_error() {
    let mjcf = r#"
        <mujoco model="bad_refsite">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="ee" pos="0 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <general name="a1" site="ee" refsite="nonexistent" gear="0 0 1 0 0 0"/>
            </actuator>
        </mujoco>
    "#;

    let result = load_model(mjcf);
    assert!(result.is_err(), "should fail for unknown refsite");
}

/// Criterion 21: Mutual exclusivity — joint + site on same actuator is error.
#[test]
fn test_mutual_exclusivity_error() {
    let mjcf = r#"
        <mujoco model="mutual_excl">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="s1" pos="0 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <general name="a1" joint="j1" site="s1" gear="1"/>
            </actuator>
        </mujoco>
    "#;

    // validate() catches mutual exclusivity; load_model doesn't call validate().
    let parsed = parse_mjcf_str(mjcf).expect("should parse");
    let result = validate(&parsed);
    assert!(
        result.is_err(),
        "should fail for multiple transmission targets"
    );
}

// ============================================================================
// Multi-DOF and Free joint
// ============================================================================

/// Site actuator on a body with a free joint.
#[test]
fn test_site_actuator_free_joint() {
    let mjcf = r#"
        <mujoco model="free_joint_site">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <freejoint name="j1"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="thrust" pos="0 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <general name="jet_z" site="thrust" gear="0 0 1 0 0 0" ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    // Free joint has 6 DOFs: 3 translational + 3 rotational.
    // gear="0 0 1 0 0 0" → force along world z.
    // At identity pose, site z = world z.
    // Translational Jacobian identity → qfrc[2] = 1.0 (z DOF).
    assert_relative_eq!(data.qfrc_actuator[2], 1.0, epsilon = 1e-10);
    // Other translational DOFs should be zero.
    assert_relative_eq!(data.qfrc_actuator[0], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.qfrc_actuator[1], 0.0, epsilon = 1e-10);
}

// ============================================================================
// MuJoCo conformance (criterion 22)
// Reference values extracted from MuJoCo 3.5.0
// ============================================================================

/// Criterion 22a: Mode A translational — length=0, velocity and qfrc match MuJoCo.
///
/// 2-link arm: j1 hinge(Y) + j2 hinge(Y), site at (0.2, 0, -0.3) on b1.
/// gear="0 0 1 0 0 0", qpos=[0.3, -0.2], qvel=[0.5, -0.3], ctrl=1.0.
#[test]
fn test_mujoco_conformance_mode_a_translational() {
    let mjcf = r#"
        <mujoco model="mode_a_conformance">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
                    <site name="thrust" pos="0.2 0 -0.3"/>
                    <body name="b2" pos="0 0 -0.5">
                        <joint name="j2" type="hinge" axis="0 1 0"/>
                        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4"/>
                    </body>
                </body>
            </worldbody>
            <actuator>
                <general name="jet" site="thrust" gear="0 0 1 0 0 0" ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;

    // MuJoCo 3.5.0 reference values
    const REF_LENGTH: f64 = 0.0;
    const REF_VELOCITY: f64 = -0.1;
    const REF_FORCE: f64 = 1.0;
    const REF_QFRC: [f64; 2] = [-0.2, 0.0];

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.2;
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.actuator_length[0], REF_LENGTH, epsilon = 1e-8);
    assert_relative_eq!(data.actuator_velocity[0], REF_VELOCITY, epsilon = 1e-8);
    assert_relative_eq!(data.actuator_force[0], REF_FORCE, epsilon = 1e-8);
    for (i, &ref_val) in REF_QFRC.iter().enumerate() {
        assert_relative_eq!(data.qfrc_actuator[i], ref_val, epsilon = 1e-8);
    }
}

/// Criterion 22b: Mode A rotational — torque about site y-axis.
///
/// Same arm geometry, gear="0 0 0 0 1 0".
#[test]
fn test_mujoco_conformance_mode_a_rotational() {
    let mjcf = r#"
        <mujoco model="mode_a_rot_conformance">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
                    <site name="torquer" pos="0.2 0 -0.3"/>
                    <body name="b2" pos="0 0 -0.5">
                        <joint name="j2" type="hinge" axis="0 1 0"/>
                        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4"/>
                    </body>
                </body>
            </worldbody>
            <actuator>
                <general name="torquer" site="torquer" gear="0 0 0 0 1 0" ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;

    // MuJoCo 3.5.0 reference values
    const REF_LENGTH: f64 = 0.0;
    const REF_VELOCITY: f64 = 0.5;
    const REF_FORCE: f64 = 1.0;
    const REF_QFRC: [f64; 2] = [1.0, 0.0];

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.2;
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.actuator_length[0], REF_LENGTH, epsilon = 1e-8);
    assert_relative_eq!(data.actuator_velocity[0], REF_VELOCITY, epsilon = 1e-8);
    assert_relative_eq!(data.actuator_force[0], REF_FORCE, epsilon = 1e-8);
    for (i, &ref_val) in REF_QFRC.iter().enumerate() {
        assert_relative_eq!(data.qfrc_actuator[i], ref_val, epsilon = 1e-8);
    }
}

/// Criterion 22c: Mode B translational — Cartesian control with refsite.
///
/// 2-link arm with wrist site and worldbody target. gear="0 0 1 0 0 0".
/// Tests translational length = z-component of (ee - target) in refsite frame.
#[test]
fn test_mujoco_conformance_mode_b_translational() {
    let mjcf = r#"
        <mujoco model="mode_b_conformance">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
                    <site name="ee" pos="0 0 -0.5"/>
                    <body name="b2" pos="0 0 -0.5">
                        <joint name="j2" type="hinge" axis="0 1 0"/>
                        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4"/>
                        <site name="wrist" pos="0 0 -0.2"/>
                    </body>
                </body>
                <site name="target" pos="0.3 0 0.5"/>
            </worldbody>
            <actuator>
                <general name="cart_z" site="wrist" refsite="target"
                         gear="0 0 1 0 0 0" ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;

    // MuJoCo 3.5.0 reference values
    const REF_LENGTH: f64 = -0.176669077618408;
    const REF_VELOCITY: f64 = 0.077873388331208;
    const REF_FORCE: f64 = 1.0;
    const REF_QFRC: [f64; 2] = [0.167726786660035, 0.019966683329366];

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.2;
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.actuator_length[0], REF_LENGTH, epsilon = 1e-8);
    assert_relative_eq!(data.actuator_velocity[0], REF_VELOCITY, epsilon = 1e-8);
    assert_relative_eq!(data.actuator_force[0], REF_FORCE, epsilon = 1e-8);
    for (i, &ref_val) in REF_QFRC.iter().enumerate() {
        assert_relative_eq!(data.qfrc_actuator[i], ref_val, epsilon = 1e-8);
    }
}

/// Criterion 22d: Mode B rotational — quaternion-based orientation error.
///
/// 2-link arm: j1 hinge(Y) + j2 hinge(X), site on b2, refsite on worldbody.
/// gear="0 0 0 0 1 0" measures y-component of expmap orientation difference.
#[test]
fn test_mujoco_conformance_mode_b_rotational() {
    let mjcf = r#"
        <mujoco model="mode_b_rot_conformance">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
                    <site name="ee" pos="0 0 -0.3"/>
                    <body name="b2" pos="0 0 -0.5">
                        <joint name="j2" type="hinge" axis="1 0 0"/>
                        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4"/>
                        <site name="wrist" pos="0 0 -0.2"/>
                    </body>
                </body>
                <site name="target" pos="0 0 0.5"/>
            </worldbody>
            <actuator>
                <general name="cart_ry" site="wrist" refsite="target"
                         gear="0 0 0 0 1 0" ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;

    // MuJoCo 3.5.0 reference values
    const REF_LENGTH: f64 = 0.295977347088498;
    const REF_VELOCITY: f64 = 0.5;
    const REF_FORCE: f64 = 1.0;
    const REF_QFRC: [f64; 2] = [1.0, 0.0];

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = 0.4;
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.actuator_length[0], REF_LENGTH, epsilon = 1e-8);
    assert_relative_eq!(data.actuator_velocity[0], REF_VELOCITY, epsilon = 1e-8);
    assert_relative_eq!(data.actuator_force[0], REF_FORCE, epsilon = 1e-8);
    for (i, &ref_val) in REF_QFRC.iter().enumerate() {
        assert_relative_eq!(data.qfrc_actuator[i], ref_val, epsilon = 1e-8);
    }
}

/// Criterion 22e: Mode B mixed gear — superposition of translational + rotational.
///
/// gear="1 0 1 0 1 0" combines x-translation, z-translation, and y-rotation.
#[test]
fn test_mujoco_conformance_mode_b_mixed() {
    let mjcf = r#"
        <mujoco model="mode_b_mixed_conformance">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
                    <site name="ee" pos="0 0 -0.3"/>
                    <body name="b2" pos="0 0 -0.5">
                        <joint name="j2" type="hinge" axis="1 0 0"/>
                        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4"/>
                        <site name="wrist" pos="0 0 -0.2"/>
                    </body>
                </body>
                <site name="target" pos="0.3 0 0.5"/>
            </worldbody>
            <actuator>
                <general name="cart_mix" site="wrist" refsite="target"
                         gear="1 0 1 0 1 0" ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;

    // MuJoCo 3.5.0 reference values
    const REF_LENGTH: f64 = -0.359874063120312;
    const REF_VELOCITY: f64 = 0.245046432829009;
    const REF_FORCE: f64 = 1.0;
    const REF_QFRC: [f64; 2] = [0.548545650570702, 0.097421308187806];

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = 0.4;
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.actuator_length[0], REF_LENGTH, epsilon = 1e-8);
    assert_relative_eq!(data.actuator_velocity[0], REF_VELOCITY, epsilon = 1e-8);
    assert_relative_eq!(data.actuator_force[0], REF_FORCE, epsilon = 1e-8);
    for (i, &ref_val) in REF_QFRC.iter().enumerate() {
        assert_relative_eq!(data.qfrc_actuator[i], ref_val, epsilon = 1e-8);
    }
}

/// Criterion 22f: Free joint with site actuator — 6-DOF translational gear.
///
/// Body at (1, 0.5, 2) with free joint, site offset (0.2, 0.1, 0), gear="0 0 1 0 0 0".
/// Tests that cross-product torque from site offset appears in rotational DOFs.
#[test]
fn test_mujoco_conformance_free_joint_site() {
    let mjcf = r#"
        <mujoco model="free_joint_conformance">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="1 0.5 2">
                    <freejoint name="j1"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="thrust" pos="0.2 0.1 0"/>
                </body>
            </worldbody>
            <actuator>
                <general name="jet" site="thrust" gear="0 0 1 0 0 0" ctrlrange="-100 100"/>
            </actuator>
        </mujoco>
    "#;

    // MuJoCo 3.5.0 reference values
    const REF_LENGTH: f64 = 0.0;
    const REF_VELOCITY: f64 = 0.37;
    const REF_FORCE: f64 = 1.0;
    // Free joint: 6 DOFs [vx, vy, vz, wx, wy, wz]
    // gear z=1 → qfrc[2]=1.0, cross-product torque: r×F = (0.1, -0.2, 0)
    const REF_QFRC: [f64; 6] = [0.0, 0.0, 1.0, 0.1, -0.2, 0.0];

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.qvel[0] = 0.1;
    data.qvel[1] = -0.2;
    data.qvel[2] = 0.3;
    data.qvel[3] = 0.5;
    data.qvel[4] = -0.1;
    data.qvel[5] = 0.4;
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.actuator_length[0], REF_LENGTH, epsilon = 1e-8);
    assert_relative_eq!(data.actuator_velocity[0], REF_VELOCITY, epsilon = 1e-8);
    assert_relative_eq!(data.actuator_force[0], REF_FORCE, epsilon = 1e-8);
    for (i, &ref_val) in REF_QFRC.iter().enumerate() {
        assert_relative_eq!(data.qfrc_actuator[i], ref_val, epsilon = 1e-8);
    }
}

/// Criterion 22g: Position actuator (kp servo) with site + refsite.
///
/// kp=100, ctrl=0.0 → force = kp*(ctrl - length) = -kp*length.
/// Tests that gain/bias model composes correctly with site transmission.
#[test]
fn test_mujoco_conformance_position_actuator_site() {
    let mjcf = r#"
        <mujoco model="pos_act_conformance">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
                    <site name="ee" pos="0 0 -0.5"/>
                    <body name="b2" pos="0 0 -0.5">
                        <joint name="j2" type="hinge" axis="0 1 0"/>
                        <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -0.4"/>
                        <site name="wrist" pos="0 0 -0.2"/>
                    </body>
                </body>
                <site name="target" pos="0 0 0.5"/>
            </worldbody>
            <actuator>
                <position name="servo" site="wrist" refsite="target" kp="100"
                          gear="0 0 1 0 0 0"/>
            </actuator>
        </mujoco>
    "#;

    // MuJoCo 3.5.0 reference values
    const REF_LENGTH: f64 = -0.176669077618408;
    const REF_VELOCITY: f64 = 0.077873388331208;
    const REF_FORCE: f64 = 17.666907761840804; // kp * (ctrl - length) = 100 * (0 - (-0.1767))
    const REF_QFRC: [f64; 2] = [2.963213669112796, 0.352749552689787];

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.qpos[0] = 0.3;
    data.qpos[1] = -0.2;
    data.qvel[0] = 0.5;
    data.qvel[1] = -0.3;
    data.ctrl[0] = 0.0;
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.actuator_length[0], REF_LENGTH, epsilon = 1e-8);
    assert_relative_eq!(data.actuator_velocity[0], REF_VELOCITY, epsilon = 1e-8);
    assert_relative_eq!(data.actuator_force[0], REF_FORCE, epsilon = 1e-8);
    for (i, &ref_val) in REF_QFRC.iter().enumerate() {
        assert_relative_eq!(data.qfrc_actuator[i], ref_val, epsilon = 1e-8);
    }
}
