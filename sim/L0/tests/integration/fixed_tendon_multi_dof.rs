//! DT-28: Ball/Free joints in fixed tendons — validation tests.
//!
//! Validates that fixed tendon length computation correctly uses
//! qpos[jnt_qpos_adr] (position space) for length and
//! ten_J[jnt_dof_adr] (velocity space) for the Jacobian.
//!
//! For hinge/slide joints, jnt_qpos_adr == jnt_dof_adr (no distinction).
//! For ball joints, qpos is quaternion (nq=4) but DOFs are angular velocity (nv=3).
//! For free joints, qpos is pos+quat (nq=7) but DOFs are lin+ang velocity (nv=6).
//!
//! MuJoCo ref: mj_tendon() in engine_core_smooth.c — fixed tendon length
//! uses coef * qpos[jnt_qposadr[joint_id]] for each wrap element.

use approx::assert_relative_eq;
use sim_mjcf::load_model;

// ============================================================================
// Test 1: Hinge joint in fixed tendon (regression baseline)
// ============================================================================

#[test]
fn fixed_tendon_hinge_regression() {
    let mjcf = r#"
    <mujoco model="dt28_hinge">
        <option gravity="0 0 0" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <joint name="J1" type="hinge" axis="0 1 0"/>
                <geom type="capsule" size="0.05 0.3" mass="1.0"/>
            </body>
        </worldbody>
        <tendon>
            <fixed name="T1"><joint joint="J1" coef="2.5"/></fixed>
        </tendon>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // At qpos0, hinge angle = 0.0, so tendon length = 2.5 * 0.0 = 0.0
    assert_relative_eq!(data.ten_length[0], 0.0, epsilon = 1e-12);
    assert_relative_eq!(model.tendon_length0[0], 0.0, epsilon = 1e-12);

    // Jacobian: dL/dqvel[dof_adr] = coef = 2.5
    let dof_adr = model.jnt_dof_adr[0];
    assert_relative_eq!(data.ten_J[0][dof_adr], 2.5, epsilon = 1e-12);

    // Move joint to 0.5 rad, recompute
    data.qpos[model.jnt_qpos_adr[0]] = 0.5;
    data.forward(&model).expect("forward failed");
    assert_relative_eq!(data.ten_length[0], 2.5 * 0.5, epsilon = 1e-12);
}

// ============================================================================
// Test 2: Ball joint in fixed tendon — couples to quaternion w
// ============================================================================

#[test]
fn fixed_tendon_ball_joint_length() {
    // Ball joint: qpos = [w, x, y, z] quaternion.
    // Fixed tendon couples to qpos[jnt_qpos_adr] = quaternion w component.
    let mjcf = r#"
    <mujoco model="dt28_ball">
        <option gravity="0 0 0" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <joint name="J_ball" type="ball"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <tendon>
            <fixed name="T1"><joint joint="J_ball" coef="3.0"/></fixed>
        </tendon>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Ball joint at identity quaternion: qpos = [1, 0, 0, 0]
    // Tendon length = coef * qpos[jnt_qpos_adr] = 3.0 * 1.0 = 3.0
    let jnt_id = 0;
    let qpos_adr = model.jnt_qpos_adr[jnt_id];
    assert_relative_eq!(data.qpos[qpos_adr], 1.0, epsilon = 1e-12); // quaternion w
    assert_relative_eq!(data.ten_length[0], 3.0, epsilon = 1e-12);
    assert_relative_eq!(model.tendon_length0[0], 3.0, epsilon = 1e-12);

    // Jacobian: dL/dqvel at dof_adr = coef = 3.0 (first angular velocity DOF)
    let dof_adr = model.jnt_dof_adr[jnt_id];
    assert_relative_eq!(data.ten_J[0][dof_adr], 3.0, epsilon = 1e-12);

    // Other DOFs of the ball joint should be zero in the Jacobian
    assert_relative_eq!(data.ten_J[0][dof_adr + 1], 0.0, epsilon = 1e-12);
    assert_relative_eq!(data.ten_J[0][dof_adr + 2], 0.0, epsilon = 1e-12);
}

// ============================================================================
// Test 3: Free joint in fixed tendon — couples to first position component
// ============================================================================

#[test]
fn fixed_tendon_free_joint_length() {
    // Free joint: qpos = [x, y, z, qw, qx, qy, qz].
    // Fixed tendon couples to qpos[jnt_qpos_adr] = x position.
    let mjcf = r#"
    <mujoco model="dt28_free">
        <option gravity="0 0 0" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="2 0 1">
                <freejoint name="J_free"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <tendon>
            <fixed name="T1"><joint joint="J_free" coef="1.5"/></fixed>
        </tendon>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    let jnt_id = 0;
    let qpos_adr = model.jnt_qpos_adr[jnt_id];
    let x_pos = data.qpos[qpos_adr];

    // Tendon length = coef * qpos[jnt_qpos_adr] = 1.5 * x_pos
    assert_relative_eq!(data.ten_length[0], 1.5 * x_pos, epsilon = 1e-12);

    // tendon_length0 should match (computed from qpos0)
    let x_pos0 = model.qpos0[qpos_adr];
    assert_relative_eq!(model.tendon_length0[0], 1.5 * x_pos0, epsilon = 1e-12);

    // Jacobian: dL/dqvel at dof_adr = coef = 1.5 (first linear velocity DOF)
    let dof_adr = model.jnt_dof_adr[jnt_id];
    assert_relative_eq!(data.ten_J[0][dof_adr], 1.5, epsilon = 1e-12);

    // Other DOFs of the free joint should be zero in the Jacobian
    for i in 1..6 {
        assert_relative_eq!(data.ten_J[0][dof_adr + i], 0.0, epsilon = 1e-12);
    }

    // Modify x position and recompute
    data.qpos[qpos_adr] = 5.0;
    data.forward(&model).expect("forward failed");
    assert_relative_eq!(data.ten_length[0], 1.5 * 5.0, epsilon = 1e-12);
}

// ============================================================================
// Test 4: Hinge + Ball in one tendon — verifies qpos/dof address divergence
// ============================================================================

#[test]
fn fixed_tendon_hinge_plus_ball_address_divergence() {
    // With a hinge (nq=1, nv=1) before the ball joint (nq=4, nv=3),
    // the ball joint has qpos_adr=1 but dof_adr=1. Still equal because hinge
    // has nq==nv==1. To get true divergence, need a ball joint before another.
    // Use two bodies: first with ball (qpos_adr=0, dof_adr=0, nq=4),
    // second with hinge (qpos_adr=4, dof_adr=3 — DIFFERENT!).
    let mjcf = r#"
    <mujoco model="dt28_divergence">
        <option gravity="0 0 0" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <joint name="J_ball" type="ball"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
            <body name="b2" pos="1 0 1">
                <joint name="J_hinge" type="hinge" axis="0 1 0"/>
                <geom type="capsule" size="0.05 0.3" mass="1.0"/>
            </body>
        </worldbody>
        <tendon>
            <fixed name="T1">
                <joint joint="J_ball" coef="2.0"/>
                <joint joint="J_hinge" coef="3.0"/>
            </fixed>
        </tendon>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward failed");

    // Ball joint: nq=4, nv=3. qpos_adr=0, dof_adr=0.
    // Hinge joint: nq=1, nv=1. qpos_adr=4, dof_adr=3.
    // This is the key case: the hinge's qpos_adr (4) != dof_adr (3)
    // because the preceding ball joint has nq=4 but nv=3.
    let ball_jnt: usize = model
        .jnt_name
        .iter()
        .position(|n| n.as_deref() == Some("J_ball"))
        .unwrap();
    let hinge_jnt: usize = model
        .jnt_name
        .iter()
        .position(|n| n.as_deref() == Some("J_hinge"))
        .unwrap();

    let hinge_qpos_adr = model.jnt_qpos_adr[hinge_jnt];
    let hinge_dof_adr = model.jnt_dof_adr[hinge_jnt];

    // After ball joint (nq=4, nv=3): hinge qpos_adr=4, dof_adr=3
    assert_ne!(
        hinge_qpos_adr, hinge_dof_adr,
        "Hinge after ball should have divergent qpos/dof addresses"
    );

    // Ball at identity: qpos[0] = 1.0 (quaternion w)
    // Hinge at zero: qpos[4] = 0.0
    // Tendon length = 2.0 * 1.0 + 3.0 * 0.0 = 2.0
    assert_relative_eq!(data.ten_length[0], 2.0, epsilon = 1e-12);
    assert_relative_eq!(model.tendon_length0[0], 2.0, epsilon = 1e-12);

    // Now set hinge to 0.5 rad
    data.qpos[hinge_qpos_adr] = 0.5;
    data.forward(&model).expect("forward failed");

    // Tendon length = 2.0 * 1.0 + 3.0 * 0.5 = 3.5
    assert_relative_eq!(data.ten_length[0], 3.5, epsilon = 1e-12);

    // Jacobian entries: ball dof at 0, hinge dof at 3
    let ball_dof_adr = model.jnt_dof_adr[ball_jnt];
    assert_relative_eq!(data.ten_J[0][ball_dof_adr], 2.0, epsilon = 1e-12);
    assert_relative_eq!(data.ten_J[0][hinge_dof_adr], 3.0, epsilon = 1e-12);
}

// ============================================================================
// Test 5: Velocity computation consistency (J * qvel == expected)
// ============================================================================

#[test]
fn fixed_tendon_ball_velocity_consistency() {
    let mjcf = r#"
    <mujoco model="dt28_ball_vel">
        <option gravity="0 0 0" timestep="0.001"/>
        <worldbody>
            <body name="b1" pos="0 0 1">
                <joint name="J_ball" type="ball"/>
                <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
        </worldbody>
        <tendon>
            <fixed name="T1"><joint joint="J_ball" coef="2.0"/></fixed>
        </tendon>
    </mujoco>
    "#;
    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Set angular velocity in the first DOF
    let dof_adr = model.jnt_dof_adr[0];
    data.qvel[dof_adr] = 1.0; // wx = 1.0 rad/s

    data.forward(&model).expect("forward failed");

    // Tendon velocity = J * qvel = coef * qvel[dof_adr] = 2.0 * 1.0 = 2.0
    let ten_vel: f64 = (0..model.nv).map(|d| data.ten_J[0][d] * data.qvel[d]).sum();
    assert_relative_eq!(ten_vel, 2.0, epsilon = 1e-12);
}
