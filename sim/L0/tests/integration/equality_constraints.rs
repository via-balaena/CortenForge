//! Phase 1.1: Equality Constraint Tests for MuJoCo Parity.
//!
//! These tests verify the correctness of equality constraint implementation:
//! - **Connect**: Ball-and-socket constraint (3 DOF position)
//! - **Weld**: Fixed frame constraint (6 DOF pose)
//! - **Joint**: Polynomial joint coupling
//!
//! Reference: MUJOCO_PARITY_SPEC.md Phase 1.1
//!
//! # Test Strategy
//!
//! Equality constraints enforce holonomic relationships between bodies or joints.
//! We verify that:
//! 1. Constraint violation stays bounded (penalty method)
//! 2. Constrained systems behave physically correctly
//! 3. Multiple constraints interact properly

use approx::assert_relative_eq;
use sim_mjcf::load_model;

// ============================================================================
// Connect Constraint Tests
// ============================================================================

/// Test: Connect constraint keeps two bodies attached.
///
/// A connect constraint acts like a ball-and-socket joint, allowing
/// rotation but preventing relative translation at the anchor point.
#[test]
fn test_connect_constraint_maintains_attachment() {
    // Two free bodies connected at their origins
    let mjcf = r#"
        <mujoco model="connect_test">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="body1" pos="0 0 1">
                    <freejoint name="j1"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="body2" pos="0 0 0.5">
                    <freejoint name="j2"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="body1" body2="body2" anchor="0 0 -0.5"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Verify constraint was loaded
    assert_eq!(model.neq, 1);

    // Get initial separation
    let _initial_sep = (data.xpos[1] - data.xpos[2]).norm();

    // Step simulation
    for _ in 0..1000 {
        data.step(&model);
    }

    // Check that the anchor point on body1 stays close to body2
    // The anchor is at (0, 0, -0.5) in body1's frame
    let anchor_world = data.xpos[1] + data.xquat[1] * nalgebra::Vector3::new(0.0, 0.0, -0.5);
    let body2_pos = data.xpos[2];
    let separation = (anchor_world - body2_pos).norm();

    // With penalty method, expect small but non-zero violation
    assert!(
        separation < 0.05,
        "Connect constraint violation too large: {separation}"
    );
}

/// Test: Connect constraint to world (fixed point in space).
#[test]
fn test_connect_constraint_to_world() {
    // Single body connected to world origin
    let mjcf = r#"
        <mujoco model="connect_world">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="pendulum" pos="0 0 1">
                    <freejoint name="free"/>
                    <geom type="sphere" size="0.1" pos="0 0 -0.5" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="pendulum" anchor="0 0 0"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Step simulation - body should swing like a pendulum
    for _ in 0..2000 {
        data.step(&model);
    }

    // The body origin should stay near world origin (0,0,0)
    // Initial position was (0, 0, 1), but constraint anchor is at body origin
    let body_origin = data.xpos[1];
    let distance_from_origin = body_origin.norm();

    assert!(
        distance_from_origin < 0.1,
        "Body should stay near world origin, got distance {distance_from_origin}"
    );
}

/// Test: Connect constraint with offset anchor.
#[test]
fn test_connect_constraint_offset_anchor() {
    // Body with anchor at a specific offset
    let mjcf = r#"
        <mujoco model="connect_offset">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="body1" pos="1 0 0">
                    <freejoint/>
                    <geom type="box" size="0.2 0.1 0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="body1" anchor="-0.2 0 0"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // The anchor at (-0.2, 0, 0) in body frame should be at world origin
    // Body position is (1, 0, 0), so anchor world = (1, 0, 0) + (-0.2, 0, 0) = (0.8, 0, 0)
    // Constraint will pull anchor toward (0, 0, 0)

    for _ in 0..5000 {
        data.step(&model);
    }

    // After settling, anchor point should be near world origin
    let anchor_world = data.xpos[1] + data.xquat[1] * nalgebra::Vector3::new(-0.2, 0.0, 0.0);
    let distance = anchor_world.norm();

    assert!(
        distance < 0.05,
        "Anchor should settle at world origin, got distance {distance}"
    );
}

// ============================================================================
// Weld Constraint Tests
// ============================================================================

/// Test: Weld constraint locks relative pose.
///
/// A weld constraint is stronger than connect - it locks both position
/// AND orientation, creating a rigid connection.
#[test]
fn test_weld_constraint_locks_pose() {
    // Two bodies welded together
    let mjcf = r#"
        <mujoco model="weld_test">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="base" pos="0 0 1">
                    <freejoint/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
                </body>
                <body name="arm" pos="0 0 0.5">
                    <freejoint/>
                    <geom type="capsule" size="0.05 0.2" mass="0.5"/>
                </body>
            </worldbody>
            <equality>
                <weld body1="base" body2="arm" anchor="0 0 -0.1"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Record initial relative orientation
    let initial_rel_quat = data.xquat[1].inverse() * data.xquat[2];

    // Step simulation
    for _ in 0..1000 {
        data.step(&model);
    }

    // Check relative orientation stayed similar
    let final_rel_quat = data.xquat[1].inverse() * data.xquat[2];
    let quat_diff = initial_rel_quat.inverse() * final_rel_quat;

    // Quaternion difference should be near identity (w ≈ 1)
    let angle_error = 2.0
        * quat_diff
            .quaternion()
            .i
            .atan2(quat_diff.quaternion().w)
            .abs();
    assert!(
        angle_error < 0.2,
        "Weld constraint should maintain orientation, got angle error {angle_error} rad"
    );
}

/// Test: Weld constraint to world (fully fixed body).
#[test]
fn test_weld_constraint_to_world_fixed() {
    // Body welded to world should not move
    let mjcf = r#"
        <mujoco model="weld_world">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="fixed" pos="0 0 1">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <weld body1="fixed"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    let initial_pos = data.xpos[1];

    // Step simulation - body should stay put despite gravity
    for _ in 0..2000 {
        data.step(&model);
    }

    let final_pos = data.xpos[1];
    let displacement = (final_pos - initial_pos).norm();

    assert!(
        displacement < 0.05,
        "Welded body should stay in place, moved {displacement}"
    );
}

// ============================================================================
// Joint Equality Constraint Tests
// ============================================================================

/// Test: Joint coupling with 1:1 ratio (mimic joint).
///
/// When joint2 = joint1 (1:1 coupling), both joints should move together.
///
/// Note: Uses softer constraint parameters (solref="0.1 1.0") for stability with
/// explicit integration. The default MuJoCo solref is designed for their implicit
/// PGS solver, which handles stiff constraints better than explicit penalty.
#[test]
fn test_joint_equality_mimic() {
    // Two hinge joints coupled 1:1
    // Add damping to both joints for stability
    let mjcf = r#"
        <mujoco model="joint_mimic">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="link1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0" damping="1"/>
                    <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                    <body name="link2" pos="0 0 -0.7">
                        <joint name="j2" type="hinge" axis="0 1 0" damping="1"/>
                        <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                    </body>
                </body>
            </worldbody>
            <equality>
                <joint joint1="j1" joint2="j2" polycoef="0 1" solref="0.1 1.0"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Start with joint1 displaced
    data.qpos[0] = 0.5;

    // Step simulation
    for _ in 0..2000 {
        data.step(&model);
    }

    // Joints should be approximately equal
    let q1 = data.qpos[0];
    let q2 = data.qpos[1];
    let diff = (q2 - q1).abs();

    assert!(
        diff < 0.1,
        "Mimic joints should have similar angles, diff = {diff}"
    );
}

/// Test: Joint coupling with gear ratio.
///
/// joint2 = 2 * joint1 (2:1 gear ratio)
#[test]
fn test_joint_equality_gear_ratio() {
    let mjcf = r#"
        <mujoco model="joint_gear">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="link1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0" damping="1"/>
                    <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                </body>
                <body name="link2" pos="1 0 1">
                    <joint name="j2" type="hinge" axis="0 1 0" damping="1"/>
                    <geom type="capsule" size="0.05 0.3" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <joint joint1="j1" joint2="j2" polycoef="0 2"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Start with joint1 at 0.3 rad
    data.qpos[0] = 0.3;
    data.qpos[1] = 0.6; // Start at target ratio

    // Step simulation
    for _ in 0..3000 {
        data.step(&model);
    }

    // joint2 should be approximately 2 * joint1
    let q1 = data.qpos[0];
    let q2 = data.qpos[1];
    let expected_q2 = 2.0 * q1;
    let error = (q2 - expected_q2).abs();

    assert!(
        error < 0.15,
        "Gear ratio should be maintained: q1={q1}, q2={q2}, expected q2={expected_q2}"
    );
}

/// Test: Joint lock (single joint fixed to constant).
///
/// When joint2 is not specified, joint1 is locked to polycoef[0].
#[test]
fn test_joint_equality_lock() {
    let mjcf = r#"
        <mujoco model="joint_lock">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="pendulum" pos="0 0 1">
                    <joint name="hinge" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" size="0.05 0.4" pos="0 0 -0.4" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <joint joint1="hinge" polycoef="0.5"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Start displaced from lock position
    data.qpos[0] = 0.0;

    // Step simulation
    for _ in 0..5000 {
        data.step(&model);
    }

    // Joint should settle near 0.5 rad
    let q = data.qpos[0];
    let error = (q - 0.5).abs();

    assert!(
        error < 0.1,
        "Locked joint should settle at 0.5 rad, got {q}"
    );
}

// ============================================================================
// Multiple Constraint Tests
// ============================================================================

/// Test: Multiple connect constraints (closed kinematic chain).
///
/// Four bodies forming a parallelogram with connect constraints.
#[test]
fn test_multiple_connect_constraints() {
    let mjcf = r#"
        <mujoco model="multi_connect">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <freejoint/>
                    <geom type="sphere" size="0.05" mass="0.5"/>
                </body>
                <body name="b2" pos="1 0 1">
                    <freejoint/>
                    <geom type="sphere" size="0.05" mass="0.5"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="b1" anchor="0 0 0"/>
                <connect body1="b2" anchor="0 0 0" body2="b1"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Verify multiple constraints loaded
    assert_eq!(model.neq, 2);

    // Step simulation
    for _ in 0..1000 {
        data.step(&model);
    }

    // b1 should be near world origin
    let b1_dist = data.xpos[1].norm();
    assert!(b1_dist < 0.1, "b1 should be near origin, got {b1_dist}");

    // b2 should be near b1
    let b2_to_b1 = (data.xpos[2] - data.xpos[1]).norm();
    assert!(b2_to_b1 < 0.1, "b2 should be near b1, got {b2_to_b1}");
}

// ============================================================================
// Inactive Constraint Tests
// ============================================================================

/// Test: Inactive constraint has no effect.
#[test]
fn test_inactive_constraint_ignored() {
    // Same as connect test but with active="false"
    let mjcf = r#"
        <mujoco model="inactive_connect">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="ball" pos="0 0 1">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="ball" active="false"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Constraint exists but is inactive
    assert_eq!(model.neq, 1);
    assert!(!model.eq_active[0]);

    // Step simulation - ball should fall freely
    for _ in 0..1000 {
        data.step(&model);
    }

    // Ball should have fallen significantly (not constrained)
    let z = data.xpos[1].z;
    assert!(
        z < 0.5,
        "Inactive constraint should allow free fall, z = {z}"
    );
}

// ============================================================================
// Edge Cases
// ============================================================================

/// Test: Zero constraints doesn't crash.
#[test]
fn test_no_equality_constraints() {
    let mjcf = r#"
        <mujoco model="no_constraints">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="ball" pos="0 0 1">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    assert_eq!(model.neq, 0);

    // Should not crash
    for _ in 0..100 {
        data.step(&model);
    }
}

/// Test: Constraint with solver parameters.
#[test]
fn test_constraint_with_solref() {
    let mjcf = r#"
        <mujoco model="solref_test">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="ball" pos="0 0 1">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="ball" solref="0.02 1.0"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Verify solref was loaded
    assert_relative_eq!(model.eq_solref[0][0], 0.02, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solref[0][1], 1.0, epsilon = 1e-10);

    // Should not crash
    for _ in 0..100 {
        data.step(&model);
    }
}

// ============================================================================
// Error Handling Tests
// ============================================================================

/// Test: Invalid body name in connect constraint returns error.
#[test]
fn test_invalid_body_name_returns_error() {
    let mjcf = r#"
        <mujoco model="invalid_body">
            <worldbody>
                <body name="real_body" pos="0 0 1">
                    <freejoint/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="nonexistent_body"/>
            </equality>
        </mujoco>
    "#;

    let result = load_model(mjcf);
    assert!(result.is_err(), "Should fail with unknown body name");
    let err_msg = result.unwrap_err().to_string();
    assert!(
        err_msg.contains("nonexistent_body") || err_msg.contains("unknown"),
        "Error should mention the invalid body name, got: {err_msg}"
    );
}

/// Test: Invalid joint name in joint equality returns error.
#[test]
fn test_invalid_joint_name_returns_error() {
    let mjcf = r#"
        <mujoco model="invalid_joint">
            <worldbody>
                <body name="body1" pos="0 0 1">
                    <joint name="real_joint" type="hinge"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
            <equality>
                <joint joint1="fake_joint"/>
            </equality>
        </mujoco>
    "#;

    let result = load_model(mjcf);
    assert!(result.is_err(), "Should fail with unknown joint name");
}

// ============================================================================
// Weld Relpose Test
// ============================================================================

/// Test: Weld constraint with custom relative pose.
///
/// The relpose parameter specifies the target relative pose between bodies.
#[test]
fn test_weld_constraint_with_relpose() {
    // Two bodies where body2 should maintain a 90-degree rotation relative to body1
    let mjcf = r#"
        <mujoco model="weld_relpose">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="base" pos="0 0 1">
                    <freejoint/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
                </body>
                <body name="rotated" pos="0.5 0 1">
                    <freejoint/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <weld body1="base" body2="rotated" relpose="0 0 0 0.707 0 0.707 0"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    // Verify relpose was loaded (quaternion w=0.707, x=0, y=0.707, z=0 ≈ 90° about Y)
    // Note: relpose in eq_data is stored as [anchor.x, anchor.y, anchor.z, qw, qx, qy, qz, ...]
    assert_relative_eq!(model.eq_data[0][3], 0.707, epsilon = 0.01);
    assert_relative_eq!(model.eq_data[0][5], 0.707, epsilon = 0.01);

    // Step simulation
    for _ in 0..2000 {
        data.step(&model);
    }

    // Bodies should maintain relative orientation (approximately 90° about Y)
    let rel_quat = data.xquat[1].inverse() * data.xquat[2];
    let target_quat = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        0.707, 0.0, 0.707, 0.0,
    ));
    let error_quat = rel_quat * target_quat.inverse();

    // Check angle error is small
    let angle_error = 2.0
        * error_quat
            .quaternion()
            .i
            .atan2(error_quat.quaternion().w)
            .abs();
    assert!(
        angle_error < 0.3,
        "Weld relpose should be maintained, got angle error {angle_error} rad"
    );
}
