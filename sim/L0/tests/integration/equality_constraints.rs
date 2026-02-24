//! Equality Constraint Integration Tests.
//!
//! These tests verify the correctness of equality constraint enforcement:
//! - **Connect**: Ball-and-socket constraint (3 DOF position lock)
//! - **Weld**: Fixed frame constraint (6 DOF pose lock)
//! - **Joint**: Polynomial joint coupling (θ₂ = poly(θ₁))
//! - **Distance**: Scalar distance constraint between geom centers (1 DOF)
//!
//! # Implementation Notes
//!
//! Equality constraints use **penalty method with Baumgarte stabilization**:
//! ```text
//! τ = -k * position_error - b * velocity_error
//! ```
//!
//! Where k (stiffness) and b (damping) are derived from solref parameters:
//! ```text
//! k = 1 / timeconst²
//! b = 2 * dampratio / timeconst
//! ```
//!
//! # Key Design Decisions
//!
//! 1. **No explicit reaction torque for joint coupling**: In articulated body
//!    dynamics, applying τ₂ to joint2 naturally propagates forces through the
//!    mass matrix. Adding τ₁ = -dq₂/dq₁ · τ₂ would double-count, causing explosion.
//!    (See Featherstone, "Rigid Body Dynamics Algorithms", Section 7.3)
//!
//! 2. **Softer defaults than MuJoCo**: MuJoCo's default solref=[0.02, 1.0] is tuned
//!    for their implicit PGS solver. For explicit penalty integration, we recommend
//!    solref="0.05 1.0" to "0.1 1.0" for stability with articulated chains.
//!
//! # Test Strategy
//!
//! Equality constraints enforce holonomic relationships between bodies or joints.
//! We verify that:
//! 1. Constraint violation stays bounded (penalty method effectiveness)
//! 2. Constrained systems behave physically correctly (energy, stability)
//! 3. Multiple constraints interact properly (no interference)
//! 4. Error handling works for invalid body/joint names

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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
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
        data.step(&model).expect("step failed");
    }
}

// ============================================================================
// Solimp (Impedance) Tests
// ============================================================================

/// Test: solimp parameters are loaded and affect constraint behavior.
///
/// Lower impedance (d0 close to 0) should produce a weaker constraint,
/// allowing more violation than higher impedance (d0 close to 1).
///
/// Physics: In steady state, `d * k * violation ≈ m * g`, so
/// `violation ∝ 1/d`. With d_strong ≈ 0.95 and d_weak ≈ 0.1, the weak
/// constraint should allow roughly `0.95/0.1 ≈ 9.5×` more violation.
/// We test for at least 3× to account for dynamic effects (damped
/// oscillation rather than true static equilibrium).
#[test]
fn test_solimp_affects_constraint_strength() {
    // Strong impedance: d0=0.95, d_width=0.99
    let mjcf_strong = r#"
        <mujoco model="solimp_strong">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="ball" pos="0 0 1">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="ball" anchor="0 0 0"
                         solimp="0.95 0.99 0.001 0.5 2"/>
            </equality>
        </mujoco>
    "#;

    // Weak impedance: d0=0.1, d_width=0.2
    let mjcf_weak = r#"
        <mujoco model="solimp_weak">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="ball" pos="0 0 1">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="ball" anchor="0 0 0"
                         solimp="0.1 0.2 0.001 0.5 2"/>
            </equality>
        </mujoco>
    "#;

    let model_strong = load_model(mjcf_strong).expect("should load strong");
    let model_weak = load_model(mjcf_weak).expect("should load weak");

    // Verify solimp was loaded correctly
    assert_relative_eq!(model_strong.eq_solimp[0][0], 0.95, epsilon = 1e-10);
    assert_relative_eq!(model_weak.eq_solimp[0][0], 0.1, epsilon = 1e-10);

    let mut data_strong = model_strong.make_data();
    let mut data_weak = model_weak.make_data();

    // Step both simulations
    for _ in 0..500 {
        data_strong.step(&model_strong).expect("step failed");
        data_weak.step(&model_weak).expect("step failed");
    }

    // Measure constraint violation (distance from world origin)
    let violation_strong = data_strong.xpos[1].norm();
    let violation_weak = data_weak.xpos[1].norm();

    // 1. Ordering: weak impedance must allow strictly more violation
    assert!(
        violation_weak > violation_strong,
        "Weak solimp (d0=0.1) should allow more violation ({violation_weak}) \
         than strong solimp (d0=0.95, violation={violation_strong})"
    );

    // 2. Magnitude: the ratio should reflect the impedance difference.
    //    Steady-state: violation ∝ 1/d, so ratio ≈ d_strong/d_weak = 9.5.
    //    We allow a wide margin (3×) because the system is dynamic, force
    //    clamping may engage, and impedance is violation-dependent.
    let ratio = violation_weak / violation_strong.max(1e-15);
    assert!(
        ratio > 3.0,
        "Violation ratio should reflect impedance difference: \
         got {ratio:.1}× (violation_weak={violation_weak:.6}, \
         violation_strong={violation_strong:.6}), expected > 3×"
    );
}

/// Test: solimp with default values works identically to pre-existing behavior.
///
/// The default solimp [0.9, 0.95, 0.001, 0.5, 2.0] should produce impedance
/// close to 0.9 for small violations, which is near-unity scaling.
#[test]
fn test_solimp_default_values_loaded() {
    let mjcf = r#"
        <mujoco model="solimp_default">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="ball" pos="0 0 1">
                    <freejoint/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="ball" anchor="0 0 0"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // Default solimp should be MuJoCo defaults
    assert_relative_eq!(model.eq_solimp[0][0], 0.9, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][1], 0.95, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][2], 0.001, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][3], 0.5, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][4], 2.0, epsilon = 1e-10);

    let mut data = model.make_data();

    // Should still work (not crash, constraint still enforced)
    for _ in 0..1000 {
        data.step(&model).expect("step failed");
    }

    let violation = data.xpos[1].norm();
    assert!(
        violation < 0.1,
        "Default solimp should still enforce constraint, violation={violation}"
    );
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

    // Debug: print initial state and constraint forces
    eprintln!("=== Initial state ===");
    eprintln!("nv={}, nq={}", model.nv, model.nq);
    eprintln!("qpos = {:?}", data.qpos.as_slice());
    eprintln!("qvel = {:?}", data.qvel.as_slice());

    // Step simulation with diagnostics
    for step in 0..10 {
        // Run forward to compute constraint forces
        data.forward(&model).expect("forward failed");

        eprintln!("\n=== Step {} ===", step);
        eprintln!("qfrc_constraint = {:?}", data.qfrc_constraint.as_slice());
        eprintln!("qvel = {:?}", data.qvel.as_slice());
        eprintln!("qacc = {:?}", data.qacc.as_slice());

        // Check for numerical issues
        let max_force = data
            .qfrc_constraint
            .iter()
            .map(|x| x.abs())
            .fold(0.0_f64, f64::max);
        let max_vel = data.qvel.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
        let max_acc = data.qacc.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);

        eprintln!(
            "max_force={:.2e}, max_vel={:.2e}, max_acc={:.2e}",
            max_force, max_vel, max_acc
        );

        if let Err(e) = data.step(&model) {
            panic!("Step {} failed: {}", step, e);
        }
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

// ============================================================================
// Distance Constraint Tests
// ============================================================================

/// Test: Distance constraint maintains separation between two free bodies.
///
/// Two free bodies start 1.0 apart along Z. A distance constraint with
/// `distance="1.0"` should keep them approximately 1.0 apart under gravity.
#[test]
fn test_distance_constraint_maintains_separation() {
    let mjcf = r#"
        <mujoco model="distance_test">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="body1" pos="0 0 1.5">
                    <freejoint name="j1"/>
                    <geom name="g1" type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="body2" pos="0 0 0.5">
                    <freejoint name="j2"/>
                    <geom name="g2" type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <distance geom1="g1" geom2="g2" distance="1.0"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    assert_eq!(model.neq, 1);
    assert_relative_eq!(model.eq_data[0][0], 1.0, epsilon = 1e-10);

    for _ in 0..1000 {
        data.step(&model).expect("step failed");
    }

    let dist = (data.geom_xpos[0] - data.geom_xpos[1]).norm();
    let error = (dist - 1.0).abs();
    assert!(
        error < 0.1,
        "Distance error too large: {error}, actual dist: {dist}"
    );
}

/// Test: Distance constraint from a worldbody geom.
///
/// A single free body at (2,0,0) with `distance="2.0"` from a geom on the
/// worldbody (at origin) should maintain distance ~2.0 (no gravity).
#[test]
fn test_distance_constraint_to_world() {
    let mjcf = r#"
        <mujoco model="distance_world">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <geom name="world_geom" type="sphere" size="0.1"/>
                <body name="body1" pos="2 0 0">
                    <freejoint/>
                    <geom name="g1" type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <distance geom1="g1" geom2="world_geom" distance="2.0"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    for _ in 0..1000 {
        data.step(&model).expect("step failed");
    }

    // Distance from g1 to world_geom (at origin)
    let dist = (data.geom_xpos[1] - data.geom_xpos[0]).norm();
    let error = (dist - 2.0).abs();
    assert!(
        error < 0.1,
        "Distance from world geom error: {error}, actual: {dist}"
    );
}

/// Test: Distance constraint with auto-computed distance (distance omitted).
///
/// Two bodies start 1.0 apart; no explicit `distance` attribute means the
/// model builder computes the initial distance as the target.
#[test]
fn test_distance_constraint_auto_distance() {
    let mjcf = r#"
        <mujoco model="distance_auto">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 1.5">
                    <freejoint/>
                    <geom name="g1" type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="b2" pos="0 0 0.5">
                    <freejoint/>
                    <geom name="g2" type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <distance geom1="g1" geom2="g2"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // Target should be ~1.0 (computed from initial configuration)
    assert_relative_eq!(model.eq_data[0][0], 1.0, epsilon = 0.01);

    let mut data = model.make_data();
    for _ in 0..1000 {
        data.step(&model).expect("step failed");
    }

    let dist = (data.geom_xpos[0] - data.geom_xpos[1]).norm();
    let error = (dist - 1.0).abs();
    assert!(error < 0.1, "Auto-distance error: {error}, actual: {dist}");
}

/// Test: Distance constraint with zero target acts like a connect constraint.
///
/// Two bodies start 1.0 apart with `distance="0.0"`. The constraint should
/// pull them together over time.
#[test]
fn test_distance_constraint_zero_target() {
    let mjcf = r#"
        <mujoco model="distance_zero">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="0 0 0.5">
                    <freejoint/>
                    <geom name="g1" type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="b2" pos="0 0 -0.5">
                    <freejoint/>
                    <geom name="g2" type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <distance geom1="g1" geom2="g2" distance="0.0"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();

    for _ in 0..2000 {
        data.step(&model).expect("step failed");
    }

    let dist = (data.geom_xpos[0] - data.geom_xpos[1]).norm();
    assert!(
        dist < 0.15,
        "Zero-distance constraint should bring geoms together, got {dist}"
    );
}

/// Test: Distance constraint with geom2 omitted (world origin sentinel).
///
/// When geom2 is not specified, the distance is measured from the world origin.
/// This exercises the `usize::MAX` sentinel path in both `builder/` and `constraint/`.
#[test]
fn test_distance_constraint_geom2_omitted() {
    let mjcf = r#"
        <mujoco model="distance_no_geom2">
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body name="b1" pos="1.5 0 0">
                    <freejoint/>
                    <geom name="g1" type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <distance geom1="g1" distance="1.5"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.neq, 1);
    assert_eq!(model.eq_obj2id[0], usize::MAX);
    assert_relative_eq!(model.eq_data[0][0], 1.5, epsilon = 1e-10);

    let mut data = model.make_data();
    for _ in 0..1000 {
        data.step(&model).expect("step failed");
    }

    // Geom should stay at distance 1.5 from world origin
    let dist = data.geom_xpos[0].norm();
    let error = (dist - 1.5).abs();
    assert!(
        error < 0.1,
        "Distance from world origin error: {error}, actual: {dist}"
    );
}

/// Test: Invalid geom name in distance constraint returns error.
#[test]
fn test_distance_invalid_geom_name_returns_error() {
    let mjcf = r#"
        <mujoco model="invalid_geom">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <freejoint/>
                    <geom name="g1" type="sphere" size="0.1"/>
                </body>
            </worldbody>
            <equality>
                <distance geom1="nonexistent_geom"/>
            </equality>
        </mujoco>
    "#;

    let result = load_model(mjcf);
    assert!(result.is_err(), "Should fail with unknown geom name");
    let err_msg = result.unwrap_err().to_string();
    assert!(
        err_msg.contains("nonexistent_geom") || err_msg.contains("unknown"),
        "Error should mention the invalid geom name, got: {err_msg}"
    );
}

/// Test: Inactive distance constraint has no effect.
///
/// A body under gravity with an inactive distance constraint should fall freely.
#[test]
fn test_distance_inactive_ignored() {
    let mjcf = r#"
        <mujoco model="distance_inactive">
            <option gravity="0 0 -9.81" timestep="0.001"/>
            <worldbody>
                <geom name="floor" type="plane" size="10 10 0.1" contype="0" conaffinity="0"/>
                <body name="b1" pos="0 0 1">
                    <freejoint/>
                    <geom name="g1" type="sphere" size="0.1" mass="1.0" contype="0" conaffinity="0"/>
                </body>
            </worldbody>
            <equality>
                <distance geom1="g1" geom2="floor" distance="1.0" active="false"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert!(!model.eq_active[0]);

    let mut data = model.make_data();
    for _ in 0..1000 {
        data.step(&model).expect("step failed");
    }

    // Ball should fall freely (constraint inactive). After 1s at g=9.81, z = 1 - 0.5*9.81*1 ≈ -3.9
    let z = data.geom_xpos[1].z;
    assert!(
        z < 0.5,
        "Inactive distance constraint should allow free fall, z = {z}"
    );
}
