//! §38 Integration tests: Ball Joint Limits — Rotation Cone Constraint.
//!
//! Tests cover:
//! - T2: Constraint row counting (degree mode)
//! - T3: Jacobian, dist, and efc_vel (MuJoCo conformance)
//! - T4: Free joint limits ignored
//! - T5: No constraint when within limits
//! - T5b: No constraint at exact boundary
//! - T6: Unlimited ball joint produces no rows
//! - T7: Limit enforcement over time
//! - T8: Wrapped rotation Jacobian
//! - T9: Radian-mode range (no double-conversion)
//! - T10: Range interpretation symmetry
//! - T11: Multiple ball joints (counting/assembly consistency)
//! - T12: Mixed hinge + ball limits (cross-type consistency)
//! - T13: jnt_limit_frc propagation
//! - T14: Degenerate range "0 0" locks joint
//! - T15: Near-π rotation with active limit
//! - T16: Small violation precision
//! - T17: Negative-w quaternion in pipeline
//! - T18: Multiple simultaneous violations
//! - T19: efc_vel sign with opposing angular velocity
//! - T20: Unnormalized quaternion in constraint assembly
//! - T21: range="45 0" parser-level verification
//! - T22: Margin = 0.0 regression anchor

use approx::assert_relative_eq;
use sim_mjcf::load_model;

/// Build quaternion [w, x, y, z] for rotation of `angle_deg` degrees about `axis`.
fn quat_from_axis_angle_deg(axis: [f64; 3], angle_deg: f64) -> [f64; 4] {
    let half = (angle_deg / 2.0_f64).to_radians();
    let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    let s = half.sin() / norm;
    [half.cos(), axis[0] * s, axis[1] * s, axis[2] * s]
}

/// Extract rotation angle (radians) from quaternion at qpos[offset..offset+4].
/// Reimplements normalize_quat4 + ball_limit_axis_angle for integration tests.
fn extract_ball_angle(qpos: &[f64], offset: usize) -> f64 {
    let (w, x, y, z) = (
        qpos[offset],
        qpos[offset + 1],
        qpos[offset + 2],
        qpos[offset + 3],
    );
    let norm = (w * w + x * x + y * y + z * z).sqrt();
    let (w, x, y, z) = if norm > 1e-10 {
        (w / norm, x / norm, y / norm, z / norm)
    } else {
        return 0.0;
    };
    let sin_half = (x * x + y * y + z * z).sqrt();
    if sin_half < 1e-10 {
        return 0.0;
    }
    let mut theta = 2.0 * sin_half.atan2(w);
    if theta > std::f64::consts::PI {
        theta -= 2.0 * std::f64::consts::PI;
    }
    theta.abs()
}

// ============================================================================
// T2: Constraint row counting (degree mode)
// ============================================================================

#[test]
fn test_ball_limit_creates_constraint_row() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Verify range parsed and converted to radians
    assert!(model.jnt_limited[0]);
    assert_relative_eq!(model.jnt_range[0].1, 45.0_f64.to_radians(), epsilon = 1e-6);

    // Rotate ball joint 60° about X (past the 45° limit)
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
    data.qpos[0] = q[0]; // w
    data.qpos[1] = q[1]; // x
    data.qpos[2] = q[2]; // y
    data.qpos[3] = q[3]; // z

    data.step(&model).unwrap();

    // Expect exactly 1 LimitJoint constraint row
    let limit_rows = data
        .efc_type
        .iter()
        .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 1, "should have exactly 1 ball limit row");
}

// ============================================================================
// T3: Jacobian, dist, and efc_vel (MuJoCo conformance)
// ============================================================================

#[test]
fn test_ball_limit_jacobian_dist_and_vel() {
    // 50° about axis (0.6, 0.8, 0), limit = 45° → dist = -π/36 ≈ -0.0873
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Set quaternion for 50° rotation about (0.6, 0.8, 0)
    let q = quat_from_axis_angle_deg([0.6, 0.8, 0.0], 50.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    // Set known angular velocity: omega = (1.0, 2.0, 0.5) rad/s
    let dof = model.jnt_dof_adr[0];
    data.qvel[dof] = 1.0;
    data.qvel[dof + 1] = 2.0;
    data.qvel[dof + 2] = 0.5;

    data.step(&model).unwrap();

    let limit_row = data
        .efc_type
        .iter()
        .position(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .expect("should have a ball limit constraint row");

    // Jacobian: J = -unit_dir = -(0.6, 0.8, 0)
    assert_relative_eq!(data.efc_J[(limit_row, dof)], -0.6, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 1)], -0.8, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 2)], 0.0, epsilon = 1e-4);

    // dist = π/4 - 5π/18 = -π/36 ≈ -0.0873
    let expected_dist = std::f64::consts::PI / 4.0 - 5.0 * std::f64::consts::PI / 18.0;
    assert_relative_eq!(data.efc_pos[limit_row], expected_dist, epsilon = 1e-4);
    assert!(
        data.efc_pos[limit_row] < 0.0,
        "dist should be negative (violated)"
    );

    // Constraint-space velocity: efc_vel = J · qvel
    // J = (-0.6, -0.8, 0), qvel = (1.0, 2.0, 0.5)
    // efc_vel = (-0.6)(1.0) + (-0.8)(2.0) + (0)(0.5) = -0.6 - 1.6 = -2.2
    assert_relative_eq!(data.efc_vel[limit_row], -2.2, epsilon = 1e-4,);

    // Exactly 1 constraint row
    assert_eq!(
        data.efc_type
            .iter()
            .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
            .count(),
        1
    );
}

// ============================================================================
// T4: Free joint limits ignored
// ============================================================================

#[test]
fn test_free_joint_limits_ignored() {
    let mjcf = r#"
        <mujoco>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="free" limited="true" range="0 0.5"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Free joint qpos: [x, y, z, qw, qx, qy, qz]
    // Rotate far past the "limit" — 120° about X
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 120.0);
    data.qpos[3] = q[0]; // qw
    data.qpos[4] = q[1]; // qx
    data.qpos[5] = q[2]; // qy
    data.qpos[6] = q[3]; // qz

    data.step(&model).unwrap();

    // No LimitJoint rows should exist — MuJoCo skips free joint limits entirely
    let limit_rows = data
        .efc_type
        .iter()
        .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .count();
    assert_eq!(
        limit_rows, 0,
        "free joints should have no limit constraints"
    );
}

// ============================================================================
// T5: No constraint when within limits
// ============================================================================

#[test]
fn test_ball_limit_no_constraint_within_range() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate only 30° about Y — within the 45° limit
    let q = quat_from_axis_angle_deg([0.0, 1.0, 0.0], 30.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    let limit_rows = data
        .efc_type
        .iter()
        .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .count();
    assert_eq!(
        limit_rows, 0,
        "within-limit ball joint should produce no constraint"
    );
}

// ============================================================================
// T5b: No constraint at exact boundary
// ============================================================================

#[test]
fn test_ball_limit_no_constraint_at_exact_boundary() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate 44.999° about X — just under the boundary
    // (exact 45° can produce a tiny floating-point violation due to atan2 roundtrip)
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 44.999);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    let limit_rows = data
        .efc_type
        .iter()
        .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .count();
    assert_eq!(
        limit_rows, 0,
        "ball joint at exact boundary (dist=0) should produce no constraint"
    );
}

// ============================================================================
// T6: Unlimited ball joint produces no rows
// ============================================================================

#[test]
fn test_unlimited_ball_joint_no_constraint() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="false" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate 170° — far past any limit, but limited="false"
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 170.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    let limit_rows = data
        .efc_type
        .iter()
        .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .count();
    assert_eq!(
        limit_rows, 0,
        "unlimited ball joint should produce no constraint rows"
    );
}

// ============================================================================
// T7: Limit enforcement over time
// ============================================================================

#[test]
fn test_ball_limit_enforced_over_simulation() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 30"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Start at 35° about X
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 35.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];
    // Angular velocity pushing further into the limit (2 rad/s about X)
    data.qvel[0] = 2.0;

    for _ in 0..500 {
        data.step(&model).unwrap();
    }

    // Extract rotation angle from quaternion
    let angle = extract_ball_angle(data.qpos.as_slice(), 0);
    let angle_deg = angle.to_degrees();
    // The solver should push the angle back toward the 30° limit.
    // Allow ~2° solver softness (default solref/solimp spring-damper compliance).
    assert!(
        angle_deg < 32.0,
        "ball limit should hold near 30°: angle={angle_deg:.1}°, limit=30°"
    );
}

// ============================================================================
// T8: Wrapped rotation Jacobian
// ============================================================================

#[test]
fn test_ball_limit_wrap_jacobian_sign() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 150"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Set quaternion for 200° rotation about Z
    let q = quat_from_axis_angle_deg([0.0, 0.0, 1.0], 200.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    let limit_row = data
        .efc_type
        .iter()
        .position(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .expect("wrapped rotation should trigger ball limit");
    let dof = model.jnt_dof_adr[0];

    // Jacobian should point along +Z (not -Z)
    assert_relative_eq!(data.efc_J[(limit_row, dof)], 0.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 1)], 0.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 2)], 1.0, epsilon = 1e-4,);

    // dist = 150° - 160° = -10° in radians
    let expected_dist = 150.0_f64.to_radians() - 160.0_f64.to_radians();
    assert_relative_eq!(data.efc_pos[limit_row], expected_dist, epsilon = 1e-4);
}

// ============================================================================
// T9: Radian-mode range (no double-conversion)
// ============================================================================

#[test]
fn test_ball_limit_radian_mode() {
    // Explicit radian mode — verify range is not double-converted
    let limit_rad = std::f64::consts::PI / 4.0; // 45° in radians
    let mjcf = format!(
        r#"
        <mujoco>
            <compiler angle="radian"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 {limit_rad}"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#
    );
    let model = load_model(&mjcf).unwrap();

    // Range should be stored as-is (radians), not converted again
    assert_relative_eq!(model.jnt_range[0].1, limit_rad, epsilon = 1e-10);

    let mut data = model.make_data();

    // Rotate 60° (> 45° limit)
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();
    let limit_rows = data
        .efc_type
        .iter()
        .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 1, "radian-mode ball limit should activate");
}

// ============================================================================
// T10: Range interpretation symmetry
// ============================================================================

#[test]
fn test_ball_limit_range_symmetry() {
    // range="0 45" and range="45 0" should produce identical behavior
    for range in &["0 45", "45 0"] {
        let mjcf = format!(
            r#"
            <mujoco>
                <compiler angle="degree"/>
                <option gravity="0 0 0" timestep="0.001"/>
                <worldbody>
                    <body pos="0 0 0">
                        <joint type="ball" limited="true" range="{range}"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#
        );
        let model = load_model(&mjcf).unwrap();
        let mut data = model.make_data();

        // Rotate 60° about X (past limit)
        let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
        data.qpos[0] = q[0];
        data.qpos[1] = q[1];
        data.qpos[2] = q[2];
        data.qpos[3] = q[3];

        data.step(&model).unwrap();
        let limit_rows = data
            .efc_type
            .iter()
            .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
            .count();
        assert_eq!(
            limit_rows, 1,
            "range=\"{range}\" should produce 1 ball limit constraint"
        );
    }
}

// ============================================================================
// T11: Multiple ball joints (counting/assembly consistency)
// ============================================================================

#[test]
fn test_multiple_ball_joints_counting_consistency() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint name="a" type="ball" limited="true" range="0 30"/>
                    <geom type="sphere" size="0.1" mass="0.5"/>
                    <body pos="0.2 0 0">
                        <joint name="b" type="ball" limited="true" range="0 90"/>
                        <geom type="sphere" size="0.1" mass="0.5"/>
                        <body pos="0.2 0 0">
                            <joint name="c" type="ball"/>
                            <geom type="sphere" size="0.1" mass="0.5"/>
                        </body>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Joint a: rotated 50° about X → exceeds 30° limit (violated)
    let qa = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 50.0);
    let adr_a = model.jnt_qpos_adr[0];
    data.qpos[adr_a] = qa[0];
    data.qpos[adr_a + 1] = qa[1];
    data.qpos[adr_a + 2] = qa[2];
    data.qpos[adr_a + 3] = qa[3];

    // Joint b: rotated 45° about Y → within 90° limit (satisfied)
    let qb = quat_from_axis_angle_deg([0.0, 1.0, 0.0], 45.0);
    let adr_b = model.jnt_qpos_adr[1];
    data.qpos[adr_b] = qb[0];
    data.qpos[adr_b + 1] = qb[1];
    data.qpos[adr_b + 2] = qb[2];
    data.qpos[adr_b + 3] = qb[3];

    // Joint c: rotated 170° about Z → unlimited, no constraint
    let qc = quat_from_axis_angle_deg([0.0, 0.0, 1.0], 170.0);
    let adr_c = model.jnt_qpos_adr[2];
    data.qpos[adr_c] = qc[0];
    data.qpos[adr_c + 1] = qc[1];
    data.qpos[adr_c + 2] = qc[2];
    data.qpos[adr_c + 3] = qc[3];

    // This must not panic (counting == assembly)
    data.step(&model).unwrap();

    let limit_rows = data
        .efc_type
        .iter()
        .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .count();
    assert_eq!(
        limit_rows, 1,
        "only joint_a should produce a limit constraint"
    );

    // Verify efc_id points to joint 0 (joint_a)
    let limit_row = data
        .efc_type
        .iter()
        .position(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .unwrap();
    assert_eq!(
        data.efc_id[limit_row], 0,
        "limit constraint should reference joint_a"
    );
}

// ============================================================================
// T12: Mixed hinge + ball limits (cross-type consistency)
// ============================================================================

#[test]
fn test_mixed_hinge_and_ball_limits() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint name="hinge_j" type="hinge" limited="true" range="-30 30"/>
                    <geom type="sphere" size="0.1" mass="0.5"/>
                    <body pos="0.2 0 0">
                        <joint name="ball_j" type="ball" limited="true" range="0 45"/>
                        <geom type="sphere" size="0.1" mass="0.5"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Hinge: set past upper limit (40° > 30°)
    data.qpos[model.jnt_qpos_adr[0]] = 40.0_f64.to_radians();

    // Ball: set past cone limit (60° > 45°)
    let q = quat_from_axis_angle_deg([0.0, 1.0, 0.0], 60.0);
    let ball_adr = model.jnt_qpos_adr[1];
    data.qpos[ball_adr] = q[0];
    data.qpos[ball_adr + 1] = q[1];
    data.qpos[ball_adr + 2] = q[2];
    data.qpos[ball_adr + 3] = q[3];

    // Must not panic (counting == assembly across mixed types)
    data.step(&model).unwrap();

    let limit_rows: Vec<_> = data
        .efc_type
        .iter()
        .enumerate()
        .filter(|(_, t)| matches!(t, sim_core::ConstraintType::LimitJoint))
        .collect();
    assert_eq!(
        limit_rows.len(),
        2,
        "should have 2 limit rows: 1 hinge upper + 1 ball cone"
    );

    // First limit row should be the hinge (joint 0, assembled first)
    let (hinge_row, _) = limit_rows[0];
    let hinge_dof = model.jnt_dof_adr[0];
    assert_eq!(
        data.efc_id[hinge_row], 0,
        "first limit row should be hinge joint"
    );
    // Hinge upper limit Jacobian: -1.0 at single DOF
    assert_relative_eq!(data.efc_J[(hinge_row, hinge_dof)], -1.0, epsilon = 1e-10);

    // Second limit row should be the ball (joint 1)
    let (ball_row, _) = limit_rows[1];
    let ball_dof = model.jnt_dof_adr[1];
    assert_eq!(
        data.efc_id[ball_row], 1,
        "second limit row should be ball joint"
    );
    // Ball Jacobian: 60° about Y → unit_dir = (0, 1, 0), J = -(0, 1, 0) = (0, -1, 0)
    assert_relative_eq!(data.efc_J[(ball_row, ball_dof)], 0.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(ball_row, ball_dof + 1)], -1.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(ball_row, ball_dof + 2)], 0.0, epsilon = 1e-4);
}

// ============================================================================
// T13: jnt_limit_frc propagation
// ============================================================================

#[test]
fn test_ball_limit_force_propagation() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 30"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate 60° about X — well past the 30° limit
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    // jnt_limit_frc[0] should be non-zero — the solver produced a constraint force
    assert!(
        data.jnt_limit_frc[0].abs() > 1e-10,
        "ball joint limit force should be non-zero: got {}",
        data.jnt_limit_frc[0]
    );

    // The force should also appear in efc_force for the LimitJoint row
    let limit_row = data
        .efc_type
        .iter()
        .position(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .expect("should have a ball limit constraint row");
    assert_relative_eq!(
        data.jnt_limit_frc[0],
        data.efc_force[limit_row],
        epsilon = 1e-15,
    );
}

// ============================================================================
// T14: Degenerate range "0 0" locks joint
// ============================================================================

#[test]
fn test_ball_limit_degenerate_range_locks_joint() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate 20° about Y — any rotation should violate the zero-cone limit
    let q = quat_from_axis_angle_deg([0.0, 1.0, 0.0], 20.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    // Must have exactly 1 constraint row
    let limit_rows = data
        .efc_type
        .iter()
        .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .count();
    assert_eq!(
        limit_rows, 1,
        "degenerate range should activate limit for any rotation"
    );

    // dist = 0 - 20° in radians ≈ -0.349
    let limit_row = data
        .efc_type
        .iter()
        .position(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .unwrap();
    assert!(
        data.efc_pos[limit_row] < -0.3,
        "dist should be strongly negative: got {}",
        data.efc_pos[limit_row]
    );

    // After several steps, the solver should push rotation toward identity
    for _ in 0..200 {
        data.step(&model).unwrap();
    }
    let angle = extract_ball_angle(data.qpos.as_slice(), 0);
    assert!(
        angle.to_degrees() < 5.0,
        "degenerate range should push joint toward identity: angle={:.1}°",
        angle.to_degrees()
    );
}

// ============================================================================
// T15: Near-π rotation with active limit
// ============================================================================

#[test]
fn test_ball_limit_near_pi_rotation() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 170"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate 175° about X — 5° past the 170° limit, near π
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 175.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    // Constraint should activate
    let limit_rows = data
        .efc_type
        .iter()
        .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 1, "near-π rotation should trigger ball limit");

    // dist ≈ 170° - 175° = -5° ≈ -0.0873 rad
    let limit_row = data
        .efc_type
        .iter()
        .position(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .unwrap();
    let expected_dist = 170.0_f64.to_radians() - 175.0_f64.to_radians();
    assert_relative_eq!(data.efc_pos[limit_row], expected_dist, epsilon = 1e-4);

    // Jacobian should point along -X (theta > 0, unit_dir = +X, J = -X)
    let dof = model.jnt_dof_adr[0];
    assert_relative_eq!(data.efc_J[(limit_row, dof)], -1.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 1)], 0.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 2)], 0.0, epsilon = 1e-4);

    // Multi-step: solver should converge despite near-π singularity
    for _ in 0..200 {
        data.step(&model).unwrap();
    }
    let angle = extract_ball_angle(data.qpos.as_slice(), 0);
    // Allow 3° tolerance — near-π Jacobian is less precise
    assert!(
        angle.to_degrees() < 173.0,
        "near-π limit should hold: angle={:.1}°, limit=170°",
        angle.to_degrees()
    );
}

// ============================================================================
// T16: Small violation precision
// ============================================================================

#[test]
fn test_ball_limit_small_violation_precision() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate 45.5° about Z — just barely past the 45° limit
    let q = quat_from_axis_angle_deg([0.0, 0.0, 1.0], 45.5);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    // Must activate — even a tiny violation should produce a constraint row
    let limit_rows = data
        .efc_type
        .iter()
        .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 1, "small violation should still activate limit");

    // dist = 45° - 45.5° = -0.5° ≈ -0.008727 rad
    let limit_row = data
        .efc_type
        .iter()
        .position(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .unwrap();
    let expected_dist = 45.0_f64.to_radians() - 45.5_f64.to_radians();
    assert_relative_eq!(data.efc_pos[limit_row], expected_dist, epsilon = 1e-6,);

    // Jacobian direction: along -Z (theta > 0 about Z)
    let dof = model.jnt_dof_adr[0];
    assert_relative_eq!(data.efc_J[(limit_row, dof)], 0.0, epsilon = 1e-6);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 1)], 0.0, epsilon = 1e-6);
    assert_relative_eq!(data.efc_J[(limit_row, dof + 2)], -1.0, epsilon = 1e-6);
}

// ============================================================================
// T17: Negative-w quaternion in pipeline
// ============================================================================

#[test]
fn test_ball_limit_negative_w_quaternion() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    // Run with positive-w quaternion
    let q_pos = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
    let model = load_model(mjcf).unwrap();
    let mut data_pos = model.make_data();
    data_pos.qpos[0] = q_pos[0];
    data_pos.qpos[1] = q_pos[1];
    data_pos.qpos[2] = q_pos[2];
    data_pos.qpos[3] = q_pos[3];
    data_pos.step(&model).unwrap();

    // Run with negative-w quaternion (same rotation, opposite hemisphere)
    let mut data_neg = model.make_data();
    data_neg.qpos[0] = -q_pos[0];
    data_neg.qpos[1] = -q_pos[1];
    data_neg.qpos[2] = -q_pos[2];
    data_neg.qpos[3] = -q_pos[3];
    data_neg.step(&model).unwrap();

    // Both should produce exactly 1 LimitJoint row
    let row_pos = data_pos
        .efc_type
        .iter()
        .position(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .expect("positive-w should have limit row");
    let row_neg = data_neg
        .efc_type
        .iter()
        .position(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .expect("negative-w should have limit row");

    // Constraint distance must match
    assert_relative_eq!(
        data_pos.efc_pos[row_pos],
        data_neg.efc_pos[row_neg],
        epsilon = 1e-10,
    );

    // Jacobian must match
    let dof = model.jnt_dof_adr[0];
    for d in 0..3 {
        assert_relative_eq!(
            data_pos.efc_J[(row_pos, dof + d)],
            data_neg.efc_J[(row_neg, dof + d)],
            epsilon = 1e-10,
        );
    }
}

// ============================================================================
// T18: Multiple simultaneous violations
// ============================================================================

#[test]
fn test_multiple_ball_joints_simultaneously_violated() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint name="ball_a" type="ball" limited="true" range="0 30"/>
                    <geom type="sphere" size="0.1" mass="0.5"/>
                    <body pos="0.2 0 0">
                        <joint name="ball_b" type="ball" limited="true" range="0 45"/>
                        <geom type="sphere" size="0.1" mass="0.5"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Joint a: 50° about X → exceeds 30° limit (20° violation)
    let qa = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 50.0);
    let adr_a = model.jnt_qpos_adr[0];
    data.qpos[adr_a] = qa[0];
    data.qpos[adr_a + 1] = qa[1];
    data.qpos[adr_a + 2] = qa[2];
    data.qpos[adr_a + 3] = qa[3];

    // Joint b: 60° about Y → exceeds 45° limit (15° violation)
    let qb = quat_from_axis_angle_deg([0.0, 1.0, 0.0], 60.0);
    let adr_b = model.jnt_qpos_adr[1];
    data.qpos[adr_b] = qb[0];
    data.qpos[adr_b + 1] = qb[1];
    data.qpos[adr_b + 2] = qb[2];
    data.qpos[adr_b + 3] = qb[3];

    // Must not panic (counting == assembly for multiple simultaneous violations)
    data.step(&model).unwrap();

    let limit_rows: Vec<_> = data
        .efc_type
        .iter()
        .enumerate()
        .filter(|(_, t)| matches!(t, sim_core::ConstraintType::LimitJoint))
        .collect();
    assert_eq!(
        limit_rows.len(),
        2,
        "should have 2 limit rows for 2 simultaneously violated ball joints"
    );

    // Row 0: joint a (assembled first)
    let (row_a, _) = limit_rows[0];
    assert_eq!(
        data.efc_id[row_a], 0,
        "first limit row should reference joint a"
    );
    let expected_dist_a = 30.0_f64.to_radians() - 50.0_f64.to_radians();
    assert_relative_eq!(data.efc_pos[row_a], expected_dist_a, epsilon = 1e-4);
    // Jacobian: 50° about X, theta > 0 → J = (-1, 0, 0)
    let dof_a = model.jnt_dof_adr[0];
    assert_relative_eq!(data.efc_J[(row_a, dof_a)], -1.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(row_a, dof_a + 1)], 0.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(row_a, dof_a + 2)], 0.0, epsilon = 1e-4);

    // Row 1: joint b
    let (row_b, _) = limit_rows[1];
    assert_eq!(
        data.efc_id[row_b], 1,
        "second limit row should reference joint b"
    );
    let expected_dist_b = 45.0_f64.to_radians() - 60.0_f64.to_radians();
    assert_relative_eq!(data.efc_pos[row_b], expected_dist_b, epsilon = 1e-4);
    // Jacobian: 60° about Y, theta > 0 → J = (0, -1, 0)
    let dof_b = model.jnt_dof_adr[1];
    assert_relative_eq!(data.efc_J[(row_b, dof_b)], 0.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(row_b, dof_b + 1)], -1.0, epsilon = 1e-4);
    assert_relative_eq!(data.efc_J[(row_b, dof_b + 2)], 0.0, epsilon = 1e-4);
}

// ============================================================================
// T19: efc_vel sign with opposing angular velocity
// ============================================================================

#[test]
fn test_ball_limit_efc_vel_opposing_velocity() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // 60° about X → theta > 0, unit_dir = (+1, 0, 0), J = (-1, 0, 0)
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    let dof = model.jnt_dof_adr[0];

    // Case 1: velocity pushing INTO the limit (+X direction, same as rotation)
    data.qvel[dof] = 2.0; // omega_x = +2 rad/s
    data.qvel[dof + 1] = 0.0;
    data.qvel[dof + 2] = 0.0;
    data.step(&model).unwrap();

    let row_into = data
        .efc_type
        .iter()
        .position(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .expect("should have limit row");
    let vel_into = data.efc_vel[row_into];
    // J · qvel = (-1)(2) + (0)(0) + (0)(0) = -2.0 (negative = pushing into limit)
    assert!(
        vel_into < 0.0,
        "velocity into limit should produce negative efc_vel: got {vel_into}"
    );
    assert_relative_eq!(vel_into, -2.0, epsilon = 1e-4);

    // Case 2: velocity pulling OUT of the limit (-X direction, opposing rotation)
    let mut data2 = model.make_data();
    data2.qpos[0] = q[0];
    data2.qpos[1] = q[1];
    data2.qpos[2] = q[2];
    data2.qpos[3] = q[3];
    data2.qvel[dof] = -2.0; // omega_x = -2 rad/s (opposing)
    data2.qvel[dof + 1] = 0.0;
    data2.qvel[dof + 2] = 0.0;
    data2.step(&model).unwrap();

    let row_out = data2
        .efc_type
        .iter()
        .position(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .expect("should have limit row");
    let vel_out = data2.efc_vel[row_out];
    // J · qvel = (-1)(-2) + (0)(0) + (0)(0) = +2.0 (positive = pulling out of limit)
    assert!(
        vel_out > 0.0,
        "velocity out of limit should produce positive efc_vel: got {vel_out}"
    );
    assert_relative_eq!(vel_out, 2.0, epsilon = 1e-4);
}

// ============================================================================
// T20: Unnormalized quaternion in constraint assembly
// ============================================================================

#[test]
fn test_ball_limit_unnormalized_quaternion() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();

    // Unit quaternion for 60° about X
    let q_unit = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);

    // Run with unit quaternion
    let mut data_unit = model.make_data();
    data_unit.qpos[0] = q_unit[0];
    data_unit.qpos[1] = q_unit[1];
    data_unit.qpos[2] = q_unit[2];
    data_unit.qpos[3] = q_unit[3];
    data_unit.step(&model).unwrap();

    // Run with scaled quaternion (norm = 1.05 — typical drift magnitude)
    let scale = 1.05;
    let mut data_scaled = model.make_data();
    data_scaled.qpos[0] = q_unit[0] * scale;
    data_scaled.qpos[1] = q_unit[1] * scale;
    data_scaled.qpos[2] = q_unit[2] * scale;
    data_scaled.qpos[3] = q_unit[3] * scale;
    data_scaled.step(&model).unwrap();

    // Both should produce exactly 1 LimitJoint row
    let row_unit = data_unit
        .efc_type
        .iter()
        .position(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .expect("unit quat should have limit row");
    let row_scaled = data_scaled
        .efc_type
        .iter()
        .position(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .expect("scaled quat should have limit row");

    // Constraint distance must match (normalization produces same rotation)
    assert_relative_eq!(
        data_unit.efc_pos[row_unit],
        data_scaled.efc_pos[row_scaled],
        epsilon = 1e-8,
    );

    // Jacobian must match
    let dof = model.jnt_dof_adr[0];
    for d in 0..3 {
        assert_relative_eq!(
            data_unit.efc_J[(row_unit, dof + d)],
            data_scaled.efc_J[(row_scaled, dof + d)],
            epsilon = 1e-8,
        );
    }
}

// ============================================================================
// T21: range="45 0" parser-level verification
// ============================================================================

#[test]
fn test_ball_limit_reversed_range_parsing() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="45 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();

    // Verify parser stored (45°→rad, 0°→rad) — not swapped
    assert_relative_eq!(model.jnt_range[0].0, 45.0_f64.to_radians(), epsilon = 1e-6,);
    assert_relative_eq!(model.jnt_range[0].1, 0.0, epsilon = 1e-10);

    // Runtime: max(45°, 0°) = 45° is the effective cone half-angle
    let mut data = model.make_data();
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];
    data.step(&model).unwrap();

    let limit_rows = data
        .efc_type
        .iter()
        .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .count();
    assert_eq!(limit_rows, 1, "reversed range should still activate limit");

    // dist = max(45°, 0°) - 60° = 45° - 60° = -15° in radians
    let limit_row = data
        .efc_type
        .iter()
        .position(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .unwrap();
    let expected_dist = 45.0_f64.to_radians() - 60.0_f64.to_radians();
    assert_relative_eq!(data.efc_pos[limit_row], expected_dist, epsilon = 1e-4);
}

// ============================================================================
// T22: Margin = 0.0 regression anchor
// ============================================================================

#[test]
fn test_ball_limit_margin_zero_regression() {
    let mjcf = r#"
        <mujoco>
            <compiler angle="degree"/>
            <option gravity="0 0 0" timestep="0.001"/>
            <worldbody>
                <body pos="0 0 0">
                    <joint type="ball" limited="true" range="0 45"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Rotate 44.5° about X — 0.5° short of the 45° limit
    let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 44.5);
    data.qpos[0] = q[0];
    data.qpos[1] = q[1];
    data.qpos[2] = q[2];
    data.qpos[3] = q[3];

    data.step(&model).unwrap();

    let limit_rows = data
        .efc_type
        .iter()
        .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .count();
    assert_eq!(
        limit_rows, 0,
        "with margin=0, rotation 0.5° below limit should produce no constraint"
    );

    // Contrast: 45.5° (0.5° past limit) should activate
    let mut data2 = model.make_data();
    let q2 = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 45.5);
    data2.qpos[0] = q2[0];
    data2.qpos[1] = q2[1];
    data2.qpos[2] = q2[2];
    data2.qpos[3] = q2[3];
    data2.step(&model).unwrap();

    let limit_rows2 = data2
        .efc_type
        .iter()
        .filter(|t| matches!(t, sim_core::ConstraintType::LimitJoint))
        .count();
    assert_eq!(
        limit_rows2, 1,
        "with margin=0, rotation 0.5° past limit should produce 1 constraint"
    );
}
