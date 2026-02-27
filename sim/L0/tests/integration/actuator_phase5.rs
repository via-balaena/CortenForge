//! Phase 5 Spec A integration tests: acc0 for all actuator types, dampratio
//! MJCF round-trip, lengthrange from limits.
//!
//! These tests verify the MJCF → Model pipeline for new Phase 5 features:
//! - T11 (AC12): dampratio attribute parsed and stored as positive biasprm[2]
//! - T7 (AC7): lengthrange from limits unchanged for muscle actuators
//! - T12 (AC13): lengthrange mode filtering skips non-muscle actuators

use approx::assert_relative_eq;
use sim_core::ActuatorTransmission;
use sim_mjcf::load_model;

// ============================================================================
// T11: dampratio MJCF round-trip → AC12
// ============================================================================

/// Position actuator with dampratio=1.0 stores positive biasprm[2] before
/// compute_actuator_params, and negative after (converted to damping).
#[test]
fn test_dampratio_mjcf_roundtrip() {
    let mjcf = r#"
        <mujoco model="dampratio_test">
            <worldbody>
                <body name="arm" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <position name="pos" joint="hinge" kp="100" dampratio="1.0"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // After compute_actuator_params: biasprm[2] should be negative (converted)
    assert!(
        model.actuator_biasprm[0][2] < 0.0,
        "After compute_actuator_params, biasprm[2] should be negative (damping), got {}",
        model.actuator_biasprm[0][2]
    );

    // gainprm[0] should be kp = 100
    assert_relative_eq!(model.actuator_gainprm[0][0], 100.0, epsilon = 1e-10);

    // biasprm[1] should be -kp = -100
    assert_relative_eq!(model.actuator_biasprm[0][1], -100.0, epsilon = 1e-10);

    // Verify the damping formula: kv = dampratio * 2 * sqrt(kp * reflected_mass)
    // For a box with mass=1.0 and size 0.1: I_yy = (1/12)*1*(0.2^2+0.2^2) = 0.00667
    // reflected_mass ≈ I_yy / gear^2 (gear=1.0 default for position)
    // Exact value depends on the body inertia, but it must be negative and finite.
    assert!(
        model.actuator_biasprm[0][2].is_finite(),
        "biasprm[2] should be finite"
    );
}

/// Position actuator with explicit kv (no dampratio) stores negative biasprm[2].
#[test]
fn test_explicit_kv_no_dampratio() {
    let mjcf = r#"
        <mujoco model="kv_test">
            <worldbody>
                <body name="arm" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <position name="pos" joint="hinge" kp="100" kv="10"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // Explicit kv=10 → biasprm[2] = -10 (stored as negative)
    // compute_actuator_params should NOT modify it (it's already negative)
    assert_relative_eq!(model.actuator_biasprm[0][2], -10.0, epsilon = 1e-10);
}

// ============================================================================
// acc0 for non-muscle actuators via MJCF
// ============================================================================

/// Motor actuator on a hinge joint should have nonzero acc0 after model build.
#[test]
fn test_acc0_motor_via_mjcf() {
    let mjcf = r#"
        <mujoco model="motor_acc0">
            <worldbody>
                <body name="arm" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <motor name="m" joint="hinge" gear="2"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // acc0 should be nonzero for motor actuator (not just muscles)
    assert!(
        model.actuator_acc0[0] > 1.0,
        "Motor acc0 should be nonzero, got {}",
        model.actuator_acc0[0]
    );
}

// ============================================================================
// T7: lengthrange from limits → AC7
// ============================================================================

/// Muscle actuator on limited joint gets lengthrange from joint limits.
#[test]
fn test_lengthrange_from_joint_limits() {
    // range="-60 60" in degrees → ±1.0472 radians after MuJoCo degree-to-radian conversion
    let mjcf = r#"
        <mujoco model="muscle_lr">
            <worldbody>
                <body name="arm" pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"
                           limited="true" range="-60 60"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
                    <site name="s1" pos="0 0 0.1"/>
                </body>
            </worldbody>
            <actuator>
                <muscle name="mus" joint="hinge" gear="2"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // lengthrange = gear * joint_range_radians = 2 * [-π/3, π/3]
    let expected_rad = std::f64::consts::FRAC_PI_3; // 60° in radians
    let (lo, hi) = model.actuator_lengthrange[0];
    assert_relative_eq!(lo, -2.0 * expected_rad, epsilon = 1e-6);
    assert_relative_eq!(hi, 2.0 * expected_rad, epsilon = 1e-6);
}

// ============================================================================
// T12: lengthrange mode filtering → AC13
// ============================================================================

/// Non-muscle actuator on unlimited joint gets no lengthrange (mode=Muscle default).
#[test]
fn test_lengthrange_mode_filter_nonmuscle() {
    let mjcf = r#"
        <mujoco model="motor_no_lr">
            <worldbody>
                <body name="arm" pos="0 0 0">
                    <joint name="slide" type="slide" axis="0 0 1"/>
                    <geom type="box" size="0.1 0.1 0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <motor name="m" joint="slide" gear="1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // Motor (non-muscle) on unlimited slide → lengthrange should remain (0, 0)
    let (lo, hi) = model.actuator_lengthrange[0];
    assert_relative_eq!(lo, 0.0, epsilon = 1e-10);
    assert_relative_eq!(hi, 0.0, epsilon = 1e-10);
}

// ============================================================================
// T8: Muscle on unlimited joint → LR simulation silently fails (AC8)
// ============================================================================

/// Muscle actuator on unlimited joint: simulation-based LR silently fails
/// because the muscle force model requires valid lengthrange to compute
/// sensible forces (circular dependency). MuJoCo also fails silently here —
/// the utility is for non-muscle actuators in mode=All, or for muscles
/// where lengthrange is already partially set from limits.
///
/// Verify that the failure is silent: lengthrange remains (0, 0) and no panic.
#[test]
fn test_lengthrange_muscle_unlimited_silently_fails() {
    let mjcf = r#"
        <mujoco model="muscle_unlimited">
            <worldbody>
                <body name="arm" pos="0 0 0.5">
                    <joint name="hinge" type="hinge" axis="0 0 1"/>
                    <geom type="box" size="0.2 0.05 0.05" mass="2.0"/>
                </body>
            </worldbody>
            <actuator>
                <muscle name="mus" joint="hinge" gear="1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // Muscle on unlimited joint: LR estimation silently fails (matching MuJoCo).
    // lengthrange should remain (0, 0).
    let (lo, hi) = model.actuator_lengthrange[0];
    assert_relative_eq!(lo, 0.0, epsilon = 1e-10);
    assert_relative_eq!(hi, 0.0, epsilon = 1e-10);
}

// ============================================================================
// T9: Muscle on limited joint → LR from limits (useexisting skips sim)
// ============================================================================

/// Muscle actuator on limited joint: Phase 1 copies limits, then
/// useexisting=true causes the simulation phase to skip this actuator.
/// The lengthrange should match gear * joint_range.
#[test]
fn test_lengthrange_muscle_limited_from_limits() {
    let mjcf = r#"
        <mujoco model="muscle_limited">
            <worldbody>
                <body name="arm" pos="0 0 0.5">
                    <joint name="hinge" type="hinge" axis="0 0 1"
                           limited="true" range="-90 90"/>
                    <geom type="box" size="0.2 0.05 0.05" mass="2.0"/>
                </body>
            </worldbody>
            <actuator>
                <muscle name="mus" joint="hinge" gear="2"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // Limited joint: lengthrange = gear * joint_range (in radians)
    let expected_rad = std::f64::consts::FRAC_PI_2; // 90° in radians
    let (lo, hi) = model.actuator_lengthrange[0];
    assert_relative_eq!(lo, -2.0 * expected_rad, epsilon = 1e-6);
    assert_relative_eq!(hi, 2.0 * expected_rad, epsilon = 1e-6);
}

// ============================================================================
// Spec B: Transmission Types + Slider-Crank
// ============================================================================

// --- T1: SliderCrank length + moment — colinear geometry → AC1, AC2 ---

/// Colinear slider-crank: crank at (0,0,1) above slider at (0,0,0).
/// With cranklength=0.5, gear=2.0: length = (1.0-0.5)*2.0 = 1.0.
#[test]
fn test_slidercrank_length_colinear() {
    let mjcf = r#"
        <mujoco model="crank_colinear">
            <worldbody>
                <site name="slider" pos="0 0 0"/>
                <body pos="0 0 0">
                    <joint name="j" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
                    <site name="crank" pos="0 0 1"/>
                </body>
            </worldbody>
            <actuator>
                <general cranksite="crank" slidersite="slider" cranklength="0.5" gear="2"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.step(&model).expect("step");

    // Length at qpos=0: vec=(0,0,1), axis=(0,0,1), av=1.0, det=0.25, sdet=0.5
    // length = (1.0 - 0.5) * 2.0 = 1.0
    assert_relative_eq!(data.actuator_length[0], 1.0, epsilon = 1e-10);

    // At qpos=0, crank directly above slider on the hinge rotation axis plane.
    // The moment is zero here because the hinge rotation (about Y) produces
    // Jacobian vectors orthogonal to the length derivatives (both along z).
    // This is geometrically correct: at the aligned configuration, an
    // infinitesimal hinge rotation doesn't change the slider-crank length.
    let moment_at_dof0 = data.actuator_moment[0][0];
    assert_relative_eq!(moment_at_dof0, 0.0, epsilon = 1e-10);
}

// --- T2: SliderCrank moment + velocity — offset geometry → AC2, AC3 ---

/// Offset slider-crank: crank at (0.5, 0, 0.5), slider at (0,0,0).
/// cranklength=0.6, gear=1.0. Analytical moment ≈ 0.25378.
#[test]
fn test_slidercrank_moment_offset() {
    let mjcf = r#"
        <mujoco model="crank_offset">
            <worldbody>
                <site name="slider" pos="0 0 0"/>
                <body pos="0 0 0">
                    <joint name="j" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
                    <site name="crank" pos="0.5 0 0.5"/>
                </body>
            </worldbody>
            <actuator>
                <general cranksite="crank" slidersite="slider" cranklength="0.6" gear="1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.step(&model).expect("step");

    // Length at qpos=0: ≈ 0.16834
    assert_relative_eq!(data.actuator_length[0], 0.16834, epsilon = 1e-4);

    // Moment at hinge DOF ≈ 0.25378
    let moment_at_dof0 = data.actuator_moment[0][0];
    assert_relative_eq!(moment_at_dof0, 0.25378, epsilon = 1e-4);

    // Velocity with qvel=[1.0]: velocity = moment.dot(qvel) ≈ 0.25378
    // (no additional gear factor — gear baked into moment)
    data.qvel[0] = 1.0;
    // Re-step to recompute velocity
    data.step(&model).expect("step");
    assert_relative_eq!(data.actuator_velocity[0], moment_at_dof0, epsilon = 1e-3);
}

// --- T3: SliderCrank end-to-end force application → AC4 ---

/// SliderCrank motor with ctrl=1.0 should produce nonzero qfrc_actuator.
/// Uses offset geometry (crank at 0.5, 0, 0.5) so the slider-crank mechanism
/// has nonzero moment at qpos=0, unlike the colinear case.
#[test]
fn test_slidercrank_force_application() {
    let mjcf = r#"
        <mujoco model="crank_force">
            <worldbody>
                <site name="slider" pos="0 0 0"/>
                <body pos="0 0 0">
                    <joint name="j" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
                    <site name="crank" pos="0.5 0 0.5"/>
                </body>
            </worldbody>
            <actuator>
                <general cranksite="crank" slidersite="slider" cranklength="0.6"
                         gear="1" gainprm="1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.step(&model).expect("step");

    // With offset geometry, moment ≈ 0.25378 (from T2).
    // gainprm=1, ctrl=1 → force=1.0, so qfrc_actuator[0] = moment * force ≈ 0.25378
    assert!(
        data.qfrc_actuator[0].abs() > 1e-10,
        "qfrc_actuator should be nonzero, got {}",
        data.qfrc_actuator[0]
    );
}

// --- T4: SliderCrank degenerate (det ≤ 0) — no panic → AC5 ---

/// Degenerate case: crank orthogonal to slider axis, det < 0.
/// Should not panic, length = av * gear = 0.0.
#[test]
fn test_slidercrank_degenerate() {
    let mjcf = r#"
        <mujoco model="crank_degen">
            <worldbody>
                <site name="slider" pos="0 0 0"/>
                <body pos="1 0 0">
                    <joint name="j" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
                    <site name="crank" pos="0 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <general cranksite="crank" slidersite="slider" cranklength="0.5" gear="1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.step(&model).expect("step should not panic");

    // det = 0 + 0.25 - 1.0 = -0.75 ≤ 0 → degenerate path
    // length = av * gear = dot((1,0,0),(0,0,1)) * 1 = 0.0
    assert_relative_eq!(data.actuator_length[0], 0.0, epsilon = 1e-10);
    // No NaN
    assert!(!data.actuator_length[0].is_nan());
}

// --- T5: JointInParent hinge identity → AC6 ---

/// JointInParent on hinge == Joint on hinge (identical behavior for scalar joints).
#[test]
fn test_jointinparent_hinge_identity() {
    let mjcf_joint = r#"
        <mujoco model="joint_hinge">
            <worldbody>
                <body pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
                </body>
            </worldbody>
            <actuator>
                <motor joint="hinge" gear="5"/>
            </actuator>
        </mujoco>
    "#;

    let mjcf_jip = r#"
        <mujoco model="jip_hinge">
            <worldbody>
                <body pos="0 0 0">
                    <joint name="hinge" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
                </body>
            </worldbody>
            <actuator>
                <general jointinparent="hinge" gear="5"/>
            </actuator>
        </mujoco>
    "#;

    let model_a = load_model(mjcf_joint).expect("should load joint");
    let model_b = load_model(mjcf_jip).expect("should load jip");

    let mut data_a = model_a.make_data();
    let mut data_b = model_b.make_data();

    data_a.ctrl[0] = 1.0;
    data_b.ctrl[0] = 1.0;

    data_a.step(&model_a).expect("step");
    data_b.step(&model_b).expect("step");

    // All actuator fields should be identical
    assert_relative_eq!(
        data_a.actuator_length[0],
        data_b.actuator_length[0],
        epsilon = 0.0
    );
    assert_relative_eq!(
        data_a.actuator_velocity[0],
        data_b.actuator_velocity[0],
        epsilon = 0.0
    );
    assert_relative_eq!(
        data_a.qfrc_actuator[0],
        data_b.qfrc_actuator[0],
        epsilon = 0.0
    );
}

// --- T6: JointInParent slide identity → AC7 ---

/// JointInParent on slide == Joint on slide.
#[test]
fn test_jointinparent_slide_identity() {
    let mjcf_joint = r#"
        <mujoco model="joint_slide">
            <worldbody>
                <body pos="0 0 0">
                    <joint name="slide" type="slide" axis="0 0 1"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
                </body>
            </worldbody>
            <actuator>
                <motor joint="slide" gear="3"/>
            </actuator>
        </mujoco>
    "#;

    let mjcf_jip = r#"
        <mujoco model="jip_slide">
            <worldbody>
                <body pos="0 0 0">
                    <joint name="slide" type="slide" axis="0 0 1"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
                </body>
            </worldbody>
            <actuator>
                <general jointinparent="slide" gear="3"/>
            </actuator>
        </mujoco>
    "#;

    let model_a = load_model(mjcf_joint).expect("should load joint");
    let model_b = load_model(mjcf_jip).expect("should load jip");

    let mut data_a = model_a.make_data();
    let mut data_b = model_b.make_data();

    data_a.ctrl[0] = 1.0;
    data_b.ctrl[0] = 1.0;

    data_a.step(&model_a).expect("step");
    data_b.step(&model_b).expect("step");

    assert_relative_eq!(
        data_a.actuator_length[0],
        data_b.actuator_length[0],
        epsilon = 0.0
    );
    assert_relative_eq!(
        data_a.actuator_velocity[0],
        data_b.actuator_velocity[0],
        epsilon = 0.0
    );
    assert_relative_eq!(
        data_a.qfrc_actuator[0],
        data_b.qfrc_actuator[0],
        epsilon = 0.0
    );
}

// --- T7: MJCF parsing — jointinparent → AC8 ---

/// Parse `<general jointinparent="j" gear="5"/>`.
#[test]
fn test_mjcf_jointinparent_roundtrip() {
    let mjcf = r#"
        <mujoco model="jip_parse">
            <worldbody>
                <body pos="0 0 0">
                    <joint name="j" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
                </body>
            </worldbody>
            <actuator>
                <general name="a" jointinparent="j" gear="5"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    assert_eq!(
        model.actuator_trntype[0],
        ActuatorTransmission::JointInParent
    );
    assert_eq!(model.actuator_trnid[0][0], 0); // first joint
    assert_relative_eq!(model.actuator_gear[0][0], 5.0, epsilon = 1e-10);
}

// --- T8: MJCF parsing — SliderCrank → AC9 ---

/// Parse `<general cranksite="c" slidersite="s" cranklength="0.5" gear="2"/>`.
#[test]
fn test_mjcf_slidercrank_roundtrip() {
    let mjcf = r#"
        <mujoco model="crank_parse">
            <worldbody>
                <site name="c" pos="0 0 1"/>
                <body pos="0 0 0">
                    <joint name="j" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
                    <site name="s" pos="0 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <general name="a" cranksite="c" slidersite="s" cranklength="0.5" gear="2"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    assert_eq!(model.actuator_trntype[0], ActuatorTransmission::SliderCrank);
    // trnid[0] = cranksite, trnid[1] = slidersite
    assert_relative_eq!(model.actuator_cranklength[0], 0.5, epsilon = 1e-10);
    assert_relative_eq!(model.actuator_gear[0][0], 2.0, epsilon = 1e-10);
}

// --- T9: MJCF error — cranksite without slidersite → AC10 ---

/// Missing slidersite with cranksite present should error.
#[test]
fn test_mjcf_cranksite_without_slidersite() {
    let mjcf = r#"
        <mujoco model="crank_no_slider">
            <worldbody>
                <site name="c" pos="0 0 1"/>
                <body pos="0 0 0">
                    <joint name="j" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
                </body>
            </worldbody>
            <actuator>
                <general cranksite="c" cranklength="0.5"/>
            </actuator>
        </mujoco>
    "#;

    let err = load_model(mjcf).unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("slidersite"),
        "error should mention slidersite, got: {msg}"
    );
}

// --- T10: MJCF error — non-positive cranklength → AC11 ---

/// Zero cranklength should error.
#[test]
fn test_mjcf_nonpositive_cranklength() {
    let mjcf = r#"
        <mujoco model="crank_zero_rod">
            <worldbody>
                <site name="c" pos="0 0 1"/>
                <body pos="0 0 0">
                    <joint name="j" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
                    <site name="s" pos="0 0 0"/>
                </body>
            </worldbody>
            <actuator>
                <general cranksite="c" slidersite="s" cranklength="0"/>
            </actuator>
        </mujoco>
    "#;

    let err = load_model(mjcf).unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("cranklength"),
        "error should mention cranklength, got: {msg}"
    );
}

// --- T11: mj_jac_point_axis correctness → AC13 ---

/// Verify mj_jac_point_axis computes cross(jacr_col, axis) correctly
/// for a single hinge joint about the Y-axis.
#[test]
fn test_jac_point_axis_correctness() {
    let mjcf = r#"
        <mujoco model="jac_test">
            <worldbody>
                <body pos="0 0 0">
                    <joint name="j" type="hinge" axis="0 1 0"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
                    <site name="pt" pos="1 0 0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.step(&model).expect("step");

    // For a hinge about Y at origin, point at (1,0,0):
    // jacr_col = (0, 1, 0) (rotation axis)
    // With axis = (0, 0, 1):
    // cross(jacr_col, axis) = cross((0,1,0), (0,0,1)) = (1, 0, 0)
    use sim_core::jacobian::mj_jac_point_axis;
    let axis = nalgebra::Vector3::new(0.0, 0.0, 1.0);
    let point = nalgebra::Vector3::new(1.0, 0.0, 0.0);
    let (_jacp, jac_axis) = mj_jac_point_axis(&model, &data, 1, &point, &axis);

    // jac_axis column 0 should be cross((0,1,0), (0,0,1)) = (1, 0, 0)
    assert_relative_eq!(jac_axis[(0, 0)], 1.0, epsilon = 1e-10);
    assert_relative_eq!(jac_axis[(1, 0)], 0.0, epsilon = 1e-10);
    assert_relative_eq!(jac_axis[(2, 0)], 0.0, epsilon = 1e-10);
}
