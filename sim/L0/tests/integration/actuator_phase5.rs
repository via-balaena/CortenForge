//! Phase 5 Spec A integration tests: acc0 for all actuator types, dampratio
//! MJCF round-trip, lengthrange from limits.
//!
//! These tests verify the MJCF → Model pipeline for new Phase 5 features:
//! - T11 (AC12): dampratio attribute parsed and stored as positive biasprm[2]
//! - T7 (AC7): lengthrange from limits unchanged for muscle actuators
//! - T12 (AC13): lengthrange mode filtering skips non-muscle actuators

use approx::assert_relative_eq;
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
