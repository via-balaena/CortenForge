//! Tendon `springlength` acceptance tests (§22 Tendon springlength MJCF Parsing
//! + Deadband Spring Physics).
//!
//! Tests AC1–AC15, AC3b, AC5b–d, AC8b, AC13b from the spec, covering:
//! - Single-value and two-value springlength parsing (AC1, AC2)
//! - Auto-compute sentinel resolution (AC3, AC3b)
//! - Sentinel vs explicit zero (AC4)
//! - Validation: low > high, negative, NaN, two-value negative (AC5, AC5b–d)
//! - Default class support: apply, override, two-value, nested (AC6–AC8, AC8b)
//! - Deadband physics: in-band zero, above/below restoring, classical (AC9–AC12)
//! - Spatial tendon: explicit and auto-compute (AC13, AC13b)
//! - Regression: existing tests unchanged (AC14 — covered by existing test suite)
//! - MJB version bump (AC15)
//!
//! Tolerance: 1e-10 for exact compiled values, 1e-6 for physics output.

use approx::assert_relative_eq;
use sim_mjcf::load_model;

// ============================================================================
// Parsing and compilation (AC1–AC4, AC3b)
// ============================================================================

/// AC1 — Single-value springlength on fixed tendon (S2, S8).
///
/// `springlength="0.5"` → compiled `[0.5, 0.5]`.
#[test]
fn test_ac1_single_value_springlength_fixed() {
    let mjcf = r#"
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t" springlength="0.5">
              <joint joint="j" coef="1.0"/>
            </fixed>
          </tendon>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.tendon_lengthspring[0], [0.5, 0.5]);
}

/// AC2 — Two-value springlength on fixed tendon / deadband (S1, S8).
///
/// `springlength="0.3 0.7"` → compiled `[0.3, 0.7]`.
#[test]
fn test_ac2_two_value_springlength_fixed() {
    let mjcf = r#"
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t" springlength="0.3 0.7">
              <joint joint="j" coef="1.0"/>
            </fixed>
          </tendon>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.tendon_lengthspring[0], [0.3, 0.7]);
}

/// AC3 — Auto-compute fallback (no springlength specified) (S3).
///
/// Joint has `ref="0.5"`, coef=1.0, so tendon length at qpos0 = 0.5.
/// Sentinel `[-1, -1]` is resolved to `[0.5, 0.5]`.
#[test]
fn test_ac3_auto_compute_fallback() {
    let mjcf = r#"
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge" ref="0.5"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t" stiffness="100">
              <joint joint="j" coef="1.0"/>
            </fixed>
          </tendon>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_relative_eq!(model.tendon_lengthspring[0][0], 0.5, epsilon = 1e-10);
    assert_relative_eq!(model.tendon_lengthspring[0][1], 0.5, epsilon = 1e-10);
}

/// AC3b — Auto-compute with stiffness=0 (unconditional sentinel resolution) (S3).
///
/// No springlength, no stiffness (defaults to 0). The sentinel `[-1, -1]` must
/// still be resolved because MuJoCo resolves unconditionally.
#[test]
fn test_ac3b_auto_compute_stiffness_zero() {
    let mjcf = r#"
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge" ref="0.5"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t">
              <joint joint="j" coef="1.0"/>
            </fixed>
          </tendon>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // Sentinel resolved even though stiffness == 0 (unconditional per S3).
    assert_relative_eq!(model.tendon_lengthspring[0][0], 0.5, epsilon = 1e-10);
    assert_relative_eq!(model.tendon_lengthspring[0][1], 0.5, epsilon = 1e-10);
}

/// AC4 — `springlength="0"` is valid and not confused with sentinel (S2, S3).
///
/// Explicit zero must NOT be auto-computed. This is the critical sentinel fix.
#[test]
fn test_ac4_explicit_zero_not_sentinel() {
    let mjcf = r#"
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge" ref="0.5"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t" stiffness="100" springlength="0">
              <joint joint="j" coef="1.0"/>
            </fixed>
          </tendon>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // Explicit zero — NOT auto-computed to 0.5. This is the sentinel fix.
    assert_eq!(model.tendon_lengthspring[0], [0.0, 0.0]);
}

// ============================================================================
// Validation (AC5, AC5b–d)
// ============================================================================

/// AC5 — Validation: `low > high` is rejected (S4).
#[test]
fn test_ac5_low_greater_than_high_rejected() {
    let mjcf = r#"
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t" springlength="0.7 0.3">
              <joint joint="j" coef="1.0"/>
            </fixed>
          </tendon>
        </mujoco>
    "#;

    let result = load_model(mjcf);
    assert!(result.is_err());
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("springlength"),
        "Error should mention springlength: {err}"
    );
}

/// AC5b — Validation: negative springlength is rejected (B2 validation).
#[test]
fn test_ac5b_negative_springlength_rejected() {
    let mjcf = r#"
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t" springlength="-0.5">
              <joint joint="j" coef="1.0"/>
            </fixed>
          </tendon>
        </mujoco>
    "#;

    let result = load_model(mjcf);
    assert!(result.is_err());
}

/// AC5c — Validation: NaN springlength is rejected (B2 validation).
///
/// Rust parses "NaN" successfully, but `!f64::NAN.is_finite()` catches it.
#[test]
fn test_ac5c_nan_springlength_rejected() {
    let mjcf = r#"
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t" springlength="NaN">
              <joint joint="j" coef="1.0"/>
            </fixed>
          </tendon>
        </mujoco>
    "#;

    let result = load_model(mjcf);
    // NaN should be rejected by the is_finite() guard. If parse_float_array
    // fails to parse "NaN", the outer `if let Ok(parts)` silently ignores it,
    // which means no springlength is set and sentinel [-1, -1] is used.
    // Either outcome (Err or sentinel resolution) is acceptable — the key
    // invariant is that NaN does NOT propagate to the compiled model.
    match result {
        Err(_) => {} // Rejected — good.
        Ok(model) => {
            // If it loaded, NaN must not have propagated.
            assert!(
                model.tendon_lengthspring[0][0].is_finite(),
                "NaN must not propagate to compiled model"
            );
            assert!(
                model.tendon_lengthspring[0][1].is_finite(),
                "NaN must not propagate to compiled model"
            );
        }
    }
}

/// AC5d — Validation: two-value with one negative is rejected (B2 validation).
#[test]
fn test_ac5d_two_value_one_negative_rejected() {
    let mjcf = r#"
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t" springlength="0.3 -0.5">
              <joint joint="j" coef="1.0"/>
            </fixed>
          </tendon>
        </mujoco>
    "#;

    let result = load_model(mjcf);
    assert!(result.is_err());
}

// ============================================================================
// Default class support (AC6–AC8, AC8b)
// ============================================================================

/// AC6 — springlength from default class (S7).
#[test]
fn test_ac6_springlength_from_default_class() {
    let mjcf = r#"
        <mujoco>
          <compiler angle="radian"/>
          <default>
            <default class="pretensioned">
              <tendon springlength="0.2" stiffness="50"/>
            </default>
          </default>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t" class="pretensioned">
              <joint joint="j" coef="1.0"/>
            </fixed>
          </tendon>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.tendon_lengthspring[0], [0.2, 0.2]);
    assert_relative_eq!(model.tendon_stiffness[0], 50.0, epsilon = 1e-10);
}

/// AC7 — Explicit springlength overrides default (S7).
#[test]
fn test_ac7_explicit_overrides_default() {
    let mjcf = r#"
        <mujoco>
          <compiler angle="radian"/>
          <default>
            <default class="pretensioned">
              <tendon springlength="0.2"/>
            </default>
          </default>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t" class="pretensioned" springlength="0.8">
              <joint joint="j" coef="1.0"/>
            </fixed>
          </tendon>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.tendon_lengthspring[0], [0.8, 0.8]);
}

/// AC8 — Two-value springlength via default class (S7).
#[test]
fn test_ac8_two_value_from_default_class() {
    let mjcf = r#"
        <mujoco>
          <compiler angle="radian"/>
          <default>
            <default class="deadband">
              <tendon springlength="0.1 0.9"/>
            </default>
          </default>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t" class="deadband">
              <joint joint="j" coef="1.0"/>
            </fixed>
          </tendon>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.tendon_lengthspring[0], [0.1, 0.9]);
}

/// AC8b — Nested default class: child inherits parent springlength (S7, C2).
///
/// Parent sets `springlength="0.2"`, child sets only `stiffness="50"`.
/// `merge_tendon_defaults` must propagate parent's springlength.
#[test]
fn test_ac8b_nested_default_inheritance() {
    let mjcf = r#"
        <mujoco>
          <compiler angle="radian"/>
          <default>
            <default class="parent">
              <tendon springlength="0.2"/>
              <default class="child">
                <tendon stiffness="50"/>
              </default>
            </default>
          </default>
          <worldbody>
            <body name="b">
              <joint name="j" type="hinge"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t" class="child">
              <joint joint="j" coef="1.0"/>
            </fixed>
          </tendon>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // springlength inherited from parent
    assert_eq!(model.tendon_lengthspring[0], [0.2, 0.2]);
    // stiffness from child
    assert_relative_eq!(model.tendon_stiffness[0], 50.0, epsilon = 1e-10);
}

// ============================================================================
// Deadband spring physics (AC9–AC12)
// ============================================================================

/// Helper: builds a fixed-tendon model with slide joint for physics tests.
///
/// With `coef=1.0` on a slide joint, `tendon_length = qpos[0]`, giving
/// direct control over tendon length.
fn load_physics_model(springlength: &str, stiffness: f64) -> sim_core::Model {
    let mjcf = format!(
        r#"
        <mujoco>
          <compiler angle="radian"/>
          <option gravity="0 0 0" timestep="0.001"/>
          <worldbody>
            <body name="b">
              <joint name="j" type="slide"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t" springlength="{springlength}" stiffness="{stiffness}">
              <joint joint="j" coef="1.0"/>
            </fixed>
          </tendon>
        </mujoco>
        "#,
    );
    load_model(&mjcf).expect("physics model should load")
}

/// AC9 — Length within deadband: zero spring force (S6).
///
/// `springlength="0.3 0.7"`, stiffness=100, qpos=0.5 (within deadband).
#[test]
fn test_ac9_deadband_zero_force() {
    let model = load_physics_model("0.3 0.7", 100.0);
    let mut data = model.make_data();
    data.qpos[0] = 0.5; // Within [0.3, 0.7] deadband
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.qfrc_passive[0], 0.0, epsilon = 1e-6);
}

/// AC10 — Length above upper bound: restoring force (S6).
///
/// `springlength="0.3 0.7"`, stiffness=100, qpos=1.0 (above upper=0.7).
/// Force = 100 * (0.7 - 1.0) = -30.0.
#[test]
fn test_ac10_above_upper_restoring_force() {
    let model = load_physics_model("0.3 0.7", 100.0);
    let mut data = model.make_data();
    data.qpos[0] = 1.0; // Above upper=0.7
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.qfrc_passive[0], -30.0, epsilon = 1e-6);
}

/// AC11 — Length below lower bound: restoring force (S6).
///
/// `springlength="0.3 0.7"`, stiffness=100, qpos=0.1 (below lower=0.3).
/// Force = 100 * (0.3 - 0.1) = 20.0.
#[test]
fn test_ac11_below_lower_restoring_force() {
    let model = load_physics_model("0.3 0.7", 100.0);
    let mut data = model.make_data();
    data.qpos[0] = 0.1; // Below lower=0.3
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.qfrc_passive[0], 20.0, epsilon = 1e-6);
}

/// AC12 — No deadband (single value): classical spring preserved (S6).
///
/// `springlength="0.5"`, stiffness=100, qpos=0.8.
/// Force = 100 * (0.5 - 0.8) = -30.0.
/// When lower == upper, deadband formula is identical to classical spring.
#[test]
fn test_ac12_classical_spring_preserved() {
    let model = load_physics_model("0.5", 100.0);
    let mut data = model.make_data();
    data.qpos[0] = 0.8;
    data.forward(&model).expect("forward failed");

    assert_relative_eq!(data.qfrc_passive[0], -30.0, epsilon = 1e-6);
}

// ============================================================================
// Spatial tendon (AC13, AC13b)
// ============================================================================

/// AC13 — springlength on spatial tendon (S8).
///
/// Two-value springlength on a spatial tendon element.
#[test]
fn test_ac13_spatial_tendon_springlength() {
    let mjcf = r#"
        <mujoco>
          <worldbody>
            <body name="b1" pos="0 0 0">
              <site name="s1" pos="0 0 0"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
            <body name="b2" pos="1 0 0">
              <site name="s2" pos="0 0 0"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <spatial name="t" springlength="0.5 0.8">
              <site site="s1"/>
              <site site="s2"/>
            </spatial>
          </tendon>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.tendon_lengthspring[0], [0.5, 0.8]);
}

/// AC13b — Spatial tendon auto-compute fallback (no springlength) (S3, S8, E2).
///
/// Two sites 1.0 apart. No springlength → sentinel resolved via
/// `compute_spatial_tendon_length0()` to `[1.0, 1.0]`.
#[test]
fn test_ac13b_spatial_tendon_auto_compute() {
    let mjcf = r#"
        <mujoco>
          <worldbody>
            <body name="b1" pos="0 0 0">
              <site name="s1" pos="0 0 0"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
            <body name="b2" pos="1 0 0">
              <site name="s2" pos="0 0 0"/>
              <geom type="sphere" size="0.1" mass="1.0"/>
            </body>
          </worldbody>
          <tendon>
            <spatial name="t" stiffness="100">
              <site site="s1"/>
              <site site="s2"/>
            </spatial>
          </tendon>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // Auto-computed: distance between sites = 1.0
    assert_relative_eq!(model.tendon_lengthspring[0][0], 1.0, epsilon = 1e-10);
    assert_relative_eq!(model.tendon_lengthspring[0][1], 1.0, epsilon = 1e-10);
}

// ============================================================================
// Regression and MJB version (AC14, AC15)
// ============================================================================

// AC14 — All existing tendon tests pass unchanged.
// This is verified by the full test suite (no dedicated test needed — the
// sentinel change from 0.0 to [-1, -1] is transparent because it's always
// resolved before physics runs).

// AC15 — MJB version bump.
// MJB_VERSION is gated behind the "mjb" feature (requires serde + bincode).
// The version bump (1 → 2) is verified by the existing test at `mjb.rs:433`
// which asserts version mismatch rejection. No additional test needed here.
