//! Integration tests for `<default>` class resolution in the model builder.
//!
//! Verifies that `DefaultResolver` is wired into `model_from_mjcf()` so that
//! `<default>` class attributes are applied to elements before per-element
//! attributes override them.
//!
//! Covers acceptance criteria 1–12 from `sim/docs/todo/future_work_2.md` §1.

use approx::assert_relative_eq;
use sim_mjcf::load_model;

// ============================================================================
// Core functionality (criteria 1–7)
// ============================================================================

/// Criterion 1: Root default joint damping applies to joints without explicit damping.
#[test]
fn test_root_default_joint_damping() {
    let mjcf = r#"
        <mujoco model="default_damping">
            <default>
                <joint damping="0.5"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_relative_eq!(model.jnt_damping[0], 0.5, epsilon = 1e-10);
}

/// Criterion 2: Per-element attributes override class defaults.
#[test]
fn test_explicit_attribute_overrides_default() {
    let mjcf = r#"
        <mujoco model="override_test">
            <default>
                <default class="heavy">
                    <joint damping="0.5"/>
                </default>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0" class="heavy" damping="1.0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_relative_eq!(model.jnt_damping[0], 1.0, epsilon = 1e-10);
}

/// Criterion 3: Nested class inheritance resolves correctly.
#[test]
fn test_nested_class_inheritance() {
    let mjcf = r#"
        <mujoco model="nested_classes">
            <default>
                <joint damping="0.1"/>
                <default class="robot">
                    <joint damping="0.5"/>
                    <default class="arm">
                        <joint armature="0.01"/>
                    </default>
                </default>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0" class="arm"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // "arm" inherits damping=0.5 from "robot", adds armature=0.01
    assert_relative_eq!(model.jnt_damping[0], 0.5, epsilon = 1e-10);
    assert_relative_eq!(model.jnt_armature[0], 0.01, epsilon = 1e-10);
}

/// Criterion 4: Geom friction defaults apply.
#[test]
fn test_geom_friction_defaults() {
    let mjcf = r#"
        <mujoco model="geom_friction">
            <default>
                <geom friction="0.8 0.01 0.001"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // geom_friction is Vector3<f64> — geom index 0 is the body's geom
    assert_relative_eq!(model.geom_friction[0].x, 0.8, epsilon = 1e-10);
    assert_relative_eq!(model.geom_friction[0].y, 0.01, epsilon = 1e-10);
    assert_relative_eq!(model.geom_friction[0].z, 0.001, epsilon = 1e-10);
}

/// Criterion 5: Actuator gear defaults apply.
#[test]
fn test_actuator_gear_defaults() {
    let mjcf = r#"
        <mujoco model="actuator_gear">
            <default>
                <general gear="100"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <general joint="j1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_relative_eq!(model.actuator_gear[0], 100.0, epsilon = 1e-10);
}

/// Criterion 6: Tendon stiffness defaults apply.
#[test]
fn test_tendon_stiffness_defaults() {
    let mjcf = r#"
        <mujoco model="tendon_stiffness">
            <default>
                <tendon stiffness="1000"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t1">
                    <joint joint="j1" coef="1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_relative_eq!(model.tendon_stiffness[0], 1000.0, epsilon = 1e-10);
}

/// Criterion 7: Sensor noise defaults apply.
#[test]
fn test_sensor_noise_defaults() {
    let mjcf = r#"
        <mujoco model="sensor_noise">
            <default>
                <sensor noise="0.01"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos joint="j1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_relative_eq!(model.sensor_noise[0], 0.01, epsilon = 1e-10);
}

// ============================================================================
// Inertia correctness (criterion 8)
// ============================================================================

/// Criterion 8: Geom density from default class affects body mass computation.
#[test]
fn test_default_density_affects_body_mass() {
    // Model A: density from default class (500)
    let mjcf_default_density = r#"
        <mujoco model="density_default">
            <default>
                <geom density="500"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    // Model B: explicit density on geom (500) — should produce same mass
    let mjcf_explicit_density = r#"
        <mujoco model="density_explicit">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" density="500"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    // Model C: no default, no explicit density — parser default is 1000
    let mjcf_parser_default = r#"
        <mujoco model="density_parser_default">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model_a = load_model(mjcf_default_density).expect("should load A");
    let model_b = load_model(mjcf_explicit_density).expect("should load B");
    let model_c = load_model(mjcf_parser_default).expect("should load C");

    // Body 0 is worldbody, body 1 is our body
    let mass_a = model_a.body_mass[1];
    let mass_b = model_b.body_mass[1];
    let mass_c = model_c.body_mass[1];

    // A and B should have the same mass (both density=500)
    assert_relative_eq!(mass_a, mass_b, epsilon = 1e-10);

    // A should have half the mass of C (density 500 vs 1000)
    assert_relative_eq!(mass_a, mass_c / 2.0, epsilon = 1e-10);
}

// ============================================================================
// Edge cases (criteria 9–11)
// ============================================================================

/// Criterion 9: Elements without a class attribute receive root defaults.
#[test]
fn test_no_class_gets_root_defaults() {
    let mjcf = r#"
        <mujoco model="root_defaults">
            <default>
                <joint damping="0.3"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // No class on joint → root defaults apply → damping=0.3
    assert_relative_eq!(model.jnt_damping[0], 0.3, epsilon = 1e-10);
}

/// Criterion 10: Nonexistent class falls through gracefully (no panic).
#[test]
fn test_nonexistent_class_no_panic() {
    let mjcf = r#"
        <mujoco model="bad_class">
            <default>
                <joint damping="0.5"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0" class="nonexistent"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    // Should not panic — element keeps its own values unchanged
    let model = load_model(mjcf).expect("should load");
    // damping should be the parser default (0.0), not the root class default (0.5),
    // because the class "nonexistent" is not found and apply_to_joint returns unchanged
    assert_relative_eq!(model.jnt_damping[0], 0.0, epsilon = 1e-10);
}

/// Criterion 11: Model with no `<default>` block builds identically to before.
#[test]
fn test_no_defaults_unchanged() {
    let mjcf = r#"
        <mujoco model="no_defaults">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0" damping="0.7"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_relative_eq!(model.jnt_damping[0], 0.7, epsilon = 1e-10);
}

// ============================================================================
// Multi-element and mixed scenarios
// ============================================================================

/// Multiple joints: some with class, some without, some with explicit overrides.
#[test]
fn test_mixed_defaults_and_overrides() {
    let mjcf = r#"
        <mujoco model="mixed">
            <default>
                <joint damping="0.2" stiffness="10.0"/>
                <default class="stiff">
                    <joint damping="5.0" stiffness="100.0"/>
                </default>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j_root" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <body name="b2" pos="0 0 0.5">
                        <joint name="j_stiff" type="hinge" axis="0 1 0" class="stiff"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <body name="b3" pos="0 0 0.5">
                            <joint name="j_override" type="hinge" axis="0 1 0" class="stiff" damping="99.0"/>
                            <geom type="sphere" size="0.1" mass="1.0"/>
                        </body>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // j_root: no class → root defaults → damping=0.2, stiffness=10.0
    assert_relative_eq!(model.jnt_damping[0], 0.2, epsilon = 1e-10);
    assert_relative_eq!(model.jnt_stiffness[0], 10.0, epsilon = 1e-10);

    // j_stiff: class="stiff" → damping=5.0, stiffness=100.0
    assert_relative_eq!(model.jnt_damping[1], 5.0, epsilon = 1e-10);
    assert_relative_eq!(model.jnt_stiffness[1], 100.0, epsilon = 1e-10);

    // j_override: class="stiff" but explicit damping=99.0 → damping=99.0, stiffness=100.0
    assert_relative_eq!(model.jnt_damping[2], 99.0, epsilon = 1e-10);
    assert_relative_eq!(model.jnt_stiffness[2], 100.0, epsilon = 1e-10);
}

/// Tendon damping defaults apply alongside stiffness.
#[test]
fn test_tendon_damping_defaults() {
    let mjcf = r#"
        <mujoco model="tendon_damping">
            <default>
                <tendon damping="50"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t1">
                    <joint joint="j1" coef="1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_relative_eq!(model.tendon_damping[0], 50.0, epsilon = 1e-10);
}
