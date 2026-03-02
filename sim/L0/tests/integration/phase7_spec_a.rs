//! Phase 7 Spec A Tests: Defaults Completeness (DT-2, DT-13, DT-14).
//!
//! T1–T15 acceptance and supplementary tests covering:
//! - DT-2: Equality constraint defaults (solref, solimp, active cascade)
//! - DT-13: `qpos_spring` array (hinge, ball, free, spring force regression)
//! - DT-14: Actuator type-specific defaults (cylinder, muscle, adhesion, damper)

use approx::assert_relative_eq;
use sim_mjcf::load_model;

// ============================================================================
// T1: Equality defaults — solref/solimp cascade → AC1, AC2
// ============================================================================

#[test]
fn t1_equality_defaults_solref_solimp_cascade() {
    let mjcf = r#"
        <mujoco model="eq_defaults_solref_solimp">
            <default>
                <equality solref="0.05 0.8" solimp="0.8 0.9 0.01 0.4 3.0"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="b2" pos="0 0 2">
                    <joint name="j2" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="b1" body2="b2" anchor="0 0 1.5"/>
                <weld body1="b1"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // AC1: connect inherits solref from defaults
    assert_relative_eq!(model.eq_solref[0][0], 0.05, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solref[0][1], 0.8, epsilon = 1e-10);
    // AC2: connect inherits solimp from defaults
    assert_relative_eq!(model.eq_solimp[0][0], 0.8, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][1], 0.9, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][2], 0.01, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][3], 0.4, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][4], 3.0, epsilon = 1e-10);
    // Weld also inherits the same defaults
    assert_relative_eq!(model.eq_solref[1][0], 0.05, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solref[1][1], 0.8, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[1][0], 0.8, epsilon = 1e-10);
}

// ============================================================================
// T2: Equality defaults — active=false cascade → AC3
// ============================================================================

#[test]
fn t2_equality_defaults_active_false_cascade() {
    let mjcf = r#"
        <mujoco model="eq_defaults_active">
            <default>
                <equality active="false"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="b2" pos="0 0 2">
                    <joint name="j2" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="b1" body2="b2" anchor="0 0 1.5"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // AC3: connect inherits active=false from defaults
    assert!(!model.eq_active[0], "active should be false from defaults");
}

// ============================================================================
// T3: Equality defaults — no class → MuJoCo built-in → AC4
// ============================================================================

#[test]
fn t3_equality_defaults_no_class_builtin() {
    let mjcf = r#"
        <mujoco model="eq_no_class">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="b2" pos="0 0 2">
                    <joint name="j2" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <connect body1="b1" body2="b2" anchor="0 0 1.5"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // AC4: No class, no defaults → MuJoCo built-in [0.02, 1.0]
    assert_relative_eq!(model.eq_solref[0][0], 0.02, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solref[0][1], 1.0, epsilon = 1e-10);
    // MuJoCo built-in solimp: [0.9, 0.95, 0.001, 0.5, 2.0]
    assert_relative_eq!(model.eq_solimp[0][0], 0.9, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][1], 0.95, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][2], 0.001, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][3], 0.5, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][4], 2.0, epsilon = 1e-10);
}

// ============================================================================
// T4: qpos_spring — hinge with explicit springref → AC5
// ============================================================================

#[test]
fn t4_qpos_spring_hinge_springref() {
    let mjcf = r#"
        <mujoco model="qpos_spring_hinge">
            <compiler angle="radian"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0" springref="0.5"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let qpos_adr = model.jnt_qpos_adr[0];
    // AC5: qpos_spring contains springref value
    assert_relative_eq!(model.qpos_spring[qpos_adr], 0.5, epsilon = 1e-10);
}

// ============================================================================
// T5: qpos_spring — ball joint identity quaternion → AC6
// ============================================================================

#[test]
fn t5_qpos_spring_ball_identity() {
    let mjcf = r#"
        <mujoco model="qpos_spring_ball">
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="ball"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let qpos_adr = model.jnt_qpos_adr[0];
    // AC6: ball joint qpos_spring = identity quaternion [1, 0, 0, 0]
    assert_relative_eq!(model.qpos_spring[qpos_adr], 1.0, epsilon = 1e-10);
    assert_relative_eq!(model.qpos_spring[qpos_adr + 1], 0.0, epsilon = 1e-10);
    assert_relative_eq!(model.qpos_spring[qpos_adr + 2], 0.0, epsilon = 1e-10);
    assert_relative_eq!(model.qpos_spring[qpos_adr + 3], 0.0, epsilon = 1e-10);
}

// ============================================================================
// T6: qpos_spring — free joint body pose → AC7
// ============================================================================

#[test]
fn t6_qpos_spring_free_body_pose() {
    let mjcf = r#"
        <mujoco model="qpos_spring_free">
            <worldbody>
                <body name="b1" pos="1 2 3" quat="0.707107 0.707107 0 0">
                    <freejoint name="j1"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // AC7: free joint qpos_spring = 7D pose from qpos0 (non-identity proves real copy)
    assert_relative_eq!(model.qpos_spring[0], 1.0, epsilon = 1e-4);
    assert_relative_eq!(model.qpos_spring[1], 2.0, epsilon = 1e-4);
    assert_relative_eq!(model.qpos_spring[2], 3.0, epsilon = 1e-4);
    assert_relative_eq!(
        model.qpos_spring[3],
        std::f64::consts::FRAC_1_SQRT_2,
        epsilon = 1e-4
    );
    assert_relative_eq!(
        model.qpos_spring[4],
        std::f64::consts::FRAC_1_SQRT_2,
        epsilon = 1e-4
    );
    assert_relative_eq!(model.qpos_spring[5], 0.0, epsilon = 1e-4);
    assert_relative_eq!(model.qpos_spring[6], 0.0, epsilon = 1e-4);
}

// ============================================================================
// T7: Spring force regression — hinge/slide unchanged → AC8
// ============================================================================

#[test]
fn t7_spring_force_regression_hinge() {
    let mjcf = r#"
        <mujoco model="spring_regression">
            <compiler angle="radian"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"
                           stiffness="100.0" springref="0.0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    let mut data = model.make_data();
    data.qpos[0] = 0.5;
    data.forward(&model).expect("forward");

    // AC8: qfrc_spring = -k * (q - springref) = -100 * (0.5 - 0.0) = -50.0
    assert_relative_eq!(data.qfrc_spring[0], -50.0, epsilon = 1e-12);
}

// ============================================================================
// T8: Cylinder area defaults cascade → AC9
// ============================================================================

#[test]
fn t8_cylinder_area_defaults_cascade() {
    let mjcf = r#"
        <mujoco model="cylinder_defaults">
            <default>
                <cylinder area="0.01"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <cylinder joint="j1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // AC9: area → gainprm[0] for cylinder actuator
    assert_relative_eq!(model.actuator_gainprm[0][0], 0.01, epsilon = 1e-10);
}

// ============================================================================
// T9: Muscle range defaults cascade → AC10
// ============================================================================

#[test]
fn t9_muscle_range_defaults_cascade() {
    let mjcf = r#"
        <mujoco model="muscle_defaults">
            <default>
                <muscle range="0.5 1.2"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <muscle joint="j1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // AC10: range → gainprm[0..2] for muscle actuator
    assert_relative_eq!(model.actuator_gainprm[0][0], 0.5, epsilon = 1e-10);
    assert_relative_eq!(model.actuator_gainprm[0][1], 1.2, epsilon = 1e-10);
}

// ============================================================================
// T10: Tendon sentinel uses qpos_spring → AC11
// ============================================================================

#[test]
fn t10_tendon_sentinel_uses_qpos_spring() {
    // No explicit springlength → defaults to sentinel [-1, -1], which is resolved
    // at the qpos_spring configuration. springref=0.3 differs from ref=0.0,
    // proving resolution uses qpos_spring (not qpos0).
    let mjcf = r#"
        <mujoco model="tendon_sentinel_qpos_spring">
            <compiler angle="radian"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"
                           ref="0.0" springref="0.3" stiffness="100"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <tendon>
                <fixed name="t1" stiffness="100">
                    <joint joint="j1" coef="1.0"/>
                </fixed>
            </tendon>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // AC11: tendon_lengthspring resolved at qpos_spring (springref=0.3), not qpos0 (ref=0.0)
    // spring_length = coef * qpos_spring[0] = 1.0 * 0.3 = 0.3
    assert_relative_eq!(model.tendon_lengthspring[0][0], 0.3, epsilon = 1e-10);
    assert_relative_eq!(model.tendon_lengthspring[0][1], 0.3, epsilon = 1e-10);
    // tendon_length0 resolved at qpos0 (ref=0.0)
    assert_relative_eq!(model.tendon_length0[0], 0.0, epsilon = 1e-10);
}

// ============================================================================
// T11: Equality defaults — nested class inheritance → (supplementary)
// ============================================================================

#[test]
fn t11_equality_defaults_nested_class_inheritance() {
    let mjcf = r#"
        <mujoco model="eq_nested_class">
            <default>
                <default class="parent">
                    <equality solref="0.05 0.8"/>
                    <default class="child">
                        <equality solimp="0.8 0.9 0.01 0.4 3.0"/>
                    </default>
                </default>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="b2" pos="0 0 2">
                    <joint name="j2" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <equality>
                <connect class="child" body1="b1" body2="b2" anchor="0 0 1.5"/>
            </equality>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // Child inherits solref from parent
    assert_relative_eq!(model.eq_solref[0][0], 0.05, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solref[0][1], 0.8, epsilon = 1e-10);
    // Child has its own solimp
    assert_relative_eq!(model.eq_solimp[0][0], 0.8, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][1], 0.9, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][2], 0.01, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][3], 0.4, epsilon = 1e-10);
    assert_relative_eq!(model.eq_solimp[0][4], 3.0, epsilon = 1e-10);
}

// ============================================================================
// T12: Multiple actuator shortcuts in one default → (supplementary)
// ============================================================================

#[test]
fn t12_multiple_actuator_shortcuts_last_wins() {
    // When both <motor/> and <cylinder area="0.02"/> appear in same <default>,
    // the last one overwrites the first (struct replacement at parse level).
    let mjcf = r#"
        <mujoco model="multi_shortcut">
            <default>
                <motor/>
                <cylinder area="0.02"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <cylinder joint="j1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // Last-wins: cylinder's area=0.02 → gainprm[0]
    assert_relative_eq!(model.actuator_gainprm[0][0], 0.02, epsilon = 1e-10);
}

// ============================================================================
// T13: Mixed joint types — qpos_spring alignment → (supplementary)
// ============================================================================

#[test]
fn t13_mixed_joint_types_qpos_spring_alignment() {
    let mjcf = r#"
        <mujoco model="mixed_qpos_spring">
            <compiler angle="radian"/>
            <worldbody>
                <body name="hinge_body" pos="0 0 1">
                    <joint name="j_hinge" type="hinge" axis="0 1 0" springref="0.5"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="slide_body" pos="0 0 2">
                    <joint name="j_slide" type="slide" axis="0 0 1" springref="0.1"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="ball_body" pos="0 0 3">
                    <joint name="j_ball" type="ball"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
                <body name="free_body" pos="1 2 3">
                    <freejoint name="j_free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");

    // Hinge: 1 qpos, springref=0.5
    assert_relative_eq!(model.qpos_spring[0], 0.5, epsilon = 1e-10);
    // Slide: 1 qpos, springref=0.1
    assert_relative_eq!(model.qpos_spring[1], 0.1, epsilon = 1e-10);
    // Ball: 4 qpos, identity quaternion [1, 0, 0, 0]
    assert_relative_eq!(model.qpos_spring[2], 1.0, epsilon = 1e-10);
    assert_relative_eq!(model.qpos_spring[3], 0.0, epsilon = 1e-10);
    assert_relative_eq!(model.qpos_spring[4], 0.0, epsilon = 1e-10);
    assert_relative_eq!(model.qpos_spring[5], 0.0, epsilon = 1e-10);
    // Free: 7 qpos, [pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z]
    assert_relative_eq!(model.qpos_spring[6], 1.0, epsilon = 1e-4);
    assert_relative_eq!(model.qpos_spring[7], 2.0, epsilon = 1e-4);
    assert_relative_eq!(model.qpos_spring[8], 3.0, epsilon = 1e-4);
    assert_relative_eq!(model.qpos_spring[9], 1.0, epsilon = 1e-4);
    assert_relative_eq!(model.qpos_spring[10], 0.0, epsilon = 1e-4);
    assert_relative_eq!(model.qpos_spring[11], 0.0, epsilon = 1e-4);
    assert_relative_eq!(model.qpos_spring[12], 0.0, epsilon = 1e-4);

    // Total qpos_spring size: 1 + 1 + 4 + 7 = 13
    assert_eq!(model.qpos_spring.len(), 13);
}

// ============================================================================
// T14: Adhesion gain defaults cascade → AC13
// ============================================================================

#[test]
fn t14_adhesion_gain_defaults_cascade() {
    let mjcf = r#"
        <mujoco model="adhesion_defaults">
            <default>
                <adhesion gain="0.5"/>
            </default>
            <worldbody>
                <body name="floor" pos="0 0 0">
                    <geom name="g_floor" type="plane" size="5 5 0.1"/>
                </body>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <adhesion body="b1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // AC13: gain → gainprm[0] for adhesion actuator
    assert_relative_eq!(model.actuator_gainprm[0][0], 0.5, epsilon = 1e-10);
}

// ============================================================================
// T15: Damper kv defaults cascade → AC14
// ============================================================================

#[test]
fn t15_damper_kv_defaults_cascade() {
    let mjcf = r#"
        <mujoco model="damper_defaults">
            <default>
                <damper kv="5.0"/>
            </default>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <damper joint="j1"/>
            </actuator>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    // AC14: kv=5.0 → damper expansion: gainprm[2] = -kv = -5.0
    assert_relative_eq!(model.actuator_gainprm[0][2], -5.0, epsilon = 1e-10);
}
