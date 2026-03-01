//! Phase 6 Spec C — missing sensor types tests.
//!
//! Tests for §62: Clock, JointActuatorFrc, GeomDist, GeomNormal, GeomFromTo.
//! MuJoCo conformance tests use analytically derived expected values.

use approx::assert_relative_eq;
use sim_core::MjSensorType;
use sim_mjcf::load_model;

// ============================================================================
// T1: Clock at t=0 → AC1
// ============================================================================

#[test]
fn t01_clock_at_t0() {
    let mjcf = r#"
        <mujoco model="t01">
            <option timestep="0.01"/>
            <worldbody>
                <body name="b" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <clock name="clk"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    assert_eq!(model.sensor_type[0], MjSensorType::Clock);
    assert_eq!(model.sensor_dim[0], 1);

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // After mj_forward at t=0, clock reads 0.0
    assert_eq!(data.sensordata[0], 0.0);
}

// ============================================================================
// T2: Clock multi-step → AC2
// ============================================================================

#[test]
fn t02_clock_multi_step() {
    let mjcf = r#"
        <mujoco model="t02">
            <option timestep="0.01"/>
            <worldbody>
                <body name="b" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <clock name="clk"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // Step 3 times: sensor reads (N-1)*timestep = 2*0.01 = 0.02
    for _ in 0..3 {
        data.step(&model).expect("step");
    }

    assert_relative_eq!(data.sensordata[0], 0.02, epsilon = 1e-12);
}

// ============================================================================
// T3: JointActuatorFrc two-actuator → AC3
// ============================================================================

#[test]
fn t03_jointactuatorfrc_two_actuator() {
    let mjcf = r#"
        <mujoco model="t03">
            <worldbody>
                <body name="b" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <actuator>
                <motor name="m1" joint="j1" gear="1"/>
                <motor name="m2" joint="j1" gear="2"/>
            </actuator>
            <sensor>
                <jointactuatorfrc name="jaf" joint="j1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    assert_eq!(model.sensor_type[0], MjSensorType::JointActuatorFrc);

    let mut data = model.make_data();
    data.ctrl[0] = 3.0; // motor 1: force = 3.0 * 1.0 = 3.0
    data.ctrl[1] = 4.0; // motor 2: force = 4.0 * 2.0 = 8.0
    data.step(&model).expect("step");

    // Net actuator force: 3.0 + 8.0 = 11.0
    assert_relative_eq!(data.sensordata[0], 11.0, epsilon = 1e-6);
}

// ============================================================================
// T4: JointActuatorFrc zero-actuator → AC4
// ============================================================================

#[test]
fn t04_jointactuatorfrc_zero_actuator() {
    let mjcf = r#"
        <mujoco model="t04">
            <worldbody>
                <body name="b" pos="0 0 1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <jointactuatorfrc name="jaf" joint="j1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.step(&model).expect("step");

    // No actuators → qfrc_actuator = 0
    assert_eq!(data.sensordata[0], 0.0);
}

// ============================================================================
// T5: JointActuatorFrc ball joint rejection → AC5
// ============================================================================

#[test]
fn t05_jointactuatorfrc_ball_reject() {
    let mjcf = r#"
        <mujoco model="t05">
            <worldbody>
                <body name="b" pos="0 0 1">
                    <joint name="ball_j" type="ball"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <jointactuatorfrc name="jaf" joint="ball_j"/>
            </sensor>
        </mujoco>
    "#;

    let err = load_model(mjcf).unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("must be slide or hinge"),
        "expected 'must be slide or hinge' in error: {msg}"
    );
}

// ============================================================================
// T6: GeomDist sphere-sphere → AC6
// ============================================================================

#[test]
fn t06_geomdist_sphere_sphere() {
    let mjcf = r#"
        <mujoco model="t06">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.1" pos="0 0 0"/>
                </body>
                <body name="b2" pos="1 0 0">
                    <geom name="g2" type="sphere" size="0.2" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <distance name="d" geom1="g1" geom2="g2" cutoff="10"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    assert_eq!(model.sensor_type[0], MjSensorType::GeomDist);
    assert_eq!(model.sensor_dim[0], 1);

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Distance: |1.0 - 0.0| - 0.1 - 0.2 = 0.7
    assert_relative_eq!(data.sensordata[0], 0.7, epsilon = 1e-10);
}

// ============================================================================
// T7: GeomNormal sphere-sphere → AC7
// ============================================================================

#[test]
fn t07_geomnormal_sphere_sphere() {
    let mjcf = r#"
        <mujoco model="t07">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.1" pos="0 0 0"/>
                </body>
                <body name="b2" pos="1 0 0">
                    <geom name="g2" type="sphere" size="0.2" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <normal name="n" geom1="g1" geom2="g2" cutoff="10"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    assert_eq!(model.sensor_type[0], MjSensorType::GeomNormal);
    assert_eq!(model.sensor_dim[0], 3);

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Normal from g1 to g2: [1, 0, 0]
    let adr = model.sensor_adr[0];
    assert_relative_eq!(data.sensordata[adr], 1.0, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 1], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 2], 0.0, epsilon = 1e-10);
}

// ============================================================================
// T8: GeomFromTo sphere-sphere → AC8
// ============================================================================

#[test]
fn t08_geomfromto_sphere_sphere() {
    let mjcf = r#"
        <mujoco model="t08">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.1" pos="0 0 0"/>
                </body>
                <body name="b2" pos="1 0 0">
                    <geom name="g2" type="sphere" size="0.2" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <fromto name="ft" geom1="g1" geom2="g2" cutoff="10"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    assert_eq!(model.sensor_type[0], MjSensorType::GeomFromTo);
    assert_eq!(model.sensor_dim[0], 6);

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // fromto: [0.1, 0, 0, 0.8, 0, 0]
    let adr = model.sensor_adr[0];
    assert_relative_eq!(data.sensordata[adr], 0.1, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 1], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 2], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 3], 0.8, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 4], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 5], 0.0, epsilon = 1e-10);
}

// ============================================================================
// T9: Cutoff=0 non-penetrating → AC9
// ============================================================================

#[test]
fn t09_cutoff_zero_non_penetrating() {
    let mjcf = r#"
        <mujoco model="t09">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.1" pos="0 0 0"/>
                </body>
                <body name="b2" pos="1 0 0">
                    <geom name="g2" type="sphere" size="0.2" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <distance name="d" geom1="g1" geom2="g2"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Cutoff=0 means dist initialized to 0. dist_new=0.7 >= 0, so 0.7 < 0 is false.
    // dist stays 0.
    assert_eq!(data.sensordata[0], 0.0);
}

// ============================================================================
// T10: Cutoff=0 penetrating → AC10
// ============================================================================

#[test]
fn t10_cutoff_zero_penetrating() {
    let mjcf = r#"
        <mujoco model="t10">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.5" pos="0 0 0"/>
                </body>
                <body name="b2" pos="0.3 0 0">
                    <geom name="g2" type="sphere" size="0.5" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <distance name="d" geom1="g1" geom2="g2"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Overlapping: dist = 0.3 - 0.5 - 0.5 = -0.7. Penetration NOT suppressed.
    assert_relative_eq!(data.sensordata[0], -0.7, epsilon = 1e-10);
}

// ============================================================================
// T11: Multi-geom body distance → AC11
// ============================================================================

#[test]
fn t11_multi_geom_body_distance() {
    let mjcf = r#"
        <mujoco model="t11">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1a" type="sphere" size="0.1" pos="0 0 0"/>
                    <geom name="g1b" type="sphere" size="0.1" pos="0.3 0 0"/>
                </body>
                <body name="b2" pos="1 0 0">
                    <geom name="g2" type="sphere" size="0.1" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <distance name="d" body1="b1" body2="b2" cutoff="10"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // All-pairs: g1a↔g2 = |1.0-0.0|-0.1-0.1=0.8, g1b↔g2 = |1.0-0.3|-0.1-0.1=0.5
    // Minimum = 0.5
    assert_relative_eq!(data.sensordata[0], 0.5, epsilon = 1e-10);
}

// ============================================================================
// T12: GeomFromTo cutoff exemption → AC12
// ============================================================================

#[test]
fn t12_geomfromto_cutoff_exemption() {
    let mjcf = r#"
        <mujoco model="t12">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.5" pos="0 0 0"/>
                </body>
                <body name="b2" pos="0.3 0 0">
                    <geom name="g2" type="sphere" size="0.5" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <fromto name="ft" geom1="g1" geom2="g2" cutoff="0.5"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Postprocess does NOT clamp fromto values.
    // fromto = [0.5, 0, 0, -0.2, 0, 0] for penetrating spheres.
    let adr = model.sensor_adr[0];
    assert_relative_eq!(data.sensordata[adr], 0.5, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 1], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 2], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 3], -0.2, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 4], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 5], 0.0, epsilon = 1e-10);
}

// ============================================================================
// T13: GeomDist postprocess clamp → AC13
// ============================================================================

#[test]
fn t13_geomdist_postprocess_clamp() {
    let mjcf = r#"
        <mujoco model="t13">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.5" pos="0 0 0"/>
                </body>
                <body name="b2" pos="0.3 0 0">
                    <geom name="g2" type="sphere" size="0.5" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <distance name="d" geom1="g1" geom2="g2" cutoff="0.5"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Actual dist = -0.7, but postprocess clamps to [-0.5, 0.5]
    assert_relative_eq!(data.sensordata[0], -0.5, epsilon = 1e-10);
}

// ============================================================================
// T14: GeomNormal cutoff exemption → AC14
// ============================================================================

#[test]
fn t14_geomnormal_cutoff_exemption() {
    let mjcf = r#"
        <mujoco model="t14">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.5" pos="0 0 0"/>
                </body>
                <body name="b2" pos="0.3 0 0">
                    <geom name="g2" type="sphere" size="0.5" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <normal name="n" geom1="g1" geom2="g2" cutoff="0.5"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Normal = [-1, 0, 0] for penetrating, NOT clamped to [-0.5, 0.5]
    let adr = model.sensor_adr[0];
    assert_relative_eq!(data.sensordata[adr], -1.0, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 1], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 2], 0.0, epsilon = 1e-10);
}

// ============================================================================
// T15: Parser recognizes all 5 element names → AC15
// ============================================================================

#[test]
fn t15_parser_recognizes_new_elements() {
    let mjcf = r#"
        <mujoco model="t15">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom name="g1" type="sphere" size="0.1"/>
                </body>
                <body name="b2" pos="1 0 0">
                    <geom name="g2" type="sphere" size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <clock name="s_clock"/>
                <jointactuatorfrc name="s_jaf" joint="j1"/>
                <distance name="s_dist" geom1="g1" geom2="g2"/>
                <normal name="s_norm" geom1="g1" geom2="g2"/>
                <fromto name="s_ft" geom1="g1" geom2="g2"/>
            </sensor>
        </mujoco>
    "#;

    // Verify all 5 sensors parse and build successfully
    let model = load_model(mjcf).expect("load");
    assert_eq!(model.nsensor, 5);

    // Verify correct MjSensorType variants assigned
    assert_eq!(model.sensor_type[0], MjSensorType::Clock);
    assert_eq!(model.sensor_type[1], MjSensorType::JointActuatorFrc);
    assert_eq!(model.sensor_type[2], MjSensorType::GeomDist);
    assert_eq!(model.sensor_type[3], MjSensorType::GeomNormal);
    assert_eq!(model.sensor_type[4], MjSensorType::GeomFromTo);

    // Verify dimensions
    assert_eq!(model.sensor_dim[0], 1); // Clock
    assert_eq!(model.sensor_dim[1], 1); // JointActuatorFrc
    assert_eq!(model.sensor_dim[2], 1); // GeomDist
    assert_eq!(model.sensor_dim[3], 3); // GeomNormal
    assert_eq!(model.sensor_dim[4], 6); // GeomFromTo
}

// ============================================================================
// T16: Strict XOR validation → AC16
// ============================================================================

#[test]
fn t16_strict_xor_validation() {
    // Both geom1 and body1 on side 1 — should warn but still parse
    let mjcf = r#"
        <mujoco model="t16">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.1"/>
                </body>
                <body name="b2" pos="1 0 0">
                    <geom name="g2" type="sphere" size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <distance name="d" geom1="g1" body1="b1" geom2="g2"/>
            </sensor>
        </mujoco>
    "#;

    // Parser should succeed (warn but not fail)
    let parsed = sim_mjcf::parse_mjcf_str(mjcf).expect("parse");
    assert_eq!(parsed.sensors.len(), 1);
    // Both are populated (parser doesn't reject)
    assert!(parsed.sensors[0].geom1.is_some());
    assert!(parsed.sensors[0].body1.is_some());
}

// ============================================================================
// T17: Beyond-cutoff asymmetry → supplementary
// ============================================================================

#[test]
fn t17_beyond_cutoff_asymmetry() {
    let mjcf = r#"
        <mujoco model="t17">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.1" pos="0 0 0"/>
                </body>
                <body name="b2" pos="10 0 0">
                    <geom name="g2" type="sphere" size="0.1" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <distance name="d" geom1="g1" geom2="g2" cutoff="1"/>
                <normal name="n" geom1="g1" geom2="g2" cutoff="1"/>
                <fromto name="ft" geom1="g1" geom2="g2" cutoff="1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Distance → cutoff value (1.0)
    assert_relative_eq!(data.sensordata[model.sensor_adr[0]], 1.0, epsilon = 1e-10);

    // Normal → [0, 0, 0] (fromto zeroed, diff is zero, guard preserves zero)
    let n_adr = model.sensor_adr[1];
    assert_eq!(data.sensordata[n_adr], 0.0);
    assert_eq!(data.sensordata[n_adr + 1], 0.0);
    assert_eq!(data.sensordata[n_adr + 2], 0.0);

    // FromTo → [0, 0, 0, 0, 0, 0]
    let ft_adr = model.sensor_adr[2];
    for i in 0..6 {
        assert_eq!(data.sensordata[ft_adr + i], 0.0);
    }
}

// ============================================================================
// T18: FromTo ordering (swap geom order) → supplementary
// ============================================================================

#[test]
fn t18_fromto_swap_ordering() {
    let mjcf = r#"
        <mujoco model="t18">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.1" pos="0 0 0"/>
                </body>
                <body name="b2" pos="1 0 0">
                    <geom name="g2" type="sphere" size="0.2" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <fromto name="ft" geom1="g2" geom2="g1" cutoff="10"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Swapped: from=g2 surface toward g1, to=g1 surface toward g2
    // g2 at (1,0,0) r=0.2, g1 at (0,0,0) r=0.1
    // from = (1,0,0) + 0.2 * (-1,0,0) = (0.8, 0, 0)
    // to = (0,0,0) + 0.1 * (1,0,0) = (0.1, 0, 0)
    let adr = model.sensor_adr[0];
    assert_relative_eq!(data.sensordata[adr], 0.8, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[adr + 3], 0.1, epsilon = 1e-10);
}

// ============================================================================
// T19: Mixed objtype/reftype → supplementary
// ============================================================================

#[test]
fn t19_mixed_geom_body() {
    let mjcf = r#"
        <mujoco model="t19">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.1" pos="0 0 0"/>
                </body>
                <body name="b2" pos="1 0 0">
                    <geom name="g2" type="sphere" size="0.1" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <distance name="d" geom1="g1" body2="b2" cutoff="10"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // geom1=g1 (single) × body2=b2 (all geoms: just g2)
    // Distance = |1.0 - 0.0| - 0.1 - 0.1 = 0.8
    assert_relative_eq!(data.sensordata[0], 0.8, epsilon = 1e-10);
}

// ============================================================================
// T20: Clock with cutoff → supplementary
// ============================================================================

#[test]
fn t20_clock_with_cutoff() {
    let mjcf = r#"
        <mujoco model="t20">
            <option timestep="1"/>
            <worldbody>
                <body name="b" pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <clock name="clk" cutoff="5"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();

    // Step 10 times: time after 10 steps = 9.0 (reads (N-1)*timestep)
    // But cutoff=5 clamps to 5.0
    for _ in 0..10 {
        data.step(&model).expect("step");
    }

    assert_relative_eq!(data.sensordata[0], 5.0, epsilon = 1e-10);
}

// ============================================================================
// T22: Coincident geoms → supplementary
// ============================================================================

#[test]
fn t22_coincident_geoms() {
    let mjcf = r#"
        <mujoco model="t22">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.5" pos="0 0 0"/>
                </body>
                <body name="b2" pos="0 0 0">
                    <geom name="g2" type="sphere" size="0.5" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <distance name="d" geom1="g1" geom2="g2" cutoff="10"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Full penetration: distance = -2 * 0.5 = -1.0
    assert_relative_eq!(data.sensordata[0], -1.0, epsilon = 1e-10);
}

// ============================================================================
// T24: Self-distance rejection → supplementary
// ============================================================================

#[test]
fn t24_self_distance_rejection() {
    let mjcf = r#"
        <mujoco model="t24">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.1" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <distance name="d" geom1="g1" geom2="g1" cutoff="10"/>
            </sensor>
        </mujoco>
    "#;

    let err = load_model(mjcf).unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("must be different"),
        "expected self-distance rejection error: {msg}"
    );
}

// ============================================================================
// T26: Same-body geoms → supplementary
// ============================================================================

#[test]
fn t26_same_body_geoms() {
    let mjcf = r#"
        <mujoco model="t26">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <geom name="g1" type="sphere" size="0.1" pos="0 0 0"/>
                    <geom name="g2" type="sphere" size="0.1" pos="0.5 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <distance name="d" geom1="g1" geom2="g2" cutoff="10"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Distance: |0.5 - 0.0| - 0.1 - 0.1 = 0.3
    assert_relative_eq!(data.sensordata[0], 0.3, epsilon = 1e-10);
}
