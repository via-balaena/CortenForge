//! Phase 6 Spec D — sensor history attributes tests.
//!
//! Tests for DT-109: `nsample`, `interp`, `delay`, `interval` parsing,
//! validation, and historyadr/nhistory computation for sensors.

use sim_core::InterpolationType;
use sim_mjcf::load_model;

// ============================================================================
// T1: Parse basic history attributes → AC1, AC2, AC3
// ============================================================================

#[test]
fn spec_d_t01_parse_basic_history_attrs() {
    let mjcf = r#"
        <mujoco model="spec_d_t01">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <geom size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="s" joint="j1" nsample="5" interp="linear" delay="0.02"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    assert_eq!(model.sensor_nsample[0], 5);
    assert_eq!(model.sensor_interp[0], InterpolationType::Linear);
    assert!((model.sensor_delay[0] - 0.02).abs() < 1e-15);
}

// ============================================================================
// T2: Parse interval attribute → AC4
// ============================================================================

#[test]
fn spec_d_t02_parse_interval() {
    let mjcf = r#"
        <mujoco model="spec_d_t02">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <geom size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="s" joint="j1" interval="0.5"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    assert_eq!(model.sensor_interval[0], (0.5, 0.0));
}

// ============================================================================
// T3: nhistory formula — dim=1, dim=3, dim=4 → AC5, AC6
// ============================================================================

#[test]
fn spec_d_t03_nhistory_formula_multi_dim() {
    let mjcf = r#"
        <mujoco model="spec_d_t03">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <geom size="0.1"/>
                    <site name="s1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="sp" joint="j1" nsample="5"/>
                <framepos name="fp" site="s1" nsample="3"/>
                <framequat name="fq" site="s1" nsample="1"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");

    // jointpos dim=1: 5*(1+1)+2 = 12, historyadr=0
    assert_eq!(model.sensor_historyadr[0], 0);
    // framepos dim=3: 3*(3+1)+2 = 14, historyadr=12
    assert_eq!(model.sensor_historyadr[1], 12);
    // framequat dim=4: 1*(4+1)+2 = 7, historyadr=26
    assert_eq!(model.sensor_historyadr[2], 26);
    // total: 12 + 14 + 7 = 33
    assert_eq!(model.nhistory, 33);
}

// ============================================================================
// T4: Mixed actuator + sensor buffer layout → AC7
// ============================================================================

#[test]
fn spec_d_t04_mixed_actuator_sensor_layout() {
    let mjcf = r#"
        <mujoco model="spec_d_t04">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <geom size="0.1"/>
                </body>
            </worldbody>
            <actuator>
                <motor joint="j1" nsample="4"/>
            </actuator>
            <sensor>
                <jointpos name="s" joint="j1" nsample="3"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    // actuator dim=1: 4*2+2 = 10, historyadr=0
    assert_eq!(model.actuator_historyadr[0], 0);
    // sensor dim=1: 3*2+2 = 8, historyadr=10
    assert_eq!(model.sensor_historyadr[0], 10);
    // total: 10 + 8 = 18
    assert_eq!(model.nhistory, 18);
}

// ============================================================================
// T5: Default values — no history attrs → AC8
// ============================================================================

#[test]
fn spec_d_t05_defaults_no_history_attrs() {
    let mjcf = r#"
        <mujoco model="spec_d_t05">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <geom size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="s" joint="j1"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    assert_eq!(model.sensor_nsample[0], 0);
    assert_eq!(model.sensor_interp[0], InterpolationType::Zoh);
    assert!((model.sensor_delay[0] - 0.0).abs() < 1e-15);
    assert_eq!(model.sensor_interval[0], (0.0, 0.0));
    assert_eq!(model.sensor_historyadr[0], -1);
    assert_eq!(model.nhistory, 0);
}

// ============================================================================
// T6: Validation — delay without nsample → AC9, AC23
// ============================================================================

#[test]
fn spec_d_t06_delay_without_nsample_error() {
    // delay > 0 without nsample → error
    let mjcf = r#"
        <mujoco model="spec_d_t06a">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <geom size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="s" joint="j1" delay="0.01"/>
            </sensor>
        </mujoco>
    "#;
    let err = load_model(mjcf).unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("setting delay > 0 without a history buffer"),
        "Expected delay/nsample error, got: {msg}"
    );

    // delay + interval without nsample → still delay error (AC23)
    let mjcf = r#"
        <mujoco model="spec_d_t06b">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <geom size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="s" joint="j1" delay="0.01" interval="0.5"/>
            </sensor>
        </mujoco>
    "#;
    let err = load_model(mjcf).unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("setting delay > 0 without a history buffer"),
        "Expected delay/nsample error, got: {msg}"
    );
}

// ============================================================================
// T7: Validation — negative interval → AC10
// ============================================================================

#[test]
fn spec_d_t07_negative_interval_error() {
    let mjcf = r#"
        <mujoco model="spec_d_t07">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <geom size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="s" joint="j1" interval="-0.5"/>
            </sensor>
        </mujoco>
    "#;
    let err = load_model(mjcf).unwrap_err();
    let msg = format!("{err}");
    assert!(
        msg.contains("negative interval in sensor"),
        "Expected negative interval error, got: {msg}"
    );
}

// ============================================================================
// T8: Invalid interp keywords → AC11, AC20
// ============================================================================

#[test]
fn spec_d_t08_invalid_interp_keywords() {
    let cases = ["Linear", "ZOH", "1", ""];
    for bad in &cases {
        let mjcf = format!(
            r#"
            <mujoco model="spec_d_t08">
                <worldbody>
                    <body>
                        <joint type="hinge" name="j1"/>
                        <geom size="0.1"/>
                    </body>
                </worldbody>
                <sensor>
                    <jointpos name="s" joint="j1" nsample="5" interp="{bad}"/>
                </sensor>
            </mujoco>
            "#
        );
        let err = load_model(&mjcf).unwrap_err();
        let msg = format!("{err}");
        assert!(
            msg.contains("invalid interp keyword"),
            "interp=\"{bad}\" should fail with 'invalid interp keyword', got: {msg}"
        );
    }
}

// ============================================================================
// T9: Multi-sensor historyadr accumulation → AC12, AC13, AC22
// ============================================================================

#[test]
fn spec_d_t09_multi_sensor_historyadr() {
    // (a) 3 sensors all with history
    let mjcf = r#"
        <mujoco model="spec_d_t09a">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <joint type="hinge" name="j2"/>
                    <geom size="0.1"/>
                    <site name="s1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="sp1" joint="j1" nsample="3"/>
                <jointpos name="sp2" joint="j2" nsample="5"/>
                <framepos name="fp" site="s1" nsample="2"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    // jointpos dim=1: 3*2+2=8, historyadr=0
    // jointpos dim=1: 5*2+2=12, historyadr=8
    // framepos dim=3: 2*4+2=10, historyadr=20
    assert_eq!(model.sensor_historyadr, vec![0, 8, 20]);
    assert_eq!(model.nhistory, 30);
    // AC22: vec-length invariant
    assert_eq!(model.sensor_nsample.len(), 3);
    assert_eq!(model.sensor_interp.len(), 3);
    assert_eq!(model.sensor_delay.len(), 3);
    assert_eq!(model.sensor_interval.len(), 3);
    assert_eq!(model.sensor_historyadr.len(), 3);

    // (b) nsample=0 in middle doesn't break chain (AC13)
    let mjcf = r#"
        <mujoco model="spec_d_t09b">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <joint type="hinge" name="j2"/>
                    <joint type="hinge" name="j3"/>
                    <geom size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="sp1" joint="j1" nsample="3"/>
                <jointpos name="sp2" joint="j2"/>
                <jointpos name="sp3" joint="j3" nsample="5"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    // sensor 0: 3*2+2=8, historyadr=0
    // sensor 1: no nsample → historyadr=-1
    // sensor 2: 5*2+2=12, historyadr=8
    assert_eq!(model.sensor_historyadr, vec![0, -1, 8]);
    assert_eq!(model.nhistory, 20);
}

// ============================================================================
// T10: Full combination — all 4 attrs → AC14
// ============================================================================

#[test]
fn spec_d_t10_full_combination() {
    let mjcf = r#"
        <mujoco model="spec_d_t10">
            <worldbody>
                <body>
                    <joint type="free"/>
                    <geom size="0.1"/>
                    <site name="s1"/>
                </body>
            </worldbody>
            <sensor>
                <framepos name="fp" site="s1" nsample="5" interp="cubic" delay="0.02" interval="0.1"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    assert_eq!(model.sensor_nsample[0], 5);
    assert_eq!(model.sensor_interp[0], InterpolationType::Cubic);
    assert!((model.sensor_delay[0] - 0.02).abs() < 1e-15);
    assert_eq!(model.sensor_interval[0], (0.1, 0.0));
    assert_eq!(model.sensor_historyadr[0], 0);
    // framepos dim=3: 5*(3+1)+2 = 22
    assert_eq!(model.nhistory, 22);
}

// ============================================================================
// T11: Actuator stability — sensors don't shift actuator offsets → AC15
// ============================================================================

#[test]
fn spec_d_t11_actuator_stability() {
    let mjcf = r#"
        <mujoco model="spec_d_t11">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <joint type="hinge" name="j2"/>
                    <geom size="0.1"/>
                </body>
            </worldbody>
            <actuator>
                <motor joint="j1" nsample="4"/>
                <motor joint="j2" nsample="2"/>
            </actuator>
            <sensor>
                <jointpos name="sp1" joint="j1" nsample="3"/>
                <jointpos name="sp2" joint="j2" nsample="5"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    // actuator 0: 4*2+2=10, historyadr=0
    // actuator 1: 2*2+2=6, historyadr=10
    assert_eq!(model.actuator_historyadr, vec![0, 10]);
    // sensor 0: 3*2+2=8, historyadr=16
    // sensor 1: 5*2+2=12, historyadr=24
    assert_eq!(model.sensor_historyadr, vec![16, 24]);
    // total: 10 + 6 + 8 + 12 = 36
    assert_eq!(model.nhistory, 36);
}

// ============================================================================
// T12: Negative nsample and negative delay accepted → AC16, AC17
// ============================================================================

#[test]
fn spec_d_t12_negative_nsample_accepted() {
    let mjcf = r#"
        <mujoco model="spec_d_t12a">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <geom size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="s" joint="j1" nsample="-1"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    assert_eq!(model.sensor_nsample[0], -1);
    assert_eq!(model.sensor_historyadr[0], -1);
    assert_eq!(model.nhistory, 0);
}

#[test]
fn spec_d_t12_negative_delay_accepted() {
    let mjcf = r#"
        <mujoco model="spec_d_t12b">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <geom size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="s" joint="j1" nsample="5" delay="-0.01"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    assert!((model.sensor_delay[0] - (-0.01)).abs() < 1e-15);
}

// ============================================================================
// T13: Attributes stored without nsample → AC18, AC19
// ============================================================================

#[test]
fn spec_d_t13_interval_without_nsample() {
    let mjcf = r#"
        <mujoco model="spec_d_t13a">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <geom size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="s" joint="j1" interval="0.5"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    assert_eq!(model.sensor_interval[0], (0.5, 0.0));
    assert_eq!(model.sensor_historyadr[0], -1);
}

#[test]
fn spec_d_t13_interp_without_nsample() {
    let mjcf = r#"
        <mujoco model="spec_d_t13b">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <geom size="0.1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="s" joint="j1" interp="linear"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    assert_eq!(model.sensor_interp[0], InterpolationType::Linear);
    assert_eq!(model.sensor_historyadr[0], -1);
}

// ============================================================================
// T14: Sensor-only model (no actuators) → AC21
// ============================================================================

#[test]
fn spec_d_t14_sensor_only_model() {
    let mjcf = r#"
        <mujoco model="spec_d_t14">
            <worldbody>
                <body>
                    <joint type="free"/>
                    <geom size="0.1"/>
                    <site name="s1"/>
                </body>
            </worldbody>
            <sensor>
                <framepos name="fp" site="s1" nsample="3"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    // framepos dim=3: 3*(3+1)+2 = 14, historyadr=0
    assert_eq!(model.sensor_historyadr[0], 0);
    assert_eq!(model.nhistory, 14);
    assert!(model.actuator_historyadr.is_empty());
}

// ============================================================================
// T15: Build regression — existing model without history attrs
// ============================================================================

#[test]
fn spec_d_t15_build_regression() {
    let mjcf = r#"
        <mujoco model="spec_d_t15">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <joint type="hinge" name="j2"/>
                    <geom size="0.1"/>
                    <site name="s1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="sp" joint="j1"/>
                <jointvel name="sv" joint="j2"/>
                <framepos name="fp" site="s1"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    assert_eq!(model.nsensor, 3);
    // 1 + 1 + 3 = 5
    assert_eq!(model.nsensordata, 5);
    assert_eq!(model.nhistory, 0);
    // All new vecs must have len == nsensor
    assert_eq!(model.sensor_nsample.len(), 3);
    assert_eq!(model.sensor_interp.len(), 3);
    assert_eq!(model.sensor_historyadr.len(), 3);
    assert_eq!(model.sensor_delay.len(), 3);
    assert_eq!(model.sensor_interval.len(), 3);
}

// ============================================================================
// T16: Mixed actuator + multi-dim sensor layout → AC24
// ============================================================================

#[test]
fn spec_d_t16_mixed_actuator_multidim_sensor() {
    let mjcf = r#"
        <mujoco model="spec_d_t16">
            <worldbody>
                <body>
                    <joint type="hinge" name="j1"/>
                    <geom size="0.1"/>
                    <site name="s1"/>
                </body>
            </worldbody>
            <actuator>
                <motor joint="j1" nsample="4"/>
            </actuator>
            <sensor>
                <framepos name="fp" site="s1" nsample="3"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).expect("should build");
    // actuator dim=1: 4*2+2=10, historyadr=0
    assert_eq!(model.actuator_historyadr[0], 0);
    // framepos dim=3: 3*(3+1)+2=14, historyadr=10
    assert_eq!(model.sensor_historyadr[0], 10);
    // total: 10 + 14 = 24
    assert_eq!(model.nhistory, 24);
}
