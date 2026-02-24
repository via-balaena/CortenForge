//! MJCF → MuJoCo pipeline sensor wiring integration tests.
//!
//! These tests verify the full round-trip from MJCF XML sensor declarations
//! through `builder/` wiring to `sensor/` evaluation.
//! They exercise `load_model()` → `make_data()` → `forward()` → check `sensordata`.
//!
//! Note: `sensors.rs` in this directory tests the standalone `sim-sensor` crate.
//! This file tests the MuJoCo pipeline's MJCF→sensor wiring specifically.

use approx::assert_relative_eq;
use sim_core::{MjObjectType, MjSensorDataType, MjSensorType};
use sim_mjcf::load_model;

// ============================================================================
// Position-stage sensors
// ============================================================================

/// JointPos sensor: MJCF `<jointpos joint="..."/>` reads `qpos` for the joint.
#[test]
fn test_jointpos_sensor_roundtrip() {
    let mjcf = r#"
        <mujoco model="sensor_test">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="hinge1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos joint="hinge1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.nsensor, 1);
    assert_eq!(model.nsensordata, 1);
    assert_eq!(model.sensor_type[0], MjSensorType::JointPos);
    assert_eq!(model.sensor_datatype[0], MjSensorDataType::Position);
    assert_eq!(model.sensor_objtype[0], MjObjectType::Joint);

    let mut data = model.make_data();
    data.qpos[model.jnt_qpos_adr[0]] = 0.5; // set joint angle
    data.forward(&model).expect("forward");
    assert!((data.sensordata[0] - 0.5).abs() < 1e-12);
}

/// FramePos sensor targeting a site: reads `data.site_xpos[id]`.
#[test]
fn test_framepos_sensor_site() {
    let mjcf = r#"
        <mujoco model="framepos_site">
            <worldbody>
                <body name="b1" pos="1 2 3">
                    <joint name="j1" type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="s1" pos="0.5 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <framepos site="s1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.nsensor, 1);
    assert_eq!(model.nsensordata, 3); // 3D position
    assert_eq!(model.sensor_type[0], MjSensorType::FramePos);
    assert_eq!(model.sensor_datatype[0], MjSensorDataType::Position);
    assert_eq!(model.sensor_objtype[0], MjObjectType::Site);

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Site s1 is at body-local offset (0.5, 0, 0) relative to body at (1, 2, 3).
    // With identity rotation, world position = body_pos + site_pos = (1.5, 2, 3).
    let site_id = model.sensor_objid[0];
    assert_relative_eq!(
        data.sensordata[0],
        data.site_xpos[site_id].x,
        epsilon = 1e-10
    );
    assert_relative_eq!(
        data.sensordata[1],
        data.site_xpos[site_id].y,
        epsilon = 1e-10
    );
    assert_relative_eq!(
        data.sensordata[2],
        data.site_xpos[site_id].z,
        epsilon = 1e-10
    );
}

/// Magnetometer sensor: reads global magnetic field rotated into site frame.
/// With identity site rotation and default magnetic field (0, -0.5, 0),
/// sensordata should be (0, -0.5, 0).
#[test]
fn test_magnetometer_sensor_roundtrip() {
    let mjcf = r#"
        <mujoco model="magnetometer_test">
            <option magnetic="0 -0.5 0"/>
            <worldbody>
                <body name="b1" pos="0 0 1">
                    <joint name="j1" type="free"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="mag_site" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <magnetometer site="mag_site"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.nsensor, 1);
    assert_eq!(model.nsensordata, 3);
    assert_eq!(model.sensor_type[0], MjSensorType::Magnetometer);
    assert_eq!(model.sensor_datatype[0], MjSensorDataType::Position);

    // Verify the magnetic field was wired through set_options()
    assert_relative_eq!(model.magnetic.y, -0.5, epsilon = 1e-12);

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Identity site rotation → magnetic field passes through unchanged
    assert_relative_eq!(data.sensordata[0], 0.0, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[1], -0.5, epsilon = 1e-10);
    assert_relative_eq!(data.sensordata[2], 0.0, epsilon = 1e-10);
}

/// TendonPos sensor: reads `data.ten_length[id]` for a spatial tendon.
#[test]
fn test_tendonpos_sensor() {
    let mjcf = r#"
        <mujoco model="tendonpos_test">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <site name="s1" pos="0 0 0"/>
                </body>
                <body name="b2" pos="0 0 1">
                    <site name="s2" pos="0 0 0"/>
                </body>
            </worldbody>
            <tendon>
                <spatial name="cable1">
                    <site site="s1"/>
                    <site site="s2"/>
                </spatial>
            </tendon>
            <sensor>
                <tendonpos tendon="cable1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.nsensor, 1);
    assert_eq!(model.nsensordata, 1); // scalar
    assert_eq!(model.sensor_type[0], MjSensorType::TendonPos);
    assert_eq!(model.sensor_datatype[0], MjSensorDataType::Position);
    assert_eq!(model.sensor_objtype[0], MjObjectType::Tendon);

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Tendon length should match ten_length computed by the pipeline
    let tendon_id = model.sensor_objid[0];
    assert_relative_eq!(
        data.sensordata[0],
        data.ten_length[tendon_id],
        epsilon = 1e-10
    );
}

/// ActuatorPos sensor: computes `gear * qpos[qpos_adr]` inline for joint transmission.
#[test]
fn test_actuatorpos_sensor() {
    let mjcf = r#"
        <mujoco model="actuatorpos_test">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <actuator>
                <motor name="motor1" joint="j1" gear="10"/>
            </actuator>
            <sensor>
                <actuatorpos actuator="motor1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.nsensor, 1);
    assert_eq!(model.nsensordata, 1);
    assert_eq!(model.sensor_type[0], MjSensorType::ActuatorPos);
    assert_eq!(model.sensor_datatype[0], MjSensorDataType::Position);
    assert_eq!(model.sensor_objtype[0], MjObjectType::Actuator);

    let mut data = model.make_data();
    data.qpos[0] = 0.3; // set joint angle
    data.forward(&model).expect("forward");

    // ActuatorPos = gear * qpos = 10 * 0.3 = 3.0
    assert_relative_eq!(data.sensordata[0], 3.0, epsilon = 1e-10);
}

// ============================================================================
// Edge cases
// ============================================================================

/// JointLimitFrc sensor is now supported — verify it wires correctly.
#[test]
fn test_joint_limit_frc_sensor() {
    let mjcf = r#"
        <mujoco model="jlf_test">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <sensor>
                <jointlimitfrc joint="j1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.nsensor, 1);
    assert_eq!(model.nsensordata, 1);
    assert_eq!(model.sensor_type[0], MjSensorType::JointLimitFrc);
}

/// Missing reference (nonexistent joint) should produce an error.
#[test]
fn test_missing_reference_error() {
    let mjcf = r#"
        <mujoco model="missing_ref">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos joint="nonexistent"/>
            </sensor>
        </mujoco>
    "#;

    let result = load_model(mjcf);
    assert!(
        result.is_err(),
        "should fail for nonexistent joint reference"
    );
}

/// Multiple sensors of different types in one model.
#[test]
fn test_multiple_sensor_types() {
    let mjcf = r#"
        <mujoco model="multi_sensor">
            <worldbody>
                <body name="b1" pos="0 0 0">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1.0"/>
                    <site name="s1" pos="0 0 0"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos joint="j1"/>
                <gyro site="s1"/>
                <accelerometer site="s1"/>
            </sensor>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("should load");
    assert_eq!(model.nsensor, 3);
    // jointpos=1D + gyro=3D + accelerometer=3D = 7D
    assert_eq!(model.nsensordata, 7);
    assert_eq!(model.sensor_type[0], MjSensorType::JointPos);
    assert_eq!(model.sensor_type[1], MjSensorType::Gyro);
    assert_eq!(model.sensor_type[2], MjSensorType::Accelerometer);

    // Check datatypes span all three stages
    assert_eq!(model.sensor_datatype[0], MjSensorDataType::Position);
    assert_eq!(model.sensor_datatype[1], MjSensorDataType::Velocity);
    assert_eq!(model.sensor_datatype[2], MjSensorDataType::Acceleration);

    // Check sensor addresses are contiguous
    assert_eq!(model.sensor_adr[0], 0); // jointpos at offset 0
    assert_eq!(model.sensor_adr[1], 1); // gyro at offset 1
    assert_eq!(model.sensor_adr[2], 4); // accelerometer at offset 4

    let mut data = model.make_data();
    data.qpos[0] = 1.0;
    data.forward(&model).expect("forward");

    // JointPos should read the joint angle
    assert_relative_eq!(data.sensordata[0], 1.0, epsilon = 1e-12);
}
