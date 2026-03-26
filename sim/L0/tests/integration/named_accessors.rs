//! Tests for convenience name-lookup and state-accessor APIs.
//!
//! Verifies `Model::joint_id()`, `Model::sensor_id()`, etc. and
//! `Data::joint_qpos()`, `Data::joint_qvel()`, `Data::sensor_data()`.

use sim_mjcf::load_model;

// ============================================================================
// Model name lookups
// ============================================================================

#[test]
fn model_joint_id_by_name() {
    let mjcf = r#"
        <mujoco>
            <worldbody>
                <body name="b1">
                    <joint name="hinge1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                    <body name="b2">
                        <joint name="slide1" type="slide" axis="1 0 0"/>
                        <geom type="sphere" size="0.1" mass="1"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    assert_eq!(model.joint_id("hinge1"), Some(0));
    assert_eq!(model.joint_id("slide1"), Some(1));
    assert_eq!(model.joint_id("nonexistent"), None);
}

#[test]
fn model_body_id_by_name() {
    let mjcf = r#"
        <mujoco>
            <worldbody>
                <body name="arm">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                    <body name="forearm">
                        <joint name="j2" type="hinge" axis="0 1 0"/>
                        <geom type="sphere" size="0.1" mass="1"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    assert_eq!(model.body_id("arm"), Some(1)); // 0 = world
    assert_eq!(model.body_id("forearm"), Some(2));
    assert_eq!(model.body_id("nonexistent_body"), None);
}

#[test]
fn model_sensor_id_by_name() {
    let mjcf = r#"
        <mujoco>
            <worldbody>
                <body name="b1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="pos_sensor" joint="j1"/>
                <jointvel name="vel_sensor" joint="j1"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    assert_eq!(model.sensor_id("pos_sensor"), Some(0));
    assert_eq!(model.sensor_id("vel_sensor"), Some(1));
    assert_eq!(model.sensor_id("nope"), None);
}

#[test]
fn model_actuator_id_by_name() {
    let mjcf = r#"
        <mujoco>
            <worldbody>
                <body name="b1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <actuator>
                <motor name="motor1" joint="j1" gear="1"/>
            </actuator>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    assert_eq!(model.actuator_id("motor1"), Some(0));
    assert_eq!(model.actuator_id("missing"), None);
}

#[test]
fn model_geom_id_by_name() {
    let mjcf = r#"
        <mujoco>
            <worldbody>
                <body name="b1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom name="ball" type="sphere" size="0.1" mass="1"/>
                    <geom name="rod" type="capsule" size="0.02 0.2" mass="0.5"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    assert!(model.geom_id("ball").is_some());
    assert!(model.geom_id("rod").is_some());
    assert_ne!(model.geom_id("ball"), model.geom_id("rod"));
    assert_eq!(model.geom_id("x"), None);
}

// ============================================================================
// Data state accessors
// ============================================================================

#[test]
fn data_joint_qpos_hinge() {
    let mjcf = r#"
        <mujoco>
            <worldbody>
                <body name="b1">
                    <joint name="hinge1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Hinge: 1 qpos element
    let qpos = data.joint_qpos(&model, 0);
    assert_eq!(qpos.len(), 1);
    assert_eq!(qpos[0], 0.0); // initial position

    // Write via mutable accessor
    data.joint_qpos_mut(&model, 0)[0] = 1.5;
    assert_eq!(data.joint_qpos(&model, 0)[0], 1.5);
}

#[test]
fn data_joint_qpos_slide() {
    let mjcf = r#"
        <mujoco>
            <worldbody>
                <body name="b1">
                    <joint name="slide1" type="slide" axis="1 0 0"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Slide: 1 qpos element
    let qpos = data.joint_qpos(&model, 0);
    assert_eq!(qpos.len(), 1);

    data.joint_qpos_mut(&model, 0)[0] = -0.3;
    assert_eq!(data.joint_qpos(&model, 0)[0], -0.3);
}

#[test]
fn data_joint_qpos_ball() {
    let mjcf = r#"
        <mujoco>
            <worldbody>
                <body name="b1">
                    <joint name="ball1" type="ball"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let data = model.make_data();

    // Ball: 4 qpos elements (quaternion w, x, y, z)
    let qpos = data.joint_qpos(&model, 0);
    assert_eq!(qpos.len(), 4);
    // Default quaternion: (1, 0, 0, 0)
    assert!((qpos[0] - 1.0).abs() < 1e-10);
    assert!(qpos[1].abs() < 1e-10);
}

#[test]
fn data_joint_qpos_free() {
    let mjcf = r#"
        <mujoco>
            <worldbody>
                <body name="b1">
                    <joint name="free1" type="free"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let data = model.make_data();

    // Free: 7 qpos elements (x, y, z, qw, qx, qy, qz)
    let qpos = data.joint_qpos(&model, 0);
    assert_eq!(qpos.len(), 7);
}

#[test]
fn data_joint_qvel_dimensions() {
    let mjcf = r#"
        <mujoco>
            <worldbody>
                <body name="b1">
                    <joint name="hinge1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                    <body name="b2">
                        <joint name="ball1" type="ball"/>
                        <geom type="sphere" size="0.1" mass="1"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    // Hinge: 1 DOF
    assert_eq!(data.joint_qvel(&model, 0).len(), 1);
    // Ball: 3 DOFs
    assert_eq!(data.joint_qvel(&model, 1).len(), 3);

    // Mutable write persists
    data.joint_qvel_mut(&model, 0)[0] = 2.5;
    assert_eq!(data.joint_qvel(&model, 0)[0], 2.5);
}

#[test]
fn data_joint_qpos_by_name() {
    let mjcf = r#"
        <mujoco>
            <worldbody>
                <body name="b1">
                    <joint name="elbow" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();

    let jid = model.joint_id("elbow").unwrap();
    data.joint_qpos_mut(&model, jid)[0] = 0.42;
    assert!((data.joint_qpos(&model, jid)[0] - 0.42).abs() < 1e-15);
}

#[test]
fn data_sensor_data_dimensions() {
    let mjcf = r#"
        <mujoco>
            <worldbody>
                <body name="b1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                    <site name="s1" pos="0 0 0.1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos name="jpos" joint="j1"/>
                <gyro name="gyro1" site="s1"/>
                <clock name="clk"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();
    let _ = data.forward(&model);

    // JointPos: 1D
    let sid_jpos = model.sensor_id("jpos").unwrap();
    assert_eq!(data.sensor_data(&model, sid_jpos).len(), 1);

    // Gyro: 3D
    let sid_gyro = model.sensor_id("gyro1").unwrap();
    assert_eq!(data.sensor_data(&model, sid_gyro).len(), 3);

    // Clock: 1D, value == data.time
    let sid_clk = model.sensor_id("clk").unwrap();
    let clk = data.sensor_data(&model, sid_clk);
    assert_eq!(clk.len(), 1);
    assert!((clk[0] - data.time).abs() < 1e-15);
}

#[test]
fn data_sensor_data_matches_raw_sensordata() {
    let mjcf = r#"
        <mujoco>
            <worldbody>
                <body name="b1">
                    <joint name="j1" type="hinge" axis="0 1 0"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                </body>
            </worldbody>
            <sensor>
                <jointpos joint="j1"/>
                <jointvel joint="j1"/>
            </sensor>
        </mujoco>
    "#;
    let model = load_model(mjcf).unwrap();
    let mut data = model.make_data();
    data.qpos[0] = 0.7;
    data.qvel[0] = -1.2;
    let _ = data.forward(&model);

    // Accessor should return the same values as raw indexing
    let s0 = data.sensor_data(&model, 0);
    assert_eq!(s0.len(), 1);
    assert!((s0[0] - data.sensordata[model.sensor_adr[0]]).abs() < 1e-15);

    let s1 = data.sensor_data(&model, 1);
    assert_eq!(s1.len(), 1);
    assert!((s1[0] - data.sensordata[model.sensor_adr[1]]).abs() < 1e-15);
}
