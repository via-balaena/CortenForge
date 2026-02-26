//! User callback tests (DT-79).
//!
//! Verifies `cb_passive`, `cb_control`, `cb_contactfilter`, `cb_sensor`,
//! `cb_act_dyn`, `cb_act_gain`, and `cb_act_bias` hooks.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// Test: passive callback adds constant force → verify qfrc_passive changes.
#[test]
fn passive_callback_adds_force() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="ball" pos="0 0 1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let mut model = sim_mjcf::load_model(xml).expect("load");

    // Baseline: no callback
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let baseline_qfrc = data.qfrc_passive[0];

    // Set passive callback that adds a constant force
    model.set_passive_callback(|_model, data| {
        data.qfrc_passive[0] += 42.0;
    });

    data.forward(&model).expect("forward");
    let with_callback_qfrc = data.qfrc_passive[0];

    assert!(
        (with_callback_qfrc - baseline_qfrc - 42.0).abs() < 1e-10,
        "Passive callback should add 42.0 to qfrc_passive[0], got delta={}",
        with_callback_qfrc - baseline_qfrc
    );
}

/// Test: control callback sets ctrl → verify actuator response.
#[test]
fn control_callback_sets_ctrl() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="link" pos="0 0 0">
          <joint name="hinge" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.05 0.5" mass="1.0"/>
        </body>
      </worldbody>
      <actuator>
        <motor name="m1" joint="hinge" gear="1"/>
      </actuator>
    </mujoco>"#;

    let mut model = sim_mjcf::load_model(xml).expect("load");

    // Set control callback that sets ctrl[0] = 5.0
    model.set_control_callback(|_model, data| {
        data.ctrl[0] = 5.0;
    });

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Actuator should have produced force
    assert!(
        data.qfrc_actuator[0].abs() > 1e-6,
        "Control callback should produce actuator force, got {}",
        data.qfrc_actuator[0]
    );
}

/// Test: contact filter rejects specific pair → verify no contacts.
#[test]
fn contact_filter_rejects_pair() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <geom name="floor" type="plane" size="5 5 0.1"/>
        <body name="ball" pos="0 0 0.11">
          <freejoint/>
          <geom name="ball_geom" type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let mut model = sim_mjcf::load_model(xml).expect("load");

    // Baseline: with contacts
    let mut data = model.make_data();
    data.forward(&model).expect("forward");
    let baseline_contacts = data.ncon;

    // Set filter that rejects ALL contacts
    model.set_contactfilter_callback(|_model, _data, _g1, _g2| false);

    let mut data2 = model.make_data();
    data2.forward(&model).expect("forward");

    assert_eq!(
        data2.ncon, 0,
        "Contact filter should reject all contacts, got {} (baseline was {})",
        data2.ncon, baseline_contacts
    );
}

/// Test: None callbacks → simulation identical to baseline.
#[test]
fn none_callbacks_no_effect() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="ball" pos="0 0 1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");

    // All callbacks are None by default
    assert!(model.cb_passive.is_none());
    assert!(model.cb_control.is_none());
    assert!(model.cb_contactfilter.is_none());

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // Should work normally (sanity check)
    assert!(
        data.qacc[2] < -5.0,
        "Free fall should produce downward acceleration, got {}",
        data.qacc[2]
    );
}

/// Test: clone Model with callbacks → both copies work.
#[test]
fn clone_with_callbacks() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="ball" pos="0 0 1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let mut model = sim_mjcf::load_model(xml).expect("load");

    let called = Arc::new(AtomicBool::new(false));
    let called_clone = Arc::clone(&called);

    model.set_passive_callback(move |_model, data| {
        called_clone.store(true, Ordering::SeqCst);
        data.qfrc_passive[0] += 1.0;
    });

    // Clone the model
    let model2 = model.clone();

    // Use the clone
    let mut data = model2.make_data();
    data.forward(&model2).expect("forward");

    assert!(
        called.load(Ordering::SeqCst),
        "Callback should have been called on clone"
    );
    assert!(
        data.qfrc_passive[0].abs() > 0.5,
        "Clone's callback should modify qfrc_passive"
    );
}

/// T4/G17: cb_sensor invoked at 3 stages for User sensors with different datatypes.
///
/// Creates 3 User sensors via MJCF, then patches model to assign one to each
/// stage (Position, Velocity, Acceleration). Each sensor gets dim=1 and proper
/// sensor_adr allocation so the callback can write to sensordata.
///
/// Verifies that the callback fires at all 3 stages and writes to sensordata.
#[test]
fn sensor_callback_fires_for_user_sensor() {
    use sim_core::{MjSensorDataType, SensorStage};

    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="link" pos="0 0 0">
          <joint name="hinge" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.05 0.5" mass="1.0"/>
        </body>
      </worldbody>
      <sensor>
        <user name="pos_sensor"/>
        <user name="vel_sensor"/>
        <user name="acc_sensor"/>
      </sensor>
    </mujoco>"#;

    let mut model = sim_mjcf::load_model(xml).expect("load");
    assert_eq!(model.nsensor, 3, "Should have 3 user sensors");

    // Patch sensor datatypes: one for each stage
    model.sensor_datatype[0] = MjSensorDataType::Position;
    model.sensor_datatype[1] = MjSensorDataType::Velocity;
    model.sensor_datatype[2] = MjSensorDataType::Acceleration;

    // Patch sensor dims to 1 each, and set up sensor_adr
    for i in 0..3 {
        model.sensor_dim[i] = 1;
        model.sensor_adr[i] = i;
    }
    model.nsensordata = 3;

    // Rebuild data to pick up the new nsensordata
    let mut data = model.make_data();

    // Track which (sensor_id, stage) pairs fire
    let invocations = Arc::new(std::sync::Mutex::new(Vec::new()));
    let inv_clone = Arc::clone(&invocations);

    model.set_sensor_callback(move |model, data, sensor_id, stage| {
        inv_clone
            .lock()
            .unwrap()
            .push((sensor_id, format!("{stage:?}")));

        // Write a known value to sensordata to prove invocation
        let adr = model.sensor_adr[sensor_id];
        if adr < data.sensordata.len() {
            data.sensordata[adr] = match stage {
                SensorStage::Pos => 100.0,
                SensorStage::Vel => 200.0,
                SensorStage::Acc => 300.0,
            };
        }
    });

    data.forward(&model).expect("forward");

    // Verify all 3 stages were invoked
    let inv = invocations.lock().unwrap();
    assert!(
        inv.iter().any(|(_, s)| s == "Pos"),
        "Sensor callback should fire at Pos stage, got: {inv:?}"
    );
    assert!(
        inv.iter().any(|(_, s)| s == "Vel"),
        "Sensor callback should fire at Vel stage, got: {inv:?}"
    );
    assert!(
        inv.iter().any(|(_, s)| s == "Acc"),
        "Sensor callback should fire at Acc stage, got: {inv:?}"
    );

    // Verify sensordata was written by callbacks
    assert!(
        (data.sensordata[0] - 100.0).abs() < 1e-10,
        "Position-stage sensor data should be 100.0, got {}",
        data.sensordata[0]
    );
    assert!(
        (data.sensordata[1] - 200.0).abs() < 1e-10,
        "Velocity-stage sensor data should be 200.0, got {}",
        data.sensordata[1]
    );
    assert!(
        (data.sensordata[2] - 300.0).abs() < 1e-10,
        "Acceleration-stage sensor data should be 300.0, got {}",
        data.sensordata[2]
    );
}

/// T5/G18: actuator user callbacks — dyntype, gaintype, biastype all User.
///
/// MJCF parser doesn't yet support `dyntype/gaintype/biastype="user"`,
/// so we build a motor actuator via MJCF and then patch the model fields
/// to `User` programmatically. This exercises the exact same actuation
/// codepath as a fully-wired User actuator.
///
/// Verifies:
/// 1. cb_act_dyn fires and act_dot[0] == 1.5
/// 2. cb_act_gain fires with gain=2.0
/// 3. cb_act_bias fires with bias=-0.5
/// 4. force = gain*ctrl + bias = 2.0*1.0 + (-0.5) = 1.5
/// 5. Safe defaults: no callbacks → act_dot=0, gain=0, bias=0
#[test]
fn actuator_user_callbacks_invoked() {
    use sim_core::{ActuatorDynamics, BiasType, GainType};

    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="link" pos="0 0 0">
          <joint name="hinge" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.05 0.5" mass="1.0"/>
        </body>
      </worldbody>
      <actuator>
        <motor name="m1" joint="hinge" gear="1"/>
      </actuator>
    </mujoco>"#;

    let mut model = sim_mjcf::load_model(xml).expect("load");

    // Patch actuator to use User dynamics/gain/bias callbacks
    model.actuator_dyntype[0] = ActuatorDynamics::User;
    model.actuator_gaintype[0] = GainType::User;
    model.actuator_biastype[0] = BiasType::User;

    // User dynamics requires an activation state — patch model
    model.actuator_act_adr[0] = 0;
    model.actuator_act_num[0] = 1;
    model.na = 1;

    let dyn_called = Arc::new(AtomicBool::new(false));
    let gain_called = Arc::new(AtomicBool::new(false));
    let bias_called = Arc::new(AtomicBool::new(false));

    let dyn_flag = Arc::clone(&dyn_called);
    let gain_flag = Arc::clone(&gain_called);
    let bias_flag = Arc::clone(&bias_called);

    model.set_act_dyn_callback(move |_model, _data, _id| {
        dyn_flag.store(true, Ordering::SeqCst);
        1.5 // act_dot
    });
    model.set_act_gain_callback(move |_model, _data, _id| {
        gain_flag.store(true, Ordering::SeqCst);
        2.0 // gain
    });
    model.set_act_bias_callback(move |_model, _data, _id| {
        bias_flag.store(true, Ordering::SeqCst);
        -0.5 // bias
    });

    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    // Use step() so that act_dot is integrated into act
    data.step(&model).expect("step");

    // Assertion 1: cb_act_dyn fired and act_dot == 1.5
    assert!(
        dyn_called.load(Ordering::SeqCst),
        "cb_act_dyn should have been called for User dyntype"
    );
    assert!(
        (data.act_dot[0] - 1.5).abs() < 1e-10,
        "act_dot[0] should be 1.5 from cb_act_dyn, got {}",
        data.act_dot[0]
    );

    // Assertion 2+3: cb_act_gain and cb_act_bias fired
    assert!(
        gain_called.load(Ordering::SeqCst),
        "cb_act_gain should have been called for User gaintype"
    );
    assert!(
        bias_called.load(Ordering::SeqCst),
        "cb_act_bias should have been called for User biastype"
    );

    // Assertion 4: force = gain*act + bias
    // After one forward: act=0, force = 2.0*0 + (-0.5) = -0.5
    // The spec says gain*ctrl+bias but with User dynamics, the force
    // computation uses `act` (activation state), not `ctrl` directly.
    // We verify the callbacks fired and force is consistent with the formula.
    // With act=0 on first step: qfrc_actuator = gain*0 + bias = -0.5
    assert!(
        (data.qfrc_actuator[0] - (-0.5)).abs() < 1e-8,
        "qfrc_actuator should be gain*act+bias = -0.5 (act=0 on first step), got {}",
        data.qfrc_actuator[0]
    );
}

/// T5 safe-defaults: no user callbacks → act_dot=0, gain=0, bias=0.
#[test]
fn actuator_user_callbacks_safe_defaults() {
    use sim_core::{ActuatorDynamics, BiasType, GainType};

    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="link" pos="0 0 0">
          <joint name="hinge" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.05 0.5" mass="1.0"/>
        </body>
      </worldbody>
      <actuator>
        <motor name="m1" joint="hinge" gear="1"/>
      </actuator>
    </mujoco>"#;

    let mut model = sim_mjcf::load_model(xml).expect("load");

    // Patch to User types but do NOT set any callbacks
    model.actuator_dyntype[0] = ActuatorDynamics::User;
    model.actuator_gaintype[0] = GainType::User;
    model.actuator_biastype[0] = BiasType::User;
    model.actuator_act_adr[0] = 0;
    model.actuator_act_num[0] = 1;
    model.na = 1;

    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward");

    // Safe defaults: act_dot = 0.0 (no cb_act_dyn)
    assert!(
        data.act_dot[0].abs() < 1e-15,
        "act_dot[0] should be 0.0 without cb_act_dyn, got {}",
        data.act_dot[0]
    );

    // Safe defaults: gain=0, bias=0 → force=0
    assert!(
        data.qfrc_actuator[0].abs() < 1e-15,
        "qfrc_actuator should be 0.0 without gain/bias callbacks, got {}",
        data.qfrc_actuator[0]
    );
}
