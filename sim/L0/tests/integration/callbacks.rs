//! User callback tests (DT-79).
//!
//! Verifies `cb_passive`, `cb_control`, `cb_contactfilter`, `cb_sensor`,
//! `cb_act_dyn`, `cb_act_gain`, and `cb_act_bias` hooks.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

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

/// G17: sensor callback fires at acceleration stage for User sensor.
///
/// User sensors default to Acceleration datatype, so cb_sensor fires
/// during mj_sensor_acc. The callback receives the sensor ID and stage.
///
/// Note: User sensor dim defaults to 0 in our builder (variable-dimension
/// sensors need programmatic dim override), so we only verify the callback
/// fires — not that sensordata is written. Writing to sensordata with dim=0
/// is harmless but out-of-bounds.
#[test]
fn sensor_callback_fires_for_user_sensor() {
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
        <user name="my_user_sensor"/>
      </sensor>
    </mujoco>"#;

    let mut model = sim_mjcf::load_model(xml).expect("load");
    assert_eq!(model.nsensor, 1, "Should have 1 user sensor");

    let call_count = Arc::new(AtomicUsize::new(0));
    let count_clone = Arc::clone(&call_count);

    // Track which stages the callback fires at
    let stages_seen = Arc::new(std::sync::Mutex::new(Vec::new()));
    let stages_clone = Arc::clone(&stages_seen);

    model.set_sensor_callback(move |_model, _data, _sensor_id, stage| {
        count_clone.fetch_add(1, Ordering::SeqCst);
        stages_clone.lock().unwrap().push(format!("{stage:?}"));
    });

    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let count = call_count.load(Ordering::SeqCst);
    assert!(
        count >= 1,
        "Sensor callback should fire at least once, fired {count} times"
    );

    // User sensors default to Acceleration datatype, so we expect Acc stage
    let stages = stages_seen.lock().unwrap();
    assert!(
        stages.iter().any(|s| s == "Acc"),
        "Sensor callback should fire at Acc stage, got stages: {stages:?}"
    );
}

/// G18: actuator user callbacks invoked for User gaintype/biastype.
///
/// MJCF parser doesn't yet support `gaintype="user"` / `biastype="user"`,
/// so we build a motor actuator via MJCF and then patch the model's
/// gaintype/biastype fields to `User` programmatically. This exercises the
/// exact same actuation codepath as a fully-wired User actuator.
#[test]
fn actuator_user_callbacks_invoked() {
    use sim_core::{BiasType, GainType};

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

    // Patch actuator to use User gain/bias callbacks
    model.actuator_gaintype[0] = GainType::User;
    model.actuator_biastype[0] = BiasType::User;

    let gain_called = Arc::new(AtomicBool::new(false));
    let bias_called = Arc::new(AtomicBool::new(false));

    let gain_flag = Arc::clone(&gain_called);
    let bias_flag = Arc::clone(&bias_called);

    model.set_act_gain_callback(move |_model, _data, _id| {
        gain_flag.store(true, Ordering::SeqCst);
        2.0 // gain
    });
    model.set_act_bias_callback(move |_model, _data, _id| {
        bias_flag.store(true, Ordering::SeqCst);
        -1.0 // bias
    });

    let mut data = model.make_data();
    data.ctrl[0] = 1.0;
    data.forward(&model).expect("forward");

    assert!(
        gain_called.load(Ordering::SeqCst),
        "cb_act_gain should have been called for User gaintype"
    );
    assert!(
        bias_called.load(Ordering::SeqCst),
        "cb_act_bias should have been called for User biastype"
    );

    // Verify the actuator force was computed using the callback values:
    // force = gain*ctrl + bias = 2.0*1.0 + (-1.0) = 1.0
    assert!(
        (data.qfrc_actuator[0] - 1.0).abs() < 1e-8,
        "qfrc_actuator should be gain*ctrl+bias = 1.0, got {}",
        data.qfrc_actuator[0]
    );
}
