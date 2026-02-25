//! Name lookup API tests (§59).
//!
//! Verifies O(1) bidirectional name↔index lookup on Model via
//! `name2id()` and `id2name()`.

use sim_core::ElementType;

fn load_named_model() -> sim_core::Model {
    let xml = r#"
    <mujoco model="name_test">
      <worldbody>
        <body name="torso" pos="0 0 1">
          <joint name="root" type="free"/>
          <geom name="torso_geom" type="sphere" size="0.1"/>
          <site name="torso_site" size="0.01"/>
          <body name="arm" pos="0.2 0 0">
            <joint name="shoulder" type="hinge" axis="0 1 0"/>
            <geom name="arm_geom" type="capsule" size="0.05 0.15"/>
          </body>
        </body>
      </worldbody>
      <tendon>
        <fixed name="ten0">
          <joint joint="shoulder" coef="1.0"/>
        </fixed>
      </tendon>
      <actuator>
        <motor name="motor0" joint="shoulder"/>
      </actuator>
      <sensor>
        <jointpos name="shoulder_pos" joint="shoulder"/>
        <jointvel name="shoulder_vel" joint="shoulder"/>
      </sensor>
      <equality>
        <joint name="eq_shoulder" joint1="shoulder" polycoef="0 1 0 0 0"/>
      </equality>
    </mujoco>"#;
    sim_mjcf::load_model(xml).expect("Failed to load named model")
}

#[test]
fn name2id_round_trip() {
    let model = load_named_model();

    // Body lookup
    assert_eq!(model.name2id(ElementType::Body, "torso"), Some(1));
    assert_eq!(model.name2id(ElementType::Body, "arm"), Some(2));
    assert_eq!(model.name2id(ElementType::Body, "world"), Some(0));

    // Joint lookup
    let root_id = model.name2id(ElementType::Joint, "root").unwrap();
    let shoulder_id = model.name2id(ElementType::Joint, "shoulder").unwrap();
    assert_ne!(root_id, shoulder_id);

    // Geom lookup
    assert!(model.name2id(ElementType::Geom, "torso_geom").is_some());
    assert!(model.name2id(ElementType::Geom, "arm_geom").is_some());

    // Site lookup
    assert!(model.name2id(ElementType::Site, "torso_site").is_some());

    // Tendon lookup
    assert!(model.name2id(ElementType::Tendon, "ten0").is_some());

    // Actuator lookup
    assert!(model.name2id(ElementType::Actuator, "motor0").is_some());

    // Sensor lookup
    assert!(model.name2id(ElementType::Sensor, "shoulder_pos").is_some());
    assert!(model.name2id(ElementType::Sensor, "shoulder_vel").is_some());

    // Equality lookup
    assert!(
        model
            .name2id(ElementType::Equality, "eq_shoulder")
            .is_some()
    );
}

#[test]
fn id2name_round_trip() {
    let model = load_named_model();

    // Round-trip: name2id → id2name → same name
    for name in &["torso", "arm"] {
        let id = model.name2id(ElementType::Body, name).unwrap();
        assert_eq!(model.id2name(ElementType::Body, id), Some(*name));
    }

    let jnt_id = model.name2id(ElementType::Joint, "shoulder").unwrap();
    assert_eq!(model.id2name(ElementType::Joint, jnt_id), Some("shoulder"));

    let act_id = model.name2id(ElementType::Actuator, "motor0").unwrap();
    assert_eq!(model.id2name(ElementType::Actuator, act_id), Some("motor0"));
}

#[test]
fn nonexistent_name_returns_none() {
    let model = load_named_model();

    assert_eq!(model.name2id(ElementType::Body, "nonexistent"), None);
    assert_eq!(model.name2id(ElementType::Joint, "fake_joint"), None);
    assert_eq!(model.name2id(ElementType::Sensor, "no_such_sensor"), None);
}

#[test]
fn out_of_bounds_id_returns_none() {
    let model = load_named_model();

    assert_eq!(model.id2name(ElementType::Body, 999), None);
    assert_eq!(model.id2name(ElementType::Joint, 999), None);
    assert_eq!(model.id2name(ElementType::Sensor, 999), None);
}
