//! Inverse dynamics tests (§52).
//!
//! Verifies `data.inverse()` computes correct generalized forces.

/// Test: round-trip — `forward()` → `inverse()` → verify
/// `qfrc_inverse ≈ qfrc_applied + qfrc_actuator`.
#[test]
fn round_trip() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="link1" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.05 0.3" mass="1.0"/>
          <body name="link2" pos="0 0 -0.6">
            <joint name="j2" type="hinge" axis="0 1 0"/>
            <geom type="capsule" size="0.04 0.2" mass="0.5"/>
          </body>
        </body>
      </worldbody>
      <actuator>
        <motor name="m1" joint="j1" gear="1"/>
        <motor name="m2" joint="j2" gear="1"/>
      </actuator>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Set some non-trivial state
    data.qpos[0] = 0.5;
    data.qpos[1] = -0.3;
    data.qvel[0] = 1.0;
    data.qvel[1] = -0.5;
    data.ctrl[0] = 5.0;
    data.ctrl[1] = -2.0;
    data.qfrc_applied[0] = 1.0;
    data.qfrc_applied[1] = -0.5;

    // Forward dynamics computes qacc from forces
    data.forward(&model).expect("forward");

    // Inverse dynamics computes forces from qacc
    data.inverse(&model);

    // qfrc_inverse should equal qfrc_applied + qfrc_actuator + qfrc_constraint
    for i in 0..model.nv {
        let expected = data.qfrc_applied[i] + data.qfrc_actuator[i] + data.qfrc_constraint[i];
        let inv = data.qfrc_inverse[i];
        assert!(
            (inv - expected).abs() < 1e-8,
            "qfrc_inverse[{i}] = {inv} should match applied+actuator+constraint = {expected}"
        );
    }
}

/// Test: inverse identity — `qfrc_inverse = M*qacc + qfrc_bias - qfrc_passive`.
///
/// With no applied forces, actuators, or constraints, qfrc_inverse should
/// be zero (the inverse is consistent with forward dynamics: free-falling
/// bodies need no external force).
#[test]
fn inverse_identity() {
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
    let mut data = model.make_data();

    // Zero applied forces → free fall
    data.forward(&model).expect("forward");

    // qacc should be non-zero (gravity drives linear acceleration)
    assert!(
        data.qacc[2].abs() > 1.0,
        "Free-fall qacc[2] should be ~-9.81, got {}",
        data.qacc[2]
    );

    data.inverse(&model);

    // qfrc_inverse should be ~zero for all DOFs
    for i in 0..model.nv {
        assert!(
            data.qfrc_inverse[i].abs() < 1e-8,
            "qfrc_inverse[{i}] should be ~0, got {}",
            data.qfrc_inverse[i]
        );
    }
}

/// Test: free body with set qacc — verify `qfrc_inverse = M * qacc`.
#[test]
fn free_body_inverse() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="ball" pos="0 0 1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="2.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Apply some force to get non-zero qacc
    data.qfrc_applied[0] = 10.0; // force_x
    data.qfrc_applied[2] = 5.0; // force_z
    data.forward(&model).expect("forward");

    // Now run inverse
    data.inverse(&model);

    // With zero gravity and zero velocity (no Coriolis):
    // M * qacc = qfrc_total = qfrc_applied (only force)
    // qfrc_inverse = M * qacc + qfrc_bias - qfrc_passive
    //              = qfrc_applied + 0 - 0 = qfrc_applied
    for i in 0..model.nv {
        assert!(
            (data.qfrc_inverse[i] - data.qfrc_applied[i]).abs() < 1e-8,
            "qfrc_inverse[{i}] = {} should match qfrc_applied[{i}] = {}",
            data.qfrc_inverse[i],
            data.qfrc_applied[i]
        );
    }
}
