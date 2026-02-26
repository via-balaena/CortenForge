//! Inverse dynamics tests (§52).
//!
//! Verifies `data.inverse()` computes correct generalized forces.

/// Test: round-trip — `forward()` → `inverse()` → verify
/// `qfrc_inverse ≈ qfrc_applied + qfrc_actuator`.
///
/// With G22 (inverse subtracts `qfrc_constraint`), the round-trip identity is:
/// `qfrc_inverse = qfrc_applied + qfrc_actuator + J^T*xfrc_applied`.
/// This test has no `xfrc_applied`, so the identity simplifies to
/// `qfrc_inverse = qfrc_applied + qfrc_actuator`.
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

    // qfrc_inverse should equal qfrc_applied + qfrc_actuator (no xfrc_applied)
    for i in 0..model.nv {
        let expected = data.qfrc_applied[i] + data.qfrc_actuator[i];
        let inv = data.qfrc_inverse[i];
        assert!(
            (inv - expected).abs() < 1e-8,
            "qfrc_inverse[{i}] = {inv} should match applied+actuator = {expected}"
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

/// T9/G22: Verify the explicit inverse dynamics formula:
///   `qfrc_inverse = M * qacc + qfrc_bias - qfrc_passive - qfrc_constraint`
///
/// This test manually assembles each term and compares against qfrc_inverse,
/// proving the formula holds even with nonzero constraints (hinge chain under
/// gravity generates constraint forces from joint limits or Coriolis).
#[test]
fn inverse_formula_explicit() {
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

    // Nontrivial state: off-vertical pose with velocity and control
    data.qpos[0] = 0.8;
    data.qpos[1] = -0.4;
    data.qvel[0] = 2.0;
    data.qvel[1] = -1.0;
    data.ctrl[0] = 3.0;
    data.ctrl[1] = -1.5;
    data.qfrc_applied[0] = 2.0;

    data.forward(&model).expect("forward");
    data.inverse(&model);

    // Assertion 1: qfrc_inverse is nonzero (nontrivial config)
    let inv_norm: f64 = (0..model.nv)
        .map(|i| data.qfrc_inverse[i].powi(2))
        .sum::<f64>()
        .sqrt();
    assert!(
        inv_norm > 0.1,
        "qfrc_inverse should be nonzero in nontrivial config, got norm={inv_norm}"
    );

    // Assertion 2: Verify formula — M * qacc + qfrc_bias - qfrc_passive - qfrc_constraint
    // Manually compute each term and compare.
    let mut m_qacc = nalgebra::DVector::zeros(model.nv);
    data.qM.mul_to(&data.qacc, &mut m_qacc);

    for i in 0..model.nv {
        let expected =
            m_qacc[i] + data.qfrc_bias[i] - data.qfrc_passive[i] - data.qfrc_constraint[i];
        assert!(
            (data.qfrc_inverse[i] - expected).abs() < 1e-10,
            "Formula mismatch at DOF {i}: qfrc_inverse={} but M*qacc+bias-passive-constraint={}",
            data.qfrc_inverse[i],
            expected
        );
    }
}

/// T9/G23: ENABLE_FWDINV flag triggers compare_fwd_inv() during forward().
///
/// When enabled, solver_fwdinv[1] is populated with the round-trip residual.
/// When disabled, solver_fwdinv stays at [0, 0].
#[test]
fn enable_fwdinv_flag() {
    use sim_core::ENABLE_FWDINV;

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

    // --- Without ENABLE_FWDINV: solver_fwdinv should stay zero ---
    let model_off = sim_mjcf::load_model(xml).expect("load");
    assert_eq!(
        model_off.enableflags & ENABLE_FWDINV,
        0,
        "FWDINV should be off by default"
    );

    let mut data_off = model_off.make_data();
    data_off.qpos[0] = 0.5;
    data_off.ctrl[0] = 3.0;
    data_off.qfrc_applied[0] = 1.0;
    data_off.forward(&model_off).expect("forward");

    assert_eq!(
        data_off.solver_fwdinv,
        [0.0, 0.0],
        "solver_fwdinv should be [0,0] without ENABLE_FWDINV"
    );

    // --- With ENABLE_FWDINV: solver_fwdinv[1] should be populated ---
    let mut model_on = sim_mjcf::load_model(xml).expect("load");
    model_on.enableflags |= ENABLE_FWDINV;

    let mut data_on = model_on.make_data();
    data_on.qpos[0] = 0.5;
    data_on.ctrl[0] = 3.0;
    data_on.qfrc_applied[0] = 1.0;
    data_on.forward(&model_on).expect("forward");

    // solver_fwdinv[1] is the L2 norm of (qfrc_inverse - fwd_applied).
    // For a well-formed round-trip this should be very small (near zero).
    assert!(
        data_on.solver_fwdinv[1] < 1e-6,
        "solver_fwdinv[1] should be near-zero for consistent fwd/inv, got {}",
        data_on.solver_fwdinv[1]
    );

    // qfrc_inverse should have been populated (nonzero with gravity + applied force)
    let inv_norm: f64 = (0..model_on.nv)
        .map(|i| data_on.qfrc_inverse[i].powi(2))
        .sum::<f64>()
        .sqrt();
    assert!(
        inv_norm > 0.1,
        "qfrc_inverse should be nonzero when ENABLE_FWDINV is set, got norm={inv_norm}"
    );
}
