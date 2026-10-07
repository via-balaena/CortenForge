//! `xfrc_applied` projection tests (DT-21).
//!
//! Verifies that Cartesian body forces (`xfrc_applied`) are correctly projected
//! into joint-space passive forces via J^T in `mj_fwd_passive()`.

/// Test: free body with upward force gets upward acceleration.
#[test]
fn free_body_upward_force() {
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

    // Apply upward force slightly larger than gravity
    let body_id = 1;
    data.xfrc_applied[body_id].force[2] = 20.0; // force_z = 20 N (> mg = 9.81)

    data.forward(&model).expect("forward");

    // The z-acceleration of the free joint (linear DOFs are 0,1,2)
    // qacc[2] should be positive (net upward force)
    let az = data.qacc[2];
    assert!(
        az > 0.0,
        "Expected positive z-acceleration with upward force, got {az}"
    );
}

/// Test: hinge pendulum with Cartesian force produces correct joint torque.
///
/// The force is applied at `xipos` (body CoM in world frame). For the torque
/// to be non-zero, `xipos` must differ from the hinge anchor. We verify this
/// by checking `qacc` (which reflects the full solve including xfrc_applied).
#[test]
fn hinge_pendulum_torque() {
    // Body at pos="0 0 1" with hinge at body origin. The geom extends downward
    // from the body frame, so the CoM is at the body origin. We apply a torque
    // directly about the hinge axis (y) — this always works regardless of CoM.
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="link" pos="0 0 1">
          <joint name="hinge" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.05 0.5" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Compute baseline (zero gravity, no force → qacc should be 0)
    data.forward(&model).expect("forward");
    let baseline_qacc = data.qacc[0];
    assert!(
        baseline_qacc.abs() < 1e-10,
        "Baseline qacc should be ~0, got {baseline_qacc}"
    );

    // Apply torque about y-axis (hinge axis) via xfrc_applied.
    // Torque about y goes through J^T as: axis · torque = [0,1,0] · [0,5,0] = 5
    data.xfrc_applied[1].torque[1] = 5.0; // torque_y = 5 Nm
    data.forward(&model).expect("forward");

    // qacc should now be non-zero (torque / inertia)
    let with_torque_qacc = data.qacc[0];
    assert!(
        with_torque_qacc.abs() > 1e-6,
        "xfrc_applied torque should produce angular acceleration, got qacc={with_torque_qacc}"
    );
}

/// Test: anti-gravity via xfrc_applied produces near-zero acceleration.
#[test]
fn anti_gravity_balance() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="ball" pos="0 0 1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="2.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Apply exactly m*g upward to cancel gravity
    let mass = model.body_mass[1];
    let g = model.gravity[2].abs();
    data.xfrc_applied[1].force[2] = mass * g; // force_z = m*g upward

    data.forward(&model).expect("forward");

    // Linear accelerations should be near zero
    let ax = data.qacc[0].abs();
    let ay = data.qacc[1].abs();
    let az = data.qacc[2].abs();
    assert!(
        ax < 1e-10 && ay < 1e-10 && az < 1e-10,
        "Anti-gravity should yield ~zero acceleration: ax={ax}, ay={ay}, az={az}"
    );
}

/// Test: pure torque (angular only) produces angular acceleration.
#[test]
fn pure_torque() {
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

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Apply pure torque about z-axis
    data.xfrc_applied[1].torque[2] = 5.0; // torque_z = 5 Nm

    data.forward(&model).expect("forward");

    // Free joint DOFs: [vx, vy, vz, wx, wy, wz] → angular DOFs are 3,4,5
    // Linear accelerations should be ~zero (pure torque, no force)
    let ax = data.qacc[0].abs();
    let ay = data.qacc[1].abs();
    let az = data.qacc[2].abs();
    assert!(
        ax < 1e-10 && ay < 1e-10 && az < 1e-10,
        "Pure torque should give zero linear acceleration: ax={ax}, ay={ay}, az={az}"
    );

    // Angular acceleration about z should be positive
    let alpha_z = data.qacc[5];
    assert!(
        alpha_z > 0.0,
        "Expected positive angular acceleration about z, got {alpha_z}"
    );
}

/// Test: a row copied from MuJoCo applies its force first and its torque
/// second. Free body, mass 2, inertia 0.1·I, no gravity: MuJoCo 3.5.0 gives
/// `qacc` = (0, 0, 10, 0, 50, 0) for the row `[0, 0, 20, 0, 5, 0]` (F = ma,
/// τ = Iα).
#[test]
fn xfrc_applied_is_force_then_torque() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="b">
          <freejoint/>
          <inertial pos="0 0 0" mass="2" diaginertia="0.1 0.1 0.1"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();
    data.xfrc_applied[1] = sim_core::BodyWrench::from_mujoco_row([0.0, 0.0, 20.0, 0.0, 5.0, 0.0]);

    data.forward(&model).expect("forward");

    assert_eq!(data.qacc.as_slice(), &[0.0, 0.0, 10.0, 0.0, 50.0, 0.0]);
}
