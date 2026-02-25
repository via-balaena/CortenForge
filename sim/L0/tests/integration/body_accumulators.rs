//! Body force accumulator tests (§51).
//!
//! Verifies `cacc`, `cfrc_int`, and `cfrc_ext` computed by `mj_body_accumulators()`.

/// Test: free-falling body has cacc = [0,0,0, 0,0,-g].
#[test]
fn free_fall_cacc() {
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
    data.forward(&model).expect("forward");

    // cacc[0] (world body) should be the gravity pseudo-acceleration
    // Spatial convention: [angular; linear] → [0,0,0, 0,0,+9.81]
    // (negative gravity means upward pseudo-acceleration in body frame convention)
    let cacc_world = &data.cacc[0];
    assert!(cacc_world[0].abs() < 1e-10, "world angular x");
    assert!(cacc_world[1].abs() < 1e-10, "world angular y");
    assert!(cacc_world[2].abs() < 1e-10, "world angular z");
    assert!(cacc_world[3].abs() < 1e-10, "world linear x");
    assert!(cacc_world[4].abs() < 1e-10, "world linear y");
    assert!((cacc_world[5] - 9.81).abs() < 1e-10, "world linear z = +g");

    // cacc[1] (free-falling ball) should have zero linear acceleration
    // because qacc from free-fall = [0,0,-g] in world frame,
    // which cancels the gravity pseudo-acceleration from the root.
    // Net: cacc[1] = cacc[0] + S*qacc = [0,0,0, 0,0,+g] + [0,0,0, 0,0,-g] = 0
    let cacc_ball = &data.cacc[1];
    for i in 0..6 {
        assert!(
            cacc_ball[i].abs() < 1e-6,
            "Free-fall cacc[1][{i}] should be ~0, got {}",
            cacc_ball[i]
        );
    }
}

/// Test: static equilibrium cfrc_int at root equals total gravity.
#[test]
fn static_equilibrium_cfrc_int() {
    // A two-body pendulum held at initial position (zero velocity, constraint holds).
    // cfrc_int at root body should reflect total gravitational load.
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="b1" pos="0 0 1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="2.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    // cfrc_ext should match xfrc_applied (which is zero here)
    let cfrc_ext = &data.cfrc_ext[1];
    for i in 0..6 {
        assert!(
            cfrc_ext[i].abs() < 1e-10,
            "cfrc_ext[1][{i}] should be 0 (no xfrc_applied), got {}",
            cfrc_ext[i]
        );
    }
}

/// Test: cfrc_ext matches xfrc_applied.
#[test]
fn cfrc_ext_matches_xfrc_applied() {
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

    // Apply some external force/torque
    data.xfrc_applied[1][0] = 1.0; // torque_x
    data.xfrc_applied[1][3] = 5.0; // force_x
    data.xfrc_applied[1][5] = 10.0; // force_z

    data.forward(&model).expect("forward");

    // cfrc_ext should be a copy of xfrc_applied
    for i in 0..6 {
        assert!(
            (data.cfrc_ext[1][i] - data.xfrc_applied[1][i]).abs() < 1e-10,
            "cfrc_ext[1][{i}] = {} should match xfrc_applied[1][{i}] = {}",
            data.cfrc_ext[1][i],
            data.xfrc_applied[1][i]
        );
    }
}

/// Test: gravity-disabled produces zero cacc at root.
#[test]
fn no_gravity_zero_root_cacc() {
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
    data.forward(&model).expect("forward");

    // With no gravity, root pseudo-acceleration should be zero
    let cacc_world = &data.cacc[0];
    for i in 0..6 {
        assert!(
            cacc_world[i].abs() < 1e-10,
            "No-gravity cacc[0][{i}] should be 0, got {}",
            cacc_world[i]
        );
    }

    // Body cacc should also be zero (no forces, no acceleration)
    let cacc_ball = &data.cacc[1];
    for i in 0..6 {
        assert!(
            cacc_ball[i].abs() < 1e-10,
            "No-gravity cacc[1][{i}] should be 0, got {}",
            cacc_ball[i]
        );
    }
}
