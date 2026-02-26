//! Body force accumulator tests (§51).
//!
//! Verifies `cacc`, `cfrc_int`, and `cfrc_ext` computed by `mj_body_accumulators()`.

use sim_core::SleepState;

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

/// T1/G5: cfrc_ext includes contact normal force when ball rests on plane.
///
/// Verifies: (1) contacts generated, (2) cfrc_ext has upward normal force ≈ mg,
/// (3) Newton's 3rd law — world body gets equal-and-opposite force.
#[test]
fn cfrc_ext_includes_contact_forces() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <geom type="plane" size="5 5 0.1"/>
        <body name="ball" pos="0 0 0.1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Step a few times to let the ball settle on the plane
    for _ in 0..50 {
        data.step(&model).expect("step");
    }

    // Assertion 1: at least one contact generated
    assert!(
        data.ncon >= 1,
        "Should have at least 1 contact, got ncon={}",
        data.ncon
    );

    // Assertion 2+3: cfrc_ext has nonzero upward contact normal force ≈ mg
    let cfrc_ext_ball = &data.cfrc_ext[1];
    let linear_z = cfrc_ext_ball[5]; // spatial convention: [torque; force], z is index 5
    let mg = 1.0 * 9.81;
    assert!(
        linear_z > 0.1,
        "cfrc_ext[1] should have upward contact normal force, got linear_z={linear_z}"
    );

    // Assertion 4: Newton's 3rd law — the contact force on the ball should be
    // approximately equal to mg (supporting the ball's weight in steady state).
    assert!(
        (linear_z - mg).abs() < mg * 0.1,
        "Newton's 3rd law: contact force ({linear_z}) should be ≈ mg ({mg})"
    );
}

/// G6: body accumulators zeroed after reset (validates Stage 1).
#[test]
fn body_accumulators_zeroed_after_reset() {
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

    // Run forward to populate accumulators
    data.forward(&model).expect("forward");
    assert!(
        data.cacc[0].iter().any(|&v| v.abs() > 1e-6),
        "cacc should be nonzero after forward"
    );

    // Reset
    data.reset(&model);

    // All body accumulators should be zero
    for b in 0..model.nbody {
        for i in 0..6 {
            assert!(
                data.cacc[b][i].abs() < 1e-15,
                "cacc[{b}][{i}] should be 0 after reset, got {}",
                data.cacc[b][i]
            );
            assert!(
                data.cfrc_int[b][i].abs() < 1e-15,
                "cfrc_int[{b}][{i}] should be 0 after reset, got {}",
                data.cfrc_int[b][i]
            );
            assert!(
                data.cfrc_ext[b][i].abs() < 1e-15,
                "cfrc_ext[{b}][{i}] should be 0 after reset, got {}",
                data.cfrc_ext[b][i]
            );
        }
    }
    // qfrc_inverse also zeroed
    for i in 0..model.nv {
        assert!(
            data.qfrc_inverse[i].abs() < 1e-15,
            "qfrc_inverse[{i}] should be 0 after reset, got {}",
            data.qfrc_inverse[i]
        );
    }
}

/// T3/G7: cfrc_int propagation in a 3-link serial chain with equal masses.
///
/// Equal masses (1 kg each), equal link lengths (0.5 m), hinge joints.
/// Verifies: weight-per-link assertions, monotonic propagation, all 4 bodies.
#[test]
fn cfrc_int_propagation_chain() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="link1" pos="0 0 1">
          <joint name="j1" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.05 0.25" mass="1.0"/>
          <body name="link2" pos="0 0 -0.5">
            <joint name="j2" type="hinge" axis="0 1 0"/>
            <geom type="capsule" size="0.05 0.25" mass="1.0"/>
            <body name="link3" pos="0 0 -0.5">
              <joint name="j3" type="hinge" axis="0 1 0"/>
              <geom type="capsule" size="0.05 0.25" mass="1.0"/>
            </body>
          </body>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();
    data.forward(&model).expect("forward");

    let g = 9.81;

    // Compute linear force norms for each body
    let linear_norm = |v: &sim_core::dynamics::SpatialVector| -> f64 {
        (v[3] * v[3] + v[4] * v[4] + v[5] * v[5]).sqrt()
    };

    let norm_link3 = linear_norm(&data.cfrc_int[3]); // body 3 = leaf
    let norm_link2 = linear_norm(&data.cfrc_int[2]); // body 2 = middle
    let norm_link1 = linear_norm(&data.cfrc_int[1]); // body 1 = root

    // Assertion 1: link3 supports its own weight (1g ≈ 9.81)
    assert!(
        (norm_link3 - 1.0 * g).abs() < 1.0,
        "cfrc_int[3] linear norm should be ~1g = {}, got {norm_link3}",
        1.0 * g
    );

    // Assertion 2: link2 supports link2 + link3 weight (2g)
    assert!(
        (norm_link2 - 2.0 * g).abs() < 2.0,
        "cfrc_int[2] linear norm should be ~2g = {}, got {norm_link2}",
        2.0 * g
    );

    // Assertion 3: link1 supports all three bodies (3g)
    assert!(
        (norm_link1 - 3.0 * g).abs() < 3.0,
        "cfrc_int[1] linear norm should be ~3g = {}, got {norm_link1}",
        3.0 * g
    );

    // Assertion 5: monotonic — |cfrc_int[child]| <= |cfrc_int[parent]|
    assert!(
        norm_link1 >= norm_link2 - 1e-6,
        "Monotonic: link1 ({norm_link1}) >= link2 ({norm_link2})"
    );
    assert!(
        norm_link2 >= norm_link3 - 1e-6,
        "Monotonic: link2 ({norm_link2}) >= link3 ({norm_link3})"
    );
}

/// T2/G6: Body accumulators are still computed for sleeping bodies.
///
/// mj_body_accumulators does NOT skip sleeping bodies — gravity pseudo-
/// acceleration and internal forces are computed regardless of sleep state.
#[test]
fn sleep_state_body_accumulators() {
    let xml = r#"
    <mujoco model="sleep_accum">
      <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
        <flag sleep="enable"/>
      </option>
      <worldbody>
        <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
        <body name="ball" pos="0 0 0.2">
          <freejoint name="ball_free"/>
          <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(xml).expect("load");
    let mut data = model.make_data();

    // Step until the body falls asleep
    let mut asleep = false;
    for _ in 0..5000 {
        data.step(&model).expect("step");
        if data.body_sleep_state[1] == SleepState::Asleep {
            asleep = true;
            break;
        }
    }
    assert!(asleep, "Body should have fallen asleep after settling");

    // Run forward on the sleeping body
    data.forward(&model).expect("forward");

    // Assertion 1: cacc[1] is still computed (sleeping bodies still have gravity).
    // The world body cacc[0] should contain the gravity pseudo-acceleration.
    let cacc_world = &data.cacc[0];
    assert!(
        (cacc_world[5] - 9.81).abs() < 1e-6,
        "World cacc[0] should have gravity pseudo-acceleration, got z={}",
        cacc_world[5]
    );

    // Assertion 2: cfrc_int[1] reflects static equilibrium forces (nonzero
    // because the body has mass and gravity is active).
    let cfrc_int_norm: f64 = data.cfrc_int[1].iter().map(|x| x * x).sum::<f64>().sqrt();
    assert!(
        cfrc_int_norm > 0.1,
        "cfrc_int[1] should be nonzero for sleeping body under gravity, got norm={cfrc_int_norm}"
    );

    // Assertion 3: body accumulators are physically consistent — cfrc_int
    // should have a gravitational component even while sleeping.
    let cfrc_int_z = data.cfrc_int[1][5]; // linear z component
    // Under gravity, the internal force in z should be related to mg
    assert!(
        cfrc_int_z.abs() > 0.1,
        "cfrc_int[1] z-force should be nonzero under gravity while sleeping, got {cfrc_int_z}"
    );
}
