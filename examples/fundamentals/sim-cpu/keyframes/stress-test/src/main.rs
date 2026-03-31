//! Stress test — headless validation of all keyframe concepts.
//!
//! 12 checks covering: state restoration (qpos/qvel/ctrl/act/time), derived
//! state clearing (qacc/contacts), multi-keyframe indexing, name lookup,
//! zero-length fields, deterministic replay, and post-reset stability.
//!
//! Run: `cargo run -p example-keyframes-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::suboptimal_flops,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::float_cmp // Intentional: keyframe reset must be bitwise-exact
)]

use sim_core::ElementType;

// ── MJCF Models ────────────────────────────────────────────────────────────

/// Model A — Single hinge, 3 named keyframes.
/// Used by checks 1-2, 5, 6, 8-9, 11.
const HINGE_MJCF: &str = r#"
<mujoco model="keyframe-hinge">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag energy="enable"/>
  </option>
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.05" fromto="0 0 0 0.5 0 0" mass="1.0"/>
    </body>
  </worldbody>
  <keyframe>
    <key name="rest"       qpos="0.0"    qvel="0.0" time="0.0"/>
    <key name="horizontal" qpos="1.5708" qvel="0.5" time="2.5"/>
    <key name="inverted"   qpos="3.1416" qvel="-1.0" time="5.0"/>
  </keyframe>
</mujoco>
"#;

/// Model B — Hinge + position actuator with dyntype=integrator.
/// Used by checks 3-4.
const ACTUATED_MJCF: &str = r#"
<mujoco model="keyframe-actuated">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.05" fromto="0 0 0 0.5 0 0" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <general name="motor" joint="hinge" gainprm="10" biasprm="0 -10 0"
             dyntype="integrator" dynprm="1"/>
  </actuator>
  <keyframe>
    <key name="home" qpos="0.5" qvel="0.0" act="0.3" ctrl="1.0"/>
  </keyframe>
</mujoco>
"#;

/// Model C — Free body + floor, contact-producing.
/// Used by checks 7, 12.
const CONTACT_MJCF: &str = r#"
<mujoco model="keyframe-contact">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002">
    <flag energy="enable"/>
  </option>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="ball" pos="0 0 0.5">
      <freejoint name="free"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
  <keyframe>
    <key name="above" qpos="0 0 0.5 1 0 0 0"/>
  </keyframe>
</mujoco>
"#;

/// Model D — No actuators, hinge only.
/// Used by check 10 (zero-length act/ctrl fields).
const NO_ACTUATOR_MJCF: &str = r#"
<mujoco model="keyframe-no-actuator">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
  <keyframe>
    <key name="tilted" qpos="0.5" qvel="1.0"/>
  </keyframe>
</mujoco>
"#;

// ── Helpers ────────────────────────────────────────────────────────────────

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

// ── Check 1: qpos exact after reset ───────────────────────────────────────

fn check_1_qpos_exact() -> (u32, u32) {
    let model = sim_mjcf::load_model(HINGE_MJCF).expect("parse");
    let mut data = model.make_data();

    // Step to mutate qpos away from any keyframe value.
    for _ in 0..100 {
        data.step(&model).expect("step");
    }
    assert!(
        (data.qpos[0] - model.keyframes[1].qpos[0]).abs() > 0.01,
        "qpos should have drifted from keyframe value"
    );

    // Reset to "horizontal" keyframe (index 1).
    data.reset_to_keyframe(&model, 1).expect("reset");

    let drift = (data.qpos[0] - model.keyframes[1].qpos[0]).abs();
    let p = check(
        "qpos exact after reset",
        drift == 0.0,
        &format!(
            "qpos = {:.15}, kf = {:.15}, drift = {drift:.2e}",
            data.qpos[0], model.keyframes[1].qpos[0]
        ),
    );
    (u32::from(p), 1)
}

// ── Check 2: qvel exact after reset ───────────────────────────────────────

fn check_2_qvel_exact() -> (u32, u32) {
    let model = sim_mjcf::load_model(HINGE_MJCF).expect("parse");
    let mut data = model.make_data();

    for _ in 0..100 {
        data.step(&model).expect("step");
    }

    // Reset to "horizontal" keyframe (qvel = 0.5).
    data.reset_to_keyframe(&model, 1).expect("reset");

    let drift = (data.qvel[0] - model.keyframes[1].qvel[0]).abs();
    let p = check(
        "qvel exact after reset",
        drift == 0.0,
        &format!(
            "qvel = {:.15}, kf = {:.15}, drift = {drift:.2e}",
            data.qvel[0], model.keyframes[1].qvel[0]
        ),
    );
    (u32::from(p), 1)
}

// ── Check 3: ctrl exact after reset ───────────────────────────────────────

fn check_3_ctrl_exact() -> (u32, u32) {
    let model = sim_mjcf::load_model(ACTUATED_MJCF).expect("parse");
    let mut data = model.make_data();

    // Step to mutate ctrl away from keyframe.
    data.ctrl[0] = 99.0;
    for _ in 0..100 {
        data.step(&model).expect("step");
    }

    // Reset to "home" keyframe (ctrl = 1.0).
    data.reset_to_keyframe(&model, 0).expect("reset");

    let drift = (data.ctrl[0] - model.keyframes[0].ctrl[0]).abs();
    let p = check(
        "ctrl exact after reset",
        drift == 0.0,
        &format!(
            "ctrl = {:.15}, kf = {:.15}, drift = {drift:.2e}",
            data.ctrl[0], model.keyframes[0].ctrl[0]
        ),
    );
    (u32::from(p), 1)
}

// ── Check 4: act exact after reset ────────────────────────────────────────

fn check_4_act_exact() -> (u32, u32) {
    let model = sim_mjcf::load_model(ACTUATED_MJCF).expect("parse");
    let mut data = model.make_data();

    // Step to mutate act (dyntype=integrator integrates ctrl into act).
    data.ctrl[0] = 5.0;
    for _ in 0..100 {
        data.step(&model).expect("step");
    }
    assert!(
        data.act[0] != 0.3,
        "act should have drifted from keyframe value"
    );

    // Reset to "home" keyframe (act = 0.3).
    data.reset_to_keyframe(&model, 0).expect("reset");

    let drift = (data.act[0] - model.keyframes[0].act[0]).abs();
    let p = check(
        "act exact after reset",
        drift == 0.0,
        &format!(
            "act = {:.15}, kf = {:.15}, drift = {drift:.2e}",
            data.act[0], model.keyframes[0].act[0]
        ),
    );
    (u32::from(p), 1)
}

// ── Check 5: time exact after reset ───────────────────────────────────────

fn check_5_time_exact() -> (u32, u32) {
    let model = sim_mjcf::load_model(HINGE_MJCF).expect("parse");
    let mut data = model.make_data();

    for _ in 0..100 {
        data.step(&model).expect("step");
    }
    assert!(data.time > 0.0, "time should have advanced");

    // Reset to "horizontal" keyframe (time = 2.5).
    data.reset_to_keyframe(&model, 1).expect("reset");

    let drift = (data.time - model.keyframes[1].time).abs();
    let p = check(
        "time exact after reset",
        drift == 0.0,
        &format!(
            "time = {:.15}, kf = {:.15}, drift = {drift:.2e}",
            data.time, model.keyframes[1].time
        ),
    );
    (u32::from(p), 1)
}

// ── Check 6: qacc zeroed after reset ──────────────────────────────────────

fn check_6_qacc_zeroed() -> (u32, u32) {
    let model = sim_mjcf::load_model(HINGE_MJCF).expect("parse");
    let mut data = model.make_data();

    // Step so qacc becomes nonzero (gravity drives acceleration).
    data.forward(&model).expect("forward");
    assert!(
        data.qacc[0].abs() > 0.0,
        "qacc should be nonzero after forward"
    );

    // Reset to "rest" keyframe.
    data.reset_to_keyframe(&model, 0).expect("reset");

    let max_qacc: f64 = data.qacc.iter().map(|v| v.abs()).fold(0.0, f64::max);
    let p = check(
        "qacc zeroed after reset",
        max_qacc < 1e-15,
        &format!("max |qacc| = {max_qacc:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 7: contacts cleared after reset ─────────────────────────────────

fn check_7_contacts_cleared() -> (u32, u32) {
    let model = sim_mjcf::load_model(CONTACT_MJCF).expect("parse");
    let mut data = model.make_data();

    // Step until the ball hits the floor and generates contacts.
    for _ in 0..500 {
        data.step(&model).expect("step");
    }
    let ncon_before = data.ncon;
    assert!(
        ncon_before > 0,
        "should have contacts after ball hits floor"
    );

    // Reset to "above" keyframe (ball at z=0.5, above floor).
    data.reset_to_keyframe(&model, 0).expect("reset");

    let p = check(
        "contacts cleared after reset",
        data.ncon == 0 && data.contacts.is_empty(),
        &format!("ncon before = {ncon_before}, after reset = {}", data.ncon),
    );
    (u32::from(p), 1)
}

// ── Check 8: 3 keyframes indexed correctly ────────────────────────────────

fn check_8_index_correctness() -> (u32, u32) {
    let model = sim_mjcf::load_model(HINGE_MJCF).expect("parse");
    let mut data = model.make_data();
    let mut passed = 0u32;

    let names = ["rest", "horizontal", "inverted"];
    let expected_qpos: Vec<f64> = model.keyframes.iter().map(|kf| kf.qpos[0]).collect();

    for (i, (name, &expected)) in names.iter().zip(&expected_qpos).enumerate() {
        data.reset_to_keyframe(&model, i).expect("reset");
        let drift = (data.qpos[0] - expected).abs();
        let ok = drift == 0.0;
        if ok {
            passed += 1;
        }
        println!(
            "    [{tag}] keyframe {i} (\"{name}\"): qpos = {:.10}, expected = {expected:.10}",
            data.qpos[0],
            tag = if ok { "PASS" } else { "FAIL" }
        );
    }

    let all_ok = passed == 3;
    let p = check(
        "3 keyframes indexed correctly",
        all_ok,
        &format!("{passed}/3 keyframes matched"),
    );
    (u32::from(p), 1)
}

// ── Check 9: name lookup → correct index → correct state ─────────────────

fn check_9_name_lookup() -> (u32, u32) {
    let model = sim_mjcf::load_model(HINGE_MJCF).expect("parse");
    let mut data = model.make_data();
    let mut passed = 0u32;

    let names = ["rest", "horizontal", "inverted"];

    for (expected_idx, name) in names.iter().enumerate() {
        let idx = model.keyframe_id(name);
        let idx_ok = idx == Some(expected_idx);

        // Also verify via name2id
        let idx2 = model.name2id(ElementType::Keyframe, name);
        let unified_ok = idx == idx2;

        // Verify state after reset
        if let Some(i) = idx {
            data.reset_to_keyframe(&model, i).expect("reset");
            let state_ok = data.qpos[0] == model.keyframes[expected_idx].qpos[0];
            let all_ok = idx_ok && unified_ok && state_ok;
            if all_ok {
                passed += 1;
            }
            println!(
                "    [{tag}] \"{name}\": keyframe_id={idx:?}, name2id={idx2:?}, \
                      qpos match={state_ok}",
                tag = if all_ok { "PASS" } else { "FAIL" }
            );
        } else {
            println!("    [FAIL] \"{name}\": keyframe_id returned None");
        }
    }

    let all_ok = passed == 3;
    let p = check(
        "name lookup → correct index → correct state",
        all_ok,
        &format!("{passed}/3 names resolved correctly"),
    );
    (u32::from(p), 1)
}

// ── Check 10: zero-length act/ctrl fields ─────────────────────────────────

fn check_10_zero_length_fields() -> (u32, u32) {
    let model = sim_mjcf::load_model(NO_ACTUATOR_MJCF).expect("parse");
    let mut data = model.make_data();

    assert_eq!(model.na, 0, "model should have no activations");
    assert_eq!(model.nu, 0, "model should have no controls");

    for _ in 0..100 {
        data.step(&model).expect("step");
    }

    // Reset should succeed even with zero-length act/ctrl.
    let result = data.reset_to_keyframe(&model, 0);

    let ok = result.is_ok();
    let qpos_ok = (data.qpos[0] - 0.5).abs() == 0.0;
    let qvel_ok = (data.qvel[0] - 1.0).abs() == 0.0;

    let p = check(
        "zero-length act/ctrl fields (na=0, nu=0)",
        ok && qpos_ok && qvel_ok,
        &format!("reset ok={ok}, qpos match={qpos_ok}, qvel match={qvel_ok}"),
    );
    (u32::from(p), 1)
}

// ── Check 11: deterministic replay after mid-sim reset ────────────────────

fn check_11_deterministic_replay() -> (u32, u32) {
    let model = sim_mjcf::load_model(HINGE_MJCF).expect("parse");

    // Run A: fresh data → reset to "horizontal" → step 500.
    let mut data_a = model.make_data();
    data_a.reset_to_keyframe(&model, 1).expect("reset");
    data_a.forward(&model).expect("forward");
    for _ in 0..500 {
        data_a.step(&model).expect("step");
    }

    // Run B: step 500 (dirty state) → reset to "horizontal" → step 500.
    let mut data_b = model.make_data();
    for _ in 0..500 {
        data_b.step(&model).expect("step");
    }
    data_b.reset_to_keyframe(&model, 1).expect("reset");
    data_b.forward(&model).expect("forward");
    for _ in 0..500 {
        data_b.step(&model).expect("step");
    }

    let qpos_drift = (data_a.qpos[0] - data_b.qpos[0]).abs();
    let qvel_drift = (data_a.qvel[0] - data_b.qvel[0]).abs();
    let time_drift = (data_a.time - data_b.time).abs();
    let max_drift = qpos_drift.max(qvel_drift).max(time_drift);

    let p = check(
        "deterministic replay after mid-sim reset",
        max_drift < 1e-12,
        &format!(
            "qpos drift = {qpos_drift:.2e}, qvel drift = {qvel_drift:.2e}, \
                  time drift = {time_drift:.2e}"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 12: simulation runs normally after reset ────────────────────────

fn check_12_post_reset_stability() -> (u32, u32) {
    let model = sim_mjcf::load_model(CONTACT_MJCF).expect("parse");
    let mut data = model.make_data();

    // Reset to "above" keyframe, then run 1000 steps.
    data.reset_to_keyframe(&model, 0).expect("reset");
    data.forward(&model).expect("forward");

    for _ in 0..1_000 {
        data.step(&model).expect("step");
    }

    // Check all qpos values are finite.
    let qpos_finite = data.qpos.iter().all(|v| v.is_finite());

    // Check quaternion norm preserved (free joint: qpos[3..7]).
    let qw = data.qpos[3];
    let qx = data.qpos[4];
    let qy = data.qpos[5];
    let qz = data.qpos[6];
    let quat_norm = (qw * qw + qx * qx + qy * qy + qz * qz).sqrt();
    let quat_drift = (quat_norm - 1.0).abs();
    let quat_ok = quat_drift < 1e-10;

    // Check energy is finite.
    let energy = data.energy_kinetic + data.energy_potential;
    let energy_finite = energy.is_finite();

    let all_ok = qpos_finite && quat_ok && energy_finite;

    let p = check(
        "simulation runs normally after reset (1000 steps)",
        all_ok,
        &format!(
            "qpos finite={qpos_finite}, |quat| = {quat_norm:.15} \
                  (drift = {quat_drift:.2e}), energy = {energy:.6} (finite={energy_finite})"
        ),
    );
    (u32::from(p), 1)
}

// ── Main ───────────────────────────────────────────────────────────────────

fn main() {
    println!("=== Keyframes — Stress Test ===\n");

    let checks: Vec<(&str, fn() -> (u32, u32))> = vec![
        ("qpos exact", check_1_qpos_exact),
        ("qvel exact", check_2_qvel_exact),
        ("ctrl exact", check_3_ctrl_exact),
        ("act exact", check_4_act_exact),
        ("time exact", check_5_time_exact),
        ("qacc zeroed", check_6_qacc_zeroed),
        ("contacts cleared", check_7_contacts_cleared),
        ("index correctness", check_8_index_correctness),
        ("name lookup", check_9_name_lookup),
        ("zero-length fields", check_10_zero_length_fields),
        ("deterministic replay", check_11_deterministic_replay),
        ("post-reset stability", check_12_post_reset_stability),
    ];

    let mut total = 0u32;
    let mut passed = 0u32;

    for (i, (label, func)) in checks.iter().enumerate() {
        println!("-- {}. {} --", i + 1, label);
        let (p, t) = func();
        passed += p;
        total += t;
        println!();
    }

    println!("============================================================");
    println!("  TOTAL: {passed}/{total} checks passed");
    if passed == total {
        println!("  ALL PASS");
    } else {
        println!("  {} FAILED", total - passed);
        std::process::exit(1);
    }
}
