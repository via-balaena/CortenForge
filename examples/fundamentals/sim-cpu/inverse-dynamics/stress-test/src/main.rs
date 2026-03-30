//! Stress test — headless validation of all inverse dynamics + Jacobian concepts.
//!
//! 12 checks covering: formula verification, round-trip identity, gravity
//! compensation, dynamic tracking, Jacobian dimensions, velocity mapping,
//! analytical form, torque profile properties, and body accumulators.
//!
//! Run: `cargo run -p example-inverse-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::cast_sign_loss,
    clippy::cast_lossless
)]

use sim_core::mj_jac_site;

// ── Shared MJCF ─────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="inverse-stress-test">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="upper" pos="0 0 0">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 -0.2" mass="2.0" diaginertia="0.02 0.02 0.005"/>
      <geom type="capsule" size="0.03" fromto="0 0 0  0 0 -0.4"/>
      <body name="lower" pos="0 0 -0.4">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 -0.15" mass="1.0" diaginertia="0.008 0.008 0.002"/>
        <geom type="capsule" size="0.025" fromto="0 0 0  0 0 -0.3"/>
        <site name="end_effector" pos="0 0 -0.3"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="shoulder_motor" joint="shoulder" gear="1"/>
    <motor name="elbow_motor" joint="elbow" gear="1"/>
  </actuator>
</mujoco>
"#;

const L1: f64 = 0.4;
const L2: f64 = 0.3;

// ── Helpers ─────────────────────────────────────────────────────────────────

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

fn trajectory(t: f64) -> ([f64; 2], [f64; 2], [f64; 2]) {
    let a1 = 0.8;
    let a2 = 0.6;
    let w1 = 2.0;
    let w2 = 3.0;
    let phi = 1.0;
    let qpos = [a1 * (w1 * t).sin(), a2 * (w2 * t + phi).sin()];
    let qvel = [a1 * w1 * (w1 * t).cos(), a2 * w2 * (w2 * t + phi).cos()];
    let qacc = [
        -a1 * w1 * w1 * (w1 * t).sin(),
        -a2 * w2 * w2 * (w2 * t + phi).sin(),
    ];
    (qpos, qvel, qacc)
}

// ── Main ────────────────────────────────────────────────────────────────────

fn main() {
    println!("=== Inverse Dynamics — Stress Test ===\n");

    let mut total = 0;
    let mut passed = 0;

    println!("── 1. Formula verification ──");
    let (p, t) = test_formula();
    passed += p;
    total += t;

    println!("\n── 2. Round-trip identity ──");
    let (p, t) = test_round_trip();
    passed += p;
    total += t;

    println!("\n── 3. Free-fall identity ──");
    let (p, t) = test_freefall();
    passed += p;
    total += t;

    println!("\n── 4. Gravity compensation ──");
    let (p, t) = test_gravity_comp();
    passed += p;
    total += t;

    println!("\n── 5. Dynamic tracking ──");
    let (p, t) = test_dynamic_tracking();
    passed += p;
    total += t;

    println!("\n── 6. Jacobian ──");
    let (p, t) = test_jacobian();
    passed += p;
    total += t;

    println!("\n── 7. Torque profile ──");
    let (p, t) = test_torque_profile();
    passed += p;
    total += t;

    println!("\n── 8. Body accumulators ──");
    let (p, t) = test_body_accumulators();
    passed += p;
    total += t;

    println!("\n============================================================");
    println!("  TOTAL: {passed}/{total} checks passed");
    if passed == total {
        println!("  ALL PASS");
    } else {
        println!("  {} FAILED", total - passed);
        std::process::exit(1);
    }
}

// ── 1. Formula: qfrc_inverse == M*qacc + bias - passive - constraint ────────

fn test_formula() -> (u32, u32) {
    let model = sim_mjcf::load_model(MJCF).expect("parse");
    let mut data = model.make_data();

    data.qpos[0] = 0.8;
    data.qpos[1] = -0.4;
    data.qvel[0] = 2.0;
    data.qvel[1] = -1.0;
    data.ctrl[0] = 3.0;
    data.ctrl[1] = -1.5;
    data.qfrc_applied[0] = 2.0;

    data.forward(&model).expect("forward");
    data.inverse(&model);

    let mut max_err: f64 = 0.0;
    for dof in 0..model.nv {
        let mut m_qacc = 0.0;
        for j in 0..model.nv {
            m_qacc += data.qM[(dof, j)] * data.qacc[j];
        }
        let formula =
            m_qacc + data.qfrc_bias[dof] - data.qfrc_passive[dof] - data.qfrc_constraint[dof];
        let err = (data.qfrc_inverse[dof] - formula).abs();
        max_err = max_err.max(err);
    }

    let p = check(
        "Formula matches",
        max_err < 1e-10,
        &format!("max err={max_err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── 2. Round-trip: qfrc_inverse == qfrc_applied + qfrc_actuator ─────────────

fn test_round_trip() -> (u32, u32) {
    let model = sim_mjcf::load_model(MJCF).expect("parse");
    let mut data = model.make_data();

    data.qpos[0] = 0.5;
    data.qpos[1] = -0.3;
    data.qvel[0] = 1.0;
    data.qvel[1] = -0.5;
    data.ctrl[0] = 5.0;
    data.ctrl[1] = -2.0;
    data.qfrc_applied[0] = 1.0;
    data.qfrc_applied[1] = -0.5;

    data.forward(&model).expect("forward");
    data.inverse(&model);

    let mut max_err: f64 = 0.0;
    for i in 0..model.nv {
        let expected = data.qfrc_applied[i] + data.qfrc_actuator[i];
        let err = (data.qfrc_inverse[i] - expected).abs();
        max_err = max_err.max(err);
    }

    let p = check(
        "Round-trip identity",
        max_err < 1e-8,
        &format!("max err={max_err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── 3. Free-fall: qfrc_inverse ≈ 0 with no applied forces ──────────────────

fn test_freefall() -> (u32, u32) {
    let mjcf = r#"
    <mujoco>
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <worldbody>
        <body pos="0 0 1">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="1.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let model = sim_mjcf::load_model(mjcf).expect("parse");
    let mut data = model.make_data();

    data.forward(&model).expect("forward");
    data.inverse(&model);

    let mut max_inv: f64 = 0.0;
    for i in 0..model.nv {
        max_inv = max_inv.max(data.qfrc_inverse[i].abs());
    }

    let p = check(
        "Free-fall inverse ≈ 0",
        max_inv < 1e-8,
        &format!("max |qfrc_inverse|={max_inv:.2e}"),
    );
    (u32::from(p), 1)
}

// ── 4. Gravity compensation: arm holds pose ─────────────────────────────────

fn test_gravity_comp() -> (u32, u32) {
    let model = sim_mjcf::load_model(MJCF).expect("parse");
    let mut data = model.make_data();
    let mut passed = 0u32;

    // Set pose, compute holding torques
    data.qpos[0] = 0.785;
    data.qpos[1] = -0.524;
    data.forward(&model).expect("forward");
    data.qacc[0] = 0.0;
    data.qacc[1] = 0.0;
    data.inverse(&model);

    // Check: inv == bias (static case)
    let mut bias_err: f64 = 0.0;
    for i in 0..model.nv {
        bias_err = bias_err.max((data.qfrc_inverse[i] - data.qfrc_bias[i]).abs());
    }
    passed += u32::from(check(
        "inv == bias (static)",
        bias_err < 1e-8,
        &format!("max err={bias_err:.2e}"),
    ));

    // Check: shoulder > elbow
    let s_gt_e = data.qfrc_inverse[0].abs() > data.qfrc_inverse[1].abs();
    passed += u32::from(check(
        "|shoulder| > |elbow|",
        s_gt_e,
        &format!(
            "|{:.3}| vs |{:.3}|",
            data.qfrc_inverse[0], data.qfrc_inverse[1]
        ),
    ));

    // Apply holding torques, simulate 5 seconds
    let tau = [data.qfrc_inverse[0], data.qfrc_inverse[1]];
    let init = [data.qpos[0], data.qpos[1]];

    // Reset and apply
    data.qpos[0] = init[0];
    data.qpos[1] = init[1];
    data.qvel[0] = 0.0;
    data.qvel[1] = 0.0;
    data.forward(&model).expect("forward");

    let steps = (5.0 / model.timestep).round() as usize;
    let mut max_drift: f64 = 0.0;

    for _ in 0..steps {
        data.set_ctrl(0, tau[0]);
        data.set_ctrl(1, tau[1]);
        data.step(&model).expect("step");

        let s_drift = (data.qpos[0] - init[0]).abs();
        let e_drift = (data.qpos[1] - init[1]).abs();
        max_drift = max_drift.max(s_drift).max(e_drift);
    }

    passed += u32::from(check(
        "Drift < 0.001 rad over 5s",
        max_drift < 0.001,
        &format!("max drift={max_drift:.2e} rad"),
    ));

    (passed, 3)
}

// ── 5. Dynamic tracking: inverse→forward replay ─────────────────────────────

fn test_dynamic_tracking() -> (u32, u32) {
    let model = sim_mjcf::load_model(MJCF).expect("parse");
    let dt = model.timestep;
    let duration = 5.0;
    let steps = (duration / dt).round() as usize;

    // Phase 1: compute torque profile
    let mut inv_data = model.make_data();
    let mut torques = Vec::with_capacity(steps);

    for i in 0..steps {
        let t = i as f64 * dt;
        let (qpos, qvel, qacc) = trajectory(t);

        inv_data.qpos[0] = qpos[0];
        inv_data.qpos[1] = qpos[1];
        inv_data.qvel[0] = qvel[0];
        inv_data.qvel[1] = qvel[1];
        inv_data.forward(&model).expect("forward");
        inv_data.qacc[0] = qacc[0];
        inv_data.qacc[1] = qacc[1];
        inv_data.inverse(&model);
        torques.push([inv_data.qfrc_inverse[0], inv_data.qfrc_inverse[1]]);
    }

    // Phase 2: replay
    let mut fwd_data = model.make_data();
    let (q0, v0, _) = trajectory(0.0);
    fwd_data.qpos[0] = q0[0];
    fwd_data.qpos[1] = q0[1];
    fwd_data.qvel[0] = v0[0];
    fwd_data.qvel[1] = v0[1];
    fwd_data.forward(&model).expect("forward");

    let mut max_err: f64 = 0.0;

    for (i, tau) in torques.iter().enumerate() {
        fwd_data.set_ctrl(0, tau[0]);
        fwd_data.set_ctrl(1, tau[1]);
        fwd_data.step(&model).expect("step");

        let t = (i + 1) as f64 * dt;
        let (qd, _, _) = trajectory(t);
        let s_err = (fwd_data.qpos[0] - qd[0]).abs();
        let e_err = (fwd_data.qpos[1] - qd[1]).abs();
        max_err = max_err.max(s_err).max(e_err);
    }

    let p = check(
        "Tracking < 0.02 rad",
        max_err < 0.02,
        &format!("max err={max_err:.4e} rad over {steps} steps"),
    );
    (u32::from(p), 1)
}

// ── 6. Jacobian ─────────────────────────────────────────────────────────────

fn test_jacobian() -> (u32, u32) {
    let model = sim_mjcf::load_model(MJCF).expect("parse");
    let mut data = model.make_data();
    let site_id = model.site_id("end_effector").expect("site");
    let mut passed = 0u32;

    // 6a. Dimensions
    data.forward(&model).expect("forward");
    let (jacp, jacr) = mj_jac_site(&model, &data, site_id);
    let dims_ok = jacp.nrows() == 3
        && jacp.ncols() == model.nv
        && jacr.nrows() == 3
        && jacr.ncols() == model.nv;
    passed += u32::from(check(
        "Dimensions 3×nv",
        dims_ok,
        &format!(
            "jacp={}x{}, jacr={}x{} (expect 3x{})",
            jacp.nrows(),
            jacp.ncols(),
            jacr.nrows(),
            jacr.ncols(),
            model.nv
        ),
    ));

    // 6b. Analytical form at zero config
    let expected_x = -(L1 + L2);
    let actual_x = jacp[(0, 0)];
    let zero_err = (actual_x - expected_x).abs();
    passed += u32::from(check(
        "Zero-config analytical",
        zero_err < 1e-6,
        &format!("shoulder_x={actual_x:.4}, expect={expected_x:.4}, err={zero_err:.2e}"),
    ));

    // 6c. Velocity match (central FD)
    data.qpos[0] = 1.0;
    data.qpos[1] = -0.5;
    data.forward(&model).expect("forward");

    let dt = model.timestep;
    let mut max_rel_err: f64 = 0.0;
    let mut samples = 0u64;
    let mut prev_pos: Option<[f64; 3]> = None;

    for _ in 0..3000 {
        let ee = data.site_xpos[site_id];
        let before = [ee.x, ee.y, ee.z];

        let (jp, _) = mj_jac_site(&model, &data, site_id);
        let mut v_pred = [0.0; 3];
        for row in 0..3 {
            for col in 0..model.nv {
                v_pred[row] += jp[(row, col)] * data.qvel[col];
            }
        }

        data.step(&model).expect("step");

        let ee_after = data.site_xpos[site_id];
        let after = [ee_after.x, ee_after.y, ee_after.z];

        if let Some(prev) = prev_pos {
            let v_act = [
                (after[0] - prev[0]) / (2.0 * dt),
                (after[1] - prev[1]) / (2.0 * dt),
                (after[2] - prev[2]) / (2.0 * dt),
            ];

            let v_mag =
                (v_pred[0] * v_pred[0] + v_pred[1] * v_pred[1] + v_pred[2] * v_pred[2]).sqrt();
            let err = ((v_pred[0] - v_act[0]).powi(2)
                + (v_pred[1] - v_act[1]).powi(2)
                + (v_pred[2] - v_act[2]).powi(2))
            .sqrt();

            if v_mag > 0.05 {
                max_rel_err = max_rel_err.max(err / v_mag);
                samples += 1;
            }
        }
        prev_pos = Some(before);
    }

    passed += u32::from(check(
        "Velocity match < 1%",
        max_rel_err < 0.01,
        &format!(
            "max rel err={:.4}% over {samples} samples",
            max_rel_err * 100.0
        ),
    ));

    (passed, 3)
}

// ── 7. Torque profile properties ────────────────────────────────────────────

fn test_torque_profile() -> (u32, u32) {
    let model = sim_mjcf::load_model(MJCF).expect("parse");
    let mut data = model.make_data();
    let dt = model.timestep;
    let mut passed = 0u32;

    let mut shoulder_peak: f64 = 0.0;
    let mut elbow_peak: f64 = 0.0;
    let mut shoulder_sum_sq: f64 = 0.0;
    let mut max_jump: f64 = 0.0;
    let mut prev_shoulder: Option<f64> = None;
    let steps = 2500;

    for i in 0..steps {
        let t = i as f64 * dt;
        let (qpos, qvel, qacc) = trajectory(t);

        data.qpos[0] = qpos[0];
        data.qpos[1] = qpos[1];
        data.qvel[0] = qvel[0];
        data.qvel[1] = qvel[1];
        data.forward(&model).expect("forward");
        data.qacc[0] = qacc[0];
        data.qacc[1] = qacc[1];
        data.inverse(&model);

        let s_tau = data.qfrc_inverse[0];
        let e_tau = data.qfrc_inverse[1];

        shoulder_peak = shoulder_peak.max(s_tau.abs());
        elbow_peak = elbow_peak.max(e_tau.abs());
        shoulder_sum_sq += s_tau * s_tau;

        if let Some(prev) = prev_shoulder {
            max_jump = max_jump.max((s_tau - prev).abs());
        }
        prev_shoulder = Some(s_tau);
    }

    let rms = (shoulder_sum_sq / steps as f64).sqrt();
    let spike_ratio = if rms > 1e-10 { max_jump / rms } else { 0.0 };

    passed += u32::from(check(
        "|shoulder| > |elbow| peak",
        shoulder_peak > elbow_peak,
        &format!("{shoulder_peak:.3} vs {elbow_peak:.3}"),
    ));

    passed += u32::from(check(
        "Smooth (spike < 3× RMS)",
        spike_ratio < 3.0,
        &format!("ratio={spike_ratio:.2}, jump={max_jump:.3}, rms={rms:.3}"),
    ));

    (passed, 2)
}

// ── 8. Body accumulators ────────────────────────────────────────────────────

fn test_body_accumulators() -> (u32, u32) {
    let model = sim_mjcf::load_model(MJCF).expect("parse");
    let mut data = model.make_data();
    let mut passed = 0u32;

    data.qpos[0] = 0.5;
    data.forward(&model).expect("forward");
    data.inverse(&model);

    // cacc should be non-zero under gravity
    let mut cacc_mag: f64 = 0.0;
    for body in 1..model.nbody {
        for i in 0..6 {
            cacc_mag = cacc_mag.max(data.cacc[body][i].abs());
        }
    }
    passed += u32::from(check(
        "cacc non-zero (gravity)",
        cacc_mag > 1.0,
        &format!("max |cacc|={cacc_mag:.3}"),
    ));

    // cfrc_ext: with xfrc_applied (Cartesian force on body), cfrc_ext should reflect it
    let mjcf_free = r#"
    <mujoco>
      <option gravity="0 0 0" timestep="0.002"/>
      <worldbody>
        <body pos="0 0 0">
          <freejoint/>
          <geom type="sphere" size="0.1" mass="2.0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    let m2 = sim_mjcf::load_model(mjcf_free).expect("parse");
    let mut d2 = m2.make_data();
    // xfrc_applied is Cartesian force/torque on body (6D: [torque, force])
    d2.xfrc_applied[1][3] = 10.0; // 10 N in x on body 1
    d2.forward(&m2).expect("forward");
    d2.inverse(&m2);

    // cfrc_ext[1] should reflect the applied Cartesian force
    let fx = d2.cfrc_ext[1][3].abs() + d2.cfrc_ext[1][4].abs() + d2.cfrc_ext[1][5].abs();
    passed += u32::from(check(
        "cfrc_ext reflects xfrc_applied",
        fx > 1.0,
        &format!("linear force mag={fx:.3}"),
    ));

    (passed, 2)
}
