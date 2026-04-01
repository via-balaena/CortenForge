//! Stress test — headless validation of all joint limit constraint invariants.
//!
//! 12 checks covering: hinge/slide/ball limit activation, JointLimitFrc sensor
//! readback, one-sided constraints, solref stiffness scaling, solimp width
//! control, motor vs limit, locked joints, penetration-force relationship, and
//! ball cone azimuthal symmetry.
//!
//! Run: `cargo run -p example-joint-limits-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::needless_range_loop,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::suboptimal_flops
)]

// ── MJCF Models ────────────────────────────────────────────────────────────

/// Model A — Hinge limit basics (checks 1, 4, 5, 6).
/// Pendulum with hinge joint, range ±45°, ref=60° (starts beyond upper limit).
/// Includes JointLimitFrc sensor.
const MODEL_HINGE: &str = r#"
<mujoco model="hinge-limit">
  <compiler angle="degree"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="pivot" pos="0 0 1.5">
      <body name="rod" pos="0 0 0">
        <joint name="hinge" type="hinge" axis="0 1 0"
               limited="true" range="-45 45" ref="60"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02" mass="1.0"/>
      </body>
    </body>
  </worldbody>
  <sensor>
    <jointlimitfrc name="hinge_lf" joint="hinge"/>
  </sensor>
</mujoco>
"#;

/// Model B — Hinge within range (check 5: sensor == 0 when interior).
/// Ref=0 → starts at center of range, no limit violation.
const MODEL_HINGE_INTERIOR: &str = r#"
<mujoco model="hinge-interior">
  <compiler angle="degree"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="pivot" pos="0 0 1.5">
      <body name="rod" pos="0 0 0">
        <joint name="hinge" type="hinge" axis="0 1 0"
               limited="true" range="-45 45" ref="0" damping="5.0"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02" mass="1.0"/>
      </body>
    </body>
  </worldbody>
  <sensor>
    <jointlimitfrc name="hinge_lf" joint="hinge"/>
  </sensor>
</mujoco>
"#;

/// Model C — Slide limit (check 2, 9).
/// Slider on X-axis, range ±0.3, ref=0.5 (starts beyond upper limit).
/// Motor pushes into positive limit.
const MODEL_SLIDE: &str = r#"
<mujoco model="slide-limit">
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.1"/>
    <body name="rail" pos="0 0 0.3">
      <body name="slider" pos="0 0 0">
        <joint name="slide" type="slide" axis="1 0 0"
               limited="true" range="-0.3 0.3" ref="0.5"/>
        <geom type="box" size="0.05 0.05 0.05" mass="1.0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="push" joint="slide" gear="1"/>
  </actuator>
  <sensor>
    <jointlimitfrc name="slide_lf" joint="slide"/>
  </sensor>
</mujoco>
"#;

/// Model D — Ball joint cone limit (checks 3, 12).
/// Ball joint with 30° cone. Initial orientation set programmatically.
const MODEL_BALL: &str = r#"
<mujoco model="ball-cone">
  <compiler angle="degree"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="pivot" pos="0 0 1.5">
      <body name="rod" pos="0 0 0">
        <joint name="ball" type="ball" limited="true" range="0 30"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02" mass="1.0"/>
      </body>
    </body>
  </worldbody>
  <sensor>
    <jointlimitfrc name="ball_lf" joint="ball"/>
  </sensor>
</mujoco>
"#;

/// Model E — Solref comparison (check 7).
/// Two hinges with different solreflimit, both starting at 60° (limit at 45°).
const MODEL_SOLREF: &str = r#"
<mujoco model="solref-compare">
  <compiler angle="degree"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="pivot_stiff" pos="-0.5 0 1.5">
      <body name="rod_stiff" pos="0 0 0">
        <joint name="stiff" type="hinge" axis="0 1 0"
               limited="true" range="-45 45" ref="60"
               solreflimit="0.005 1.0"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02" mass="1.0"/>
      </body>
    </body>
    <body name="pivot_soft" pos="0.5 0 1.5">
      <body name="rod_soft" pos="0 0 0">
        <joint name="soft" type="hinge" axis="0 1 0"
               limited="true" range="-45 45" ref="60"
               solreflimit="0.08 1.0"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02" mass="1.0"/>
      </body>
    </body>
  </worldbody>
  <sensor>
    <jointlimitfrc name="stiff_lf" joint="stiff"/>
    <jointlimitfrc name="soft_lf" joint="soft"/>
  </sensor>
</mujoco>
"#;

/// Model F — Solimp width comparison (check 8).
/// Two sliders with different solimp width (3rd element), each pushed by a motor
/// into the upper limit. At equilibrium, wider solimp allows more penetration
/// because the impedance sigmoid stays at d0 (near zero) longer.
/// d0=0.01, d_width=0.99: dramatic contrast between narrow and wide transition.
const MODEL_SOLIMP: &str = r#"
<mujoco model="solimp-compare">
  <option gravity="0 0 0" timestep="0.002"/>
  <worldbody>
    <body name="narrow_body" pos="-0.5 0 0.3">
      <joint name="narrow" type="slide" axis="1 0 0"
             limited="true" range="-0.5 0.3" damping="5.0"
             solimplimit="0.01 0.99 0.001 0.5 2.0"/>
      <geom type="box" size="0.05 0.05 0.05" mass="1.0"/>
    </body>
    <body name="wide_body" pos="0.5 0 0.3">
      <joint name="wide" type="slide" axis="1 0 0"
             limited="true" range="-0.5 0.3" damping="5.0"
             solimplimit="0.01 0.99 0.5 0.5 2.0"/>
      <geom type="box" size="0.05 0.05 0.05" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="push_narrow" joint="narrow" gear="1"/>
    <motor name="push_wide" joint="wide" gear="1"/>
  </actuator>
</mujoco>
"#;

/// Model G — Zero-width range (check 10).
/// Hinge with range="0 0" → effectively locked at 0.
const MODEL_LOCKED: &str = r#"
<mujoco model="locked-hinge">
  <compiler angle="degree"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="pivot" pos="0 0 1.5">
      <body name="rod" pos="0 0 0">
        <joint name="locked" type="hinge" axis="0 1 0"
               limited="true" range="0 0" ref="0"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02" mass="1.0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model H — Penetration vs force (check 11).
/// Two hinges at different initial angles beyond the same limit.
/// "mild" starts at 50° (5° beyond limit), "severe" at 70° (25° beyond limit).
const MODEL_PENETRATION: &str = r#"
<mujoco model="penetration-force">
  <compiler angle="degree"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="pivot_mild" pos="-0.5 0 1.5">
      <body name="rod_mild" pos="0 0 0">
        <joint name="mild" type="hinge" axis="0 1 0"
               limited="true" range="-45 45" ref="50"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02" mass="1.0"/>
      </body>
    </body>
    <body name="pivot_severe" pos="0.5 0 1.5">
      <body name="rod_severe" pos="0 0 0">
        <joint name="severe" type="hinge" axis="0 1 0"
               limited="true" range="-45 45" ref="70"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02" mass="1.0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Helpers ────────────────────────────────────────────────────────────────

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

use sim_core::validation::quat_rotation_angle;

// ── Check 1: Hinge limit activates ─────────────────────────────────────────

fn check_1_hinge_limit_activates() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_HINGE).expect("parse");
    let mut data = model.make_data();

    for _ in 0..1000 {
        data.step(&model).expect("step");
    }

    let jid = model.joint_id("hinge").expect("joint");
    let q = data.qpos[model.jnt_qpos_adr[jid]];
    let limit_deg = 45.0;
    let tol_deg = 3.0;
    let q_deg = q.to_degrees();
    let p = check(
        "Hinge limit activates",
        q_deg <= limit_deg + tol_deg,
        &format!("angle = {q_deg:.2}° (limit = {limit_deg}°, tol = {tol_deg}°)"),
    );
    (u32::from(p), 1)
}

// ── Check 2: Slide limit activates ─────────────────────────────────────────

fn check_2_slide_limit_activates() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_SLIDE).expect("parse");
    let mut data = model.make_data();

    for _ in 0..1000 {
        data.step(&model).expect("step");
    }

    let jid = model.joint_id("slide").expect("joint");
    let q = data.qpos[model.jnt_qpos_adr[jid]];
    let limit = 0.3;
    let tol = 0.02;
    let p = check(
        "Slide limit activates",
        q <= limit + tol,
        &format!("pos = {q:.4} (limit = {limit}, tol = {tol})"),
    );
    (u32::from(p), 1)
}

// ── Check 3: Ball cone limit activates ─────────────────────────────────────

fn check_3_ball_cone_activates() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_BALL).expect("parse");
    let mut data = model.make_data();

    // Set initial quaternion to ~45° tilt about X (beyond 30° cone)
    let jid = model.joint_id("ball").expect("joint");
    let adr = model.jnt_qpos_adr[jid];
    let angle = 45.0_f64.to_radians();
    let half = angle / 2.0;
    data.qpos[adr] = half.cos(); // w
    data.qpos[adr + 1] = half.sin(); // x
    data.qpos[adr + 2] = 0.0; // y
    data.qpos[adr + 3] = 0.0; // z

    for _ in 0..1000 {
        data.step(&model).expect("step");
    }

    let q = data.joint_qpos(&model, jid);
    let deflection = quat_rotation_angle(q[0], q[1], q[2], q[3]).to_degrees();
    let limit_deg = 30.0;
    let tol_deg = 5.0;
    let p = check(
        "Ball cone limit activates",
        deflection <= limit_deg + tol_deg,
        &format!("deflection = {deflection:.2}° (limit = {limit_deg}°, tol = {tol_deg}°)"),
    );
    (u32::from(p), 1)
}

// ── Check 4: JointLimitFrc > 0 at limit ────────────────────────────────────

fn check_4_sensor_positive_at_limit() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_HINGE).expect("parse");
    let mut data = model.make_data();

    // Starts at 60° (beyond 45° limit) — limit fires on the very first step.
    // Track peak force over first 10 steps (before pendulum bounces back).
    let jid = model.joint_id("hinge").expect("joint");
    let mut max_frc = 0.0_f64;
    for _ in 0..10 {
        data.step(&model).expect("step");
        max_frc = max_frc.max(data.jnt_limit_frc[jid]);
    }

    let p = check(
        "JointLimitFrc > 0 at limit",
        max_frc > 0.0,
        &format!("peak limit_frc = {max_frc:.4} (over first 10 steps)"),
    );
    (u32::from(p), 1)
}

// ── Check 5: JointLimitFrc == 0 when interior ──────────────────────────────

fn check_5_sensor_zero_interior() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_HINGE_INTERIOR).expect("parse");
    let mut data = model.make_data();

    // Starts at 0° (center of ±45° range), heavy damping to kill oscillation.
    // Step enough to settle.
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    let jid = model.joint_id("hinge").expect("joint");
    let q_deg = data.qpos[model.jnt_qpos_adr[jid]].to_degrees();
    let frc = data.jnt_limit_frc[jid];
    let p = check(
        "JointLimitFrc == 0 when interior",
        frc == 0.0,
        &format!("limit_frc = {frc:.6}, angle = {q_deg:.2}° (inside ±45° range)"),
    );
    (u32::from(p), 1)
}

// ── Check 6: One-sided constraint ──────────────────────────────────────────

fn check_6_one_sided() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_HINGE).expect("parse");
    let mut data = model.make_data();

    // Starts at 60° → upper limit (45°) is violated on the first step.
    // Check immediately after step 1: angle > 0 and force > 0 (upper active),
    // lower limit at -45° is nowhere near active.
    let jid = model.joint_id("hinge").expect("joint");

    data.step(&model).expect("step");

    let q_deg = data.qpos[model.jnt_qpos_adr[jid]].to_degrees();
    let frc = data.jnt_limit_frc[jid];

    let upper_active = q_deg > 40.0 && frc > 0.0;
    let p = check(
        "Limit is one-sided",
        upper_active,
        &format!(
            "angle = {q_deg:.2}° (above upper limit), limit_frc = {frc:.4} (lower at -45° not active)"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 7: Solref stiffness scales force ─────────────────────────────────

fn check_7_solref_stiffness() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_SOLREF).expect("parse");
    let mut data = model.make_data();

    let stiff_id = model.joint_id("stiff").expect("stiff joint");
    let soft_id = model.joint_id("soft").expect("soft joint");

    // Track peak limit force over the first 200 steps (initial impact)
    let mut stiff_max = 0.0_f64;
    let mut soft_max = 0.0_f64;

    for _ in 0..200 {
        data.step(&model).expect("step");
        stiff_max = stiff_max.max(data.jnt_limit_frc[stiff_id]);
        soft_max = soft_max.max(data.jnt_limit_frc[soft_id]);
    }

    let p = check(
        "Solref stiffness scales force",
        stiff_max > soft_max && soft_max > 0.0,
        &format!(
            "stiff peak = {stiff_max:.4} (solref=0.005), soft peak = {soft_max:.4} (solref=0.08)"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 8: Solimp width controls penetration ────────────────────────────

fn check_8_solimp_width() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_SOLIMP).expect("parse");
    let mut data = model.make_data();

    let narrow_id = model.joint_id("narrow").expect("narrow joint");
    let wide_id = model.joint_id("wide").expect("wide joint");
    let limit = 0.3;

    // Push both sliders into the upper limit with equal motor force
    data.ctrl[0] = 5.0;
    data.ctrl[1] = 5.0;

    // Let them settle (damping=5.0 in zero-gravity)
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    let narrow_q = data.qpos[model.jnt_qpos_adr[narrow_id]];
    let wide_q = data.qpos[model.jnt_qpos_adr[wide_id]];
    let narrow_pen = narrow_q - limit;
    let wide_pen = wide_q - limit;

    let p = check(
        "Solimp width controls penetration",
        wide_pen > narrow_pen,
        &format!(
            "narrow pen = {narrow_pen:.6}m (width=0.001), wide pen = {wide_pen:.6}m (width=0.5)"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 9: Motor cannot push past limit ──────────────────────────────────

fn check_9_motor_vs_limit() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_SLIDE).expect("parse");
    let mut data = model.make_data();

    // Reset slider to inside range, then let motor push it into limit
    let jid = model.joint_id("slide").expect("joint");
    let adr = model.jnt_qpos_adr[jid];
    data.qpos[adr] = 0.0; // start at center

    // Apply constant force pushing into positive limit
    data.ctrl[0] = 10.0;

    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    let q = data.qpos[adr];
    let limit = 0.3;
    let tol = 0.02;
    let p = check(
        "Motor cannot push past limit",
        q <= limit + tol,
        &format!("pos = {q:.4} with motor force = 10.0 (limit = {limit}, tol = {tol})"),
    );
    (u32::from(p), 1)
}

// ── Check 10: Zero-width range (locked joint) ─────────────────────────────

fn check_10_locked_joint() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_LOCKED).expect("parse");
    let mut data = model.make_data();

    let jid = model.joint_id("locked").expect("joint");
    let adr = model.jnt_qpos_adr[jid];

    // Track max deviation from 0 over 1000 steps
    let mut max_dev = 0.0_f64;
    for _ in 0..1000 {
        data.step(&model).expect("step");
        max_dev = max_dev.max(data.qpos[adr].abs());
    }

    let tol_deg = 1.0_f64;
    let tol_rad = tol_deg.to_radians();
    let p = check(
        "Zero-width range holds position",
        max_dev < tol_rad,
        &format!(
            "max deviation = {:.4}° (tol = {tol_deg}°)",
            max_dev.to_degrees()
        ),
    );
    (u32::from(p), 1)
}

// ── Check 11: Higher penetration → higher peak force ───────────────────────

fn check_11_force_vs_penetration() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_PENETRATION).expect("parse");
    let mut data = model.make_data();

    let mild_id = model.joint_id("mild").expect("mild joint");
    let severe_id = model.joint_id("severe").expect("severe joint");

    let mut mild_max = 0.0_f64;
    let mut severe_max = 0.0_f64;

    for _ in 0..200 {
        data.step(&model).expect("step");
        mild_max = mild_max.max(data.jnt_limit_frc[mild_id]);
        severe_max = severe_max.max(data.jnt_limit_frc[severe_id]);
    }

    let p = check(
        "Higher penetration → higher peak force",
        severe_max > mild_max && mild_max > 0.0,
        &format!("mild peak (ref=50°) = {mild_max:.4}, severe peak (ref=70°) = {severe_max:.4}"),
    );
    (u32::from(p), 1)
}

// ── Check 12: Ball cone azimuthal symmetry ─────────────────────────────────

fn check_12_ball_cone_symmetry() -> (u32, u32) {
    // Run two simulations: one tilted about X, one about Y.
    // Both should produce similar peak limit forces.
    let angle = 45.0_f64.to_radians();
    let half = angle / 2.0;

    // Tilt about X
    let model_x = sim_mjcf::load_model(MODEL_BALL).expect("parse");
    let mut data_x = model_x.make_data();
    let jid = model_x.joint_id("ball").expect("joint");
    let adr = model_x.jnt_qpos_adr[jid];
    data_x.qpos[adr] = half.cos();
    data_x.qpos[adr + 1] = half.sin();
    data_x.qpos[adr + 2] = 0.0;
    data_x.qpos[adr + 3] = 0.0;

    let mut max_frc_x = 0.0_f64;
    for _ in 0..200 {
        data_x.step(&model_x).expect("step");
        max_frc_x = max_frc_x.max(data_x.jnt_limit_frc[jid]);
    }

    // Tilt about Y
    let model_y = sim_mjcf::load_model(MODEL_BALL).expect("parse");
    let mut data_y = model_y.make_data();
    data_y.qpos[adr] = half.cos();
    data_y.qpos[adr + 1] = 0.0;
    data_y.qpos[adr + 2] = half.sin();
    data_y.qpos[adr + 3] = 0.0;

    let mut max_frc_y = 0.0_f64;
    for _ in 0..200 {
        data_y.step(&model_y).expect("step");
        max_frc_y = max_frc_y.max(data_y.jnt_limit_frc[jid]);
    }

    // Both should be positive and within 20% of each other
    let ratio = if max_frc_x > max_frc_y {
        max_frc_y / max_frc_x
    } else {
        max_frc_x / max_frc_y
    };
    let symmetric = ratio > 0.8 && max_frc_x > 0.0 && max_frc_y > 0.0;

    let p = check(
        "Ball cone azimuthal symmetry",
        symmetric,
        &format!("X-tilt peak = {max_frc_x:.4}, Y-tilt peak = {max_frc_y:.4}, ratio = {ratio:.3}"),
    );
    (u32::from(p), 1)
}

// ── Main ──────────────────────────────────────────────────────────────────

fn main() {
    println!("=== Joint Limits — Stress Test ===\n");

    let checks: Vec<(&str, fn() -> (u32, u32))> = vec![
        ("Hinge limit activates", check_1_hinge_limit_activates),
        ("Slide limit activates", check_2_slide_limit_activates),
        ("Ball cone limit activates", check_3_ball_cone_activates),
        (
            "JointLimitFrc > 0 at limit",
            check_4_sensor_positive_at_limit,
        ),
        (
            "JointLimitFrc == 0 when interior",
            check_5_sensor_zero_interior,
        ),
        ("Limit is one-sided", check_6_one_sided),
        ("Solref stiffness scales force", check_7_solref_stiffness),
        ("Solimp width controls penetration", check_8_solimp_width),
        ("Motor cannot push past limit", check_9_motor_vs_limit),
        ("Zero-width range holds position", check_10_locked_joint),
        (
            "Higher penetration -> higher peak force",
            check_11_force_vs_penetration,
        ),
        ("Ball cone azimuthal symmetry", check_12_ball_cone_symmetry),
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
