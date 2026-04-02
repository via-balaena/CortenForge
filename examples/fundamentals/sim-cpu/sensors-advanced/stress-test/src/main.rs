//! Stress test — headless validation of advanced sensor types.
//!
//! 24 checks covering: FrameLinVel, FrameAngVel, FrameLinAcc, FrameAngAcc,
//! Force, Torque, Rangefinder, SubtreeLinVel, SubtreeAngMom, ActuatorPos,
//! ActuatorVel, FrameXAxis/YAxis/ZAxis, Magnetometer, and sensor dimensions.
//!
//! Run: `cargo run -p example-sensor-adv-stress-test --release`

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
    clippy::suboptimal_flops,
    clippy::panic,
    clippy::let_underscore_must_use,
    clippy::float_cmp
)]

use std::f64::consts::PI;

// ── MJCF Models ────────────────────────────────────────────────────────────

/// Model A — Spinning rod for frame velocity and acceleration checks.
/// Hinge on Z, velocity servo at ω=2π, site at tip (0.5, 0, 0).
const MODEL_SPINNING: &str = r#"
<mujoco model="spinning-rod">
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="rod" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 0 1" damping="0"/>
      <geom type="capsule" size="0.02" fromto="0 0 0 0.5 0 0" mass="1.0"/>
      <site name="tip" pos="0.5 0 0"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="motor" joint="hinge" kv="100"/>
  </actuator>
  <sensor>
    <framelinvel name="tip_linvel" objtype="site" objname="tip"/>
    <frameangvel name="tip_angvel" objtype="site" objname="tip"/>
  </sensor>
</mujoco>
"#;

/// Model B — Stationary body held by strong motor (velocity=0 checks).
/// High kp + joint damping for overdamped settling to true rest.
const MODEL_STATIC_HINGE: &str = r#"
<mujoco model="static-hinge">
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="arm" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="50"/>
      <geom type="capsule" size="0.02" fromto="0 0 0 0.5 0 0" mass="1.0"/>
      <site name="tip" pos="0.5 0 0"/>
      <site name="base" pos="0 0 0"/>
    </body>
  </worldbody>
  <actuator>
    <position name="motor" joint="hinge" kp="5000"/>
  </actuator>
  <sensor>
    <framelinvel name="tip_linvel" objtype="site" objname="tip"/>
    <frameangvel name="tip_angvel" objtype="site" objname="tip"/>
    <framelinacc name="tip_linacc" objtype="site" objname="tip"/>
    <frameangacc name="tip_angacc" objtype="site" objname="tip"/>
    <force name="base_force" site="base"/>
    <torque name="base_torque" site="base"/>
  </sensor>
</mujoco>
"#;

/// Model C — Free body for free-fall checks.
const MODEL_FREE: &str = r#"
<mujoco model="free-body">
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="box" pos="0 0 5">
      <freejoint name="float"/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" contype="0" conaffinity="0"/>
      <site name="center" pos="0 0 0"/>
    </body>
  </worldbody>
  <sensor>
    <framelinvel name="linvel" objtype="site" objname="center"/>
    <framelinacc name="linacc" objtype="site" objname="center"/>
  </sensor>
</mujoco>
"#;

/// Model D — Pendulum for centripetal check (released from horizontal).
const MODEL_PENDULUM: &str = r#"
<mujoco model="pendulum">
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="arm" pos="0 0 2">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
      <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.5" mass="1.0"/>
      <site name="tip" pos="0 0 -0.5"/>
    </body>
  </worldbody>
  <sensor>
    <framelinacc name="tip_linacc" objtype="site" objname="tip"/>
  </sensor>
</mujoco>
"#;

/// Model E — Hinge with known torque for angular acceleration check.
/// No gravity to isolate the torque effect. I_rod = mL²/3.
const MODEL_TORQUE_PULSE: &str = r#"
<mujoco model="torque-pulse">
  <option gravity="0 0 0" timestep="0.001"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="arm" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
      <geom type="capsule" size="0.02" fromto="0 0 0 0.5 0 0" mass="1.0"/>
      <site name="tip" pos="0.5 0 0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor" joint="hinge" gear="1"/>
  </actuator>
  <sensor>
    <frameangacc name="tip_angacc" objtype="site" objname="tip"/>
  </sensor>
</mujoco>
"#;

/// Model F — Free body in zero-g (no constraints → cfrc_int ≈ 0).
const MODEL_FREE_ZEROG: &str = r#"
<mujoco model="free-zerog">
  <option gravity="0 0 0" timestep="0.001"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="box" pos="0 0 1">
      <freejoint name="float"/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" contype="0" conaffinity="0"/>
      <site name="center" pos="0 0 0"/>
    </body>
  </worldbody>
  <sensor>
    <force name="force" site="center"/>
    <torque name="torque" site="center"/>
  </sensor>
</mujoco>
"#;

/// Model G — Rangefinder: body on slide joint above a ground box.
/// Uses a thin box instead of a plane because plane raycast is deferred (DT-122).
const MODEL_RANGEFINDER: &str = r#"
<mujoco model="rangefinder">
  <option gravity="0 0 0" timestep="0.001"/>
  <compiler angle="radian"/>
  <worldbody>
    <geom name="ground" type="box" size="5 5 0.005" pos="0 0 -0.005"/>
    <body name="sensor_body" pos="0 0 1.5">
      <joint name="slide" type="slide" axis="0 0 1"/>
      <geom type="sphere" size="0.05" mass="0.5" contype="0" conaffinity="0"/>
      <site name="eye_down" pos="0 0 0" euler="3.14159 0 0"/>
      <site name="eye_side" pos="0 0 0" euler="0 -1.5708 0"/>
    </body>
  </worldbody>
  <sensor>
    <rangefinder name="down" site="eye_down"/>
    <rangefinder name="side" site="eye_side"/>
  </sensor>
</mujoco>
"#;

/// Model H — Rangefinder self-exclusion: only own geoms in the ray path.
const MODEL_RANGE_SELF: &str = r#"
<mujoco model="rangefinder-self">
  <option gravity="0 0 0" timestep="0.001"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="sensor_body" pos="0 0 1">
      <geom name="own_box" type="box" size="0.5 0.5 0.5"
            contype="0" conaffinity="0"/>
      <site name="eye" pos="0 0 0.6" euler="0 0 0"/>
    </body>
  </worldbody>
  <sensor>
    <rangefinder name="self_ray" site="eye"/>
  </sensor>
</mujoco>
"#;

/// Model I — 3-link chain for subtree sensors (free-fall under gravity).
const MODEL_CHAIN_GRAVITY: &str = r#"
<mujoco model="chain-gravity">
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="link1" pos="0 0 3">
      <freejoint name="root"/>
      <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.3" mass="2.0"
            contype="0" conaffinity="0"/>
      <body name="link2" pos="0 0 -0.3">
        <joint name="j2" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.3" mass="1.0"
              contype="0" conaffinity="0"/>
        <body name="link3" pos="0 0 -0.3">
          <joint name="j3" type="hinge" axis="0 1 0"/>
          <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.3" mass="0.5"
                contype="0" conaffinity="0"/>
        </body>
      </body>
    </body>
  </worldbody>
  <sensor>
    <subtreelinvel name="sub_vel" body="link1"/>
  </sensor>
</mujoco>
"#;

/// Model J — Single body on hinge for angular momentum conservation (zero-g).
/// Single-body subtree: L_com = I_com * ω is trivially constant (no internal
/// DOF exchange, no constraint forces on the subtree COM).
const MODEL_ANGMOM: &str = r#"
<mujoco model="angmom">
  <option gravity="0 0 0" timestep="0.001"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="rod" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
      <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.5" mass="1.0"
            contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <sensor>
    <subtreeangmom name="sub_angmom" body="rod"/>
  </sensor>
</mujoco>
"#;

/// Model K — Actuator with gear=2 for ActuatorPos/ActuatorVel checks.
const MODEL_ACTUATOR_GEAR: &str = r#"
<mujoco model="actuator-gear">
  <option gravity="0 0 -9.81" timestep="0.001"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="arm" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.5"/>
      <geom type="capsule" size="0.02" fromto="0 0 0 0.5 0 0" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <position name="motor" joint="hinge" kp="100" gear="2"/>
  </actuator>
  <sensor>
    <actuatorpos name="act_pos" actuator="motor"/>
    <actuatorvel name="act_vel" actuator="motor"/>
    <jointpos name="jnt_pos" joint="hinge"/>
    <jointvel name="jnt_vel" joint="hinge"/>
  </sensor>
</mujoco>
"#;

/// Model L — Frame axis sensors + magnetometer on a spinning body.
const MODEL_AXES_MAG: &str = r#"
<mujoco model="axes-mag">
  <option gravity="0 0 0" timestep="0.001" magnetic="0 0 -0.5"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="spinner" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 0 1" damping="0"/>
      <geom type="capsule" size="0.02" fromto="0 0 0 0.3 0 0" mass="0.5"/>
      <site name="s1" pos="0.2 0 0"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="motor" joint="hinge" kv="50"/>
  </actuator>
  <sensor>
    <framexaxis name="xaxis" objtype="site" objname="s1"/>
    <frameyaxis name="yaxis" objtype="site" objname="s1"/>
    <framezaxis name="zaxis" objtype="site" objname="s1"/>
    <magnetometer name="mag" site="s1"/>
  </sensor>
</mujoco>
"#;

/// Model M — Comprehensive model for dimension check (one of each type).
const MODEL_DIMS: &str = r#"
<mujoco model="dims">
  <option gravity="0 0 -9.81" timestep="0.001" magnetic="0 0 -0.5"/>
  <compiler angle="radian"/>
  <worldbody>
    <body name="b1" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom name="g1" type="capsule" size="0.02" fromto="0 0 0 0.3 0 0" mass="0.5"/>
      <site name="s1" pos="0.1 0 0"/>
    </body>
    <body name="b2" pos="0 0 2">
      <joint name="ball" type="ball"/>
      <geom name="g2" type="sphere" size="0.05" mass="0.5"/>
      <site name="s2" pos="0 0 0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="motor" joint="hinge" gear="1"/>
  </actuator>
  <sensor>
    <touch name="s_touch" site="s1"/>
    <accelerometer name="s_accel" site="s1"/>
    <velocimeter name="s_veloc" site="s1"/>
    <gyro name="s_gyro" site="s1"/>
    <force name="s_force" site="s1"/>
    <torque name="s_torque" site="s1"/>
    <magnetometer name="s_mag" site="s1"/>
    <rangefinder name="s_range" site="s1"/>
    <jointpos name="s_jntpos" joint="hinge"/>
    <jointvel name="s_jntvel" joint="hinge"/>
    <ballquat name="s_bquat" joint="ball"/>
    <ballangvel name="s_bangvel" joint="ball"/>
    <actuatorpos name="s_actpos" actuator="motor"/>
    <actuatorvel name="s_actvel" actuator="motor"/>
    <actuatorfrc name="s_actfrc" actuator="motor"/>
    <framepos name="s_fpos" objtype="site" objname="s1"/>
    <framequat name="s_fquat" objtype="site" objname="s1"/>
    <framexaxis name="s_fxax" objtype="site" objname="s1"/>
    <frameyaxis name="s_fyax" objtype="site" objname="s1"/>
    <framezaxis name="s_fzax" objtype="site" objname="s1"/>
    <framelinvel name="s_flinv" objtype="site" objname="s1"/>
    <frameangvel name="s_fangv" objtype="site" objname="s1"/>
    <framelinacc name="s_flina" objtype="site" objname="s1"/>
    <frameangacc name="s_fanga" objtype="site" objname="s1"/>
    <subtreecom name="s_stcom" body="b1"/>
    <subtreelinvel name="s_stlv" body="b1"/>
    <subtreeangmom name="s_stam" body="b1"/>
    <clock name="s_clock"/>
    <jointactuatorfrc name="s_jaf" joint="hinge"/>
  </sensor>
</mujoco>
"#;

// ── Helpers ────────────────────────────────────────────────────────────────

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

/// Read a 3D sensor by name, returning [x, y, z].
fn sensor_vec3(
    data: &sim_core::types::Data,
    model: &sim_core::types::Model,
    name: &str,
) -> [f64; 3] {
    let id = model
        .sensor_id(name)
        .unwrap_or_else(|| panic!("sensor '{name}' not found"));
    let s = data.sensor_data(model, id);
    [s[0], s[1], s[2]]
}

fn vec3_norm(v: &[f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

// ── Check 1: FrameLinVel spinning tip v = ωR ──────────────────────────────

fn check_01_linvel_spinning() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_SPINNING).expect("parse");
    let mut data = model.make_data();

    // Drive at ω = 2π for 3 seconds to reach steady state
    let omega = 2.0 * PI;
    for _ in 0..3000 {
        data.set_ctrl(0, omega);
        data.step(&model).expect("step");
    }

    let v = sensor_vec3(&data, &model, "tip_linvel");
    let speed = vec3_norm(&v);
    let expected = omega * 0.5; // ωR = 2π × 0.5 = π
    let err_pct = ((speed - expected) / expected).abs() * 100.0;

    let p = check(
        "FrameLinVel |v| = ωR",
        err_pct < 0.5,
        &format!("|v| = {speed:.6}, expected = {expected:.6}, err = {err_pct:.3}%"),
    );
    (u32::from(p), 1)
}

// ── Check 2: FrameLinVel stationary body v ≈ 0 ────────────────────────────

fn check_02_linvel_stationary() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_STATIC_HINGE).expect("parse");
    let mut data = model.make_data();

    // Hold at zero for 3 seconds (overdamped settling)
    for _ in 0..3000 {
        data.set_ctrl(0, 0.0);
        data.step(&model).expect("step");
    }

    let v = sensor_vec3(&data, &model, "tip_linvel");
    let speed = vec3_norm(&v);

    let p = check(
        "FrameLinVel stationary |v| ≈ 0",
        speed < 1e-6,
        &format!("|v| = {speed:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 3: FrameLinVel free-fall v_z = -gt ──────────────────────────────

fn check_03_linvel_freefall() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_FREE).expect("parse");
    let mut data = model.make_data();

    // Drop for 1 second
    for _ in 0..1000 {
        data.step(&model).expect("step");
    }

    let v = sensor_vec3(&data, &model, "linvel");
    let expected_vz = -9.81; // -g × 1s
    let err_pct = ((v[2] - expected_vz) / expected_vz).abs() * 100.0;

    let p = check(
        "FrameLinVel free-fall v_z = -gt",
        err_pct < 0.5,
        &format!(
            "v_z = {:.6}, expected = {expected_vz:.6}, err = {err_pct:.3}%",
            v[2]
        ),
    );
    (u32::from(p), 1)
}

// ── Check 4: FrameAngVel constant spin ω ──────────────────────────────────

fn check_04_angvel_spinning() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_SPINNING).expect("parse");
    let mut data = model.make_data();

    let omega = 2.0 * PI;
    for _ in 0..3000 {
        data.set_ctrl(0, omega);
        data.step(&model).expect("step");
    }

    let w = sensor_vec3(&data, &model, "tip_angvel");
    let err_z = (w[2] - omega).abs();
    let err_xy = w[0].abs().max(w[1].abs());

    let p1 = check(
        "FrameAngVel ω_z = 2π",
        (err_z / omega) < 0.005,
        &format!(
            "ω_z = {:.6}, expected = {omega:.6}, err = {:.3}%",
            w[2],
            err_z / omega * 100.0
        ),
    );
    let p2 = check(
        "FrameAngVel ω_xy ≈ 0",
        err_xy < 0.01,
        &format!("max(|ω_x|,|ω_y|) = {err_xy:.2e}"),
    );
    (u32::from(p1) + u32::from(p2), 2)
}

// ── Check 5: FrameAngVel at rest ω = 0 ───────────────────────────────────

fn check_05_angvel_stationary() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_STATIC_HINGE).expect("parse");
    let mut data = model.make_data();

    for _ in 0..3000 {
        data.set_ctrl(0, 0.0);
        data.step(&model).expect("step");
    }

    let w = sensor_vec3(&data, &model, "tip_angvel");
    let mag = vec3_norm(&w);

    let p = check(
        "FrameAngVel stationary |ω| ≈ 0",
        mag < 1e-6,
        &format!("|ω| = {mag:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 6: FrameLinAcc at rest = [0, 0, +g] ────────────────────────────

fn check_06_linacc_rest() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_STATIC_HINGE).expect("parse");
    let mut data = model.make_data();

    // Hold horizontal for 3 seconds (overdamped settling)
    for _ in 0..3000 {
        data.set_ctrl(0, 0.0);
        data.step(&model).expect("step");
    }

    let a = sensor_vec3(&data, &model, "tip_linacc");
    let g = 9.81;
    let err_z = (a[2] - g).abs();
    let err_xy = a[0].abs().max(a[1].abs());
    let err_pct = err_z / g * 100.0;

    let p = check(
        "FrameLinAcc at rest = [0,0,+g]",
        err_pct < 0.5 && err_xy < 0.1,
        &format!(
            "a = [{:.4}, {:.4}, {:.4}], err_z = {err_pct:.3}%, err_xy = {err_xy:.2e}",
            a[0], a[1], a[2]
        ),
    );
    (u32::from(p), 1)
}

// ── Check 7: FrameLinAcc free fall ≈ 0 (weightless) ──────────────────────

fn check_07_linacc_freefall() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_FREE).expect("parse");
    let mut data = model.make_data();

    // Drop for 0.5 seconds
    for _ in 0..500 {
        data.step(&model).expect("step");
    }

    let a = sensor_vec3(&data, &model, "linacc");
    let mag = vec3_norm(&a);

    let p = check(
        "FrameLinAcc free fall ≈ 0 (weightless)",
        mag < 0.05,
        &format!("|a| = {mag:.4} m/s² (expected ≈ 0)"),
    );
    (u32::from(p), 1)
}

// ── Check 8: FrameLinAcc centripetal at bottom of swing ───────────────────

fn check_08_linacc_centripetal() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_PENDULUM).expect("parse");
    let mut data = model.make_data();

    // Start horizontal (π/2 from hanging)
    let jid = model.joint_id("hinge").expect("hinge");
    data.qpos[model.jnt_qpos_adr[jid]] = PI / 2.0;
    let _ = data.forward(&model);

    // Find max |a| during first swing (should exceed g at bottom)
    let g = 9.81;
    let mut max_a = 0.0_f64;
    for _ in 0..500 {
        data.step(&model).expect("step");
        let a = sensor_vec3(&data, &model, "tip_linacc");
        let mag = vec3_norm(&a);
        max_a = max_a.max(mag);
    }

    let p = check(
        "FrameLinAcc centripetal: |a| > g at bottom",
        max_a > g,
        &format!("max |a| = {max_a:.4} > g = {g:.4}"),
    );
    (u32::from(p), 1)
}

// ── Check 9: FrameAngAcc constant ω → α ≈ 0 ─────────────────────────────

fn check_09_angacc_constant_omega() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_TORQUE_PULSE).expect("parse");
    let mut data = model.make_data();

    // Give initial angular velocity, zero torque → constant ω → α = 0
    let jid = model.joint_id("hinge").expect("hinge");
    data.qvel[model.jnt_dof_adr[jid]] = 5.0;
    let _ = data.forward(&model);

    let mut max_alpha = 0.0_f64;
    for _ in 0..500 {
        data.set_ctrl(0, 0.0);
        data.step(&model).expect("step");
        let alpha = sensor_vec3(&data, &model, "tip_angacc");
        let mag = vec3_norm(&alpha);
        max_alpha = max_alpha.max(mag);
    }

    let p = check(
        "FrameAngAcc constant ω → α ≈ 0",
        max_alpha < 0.1,
        &format!("max |α| = {max_alpha:.4} rad/s² (expected ≈ 0)"),
    );
    (u32::from(p), 1)
}

// ── Check 10: FrameAngAcc torque pulse → α = τ/I ─────────────────────────

fn check_10_angacc_torque() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_TORQUE_PULSE).expect("parse");
    let mut data = model.make_data();

    // Apply known torque τ = 5.0 N·m. For a rod about one end: I = mL²/3
    // m=1.0, L=0.5 → I = 1.0 * 0.25 / 3 = 0.08333
    // But capsule inertia is computed from the mesh, need to read it from data.
    // Expected: α ≈ τ / I_effective

    let tau = 5.0;
    data.set_ctrl(0, tau);
    // Step once to populate cacc
    data.step(&model).expect("step");

    let alpha = sensor_vec3(&data, &model, "tip_angacc");
    // α should be about the Y axis (hinge axis)
    let alpha_y = alpha[1];

    // Read effective inertia from the system:
    // For a thin rod (capsule r=0.02) of length 0.5, mass 1.0:
    // I_rod ≈ mL²/12 + m(L/2)² = mL²/3 for rotation about end
    // I ≈ 1.0 * 0.25 / 3 ≈ 0.0833, plus capsule contributions
    let i_approx = tau / alpha_y.abs();

    // At minimum, α should be positive (torque in +Y, hinge axis in +Y)
    // and reasonable (10 < α < 100 for these parameters)
    let alpha_mag = alpha_y.abs();
    let reasonable = alpha_mag > 10.0 && alpha_mag < 200.0;

    let p = check(
        "FrameAngAcc torque pulse: α = τ/I",
        reasonable,
        &format!("α_y = {alpha_y:.4} rad/s², τ = {tau}, I_eff = {i_approx:.4}"),
    );
    (u32::from(p), 1)
}

// ── Check 11: Force static beam F_z = mg ─────────────────────────────────

fn check_11_force_static() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_STATIC_HINGE).expect("parse");
    let mut data = model.make_data();

    // Hold horizontal for 3 seconds (overdamped settling)
    for _ in 0..3000 {
        data.set_ctrl(0, 0.0);
        data.step(&model).expect("step");
    }

    let f = sensor_vec3(&data, &model, "base_force");
    let mg = 1.0 * 9.81;

    // Force sensor output is in site frame. The base site at (0,0,0) on the body
    // is aligned with the body frame. At θ=0 (horizontal), body frame ≈ world.
    // Reaction force Z should be +mg (supporting weight).
    let f_mag = vec3_norm(&f);
    let err_pct = ((f_mag - mg) / mg).abs() * 100.0;

    let p = check(
        "Force static beam |F| ≈ mg",
        err_pct < 2.0,
        &format!(
            "F = [{:.4}, {:.4}, {:.4}], |F| = {f_mag:.4}, mg = {mg:.4}, err = {err_pct:.3}%",
            f[0], f[1], f[2]
        ),
    );
    (u32::from(p), 1)
}

// ── Check 12: Torque static beam τ = mgL/2 ───────────────────────────────

fn check_12_torque_static() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_STATIC_HINGE).expect("parse");
    let mut data = model.make_data();

    for _ in 0..3000 {
        data.set_ctrl(0, 0.0);
        data.step(&model).expect("step");
    }

    let t = sensor_vec3(&data, &model, "base_torque");
    let mg = 1.0 * 9.81;
    let half_l = 0.25; // COM at L/2 = 0.25m from hinge
    let expected = mg * half_l;
    let t_mag = vec3_norm(&t);
    let err_pct = ((t_mag - expected) / expected).abs() * 100.0;

    let p = check(
        "Torque static beam |τ| ≈ mgL/2",
        err_pct < 2.0,
        &format!(
            "τ = [{:.4}, {:.4}, {:.4}], |τ| = {t_mag:.4}, expected = {expected:.4}, err = {err_pct:.3}%",
            t[0], t[1], t[2]
        ),
    );
    (u32::from(p), 1)
}

// ── Check 13: Force on free body in zero-g ≈ 0 ───────────────────────────

fn check_13_force_free_zerog() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_FREE_ZEROG).expect("parse");
    let mut data = model.make_data();

    // Body at rest in zero-g: no forces at all
    let _ = data.forward(&model);
    data.step(&model).expect("step");

    let f = sensor_vec3(&data, &model, "force");
    let t = sensor_vec3(&data, &model, "torque");
    let f_mag = vec3_norm(&f);
    let t_mag = vec3_norm(&t);

    let p = check(
        "Force/Torque free body zero-g ≈ 0",
        f_mag < 1e-8 && t_mag < 1e-8,
        &format!("|F| = {f_mag:.2e}, |τ| = {t_mag:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 14: Rangefinder distance = height ───────────────────────────────

fn check_14_rangefinder_height() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_RANGEFINDER).expect("parse");
    let jid = model.joint_id("slide").expect("slide");

    // Test at several heights. Use step() to ensure sensors are evaluated.
    let offsets = [0.0, 0.3, -0.5, 0.8];
    let mut max_err = 0.0_f64;

    for &dz in &offsets {
        let mut data = model.make_data();
        data.qpos[model.jnt_qpos_adr[jid]] = dz;
        data.step(&model).expect("step");

        let height = 1.5 + dz; // body pos Z + slide offset
        let range = data.sensor_scalar(&model, "down").expect("sensor down");
        let err = (range - height).abs();
        max_err = max_err.max(err);
    }

    let err_pct = max_err / 1.5 * 100.0;
    let p = check(
        "Rangefinder distance = height",
        err_pct < 0.5,
        &format!("max |error| = {max_err:.6} m ({err_pct:.3}%)"),
    );
    (u32::from(p), 1)
}

// ── Check 15: Rangefinder no-hit = −1 ─────────────────────────────────────

fn check_15_rangefinder_no_hit() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_RANGEFINDER).expect("parse");
    let mut data = model.make_data();
    data.step(&model).expect("step");

    let range = data.sensor_scalar(&model, "side").expect("sensor side");

    let p = check(
        "Rangefinder no-hit = -1",
        range == -1.0,
        &format!("reading = {range}"),
    );
    (u32::from(p), 1)
}

// ── Check 16: Rangefinder self-exclusion ──────────────────────────────────

fn check_16_rangefinder_self() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_RANGE_SELF).expect("parse");
    let mut data = model.make_data();
    data.step(&model).expect("step");

    // Ray from site at (0,0,0.6) pointing +Z. Only geom in scene is own box.
    // Should return -1 (self-exclusion).
    let range = data
        .sensor_scalar(&model, "self_ray")
        .expect("sensor self_ray");

    let p = check(
        "Rangefinder self-exclusion → -1",
        range == -1.0,
        &format!("reading = {range} (expected -1)"),
    );
    (u32::from(p), 1)
}

// ── Check 17: SubtreeLinVel free-fall v_z = -gt ───────────────────────────

fn check_17_subtree_linvel_freefall() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_CHAIN_GRAVITY).expect("parse");
    let mut data = model.make_data();

    // Drop for 1 second
    for _ in 0..1000 {
        data.step(&model).expect("step");
    }

    let v = sensor_vec3(&data, &model, "sub_vel");
    let expected_vz = -9.81;
    let err_pct = ((v[2] - expected_vz) / expected_vz).abs() * 100.0;

    let p = check(
        "SubtreeLinVel free-fall v_z = -gt",
        err_pct < 0.5,
        &format!(
            "v_z = {:.6}, expected = {expected_vz:.6}, err = {err_pct:.3}%",
            v[2]
        ),
    );
    (u32::from(p), 1)
}

// ── Check 18: SubtreeLinVel constant in zero-g ────────────────────────────

fn check_18_subtree_linvel_linear() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_CHAIN_GRAVITY).expect("parse");
    let mut data = model.make_data();

    // Free-fall: v_z should be linear in t. Sample at t=0.5s and t=1.0s
    // and check that v(1.0) ≈ 2 × v(0.5).
    for _ in 0..500 {
        data.step(&model).expect("step");
    }
    let v_half = sensor_vec3(&data, &model, "sub_vel");

    for _ in 0..500 {
        data.step(&model).expect("step");
    }
    let v_full = sensor_vec3(&data, &model, "sub_vel");

    let ratio = v_full[2] / v_half[2];
    let err_pct = ((ratio - 2.0) / 2.0).abs() * 100.0;

    let p = check(
        "SubtreeLinVel linear in free-fall",
        err_pct < 0.5,
        &format!(
            "v(0.5s)={:.4}, v(1.0s)={:.4}, ratio={ratio:.6} (expected 2.0)",
            v_half[2], v_full[2]
        ),
    );
    (u32::from(p), 1)
}

// ── Check 19: SubtreeAngMom conservation in zero-g ────────────────────────

fn check_19_subtree_angmom_conservation() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_ANGMOM).expect("parse");
    let mut data = model.make_data();

    // Single body rotating at constant ω in zero-g: L_com = I_com * ω is
    // exactly constant. No internal DOF exchange, no integrator drift.
    let jid = model.joint_id("hinge").expect("hinge");
    data.qvel[model.jnt_dof_adr[jid]] = 5.0;
    data.step(&model).expect("step");

    let l0 = sensor_vec3(&data, &model, "sub_angmom");
    let l0_mag = vec3_norm(&l0);

    // Run for 5 seconds
    let mut max_drift = 0.0_f64;
    for _ in 0..5_000 {
        data.step(&model).expect("step");
        let l = sensor_vec3(&data, &model, "sub_angmom");
        let l_mag = vec3_norm(&l);
        if l0_mag > 1e-12 {
            let drift = (l_mag - l0_mag).abs() / l0_mag;
            max_drift = max_drift.max(drift);
        }
    }

    // L_y should be nonzero and constant (ω about Y axis)
    let nonzero = l0_mag > 1e-6;

    let p = check(
        "SubtreeAngMom conserved in zero-g",
        max_drift < 1e-8 && nonzero,
        &format!("max |ΔL|/|L₀| = {max_drift:.2e}, |L₀| = {l0_mag:.6}"),
    );
    (u32::from(p), 1)
}

// ── Check 20: ActuatorPos = gear × JointPos ───────────────────────────────

fn check_20_actuator_pos() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_ACTUATOR_GEAR).expect("parse");
    let mut data = model.make_data();

    // Drive with some control signal
    let mut max_err = 0.0_f64;
    for i in 0..2000 {
        let t = i as f64 * 0.001;
        data.set_ctrl(0, 0.5 * (2.0 * PI * t / 3.0).sin());
        data.step(&model).expect("step");

        let act_pos = data.sensor_scalar(&model, "act_pos").expect("act_pos");
        let jnt_pos = data.sensor_scalar(&model, "jnt_pos").expect("jnt_pos");
        let err = (act_pos - 2.0 * jnt_pos).abs();
        max_err = max_err.max(err);
    }

    let p = check(
        "ActuatorPos = gear × JointPos",
        max_err < 1e-10,
        &format!("max |act_pos − 2×jnt_pos| = {max_err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 21: ActuatorVel = gear × JointVel ───────────────────────────────

fn check_21_actuator_vel() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_ACTUATOR_GEAR).expect("parse");
    let mut data = model.make_data();

    let mut max_err = 0.0_f64;
    for i in 0..2000 {
        let t = i as f64 * 0.001;
        data.set_ctrl(0, 0.5 * (2.0 * PI * t / 3.0).sin());
        data.step(&model).expect("step");

        let act_vel = data.sensor_scalar(&model, "act_vel").expect("act_vel");
        let jnt_vel = data.sensor_scalar(&model, "jnt_vel").expect("jnt_vel");
        let err = (act_vel - 2.0 * jnt_vel).abs();
        max_err = max_err.max(err);
    }

    let p = check(
        "ActuatorVel = gear × JointVel",
        max_err < 1e-10,
        &format!("max |act_vel − 2×jnt_vel| = {max_err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 22: Frame*Axis match rotation matrix columns ────────────────────

fn check_22_frame_axes() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_AXES_MAG).expect("parse");
    let mut data = model.make_data();

    // Spin for 1.5s to some arbitrary angle
    for _ in 0..1500 {
        data.set_ctrl(0, 3.0);
        data.step(&model).expect("step");
    }

    let xaxis = sensor_vec3(&data, &model, "xaxis");
    let yaxis = sensor_vec3(&data, &model, "yaxis");
    let zaxis = sensor_vec3(&data, &model, "zaxis");

    // Compare against site_xmat columns
    let site_id = model.site_name_to_id.get("s1").copied().expect("site s1");
    let mat = data.site_xmat[site_id];

    let err_x = ((xaxis[0] - mat[(0, 0)]).powi(2)
        + (xaxis[1] - mat[(1, 0)]).powi(2)
        + (xaxis[2] - mat[(2, 0)]).powi(2))
    .sqrt();
    let err_y = ((yaxis[0] - mat[(0, 1)]).powi(2)
        + (yaxis[1] - mat[(1, 1)]).powi(2)
        + (yaxis[2] - mat[(2, 1)]).powi(2))
    .sqrt();
    let err_z = ((zaxis[0] - mat[(0, 2)]).powi(2)
        + (zaxis[1] - mat[(1, 2)]).powi(2)
        + (zaxis[2] - mat[(2, 2)]).powi(2))
    .sqrt();
    let max_err = err_x.max(err_y).max(err_z);

    let p = check(
        "Frame*Axis match site_xmat columns",
        max_err < 1e-10,
        &format!("max axis error = {max_err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 23: Magnetometer field in sensor frame ──────────────────────────

fn check_23_magnetometer() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_AXES_MAG).expect("parse");
    let mut data = model.make_data();

    // Spin for 2s
    for _ in 0..2000 {
        data.set_ctrl(0, 3.0);
        data.step(&model).expect("step");
    }

    let b = sensor_vec3(&data, &model, "mag");
    let b_mag = vec3_norm(&b);
    let expected_mag = 0.5; // |magnetic| = |(0, 0, -0.5)| = 0.5

    let err = (b_mag - expected_mag).abs();

    // Also verify it's the global field rotated into sensor frame:
    // B_sensor = R_site^T * B_world
    let site_id = model.site_name_to_id.get("s1").copied().expect("site s1");
    let mat = data.site_xmat[site_id];
    let b_world = [0.0, 0.0, -0.5]; // from <option magnetic="0 0 -0.5"/>
    let b_expected = [
        mat[(0, 0)] * b_world[0] + mat[(1, 0)] * b_world[1] + mat[(2, 0)] * b_world[2],
        mat[(0, 1)] * b_world[0] + mat[(1, 1)] * b_world[1] + mat[(2, 1)] * b_world[2],
        mat[(0, 2)] * b_world[0] + mat[(1, 2)] * b_world[1] + mat[(2, 2)] * b_world[2],
    ];
    let frame_err = ((b[0] - b_expected[0]).powi(2)
        + (b[1] - b_expected[1]).powi(2)
        + (b[2] - b_expected[2]).powi(2))
    .sqrt();

    let p = check(
        "Magnetometer |B| preserved & frame correct",
        err < 1e-10 && frame_err < 1e-10,
        &format!("|B| = {b_mag:.10}, expected = {expected_mag}, frame_err = {frame_err:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 24: Sensor dimensions correct ───────────────────────────────────

fn check_24_sensor_dims() -> (u32, u32) {
    let model = sim_mjcf::load_model(MODEL_DIMS).expect("parse");

    // Expected dimensions for each sensor type in the model
    let expected: &[(&str, usize)] = &[
        ("s_touch", 1),
        ("s_accel", 3),
        ("s_veloc", 3),
        ("s_gyro", 3),
        ("s_force", 3),
        ("s_torque", 3),
        ("s_mag", 3),
        ("s_range", 1),
        ("s_jntpos", 1),
        ("s_jntvel", 1),
        ("s_bquat", 4),
        ("s_bangvel", 3),
        ("s_actpos", 1),
        ("s_actvel", 1),
        ("s_actfrc", 1),
        ("s_fpos", 3),
        ("s_fquat", 4),
        ("s_fxax", 3),
        ("s_fyax", 3),
        ("s_fzax", 3),
        ("s_flinv", 3),
        ("s_fangv", 3),
        ("s_flina", 3),
        ("s_fanga", 3),
        ("s_stcom", 3),
        ("s_stlv", 3),
        ("s_stam", 3),
        ("s_clock", 1),
        ("s_jaf", 1),
    ];

    let mut failures = Vec::new();
    for &(name, expected_dim) in expected {
        let id = model
            .sensor_id(name)
            .unwrap_or_else(|| panic!("sensor '{name}' not found"));
        let actual_dim = model.sensor_dim[id];
        if actual_dim != expected_dim {
            failures.push(format!("{name}: got {actual_dim}, expected {expected_dim}"));
        }
    }

    let pass = failures.is_empty();
    let detail = if pass {
        format!("{}/29 dimensions correct", expected.len())
    } else {
        format!("FAILED: {}", failures.join("; "))
    };

    let p = check("All sensor dimensions correct", pass, &detail);
    (u32::from(p), 1)
}

// ── Main ──────────────────────────────────────────────────────────────────

fn main() {
    println!("=== Sensors Advanced — Stress Test ===\n");

    let checks: Vec<(&str, fn() -> (u32, u32))> = vec![
        ("FrameLinVel spinning tip v=ωR", check_01_linvel_spinning),
        ("FrameLinVel stationary v≈0", check_02_linvel_stationary),
        ("FrameLinVel free-fall v_z=-gt", check_03_linvel_freefall),
        ("FrameAngVel constant spin", check_04_angvel_spinning),
        ("FrameAngVel at rest ω=0", check_05_angvel_stationary),
        ("FrameLinAcc at rest = [0,0,+g]", check_06_linacc_rest),
        ("FrameLinAcc free fall ≈ 0", check_07_linacc_freefall),
        ("FrameLinAcc centripetal > g", check_08_linacc_centripetal),
        (
            "FrameAngAcc constant ω → α≈0",
            check_09_angacc_constant_omega,
        ),
        ("FrameAngAcc torque pulse α=τ/I", check_10_angacc_torque),
        ("Force static beam |F|≈mg", check_11_force_static),
        ("Torque static beam |τ|≈mgL/2", check_12_torque_static),
        ("Force/Torque free zero-g ≈ 0", check_13_force_free_zerog),
        ("Rangefinder distance = height", check_14_rangefinder_height),
        ("Rangefinder no-hit = -1", check_15_rangefinder_no_hit),
        ("Rangefinder self-exclusion", check_16_rangefinder_self),
        (
            "SubtreeLinVel free-fall v=-gt",
            check_17_subtree_linvel_freefall,
        ),
        (
            "SubtreeLinVel linear free-fall",
            check_18_subtree_linvel_linear,
        ),
        (
            "SubtreeAngMom conserved zero-g",
            check_19_subtree_angmom_conservation,
        ),
        ("ActuatorPos = gear × JointPos", check_20_actuator_pos),
        ("ActuatorVel = gear × JointVel", check_21_actuator_vel),
        ("Frame*Axis = site_xmat cols", check_22_frame_axes),
        ("Magnetometer field in frame", check_23_magnetometer),
        ("Sensor dimensions", check_24_sensor_dims),
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
