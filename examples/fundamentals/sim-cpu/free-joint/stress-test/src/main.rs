//! Stress test — headless validation of all free joint concepts.
//!
//! 12 checks covering: quaternion norm, linear/angular momentum conservation,
//! projectile trajectory, dimensions, no-limits, gravity acceleration, body
//! independence, exponential map integration, energy conservation, and
//! gravity-momentum coupling.
//!
//! Run: `cargo run -p example-free-joint-stress-test --release`

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

use nalgebra::{UnitQuaternion, Vector3};

// ── MJCF Models ────────────────────────────────────────────────────────────

/// Model A — Zero gravity, asymmetric inertia.
/// Used by checks 1, 2, 3, 10, 11.
const ZERO_G_MJCF: &str = r#"
<mujoco model="free-zero-g">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>
  <worldbody>
    <body name="box" pos="0 0 0">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="2.0" diaginertia="0.10 0.06 0.03"/>
      <geom type="box" size="0.15 0.1 0.08" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <sensor>
    <subtreeangmom name="angmom" body="box"/>
  </sensor>
</mujoco>
"#;

/// Model B — Gravity, spherical body.
/// Used by checks 4, 5, 8, 12.
const GRAVITY_MJCF: &str = r#"
<mujoco model="free-gravity">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>
  <worldbody>
    <body name="ball" pos="0 0 0">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.004 0.004 0.004"/>
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Model C — Two independent bodies, zero gravity.
/// Used by check 9.
const TWO_BODIES_MJCF: &str = r#"
<mujoco model="free-two-bodies">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>
  <worldbody>
    <body name="a" pos="0 0 0">
      <freejoint name="free_a"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
    </body>
    <body name="b" pos="2 0 0">
      <freejoint name="free_b"/>
      <inertial pos="0 0 0" mass="3.0" diaginertia="0.05 0.05 0.05"/>
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
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

fn quat_norm(qpos: &[f64], offset: usize) -> f64 {
    (qpos[offset].powi(2)
        + qpos[offset + 1].powi(2)
        + qpos[offset + 2].powi(2)
        + qpos[offset + 3].powi(2))
    .sqrt()
}

// ── Check 1: Quaternion norm preserved ─────────────────────────────────────

fn check_1_quaternion_norm() -> (u32, u32) {
    let model = sim_mjcf::load_model(ZERO_G_MJCF).expect("parse");
    let mut data = model.make_data();

    data.qvel[0] = 0.5;
    data.qvel[1] = 0.3;
    data.qvel[3] = 2.0;
    data.qvel[4] = 0.5;
    data.qvel[5] = 0.1;

    for _ in 0..10_000 {
        data.step(&model).expect("step");
    }

    let norm = quat_norm(data.qpos.as_slice(), 3);
    let drift = (norm - 1.0).abs();

    let p = check(
        "Quaternion norm preserved (10s)",
        drift < 1e-10,
        &format!("|quat| = {norm:.15}, drift = {drift:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 2: Linear momentum conserved (zero-g) ───────────────────────────

fn check_2_linear_momentum() -> (u32, u32) {
    let model = sim_mjcf::load_model(ZERO_G_MJCF).expect("parse");
    let mut data = model.make_data();

    data.qvel[0] = 0.5;
    data.qvel[1] = 0.3;
    data.qvel[2] = -0.2;

    let mass = model.body_mass[1];
    let p0 = [
        mass * data.qvel[0],
        mass * data.qvel[1],
        mass * data.qvel[2],
    ];

    for _ in 0..10_000 {
        data.step(&model).expect("step");
    }

    let p_final = [
        mass * data.qvel[0],
        mass * data.qvel[1],
        mass * data.qvel[2],
    ];
    let drift = ((p_final[0] - p0[0]).powi(2)
        + (p_final[1] - p0[1]).powi(2)
        + (p_final[2] - p0[2]).powi(2))
    .sqrt();

    let p = check(
        "Linear momentum conserved (zero-g, 10s)",
        drift < 1e-10,
        &format!("drift = {drift:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 3: Angular momentum conserved (zero-g) ──────────────────────────

fn check_3_angular_momentum() -> (u32, u32) {
    let model = sim_mjcf::load_model(ZERO_G_MJCF).expect("parse");
    let mut data = model.make_data();

    data.qvel[3] = 2.0;
    data.qvel[4] = 0.5;
    data.qvel[5] = 0.1;

    data.forward(&model).expect("forward");
    let sensor_adr = model.sensor_adr[0];
    let l0_mag = (data.sensordata[sensor_adr].powi(2)
        + data.sensordata[sensor_adr + 1].powi(2)
        + data.sensordata[sensor_adr + 2].powi(2))
    .sqrt();

    for _ in 0..10_000 {
        data.step(&model).expect("step");
    }
    data.forward(&model).expect("forward");

    let l_mag = (data.sensordata[sensor_adr].powi(2)
        + data.sensordata[sensor_adr + 1].powi(2)
        + data.sensordata[sensor_adr + 2].powi(2))
    .sqrt();
    let drift = (l_mag - l0_mag).abs();

    let p = check(
        "Angular momentum conserved (zero-g, 10s)",
        drift < 1e-8,
        &format!("|L_0| = {l0_mag:.10}, |L_f| = {l_mag:.10}, drift = {drift:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 4: Projectile apex height ────────────────────────────────────────

fn check_4_projectile_apex() -> (u32, u32) {
    let model = sim_mjcf::load_model(GRAVITY_MJCF).expect("parse");
    let mut data = model.make_data();

    let v0 = 5.0_f64;
    let theta = std::f64::consts::FRAC_PI_4;
    let g = 9.81;

    data.qvel[0] = v0 * theta.cos();
    data.qvel[2] = v0 * theta.sin();

    let apex_analytical = v0 * v0 * theta.sin().powi(2) / (2.0 * g);

    // Step until vz crosses zero (rising -> falling)
    let mut prev_vz = data.qvel[2];
    let mut z_apex = 0.0_f64;
    for _ in 0..10_000 {
        data.step(&model).expect("step");
        if prev_vz > 0.0 && data.qvel[2] <= 0.0 {
            z_apex = data.qpos[2];
            break;
        }
        prev_vz = data.qvel[2];
    }

    let err_pct = ((z_apex - apex_analytical) / apex_analytical).abs() * 100.0;

    let p = check(
        "Projectile apex height",
        err_pct < 0.1,
        &format!("z_apex = {z_apex:.6}, analytical = {apex_analytical:.6}, err = {err_pct:.4}%"),
    );
    (u32::from(p), 1)
}

// ── Check 5: Projectile range ──────────────────────────────────────────────

fn check_5_projectile_range() -> (u32, u32) {
    let model = sim_mjcf::load_model(GRAVITY_MJCF).expect("parse");
    let mut data = model.make_data();

    let v0 = 5.0_f64;
    let theta = std::f64::consts::FRAC_PI_4;
    let g = 9.81;

    data.qvel[0] = v0 * theta.cos();
    data.qvel[2] = v0 * theta.sin();

    let range_analytical = v0 * v0 * (2.0 * theta).sin() / g;

    // Step until z crosses back below 0 (launched from z=0)
    let mut passed_apex = false;
    let mut x_range = 0.0_f64;
    let mut prev_z = 0.0_f64;
    for i in 0..10_000 {
        data.step(&model).expect("step");
        if i > 10 && data.qvel[2] < 0.0 {
            passed_apex = true;
        }
        if passed_apex && prev_z >= 0.0 && data.qpos[2] < 0.0 {
            // Linear interpolation for more precise crossing
            let frac = prev_z / (prev_z - data.qpos[2]);
            x_range = data.qpos[0] - (1.0 - frac) * data.qvel[0] * model.timestep;
            break;
        }
        prev_z = data.qpos[2];
    }

    let err_pct = ((x_range - range_analytical) / range_analytical).abs() * 100.0;

    let p = check(
        "Projectile range",
        err_pct < 0.1,
        &format!("x_range = {x_range:.6}, analytical = {range_analytical:.6}, err = {err_pct:.4}%"),
    );
    (u32::from(p), 1)
}

// ── Check 6: Dimensions (nq=7, nv=6) ──────────────────────────────────────

fn check_6_dimensions() -> (u32, u32) {
    let model = sim_mjcf::load_model(GRAVITY_MJCF).expect("parse");
    let data = model.make_data();

    let nq_ok = model.nq == 7;
    let nv_ok = model.nv == 6;
    let qpos_len = data.joint_qpos(&model, 0).len();
    let qvel_len = data.joint_qvel(&model, 0).len();
    let qpos_ok = qpos_len == 7;
    let qvel_ok = qvel_len == 6;

    let all_ok = nq_ok && nv_ok && qpos_ok && qvel_ok;

    let p = check(
        "Dimensions nq=7, nv=6",
        all_ok,
        &format!(
            "nq={}, nv={}, joint_qpos.len={qpos_len}, joint_qvel.len={qvel_len}",
            model.nq, model.nv
        ),
    );
    (u32::from(p), 1)
}

// ── Check 7: Free joint has no limits ──────────────────────────────────────

fn check_7_no_limits() -> (u32, u32) {
    let model = sim_mjcf::load_model(GRAVITY_MJCF).expect("parse");
    let mut data = model.make_data();

    // Launch at high velocity — should travel arbitrarily far
    data.qvel[0] = 100.0;

    for _ in 0..1_000 {
        data.step(&model).expect("step");
    }

    // After 1s at 100 m/s, x should be ~100.0 (gravity only affects z)
    let x = data.qpos[0];
    let p = check(
        "Free joint has no limits",
        x > 90.0,
        &format!("x after 1s at v=100 m/s: {x:.2} (no limit clamped it)"),
    );
    (u32::from(p), 1)
}

// ── Check 8: Gravity acceleration ──────────────────────────────────────────

fn check_8_gravity_acceleration() -> (u32, u32) {
    let model = sim_mjcf::load_model(GRAVITY_MJCF).expect("parse");
    let mut data = model.make_data();

    data.forward(&model).expect("forward");

    let az = data.qacc[2];
    let ax = data.qacc[0];
    let ay = data.qacc[1];
    let a_ang_x = data.qacc[3];
    let a_ang_y = data.qacc[4];
    let a_ang_z = data.qacc[5];

    let z_ok = (az - (-9.81)).abs() < 1e-10;
    let lateral_ok = ax.abs() < 1e-10 && ay.abs() < 1e-10;
    let angular_ok = a_ang_x.abs() < 1e-10 && a_ang_y.abs() < 1e-10 && a_ang_z.abs() < 1e-10;

    let p = check(
        "Gravity: qacc = [0,0,-g,0,0,0]",
        z_ok && lateral_ok && angular_ok,
        &format!(
            "qacc = [{ax:.2e}, {ay:.2e}, {az:.10}, {a_ang_x:.2e}, {a_ang_y:.2e}, {a_ang_z:.2e}]"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 9: No interaction without contact ────────────────────────────────

fn check_9_no_interaction() -> (u32, u32) {
    let model = sim_mjcf::load_model(TWO_BODIES_MJCF).expect("parse");
    let mut data = model.make_data();

    // Body A: linear X velocity. Body B: linear Z velocity.
    // Free joint layout: body A DOFs [0..6], body B DOFs [6..12]
    data.qvel[0] = 1.0; // A: vx
    data.qvel[8] = 0.5; // B: vz (DOF 6+2=8)

    for _ in 0..5_000 {
        data.step(&model).expect("step");
    }

    let drift_a = (data.qvel[0] - 1.0).abs();
    let drift_b = (data.qvel[8] - 0.5).abs();
    let max_drift = drift_a.max(drift_b);

    let p = check(
        "No interaction without contact",
        max_drift < 1e-12,
        &format!("A vx drift = {drift_a:.2e}, B vz drift = {drift_b:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 10: Quaternion integration matches exponential map ───────────────

/// Euler integrator + symmetric inertia — angular velocity stays constant
/// (no gyroscopic torque), so one Euler step is exactly one exp map.
const EULER_SYMMETRIC_MJCF: &str = r#"
<mujoco model="free-euler-sym">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="Euler"/>
  <worldbody>
    <body name="sphere" pos="0 0 0">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="2.0" diaginertia="0.04 0.04 0.04"/>
      <geom type="sphere" size="0.1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"#;

fn check_10_exp_map() -> (u32, u32) {
    let model = sim_mjcf::load_model(EULER_SYMMETRIC_MJCF).expect("parse");
    let mut data = model.make_data();

    let omega = Vector3::new(1.0, 0.5, 0.3);
    data.qvel[3] = omega.x;
    data.qvel[4] = omega.y;
    data.qvel[5] = omega.z;

    // Record initial quaternion
    let q_old = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        data.qpos[3],
        data.qpos[4],
        data.qpos[5],
        data.qpos[6],
    ));

    // Compute analytical result: q_new = q_old * exp(omega * dt)
    let dt = model.timestep;
    let omega_norm = omega.norm();
    let dq = UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(omega / omega_norm),
        omega_norm * dt,
    );
    let q_analytical = q_old * dq;

    // One Euler step = exactly one exp map application
    data.step(&model).expect("step");

    let q_engine = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        data.qpos[3],
        data.qpos[4],
        data.qpos[5],
        data.qpos[6],
    ));

    // Compare quaternions (handle sign ambiguity: q and -q are the same rotation)
    let angle_err = q_engine.angle_to(&q_analytical);

    let p = check(
        "Exp map: engine matches analytical (1 Euler step)",
        angle_err < 1e-12,
        &format!("angle error = {angle_err:.2e} rad"),
    );
    (u32::from(p), 1)
}

// ── Check 11: Energy conservation (zero-g) ─────────────────────────────────

fn check_11_energy_conservation() -> (u32, u32) {
    let model = sim_mjcf::load_model(ZERO_G_MJCF).expect("parse");
    let mut data = model.make_data();

    data.qvel[0] = 0.5;
    data.qvel[1] = 0.3;
    data.qvel[3] = 2.0;
    data.qvel[4] = 0.5;
    data.qvel[5] = 0.1;

    data.forward(&model).expect("forward");
    let e0 = data.energy_kinetic + data.energy_potential;

    for _ in 0..10_000 {
        data.step(&model).expect("step");
    }

    let e_final = data.energy_kinetic + data.energy_potential;
    let drift = (e_final - e0).abs();

    let p = check(
        "Energy conservation (zero-g, 10s)",
        drift < 1e-10,
        &format!("E_0 = {e0:.12}, E_f = {e_final:.12}, drift = {drift:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 12: Gravity changes momentum at rate mg ──────────────────────────

fn check_12_gravity_momentum() -> (u32, u32) {
    let model = sim_mjcf::load_model(GRAVITY_MJCF).expect("parse");
    let mut data = model.make_data();

    let mass = model.body_mass[1];
    let g = 9.81;

    for _ in 0..1_000 {
        data.step(&model).expect("step");
    }

    // After 1s from rest: p_z = mass * vz, expected = mass * (-g * 1.0)
    let pz_actual = mass * data.qvel[2];
    let pz_expected = mass * (-g * 1.0);
    let drift = (pz_actual - pz_expected).abs();

    let p = check(
        "Gravity: dp/dt = mg (1s from rest)",
        drift < 1e-6,
        &format!("p_z = {pz_actual:.8}, expected = {pz_expected:.8}, drift = {drift:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Main ───────────────────────────────────────────────────────────────────

fn main() {
    println!("=== Free Joint — Stress Test ===\n");

    let checks: Vec<(&str, fn() -> (u32, u32))> = vec![
        ("Quaternion norm", check_1_quaternion_norm),
        ("Linear momentum", check_2_linear_momentum),
        ("Angular momentum", check_3_angular_momentum),
        ("Projectile apex", check_4_projectile_apex),
        ("Projectile range", check_5_projectile_range),
        ("Dimensions", check_6_dimensions),
        ("No limits", check_7_no_limits),
        ("Gravity acceleration", check_8_gravity_acceleration),
        ("No interaction", check_9_no_interaction),
        ("Exp map integration", check_10_exp_map),
        ("Energy conservation", check_11_energy_conservation),
        ("Gravity momentum", check_12_gravity_momentum),
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
