//! Stress test — headless validation of all energy-momentum conservation concepts.
//!
//! 12 checks covering: free-flight conservation (KE, linear momentum, angular
//! momentum, velocity components), pendulum energy exchange, elastic bounce,
//! damped dissipation, energy flag, and multi-body conservation.
//!
//! Run: `cargo run -p example-energy-stress-test --release`

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

use sim_core::ENABLE_ENERGY;

// ── MJCF Models ────────────────────────────────────────────────────────────

/// Free body in zero gravity with initial linear + angular velocity.
const FREE_FLIGHT_MJCF: &str = r#"
<mujoco model="free-flight">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="box" pos="0 0 0">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="2.0" diaginertia="0.05 0.03 0.02"/>
      <geom type="box" size="0.15 0.1 0.08" contype="0" conaffinity="0"/>
    </body>
  </worldbody>

  <sensor>
    <subtreeangmom name="angmom" body="box"/>
  </sensor>
</mujoco>
"#;

/// Undamped pendulum for energy conservation.
const PENDULUM_MJCF: &str = r#"
<mujoco model="pendulum-energy">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
      <inertial pos="0 0 -0.2" mass="1.0" diaginertia="0.02 0.02 0.002"/>
      <geom type="capsule" size="0.02" fromto="0 0 0  0 0 -0.4" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Damped pendulum for dissipation check.
const DAMPED_MJCF: &str = r#"
<mujoco model="damped-pendulum">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.5"/>
      <inertial pos="0 0 -0.2" mass="1.0" diaginertia="0.02 0.02 0.002"/>
      <geom type="capsule" size="0.02" fromto="0 0 0  0 0 -0.4" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Ball dropped onto plane for elastic bounce.
const BOUNCE_MJCF: &str = r#"
<mujoco model="elastic-bounce">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.0005" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <default>
    <geom solref="-10000 -0.99" solimp="0.99 0.99 0.001 0.5 2"/>
  </default>

  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" pos="0 0 0"/>
    <body name="ball" pos="0 0 0.55">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.001 0.001 0.001"/>
      <geom name="sphere" type="sphere" size="0.05" condim="1"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Multi-body free flight (3 bodies, zero gravity).
const MULTI_BODY_MJCF: &str = r#"
<mujoco model="multi-body">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="a" pos="0 0 0">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
    </body>
    <body name="b" pos="1 0 0">
      <freejoint/>
      <inertial pos="0 0 0" mass="2.0" diaginertia="0.02 0.03 0.01"/>
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
    </body>
    <body name="c" pos="0 1 0">
      <freejoint/>
      <inertial pos="0 0 0" mass="0.5" diaginertia="0.005 0.005 0.005"/>
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Energy flag disabled — verify fields stay zero.
const NO_ENERGY_MJCF: &str = r#"
<mujoco model="no-energy">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 -0.2" mass="1.0" diaginertia="0.02 0.02 0.002"/>
      <geom type="capsule" size="0.02" fromto="0 0 0  0 0 -0.4" contype="0" conaffinity="0"/>
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

// ── Check 1: Free-flight KE conservation ───────────────────────────────────

fn check_1_free_flight_ke() -> (u32, u32) {
    let model = sim_mjcf::load_model(FREE_FLIGHT_MJCF).expect("parse");
    let mut data = model.make_data();

    // Initial velocity: linear [0.5, 0.3, 0] + angular [0.2, 0.1, 0.4]
    data.qvel[0] = 0.5;
    data.qvel[1] = 0.3;
    data.qvel[2] = 0.0;
    data.qvel[3] = 0.2;
    data.qvel[4] = 0.1;
    data.qvel[5] = 0.4;

    data.forward(&model).expect("forward");
    let ke_initial = data.energy_kinetic;

    // Simulate 10 seconds
    let steps = 10_000; // 10s at dt=0.001
    for _ in 0..steps {
        data.step(&model).expect("step");
    }

    let ke_final = data.energy_kinetic;
    let drift = (ke_final - ke_initial).abs();

    let p = check(
        "Free-flight KE",
        drift < 1e-10,
        &format!("initial={ke_initial:.12}, final={ke_final:.12}, drift={drift:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 2: Free-flight linear momentum ───────────────────────────────────

fn check_2_free_flight_linear_momentum() -> (u32, u32) {
    let model = sim_mjcf::load_model(FREE_FLIGHT_MJCF).expect("parse");
    let mut data = model.make_data();

    data.qvel[0] = 0.5;
    data.qvel[1] = 0.3;
    data.qvel[2] = 0.0;
    data.qvel[3] = 0.2;
    data.qvel[4] = 0.1;
    data.qvel[5] = 0.4;

    let mass = model.body_mass[1]; // body 0 is world
    let p_initial = [
        mass * data.qvel[0],
        mass * data.qvel[1],
        mass * data.qvel[2],
    ];

    let steps = 10_000;
    for _ in 0..steps {
        data.step(&model).expect("step");
    }

    let p_final = [
        mass * data.qvel[0],
        mass * data.qvel[1],
        mass * data.qvel[2],
    ];

    let drift = ((p_final[0] - p_initial[0]).powi(2)
        + (p_final[1] - p_initial[1]).powi(2)
        + (p_final[2] - p_initial[2]).powi(2))
    .sqrt();

    let p = check(
        "Free-flight linear momentum",
        drift < 1e-10,
        &format!(
            "p_init=[{:.6}, {:.6}, {:.6}], p_final=[{:.6}, {:.6}, {:.6}], drift={drift:.2e}",
            p_initial[0], p_initial[1], p_initial[2], p_final[0], p_final[1], p_final[2]
        ),
    );
    (u32::from(p), 1)
}

// ── Check 3: Free-flight angular momentum ──────────────────────────────────

fn check_3_free_flight_angular_momentum() -> (u32, u32) {
    let model = sim_mjcf::load_model(FREE_FLIGHT_MJCF).expect("parse");
    let mut data = model.make_data();

    data.qvel[0] = 0.5;
    data.qvel[1] = 0.3;
    data.qvel[2] = 0.0;
    data.qvel[3] = 0.2;
    data.qvel[4] = 0.1;
    data.qvel[5] = 0.4;

    // Run forward to populate sensor data (triggers mj_subtree_vel)
    data.forward(&model).expect("forward");
    let sensor_adr = model.sensor_adr[0]; // subtreeangmom sensor
    let l_initial = [
        data.sensordata[sensor_adr],
        data.sensordata[sensor_adr + 1],
        data.sensordata[sensor_adr + 2],
    ];
    let l_initial_mag = (l_initial[0].powi(2) + l_initial[1].powi(2) + l_initial[2].powi(2)).sqrt();

    let steps = 10_000;
    for _ in 0..steps {
        data.step(&model).expect("step");
    }
    // Need a forward() to recompute sensors after final step
    data.forward(&model).expect("forward");

    let l_final = [
        data.sensordata[sensor_adr],
        data.sensordata[sensor_adr + 1],
        data.sensordata[sensor_adr + 2],
    ];
    let l_final_mag = (l_final[0].powi(2) + l_final[1].powi(2) + l_final[2].powi(2)).sqrt();

    let drift = (l_final_mag - l_initial_mag).abs();

    let p = check(
        "Free-flight angular momentum",
        drift < 1e-8,
        &format!("|L_init|={l_initial_mag:.10}, |L_final|={l_final_mag:.10}, drift={drift:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 4: Free-flight velocity components constant (symmetric body) ─────

/// Symmetric body — angular velocity is constant in world frame (no precession).
const SYMMETRIC_MJCF: &str = r#"
<mujoco model="symmetric-free">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="sphere" pos="0 0 0">
      <freejoint name="free"/>
      <inertial pos="0 0 0" mass="2.0" diaginertia="0.04 0.04 0.04"/>
      <geom type="sphere" size="0.1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"#;

fn check_4_free_flight_velocity_components() -> (u32, u32) {
    let model = sim_mjcf::load_model(SYMMETRIC_MJCF).expect("parse");
    let mut data = model.make_data();

    let init_vel = [0.5, 0.3, 0.0, 0.2, 0.1, 0.4];
    for i in 0..6 {
        data.qvel[i] = init_vel[i];
    }

    let steps = 10_000;
    for _ in 0..steps {
        data.step(&model).expect("step");
    }

    let mut max_drift: f64 = 0.0;
    for i in 0..6 {
        let drift = (data.qvel[i] - init_vel[i]).abs();
        max_drift = max_drift.max(drift);
    }

    let p = check(
        "Free-flight velocity components (symmetric)",
        max_drift < 1e-10,
        &format!(
            "qvel=[{:.10}, {:.10}, {:.10}, {:.10}, {:.10}, {:.10}], max_drift={max_drift:.2e}",
            data.qvel[0], data.qvel[1], data.qvel[2], data.qvel[3], data.qvel[4], data.qvel[5]
        ),
    );
    (u32::from(p), 1)
}

// ── Check 5: Undamped pendulum energy conservation ─────────────────────────

fn check_5_pendulum_energy_conservation() -> (u32, u32) {
    let model = sim_mjcf::load_model(PENDULUM_MJCF).expect("parse");
    let mut data = model.make_data();

    // Start at 45 degrees
    data.qpos[0] = std::f64::consts::FRAC_PI_4;
    data.forward(&model).expect("forward");
    let e_initial = data.total_energy();

    let steps = 10_000; // 10s
    let mut max_drift_pct: f64 = 0.0;
    for _ in 0..steps {
        data.step(&model).expect("step");
        let drift_pct = ((data.total_energy() - e_initial) / e_initial.abs()).abs() * 100.0;
        max_drift_pct = max_drift_pct.max(drift_pct);
    }

    let p = check(
        "Undamped pendulum energy",
        max_drift_pct < 0.5,
        &format!(
            "E_init={e_initial:.6}, E_final={:.6}, max_drift={max_drift_pct:.4}%",
            data.total_energy()
        ),
    );
    (u32::from(p), 1)
}

// ── Check 6: Pendulum energy partition ─────────────────────────────────────

fn check_6_pendulum_energy_partition() -> (u32, u32) {
    let model = sim_mjcf::load_model(PENDULUM_MJCF).expect("parse");
    let mut data = model.make_data();

    // Start at 90 degrees (horizontal) for large energy partition
    data.qpos[0] = std::f64::consts::FRAC_PI_2;
    data.forward(&model).expect("forward");
    let pe_initial = data.energy_potential;

    // Simulate until pendulum reaches bottom (qpos crosses 0 going negative)
    let mut prev_qpos = data.qpos[0];
    let mut found_bottom = false;
    for _ in 0..5_000 {
        data.step(&model).expect("step");
        let q = data.qpos[0];
        if prev_qpos > 0.0 && q <= 0.0 {
            found_bottom = true;
            break;
        }
        prev_qpos = q;
    }

    // At bottom: all PE drop should have converted to KE
    let pe_drop = pe_initial - data.energy_potential;
    let ke_ratio = if pe_drop.abs() > 1e-12 {
        data.energy_kinetic / pe_drop
    } else {
        0.0
    };

    let p = check(
        "Pendulum energy partition",
        found_bottom && ke_ratio > 0.9,
        &format!(
            "at bottom: KE={:.6}, PE_drop={pe_drop:.6}, KE/PE_drop={ke_ratio:.4} (need >0.9)",
            data.energy_kinetic,
        ),
    );
    (u32::from(p), 1)
}

// ── Check 7: Elastic bounce energy ─────────────────────────────────────────

fn check_7_elastic_bounce_energy() -> (u32, u32) {
    let model = sim_mjcf::load_model(BOUNCE_MJCF).expect("parse");
    let mut data = model.make_data();

    data.forward(&model).expect("forward");
    let e_initial = data.total_energy();

    // Simulate to the apex after first bounce (vz crosses zero going negative).
    // Energy measured at apex is in free flight — no contact spring artifacts.
    let mut hit_ground = false;
    let mut rising = false;
    let mut found_apex = false;
    let mut prev_vz: f64 = 0.0;
    let mut e_at_apex = e_initial;

    for _ in 0..40_000 {
        // 20s at dt=0.0005
        data.step(&model).expect("step");
        let z = data.qpos[2];
        let vz = data.qvel[2];

        if z < 0.1 && vz < 0.0 {
            hit_ground = true;
        }
        if hit_ground && vz > 0.0 {
            rising = true;
        }
        // Apex: was rising, now falling (vz crosses zero going negative)
        if rising && prev_vz > 0.0 && vz <= 0.0 {
            found_apex = true;
            e_at_apex = data.total_energy();
            break;
        }
        prev_vz = vz;
    }

    let loss_pct = if e_initial.abs() > 1e-12 {
        ((e_initial - e_at_apex) / e_initial.abs()).abs() * 100.0
    } else {
        100.0
    };

    let p = check(
        "Elastic bounce energy",
        found_apex && loss_pct < 5.0,
        &format!("E_init={e_initial:.6}, E_apex={e_at_apex:.6}, loss={loss_pct:.2}%"),
    );
    (u32::from(p), 1)
}

// ── Check 8: Bounce height (restitution) ───────────────────────────────────

fn check_8_bounce_height() -> (u32, u32) {
    let model = sim_mjcf::load_model(BOUNCE_MJCF).expect("parse");
    let mut data = model.make_data();

    let z_drop = data.qpos[2]; // initial height
    let sphere_r = 0.05;

    // Simulate and track max height after first bounce
    let mut hit_ground = false;
    let mut max_z_after_bounce: f64 = 0.0;
    let mut tracking_rise = false;
    let mut prev_vz = 0.0;

    for _ in 0..40_000 {
        // 20s at dt=0.0005
        data.step(&model).expect("step");
        let z = data.qpos[2];
        let vz = data.qvel[2];

        if z < sphere_r + 0.02 && vz < 0.0 {
            hit_ground = true;
        }
        if hit_ground && vz > 0.0 {
            tracking_rise = true;
        }
        if tracking_rise {
            max_z_after_bounce = max_z_after_bounce.max(z);
            // Stop when ball starts falling again after bounce
            if prev_vz > 0.0 && vz <= 0.0 {
                break;
            }
        }
        prev_vz = vz;
    }

    let restitution = if z_drop > sphere_r {
        (max_z_after_bounce - sphere_r) / (z_drop - sphere_r)
    } else {
        0.0
    };

    let p = check(
        "Bounce height",
        hit_ground && restitution > 0.85,
        &format!(
            "drop={z_drop:.4}, bounce_apex={max_z_after_bounce:.4}, restitution={restitution:.4}"
        ),
    );
    (u32::from(p), 1)
}

// ── Check 9: Damped pendulum monotonic ─────────────────────────────────────

fn check_9_damped_monotonic() -> (u32, u32) {
    let model = sim_mjcf::load_model(DAMPED_MJCF).expect("parse");
    let mut data = model.make_data();

    data.qpos[0] = std::f64::consts::FRAC_PI_4;
    data.forward(&model).expect("forward");
    let mut prev_energy = data.total_energy();

    let steps = 10_000;
    let mut violations = 0u32;
    let mut max_increase: f64 = 0.0;

    for _ in 0..steps {
        data.step(&model).expect("step");
        let e = data.total_energy();
        let increase = e - prev_energy;
        if increase > 1e-12 {
            violations += 1;
            max_increase = max_increase.max(increase);
        }
        prev_energy = e;
    }

    let p = check(
        "Damped pendulum monotonic",
        violations == 0,
        &format!("violations={violations}, max_increase={max_increase:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Check 10: Damped pendulum dissipation ──────────────────────────────────

fn check_10_damped_dissipation() -> (u32, u32) {
    let model = sim_mjcf::load_model(DAMPED_MJCF).expect("parse");
    let mut data = model.make_data();

    data.qpos[0] = std::f64::consts::FRAC_PI_4;
    data.forward(&model).expect("forward");

    // Track max KE (occurs at first bottom swing) and final KE
    let steps = 10_000;
    let mut ke_max: f64 = 0.0;
    for _ in 0..steps {
        data.step(&model).expect("step");
        ke_max = ke_max.max(data.energy_kinetic);
    }
    let ke_final = data.energy_kinetic;

    let remaining_pct = if ke_max > 1e-12 {
        (ke_final / ke_max) * 100.0
    } else {
        0.0
    };

    let p = check(
        "Damped pendulum dissipation",
        ke_max > 0.01 && remaining_pct < 20.0,
        &format!("KE_max={ke_max:.6}, KE_final={ke_final:.8}, remaining={remaining_pct:.2}%"),
    );
    (u32::from(p), 1)
}

// ── Check 11: Energy flag disabled ─────────────────────────────────────────

fn check_11_energy_flag_disabled() -> (u32, u32) {
    let model = sim_mjcf::load_model(NO_ENERGY_MJCF).expect("parse");
    let mut data = model.make_data();

    // Verify flag is not set
    assert!(
        model.enableflags & ENABLE_ENERGY == 0,
        "energy should be disabled"
    );

    data.qpos[0] = std::f64::consts::FRAC_PI_4;

    for _ in 0..100 {
        data.step(&model).expect("step");
    }

    let p = check(
        "Energy flag disabled",
        data.energy_kinetic == 0.0 && data.energy_potential == 0.0,
        &format!(
            "KE={}, PE={} (both should be 0.0)",
            data.energy_kinetic, data.energy_potential
        ),
    );
    (u32::from(p), 1)
}

// ── Check 12: Multi-body conservation ──────────────────────────────────────

fn check_12_multi_body_conservation() -> (u32, u32) {
    let model = sim_mjcf::load_model(MULTI_BODY_MJCF).expect("parse");
    let mut data = model.make_data();

    // Give each body different velocities
    // Body 0 (world) has no DOFs. Bodies 1,2,3 each have 6 DOFs.
    // qvel layout: [body1_lin(3), body1_ang(3), body2_lin(3), body2_ang(3), body3_lin(3), body3_ang(3)]
    let velocities = [
        0.3, 0.1, -0.2, 0.1, 0.0, 0.3, // body a
        -0.2, 0.4, 0.1, 0.0, -0.2, 0.1, // body b
        0.1, -0.1, 0.5, 0.2, 0.1, -0.1, // body c
    ];
    for (i, &v) in velocities.iter().enumerate() {
        data.qvel[i] = v;
    }

    data.forward(&model).expect("forward");
    let ke_initial = data.energy_kinetic;

    let steps = 10_000;
    for _ in 0..steps {
        data.step(&model).expect("step");
    }

    let ke_final = data.energy_kinetic;
    let drift = (ke_final - ke_initial).abs();

    let p = check(
        "Multi-body conservation",
        drift < 1e-10,
        &format!("KE_init={ke_initial:.10}, KE_final={ke_final:.10}, drift={drift:.2e}"),
    );
    (u32::from(p), 1)
}

// ── Main ───────────────────────────────────────────────────────────────────

fn main() {
    println!("=== Energy-Momentum — Stress Test ===\n");

    let mut total = 0u32;
    let mut passed = 0u32;

    println!("-- 1. Free-flight KE conservation --");
    let (p, t) = check_1_free_flight_ke();
    passed += p;
    total += t;

    println!("\n-- 2. Free-flight linear momentum --");
    let (p, t) = check_2_free_flight_linear_momentum();
    passed += p;
    total += t;

    println!("\n-- 3. Free-flight angular momentum --");
    let (p, t) = check_3_free_flight_angular_momentum();
    passed += p;
    total += t;

    println!("\n-- 4. Free-flight velocity components --");
    let (p, t) = check_4_free_flight_velocity_components();
    passed += p;
    total += t;

    println!("\n-- 5. Undamped pendulum energy --");
    let (p, t) = check_5_pendulum_energy_conservation();
    passed += p;
    total += t;

    println!("\n-- 6. Pendulum energy partition --");
    let (p, t) = check_6_pendulum_energy_partition();
    passed += p;
    total += t;

    println!("\n-- 7. Elastic bounce energy --");
    let (p, t) = check_7_elastic_bounce_energy();
    passed += p;
    total += t;

    println!("\n-- 8. Bounce height --");
    let (p, t) = check_8_bounce_height();
    passed += p;
    total += t;

    println!("\n-- 9. Damped pendulum monotonic --");
    let (p, t) = check_9_damped_monotonic();
    passed += p;
    total += t;

    println!("\n-- 10. Damped pendulum dissipation --");
    let (p, t) = check_10_damped_dissipation();
    passed += p;
    total += t;

    println!("\n-- 11. Energy flag disabled --");
    let (p, t) = check_11_energy_flag_disabled();
    passed += p;
    total += t;

    println!("\n-- 12. Multi-body conservation --");
    let (p, t) = check_12_multi_body_conservation();
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
