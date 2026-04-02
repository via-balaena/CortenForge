//! Spring-Damper Tuning — Underdamped, Critically Damped, Overdamped
//!
//! Three identical hinge arms in zero gravity, each with the same torsional
//! spring but different damping. Damping values are computed programmatically
//! from the mass matrix diagonal to hit exact damping ratios:
//!   zeta = c / (2 * sqrt(k * I_eff))
//!
//! - Underdamped  (zeta=0.1): oscillates with decaying amplitude
//! - Critically damped (zeta=1.0): fastest return, no overshoot
//! - Overdamped   (zeta=3.0): sluggish exponential return
//!
//! Run: `cargo run -p example-passive-spring-damper-tuning --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::needless_range_loop,
    clippy::needless_pass_by_value,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::let_underscore_must_use,
    clippy::unwrap_used
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF ──────────────────────────────────────────────────────────────────

/// Zero gravity, three identical arms. Stiffness and damping start at zero
/// (arms hold initial angle). At t=2s, stiffness and damping activate.
const MJCF: &str = r#"
<mujoco model="spring-damper-tuning">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001"/>
  <worldbody>
    <body name="arm_a" pos="-1.5 0 0">
      <joint name="j_a" type="hinge" axis="0 1 0"
             stiffness="0" damping="0" springref="0"/>
      <geom name="rod_a" type="capsule" fromto="0 0 0 0 0 -1"
            size="0.04" mass="1.0" rgba="0.3 0.6 1.0 1"/>
      <geom name="tip_a" type="sphere" pos="0 0 -1"
            size="0.08" mass="0.5" rgba="0.3 0.6 1.0 1"/>
    </body>
    <body name="arm_b" pos="0 0 0">
      <joint name="j_b" type="hinge" axis="0 1 0"
             stiffness="0" damping="0" springref="0"/>
      <geom name="rod_b" type="capsule" fromto="0 0 0 0 0 -1"
            size="0.04" mass="1.0" rgba="0.2 0.8 0.3 1"/>
      <geom name="tip_b" type="sphere" pos="0 0 -1"
            size="0.08" mass="0.5" rgba="0.2 0.8 0.3 1"/>
    </body>
    <body name="arm_c" pos="1.5 0 0">
      <joint name="j_c" type="hinge" axis="0 1 0"
             stiffness="0" damping="0" springref="0"/>
      <geom name="rod_c" type="capsule" fromto="0 0 0 0 0 -1"
            size="0.04" mass="1.0" rgba="0.9 0.3 0.2 1"/>
      <geom name="tip_c" type="sphere" pos="0 0 -1"
            size="0.08" mass="0.5" rgba="0.9 0.3 0.2 1"/>
    </body>
  </worldbody>
</mujoco>
"#;

const STIFFNESS: f64 = 20.0;
const ZETA_A: f64 = 0.1; // underdamped
const ZETA_B: f64 = 1.0; // critically damped
const ZETA_C: f64 = 3.0; // overdamped
const INITIAL_ANGLE: f64 = 0.8; // ~45 degrees
const DELAY: f64 = 2.0;
const REPORT_TIME: f64 = 17.0; // 2s delay + 15s of physics

// ── Bevy app ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Spring-Damper Tuning ===");
    println!("  Three arms: underdamped (blue), critical (green), overdamped (red).");
    println!("  Zero gravity, pure torsional spring-damper response.\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Spring-Damper Tuning — Second-Order Response".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<SpringDamperValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(REPORT_TIME)
                .print_every(1.0)
                .display(|m, d| {
                    let t = d.time;
                    let qa = d.qpos[m.jnt_qpos_adr[0]];
                    let qb = d.qpos[m.jnt_qpos_adr[1]];
                    let qc = d.qpos[m.jnt_qpos_adr[2]];
                    format!("t={t:.1}s  under={qa:.3}  crit={qb:.3}  over={qc:.3} rad")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (release_springs, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                spring_damper_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Setup ─────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    // Run forward to populate mass matrix (CRBA)
    data.forward(&model).expect("forward for qM");

    // Read effective inertia and pre-compute damping values for release
    let joints = ["j_a", "j_b", "j_c"];
    let zetas = [ZETA_A, ZETA_B, ZETA_C];
    let mut damping_values = [0.0f64; 3];

    for (i, (jname, zeta)) in joints.iter().zip(zetas.iter()).enumerate() {
        let jid = model.joint_id(jname).expect(jname);
        let dof_adr = model.jnt_dof_adr[jid];
        let i_eff = data.qM[(dof_adr, dof_adr)];
        damping_values[i] = 2.0 * zeta * (STIFFNESS * i_eff).sqrt();

        println!(
            "  {jname}: I_eff={i_eff:.4}, zeta={zeta:.1}, damping={:.4}",
            damping_values[i]
        );
    }
    println!();

    // Displace all joints to initial angle
    for jname in &joints {
        let jid = model.joint_id(jname).expect(jname);
        let qadr = model.jnt_qpos_adr[jid];
        data.qpos[qadr] = INITIAL_ANGLE;
    }

    // Compute omega_n for the HUD
    let jid_a = model.joint_id("j_a").expect("j_a");
    let i_eff = data.qM[(model.jnt_dof_adr[jid_a], model.jnt_dof_adr[jid_a])];
    let omega_n = (STIFFNESS / i_eff).sqrt();
    commands.insert_resource(DynamicsParams {
        omega_n,
        i_eff,
        damping_values,
    });

    // Materials
    let mat_a = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.6, 1.0)));
    let mat_b = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.8, 0.3)));
    let mat_c = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.3, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("rod_a", mat_a.clone()),
            ("tip_a", mat_a),
            ("rod_b", mat_b.clone()),
            ("tip_b", mat_b),
            ("rod_c", mat_c.clone()),
            ("tip_c", mat_c),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -0.5, 0.0),
        5.0,
        std::f32::consts::FRAC_PI_2, // 90 deg: looking along Bevy -X
        0.0,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

#[derive(Resource)]
struct DynamicsParams {
    omega_n: f64,
    i_eff: f64,
    /// Pre-computed damping for [j_a, j_b, j_c], applied at release time
    damping_values: [f64; 3],
}

// ── Release springs after delay ───────────────────────────────────────────

fn release_springs(
    mut model: ResMut<PhysicsModel>,
    data: Res<PhysicsData>,
    params: Res<DynamicsParams>,
) {
    if data.0.time >= DELAY && model.0.jnt_stiffness[0] == 0.0 {
        let joints = ["j_a", "j_b", "j_c"];
        for (i, jname) in joints.iter().enumerate() {
            let jid = model.0.joint_id(jname).expect(jname);
            model.0.jnt_stiffness[jid] = STIFFNESS;
            model.0.jnt_damping[jid] = params.damping_values[i];
        }
    }
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    params: Res<DynamicsParams>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();

    let m = &model.0;
    let d = &data.0;

    hud.section("Second-Order Response");
    hud.raw(String::new());

    let arms: [(&str, &str, f64); 3] = [
        ("Under  ", "j_a", ZETA_A),
        ("Crit   ", "j_b", ZETA_B),
        ("Over   ", "j_c", ZETA_C),
    ];

    hud.raw("           angle    vel".to_string());

    for (label, jname, zeta) in &arms {
        let jid = m.joint_id(jname).expect(jname);
        let qadr = m.jnt_qpos_adr[jid];
        let dadr = m.jnt_dof_adr[jid];
        let q_deg = d.qpos[qadr].to_degrees();
        let qd = d.qvel[dadr];

        hud.raw(format!(
            "  {label}(z={zeta:.1})  {q_deg:6.1} deg  {qd:6.3} r/s"
        ));
    }

    hud.raw(String::new());
    hud.scalar("omega_n (rad/s)", params.omega_n, 2);
    hud.scalar("I_eff (kg m2)", params.i_eff, 4);
    hud.scalar("k (N m/rad)", STIFFNESS, 0);
    hud.scalar("time (s)", d.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SpringDamperValidation {
    /// Zero-crossing counts (sign changes of qpos)
    crossings_a: u32,
    crossings_b: u32,
    crossings_c: u32,
    prev_sign_a: f64,
    prev_sign_b: f64,
    prev_sign_c: f64,
    /// Time when critical damping arm first enters |q| < 0.01
    crit_settle_time: Option<f64>,
    /// Time when overdamped arm first enters |q| < 0.01
    over_settle_time: Option<f64>,
    /// Track underdamped zero-crossing times for period measurement
    crossing_times_a: Vec<f64>,
    reported: bool,
}

fn spring_damper_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    params: Res<DynamicsParams>,
    mut val: ResMut<SpringDamperValidation>,
) {
    let m = &model.0;
    let d = &data.0;

    // Don't track until springs are released
    if d.time < DELAY + 0.01 {
        val.prev_sign_a = INITIAL_ANGLE;
        val.prev_sign_b = INITIAL_ANGLE;
        val.prev_sign_c = INITIAL_ANGLE;
        return;
    }

    let qa = d.qpos[m.jnt_qpos_adr[0]];
    let qb = d.qpos[m.jnt_qpos_adr[1]];
    let qc = d.qpos[m.jnt_qpos_adr[2]];

    // Track zero-crossings
    if qa * val.prev_sign_a < 0.0 {
        val.crossings_a += 1;
        val.crossing_times_a.push(d.time);
    }
    if qb * val.prev_sign_b < 0.0 {
        val.crossings_b += 1;
    }
    if qc * val.prev_sign_c < 0.0 {
        val.crossings_c += 1;
    }
    val.prev_sign_a = qa;
    val.prev_sign_b = qb;
    val.prev_sign_c = qc;

    // Track settle times
    if val.crit_settle_time.is_none() && qb.abs() < 0.01 {
        val.crit_settle_time = Some(d.time);
    }
    if val.over_settle_time.is_none() && qc.abs() < 0.01 {
        val.over_settle_time = Some(d.time);
    }

    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    // Measure underdamped period from zero-crossing pairs
    let measured_period = if val.crossing_times_a.len() >= 3 {
        // Period = time between every 2nd crossing (full cycle)
        let t0 = val.crossing_times_a[0];
        let t2 = val.crossing_times_a[2];
        Some(t2 - t0)
    } else {
        None
    };

    let omega_d = params.omega_n * ZETA_A.mul_add(-ZETA_A, 1.0).sqrt();
    let expected_period = 2.0 * std::f64::consts::PI / omega_d;

    let period_err =
        measured_period.map(|mp| ((mp - expected_period) / expected_period * 100.0).abs());

    let crit_settle = val.crit_settle_time.unwrap_or(f64::MAX);
    let over_settle = val.over_settle_time.unwrap_or(f64::MAX);

    let checks = vec![
        Check {
            name: "Underdamped: >= 3 zero-crossings",
            pass: val.crossings_a >= 3,
            detail: format!("crossings = {}", val.crossings_a),
        },
        Check {
            name: "Critical: no zero-crossing (no overshoot)",
            pass: val.crossings_b == 0,
            detail: format!("crossings = {}", val.crossings_b),
        },
        Check {
            name: "Overdamped: no zero-crossing",
            pass: val.crossings_c == 0,
            detail: format!("crossings = {}", val.crossings_c),
        },
        Check {
            name: "Overdamped slower than critical",
            pass: over_settle > crit_settle,
            detail: format!("crit={crit_settle:.2}s, over={over_settle:.2}s"),
        },
        Check {
            name: "All converge to springref",
            pass: qa.abs() < 0.01 && qb.abs() < 0.01 && qc.abs() < 0.01,
            detail: format!(
                "|q_a|={:.4}, |q_b|={:.4}, |q_c|={:.4} (need < 0.01)",
                qa.abs(),
                qb.abs(),
                qc.abs()
            ),
        },
        Check {
            name: "Underdamped period within 5%",
            pass: period_err.is_some_and(|e| e < 5.0),
            detail: format!(
                "measured={:.4}s, expected={expected_period:.4}s, err={:.2}%",
                measured_period.unwrap_or(0.0),
                period_err.unwrap_or(f64::MAX)
            ),
        },
    ];

    let _ = print_report("Spring-Damper Tuning (t=15s)", &checks);
}
