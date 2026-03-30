//! Gravity Compensation — Static Holding Torques via Inverse Dynamics
//!
//! The simplest use of inverse dynamics: compute the torques that keep
//! a two-link arm stationary at a non-vertical pose.
//!
//! With `qacc = 0` and `qvel = 0`, `data.inverse()` yields the pure
//! gravity compensation torques. These are applied as constant motor
//! `ctrl` values — the arm should hold its pose indefinitely.
//!
//! Validates:
//! - `qfrc_inverse == qfrc_bias` when qvel=0, qacc=0, no passive/constraint
//! - Arm holds pose < 0.001 rad drift over 5 seconds
//! - Shoulder torque > elbow torque (more mass below)
//! - Energy stays constant (open-loop gravity comp does zero net work)
//!
//! Run with: `cargo run -p example-inverse-gravity-compensation --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::struct_excessive_bools
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

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="gravity-compensation">
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
      <geom name="upper_rod" type="capsule" size="0.03"
            fromto="0 0 0  0 0 -0.4" rgba="0.48 0.48 0.50 1"/>
      <body name="lower" pos="0 0 -0.4">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 -0.15" mass="1.0" diaginertia="0.008 0.008 0.002"/>
        <geom name="lower_rod" type="capsule" size="0.025"
              fromto="0 0 0  0 0 -0.3" rgba="0.48 0.48 0.50 1"/>
        <geom name="tip" type="sphere" size="0.04"
              pos="0 0 -0.3" rgba="0.85 0.3 0.2 1"/>
        <site name="end_effector" pos="0 0 -0.3"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="shoulder_motor" joint="shoulder" gear="1"/>
    <motor name="elbow_motor" joint="elbow" gear="1"/>
  </actuator>

  <sensor>
    <jointpos name="s_jpos" joint="shoulder"/>
    <jointpos name="e_jpos" joint="elbow"/>
  </sensor>
</mujoco>
"#;

// Initial pose: shoulder 45°, elbow -30°
const SHOULDER_INIT: f64 = 0.785; // ~45°
const ELBOW_INIT: f64 = -0.524; // ~-30°

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Gravity Compensation ===");
    println!("  Static holding torques via inverse dynamics");
    println!("  Pose: shoulder={SHOULDER_INIT:.3} rad, elbow={ELBOW_INIT:.3} rad");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Gravity Compensation".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<GravCompValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(6.0)
                .print_every(1.0)
                .display(|m, d| {
                    let s = d.sensor_scalar(m, "s_jpos").unwrap_or(0.0);
                    let e = d.sensor_scalar(m, "e_jpos").unwrap_or(0.0);
                    format!(
                        "shoulder={s:.4} elbow={e:.4}  drift={:.1e}/{:.1e}",
                        (s - SHOULDER_INIT).abs(),
                        (e - ELBOW_INIT).abs()
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                grav_comp_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Holding torques computed via inverse dynamics ───────────────────────────

#[derive(Resource)]
struct HoldingTorques {
    shoulder: f64,
    elbow: f64,
    bias_max_err: f64,
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    // 1. Set desired pose, zero velocity
    data.qpos[0] = SHOULDER_INIT;
    data.qpos[1] = ELBOW_INIT;
    // qvel is already zero from make_data()

    // 2. forward() computes M, qfrc_bias, kinematics
    data.forward(&model).expect("forward");

    // 3. Set qacc = 0 (we want zero acceleration — discard forward's qacc)
    data.qacc[0] = 0.0;
    data.qacc[1] = 0.0;

    // 4. inverse() → qfrc_inverse = holding torques
    data.inverse(&model);

    let shoulder_torque = data.qfrc_inverse[0];
    let elbow_torque = data.qfrc_inverse[1];

    // Verify: with qvel=0, qacc=0, no passive, no constraints,
    // qfrc_inverse should exactly equal qfrc_bias (pure gravity)
    let bias_match_0 = (data.qfrc_inverse[0] - data.qfrc_bias[0]).abs();
    let bias_match_1 = (data.qfrc_inverse[1] - data.qfrc_bias[1]).abs();
    println!("  Inverse dynamics at static pose:");
    println!("    shoulder torque = {shoulder_torque:+.4} N·m");
    println!("    elbow torque    = {elbow_torque:+.4} N·m");
    println!("    |inv - bias|    = [{bias_match_0:.2e}, {bias_match_1:.2e}]");
    println!();

    // 5. Apply holding torques as constant ctrl
    data.set_ctrl(0, shoulder_torque);
    data.set_ctrl(1, elbow_torque);

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("upper_rod", mat_rod.clone()),
            ("lower_rod", mat_rod),
            ("tip", mat_tip),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, -0.25),
        1.8,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(HoldingTorques {
        shoulder: shoulder_torque,
        elbow: elbow_torque,
        bias_max_err: bias_match_0.max(bias_match_1),
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    torques: Res<HoldingTorques>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();
    hud.section("Gravity Compensation");

    let s_pos = data.sensor_scalar(&model, "s_jpos").unwrap_or(0.0);
    let e_pos = data.sensor_scalar(&model, "e_jpos").unwrap_or(0.0);
    let s_drift = s_pos - SHOULDER_INIT;
    let e_drift = e_pos - ELBOW_INIT;

    hud.scalar("shoulder", s_pos, 4);
    hud.scalar("elbow", e_pos, 4);
    hud.scalar("s_drift", s_drift, 6);
    hud.scalar("e_drift", e_drift, 6);
    hud.scalar("s_torque", torques.shoulder, 3);
    hud.scalar("e_torque", torques.elbow, 3);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct GravCompValidation {
    max_shoulder_drift: f64,
    max_elbow_drift: f64,
    initial_energy: Option<f64>,
    max_energy_err_pct: f64,
    bias_match_checked: bool,
    shoulder_gt_elbow: bool,
    reported: bool,
}

fn grav_comp_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    torques: Res<HoldingTorques>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<GravCompValidation>,
) {
    let s_pos = data.sensor_scalar(&model, "s_jpos").unwrap_or(0.0);
    let e_pos = data.sensor_scalar(&model, "e_jpos").unwrap_or(0.0);

    // Track maximum drift from initial pose
    let s_drift = (s_pos - SHOULDER_INIT).abs();
    let e_drift = (e_pos - ELBOW_INIT).abs();
    if s_drift > val.max_shoulder_drift {
        val.max_shoulder_drift = s_drift;
    }
    if e_drift > val.max_elbow_drift {
        val.max_elbow_drift = e_drift;
    }

    // Track energy conservation
    let energy = data.total_energy();
    if val.initial_energy.is_none() && data.time > 0.01 {
        val.initial_energy = Some(energy);
    }
    if let Some(e0) = val.initial_energy
        && e0.abs() > 1e-10
    {
        let err_pct = ((energy - e0) / e0).abs() * 100.0;
        if err_pct > val.max_energy_err_pct {
            val.max_energy_err_pct = err_pct;
        }
    }

    // One-time checks
    if !val.bias_match_checked {
        val.bias_match_checked = true;
        // Shoulder torque should be larger (supports both links)
        val.shoulder_gt_elbow = torques.shoulder.abs() > torques.elbow.abs();
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        let checks = vec![
            Check {
                name: "inv == bias (static)",
                pass: torques.bias_max_err < 1e-8,
                detail: format!("max |inv-bias|={:.2e}", torques.bias_max_err),
            },
            Check {
                name: "Shoulder drift < 0.001 rad",
                pass: val.max_shoulder_drift < 0.001,
                detail: format!("max={:.2e} rad", val.max_shoulder_drift),
            },
            Check {
                name: "Elbow drift < 0.001 rad",
                pass: val.max_elbow_drift < 0.001,
                detail: format!("max={:.2e} rad", val.max_elbow_drift),
            },
            Check {
                name: "|shoulder| > |elbow| torque",
                pass: val.shoulder_gt_elbow,
                detail: format!("|{:.3}| vs |{:.3}|", torques.shoulder, torques.elbow),
            },
            Check {
                name: "Energy constant (< 0.1%)",
                pass: val.max_energy_err_pct < 0.1,
                detail: format!("max err={:.4}%", val.max_energy_err_pct),
            },
        ];
        let _ = print_report("Gravity Compensation (t=6s)", &checks);
    }
}
