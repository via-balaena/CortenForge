//! Velocity Servo Actuator — Velocity Tracking
//!
//! Velocity shortcut: `GainType::Fixed(kv)`, `BiasType::Affine(0, 0, -kv)`,
//! `ActuatorDynamics::None`. The resulting force is `kv * (ctrl - qdot)`.
//!
//! With gravity disabled and a constant ctrl = 2.0 rad/s, the joint velocity
//! follows an exponential approach: ω(t) = TARGET_VEL * (1 - e^(-t/τ_v))
//! where τ_v = I_eff / kv.
//!
//! Validates:
//! - Reaches target ω ≈ 2.0 rad/s after 0.5s
//! - Exponential approach: ω ≈ 0.632 * TARGET_VEL at t ≈ τ_v
//! - Force at start ≈ kv * ctrl = 2.0 N·m
//! - Force at steady state ≈ 0 N·m (velocity reached)
//!
//! Run with: `cargo run -p example-actuator-velocity-servo --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
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
<mujoco model="velocity-servo">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" armature="0.01"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
      <geom name="tip" type="sphere" size="0.05"
            pos="0 0 -0.5" rgba="0.3 0.4 0.9 1"/>
    </body>
  </worldbody>

  <actuator>
    <velocity name="vel_servo" joint="hinge" kv="1"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="vel_servo"/>
    <jointvel name="jvel" joint="hinge"/>
  </sensor>
</mujoco>
"#;

// I_eff = diaginertia_Y + armature + m*d^2 = 0.01 + 0.01 + 1.0 * 0.25^2
const I_EFF: f64 = 0.0825;
const KV: f64 = 1.0;
const TARGET_VEL: f64 = 2.0;
const TAU_V: f64 = I_EFF / KV; // 0.0825 s (time constant)

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Velocity Servo ===");
    println!("  Velocity tracking — force = kv * (ctrl - qdot)");
    println!("  ctrl = {TARGET_VEL} rad/s target velocity");
    println!("  I_eff = {I_EFF} kg·m², kv = {KV}, τ_v = {TAU_V:.4} s");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Velocity Servo".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<VelocityServoValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let force = d.sensor_data(m, 0)[0];
                    let vel = d.sensor_data(m, 1)[0];
                    format!("force={force:.3}  ω={vel:.4}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_ctrl, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                velocity_servo_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let data = model.make_data();

    println!(
        "  Model: {} bodies, {} joints, {} actuators, {} sensors\n",
        model.nbody, model.njnt, model.nu, model.nsensor
    );

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.4, 0.9)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("rod", mat_rod), ("tip", mat_tip)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -0.2, 0.0),
        1.8,
        std::f32::consts::FRAC_PI_4,
        0.35,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Control ─────────────────────────────────────────────────────────────────

fn apply_ctrl(mut data: ResMut<PhysicsData>) {
    if !data.ctrl.is_empty() {
        data.ctrl[0] = TARGET_VEL;
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Velocity Servo");

    let force = data.sensor_data(&model, 0)[0];
    let vel = data.sensor_data(&model, 1)[0];

    hud.scalar("ctrl", TARGET_VEL, 3);
    hud.scalar("force", force, 4);
    hud.scalar("omega", vel, 4);
    hud.scalar("time", data.time, 2);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct VelocityServoValidation {
    earliest_force: Option<f64>,
    steady_state_omega: Option<f64>,
    steady_state_force: Option<f64>,
    /// Sample closest to t = TAU_V for exponential check
    tau_sample: Option<(f64, f64)>, // (time, omega)
    reported: bool,
}

fn velocity_servo_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<VelocityServoValidation>,
) {
    let time = data.time;
    let force = data.sensor_data(&model, 0)[0];
    let omega = data.sensor_data(&model, 1)[0];

    // Skip t=0 frame (actuator_force not yet computed before first step)
    if time < 1e-6 {
        return;
    }

    // Track earliest force sample
    if val.earliest_force.is_none() {
        val.earliest_force = Some(force);
    }

    // Track sample closest to TAU_V for exponential check
    match val.tau_sample {
        None => {
            val.tau_sample = Some((time, omega));
        }
        Some((prev_t, _)) => {
            if (time - TAU_V).abs() < (prev_t - TAU_V).abs() {
                val.tau_sample = Some((time, omega));
            }
        }
    }

    // Track steady-state values (after 0.5s, keep updating to latest)
    if time > 0.5 {
        val.steady_state_omega = Some(omega);
        val.steady_state_force = Some(force);
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        let ss_omega = val.steady_state_omega.unwrap_or(0.0);
        let ss_force = val.steady_state_force.unwrap_or(f64::MAX);
        let start_force = val.earliest_force.unwrap_or(0.0);
        let (tau_t, tau_omega) = val.tau_sample.unwrap_or((0.0, 0.0));
        let expected_tau_omega = 0.632 * TARGET_VEL;
        let tau_err_pct = ((tau_omega - expected_tau_omega) / expected_tau_omega).abs() * 100.0;

        let checks = vec![
            Check {
                name: "Reaches target ω",
                pass: (ss_omega - TARGET_VEL).abs() < 0.05,
                detail: format!("ω={ss_omega:.4} (target={TARGET_VEL})"),
            },
            Check {
                name: "Exponential approach at τ",
                pass: tau_err_pct < 10.0,
                detail: format!(
                    "ω({tau_t:.4})={tau_omega:.4} (expect {expected_tau_omega:.4}), err={tau_err_pct:.2}%"
                ),
            },
            Check {
                name: "Force at start ≈ kv*ctrl",
                pass: (start_force - KV * TARGET_VEL).abs() < 0.8,
                detail: format!("force={start_force:.4} (expect {:.1})", KV * TARGET_VEL),
            },
            Check {
                name: "Force at steady state ≈ 0",
                pass: ss_force.abs() < 0.05,
                detail: format!("force={ss_force:.4}"),
            },
        ];
        let _ = print_report("Velocity Servo (t=15s)", &checks);
    }
}
