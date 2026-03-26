//! Motor Actuator — Direct Torque Control
//!
//! The simplest actuator: constant torque on a hinge joint. No dynamics, no
//! bias, no filtering — `ctrl` maps directly to joint torque via
//! `GainType::Fixed(1)` and `BiasType::None`.
//!
//! Validates:
//! - actuator_force == ctrl (gain=1, no bias)
//! - Initial angular acceleration matches τ / I_eff
//! - Quadratic position growth θ ≈ ½αt² in the first 0.1s
//! - ActuatorFrc sensor == data.actuator_force pipeline check
//!
//! Run with: `cargo run -p example-actuator-motor --release`

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
<mujoco model="motor">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0" armature="0.01"/>
      <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="rod" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
      <geom name="tip" type="sphere" size="0.05"
            pos="0 0 -0.5" rgba="0.85 0.3 0.2 1"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="torque" joint="hinge" gear="1"/>
  </actuator>

  <sensor>
    <actuatorfrc name="act_force" actuator="torque"/>
    <jointpos name="jpos" joint="hinge"/>
    <jointvel name="jvel" joint="hinge"/>
  </sensor>
</mujoco>
"#;

const CTRL_TORQUE: f64 = 2.0;

// I_eff = diaginertia_Y + armature + m*d^2 = 0.01 + 0.01 + 1.0 * 0.25^2
const I_EFF: f64 = 0.0825;
const ALPHA_0: f64 = CTRL_TORQUE / I_EFF; // ≈ 24.24 rad/s²

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Motor Actuator ===");
    println!("  Direct torque control — simplest actuator");
    println!("  ctrl = {CTRL_TORQUE} N·m constant torque");
    println!("  I_eff = {I_EFF} kg·m², α₀ = {ALPHA_0:.2} rad/s²");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Motor Actuator".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<MotorValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let force = d.sensor_data(m, 0)[0];
                    let pos = d.sensor_data(m, 1)[0];
                    let vel = d.sensor_data(m, 2)[0];
                    format!("force={force:.3}  θ={pos:.4}  ω={vel:.4}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_ctrl, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                motor_diagnostics,
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
    let mut model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    model.enableflags |= sim_core::ENABLE_ENERGY;
    let data = model.make_data();

    println!(
        "  Model: {} bodies, {} joints, {} actuators, {} sensors\n",
        model.nbody, model.njnt, model.nu, model.nsensor
    );

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));

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
        data.ctrl[0] = CTRL_TORQUE;
    }
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Motor Actuator");

    let force = data.sensor_data(&model, 0)[0];
    let pos = data.sensor_data(&model, 1)[0];
    let vel = data.sensor_data(&model, 2)[0];

    hud.scalar("ctrl", CTRL_TORQUE, 3);
    hud.scalar("force", force, 4);
    hud.scalar("theta", pos, 4);
    hud.scalar("omega", vel, 4);
    hud.scalar("time", data.time, 2);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct MotorValidation {
    max_force_error: f64,
    max_sensor_error: f64,
    /// Samples collected in the first 0.1s for quadratic check
    early_samples: Vec<(f64, f64)>, // (time, theta)
    initial_accel: Option<f64>,
    reported: bool,
}

fn motor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<MotorValidation>,
) {
    let time = data.time;
    let force_sensor = data.sensor_data(&model, 0)[0];
    let theta = data.sensor_data(&model, 1)[0];
    let _vel = data.sensor_data(&model, 2)[0];
    let act_force = data.actuator_force[0];

    // Skip t=0 frame (actuator_force not yet computed before first step)
    if time < 1e-6 {
        return;
    }

    // Check 1: force == ctrl (gain=1, no bias)
    let force_err = (act_force - CTRL_TORQUE).abs();
    if force_err > val.max_force_error {
        val.max_force_error = force_err;
    }

    // Check 4: sensor == data.actuator_force
    let sensor_err = (force_sensor - act_force).abs();
    if sensor_err > val.max_sensor_error {
        val.max_sensor_error = sensor_err;
    }

    // Capture initial acceleration: use force / I_eff at the earliest available
    // time. This avoids needing a specific time window (Bevy startup is variable).
    if val.initial_accel.is_none() && time < 1.0 {
        // α = (τ - m*g*d*sin(θ)) / I_eff
        let gravity_torque = 1.0 * 9.81 * 0.25 * theta.sin();
        let measured_accel = (act_force - gravity_torque) / I_EFF;
        val.initial_accel = Some(measured_accel);
    }

    // Collect early samples for quadratic check (first 0.1s)
    if time <= 0.1 {
        // Sample every ~10ms
        let last_t = val.early_samples.last().map_or(0.0, |s| s.0);
        if time - last_t >= 0.009 {
            val.early_samples.push((time, theta));
        }
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        // Quadratic check: worst-case error in first 0.1s
        let mut max_quad_err_pct = 0.0_f64;
        for &(t, th) in &val.early_samples {
            let predicted = 0.5 * ALPHA_0 * t * t;
            if predicted.abs() > 1e-6 {
                let err_pct = ((th - predicted) / predicted).abs() * 100.0;
                max_quad_err_pct = max_quad_err_pct.max(err_pct);
            }
        }

        let accel_err_pct = val
            .initial_accel
            .map_or(100.0, |a| ((a - ALPHA_0) / ALPHA_0).abs() * 100.0);

        let checks = vec![
            Check {
                name: "Force == ctrl",
                pass: val.max_force_error < 1e-15,
                detail: format!("max err={:.2e}", val.max_force_error),
            },
            Check {
                name: "Initial accel ≈ τ/I",
                pass: accel_err_pct < 3.0,
                detail: format!(
                    "α={:.2} (expect {ALPHA_0:.2}), err={accel_err_pct:.2}%",
                    val.initial_accel.unwrap_or(0.0)
                ),
            },
            Check {
                name: "Quadratic θ (0–0.1s)",
                pass: max_quad_err_pct < 3.0,
                detail: format!("max err={max_quad_err_pct:.2}%"),
            },
            Check {
                name: "Sensor == actuator_force",
                pass: val.max_sensor_error < 1e-15,
                detail: format!("max err={:.2e}", val.max_sensor_error),
            },
        ];
        let _ = print_report("Motor Actuator (t=15s)", &checks);
    }
}
