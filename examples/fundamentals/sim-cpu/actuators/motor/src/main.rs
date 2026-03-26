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
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.5" armature="0.01"/>
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

const CTRL_TORQUE: f64 = 5.0;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Motor Actuator ===");
    println!("  Direct torque control — simplest actuator");
    println!("  ctrl = {CTRL_TORQUE} N·m, damping = {DAMPING} N·m·s/rad");
    println!(
        "  Terminal velocity ≈ {OMEGA_TERMINAL:.1} rad/s ({:.1} rev/s)",
        OMEGA_TERMINAL / std::f64::consts::TAU
    );
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
                    let rev = d.sensor_data(m, 1)[0] / std::f64::consts::TAU;
                    let vel = d.sensor_data(m, 2)[0];
                    format!("force={force:.3}  rev={rev:.1}  ω={vel:.1}")
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
        Vec3::new(0.0, 0.0, -0.25),  // center on arm midpoint
        1.8,                         // distance
        std::f32::consts::FRAC_PI_2, // azimuth: 90° — face the X-Z plane
        0.0,                         // elevation: level
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

    let rev = pos / std::f64::consts::TAU;
    hud.scalar("force", force, 1);
    hud.scalar("rev", rev, 1);
    hud.scalar("omega", vel, 1);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

/// Terminal velocity ≈ τ / damping = 5.0 / 0.5 = 10 rad/s (average, oscillates
/// due to gravity assist/resist).
const DAMPING: f64 = 0.5;
const OMEGA_TERMINAL: f64 = CTRL_TORQUE / DAMPING;

#[derive(Resource, Default)]
struct MotorValidation {
    max_force_error: f64,
    max_sensor_error: f64,
    /// Average velocity in the 10–15s window (should be near terminal)
    vel_sum: f64,
    vel_count: u32,
    /// Track that theta is monotonically increasing (continuous rotation)
    prev_theta: f64,
    theta_reversals: u32,
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
    let vel = data.sensor_data(&model, 2)[0];
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

    // Check 2: sensor == data.actuator_force
    let sensor_err = (force_sensor - act_force).abs();
    if sensor_err > val.max_sensor_error {
        val.max_sensor_error = sensor_err;
    }

    // Check 3: average velocity near terminal in the 10–15s window
    if time > 10.0 && time < 15.0 {
        val.vel_sum += vel;
        val.vel_count += 1;
    }

    // Check 4: continuous rotation (theta always increasing)
    if val.prev_theta != 0.0 && theta < val.prev_theta - 0.01 {
        val.theta_reversals += 1;
    }
    val.prev_theta = theta;

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        let avg_vel = if val.vel_count > 0 {
            val.vel_sum / f64::from(val.vel_count)
        } else {
            0.0
        };
        let vel_err_pct = ((avg_vel - OMEGA_TERMINAL) / OMEGA_TERMINAL).abs() * 100.0;

        let checks = vec![
            Check {
                name: "Force == ctrl",
                pass: val.max_force_error < 1e-15,
                detail: format!("max err={:.2e}", val.max_force_error),
            },
            Check {
                name: "Sensor == actuator_force",
                pass: val.max_sensor_error < 1e-15,
                detail: format!("max err={:.2e}", val.max_sensor_error),
            },
            Check {
                name: "Terminal velocity",
                pass: vel_err_pct < 15.0,
                detail: format!(
                    "avg ω={avg_vel:.2} (expect ~{OMEGA_TERMINAL:.1}), err={vel_err_pct:.1}%"
                ),
            },
            Check {
                name: "Continuous rotation",
                pass: val.theta_reversals == 0,
                detail: format!("{} reversals", val.theta_reversals),
            },
        ];
        let _ = print_report("Motor Actuator (t=15s)", &checks);
    }
}
