//! Clock Sensor
//!
//! The simplest possible sensor: reads `data.time` directly. A hinge pendulum
//! provides visual motion while the clock ticks in the console.
//!
//! Validates:
//! - Clock sensor == data.time to machine precision
//! - Clock monotonically increases every frame
//! - Clock > 14.0 at report time (proves time actually advanced)
//!
//! Run with: `cargo run -p example-sensor-clock --release`

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
//
// A pendulum so there's something to watch. The clock sensor is the only thing
// under test — the pendulum just provides visual life.
//
const MJCF: &str = r#"
<mujoco model="clock">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4">
        <flag energy="enable"/>
    </option>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <body name="arm" pos="0 0 0">
            <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
            <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="rod" type="capsule" size="0.02"
                  fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip" type="sphere" size="0.05"
                  pos="0 0 -0.5" rgba="0.2 0.5 0.85 1"/>
        </body>
    </worldbody>

    <sensor>
        <clock name="clock"/>
    </sensor>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Clock Sensor ===");
    println!("  Simplest sensor — reads data.time directly");
    println!("  Pendulum provides visual motion");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Clock Sensor".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<SensorValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let clock = d.sensor_scalar(m, "clock").unwrap_or(0.0);
                    format!("clock={clock:.5}  time={:.5}", d.time)
                })
                .track_energy(0.5),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                sensor_diagnostics,
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
    let mut data = model.make_data();

    // Initial displacement: 45° so the pendulum swings visually
    let qpos_adr = model.jnt_qpos_adr[0];
    data.qpos[qpos_adr] = std::f64::consts::FRAC_PI_4;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} sensors\n",
        model.nbody, model.njnt, model.nsensor
    );

    // ── Materials ────────────────────────────────────────────────────────
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));

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

// ── HUD ────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Clock Sensor");
    let clock = data.sensor_scalar(&model, "clock").unwrap_or(0.0);
    hud.scalar("clock", clock, 5);
    hud.scalar("time", data.time, 5);
    hud.scalar("error", (clock - data.time).abs(), 2);
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    max_error: f64,
    last_clock: f64,
    monotonic_violations: u32,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    let clock_sensor = data.sensor_scalar(&model, "clock").unwrap_or(0.0);
    let time = data.time;

    // Pipeline check: sensor == data.time
    let err = (clock_sensor - time).abs();
    if err > val.max_error {
        val.max_error = err;
    }

    // Monotonicity check
    if clock_sensor < val.last_clock && val.last_clock > 0.0 {
        val.monotonic_violations += 1;
    }
    val.last_clock = clock_sensor;

    // Report when harness fires
    if harness.reported() && !val.reported {
        val.reported = true;
        let checks = vec![
            Check {
                name: "Clock == time",
                pass: val.max_error < 1e-15,
                detail: format!("max error={:.2e}", val.max_error),
            },
            Check {
                name: "Monotonic",
                pass: val.monotonic_violations == 0,
                detail: format!("{} violations", val.monotonic_violations),
            },
            Check {
                name: "Clock > 14s",
                pass: clock_sensor > 14.0,
                detail: format!("clock={clock_sensor:.3}s"),
            },
        ];
        let _ = print_report("Clock Sensor (t=15s)", &checks);
    }
}
