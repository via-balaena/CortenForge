//! Accelerometer Sensor
//!
//! A box on a free joint hovers at z=4.0 for 1 second, then drops onto a
//! ground plane. Two phases after the drop:
//!
//! 1. Free-fall (t < 1.2s): accelerometer ≈ [0, 0, 0] (weightless)
//! 2. At-rest (t > 5s): accelerometer ≈ [0, 0, +9.81] (proper acceleration
//!    from ground reaction force — the classic IMU-at-rest test)
//!
//! Uses `condim="1"` (frictionless normal-only contact) to avoid pyramidal
//! friction forces that would produce spurious lateral accelerations.
//!
//! Validates:
//! - Free-fall: |accel_z| < 0.5 m/s² (near-zero)
//! - At-rest: accel_z ≈ +9.81 within 2%
//! - At-rest: |accel| magnitude in [9.5, 10.1]
//!
//! Run with: `cargo run -p example-sensor-accelerometer --release`

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
<mujoco model="accelerometer">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <worldbody>
        <geom name="floor" type="plane" size="5 5 0.1"
              contype="1" conaffinity="1" condim="1" rgba="0.3 0.3 0.35 1"/>
        <body name="box" pos="0 0 4.0">
            <joint name="free" type="free"/>
            <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="cube" type="box" size="0.1 0.1 0.1"
                  contype="1" conaffinity="1" condim="1" rgba="0.82 0.22 0.15 1"/>
            <site name="imu" pos="0 0 0"/>
        </body>
    </worldbody>

    <sensor>
        <accelerometer name="accel" site="imu"/>
    </sensor>
</mujoco>
"#;

const G: f64 = 9.81;
const START_DELAY: f32 = 1.0; // seconds before physics starts (visual pause)

/// Run condition: physics starts after a 1-second visual delay.
fn after_start_delay(time: Res<Time>) -> bool {
    time.elapsed_secs() > START_DELAY
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Accelerometer Sensor ===");
    println!("  Box hovers at z=4.0 for 1s, then drops onto ground plane");
    println!("  Free-fall: accel ≈ 0  |  At-rest: accel ≈ [0, 0, +9.81]");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Accelerometer".into(),
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
                .print_every(0.5)
                .display(|m, d| {
                    let a = d.sensor_data(m, 0);
                    let mag = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
                    format!(
                        "accel=({:+6.2},{:+6.2},{:+6.2})  |a|={mag:.2}  expected={G:.2}",
                        a[0], a[1], a[2],
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime.run_if(after_start_delay))
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
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} sensors\n",
        model.nbody, model.njnt, model.nsensor
    );

    // ── Materials ────────────────────────────────────────────────────────
    let mat_floor = materials.add(MetalPreset::BrushedMetal.material());
    let mat_cube =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.82, 0.22, 0.15)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("floor", mat_floor), ("cube", mat_cube)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 2.0, 0.0),
        8.0,
        std::f32::consts::FRAC_PI_4,
        0.25,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    let a = data.sensor_data(&model, 0);
    let mag = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    let phase = if mag < 1.0 { "FREE-FALL" } else { "AT REST" };
    hud.section(&format!("Accelerometer — {phase}"));
    hud.raw(String::new());
    hud.vec3("accel", a, 3);
    hud.raw(String::new());
    hud.scalar("|a|", mag, 3);
    hud.raw(String::new());
    hud.scalar("expected", G, 2);
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    freefall_max_az: f64,
    freefall_sampled: bool,
    rest_az_sum: f64,
    rest_ax_max: f64,
    rest_ay_max: f64,
    rest_samples: u32,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    let accel = data.sensor_data(&model, 0);
    let t = data.time;

    // Free-fall phase: t < 0.7s (impact at ~0.89s from z=4.0, leave margin)
    if t > 0.01 && t < 0.7 {
        val.freefall_sampled = true;
        let az_abs = accel[2].abs();
        if az_abs > val.freefall_max_az {
            val.freefall_max_az = az_abs;
        }
    }

    // At-rest phase: t > 5.0s (plenty of time to settle from z=4.0)
    if t > 5.0 {
        val.rest_az_sum += accel[2];
        val.rest_samples += 1;
        if accel[0].abs() > val.rest_ax_max {
            val.rest_ax_max = accel[0].abs();
        }
        if accel[1].abs() > val.rest_ay_max {
            val.rest_ay_max = accel[1].abs();
        }
    }

    // Report
    if harness.reported() && !val.reported {
        val.reported = true;

        #[allow(clippy::cast_precision_loss)]
        let rest_az_mean = if val.rest_samples > 0 {
            val.rest_az_sum / f64::from(val.rest_samples)
        } else {
            0.0
        };
        let rest_mag =
            (val.rest_ax_max.powi(2) + val.rest_ay_max.powi(2) + rest_az_mean.powi(2)).sqrt();

        let checks = vec![
            Check {
                name: "Free-fall |az| < 0.5",
                pass: val.freefall_sampled && val.freefall_max_az < 0.5,
                detail: format!("max |az|={:.3} m/s²", val.freefall_max_az),
            },
            Check {
                name: "Rest az ≈ +9.81",
                pass: (rest_az_mean - G).abs() / G < 0.02,
                detail: format!(
                    "mean az={rest_az_mean:.3}  error={:.2}%",
                    (rest_az_mean - G).abs() / G * 100.0
                ),
            },
            Check {
                name: "Rest |a| in [9.5, 10.1]",
                pass: rest_mag > 9.5 && rest_mag < 10.1,
                detail: format!("|a|={rest_mag:.3} m/s²"),
            },
        ];
        let _ = print_report("Accelerometer (t=15s)", &checks);
    }
}
