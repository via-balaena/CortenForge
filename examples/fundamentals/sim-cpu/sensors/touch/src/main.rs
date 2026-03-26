//! Touch Sensor
//!
//! A sphere drops from z=0.5 onto a ground plane. Touch sensor on the sphere's
//! body reports the sum of normal contact forces. Uses `condim="1"` (frictionless
//! normal-only contact) to avoid the pyramidal friction approximation which would
//! read ~75% of the true normal force.
//!
//! Two phases:
//! 1. Free-fall (t < 0.2s): touch == 0.0 (no contact)
//! 2. At-rest (t > 3s): touch ≈ m*g = 9.81 N
//!
//! Validates:
//! - Free-fall: touch == 0.0 exactly
//! - At-rest: touch ≈ 9.81 N within 5%
//! - Non-negativity: touch >= 0 every frame
//!
//! Run with: `cargo run -p example-sensor-touch --release`

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
use sim_bevy::examples::{ValidationHarness, spawn_example_camera, validation_system};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="touch">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <worldbody>
        <geom name="floor" type="plane" size="5 5 0.1" condim="1"
              contype="1" conaffinity="1" rgba="0.3 0.3 0.35 1"/>
        <body name="ball_body" pos="0 0 0.5">
            <joint name="free" type="free"/>
            <inertial pos="0 0 0" mass="1.0" diaginertia="0.004 0.004 0.004"/>
            <geom name="ball" type="sphere" size="0.1" condim="1"
                  contype="1" conaffinity="1" rgba="0.2 0.5 0.85 1"/>
            <site name="touch_site" pos="0 0 0"/>
        </body>
    </worldbody>

    <sensor>
        <touch name="touch" site="touch_site"/>
    </sensor>
</mujoco>
"#;

const MASS: f64 = 1.0;
const G: f64 = 9.81;
const EXPECTED_FORCE: f64 = MASS * G;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Touch Sensor ===");
    println!("  Sphere drops onto ground plane (condim=1, frictionless)");
    println!("  Free-fall: touch=0  |  At-rest: touch ≈ {EXPECTED_FORCE:.2} N");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Touch Sensor".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<SensorValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let touch = d.sensor_data(m, 0)[0];
                    let contact = if touch > 0.0 { "yes" } else { "no " };
                    format!(
                        "touch={touch:6.2} N  contact={contact}  expected={EXPECTED_FORCE:.2} N"
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (sync_geom_transforms, validation_system, sensor_diagnostics).chain(),
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
    let mat_ball =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("floor", mat_floor), ("ball", mat_ball)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.2, 0.0),
        2.0,
        std::f32::consts::FRAC_PI_4,
        0.4,
    );

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    freefall_max_touch: f64,
    freefall_sampled: bool,
    rest_touch_sum: f64,
    rest_samples: u32,
    negative_count: u32,
    reported: bool,
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    let touch = data.sensor_data(&model, 0)[0];
    let t = data.time;

    // Non-negativity every frame
    if touch < 0.0 {
        val.negative_count += 1;
    }

    // Free-fall phase: t < 0.2s
    if t > 0.01 && t < 0.2 {
        val.freefall_sampled = true;
        if touch > val.freefall_max_touch {
            val.freefall_max_touch = touch;
        }
    }

    // At-rest phase: t > 3.0s
    if t > 3.0 {
        val.rest_touch_sum += touch;
        val.rest_samples += 1;
    }

    // Report
    if harness.reported() && !val.reported {
        val.reported = true;

        #[allow(clippy::cast_precision_loss)]
        let rest_mean = if val.rest_samples > 0 {
            val.rest_touch_sum / f64::from(val.rest_samples)
        } else {
            0.0
        };
        let rest_err_pct = (rest_mean - EXPECTED_FORCE).abs() / EXPECTED_FORCE * 100.0;

        let checks = vec![
            Check {
                name: "Free-fall touch=0",
                pass: val.freefall_sampled && val.freefall_max_touch < 1e-10,
                detail: format!("max touch={:.2e}", val.freefall_max_touch),
            },
            Check {
                name: "Rest touch ≈ mg",
                pass: rest_err_pct < 5.0,
                detail: format!(
                    "mean={rest_mean:.3} N  expected={EXPECTED_FORCE:.2} N  error={rest_err_pct:.2}%"
                ),
            },
            Check {
                name: "Non-negative",
                pass: val.negative_count == 0,
                detail: format!("{} negative samples", val.negative_count),
            },
        ];
        let _ = print_report("Touch Sensor (t=15s)", &checks);
    }
}
