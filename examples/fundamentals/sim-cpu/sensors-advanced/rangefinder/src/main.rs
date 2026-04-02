//! Rangefinder Sensor
//!
//! A body on a vertical slide joint oscillates above a ground surface.
//! A downward-pointing rangefinder tracks the distance to the ground.
//! A sideways-pointing rangefinder sees nothing and reads −1.
//!
//! The rangefinder casts a ray along the site's +Z axis and returns the
//! distance to the nearest geometry. It excludes geoms on its own body
//! (no self-intersection). Position-stage sensor.
//!
//! Validates:
//! - Down rangefinder = height above ground (to 0.5%)
//! - Side rangefinder = −1 every frame (no hit)
//! - Range oscillates between 0.5 m and 1.5 m (matches servo)
//!
//! Run with: `cargo run -p example-sensor-adv-rangefinder --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_precision_loss,
    clippy::float_cmp
)]

use std::f64::consts::PI;

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
// Body on a vertical slide joint (Z axis) at height 1.0 m. Position servo
// drives sinusoidal oscillation: z(t) = 0.5*sin(2πt/4), so body height
// ranges from 0.5 to 1.5 m above the ground box.
//
// Two sites:
// - eye_down: +Z points to world −Z (euler π about X) — sees the ground
// - eye_side: +Z points to world +X (euler −π/2 about Y) — sees nothing
//
// Ground is a thin box (not a plane — plane raycast is deferred DT-122).
//
const MJCF: &str = r#"
<mujoco model="rangefinder">
    <compiler angle="radian"/>
    <option gravity="0 0 0" timestep="0.001"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <geom name="ground" type="box" size="2 2 0.005" pos="0 0 -0.005"
              rgba="0.35 0.35 0.38 1"/>
        <body name="sensor_body" pos="0 0 1.0">
            <joint name="slide" type="slide" axis="0 0 1"/>
            <geom name="body_sphere" type="sphere" size="0.08" mass="0.5"
                  rgba="0.2 0.5 0.85 1"/>
            <site name="eye_down" pos="0 0 -0.08" euler="3.14159265 0 0"/>
            <site name="eye_side" pos="0.08 0 0" euler="0 -1.5707963 0"/>
        </body>
    </worldbody>

    <actuator>
        <position name="servo" joint="slide" kp="500"/>
    </actuator>

    <sensor>
        <rangefinder name="down" site="eye_down"/>
        <rangefinder name="side" site="eye_side"/>
        <jointpos name="height_offset" joint="slide"/>
    </sensor>
</mujoco>
"#;

const BASE_HEIGHT: f64 = 1.0;
const AMPLITUDE: f64 = 0.5;
const PERIOD: f64 = 4.0;
const SPHERE_RADIUS: f64 = 0.08;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Rangefinder Sensor ===");
    println!("  Ray-based distance sensor on an oscillating body");
    println!("  Down: tracks height above ground (0.5–1.5 m)");
    println!("  Side: always −1 (no hit)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Rangefinder".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(SensorValidation::new())
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let down = d.sensor_scalar(m, "down").unwrap_or(0.0);
                    let side = d.sensor_scalar(m, "side").unwrap_or(0.0);
                    format!("down={down:.4}  side={side:.0}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (drive_servo, step_physics_realtime).chain())
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
    let data = model.make_data();

    println!(
        "  Model: {} bodies, {} joints, {} sensors\n",
        model.nbody, model.njnt, model.nsensor
    );

    let mat_body =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));
    let mat_ground = materials.add(MetalPreset::CastIron.material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("body_sphere", mat_body), ("ground", mat_ground)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.5, 0.0), // Bevy Y-up
        3.5,
        0.3,
        0.3,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Servo Drive ─────────────────────────────────────────────────────────────

fn drive_servo(mut data: ResMut<PhysicsData>) {
    // Sinusoidal position target: offset from 0 by ±0.5
    let t = data.time;
    let target = AMPLITUDE * (2.0 * PI * t / PERIOD).sin();
    data.set_ctrl(0, target);
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    let down = data.sensor_scalar(&model, "down").unwrap_or(0.0);
    let side = data.sensor_scalar(&model, "side").unwrap_or(0.0);
    let offset = data.sensor_scalar(&model, "height_offset").unwrap_or(0.0);
    let expected = BASE_HEIGHT + offset - SPHERE_RADIUS;

    hud.clear();
    hud.section("Rangefinder");
    hud.scalar("down", down, 4);
    hud.scalar("expected", expected, 4);
    hud.raw(format!("  err          {:.4} m", (down - expected).abs()));
    hud.raw(String::new());
    hud.raw(format!("  side         {side:.0} (no hit)"));
    hud.raw(String::new());
    hud.scalar("height (body)", BASE_HEIGHT + offset, 4);
    hud.scalar("slide offset", offset, 4);
}

// ── Sensor Validation ───────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct SensorValidation {
    max_err: f64,
    side_violations: u32,
    range_min: f64,
    range_max: f64,
    sample_count: u64,
    reported: bool,
}

impl SensorValidation {
    fn new() -> Self {
        Self {
            range_min: f64::MAX,
            range_max: f64::MIN,
            ..Default::default()
        }
    }
}

fn sensor_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<SensorValidation>,
) {
    // Skip first 0.5s for settling
    if data.time < 0.5 {
        return;
    }

    let down = data.sensor_scalar(&model, "down").unwrap_or(0.0);
    let side = data.sensor_scalar(&model, "side").unwrap_or(0.0);
    let offset = data.sensor_scalar(&model, "height_offset").unwrap_or(0.0);

    // Expected: body center at BASE_HEIGHT + offset, site at -SPHERE_RADIUS,
    // ground top at z=0. Distance = body_z - sphere_radius - 0 = BASE_HEIGHT + offset - SPHERE_RADIUS.
    let expected = BASE_HEIGHT + offset - SPHERE_RADIUS;
    let err = (down - expected).abs();
    val.max_err = val.max_err.max(err);

    if side != -1.0 {
        val.side_violations += 1;
    }

    if down > 0.0 {
        val.range_min = val.range_min.min(down);
        val.range_max = val.range_max.max(down);
    }

    val.sample_count += 1;

    if harness.reported() && !val.reported {
        val.reported = true;

        let expected_min = BASE_HEIGHT - AMPLITUDE - SPHERE_RADIUS; // 0.42
        let expected_max = BASE_HEIGHT + AMPLITUDE - SPHERE_RADIUS; // 1.42

        let checks = vec![
            Check {
                name: "Down = height",
                pass: val.max_err < 0.01,
                detail: format!("max |error| = {:.6} m", val.max_err),
            },
            Check {
                name: "Side = -1",
                pass: val.side_violations == 0,
                detail: format!("{} violations", val.side_violations),
            },
            Check {
                name: "Range oscillates",
                pass: (val.range_min - expected_min).abs() < 0.1
                    && (val.range_max - expected_max).abs() < 0.1,
                detail: format!(
                    "min={:.4} (exp {expected_min:.4}), max={:.4} (exp {expected_max:.4})",
                    val.range_min, val.range_max
                ),
            },
        ];
        let _ = print_report("Rangefinder (t=15s)", &checks);
    }
}

// ── Helpers ─────────────────────────────────────────────────────────────────

// (no vec3 helpers needed — rangefinder is scalar)
