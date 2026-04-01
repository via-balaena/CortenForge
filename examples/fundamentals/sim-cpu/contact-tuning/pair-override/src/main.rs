//! Pair Override — Explicit Contact Parameter Override
//!
//! Two identical boxes on a 15° tilted plane. Both have geom friction=1.0.
//! The floor has friction=0, so auto-combined friction = MAX(0, 1.0) = 1.0.
//! Both should hold.
//!
//! But one box has an explicit `<pair>` override setting friction to 0.1.
//! This override completely replaces the auto-combined value.
//!
//! - Auto box (green)    → holds (mu=1.0 > tan(15°)=0.268)
//! - Override box (red)  → slides (pair overrides mu to 0.1 < tan(15°))
//!
//! Demonstrates the three-level parameter hierarchy:
//!   explicit `<pair>` > geom combination > defaults
//!
//! Validates:
//! - Auto box holds (vel < 0.05 m/s)
//! - Override box slides (vel > 0.1 m/s)
//!
//! Run with: `cargo run -p example-contact-pair-override --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::uninlined_format_args
)]

use bevy::prelude::*;
use nalgebra::Vector3;
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
<mujoco model="pair-override">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
          iterations="50" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="ramp" euler="0 0.2618 0">
      <geom name="floor" type="plane" size="5 5 0.01"
            friction="0 0 0" rgba="0.35 0.35 0.35 1"/>
    </body>

    <!-- Auto-combined: MAX(1.0, 1.0) = 1.0 → holds -->
    <body name="auto" pos="0 -0.15 0.045">
      <freejoint/>
      <geom name="g_auto" type="box" size="0.04 0.04 0.04" mass="0.5"
            friction="1 0.005 0.001" rgba="0.2 0.8 0.2 1"/>
    </body>

    <!-- Same geom friction, but <pair> override sets mu=0.1 → slides -->
    <body name="over" pos="0 0.15 0.045">
      <freejoint/>
      <geom name="g_over" type="box" size="0.04 0.04 0.04" mass="0.5"
            friction="1 0.005 0.001" rgba="0.9 0.2 0.2 1"/>
    </body>
  </worldbody>

  <contact>
    <pair geom1="floor" geom2="g_over"
          friction="0.1 0.1 0.005 0.001 0.001"/>
  </contact>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Pair Override ===");
    println!("  Two identical boxes on 15° ramp (geom friction=1.0, floor=0)");
    println!("  Green = auto friction mu=1.0 → holds");
    println!("  Red = <pair> override replaces mu to 0.1 → slides");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Pair Override".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<Validation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(5.0)
                .print_every(1.0)
                .display(|_m, d| {
                    let va = Vector3::new(d.qvel[0], d.qvel[1], d.qvel[2]).norm();
                    let vo = Vector3::new(d.qvel[6], d.qvel[7], d.qvel[8]).norm();
                    format!("auto_v={va:.3}  over_v={vo:.2}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                diagnostics,
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
        "  Model: {} bodies, {} contact pairs\n",
        model.nbody,
        model.contact_pairs.len()
    );

    let mat_auto = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.8, 0.2)));
    let mat_over = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.2, 0.2)));
    let mat_floor = materials.add(MetalPreset::BrushedMetal.material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("floor", mat_floor),
            ("g_auto", mat_auto),
            ("g_over", mat_over),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.0),
        2.0,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Pair Override");

    let auto_v = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]).norm();
    let over_v = Vector3::new(data.qvel[6], data.qvel[7], data.qvel[8]).norm();

    hud.scalar("auto vel (m/s)", auto_v, 4);
    hud.scalar("override vel (m/s)", over_v, 2);
    hud.scalar("contacts", data.ncon as f64, 0);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct Validation {
    over_peak_vel: f64,
    reported: bool,
}

fn diagnostics(
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<Validation>,
) {
    let over_v = Vector3::new(data.qvel[6], data.qvel[7], data.qvel[8]).norm();
    val.over_peak_vel = val.over_peak_vel.max(over_v);

    if harness.reported() && !val.reported {
        val.reported = true;

        let auto_v = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]).norm();

        let checks = vec![
            Check {
                name: "Auto box holds (vel < 0.05)",
                pass: auto_v < 0.05,
                detail: format!("vel = {:.4} m/s", auto_v),
            },
            Check {
                name: "Override box slides (peak_vel > 0.5)",
                pass: val.over_peak_vel > 0.5,
                detail: format!("peak_vel = {:.3} m/s", val.over_peak_vel),
            },
        ];
        let _ = print_report("Pair Override (t=5s)", &checks);
    }
}
