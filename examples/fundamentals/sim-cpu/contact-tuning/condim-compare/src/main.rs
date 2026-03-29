//! Condim Compare — Contact Dimensionality
//!
//! Three spheres on a 15° tilted plane, each with a different `condim`
//! via `<pair>` overrides:
//!
//! - condim=1 (red)   → frictionless, slides at g*sin(θ)
//! - condim=3 (green)  → sliding friction, rolls at (5/7)*g*sin(θ)
//! - condim=6 (blue)   → rolling friction, decelerates and stops
//!
//! Demonstrates how contact dimensionality changes the number of constraint
//! rows and the resulting physics: frictionless sliding vs rolling without
//! slipping vs rolling resistance.
//!
//! Validates:
//! - condim=1 sphere accelerates (vel > 1 m/s at t=2s)
//! - condim=3 sphere rolls at (5/7)*g*sin(θ)*t ± 15%
//! - condim=6 sphere slower than condim=3 (rolling friction)
//!
//! Run with: `cargo run -p example-contact-condim-compare --release`

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
<mujoco model="condim-compare">
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

    <body name="frictionless" pos="0 -0.2 0.06">
      <freejoint/>
      <geom name="g_fric" type="sphere" size="0.05" mass="0.5"
            friction="1 0.005 0.001" rgba="0.9 0.2 0.2 1"/>
    </body>

    <body name="sliding" pos="0 0 0.06">
      <freejoint/>
      <geom name="g_slide" type="sphere" size="0.05" mass="0.5"
            friction="1 0.005 0.001" rgba="0.2 0.8 0.2 1"/>
    </body>

    <body name="rolling" pos="0 0.2 0.06">
      <freejoint/>
      <geom name="g_roll" type="sphere" size="0.05" mass="0.5"
            friction="1 0.005 0.001" rgba="0.2 0.4 0.9 1"/>
    </body>
  </worldbody>

  <contact>
    <pair geom1="floor" geom2="g_fric" condim="1"
          friction="0 0 0 0 0"/>
    <pair geom1="floor" geom2="g_slide" condim="3"
          friction="1 1 0.005 0.001 0.001"/>
    <pair geom1="floor" geom2="g_roll" condim="6"
          friction="1 1 0.005 1.0 1.0"/>
  </contact>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Condim Compare ===");
    println!("  Three spheres on 15° ramp: condim=1 (red), 3 (green), 6 (blue)");
    println!("  Frictionless slides, condim=3 rolls, condim=6 rolls with resistance");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Condim Compare".into(),
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
                    let v1 = Vector3::new(d.qvel[0], d.qvel[1], d.qvel[2]).norm();
                    let v3 = Vector3::new(d.qvel[6], d.qvel[7], d.qvel[8]).norm();
                    let v6 = Vector3::new(d.qvel[12], d.qvel[13], d.qvel[14]).norm();
                    format!("cd1={v1:.2}  cd3={v3:.2}  cd6={v6:.2} m/s")
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
        "  Model: {} bodies, {} geoms, {} contact pairs\n",
        model.nbody,
        model.ngeom,
        model.contact_pairs.len()
    );

    let mat_fric = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.2, 0.2)));
    let mat_slide =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.8, 0.2)));
    let mat_roll = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.4, 0.9)));
    let mat_floor = materials.add(MetalPreset::BrushedMetal.material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("floor", mat_floor),
            ("g_fric", mat_fric),
            ("g_slide", mat_slide),
            ("g_roll", mat_roll),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.0),
        2.5,
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
    hud.section("Condim Compare");

    let v1 = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]).norm();
    let v3 = Vector3::new(data.qvel[6], data.qvel[7], data.qvel[8]).norm();
    let v6 = Vector3::new(data.qvel[12], data.qvel[13], data.qvel[14]).norm();

    hud.scalar("condim=1 vel (m/s)", v1, 2);
    hud.scalar("condim=3 vel (m/s)", v3, 2);
    hud.scalar("condim=6 vel (m/s)", v6, 2);
    hud.scalar("contacts", data.ncon as f64, 0);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct Validation {
    reported: bool,
}

fn diagnostics(
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<Validation>,
) {
    if harness.reported() && !val.reported {
        val.reported = true;

        let v1 = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]).norm();
        let v3 = Vector3::new(data.qvel[6], data.qvel[7], data.qvel[8]).norm();
        let v6 = Vector3::new(data.qvel[12], data.qvel[13], data.qvel[14]).norm();

        // Rolling without slipping: a = (5/7)*g*sin(θ), v = a*t
        let a_roll = (5.0 / 7.0) * 9.81 * (0.2618_f64).sin();
        let expected_v3 = a_roll * data.time;

        let checks = vec![
            Check {
                name: "condim=1 has velocity",
                pass: v1 > 1.0,
                detail: format!("vel = {:.2} m/s", v1),
            },
            Check {
                name: "condim=3 matches rolling theory",
                pass: (v3 - expected_v3).abs() / expected_v3 < 0.15,
                detail: format!(
                    "vel = {:.2} vs expected {:.2}, err = {:.1}%",
                    v3,
                    expected_v3,
                    (v3 - expected_v3).abs() / expected_v3 * 100.0
                ),
            },
            Check {
                name: "condim=6 slower than condim=3",
                pass: v6 < v3,
                detail: format!("cd6={:.2} < cd3={:.2}", v6, v3),
            },
        ];
        let _ = print_report("Condim Compare (t=5s)", &checks);
    }
}
