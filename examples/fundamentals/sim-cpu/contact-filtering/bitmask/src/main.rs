//! Bitmask — Contype/Conaffinity Filtering
//!
//! Four spheres with different contype/conaffinity bitmask values fall onto a
//! ground plane. Two rest on the ground, two fall through — demonstrating the
//! bitmask AND/OR rule that governs contact filtering.
//!
//! Run with: `cargo run -p example-contact-filtering-bitmask --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_sign_loss,
    clippy::similar_names
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::physics_pos;
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

// ── MJCF Model ────────────────────────────────────────────────────────────

/// Four spheres with different bitmask values, ground plane with defaults (1,1).
///
/// | Sphere    | contype | conaffinity | Ground collision? | Why                        |
/// |-----------|---------|-------------|-------------------|----------------------------|
/// | A "full"  | 1       | 1           | YES               | (1&1)||(1&1) = true        |
/// | B "layer2"| 2       | 2           | NO                | (2&1)||(1&2) = 0||0 = false|
/// | C "off"   | 0       | 0           | NO                | (0&1)||(1&0) = 0||0 = false|
/// | D "type"  | 1       | 0           | YES               | (1&1)||(1&0) = 1||0 = true |
const MJCF: &str = r#"
<mujoco model="bitmask-demo">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>

  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.1" rgba="0.25 0.25 0.25 1"/>

    <body name="full" pos="-0.6 0 1.0">
      <freejoint name="full_free"/>
      <geom name="full_geom" type="sphere" size="0.12" mass="1.0"
            contype="1" conaffinity="1" rgba="0.3 0.5 0.85 1"/>
    </body>

    <body name="layer2" pos="-0.2 0 1.0">
      <freejoint name="layer2_free"/>
      <geom name="layer2_geom" type="sphere" size="0.12" mass="1.0"
            contype="2" conaffinity="2" rgba="0.2 0.75 0.3 1"/>
    </body>

    <body name="off" pos="0.2 0 1.0">
      <freejoint name="off_free"/>
      <geom name="off_geom" type="sphere" size="0.12" mass="1.0"
            contype="0" conaffinity="0" rgba="0.85 0.2 0.2 0.5"/>
    </body>

    <body name="type_only" pos="0.6 0 1.0">
      <freejoint name="type_free"/>
      <geom name="type_geom" type="sphere" size="0.12" mass="1.0"
            contype="1" conaffinity="0" rgba="0.85 0.7 0.2 1"/>
    </body>
  </worldbody>
</mujoco>
"#;

const RADIUS: f64 = 0.12;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Bitmask — Contype/Conaffinity Filtering ===");
    println!("  Blue (1,1)  = rests on ground");
    println!("  Green (2,2) = falls through (different layer)");
    println!("  Red (0,0)   = falls through (disabled)");
    println!("  Gold (1,0)  = rests on ground (contype matches ground's conaffinity)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Bitmask (Contact Filtering)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(5.0)
                .print_every(0.5)
                .display(|m, d| {
                    let az = d.xpos[m.body_id("full").unwrap()].z;
                    let bz = d.xpos[m.body_id("layer2").unwrap()].z;
                    let cz = d.xpos[m.body_id("off").unwrap()].z;
                    let dz = d.xpos[m.body_id("type_only").unwrap()].z;
                    format!("A={az:.2} B={bz:.2} C={cz:.2} D={dz:.2}")
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

// ── Setup ─────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let data = model.make_data();

    let mat_full =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.85)));
    let mat_layer2 =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.75, 0.3)));
    let mat_off = materials.add(StandardMaterial {
        base_color: Color::srgba(0.85, 0.2, 0.2, 0.4),
        alpha_mode: AlphaMode::Blend,
        metallic: 0.3,
        perceptual_roughness: 0.5,
        ..default()
    });
    let mat_type =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.7, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("full_geom", mat_full),
            ("layer2_geom", mat_layer2),
            ("off_geom", mat_off),
            ("type_geom", mat_type),
        ],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 0.5),
        3.5,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Bitmask — Contype/Conaffinity");

    let bodies = ["full", "layer2", "off", "type_only"];
    let labels = [
        "A full (1,1)",
        "B layer2 (2,2)",
        "C off (0,0)",
        "D type (1,0)",
    ];

    for (name, label) in bodies.iter().zip(labels.iter()) {
        let bid = model.body_id(name).expect("body exists");
        let z = data.xpos[bid].z;
        let status = if z > RADIUS - 0.05 {
            "RESTING"
        } else {
            "FALLING"
        };
        hud.raw(format!("{label}: z={z:.3} [{status}]"));
    }

    hud.scalar("ncon", data.ncon as f64, 0);
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    if !harness.reported() || state.reported {
        return;
    }
    state.reported = true;

    let az = data.xpos[model.body_id("full").expect("full")].z;
    let bz = data.xpos[model.body_id("layer2").expect("layer2")].z;
    let cz = data.xpos[model.body_id("off").expect("off")].z;
    let dz = data.xpos[model.body_id("type_only").expect("type_only")].z;

    let tol = 0.03;
    let checks = vec![
        Check {
            name: "A (1,1) rests on ground",
            pass: (az - RADIUS).abs() < tol,
            detail: format!("z = {az:.4}, expected ~{RADIUS}"),
        },
        Check {
            name: "B (2,2) falls through",
            pass: bz < -1.0,
            detail: format!("z = {bz:.4}, expected < -1.0"),
        },
        Check {
            name: "C (0,0) falls through",
            pass: cz < -1.0,
            detail: format!("z = {cz:.4}, expected < -1.0"),
        },
        Check {
            name: "D (1,0) rests on ground",
            pass: (dz - RADIUS).abs() < tol,
            detail: format!("z = {dz:.4}, expected ~{RADIUS}"),
        },
    ];
    let _ = print_report("Bitmask (t=5s)", &checks);
}
