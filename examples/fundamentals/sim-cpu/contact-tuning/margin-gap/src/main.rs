//! Margin / Gap — Contact Activation Distance
//!
//! Three spheres on a flat plane with different `margin` values:
//!
//! - Zero margin (green):  rests at the surface (z ≈ radius)
//! - margin=0.02 (yellow): floats ~20mm above the surface
//! - margin=0.05 (red):    floats ~50mm above the surface
//!
//! `margin` creates a buffer zone around the geom. Contact constraints
//! activate when the distance between geoms is less than `margin`, before
//! physical penetration occurs. The equilibrium position shifts outward
//! by approximately the margin value.
//!
//! `gap` subtracts from margin: `includemargin = margin - gap`. A geom
//! with margin=0.05 and gap=0.03 behaves like margin=0.02.
//!
//! Validates:
//! - Zero-margin ball near surface (z < 0.055)
//! - Mid-margin ball floats higher than zero-margin
//! - High-margin ball floats highest
//!
//! Run with: `cargo run -p example-contact-margin-gap --release`

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

const RADIUS: f64 = 0.05;

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="margin-gap">
  <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
          iterations="50" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.01" rgba="0.35 0.35 0.35 1"/>

    <!-- Zero margin: rests at the surface -->
    <body name="zero" pos="-0.2 0 0.06">
      <freejoint/>
      <geom name="g_zero" type="sphere" size="0.05" mass="0.5"
            rgba="0.2 0.8 0.2 1"/>
    </body>

    <!-- margin=0.02: contact activates 20mm before touching -->
    <body name="mid" pos="0 0 0.08">
      <freejoint/>
      <geom name="g_mid" type="sphere" size="0.05" mass="0.5"
            margin="0.02" rgba="0.9 0.8 0.1 1"/>
    </body>

    <!-- margin=0.05: contact activates 50mm before touching -->
    <body name="high" pos="0.2 0 0.11">
      <freejoint/>
      <geom name="g_high" type="sphere" size="0.05" mass="0.5"
            margin="0.05" rgba="0.9 0.2 0.2 1"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Margin / Gap ===");
    println!("  Three balls with different margin: 0 (green), 0.02 (yellow), 0.05 (red)");
    println!("  Larger margin = ball floats higher above the surface");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Margin / Gap".into(),
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
                    let gap0 = (d.qpos[2] - RADIUS) * 1000.0;
                    let gap1 = (d.qpos[9] - RADIUS) * 1000.0;
                    let gap2 = (d.qpos[16] - RADIUS) * 1000.0;
                    format!("gap0={gap0:.1}mm  gap20={gap1:.1}mm  gap50={gap2:.1}mm")
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

    println!("  Model: {} bodies, {} geoms\n", model.nbody, model.ngeom);

    let mat_zero = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.8, 0.2)));
    let mat_mid = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.8, 0.1)));
    let mat_high = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.2, 0.2)));
    let mat_floor = materials.add(MetalPreset::BrushedMetal.material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("ground", mat_floor),
            ("g_zero", mat_zero),
            ("g_mid", mat_mid),
            ("g_high", mat_high),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.05),
        1.5,
        std::f32::consts::FRAC_PI_6,
        0.1,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Margin / Gap");

    let float_zero = (data.qpos[2] - RADIUS) * 1000.0;
    let float_mid = (data.qpos[9] - RADIUS) * 1000.0;
    let float_high = (data.qpos[16] - RADIUS) * 1000.0;

    hud.scalar("margin=0 gap (mm)", float_zero, 1);
    hud.scalar("margin=0.02 gap (mm)", float_mid, 1);
    hud.scalar("margin=0.05 gap (mm)", float_high, 1);
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

        let z_zero = data.qpos[2];
        let z_mid = data.qpos[9];
        let z_high = data.qpos[16];

        let checks = vec![
            Check {
                name: "Zero-margin near surface",
                pass: z_zero < 0.055,
                detail: format!("z = {:.4}", z_zero),
            },
            Check {
                name: "Mid-margin floats higher",
                pass: z_mid > z_zero + 0.005,
                detail: format!("z_mid={:.4} > z_zero={:.4} + 5mm", z_mid, z_zero),
            },
            Check {
                name: "High-margin floats highest",
                pass: z_high > z_mid + 0.005,
                detail: format!("z_high={:.4} > z_mid={:.4} + 5mm", z_high, z_mid),
            },
        ];
        let _ = print_report("Margin / Gap (t=5s)", &checks);
    }
}
