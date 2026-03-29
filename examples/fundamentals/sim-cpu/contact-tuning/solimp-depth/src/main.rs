//! Solimp Depth — Impedance Curve and Penetration Depth
//!
//! Two heavy spheres resting on a flat plane with different `solimp`
//! impedance curve parameters via `<pair>` overrides:
//!
//! - Stiff (green):  solimp=[0.9, 0.95, 0.001, 0.5, 2]  → d0=0.9, stiff from first contact
//! - Soft (red):     solimp=[0.1, 0.95, 0.05, 0.5, 2]   → d0=0.1, starts soft, stiffens with depth
//!
//! `solimp` controls how constraint stiffness varies with penetration depth:
//! - d0: impedance at zero violation (low = soft start)
//! - d_width: impedance at full width (endpoint)
//! - width: transition zone (meters)
//! - midpoint, power: sigmoid shape
//!
//! The soft ball sinks ~19mm into the surface; the stiff ball barely penetrates.
//! This is like foam (soft start) vs steel (stiff from contact).
//!
//! Validates:
//! - Soft ball penetrates deeper than stiff
//! - Stiff ball penetration < 5mm
//! - Soft ball penetration > 2mm
//!
//! Run with: `cargo run -p example-contact-solimp-depth --release`

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

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="solimp-depth">
  <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
          iterations="50" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.01" rgba="0.35 0.35 0.35 1"/>

    <!-- Default solimp: d0=0.9 → stiff from first contact -->
    <body name="stiff" pos="-0.15 0 0.06">
      <freejoint/>
      <geom name="g_stiff" type="sphere" size="0.05" mass="2"
            rgba="0.2 0.8 0.2 1"/>
    </body>

    <!-- Soft start: d0=0.1, wide transition (50mm) -->
    <body name="soft" pos="0.15 0 0.06">
      <freejoint/>
      <geom name="g_soft" type="sphere" size="0.05" mass="2"
            rgba="0.9 0.2 0.2 1"/>
    </body>
  </worldbody>

  <contact>
    <pair geom1="ground" geom2="g_stiff"
          solimp="0.9 0.95 0.001 0.5 2" condim="1"/>
    <pair geom1="ground" geom2="g_soft"
          solimp="0.1 0.95 0.05 0.5 2" condim="1"/>
  </contact>
</mujoco>
"#;

const RADIUS: f64 = 0.05;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Solimp Depth ===");
    println!("  Two heavy balls on a plane with different impedance curves");
    println!("  Green (d0=0.9) = stiff from start → minimal penetration");
    println!("  Red (d0=0.1, width=50mm) = soft start → sinks deeper");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Solimp Depth".into(),
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
                    let pen_s = (RADIUS - d.qpos[2]) * 1000.0;
                    let pen_f = (RADIUS - d.qpos[9]) * 1000.0;
                    format!("stiff_pen={pen_s:.2}mm  soft_pen={pen_f:.2}mm")
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

    let mat_stiff =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.8, 0.2)));
    let mat_soft = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.2, 0.2)));
    let mat_floor = materials.add(MetalPreset::BrushedMetal.material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("ground", mat_floor),
            ("g_stiff", mat_stiff),
            ("g_soft", mat_soft),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.0),
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
    hud.section("Solimp Depth");

    let pen_stiff = (RADIUS - data.qpos[2]) * 1000.0;
    let pen_soft = (RADIUS - data.qpos[9]) * 1000.0;

    hud.scalar("stiff pen (mm)", pen_stiff, 2);
    hud.scalar("soft pen (mm)", pen_soft, 2);
    hud.scalar("stiff z (m)", data.qpos[2], 4);
    hud.scalar("soft z (m)", data.qpos[9], 4);
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

        let pen_stiff = RADIUS - data.qpos[2];
        let pen_soft = RADIUS - data.qpos[9];

        let checks = vec![
            Check {
                name: "Soft sinks deeper than stiff",
                pass: pen_soft > pen_stiff,
                detail: format!(
                    "soft={:.2}mm > stiff={:.2}mm",
                    pen_soft * 1000.0,
                    pen_stiff * 1000.0
                ),
            },
            Check {
                name: "Stiff penetration < 5mm",
                pass: pen_stiff < 0.005,
                detail: format!("pen = {:.2} mm", pen_stiff * 1000.0),
            },
            Check {
                name: "Soft penetration > 2mm",
                pass: pen_soft > 0.002,
                detail: format!("pen = {:.2} mm", pen_soft * 1000.0),
            },
        ];
        let _ = print_report("Solimp Depth (t=5s)", &checks);
    }
}
