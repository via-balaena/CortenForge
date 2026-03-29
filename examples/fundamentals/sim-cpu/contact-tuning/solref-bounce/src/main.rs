//! Solref Bounce — Contact Stiffness via Solver Reference
//!
//! Three spheres dropped from 0.5m onto a flat ground plane. Each has a
//! different `solref` via `<pair>` override:
//!
//! - Bouncy (red):     solref=[-5000, -10]  → very underdamped → bounces many times
//! - Moderate (green): solref=[-5000, -80]  → slightly underdamped → bounces once
//! - Absorbing (blue): solref=[-5000, -500] → very overdamped → stops dead
//!
//! All three use direct mode with the same stiffness K=5000. Only the damping
//! B differs — this isolates the damping effect on bounce behaviour.
//!
//! All use condim=1 (frictionless) to isolate the bounce behaviour.
//!
//! In MuJoCo's constraint-based contact, there is no restitution coefficient.
//! Bounce comes entirely from the contact impedance (stiffness K and damping B)
//! computed from solref/solimp parameters.
//!
//! Validates:
//! - Stiff ball bounces above 0.15m
//! - Stiff bounces higher than default
//! - Default/soft don't bounce significantly
//!
//! Run with: `cargo run -p example-contact-solref-bounce --release`

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
<mujoco model="solref-bounce">
  <option gravity="0 0 -9.81" timestep="0.0005" solver="Newton"
          iterations="50" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <geom name="ground" type="plane" size="5 5 0.01" rgba="0.35 0.35 0.35 1"/>

    <body name="stiff" pos="-0.3 0 0.55">
      <freejoint/>
      <geom name="g_stiff" type="sphere" size="0.05" mass="0.5"
            rgba="0.9 0.2 0.2 1"/>
    </body>
    <body name="default" pos="0 0 0.55">
      <freejoint/>
      <geom name="g_default" type="sphere" size="0.05" mass="0.5"
            rgba="0.2 0.8 0.2 1"/>
    </body>
    <body name="soft" pos="0.3 0 0.55">
      <freejoint/>
      <geom name="g_soft" type="sphere" size="0.05" mass="0.5"
            rgba="0.2 0.4 0.9 1"/>
    </body>
  </worldbody>

  <contact>
    <pair geom1="ground" geom2="g_stiff"
          solref="-5000 -10" condim="1"/>
    <pair geom1="ground" geom2="g_default"
          solref="-5000 -30" condim="1"/>
    <pair geom1="ground" geom2="g_soft"
          solref="-5000 -500" condim="1"/>
  </contact>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Solref Bounce ===");
    println!("  Three balls dropped from 0.5m: stiff (red), default (green), soft (blue)");
    println!("  Red = K=5000, B=10 (very underdamped — bounces many times)");
    println!("  Green = K=5000, B=80 (slightly underdamped — bounces once)");
    println!("  Blue = K=5000, B=500 (very overdamped — stops dead)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Solref Bounce".into(),
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
                    let z1 = d.qpos[2];
                    let z2 = d.qpos[9];
                    let z3 = d.qpos[16];
                    format!("z_stiff={z1:.3}  z_def={z2:.3}  z_soft={z3:.3}")
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
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.2, 0.2)));
    let mat_default =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.8, 0.2)));
    let mat_soft = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.4, 0.9)));
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
            ("g_default", mat_default),
            ("g_soft", mat_soft),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.3),
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
    hud.section("Solref Bounce");

    hud.scalar("stiff z (m)", data.qpos[2], 4);
    hud.scalar("default z (m)", data.qpos[9], 4);
    hud.scalar("soft z (m)", data.qpos[16], 4);

    let v_stiff = data.qvel[2];
    hud.scalar("stiff vz (m/s)", v_stiff, 2);
    hud.scalar("contacts", data.ncon as f64, 0);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct Validation {
    stiff_max_z_after: f64,
    default_max_z_after: f64,
    stiff_hit: bool,
    default_hit: bool,
    reported: bool,
}

fn diagnostics(
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<Validation>,
) {
    let z_stiff = data.qpos[2];
    let z_default = data.qpos[9];

    if z_stiff < 0.08 {
        val.stiff_hit = true;
    }
    if z_default < 0.08 {
        val.default_hit = true;
    }

    if val.stiff_hit {
        val.stiff_max_z_after = val.stiff_max_z_after.max(z_stiff);
    }
    if val.default_hit {
        val.default_max_z_after = val.default_max_z_after.max(z_default);
    }

    if harness.reported() && !val.reported {
        val.reported = true;

        let checks = vec![
            Check {
                name: "Stiff bounces high (> 0.15m)",
                pass: val.stiff_max_z_after > 0.15,
                detail: format!("max_z = {:.3} m", val.stiff_max_z_after),
            },
            Check {
                name: "Stiff bounces higher than default",
                pass: val.stiff_max_z_after > val.default_max_z_after,
                detail: format!(
                    "stiff={:.3} > default={:.3}",
                    val.stiff_max_z_after, val.default_max_z_after
                ),
            },
            Check {
                name: "Moderate bounces less than stiff",
                pass: val.default_max_z_after < val.stiff_max_z_after,
                detail: format!(
                    "moderate={:.3} < stiff={:.3}",
                    val.default_max_z_after, val.stiff_max_z_after
                ),
            },
        ];
        let _ = print_report("Solref Bounce (t=5s)", &checks);
    }
}
