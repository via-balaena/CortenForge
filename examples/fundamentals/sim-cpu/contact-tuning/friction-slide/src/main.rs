//! Friction Slide — Coulomb Friction on a Tilted Plane
//!
//! Three boxes on a 15° tilted plane with different friction coefficients.
//! Boxes (not spheres) are used because they can't roll — friction directly
//! determines whether they slide or hold.
//!
//! - mu=0.1 (red)   → slides (mu < tan(15°) = 0.268)
//! - mu=0.5 (green)  → holds  (mu > tan(15°))
//! - mu=1.0 (blue)   → holds easily
//!
//! The floor has friction=0 so the combined friction (element-wise MAX)
//! equals the box's friction.
//!
//! Validates:
//! - Low-mu box slides significantly (disp > 0.1m)
//! - Medium and high-mu boxes hold (vel < 0.05 m/s)
//!
//! Run with: `cargo run -p example-contact-friction-slide --release`

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
//
// Three boxes on a 15° tilted plane. Floor friction=0 so MAX(0, box_mu) = box_mu.
// Boxes spaced along Y (horizontal on ramp), placed just above the plane surface.

const MJCF: &str = r#"
<mujoco model="friction-slide">
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

    <body name="low" pos="0 -0.2 0.045">
      <freejoint/>
      <geom name="g_low" type="box" size="0.04 0.04 0.04" mass="0.5"
            friction="0.1 0.005 0.001" rgba="0.9 0.2 0.2 1"/>
    </body>

    <body name="med" pos="0 0 0.045">
      <freejoint/>
      <geom name="g_med" type="box" size="0.04 0.04 0.04" mass="0.5"
            friction="0.5 0.005 0.001" rgba="0.2 0.8 0.2 1"/>
    </body>

    <body name="high" pos="0 0.2 0.045">
      <freejoint/>
      <geom name="g_high" type="box" size="0.04 0.04 0.04" mass="0.5"
            friction="1.0 0.005 0.001" rgba="0.2 0.4 0.9 1"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Friction Slide ===");
    println!("  Three boxes on a 15° ramp: mu=0.1 (red), 0.5 (green), 1.0 (blue)");
    println!("  tan(15°) = 0.268 — only red box slides");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Friction Slide".into(),
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
                    let low_v = Vector3::new(d.qvel[0], d.qvel[1], d.qvel[2]).norm();
                    let med_v = Vector3::new(d.qvel[6], d.qvel[7], d.qvel[8]).norm();
                    format!("low_v={low_v:.2}  med_v={med_v:.3}")
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
        "  Model: {} bodies, {} geoms, ncon after fwd={}\n",
        model.nbody, model.ngeom, data.ncon
    );

    let mat_low = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.9, 0.2, 0.2)));
    let mat_med = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.8, 0.2)));
    let mat_high = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.4, 0.9)));
    let mat_floor = materials.add(MetalPreset::BrushedMetal.material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("floor", mat_floor),
            ("g_low", mat_low),
            ("g_med", mat_med),
            ("g_high", mat_high),
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
    hud.section("Friction Slide");

    let low_v = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]).norm();
    let med_v = Vector3::new(data.qvel[6], data.qvel[7], data.qvel[8]).norm();
    let high_v = Vector3::new(data.qvel[12], data.qvel[13], data.qvel[14]).norm();

    hud.scalar("low mu=0.1 vel (m/s)", low_v, 3);
    hud.scalar("med mu=0.5 vel (m/s)", med_v, 4);
    hud.scalar("high mu=1.0 vel (m/s)", high_v, 4);
    hud.scalar("contacts", data.ncon as f64, 0);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct Validation {
    settled: bool,
    settled_low: nalgebra::Vector3<f64>,
    low_peak_vel: f64,
    reported: bool,
}

fn diagnostics(
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<Validation>,
) {
    // Record settled position after 0.5s
    if data.time > 0.5 && !val.settled {
        val.settled = true;
        val.settled_low = nalgebra::Vector3::new(data.qpos[0], data.qpos[1], data.qpos[2]);
    }

    // Track peak velocity of low-mu box
    if val.settled {
        let v = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]).norm();
        val.low_peak_vel = val.low_peak_vel.max(v);
    }

    if harness.reported() && !val.reported {
        val.reported = true;

        let low_pos = nalgebra::Vector3::new(data.qpos[0], data.qpos[1], data.qpos[2]);
        let low_disp = (low_pos - val.settled_low).norm();
        let med_v = Vector3::new(data.qvel[6], data.qvel[7], data.qvel[8]).norm();
        let high_v = Vector3::new(data.qvel[12], data.qvel[13], data.qvel[14]).norm();

        let checks = vec![
            Check {
                name: "Low-mu slides (disp > 0.1m)",
                pass: low_disp > 0.1,
                detail: format!("disp = {:.3} m", low_disp),
            },
            Check {
                name: "Low-mu had peak velocity > 0.5 m/s",
                pass: val.low_peak_vel > 0.5,
                detail: format!("peak_vel = {:.3} m/s", val.low_peak_vel),
            },
            Check {
                name: "Med-mu holds (vel < 0.05)",
                pass: med_v < 0.05,
                detail: format!("vel = {:.4} m/s", med_v),
            },
            Check {
                name: "High-mu holds (vel < 0.05)",
                pass: high_v < 0.05,
                detail: format!("vel = {:.4} m/s", high_v),
            },
        ];
        let _ = print_report("Friction Slide (t=5s)", &checks);
    }
}
