//! Joint Mimic — 1:1 Joint Coupling
//!
//! Two hinge arms side by side with a joint equality constraint enforcing
//! `j1 = j2` (polycoef="0 1"). They start at different angles (1.2 and -1.0
//! rad). The soft constraint gradually pulls them into sync over several
//! seconds, then they swing in unison under gravity.
//!
//! Demonstrates: `<joint>` equality constraint, polycoef polynomial coupling,
//! mimic joints (1:1 ratio), constraint convergence from mismatched initial
//! conditions.
//!
//! Validates:
//! - Mimic converges (error < 0.1 rad by 3s)
//! - Mimic tracks (stays < 0.1 rad after convergence)
//!
//! Run with: `cargo run -p example-equality-joint-mimic --release`

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
//
// Two hinge arms coupled 1:1. Start at different angles to show convergence.

const MJCF: &str = r#"
<mujoco model="joint-mimic">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001">
    <flag energy="enable"/>
  </option>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="arm1" pos="-0.4 0 2.0">
      <joint name="j1" type="hinge" axis="0 1 0" damping="0"/>
      <geom name="arm1" type="capsule" fromto="0 0 0 0 0 -0.7" size="0.03" mass="1.0"/>
    </body>
    <body name="arm2" pos="0.4 0 2.0">
      <joint name="j2" type="hinge" axis="0 1 0" damping="0"/>
      <geom name="arm2" type="capsule" fromto="0 0 0 0 0 -0.7" size="0.03" mass="1.0"/>
    </body>
  </worldbody>

  <equality>
    <joint joint1="j1" joint2="j2" polycoef="0 1" solref="0.8 0.5"/>
  </equality>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Joint Mimic (1:1) ===");
    println!("  Two hinge arms coupled 1:1 — start at different angles, converge");
    println!("  j1 starts at 1.2 rad, j2 starts at -1.0 rad");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Joint Mimic (1:1)".into(),
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
                    let err = (d.qpos[0] - d.qpos[1]).abs();
                    format!("j1={:.3}  j2={:.3}  err={err:.4}rad", d.qpos[0], d.qpos[1])
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

    // Start at dramatically different angles so convergence is visible
    data.qpos[0] = 1.2; // ~70° forward
    data.qpos[1] = -1.0; // ~57° backward
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} equality constraints\n",
        model.nbody, model.njnt, model.neq
    );

    let mat1 = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.45, 0.15)));
    let mat2 = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.55, 0.25)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("arm1", mat1), ("arm2", mat2)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 1.2, 0.0),
        3.5,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(_model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Joint Mimic (1:1)");

    hud.scalar("j1 (rad)", data.qpos[0], 3);
    hud.scalar("j2 (rad)", data.qpos[1], 3);
    hud.scalar("mimic err (rad)", (data.qpos[0] - data.qpos[1]).abs(), 4);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct Validation {
    converged: bool,
    max_err_after_converge: f64,
    reported: bool,
}

fn diagnostics(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<Validation>,
) {
    let err = (data.qpos[0] - data.qpos[1]).abs();

    if data.time >= 3.0 {
        if !val.converged && err < 0.1 {
            val.converged = true;
        }
        val.max_err_after_converge = val.max_err_after_converge.max(err);
    }

    if harness.reported() && !val.reported {
        val.reported = true;
        let checks = vec![
            Check {
                name: "Mimic converges",
                pass: val.converged,
                detail: "converged by 3s".into(),
            },
            Check {
                name: "Mimic tracks",
                pass: val.max_err_after_converge < 0.1,
                detail: format!("max err = {:.4} rad", val.max_err_after_converge),
            },
        ];
        let _ = print_report("Joint Mimic (t=5s)", &checks);
    }
}
