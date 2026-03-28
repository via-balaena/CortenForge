//! Solver Comparison — 3 Stacks Side by Side
//!
//! Three identical two-box stacks, each using a different constraint solver.
//! The stacks look identical — the difference is in the HUD: iteration count
//! per solver shows Newton converging in 0-2 iterations while PGS needs ~47.
//!
//! - **PGS** (left) — Projected Gauss-Seidel, ~47 iterations
//! - **CG** (center) — Conjugate Gradient, ~31 iterations
//! - **Newton** (right) — Full Newton, ~0 iterations
//!
//! Run with: `cargo run -p example-solver-comparison-visual --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, render_physics_hud, spawn_example_camera, spawn_physics_hud};
use sim_bevy::materials::MetalPreset;
use sim_bevy::multi_scene::{MultiScenePlugin, PhysicsScenes, spawn_scene_geoms};

// ── Config ──────────────────────────────────────────────────────────────────

const SPACING: f32 = 0.6;

struct SolverDef {
    label: &'static str,
    mjcf_name: &'static str,
    box_a_color: Color,
}

const SOLVERS: [SolverDef; 3] = [
    SolverDef {
        label: "PGS",
        mjcf_name: "PGS",
        box_a_color: Color::srgb(0.82, 0.22, 0.15),
    },
    SolverDef {
        label: "CG",
        mjcf_name: "CG",
        box_a_color: Color::srgb(0.75, 0.55, 0.15),
    },
    SolverDef {
        label: "Newton",
        mjcf_name: "Newton",
        box_a_color: Color::srgb(0.15, 0.65, 0.30),
    },
];

fn mjcf(solver: &str) -> String {
    format!(
        r#"
<mujoco model="solver-{solver}">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" solver="{solver}"
          iterations="100" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <geom name="ground" type="plane" size="2 2 0.01"
          friction="0.5 0.005 0.001" rgba="0.35 0.35 0.38 1"/>

    <body name="box_a" pos="0 0 0.1">
      <joint type="free" name="jnt_a"/>
      <geom name="box_a" type="box" size="0.1 0.1 0.1" mass="1.0"
            friction="0.5 0.005 0.001" rgba="0.82 0.22 0.15 1"/>
    </body>

    <body name="box_b" pos="0 0 0.3">
      <joint type="free" name="jnt_b"/>
      <geom name="box_b" type="box" size="0.1 0.1 0.1" mass="0.5"
            friction="0.5 0.005 0.001" rgba="0.15 0.45 0.82 1"/>
    </body>
  </worldbody>
</mujoco>
"#
    )
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Solver Comparison (Visual) ===");
    println!("  3 stacks: PGS (left) | CG (center) | Newton (right)");
    println!("  Same scene, different solvers — watch the iteration counts");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Solver Comparison".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .add_plugins(MultiScenePlugin)
        .init_resource::<PhysicsHud>()
        .add_systems(Startup, setup)
        .add_systems(PostUpdate, (update_hud, render_physics_hud).chain())
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut scenes = PhysicsScenes::default();

    for (i, def) in SOLVERS.iter().enumerate() {
        let xml = mjcf(def.mjcf_name);
        let model = sim_mjcf::load_model(&xml).expect("MJCF should parse");
        let mut data = model.make_data();
        data.forward(&model).expect("forward should succeed");

        let id = scenes.add(def.label, model, data);

        let mat_ground = materials.add(MetalPreset::BrushedMetal.material());
        let mat_box_a = materials.add(MetalPreset::PolishedSteel.with_color(def.box_a_color));
        let mat_box_b =
            materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.15, 0.45, 0.82)));

        let x = (i as f32 - 1.0) * SPACING;
        spawn_scene_geoms(
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut scenes,
            id,
            Vec3::new(x, 0.0, 0.0),
            &[
                ("ground", mat_ground),
                ("box_a", mat_box_a),
                ("box_b", mat_box_b),
            ],
        );
    }

    println!("  {} scenes loaded\n", scenes.len());

    commands.insert_resource(scenes);

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.15),
        2.0,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );

    spawn_physics_hud(&mut commands);
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(scenes: Res<PhysicsScenes>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Solvers");

    for scene in scenes.iter() {
        hud.raw(format!(
            "{:<8} iter={:<3} ncon={}",
            scene.label, scene.data.solver_niter, scene.data.ncon,
        ));
    }

    if let Some(s) = scenes.get(0) {
        hud.scalar("energy", s.data.total_energy(), 4);
        hud.scalar("time", s.data.time, 1);
    }
}
