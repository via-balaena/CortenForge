//! Integrator Comparison — 3 Double Pendulums Side by Side
//!
//! Three identical double pendulums, each using a different integrator.
//! The double pendulum is chaotic — tiny numerical differences from the
//! integrator compound exponentially. Within 5-10 seconds the three
//! pendulums are doing completely different things.
//!
//! - **Euler** (orange) — first-order, drifts fastest
//! - **RK4** (blue) — fourth-order, tracks the true trajectory longest
//! - **ImplicitSpringDamper** (gold) — different code path, diverges differently
//!
//! Run with: `cargo run -p example-integrator-comparison-visual --release`

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

const SPACING: f32 = 3.0;

struct IntegratorDef {
    label: &'static str,
    mjcf_name: &'static str,
    tip_color: Color,
}

const INTEGRATORS: [IntegratorDef; 3] = [
    IntegratorDef {
        label: "Euler",
        mjcf_name: "Euler",
        tip_color: Color::srgb(0.85, 0.45, 0.15),
    },
    IntegratorDef {
        label: "RK4",
        mjcf_name: "RK4",
        tip_color: Color::srgb(0.2, 0.6, 0.85),
    },
    IntegratorDef {
        label: "ImplSpDmp",
        mjcf_name: "implicitspringdamper",
        tip_color: Color::srgb(0.85, 0.75, 0.15),
    },
];

fn mjcf(integrator: &str) -> String {
    format!(
        r#"
<mujoco model="double-pendulum-{integrator}">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.005" integrator="{integrator}">
    <flag energy="enable" contact="disable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <geom name="bracket" type="box" size="0.06 0.04 0.03"
          pos="0 0 0" rgba="0.45 0.45 0.48 1"/>
    <body name="link1" pos="0 0 0">
      <joint name="hinge1" type="hinge" axis="0 1 0" damping="0"/>
      <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="socket1" type="sphere" size="0.035"
            pos="0 0 0" rgba="0.35 0.33 0.32 1"/>
      <geom name="rod1" type="capsule" size="0.025"
            fromto="0 0 0  0 0 -1.0" rgba="0.50 0.50 0.53 1"/>
      <geom name="joint_ball" type="sphere" size="0.03"
            pos="0 0 -1.0" rgba="0.35 0.33 0.32 1"/>
      <body name="link2" pos="0 0 -1.0">
        <joint name="hinge2" type="hinge" axis="0 1 0" damping="0"/>
        <inertial pos="0 0 -0.35" mass="0.5" diaginertia="0.005 0.005 0.005"/>
        <geom name="rod2" type="capsule" size="0.02"
              fromto="0 0 0  0 0 -0.7" rgba="0.48 0.48 0.50 1"/>
        <geom name="tip" type="sphere" size="0.05"
              pos="0 0 -0.7"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"#
    )
}

// Initial angles: 120° and 90° — deep in the chaotic regime
const THETA1: f64 = 2.094; // ~120 degrees
const THETA2: f64 = 1.571; // ~90 degrees

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Integrator Comparison (Visual) ===");
    println!("  3 double pendulums: Euler (orange) | RK4 (blue) | ImplSpDmp (gold)");
    println!("  Chaotic system — identical start, watch them diverge");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Integrator Comparison (Double Pendulum)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .add_plugins(MultiScenePlugin)
        .init_resource::<PhysicsHud>()
        .init_resource::<InitialEnergies>()
        .add_systems(Startup, setup)
        .add_systems(PostUpdate, (update_hud, render_physics_hud).chain())
        .run();
}

#[derive(Resource, Default)]
struct InitialEnergies(Vec<f64>);

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut initial: ResMut<InitialEnergies>,
) {
    let mut scenes = PhysicsScenes::default();

    for (i, def) in INTEGRATORS.iter().enumerate() {
        let xml = mjcf(def.mjcf_name);
        let model = sim_mjcf::load_model(&xml).expect("MJCF should parse");
        let mut data = model.make_data();

        // Same initial angles for all three
        data.qpos[0] = THETA1;
        data.qpos[1] = THETA2;
        data.forward(&model).expect("forward should succeed");
        initial.0.push(data.total_energy());

        let id = scenes.add(def.label, model, data);

        let mat_rod1 = materials.add(MetalPreset::BrushedMetal.material());
        let mat_rod2 = materials.add(MetalPreset::BrushedMetal.material());
        let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(def.tip_color));

        let x = (i as f32 - 1.0) * SPACING;
        spawn_scene_geoms(
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut scenes,
            id,
            Vec3::new(x, 0.0, 0.0),
            &[("rod1", mat_rod1), ("rod2", mat_rod2), ("tip", mat_tip)],
        );
    }

    println!("  {} scenes, E₀ = {:.4} J\n", scenes.len(), initial.0[0]);

    commands.insert_resource(scenes);

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, -0.8),
        7.0,
        std::f32::consts::FRAC_PI_2,
        0.2,
    );

    spawn_physics_hud(&mut commands);
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(
    scenes: Res<PhysicsScenes>,
    initial: Res<InitialEnergies>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();
    hud.section("Integrators");

    for (i, scene) in scenes.iter().enumerate() {
        let e0 = initial.0.get(i).copied().unwrap_or(0.0);
        let energy = scene.data.total_energy();
        let drift_j = energy - e0;
        hud.raw(format!(
            "{:<10} E={:+.4}J  dE={:+.4}J",
            scene.label, energy, drift_j
        ));
    }

    if let Some(s) = scenes.get(0) {
        hud.scalar("time", s.data.time, 1);
    }
}
