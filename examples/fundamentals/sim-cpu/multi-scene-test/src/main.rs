//! Multi-Scene Visual Test
//!
//! Three identical pendulums side by side, each in its own physics scene.
//! Proves that MultiScenePlugin spawning, lockstep stepping, and per-scene
//! offset sync all work correctly.
//!
//! Expected: three pendulums swinging in perfect unison, spaced 1.5 units apart.
//!
//! Run with: `cargo run -p example-multi-scene-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, render_physics_hud, spawn_example_camera, spawn_physics_hud};
use sim_bevy::materials::MetalPreset;
use sim_bevy::multi_scene::{MultiScenePlugin, PhysicsScenes, spawn_scene_geoms};

const MJCF: &str = r#"
<mujoco model="pendulum">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.002"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <body name="arm" pos="0 0 0">
            <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
            <inertial pos="0 0 -0.25" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="rod" type="capsule" size="0.02"
                  fromto="0 0 0  0 0 -0.5" rgba="0.48 0.48 0.50 1"/>
            <geom name="tip" type="sphere" size="0.05"
                  pos="0 0 -0.5" rgba="0.2 0.5 0.85 1"/>
        </body>
    </worldbody>
</mujoco>
"#;

const LABELS: [&str; 3] = ["Left", "Center", "Right"];
const SPACING: f32 = 1.5;

fn main() {
    println!("=== CortenForge: Multi-Scene Visual Test ===");
    println!("  3 identical pendulums, each in its own physics scene");
    println!("  They should swing in perfect unison, spaced {SPACING} apart");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Multi-Scene Test".into(),
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

    // Three colors so you can tell them apart
    let tip_colors = [
        Color::srgb(0.85, 0.2, 0.2), // red
        Color::srgb(0.2, 0.75, 0.2), // green
        Color::srgb(0.2, 0.5, 0.85), // blue
    ];

    for (i, label) in LABELS.iter().enumerate() {
        let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
        let mut data = model.make_data();

        // Same initial displacement for all — they should stay in sync
        let qpos_adr = model.jnt_qpos_adr[0];
        data.qpos[qpos_adr] = std::f64::consts::FRAC_PI_4;
        let _ = data.forward(&model);

        let id = scenes.add(*label, model, data);

        let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
        let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(tip_colors[i]));

        // Center the group: offsets at -1.5, 0, +1.5
        let x = (i as f32 - 1.0) * SPACING;
        spawn_scene_geoms(
            &mut commands,
            &mut meshes,
            &mut materials,
            id,
            scenes.get(id).expect("just added"),
            Vec3::new(x, 0.0, 0.0),
            &[("rod", mat_rod), ("tip", mat_tip)],
        );
    }

    // Store offsets for sync
    for (i, _) in LABELS.iter().enumerate() {
        let x = (i as f32 - 1.0) * SPACING;
        scenes.get_mut(i).expect("just added").offset = Vec3::new(x, 0.0, 0.0);
    }

    commands.insert_resource(scenes);

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -0.2, 0.0),
        3.5,
        std::f32::consts::FRAC_PI_4,
        0.35,
    );

    spawn_physics_hud(&mut commands);
}

fn update_hud(scenes: Res<PhysicsScenes>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Multi-Scene Test");
    for (i, scene) in scenes.iter().enumerate() {
        let angle = if scene.data.qpos.is_empty() {
            0.0
        } else {
            scene.data.qpos[0]
        };
        hud.scalar(&format!("{} angle", LABELS[i]), angle, 4);
    }
    if let Some(s) = scenes.get(0) {
        hud.scalar("time", s.data.time, 3);
    }
}
