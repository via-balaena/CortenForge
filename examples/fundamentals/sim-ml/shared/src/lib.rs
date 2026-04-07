//! Shared Bevy scaffolding for ML visual examples.
//!
//! Extracts the ~150 lines of scene setup duplicated across CEM, REINFORCE,
//! PPO, TD3, and SAC examples: model loading, arm geometry, target spheres,
//! camera, HUD, and validation dummies.

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::needless_pass_by_value
)]

use bevy::prelude::*;
use sim_bevy::convert::physics_pos;
use sim_bevy::examples::{
    HudPosition, insert_batch_validation_dummies, spawn_example_camera, spawn_physics_hud_at,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::multi_scene::{PhysicsScenes, spawn_scene_geoms};
use sim_ml_bridge::{TaskConfig, VecEnv};

// Re-export for callers' sync systems.
pub use sim_bevy::multi_scene::sync_batch_geoms;

/// Lane spacing between adjacent arms (meters).
const SPACING: f32 = 0.8;

/// Target end-effector position for the reaching task (x, y, z).
const TARGET: [f64; 3] = [0.4, 0.0, 0.3];

/// Build a [`VecEnv`] from a [`TaskConfig`] and spawn all shared Bevy
/// entities: arm geometry (blue upper, orange forearm), green target spheres,
/// camera, HUD, and validation dummies.
///
/// Returns `(vec_env, scenes)`. The caller wraps these in their
/// algorithm-specific `Resource` and inserts `scenes` into the world.
///
/// # Panics
///
/// Panics if the task's `build_vec_env` fails or if `forward()` fails
/// on freshly-created scene data.
pub fn setup_reaching_arms(
    task: &TaskConfig,
    n_envs: usize,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> (VecEnv, PhysicsScenes) {
    // 1. Build VecEnv from task config
    let vec_env = task
        .build_vec_env(n_envs)
        .expect("build_vec_env from TaskConfig");

    // 2. Create PhysicsScenes
    let mut scenes = PhysicsScenes::default();
    let model = vec_env.model();

    let mat_upper =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.85)));
    let mat_forearm =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.4, 0.2)));

    let mat_target = materials.add(StandardMaterial {
        base_color: Color::srgba(0.2, 0.9, 0.2, 0.5),
        alpha_mode: AlphaMode::Blend,
        unlit: true,
        ..default()
    });
    let target_mesh = meshes.add(Sphere::new(0.03));

    for i in 0..n_envs {
        let scene_model = model.clone();
        let mut scene_data = scene_model.make_data();
        scene_data.forward(&scene_model).expect("forward");

        let id = scenes.add(format!("env-{i}"), scene_model, scene_data);

        let lane = (i as f32 - (n_envs as f32 - 1.0) / 2.0) * SPACING;
        spawn_scene_geoms(
            commands,
            meshes,
            materials,
            &mut scenes,
            id,
            physics_pos(lane, 0.0, 0.0),
            &[
                ("upper_geom", mat_upper.clone()),
                ("forearm_geom", mat_forearm.clone()),
            ],
        );

        // Target sphere
        let target_bevy = physics_pos(lane + TARGET[0] as f32, 0.0, TARGET[2] as f32);
        commands.spawn((
            Mesh3d(target_mesh.clone()),
            MeshMaterial3d(mat_target.clone()),
            Transform::from_translation(target_bevy),
        ));
    }

    // 3. Camera: front view of the arm row
    spawn_example_camera(
        commands,
        physics_pos(0.0, -0.5, 0.0),
        35.0,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );

    // 4. HUD + validation dummies
    spawn_physics_hud_at(commands, HudPosition::BottomLeft);
    insert_batch_validation_dummies(commands, model);

    (vec_env, scenes)
}
