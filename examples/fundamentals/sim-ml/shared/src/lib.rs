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
        .build_vec_env(n_envs, 0)
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

// ── 6-DOF arm setup ──────────────────────────────────────────────────────

/// Lane spacing for 6-DOF arms (wider than 2-DOF due to 3 segments).
const SPACING_6DOF: f32 = 1.2;

/// Target joint configuration for 6-DOF reaching (must match task.rs).
const TARGET_JOINTS_6DOF: [f64; 6] = [0.5, 0.2, -0.8, 0.1, 0.5, -0.1];

/// Build a [`VecEnv`] from a 6-DOF [`TaskConfig`] and spawn all shared Bevy
/// entities: 3-segment arm geometry (blue→cyan→orange), green target spheres,
/// camera, HUD, and validation dummies.
///
/// The 6-DOF arm has 3 named capsule geoms: `seg1_geom`, `seg2_geom`,
/// `seg3_geom`, colored steel-blue / cyan / warm-orange respectively.
///
/// # Panics
///
/// Panics if `build_vec_env` fails or if FK computation fails.
pub fn setup_reaching_6dof_arms(
    task: &TaskConfig,
    n_envs: usize,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> (VecEnv, PhysicsScenes) {
    let vec_env = task
        .build_vec_env(n_envs, 0)
        .expect("build_vec_env from TaskConfig");

    let mut scenes = PhysicsScenes::default();
    let model = vec_env.model();

    let mat_seg1 =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.85)));
    let mat_seg2 =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.7, 0.75)));
    let mat_seg3 =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.45, 0.2)));

    // Compute target fingertip position via forward kinematics on a scratch Data
    let target_tip = {
        let mut data = model.make_data();
        for (j, &q) in TARGET_JOINTS_6DOF.iter().enumerate() {
            data.qpos[j] = q;
        }
        data.forward(model).expect("6-DOF FK");
        let tip = data.site_xpos[0]; // fingertip site
        [tip.x as f32, tip.y as f32, tip.z as f32]
    };

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

        let lane = (i as f32 - (n_envs as f32 - 1.0) / 2.0) * SPACING_6DOF;
        spawn_scene_geoms(
            commands,
            meshes,
            materials,
            &mut scenes,
            id,
            physics_pos(lane, 0.0, 0.0),
            &[
                ("seg1_geom", mat_seg1.clone()),
                ("seg2_geom", mat_seg2.clone()),
                ("seg3_geom", mat_seg3.clone()),
            ],
        );

        // Target sphere
        let target_bevy = physics_pos(lane + target_tip[0], target_tip[1], target_tip[2]);
        commands.spawn((
            Mesh3d(target_mesh.clone()),
            MeshMaterial3d(mat_target.clone()),
            Transform::from_translation(target_bevy),
        ));
    }

    // Camera: wider view for 6-DOF arms
    let cam_dist = if n_envs > 20 { 45.0 } else { 15.0 };
    spawn_example_camera(
        commands,
        physics_pos(0.0, -0.3, 0.2),
        cam_dist,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );

    spawn_physics_hud_at(commands, HudPosition::BottomLeft);
    insert_batch_validation_dummies(commands, model);

    (vec_env, scenes)
}
