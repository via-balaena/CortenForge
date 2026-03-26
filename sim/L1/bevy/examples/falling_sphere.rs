//! Minimal example: a sphere falling under gravity onto a ground plane.
//!
//! Demonstrates the MJCF → Model/Data → Bevy pipeline:
//! 1. Load MJCF model with ground plane + free-body sphere
//! 2. `spawn_model_geoms` auto-generates visual entities from geom data
//! 3. `ModelDataPlugin` steps physics and syncs geom transforms
//!
//! Run with: `cargo run -p sim-bevy --example falling_sphere --release`

#![allow(clippy::expect_used)]
#![allow(clippy::needless_pass_by_value)]

use bevy::prelude::*;
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::model_data::{ModelDataPlugin, PhysicsData, PhysicsModel};
use sim_bevy::prelude::{load_model, spawn_model_geoms};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(OrbitCameraPlugin)
        .add_plugins(ModelDataPlugin::new().with_auto_step())
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mjcf = r#"
        <mujoco model="falling_sphere">
            <option gravity="0 0 -9.81" timestep="0.002"/>
            <worldbody>
                <geom name="ground" type="plane" size="10 10 0.1" rgba="0.8 0.8 0.8 1"/>
                <body name="sphere" pos="0 0 5">
                    <freejoint name="sphere_joint"/>
                    <geom name="sphere_geom" type="sphere" size="0.5" mass="1.0" rgba="0.2 0.6 1.0 1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("Failed to load MJCF model");
    let mut data = model.make_data();
    let _ = data.forward(&model);

    // Auto-spawn visual entities for all geoms
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[],
    );

    // Camera
    let orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, 2.0, 0.0))
        .with_distance(12.0)
        .with_angles(0.3, 0.3);
    let mut cam_t = Transform::default();
    orbit.apply_to_transform(&mut cam_t);
    commands.spawn((Camera3d::default(), orbit, cam_t));

    // Lighting
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    println!("Falling Sphere — sphere falls from Z=5, contacts ground plane.");

    commands.insert_resource(PhysicsModel::new(model));
    commands.insert_resource(PhysicsData::new(data));
}
