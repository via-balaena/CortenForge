//! Pendulum Simulation — MuJoCo-Aligned Physics with Bevy Visualization
//!
//! Load a double pendulum from MJCF, step the simulation with CortenForge's
//! MuJoCo-aligned physics engine, and visualize it in real-time with Bevy.
//!
//! Shows: Model/Data architecture, forward kinematics, energy tracking.
//!
//! Run with: `cargo run -p example-pendulum-sim --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use
)]

use bevy::prelude::*;
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::convert::vec3_from_vector;
use sim_bevy::model_data::{ModelBodyIndex, PhysicsData, PhysicsModel, step_model_data};
use sim_core::ENABLE_ENERGY;

/// Double pendulum MJCF definition.
const DOUBLE_PENDULUM_MJCF: &str = r#"
<mujoco model="double_pendulum">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001"/>
    <worldbody>
        <body name="link1" pos="0 0 0">
            <joint name="joint1" type="hinge" axis="0 1 0" damping="0.02"/>
            <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.1 0.1 0.01"/>
            <geom type="capsule" size="0.04" fromto="0 0 0 0 0 -1.0" rgba="0.9 0.3 0.2 1"/>

            <body name="link2" pos="0 0 -1.0">
                <joint name="joint2" type="hinge" axis="0 1 0" damping="0.02"/>
                <inertial pos="0 0 -0.35" mass="0.5" diaginertia="0.05 0.05 0.005"/>
                <geom type="capsule" size="0.03" fromto="0 0 0 0 0 -0.7" rgba="0.2 0.5 0.9 1"/>
                <geom type="sphere" size="0.06" pos="0 0 -0.7" rgba="0.2 0.5 0.9 1"/>
            </body>
        </body>
    </worldbody>
</mujoco>
"#;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<InfoTimer>()
        .add_systems(Startup, setup)
        .add_systems(Update, step_model_data)
        .add_systems(PostUpdate, (sync_bodies, print_info))
        .run();
}

/// Sync body visuals from physics state.
fn sync_bodies(data: Res<PhysicsData>, mut query: Query<(&ModelBodyIndex, &mut Transform)>) {
    for (body_idx, mut transform) in &mut query {
        let i = body_idx.0;
        if i < data.xipos.len() {
            transform.translation = vec3_from_vector(&data.xipos[i]);
        }
        if i < data.xquat.len() {
            let q = &data.xquat[i];
            transform.rotation = Quat::from_xyzw(
                q.coords[1] as f32,
                q.coords[2] as f32,
                q.coords[3] as f32,
                q.coords[0] as f32,
            );
        }
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // ── Physics ─────────────────────────────────────────────────────────
    let mut model = sim_mjcf::load_model(DOUBLE_PENDULUM_MJCF).expect("MJCF should parse");
    model.enableflags |= ENABLE_ENERGY;
    let mut data = model.make_data();

    // Start at an interesting angle
    data.qpos[0] = std::f64::consts::FRAC_PI_3;
    data.qpos[1] = std::f64::consts::FRAC_PI_4;
    let _ = data.forward(&model);

    println!("=== CortenForge: Double Pendulum ===");
    println!(
        "  Model: {} bodies, {} joints, {} DOFs",
        model.nbody, model.njnt, model.nv
    );
    println!(
        "  Initial angles: {:.0}deg, {:.0}deg",
        data.qpos[0].to_degrees(),
        data.qpos[1].to_degrees()
    );
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    // ── Visuals ─────────────────────────────────────────────────────────
    // Link 1 (body index 1)
    commands.spawn((
        ModelBodyIndex(1),
        Mesh3d(meshes.add(Capsule3d::new(0.04, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.3, 0.2),
            metallic: 0.6,
            perceptual_roughness: 0.3,
            ..default()
        })),
        Transform::default(),
    ));

    // Link 2 (body index 2)
    commands.spawn((
        ModelBodyIndex(2),
        Mesh3d(meshes.add(Capsule3d::new(0.03, 0.7))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.5, 0.9),
            metallic: 0.6,
            perceptual_roughness: 0.3,
            ..default()
        })),
        Transform::default(),
    ));

    // Pivot marker
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.06))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Camera
    let orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, 0.0, -0.8))
        .with_distance(5.0)
        .with_angles(0.5, 0.3);
    let mut cam_transform = Transform::default();
    orbit.apply_to_transform(&mut cam_transform);
    commands.spawn((Camera3d::default(), orbit, cam_transform));

    // Lighting
    commands.spawn((
        DirectionalLight {
            illuminance: 12000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Ground
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(3.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.5, 0.5, 0.5, 0.3),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -1.8, 0.0),
    ));

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

#[derive(Resource, Default)]
struct InfoTimer {
    last: f64,
}

fn print_info(data: Res<PhysicsData>, mut timer: ResMut<InfoTimer>) {
    if data.time - timer.last > 2.0 {
        timer.last = data.time;
        let energy = data.energy_kinetic + data.energy_potential;
        println!(
            "t={:.1}s  theta1={:+.1}deg  theta2={:+.1}deg  E={:.4}J",
            data.time,
            data.qpos[0].to_degrees(),
            data.qpos[1].to_degrees(),
            energy,
        );
    }
}
