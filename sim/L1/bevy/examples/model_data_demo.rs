//! Model/Data Architecture Demo
//!
//! This example demonstrates the MuJoCo-style Model/Data architecture with Bevy.
//!
//! Key concepts:
//! - Model (static) contains the kinematic tree, joint definitions, masses
//! - Data (dynamic) contains qpos/qvel as the source of truth
//! - Body poses (xpos/xquat) are COMPUTED from qpos via forward kinematics
//! - xipos (COM position) shows where the mass is — use this for visualization
//!
//! Run with: `cargo run -p sim-bevy --example model_data_demo --release`

#![allow(clippy::needless_pass_by_value)]

use bevy::prelude::*;
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::convert::vec3_from_vector;
use sim_bevy::model_data::{ModelBodyIndex, PhysicsData, PhysicsModel, step_model_data};
use sim_core::{ENABLE_ENERGY, Model};

/// Pendulum length in meters
const PENDULUM_LENGTH: f64 = 1.0;
/// Pendulum mass in kg
const PENDULUM_MASS: f64 = 1.0;
/// Initial angle in radians
const INITIAL_ANGLE: f64 = std::f64::consts::PI / 3.0;

/// Marker for the rod visual connecting pivot to bob.
#[derive(Component)]
struct Rod;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<DebugPrintState>()
        .add_systems(Startup, setup_physics_and_scene)
        .add_systems(Update, step_model_data)
        .add_systems(PostUpdate, (sync_bob_and_rod, debug_info))
        .run();
}

/// Sync bob position from xipos (COM) and update rod visual.
///
/// MuJoCo convention: xpos is the body frame (at the joint), xipos is the
/// center of mass. For pendulum visualization, xipos is the bob position.
fn sync_bob_and_rod(
    data: Res<PhysicsData>,
    mut bobs: Query<(&ModelBodyIndex, &mut Transform), Without<Rod>>,
    mut rods: Query<&mut Transform, With<Rod>>,
) {
    for (body_idx, mut transform) in &mut bobs {
        let idx = body_idx.0;
        if idx < data.xipos.len() {
            // Use xipos (COM position), not xpos (joint position)
            transform.translation = vec3_from_vector(&data.xipos[idx]);
        }
    }

    // Update rod to connect pivot (origin) to bob
    if let Some(bob_pos) = data.xipos.get(1).map(vec3_from_vector) {
        let pivot = Vec3::ZERO;
        for mut transform in rods.iter_mut() {
            let midpoint = (pivot + bob_pos) / 2.0;
            let direction = (bob_pos - pivot).normalize_or_zero();
            let length = (bob_pos - pivot).length();

            transform.translation = midpoint;
            transform.scale = Vec3::new(1.0, length, 1.0);

            if direction.length_squared() > 0.0001 {
                transform.rotation = Quat::from_rotation_arc(Vec3::Y, direction);
            }
        }
    }
}

/// Setup both physics (Model/Data) and visual scene.
fn setup_physics_and_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create physics model using factory and enable energy tracking
    let mut model = Model::n_link_pendulum(1, PENDULUM_LENGTH, PENDULUM_MASS);
    model.enableflags |= ENABLE_ENERGY;
    let mut data = model.make_data();

    // Set initial angle
    data.qpos[0] = INITIAL_ANGLE;

    // Run forward kinematics to compute initial body poses
    let _ = data.forward(&model);

    println!("========================================");
    println!("Model/Data Architecture Demo");
    println!("========================================");
    println!("Model (static):");
    println!("  nbody: {}", model.nbody);
    println!("  njnt:  {}", model.njnt);
    println!("  nq:    {} (position coordinates)", model.nq);
    println!("  nv:    {} (velocity DOFs)", model.nv);
    println!("----------------------------------------");
    println!("Data (dynamic):");
    println!(
        "  qpos[0]: {:.3} rad ({:.1}°)",
        data.qpos[0],
        data.qpos[0].to_degrees()
    );
    println!(
        "  xpos[1]  (joint): [{:.3}, {:.3}, {:.3}]",
        data.xpos[1].x, data.xpos[1].y, data.xpos[1].z
    );
    println!(
        "  xipos[1] (COM):   [{:.3}, {:.3}, {:.3}]",
        data.xipos[1].x, data.xipos[1].y, data.xipos[1].z
    );
    println!("========================================");

    // Spawn bob at COM position (xipos), not body frame (xpos)
    let bob_pos = vec3_from_vector(&data.xipos[1]);

    // Bob (red sphere)
    commands.spawn((
        ModelBodyIndex(1),
        Mesh3d(meshes.add(Sphere::new(0.1))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.2, 0.2),
            metallic: 0.5,
            perceptual_roughness: 0.3,
            ..default()
        })),
        Transform::from_translation(bob_pos),
    ));

    // Rod (connects pivot to bob)
    commands.spawn((
        Rod,
        Mesh3d(meshes.add(Cylinder::new(0.02, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.6, 0.6, 0.6),
            metallic: 0.8,
            perceptual_roughness: 0.2,
            ..default()
        })),
        Transform::default(),
    ));

    // Pivot point (gray sphere at origin)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.08))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Camera with orbit controls (left-drag orbit, right-drag pan, scroll zoom)
    let orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, -0.5, 0.0))
        .with_distance(4.5)
        .with_angles(0.8, 0.3);
    let mut cam_transform = Transform::default();
    orbit.apply_to_transform(&mut cam_transform);
    commands.spawn((Camera3d::default(), orbit, cam_transform));

    // Lighting
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Ground reference plane
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(2.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.5, 0.5, 0.5, 0.3),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -(PENDULUM_LENGTH as f32), 0.0),
    ));

    // Insert physics resources
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

/// Resource to track last print time
#[derive(Resource, Default)]
struct DebugPrintState {
    last_print_sim_time: f64,
}

/// Print debug information periodically
fn debug_info(data: Res<PhysicsData>, mut state: ResMut<DebugPrintState>) {
    if data.time - state.last_print_sim_time > 2.0 {
        state.last_print_sim_time = data.time;

        let qpos = data.qpos[0];
        let qvel = data.qvel[0];
        let energy = data.energy_kinetic + data.energy_potential;

        println!(
            "t={:.1}s  θ={:+.2}rad ({:+.0}°)  ω={:+.2}rad/s  E={:.4}J",
            data.time,
            qpos,
            qpos.to_degrees(),
            qvel,
            energy,
        );
    }
}
