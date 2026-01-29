//! Spherical Pendulum using Model/Data Architecture
//!
//! This example demonstrates a 3-DOF spherical pendulum using the unified
//! Model/Data architecture with a ball joint. It showcases:
//!
//! 1. `Model::spherical_pendulum()` factory method with ball joint
//! 2. Quaternion representation (nq=4, nv=3) for 3D rotation
//! 3. Canonical `step_model_data` and `sync_model_data_to_bevy` systems
//! 4. Proper quaternion integration on SO(3) manifold
//!
//! ## Key Architectural Points
//!
//! - **Model** is static: ball joint connecting pendulum bob to world
//! - **Data** is dynamic: qpos[0..4] is quaternion [w,x,y,z], qvel[0..3] is angular velocity
//! - The quaternion is the ONLY rotational state; xpos/xquat are computed via FK
//! - Uses canonical systems from `sim_bevy::model_data` module
//!
//! The spherical pendulum exhibits rich 3D dynamics including:
//! - Precession when started with azimuthal velocity
//! - Smooth geodesic paths on the constraint sphere
//! - Conservation of both energy and vertical angular momentum
//!
//! Run with: `cargo run -p sim-bevy --example spherical_pendulum --release`

#![allow(clippy::needless_pass_by_value)]

use bevy::prelude::*;
use nalgebra::{UnitQuaternion, Vector3};
use sim_bevy::convert::vec3_from_vector;
use sim_bevy::model_data::{
    step_model_data, sync_model_data_to_bevy, ModelBodyIndex, PhysicsData, PhysicsModel,
};
use sim_core::Model;
use std::f64::consts::PI;

// ============================================================================
// Configuration
// ============================================================================

/// Pendulum length in meters.
const PENDULUM_LENGTH: f64 = 1.5;
/// Pendulum mass in kg.
const PENDULUM_MASS: f64 = 1.0;
/// Bob visual radius.
const BOB_RADIUS: f32 = 0.1;
/// Number of trail points to show.
const TRAIL_LENGTH: usize = 200;

// ============================================================================
// Components
// ============================================================================

/// Marker for the rod visual.
#[derive(Component)]
struct Rod;

/// Trail point with index.
#[derive(Component)]
struct TrailPoint(usize);

/// Reference sphere visual.
#[derive(Component)]
struct ReferenceSphere;

// ============================================================================
// Resources
// ============================================================================

/// Trail of recent bob positions.
#[derive(Resource, Default)]
struct Trail {
    positions: Vec<Vec3>,
}

/// Initial conserved quantities for validation.
#[derive(Resource)]
struct InitialState {
    energy: f64,
    angular_momentum_z: f64,
}

/// Debug print timing.
#[derive(Resource, Default)]
struct DebugPrintState {
    last_print_time: f64,
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .init_resource::<Trail>()
        .init_resource::<DebugPrintState>()
        .add_systems(Startup, setup_physics_and_scene)
        // Use canonical step_model_data system from module
        .add_systems(Update, step_model_data)
        // Use canonical sync_model_data_to_bevy for body transforms,
        // plus custom systems for rod, trail, and debug output
        .add_systems(
            PostUpdate,
            (
                sync_model_data_to_bevy,
                sync_rod_and_trail_and_debug,
            )
                .chain(),
        )
        .run();
}

/// Compute angular momentum about vertical (Z) axis.
///
/// For a point mass on a sphere: L_z = m * (x * vy - y * vx)
/// where (x, y, z) is position and (vx, vy, vz) is velocity.
fn angular_momentum_z(model: &Model, data: &PhysicsData) -> f64 {
    let pos = &data.xpos[1];
    let mass = model.body_mass[0]; // First non-world body

    // Get angular velocity in world frame
    let quat = &data.xquat[1];
    let omega_body = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]);
    let omega_world = quat * omega_body;

    // Linear velocity = omega × r (for rotation about origin)
    let vel = omega_world.cross(pos);

    // L_z = m * (x * vy - y * vx)
    mass * (pos.x * vel.y - pos.y * vel.x)
}

/// Setup physics model, data, and visual scene.
fn setup_physics_and_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create the physics model using factory method
    let model = Model::spherical_pendulum(PENDULUM_LENGTH, PENDULUM_MASS);
    let mut data = model.make_data();

    // Set initial orientation: tilted 45° from vertical with azimuthal velocity
    // The pendulum hangs down in -Z direction, so we rotate around Y to tilt it
    let theta = PI / 4.0; // 45° from vertical
    let phi_dot = 2.0; // Azimuthal angular velocity

    // Initial quaternion: rotation around Y axis by theta
    let initial_quat = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), theta);
    data.qpos[0] = initial_quat.w;
    data.qpos[1] = initial_quat.i;
    data.qpos[2] = initial_quat.j;
    data.qpos[3] = initial_quat.k;

    // Initial angular velocity: precession around Z axis
    // In the body frame, this appears as rotation around the local Z axis
    // after the tilt transformation
    data.qvel[0] = 0.0;
    data.qvel[1] = 0.0;
    data.qvel[2] = phi_dot;

    // Compute initial body poses via forward kinematics
    let _ = data.forward(&model);

    // Store initial conserved quantities
    let initial_energy = data.energy_kinetic + data.energy_potential;
    let initial_lz = angular_momentum_z(&model, &PhysicsData(data.clone()));

    // Print system info
    println!("=============================================");
    println!("Spherical Pendulum (Model/Data Architecture)");
    println!("=============================================");
    println!("Model (static):");
    println!("  nbody:  {} (world + bob)", model.nbody);
    println!("  njnt:   {} (ball joint)", model.njnt);
    println!("  nq:     {} (quaternion: w,x,y,z)", model.nq);
    println!("  nv:     {} (angular velocity: ωx,ωy,ωz)", model.nv);
    println!("---------------------------------------------");
    println!("Configuration:");
    println!("  Length: {:.2} m", PENDULUM_LENGTH);
    println!("  Mass:   {:.2} kg", PENDULUM_MASS);
    println!("---------------------------------------------");
    println!("Initial state:");
    println!("  θ = {:.1}° (tilt from vertical)", theta.to_degrees());
    println!("  φ̇ = {:.1} rad/s (azimuthal velocity)", phi_dot);
    println!("  E₀ = {:.4} J", initial_energy);
    println!("  L_z₀ = {:.4} kg⋅m²/s", initial_lz);
    println!("  dt = {:.5} s", model.timestep);
    println!("=============================================");
    println!("Key invariants:");
    println!("  - qpos is quaternion (4D), qvel is angular velocity (3D)");
    println!("  - Quaternion integrated on SO(3) manifold");
    println!("  - Energy E and angular momentum L_z conserved");
    println!("  - Bob moves on constraint sphere (FK guarantees)");
    println!("=============================================");
    println!();

    // Get initial bob position
    let bob_pos = vec3_from_vector(&data.xpos[1]);

    // Camera - positioned to see the 3D motion clearly
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(3.0, 1.0, 3.0).looking_at(Vec3::new(0.0, -0.5, 0.0), Vec3::Y),
    ));

    // Lighting
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Additional point light for better visibility
    commands.spawn((
        PointLight {
            intensity: 500_000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-3.0, 3.0, -3.0),
    ));

    // Pivot point
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.08))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Reference sphere (shows the constraint surface)
    commands.spawn((
        ReferenceSphere,
        Mesh3d(meshes.add(Sphere::new(PENDULUM_LENGTH as f32))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.3, 0.5, 0.8, 0.1),
            alpha_mode: AlphaMode::Blend,
            cull_mode: None,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Bob (golden color for visibility) - linked to physics body
    commands.spawn((
        ModelBodyIndex(1),
        Mesh3d(meshes.add(Sphere::new(BOB_RADIUS))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.7, 0.2),
            metallic: 0.8,
            perceptual_roughness: 0.2,
            ..default()
        })),
        Transform::from_translation(bob_pos),
    ));

    // Rod
    commands.spawn((
        Rod,
        Mesh3d(meshes.add(Cylinder::new(0.02, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.5, 0.5, 0.5),
            metallic: 0.8,
            perceptual_roughness: 0.2,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Trail points (small spheres that show the path)
    let trail_material = materials.add(StandardMaterial {
        base_color: Color::srgba(1.0, 0.3, 0.3, 0.7),
        alpha_mode: AlphaMode::Blend,
        ..default()
    });

    for i in 0..TRAIL_LENGTH {
        commands.spawn((
            TrailPoint(i),
            Mesh3d(meshes.add(Sphere::new(0.015))),
            MeshMaterial3d(trail_material.clone()),
            Transform::from_xyz(0.0, -100.0, 0.0), // Start off-screen
        ));
    }

    // Ground reference (XZ plane indicator)
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(4.0, 0.01, 4.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.4, 0.4, 0.4, 0.3),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -(PENDULUM_LENGTH as f32) - 0.1, 0.0),
    ));

    // Vertical axis indicator
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.01, 4.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.2, 0.8, 0.2, 0.5),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -1.0, 0.0),
    ));

    // Insert physics resources
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
    commands.insert_resource(InitialState {
        energy: initial_energy,
        angular_momentum_z: initial_lz,
    });
}

/// Synchronize rod, update trail, and print debug info.
///
/// This runs AFTER sync_model_data_to_bevy so body positions are already updated.
fn sync_rod_and_trail_and_debug(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut rod_query: Query<&mut Transform, With<Rod>>,
    mut trail: ResMut<Trail>,
    mut trail_query: Query<(&mut Transform, &TrailPoint), Without<Rod>>,
    initial: Res<InitialState>,
    mut state: ResMut<DebugPrintState>,
) {
    let pivot = Vec3::ZERO;
    let bob_pos = vec3_from_vector(&data.xpos[1]);

    // Update rod
    for mut transform in rod_query.iter_mut() {
        let midpoint = (pivot + bob_pos) / 2.0;
        let direction = (bob_pos - pivot).normalize_or_zero();
        let length = (bob_pos - pivot).length();

        transform.translation = midpoint;
        transform.scale = Vec3::new(1.0, length, 1.0);

        if direction.length_squared() > 0.0001 {
            transform.rotation = Quat::from_rotation_arc(Vec3::Y, direction);
        }
    }

    // Update trail
    let should_add = trail
        .positions
        .last()
        .map(|last| (*last - bob_pos).length() > 0.01)
        .unwrap_or(true);

    if should_add {
        trail.positions.push(bob_pos);

        if trail.positions.len() > TRAIL_LENGTH {
            trail.positions.remove(0);
        }
    }

    for (mut transform, point) in trail_query.iter_mut() {
        let idx = trail
            .positions
            .len()
            .saturating_sub(1)
            .saturating_sub(point.0);
        if idx < trail.positions.len() {
            transform.translation = trail.positions[idx];
        } else {
            transform.translation = Vec3::new(0.0, -100.0, 0.0);
        }
    }

    // Debug output every 2 seconds of simulation time
    let interval = 2.0;
    if data.time - state.last_print_time > interval {
        state.last_print_time = data.time;

        let energy = data.energy_kinetic + data.energy_potential;
        let lz = angular_momentum_z(&model, &data);

        let energy_error = (energy - initial.energy).abs() / initial.energy.abs() * 100.0;
        let lz_error = if initial.angular_momentum_z.abs() > 1e-10 {
            (lz - initial.angular_momentum_z).abs() / initial.angular_momentum_z.abs() * 100.0
        } else {
            0.0
        };

        // Extract theta and phi from body position
        let pos = &data.xpos[1];
        let r = (pos.x * pos.x + pos.y * pos.y + pos.z * pos.z).sqrt();
        let theta = (pos.z / -r).acos();
        let phi = pos.y.atan2(pos.x);

        // Verify constraint
        let constraint_error = (bob_pos.length() - PENDULUM_LENGTH as f32).abs();

        println!(
            "t={:.1}s  θ={:+.1}° φ={:+.1}°  E={:.3}J ΔE={:.2}%  L_z={:.3} ΔL_z={:.2}%  |r|={:.4}",
            data.time,
            theta.to_degrees(),
            phi.to_degrees(),
            energy,
            energy_error,
            lz,
            lz_error,
            PENDULUM_LENGTH as f32 + constraint_error
        );
    }
}
