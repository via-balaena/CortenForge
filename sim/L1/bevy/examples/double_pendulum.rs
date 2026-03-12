//! Double Pendulum using Model/Data Architecture
//!
//! This example demonstrates the unified Model/Data physics pipeline for a
//! double pendulum system. It showcases:
//!
//! 1. `Model::double_pendulum()` factory method for creating the physics model
//! 2. `PhysicsModel` and `PhysicsData` Bevy resources
//! 3. `ModelBodyIndex` component linking entities to bodies
//! 4. Canonical `step_model_data` and `sync_model_data_to_bevy` systems
//! 5. Forward kinematics computing xpos/xquat from qpos
//!
//! ## Key Architectural Points
//!
//! - **Model** is static: kinematic tree, joint definitions, mass properties
//! - **Data** is dynamic: qpos/qvel are the ONLY state variables
//! - Body poses (xpos/xquat) are COMPUTED via forward kinematics, never stored
//! - Uses canonical systems from `sim_bevy::model_data` module
//!
//! The double pendulum is a classic chaotic system. Small differences in
//! initial conditions lead to vastly different trajectories. However:
//! - Energy should be approximately conserved (semi-implicit Euler)
//! - The links should stay connected (forward kinematics ensures this)
//! - Motion should look physically plausible
//!
//! Run with: `cargo run -p sim-bevy --example double_pendulum --features x11`

#![allow(clippy::needless_pass_by_value)]

use bevy::prelude::*;
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::convert::vec3_from_vector;
use sim_bevy::model_data::{ModelBodyIndex, PhysicsData, PhysicsModel, step_model_data};
use sim_core::{ENABLE_ENERGY, Integrator, Model};
use std::f64::consts::PI;

// ============================================================================
// Physics Constants
// ============================================================================

/// Link length in meters.
const LINK_LENGTH: f64 = 1.0;
/// Link mass in kg.
const LINK_MASS: f64 = 1.0;
/// Initial angle for first link (radians from vertical).
const INITIAL_THETA1: f64 = PI / 2.0; // 90° - horizontal
/// Initial angle for second link (radians relative to first).
const INITIAL_THETA2: f64 = PI / 4.0; // 45°
/// Bob visual radius.
const BOB_RADIUS: f32 = 0.08;

// ============================================================================
// Components for Visual Elements
// ============================================================================

/// Marker for the first rod visual.
#[derive(Component)]
struct Rod1;

/// Marker for the second rod visual.
#[derive(Component)]
struct Rod2;

// ============================================================================
// Bevy Integration
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<DebugPrintState>()
        .add_systems(Startup, setup_physics_and_scene)
        // Use canonical step_model_data system from module
        .add_systems(Update, step_model_data)
        // Custom sync using xipos (COM positions) for pendulum visualization
        .add_systems(PostUpdate, sync_rods_and_debug)
        .run();
}

/// Setup physics model, data, and visual scene.
fn setup_physics_and_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create the physics model using factory method
    let mut model = Model::double_pendulum(LINK_LENGTH, LINK_MASS);
    model.enableflags |= ENABLE_ENERGY;

    // Implicit integrator for stability (Euler has ~75% energy drift)
    model.integrator = Integrator::RungeKutta4;

    let mut data = model.make_data();

    // Set initial angles
    data.qpos[0] = INITIAL_THETA1;
    data.qpos[1] = INITIAL_THETA2;

    // Compute initial body poses via forward kinematics
    let _ = data.forward(&model);

    // Store initial energy for validation
    let initial_energy = data.energy_kinetic + data.energy_potential;

    // Print system info
    println!("=============================================");
    println!("Double Pendulum (Model/Data Architecture)");
    println!("=============================================");
    println!("Model (static):");
    println!("  nbody:  {} (world + 2 links)", model.nbody);
    println!("  njnt:   {} (2 hinge joints)", model.njnt);
    println!("  nq:     {} (joint angles)", model.nq);
    println!("  nv:     {} (angular velocities)", model.nv);
    println!("---------------------------------------------");
    println!("Initial state:");
    println!(
        "  θ₁ = {:.1}° (first link from vertical)",
        INITIAL_THETA1.to_degrees()
    );
    println!(
        "  θ₂ = {:.1}° (second link relative)",
        INITIAL_THETA2.to_degrees()
    );
    println!("  E₀ = {:.4} J (initial energy)", initial_energy);
    println!("  dt = {:.5} s (timestep)", model.timestep);
    println!("=============================================");
    println!("Key invariants to observe:");
    println!("  - qpos/qvel are the ONLY state variables");
    println!("  - xpos/xquat computed via forward kinematics");
    println!("  - Energy drift should be minimal (ΔE < 1%)");
    println!("  - Links stay connected (FK guarantees this)");
    println!("=============================================");
    println!();

    // Get initial COM positions for entity spawning (xipos, not xpos)
    let bob1_pos = vec3_from_vector(&data.xipos[1]);
    let bob2_pos = vec3_from_vector(&data.xipos[2]);

    // Camera with orbit controls
    let orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, -1.0, 0.0))
        .with_distance(5.0)
        .with_angles(0.0, 0.1);
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

    // Pivot point (fixed point where first link attaches)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.1))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // First bob (red) - linked to body 1
    commands.spawn((
        ModelBodyIndex(1),
        Mesh3d(meshes.add(Sphere::new(BOB_RADIUS))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.2, 0.2),
            metallic: 0.5,
            perceptual_roughness: 0.3,
            ..default()
        })),
        Transform::from_translation(bob1_pos),
    ));

    // Second bob (blue) - linked to body 2
    commands.spawn((
        ModelBodyIndex(2),
        Mesh3d(meshes.add(Sphere::new(BOB_RADIUS))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.4, 0.9),
            metallic: 0.5,
            perceptual_roughness: 0.3,
            ..default()
        })),
        Transform::from_translation(bob2_pos),
    ));

    // First rod (dark gray) - connects pivot to bob1
    commands.spawn((
        Rod1,
        Mesh3d(meshes.add(Cylinder::new(0.02, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.5, 0.5, 0.5),
            metallic: 0.8,
            perceptual_roughness: 0.2,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Second rod (lighter gray) - connects bob1 to bob2
    commands.spawn((
        Rod2,
        Mesh3d(meshes.add(Cylinder::new(0.02, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.7, 0.7, 0.7),
            metallic: 0.8,
            perceptual_roughness: 0.2,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Ground reference plane
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(5.0, 0.01, 0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.4, 0.4, 0.4, 0.5),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -(2.0 * LINK_LENGTH as f32), 0.0),
    ));

    // Insert physics resources
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
    commands.insert_resource(InitialEnergy(initial_energy));
}

/// Resource to store initial energy for drift calculation.
#[derive(Resource)]
struct InitialEnergy(f64);

/// Resource to track debug print timing.
#[derive(Resource, Default)]
struct DebugPrintState {
    last_print_time: f64,
}

/// Synchronize rod visuals and print debug info.
///
/// Rods are visual elements that connect body positions - they're not physics
/// bodies, so they need custom sync logic. This runs AFTER sync_model_data_to_bevy
/// so body positions are already updated.
fn sync_rods_and_debug(
    data: Res<PhysicsData>,
    mut bobs: Query<(&ModelBodyIndex, &mut Transform), (Without<Rod1>, Without<Rod2>)>,
    mut rod1_query: Query<&mut Transform, (With<Rod1>, Without<Rod2>, Without<ModelBodyIndex>)>,
    mut rod2_query: Query<&mut Transform, (With<Rod2>, Without<Rod1>, Without<ModelBodyIndex>)>,
    mut state: ResMut<DebugPrintState>,
    initial_energy: Res<InitialEnergy>,
) {
    // Sync bob positions from xipos (COM), not xpos (joint frame)
    for (body_idx, mut transform) in &mut bobs {
        let idx = body_idx.0;
        if idx < data.xipos.len() {
            transform.translation = vec3_from_vector(&data.xipos[idx]);
        }
    }

    let pivot = Vec3::ZERO;
    let bob1_pos = vec3_from_vector(&data.xipos[1]);
    let bob2_pos = vec3_from_vector(&data.xipos[2]);

    // Update first rod (pivot to bob1)
    for mut transform in rod1_query.iter_mut() {
        update_rod_transform(&mut transform, pivot, bob1_pos);
    }

    // Update second rod (bob1 to bob2)
    for mut transform in rod2_query.iter_mut() {
        update_rod_transform(&mut transform, bob1_pos, bob2_pos);
    }

    // Debug output every 2 seconds of simulation time
    let interval = 2.0;
    if data.time - state.last_print_time > interval {
        state.last_print_time = data.time;

        let energy = data.energy_kinetic + data.energy_potential;
        let energy_error = (energy - initial_energy.0).abs() / initial_energy.0 * 100.0;

        let theta1 = data.qpos[0];
        let theta2 = data.qpos[1];
        let omega1 = data.qvel[0];
        let omega2 = data.qvel[1];

        // Verify link lengths (should be constant due to FK)
        let link1_len = bob1_pos.length();
        let link2_len = (bob2_pos - bob1_pos).length();

        println!(
            "t={:.1}s  θ₁={:+.1}° θ₂={:+.1}°  ω₁={:+.1} ω₂={:+.1}  E={:.3}J  ΔE={:.2}%  L₁={:.3} L₂={:.3}",
            data.time,
            theta1.to_degrees(),
            theta2.to_degrees(),
            omega1,
            omega2,
            energy,
            energy_error,
            link1_len,
            link2_len
        );
    }
}

/// Update a rod transform to connect two points.
fn update_rod_transform(transform: &mut Transform, start: Vec3, end: Vec3) {
    let midpoint = (start + end) / 2.0;
    let direction = (end - start).normalize_or_zero();
    let length = (end - start).length();

    transform.translation = midpoint;
    transform.scale = Vec3::new(1.0, length, 1.0);

    if direction.length_squared() > 0.0001 {
        transform.rotation = Quat::from_rotation_arc(Vec3::Y, direction);
    }
}
