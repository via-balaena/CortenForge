//! N-Link Pendulum using Model/Data Architecture
//!
//! This example demonstrates the unified Model/Data physics pipeline for a
//! general n-link serial pendulum. It showcases:
//!
//! 1. `Model::n_link_pendulum(n, ...)` factory method for arbitrary chain length
//! 2. General n×n CRBA mass matrix computation
//! 3. General RNE for Coriolis + gravity bias forces
//! 4. Canonical `step_model_data` and `sync_model_data_to_bevy` systems
//! 5. Forward kinematics computing body poses from joint angles
//!
//! ## Key Architectural Points
//!
//! - **Model** is static: kinematic tree for n bodies in a serial chain
//! - **Data** is dynamic: qpos[0..n] and qvel[0..n] are the ONLY state
//! - Body poses xpos[1..n+1] are COMPUTED via forward kinematics
//! - Uses canonical systems from `sim_bevy::model_data` module
//!
//! The n-link pendulum generalizes the double pendulum. With 5+ links,
//! the system exhibits complex chaotic behavior while maintaining:
//! - Energy conservation (semi-implicit Euler, ~1% drift over minutes)
//! - Structural integrity (FK guarantees links stay connected)
//!
//! Run with: `cargo run -p sim-bevy --example nlink_pendulum --release`

#![allow(clippy::needless_pass_by_value)]

use bevy::prelude::*;
use sim_bevy::convert::vec3_from_vector;
use sim_bevy::model_data::{
    ModelBodyIndex, PhysicsData, PhysicsModel, step_model_data, sync_model_data_to_bevy,
};
use sim_core::Model;
use std::f64::consts::PI;

// ============================================================================
// Configuration
// ============================================================================

/// Number of links in the pendulum chain.
const N_LINKS: usize = 5;
/// Link length in meters.
const LINK_LENGTH: f64 = 0.4;
/// Link mass in kg.
const LINK_MASS: f64 = 1.0;
/// Bob visual radius.
const BOB_RADIUS: f32 = 0.06;

// ============================================================================
// Components
// ============================================================================

/// Marker for rod visuals (connects body i-1 to body i).
#[derive(Component)]
struct Rod(usize);

// ============================================================================
// Main
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .init_resource::<DebugPrintState>()
        .add_systems(Startup, setup_physics_and_scene)
        // Use canonical step_model_data system from module
        .add_systems(Update, step_model_data)
        // Use canonical sync_model_data_to_bevy for body transforms,
        // plus custom system for rods and debug output
        .add_systems(
            PostUpdate,
            (sync_model_data_to_bevy, sync_rods_and_debug).chain(),
        )
        .run();
}

/// Setup physics model, data, and visual scene.
fn setup_physics_and_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create the physics model using factory method
    let model = Model::n_link_pendulum(N_LINKS, LINK_LENGTH, LINK_MASS);
    let mut data = model.make_data();

    // Set initial angles - varied for interesting motion
    for i in 0..N_LINKS {
        data.qpos[i] = PI / 3.0 - (i as f64) * PI / 20.0;
    }

    // Compute initial body poses via forward kinematics
    let _ = data.forward(&model);

    // Store initial energy for validation
    let initial_energy = data.energy_kinetic + data.energy_potential;

    // Print system info
    println!("=============================================");
    println!("{}-Link Pendulum (Model/Data Architecture)", N_LINKS);
    println!("=============================================");
    println!("Model (static):");
    println!("  nbody:  {} (world + {} links)", model.nbody, N_LINKS);
    println!("  njnt:   {} (hinge joints)", model.njnt);
    println!("  nq:     {} (joint angles)", model.nq);
    println!("  nv:     {} (angular velocities)", model.nv);
    println!("---------------------------------------------");
    println!("Configuration:");
    println!("  Link length: {:.2} m", LINK_LENGTH);
    println!("  Link mass:   {:.2} kg", LINK_MASS);
    println!("  Total length: {:.2} m", N_LINKS as f64 * LINK_LENGTH);
    println!("---------------------------------------------");
    println!("Initial state:");
    for i in 0..N_LINKS.min(3) {
        println!("  θ[{}] = {:.1}°", i, data.qpos[i].to_degrees());
    }
    if N_LINKS > 3 {
        println!("  ... ({} more)", N_LINKS - 3);
    }
    println!("  E₀ = {:.4} J", initial_energy);
    println!("  dt = {:.5} s", model.timestep);
    println!("=============================================");
    println!("Key invariants:");
    println!("  - qpos/qvel are the ONLY state variables");
    println!("  - xpos computed via forward kinematics");
    println!("  - Energy drift should be minimal");
    println!("=============================================");
    println!();

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, -1.0, 4.0).looking_at(Vec3::new(0.0, -1.0, 0.0), Vec3::Y),
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

    // Pivot point
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.08))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Color gradient for bobs (red to blue)
    let colors: Vec<Color> = (0..N_LINKS)
        .map(|i| {
            let t = i as f32 / (N_LINKS - 1).max(1) as f32;
            Color::srgb(1.0 - t * 0.8, 0.2 + t * 0.2, 0.2 + t * 0.7)
        })
        .collect();

    // Create bobs (linked to physics bodies) and rods
    for (i, &color) in colors.iter().enumerate() {
        let body_id = i + 1; // Body 0 is world
        let bob_pos = vec3_from_vector(&data.xpos[body_id]);

        // Bob - linked to physics body via ModelBodyIndex
        commands.spawn((
            ModelBodyIndex(body_id),
            Mesh3d(meshes.add(Sphere::new(BOB_RADIUS))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                metallic: 0.5,
                perceptual_roughness: 0.3,
                ..default()
            })),
            Transform::from_translation(bob_pos),
        ));

        // Rod - visual element connecting body i-1 to body i
        commands.spawn((
            Rod(i),
            Mesh3d(meshes.add(Cylinder::new(0.015, 1.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.5, 0.5, 0.5),
                metallic: 0.8,
                perceptual_roughness: 0.2,
                ..default()
            })),
            Transform::from_xyz(0.0, 0.0, 0.0),
        ));
    }

    // Ground reference
    let total_length = N_LINKS as f32 * LINK_LENGTH as f32;
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(4.0, 0.01, 0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.4, 0.4, 0.4, 0.5),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -total_length - 0.1, 0.0),
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
/// Rods are visual elements connecting body positions. This runs AFTER
/// sync_model_data_to_bevy so body positions are already updated.
fn sync_rods_and_debug(
    data: Res<PhysicsData>,
    mut rod_query: Query<(&mut Transform, &Rod)>,
    mut state: ResMut<DebugPrintState>,
    initial_energy: Res<InitialEnergy>,
) {
    // Build position array: [pivot, body1, body2, ...]
    let mut positions: Vec<Vec3> = vec![Vec3::ZERO]; // Pivot at origin
    for i in 1..=N_LINKS {
        positions.push(vec3_from_vector(&data.xpos[i]));
    }

    // Update rods
    for (mut transform, rod) in rod_query.iter_mut() {
        let start = positions[rod.0];
        let end = positions[rod.0 + 1];
        update_rod_transform(&mut transform, start, end);
    }

    // Debug output every 2 seconds of simulation time
    let interval = 2.0;
    if data.time - state.last_print_time > interval {
        state.last_print_time = data.time;

        let energy = data.energy_kinetic + data.energy_potential;
        let energy_error = (energy - initial_energy.0).abs() / initial_energy.0 * 100.0;

        // Print first few angles
        let angles: Vec<String> = (0..3.min(N_LINKS))
            .map(|i| format!("{:+.0}°", data.qpos[i].to_degrees()))
            .collect();

        // Verify a few link lengths (should all be constant due to FK)
        let link_lengths: Vec<String> = (0..2.min(N_LINKS))
            .map(|i| {
                let len = (positions[i + 1] - positions[i]).length();
                format!("{:.3}", len)
            })
            .collect();

        println!(
            "t={:.1}s  θ=[{}...]  E={:.3}J  ΔE={:.2}%  L=[{}...]",
            data.time,
            angles.join(", "),
            energy,
            energy_error,
            link_lengths.join(", ")
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
