//! Phase 3: Double Pendulum with Multi-DOF Physics
//!
//! This demonstrates the full multi-DOF physics pipeline:
//! 1. Composite Rigid Body Algorithm (CRBA) for 2×2 inertia matrix
//! 2. Recursive Newton-Euler (RNE) for Coriolis + gravity bias forces
//! 3. Matrix solve for accelerations: M @ qacc = qfrc_smooth
//! 4. Semi-implicit Euler integration
//!
//! The double pendulum is a classic chaotic system. Small differences in
//! initial conditions lead to vastly different trajectories. However:
//! - Energy should be approximately conserved
//! - The links should stay connected
//! - Motion should look physically plausible
//!
//! Visual validation:
//! - The pendulum should swing chaotically but smoothly
//! - Both links should stay connected (no stretching/breaking)
//! - The system shouldn't "explode" or gain energy spontaneously
//!
//! Run with: `cargo run -p sim-bevy --example double_pendulum --features x11`

#![allow(clippy::needless_pass_by_value)]

use bevy::prelude::*;
use std::f64::consts::PI;

// Import the double pendulum from sim-core
use sim_core::mujoco_pipeline::DoublePendulum;

// ============================================================================
// Physics Constants
// ============================================================================

const DT: f64 = 1.0 / 480.0; // 480 Hz for better energy conservation
const PENDULUM_LENGTH: f64 = 1.0;
const BOB_RADIUS: f32 = 0.08;

// ============================================================================
// Bevy Resources
// ============================================================================

#[derive(Resource)]
struct PendulumState {
    pendulum: DoublePendulum,
    time: f64,
    initial_energy: f64,
}

impl Default for PendulumState {
    fn default() -> Self {
        // Start with both links displaced for interesting chaotic motion
        let pendulum = DoublePendulum::new(PENDULUM_LENGTH, 1.0).with_angles(PI / 2.0, PI / 4.0); // First link horizontal, second 45°
        let initial_energy = pendulum.total_energy();
        Self {
            pendulum,
            time: 0.0,
            initial_energy,
        }
    }
}

// ============================================================================
// Bevy Integration
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .init_resource::<PendulumState>()
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (step_physics, sync_pendulum_visual, print_debug_info),
        )
        .run();
}

#[derive(Component)]
struct Bob1;

#[derive(Component)]
struct Bob2;

#[derive(Component)]
struct Rod1;

#[derive(Component)]
struct Rod2;

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera - positioned to see both links clearly
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, -0.5, 5.0).looking_at(Vec3::new(0.0, -1.0, 0.0), Vec3::Y),
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

    // Pivot point (fixed point where first link attaches)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.1))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // First bob (red)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(BOB_RADIUS))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.2, 0.2),
            metallic: 0.5,
            perceptual_roughness: 0.3,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        Bob1,
    ));

    // Second bob (blue)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(BOB_RADIUS))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.4, 0.9),
            metallic: 0.5,
            perceptual_roughness: 0.3,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        Bob2,
    ));

    // First rod (dark gray)
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.02, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.5, 0.5, 0.5),
            metallic: 0.8,
            perceptual_roughness: 0.2,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        Rod1,
    ));

    // Second rod (lighter gray)
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.02, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.7, 0.7, 0.7),
            metallic: 0.8,
            perceptual_roughness: 0.2,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        Rod2,
    ));

    // Ground reference plane
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(5.0, 0.01, 0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.4, 0.4, 0.4, 0.5),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -(2.0 * PENDULUM_LENGTH as f32), 0.0),
    ));

    println!("=============================================");
    println!("Phase 3: Double Pendulum (CRBA + RNE)");
    println!("=============================================");
    println!("Link Length: {:.2} m (each)", PENDULUM_LENGTH);
    println!("Initial θ₁:  {:.1}° (first link)", (PI / 2.0).to_degrees());
    println!(
        "Initial θ₂:  {:.1}° (second link, relative)",
        (PI / 4.0).to_degrees()
    );
    println!("Timestep:    {:.5} s (480 Hz)", DT);
    println!("=============================================");
    println!("Watch for:");
    println!("  - Chaotic but smooth motion");
    println!("  - Links stay connected (no stretching)");
    println!("  - Energy conservation (check ΔE%)");
    println!("  - Red bob = first mass, Blue bob = second mass");
    println!("Press Ctrl+C to exit");
    println!();
}

fn step_physics(mut state: ResMut<PendulumState>, time: Res<Time>) {
    let frame_time = time.delta_secs_f64();
    let mut accumulated = 0.0;

    while accumulated < frame_time {
        state.pendulum.step(DT);
        state.time += DT;
        accumulated += DT;
    }
}

fn sync_pendulum_visual(
    state: Res<PendulumState>,
    mut bob1_query: Query<
        &mut Transform,
        (With<Bob1>, Without<Bob2>, Without<Rod1>, Without<Rod2>),
    >,
    mut bob2_query: Query<
        &mut Transform,
        (With<Bob2>, Without<Bob1>, Without<Rod1>, Without<Rod2>),
    >,
    mut rod1_query: Query<
        &mut Transform,
        (With<Rod1>, Without<Bob1>, Without<Bob2>, Without<Rod2>),
    >,
    mut rod2_query: Query<
        &mut Transform,
        (With<Rod2>, Without<Bob1>, Without<Bob2>, Without<Rod1>),
    >,
) {
    // Get positions from physics state
    let pos1 = state.pendulum.position1();
    let pos2 = state.pendulum.position2();

    let bob1_pos = Vec3::new(pos1.x as f32, pos1.y as f32, 0.0);
    let bob2_pos = Vec3::new(pos2.x as f32, pos2.y as f32, 0.0);
    let pivot = Vec3::ZERO;

    // Update first bob
    for mut transform in bob1_query.iter_mut() {
        transform.translation = bob1_pos;
    }

    // Update second bob
    for mut transform in bob2_query.iter_mut() {
        transform.translation = bob2_pos;
    }

    // Update first rod (pivot to bob1)
    for mut transform in rod1_query.iter_mut() {
        let midpoint = (pivot + bob1_pos) / 2.0;
        let direction = (bob1_pos - pivot).normalize_or_zero();
        let length = (bob1_pos - pivot).length();

        transform.translation = midpoint;
        transform.scale = Vec3::new(1.0, length, 1.0);

        if direction.length_squared() > 0.0001 {
            transform.rotation = Quat::from_rotation_arc(Vec3::Y, direction);
        }
    }

    // Update second rod (bob1 to bob2)
    for mut transform in rod2_query.iter_mut() {
        let midpoint = (bob1_pos + bob2_pos) / 2.0;
        let direction = (bob2_pos - bob1_pos).normalize_or_zero();
        let length = (bob2_pos - bob1_pos).length();

        transform.translation = midpoint;
        transform.scale = Vec3::new(1.0, length, 1.0);

        if direction.length_squared() > 0.0001 {
            transform.rotation = Quat::from_rotation_arc(Vec3::Y, direction);
        }
    }
}

fn print_debug_info(state: Res<PendulumState>, time: Res<Time>) {
    let interval = 2.0;
    let prev_bucket = ((state.time - time.delta_secs_f64()) / interval) as i32;
    let curr_bucket = (state.time / interval) as i32;

    if curr_bucket > prev_bucket {
        let energy = state.pendulum.total_energy();
        let energy_error = (energy - state.initial_energy).abs() / state.initial_energy * 100.0;

        let theta1 = state.pendulum.qpos[0];
        let theta2 = state.pendulum.qpos[1];
        let omega1 = state.pendulum.qvel[0];
        let omega2 = state.pendulum.qvel[1];

        // Check link lengths for debugging
        let pos1 = state.pendulum.position1();
        let pos2 = state.pendulum.position2();
        let link1_len = pos1.norm();
        let link2_len = (pos2 - pos1).norm();

        println!(
            "t={:.1}s  θ₁={:+.1}° θ₂={:+.1}°  ω₁={:+.1} ω₂={:+.1}  E={:.3}J  ΔE={:.2}%  L₁={:.3} L₂={:.3}",
            state.time,
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
