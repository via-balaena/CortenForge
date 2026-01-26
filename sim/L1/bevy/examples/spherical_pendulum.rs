//! Phase 5: Spherical Pendulum (3D Ball Joint)
//!
//! This demonstrates the spherical pendulum - a 2-DOF system where a point
//! mass swings freely in 3D, constrained only to move on a sphere.
//!
//! The state is represented in spherical coordinates:
//! - theta: polar angle from vertical (0 = hanging down)
//! - phi: azimuthal angle in the horizontal plane
//!
//! Conserved quantities:
//! - Total energy E = T + V
//! - Angular momentum about vertical axis L_z = m*L²*sin²(θ)*φ̇
//!
//! Visual validation:
//! - The bob should trace smooth paths on a sphere
//! - Energy and L_z should be approximately conserved
//! - Conical motion (constant theta) demonstrates precession
//!
//! Run with: `cargo run -p sim-bevy --example spherical_pendulum --release`

#![allow(clippy::needless_pass_by_value)]

use bevy::prelude::*;
use std::f64::consts::PI;

use sim_core::mujoco_pipeline::SphericalPendulum;

// ============================================================================
// Configuration
// ============================================================================

const DT: f64 = 1.0 / 480.0; // 480 Hz for energy conservation
const PENDULUM_LENGTH: f64 = 1.5;
const BOB_RADIUS: f32 = 0.1;
const TRAIL_LENGTH: usize = 200; // Number of trail points

// ============================================================================
// Bevy Resources
// ============================================================================

#[derive(Resource)]
struct PendulumState {
    pendulum: SphericalPendulum,
    time: f64,
    initial_energy: f64,
    initial_lz: f64,
}

impl Default for PendulumState {
    fn default() -> Self {
        // Start tilted with azimuthal velocity for interesting 3D motion
        let theta = PI / 4.0; // 45 degrees from vertical
        let phi_dot = 2.0; // Azimuthal velocity for precession

        let pendulum = SphericalPendulum::new(PENDULUM_LENGTH, 1.0)
            .with_theta(theta)
            .with_velocities(0.0, phi_dot);

        let initial_energy = pendulum.total_energy();
        let initial_lz = pendulum.angular_momentum_z();

        Self {
            pendulum,
            time: 0.0,
            initial_energy,
            initial_lz,
        }
    }
}

#[derive(Resource, Default)]
struct Trail {
    positions: Vec<Vec3>,
}

// ============================================================================
// Components
// ============================================================================

#[derive(Component)]
struct Bob;

#[derive(Component)]
struct Rod;

#[derive(Component)]
struct TrailPoint(usize);

#[derive(Component)]
struct ReferenceSphere;

// ============================================================================
// Main
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .init_resource::<PendulumState>()
        .init_resource::<Trail>()
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (step_physics, sync_visuals, update_trail, print_debug_info),
        )
        .run();
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
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
            intensity: 500000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-3.0, 3.0, -3.0),
    ));

    // Pivot point
    commands.spawn((
        Mesh3d(meshes.add(bevy::math::primitives::Sphere::new(0.08))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Reference sphere (wireframe effect using transparency)
    commands.spawn((
        Mesh3d(meshes.add(bevy::math::primitives::Sphere::new(PENDULUM_LENGTH as f32))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.3, 0.5, 0.8, 0.1),
            alpha_mode: AlphaMode::Blend,
            cull_mode: None,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        ReferenceSphere,
    ));

    // Bob (golden color for visibility)
    commands.spawn((
        Mesh3d(meshes.add(bevy::math::primitives::Sphere::new(BOB_RADIUS))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.7, 0.2),
            metallic: 0.8,
            perceptual_roughness: 0.2,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        Bob,
    ));

    // Rod
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.02, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.5, 0.5, 0.5),
            metallic: 0.8,
            perceptual_roughness: 0.2,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        Rod,
    ));

    // Trail points (small spheres that show the path)
    let trail_material = materials.add(StandardMaterial {
        base_color: Color::srgba(1.0, 0.3, 0.3, 0.7),
        alpha_mode: AlphaMode::Blend,
        ..default()
    });

    for i in 0..TRAIL_LENGTH {
        commands.spawn((
            Mesh3d(meshes.add(bevy::math::primitives::Sphere::new(0.015))),
            MeshMaterial3d(trail_material.clone()),
            Transform::from_xyz(0.0, -100.0, 0.0), // Start off-screen
            TrailPoint(i),
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

    println!("=============================================");
    println!("Phase 5: Spherical Pendulum (3D Ball Joint)");
    println!("=============================================");
    println!("Length:      {:.2} m", PENDULUM_LENGTH);
    println!(
        "Initial θ:   {:.1}° (tilt from vertical)",
        (PI / 4.0).to_degrees()
    );
    println!("Initial φ̇:   {:.1} rad/s (azimuthal velocity)", 2.0);
    println!("Timestep:    {:.5} s (480 Hz)", DT);
    println!("=============================================");
    println!("Watch for:");
    println!("  - 3D precession motion (not just 2D swing)");
    println!("  - Bob traces path on sphere surface");
    println!("  - Energy conservation (check ΔE%)");
    println!("  - L_z conservation (angular momentum)");
    println!("  - Red trail shows recent path");
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

fn sync_visuals(
    state: Res<PendulumState>,
    mut bob_query: Query<&mut Transform, (With<Bob>, Without<Rod>)>,
    mut rod_query: Query<&mut Transform, (With<Rod>, Without<Bob>)>,
) {
    let pos = state.pendulum.position();
    let bob_pos = Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32);
    let pivot = Vec3::ZERO;

    // Update bob
    for mut transform in bob_query.iter_mut() {
        transform.translation = bob_pos;
    }

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
}

fn update_trail(
    state: Res<PendulumState>,
    mut trail: ResMut<Trail>,
    mut trail_query: Query<(&mut Transform, &TrailPoint)>,
) {
    // Add current position to trail
    let pos = state.pendulum.position();
    let current_pos = Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32);

    // Only add if moved enough
    let should_add = trail
        .positions
        .last()
        .map(|last| (*last - current_pos).length() > 0.01)
        .unwrap_or(true);

    if should_add {
        trail.positions.push(current_pos);

        // Trim to max length
        if trail.positions.len() > TRAIL_LENGTH {
            trail.positions.remove(0);
        }
    }

    // Update trail point transforms
    for (mut transform, point) in trail_query.iter_mut() {
        let idx = trail
            .positions
            .len()
            .saturating_sub(1)
            .saturating_sub(point.0);
        if idx < trail.positions.len() {
            transform.translation = trail.positions[idx];
        } else {
            transform.translation = Vec3::new(0.0, -100.0, 0.0); // Hide unused points
        }
    }
}

fn print_debug_info(state: Res<PendulumState>, time: Res<Time>) {
    let interval = 2.0;
    let prev_bucket = ((state.time - time.delta_secs_f64()) / interval) as i32;
    let curr_bucket = (state.time / interval) as i32;

    if curr_bucket > prev_bucket {
        let energy = state.pendulum.total_energy();
        let lz = state.pendulum.angular_momentum_z();

        let energy_error = (energy - state.initial_energy).abs() / state.initial_energy * 100.0;
        let lz_error = (lz - state.initial_lz).abs() / state.initial_lz.abs() * 100.0;

        let theta_deg = state.pendulum.theta.to_degrees();
        let phi_deg = state.pendulum.phi.to_degrees();

        println!(
            "t={:.1}s  θ={:+.1}° φ={:+.1}°  E={:.3}J ΔE={:.2}%  L_z={:.3} ΔL_z={:.2}%",
            state.time, theta_deg, phi_deg, energy, energy_error, lz, lz_error
        );
    }
}
