//! Phase 2: Constrained Pendulum with Constraint-Based Physics
//!
//! This demonstrates the constraint-based approach to articulated body dynamics.
//! Unlike the simple pendulum (Phase 1) which works in joint space, this one
//! works in Cartesian space and uses a constraint solver to maintain the
//! distance constraint.
//!
//! Key differences from Phase 1:
//! - State is (x, y, vx, vy) instead of (θ, θ̇)
//! - Constraint solver computes forces to maintain x² + y² = L²
//! - Baumgarte stabilization corrects constraint drift
//! - This is how MuJoCo handles articulated bodies
//!
//! Visual validation:
//! - The pendulum should swing similarly to Phase 1
//! - The distance from pivot should stay constant (constraint maintained)
//! - Energy should be approximately conserved
//!
//! Run with: `cargo run -p sim-bevy --example constrained_pendulum --features x11`

#![allow(clippy::needless_pass_by_value)]

use bevy::prelude::*;
use std::f64::consts::PI;

// Import the constrained pendulum from sim-core
use sim_core::mujoco_pipeline::ConstrainedPendulum;

// ============================================================================
// Physics Constants
// ============================================================================

const DT: f64 = 1.0 / 240.0; // 240 Hz timestep
const PENDULUM_LENGTH: f64 = 1.0;
const PENDULUM_RADIUS: f32 = 0.08;

// ============================================================================
// Bevy Resources
// ============================================================================

#[derive(Resource)]
struct PendulumState {
    pendulum: ConstrainedPendulum,
    time: f64,
    initial_energy: f64,
}

impl Default for PendulumState {
    fn default() -> Self {
        let pendulum = ConstrainedPendulum::new(PENDULUM_LENGTH, 1.0, PI / 3.0);
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
struct PendulumBob;

#[derive(Component)]
struct PendulumRod;

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 0.0, 4.0).looking_at(Vec3::new(0.0, -0.5, 0.0), Vec3::Y),
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
        Mesh3d(meshes.add(Sphere::new(0.1))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Pendulum bob (blue to distinguish from Phase 1's red)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(PENDULUM_RADIUS))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.4, 0.9),
            metallic: 0.5,
            perceptual_roughness: 0.3,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        PendulumBob,
    ));

    // Pendulum rod
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.02, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.6, 0.6, 0.6),
            metallic: 0.8,
            perceptual_roughness: 0.2,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        PendulumRod,
    ));

    // Ground reference
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(3.0, 0.01, 0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.4, 0.4, 0.4, 0.5),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -(PENDULUM_LENGTH as f32), 0.0),
    ));

    // Constraint circle (shows the valid positions)
    // We'll draw this as a thin torus to show where the bob should stay
    commands.spawn((
        Mesh3d(meshes.add(Torus::new(0.98, 1.02))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.2, 0.8, 0.2, 0.3),
            alpha_mode: AlphaMode::Blend,
            unlit: true,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0).with_rotation(Quat::from_rotation_x(PI as f32 / 2.0)),
    ));

    println!("==============================================");
    println!("Phase 2: Constrained Pendulum (Constraint-Based)");
    println!("==============================================");
    println!("Pendulum Length: {:.2} m", PENDULUM_LENGTH);
    println!("Initial Angle:   {:.1}°", (PI / 3.0).to_degrees());
    println!("Timestep:        {:.4} s (240 Hz)", DT);
    println!("==============================================");
    println!("Watch for:");
    println!("  - Smooth oscillation (like Phase 1)");
    println!("  - Bob stays on the green circle (constraint)");
    println!("  - Constraint violation stays small");
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
    mut bob_query: Query<&mut Transform, (With<PendulumBob>, Without<PendulumRod>)>,
    mut rod_query: Query<&mut Transform, (With<PendulumRod>, Without<PendulumBob>)>,
) {
    // Get bob position from Cartesian state
    // The physics uses (x, y) where y is vertical
    // Bevy uses Y-up, so we map directly
    let bob_pos = Vec3::new(
        state.pendulum.pos.x as f32,
        state.pendulum.pos.y as f32,
        0.0,
    );

    for mut transform in bob_query.iter_mut() {
        transform.translation = bob_pos;
    }

    for mut transform in rod_query.iter_mut() {
        let pivot = Vec3::ZERO;
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

fn print_debug_info(state: Res<PendulumState>, time: Res<Time>) {
    let interval = 2.0;
    let prev_bucket = ((state.time - time.delta_secs_f64()) / interval) as i32;
    let curr_bucket = (state.time / interval) as i32;

    if curr_bucket > prev_bucket {
        let energy = state.pendulum.total_energy();
        let energy_error = (energy - state.initial_energy).abs() / state.initial_energy * 100.0;

        // Constraint violation: how far from ideal distance
        let actual_dist = (state.pendulum.pos.x.powi(2) + state.pendulum.pos.y.powi(2)).sqrt();
        let dist_error = (actual_dist - state.pendulum.length).abs();

        let angle = state.pendulum.angle();
        let angular_vel = state.pendulum.angular_velocity();

        println!(
            "t={:.1}s  θ={:+.2}rad ({:+.0}°)  ω={:+.2}rad/s  E={:.4}J  ΔE={:.2}%  |Δr|={:.6}m",
            state.time,
            angle,
            angle.to_degrees(),
            angular_vel,
            energy,
            energy_error,
            dist_error
        );
    }
}
