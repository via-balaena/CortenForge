//! Phase 6: Bouncing Balls with Contact Physics
//!
//! This demonstrates the contact constraint solver:
//! 1. Sphere-ground collision detection
//! 2. Contact normal force (prevents penetration)
//! 3. Friction force (Coulomb model)
//! 4. Coefficient of restitution (bounce damping)
//! 5. Sphere-sphere collisions
//!
//! Visual validation:
//! - Balls should bounce realistically
//! - Multiple balls should collide and separate
//! - Energy should dissipate with each bounce
//! - Balls should eventually come to rest
//!
//! Run with: `cargo run -p sim-bevy --example bouncing_balls --release`

#![allow(clippy::needless_pass_by_value)]

use bevy::prelude::*;

use sim_core::mujoco_pipeline::SpherePile;

// ============================================================================
// Configuration
// ============================================================================

const DT: f64 = 1.0 / 480.0; // 480 Hz for smooth contact
const N_BALLS: usize = 5;
const BALL_RADIUS: f64 = 0.15;

// ============================================================================
// Bevy Resources
// ============================================================================

#[derive(Resource)]
struct PhysicsState {
    pile: SpherePile,
    time: f64,
    initial_energy: f64,
}

impl Default for PhysicsState {
    fn default() -> Self {
        // Create a pile of balls with some horizontal offset for interesting motion
        let mut pile = SpherePile::new(N_BALLS, BALL_RADIUS, 1.0)
            .with_restitution(0.7)
            .with_friction(0.3);

        // Add some horizontal spread and initial velocities
        for i in 0..N_BALLS {
            let angle = (i as f64) * 0.5;
            pile.positions[i].x = 0.3 * angle.sin();
            pile.positions[i].z = 0.3 * angle.cos();
            pile.velocities[i].x = angle.cos() * 0.5;
            pile.velocities[i].z = angle.sin() * 0.5;
        }

        let initial_energy = pile.total_energy();

        Self {
            pile,
            time: 0.0,
            initial_energy,
        }
    }
}

// ============================================================================
// Components
// ============================================================================

#[derive(Component)]
struct Ball(usize);

#[derive(Component)]
struct Ground;

// ============================================================================
// Main
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .init_resource::<PhysicsState>()
        .add_systems(Startup, setup_scene)
        .add_systems(Update, (step_physics, sync_visuals, print_debug_info))
        .run();
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(3.0, 2.0, 3.0).looking_at(Vec3::new(0.0, 0.5, 0.0), Vec3::Y),
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

    // Additional light
    commands.spawn((
        PointLight {
            intensity: 500000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-3.0, 3.0, -3.0),
    ));

    // Ground plane
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(4.0, 0.1, 4.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.5, 0.4),
            metallic: 0.1,
            perceptual_roughness: 0.9,
            ..default()
        })),
        Transform::from_xyz(0.0, -0.05, 0.0),
        Ground,
    ));

    // Create balls with different colors
    let colors = [
        Color::srgb(0.9, 0.2, 0.2), // Red
        Color::srgb(0.2, 0.7, 0.2), // Green
        Color::srgb(0.2, 0.4, 0.9), // Blue
        Color::srgb(0.9, 0.7, 0.2), // Gold
        Color::srgb(0.7, 0.2, 0.9), // Purple
    ];

    for i in 0..N_BALLS {
        commands.spawn((
            Mesh3d(meshes.add(bevy::math::primitives::Sphere::new(BALL_RADIUS as f32))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: colors[i % colors.len()],
                metallic: 0.7,
                perceptual_roughness: 0.3,
                ..default()
            })),
            Transform::from_xyz(0.0, 0.0, 0.0),
            Ball(i),
        ));
    }

    println!("=============================================");
    println!("Phase 6: Bouncing Balls (Contact Physics)");
    println!("=============================================");
    println!("Number of balls: {}", N_BALLS);
    println!("Ball radius:     {:.2} m", BALL_RADIUS);
    println!("Restitution:     0.7 (slightly inelastic)");
    println!("Friction:        0.3");
    println!("Timestep:        {:.5} s (480 Hz)", DT);
    println!("=============================================");
    println!("Watch for:");
    println!("  - Realistic bouncing behavior");
    println!("  - Ball-ball collisions");
    println!("  - Energy dissipation (check ΔE%)");
    println!("  - Balls coming to rest");
    println!("Press Ctrl+C to exit");
    println!();
}

fn step_physics(mut state: ResMut<PhysicsState>, time: Res<Time>) {
    let frame_time = time.delta_secs_f64();
    let mut accumulated = 0.0;

    while accumulated < frame_time {
        state.pile.step(DT);
        state.time += DT;
        accumulated += DT;
    }
}

fn sync_visuals(state: Res<PhysicsState>, mut ball_query: Query<(&mut Transform, &Ball)>) {
    for (mut transform, ball) in ball_query.iter_mut() {
        let pos = state.pile.positions[ball.0];
        transform.translation = Vec3::new(pos.x as f32, pos.y as f32, pos.z as f32);
    }
}

fn print_debug_info(state: Res<PhysicsState>, time: Res<Time>) {
    let interval = 2.0;
    let prev_bucket = ((state.time - time.delta_secs_f64()) / interval) as i32;
    let curr_bucket = (state.time / interval) as i32;

    if curr_bucket > prev_bucket {
        let energy = state.pile.total_energy();
        let energy_error = (energy - state.initial_energy) / state.initial_energy * 100.0;

        // Count balls in contact with ground
        let balls_on_ground = state
            .pile
            .positions
            .iter()
            .zip(state.pile.radii.iter())
            .filter(|(pos, radius)| pos.y - *radius < 0.01)
            .count();

        // Average speed
        let avg_speed: f64 =
            state.pile.velocities.iter().map(|v| v.norm()).sum::<f64>() / N_BALLS as f64;

        println!(
            "t={:.1}s  E={:.3}J  ΔE={:+.1}%  on_ground={}/{}  avg_speed={:.2} m/s",
            state.time, energy, energy_error, balls_on_ground, N_BALLS, avg_speed
        );
    }
}
