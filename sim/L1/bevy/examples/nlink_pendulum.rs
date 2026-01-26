//! Phase 4: N-Link Pendulum with General CRBA and RNE
//!
//! This demonstrates the general n-DOF physics pipeline:
//! 1. CRBA for n×n inertia matrix M(θ)
//! 2. RNE for Coriolis + gravity bias forces
//! 3. LU solve for accelerations: M @ qacc = qfrc_smooth
//! 4. Semi-implicit Euler integration
//!
//! The n-link pendulum generalizes the double pendulum to any number
//! of links. This is a chaotic system where small differences in
//! initial conditions lead to vastly different trajectories.
//!
//! Visual validation:
//! - All links should swing smoothly and stay connected
//! - Energy should be approximately conserved (check ΔE%)
//! - More links = more complex motion
//!
//! Run with: `cargo run -p sim-bevy --example nlink_pendulum --release`

#![allow(clippy::needless_pass_by_value)]

use bevy::prelude::*;
use std::f64::consts::PI;

use sim_core::mujoco_pipeline::NLinkPendulum;

// ============================================================================
// Configuration
// ============================================================================

const N_LINKS: usize = 5;
const DT: f64 = 1.0 / 480.0; // 480 Hz for energy conservation
const LINK_LENGTH: f64 = 0.4; // Shorter links for visibility
const BOB_RADIUS: f32 = 0.06;

// ============================================================================
// Bevy Resources
// ============================================================================

#[derive(Resource)]
struct PendulumState {
    pendulum: NLinkPendulum,
    time: f64,
    initial_energy: f64,
}

impl Default for PendulumState {
    fn default() -> Self {
        // Start with varied angles for interesting motion
        let mut angles = vec![0.0; N_LINKS];
        for (i, angle) in angles.iter_mut().enumerate() {
            *angle = PI / 3.0 - (i as f64) * PI / 20.0;
        }

        let pendulum = NLinkPendulum::new(N_LINKS, LINK_LENGTH, 1.0).with_angles(&angles);
        let initial_energy = pendulum.total_energy();
        Self {
            pendulum,
            time: 0.0,
            initial_energy,
        }
    }
}

// ============================================================================
// Components
// ============================================================================

#[derive(Component)]
struct Bob(usize);

#[derive(Component)]
struct Rod(usize);

// ============================================================================
// Main
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .init_resource::<PendulumState>()
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

    // Create bobs and rods
    for (i, &color) in colors.iter().enumerate() {
        // Bob
        commands.spawn((
            Mesh3d(meshes.add(Sphere::new(BOB_RADIUS))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                metallic: 0.5,
                perceptual_roughness: 0.3,
                ..default()
            })),
            Transform::from_xyz(0.0, 0.0, 0.0),
            Bob(i),
        ));

        // Rod
        commands.spawn((
            Mesh3d(meshes.add(Cylinder::new(0.015, 1.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.5, 0.5, 0.5),
                metallic: 0.8,
                perceptual_roughness: 0.2,
                ..default()
            })),
            Transform::from_xyz(0.0, 0.0, 0.0),
            Rod(i),
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

    println!("=============================================");
    println!("Phase 4: {}-Link Pendulum (General CRBA + RNE)", N_LINKS);
    println!("=============================================");
    println!("Link Length: {:.2} m (each)", LINK_LENGTH);
    println!("Total Length: {:.2} m", N_LINKS as f64 * LINK_LENGTH);
    println!("Timestep:    {:.5} s (480 Hz)", DT);
    println!("=============================================");
    println!("Watch for:");
    println!("  - Complex chaotic motion");
    println!("  - All links stay connected");
    println!("  - Energy conservation (check ΔE%)");
    println!("  - Color gradient: red→blue (link 0→{})", N_LINKS - 1);
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
    mut bob_query: Query<(&mut Transform, &Bob), Without<Rod>>,
    mut rod_query: Query<(&mut Transform, &Rod), Without<Bob>>,
) {
    // Get all bob positions
    let mut positions: Vec<Vec3> = vec![Vec3::ZERO]; // Start with pivot
    for i in 0..state.pendulum.n_links() {
        let pos = state.pendulum.position(i);
        positions.push(Vec3::new(pos.x as f32, pos.y as f32, 0.0));
    }

    // Update bobs
    for (mut transform, bob) in bob_query.iter_mut() {
        let pos = positions[bob.0 + 1]; // +1 because positions[0] is pivot
        transform.translation = pos;
    }

    // Update rods
    for (mut transform, rod) in rod_query.iter_mut() {
        let start = positions[rod.0];
        let end = positions[rod.0 + 1];

        let midpoint = (start + end) / 2.0;
        let direction = (end - start).normalize_or_zero();
        let length = (end - start).length();

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

        // Print first few angles
        let angles: Vec<String> = (0..3.min(state.pendulum.n_links()))
            .map(|i| format!("{:+.0}°", state.pendulum.qpos[i].to_degrees()))
            .collect();

        println!(
            "t={:.1}s  θ=[{}...]  E={:.3}J  ΔE={:.2}%",
            state.time,
            angles.join(", "),
            energy,
            energy_error
        );
    }
}
