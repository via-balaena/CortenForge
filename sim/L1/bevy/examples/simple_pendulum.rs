//! Phase 1: Simple Pendulum with MuJoCo-Exact Physics
//!
//! This implements a single pendulum following MuJoCo's physics pipeline EXACTLY:
//! 1. Forward kinematics (compute body position from joint angle)
//! 2. Compute M (scalar inertia for 1-DOF)
//! 3. Compute bias forces (gravity torque via RNE)
//! 4. Semi-implicit Euler integration (velocity first, then position)
//!
//! Visual validation: The pendulum must swing naturally under gravity.
//!
//! Run with: `cargo run -p sim-bevy --example simple_pendulum --features x11`

#![allow(clippy::needless_pass_by_value)]

use bevy::prelude::*;
use std::f64::consts::PI;

// ============================================================================
// Physics Constants
// ============================================================================

/// Pendulum parameters matching MuJoCo defaults
const GRAVITY: f64 = 9.81; // m/s^2
const DT: f64 = 1.0 / 240.0; // 240 Hz timestep (MuJoCo default)

// Pendulum physical properties
const PENDULUM_LENGTH: f64 = 1.0; // meters (pivot to COM)
const PENDULUM_MASS: f64 = 1.0; // kg
const PENDULUM_RADIUS: f32 = 0.08; // meters (for visualization, spherical bob)

// ============================================================================
// Pendulum State (Joint-Space Representation)
// ============================================================================

/// The pendulum state in generalized coordinates (MuJoCo style).
///
/// For a 1-DOF revolute joint:
/// - qpos: joint angle θ (radians)
/// - qvel: joint velocity θ_dot (rad/s)
#[derive(Resource)]
struct PendulumState {
    /// Joint angle (θ) in radians. 0 = hanging down, positive = CCW
    qpos: f64,
    /// Joint angular velocity (θ_dot) in rad/s
    qvel: f64,
    /// Simulation time
    time: f64,
    /// Initial total energy (for conservation check)
    initial_energy: f64,
}

impl Default for PendulumState {
    fn default() -> Self {
        // Start at 60 degrees to the side to see clear swinging
        let initial_angle = PI / 3.0;
        let initial_vel = 0.0;

        // Compute initial energy
        let kinetic =
            0.5 * PENDULUM_MASS * PENDULUM_LENGTH * PENDULUM_LENGTH * initial_vel * initial_vel;
        let h = PENDULUM_LENGTH * (1.0 - initial_angle.cos());
        let potential = PENDULUM_MASS * GRAVITY * h;

        Self {
            qpos: initial_angle,
            qvel: initial_vel,
            time: 0.0,
            initial_energy: kinetic + potential,
        }
    }
}

// ============================================================================
// MuJoCo-Exact Physics Pipeline
// ============================================================================

/// Compute the effective inertia M for a simple pendulum.
///
/// For a point mass at distance L from the pivot:
/// M = m * L^2
///
/// For MuJoCo, this would come from the Composite Rigid Body Algorithm (CRBA),
/// but for a 1-DOF system, it simplifies to this scalar.
fn compute_inertia() -> f64 {
    PENDULUM_MASS * PENDULUM_LENGTH * PENDULUM_LENGTH
}

/// Compute the bias force (gravity torque) using Recursive Newton-Euler (RNE).
///
/// In MuJoCo's RNE formulation, the bias force is the generalized force needed
/// to produce zero acceleration. For a pendulum at angle θ:
/// - Gravity pulls on the mass with force F = m*g (downward)
/// - This creates a torque τ = m*g*L*sin(θ) about the pivot
/// - The bias force is this torque: qfrc_bias = m*g*L*sin(θ)
///
/// When we compute qfrc_smooth = -qfrc_bias (since there are no other forces),
/// we get qacc = M^(-1) * (-m*g*L*sin(θ)) = -g/L * sin(θ), which is correct!
fn compute_bias_force(qpos: f64) -> f64 {
    // Bias force = torque needed to maintain zero acceleration
    // When θ > 0, we'd need positive torque to hold the pendulum there against gravity
    PENDULUM_MASS * GRAVITY * PENDULUM_LENGTH * qpos.sin()
}

/// MuJoCo Step: Semi-implicit Euler Integration
///
/// This is the core of MuJoCo's `mj_Euler` function:
/// 1. Compute qacc = M^(-1) * (τ_applied + τ_passive - τ_bias)
/// 2. Update velocity: qvel_new = qvel + dt * qacc
/// 3. Update position: qpos_new = qpos + dt * qvel_new  (uses NEW velocity!)
///
/// The use of qvel_new for position update makes this "semi-implicit" and
/// provides much better energy conservation than explicit Euler.
fn mj_step(state: &mut PendulumState) {
    let dt = DT;

    // Stage 1: Forward Position (mj_fwdPosition)
    // - For a simple pendulum, kinematics are trivial
    // - Inertia M is constant (no configuration-dependent terms)
    let m_inv = 1.0 / compute_inertia();

    // Stage 2: Forward Velocity (mj_fwdVelocity)
    // - Compute bias forces (Coriolis + gravity)
    // - For 1-DOF, no Coriolis terms
    let qfrc_bias = compute_bias_force(state.qpos);

    // Stage 3: Forward Actuation (mj_fwdActuation)
    // - No actuators for this simple pendulum
    let qfrc_actuator = 0.0;

    // Stage 4: Forward Acceleration (mj_fwdAcceleration)
    // qfrc_smooth = qfrc_passive + qfrc_actuator + qfrc_applied - qfrc_bias
    // Note: MuJoCo SUBTRACTS bias forces!
    let qfrc_passive = 0.0; // No springs/dampers
    let qfrc_applied = 0.0; // No external torques
    let qfrc_smooth = qfrc_passive + qfrc_actuator + qfrc_applied - qfrc_bias;

    // qacc = M^(-1) * qfrc_smooth
    let qacc = m_inv * qfrc_smooth;

    // Stage 5: Integration (mj_Euler) - SEMI-IMPLICIT
    // CRITICAL: Velocity update uses current acceleration
    let qvel_new = state.qvel + dt * qacc;

    // CRITICAL: Position update uses NEW velocity (semi-implicit)
    let qpos_new = state.qpos + dt * qvel_new;

    // Update state
    state.qvel = qvel_new;
    state.qpos = qpos_new;
    state.time += dt;
}

// ============================================================================
// Forward Kinematics: Joint Angle → World Position
// ============================================================================

/// Compute the world position of the pendulum bob from joint angle.
///
/// Convention (matching Bevy's Y-up):
/// - Pivot is at origin
/// - θ = 0 means hanging straight down (along -Y axis)
/// - θ > 0 rotates toward +X (counter-clockwise when viewed from +Z)
fn joint_to_world_position(qpos: f64) -> Vec3 {
    // In Bevy's Y-up coordinate system:
    // x = L * sin(θ)
    // y = -L * cos(θ)  (negative because θ=0 is hanging down)
    let x = (PENDULUM_LENGTH * qpos.sin()) as f32;
    let y = (-PENDULUM_LENGTH * qpos.cos()) as f32;
    Vec3::new(x, y, 0.0)
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

/// Marker component for the pendulum bob visual
#[derive(Component)]
struct PendulumBob;

/// Marker component for the pendulum rod visual
#[derive(Component)]
struct PendulumRod;

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera - positioned to see the pendulum clearly
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

    // Pivot point (fixed point where pendulum attaches)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.1))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Pendulum bob (red sphere)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(PENDULUM_RADIUS))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.2, 0.2),
            metallic: 0.5,
            perceptual_roughness: 0.3,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        PendulumBob,
    ));

    // Pendulum rod (thin cylinder connecting pivot to bob)
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

    // Ground reference line (thin gray box at y = -L)
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(3.0, 0.01, 0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.4, 0.4, 0.4, 0.5),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -(PENDULUM_LENGTH as f32), 0.0),
    ));

    println!("========================================");
    println!("Phase 1: Simple Pendulum (MuJoCo-Exact)");
    println!("========================================");
    println!("Pendulum Length: {:.2} m", PENDULUM_LENGTH);
    println!("Pendulum Mass:   {:.2} kg", PENDULUM_MASS);
    println!("Initial Angle:   {:.1}°", (PI / 3.0).to_degrees());
    println!("Timestep:        {:.4} s (240 Hz)", DT);
    println!("========================================");
    println!("Watch for: smooth oscillation with energy conservation");
    println!("Press Ctrl+C to exit");
    println!();
}

/// Step the physics simulation (runs every frame, but uses fixed timestep internally)
fn step_physics(mut state: ResMut<PendulumState>, time: Res<Time>) {
    // Run physics at fixed timestep (240 Hz)
    // This may run multiple steps per frame if frame rate is lower
    let frame_time = time.delta_secs_f64();
    let mut accumulated = 0.0;

    while accumulated < frame_time {
        mj_step(&mut state);
        accumulated += DT;
    }
}

/// Sync the visual pendulum position with physics state
fn sync_pendulum_visual(
    state: Res<PendulumState>,
    mut bob_query: Query<&mut Transform, (With<PendulumBob>, Without<PendulumRod>)>,
    mut rod_query: Query<&mut Transform, (With<PendulumRod>, Without<PendulumBob>)>,
) {
    // Get bob position from physics
    let bob_pos = joint_to_world_position(state.qpos);

    // Update bob position
    for mut transform in bob_query.iter_mut() {
        transform.translation = bob_pos;
    }

    // Update rod - it connects pivot (origin) to bob
    for mut transform in rod_query.iter_mut() {
        let pivot = Vec3::ZERO;
        let midpoint = (pivot + bob_pos) / 2.0;
        let direction = (bob_pos - pivot).normalize_or_zero();
        let length = (bob_pos - pivot).length();

        transform.translation = midpoint;
        transform.scale = Vec3::new(1.0, length, 1.0);

        // Rotate to point from pivot to bob
        // Cylinder is aligned with Y axis by default
        if direction.length_squared() > 0.0001 {
            transform.rotation = Quat::from_rotation_arc(Vec3::Y, direction);
        }
    }
}

/// Print debug info periodically
fn print_debug_info(state: Res<PendulumState>, time: Res<Time>) {
    // Print every 2 seconds of simulation time
    let interval = 2.0;
    let prev_bucket = ((state.time - time.delta_secs_f64()) / interval) as i32;
    let curr_bucket = (state.time / interval) as i32;

    if curr_bucket > prev_bucket {
        // Compute current energy
        let kinetic =
            0.5 * PENDULUM_MASS * PENDULUM_LENGTH * PENDULUM_LENGTH * state.qvel * state.qvel;
        let h = PENDULUM_LENGTH * (1.0 - state.qpos.cos());
        let potential = PENDULUM_MASS * GRAVITY * h;
        let total = kinetic + potential;
        let energy_error = (total - state.initial_energy).abs() / state.initial_energy * 100.0;

        println!(
            "t={:.1}s  θ={:+.2}rad ({:+.0}°)  ω={:+.2}rad/s  E={:.4}J  ΔE={:.2}%",
            state.time,
            state.qpos,
            state.qpos.to_degrees(),
            state.qvel,
            total,
            energy_error
        );
    }
}
