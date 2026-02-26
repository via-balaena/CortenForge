//! MJCF Humanoid using Model/Data Architecture
//!
//! This example demonstrates loading an MJCF model and simulating it using
//! the unified Model/Data physics pipeline. It showcases:
//!
//! 1. `sim_mjcf::load_model()` to parse MJCF directly into a `Model`
//! 2. `PhysicsModel` and `PhysicsData` Bevy resources
//! 3. `ModelBodyIndex` components linking entities to bodies
//! 4. Forward kinematics computing all body poses from joint state
//! 5. Mixed joint types (free, ball, hinge) in a single model
//!
//! ## Key Architectural Points
//!
//! - **Model** is static: parsed from MJCF, contains kinematic tree
//! - **Data** is dynamic: qpos/qvel span all joints (nq may differ from nv)
//! - Body poses (xpos/xquat) are COMPUTED via forward kinematics
//! - MJCF provides a declarative way to define complex articulated systems
//!
//! The humanoid demonstrates the power of the architecture:
//! - Multiple joint types in a single kinematic tree
//! - Proper mass/inertia from MJCF defaults
//! - Forward kinematics handles the entire tree in one pass
//!
//! Run with: `cargo run -p sim-bevy --example mjcf_humanoid --release --features x11`

#![allow(clippy::needless_pass_by_value)]

use bevy::prelude::*;
use sim_bevy::convert::vec3_from_vector;
use sim_bevy::model_data::{ModelBodyIndex, PhysicsData, PhysicsModel, sync_model_data_to_bevy};
use sim_core::ENABLE_ENERGY;
use sim_mjcf::load_model;

// ============================================================================
// MJCF Model Definition
// ============================================================================

/// A simplified humanoid MJCF model.
///
/// This model demonstrates:
/// - Free joint for floating base (torso)
/// - Ball joints for shoulders/hips (3-DOF each)
/// - Hinge joints for elbows/knees (1-DOF each)
const HUMANOID_MJCF: &str = r#"
<mujoco model="humanoid">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.002"/>

    <default>
        <joint damping="5.0"/>
        <geom rgba="0.8 0.6 0.4 1"/>
    </default>

    <worldbody>
        <!-- Ground plane (visual reference, no collision) -->
        <geom type="plane" size="5 5 0.1" rgba="0.3 0.3 0.35 1" contype="0" conaffinity="0"/>

        <!-- Torso (floating base) -->
        <body name="torso" pos="0 0 1.2">
            <joint type="free"/>
            <geom type="capsule" fromto="0 0 0 0 0 0.3" size="0.12"/>
            <inertial pos="0 0 0.15" mass="10.0" diaginertia="0.5 0.5 0.2"/>

            <!-- Head -->
            <body name="head" pos="0 0 0.4">
                <joint name="neck" type="ball" damping="8.0"/>
                <geom type="sphere" size="0.1" rgba="0.9 0.7 0.6 1"/>
                <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            </body>

            <!-- Left Arm -->
            <body name="left_upper_arm" pos="0.15 0 0.28">
                <joint name="left_shoulder" type="ball" damping="4.0"/>
                <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.035"/>
                <inertial pos="0.1 0 0" mass="1.0" diaginertia="0.01 0.01 0.005"/>

                <body name="left_lower_arm" pos="0.22 0 0">
                    <joint name="left_elbow" type="hinge" axis="0 1 0" range="-2.5 0" limited="true" damping="3.0"/>
                    <geom type="capsule" fromto="0 0 0 0.18 0 0" size="0.03"/>
                    <inertial pos="0.09 0 0" mass="0.5" diaginertia="0.005 0.005 0.002"/>
                </body>
            </body>

            <!-- Right Arm -->
            <body name="right_upper_arm" pos="-0.15 0 0.28">
                <joint name="right_shoulder" type="ball" damping="4.0"/>
                <geom type="capsule" fromto="0 0 0 -0.2 0 0" size="0.035"/>
                <inertial pos="-0.1 0 0" mass="1.0" diaginertia="0.01 0.01 0.005"/>

                <body name="right_lower_arm" pos="-0.22 0 0">
                    <joint name="right_elbow" type="hinge" axis="0 1 0" range="0 2.5" limited="true" damping="3.0"/>
                    <geom type="capsule" fromto="0 0 0 -0.18 0 0" size="0.03"/>
                    <inertial pos="-0.09 0 0" mass="0.5" diaginertia="0.005 0.005 0.002"/>
                </body>
            </body>

            <!-- Pelvis -->
            <body name="pelvis" pos="0 0 -0.1">
                <joint name="spine" type="ball" damping="15.0"/>
                <geom type="capsule" fromto="-0.08 0 0 0.08 0 0" size="0.07"/>
                <inertial pos="0 0 0" mass="5.0" diaginertia="0.2 0.2 0.1"/>

                <!-- Left Leg -->
                <body name="left_upper_leg" pos="0.08 0 -0.05">
                    <joint name="left_hip" type="ball" damping="8.0"/>
                    <geom type="capsule" fromto="0 0 0 0 0 -0.28" size="0.045"/>
                    <inertial pos="0 0 -0.14" mass="2.0" diaginertia="0.04 0.04 0.01"/>

                    <body name="left_lower_leg" pos="0 0 -0.3">
                        <joint name="left_knee" type="hinge" axis="1 0 0" range="-2.5 0" limited="true" damping="6.0"/>
                        <geom type="capsule" fromto="0 0 0 0 0 -0.25" size="0.038"/>
                        <inertial pos="0 0 -0.125" mass="1.5" diaginertia="0.03 0.03 0.008"/>

                        <body name="left_foot" pos="0 0 -0.27">
                            <joint name="left_ankle" type="ball" damping="3.0"/>
                            <geom type="box" size="0.04 0.07 0.015" pos="0 0.02 0"/>
                            <inertial pos="0 0.02 0" mass="0.3" diaginertia="0.002 0.002 0.001"/>
                        </body>
                    </body>
                </body>

                <!-- Right Leg -->
                <body name="right_upper_leg" pos="-0.08 0 -0.05">
                    <joint name="right_hip" type="ball" damping="8.0"/>
                    <geom type="capsule" fromto="0 0 0 0 0 -0.28" size="0.045"/>
                    <inertial pos="0 0 -0.14" mass="2.0" diaginertia="0.04 0.04 0.01"/>

                    <body name="right_lower_leg" pos="0 0 -0.3">
                        <joint name="right_knee" type="hinge" axis="1 0 0" range="-2.5 0" limited="true" damping="6.0"/>
                        <geom type="capsule" fromto="0 0 0 0 0 -0.25" size="0.038"/>
                        <inertial pos="0 0 -0.125" mass="1.5" diaginertia="0.03 0.03 0.008"/>

                        <body name="right_foot" pos="0 0 -0.27">
                            <joint name="right_ankle" type="ball" damping="3.0"/>
                            <geom type="box" size="0.04 0.07 0.015" pos="0 0.02 0"/>
                            <inertial pos="0 0.02 0" mass="0.3" diaginertia="0.002 0.002 0.001"/>
                        </body>
                    </body>
                </body>
            </body>
        </body>
    </worldbody>
</mujoco>
"#;

// ============================================================================
// Configuration
// ============================================================================

/// Physics substeps per frame.
const SUBSTEPS: usize = 4;

// ============================================================================
// Resources
// ============================================================================

/// Debug print timing.
#[derive(Resource, Default)]
struct DebugPrintState {
    last_print_time: f64,
}

/// Initial energy for drift calculation.
#[derive(Resource)]
struct InitialEnergy(f64);

// ============================================================================
// Main
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .init_resource::<DebugPrintState>()
        .add_systems(Startup, setup_physics_and_scene)
        // Custom step_physics_with_substeps for humanoid (4 substeps per frame)
        .add_systems(Update, step_physics_with_substeps)
        // Use canonical sync_model_data_to_bevy for body transforms,
        // plus custom debug_info system for periodic output
        .add_systems(
            PostUpdate,
            (sync_model_data_to_bevy, debug_info).chain(),
        )
        .run();
}

/// Setup physics model from MJCF and visual scene.
fn setup_physics_and_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Parse MJCF directly into Model
    let mut model = match load_model(HUMANOID_MJCF) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("Failed to load MJCF: {e}");
            return;
        }
    };
    model.enableflags |= ENABLE_ENERGY;

    // Create data and run forward kinematics
    let mut data = model.make_data();
    if let Err(e) = data.forward(&model) {
        eprintln!("Forward kinematics failed: {e}");
        return;
    }

    // Store initial energy
    let initial_energy = data.energy_kinetic + data.energy_potential;

    // Print model information
    println!("=============================================");
    println!("MJCF Humanoid (Model/Data Architecture)");
    println!("=============================================");
    println!("Model (static, from MJCF):");
    println!("  nbody:  {} bodies", model.nbody);
    println!("  njnt:   {} joints", model.njnt);
    println!("  nq:     {} (position coordinates)", model.nq);
    println!("  nv:     {} (velocity DOFs)", model.nv);
    println!("---------------------------------------------");
    println!("Joint breakdown:");
    println!("  - Free: 1 (torso, nq=7, nv=6)");
    println!("  - Ball: 8 (neck, shoulders, spine, hips, ankles)");
    println!("  - Hinge: 4 (elbows, knees)");
    println!("---------------------------------------------");
    println!("Initial state:");
    println!("  E₀ = {:.4} J", initial_energy);
    println!("  dt = {:.5} s", model.timestep);
    println!("=============================================");
    println!("Key invariants:");
    println!("  - MJCF parsed directly to Model (no World)");
    println!("  - qpos/qvel are ONLY state variables");
    println!("  - All {} body poses via FK", model.nbody);
    println!("=============================================");
    println!();

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(3.0, 1.0, 3.0).looking_at(Vec3::new(0.0, 0.5, 0.0), Vec3::Y),
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
            intensity: 300_000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-2.0, 3.0, -2.0),
    ));

    // Ground plane visual
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(5.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.35),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Color palette for body visualization
    let body_colors = [
        Color::srgb(0.8, 0.6, 0.4),  // Torso
        Color::srgb(0.9, 0.7, 0.6),  // Head
        Color::srgb(0.7, 0.5, 0.4),  // Left arm
        Color::srgb(0.6, 0.4, 0.3),  // Left forearm
        Color::srgb(0.7, 0.5, 0.4),  // Right arm
        Color::srgb(0.6, 0.4, 0.3),  // Right forearm
        Color::srgb(0.5, 0.4, 0.3),  // Pelvis
        Color::srgb(0.6, 0.4, 0.35), // Left leg
        Color::srgb(0.5, 0.35, 0.3), // Left lower leg
        Color::srgb(0.4, 0.3, 0.25), // Left foot
        Color::srgb(0.6, 0.4, 0.35), // Right leg
        Color::srgb(0.5, 0.35, 0.3), // Right lower leg
        Color::srgb(0.4, 0.3, 0.25), // Right foot
    ];

    // Spawn visual representations for each body (skip world body 0)
    for body_id in 1..model.nbody {
        let bob_pos = vec3_from_vector(&data.xpos[body_id]);
        let color_idx = (body_id - 1) % body_colors.len();

        commands.spawn((
            ModelBodyIndex(body_id),
            Mesh3d(meshes.add(Sphere::new(0.05))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: body_colors[color_idx],
                metallic: 0.3,
                perceptual_roughness: 0.5,
                ..default()
            })),
            Transform::from_translation(bob_pos),
        ));
    }

    // Insert physics resources
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
    commands.insert_resource(InitialEnergy(initial_energy));
}

/// Step physics simulation with substeps.
///
/// The humanoid model benefits from multiple substeps per frame for stability.
/// This is a custom step function - for simpler models, use the canonical
/// `step_model_data` from the module which does a single step per frame.
fn step_physics_with_substeps(model: Res<PhysicsModel>, mut data: ResMut<PhysicsData>) {
    for _ in 0..SUBSTEPS {
        if let Err(e) = data.step(&model) {
            eprintln!("Physics step failed: {e}");
            return;
        }
    }
}

/// Print debug information periodically.
fn debug_info(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    initial: Res<InitialEnergy>,
    mut state: ResMut<DebugPrintState>,
) {
    let interval = 2.0;
    if data.time - state.last_print_time > interval {
        state.last_print_time = data.time;

        let energy = data.energy_kinetic + data.energy_potential;
        let energy_error = if initial.0.abs() > 1e-10 {
            (energy - initial.0).abs() / initial.0.abs() * 100.0
        } else {
            0.0
        };

        // Get torso position (body 1)
        let torso_pos = &data.xpos[1];

        println!(
            "t={:.1}s  Torso=[{:+.2}, {:+.2}, {:+.2}]  E={:.2}J  ΔE={:.2}%  nq={}  nv={}",
            data.time,
            torso_pos.x,
            torso_pos.y,
            torso_pos.z,
            energy,
            energy_error,
            model.nq,
            model.nv
        );
    }
}
