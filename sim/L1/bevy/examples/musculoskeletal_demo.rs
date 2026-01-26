//! Example: Musculoskeletal visualization demo.
//!
//! Demonstrates the Phase 3.1 visualization features for muscles, tendons, and sensors.
//! This example shows:
//! - Muscle visualization with activation-based coloring (blue → red)
//! - Tendon path visualization with tension-based opacity
//! - Sensor visualization (IMU, force/torque, touch, rangefinder, magnetometer)
//!
//! Run with: `cargo run -p sim-bevy --example musculoskeletal_demo --features x11`
//! (or `--features wayland` on Wayland systems)
//!
//! Controls:
//! - Mouse drag: Rotate camera
//! - Scroll: Zoom
//! - M: Toggle muscle visualization
//! - T: Toggle tendon visualization
//! - S: Toggle sensor visualization
//! - Space: Pause/resume simulation
//! - 1-5: Activate different muscle groups (hold to activate)
//! - R: Reset muscle activations

#![allow(clippy::needless_pass_by_value)] // Bevy system parameters

use bevy::prelude::*;
use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_bevy::prelude::*;
use sim_core::Stepper;

/// Simple MJCF model with a two-link arm for musculoskeletal demonstration.
const ARM_MJCF: &str = r#"
<mujoco model="musculoskeletal_arm">
    <option gravity="0 0 -9.81" timestep="0.002"/>

    <default>
        <geom rgba="0.7 0.7 0.75 1"/>
        <joint damping="0.5"/>
    </default>

    <worldbody>
        <!-- Ground plane -->
        <geom type="plane" size="3 3 0.1" rgba="0.2 0.25 0.3 1"/>

        <!-- Base/shoulder mount (fixed) -->
        <body name="base" pos="0 0 1.5">
            <geom type="box" size="0.1 0.1 0.1" rgba="0.4 0.4 0.5 1"/>

            <!-- Upper arm -->
            <body name="upper_arm" pos="0 0 -0.15">
                <joint name="shoulder" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
                <geom type="capsule" fromto="0 0 0 0 0 -0.35" size="0.04" rgba="0.8 0.6 0.5 1"/>

                <!-- Forearm -->
                <body name="forearm" pos="0 0 -0.4">
                    <joint name="elbow" type="hinge" axis="0 1 0" range="-2.5 0"/>
                    <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.035" rgba="0.8 0.6 0.5 1"/>

                    <!-- Hand -->
                    <body name="hand" pos="0 0 -0.35">
                        <joint name="wrist" type="hinge" axis="0 1 0" range="-1.0 1.0"/>
                        <geom type="box" size="0.04 0.06 0.015" rgba="0.85 0.65 0.55 1"/>
                    </body>
                </body>
            </body>
        </body>

        <!-- Target sphere for rangefinder demo -->
        <body name="target" pos="0.5 0 0.5">
            <geom type="sphere" size="0.1" rgba="1.0 0.3 0.3 1"/>
        </body>
    </worldbody>
</mujoco>
"#;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimViewerPlugin::new())
        .add_systems(Startup, setup_scene)
        .init_resource::<PhysicsStepper>()
        .init_resource::<MuscleState>()
        .init_resource::<SimulationPaused>()
        .add_systems(Update, step_physics)
        .add_systems(Update, handle_input)
        .add_systems(Update, update_muscle_activations)
        .add_systems(Update, update_visualizations)
        .add_systems(Update, print_help.run_if(run_once))
        .run();
}

/// Resource to track muscle activation state.
#[derive(Resource, Default)]
struct MuscleState {
    /// Biceps activation (0-1)
    biceps: f64,
    /// Triceps activation (0-1)
    triceps: f64,
    /// Brachialis activation (0-1)
    brachialis: f64,
    /// Wrist flexor activation (0-1)
    wrist_flexor: f64,
    /// Wrist extensor activation (0-1)
    wrist_extensor: f64,
    /// Elapsed time for animation
    time: f64,
}

/// Resource to track if simulation is paused.
#[derive(Resource, Default)]
struct SimulationPaused(bool);

/// Resource to hold the physics stepper.
#[derive(Resource, Default)]
struct PhysicsStepper {
    stepper: Stepper,
}

/// Set up the scene with the arm model.
fn setup_scene(mut commands: Commands, mut config: ResMut<ViewerConfig>) {
    // Enable musculoskeletal visualization by default
    config.show_muscles = true;
    config.show_tendons = true;
    config.show_sensors = true;
    config.show_collision_shapes = true;
    config.show_joint_axes = true;

    // Create the physics world
    let world = sim_core::World::default();
    let mut sim_handle = SimulationHandle::new(world);

    // Load the MJCF model
    match MjcfModel::from_xml(ARM_MJCF) {
        Ok(model) => {
            println!("Loaded MJCF model: {}", model.name());
            match model.spawn_into(&mut sim_handle) {
                Ok(spawned) => {
                    println!("Spawned model: {}", spawned.name());
                    commands.insert_resource(ArmBodyIds {
                        base: spawned.body_id("base"),
                        upper_arm: spawned.body_id("upper_arm"),
                        forearm: spawned.body_id("forearm"),
                        hand: spawned.body_id("hand"),
                    });
                }
                Err(e) => eprintln!("Failed to spawn model: {e}"),
            }
        }
        Err(e) => eprintln!("Failed to load MJCF: {e}"),
    }

    commands.insert_resource(sim_handle);
}

/// Stores the body IDs for the arm segments.
#[derive(Resource, Default)]
struct ArmBodyIds {
    base: Option<sim_types::BodyId>,
    upper_arm: Option<sim_types::BodyId>,
    forearm: Option<sim_types::BodyId>,
    hand: Option<sim_types::BodyId>,
}

/// Step the physics simulation.
fn step_physics(
    mut sim_handle: ResMut<SimulationHandle>,
    mut physics_stepper: ResMut<PhysicsStepper>,
    mut muscle_state: ResMut<MuscleState>,
    paused: Res<SimulationPaused>,
    time: Res<Time>,
) {
    if paused.0 {
        return;
    }

    if let Some(world) = sim_handle.world_mut() {
        let _ = physics_stepper.stepper.step(world);
    }

    // Update time for animation
    muscle_state.time += time.delta_secs_f64();
}

/// Handle keyboard input.
fn handle_input(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut config: ResMut<ViewerConfig>,
    mut muscle_state: ResMut<MuscleState>,
    mut paused: ResMut<SimulationPaused>,
) {
    // Toggle visualization modes
    if keyboard.just_pressed(KeyCode::KeyC) {
        config.show_collision_shapes = !config.show_collision_shapes;
        println!(
            "Collision shapes: {}",
            if config.show_collision_shapes {
                "ON"
            } else {
                "OFF"
            }
        );
    }

    if keyboard.just_pressed(KeyCode::KeyJ) {
        config.show_joint_axes = !config.show_joint_axes;
        println!(
            "Joint axes: {}",
            if config.show_joint_axes { "ON" } else { "OFF" }
        );
    }

    if keyboard.just_pressed(KeyCode::KeyM) {
        config.show_muscles = !config.show_muscles;
        println!(
            "Muscles: {}",
            if config.show_muscles { "ON" } else { "OFF" }
        );
    }

    if keyboard.just_pressed(KeyCode::KeyT) {
        config.show_tendons = !config.show_tendons;
        println!(
            "Tendons: {}",
            if config.show_tendons { "ON" } else { "OFF" }
        );
    }

    if keyboard.just_pressed(KeyCode::KeyS) {
        config.show_sensors = !config.show_sensors;
        println!(
            "Sensors: {}",
            if config.show_sensors { "ON" } else { "OFF" }
        );
    }

    if keyboard.just_pressed(KeyCode::Space) {
        paused.0 = !paused.0;
        println!(
            "Simulation: {}",
            if paused.0 { "PAUSED" } else { "RUNNING" }
        );
    }

    // Reset activations
    if keyboard.just_pressed(KeyCode::KeyR) {
        muscle_state.biceps = 0.0;
        muscle_state.triceps = 0.0;
        muscle_state.brachialis = 0.0;
        muscle_state.wrist_flexor = 0.0;
        muscle_state.wrist_extensor = 0.0;
        println!("Muscle activations reset");
    }
}

/// Update muscle activations based on key presses.
fn update_muscle_activations(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut muscle_state: ResMut<MuscleState>,
    time: Res<Time>,
) {
    let activation_rate = 3.0; // How fast muscles activate
    let deactivation_rate = 2.0; // How fast muscles relax
    let dt = time.delta_secs_f64();

    // Helper to update activation
    let update = |current: &mut f64, is_pressed: bool| {
        if is_pressed {
            *current = (*current + activation_rate * dt).min(1.0);
        } else {
            *current = (*current - deactivation_rate * dt).max(0.0);
        }
    };

    // Key 1: Biceps
    update(&mut muscle_state.biceps, keyboard.pressed(KeyCode::Digit1));

    // Key 2: Triceps
    update(&mut muscle_state.triceps, keyboard.pressed(KeyCode::Digit2));

    // Key 3: Brachialis
    update(
        &mut muscle_state.brachialis,
        keyboard.pressed(KeyCode::Digit3),
    );

    // Key 4: Wrist flexor
    update(
        &mut muscle_state.wrist_flexor,
        keyboard.pressed(KeyCode::Digit4),
    );

    // Key 5: Wrist extensor
    update(
        &mut muscle_state.wrist_extensor,
        keyboard.pressed(KeyCode::Digit5),
    );
}

/// Update visualization resources with current muscle/tendon/sensor state.
fn update_visualizations(
    sim_handle: Res<SimulationHandle>,
    arm_ids: Option<Res<ArmBodyIds>>,
    muscle_state: Res<MuscleState>,
    mut muscles_vis: ResMut<MuscleVisualization>,
    mut tendons_vis: ResMut<TendonVisualization>,
    mut sensors_vis: ResMut<SensorVisualization>,
) {
    // Clear previous frame's data
    muscles_vis.clear();
    tendons_vis.clear();
    sensors_vis.clear();

    let Some(world) = sim_handle.world() else {
        return;
    };
    let Some(arm) = arm_ids else {
        return;
    };

    // Get body poses for attachment point calculation
    let base_pose = arm
        .base
        .and_then(|id| world.body(id))
        .map(|b| b.state.pose)
        .unwrap_or_default();
    let upper_arm_pose = arm
        .upper_arm
        .and_then(|id| world.body(id))
        .map(|b| b.state.pose)
        .unwrap_or_default();
    let forearm_pose = arm
        .forearm
        .and_then(|id| world.body(id))
        .map(|b| b.state.pose)
        .unwrap_or_default();
    let hand_pose = arm
        .hand
        .and_then(|id| world.body(id))
        .map(|b| b.state.pose)
        .unwrap_or_default();

    // Helper to transform a local point to world coordinates
    let to_world = |pose: &sim_types::Pose, local: Point3<f64>| -> Point3<f64> {
        pose.position + pose.rotation * local.coords
    };

    // ==========================================================================
    // MUSCLES
    // ==========================================================================

    // Biceps: from shoulder (base) to forearm
    // Origin on front of shoulder, insertion on radius (forearm)
    let biceps_origin = to_world(&base_pose, Point3::new(0.05, 0.0, -0.1));
    let biceps_insertion = to_world(&forearm_pose, Point3::new(0.03, 0.0, 0.05));
    let biceps_via = to_world(&upper_arm_pose, Point3::new(0.06, 0.0, -0.15));
    muscles_vis.add(
        MuscleVisualData::new("Biceps", biceps_origin, biceps_insertion)
            .with_activation(muscle_state.biceps)
            .with_via_points(vec![biceps_via])
            .with_force(muscle_state.biceps * 150.0), // Max ~150N
    );

    // Triceps: from shoulder (back) to elbow (olecranon)
    let triceps_origin = to_world(&base_pose, Point3::new(-0.05, 0.0, -0.1));
    let triceps_insertion = to_world(&forearm_pose, Point3::new(-0.02, 0.0, 0.08));
    let triceps_via = to_world(&upper_arm_pose, Point3::new(-0.05, 0.0, -0.2));
    muscles_vis.add(
        MuscleVisualData::new("Triceps", triceps_origin, triceps_insertion)
            .with_activation(muscle_state.triceps)
            .with_via_points(vec![triceps_via])
            .with_force(muscle_state.triceps * 120.0),
    );

    // Brachialis: deep muscle from humerus to ulna
    let brachialis_origin = to_world(&upper_arm_pose, Point3::new(0.02, 0.0, -0.2));
    let brachialis_insertion = to_world(&forearm_pose, Point3::new(0.0, 0.0, 0.03));
    muscles_vis.add(
        MuscleVisualData::new("Brachialis", brachialis_origin, brachialis_insertion)
            .with_activation(muscle_state.brachialis)
            .with_force(muscle_state.brachialis * 80.0),
    );

    // Wrist flexor: from forearm to palm
    let wrist_flexor_origin = to_world(&forearm_pose, Point3::new(0.02, 0.0, -0.1));
    let wrist_flexor_insertion = to_world(&hand_pose, Point3::new(0.02, 0.0, 0.0));
    muscles_vis.add(
        MuscleVisualData::new("Wrist Flexor", wrist_flexor_origin, wrist_flexor_insertion)
            .with_activation(muscle_state.wrist_flexor)
            .with_force(muscle_state.wrist_flexor * 30.0),
    );

    // Wrist extensor: from forearm to back of hand
    let wrist_extensor_origin = to_world(&forearm_pose, Point3::new(-0.02, 0.0, -0.1));
    let wrist_extensor_insertion = to_world(&hand_pose, Point3::new(-0.02, 0.0, 0.0));
    muscles_vis.add(
        MuscleVisualData::new(
            "Wrist Extensor",
            wrist_extensor_origin,
            wrist_extensor_insertion,
        )
        .with_activation(muscle_state.wrist_extensor)
        .with_force(muscle_state.wrist_extensor * 25.0),
    );

    // ==========================================================================
    // TENDONS
    // ==========================================================================

    // Biceps tendon: long tendon from biceps insertion to forearm
    let biceps_tendon_path = vec![
        biceps_insertion,
        to_world(&forearm_pose, Point3::new(0.025, 0.0, 0.0)),
        to_world(&forearm_pose, Point3::new(0.02, 0.0, -0.05)),
    ];
    tendons_vis.add(
        TendonVisualData::new("Biceps Tendon", biceps_tendon_path)
            .with_tension(muscle_state.biceps * 100.0)
            .with_length(0.08, 0.075),
    );

    // Finger flexor tendon (running through the forearm to hand)
    let flexor_tendon_path = vec![
        to_world(&forearm_pose, Point3::new(0.0, 0.02, -0.05)),
        to_world(&forearm_pose, Point3::new(0.0, 0.02, -0.15)),
        to_world(&forearm_pose, Point3::new(0.0, 0.02, -0.25)),
        to_world(&hand_pose, Point3::new(0.0, 0.02, -0.02)),
    ];
    // Animate tendon tension
    let flexor_tension = (muscle_state.time * 2.0).sin().abs() * 50.0;
    tendons_vis.add(
        TendonVisualData::new("Finger Flexor", flexor_tendon_path)
            .with_tension(flexor_tension)
            .with_length(0.35, 0.33),
    );

    // ==========================================================================
    // SENSORS
    // ==========================================================================

    // IMU on the forearm
    let forearm_center = to_world(&forearm_pose, Point3::new(0.0, 0.0, -0.15));
    sensors_vis.add(SensorVisualData::imu(
        forearm_center,
        UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            forearm_pose.rotation.w,
            forearm_pose.rotation.i,
            forearm_pose.rotation.j,
            forearm_pose.rotation.k,
        )),
    ));

    // Force/torque sensor at the wrist
    let wrist_pos = to_world(&forearm_pose, Point3::new(0.0, 0.0, -0.28));
    // Simulate some force based on muscle activation
    let grip_force = (muscle_state.wrist_flexor + muscle_state.wrist_extensor) * 10.0;
    sensors_vis.add(SensorVisualData::force_torque(
        wrist_pos,
        UnitQuaternion::identity(),
        Vector3::new(0.0, 0.0, -grip_force),
        Vector3::new(muscle_state.wrist_flexor * 2.0, 0.0, 0.0),
    ));

    // Touch sensors on the fingertips (simulated as active when wrist flexor is engaged)
    let finger_tips = [
        to_world(&hand_pose, Point3::new(0.02, 0.04, -0.01)),
        to_world(&hand_pose, Point3::new(0.0, 0.05, -0.01)),
        to_world(&hand_pose, Point3::new(-0.02, 0.04, -0.01)),
    ];
    for (i, tip) in finger_tips.iter().enumerate() {
        let is_touching = muscle_state.wrist_flexor > 0.5 + (i as f64) * 0.15;
        sensors_vis.add(SensorVisualData::touch(*tip, is_touching));
    }

    // Rangefinder on the hand pointing forward
    let rangefinder_pos = to_world(&hand_pose, Point3::new(0.0, 0.0, -0.02));
    // Direction is hand's forward direction (negative Z in hand frame)
    let hand_forward = forearm_pose.rotation * Vector3::new(0.0, 0.0, -1.0);
    // Simulate distance (oscillating for demo)
    let range_distance = 0.3 + (muscle_state.time * 1.5).sin().abs() * 0.5;
    sensors_vis.add(SensorVisualData::rangefinder(
        rangefinder_pos,
        hand_forward,
        range_distance,
    ));

    // Magnetometer on upper arm
    let magnetometer_pos = to_world(&upper_arm_pose, Point3::new(0.0, 0.0, -0.2));
    // Earth's magnetic field pointing roughly north
    let magnetic_field = Vector3::new(0.2, 0.8, -0.3).normalize();
    sensors_vis.add(SensorVisualData::magnetometer(
        magnetometer_pos,
        magnetic_field,
    ));
}

/// Print help information once.
fn print_help() {
    println!();
    println!("╔═══════════════════════════════════════════════════════════════╗");
    println!("║           Musculoskeletal Visualization Demo                   ║");
    println!("╠═══════════════════════════════════════════════════════════════╣");
    println!("║  This demo shows Phase 3.1 visualization features:             ║");
    println!("║  • Muscles: color changes from blue (relaxed) to red (active)  ║");
    println!("║  • Tendons: opacity increases with tension                     ║");
    println!("║  • Sensors: IMU frames, force arrows, touch, rangefinder       ║");
    println!("╠═══════════════════════════════════════════════════════════════╣");
    println!("║  Controls:                                                      ║");
    println!("║    Mouse drag  - Rotate camera                                  ║");
    println!("║    Scroll      - Zoom                                           ║");
    println!("║    C           - Toggle collision shapes                        ║");
    println!("║    J           - Toggle joint axes                              ║");
    println!("║    M           - Toggle muscle visualization                    ║");
    println!("║    T           - Toggle tendon visualization                    ║");
    println!("║    S           - Toggle sensor visualization                    ║");
    println!("║    Space       - Pause/resume simulation                        ║");
    println!("║    1-5         - Activate muscles (hold)                        ║");
    println!("║    R           - Reset all muscle activations                   ║");
    println!("╚═══════════════════════════════════════════════════════════════╝");
    println!();
}
