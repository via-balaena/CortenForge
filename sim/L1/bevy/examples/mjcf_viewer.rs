//! Example: MJCF model viewer.
//!
//! Demonstrates loading and visualizing MJCF (MuJoCo XML) models.
//! This example shows how to load a model from a string and spawn it
//! into the physics simulation for visualization.
//!
//! Run with: `cargo run -p sim-bevy --example mjcf_viewer --features x11`
//! (or `--features wayland` on Wayland systems)

#![allow(clippy::needless_pass_by_value)] // Bevy system parameters

use bevy::prelude::*;
use sim_bevy::prelude::*;
use sim_core::Stepper;

/// A simple humanoid-like MJCF model for demonstration.
const SIMPLE_HUMANOID_MJCF: &str = r#"
<mujoco model="simple_humanoid">
    <option gravity="0 0 -9.81" timestep="0.002"/>

    <default>
        <geom rgba="0.8 0.6 0.4 1"/>
        <joint damping="1.0"/>
    </default>

    <worldbody>
        <!-- Ground plane -->
        <geom type="plane" size="5 5 0.1" rgba="0.3 0.3 0.35 1"/>

        <!-- Torso -->
        <body name="torso" pos="0 0 1.5">
            <joint type="free"/>
            <geom type="capsule" fromto="0 0 0 0 0 0.4" size="0.15"/>

            <!-- Head -->
            <body name="head" pos="0 0 0.5">
                <joint name="neck" type="ball" damping="2.0"/>
                <geom type="sphere" size="0.12" rgba="0.9 0.7 0.6 1"/>
            </body>

            <!-- Left Arm -->
            <body name="left_upper_arm" pos="0.2 0 0.35">
                <joint name="left_shoulder" type="ball" damping="0.5"/>
                <geom type="capsule" fromto="0 0 0 0.25 0 0" size="0.04"/>

                <body name="left_lower_arm" pos="0.28 0 0">
                    <joint name="left_elbow" type="hinge" axis="0 1 0" range="-2.5 0"/>
                    <geom type="capsule" fromto="0 0 0 0.22 0 0" size="0.035"/>
                </body>
            </body>

            <!-- Right Arm -->
            <body name="right_upper_arm" pos="-0.2 0 0.35">
                <joint name="right_shoulder" type="ball" damping="0.5"/>
                <geom type="capsule" fromto="0 0 0 -0.25 0 0" size="0.04"/>

                <body name="right_lower_arm" pos="-0.28 0 0">
                    <joint name="right_elbow" type="hinge" axis="0 1 0" range="0 2.5"/>
                    <geom type="capsule" fromto="0 0 0 -0.22 0 0" size="0.035"/>
                </body>
            </body>

            <!-- Pelvis -->
            <body name="pelvis" pos="0 0 -0.15">
                <joint name="spine" type="ball" damping="2.0"/>
                <geom type="capsule" fromto="-0.1 0 0 0.1 0 0" size="0.08"/>

                <!-- Left Leg -->
                <body name="left_upper_leg" pos="0.1 0 -0.05">
                    <joint name="left_hip" type="ball" damping="1.0"/>
                    <geom type="capsule" fromto="0 0 0 0 0 -0.35" size="0.055"/>

                    <body name="left_lower_leg" pos="0 0 -0.38">
                        <joint name="left_knee" type="hinge" axis="1 0 0" range="-2.5 0"/>
                        <geom type="capsule" fromto="0 0 0 0 0 -0.32" size="0.045"/>

                        <body name="left_foot" pos="0 0 -0.35">
                            <joint name="left_ankle" type="ball"/>
                            <geom type="box" size="0.05 0.09 0.02" pos="0 0.03 0"/>
                        </body>
                    </body>
                </body>

                <!-- Right Leg -->
                <body name="right_upper_leg" pos="-0.1 0 -0.05">
                    <joint name="right_hip" type="ball" damping="1.0"/>
                    <geom type="capsule" fromto="0 0 0 0 0 -0.35" size="0.055"/>

                    <body name="right_lower_leg" pos="0 0 -0.38">
                        <joint name="right_knee" type="hinge" axis="1 0 0" range="-2.5 0"/>
                        <geom type="capsule" fromto="0 0 0 0 0 -0.32" size="0.045"/>

                        <body name="right_foot" pos="0 0 -0.35">
                            <joint name="right_ankle" type="ball"/>
                            <geom type="box" size="0.05 0.09 0.02" pos="0 0.03 0"/>
                        </body>
                    </body>
                </body>
            </body>
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
        .add_systems(Update, step_physics)
        .add_systems(Update, toggle_debug_viz)
        .add_systems(Update, print_model_info.run_if(run_once))
        .run();
}

/// Set up the scene by loading an MJCF model.
fn setup_scene(mut commands: Commands) {
    // Create an empty world
    let world = sim_core::World::default();
    let mut sim_handle = SimulationHandle::new(world);

    // Load the MJCF model
    match MjcfModel::from_xml(SIMPLE_HUMANOID_MJCF) {
        Ok(model) => {
            println!("Loaded MJCF model: {}", model.name());
            println!("  Bodies: {}", model.body_count());
            println!("  Joints: {}", model.joint_count());

            // Spawn the model into the physics world
            match model.spawn_into(&mut sim_handle) {
                Ok(spawned) => {
                    println!("Spawned model: {}", spawned.name());

                    // Store the spawned model info as a resource for later queries
                    commands.insert_resource(SpawnedModelInfo {
                        name: spawned.name().to_string(),
                        torso_id: spawned.body_id("torso"),
                    });
                }
                Err(e) => {
                    eprintln!("Failed to spawn model: {e}");
                }
            }
        }
        Err(e) => {
            eprintln!("Failed to load MJCF model: {e}");
        }
    }

    // Insert the simulation handle
    commands.insert_resource(sim_handle);

    // Print controls
    println!();
    println!("MJCF Viewer Example");
    println!("===================");
    println!("Controls:");
    println!("  Mouse drag: Rotate camera");
    println!("  Scroll: Zoom");
    println!("  C: Toggle collision shapes");
    println!("  J: Toggle joint axes");
    println!("  N: Toggle contact normals");
    println!("  V: Toggle velocity vectors");
}

/// Resource to store spawned model information.
#[derive(Resource)]
struct SpawnedModelInfo {
    name: String,
    torso_id: Option<sim_types::BodyId>,
}

/// Resource to hold the physics stepper.
#[derive(Resource, Default)]
struct PhysicsStepper {
    stepper: Stepper,
}

/// Step the physics simulation each frame.
fn step_physics(
    mut sim_handle: ResMut<SimulationHandle>,
    mut physics_stepper: ResMut<PhysicsStepper>,
) {
    if let Some(world) = sim_handle.world_mut() {
        let _ = physics_stepper.stepper.step(world);
    }
}

/// Print model information once after the first frame.
fn print_model_info(model_info: Option<Res<SpawnedModelInfo>>, sim_handle: Res<SimulationHandle>) {
    if let Some(info) = model_info {
        if let Some(world) = sim_handle.world() {
            println!();
            println!("Model '{}' is now simulating.", info.name);
            println!("Total bodies in world: {}", world.body_count());
            println!("Total joints in world: {}", world.joint_count());

            if let Some(torso_id) = info.torso_id {
                if let Some(body) = world.body(torso_id) {
                    let pos = body.state.pose.position;
                    println!("Torso position: ({:.2}, {:.2}, {:.2})", pos.x, pos.y, pos.z);
                }
            }
        }
    }
}

/// Toggle debug visualization options with keyboard.
fn toggle_debug_viz(keyboard: Res<ButtonInput<KeyCode>>, mut config: ResMut<ViewerConfig>) {
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

    if keyboard.just_pressed(KeyCode::KeyN) {
        config.show_contact_normals = !config.show_contact_normals;
        println!(
            "Contact normals: {}",
            if config.show_contact_normals {
                "ON"
            } else {
                "OFF"
            }
        );
    }

    if keyboard.just_pressed(KeyCode::KeyV) {
        config.show_velocities = !config.show_velocities;
        println!(
            "Velocity vectors: {}",
            if config.show_velocities { "ON" } else { "OFF" }
        );
    }
}
