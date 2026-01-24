//! Example: URDF model viewer.
//!
//! Demonstrates loading and visualizing URDF (Unified Robot Description Format) models.
//! This example shows how to load a robot from a string and spawn it
//! into the physics simulation for visualization.
//!
//! Run with: `cargo run -p sim-bevy --example urdf_viewer --features x11`
//! (or `--features wayland` on Wayland systems)

#![allow(clippy::needless_pass_by_value)] // Bevy system parameters

use bevy::prelude::*;
use sim_bevy::prelude::*;
use sim_core::Stepper;

/// A simple robot arm URDF model for demonstration.
const SIMPLE_ARM_URDF: &str = r#"
<?xml version="1.0"?>
<robot name="simple_arm">
    <!-- Base link (fixed to world) -->
    <link name="base_link">
        <inertial>
            <mass value="5.0"/>
            <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
        </inertial>
        <collision>
            <geometry>
                <cylinder radius="0.15" length="0.1"/>
            </geometry>
        </collision>
    </link>

    <!-- Shoulder link -->
    <link name="shoulder_link">
        <inertial>
            <mass value="2.0"/>
            <inertia ixx="0.05" iyy="0.05" izz="0.02" ixy="0" ixz="0" iyz="0"/>
        </inertial>
        <collision>
            <geometry>
                <cylinder radius="0.08" length="0.15"/>
            </geometry>
        </collision>
    </link>

    <!-- Shoulder joint (revolute around Z) -->
    <joint name="shoulder_yaw" type="revolute">
        <parent link="base_link"/>
        <child link="shoulder_link"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
    </joint>

    <!-- Upper arm link -->
    <link name="upper_arm_link">
        <inertial>
            <mass value="1.5"/>
            <inertia ixx="0.04" iyy="0.04" izz="0.01" ixy="0" ixz="0" iyz="0"/>
        </inertial>
        <collision>
            <geometry>
                <box size="0.08 0.08 0.4"/>
            </geometry>
        </collision>
    </link>

    <!-- Shoulder pitch joint -->
    <joint name="shoulder_pitch" type="revolute">
        <parent link="shoulder_link"/>
        <child link="upper_arm_link"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57" effort="80" velocity="1.0"/>
    </joint>

    <!-- Lower arm link -->
    <link name="lower_arm_link">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.03" iyy="0.03" izz="0.005" ixy="0" ixz="0" iyz="0"/>
        </inertial>
        <collision>
            <geometry>
                <box size="0.06 0.06 0.35"/>
            </geometry>
        </collision>
    </link>

    <!-- Elbow joint -->
    <joint name="elbow" type="revolute">
        <parent link="upper_arm_link"/>
        <child link="lower_arm_link"/>
        <origin xyz="0 0 0.35" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="0" upper="2.5" effort="60" velocity="1.5"/>
    </joint>

    <!-- Wrist link -->
    <link name="wrist_link">
        <inertial>
            <mass value="0.5"/>
            <inertia ixx="0.01" iyy="0.01" izz="0.005" ixy="0" ixz="0" iyz="0"/>
        </inertial>
        <collision>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
        </collision>
    </link>

    <!-- Wrist joint -->
    <joint name="wrist" type="revolute">
        <parent link="lower_arm_link"/>
        <child link="wrist_link"/>
        <origin xyz="0 0 0.3" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="30" velocity="2.0"/>
    </joint>

    <!-- Gripper base link -->
    <link name="gripper_base">
        <inertial>
            <mass value="0.3"/>
            <inertia ixx="0.005" iyy="0.005" izz="0.003" ixy="0" ixz="0" iyz="0"/>
        </inertial>
        <collision>
            <geometry>
                <box size="0.08 0.04 0.02"/>
            </geometry>
        </collision>
    </link>

    <!-- Gripper base joint -->
    <joint name="gripper_base_joint" type="fixed">
        <parent link="wrist_link"/>
        <child link="gripper_base"/>
        <origin xyz="0 0 0.06" rpy="0 0 0"/>
    </joint>

    <!-- Left finger -->
    <link name="left_finger">
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
        </inertial>
        <collision>
            <geometry>
                <box size="0.02 0.02 0.08"/>
            </geometry>
        </collision>
    </link>

    <!-- Left finger joint (prismatic) -->
    <joint name="left_finger_joint" type="prismatic">
        <parent link="gripper_base"/>
        <child link="left_finger"/>
        <origin xyz="0.03 0 0.04" rpy="0 0 0"/>
        <axis xyz="1 0 0"/>
        <limit lower="0" upper="0.04" effort="10" velocity="0.5"/>
    </joint>

    <!-- Right finger -->
    <link name="right_finger">
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
        </inertial>
        <collision>
            <geometry>
                <box size="0.02 0.02 0.08"/>
            </geometry>
        </collision>
    </link>

    <!-- Right finger joint (prismatic) -->
    <joint name="right_finger_joint" type="prismatic">
        <parent link="gripper_base"/>
        <child link="right_finger"/>
        <origin xyz="-0.03 0 0.04" rpy="0 0 0"/>
        <axis xyz="1 0 0"/>
        <limit lower="-0.04" upper="0" effort="10" velocity="0.5"/>
    </joint>
</robot>
"#;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimViewerPlugin::new())
        .add_systems(Startup, setup_scene)
        .init_resource::<PhysicsStepper>()
        .add_systems(Update, step_physics)
        .add_systems(Update, toggle_debug_viz)
        .add_systems(Update, print_robot_info.run_if(run_once))
        .run();
}

/// Set up the scene by loading a URDF robot.
fn setup_scene(mut commands: Commands) {
    // Create an empty world
    let world = sim_core::World::default();
    let mut sim_handle = SimulationHandle::new(world);

    // Load the URDF robot
    match UrdfModel::from_xml(SIMPLE_ARM_URDF) {
        Ok(robot) => {
            println!("Loaded URDF robot: {}", robot.name());
            println!("  Links: {}", robot.link_count());
            println!("  Joints: {}", robot.joint_count());

            // Spawn the robot into the physics world
            match robot.spawn_into(&mut sim_handle) {
                Ok(spawned) => {
                    println!("Spawned robot: {}", spawned.name());

                    // Store the spawned robot info as a resource
                    commands.insert_resource(SpawnedRobotInfo {
                        name: spawned.name().to_string(),
                        base_id: spawned.link_id("base_link"),
                        gripper_id: spawned.link_id("gripper_base"),
                    });
                }
                Err(e) => {
                    eprintln!("Failed to spawn robot: {e}");
                }
            }
        }
        Err(e) => {
            eprintln!("Failed to load URDF robot: {e}");
        }
    }

    // Insert the simulation handle
    commands.insert_resource(sim_handle);

    // Print controls
    println!();
    println!("URDF Viewer Example");
    println!("===================");
    println!("Controls:");
    println!("  Mouse drag: Rotate camera");
    println!("  Scroll: Zoom");
    println!("  C: Toggle collision shapes");
    println!("  J: Toggle joint axes");
    println!("  L: Toggle joint limits");
}

/// Resource to store spawned robot information.
#[derive(Resource)]
struct SpawnedRobotInfo {
    name: String,
    base_id: Option<sim_types::BodyId>,
    gripper_id: Option<sim_types::BodyId>,
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

/// Print robot information once after the first frame.
fn print_robot_info(robot_info: Option<Res<SpawnedRobotInfo>>, sim_handle: Res<SimulationHandle>) {
    if let Some(info) = robot_info {
        if let Some(world) = sim_handle.world() {
            println!();
            println!("Robot '{}' is now simulating.", info.name);
            println!("Total bodies in world: {}", world.body_count());
            println!("Total joints in world: {}", world.joint_count());

            if let Some(base_id) = info.base_id {
                if let Some(body) = world.body(base_id) {
                    let pos = body.state.pose.position;
                    println!("Base position: ({:.2}, {:.2}, {:.2})", pos.x, pos.y, pos.z);
                }
            }

            if let Some(gripper_id) = info.gripper_id {
                if let Some(body) = world.body(gripper_id) {
                    let pos = body.state.pose.position;
                    println!(
                        "Gripper position: ({:.2}, {:.2}, {:.2})",
                        pos.x, pos.y, pos.z
                    );
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

    if keyboard.just_pressed(KeyCode::KeyL) {
        config.show_joint_limits = !config.show_joint_limits;
        println!(
            "Joint limits: {}",
            if config.show_joint_limits {
                "ON"
            } else {
                "OFF"
            }
        );
    }
}
