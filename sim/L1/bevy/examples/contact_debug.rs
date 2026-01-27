//! Example: Contact debug visualization.
//!
//! Demonstrates the debug visualization features for contacts, forces, and joints.
//! Multiple spheres fall and collide, showing contact points and normals.
//!
//! Run with: `cargo run -p sim-bevy --example contact_debug --features x11`
//! (or `--features wayland` on Wayland systems)

#![allow(clippy::needless_pass_by_value)] // Bevy system parameters
#![allow(clippy::expect_used)] // Examples use expect for clarity

use bevy::prelude::*;
use sim_bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimViewerPlugin::new())
        .add_plugins(ModelDataPlugin::new().with_auto_step())
        .add_systems(Startup, setup_scene)
        .add_systems(Update, toggle_debug_viz)
        .run();
}

/// Set up a scene with multiple falling bodies.
fn setup_scene(mut commands: Commands) {
    // Define the scene using MJCF
    let mjcf = r#"
        <mujoco model="contact_debug">
            <option gravity="0 0 -9.81" timestep="0.002"/>
            <worldbody>
                <!-- Ground plane -->
                <geom name="ground" type="plane" size="10 10 0.1" rgba="0.8 0.8 0.8 1"/>

                <!-- Multiple falling spheres at different heights -->
                <body name="sphere1" pos="0 0 3">
                    <freejoint name="sphere1_joint"/>
                    <geom name="sphere1_geom" type="sphere" size="0.4" mass="1.0" rgba="0.2 0.6 1.0 1"/>
                </body>

                <body name="sphere2" pos="0.5 0.3 4">
                    <freejoint name="sphere2_joint"/>
                    <geom name="sphere2_geom" type="sphere" size="0.4" mass="1.0" rgba="0.2 0.8 0.4 1"/>
                </body>

                <body name="sphere3" pos="-0.3 -0.2 5">
                    <freejoint name="sphere3_joint"/>
                    <geom name="sphere3_geom" type="sphere" size="0.4" mass="1.0" rgba="0.8 0.4 0.2 1"/>
                </body>

                <body name="sphere4" pos="0.2 0.5 6">
                    <freejoint name="sphere4_joint"/>
                    <geom name="sphere4_geom" type="sphere" size="0.4" mass="1.0" rgba="0.6 0.2 0.8 1"/>
                </body>

                <!-- A box to demonstrate box-sphere contacts -->
                <body name="box1" pos="-1.5 0 2">
                    <freejoint name="box1_joint"/>
                    <geom name="box1_geom" type="box" size="0.4 0.4 0.4" mass="2.0" rgba="1.0 0.8 0.2 1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    // Load the model
    let model = load_model(mjcf).expect("Failed to load MJCF model");

    // Create data and run initial forward pass
    let mut data = model.make_data();
    data.forward(&model);

    // Insert as Bevy resources
    commands.insert_resource(PhysicsModel::new(model));
    commands.insert_resource(PhysicsData::new(data));

    println!("Contact Debug Example");
    println!("=====================");
    println!("Press 'C' to toggle contact point visualization");
    println!("Press 'N' to toggle contact normal arrows");
    println!("Press 'F' to toggle force vector visualization");
    println!("Press 'J' to toggle joint axis visualization");
    println!("Press 'L' to toggle joint limit visualization");
    println!();
    println!("Use mouse to orbit camera:");
    println!("  Left button + drag: rotate");
    println!("  Scroll: zoom");
}

/// Toggle debug visualization options with keyboard.
fn toggle_debug_viz(keyboard: Res<ButtonInput<KeyCode>>, mut config: ResMut<ViewerConfig>) {
    if keyboard.just_pressed(KeyCode::KeyC) {
        config.show_contacts = !config.show_contacts;
        println!(
            "Contact points: {}",
            if config.show_contacts { "ON" } else { "OFF" }
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

    if keyboard.just_pressed(KeyCode::KeyF) {
        config.show_forces = !config.show_forces;
        println!(
            "Force vectors: {}",
            if config.show_forces { "ON" } else { "OFF" }
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
