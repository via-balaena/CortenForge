//! Minimal example: a sphere falling under gravity.
//!
//! This demonstrates the basic sim-bevy workflow with the Model/Data API:
//! 1. Create a Model from MJCF (static configuration)
//! 2. Create Data from the Model (dynamic state)
//! 3. Add the ModelDataPlugin for automatic stepping and sync
//!
//! Run with: `cargo run -p sim-bevy --example falling_sphere --features x11`
//! (or `--features wayland` on Wayland systems)

#![allow(clippy::expect_used)] // Examples use expect for clarity
#![allow(clippy::needless_pass_by_value)] // Bevy system parameters

use bevy::prelude::*;
use sim_bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimViewerPlugin::new())
        .add_plugins(ModelDataPlugin::new().with_auto_step())
        .add_systems(Startup, setup_physics)
        .run();
}

/// Set up the physics simulation with a falling sphere and ground plane.
fn setup_physics(mut commands: Commands) {
    // Define the scene using MJCF (MuJoCo XML Format)
    let mjcf = r#"
        <mujoco model="falling_sphere">
            <option gravity="0 0 -9.81" timestep="0.002"/>
            <worldbody>
                <!-- Ground plane -->
                <geom name="ground" type="plane" size="10 10 0.1" rgba="0.8 0.8 0.8 1"/>

                <!-- Falling sphere at Z=5 -->
                <body name="sphere" pos="0 0 5">
                    <freejoint name="sphere_joint"/>
                    <geom name="sphere_geom" type="sphere" size="0.5" mass="1.0" rgba="0.2 0.6 1.0 1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    // Load the model (static configuration)
    let model = load_model(mjcf).expect("Failed to load MJCF model");

    // Create data (dynamic state) and run initial forward pass
    let mut data = model.make_data();
    data.forward(&model);

    // Insert as Bevy resources
    commands.insert_resource(PhysicsModel::new(model));
    commands.insert_resource(PhysicsData::new(data));
}
