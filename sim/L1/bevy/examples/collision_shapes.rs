//! Example: All primitive collision shapes.
//!
//! Displays all supported collision shape types in a grid layout.
//!
//! Run with: `cargo run -p sim-bevy --example collision_shapes --features x11`
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
        .add_systems(Startup, setup_shapes)
        .run();
}

/// Set up the physics world with all primitive collision shapes.
fn setup_shapes(mut commands: Commands) {
    // Define a grid of various collision shapes using MJCF
    // Grid layout: 2 rows, 4 columns with spacing=3.0
    let mjcf = r#"
        <mujoco model="collision_shapes">
            <option gravity="0 0 -9.81" timestep="0.002"/>
            <worldbody>
                <!-- Ground plane -->
                <geom name="ground" type="plane" size="20 20 0.1" rgba="0.8 0.8 0.8 1"/>

                <!-- Row 1 (z=1.5): Sphere, Box, Capsule, Cylinder -->
                <body name="sphere" pos="-4.5 1.5 1.5">
                    <freejoint name="sphere_joint"/>
                    <geom name="sphere_geom" type="sphere" size="0.8" mass="1.0" rgba="0.9 0.3 0.3 1"/>
                </body>

                <body name="box" pos="-1.5 1.5 1.5">
                    <freejoint name="box_joint"/>
                    <geom name="box_geom" type="box" size="0.6 0.8 0.5" mass="1.0" rgba="0.3 0.9 0.3 1"/>
                </body>

                <body name="capsule" pos="1.5 1.5 1.5">
                    <freejoint name="capsule_joint"/>
                    <geom name="capsule_geom" type="capsule" size="0.3 0.5" mass="1.0" rgba="0.3 0.3 0.9 1"/>
                </body>

                <body name="cylinder" pos="4.5 1.5 1.5">
                    <freejoint name="cylinder_joint"/>
                    <geom name="cylinder_geom" type="cylinder" size="0.4 0.6" mass="1.0" rgba="0.9 0.9 0.3 1"/>
                </body>

                <!-- Row 2 (z=-1.5): Ellipsoid, Tall Box, Small Sphere, Wide Cylinder -->
                <body name="ellipsoid" pos="-4.5 1.5 -1.5">
                    <freejoint name="ellipsoid_joint"/>
                    <geom name="ellipsoid_geom" type="ellipsoid" size="0.8 0.5 0.4" mass="1.0" rgba="0.9 0.3 0.9 1"/>
                </body>

                <body name="tall_box" pos="-1.5 1.5 -1.5">
                    <freejoint name="tall_box_joint"/>
                    <geom name="tall_box_geom" type="box" size="0.3 1.0 0.3" mass="1.0" rgba="0.3 0.9 0.9 1"/>
                </body>

                <body name="small_sphere" pos="1.5 1.5 -1.5">
                    <freejoint name="small_sphere_joint"/>
                    <geom name="small_sphere_geom" type="sphere" size="0.4" mass="1.0" rgba="0.9 0.6 0.3 1"/>
                </body>

                <body name="wide_cylinder" pos="4.5 1.5 -1.5">
                    <freejoint name="wide_cylinder_joint"/>
                    <geom name="wide_cylinder_geom" type="cylinder" size="0.8 0.3" mass="1.0" rgba="0.6 0.3 0.9 1"/>
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
}
