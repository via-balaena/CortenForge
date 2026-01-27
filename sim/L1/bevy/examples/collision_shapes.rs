//! Example: All primitive collision shapes.
//!
//! Displays all supported collision shape types in a grid layout.
//!
//! Run with: `cargo run -p sim-bevy --example collision_shapes --features x11`
//! (or `--features wayland` on Wayland systems)

#![allow(clippy::needless_pass_by_value)] // Bevy system parameters

use bevy::prelude::*;
use sim_bevy::prelude::*;
use sim_core::world::{CollisionShape, World};
use sim_types::{MassProperties, Pose, RigidBodyState};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimViewerPlugin::new())
        .add_systems(Startup, setup_shapes)
        .run();
}

/// Set up the physics world with all primitive collision shapes.
fn setup_shapes(mut commands: Commands) {
    let mut world = World::default();

    // Grid layout: 2 rows, 4 columns
    let spacing = 3.0;
    let shapes: Vec<(&str, CollisionShape)> = vec![
        // Row 1
        ("Sphere", CollisionShape::sphere(0.8)),
        (
            "Box",
            CollisionShape::box_shape(nalgebra::Vector3::new(0.6, 0.8, 0.5)),
        ),
        ("Capsule", CollisionShape::capsule(0.5, 0.3)),
        ("Cylinder", CollisionShape::cylinder(0.6, 0.4)),
        // Row 2
        (
            "Ellipsoid",
            CollisionShape::ellipsoid(nalgebra::Vector3::new(0.8, 0.5, 0.4)),
        ),
        (
            "Tall Box",
            CollisionShape::box_shape(nalgebra::Vector3::new(0.3, 1.0, 0.3)),
        ),
        ("Small Sphere", CollisionShape::sphere(0.4)),
        ("Wide Cylinder", CollisionShape::cylinder(0.3, 0.8)),
    ];

    for (i, (_name, shape)) in shapes.into_iter().enumerate() {
        let col = i % 4;
        let row = i / 4;

        let x = (col as f64 - 1.5) * spacing;
        let z = (row as f64 - 0.5) * spacing;
        let y = 1.5;

        let pose = Pose::from_position(nalgebra::Point3::new(x, y, z));
        let state = RigidBodyState::at_rest(pose);
        let mass = MassProperties::sphere(1.0, 0.5); // Approximate mass
        let body_id = world.add_body(state, mass);

        if let Some(body) = world.body_mut(body_id) {
            body.collision_shape = Some(shape);
        }
    }

    // Add ground plane
    let ground_pose = Pose::from_position(nalgebra::Point3::new(0.0, 0.0, 0.0));
    let ground_id = world.add_static_body(ground_pose);
    if let Some(body) = world.body_mut(ground_id) {
        body.collision_shape = Some(CollisionShape::ground_plane(0.0));
    }

    // Insert the world as a resource
    commands.insert_resource(SimulationHandle::new(world));
}
