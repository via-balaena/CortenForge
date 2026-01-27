//! Example: Contact debug visualization.
//!
//! Demonstrates the debug visualization features for contacts, forces, and joints.
//! Multiple spheres fall and collide, showing contact points and normals.
//!
//! Run with: `cargo run -p sim-bevy --example contact_debug --features x11`
//! (or `--features wayland` on Wayland systems)

#![allow(clippy::needless_pass_by_value)] // Bevy system parameters

use bevy::prelude::*;
use sim_bevy::prelude::*;
use sim_core::world::{CollisionShape, World};
use sim_core::Stepper;
use sim_types::{MassProperties, Pose, RigidBodyState};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(SimViewerPlugin::new())
        .add_systems(Startup, setup_scene)
        .init_resource::<PhysicsStepper>()
        .add_systems(Update, step_physics)
        .add_systems(Update, toggle_debug_viz)
        .run();
}

/// Set up a scene with multiple falling bodies.
fn setup_scene(mut commands: Commands) {
    let mut world = World::default();

    // Add multiple spheres at different positions
    let sphere_radius = 0.4;
    let positions = [
        (0.0, 3.0, 0.0),
        (0.5, 4.0, 0.3),
        (-0.3, 5.0, -0.2),
        (0.2, 6.0, 0.5),
    ];

    for (x, y, z) in positions {
        let pose = Pose::from_position(nalgebra::Point3::new(x, y, z));
        let state = RigidBodyState::at_rest(pose);
        let mass = MassProperties::sphere(1.0, sphere_radius);
        let body_id = world.add_body(state, mass);

        if let Some(body) = world.body_mut(body_id) {
            body.collision_shape = Some(CollisionShape::sphere(sphere_radius));
        }
    }

    // Add a box to demonstrate box-sphere contacts
    let box_pose = Pose::from_position(nalgebra::Point3::new(-1.5, 2.0, 0.0));
    let box_state = RigidBodyState::at_rest(box_pose);
    let box_half_extents = nalgebra::Vector3::new(0.4, 0.4, 0.4);
    let box_mass = MassProperties::box_shape(2.0, box_half_extents);
    let box_id = world.add_body(box_state, box_mass);

    if let Some(body) = world.body_mut(box_id) {
        body.collision_shape = Some(CollisionShape::box_shape(box_half_extents));
    }

    // Add ground plane
    let ground_pose = Pose::from_position(nalgebra::Point3::new(0.0, 0.0, 0.0));
    let ground_id = world.add_static_body(ground_pose);
    if let Some(body) = world.body_mut(ground_id) {
        body.collision_shape = Some(CollisionShape::ground_plane(0.0));
    }

    // Insert the world as a resource
    commands.insert_resource(SimulationHandle::new(world));

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
