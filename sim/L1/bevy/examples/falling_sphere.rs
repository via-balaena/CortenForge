//! Minimal example: a sphere falling under gravity.
//!
//! This demonstrates the basic sim-bevy workflow:
//! 1. Create a sim-core World with bodies
//! 2. Add the SimViewerPlugin
//! 3. Step the simulation in an Update system
//!
//! Run with: `cargo run -p sim-bevy --example falling_sphere --features x11`
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
        .add_systems(Startup, setup_physics)
        .add_systems(Update, step_physics)
        .run();
}

/// Set up the physics world with a sphere and ground plane.
fn setup_physics(mut commands: Commands) {
    let mut world = World::default();

    // Add a falling sphere (Z=5 means 5 units up in Z-up physics coordinates)
    let sphere_pose = Pose::from_position(nalgebra::Point3::new(0.0, 0.0, 5.0));
    let sphere_state = RigidBodyState::at_rest(sphere_pose);
    let sphere_mass = MassProperties::sphere(1.0, 0.5);
    let sphere_id = world.add_body(sphere_state, sphere_mass);

    // Set collision shape
    if let Some(body) = world.body_mut(sphere_id) {
        body.collision_shape = Some(CollisionShape::sphere(0.5));
    }

    // Add ground plane (static body)
    let ground_pose = Pose::from_position(nalgebra::Point3::new(0.0, 0.0, 0.0));
    let ground_id = world.add_static_body(ground_pose);

    if let Some(body) = world.body_mut(ground_id) {
        body.collision_shape = Some(CollisionShape::ground_plane(0.0));
    }

    // Insert the world as a resource
    commands.insert_resource(SimulationHandle::new(world));
    commands.insert_resource(PhysicsStepper::default());
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
        // Step the simulation (uses world's configured timestep)
        let _ = physics_stepper.stepper.step(world);
    }
}
