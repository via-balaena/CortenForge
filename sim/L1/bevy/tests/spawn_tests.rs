//! Integration tests for entity spawning.
//!
//! Tests that sim-core bodies are correctly spawned as Bevy entities
//! with proper components.

#![allow(clippy::unwrap_used, clippy::expect_used)] // Standard in tests

use bevy::prelude::*;
use sim_bevy::prelude::*;
use sim_core::{CollisionShape, World};
use sim_types::{MassProperties, Pose, RigidBodyState};

/// Create a minimal Bevy app for testing (no rendering).
fn test_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    // Asset plugin is required for mesh/material handling
    app.add_plugins(bevy::asset::AssetPlugin::default());
    app.init_resource::<bevy::prelude::Assets<bevy::prelude::Mesh>>();
    app.init_resource::<bevy::prelude::Assets<bevy::prelude::StandardMaterial>>();
    app.add_plugins(SimViewerPlugin::headless());
    app
}

/// Create a test world with a single sphere body.
fn create_world_with_sphere() -> World {
    let mut world = World::default();

    let pose = Pose::from_position(nalgebra::Point3::new(1.0, 2.0, 3.0));
    let state = RigidBodyState::at_rest(pose);
    let mass = MassProperties::sphere(1.0, 0.5);
    let body_id = world.add_body(state, mass);

    if let Some(body) = world.body_mut(body_id) {
        body.collision_shape = Some(CollisionShape::sphere(0.5));
    }

    world
}

/// Create a test world with multiple bodies.
fn create_world_with_multiple_bodies() -> World {
    let mut world = World::default();

    // Sphere
    let sphere_pose = Pose::from_position(nalgebra::Point3::new(0.0, 5.0, 0.0));
    let sphere_id = world.add_body(
        RigidBodyState::at_rest(sphere_pose),
        MassProperties::sphere(1.0, 0.5),
    );
    if let Some(body) = world.body_mut(sphere_id) {
        body.collision_shape = Some(CollisionShape::sphere(0.5));
    }

    // Box
    let box_pose = Pose::from_position(nalgebra::Point3::new(2.0, 5.0, 0.0));
    let box_id = world.add_body(
        RigidBodyState::at_rest(box_pose),
        MassProperties::box_shape(1.0, nalgebra::Vector3::new(0.5, 0.5, 0.5)),
    );
    if let Some(body) = world.body_mut(box_id) {
        body.collision_shape = Some(CollisionShape::box_shape(nalgebra::Vector3::new(
            0.5, 0.5, 0.5,
        )));
    }

    // Static ground
    let ground_pose = Pose::from_position(nalgebra::Point3::new(0.0, 0.0, 0.0));
    let ground_id = world.add_static_body(ground_pose);
    if let Some(body) = world.body_mut(ground_id) {
        body.collision_shape = Some(CollisionShape::ground_plane(0.0));
    }

    world
}

#[test]
fn spawning_world_creates_entities() {
    let mut app = test_app();
    let world = create_world_with_sphere();
    let body_count = world.body_count();

    app.insert_resource(SimulationHandle::new(world));
    app.update();

    // Check that a PhysicsBody entity was created
    let physics_bodies: Vec<_> = app
        .world_mut()
        .query::<&PhysicsBody>()
        .iter(app.world())
        .collect();

    assert_eq!(
        physics_bodies.len(),
        body_count,
        "Expected {} PhysicsBody entities, found {}",
        body_count,
        physics_bodies.len()
    );
}

#[test]
fn spawned_entities_have_transforms() {
    let mut app = test_app();
    let world = create_world_with_sphere();

    app.insert_resource(SimulationHandle::new(world));
    app.update();

    // Check that all PhysicsBody entities have Transform components
    let bodies_with_transforms: Vec<_> = app
        .world_mut()
        .query::<(&PhysicsBody, &Transform)>()
        .iter(app.world())
        .collect();

    assert!(
        !bodies_with_transforms.is_empty(),
        "PhysicsBody entities should have Transform components"
    );
}

#[test]
fn spawned_entities_have_collision_shape_visuals() {
    let mut app = test_app();
    let world = create_world_with_sphere();

    app.insert_resource(SimulationHandle::new(world));
    app.update();

    // Check that CollisionShapeVisual components exist
    let shape_visuals: Vec<_> = app
        .world_mut()
        .query::<&CollisionShapeVisual>()
        .iter(app.world())
        .collect();

    assert!(
        !shape_visuals.is_empty(),
        "Should have CollisionShapeVisual components for bodies with shapes"
    );
}

#[test]
fn multiple_bodies_spawn_correctly() {
    let mut app = test_app();
    let world = create_world_with_multiple_bodies();
    let expected_count = world.body_count();

    app.insert_resource(SimulationHandle::new(world));
    app.update();

    let physics_bodies: Vec<_> = app
        .world_mut()
        .query::<&PhysicsBody>()
        .iter(app.world())
        .collect();

    assert_eq!(
        physics_bodies.len(),
        expected_count,
        "All bodies should be spawned as entities"
    );
}

#[test]
fn body_entity_map_is_populated() {
    let mut app = test_app();
    let world = create_world_with_sphere();
    let body_ids: Vec<_> = world.body_ids().collect();

    app.insert_resource(SimulationHandle::new(world));
    app.update();

    let body_map = app.world().resource::<BodyEntityMap>();

    for body_id in &body_ids {
        assert!(
            body_map.contains_body(*body_id),
            "BodyEntityMap should contain all body IDs"
        );
    }

    assert_eq!(
        body_map.len(),
        body_ids.len(),
        "BodyEntityMap should have correct count"
    );
}

#[test]
fn removing_body_despawns_entity() {
    let mut app = test_app();

    // Create world with a sphere
    let world = create_world_with_sphere();
    let body_id = world.body_ids().next().expect("should have a body");

    app.insert_resource(SimulationHandle::new(world));
    app.update();

    // Verify entity exists
    {
        let body_map = app.world().resource::<BodyEntityMap>();
        assert!(
            body_map.contains_body(body_id),
            "Body should exist initially"
        );
    }

    // Remove the body from sim-core world
    {
        let mut sim_handle = app.world_mut().resource_mut::<SimulationHandle>();
        if let Some(world) = sim_handle.world_mut() {
            world.remove_body(body_id);
        }
    }

    // Run update to sync
    app.update();

    // Verify entity was despawned
    {
        let body_map = app.world().resource::<BodyEntityMap>();
        assert!(
            !body_map.contains_body(body_id),
            "Body should be removed from map after removal"
        );
    }
}
