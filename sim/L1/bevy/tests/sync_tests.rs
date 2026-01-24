//! Integration tests for transform synchronization.
//!
//! Tests that sim-core body poses are correctly synced to Bevy transforms.
//!
//! Note: sim-core uses Z-up coordinates, while Bevy uses Y-up.
//! The conversion swaps Y and Z: `(x, y, z)_physics -> (x, z, y)_bevy`

#![allow(clippy::unwrap_used, clippy::expect_used)] // Standard in tests

use bevy::prelude::*;
use sim_bevy::prelude::*;
use sim_core::{CollisionShape, World};
use sim_types::{MassProperties, Pose, RigidBodyState, Twist};

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

#[test]
fn transform_matches_initial_pose() {
    let mut app = test_app();

    // Create world with body at specific position
    let mut world = World::default();
    let position = nalgebra::Point3::new(1.0, 2.0, 3.0);
    let pose = Pose::from_position(position);
    let state = RigidBodyState::at_rest(pose);
    let mass = MassProperties::sphere(1.0, 0.5);
    let body_id = world.add_body(state, mass);

    if let Some(body) = world.body_mut(body_id) {
        body.collision_shape = Some(CollisionShape::sphere(0.5));
    }

    app.insert_resource(SimulationHandle::new(world));
    app.update();

    // Check transform matches pose
    let body_map = app.world().resource::<BodyEntityMap>();
    let entity = body_map.get_entity(body_id).expect("entity should exist");

    let transform = app
        .world()
        .get::<Transform>(entity)
        .expect("entity should have Transform");

    // Physics (Z-up): (1, 2, 3) -> Bevy (Y-up): (1, 3, 2)
    let expected = Vec3::new(1.0, 3.0, 2.0);
    assert!(
        (transform.translation - expected).length() < 0.001,
        "Transform translation {:?} should match converted pose position {:?}",
        transform.translation,
        expected
    );
}

#[test]
fn transform_updates_after_pose_change() {
    let mut app = test_app();

    // Create world with body
    let mut world = World::default();
    let initial_pos = nalgebra::Point3::new(0.0, 5.0, 0.0);
    let pose = Pose::from_position(initial_pos);
    let state = RigidBodyState::at_rest(pose);
    let mass = MassProperties::sphere(1.0, 0.5);
    let body_id = world.add_body(state, mass);

    if let Some(body) = world.body_mut(body_id) {
        body.collision_shape = Some(CollisionShape::sphere(0.5));
    }

    app.insert_resource(SimulationHandle::new(world));
    app.update();

    // Modify the body's pose in sim-core
    let new_pos = nalgebra::Point3::new(10.0, 20.0, 30.0);
    {
        let mut sim_handle = app.world_mut().resource_mut::<SimulationHandle>();
        if let Some(world) = sim_handle.world_mut() {
            if let Some(body) = world.body_mut(body_id) {
                body.state.pose.position = new_pos;
            }
        }
    }

    // Run update to sync transforms
    app.update();

    // Check transform was updated
    let body_map = app.world().resource::<BodyEntityMap>();
    let entity = body_map.get_entity(body_id).expect("entity should exist");

    let transform = app
        .world()
        .get::<Transform>(entity)
        .expect("entity should have Transform");

    // Physics (Z-up): (10, 20, 30) -> Bevy (Y-up): (10, 30, 20)
    let expected = Vec3::new(10.0, 30.0, 20.0);
    assert!(
        (transform.translation - expected).length() < 0.001,
        "Transform should update to converted position {:?}, got {:?}",
        expected,
        transform.translation
    );
}

#[test]
fn rotation_syncs_correctly() {
    let mut app = test_app();

    // Create world with rotated body
    let mut world = World::default();
    let position = nalgebra::Point3::new(0.0, 0.0, 0.0);
    // 90 degree rotation around Z axis (physics up)
    // This should become rotation around Y axis in Bevy (graphics up)
    let rotation = nalgebra::UnitQuaternion::from_axis_angle(
        &nalgebra::Vector3::z_axis(),
        std::f64::consts::FRAC_PI_2,
    );
    let pose = Pose::from_position_rotation(position, rotation);
    let state = RigidBodyState::at_rest(pose);
    let mass = MassProperties::sphere(1.0, 0.5);
    let body_id = world.add_body(state, mass);

    if let Some(body) = world.body_mut(body_id) {
        body.collision_shape = Some(CollisionShape::sphere(0.5));
    }

    app.insert_resource(SimulationHandle::new(world));
    app.update();

    // Check rotation
    let body_map = app.world().resource::<BodyEntityMap>();
    let entity = body_map.get_entity(body_id).expect("entity should exist");

    let transform = app
        .world()
        .get::<Transform>(entity)
        .expect("entity should have Transform");

    // Physics rotation around Z (up) -> Bevy rotation around Y (up)
    let expected = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);

    // Compare quaternions (allowing for sign flip since -q and q represent same rotation)
    let dot = transform.rotation.dot(expected).abs();
    assert!(
        dot > 0.999,
        "Rotation should match: got {:?}, expected {:?}",
        transform.rotation,
        expected
    );
}

#[test]
fn multiple_bodies_sync_independently() {
    let mut app = test_app();

    // Create world with two bodies at different positions
    let mut world = World::default();

    let pos1 = nalgebra::Point3::new(1.0, 0.0, 0.0);
    let body1 = world.add_body(
        RigidBodyState::at_rest(Pose::from_position(pos1)),
        MassProperties::sphere(1.0, 0.5),
    );
    if let Some(body) = world.body_mut(body1) {
        body.collision_shape = Some(CollisionShape::sphere(0.5));
    }

    let pos2 = nalgebra::Point3::new(-1.0, 0.0, 0.0);
    let body2 = world.add_body(
        RigidBodyState::at_rest(Pose::from_position(pos2)),
        MassProperties::sphere(1.0, 0.5),
    );
    if let Some(body) = world.body_mut(body2) {
        body.collision_shape = Some(CollisionShape::sphere(0.5));
    }

    app.insert_resource(SimulationHandle::new(world));
    app.update();

    // Move only body1
    let new_pos1 = nalgebra::Point3::new(5.0, 0.0, 0.0);
    {
        let mut sim_handle = app.world_mut().resource_mut::<SimulationHandle>();
        if let Some(world) = sim_handle.world_mut() {
            if let Some(body) = world.body_mut(body1) {
                body.state.pose.position = new_pos1;
            }
        }
    }

    app.update();

    // Check body1 moved
    let body_map = app.world().resource::<BodyEntityMap>();

    let entity1 = body_map.get_entity(body1).expect("entity1 should exist");
    let transform1 = app.world().get::<Transform>(entity1).unwrap();
    assert!(
        (transform1.translation.x - 5.0).abs() < 0.001,
        "Body1 should have moved to x=5.0"
    );

    // Check body2 stayed
    let entity2 = body_map.get_entity(body2).expect("entity2 should exist");
    let transform2 = app.world().get::<Transform>(entity2).unwrap();
    assert!(
        (transform2.translation.x - (-1.0)).abs() < 0.001,
        "Body2 should remain at x=-1.0"
    );
}

#[test]
fn velocity_does_not_affect_transform_directly() {
    let mut app = test_app();

    // Create world with moving body
    let mut world = World::default();
    let position = nalgebra::Point3::new(0.0, 0.0, 0.0);
    let pose = Pose::from_position(position);
    let twist = Twist {
        linear: nalgebra::Vector3::new(10.0, 0.0, 0.0),
        angular: nalgebra::Vector3::zeros(),
    };
    let state = RigidBodyState { pose, twist };
    let mass = MassProperties::sphere(1.0, 0.5);
    let body_id = world.add_body(state, mass);

    if let Some(body) = world.body_mut(body_id) {
        body.collision_shape = Some(CollisionShape::sphere(0.5));
    }

    app.insert_resource(SimulationHandle::new(world));
    app.update();

    // Transform should reflect position, not velocity
    // (sim-bevy is read-only; it doesn't step physics)
    let body_map = app.world().resource::<BodyEntityMap>();
    let entity = body_map.get_entity(body_id).expect("entity should exist");
    let transform = app.world().get::<Transform>(entity).unwrap();

    assert!(
        (transform.translation.x - 0.0).abs() < 0.001,
        "Transform should be at origin (velocity doesn't move it without stepping)"
    );
}
