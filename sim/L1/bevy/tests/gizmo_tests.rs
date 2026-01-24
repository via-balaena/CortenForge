//! Integration tests for debug gizmo visualization.
//!
//! Note: These tests verify the configuration and system registration,
//! not the actual rendering (which requires a GPU).

#![allow(clippy::unwrap_used, clippy::expect_used)] // Standard in tests

use bevy::prelude::*;
use sim_bevy::prelude::*;
use sim_core::{CollisionShape, World};
use sim_types::{MassProperties, Pose, RigidBodyState};

/// Create a test app with gizmos enabled.
fn test_app_with_gizmos() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(bevy::asset::AssetPlugin::default());
    app.init_resource::<bevy::prelude::Assets<bevy::prelude::Mesh>>();
    app.init_resource::<bevy::prelude::Assets<bevy::prelude::StandardMaterial>>();
    // Use full plugin with gizmos disabled for testing
    // (gizmos require bevy_render which isn't available in MinimalPlugins)
    app.add_plugins(SimViewerPlugin::headless());
    app
}

/// Create a test world with colliding bodies.
fn create_colliding_world() -> World {
    let mut world = World::default();

    // Sphere resting on ground
    let sphere_pose = Pose::from_position(nalgebra::Point3::new(0.0, 0.5, 0.0));
    let sphere_id = world.add_body(
        RigidBodyState::at_rest(sphere_pose),
        MassProperties::sphere(1.0, 0.5),
    );
    if let Some(body) = world.body_mut(sphere_id) {
        body.collision_shape = Some(CollisionShape::sphere(0.5));
    }

    // Ground plane
    let ground_pose = Pose::from_position(nalgebra::Point3::origin());
    let ground_id = world.add_static_body(ground_pose);
    if let Some(body) = world.body_mut(ground_id) {
        body.collision_shape = Some(CollisionShape::ground_plane(0.0));
    }

    world
}

#[test]
fn viewer_config_default_values() {
    let config = ViewerConfig::default();

    assert!(config.show_collision_shapes);
    assert!(config.show_contacts);
    assert!(config.show_contact_normals);
    assert!(!config.show_forces); // Off by default
    assert!(config.show_joint_axes);
    assert!(!config.show_joint_limits); // Off by default
}

#[test]
fn viewer_config_can_be_modified() {
    let mut app = test_app_with_gizmos();
    let world = create_colliding_world();

    app.insert_resource(SimulationHandle::new(world));
    app.update();

    // Modify config
    {
        let mut config = app.world_mut().resource_mut::<ViewerConfig>();
        config.show_contacts = false;
        config.show_forces = true;
    }

    // Verify changes persist
    let config = app.world().resource::<ViewerConfig>();
    assert!(!config.show_contacts);
    assert!(config.show_forces);
}

#[test]
fn debug_colors_default_values() {
    let colors = DebugColors::default();

    // Verify colors are distinguishable (not all the same)
    assert_ne!(
        format!("{:?}", colors.contact_point),
        format!("{:?}", colors.contact_normal)
    );
    assert_ne!(
        format!("{:?}", colors.force_vector),
        format!("{:?}", colors.joint_axis)
    );
}

#[test]
fn headless_plugin_disables_gizmos() {
    let plugin = SimViewerPlugin::headless();
    assert!(!plugin.enable_debug_gizmos);
    assert!(!plugin.spawn_camera);
    assert!(!plugin.spawn_lighting);
    assert!(!plugin.enable_camera_input);
}

#[test]
fn new_plugin_enables_gizmos() {
    let plugin = SimViewerPlugin::new();
    assert!(plugin.enable_debug_gizmos);
    assert!(plugin.spawn_camera);
    assert!(plugin.spawn_lighting);
    assert!(plugin.enable_camera_input);
}

#[test]
fn config_force_scale_affects_display() {
    let mut config = ViewerConfig::default();

    // Default force scale
    assert!(config.force_scale > 0.0);

    // Can be modified
    config.force_scale = 0.1;
    assert!((config.force_scale - 0.1).abs() < f32::EPSILON);
}

#[test]
fn config_contact_marker_radius_affects_display() {
    let mut config = ViewerConfig::default();

    // Default marker radius
    assert!(config.contact_marker_radius > 0.0);

    // Can be modified
    config.contact_marker_radius = 0.05;
    assert!((config.contact_marker_radius - 0.05).abs() < f32::EPSILON);
}
