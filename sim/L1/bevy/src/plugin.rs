//! Plugin composition for physics visualization.
//!
//! `SimViewerPlugin` provides visualization for the Model/Data API.
//! For Model/Data systems, see [`ModelDataPlugin`](crate::model_data::ModelDataPlugin).

use bevy::gizmos::config::{DefaultGizmoConfigGroup, GizmoConfigStore};
use bevy::prelude::*;

use crate::camera::{OrbitCameraPlugin, spawn_orbit_camera};
use crate::gizmos::{
    DebugGizmosSet, draw_contact_normals, draw_contact_points, draw_muscles, draw_sensors,
    draw_tendons,
};
use crate::resources::{
    BodyEntityMap, CachedContacts, MuscleVisualization, SensorVisualization, TendonVisualization,
    ViewerConfig,
};
use crate::systems::{SimViewerSet, update_cached_contacts, update_shape_visibility};

/// Physics visualization plugin for Bevy.
///
/// This plugin provides real-time visualization of sim-core physics
/// simulations, including collision shapes, contacts, forces, and joints.
///
/// # Example
///
/// ```no_run,ignore
/// use bevy::prelude::*;
/// use sim_bevy::prelude::*;
///
/// fn main() {
///     App::new()
///         .add_plugins(DefaultPlugins)
///         .add_plugins(SimViewerPlugin::default())
///         .run();
/// }
/// ```
///
/// # Architecture
///
/// The plugin is read-only with respect to physics state. Users are
/// responsible for stepping the simulation; sim-bevy only visualizes.
///
/// Systems run in this order:
/// 1. `Update`: Transform synchronization (via Model/Data systems)
/// 2. `PostUpdate`: Debug visualization (gizmos)
#[derive(Default)]
#[allow(clippy::struct_excessive_bools)] // Plugin config with multiple toggles
pub struct SimViewerPlugin {
    /// Initial viewer configuration.
    pub config: ViewerConfig,
    /// Whether to spawn a default orbit camera.
    pub spawn_camera: bool,
    /// Whether to spawn default lighting.
    pub spawn_lighting: bool,
    /// Whether to add camera input systems (requires input resources).
    pub enable_camera_input: bool,
    /// Whether to enable debug gizmo rendering (requires `bevy_gizmos`).
    pub enable_debug_gizmos: bool,
}

impl SimViewerPlugin {
    /// Create a new plugin with default settings.
    #[must_use]
    pub fn new() -> Self {
        Self {
            spawn_camera: true,
            spawn_lighting: true,
            enable_camera_input: true,
            enable_debug_gizmos: true,
            ..Default::default()
        }
    }

    /// Create a plugin for headless/testing mode (no camera, no lighting, no input, no gizmos).
    #[must_use]
    pub fn headless() -> Self {
        Self {
            spawn_camera: false,
            spawn_lighting: false,
            enable_camera_input: false,
            enable_debug_gizmos: false,
            ..Default::default()
        }
    }

    /// Set the viewer configuration.
    #[must_use]
    pub fn with_config(mut self, config: ViewerConfig) -> Self {
        self.config = config;
        self
    }

    /// Disable automatic camera spawning.
    #[must_use]
    pub fn without_camera(mut self) -> Self {
        self.spawn_camera = false;
        self
    }

    /// Disable automatic lighting spawning.
    #[must_use]
    pub fn without_lighting(mut self) -> Self {
        self.spawn_lighting = false;
        self
    }
}

impl Plugin for SimViewerPlugin {
    fn build(&self, app: &mut App) {
        // Resources
        app.insert_resource(self.config.clone())
            .init_resource::<BodyEntityMap>()
            .init_resource::<CachedContacts>()
            .init_resource::<MuscleVisualization>()
            .init_resource::<TendonVisualization>()
            .init_resource::<SensorVisualization>();

        // Configure system sets
        app.configure_sets(Update, SimViewerSet::TransformSync);

        // Core systems - update shape visibility based on config
        app.add_systems(
            PostUpdate,
            update_shape_visibility.in_set(SimViewerSet::TransformSync),
        );

        // Debug visualization gizmos (only if enabled - requires bevy_gizmos)
        if self.enable_debug_gizmos {
            // Configure gizmos to render on top of meshes (disable depth testing)
            app.add_systems(Startup, configure_gizmos);

            // Configure system ordering: TransformSync -> ContactCache -> DebugGizmosSet
            app.configure_sets(
                PostUpdate,
                (
                    SimViewerSet::ContactCache.after(SimViewerSet::TransformSync),
                    DebugGizmosSet.after(SimViewerSet::ContactCache),
                ),
            );

            // Update contact cache before gizmo drawing
            app.add_systems(
                PostUpdate,
                update_cached_contacts.in_set(SimViewerSet::ContactCache),
            );

            app.add_systems(
                PostUpdate,
                (
                    draw_contact_points,
                    draw_contact_normals,
                    // Musculoskeletal visualization
                    draw_muscles,
                    draw_tendons,
                    draw_sensors,
                )
                    .in_set(DebugGizmosSet),
            );
        }

        // Camera (only add input systems if enabled - requires input resources)
        if self.enable_camera_input {
            app.add_plugins(OrbitCameraPlugin);
        }

        if self.spawn_camera {
            app.add_systems(Startup, spawn_orbit_camera);
        }

        // Lighting
        if self.spawn_lighting {
            app.add_systems(Startup, spawn_default_lighting);
        }
    }
}

/// Configure gizmos to render on top of meshes.
fn configure_gizmos(mut config_store: ResMut<GizmoConfigStore>) {
    let (config, _) = config_store.config_mut::<DefaultGizmoConfigGroup>();
    // Negative depth bias to render gizmos in front of meshes
    config.depth_bias = -0.01;
}

/// Spawn default lighting for the scene.
fn spawn_default_lighting(mut commands: Commands) {
    // Directional light (sun)
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.5, 0.5, 0.0)),
    ));

    // Ambient light (Bevy 0.18: spawned as entity, not a resource)
    commands.spawn(AmbientLight {
        color: Color::WHITE,
        brightness: 200.0,
        ..default()
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plugin_builder_pattern() {
        let config = ViewerConfig {
            show_contacts: false,
            ..Default::default()
        };

        let plugin = SimViewerPlugin::new()
            .with_config(config.clone())
            .without_camera()
            .without_lighting();

        assert!(!plugin.spawn_camera);
        assert!(!plugin.spawn_lighting);
        assert!(!plugin.config.show_contacts);
    }
}
