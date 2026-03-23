//! Example scene setup helpers.
//!
//! Provides a builder for the standard orbit-camera + two-point-lighting + ground
//! plane that every CortenForge example uses. Eliminates ~20 lines of boilerplate
//! per example while remaining configurable for outliers.

use bevy::prelude::*;

use crate::camera::OrbitCamera;

/// Builder for a standard example scene (camera, lights, ground plane).
///
/// # Example
///
/// ```ignore
/// ExampleScene::new(80.0, 60.0).spawn(&mut commands, &mut meshes, &mut materials);
/// ```
pub struct ExampleScene {
    /// Orbit camera target.
    pub target: Vec3,
    /// Camera distance from target.
    pub distance: f32,
    /// Camera (azimuth, elevation) in radians.
    pub angles: (f32, f32),
    /// Maximum zoom-out distance.
    pub max_distance: f32,
    /// Ground plane half-extent.
    pub ground_size: f32,
    /// Ground plane Y position (Bevy space).
    pub ground_y: f32,
}

impl ExampleScene {
    /// Create a scene with the given camera distance and ground size.
    ///
    /// Uses sensible defaults for everything else. Override with builder methods.
    #[must_use]
    pub fn new(distance: f32, ground_size: f32) -> Self {
        Self {
            target: Vec3::ZERO,
            distance,
            angles: (0.5, 0.5),
            max_distance: 500.0,
            ground_size,
            ground_y: -15.0,
        }
    }

    /// Set the camera orbit target.
    #[must_use]
    pub fn with_target(mut self, target: Vec3) -> Self {
        self.target = target;
        self
    }

    /// Set the camera angles (azimuth, elevation) in radians.
    #[must_use]
    pub fn with_angles(mut self, azimuth: f32, elevation: f32) -> Self {
        self.angles = (azimuth, elevation);
        self
    }

    /// Set the maximum zoom-out distance.
    #[must_use]
    pub fn with_max_distance(mut self, max_distance: f32) -> Self {
        self.max_distance = max_distance;
        self
    }

    /// Set the ground plane Y position.
    #[must_use]
    pub fn with_ground_y(mut self, y: f32) -> Self {
        self.ground_y = y;
        self
    }

    /// Spawn the camera, lights, and ground plane.
    pub fn spawn(
        self,
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
    ) {
        // ── Camera ──────────────────────────────────────────────────
        let mut orbit = OrbitCamera::new()
            .with_target(self.target)
            .with_angles(self.angles.0, self.angles.1);
        orbit.max_distance = self.max_distance;
        orbit.min_distance = 0.5;
        orbit.orbit_speed = 0.008;
        orbit.pan_speed = 0.015;
        orbit.zoom_speed = 0.15;
        orbit.distance = self.distance;
        let mut cam_transform = Transform::default();
        orbit.apply_to_transform(&mut cam_transform);
        commands.spawn((Camera3d::default(), orbit, cam_transform));

        // ── Ambient light ────────────────────────────────────────────
        // Bevy 0.18: GlobalAmbientLight is the world-wide resource.
        // (AmbientLight is now a per-camera component override.)
        commands.insert_resource(GlobalAmbientLight {
            color: Color::WHITE,
            brightness: 1_200.0,
            ..default()
        });

        // ── Key light ───────────────────────────────────────────────
        commands.spawn((
            DirectionalLight {
                illuminance: 12_000.0,
                shadows_enabled: true,
                ..default()
            },
            Transform::from_xyz(30.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
        ));

        // ── Fill light ──────────────────────────────────────────────
        commands.spawn((
            DirectionalLight {
                illuminance: 4_000.0,
                shadows_enabled: false,
                ..default()
            },
            Transform::from_xyz(-20.0, 30.0, -30.0).looking_at(Vec3::ZERO, Vec3::Y),
        ));

        // ── Ground plane ────────────────────────────────────────────
        commands.spawn((
            Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(self.ground_size)))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(0.5, 0.5, 0.5, 0.3),
                alpha_mode: AlphaMode::Blend,
                ..default()
            })),
            Transform::from_xyz(0.0, self.ground_y, 0.0),
        ));
    }
}
