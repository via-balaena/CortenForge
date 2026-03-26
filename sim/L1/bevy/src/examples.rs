//! Example-specific utilities.
//!
//! Helpers shared across CortenForge example programs. These are convenience
//! functions for demos and tutorials — not for production simulation code.

use bevy::prelude::*;

use crate::camera::OrbitCamera;

/// Timer for periodic diagnostic printing in examples.
///
/// Tracks the last physics time at which diagnostics were printed.
/// Compare against `data.time` to print at a fixed interval:
///
/// ```ignore
/// if data.time - timer.last > 1.0 {
///     timer.last = data.time;
///     println!("t={:.1}s  ...", data.time);
/// }
/// ```
#[derive(Resource, Default)]
pub struct DiagTimer {
    /// Last physics time at which diagnostics were printed.
    pub last: f64,
}

/// Spawn a standard example camera and two-point lighting rig.
///
/// Creates an [`OrbitCamera`] aimed at `target` with the given `distance`,
/// `azimuth`, and `elevation`, plus ambient light, a key directional light
/// (with shadows), and a fill directional light (no shadows).
///
/// `target` is in **Bevy coordinates (Y-up)**. Use [`physics_pos()`](crate::convert::physics_pos)
/// at the call site when converting from MJCF (Z-up) coordinates.
///
/// Does NOT spawn a ground plane — use [`ExampleScene`](crate::scene::ExampleScene)
/// if you need one.
pub fn spawn_example_camera(
    commands: &mut Commands,
    target: Vec3,
    distance: f32,
    azimuth: f32,
    elevation: f32,
) {
    let mut orbit = OrbitCamera::new()
        .with_target(target)
        .with_angles(azimuth, elevation);
    orbit.max_distance = 20.0;
    orbit.distance = distance;
    let mut cam_transform = Transform::default();
    orbit.apply_to_transform(&mut cam_transform);
    commands.spawn((Camera3d::default(), orbit, cam_transform));

    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 800.0,
        ..default()
    });
    commands.spawn((
        DirectionalLight {
            illuminance: 15_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(30.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 5_000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-20.0, 30.0, -30.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
