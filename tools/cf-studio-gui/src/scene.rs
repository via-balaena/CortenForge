//! The 3D view behind the wizard: camera, light, and the standing placeholder
//! body.
//!
//! ⚠ The placeholder is **step 1's empty state**, not a render-safety guard. An
//! earlier note in this workspace claimed a drawn mesh must exist wherever quit
//! is reachable, to avoid a macOS quit deadlock. That attribution was measured
//! and is wrong: the deadlock is Bevy's pipelined-rendering teardown and fires
//! with a mesh on screen. `main.rs` carries the real fix. Do not re-derive a
//! scene constraint from it.

use bevy::prelude::*;
use cf_bevy_common::camera::OrbitCamera;

/// The placeholder body, so a later PR can find and replace it with the scan.
#[derive(Component)]
pub(crate) struct Placeholder;

/// Muted clay, readable against the light background without competing with
/// the panel for attention.
const PLACEHOLDER_COLOR: Color = Color::srgb(0.72, 0.70, 0.66);

/// Spawn the camera, a key light, and the placeholder body.
pub(crate) fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        OrbitCamera {
            distance: 4.0,
            elevation: 0.35,
            ..OrbitCamera::default()
        },
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 8_000.0,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Placeholder,
        Mesh3d(meshes.add(Sphere::new(1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: PLACEHOLDER_COLOR,
            perceptual_roughness: 0.85,
            ..default()
        })),
    ));
}
