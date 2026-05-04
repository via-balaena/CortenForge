//! `cf-view` — workspace-wide visual-review viewer.

use std::path::PathBuf;

use anyhow::{Result, anyhow};
use bevy::prelude::*;
use cf_viewer::{
    ViewerInput, load_input,
    mesh::{POINT_RADIUS_FRACTION, build_face_mesh, vertex_position},
};
use mesh_types::Bounded;

fn main() -> Result<()> {
    let path: PathBuf = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .ok_or_else(|| anyhow!("usage: cf-view <path-to-ply>"))?;

    let input = load_input(&path)?;
    println!(
        "loaded {} vertices, {} scalars: {:?}",
        input.mesh.vertex_count(),
        input.scalar_names.len(),
        input.scalar_names,
    );

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cf-view".into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
        .insert_resource(input)
        .add_systems(Startup, setup_scene)
        .add_systems(Update, exit_on_esc)
        .run();

    Ok(())
}

/// Spawn geometry entities, a static placeholder camera, and lighting from
/// the loaded [`ViewerInput`].
///
/// **Faces case** (`face_count() > 0`): one `Mesh3d` entity carrying the
/// converted Bevy mesh.
///
/// **Faces-empty case** (point cloud): one tiny sphere entity per vertex,
/// sharing a single mesh handle and a single material handle. Sphere radius
/// is `bbox.diagonal() * POINT_RADIUS_FRACTION`. See `docs/VIEWER_DESIGN.md`
/// iter-2 still-open #6 for the rendering-approach decision.
///
/// The camera + lights size themselves to the input's AABB. Orbit camera
/// (commit 5) replaces this static placeholder.
#[allow(clippy::cast_possible_truncation)] // f64 → f32 is intentional for Bevy
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    input: Res<ViewerInput>,
) {
    // Workspace input is Z-up; the Y/Z swap on the AABB center keeps it
    // aligned with the converted geometry's Bevy frame.
    let aabb = input.mesh.aabb();
    let center_physics = aabb.center();
    let center_bevy = Vec3::new(
        center_physics.x as f32,
        center_physics.z as f32,
        center_physics.y as f32,
    );
    // Empty-AABB sentinel (min > max) yields negative diagonal — clamp so
    // camera + sphere-radius calculations stay sane on degenerate input.
    let diagonal = (aabb.diagonal() as f32).max(1.0);

    // Camera: ~1.5× diagonal off the (+x, +y, +z) corner so the bbox sits
    // inside the frustum at startup. Orbit camera (commit 5) replaces this.
    let cam_offset = Vec3::splat(diagonal * 1.5);
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(center_bevy + cam_offset).looking_at(center_bevy, Vec3::Y),
    ));

    // Strong directional key light + bright global ambient so geometry stays
    // readable even before the colormap pipeline lands. Bevy 0.18:
    // `AmbientLight` is now per-camera; `GlobalAmbientLight` is the
    // world-wide resource (sim-bevy `scene.rs:101` precedent).
    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_translation(center_bevy + Vec3::new(diagonal, diagonal * 2.0, diagonal))
            .looking_at(center_bevy, Vec3::Y),
    ));
    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 1_200.0,
        ..default()
    });

    let material_handle = materials.add(StandardMaterial {
        base_color: Color::srgb(0.70, 0.72, 0.78),
        metallic: 0.10,
        perceptual_roughness: 0.6,
        double_sided: true,
        cull_mode: None,
        ..default()
    });

    if input.mesh.face_count() > 0 {
        let bevy_mesh = build_face_mesh(&input.mesh);
        commands.spawn((
            Mesh3d(meshes.add(bevy_mesh)),
            MeshMaterial3d(material_handle),
        ));
    } else {
        let radius = (diagonal * POINT_RADIUS_FRACTION).max(1e-3);
        let sphere_handle = meshes.add(Sphere::new(radius));
        for v in &input.mesh.geometry.vertices {
            commands.spawn((
                Mesh3d(sphere_handle.clone()),
                MeshMaterial3d(material_handle.clone()),
                Transform::from_translation(Vec3::from_array(vertex_position(v))),
            ));
        }
    }
}

#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
fn exit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}
