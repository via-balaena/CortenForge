//! `cf-view` — workspace-wide visual-review viewer.

use anyhow::Result;
use bevy::prelude::*;
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};
use cf_viewer::{
    UpAxis, ViewerInput,
    camera::{OrbitCamera, orbit_camera_input, update_orbit_camera},
    cli::{Cli, seed_selection},
    colormap::{Colormap, ColormapKind},
    load_input,
    mesh::{POINT_RADIUS_FRACTION, build_face_mesh, vertex_position},
    ui::{ColormapOverride, GeometryEntity, Selection, scalar_and_colormap_panel},
};
use clap::Parser;
use mesh_types::Bounded;

fn main() -> Result<()> {
    let cli = Cli::parse();

    let input = load_input(&cli.path)?;
    println!(
        "loaded {} vertices, {} scalars: {:?}",
        input.mesh.vertex_count(),
        input.scalar_names.len(),
        input.scalar_names,
    );

    let selection = seed_selection(&cli, &input.scalar_names)?;
    let up_axis: UpAxis = cli.up.into();

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cf-view".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
        .insert_resource(input)
        .insert_resource(selection)
        .insert_resource(up_axis)
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
                orbit_camera_input,
                update_orbit_camera.after(orbit_camera_input),
                spawn_geometry,
                exit_on_esc,
            ),
        )
        .add_systems(EguiPrimaryContextPass, scalar_and_colormap_panel)
        .run();

    Ok(())
}

/// Spawn the orbit camera + lighting once at startup. Geometry is spawned
/// (and re-spawned) by [`spawn_geometry`] in response to [`Selection`]
/// changes, so this system is no longer the geometry-spawn entry point.
#[allow(clippy::cast_possible_truncation)] // f64 → f32 is intentional for Bevy
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
fn setup_scene(mut commands: Commands, input: Res<ViewerInput>, up: Res<UpAxis>) {
    let aabb = input.mesh.aabb();
    let center_physics = aabb.center();
    // Same input → Bevy frame swap as `mesh::vertex_position` so the light
    // anchor stays aligned with the rendered geometry under any `--up=<...>`.
    let center_bevy = Vec3::from_array(vertex_position(&center_physics, *up));
    // Local `diagonal` is used here only for directional-light placement;
    // the camera's framing has its own clamp inside
    // `OrbitCamera::framing_for_aabb`. Single-point degenerate AABBs
    // and very small bboxes would otherwise place the light below the
    // visible-on-screen floor; clamp to 1.0 so it stays useful.
    let diagonal = (aabb.diagonal() as f32).max(1.0);

    // Orbit camera framed corner-on at 1.5 × diagonal, matching commit 3's
    // static placeholder pose. `framing_for_aabb` mirrors the up-axis swap
    // so the camera target tracks the rendered geometry's center.
    let orbit = OrbitCamera::framing_for_aabb(&aabb, *up);
    let mut transform = Transform::default();
    orbit.apply_to_transform(&mut transform);
    commands.spawn((Camera3d::default(), orbit, transform));

    // Strong directional key light + bright global ambient so geometry
    // stays readable in the geometry-only path. Bevy 0.18: `AmbientLight`
    // is now per-camera; world-wide ambient is `GlobalAmbientLight`
    // (sim-bevy `scene.rs:101` precedent).
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
}

/// Despawn-and-respawn the geometry whenever [`Selection`] changes (also
/// fires on first frame because Bevy treats freshly-inserted resources as
/// "changed"). Marker-tags every spawned entity with [`GeometryEntity`]
/// so the despawn step is bounded.
///
/// Two regimes (per `docs/VIEWER_DESIGN.md` iter 1.6 lock):
///
/// - **Scalar data present** → `unlit = true`. The base_color (set per
///   vertex/material from the colormap) renders faithfully without PBR
///   shading × lights × tonemap desaturating the hue. For a viz tool the
///   color *is* the data; shading fights it.
/// - **No scalars** → `unlit = false` (default). The mesh-v1.0
///   geometry-only path needs proper shading to read surface form.
///
/// **Faces case:** one `Mesh3d` entity carrying the converted Bevy mesh.
/// Per-vertex colors (when scalars are present) are baked into
/// `Mesh::ATTRIBUTE_COLOR`; the PBR shader overwrites
/// `material.base_color` from them.
///
/// **Faces-empty case (point cloud):** one tiny sphere entity per vertex,
/// sharing a single `Sphere` mesh handle. With scalars present each
/// entity gets its own `StandardMaterial` clone with `base_color` set to
/// the colormapped value (option A per iter-2 still-open #7).
#[allow(clippy::cast_possible_truncation)] // f64 → f32 is intentional for Bevy
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
fn spawn_geometry(
    selection: Res<Selection>,
    input: Res<ViewerInput>,
    up: Res<UpAxis>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    geometry: Query<Entity, With<GeometryEntity>>,
) {
    if !selection.is_changed() {
        return;
    }
    for entity in &geometry {
        commands.entity(entity).despawn();
    }

    // Resolve the active scalar (Q4 alphabetical-first-pick or UI dropdown
    // override). Empty scalar list → geometry-only path.
    let scalar_values: Option<&Vec<f32>> = if input.scalar_names.is_empty() {
        None
    } else {
        let i = selection.clamped_scalar_index(input.scalar_names.len());
        input.mesh.extras.get(&input.scalar_names[i])
    };
    let colormap = scalar_values.map(|values| {
        let mut cm = Colormap::from_values(values);
        // Apply the UI override (Auto = leave detector's pick alone).
        if let Some(kind) = override_to_kind(selection.colormap_override) {
            cm.kind = kind;
        }
        cm
    });
    let vertex_colors: Option<Vec<[f32; 4]>> = scalar_values
        .zip(colormap.as_ref())
        .map(|(values, cm)| values.iter().map(|&v| cm.rgba(v)).collect());

    let unlit = vertex_colors.is_some();
    let template_material = StandardMaterial {
        base_color: Color::srgb(0.70, 0.72, 0.78),
        metallic: 0.10,
        perceptual_roughness: 0.6,
        double_sided: true,
        cull_mode: None,
        unlit,
        ..default()
    };

    if input.mesh.face_count() > 0 {
        let bevy_mesh = build_face_mesh(&input.mesh, vertex_colors.as_deref(), *up);
        let material_handle = materials.add(template_material);
        commands.spawn((
            GeometryEntity,
            Mesh3d(meshes.add(bevy_mesh)),
            MeshMaterial3d(material_handle),
        ));
    } else {
        let aabb = input.mesh.aabb();
        let diagonal = (aabb.diagonal() as f32).max(1.0);
        let radius = (diagonal * POINT_RADIUS_FRACTION).max(1e-3);
        let sphere_handle = meshes.add(Sphere::new(radius));
        match vertex_colors.as_deref() {
            Some(colors) => {
                for (v, color) in input.mesh.geometry.vertices.iter().zip(colors) {
                    let material = materials.add(StandardMaterial {
                        base_color: Color::srgba(color[0], color[1], color[2], color[3]),
                        ..template_material.clone()
                    });
                    commands.spawn((
                        GeometryEntity,
                        Mesh3d(sphere_handle.clone()),
                        MeshMaterial3d(material),
                        Transform::from_translation(Vec3::from_array(vertex_position(v, *up))),
                    ));
                }
            }
            None => {
                let material_handle = materials.add(template_material);
                for v in &input.mesh.geometry.vertices {
                    commands.spawn((
                        GeometryEntity,
                        Mesh3d(sphere_handle.clone()),
                        MeshMaterial3d(material_handle.clone()),
                        Transform::from_translation(Vec3::from_array(vertex_position(v, *up))),
                    ));
                }
            }
        }
    }
}

/// Translate a UI override into a colormap kind. `Auto` returns `None`
/// so the caller leaves the detector's pick in place.
fn override_to_kind(o: ColormapOverride) -> Option<ColormapKind> {
    match o {
        ColormapOverride::Auto => None,
        ColormapOverride::Divergent => Some(ColormapKind::Divergent),
        ColormapOverride::Sequential => Some(ColormapKind::Sequential),
        ColormapOverride::Categorical => Some(ColormapKind::Categorical),
    }
}

#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
fn exit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn override_to_kind_maps_each_variant() {
        assert_eq!(override_to_kind(ColormapOverride::Auto), None);
        assert_eq!(
            override_to_kind(ColormapOverride::Divergent),
            Some(ColormapKind::Divergent)
        );
        assert_eq!(
            override_to_kind(ColormapOverride::Sequential),
            Some(ColormapKind::Sequential)
        );
        assert_eq!(
            override_to_kind(ColormapOverride::Categorical),
            Some(ColormapKind::Categorical)
        );
    }
}
