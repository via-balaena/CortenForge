//! `cf-view` — workspace-wide visual-review viewer.

use std::path::PathBuf;

use anyhow::{Result, anyhow};
use bevy::prelude::*;
use cf_viewer::{
    ViewerInput,
    colormap::Colormap,
    load_input,
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
/// converted Bevy mesh. Per-vertex colors (when scalars are present) are
/// baked into `Mesh::ATTRIBUTE_COLOR` and the PBR shader overwrites
/// `material.base_color` from them.
///
/// **Faces-empty case** (point cloud): one tiny sphere entity per vertex,
/// sharing a single `Sphere` mesh handle (per `docs/VIEWER_DESIGN.md`
/// iter-2 still-open #6). Sphere radius is `bbox.diagonal() *
/// POINT_RADIUS_FRACTION`. With scalars present, each entity gets its own
/// `StandardMaterial` clone with `base_color` set to the colormapped value
/// (option A per iter-2 still-open #7); without scalars, all spheres share
/// the neutral grey template material.
///
/// **Scalar selection:** auto-picks `input.scalar_names[0]`
/// (`BTreeMap` iteration is alphabetical → deterministic) per Q4. The
/// `--scalar=<name>` CLI flag (commit 6) overrides.
///
/// The camera + lights size themselves to the input's AABB. Orbit camera
/// (commit 5) replaces this static placeholder.
#[allow(clippy::cast_possible_truncation)] // f64 → f32 is intentional for Bevy
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
#[allow(clippy::too_many_lines)] // single-system scene setup; orbit-cam commit 5 will split
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
    // Single-point degenerate case (min == max → diagonal == 0) and very
    // small AABBs would shrink the camera offset + sphere radius below the
    // visible-on-screen floor; clamp to 1.0 so framing stays sane.
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

    // Auto-select the first scalar (Q4 alphabetical-first-pick). `BTreeMap`
    // iteration is alphabetical → `scalar_names[0]` is the deterministic
    // pick. The CLI override (`--scalar=<name>`) lands in commit 6.
    let scalar_values: Option<&Vec<f32>> = input
        .scalar_names
        .first()
        .and_then(|name| input.mesh.extras.get(name));
    let colormap = scalar_values.map(|values| Colormap::from_values(values));
    let vertex_colors: Option<Vec<[f32; 4]>> = scalar_values
        .zip(colormap.as_ref())
        .map(|(values, cm)| values.iter().map(|&v| cm.rgba(v)).collect());

    // Template material. Two regimes (iter 1.6 lock):
    //
    // - **Scalar data present** → `unlit = true`. The base_color (set per
    //   vertex/material from the colormap) renders faithfully without
    //   PBR shading × bright lights × tonemap desaturating the hue.
    //   For a viz tool the color *is* the data; shading fights it.
    //
    // - **No scalars** → `unlit = false` (default). The mesh-v1.0
    //   geometry-only path needs proper shading to read surface form.
    //
    // The PBR shader still overwrites `base_color` from
    // `Mesh::ATTRIBUTE_COLOR` when present (faces case), so the
    // template's grey is moot in that branch — the unlit flag is what
    // actually keeps the colors honest.
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
        let bevy_mesh = build_face_mesh(&input.mesh, vertex_colors.as_deref());
        let material_handle = materials.add(template_material);
        commands.spawn((
            Mesh3d(meshes.add(bevy_mesh)),
            MeshMaterial3d(material_handle),
        ));
    } else {
        let radius = (diagonal * POINT_RADIUS_FRACTION).max(1e-3);
        let sphere_handle = meshes.add(Sphere::new(radius));
        match vertex_colors.as_deref() {
            Some(colors) => {
                // Per-vertex `StandardMaterial` (iter-2 still-open #7
                // option A): one material clone per sphere with
                // `base_color` set to the colormapped value. Wasteful but
                // simple; combined-mesh (option B) is the polish-commit
                // direction if a future consumer pushes past ~10K verts.
                for (v, color) in input.mesh.geometry.vertices.iter().zip(colors) {
                    let material = materials.add(StandardMaterial {
                        base_color: Color::srgba(color[0], color[1], color[2], color[3]),
                        ..template_material.clone()
                    });
                    commands.spawn((
                        Mesh3d(sphere_handle.clone()),
                        MeshMaterial3d(material),
                        Transform::from_translation(Vec3::from_array(vertex_position(v))),
                    ));
                }
            }
            None => {
                let material_handle = materials.add(template_material);
                for v in &input.mesh.geometry.vertices {
                    commands.spawn((
                        Mesh3d(sphere_handle.clone()),
                        MeshMaterial3d(material_handle.clone()),
                        Transform::from_translation(Vec3::from_array(vertex_position(v))),
                    ));
                }
            }
        }
    }
}

#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
fn exit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}
