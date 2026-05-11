//! `cf-view` — workspace-wide visual-review viewer.

use anyhow::Result;
use bevy::prelude::*;
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};
use cf_bevy_common::prelude::*;
use cf_viewer::{
    InputMode, UpAxis, ViewerInput,
    cli::{Cli, seed_selection},
    colormap::{Colormap, ColormapKind},
    detect_input_mode, load_input,
    mesh::{POINT_RADIUS_FRACTION, build_face_mesh},
    sequence::sequence_info_panel,
    ui::{ColormapOverride, GeometryEntity, Selection, scalar_and_colormap_panel},
};
use clap::Parser;
use mesh_types::{Aabb, Bounded, Point3};

fn main() -> Result<()> {
    let cli = Cli::parse();

    let mode = detect_input_mode(&cli.path)?;
    // Sequence-mode log lives in `main` (not in `detect_input_mode`) so
    // the lib function stays side-effect free and reusable.
    if let InputMode::Sequence(seq) = &mode {
        let first_name = seq.current_name().unwrap_or("?");
        println!(
            "detected PLY sequence: {} frame(s) in {}; rendering frame 1/{} ({})",
            seq.len(),
            cli.path.display(),
            seq.len(),
            first_name,
        );
    }
    let frame_path = mode.initial_frame()?;

    let input = load_input(&frame_path)?;
    let raw_diagonal = input.mesh.aabb().diagonal() as f32;
    let render_scale = compute_render_scale(raw_diagonal);
    println!(
        "loaded {} vertices, {} scalars: {:?}",
        input.mesh.vertex_count(),
        input.scalar_names.len(),
        input.scalar_names,
    );
    println!(
        "bbox diagonal = {raw_diagonal:.4} m; render_scale = {render_scale:.2}× \
         (sub-meter scenes are lifted to ~1 m for Bevy's pipeline-default \
         camera + lighting; ≥1 m scenes render at native scale)"
    );

    let selection = seed_selection(&cli, &input.scalar_names)?;
    let up_axis: UpAxis = cli.up.into();

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "cf-view".into(),
            ..default()
        }),
        ..default()
    }))
    .add_plugins(EguiPlugin::default())
    .add_plugins(OrbitCameraPlugin)
    .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
    .insert_resource(input)
    .insert_resource(selection)
    .insert_resource(up_axis)
    .insert_resource(RenderScale(render_scale));

    // Sequence-mode-only resource. The `sequence_info_panel` system is
    // added unconditionally below but is itself gated on
    // `Option<Res<Sequence>>` inside the handler, so single-file mode
    // pays nothing.
    if let InputMode::Sequence(seq) = mode {
        app.insert_resource(seq);
    }

    app.add_systems(Startup, setup_scene)
        .add_systems(Update, (spawn_geometry, exit_on_esc))
        // Both panel systems take `EguiContexts` so Bevy auto-serializes
        // them, but the relative order isn't otherwise guaranteed.
        // `.chain()` pins it so layout (which panel claims space first
        // within an egui frame) is deterministic across runs.
        .add_systems(
            EguiPrimaryContextPass,
            (scalar_and_colormap_panel, sequence_info_panel).chain(),
        )
        .run();

    Ok(())
}

/// Render-side scale factor applied uniformly to all spawned geometry so
/// sub-meter scenes lift to Bevy's pipeline-default human-scale (~1 m)
/// regime. Bevy 0.18's defaults — near plane `0.1 m`,
/// [`OrbitCamera::framing_for_aabb`]'s internal `.max(1.0)` clamp on
/// diagonal, AmbientLight brightness — were tuned for human-scale scenes;
/// at sim-soft's cm-scale (e.g. row 13's 52.6 mm bbox diagonal — the
/// BCC mesher allocates a cube of side `2 (R + margin)` around the
/// 1 cm sphere, with margin scaling per cell-size), the framing helper
/// clamps diagonal up to `1.0` and places the camera 1.5 m away —
/// geometry then renders as a single dot. Lifting the rendered scene to
/// ~1 m diagonal puts everything safely within the defaults' working
/// range. mesh-v1.0 examples already at meter scale get
/// `render_scale = 1.0` (no change).
///
/// Banked at sim-soft EXAMPLE_INVENTORY iter-12 as the cf-view application
/// of inventory iter-11 pattern (b) (RENDER_SCALE-as-rendering-pipeline-
/// default-workaround); same root cause as sim-bevy-soft's row-12 +
/// row-13 RENDER_SCALE policy.
#[derive(Resource, Clone, Copy, Debug)]
struct RenderScale(f32);

/// Compute the render scale from the raw bbox diagonal: lift sub-meter
/// scenes to a 1 m target diagonal; meter+ scenes render at native scale.
/// Degenerate (zero / non-finite) diagonals fall back to `1.0` so the
/// downstream framing helper's own clamp handles them.
fn compute_render_scale(raw_diagonal: f32) -> f32 {
    const TARGET_DIAGONAL: f32 = 1.0;
    if !raw_diagonal.is_finite() || raw_diagonal <= 0.0 || raw_diagonal >= TARGET_DIAGONAL {
        1.0
    } else {
        TARGET_DIAGONAL / raw_diagonal
    }
}

/// Apply a uniform scale factor to an [`Aabb`]'s corners. Used to compute
/// the camera-framing AABB at render scale (the rendered geometry's
/// bbox), distinct from the loaded mesh's physics-scale AABB.
fn scale_aabb(raw: &Aabb, scale: f32) -> Aabb {
    let s = scale as f64;
    Aabb::from_corners(
        Point3::new(raw.min.x * s, raw.min.y * s, raw.min.z * s),
        Point3::new(raw.max.x * s, raw.max.y * s, raw.max.z * s),
    )
}

/// Spawn the orbit camera + lighting once at startup. Geometry is spawned
/// (and re-spawned) by [`spawn_geometry`] in response to [`Selection`]
/// changes, so this system is no longer the geometry-spawn entry point.
///
/// The orbit camera frames the **rendered** AABB (raw mesh AABB scaled by
/// [`RenderScale`]) — at native scale for meter+ meshes, lifted to ~1 m
/// for sub-meter meshes. See [`RenderScale`] for the policy.
// f64 → f32 is intentional for Bevy
#[allow(clippy::cast_possible_truncation)]
// Bevy systems take resources by value (Res / ResMut / Query / Commands).
#[allow(clippy::needless_pass_by_value)]
fn setup_scene(
    mut commands: Commands,
    input: Res<ViewerInput>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
) {
    let raw_aabb = input.mesh.aabb();
    let aabb = scale_aabb(&raw_aabb, render_scale.0);
    let center_physics = aabb.center();
    // Same input → Bevy frame swap as `build_face_mesh` so the light
    // anchor stays aligned with the rendered geometry under any `--up=<...>`.
    let center_bevy = Vec3::from_array(up.to_bevy_point(&center_physics));
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
// f64 → f32 is intentional for Bevy
#[allow(clippy::cast_possible_truncation)]
// Bevy systems take resources by value (Res / ResMut / Query / Commands).
#[allow(clippy::needless_pass_by_value)]
// 8 args (1 over the 7-default) — Bevy systems pull each Res / ResMut /
// Query / Commands as a separate parameter; threading them through a
// SystemParam-derive tuple costs more boilerplate than the +1 buys in
// readability. Mirrors the example precedent of `setup_visual_scene` in
// `examples/sim-soft/soft-drop-on-plane/src/main.rs`.
#[allow(clippy::too_many_arguments)]
fn spawn_geometry(
    selection: Res<Selection>,
    input: Res<ViewerInput>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
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

    let scale = render_scale.0;
    if input.mesh.face_count() > 0 {
        // Faces case — single Mesh3d carrying the full triangle set, with
        // a uniform Transform::from_scale lifting cm-scale physics
        // positions to Bevy's human-scale rendering regime (or = 1.0 for
        // meter+ meshes; no change from native).
        let bevy_mesh = build_face_mesh(&input.mesh, vertex_colors.as_deref(), *up);
        let material_handle = materials.add(template_material);
        commands.spawn((
            GeometryEntity,
            Mesh3d(meshes.add(bevy_mesh)),
            MeshMaterial3d(material_handle),
            Transform::from_scale(Vec3::splat(scale)),
        ));
    } else {
        // Point-cloud case — one Mesh3d per vertex sharing a Sphere mesh
        // handle. The sphere RADIUS is computed from the SCALED bbox
        // diagonal so per-vertex sphere size scales with the rendered
        // scene; per-vertex POSITION is multiplied by `scale` at spawn so
        // the cluster spans the rendered bbox not the physics bbox.
        //
        // Radius is the MIN of two bounds:
        //   1. `diagonal × POINT_RADIUS_FRACTION` — the original
        //      empirical bound, calibrated against sphere-sdf-eval's
        //      sparse 11³ grid.
        //   2. `0.4 × (V/N)^(1/3)` — density-aware bound. With `N`
        //      points uniformly distributed in volume `V`, the average
        //      inter-point spacing is `(V/N)^(1/3)`; rendering each as
        //      a sphere ~40% of that spacing keeps adjacent spheres
        //      visually discrete (~20% gap). Surfaces dense per-tet
        //      centroid clouds (sim-soft layered-scalar-field row 8 +
        //      successors) where the diagonal-only rule gave overlap.
        let raw_aabb = input.mesh.aabb();
        let scaled_aabb = scale_aabb(&raw_aabb, scale);
        let diagonal = (scaled_aabb.diagonal() as f32).max(1.0);
        let n_points = input.mesh.geometry.vertices.len();
        let diagonal_radius = diagonal * POINT_RADIUS_FRACTION;
        // `n_points == 0` is unreachable here (this branch is gated on a
        // non-empty mesh up the call stack), but defensive against a
        // future caller change. `cbrt` is well-defined on f32.
        let density_radius = if n_points > 0 {
            // `n_points as f32` precision loss only matters above ~16M
            // points where f32 representation truncates; sim-soft and
            // mesh examples all sit < 100k. Cast is loss-free in scope.
            #[allow(clippy::cast_precision_loss)]
            let n_f = n_points as f32;
            #[allow(clippy::cast_possible_truncation)]
            let volume = scaled_aabb.volume() as f32;
            0.4 * (volume / n_f).cbrt()
        } else {
            f32::INFINITY
        };
        let radius = diagonal_radius.min(density_radius).max(1e-3);
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
                        Transform::from_translation(Vec3::from_array(up.to_bevy_point(v)) * scale),
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
                        Transform::from_translation(Vec3::from_array(up.to_bevy_point(v)) * scale),
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
