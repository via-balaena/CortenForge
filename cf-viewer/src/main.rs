//! `cf-view` — workspace-wide visual-review viewer.

use anyhow::Result;
use bevy::prelude::*;
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};
use cf_bevy_common::prelude::*;
use cf_viewer::{
    AssemblyInputs, InputMode, RenderScale, UpAxis, ViewerInput,
    cli::{Cli, seed_selection},
    colormap::{Colormap, ColormapKind},
    compute_render_scale, detect_input_mode, load_assembly_inputs, load_input,
    mesh::{POINT_RADIUS_FRACTION, build_face_mesh},
    scale_aabb,
    sequence::{
        Playback, advance_playback_on_clock, handle_frame_navigation, handle_playback_input,
        reload_frame_on_change, sequence_info_panel,
    },
    setup_camera_and_lighting,
    ui::{ColormapOverride, GeometryEntity, Selection, scalar_and_colormap_panel},
};
use clap::Parser;
use mesh_types::Bounded;

fn main() -> Result<()> {
    let cli = Cli::parse();

    // `--assembly` opts the directory-input path into Option C
    // (multi-piece-in-one-scene) instead of the default scrub-through-
    // pieces sequence mode. Only meaningful when `path` is a directory
    // of STLs; ignored for single-file or PLY-sequence inputs.
    let mode = if cli.assembly && cli.path.is_dir() {
        let stls = cf_viewer::discover_stl_sequence(&cli.path)?;
        InputMode::Assembly { stls }
    } else {
        detect_input_mode(&cli.path)?
    };
    match &mode {
        InputMode::Sequence(seq) => {
            let first_name = seq.current_name().unwrap_or("?");
            println!(
                "detected sequence: {} frame(s) in {}; rendering frame 1/{} ({})",
                seq.len(),
                cli.path.display(),
                seq.len(),
                first_name,
            );
        }
        InputMode::Assembly { stls } => {
            println!(
                "assembly mode: loading {} STLs from {}",
                stls.len(),
                cli.path.display(),
            );
        }
        InputMode::Single(_) => {}
    }

    // Assembly mode bypasses the single-mesh ViewerInput pipeline
    // entirely — load all STLs up front, insert as an AssemblyInputs
    // resource, skip seed_selection / Selection (no scalar dropdown
    // for STL-only data).
    if let InputMode::Assembly { stls } = &mode {
        return run_assembly_mode(&cli, stls);
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

    // Sequence-mode-only resources. The sequence systems below are
    // added unconditionally but each one is gated on
    // `Option<Res<Sequence>>` / `Option<Res<Playback>>` inside the
    // handler, so single-file mode pays nothing.
    if let InputMode::Sequence(seq) = mode {
        app.insert_resource(seq);
        app.insert_resource(Playback::default());
    }

    app.add_systems(Startup, setup_scene)
        // Sequence-mode chain: explicit user input → automatic clock
        // advance → reload → respawn, all within a single frame so a
        // keyboard step / space toggle / scrub drag renders the new
        // frame on the same tick. Manual input runs first so the
        // clock can't race a just-stepped frame; reload runs after
        // any `Sequence::current` mutation so it sees the latest
        // index. `exit_on_esc` has no resource conflicts and lives
        // outside the chain.
        .add_systems(
            Update,
            (
                handle_frame_navigation,
                handle_playback_input,
                advance_playback_on_clock,
                reload_frame_on_change,
                spawn_geometry,
            )
                .chain(),
        )
        .add_systems(Update, exit_on_esc)
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

// ============================================================
// Assembly mode (Option C) — multi-piece-in-one-scene.
// ============================================================

/// Per-piece marker for [`InputMode::Assembly`]'s spawn output.
/// Each piece's spawned entity carries this component so the
/// `assembly_piece_panel` UI can flip `Visibility` per piece by
/// name.
#[derive(Component, Debug)]
struct AssemblyPiece {
    /// Zero-based index into [`AssemblyInputs::pieces`].
    #[allow(dead_code)] // Reserved for future C.x slices that need stable indices.
    index: usize,
    /// Filename — used by the side-panel UI as the per-piece label
    /// + the visibility-toggle key.
    name: String,
}

/// Per-piece visibility state — `pieces[name] = visible?`.
/// Default all-visible at startup; mutated by
/// `assembly_piece_panel`; consumed by `apply_assembly_visibility`
/// each frame.
#[derive(Resource, Debug, Default)]
struct AssemblyVisibility {
    visible: std::collections::HashMap<String, bool>,
}

/// Drive the Bevy app for Option C assembly mode — load all STLs
/// up front, insert as [`AssemblyInputs`], skip scrub UI + scalar
/// dropdown machinery.
///
/// Branches off `main` BEFORE the standard PLY/Sequence pipeline
/// builds its `ViewerInput` resource. Assembly mode runs an
/// independent Bevy app with its own setup + spawn systems.
fn run_assembly_mode(cli: &Cli, stls: &[std::path::PathBuf]) -> Result<()> {
    let assembly = load_assembly_inputs(stls)?;
    println!("loaded {} assembly pieces:", assembly.pieces.len(),);
    for piece in &assembly.pieces {
        println!(
            "  {} ({} vertices, {} faces)",
            piece.name,
            piece.mesh.vertex_count(),
            piece.mesh.geometry.faces.len(),
        );
    }

    // Compute the combined AABB across all pieces so the camera
    // frames the whole assembly, not just the first piece.
    let combined_aabb = combined_assembly_aabb(&assembly);
    let raw_diagonal = combined_aabb.diagonal() as f32;
    let render_scale = compute_render_scale(raw_diagonal);
    println!("assembly bbox diagonal = {raw_diagonal:.4} m; render_scale = {render_scale:.2}×",);

    let up_axis: UpAxis = cli.up.into();

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "cf-view (assembly)".into(),
            ..default()
        }),
        ..default()
    }))
    .add_plugins(EguiPlugin::default())
    .add_plugins(OrbitCameraPlugin)
    .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
    .insert_resource(assembly)
    .insert_resource(AssemblyVisibility::default())
    .insert_resource(up_axis)
    .insert_resource(RenderScale(render_scale));

    app.add_systems(
        Startup,
        (setup_assembly_scene, spawn_assembly_pieces).chain(),
    )
    .add_systems(Update, (apply_assembly_visibility, exit_on_esc))
    .add_systems(EguiPrimaryContextPass, assembly_piece_panel)
    .run();

    Ok(())
}

/// Compute the union AABB across all pieces' raw mesh AABBs. The
/// camera + render-scale logic frames this so all pieces are
/// visible at startup.
fn combined_assembly_aabb(assembly: &AssemblyInputs) -> mesh_types::Aabb {
    use mesh_types::Point3;
    let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for piece in &assembly.pieces {
        let aabb = piece.mesh.aabb();
        min.x = min.x.min(aabb.min.x);
        min.y = min.y.min(aabb.min.y);
        min.z = min.z.min(aabb.min.z);
        max.x = max.x.max(aabb.max.x);
        max.y = max.y.max(aabb.max.y);
        max.z = max.z.max(aabb.max.z);
    }
    mesh_types::Aabb::from_corners(min, max)
}

/// Startup system — frame the camera on the combined assembly AABB.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take Res/ResMut/Query/Commands by value
fn setup_assembly_scene(
    mut commands: Commands,
    assembly: Res<AssemblyInputs>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
) {
    let raw_aabb = combined_assembly_aabb(&assembly);
    let aabb = scale_aabb(&raw_aabb, render_scale.0);
    setup_camera_and_lighting(&mut commands, &aabb, *up);
}

/// `tab10` categorical palette — the same 10 colors matplotlib
/// uses by default for categorical line plots, well-tested for
/// visual distinguishability. Assembly mode cycles through this
/// palette by piece index so each piece gets a distinct color.
/// Wraps at index 10 (cf-cast v2 emits up to `2 * L + 1` STLs,
/// so 7 for the 3-layer fixture; for 5-layer that's 11, which
/// wraps but only one collision).
const TAB10: [Color; 10] = [
    Color::srgb(0.121, 0.466, 0.705), // blue
    Color::srgb(1.000, 0.498, 0.054), // orange
    Color::srgb(0.172, 0.627, 0.172), // green
    Color::srgb(0.839, 0.152, 0.156), // red
    Color::srgb(0.580, 0.403, 0.741), // purple
    Color::srgb(0.549, 0.337, 0.294), // brown
    Color::srgb(0.890, 0.466, 0.760), // pink
    Color::srgb(0.498, 0.498, 0.498), // gray
    Color::srgb(0.737, 0.741, 0.133), // olive
    Color::srgb(0.090, 0.745, 0.811), // cyan
];

/// Startup system — spawn one entity per piece. Each piece gets a
/// flat-shaded mesh + a [`TAB10`] color indexed by piece position
/// so overlapping pieces stay visually distinguishable.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take Res/ResMut/Query/Commands by value
fn spawn_assembly_pieces(
    assembly: Res<AssemblyInputs>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let scale = render_scale.0;
    for (index, piece) in assembly.pieces.iter().enumerate() {
        let bevy_mesh = build_face_mesh(&piece.mesh, None, *up);
        let piece_color = TAB10[index % TAB10.len()];
        let material = StandardMaterial {
            base_color: piece_color,
            metallic: 0.10,
            perceptual_roughness: 0.6,
            double_sided: true,
            cull_mode: None,
            unlit: false,
            // Slight transparency so overlapping pieces don't fully
            // hide each other on first orbit. Workshop user can
            // toggle individual pieces off via the side panel for
            // a fully opaque look at any single piece.
            alpha_mode: AlphaMode::Blend,
            ..default()
        };
        let mut material_with_alpha = material;
        material_with_alpha.base_color = piece_color.with_alpha(0.55);
        commands.spawn((
            AssemblyPiece {
                index,
                name: piece.name.clone(),
            },
            Mesh3d(meshes.add(bevy_mesh)),
            MeshMaterial3d(materials.add(material_with_alpha)),
            Transform::from_scale(Vec3::splat(scale)),
        ));
    }
}

/// Side panel: list every assembly piece with a visibility
/// checkbox. Default all-on. Mutates [`AssemblyVisibility`] which
/// [`apply_assembly_visibility`] then propagates to each entity's
/// `Visibility` component.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take Res/ResMut/Query/Commands by value
fn assembly_piece_panel(
    mut contexts: bevy_egui::EguiContexts,
    assembly: Res<AssemblyInputs>,
    mut visibility: ResMut<AssemblyVisibility>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };
    bevy_egui::egui::SidePanel::left("assembly_pieces")
        .min_width(220.0)
        .show(ctx, |ui| {
            ui.heading("cf-view (assembly)");
            ui.label("Per-piece visibility:");
            ui.separator();
            for piece in &assembly.pieces {
                let entry = visibility.visible.entry(piece.name.clone()).or_insert(true);
                ui.checkbox(entry, &piece.name);
            }
            ui.separator();
            ui.label(format!("{} pieces total", assembly.pieces.len()));
            ui.label("LMB orbit · RMB pan · scroll zoom · Esc exit");
        });
}

/// Each-frame system: read [`AssemblyVisibility`], flip each
/// `AssemblyPiece` entity's `Visibility` to match. Egui mutates the
/// resource on checkbox click; this system applies it.
fn apply_assembly_visibility(
    visibility: Res<AssemblyVisibility>,
    mut query: Query<(&AssemblyPiece, &mut Visibility)>,
) {
    if !visibility.is_changed() {
        return;
    }
    for (piece, mut vis) in &mut query {
        let visible = visibility.visible.get(&piece.name).copied().unwrap_or(true);
        *vis = if visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}

/// Spawn the orbit camera + lighting once at startup. Geometry is spawned
/// (and re-spawned) by [`spawn_geometry`] in response to [`Selection`]
/// changes, so this system is no longer the geometry-spawn entry point.
///
/// The orbit camera frames the **rendered** AABB (raw mesh AABB scaled by
/// [`RenderScale`]) — at native scale for meter+ meshes, lifted to ~1 m
/// for sub-meter meshes. See [`RenderScale`] for the policy. Camera + light
/// placement is delegated to [`setup_camera_and_lighting`] so
/// `cf-scan-prep` can reuse the same framing.
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
    setup_camera_and_lighting(&mut commands, &aabb, *up);
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
    // Respawn when either the user changes the scalar/colormap selection
    // OR the loaded frame swaps (sequence-mode frame navigation in D1.3
    // mutates `ViewerInput` via `sequence::reload_frame_on_change`).
    if !selection.is_changed() && !input.is_changed() {
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
