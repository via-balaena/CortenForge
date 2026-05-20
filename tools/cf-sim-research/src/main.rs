//! cf-sim-research — sim-research viewer skeleton.
//!
//! Phase 2 of the sim-decouple refactor
//! (`docs/SIM_DECOUPLE_REFACTOR_PLAN.md` §4). The CAD binary
//! (`cf-device-design`) and this sim-research binary
//! (`cf-sim-research`) both consume `cf-device-types` so they
//! describe a layered-silicone device the same way; Phase 3 then
//! moves the insertion-sim + heat-map + sliding-intruder rendering
//! out of cf-device-design into this crate, and Phase 4 strips them
//! from cf-device-design entirely.
//!
//! This Phase-2 binary intentionally does **not** wire any sim:
//! - No FEM. No insertion-sim panel.
//! - No SDF iso-extraction of cavity / layer shells (mesh-sdf +
//!   mesh-offset are deliberately not in our dep graph yet — Phase 3
//!   adds them).
//! - No `.design.toml` load (the schema is owned by cf-device-design's
//!   private `design_toml` module today; Phase 4 / a Phase 2.5
//!   mini-arc lifts it to a shared crate). Phase 2 loads scan +
//!   `.prep.toml` only and surfaces the default cavity + layer stack
//!   in the read-only panel.
//!
//! What Phase 2 DOES wire: the scan → cf-device-types →
//! Bevy/egui render pipeline. If this binary launches a window
//! showing the scan, centerline overlay, and a right-side panel
//! listing the default layer stack with palette-tinted swatches
//! (the per-layer color is in the panel row, not on a 3D shell —
//! shell rendering arrives in Phase 3 with mesh-sdf + mesh-offset),
//! the cross-binary type-share contract is proved.

use std::path::PathBuf;

use anyhow::{Context, Result};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use cf_bevy_common::prelude::*;
use cf_device_types::{
    CavityState, Centerline, LAYER_MATERIALS, LayersState, ScanFilePath, ScanInfo, ScanMesh,
    ScanMeshVisible, material_density,
};
use cf_viewer::{
    RenderScale, compute_render_scale, scale_aabb, setup_camera_and_lighting, spawn_face_mesh,
};

use clap::Parser;
use mesh_io::load_stl;
use mesh_types::{Bounded, IndexedMesh, Point3};
use serde::Deserialize;

/// Cast-frame demolding-axis convention: `+Z` is up. Inherited from
/// cf-scan-prep + cf-cast + cf-device-design — every CortenForge cast
/// tool assumes the `UpAxis::PlusZ` swap from physics frame to Bevy
/// frame.
const DEVICE_UP_AXIS: UpAxis = UpAxis::PlusZ;

/// Palette tinting the per-layer rows in the right-side readout
/// panel (a 14×14 px swatch per row — no 3D shells exist yet, see
/// the lib-level docstring). Mirrors cf-device-design's
/// `LAYER_SURFACE_PALETTE` by-value so the two viewers tint the
/// same layer-index with the same color; Phase 4 lifts this
/// constant to a shared crate when the layer-mesh helpers
/// consolidate.
const LAYER_SURFACE_PALETTE: &[(f32, f32, f32)] = &[
    (0.95, 0.80, 0.35), // amber
    (0.45, 0.70, 0.95), // sky blue
    (0.75, 0.55, 0.95), // lavender
    (0.55, 0.95, 0.65), // mint
    (0.95, 0.45, 0.70), // pink
];

#[derive(Parser, Debug)]
#[command(
    name = "cf-sim-research",
    about = "Sim-research viewer skeleton for layered-silicone devices (Phase 2 — scan + centerline + cavity + layer-stack readout; sim-side rendering lands in Phase 3)",
    version
)]
struct Cli {
    /// Path to the cleaned scan STL (cf-scan-prep output;
    /// `<stem>.cleaned.stl`).
    cleaned_stl: PathBuf,

    /// Optional path to a `.design.toml` companion file. Accepted
    /// here so the CLI shape matches cf-device-design's; the actual
    /// load lands in Phase 3 (the loader currently lives behind
    /// cf-device-design's private `design_toml` module and will be
    /// promoted to a shared crate then). Phase 2 logs a notice if
    /// the resolved sibling file exists; the viewer always renders
    /// the `default_for_scan` cavity + layer stack.
    #[arg(long, value_name = "PATH")]
    design: Option<PathBuf>,

    /// Optional path to the cf-scan-prep `.prep.toml` companion file.
    /// Defaults to `<cleaned_stl_stem>.prep.toml` next to the cleaned
    /// STL. The `[centerline].points_m` block drives the on-screen
    /// centerline overlay; absent, the overlay does not render. Cap
    /// planes are parsed alongside and their count is logged, but
    /// the planes themselves are discarded — Phase 3 stores them as
    /// a resource when the cavity/layer iso-extraction wires in.
    #[arg(long, value_name = "PATH")]
    prep_toml: Option<PathBuf>,
}

#[derive(Deserialize)]
struct PrepTomlSubset {
    centerline: Option<CenterlineBlock>,
}

#[derive(Deserialize)]
struct CenterlineBlock {
    points_m: Vec<[f64; 3]>,
}

/// Parse a `.prep.toml` string and return the centerline polyline.
/// Returns an empty `Vec` if the `[centerline]` block is absent or
/// empty — same posture as cf-device-design's `parse_centerline`.
fn parse_centerline(text: &str) -> Result<Vec<Point3<f64>>> {
    let subset: PrepTomlSubset = toml::from_str(text)?;
    Ok(subset
        .centerline
        .map(|c| c.points_m)
        .unwrap_or_default()
        .into_iter()
        .map(|[x, y, z]| Point3::new(x, y, z))
        .collect())
}

/// Resolve the `.prep.toml` path from CLI args. Honors `--prep-toml
/// <path>` when supplied; else falls back to `<cleaned_stl_stem>
/// .prep.toml` next to the cleaned STL. Mirrors cf-device-design's
/// stem-stripping convention so both viewers resolve the same
/// companion file from the same cleaned STL.
fn resolve_prep_toml_path(cli: &Cli) -> Option<PathBuf> {
    if let Some(p) = &cli.prep_toml {
        return Some(p.clone());
    }
    let parent = cli.cleaned_stl.parent()?;
    let stem = cli.cleaned_stl.file_stem()?.to_str()?;
    let base_stem = stem.strip_suffix(".cleaned").unwrap_or(stem);
    Some(parent.join(format!("{base_stem}.prep.toml")))
}

/// Resolve the `.design.toml` path — `--design` wins when supplied;
/// else `<cleaned_stl_stem-minus-.cleaned>.design.toml` next to the
/// cleaned STL. Same semantics as cf-device-design's
/// `design_toml::resolve_design_toml_path` (which takes
/// `(&Path, Option<&Path>)` directly; we take `&Cli` and unpack
/// the same way internally). Used in Phase 2 only to log a notice
/// when a sibling file exists; the actual parse lands in Phase 3.
fn resolve_design_toml_path(cli: &Cli) -> Option<PathBuf> {
    if let Some(p) = &cli.design {
        return Some(p.clone());
    }
    let parent = cli.cleaned_stl.parent()?;
    let stem = cli.cleaned_stl.file_stem()?.to_str()?;
    let base_stem = stem.strip_suffix(".cleaned").unwrap_or(stem);
    Some(parent.join(format!("{base_stem}.design.toml")))
}

/// Centerline polyline total arc length in meters. Zero for fewer
/// than 2 points.
fn centerline_arc_length_m(points: &[Point3<f64>]) -> f64 {
    points
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).norm())
        .sum()
}

fn build_scan_info(
    path: &std::path::Path,
    mesh: &IndexedMesh,
    centerline_points: &[Point3<f64>],
) -> ScanInfo {
    let aabb = mesh.aabb();
    let extents_mm = [
        (aabb.max.x - aabb.min.x) * 1000.0,
        (aabb.max.y - aabb.min.y) * 1000.0,
        (aabb.max.z - aabb.min.z) * 1000.0,
    ];
    ScanInfo {
        file_label: path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("(unknown)")
            .to_string(),
        vertex_count: mesh.vertices.len(),
        face_count: mesh.faces.len(),
        aabb_mm_extents: extents_mm,
        bbox_diagonal_m: aabb.diagonal(),
        centerline_arc_length_m: centerline_arc_length_m(centerline_points),
        centerline_point_count: centerline_points.len(),
    }
}

#[derive(Component)]
struct ScanMeshEntity;

fn main() -> Result<()> {
    let cli = Cli::parse();
    let scan_mesh = load_stl(&cli.cleaned_stl)
        .with_context(|| format!("load cleaned STL {}", cli.cleaned_stl.display()))?;

    let prep_toml_path = resolve_prep_toml_path(&cli);
    let (centerline_points, cap_plane_count) = if let Some(prep_path) = &prep_toml_path {
        match std::fs::read_to_string(prep_path) {
            Ok(text) => {
                let centerline = parse_centerline(&text).with_context(|| {
                    format!("parse `.prep.toml` centerline at {}", prep_path.display())
                })?;
                let caps = cf_cap_planes::parse_cap_planes(&text).with_context(|| {
                    format!("parse `.prep.toml` caps at {}", prep_path.display())
                })?;
                (centerline, caps.len())
            }
            Err(e) => {
                eprintln!(
                    "warning: could not read {} ({e:#}); centerline overlay disabled",
                    prep_path.display()
                );
                (Vec::new(), 0)
            }
        }
    } else {
        (Vec::new(), 0)
    };

    let scan_info = build_scan_info(&cli.cleaned_stl, &scan_mesh, &centerline_points);
    println!(
        "loaded {} vertices, {} faces; bbox diagonal = {:.4} m; \
         centerline = {} points / {:.4} m arc; cap planes = {cap_plane_count}",
        scan_info.vertex_count,
        scan_info.face_count,
        scan_info.bbox_diagonal_m,
        scan_info.centerline_point_count,
        scan_info.centerline_arc_length_m,
    );

    // Phase 2: defer `.design.toml` ingest. The loader + schema live
    // behind cf-device-design's private `design_toml` module; Phase 3
    // promotes it (or duplicates it in a controlled way) so this
    // binary can apply it onto the cf-device-types resources. For
    // now we resolve the path and surface a notice if a sibling file
    // exists — keeps the user informed without claiming functionality
    // we don't have.
    if let Some(p) = resolve_design_toml_path(&cli) {
        if p.exists() {
            println!(
                "design toml present at {} — load is wired in Phase 3 \
                 (this Phase-2 viewer renders the default layer stack)",
                p.display()
            );
        }
    }

    run_render_app(
        scan_mesh,
        scan_info,
        centerline_points,
        cli.cleaned_stl.clone(),
    );
    Ok(())
}

fn run_render_app(
    scan_mesh: IndexedMesh,
    scan_info: ScanInfo,
    centerline_points: Vec<Point3<f64>>,
    cleaned_stl_path: PathBuf,
) {
    // Cavity + layer-stack default state — Phase 3 splices the
    // `apply_design_toml` call in HERE (between the defaults and
    // `App::new()`) so callers don't need to change. Matches
    // cf-device-design's `run_render_app` shape at
    // `tools/cf-device-design/src/main.rs:2050-2061`.
    let cavity = CavityState::default_for_scan();
    let layers = LayersState::default_for_scan();
    #[allow(clippy::cast_possible_truncation)] // f64 → f32 is intentional for Bevy.
    let raw_diagonal = scan_mesh.aabb().diagonal() as f32;
    let render_scale = compute_render_scale(raw_diagonal);

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cf-sim-research".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
        .insert_resource(DEVICE_UP_AXIS)
        .insert_resource(RenderScale(render_scale))
        .insert_resource(ScanMesh(scan_mesh))
        .insert_resource(ScanFilePath(cleaned_stl_path))
        .insert_resource(scan_info)
        .insert_resource(Centerline {
            points_m: centerline_points,
        })
        .insert_resource(cavity)
        .insert_resource(layers)
        .insert_resource(ScanMeshVisible::default())
        .add_systems(Startup, setup_render_scene)
        .add_systems(
            Update,
            (
                block_orbit_input_when_over_egui.before(orbit_camera_input),
                draw_reference_overlays,
                apply_scan_mesh_visibility,
                exit_on_esc,
            ),
        )
        .add_systems(EguiPrimaryContextPass, sim_research_panel)
        .run();
}

#[allow(clippy::needless_pass_by_value)]
fn setup_render_scene(
    mut commands: Commands,
    scan: Res<ScanMesh>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let scale = render_scale.0;
    let raw_aabb = scan.0.aabb();
    let scaled_aabb = scale_aabb(&raw_aabb, scale);
    setup_camera_and_lighting(&mut commands, &scaled_aabb, *up);
    let entity_transform = Transform::from_scale(Vec3::splat(scale));
    let scan_entity = spawn_face_mesh(
        &mut commands,
        &mut meshes,
        &mut materials,
        &scan.0,
        None,
        *up,
        entity_transform,
    );
    commands.entity(scan_entity).insert(ScanMeshEntity);
}

/// Draw the world-origin axis arrows + the centerline polyline (if
/// present). Mirrors cf-device-design's reference-overlay shape so
/// the two viewers frame the scan identically.
#[allow(clippy::needless_pass_by_value)]
fn draw_reference_overlays(
    centerline: Res<Centerline>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    scan: Res<ScanMesh>,
    mut gizmos: Gizmos,
) {
    #[allow(clippy::cast_possible_truncation)]
    let arrow_length = (scan.0.aabb().diagonal() * 0.6 * f64::from(render_scale.0)) as f32;
    gizmos.arrow(
        Vec3::ZERO,
        Vec3::X * arrow_length,
        Color::srgb(1.0, 0.30, 0.30),
    );
    gizmos.arrow(
        Vec3::ZERO,
        Vec3::Z * arrow_length,
        Color::srgb(0.30, 1.0, 0.30),
    );
    gizmos.arrow(
        Vec3::ZERO,
        Vec3::Y * arrow_length,
        Color::srgb(0.40, 0.60, 1.00),
    );

    if centerline.points_m.is_empty() {
        return;
    }
    let cyan = Color::srgb(0.25, 0.85, 1.0);
    let positions: Vec<Vec3> = centerline
        .points_m
        .iter()
        .map(|p| Vec3::from_array(up.to_bevy_point(p)) * render_scale.0)
        .collect();
    for window in positions.windows(2) {
        gizmos.line(window[0], window[1], cyan);
    }
}

#[allow(clippy::needless_pass_by_value)]
fn apply_scan_mesh_visibility(
    visible: Res<ScanMeshVisible>,
    mut q: Query<&mut Visibility, With<ScanMeshEntity>>,
) {
    for mut v in &mut q {
        *v = if visible.0 {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}

#[allow(clippy::needless_pass_by_value)]
fn block_orbit_input_when_over_egui(
    mut contexts: EguiContexts,
    mut motion: ResMut<AccumulatedMouseMotion>,
    mut scroll: ResMut<AccumulatedMouseScroll>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    if ctx.wants_pointer_input() || ctx.is_pointer_over_area() {
        motion.delta = Vec2::ZERO;
        scroll.delta = Vec2::ZERO;
    }
    Ok(())
}

#[allow(clippy::needless_pass_by_value)]
fn exit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}

/// Right-side egui sidebar. Phase 2 surfaces a Scan Info readout, a
/// Scan visibility toggle, a Cavity inset readout, and a Layers
/// readout that tints each row by [`LAYER_SURFACE_PALETTE`]. Future
/// Phase-3 sim panels (insertion ramp, heat-map mode, sliding-
/// intruder scrub) attach below this in the same panel.
#[allow(clippy::needless_pass_by_value)]
fn sim_research_panel(
    mut contexts: EguiContexts,
    scan_info: Res<ScanInfo>,
    cavity: Res<CavityState>,
    layers: Res<LayersState>,
    mut scan_visible: ResMut<ScanMeshVisible>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("sim_research_panel")
        .min_width(300.0)
        .show(ctx, |ui| {
            ui.heading("cf-sim-research");
            ui.label(
                egui::RichText::new("Phase 2 skeleton — Phase 3 wires the sim.")
                    .small()
                    .italics(),
            );
            ui.separator();

            egui::CollapsingHeader::new("Scan Info")
                .default_open(true)
                .show(ui, |ui| {
                    ui.label(format!("File: {}", scan_info.file_label));
                    ui.label(format!("Vertices: {}", scan_info.vertex_count));
                    ui.label(format!("Faces: {}", scan_info.face_count));
                    ui.label(format!(
                        "AABB extents (mm): {:.1} × {:.1} × {:.1}",
                        scan_info.aabb_mm_extents[0],
                        scan_info.aabb_mm_extents[1],
                        scan_info.aabb_mm_extents[2],
                    ));
                    ui.label(format!("BBox diagonal: {:.3} m", scan_info.bbox_diagonal_m,));
                    ui.label(format!(
                        "Centerline: {} points / {:.3} m arc",
                        scan_info.centerline_point_count, scan_info.centerline_arc_length_m,
                    ));
                    let mut visible = scan_visible.0;
                    if ui.checkbox(&mut visible, "Show scan mesh").changed() {
                        scan_visible.0 = visible;
                    }
                });

            egui::CollapsingHeader::new("Cavity")
                .default_open(true)
                .show(ui, |ui| {
                    ui.label(format!(
                        "Inset: {:.2} mm (read-only, defaults — Phase 3 \
                         wires the editable slider)",
                        cavity.inset_m * 1e3,
                    ));
                });

            egui::CollapsingHeader::new("Layers")
                .default_open(true)
                .show(ui, |ui| {
                    if layers.layers.is_empty() {
                        ui.label("(no layers)");
                        return;
                    }
                    for (i, layer) in layers.layers.iter().enumerate() {
                        let (r, g, b) = LAYER_SURFACE_PALETTE[i % LAYER_SURFACE_PALETTE.len()];
                        let swatch = egui::Color32::from_rgb(
                            (r * 255.0) as u8,
                            (g * 255.0) as u8,
                            (b * 255.0) as u8,
                        );
                        ui.horizontal(|ui| {
                            let (rect, _) = ui
                                .allocate_exact_size(egui::vec2(14.0, 14.0), egui::Sense::hover());
                            ui.painter().rect_filled(rect, 2.0, swatch);
                            let label = layer_display_label(layer.material_anchor_key);
                            ui.label(format!(
                                "L{i}: {label} — {:.1} mm, slacker {:.0} %",
                                layer.thickness_m * 1e3,
                                layer.slacker_fraction * 100.0,
                            ));
                        });
                        ui.label(format!(
                            "    density {:.0} kg/m³",
                            material_density(layer.material_anchor_key),
                        ));
                    }
                });
        });
    Ok(())
}

/// Look up the display label for a silicone anchor key in
/// [`LAYER_MATERIALS`]. Falls back to the raw key for an unrecognized
/// entry (defensive only — the default layer stack ships an Ecoflex
/// 00-30 layer, which is in the catalog).
fn layer_display_label(anchor_key: &str) -> &str {
    LAYER_MATERIALS
        .iter()
        .find(|(key, _, _)| *key == anchor_key)
        .map_or(anchor_key, |(_, label, _)| *label)
}

#[cfg(test)]
mod tests {
    // `unwrap()` + `expect()` are denied at the crate level; allow
    // them inside tests so assertions stay readable.
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use std::path::Path;

    fn unit_cube_mesh() -> IndexedMesh {
        let mut m = IndexedMesh::new();
        let h = 0.05;
        for (x, y, z) in [
            (-h, -h, -h),
            (h, -h, -h),
            (h, h, -h),
            (-h, h, -h),
            (-h, -h, h),
            (h, -h, h),
            (h, h, h),
            (-h, h, h),
        ] {
            m.vertices.push(Point3::new(x, y, z));
        }
        for f in [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ] {
            m.faces.push(f);
        }
        m
    }

    #[test]
    fn parses_prep_toml_centerline_when_present() {
        let text = r#"
[centerline]
points_m = [
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 0.1],
    [0.0, 0.0, 0.2],
]
"#;
        let pts = parse_centerline(text).unwrap();
        assert_eq!(pts.len(), 3);
        assert!((pts[2].z - 0.2).abs() < 1e-12);
    }

    #[test]
    fn parse_centerline_absent_block_returns_empty() {
        let text = r#"
[scan_prep]
source_stl = "raw.stl"
"#;
        let pts = parse_centerline(text).unwrap();
        assert!(pts.is_empty());
    }

    #[test]
    fn centerline_arc_length_sums_segment_distances() {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        assert!((centerline_arc_length_m(&pts) - 3.0).abs() < 1e-12);
    }

    #[test]
    fn centerline_arc_length_zero_for_short_polylines() {
        assert_eq!(centerline_arc_length_m(&[]), 0.0);
        assert_eq!(centerline_arc_length_m(&[Point3::origin()]), 0.0);
    }

    #[test]
    fn build_scan_info_populates_all_fields() {
        let mesh = unit_cube_mesh();
        let centerline = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.05)];
        let info = build_scan_info(Path::new("/scans/test.cleaned.stl"), &mesh, &centerline);
        assert_eq!(info.file_label, "test.cleaned.stl");
        assert_eq!(info.vertex_count, 8);
        assert_eq!(info.face_count, 12);
        assert!((info.aabb_mm_extents[0] - 100.0).abs() < 1e-9);
        assert_eq!(info.centerline_point_count, 2);
        assert!((info.centerline_arc_length_m - 0.05).abs() < 1e-12);
    }

    #[test]
    fn resolve_prep_toml_strips_cleaned_suffix_from_stem() {
        let cli = Cli {
            cleaned_stl: PathBuf::from("/scans/sock.cleaned.stl"),
            design: None,
            prep_toml: None,
        };
        let p = resolve_prep_toml_path(&cli);
        assert_eq!(p, Some(PathBuf::from("/scans/sock.prep.toml")));
    }

    #[test]
    fn resolve_prep_toml_honors_explicit_flag() {
        let cli = Cli {
            cleaned_stl: PathBuf::from("/scans/sock.cleaned.stl"),
            design: None,
            prep_toml: Some(PathBuf::from("/elsewhere/custom.prep.toml")),
        };
        let p = resolve_prep_toml_path(&cli);
        assert_eq!(p, Some(PathBuf::from("/elsewhere/custom.prep.toml")));
    }

    #[test]
    fn resolve_design_toml_strips_cleaned_suffix_from_stem() {
        let cli = Cli {
            cleaned_stl: PathBuf::from("/scans/sock.cleaned.stl"),
            design: None,
            prep_toml: None,
        };
        let p = resolve_design_toml_path(&cli);
        assert_eq!(p, Some(PathBuf::from("/scans/sock.design.toml")));
    }

    #[test]
    fn resolve_design_toml_honors_explicit_flag() {
        let cli = Cli {
            cleaned_stl: PathBuf::from("/scans/sock.cleaned.stl"),
            design: Some(PathBuf::from("/elsewhere/d.design.toml")),
            prep_toml: None,
        };
        let p = resolve_design_toml_path(&cli);
        assert_eq!(p, Some(PathBuf::from("/elsewhere/d.design.toml")));
    }

    #[test]
    fn layer_display_label_resolves_known_anchor() {
        assert!(layer_display_label("ECOFLEX_00_30").starts_with("Ecoflex 00-30"));
    }

    #[test]
    fn layer_display_label_falls_back_to_anchor_key() {
        assert_eq!(layer_display_label("UNKNOWN_KEY"), "UNKNOWN_KEY");
    }

    /// Regression gate: the panel always has at least one layer to
    /// tint. If `LayersState::default_for_scan` ever ships an empty
    /// stack, the Phase-2 viewer would render a blank Layers section
    /// — and so would Phase 3, since `apply_design_toml` only writes
    /// over the defaults. Catches the regression at zero cost.
    #[test]
    fn default_layer_stack_is_non_empty() {
        assert!(!LayersState::default_for_scan().layers.is_empty());
    }
}
