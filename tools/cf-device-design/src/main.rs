//! cf-device-design — layered-silicone-device design + testing suite
//! (`docs/ENGINEERING_SUITE_DESIGN.md`).
//!
//! Slice 2 (this commit): crate scaffold + STL/prep load + centerline
//! overlay + Scan Info panel only. Later slices wire the Outer
//! Envelope / Cavity / Layers / Features / Validations / Health / Save
//! panels.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use cf_bevy_common::prelude::*;
use cf_viewer::{
    RenderScale, compute_render_scale, scale_aabb, setup_camera_and_lighting, spawn_face_mesh,
};
use clap::Parser;
use mesh_io::load_stl;
use mesh_types::{Bounded, IndexedMesh, Point3};
use serde::Deserialize;

/// Cast-frame demolding-axis convention: `+Z` is up. Inherited from
/// cf-scan-prep + cf-cast — every CortenForge cast tool assumes the
/// `UpAxis::PlusZ` swap from physics frame to Bevy frame.
const DEVICE_UP_AXIS: UpAxis = UpAxis::PlusZ;

#[derive(Parser, Debug)]
#[command(
    name = "cf-device-design",
    about = "Layered-silicone-device design + testing suite (composes cleaned scan + outer envelope + cavity + layers + features into a device design TOML)",
    version
)]
struct Cli {
    /// Path to the cleaned scan STL (cf-scan-prep output;
    /// `<stem>.cleaned.stl`).
    cleaned_stl: PathBuf,

    /// Optional path to a previously-saved design TOML to reopen +
    /// iterate on. When supplied, the suite pre-populates panels from
    /// the file; absent, panels start at defaults. Wired in slice 8;
    /// at this slice the flag is parsed but only the cleaned STL is
    /// honored.
    #[arg(long, value_name = "PATH")]
    design: Option<PathBuf>,

    /// Optional path to the cf-scan-prep `.prep.toml` companion file.
    /// Defaults to `<cleaned_stl_stem>.prep.toml` next to the cleaned
    /// STL. The `[centerline].points_m` block is required for the
    /// curve-following outer envelope + cavity construction in later
    /// slices; absent for slice 2, the centerline overlay simply
    /// doesn't render.
    #[arg(long, value_name = "PATH")]
    prep_toml: Option<PathBuf>,
}

/// Bevy resource carrying the loaded cleaned scan in physics-frame
/// meters. Mirror of cf-scan-prep's `ScanMesh` (same posture).
#[derive(Resource)]
struct ScanMesh(IndexedMesh);

/// Bevy resource carrying scan-info readouts surfaced in the Scan
/// Info panel. Computed once at startup; immutable post-construction.
#[derive(Resource, Debug, Clone)]
struct ScanInfo {
    file_label: String,
    vertex_count: usize,
    face_count: usize,
    /// Raw AABB extents in millimeters (workshop convention).
    aabb_mm_extents: [f64; 3],
    /// Bbox diagonal in meters — used by the Outer Envelope panel
    /// (slice 3) to size the default radius slider's range.
    bbox_diagonal_m: f64,
    /// Centerline polyline length in physics meters (sum of
    /// segment distances). Zero if the centerline is absent or
    /// empty. Used by the Cavity panel (slice 4) to bound the
    /// insertable-length slider.
    centerline_arc_length_m: f64,
    /// Centerline point count (display-only). Zero if the
    /// centerline is absent.
    centerline_point_count: usize,
}

/// Bevy resource carrying the cf-scan-prep centerline polyline (in
/// post-bake physics-frame meters — matches the cleaned STL's
/// coordinate system). Empty when no `.prep.toml` is present or its
/// `[centerline]` block is absent.
#[derive(Resource, Default, Clone)]
struct Centerline {
    points_m: Vec<Point3<f64>>,
}

// ----- .prep.toml parsing (subset; centerline only) -----------------

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
/// empty — caller treats both as "no centerline available," same
/// posture as cf-cast-cli's prep parser.
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
/// .prep.toml` next to the cleaned STL. Returns `None` if no
/// inferred path exists (extremely rare: bare filename with no
/// parent).
fn resolve_prep_toml_path(cli: &Cli) -> Option<PathBuf> {
    if let Some(p) = &cli.prep_toml {
        return Some(p.clone());
    }
    // cf-scan-prep writes `<stem>.prep.toml` alongside
    // `<stem>.cleaned.stl`. Strip `.cleaned` from the cleaned STL's
    // stem to recover the original stem.
    let parent = cli.cleaned_stl.parent()?;
    let stem = cli.cleaned_stl.file_stem()?.to_str()?;
    let base_stem = stem.strip_suffix(".cleaned").unwrap_or(stem);
    Some(parent.join(format!("{base_stem}.prep.toml")))
}

/// Compute the centerline polyline's total arc length in meters
/// (sum of consecutive segment distances). Zero for a polyline with
/// fewer than 2 points.
fn centerline_arc_length_m(points: &[Point3<f64>]) -> f64 {
    points
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).norm())
        .sum()
}

// ----- Bevy app -----------------------------------------------------

#[derive(Component)]
struct ScanMeshEntity;

fn main() -> Result<()> {
    let cli = Cli::parse();
    let scan_mesh = load_stl(&cli.cleaned_stl)
        .with_context(|| format!("load cleaned STL {}", cli.cleaned_stl.display()))?;

    let prep_toml_path = resolve_prep_toml_path(&cli);
    let centerline_points = if let Some(prep_path) = &prep_toml_path {
        match std::fs::read_to_string(prep_path) {
            Ok(text) => parse_centerline(&text).with_context(|| {
                format!("parse `.prep.toml` centerline at {}", prep_path.display())
            })?,
            Err(e) => {
                eprintln!(
                    "warning: could not read {} ({e:#}); centerline overlay disabled",
                    prep_path.display()
                );
                Vec::new()
            }
        }
    } else {
        Vec::new()
    };

    let scan_info = build_scan_info(&cli.cleaned_stl, &scan_mesh, &centerline_points);
    println!(
        "loaded {} vertices, {} faces; bbox diagonal = {:.4} m; \
         centerline = {} points / {:.4} m arc",
        scan_info.vertex_count,
        scan_info.face_count,
        scan_info.bbox_diagonal_m,
        scan_info.centerline_point_count,
        scan_info.centerline_arc_length_m,
    );

    run_render_app(scan_mesh, scan_info, centerline_points);
    Ok(())
}

fn build_scan_info(path: &Path, mesh: &IndexedMesh, centerline_points: &[Point3<f64>]) -> ScanInfo {
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

#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
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
    let entity = spawn_face_mesh(
        &mut commands,
        meshes.as_mut(),
        materials.as_mut(),
        &scan.0,
        None,
        *up,
        entity_transform,
    );
    commands.entity(entity).insert(ScanMeshEntity);
}

/// Draw the always-on viewport reference overlays each frame:
///
/// 1. **Three colored axis arrows** at world origin (cast-frame
///    convention: X = red, Y = green, Z = blue). Sized to the
///    rendered scan's diagonal.
/// 2. **Centerline polyline overlay** (cyan), if a centerline was
///    parsed from `.prep.toml`. Each polyline point is projected
///    through the same `UpAxis::PlusZ` swap + `render_scale` lift
///    the scan mesh entity uses, so the centerline tracks the mesh
///    exactly.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn draw_reference_overlays(
    centerline: Res<Centerline>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    scan: Res<ScanMesh>,
    mut gizmos: Gizmos,
) {
    #[allow(clippy::cast_possible_truncation)] // f64 -> f32 for Bevy gizmo length.
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

/// Block orbit-camera input when the pointer is over the egui side
/// panel — prevents the sidebar from accidentally rotating the
/// camera when the user scrolls a slider list. Mirror of
/// cf-scan-prep's matching system.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
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

#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn exit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}

/// Right-side egui sidebar carrying the design-suite panels. Slice 2
/// renders only the Scan Info section; subsequent slices add Outer
/// Envelope, Cavity, Layers, Validations, Health, Save / Open.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn device_design_panel(
    mut contexts: EguiContexts,
    info: Res<ScanInfo>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("cf-device-design-panels")
        .resizable(false)
        .default_width(320.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    render_scan_info_section(ui, &info);
                    render_panel_stubs(ui);
                });
        });
    Ok(())
}

fn render_scan_info_section(ui: &mut egui::Ui, info: &ScanInfo) {
    egui::CollapsingHeader::new("Scan Info")
        .default_open(true)
        .show(ui, |ui| {
            ui.label(format!("File:  {}", info.file_label));
            ui.label(format!("Vertices: {}", human_count(info.vertex_count)));
            ui.label(format!("Faces: {}", human_count(info.face_count)));
            ui.label("Raw AABB:");
            ui.label(format!("  W: {:.1} mm", info.aabb_mm_extents[0]));
            ui.label(format!("  D: {:.1} mm", info.aabb_mm_extents[1]));
            ui.label(format!("  H: {:.1} mm", info.aabb_mm_extents[2]));
            ui.label(format!("Diagonal: {:.1} mm", info.bbox_diagonal_m * 1000.0));
            ui.separator();
            if info.centerline_point_count == 0 {
                ui.label("Centerline: not available");
                ui.label("  Reopen the scan in cf-scan-prep,");
                ui.label("  apply [Cap] + centerline,");
                ui.label("  and re-save to enable.");
            } else {
                ui.label(format!(
                    "Centerline: {} points",
                    info.centerline_point_count
                ));
                ui.label(format!(
                    "  arc length: {:.1} mm",
                    info.centerline_arc_length_m * 1000.0
                ));
            }
        });
}

/// Stub-section placeholders for panels arriving in later slices. UI
/// only — no state to surface yet. Pre-populating with the future
/// panel names so the user can see the planned layout at slice-2
/// landing time.
fn render_panel_stubs(ui: &mut egui::Ui) {
    ui.separator();
    for name in [
        "Outer Envelope (slice 3)",
        "Cavity (slice 4)",
        "Layers (slice 5)",
        "Validations (slice 6)",
        "Features → Texture (slice 7)",
        "Save / Open (slice 8)",
    ] {
        ui.add_enabled(false, egui::Label::new(name));
    }
}

/// Truncate large counts to k / M suffixes for compact panel
/// display. Matches cf-scan-prep's formatter.
fn human_count(n: usize) -> String {
    if n >= 1_000_000 {
        #[allow(clippy::cast_precision_loss)]
        let m = n as f64 / 1_000_000.0;
        format!("{m:.2}M")
    } else if n >= 1_000 {
        #[allow(clippy::cast_precision_loss)]
        let k = n as f64 / 1_000.0;
        format!("{k:.1}k")
    } else {
        n.to_string()
    }
}

fn run_render_app(
    scan_mesh: IndexedMesh,
    scan_info: ScanInfo,
    centerline_points: Vec<Point3<f64>>,
) {
    #[allow(clippy::cast_possible_truncation)] // f64 → f32 is intentional for Bevy.
    let raw_diagonal = scan_mesh.aabb().diagonal() as f32;
    let render_scale = compute_render_scale(raw_diagonal);

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cf-device-design".into(),
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
        .insert_resource(scan_info)
        .insert_resource(Centerline {
            points_m: centerline_points,
        })
        .add_systems(Startup, setup_render_scene)
        .add_systems(
            Update,
            (
                block_orbit_input_when_over_egui.before(orbit_camera_input),
                draw_reference_overlays,
                exit_on_esc,
            ),
        )
        .add_systems(EguiPrimaryContextPass, device_design_panel)
        .run();
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    // `unwrap()` + `expect()` are denied at the crate level for
    // production safety; allow them inside tests so assertions can
    // pull values out of `Option` / `Result` returns without
    // multi-line `match` ceremony.
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

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
[scan_prep]
source_stl = "raw.stl"
tool_version = "0.0.0-test"

[centerline]
points_m = [
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 0.1],
    [0.0, 0.0, 0.2],
]
algorithm = "cross_section_centroids"
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
    fn parse_centerline_extra_fields_tolerated() {
        // Forward-compatibility: schema additions in cf-scan-prep
        // don't break cf-device-design's loader.
        let text = r#"
[scan_prep]
source_stl = "raw.stl"
future_field = 42

[centerline]
points_m = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]
algorithm = "cross_section_centroids"
another_future_field = "foo"
"#;
        let pts = parse_centerline(text).unwrap();
        assert_eq!(pts.len(), 2);
    }

    #[test]
    fn centerline_arc_length_sums_segment_distances() {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        let arc = centerline_arc_length_m(&pts);
        assert!((arc - 3.0).abs() < 1e-12, "arc = {arc}");
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
    fn resolve_prep_toml_handles_stem_without_cleaned_suffix() {
        // Defensive — user supplies a non-cleaned STL; we still try
        // the `<stem>.prep.toml` sibling.
        let cli = Cli {
            cleaned_stl: PathBuf::from("/scans/raw.stl"),
            design: None,
            prep_toml: None,
        };
        let p = resolve_prep_toml_path(&cli);
        assert_eq!(p, Some(PathBuf::from("/scans/raw.prep.toml")));
    }

    #[test]
    fn human_count_formats_orders_of_magnitude() {
        assert_eq!(human_count(0), "0");
        assert_eq!(human_count(500), "500");
        assert_eq!(human_count(1_500), "1.5k");
        assert_eq!(human_count(200_000), "200.0k");
        assert_eq!(human_count(3_350_000), "3.35M");
    }

    #[test]
    fn device_up_axis_is_plus_z() {
        // Pinned: every cast tool in CortenForge uses +Z up. Drifting
        // away would mismatch cf-scan-prep and cf-cast.
        assert_eq!(DEVICE_UP_AXIS, UpAxis::PlusZ);
    }

    /// `Aabb` re-export from `mesh_types` (transitive from
    /// `cf_geometry`) is the canonical aabb type the suite consumes
    /// for scan AABB readouts + outer-envelope sizing in slice 3.
    /// Pin compile-time visibility so a future workspace dep shuffle
    /// doesn't silently break the loader.
    #[test]
    fn aabb_canonical_type_in_scope() {
        use mesh_types::Aabb;
        let a = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        assert!((a.diagonal() - (3.0_f64).sqrt()).abs() < 1e-12);
    }
}
