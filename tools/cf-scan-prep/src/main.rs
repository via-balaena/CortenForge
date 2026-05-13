//! `cf-scan-prep` — GUI tool for preprocessing raw 3D scans into
//! watertight cast-frame STLs for the cf-cast mold pipeline.
//!
//! Stage 2.5 of the casting roadmap (`docs/CASTING_ROADMAP.md`); closes
//! the gap between the v1 capsule fallback example and a real
//! scan-driven cast example by producing a cleaned STL + provenance
//! TOML that the cast example (`examples/cast/layered-silicone-device-v1-scan/`,
//! commit #14) consumes. Full design at `docs/SCAN_PREP_DESIGN.md`.
//!
//! ## Scope through commit #4 (this file's current shape)
//!
//! - Positional CLI arg: path to the input scan STL.
//! - `--stl-units mm|m|inch` flag (default `mm`) — STL files don't carry
//!   unit metadata; cf-cast and the rest of the workspace work in meters,
//!   so loaded vertex coordinates are scaled into meters at load.
//! - STL load via [`mesh_io::load_stl`].
//! - Bevy window rendering the raw scan via cf-viewer's
//!   [`cf_viewer::setup_camera_and_lighting`] +
//!   [`cf_viewer::spawn_face_mesh`] helpers under the cast-frame
//!   [`UpAxis::PlusZ`] convention (`+Z` is the demolding direction).
//! - The orbit camera frames the raw mesh's AABB at load.
//! - Right-side egui sidebar with a `Scan Info` collapsing-header
//!   section (file basename + hover-full-path / vertex + face counts /
//!   `--stl-units` assumption / raw AABB in mm). Future commits add
//!   Simplify / Reorient / Recenter / Clip / Cap / Mouth / Cleaned AABB
//!   / Save as additional sections of the same `SidePanel`.
//! - Load failures surface as a red error-text overlay in the Bevy
//!   window; the app stays open so the user can read the message before
//!   pressing Esc to exit.
//!
//! Subsequent commits add the remaining egui panels, the cleaned-STL +
//! TOML writer, and the load-time mesh-repair diagnostics.

use std::path::{Path, PathBuf};

use anyhow::Result;
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use cf_bevy_common::prelude::*;
use cf_viewer::{
    RenderScale, compute_render_scale, scale_aabb, setup_camera_and_lighting, spawn_face_mesh,
};
use clap::{Parser, ValueEnum};
use mesh_io::load_stl;
use mesh_types::{Bounded, IndexedMesh};

/// Cast-frame up-axis convention: `+Z` is the demolding direction
/// (`docs/SCAN_PREP_DESIGN.md` §Architectural decisions §Tool home). All
/// future panels (Reorient / Recenter / Clip / Mouth ext.) work in this
/// frame; the user adjusts the scan-frame → cast-frame transform via the
/// Reorient sliders (commit #6).
const SCAN_UP_AXIS: UpAxis = UpAxis::PlusZ;

#[derive(Parser, Debug)]
#[command(
    name = "cf-scan-prep",
    about = "Preprocess a raw 3D scan STL into a watertight cast-frame STL for cf-cast",
    version
)]
struct Cli {
    /// Path to the input scan STL.
    path: PathBuf,

    /// Unit convention assumed for the input STL's vertex coordinates.
    /// STL files carry no unit metadata; industry default is millimeters.
    /// cf-cast operates in meters, so loaded vertices are multiplied by
    /// the conversion factor at load. Surfaces in the Scan Info panel
    /// (commit #4) so the assumption stays visible — loading a meter-
    /// scale STL with default `mm` produces a 1000× too-small mesh.
    #[arg(long, value_enum, default_value_t = StlUnits::Mm)]
    stl_units: StlUnits,
}

/// Unit convention for the input STL's vertex coordinates. The selected
/// variant's [`Self::to_meters_factor`] is multiplied into every vertex
/// at load to bring coordinates into the workspace's meter convention.
#[derive(ValueEnum, Clone, Copy, Debug, PartialEq, Eq)]
enum StlUnits {
    /// Millimeters — industry default for STL exports.
    Mm,
    /// Meters — identity; no unit scaling applied.
    M,
    /// Inches — NIST-exact conversion (`1 in ≡ 0.0254 m`).
    Inch,
}

impl StlUnits {
    /// Multiplier applied to each vertex coordinate to convert the
    /// loaded STL into meters.
    fn to_meters_factor(self) -> f64 {
        match self {
            Self::Mm => 0.001,
            Self::M => 1.0,
            Self::Inch => 0.0254,
        }
    }

    /// Human-readable label surfaced in the Scan Info panel's "STL units"
    /// row. Strings match the spec exactly (`docs/SCAN_PREP_DESIGN.md`
    /// §Panel specifications §1) so users see the load-time conversion
    /// factor and notice mismatches when the displayed AABB reads absurd
    /// (e.g. `0.08 mm` for a meter-scale STL with default `mm`).
    fn panel_label(self) -> &'static str {
        match self {
            Self::Mm => "mm (vertex × 0.001 → meters)",
            Self::M => "m (no scale)",
            Self::Inch => "inch (vertex × 0.0254 → meters)",
        }
    }
}

/// Bevy resource carrying the loaded scan mesh in meters (post unit
/// conversion). Owned by the render-mode `App`; absent under the
/// error-overlay path.
#[derive(Resource)]
struct ScanMesh(IndexedMesh);

/// Bevy resource carrying the load-failure message under the
/// error-overlay path. Mutually exclusive with [`ScanMesh`].
#[derive(Resource)]
struct LoadError(String);

/// Pre-computed display fields for the Scan Info panel
/// (`docs/SCAN_PREP_DESIGN.md` §Panel specifications §1). Built once at
/// startup from the loaded scan + CLI metadata; never changes (the panel
/// is read-only per spec).
///
/// AABB dimensions are stored in **millimeters regardless of
/// `--stl-units`** — workshop measurements are in mm; consistency with
/// the user's mental model. Computed from the post-unit-conversion meter
/// AABB by scaling by 1000.
#[derive(Resource, Debug, Clone)]
struct ScanInfo {
    /// Basename of the input STL (e.g., `"scan.stl"`). Falls back to the
    /// full display path if the input has no basename (e.g., the user
    /// supplied `"."`).
    file_label: String,
    /// Full display path of the input STL, surfaced as the `file_label`
    /// hover-text so paths longer than the panel width are still visible.
    file_full_path: String,
    vertex_count: usize,
    face_count: usize,
    /// `--stl-units` assumption string per [`StlUnits::panel_label`].
    stl_units_label: &'static str,
    aabb_width_mm: f64,
    aabb_depth_mm: f64,
    aabb_height_mm: f64,
}

impl ScanInfo {
    /// Build `ScanInfo` from the loaded scan + CLI metadata. The mesh
    /// must already be in meters (post [`scale_vertices_in_place`]);
    /// AABB extents are multiplied by 1000 to display in millimeters.
    fn from_loaded(path: &Path, scan: &IndexedMesh, units: StlUnits) -> Self {
        let file_label = path
            .file_name()
            .and_then(|n| n.to_str())
            .map_or_else(|| path.display().to_string(), str::to_string);
        let file_full_path = path.display().to_string();
        let aabb_m = scan.aabb();
        let aabb_width_mm = (aabb_m.max.x - aabb_m.min.x) * 1000.0;
        let aabb_depth_mm = (aabb_m.max.y - aabb_m.min.y) * 1000.0;
        let aabb_height_mm = (aabb_m.max.z - aabb_m.min.z) * 1000.0;
        Self {
            file_label,
            file_full_path,
            vertex_count: scan.vertices.len(),
            face_count: scan.faces.len(),
            stl_units_label: units.panel_label(),
            aabb_width_mm,
            aabb_depth_mm,
            aabb_height_mm,
        }
    }
}

/// Format an integer count with `k` / `M` suffixes for compact display
/// in the Scan Info panel. Mirrors the spec's wording (`"18.4k"`,
/// `"3.35M"`) which the commit #5 load-time auto-suggest banner reuses.
fn human_count(n: usize) -> String {
    // f64 keeps 53 bits of precision; usize → f64 is lossless for the
    // counts cf-scan-prep operates on (typical scans 10k-10M faces; the
    // 2^53 boundary is ~9×10^15).
    #[allow(clippy::cast_precision_loss)]
    if n >= 1_000_000 {
        format!("{:.2}M", n as f64 / 1_000_000.0)
    } else if n >= 1_000 {
        format!("{:.1}k", n as f64 / 1_000.0)
    } else {
        n.to_string()
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match try_load_scan(&cli) {
        Ok(scan_mesh) => {
            let scan_info = ScanInfo::from_loaded(&cli.path, &scan_mesh, cli.stl_units);
            run_render_app(scan_mesh, scan_info);
        }
        Err(err) => {
            let msg = format!("Failed to load {}: {err:#}", cli.path.display());
            eprintln!("{msg}");
            run_error_overlay_app(msg);
        }
    }
    Ok(())
}

/// Load the STL at `cli.path` and convert its vertex coordinates into
/// meters per `cli.stl_units`. Returns the loaded mesh on success.
///
/// # Errors
///
/// Bubbles up [`mesh_io::load_stl`]'s I/O + parse errors verbatim. The
/// caller wraps the message into the error-overlay path.
fn try_load_scan(cli: &Cli) -> Result<IndexedMesh> {
    let mut mesh = load_stl(&cli.path)?;
    scale_vertices_in_place(&mut mesh, cli.stl_units.to_meters_factor());
    Ok(mesh)
}

/// Multiply each vertex of `mesh` by `factor` in place. No-op when
/// `factor` is exactly `1.0` (skips the f64 vertex walk for the
/// `--stl-units m` identity case).
fn scale_vertices_in_place(mesh: &mut IndexedMesh, factor: f64) {
    if factor == 1.0 {
        return;
    }
    for v in &mut mesh.vertices {
        v.x *= factor;
        v.y *= factor;
        v.z *= factor;
    }
}

/// Run the render-mode Bevy app: spawn camera + lighting + the loaded
/// scan mesh, framed on its raw AABB; right-side egui sidebar shows the
/// Scan Info section. Returns when the window closes.
fn run_render_app(scan_mesh: IndexedMesh, scan_info: ScanInfo) {
    #[allow(clippy::cast_possible_truncation)] // f64 → f32 is intentional for Bevy.
    let raw_diagonal = scan_mesh.aabb().diagonal() as f32;
    let render_scale = compute_render_scale(raw_diagonal);
    println!(
        "loaded {} vertices, {} faces; bbox diagonal = {raw_diagonal:.4} m; \
         render_scale = {render_scale:.2}×",
        scan_mesh.vertices.len(),
        scan_mesh.faces.len(),
    );

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cf-scan-prep".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
        .insert_resource(SCAN_UP_AXIS)
        .insert_resource(RenderScale(render_scale))
        .insert_resource(ScanMesh(scan_mesh))
        .insert_resource(scan_info)
        .add_systems(Startup, setup_render_scene)
        .add_systems(Update, exit_on_esc)
        .add_systems(EguiPrimaryContextPass, scan_prep_panel)
        .run();
}

/// Run the error-overlay Bevy app: blank 3D scene + a red error text
/// node + a hint to press Esc. The app stays open until the user exits
/// so the load failure is visible (and recoverable: fix the path /
/// `--stl-units` and re-run).
fn run_error_overlay_app(err_msg: String) {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cf-scan-prep — load failed".into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
        .insert_resource(LoadError(err_msg))
        .add_systems(Startup, setup_error_overlay)
        .add_systems(Update, exit_on_esc)
        .run();
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
    let _entity = spawn_face_mesh(
        &mut commands,
        meshes.as_mut(),
        materials.as_mut(),
        &scan.0,
        None,
        *up,
        entity_transform,
    );
}

#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn setup_error_overlay(mut commands: Commands, err: Res<LoadError>) {
    // Camera3d is required so the UI render pass has a target; the scene
    // is intentionally empty (the overlay sits over a dark backdrop).
    commands.spawn(Camera3d::default());
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(24.0),
            left: Val::Px(24.0),
            max_width: Val::Vw(80.0),
            ..default()
        },
        Text::new(err.0.clone()),
        TextColor(Color::srgb(0.95, 0.30, 0.30)),
        TextFont {
            font_size: 18.0,
            ..default()
        },
    ));
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(24.0),
            left: Val::Px(24.0),
            ..default()
        },
        Text::new("press Esc to exit"),
        TextColor(Color::srgb(0.70, 0.70, 0.70)),
        TextFont {
            font_size: 14.0,
            ..default()
        },
    ));
}

#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn exit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}

/// Right-side egui sidebar carrying the scan-prep panels. At commit #4
/// this renders the single `Scan Info` section; future commits add
/// Simplify / Reorient / Recenter / Clip / Cap / Mouth / Cleaned AABB /
/// Save as additional collapsing sections inside the same `SidePanel`
/// per the spec's window-layout mockup
/// (`docs/SCAN_PREP_DESIGN.md` §Window layout).
// `Result` is shadowed by `use anyhow::Result` at the top of this file;
// use Bevy's prelude alias explicitly so the panel system's
// `EguiContexts::ctx_mut()?` short-circuit returns `BevyError` not
// `anyhow::Error`.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn scan_prep_panel(mut contexts: EguiContexts, info: Res<ScanInfo>) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("cf-scan-prep-panels")
        .resizable(false)
        .default_width(320.0)
        .show(ctx, |ui| {
            render_scan_info_section(ui, &info);
        });
    Ok(())
}

/// `Scan Info` section (`docs/SCAN_PREP_DESIGN.md` §Panel specifications
/// §1) — read-only, never changes after load. Five logical fields:
///
/// - File basename + hover-text full path.
/// - Vertex count (compact `human_count` format).
/// - Face count (compact `human_count` format).
/// - `--stl-units` assumption (surfaces the load-time conversion factor).
/// - Raw AABB header followed by three indented W / D / H rows in mm
///   (workshop convention regardless of CLI units).
fn render_scan_info_section(ui: &mut egui::Ui, info: &ScanInfo) {
    egui::CollapsingHeader::new("Scan Info")
        .default_open(true)
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("File:");
                ui.label(&info.file_label)
                    .on_hover_text(&info.file_full_path);
            });
            ui.label(format!("Vertices: {}", human_count(info.vertex_count)));
            ui.label(format!("Faces: {}", human_count(info.face_count)));
            ui.label(format!("STL units: {}", info.stl_units_label));
            ui.add_space(4.0);
            ui.label("Raw AABB:");
            ui.label(format!("  W: {:.1} mm", info.aabb_width_mm));
            ui.label(format!("  D: {:.1} mm", info.aabb_depth_mm));
            ui.label(format!("  H: {:.1} mm", info.aabb_height_mm));
        });
}

#[cfg(test)]
mod tests {
    use super::*;

    use mesh_types::Point3;

    // ----- StlUnits conversion factors -------------------------------

    #[test]
    fn stl_units_mm_returns_one_thousandth() {
        assert!((StlUnits::Mm.to_meters_factor() - 0.001).abs() < f64::EPSILON);
    }

    #[test]
    fn stl_units_m_is_identity() {
        assert_eq!(StlUnits::M.to_meters_factor(), 1.0);
    }

    /// `1 inch ≡ 0.0254 m` exactly (NIST conversion). Pinned bit-exact so
    /// future changes that drift the factor surface here, not in
    /// downstream cast geometry.
    #[test]
    fn stl_units_inch_uses_nist_exact_factor() {
        assert_eq!(StlUnits::Inch.to_meters_factor(), 0.0254);
    }

    // ----- scale_vertices_in_place -----------------------------------

    fn one_triangle_at(scale: f64) -> IndexedMesh {
        let mut mesh = IndexedMesh::with_capacity(3, 1);
        mesh.vertices.push(Point3::new(scale, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, scale, 0.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, scale));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    /// `factor = 1.0` is a no-op fast path — load with `--stl-units m`
    /// must leave vertices bit-identical (avoids subtle FP drift on
    /// re-saved meter-scale STLs).
    #[test]
    fn scale_vertices_identity_is_bit_exact_no_op() {
        let mut mesh = one_triangle_at(1.234_567_891_234_567);
        let original = mesh.vertices.clone();
        scale_vertices_in_place(&mut mesh, 1.0);
        assert_eq!(mesh.vertices, original);
    }

    /// `factor = 0.001` (the `mm → m` case) walks every vertex and
    /// scales each component. Asserts on the three components of the
    /// scaled triangle independently to catch axis-confusion bugs.
    #[test]
    fn scale_vertices_mm_to_meters_scales_each_component() {
        let mut mesh = one_triangle_at(80.0); // 80 mm
        scale_vertices_in_place(&mut mesh, 0.001);
        assert!((mesh.vertices[0].x - 0.080).abs() < 1e-12);
        assert!((mesh.vertices[1].y - 0.080).abs() < 1e-12);
        assert!((mesh.vertices[2].z - 0.080).abs() < 1e-12);
    }

    /// `factor = 0.0254` (the `inch → m` case) on a unit-inch triangle
    /// lands on the NIST-exact meter coordinate; no FP drift beyond f64
    /// representation of the constant itself.
    #[test]
    fn scale_vertices_inch_to_meters_uses_nist_factor() {
        let mut mesh = one_triangle_at(1.0); // 1 inch
        scale_vertices_in_place(&mut mesh, StlUnits::Inch.to_meters_factor());
        assert_eq!(mesh.vertices[0].x, 0.0254);
        assert_eq!(mesh.vertices[1].y, 0.0254);
        assert_eq!(mesh.vertices[2].z, 0.0254);
    }

    // ----- CLI parsing ------------------------------------------------

    /// Default unit is `Mm` per the workspace convention surfaced in
    /// the Scan Info panel (commit #4). Pinned because changing the
    /// default silently 1000×-shifts every cleaned STL.
    #[test]
    fn cli_default_stl_units_is_mm() -> Result<(), clap::Error> {
        let cli = Cli::try_parse_from(["cf-scan-prep", "scan.stl"])?;
        assert_eq!(cli.stl_units, StlUnits::Mm);
        assert_eq!(cli.path, PathBuf::from("scan.stl"));
        Ok(())
    }

    /// `--stl-units m` parses to the identity-factor variant.
    #[test]
    fn cli_stl_units_m_parses_to_meters_variant() -> Result<(), clap::Error> {
        let cli = Cli::try_parse_from(["cf-scan-prep", "scan.stl", "--stl-units", "m"])?;
        assert_eq!(cli.stl_units, StlUnits::M);
        Ok(())
    }

    /// `--stl-units inch` parses to the NIST-factor variant.
    #[test]
    fn cli_stl_units_inch_parses_to_inch_variant() -> Result<(), clap::Error> {
        let cli = Cli::try_parse_from(["cf-scan-prep", "scan.stl", "--stl-units", "inch"])?;
        assert_eq!(cli.stl_units, StlUnits::Inch);
        Ok(())
    }

    // ----- StlUnits::panel_label spec-string pins --------------------

    /// Spec-exact string for `mm` units. Changing this drift-checks the
    /// Scan Info panel against the spec mockup (`docs/SCAN_PREP_DESIGN.md`
    /// §Panel specifications §1).
    #[test]
    fn stl_units_mm_panel_label_matches_spec() {
        assert_eq!(StlUnits::Mm.panel_label(), "mm (vertex × 0.001 → meters)");
    }

    #[test]
    fn stl_units_m_panel_label_matches_spec() {
        assert_eq!(StlUnits::M.panel_label(), "m (no scale)");
    }

    #[test]
    fn stl_units_inch_panel_label_matches_spec() {
        assert_eq!(
            StlUnits::Inch.panel_label(),
            "inch (vertex × 0.0254 → meters)"
        );
    }

    // ----- human_count ------------------------------------------------

    /// Small counts pass through as bare integers (no suffix). The
    /// `1`/`999` boundary cases pin the `< 1000` threshold.
    #[test]
    fn human_count_small_returns_bare_integer() {
        assert_eq!(human_count(0), "0");
        assert_eq!(human_count(1), "1");
        assert_eq!(human_count(999), "999");
    }

    /// Thousands use `k` suffix with one decimal. `18_432` matches the
    /// spec mockup's `"18.4k"` example exactly.
    #[test]
    fn human_count_thousands_use_k_suffix() {
        assert_eq!(human_count(1_000), "1.0k");
        assert_eq!(human_count(18_432), "18.4k");
    }

    /// Millions use `M` suffix with two decimals. `3_352_068` matches
    /// the iter-1 fixture's face count (`sock_over_capsule.stl`, 3.35M
    /// faces) — banked in MEMORY.md's Resume-here block as the spec's
    /// canonical perf-calibration value.
    #[test]
    fn human_count_millions_use_m_suffix() {
        assert_eq!(human_count(1_000_000), "1.00M");
        assert_eq!(human_count(3_352_068), "3.35M");
    }

    // ----- ScanInfo::from_loaded -------------------------------------

    /// `from_loaded` populates every field: basename + full path,
    /// vertex/face counts, units label, and mm-scaled AABB extents. The
    /// mesh is built in meters (post unit conversion) and the expected
    /// mm extents are `meters × 1000`.
    #[test]
    fn scan_info_from_loaded_populates_all_fields() {
        // Triangle in meters; 80mm extent = 0.080 m.
        let mesh = one_triangle_at(0.080);
        let info = ScanInfo::from_loaded(Path::new("/tmp/test_scan.stl"), &mesh, StlUnits::Mm);
        assert_eq!(info.file_label, "test_scan.stl");
        assert_eq!(info.file_full_path, "/tmp/test_scan.stl");
        assert_eq!(info.vertex_count, 3);
        assert_eq!(info.face_count, 1);
        assert_eq!(info.stl_units_label, "mm (vertex × 0.001 → meters)");
        assert!((info.aabb_width_mm - 80.0).abs() < 1e-9);
        assert!((info.aabb_depth_mm - 80.0).abs() < 1e-9);
        assert!((info.aabb_height_mm - 80.0).abs() < 1e-9);
    }
}
