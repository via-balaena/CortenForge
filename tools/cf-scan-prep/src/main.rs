//! `cf-scan-prep` — GUI tool for preprocessing raw 3D scans into
//! watertight cast-frame STLs for the cf-cast mold pipeline.
//!
//! Stage 2.5 of the casting roadmap (`docs/CASTING_ROADMAP.md`); closes
//! the gap between the v1 capsule fallback example and a real
//! scan-driven cast example by producing a cleaned STL + provenance
//! TOML that the cast example (`examples/cast/layered-silicone-device-v1-scan/`,
//! commit #14) consumes. Full design at `docs/SCAN_PREP_DESIGN.md`.
//!
//! ## Architecture
//!
//! - Positional CLI arg: path to the input scan STL.
//! - `--stl-units mm|m|inch` flag (default `mm`) — STL files don't carry
//!   unit metadata; cf-cast and the rest of the workspace work in meters,
//!   so loaded vertex coordinates are scaled into meters at load.
//! - STL load via [`mesh_io::load_stl`].
//! - Bevy window rendering the loaded scan via cf-viewer's
//!   [`cf_viewer::setup_camera_and_lighting`] +
//!   [`cf_viewer::spawn_face_mesh`] helpers under the cast-frame
//!   [`UpAxis::PlusZ`] convention (`+Z` is the demolding direction).
//!   The orbit camera frames the raw mesh's AABB at load.
//! - Right-side egui sidebar with collapsing sections — one per spec'd
//!   panel. The set of currently-implemented sections is defined by
//!   the `render_*_section` functions in this file (grep `^fn render_`
//!   for the live list); the spec's complete panel set is at
//!   `docs/SCAN_PREP_DESIGN.md` §Window layout.
//! - Bottom egui status bar surfaces the load-time auto-suggest banner
//!   (when face count exceeds [`AUTO_SUGGEST_FACE_THRESHOLD`]) +
//!   apply/reset achievement messages with [`STATUS_NORMAL_TTL_SECS`]
//!   TTL. Renders nothing when empty so it doesn't claim screen space.
//! - Load failures surface as a red error-text overlay in the Bevy
//!   window; the app stays open so the user can read the message
//!   before pressing Esc to exit.
//!
//! Per-commit additions are tracked in the spec's slice-ship-log
//! (`docs/SCAN_PREP_DESIGN.md` §Slice ship log).

use std::path::{Path, PathBuf};
use std::time::Instant;

use anyhow::Result;
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use cf_bevy_common::prelude::*;
use cf_viewer::{
    RenderScale, compute_render_scale, scale_aabb, setup_camera_and_lighting, spawn_face_mesh,
};
use clap::{Parser, ValueEnum};
use mesh_io::load_stl;
use mesh_repair::{remove_unreferenced_vertices, weld_vertices};
use mesh_types::{Bounded, IndexedMesh};
use meshopt::{SimplifyOptions, simplify_decoder};

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
    /// so the assumption stays visible — loading a meter-scale STL with
    /// default `mm` produces a 1000× too-small mesh.
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
    ///
    /// The arrow uses ASCII `->` rather than Unicode `→` (U+2192)
    /// because egui's default `ProggyClean` font ships with Latin-1 +
    /// basic punctuation coverage only; U+2192 falls outside the glyph
    /// set and renders as `□` (missing-glyph box). `×` (U+00D7) is in
    /// Latin-1 Supplement and renders fine, so it stays.
    fn panel_label(self) -> &'static str {
        match self {
            Self::Mm => "mm (vertex × 0.001 -> meters)",
            Self::M => "m (no scale)",
            Self::Inch => "inch (vertex × 0.0254 -> meters)",
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

/// Bevy resource carrying a clone of the originally loaded scan mesh
/// (in meters, post unit conversion). Used by the Simplify panel's
/// `[Reset to original]` action (`docs/SCAN_PREP_DESIGN.md` §Panel
/// specifications §2) to restore the unsimplified geometry without
/// re-loading from disk. ~240 MB on the iter-1 fixture
/// (`sock_over_capsule.stl`, 10M vertices); accepted in exchange for
/// instant reset vs. a ~3-5 s STL reload.
#[derive(Resource)]
struct OriginalScanMesh(IndexedMesh);

/// Marker for the Bevy entity rendering the current scan mesh. Used by
/// the Simplify Apply/Reset handler to despawn the existing entity
/// before respawning from the new mesh — without a marker the handler
/// would have to track the spawned `Entity` id through Bevy state.
#[derive(Component)]
struct ScanMeshEntity;

/// Simplify panel state (`docs/SCAN_PREP_DESIGN.md` §Panel
/// specifications §2). The slider mutates `target_face_count`
/// continuously; clicking `[Apply simplify]` / `[Reset to original]`
/// sets `pending_action` to the queued command which
/// [`handle_simplify_actions`] consumes on the next Update tick.
#[derive(Resource, Debug, Clone)]
struct SimplifyState {
    /// Current slider value, clamped to
    /// `[SIMPLIFY_TARGET_MIN, SIMPLIFY_TARGET_MAX]` by the egui slider.
    /// Default 200_000 per spec §Architectural decisions §Simplify
    /// algorithm.
    target_face_count: usize,
    /// Queued user action; consumed (cleared) by
    /// [`handle_simplify_actions`] each Update tick.
    pending_action: Option<SimplifyAction>,
}

impl Default for SimplifyState {
    fn default() -> Self {
        Self {
            target_face_count: SIMPLIFY_TARGET_DEFAULT,
            pending_action: None,
        }
    }
}

/// User action queued from the Simplify panel buttons. Consumed by
/// [`handle_simplify_actions`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SimplifyAction {
    /// Run `meshopt::simplify` on the current scan mesh down to
    /// `SimplifyState::target_face_count`. Despawns + respawns the
    /// Bevy mesh entity; updates `ScanInfo` vertex/face counts; sets a
    /// status-bar achievement message.
    Apply,
    /// Restore the originally loaded scan from [`OriginalScanMesh`].
    /// Mirrors the Apply pipeline but skips the meshopt call.
    Reset,
}

/// Default target face count for the Simplify panel slider per spec
/// §Architectural decisions §Simplify algorithm: 10× over the 50k
/// surface-continuity floor, well below the 500k "may be sluggish"
/// threshold, matches cf-cast's 2 mm-cell SDF sampling density
/// × surface-continuity headroom.
const SIMPLIFY_TARGET_DEFAULT: usize = 200_000;

/// Lower slider bound. Below this the simplified mesh loses too much
/// surface continuity for cast purposes.
const SIMPLIFY_TARGET_MIN: usize = 1_000;

/// Upper slider bound. Above this we're not really simplifying typical
/// scans anymore.
const SIMPLIFY_TARGET_MAX: usize = 1_000_000;

/// Face-count threshold above which the load-time status bar surfaces
/// the auto-suggest banner. Per spec §Panel specifications §2.
const AUTO_SUGGEST_FACE_THRESHOLD: usize = 500_000;

/// Spatial-hash welding tolerance applied pre-`meshopt::simplify` so
/// the STL's 3N unshared vertices collapse to ~N shared. 1 µm in
/// meters; tighter than any practical scan precision.
const SIMPLIFY_WELD_EPSILON_M: f64 = 1e-6;

/// `target_error` parameter passed to `meshopt::simplify_decoder`.
/// Relative to mesh extents (no `ErrorAbsolute` flag), so 0.01 means
/// "tolerate up to ~1 % of mesh diagonal of geometric error during the
/// quadric edge collapse." Chosen as a balance: tight enough to
/// preserve scan surface detail in the simplified output, loose enough
/// that meshopt doesn't early-exit before reaching the requested face
/// count. Re-tune if iter-1 eyes-on-pixels surfaces noticeable surface
/// artifacts at typical target counts.
const SIMPLIFY_TARGET_ERROR: f32 = 0.01;

/// Default TTL for `Normal`-kind status messages (apply-result,
/// reset-result). Persistent messages (auto-suggest banner) set
/// `auto_clear_at_secs` to `None` explicitly.
const STATUS_NORMAL_TTL_SECS: f64 = 8.0;

/// Status-bar state. Empty `text` means the bottom panel doesn't
/// render. Auto-suggest banner at load is `Warning` kind + persistent
/// (`auto_clear_at_secs = None`). Apply / Reset achievement messages
/// are `Normal` kind with a `STATUS_NORMAL_TTL_SECS` timer.
#[derive(Resource, Default, Debug, Clone)]
struct StatusBar {
    text: String,
    kind: StatusKind,
    /// Absolute time (seconds since `Time::elapsed_secs_f64()`) at
    /// which [`auto_clear_status`] clears `text`. `None` = persistent
    /// until the next status update overwrites it.
    auto_clear_at_secs: Option<f64>,
}

/// Severity tag for [`StatusBar`] used by the bottom-panel renderer to
/// pick a text color (gray for `Normal`, yellow for `Warning`). An
/// `Error` variant (red) lands at commit #12 alongside the Save panel's
/// FS-error pattern + non-finite-transform rejection — kept off the
/// enum here so the rendering match doesn't pretend to dispatch on a
/// case it can't reach.
#[derive(Default, Clone, Copy, Debug, PartialEq, Eq)]
enum StatusKind {
    /// Neutral information (achievement messages, restore-confirmed).
    #[default]
    Normal,
    /// Advisory but non-blocking (auto-suggest banner, soft caps).
    Warning,
}

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
/// in the Scan Info panel + Simplify panel + load-time auto-suggest
/// banner. Mirrors the spec's wording (`"18.4k"`, `"3.35M"`).
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
            // Clone the loaded mesh into `OriginalScanMesh` so the
            // Simplify panel's `[Reset to original]` action can restore
            // unsimplified geometry without re-reading the STL.
            let original = OriginalScanMesh(scan_mesh.clone());
            run_render_app(scan_mesh, original, scan_info);
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

/// Result of [`simplify_mesh`]: the decimated mesh + wall-clock elapsed
/// for the status-bar achievement message.
struct SimplifyResult {
    mesh: IndexedMesh,
    elapsed_secs: f64,
}

/// Run boundary-preserving quadric edge collapse decimation on `original`
/// down to (approximately) `target_face_count` faces. Returns the
/// simplified mesh + wall-clock elapsed time.
///
/// Pipeline:
///
/// 1. **Weld vertices** ([`mesh_repair::weld_vertices`] with epsilon
///    `SIMPLIFY_WELD_EPSILON_M`). STL load produces 3N unshared
///    vertices (one set per triangle); meshopt operates on indexed
///    buffers and needs shared vertex indices across adjacent triangles
///    to find collapsible edges. Without this step `meshopt::simplify`
///    would see every triangle as topologically disconnected and would
///    refuse to decimate.
/// 2. **Convert positions to `[f32; 3]`** for meshopt's
///    `DecodePosition` impl. f64 → f32 loses ~16 ulps of precision at
///    the scan's mm scale; below the 1 µm weld tolerance.
/// 3. **`meshopt::simplify_decoder`** with
///    `SimplifyOptions::LockBorder`. `target_count` is in **indices**
///    (`target_face_count × 3`); the C-side `meshopt_simplify` takes
///    `target_index_count`. `LockBorder` pins open-boundary-loop
///    vertices in place so the Cap panel (commit #9) sees the same
///    boundary topology after simplification (the spec's load-bearing
///    boundary-preservation requirement).
/// 4. **Reassemble `IndexedMesh`** + strip unreferenced vertices left
///    over from collapse so the simplified mesh has tight memory shape.
///
/// The output mesh shares its vertex coordinates with the welded
/// intermediate (no further conversion); faces reference the surviving
/// vertex indices.
fn simplify_mesh(original: &IndexedMesh, target_face_count: usize) -> SimplifyResult {
    let start = Instant::now();

    // Step 1: weld unshared vertices into shared indices.
    let mut welded = original.clone();
    weld_vertices(&mut welded, SIMPLIFY_WELD_EPSILON_M);

    // Step 2: convert positions to meshopt-friendly f32 triples.
    // f64 → f32 cast is intentional; meshopt's C API operates on f32.
    #[allow(clippy::cast_possible_truncation)]
    let positions: Vec<[f32; 3]> = welded
        .vertices
        .iter()
        .map(|p| [p.x as f32, p.y as f32, p.z as f32])
        .collect();
    let indices: Vec<u32> = welded.faces.iter().flatten().copied().collect();

    // Step 3: run meshopt. `target_count` is in INDICES, not faces.
    let target_index_count = target_face_count.saturating_mul(3);
    let mut result_error: f32 = 0.0;
    let simplified_indices = simplify_decoder(
        &indices,
        &positions,
        target_index_count,
        SIMPLIFY_TARGET_ERROR,
        SimplifyOptions::LockBorder,
        Some(&mut result_error),
    );

    // Step 4: reassemble. Indices reference the welded vertex buffer.
    let achieved_face_count = simplified_indices.len() / 3;
    let mut simplified_faces: Vec<[u32; 3]> = Vec::with_capacity(achieved_face_count);
    for tri in simplified_indices.chunks_exact(3) {
        simplified_faces.push([tri[0], tri[1], tri[2]]);
    }
    let mut simplified_mesh = welded;
    simplified_mesh.faces = simplified_faces;
    remove_unreferenced_vertices(&mut simplified_mesh);

    SimplifyResult {
        mesh: simplified_mesh,
        elapsed_secs: start.elapsed().as_secs_f64(),
    }
}

/// Run the render-mode Bevy app: spawn camera + lighting + the loaded
/// scan mesh, framed on its raw AABB; right-side egui sidebar shows the
/// Scan Info + Simplify sections; bottom status bar surfaces the
/// auto-suggest banner at load + apply/reset achievement messages.
/// Returns when the window closes.
fn run_render_app(scan_mesh: IndexedMesh, original: OriginalScanMesh, scan_info: ScanInfo) {
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
        .insert_resource(original)
        .insert_resource(scan_info)
        .insert_resource(SimplifyState::default())
        .insert_resource(StatusBar::default())
        .add_systems(Startup, (setup_render_scene, init_status_for_load))
        // Apply/Reset handler runs before auto-clear so a newly-set
        // status's TTL is checked against the same tick's `Time`. Both
        // are idempotent (no-op when nothing's pending / nothing's
        // expired), so order doesn't change the steady-state behavior.
        .add_systems(
            Update,
            (handle_simplify_actions, auto_clear_status, exit_on_esc),
        )
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
    let entity = spawn_face_mesh(
        &mut commands,
        meshes.as_mut(),
        materials.as_mut(),
        &scan.0,
        None,
        *up,
        entity_transform,
    );
    // Marker so the Simplify Apply/Reset handler can despawn the
    // current scan render before respawning from the new mesh.
    commands.entity(entity).insert(ScanMeshEntity);
}

/// Startup system: surface the auto-suggest banner if the loaded scan
/// exceeds `AUTO_SUGGEST_FACE_THRESHOLD` faces. The banner is
/// `Warning`-kind + persistent (`auto_clear_at_secs = None`) so it
/// stays visible until the user runs `[Apply simplify]` or
/// `[Reset to original]`, both of which overwrite the status text. No-op
/// for already-small scans.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn init_status_for_load(scan: Res<ScanMesh>, mut status: ResMut<StatusBar>) {
    let face_count = scan.0.faces.len();
    if face_count <= AUTO_SUGGEST_FACE_THRESHOLD {
        return;
    }
    status.text = format!(
        "Scan has {} faces. Recommended: simplify to ~{}k for performance. \
         Open the Simplify panel to decimate.",
        human_count(face_count),
        SIMPLIFY_TARGET_DEFAULT / 1_000,
    );
    status.kind = StatusKind::Warning;
    status.auto_clear_at_secs = None;
}

/// Update system: consume `SimplifyState::pending_action` (if any) by
/// running the Simplify pipeline or restoring from `OriginalScanMesh`.
/// Idempotent — does nothing when no action is queued, so it can run
/// every tick safely.
///
/// On Apply: clones the current scan, runs [`simplify_mesh`], updates
/// `ScanMesh` + `ScanInfo` vertex/face counts, despawns the existing
/// `ScanMeshEntity` and respawns from the simplified mesh, sets a
/// `Normal`-kind status message with TTL `STATUS_NORMAL_TTL_SECS`
/// per spec §Panel specifications §2.
///
/// On Reset: clones `OriginalScanMesh`, updates `ScanMesh` + `ScanInfo`,
/// respawns the entity, sets a confirmation status message.
// Bevy systems take resources by value.
#[allow(clippy::needless_pass_by_value)]
// Bevy systems pull each Res / ResMut / Query / Commands as a separate
// parameter; threading through a SystemParam-derive tuple costs more
// boilerplate than the +N buys in readability.
#[allow(clippy::too_many_arguments)]
fn handle_simplify_actions(
    mut simplify_state: ResMut<SimplifyState>,
    mut scan: ResMut<ScanMesh>,
    original: Res<OriginalScanMesh>,
    mut info: ResMut<ScanInfo>,
    mut status: ResMut<StatusBar>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    time: Res<Time>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    existing: Query<Entity, With<ScanMeshEntity>>,
) {
    let Some(action) = simplify_state.pending_action.take() else {
        return;
    };

    let original_face_count = scan.0.faces.len();
    let target = simplify_state.target_face_count;

    let (new_mesh, status_text) = match action {
        SimplifyAction::Apply => {
            let result = simplify_mesh(&scan.0, target);
            let achieved = result.mesh.faces.len();
            let text = format!(
                "Reduced {} -> {} faces in {:.1}s",
                human_count(original_face_count),
                format_count_with_separators(achieved),
                result.elapsed_secs,
            );
            (result.mesh, text)
        }
        SimplifyAction::Reset => {
            let cloned = original.0.clone();
            let text = format!(
                "Restored original mesh ({} faces)",
                human_count(cloned.faces.len()),
            );
            (cloned, text)
        }
    };

    info.vertex_count = new_mesh.vertices.len();
    info.face_count = new_mesh.faces.len();
    scan.0 = new_mesh;
    respawn_scan_entity(
        &mut commands,
        &existing,
        &scan.0,
        meshes.as_mut(),
        materials.as_mut(),
        *up,
        render_scale.0,
    );

    status.text = status_text;
    status.kind = StatusKind::Normal;
    status.auto_clear_at_secs = Some(time.elapsed_secs_f64() + STATUS_NORMAL_TTL_SECS);
}

/// Despawn the existing `ScanMeshEntity` and spawn a fresh one from
/// `mesh` with the cast-frame up-axis swap + render-scale transform.
/// Used by [`handle_simplify_actions`] for both Apply and Reset paths.
fn respawn_scan_entity(
    commands: &mut Commands,
    existing: &Query<Entity, With<ScanMeshEntity>>,
    mesh: &IndexedMesh,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    up: UpAxis,
    render_scale: f32,
) {
    for entity in existing {
        commands.entity(entity).despawn();
    }
    let entity = spawn_face_mesh(
        commands,
        meshes,
        materials,
        mesh,
        None,
        up,
        Transform::from_scale(Vec3::splat(render_scale)),
    );
    commands.entity(entity).insert(ScanMeshEntity);
}

/// Update system: clear the status bar text if its TTL has elapsed.
/// Persistent messages (`auto_clear_at_secs = None`) are never cleared
/// here — they're overwritten by the next status update instead.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn auto_clear_status(mut status: ResMut<StatusBar>, time: Res<Time>) {
    let Some(deadline) = status.auto_clear_at_secs else {
        return;
    };
    if time.elapsed_secs_f64() >= deadline {
        status.text.clear();
        status.auto_clear_at_secs = None;
    }
}

/// Format a count with thousands separators (e.g., `1_234_567` →
/// `"1,234,567"`). Used in Simplify achievement messages where the
/// `human_count` `k`/`M` suffix would hide the precise meshopt-achieved
/// face count.
fn format_count_with_separators(n: usize) -> String {
    let raw = n.to_string();
    // Insert commas every 3 digits from the right.
    let mut out = String::with_capacity(raw.len() + raw.len() / 3);
    let bytes = raw.as_bytes();
    for (i, b) in bytes.iter().enumerate() {
        if i > 0 && (bytes.len() - i).is_multiple_of(3) {
            out.push(',');
        }
        out.push(*b as char);
    }
    out
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

/// Right-side egui sidebar carrying the scan-prep panels + bottom
/// status bar. Renders the currently-implemented set of collapsing
/// sections (one `render_*_section` call per section, in spec-mockup
/// order) inside a single `SidePanel` per the spec's window-layout
/// mockup (`docs/SCAN_PREP_DESIGN.md` §Window layout). Adding a new
/// panel = add a new `render_*_section` function + a single call here.
// `Result` is shadowed by `use anyhow::Result` at the top of this file;
// use Bevy's prelude alias explicitly so the panel system's
// `EguiContexts::ctx_mut()?` short-circuit returns `BevyError` not
// `anyhow::Error`.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn scan_prep_panel(
    mut contexts: EguiContexts,
    info: Res<ScanInfo>,
    mut simplify_state: ResMut<SimplifyState>,
    status: Res<StatusBar>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    // Status bar first so its bottom-anchored space is claimed before
    // the right-side panel computes its own layout (egui evaluates
    // panels in call order; later panels see the remaining central
    // region).
    render_status_bar(ctx, &status);
    egui::SidePanel::right("cf-scan-prep-panels")
        .resizable(false)
        .default_width(320.0)
        .show(ctx, |ui| {
            render_scan_info_section(ui, &info);
            render_simplify_section(ui, &info, &mut simplify_state);
        });
    Ok(())
}

/// `Scan Info` section (`docs/SCAN_PREP_DESIGN.md` §Panel specifications
/// §1) — static fields (file path, STL units, raw AABB) loaded at
/// startup; vertex/face counts update when the Simplify panel applies a
/// decimation pass.
///
/// Five logical fields:
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

/// `Simplify (decimate)` section (`docs/SCAN_PREP_DESIGN.md` §Panel
/// specifications §2). Boundary-preserving quadric edge collapse via
/// meshopt; load-bearing for any scan-driven cf-cast workflow on
/// modern multi-million-face scans.
///
/// Layout:
///
/// - Current face count (live; reflects the most recent simplify if
///   any).
/// - Logarithmic slider over `[SIMPLIFY_TARGET_MIN,
///   SIMPLIFY_TARGET_MAX]` (1k–1M). Logarithmic so the 200k default
///   sits near the middle of the slider track instead of the 20 %
///   mark.
/// - `[Apply simplify]` + `[Reset to original]` buttons queue
///   [`SimplifyAction`] on `SimplifyState`; [`handle_simplify_actions`]
///   consumes them next Update tick.
fn render_simplify_section(ui: &mut egui::Ui, info: &ScanInfo, state: &mut SimplifyState) {
    egui::CollapsingHeader::new("Simplify (decimate)")
        .default_open(true)
        .show(ui, |ui| {
            ui.label(format!("Current: {} faces", human_count(info.face_count)));

            ui.add_space(4.0);
            ui.label("Target face count:");
            ui.add(
                egui::Slider::new(
                    &mut state.target_face_count,
                    SIMPLIFY_TARGET_MIN..=SIMPLIFY_TARGET_MAX,
                )
                .logarithmic(true)
                .text("faces"),
            );

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                if ui.button("Apply simplify").clicked() {
                    state.pending_action = Some(SimplifyAction::Apply);
                }
                if ui.button("Reset to original").clicked() {
                    state.pending_action = Some(SimplifyAction::Reset);
                }
            });
        });
}

/// Bottom status bar. Renders nothing when `status.text` is empty so an
/// empty status doesn't claim window space. Color-codes the text by
/// [`StatusKind`]: gray for Normal, yellow for Warning (auto-suggest
/// banner / soft caps). An `Error` color path (red) lands at commit #12
/// alongside the Save panel's FS-error pattern.
fn render_status_bar(ctx: &egui::Context, status: &StatusBar) {
    if status.text.is_empty() {
        return;
    }
    let color = match status.kind {
        StatusKind::Normal => egui::Color32::from_gray(220),
        StatusKind::Warning => egui::Color32::from_rgb(240, 200, 80),
    };
    egui::TopBottomPanel::bottom("cf-scan-prep-status")
        .resizable(false)
        .show(ctx, |ui| {
            ui.colored_label(color, &status.text);
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
    /// the Scan Info panel. Pinned because changing the default
    /// silently 1000×-shifts every cleaned STL.
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
    /// §Panel specifications §1). ASCII `->` (not Unicode `→`) because
    /// egui's default font lacks U+2192 — see [`StlUnits::panel_label`].
    #[test]
    fn stl_units_mm_panel_label_matches_spec() {
        assert_eq!(StlUnits::Mm.panel_label(), "mm (vertex × 0.001 -> meters)");
    }

    #[test]
    fn stl_units_m_panel_label_matches_spec() {
        assert_eq!(StlUnits::M.panel_label(), "m (no scale)");
    }

    #[test]
    fn stl_units_inch_panel_label_matches_spec() {
        assert_eq!(
            StlUnits::Inch.panel_label(),
            "inch (vertex × 0.0254 -> meters)"
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
        assert_eq!(info.stl_units_label, "mm (vertex × 0.001 -> meters)");
        assert!((info.aabb_width_mm - 80.0).abs() < 1e-9);
        assert!((info.aabb_depth_mm - 80.0).abs() < 1e-9);
        assert!((info.aabb_height_mm - 80.0).abs() < 1e-9);
    }

    // ----- SimplifyState defaults ------------------------------------

    /// Default target face count is `SIMPLIFY_TARGET_DEFAULT` (200k)
    /// per spec §Architectural decisions §Simplify algorithm. Pinned so
    /// silent default-shift cascades don't break the iter-1 fixture's
    /// expected ~3.35M → ~200k decimation.
    #[test]
    fn simplify_state_default_target_is_200k() {
        let state = SimplifyState::default();
        assert_eq!(state.target_face_count, 200_000);
        assert_eq!(state.target_face_count, SIMPLIFY_TARGET_DEFAULT);
        assert!(state.pending_action.is_none());
    }

    /// Slider bounds spec-pin (1k–1M) per spec §Panel specifications §2.
    /// Changing either bound silently shifts the slider's logarithmic
    /// midpoint (where the 200k default sits in the track), so worth
    /// a regression test.
    #[test]
    fn simplify_slider_bounds_match_spec() {
        assert_eq!(SIMPLIFY_TARGET_MIN, 1_000);
        assert_eq!(SIMPLIFY_TARGET_MAX, 1_000_000);
    }

    /// Auto-suggest threshold is 500k faces per spec §Panel
    /// specifications §2.
    #[test]
    fn auto_suggest_threshold_is_500k_faces() {
        assert_eq!(AUTO_SUGGEST_FACE_THRESHOLD, 500_000);
    }

    // ----- format_count_with_separators ------------------------------

    /// Achievement-message format helper. Used in the "Reduced X -> Y
    /// faces in Zs" status text where Y needs the precise meshopt-
    /// achieved face count (not the human_count `k`/`M` summary).
    #[test]
    fn format_count_with_separators_handles_size_ranges() {
        assert_eq!(format_count_with_separators(0), "0");
        assert_eq!(format_count_with_separators(42), "42");
        assert_eq!(format_count_with_separators(999), "999");
        assert_eq!(format_count_with_separators(1_000), "1,000");
        assert_eq!(format_count_with_separators(12_345), "12,345");
        assert_eq!(format_count_with_separators(198_432), "198,432");
        assert_eq!(format_count_with_separators(1_234_567), "1,234,567");
    }

    // ----- simplify_mesh end-to-end (small fixture) ------------------

    /// Build a regular grid-subdivided square (in the XY plane) with
    /// `cells × cells` quads, each split into 2 triangles. Vertices are
    /// shared (proper indexed mesh; no STL-style 3N unsharing). Used
    /// for the simplify-on-known-shape test — small enough to be fast
    /// in CI while large enough that meshopt has something to collapse.
    fn grid_square(cells: usize) -> IndexedMesh {
        let mut mesh = IndexedMesh::with_capacity((cells + 1) * (cells + 1), cells * cells * 2);
        for j in 0..=cells {
            for i in 0..=cells {
                #[allow(clippy::cast_precision_loss)]
                let x = i as f64 / cells as f64;
                #[allow(clippy::cast_precision_loss)]
                let y = j as f64 / cells as f64;
                mesh.vertices.push(mesh_types::Point3::new(x, y, 0.0));
            }
        }
        #[allow(clippy::cast_possible_truncation)]
        let stride = (cells + 1) as u32;
        for j in 0..cells {
            for i in 0..cells {
                #[allow(clippy::cast_possible_truncation)]
                let row = j as u32 * stride;
                #[allow(clippy::cast_possible_truncation)]
                let next_row = (j as u32 + 1) * stride;
                #[allow(clippy::cast_possible_truncation)]
                let col = i as u32;
                let a = row + col;
                let b = row + col + 1;
                let c = next_row + col + 1;
                let d = next_row + col;
                mesh.faces.push([a, b, c]);
                mesh.faces.push([a, c, d]);
            }
        }
        mesh
    }

    /// `simplify_mesh` on a 20×20-cell grid (800 faces) targeting 100
    /// faces returns a mesh with face count ≤ target. meshopt doesn't
    /// guarantee hitting the target exactly (topology may force fewer
    /// collapses), so the assertion is upper-bounded.
    #[test]
    fn simplify_mesh_reduces_face_count_within_target() {
        let original = grid_square(20);
        assert_eq!(original.faces.len(), 800);

        let result = simplify_mesh(&original, 100);
        assert!(
            result.mesh.faces.len() <= 100,
            "simplified face count {} should not exceed target 100",
            result.mesh.faces.len(),
        );
        // Strictly smaller than original (some progress was made).
        assert!(
            result.mesh.faces.len() < original.faces.len(),
            "simplify must reduce face count (got {} from original {})",
            result.mesh.faces.len(),
            original.faces.len(),
        );
        // Elapsed time is finite + non-negative (sanity on the timer).
        assert!(result.elapsed_secs >= 0.0);
        assert!(result.elapsed_secs.is_finite());
    }

    /// `simplify_mesh` post-process strips unreferenced vertices, so
    /// every vertex in the output is touched by at least one face. The
    /// converse (every face references valid vertex indices) is
    /// trivially true since `remove_unreferenced_vertices` preserves
    /// face validity by construction.
    #[test]
    fn simplify_mesh_strips_unreferenced_vertices() {
        let original = grid_square(20);
        let result = simplify_mesh(&original, 50);

        let vertex_count = result.mesh.vertices.len();
        let mut touched = vec![false; vertex_count];
        for face in &result.mesh.faces {
            for &idx in face {
                touched[idx as usize] = true;
            }
        }
        let untouched = touched.iter().filter(|t| !**t).count();
        assert_eq!(
            untouched, 0,
            "simplified mesh should have no unreferenced vertices",
        );
    }
}
