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

use anyhow::{Context, Result};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use cf_bevy_common::prelude::*;
use cf_viewer::{
    RenderScale, compute_render_scale, scale_aabb, setup_camera_and_lighting, spawn_face_mesh,
};
use clap::{Parser, ValueEnum};
use mesh_io::{load_stl, save_stl};
use mesh_repair::{
    MeshAdjacency, holes, remove_degenerate_triangles, remove_small_components,
    remove_unreferenced_vertices, weld_vertices,
};
use mesh_types::{Aabb, Bounded, IndexedMesh, Point3};
use meshopt::{SimplifyOptions, simplify_decoder};
use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use serde::Serialize;

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

    /// Output directory for the Save panel's `.cleaned.stl` +
    /// `.prep.toml` writes. Defaults to the input scan's parent
    /// directory. Per spec §Architectural decisions §Save behavior:
    /// useful when the input directory is read-only.
    #[arg(long, value_name = "PATH")]
    output_dir: Option<PathBuf>,
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

/// Area threshold for `remove_degenerate_triangles` in the
/// `build_cleaned_mesh` cleanup pass (CSP.3). Square meters; 1e-15 m²
/// is well below cf-cast's 2 mm-cell SDF resolution (4e-6 m² per cell
/// face) and below the f32 quantization floor meshopt operates at,
/// so anything we strip here is FP noise the downstream couldn't have
/// used anyway. Catches zero-area triangles from cap ear-clip
/// degeneracies, clip-intersection sliver triangles, and any
/// preexisting degenerate faces the raw STL carried in. The latter
/// is what made cf-device-design's `simplify_decoder` retain its
/// full 3.34M face count on the iter-1 fixture
/// (`tools/cf-device-design/src/main.rs:419-425`).
const CLEANUP_DEGENERATE_AREA_M2: f64 = 1e-15;

/// Minimum face count for a connected component to survive the
/// `build_cleaned_mesh` cleanup pass. Components smaller than this
/// are dropped as scanner noise. 10 is conservative — a meaningful
/// shell (even a small cyst-like protrusion) has dozens of faces;
/// noise islands are typically 1-3 stray triangles from scanner
/// registration glitches or self-intersection artifacts. Spec
/// §Strategic context assumes the user has pre-trimmed to a single
/// shell externally; this threshold is the safety net for the
/// "user forgot" case.
const CLEANUP_MIN_COMPONENT_FACES: usize = 10;

/// `target_error` parameter passed to `meshopt::simplify_decoder`.
/// Normalized to mesh extents — `1.0` means "deviate by up to half
/// the longest mesh axis"; `0.05` ≈ 5 % of half-extent.
///
/// **Calibrated against cf-cast's SDF sampling resolution.** cf-cast
/// samples its `SignedDistanceField` on a 2 mm grid (spec
/// §Strategic context); geometric detail below 2 mm is sub-cell and
/// won't transfer to the silicone mold. Setting the threshold above
/// that floor means we're trading "wasted detail meshopt couldn't
/// have preserved anyway" for "actually reaching the requested face
/// count". For the iter-1 fixture (130 mm tall), `0.05` × 65 mm
/// half-axis = ~3.25 mm worst-case tolerance — comfortably above the
/// 2 mm SDF floor + below typical scan-feature scales.
///
/// **History**: started at `0.01` (~0.65 mm tolerance for iter-1)
/// which was too tight — sock-textile scans hit the error budget
/// before reaching the requested face count (~3.35M → ~3.33M,
/// effectively zero reduction). Loosened to `0.05` so meshopt
/// actually decimates. Re-tune if iter-1 cast surfaces visible
/// surface artifacts; tighten back toward `0.01` if so.
const SIMPLIFY_TARGET_ERROR: f32 = 0.05;

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
/// pick a text color (gray for `Normal`, yellow for `Warning`, red for
/// `Error`). The `Error` variant carries the Save panel's FS-error +
/// disk-write-failure pattern per spec §Architectural decisions §FS
/// error handling.
#[derive(Default, Clone, Copy, Debug, PartialEq, Eq)]
enum StatusKind {
    /// Neutral information (achievement messages, restore-confirmed).
    #[default]
    Normal,
    /// Advisory but non-blocking (auto-suggest banner, soft caps).
    Warning,
    /// Save-time disk error or non-finite transform (Save panel; spec
    /// §Architectural decisions §FS error handling). Rendered red.
    Error,
}

/// Pre-computed display fields for the Scan Info panel
/// (`docs/SCAN_PREP_DESIGN.md` §Panel specifications §1). Built once at
/// startup from the loaded scan + CLI metadata. Static fields
/// (`file_label`, `file_full_path`, `stl_units_label`, the three
/// `aabb_*_mm` extents) never change after construction; `vertex_count`
/// and `face_count` are mutated by [`handle_simplify_actions`] to track
/// the Simplify panel's `[Apply simplify]` / `[Reset to original]`
/// effects so the panel display reflects the currently-rendered mesh.
///
/// AABB dimensions are stored in **millimeters regardless of
/// `--stl-units`** — workshop measurements are in mm; consistency with
/// the user's mental model. Computed from the post-unit-conversion meter
/// AABB by scaling by 1000. The AABB extents stay anchored to the
/// loaded mesh (not the simplified mesh) because that's what the spec's
/// "Raw AABB" label means; for the post-transform AABB see the Cleaned
/// AABB panel (commit #11).
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

/// Reorient panel state (`docs/SCAN_PREP_DESIGN.md` §Panel
/// specifications §3). Holds the user's scan-frame → cast-frame
/// rotation as three intrinsic XYZ Euler-angle degrees.
///
/// **Source-of-truth convention**: the three slider values
/// (`roll_deg` / `pitch_deg` / `yaw_deg`) are the source of truth;
/// [`quaternion_physics`](Self::quaternion_physics) derives the unit
/// quaternion at read time. Per spec §Risks §"Quaternion-to-Euler-to-
/// quaternion round-trip drift" this avoids the round-trip drift that
/// would accumulate if we stored only a quaternion and re-derived the
/// Euler angles for the sliders each tick. Snap buttons set the three
/// slider values to known degree triples; manual slider drags update
/// one value at a time.
///
/// **Frame convention**: the rotation is conceptually in *physics
/// frame* (scan-vertex coordinates pre-Bevy-up-axis-swap). The Bevy
/// `Transform` component on `ScanMeshEntity` gets a bevy-frame
/// quaternion via [`physics_quat_to_bevy_for_plus_z`] per render tick;
/// the saved STL (commit #12) bakes the physics-frame rotation into
/// vertex positions directly.
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
struct ReorientState {
    /// Rotation about the physics-frame X axis, in degrees. Range
    /// `[-180, 180]` enforced by the egui slider.
    roll_deg: f64,
    /// Intrinsic rotation about the body-frame Y axis (after roll), in
    /// degrees.
    pitch_deg: f64,
    /// Intrinsic rotation about the body-frame Z axis (after roll +
    /// pitch), in degrees.
    yaw_deg: f64,
}

impl Default for ReorientState {
    fn default() -> Self {
        Self {
            roll_deg: 0.0,
            pitch_deg: 0.0,
            yaw_deg: 0.0,
        }
    }
}

impl ReorientState {
    /// Derive the physics-frame unit quaternion from the current
    /// slider values (intrinsic XYZ Euler, radians).
    fn quaternion_physics(self) -> UnitQuaternion<f64> {
        UnitQuaternion::from_euler_angles(
            self.roll_deg.to_radians(),
            self.pitch_deg.to_radians(),
            self.yaw_deg.to_radians(),
        )
    }
}

/// Recenter panel state (`docs/SCAN_PREP_DESIGN.md` §Panel
/// specifications §4). Three independent translation components in
/// **physics-frame millimeters** — mm for display + workshop-convention
/// consistency, physics-frame because the saved STL (commit #12) bakes
/// the translation into vertex positions directly (without an up-axis
/// swap).
///
/// The `[Center origin]` and `[Floor -> z=0]` buttons compute their
/// effects at click time using the current `ReorientState` quaternion
/// applied to the raw scan AABB corners — so the centering is correct
/// for the rotated mesh, not the unrotated one. If the user changes
/// rotation after clicking, the centering becomes stale; they re-click
/// to re-center. Spec calls them buttons (not continuous behaviors).
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
struct RecenterState {
    /// Physics-frame +X translation in mm.
    tx_mm: f64,
    /// Physics-frame +Y translation in mm.
    ty_mm: f64,
    /// Physics-frame +Z translation in mm.
    tz_mm: f64,
}

impl Default for RecenterState {
    fn default() -> Self {
        Self {
            tx_mm: 0.0,
            ty_mm: 0.0,
            tz_mm: 0.0,
        }
    }
}

impl RecenterState {
    /// Project the physics-frame mm translation into a Bevy-world-space
    /// `Vec3`. Applies the `UpAxis::PlusZ` swap (physics `(x, y, z)` →
    /// bevy `(x, z, y)`) and the `render_scale` lift in one step so
    /// `apply_world_transform_to_scan_entity` can fold the result into
    /// `Transform.translation`.
    fn translation_world(self, up: UpAxis, render_scale: f32) -> Vec3 {
        let translation_m =
            mesh_types::Point3::new(self.tx_mm * 0.001, self.ty_mm * 0.001, self.tz_mm * 0.001);
        Vec3::from_array(up.to_bevy_point(&translation_m)) * render_scale
    }
}

/// Clip floor panel state (`docs/SCAN_PREP_DESIGN.md` §Panel
/// specifications §5). A horizontal-in-cast-frame plane at world
/// `z = z_mm` (physics `+Z` direction; bevy `+Y` post-swap) that
/// removes everything below it from the scan.
///
/// Scope at this commit: panel UI, the triangle-clipping algorithm,
/// the disc visualization, and the live drop-percentage readout. The
/// algorithm sits dormant — it's not yet applied to the displayed
/// mesh to avoid the respawn-race complexity with
/// `handle_simplify_actions`. Cap (commit #9) consumes the clipped
/// mesh for boundary detection; Save (commit #12) bakes the clip into
/// the cleaned STL. Until then the user sees the disc and the
/// "Drops X% verts" readout as visual feedback, but the rendered
/// mesh stays intact.
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
struct ClipState {
    /// When `false`, the disc visualization stays hidden and the panel
    /// readout doesn't render. The slider value persists across
    /// toggle.
    enabled: bool,
    /// Plane height in cast-frame world millimeters (post-Reorient +
    /// post-Recenter). Spec convention: `z >= z_mm` is kept; `z < z_mm`
    /// is removed.
    z_mm: f64,
    /// Cached percentage of vertices below the clip plane, populated
    /// by [`update_clip_drop_pct`] only when relevant state actually
    /// changes. Read-only by the panel.
    ///
    /// The vertex walk is O(N) — N can be 10M+ on an unsimplified
    /// scan, which takes ~0.5-1.0 s per call. Computing on every
    /// panel render (60 Hz) would freeze the UI; caching here makes
    /// the panel render O(1).
    ///
    /// Written via `bypass_change_detection` so updating the cache
    /// doesn't re-trigger the update system in a self-feedback loop.
    drop_pct: f64,
}

impl Default for ClipState {
    fn default() -> Self {
        Self {
            enabled: false,
            z_mm: 0.0,
            drop_pct: 0.0,
        }
    }
}

/// One detected boundary loop on the scan, with its least-squares
/// plane fit + per-loop user-decided cap-include state. Populated by
/// the `[Scan]` button's run-once handler; consumed by
/// `render_cap_section` (display) + `draw_cap_overlays` (gizmo
/// linestrip + plane outline) + commit #12 (bake into cleaned STL on
/// save).
///
/// All vertex indices are into the **scan mesh as it was at scan
/// time** — the boundary loop's vertex IDs reference positions in
/// `ScanMesh` snapshotted when the user clicked `[Scan]`. If the user
/// then simplifies or otherwise mutates `ScanMesh`, the indices go
/// stale and the loop list dims via the `CapState::stale` flag.
#[derive(Debug, Clone)]
struct DetectedCapLoop {
    /// Ordered vertex indices forming the boundary loop (closed: the
    /// last edge connects back to vertex 0).
    vertex_indices: Vec<u32>,
    /// Least-squares fit plane: position `centroid` and unit normal.
    /// Centroid is the average of loop vertex positions in mesh-local
    /// physics-frame meters; normal is outward-oriented per the
    /// mesh-side-of-plane heuristic. Stored at scan time for use by
    /// commit #12's ear-clipping triangulation (the bake step needs
    /// both centroid + normal to project loop vertices onto the fit
    /// plane before triangulating into cap faces).
    #[allow(dead_code)] // Used at commit #12 (Save) for cap triangulation.
    plane_centroid: Point3<f64>,
    plane_normal: Vector3<f64>,
    /// Plane-fit R²: 1.0 = perfectly planar loop; 0.0 = totally
    /// non-planar. Surfaced to the user so they can judge whether
    /// auto-fit produced a reasonable cap.
    plane_fit_r_squared: f64,
    /// User decision: include this loop in `[Apply caps]` /
    /// downstream save? Default `true` if vertex count >= 8 (per
    /// spec: small loops are often acceptable holes / scanner
    /// artifacts), `false` otherwise.
    include: bool,
}

/// Cap open boundaries state (`docs/SCAN_PREP_DESIGN.md` §Panel
/// specifications §6). Holds the latest scan result + the centerline
/// polyline (option-3 hedge for curve-following cf-cast).
///
/// **Scan vs Apply**: the `[Scan]` button populates `loops` +
/// `centerline_polyline` from the current `ScanMesh`. `[Apply caps]`
/// just commits the user's per-loop include selections — the actual
/// cap-triangle generation + bake into the saved STL happens at
/// commit #12 (Save panel). Until then `CapState` is purely
/// informational + visualization-driving.
///
/// **Stale invalidation**: when the user changes Reorient / Recenter
/// / Simplify after a scan, the loop indices may point at vertices
/// that no longer exist (post-Simplify) or the plane fits are in a
/// stale physics frame relative to the rotated/translated mesh. The
/// `stale` flag is set on any such change; the panel dims the loop
/// list + shows a "Transform changed — re-Scan to refresh" overlay
/// until the user re-clicks `[Scan]`.
#[derive(Resource, Debug, Clone, Default)]
struct CapState {
    /// Loops detected by the most recent `[Scan]` run. Empty until
    /// the user clicks Scan for the first time.
    loops: Vec<DetectedCapLoop>,
    /// Cross-section-centroid polyline approximating the scan's
    /// centerline, in physics-frame meters. Option-3 hedge: validates
    /// the centerline algorithm cf-cast will need for curve-following
    /// multi-piece molds, without yet building the full cf-cast
    /// architecture. Empty until first scan.
    centerline_polyline: Vec<Point3<f64>>,
    /// `true` when any state changes after a scan have invalidated
    /// the cached loops / centerline. The panel dims its readout +
    /// the overlays draw with reduced opacity until the user
    /// re-scans.
    stale: bool,
}

/// User action queued by the Cap panel's `[Scan]` button. Consumed
/// next Update tick by `handle_cap_actions`, which runs the actual
/// detection + plane fit + centerline math.
#[derive(Resource, Debug, Default)]
struct CapPendingAction {
    /// Set to `true` when the user clicks `[Scan]`; consumed
    /// (cleared) by `handle_cap_actions`.
    rescan: bool,
}

/// Centerline trim panel state (CSP.4b). The user dials how much
/// to shave off each end of the centerline polyline; the save
/// handler applies the cuts (perpendicular planes at each end
/// distance) and auto-caps the new boundaries.
///
/// Both values are in **millimeters** along the polyline arc — they
/// project to physics-meter coordinates inside
/// [`trim_mesh_along_centerline`] when the plane is constructed.
/// `0.0` means "no trim at that end."
///
/// The sliders only become meaningful after the user clicks
/// Cap → Scan (which populates `CapState::centerline_polyline`).
/// Until then the panel renders a hint to "Run Cap → Scan first."
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
struct CenterlineTrimState {
    /// Distance from the centerline polyline's tip (index 0) end
    /// to trim, in millimeters. Range `0..=max_trim_mm` enforced by
    /// the panel slider, where `max_trim_mm` is derived from the
    /// polyline arc length at panel-render time.
    trim_tip_mm: f64,
    /// Distance from the floor (index N-1) end to trim, in mm.
    trim_floor_mm: f64,
}

impl Default for CenterlineTrimState {
    fn default() -> Self {
        Self {
            trim_tip_mm: 0.0,
            trim_floor_mm: 0.0,
        }
    }
}

/// User action queued by the Save panel's `[Save]` button. Consumed
/// next Update tick by `handle_save_action`, which builds the
/// cleaned mesh + the `.prep.toml` and writes both files atomically.
#[derive(Resource, Debug, Default)]
struct SavePendingAction {
    save_now: bool,
}

/// Output directory for the Save panel's writes. Built at startup
/// from `--output-dir <path>` if supplied, else from the input
/// scan's parent directory. Carried as a resource so the Save
/// system reads it without re-parsing CLI args.
#[derive(Resource, Debug, Clone)]
struct SaveOutputDir(PathBuf);

/// Input scan path (`cli.path`) carried as a resource so the Save
/// handler can derive output filenames (`<stem>.cleaned.stl` /
/// `<stem>.prep.toml`) + record the source in `.prep.toml`'s
/// provenance block. Read-only after startup.
#[derive(Resource, Debug, Clone)]
struct SourceStlPath(PathBuf);

/// CLI `--stl-units` value carried as a resource so the Save
/// handler can record the load-time unit convention in
/// `.prep.toml`'s provenance block. Read-only after startup.
#[derive(Resource, Debug, Clone, Copy)]
struct StlUnitsResource(StlUnits);

/// Auto-center offset applied at load (CSP.3.5). In physics-frame
/// meters; equals `-raw_aabb.center()` from the source STL after
/// unit scaling. Read by `handle_save_action` to fill the
/// `[scan_prep].auto_center_offset_m` provenance line. Read-only
/// after startup. Zero vector when the source already happened to
/// sit at the origin (rare).
#[derive(Resource, Debug, Clone, Copy)]
struct AutoCenterOffset(Vector3<f64>);

/// Auto-PCA rotation applied at load (CSP.4a). Physics-frame unit
/// quaternion taking the scan's principal axis to `+Z` (cast-frame
/// demolding direction). Baked into vertex positions at load, just
/// like auto-center — so the in-memory + saved-to-disk mesh is in
/// canonical orientation. The user's Reorient sliders therefore
/// start at identity in the typical workflow; further manual
/// adjustment via those sliders is still available.
///
/// `None` when PCA failed (degenerate mesh, < 3 vertices, all
/// vertices coincident) — these scans are unworkable downstream
/// anyway; the auto-orient just becomes a no-op + the user sees the
/// raw orientation.
///
/// Recorded in `.prep.toml`'s `[scan_prep].auto_pca_quaternion` so
/// the source-frame position is recoverable (multiply: auto_center
/// + auto_pca then user transforms).
#[derive(Resource, Debug, Clone, Copy)]
struct AutoPcaQuaternion(Option<UnitQuaternion<f64>>);

/// Walk the scan's vertices, transform each into cast-frame world
/// coordinates (rotation + translation_z), and return the percentage
/// that fall below `clip_z_mm`. Called by [`update_clip_drop_pct`]
/// only on relevant state changes — not on every panel render — so
/// the O(N) vertex walk doesn't freeze the UI.
///
/// Returns 0.0 for an empty mesh (no vertices to drop).
///
/// **Inner-loop optimization**: we only need the **z-component** of
/// each rotated vertex. That's the 3rd row of the rotation matrix
/// dotted with the vertex vector (~6 ops), not the full
/// `transform_vector` call (~30 ops via `q · v · q^-1`). On a 10M-
/// vertex unsimplified scan this is the difference between ~50 ms
/// (acceptable single-shot lag on toggle / slider tick) and ~250 ms
/// (perceptible UI hitch). For 200k simplified vertices both paths
/// are sub-frame; the optimization mostly matters when the user
/// enables Clip *before* simplifying.
fn compute_drop_percentage(
    scan: &IndexedMesh,
    rotation: UnitQuaternion<f64>,
    translation_z_mm: f64,
    clip_z_mm: f64,
) -> f64 {
    if scan.vertices.is_empty() {
        return 0.0;
    }
    // Pull the rotation matrix's 3rd row once outside the hot loop.
    // `to_rotation_matrix` allocates a Matrix3; doing it per-vertex
    // would dominate the loop.
    let r = rotation.to_rotation_matrix();
    let z_row_x = r[(2, 0)];
    let z_row_y = r[(2, 1)];
    let z_row_z = r[(2, 2)];

    let mut below: usize = 0;
    for v in &scan.vertices {
        // Vertices are in physics-frame meters; lift to mm to match
        // the clip plane's mm convention. Compute only the z-component
        // of `rotation * v + translation_z`.
        let world_z_mm = z_row_x * v.x * 1000.0
            + z_row_y * v.y * 1000.0
            + z_row_z * v.z * 1000.0
            + translation_z_mm;
        if world_z_mm < clip_z_mm {
            below += 1;
        }
    }
    // `as f64` is loss-free for usize values cf-scan-prep handles
    // (well below 2^53).
    #[allow(clippy::cast_precision_loss)]
    {
        100.0 * (below as f64) / (scan.vertices.len() as f64)
    }
}

/// Update system: recompute [`ClipState::drop_pct`] when the clip
/// state, scan mesh, reorient state, or recenter state has actually
/// changed. Early-returns when clip is disabled (cached value stays
/// dormant; re-enable toggle triggers a fresh recompute).
///
/// **Why a separate system, not panel-driven**: see [`ClipState::drop_pct`]
/// docstring. The vertex walk is too expensive (~0.5-1.0 s on 10M
/// verts) to run on every panel render (60 Hz). Doing it via an
/// Update system + change detection makes the panel render O(1) and
/// limits the heavy work to actual state transitions.
///
/// **`bypass_change_detection` write**: writing to `clip.drop_pct`
/// via the normal `ResMut<ClipState>` path would mark `ClipState` as
/// changed next tick, re-triggering this system in a self-feedback
/// loop. `bypass_change_detection` returns `&mut ClipState` without
/// updating the change-detection tick, breaking the cycle.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn update_clip_drop_pct(
    mut clip: ResMut<ClipState>,
    scan: Res<ScanMesh>,
    reorient: Res<ReorientState>,
    recenter: Res<RecenterState>,
) {
    if !clip.enabled {
        return;
    }
    if !clip.is_changed() && !scan.is_changed() && !reorient.is_changed() && !recenter.is_changed()
    {
        return;
    }
    let pct = compute_drop_percentage(
        &scan.0,
        reorient.quaternion_physics(),
        recenter.tz_mm,
        clip.z_mm,
    );
    clip.bypass_change_detection().drop_pct = pct;
}

/// Linear interpolation between two `Point3<f64>` positions.
/// `t = 0` returns `a`; `t = 1` returns `b`.
fn lerp_point(a: &Point3<f64>, b: &Point3<f64>, t: f64) -> Point3<f64> {
    Point3::new(
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
        a.z + t * (b.z - a.z),
    )
}

/// True-plane-intersection mesh clip against a plane in the mesh's
/// own coordinate frame, expressed as the equation
/// `plane_normal · v >= plane_d` (kept side).
///
/// Shared core algorithm — both [`clip_mesh_against_world_z`] (the
/// horizontal-Z clip from the Clip floor panel) and
/// [`clip_mesh_against_plane`] (the centerline-trim cuts at each
/// end) derive their inputs and then forward here.
///
/// For each triangle:
/// - **All 3 vertices on/above** the plane → keep as-is.
/// - **All 3 below** → drop.
/// - **Mixed** → compute intersection points on the crossing edges +
///   triangulate the surviving polygon (3 or 4 vertices) via a fan
///   from the first vertex. The result is a clean planar cut along
///   the plane, suitable for capping (boundary stays planar so
///   `mesh-repair`'s plane fit gets `R² ≈ 1.0`).
///
/// Vertices exactly on the plane (`dist == 0`) are classified as
/// "above" so the surrounding triangle is kept; this avoids
/// degenerate sub-triangles in the cut.
///
/// Output mesh's vertex buffer = original vertices + new intersection
/// vertices appended; `remove_unreferenced_vertices` strips the
/// dropped (all-below) original vertices afterwards so the result is
/// tight.
fn clip_mesh_against_plane_eq(
    mesh: &IndexedMesh,
    plane_normal: Vector3<f64>,
    plane_d: f64,
) -> IndexedMesh {
    // Output buffers — start with the original vertices (later
    // stripped of unreferenced); append intersection points as we go.
    let mut new_vertices: Vec<Point3<f64>> = mesh.vertices.clone();
    let mut new_faces: Vec<[u32; 3]> = Vec::with_capacity(mesh.faces.len());

    for face in &mesh.faces {
        let verts = [
            mesh.vertices[face[0] as usize],
            mesh.vertices[face[1] as usize],
            mesh.vertices[face[2] as usize],
        ];
        // Signed distance from the plane. dist >= 0 → above (keep).
        let dists = [
            plane_normal.dot(&verts[0].coords) - plane_d,
            plane_normal.dot(&verts[1].coords) - plane_d,
            plane_normal.dot(&verts[2].coords) - plane_d,
        ];
        let above_count = dists.iter().filter(|d| **d >= 0.0).count();

        if above_count == 3 {
            new_faces.push(*face);
            continue;
        }
        if above_count == 0 {
            continue;
        }

        // Mixed case: walk the triangle CCW, collecting above-plane
        // vertices + intersection points where edges cross. The
        // resulting polygon has 3 (1-above) or 4 (2-above) vertices —
        // both convex, so fan-triangulate from vertex[0].
        let mut poly_indices: Vec<u32> = Vec::with_capacity(4);
        for i in 0..3 {
            let next = (i + 1) % 3;
            let d_curr = dists[i];
            let d_next = dists[next];
            if d_curr >= 0.0 {
                poly_indices.push(face[i]);
            }
            // Edge crosses the plane iff the signs of d_curr and
            // d_next differ. The `>= 0.0` convention puts zeros on
            // the "above" side, so an edge with one zero and one
            // negative still crosses.
            if (d_curr >= 0.0) != (d_next >= 0.0) {
                let t = d_curr / (d_curr - d_next);
                let intersection = lerp_point(&verts[i], &verts[next], t);
                #[allow(clippy::cast_possible_truncation)]
                let new_idx = new_vertices.len() as u32;
                new_vertices.push(intersection);
                poly_indices.push(new_idx);
            }
        }

        // Fan triangulation from poly_indices[0].
        for i in 1..(poly_indices.len() - 1) {
            new_faces.push([poly_indices[0], poly_indices[i], poly_indices[i + 1]]);
        }
    }

    let mut clipped = IndexedMesh::with_capacity(new_vertices.len(), new_faces.len());
    clipped.vertices = new_vertices;
    clipped.faces = new_faces;
    remove_unreferenced_vertices(&mut clipped);
    clipped
}

/// Clip `mesh` against the world-frame horizontal plane at
/// `z = clip_z_m`, given that mesh-local coords are related to world
/// coords by `world = rotation * mesh + (0, 0, translation_z_m)`.
/// CSP.2 Clip-floor wiring; see `docs/SCAN_PREP_DESIGN.md` §Panel
/// specifications §5.
fn clip_mesh_against_world_z(
    mesh: &IndexedMesh,
    rotation: UnitQuaternion<f64>,
    translation_z_m: f64,
    clip_z_m: f64,
) -> IndexedMesh {
    // Express the world-frame plane `z = clip_z_m` in mesh-local frame.
    // World `z = (R * v).z + T.z`; keep `z >= clip_z` becomes
    // `dot(v, R^T · e_z) >= clip_z - T.z` in mesh-local coords.
    let plane_normal = rotation
        .inverse()
        .transform_vector(&Vector3::new(0.0, 0.0, 1.0));
    let plane_d = clip_z_m - translation_z_m;
    clip_mesh_against_plane_eq(mesh, plane_normal, plane_d)
}

/// Clip `mesh` against an arbitrary plane defined by a point on the
/// plane and a normal. Kept side: where
/// `(p - plane_point) · plane_normal >= 0`. CSP.4b — used by
/// [`trim_mesh_along_centerline`] to clip each end of the centerline
/// at a user-chosen distance.
///
/// `plane_normal` MUST be a unit vector — the algorithm relies on the
/// dot product producing signed distances. Callers passing a
/// non-unit normal will get a clip that succeeds but with wrong
/// intersection-point math; cheaper than asserting here.
fn clip_mesh_against_plane(
    mesh: &IndexedMesh,
    plane_point: Point3<f64>,
    plane_normal: Vector3<f64>,
) -> IndexedMesh {
    // `plane_normal · (p - plane_point) >= 0`
    //   ⇔ `plane_normal · p >= plane_normal · plane_point`
    let plane_d = plane_normal.dot(&plane_point.coords);
    clip_mesh_against_plane_eq(mesh, plane_normal, plane_d)
}

/// Walk along `polyline`'s arc and return the position + local
/// tangent at arc-length `distance_m` from `polyline[0]`. The
/// tangent is the unit direction of the segment containing the
/// target point, pointing from `polyline[0]` toward
/// `polyline[N-1]`. CSP.4b.3 — the load-bearing primitive for both
/// the mesh-trim algorithm + the live overlay; both used to
/// extrapolate linearly from the first/last segment's tangent
/// (CSP.4b initial), which diverged from the actual centerline
/// path on curved scans (user-reported via screenshots 2026-05-15).
///
/// Returns `None` for a < 2-point polyline (no segments to walk).
/// For `distance_m` past the polyline's total arc length, returns
/// a linear extrapolation along the LAST segment's tangent — fine
/// for the panel-clamped slider range (max = half arc length per
/// end), but explicit so future call sites that pass arbitrary
/// distances don't hit a silent failure.
fn point_along_polyline_at_arc_distance(
    polyline: &[Point3<f64>],
    distance_m: f64,
) -> Option<(Point3<f64>, Vector3<f64>)> {
    if polyline.len() < 2 {
        return None;
    }
    let target_m = distance_m.max(0.0);
    let mut walked_m = 0.0_f64;
    for i in 0..polyline.len() - 1 {
        let seg_vec = polyline[i + 1].coords - polyline[i].coords;
        let seg_len = seg_vec.norm();
        if seg_len < f64::EPSILON {
            continue;
        }
        if walked_m + seg_len >= target_m {
            // Target lies within segment `i..i+1`. Lerp to the
            // exact point + return that segment's unit tangent.
            let t = ((target_m - walked_m) / seg_len).clamp(0.0, 1.0);
            let pos = Point3::from(polyline[i].coords + t * seg_vec);
            let tangent = seg_vec / seg_len;
            return Some((pos, tangent));
        }
        walked_m += seg_len;
    }
    // Past the end of the polyline — linear extrapolation along the
    // last segment's tangent. Reaches this branch only if the
    // caller's distance exceeds the polyline arc length (the
    // user-facing panel clamps to half arc length per end, so this
    // is a safety fallback, not a hot path).
    let n = polyline.len();
    let last_seg = polyline[n - 1].coords - polyline[n - 2].coords;
    let last_seg_len = last_seg.norm();
    if last_seg_len < f64::EPSILON {
        return None;
    }
    let overrun_m = target_m - walked_m;
    let tangent = last_seg / last_seg_len;
    let pos = Point3::from(polyline[n - 1].coords + tangent * overrun_m);
    Some((pos, tangent))
}

/// Total arc length of `polyline` in meters. Returns `0.0` for
/// `< 2` points.
fn polyline_arc_length_m(polyline: &[Point3<f64>]) -> f64 {
    polyline
        .windows(2)
        .map(|w| (w[1].coords - w[0].coords).norm())
        .sum()
}

/// Trim `mesh` along its centerline polyline: clips off
/// `trim_tip_mm` from the tip end (centerline\[0\]) and
/// `trim_floor_mm` from the floor end (centerline\[N-1\]) of the
/// polyline. Returns the trimmed mesh.
///
/// CSP.4b user-facing feature. The trim planes are perpendicular to
/// the centerline tangent **at the cut point** (computed via
/// [`point_along_polyline_at_arc_distance`]). For a roughly-straight
/// centerline this is a horizontal cut at the corresponding world-Z
/// height; for a curved centerline (sock fixture with PCA-induced
/// curvature) the cut tilts with the local spine direction — the
/// natural workshop intent of "shave off the noisy end along the
/// spine."
///
/// CSP.4b.3 — initial CSP.4b used the first/last polyline segment's
/// tangent + linear extrapolation, which diverged badly from the
/// real centerline on curved scans (visible in trim-overlay
/// screenshots). The current implementation walks the polyline arc.
///
/// Centerline polyline ordering convention: index 0 is the tip
/// (closed end / dome), index N-1 is the floor (open end / rim).
/// This is the order [`compute_centerline_polyline`] returns when
/// driven by the first cap loop's outward normal (which points
/// outward from the rim, toward -Z under PCA convention).
///
/// `trim_tip_mm == 0 && trim_floor_mm == 0` is a fast-path no-op
/// (returns `mesh.clone()`). A degenerate centerline (< 2 points)
/// also returns the input unchanged.
///
/// The trimmed mesh has new open boundaries at each non-zero cut.
/// The caller is responsible for capping those — see
/// [`auto_cap_open_boundaries`] for the auto-cap step.
fn trim_mesh_along_centerline(
    mesh: &IndexedMesh,
    centerline: &[Point3<f64>],
    trim_tip_mm: f64,
    trim_floor_mm: f64,
) -> IndexedMesh {
    if centerline.len() < 2 || (trim_tip_mm <= 0.0 && trim_floor_mm <= 0.0) {
        return mesh.clone();
    }
    let mut out = mesh.clone();

    // Tip-end trim: plane at arc-distance `trim_tip_mm` forward
    // from the tip; plane normal = local tangent (points toward
    // floor). Kept side = the "floor" side.
    if trim_tip_mm > 0.0 {
        if let Some((plane_point, tangent)) =
            point_along_polyline_at_arc_distance(centerline, trim_tip_mm * 0.001)
        {
            out = clip_mesh_against_plane(&out, plane_point, tangent);
        }
    }

    // Floor-end trim: plane at arc-distance
    // `total_length - trim_floor_mm` from the tip (i.e.,
    // `trim_floor_mm` backward from the floor). Plane normal =
    // -tangent (points toward tip). Kept side = the "tip" side.
    if trim_floor_mm > 0.0 {
        let total_length_m = polyline_arc_length_m(centerline);
        let target_m = total_length_m - trim_floor_mm * 0.001;
        if target_m > 0.0 {
            if let Some((plane_point, tangent)) =
                point_along_polyline_at_arc_distance(centerline, target_m)
            {
                out = clip_mesh_against_plane(&out, plane_point, -tangent);
            }
        }
    }

    out
}

/// Trim the centerline polyline by the same distances applied to
/// the mesh in [`trim_mesh_along_centerline`]: drop `trim_tip_mm`
/// from the start (index 0 = tip) and `trim_floor_mm` from the end
/// (index N-1 = floor). Both trims measured in millimeters along
/// the cumulative arc length.
///
/// CSP.4b — keeps the `.prep.toml`'s `[centerline].points_m` line
/// in sync with the cleaned STL on disk. Without this, the TOML
/// would describe a longer centerline than the mesh actually has.
///
/// Returns an empty vec if the trim consumes the whole polyline
/// (`trim_tip_mm + trim_floor_mm >= total arc length`) — the
/// caller's `[centerline]` block then drops out per the
/// `skip_serializing_if` rule.
fn trim_centerline_polyline(
    centerline: &[Point3<f64>],
    trim_tip_mm: f64,
    trim_floor_mm: f64,
) -> Vec<Point3<f64>> {
    if centerline.len() < 2 || (trim_tip_mm <= 0.0 && trim_floor_mm <= 0.0) {
        return centerline.to_vec();
    }
    // Cumulative arc length from start, in millimeters. Seeded
    // with `0.0` and built by single-pass accumulation; `last`
    // dereferences are unwrap-free via tracking `accum` separately.
    let mut cum_lengths_mm: Vec<f64> = Vec::with_capacity(centerline.len());
    cum_lengths_mm.push(0.0);
    let mut accum = 0.0_f64;
    for w in centerline.windows(2) {
        accum += (w[1].coords - w[0].coords).norm() * 1000.0;
        cum_lengths_mm.push(accum);
    }
    let total_length_mm = accum;
    let start_mm = trim_tip_mm.max(0.0);
    let end_mm = (total_length_mm - trim_floor_mm.max(0.0)).max(start_mm);
    if end_mm <= start_mm + f64::EPSILON {
        return Vec::new();
    }

    // Linear interpolation at arc-length `target_mm` along the
    // polyline. Walks segments; for a 30-point polyline this is
    // trivial cost.
    let interp = |target_mm: f64| -> Point3<f64> {
        for i in 0..centerline.len() - 1 {
            let s = cum_lengths_mm[i];
            let e = cum_lengths_mm[i + 1];
            if target_mm <= e {
                let t = if e > s {
                    (target_mm - s) / (e - s)
                } else {
                    0.0
                };
                let interpolated =
                    centerline[i].coords + t * (centerline[i + 1].coords - centerline[i].coords);
                return Point3::from(interpolated);
            }
        }
        centerline[centerline.len() - 1]
    };

    let mut trimmed: Vec<Point3<f64>> = Vec::with_capacity(centerline.len());
    trimmed.push(interp(start_mm));
    for (i, p) in centerline.iter().enumerate() {
        if cum_lengths_mm[i] > start_mm + f64::EPSILON && cum_lengths_mm[i] < end_mm - f64::EPSILON
        {
            trimmed.push(*p);
        }
    }
    let last = trimmed.last().copied();
    let end_pt = interp(end_mm);
    if last.is_none_or(|p| (p.coords - end_pt.coords).norm_squared() > 1e-18) {
        trimmed.push(end_pt);
    }
    trimmed
}

/// Detect all open boundary loops on `mesh`, fit a plane to each via
/// SVD, orient the normal outward, and append ear-clipped cap faces
/// that close every detected loop. CSP.4b — runs after
/// [`trim_mesh_along_centerline`] so the trim cuts get sealed.
///
/// Mutates `mesh` in place. Returns the number of loops capped.
///
/// Same projection + winding logic as `build_cleaned_mesh`'s cap
/// step (CSP.3a): boundary vertices are projected onto the fit plane
/// before ear-clip so the cap is exactly planar. Cap faces use the
/// existing loop vertex indices (no new vertices appended for the
/// cap; only re-uses the now-projected boundary positions).
fn auto_cap_open_boundaries(mesh: &mut IndexedMesh) -> usize {
    let loops = detect_boundary_loops(mesh);
    let mut capped = 0;
    for loop_data in &loops {
        if !loop_data.is_valid() {
            continue;
        }
        let loop_points_3d: Vec<Point3<f64>> = loop_data
            .vertices
            .iter()
            .filter_map(|&idx| mesh.vertices.get(idx as usize).copied())
            .collect();
        if loop_points_3d.len() != loop_data.vertices.len() {
            continue;
        }
        let (plane_centroid, plane_normal_raw, _r_squared) = fit_plane_to_points(&loop_points_3d);
        let plane_normal = orient_cap_normal_outward(mesh, plane_centroid, plane_normal_raw);

        // CSP.3a — project loop vertices onto the fit plane so the
        // cap is planar.
        for &idx in &loop_data.vertices {
            if let Some(p) = mesh.vertices.get(idx as usize).copied() {
                let signed = (p.coords - plane_centroid.coords).dot(&plane_normal);
                let projected = p.coords - plane_normal * signed;
                mesh.vertices[idx as usize] = Point3::from(projected);
            }
        }

        let loop_points_projected: Vec<Point3<f64>> = loop_data
            .vertices
            .iter()
            .filter_map(|&idx| mesh.vertices.get(idx as usize).copied())
            .collect();
        let verts_2d =
            project_loop_to_plane_2d(&loop_points_projected, plane_centroid, plane_normal);
        let triangulation = triangulate_polygon_2d_earclip(&verts_2d);

        for tri in triangulation {
            let a = loop_data.vertices[tri[0] as usize];
            let b = loop_data.vertices[tri[1] as usize];
            let c = loop_data.vertices[tri[2] as usize];
            // Same outward-winding flip as `build_cleaned_mesh`.
            let p0 = verts_2d[tri[0] as usize];
            let p1 = verts_2d[tri[1] as usize];
            let p2 = verts_2d[tri[2] as usize];
            let signed_area_2d = (p1.0 - p0.0) * (p2.1 - p0.1) - (p1.1 - p0.1) * (p2.0 - p0.0);
            if signed_area_2d >= 0.0 {
                mesh.faces.push([a, c, b]);
            } else {
                mesh.faces.push([a, b, c]);
            }
        }
        capped += 1;
    }
    capped
}

/// Detect all open boundary loops in `mesh`. Wraps `mesh-repair`'s
/// `detect_holes`, which builds adjacency + walks boundary edges into
/// closed vertex-index loops. Returns a `Vec<BoundaryLoop>` (each
/// loop's `vertices` is an ordered list of vertex indices closing
/// back to vertex 0).
fn detect_boundary_loops(mesh: &IndexedMesh) -> Vec<holes::BoundaryLoop> {
    let adjacency = MeshAdjacency::build(&mesh.faces);
    holes::detect_holes(mesh, &adjacency)
}

/// Least-squares fit a plane to a set of 3D points via SVD on the
/// centered covariance matrix. Returns `(centroid, normal, r_squared)`.
///
/// **Algorithm**: compute the points' centroid, build the covariance
/// matrix `Σ (p - centroid)(p - centroid)^T`, SVD-decompose it. The
/// singular vector corresponding to the smallest singular value is
/// the plane normal (direction of least variance through the cloud).
/// R² is `1.0 - (smallest_singular_value² / sum_of_singular_values²)` —
/// measures how concentrated the variance is in the two larger
/// directions vs. the normal direction. A perfectly planar loop has
/// `smallest = 0` → `R² = 1.0`; a spherical cloud has all three
/// roughly equal → `R² ≈ 2/3`.
///
/// **Caller responsibility**: the returned normal is *unsigned* —
/// the SVD doesn't tell us which side is "outward". Pair with
/// [`orient_cap_normal_outward`] to flip the normal so it points
/// away from the mesh interior.
///
/// Handles degenerate input: < 3 points returns an identity-ish
/// fallback (`+Z` normal, centroid at origin, R² = 0). Real loops
/// have ≥ 3 vertices via mesh-repair's `BoundaryLoop::is_valid`.
fn fit_plane_to_points(points: &[Point3<f64>]) -> (Point3<f64>, Vector3<f64>, f64) {
    if points.len() < 3 {
        return (Point3::origin(), Vector3::new(0.0, 0.0, 1.0), 0.0);
    }
    // Compute centroid.
    let mut sum = Vector3::zeros();
    for p in points {
        sum += p.coords;
    }
    #[allow(clippy::cast_precision_loss)]
    let n = points.len() as f64;
    let centroid_v = sum / n;
    let centroid = Point3::from(centroid_v);

    // Build covariance matrix Σ (p - centroid)(p - centroid)^T.
    let mut cov = Matrix3::<f64>::zeros();
    for p in points {
        let d = p.coords - centroid_v;
        cov += d * d.transpose();
    }

    // SVD: cov = U Σ V^T. For a symmetric matrix, U = V; the columns
    // of U are eigenvectors and the singular values are eigenvalues
    // (non-negative).
    let svd = cov.svd(true, true);
    let singular_values = svd.singular_values;
    // Smallest singular value's index — that's our normal direction.
    let (min_idx, min_sv) =
        if singular_values[2] <= singular_values[1] && singular_values[2] <= singular_values[0] {
            (2, singular_values[2])
        } else if singular_values[1] <= singular_values[0] {
            (1, singular_values[1])
        } else {
            (0, singular_values[0])
        };

    let normal = if let Some(u) = &svd.u {
        let col = u.column(min_idx);
        Vector3::new(col[0], col[1], col[2])
    } else {
        Vector3::new(0.0, 0.0, 1.0)
    };

    // R² = 1 - (min_sv² / sum_sv²). If all singular values are zero
    // (all points coincident), the plane is undefined; fall back to
    // R² = 0.
    let sum_sq = singular_values[0] * singular_values[0]
        + singular_values[1] * singular_values[1]
        + singular_values[2] * singular_values[2];
    let r_squared = if sum_sq > f64::EPSILON {
        1.0 - (min_sv * min_sv) / sum_sq
    } else {
        0.0
    };
    (centroid, normal.normalize(), r_squared)
}

/// Flip `normal` so it points **away from the mesh interior**.
///
/// **Heuristic**: sample mesh vertices, count how many fall on each
/// side of the plane (loop_centroid + normal · t). The side with the
/// MORE vertices is the "interior" (since the mesh extends inward
/// from the loop). The normal should point to the side with FEWER
/// vertices. If the heuristic is ambiguous (~50/50 split — possible
/// when the loop wraps a thin protrusion), the input normal is
/// returned unchanged; user can manually override via the cap panel's
/// manual sub-section (out of MVP scope; deferred).
///
/// Per spec §Architectural decisions §"Cap normal orientation": the
/// triangulated cap winding determines outward direction; we need
/// the normal to face away from the mesh interior so `mesh_sdf`
/// computes correct inside/outside at commit #12.
fn orient_cap_normal_outward(
    mesh: &IndexedMesh,
    plane_centroid: Point3<f64>,
    normal: Vector3<f64>,
) -> Vector3<f64> {
    let mut above: usize = 0;
    let mut below: usize = 0;
    for v in &mesh.vertices {
        let signed = (v.coords - plane_centroid.coords).dot(&normal);
        if signed > 0.0 {
            above += 1;
        } else if signed < 0.0 {
            below += 1;
        }
    }
    // The "mesh-majority side" is the side with more vertices. The
    // outward normal points away from that side.
    if above > below { -normal } else { normal }
}

/// Build a cap-loop record from a detected boundary loop: fits its
/// plane, orients the normal outward, decides the default per-loop
/// include flag based on vertex count.
fn build_detected_cap_loop(mesh: &IndexedMesh, loop_data: &holes::BoundaryLoop) -> DetectedCapLoop {
    let loop_points: Vec<Point3<f64>> = loop_data
        .vertices
        .iter()
        .map(|&idx| mesh.vertices[idx as usize])
        .collect();
    let (plane_centroid, plane_normal_raw, r_squared) = fit_plane_to_points(&loop_points);
    let plane_normal = orient_cap_normal_outward(mesh, plane_centroid, plane_normal_raw);

    // Spec §Panel specifications §6: default-check loops with vertex
    // count ≥ 8 (small loops are usually acceptable holes or scanner
    // artifacts).
    let include = loop_data.vertices.len() >= 8;

    DetectedCapLoop {
        vertex_indices: loop_data.vertices.clone(),
        plane_centroid,
        plane_normal,
        plane_fit_r_squared: r_squared,
        include,
    }
}

/// Approximate the scan's centerline via cross-section centroids
/// along a "spine" axis. Option-3 hedge: validates the centerline
/// algorithm cf-cast needs for curve-following multi-piece molds.
///
/// **Algorithm**: pick a spine direction (the first cap loop's
/// outward normal — points from cup-mouth toward the appendage tip).
/// Project every mesh vertex onto this axis to get its "depth"; bin
/// vertices into `n_slices` slabs covering the depth range; take the
/// 3D centroid of each slab → polyline of centroids.
///
/// For an elongated, mostly-tubular scan (the prosthetic-appendage
/// case): each slab's centroid is roughly where the spine sits at
/// that depth. The polyline follows the natural curve. Fails on
/// branching geometry (multi-finger hands, multi-limb torsos) where
/// a single centroid can't represent multiple branches; those cases
/// would need a real medial-axis algorithm (MCF / Voronoi). Not in
/// MVP scope.
///
/// **Empty / degenerate input**: returns `Vec::new()` if the mesh
/// has no vertices or the spine direction is zero-magnitude; the
/// drawing system then skips the centerline overlay.
fn compute_centerline_polyline(
    mesh: &IndexedMesh,
    spine_direction: Vector3<f64>,
    n_slices: usize,
) -> Vec<Point3<f64>> {
    if mesh.vertices.is_empty() || spine_direction.norm_squared() < f64::EPSILON {
        return Vec::new();
    }
    let axis = spine_direction.normalize();

    // Single pass: collect depth + 3D position per vertex.
    let depths: Vec<f64> = mesh.vertices.iter().map(|v| axis.dot(&v.coords)).collect();
    let min_d = depths.iter().copied().fold(f64::INFINITY, f64::min);
    let max_d = depths.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let range = max_d - min_d;
    if range < f64::EPSILON {
        return Vec::new();
    }
    #[allow(clippy::cast_precision_loss)]
    let slab_width = range / (n_slices as f64);

    let mut polyline: Vec<Point3<f64>> = Vec::with_capacity(n_slices);
    for i in 0..n_slices {
        #[allow(clippy::cast_precision_loss)]
        let lo = min_d + (i as f64) * slab_width;
        let hi = lo + slab_width;
        let mut sum = Vector3::zeros();
        let mut count: usize = 0;
        for (j, v) in mesh.vertices.iter().enumerate() {
            if depths[j] >= lo && depths[j] < hi {
                sum += v.coords;
                count += 1;
            }
        }
        if count > 0 {
            #[allow(clippy::cast_precision_loss)]
            let centroid = sum / (count as f64);
            polyline.push(Point3::from(centroid));
        }
    }
    polyline
}

/// Project a physics-frame mesh-local point through the live
/// Reorient + Recenter + render_scale + up-axis-swap pipeline into
/// Bevy world-space, with rotation pivoted around the scan's raw
/// AABB centroid (Bevy-frame, post-swap, post-scale —
/// [`OverlayLengths::bbox_center_bevy`]).
///
/// Mirror of [`apply_world_transform_to_scan_entity`]'s Transform
/// composition so cap-overlay loop vertices + centerline polyline
/// points track the mesh exactly. Used by `draw_cap_overlays`.
fn project_mesh_local_to_world(
    physics_local: &Point3<f64>,
    up: UpAxis,
    reorient_rotation_bevy: Quat,
    recenter_world: Vec3,
    render_scale: f32,
    bbox_center_bevy: Vec3,
) -> Vec3 {
    let bevy_local_scaled = Vec3::from_array(up.to_bevy_point(physics_local)) * render_scale;
    // R * (bevy_local_scaled - c) + c + recenter
    let centered = bevy_local_scaled - bbox_center_bevy;
    let rotated_centered = reorient_rotation_bevy * centered;
    recenter_world + rotated_centered + bbox_center_bevy
}

/// Triangulate a 2D simple polygon via ear-clipping. Input: ordered
/// polygon vertices (CCW or CW; algorithm reverses on the fly if
/// signed area is negative). Output: triangles as index triplets
/// into the input vertex list. Always closes the polygon (last
/// edge connects vertex `n-1` back to vertex `0`).
///
/// Per spec §Panel specifications §6: cap polygons triangulated via
/// inline ear-clipping with **fan-fallback** when ear-clipping fails
/// (degenerate / self-intersecting polygons after projection). Fan
/// triangulation from vertex 0 produces valid faces for any
/// star-shaped polygon, which boundary loops typically are.
///
/// **Complexity**: O(n²) for the ear-clip path. For a 2609-vertex
/// loop (iter-1 fixture's open boundary): ~6.8M ops; ~5-20 ms. Fast
/// enough at save time. Larger loops (10k+) would benefit from
/// Delaunay or constrained-Delaunay; out of scope until iter-1
/// surfaces pathological cases.
///
/// **Fan fallback trigger**: ear-clipping stops finding ears (no
/// ear is convex + empty) before reducing to 3 vertices. Either the
/// polygon is non-simple (self-intersecting) or near-degenerate. Fan
/// from vertex 0 always works for star-shaped polygons; produces
/// possibly-thin triangles but no NaN / inverted faces.
fn triangulate_polygon_2d_earclip(verts_2d: &[(f64, f64)]) -> Vec<[u32; 3]> {
    let n = verts_2d.len();
    if n < 3 {
        return Vec::new();
    }

    // Signed area (shoelace) — negative → CW; reverse to get CCW.
    let signed_area: f64 = (0..n)
        .map(|i| {
            let (x0, y0) = verts_2d[i];
            let (x1, y1) = verts_2d[(i + 1) % n];
            x0 * y1 - x1 * y0
        })
        .sum::<f64>()
        * 0.5;

    // `working` holds the indices into the input array, in CCW
    // order. We splice ears out of this list as we go.
    #[allow(clippy::cast_possible_truncation)]
    let mut working: Vec<u32> = if signed_area >= 0.0 {
        (0..n as u32).collect()
    } else {
        (0..n as u32).rev().collect()
    };

    let mut triangles: Vec<[u32; 3]> = Vec::with_capacity(n.saturating_sub(2));

    // Ear-clip loop. Each iteration finds one ear, emits a triangle,
    // and removes the ear-tip index from `working`.
    while working.len() > 3 {
        let mut found_ear = false;
        let m = working.len();
        for i in 0..m {
            let prev_idx = working[(i + m - 1) % m];
            let curr_idx = working[i];
            let next_idx = working[(i + 1) % m];
            let prev = verts_2d[prev_idx as usize];
            let curr = verts_2d[curr_idx as usize];
            let next = verts_2d[next_idx as usize];

            // Is the triangle (prev, curr, next) convex in CCW order?
            // Cross-product z-component: positive → CCW (convex);
            // negative or zero → reflex / collinear, skip.
            let cross =
                (curr.0 - prev.0) * (next.1 - prev.1) - (curr.1 - prev.1) * (next.0 - prev.0);
            if cross <= 0.0 {
                continue;
            }

            // Does any other polygon vertex lie strictly inside the
            // ear triangle? If yes, not an ear; skip.
            let any_inside = working
                .iter()
                .enumerate()
                .filter(|(j, _)| *j != (i + m - 1) % m && *j != i && *j != (i + 1) % m)
                .map(|(_, &k)| verts_2d[k as usize])
                .any(|p| point_in_triangle_2d(p, prev, curr, next));
            if any_inside {
                continue;
            }

            triangles.push([prev_idx, curr_idx, next_idx]);
            working.remove(i);
            found_ear = true;
            break;
        }

        if !found_ear {
            // Degenerate / self-intersecting input — fall back to
            // fan triangulation over the remaining `working` indices.
            let anchor = working[0];
            for k in 1..(working.len() - 1) {
                triangles.push([anchor, working[k], working[k + 1]]);
            }
            return triangles;
        }
    }

    // Final triangle from the last 3 remaining vertices.
    if working.len() == 3 {
        triangles.push([working[0], working[1], working[2]]);
    }
    triangles
}

/// Standard 2D point-in-triangle test via barycentric sign check.
/// Returns `true` if `p` is strictly inside the triangle `(a, b, c)`.
/// Points on edges return `false` to avoid spurious ear rejection
/// from shared boundary vertices.
fn point_in_triangle_2d(p: (f64, f64), a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> bool {
    let d1 = (p.0 - b.0) * (a.1 - b.1) - (a.0 - b.0) * (p.1 - b.1);
    let d2 = (p.0 - c.0) * (b.1 - c.1) - (b.0 - c.0) * (p.1 - c.1);
    let d3 = (p.0 - a.0) * (c.1 - a.1) - (c.0 - a.0) * (p.1 - a.1);
    let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(has_neg && has_pos)
}

/// Project a 3D loop onto its fit plane and return 2D coordinates
/// for ear-clipping. Picks an arbitrary orthonormal basis in the
/// plane (Gram-Schmidt against a non-parallel world axis); the
/// orientation of the 2D frame doesn't matter because the
/// ear-clip handles CW/CCW input automatically.
fn project_loop_to_plane_2d(
    loop_points_3d: &[Point3<f64>],
    plane_centroid: Point3<f64>,
    plane_normal: Vector3<f64>,
) -> Vec<(f64, f64)> {
    // Pick a world axis that isn't parallel to `plane_normal`, then
    // project it orthogonal to the normal → first basis vector `u`.
    // `v = normal × u` → second basis vector.
    let world_axis = if plane_normal.x.abs() < 0.9 {
        Vector3::new(1.0, 0.0, 0.0)
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    };
    let u = (world_axis - plane_normal * world_axis.dot(&plane_normal)).normalize();
    let v = plane_normal.cross(&u).normalize();

    loop_points_3d
        .iter()
        .map(|p| {
            let d = p.coords - plane_centroid.coords;
            (u.dot(&d), v.dot(&d))
        })
        .collect()
}

/// Build the cleaned IndexedMesh from the working scan + Reorient +
/// Recenter + included cap loops + (CSP.2, 2026-05-15) the Clip floor
/// state when enabled.
///
/// Pipeline:
/// 1. Clone `scan` into `out`.
/// 2. Bake rotation + translation into vertex positions (in place).
///    After this step `out.vertices` are in world frame.
/// 3. For each included cap loop, ear-clip its 2D projection and
///    append triangles (using existing loop vertex indices — no new
///    vertices added). Cap faces inherit the world-frame coordinates
///    from step 2.
/// 4. If `clip.enabled`, apply `clip_mesh_against_world_z` in world
///    frame (identity rotation + zero translation since `out` is
///    already in world frame). Drops faces below `z = clip.z_mm` and
///    generates new intersection vertices on the clip plane for
///    crossing triangles. Cap faces above the clip plane survive;
///    cap faces below are dropped along with the underlying mesh.
///
/// **Cap normal orientation**: the spec mandates that the cap's
/// outward normal point away from the mesh interior. We orient the
/// triangulation's vertex winding to match the loop's stored
/// `plane_normal` (which `build_detected_cap_loop` already flipped
/// to face outward).
///
/// **Why clip lands AFTER cap triangulation, not before**: the cap
/// loops were detected on the pre-bake mesh; their vertex indices
/// reference positions in `out` that are now in world frame. Running
/// clip first would re-index `out`'s vertex buffer, invalidating the
/// cap-loop indices. Caps-then-clip preserves indexing.
///
/// **Why clip does NOT re-cap**: the clip cut produces a planar open
/// boundary on the kept side. v1.0 leaves that uncapped — workshop
/// iter-1 will use the cleaned cap from `[caps]` (above the clip
/// plane) for the rim; the clipped bottom is the "open" side that
/// downstream cf-cast (curve-following multi-piece) handles via the
/// pour gate, not via SDF. A future v1.1 could re-detect + re-cap
/// post-clip if iter-1 shows that's needed.
fn build_cleaned_mesh(
    scan: &IndexedMesh,
    reorient: &ReorientState,
    recenter: &RecenterState,
    clip: &ClipState,
    cap: &CapState,
) -> IndexedMesh {
    let rotation = reorient.quaternion_physics();
    let translation = Vector3::new(
        recenter.tx_mm * 0.001,
        recenter.ty_mm * 0.001,
        recenter.tz_mm * 0.001,
    );
    // Pivot the bake rotation around the raw scan AABB centroid so
    // rotation rotates the mesh in place (matches the viewport's
    // centroid-pivot Transform composition). Empty mesh → centroid
    // at origin (matches `IndexedMesh::aabb()` returning
    // `Aabb::empty()`); the loop below is a no-op anyway.
    let centroid = scan.aabb().center();

    let mut out = scan.clone();

    // Step 2: bake transforms. Compute centroid-pivoted rotation +
    // recenter translation for each vertex from the ORIGINAL
    // (pre-bake) coordinates so cap triangulation can still reference
    // the original positions for plane fit etc. We then mutate
    // `out.vertices` in place.
    for v in out.vertices.iter_mut() {
        *v = bake_vertex_with_pivot(v, rotation, &centroid, translation);
    }

    // Step 3: triangulate + append included caps. Loop vertex
    // indices were captured at scan time + reference positions in
    // `scan.0.vertices` — which after our in-place transform are
    // now in world frame. So both the original-loop-positions math
    // (for the 2D projection) AND the resulting face indices stay
    // valid.
    for cap_loop in &cap.loops {
        if !cap_loop.include {
            continue;
        }
        // Loop's stored `plane_centroid` and `plane_normal` are in
        // pre-bake physics-local frame. Bake centroid through the
        // same pivot transform as the vertices; rotate the plane
        // normal (a direction, no pivot or translation).
        let centroid_world =
            bake_vertex_with_pivot(&cap_loop.plane_centroid, rotation, &centroid, translation);
        let normal_world = rotation.transform_vector(&cap_loop.plane_normal);

        // CSP.3a — project each loop vertex onto the fit plane
        // BEFORE the ear-clip. The boundary loop's points have
        // sub-mm wobble for typical R² ≈ 0.9 fixtures; without this
        // projection the resulting cap faces lie OFF the plane
        // (because ear-clip's 2D projection is into the plane, but
        // the final 3D faces draw from the unprojected vertex
        // positions). Snapping the rim onto the plane keeps the cap
        // planar at the cost of moving the scan-body rim ring by
        // the wobble magnitude — sub-mm, well below the 2 mm SDF
        // cell + below the meshopt target_error threshold. The
        // alternative (append separate cap vertices) leaves a
        // sub-mm seam gap that breaks watertightness for mesh_sdf.
        // All loop indices are validated below; we project only the
        // ones that exist in the current `out.vertices`.
        for &idx in &cap_loop.vertex_indices {
            if let Some(p) = out.vertices.get(idx as usize).copied() {
                let signed = (p.coords - centroid_world.coords).dot(&normal_world);
                let projected = p.coords - normal_world * signed;
                out.vertices[idx as usize] = Point3::from(projected);
            }
        }

        let loop_points_world: Vec<Point3<f64>> = cap_loop
            .vertex_indices
            .iter()
            .filter_map(|&idx| out.vertices.get(idx as usize).copied())
            .collect();
        if loop_points_world.len() != cap_loop.vertex_indices.len() {
            // Some indices invalid (mesh changed after scan); skip
            // this loop rather than emit malformed faces.
            continue;
        }

        let verts_2d = project_loop_to_plane_2d(&loop_points_world, centroid_world, normal_world);
        let triangulation = triangulate_polygon_2d_earclip(&verts_2d);

        // Map ear-clip's local indices (into the loop) back to
        // global mesh vertex indices.
        for tri in triangulation {
            let a = cap_loop.vertex_indices[tri[0] as usize];
            let b = cap_loop.vertex_indices[tri[1] as usize];
            let c = cap_loop.vertex_indices[tri[2] as usize];
            // Determine winding from the triangulation's 2D
            // orientation + match the outward normal. If 2D CCW
            // (positive cross) the triangle faces +normal in 3D;
            // we want -normal (outward). Flip if needed.
            let p0 = verts_2d[tri[0] as usize];
            let p1 = verts_2d[tri[1] as usize];
            let p2 = verts_2d[tri[2] as usize];
            let signed_area_2d = (p1.0 - p0.0) * (p2.1 - p0.1) - (p1.1 - p0.1) * (p2.0 - p0.0);
            if signed_area_2d >= 0.0 {
                // CCW in 2D → triangle faces +normal in 3D. We want
                // outward = away-from-mesh-interior, which is
                // -normal_world (because orient_cap_normal_outward
                // already flipped normal to point outward; the cap
                // surface NORMAL points outward, so the triangle's
                // GEOMETRIC normal needs to match → 2D-CCW order
                // gives +normal, we need -normal, so reverse).
                out.faces.push([a, c, b]);
            } else {
                out.faces.push([a, b, c]);
            }
        }
    }

    // Step 4: clip-floor bake (CSP.2). When the user enabled the Clip
    // panel toggle, `out` is in world frame (transforms + caps baked
    // in steps 2/3) so the clip operates on world `z = clip.z_mm` with
    // identity rotation + zero translation. The algorithm is the same
    // one unit-tested at commit #8 — wired here, not at #8, because
    // the v2 cf-cast pivot (curve-following molds) made the clip
    // workflow-optional rather than mandatory.
    if clip.enabled {
        let clip_z_m = clip.z_mm * 0.001;
        out = clip_mesh_against_world_z(&out, UnitQuaternion::identity(), 0.0, clip_z_m);
    }

    out
}

/// Mesh hygiene cleanup pass applied at save time (CSP.3b).
///
/// Before this function existed the cleaned STL inherited the raw
/// scan's vertex layout — 3N unshared verts from `mesh_io::load_stl`,
/// no degenerate-triangle strip, no smallest-component drop — plus
/// whatever new geometry the cap + clip steps appended. Downstream
/// `simplify_decoder` choked on that (cf-device-design
/// `main.rs:419-425` documents the iter-1 fixture retaining its full
/// 3.34M face count under topology-preserving simplification, forcing
/// a switch to `simplify_sloppy_decoder`).
///
/// This pass runs in `handle_save_action` between `build_cleaned_mesh`
/// and the save-time simplify (slice 9.8). The fix-set lands here,
/// not inside `build_cleaned_mesh`, so the cap-construction concern
/// (loop-vertex projection) stays isolated from the disk-hygiene
/// concern, and so unit-test fixtures with tiny face counts don't
/// have to fight the `min_component_faces` cutoff.
///
/// Pipeline:
///
///   1. **weld_vertices** — collapse 3N STL-unshared verts to ~N
///      shared so collapse-edge algorithms can find topology.
///   2. **remove_degenerate_triangles** — drop FP-noise zero-area
///      triangles from cap ear-clip + clip intersection slivers.
///   3. **remove_small_components** — drop scanner-noise islands
///      (`< CLEANUP_MIN_COMPONENT_FACES` per shell). Spec assumes
///      the user pre-trimmed to single shell; this is the safety net.
///   4. **remove_unreferenced_vertices** — tighten memory layout
///      after the other passes orphan vertices.
///
/// Order matters: weld first (degenerate detection needs shared
/// indices), then degenerate, then component analysis (so components
/// don't bridge via zero-area triangles), then unreferenced cleanup
/// last.
///
/// Mutates `mesh` in place. Returns a [`CleanupReport`] summarizing
/// each step's effect so the Save status message can surface how
/// aggressive the cleanup was for the user's confidence.
fn cleanup_cleaned_mesh_for_disk(mesh: &mut IndexedMesh) -> CleanupReport {
    let welded = weld_vertices(mesh, SIMPLIFY_WELD_EPSILON_M);
    let degenerate = remove_degenerate_triangles(mesh, CLEANUP_DEGENERATE_AREA_M2);
    let small_components = remove_small_components(mesh, CLEANUP_MIN_COMPONENT_FACES);
    let unreferenced = remove_unreferenced_vertices(mesh);
    CleanupReport {
        welded,
        degenerate,
        small_components,
        unreferenced,
    }
}

/// Per-pass counts from [`cleanup_cleaned_mesh_for_disk`].
///
/// Sums are surfaced in the Save panel status message when non-zero,
/// so the user can tell whether the cleanup pass did meaningful work
/// (workshop-iter-1 fixture: high counts mean the raw scan needed
/// hygiene help; future cleaned scans approaching ~zero counts
/// indicate the pre-prep pipeline is producing cleaner inputs).
#[derive(Debug, Clone, Copy, Default)]
struct CleanupReport {
    welded: usize,
    degenerate: usize,
    small_components: usize,
    unreferenced: usize,
}

impl CleanupReport {
    /// Total operations across all four passes. Zero when the input
    /// was already disk-ready (no cleanup needed).
    fn total(self) -> usize {
        self.welded + self.degenerate + self.small_components + self.unreferenced
    }
}

// ----- .prep.toml serializable structures -----
//
// Block names match `docs/SCAN_PREP_DESIGN.md` §Output format (v1.0
// completion rename CSP.1, 2026-05-15). Earlier as-built used
// `[reorient]` / `[recenter]` — those names exposed internal egui-panel
// nouns rather than the conceptual transform operation. Downstream
// consumers (cf-cast-cli `prep.rs`, cf-device-design `parse_centerline`)
// read only `[centerline]` and tolerate unknown sibling keys, so this
// rename is downstream-safe.

#[derive(Serialize)]
struct PrepToml {
    scan_prep: PrepScanPrepBlock,
    simplify: PrepSimplifyBlock,
    transform: PrepTransformBlock,
    clip: PrepClipBlock,
    caps: PrepCapsBlock,
    #[serde(skip_serializing_if = "Option::is_none")]
    centerline: Option<PrepCenterlineBlock>,
    centerline_trim: PrepCenterlineTrimBlock,
    output: PrepOutputBlock,
}

#[derive(Serialize)]
struct PrepScanPrepBlock {
    source_stl: String,
    tool_version: &'static str,
    generated_at: String,
    stl_units_at_load: &'static str,
    /// Physics-frame translation auto-applied at load (CSP.3.5) so
    /// the scan's AABB centroid lands at origin. Provenance only —
    /// the cleaned STL is in the auto-centered frame (plus the
    /// user's transforms). To reconstruct the source-frame position
    /// of a cleaned-STL vertex, INVERT the user's Reorient +
    /// Recenter recorded in `[transform]`, then add this offset.
    auto_center_offset_m: [f64; 3],
    /// Auto-PCA rotation applied at load (CSP.4a) — quaternion in
    /// `(w, x, y, z)` order that takes the source's principal axis
    /// to `+Z`. Skipped (serialized as `null`) when PCA was
    /// degenerate (rare). Pairs with `auto_center_offset_m` for
    /// full source-frame reconstruction.
    #[serde(skip_serializing_if = "Option::is_none")]
    auto_pca_quaternion: Option<[f64; 4]>,
}

/// `[simplify]` provenance — what decimation, if any, was applied at
/// save time. Per spec §Output format. Records both the user-targeted
/// budget (slice 9.8: Simplify panel slider IS the save-time budget)
/// and the actually-achieved face count, plus the originally-loaded
/// count so the reduction ratio is visible.
#[derive(Serialize)]
struct PrepSimplifyBlock {
    /// `true` when meshopt was invoked at save time (target < pre-
    /// simplify cleaned-mesh face count). `false` when the cleaned
    /// mesh's face count was already at or below the target (no
    /// decimation; achieved == original).
    applied: bool,
    /// Algorithm identifier — pinned literal so downstream audits know
    /// what code path produced this file.
    algorithm: &'static str,
    /// Version string for the algorithm dependency. Tracks the
    /// workspace `meshopt` Cargo dep; bump this constant in lockstep
    /// when that dep is updated. Hard-coded literal because the
    /// meshopt-rs crate doesn't expose its version at runtime and a
    /// build-script lift for one provenance line would be overkill.
    algorithm_version: &'static str,
    /// Target face count from the Simplify panel slider at save time.
    target_face_count: usize,
    /// Actual face count of the cleaned mesh on disk. Equals
    /// `target_face_count` ± boundary-locked vertex topology slack
    /// when `applied`; equals `original_face_count` when `!applied`.
    achieved_face_count: usize,
    /// Face count of the as-loaded scan (in `OriginalScanMesh`,
    /// before any decimation or transforms). Distinct from
    /// `achieved_face_count` whenever simplify ran at all.
    original_face_count: usize,
    /// Always `true` — cf-scan-prep uses
    /// `meshopt::SimplifyOptions::LockBorder` (NOT
    /// `simplify_sloppy`) per spec §Architectural decisions §Simplify
    /// algorithm. Surfaced in the TOML for audit purposes.
    boundary_preserved: bool,
}

/// `[transform]` umbrella — `rotation` + `translation` sub-tables
/// match spec §Output format. Each renders as `[transform.rotation]`
/// + `[transform.translation]` in TOML.
#[derive(Serialize)]
struct PrepTransformBlock {
    rotation: PrepRotationBlock,
    translation: PrepTranslationBlock,
}

#[derive(Serialize)]
struct PrepRotationBlock {
    /// Physics-frame unit quaternion `(w, x, y, z)`. Source of truth
    /// for downstream reconstruction; the Euler angles below mirror
    /// the cf-scan-prep slider source-of-truth for human readability.
    quaternion: [f64; 4],
    roll_deg: f64,
    pitch_deg: f64,
    yaw_deg: f64,
}

#[derive(Serialize)]
struct PrepTranslationBlock {
    /// Physics-frame translation in meters. Spec §Output format
    /// names this `m`; the panel state stores mm but the on-disk
    /// units convention is meters (matches cf-cast / the rest of
    /// the workspace).
    m: [f64; 3],
}

#[derive(Serialize)]
struct PrepClipBlock {
    enabled: bool,
    /// World-frame Z height in meters.
    z_m: f64,
    /// Cached drop percentage at save time.
    drop_pct: f64,
    /// `true` when the Clip toggle was enabled at save time and
    /// `clip_mesh_against_world_z` ran on the cleaned mesh (CSP.2
    /// wired this; before CSP.2 the clip was advisory-only and this
    /// flag was hard-coded `false`). Equals `enabled` exactly under
    /// the current implementation; kept as a separate field so a
    /// future "advisory-mode" toggle can break the equality cleanly.
    baked: bool,
}

#[derive(Serialize)]
struct PrepCapsBlock {
    /// Whether ear-clipping was applied at save time to close the
    /// included loops. Always `true` if any loops were detected
    /// AND included.
    applied: bool,
    loops: Vec<PrepCapLoop>,
}

#[derive(Serialize)]
struct PrepCapLoop {
    loop_index: usize,
    vertex_count: usize,
    plane_fit_r_squared: f64,
    /// Physics-frame outward normal at scan time (pre-transform bake).
    plane_normal: [f64; 3],
    /// Physics-frame centroid at scan time.
    plane_centroid_m: [f64; 3],
    included: bool,
}

#[derive(Serialize)]
struct PrepCenterlineBlock {
    /// Polyline in **post-bake, post-trim world-frame meters**
    /// (matches the cleaned STL's coordinate system; v2 cf-cast
    /// consumes directly). CSP.4b — when the user dialed centerline
    /// trim, this is the polyline between the trim cut planes, not
    /// the full pre-trim polyline.
    points_m: Vec<[f64; 3]>,
    algorithm: &'static str,
}

/// `[centerline_trim]` provenance — what user-driven centerline
/// trim was applied at save time (CSP.4b). Always emitted, even
/// when no trim was requested (the explicit `0.0 / 0.0` record
/// makes "saved without trim" indistinguishable from an audit
/// perspective).
#[derive(Serialize)]
struct PrepCenterlineTrimBlock {
    trim_tip_mm: f64,
    trim_floor_mm: f64,
    /// Number of boundary loops auto-capped after the trim cuts.
    /// For a "trim only the floor end" save on a closed-tip sock,
    /// this is 1; for "trim both ends" it's 2; for "no trim" it's
    /// 0.
    capped_loops: usize,
}

#[derive(Serialize)]
struct PrepOutputBlock {
    cleaned_stl: String,
    /// AABB of the cleaned mesh on disk (post-transform, post-cap,
    /// post-save-time-simplify), in meters. Spec §Output format
    /// promised this so downstream tooling can sanity-check the
    /// cleaned scan extents without re-loading the STL.
    aabb_m: PrepAabbBlock,
}

#[derive(Serialize)]
struct PrepAabbBlock {
    min: [f64; 3],
    max: [f64; 3],
}

/// Algorithm identifier for `[simplify].algorithm`. Pinned literal so
/// downstream audits can distinguish cf-scan-prep's boundary-preserving
/// quadric collapse from other decimation strategies (e.g.,
/// cf-device-design's `simplify_sloppy` proxy).
const SIMPLIFY_ALGORITHM_NAME: &str = "meshopt_quadric_edge_collapse";

/// Tracks the workspace `meshopt` Cargo dep. Update in lockstep with
/// `Cargo.toml`'s `meshopt = "X.Y.Z"`.
const SIMPLIFY_ALGORITHM_VERSION: &str = "0.6.2";

/// Build the `.prep.toml` string from the current cf-scan-prep state.
/// Includes provenance for every transform / cap / centerline /
/// simplify-at-save so v2 cf-cast (or a future audit) can reconstruct
/// what cf-scan-prep did to produce the cleaned STL.
///
/// `pivot_centroid_m` is the raw scan AABB centroid (in physics-frame
/// meters). The centerline polyline is baked through the same
/// centroid-pivot transform `bake_vertex_with_pivot` uses for mesh
/// vertices, so the polyline coordinates emitted into `.prep.toml`
/// agree with the cleaned STL on disk.
///
/// `simplify_target_face_count`, `original_face_count`, and
/// `cleaned_aabb_m` feed the spec-promised `[simplify]` and
/// `[output.aabb_m]` blocks. The caller computes them from the live
/// `SimplifyState` / `OriginalScanMesh` / final cleaned mesh AABB so
/// this function stays pure (no Res lookups).
#[allow(clippy::too_many_arguments)]
fn build_prep_toml_string(
    source_stl: &Path,
    stl_units: StlUnits,
    auto_center_offset_m: Vector3<f64>,
    auto_pca_quat: Option<UnitQuaternion<f64>>,
    reorient: &ReorientState,
    recenter: &RecenterState,
    clip: &ClipState,
    cap: &CapState,
    centerline_trim: &CenterlineTrimState,
    centerline_trim_capped: usize,
    cleaned_stl_name: &str,
    rotation_for_centerline: UnitQuaternion<f64>,
    translation_for_centerline_m: Vector3<f64>,
    pivot_centroid_m: Point3<f64>,
    simplify_target_face_count: usize,
    original_face_count: usize,
    achieved_face_count: usize,
    cleaned_aabb_m: &Aabb,
) -> Result<String> {
    let q = reorient.quaternion_physics();
    let timestamp = chrono_like_timestamp();

    // Project the centerline polyline into world frame so v2 cf-cast
    // can consume it directly without redoing transform math. Same
    // centroid-pivot transform as `build_cleaned_mesh` uses for mesh
    // vertices. CSP.4b — if the user dialed trim, emit the
    // POST-TRIM polyline so the TOML record matches what's actually
    // in the cleaned STL on disk.
    let baked_polyline: Vec<Point3<f64>> = cap
        .centerline_polyline
        .iter()
        .map(|p| {
            bake_vertex_with_pivot(
                p,
                rotation_for_centerline,
                &pivot_centroid_m,
                translation_for_centerline_m,
            )
        })
        .collect();
    let trimmed_polyline = trim_centerline_polyline(
        &baked_polyline,
        centerline_trim.trim_tip_mm,
        centerline_trim.trim_floor_mm,
    );
    let centerline_world: Vec<[f64; 3]> =
        trimmed_polyline.iter().map(|p| [p.x, p.y, p.z]).collect();

    let toml_struct = PrepToml {
        scan_prep: PrepScanPrepBlock {
            source_stl: source_stl.display().to_string(),
            tool_version: env!("CARGO_PKG_VERSION"),
            generated_at: timestamp,
            stl_units_at_load: stl_units.panel_label(),
            auto_center_offset_m: [
                auto_center_offset_m.x,
                auto_center_offset_m.y,
                auto_center_offset_m.z,
            ],
            auto_pca_quaternion: auto_pca_quat.map(|q| [q.w, q.i, q.j, q.k]),
        },
        simplify: PrepSimplifyBlock {
            // `applied = true` iff save-time simplify ran. The save
            // handler invokes simplify only when the slider target is
            // strictly below the current cleaned-mesh face count;
            // mirror that condition here for the provenance flag.
            applied: simplify_target_face_count < original_face_count
                && achieved_face_count < original_face_count,
            algorithm: SIMPLIFY_ALGORITHM_NAME,
            algorithm_version: SIMPLIFY_ALGORITHM_VERSION,
            target_face_count: simplify_target_face_count,
            achieved_face_count,
            original_face_count,
            boundary_preserved: true,
        },
        transform: PrepTransformBlock {
            rotation: PrepRotationBlock {
                quaternion: [q.w, q.i, q.j, q.k],
                roll_deg: reorient.roll_deg,
                pitch_deg: reorient.pitch_deg,
                yaw_deg: reorient.yaw_deg,
            },
            translation: PrepTranslationBlock {
                m: [
                    recenter.tx_mm * 0.001,
                    recenter.ty_mm * 0.001,
                    recenter.tz_mm * 0.001,
                ],
            },
        },
        clip: PrepClipBlock {
            enabled: clip.enabled,
            z_m: clip.z_mm * 0.001,
            drop_pct: clip.drop_pct,
            // CSP.2: `baked` tracks reality. `build_cleaned_mesh`
            // applies the clip iff `clip.enabled`, so the two flags
            // line up exactly.
            baked: clip.enabled,
        },
        caps: PrepCapsBlock {
            applied: cap.loops.iter().any(|l| l.include),
            loops: cap
                .loops
                .iter()
                .enumerate()
                .map(|(i, cl)| PrepCapLoop {
                    loop_index: i,
                    vertex_count: cl.vertex_indices.len(),
                    plane_fit_r_squared: cl.plane_fit_r_squared,
                    plane_normal: [cl.plane_normal.x, cl.plane_normal.y, cl.plane_normal.z],
                    plane_centroid_m: [
                        cl.plane_centroid.x,
                        cl.plane_centroid.y,
                        cl.plane_centroid.z,
                    ],
                    included: cl.include,
                })
                .collect(),
        },
        centerline: if centerline_world.is_empty() {
            None
        } else {
            Some(PrepCenterlineBlock {
                points_m: centerline_world,
                algorithm: "cross_section_centroids",
            })
        },
        centerline_trim: PrepCenterlineTrimBlock {
            trim_tip_mm: centerline_trim.trim_tip_mm,
            trim_floor_mm: centerline_trim.trim_floor_mm,
            capped_loops: centerline_trim_capped,
        },
        output: PrepOutputBlock {
            cleaned_stl: cleaned_stl_name.to_string(),
            aabb_m: PrepAabbBlock {
                min: [
                    cleaned_aabb_m.min.x,
                    cleaned_aabb_m.min.y,
                    cleaned_aabb_m.min.z,
                ],
                max: [
                    cleaned_aabb_m.max.x,
                    cleaned_aabb_m.max.y,
                    cleaned_aabb_m.max.z,
                ],
            },
        },
    };
    toml::to_string_pretty(&toml_struct).context("serialize PrepToml to TOML")
}

/// Minimal RFC 3339-ish timestamp without pulling in `chrono`. Uses
/// the OS clock + `std::time::SystemTime` for a UTC-shaped string
/// good enough for provenance. If the clock returns an error
/// (extremely unusual), falls back to a placeholder.
fn chrono_like_timestamp() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    // Plain seconds-since-epoch (the user's timezone is irrelevant
    // for provenance — they can convert if needed). Format as ISO
    // 8601 UTC by computing a calendar date from the unix timestamp.
    iso8601_utc_from_unix_seconds(secs)
}

/// Convert a unix timestamp (seconds since 1970-01-01 UTC) into an
/// ISO 8601 UTC string like `2026-05-12T22:34:00Z`. Inline because
/// the only alternative is pulling in `chrono` for one function.
fn iso8601_utc_from_unix_seconds(unix_secs: u64) -> String {
    // Days since epoch.
    let days = unix_secs / 86_400;
    let secs_in_day = unix_secs % 86_400;
    let hours = secs_in_day / 3600;
    let minutes = (secs_in_day % 3600) / 60;
    let seconds = secs_in_day % 60;

    // Walk forward from 1970-01-01 day-by-day. Slow for far-future
    // dates but trivial for "now". Use a calendar table.
    let (year, month, day) = unix_days_to_ymd(days as i64);
    format!("{year:04}-{month:02}-{day:02}T{hours:02}:{minutes:02}:{seconds:02}Z")
}

/// Convert "days since 1970-01-01" to a (year, month, day) tuple.
/// Algorithm: Howard Hinnant's civil-from-days; well-known + branch-
/// less. ~30 LOC inline, avoids the `chrono` dep.
fn unix_days_to_ymd(z: i64) -> (i64, u32, u32) {
    // Shift so the "year 0" anchor is March 1 of year 0 (so leap
    // days fall at year boundaries cleanly).
    let z_shifted = z + 719_468;
    let era = if z_shifted >= 0 {
        z_shifted / 146_097
    } else {
        (z_shifted - 146_096) / 146_097
    };
    let doe = (z_shifted - era * 146_097) as u64; // [0, 146096]
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365; // [0, 399]
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // [0, 365]
    let mp = (5 * doy + 2) / 153; // [0, 11]
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32; // [1, 31]
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32; // [1, 12]
    let year = if m <= 2 { y + 1 } else { y };
    (year, m, d)
}

/// Atomic two-file write: writes `cleaned_stl_path` + `prep_toml_path`
/// to `.tmp` siblings first, then renames both to final names. If
/// either step fails, BOTH `.tmp` files are cleaned up so the user
/// doesn't end up with a half-written set. Spec §Architectural
/// decisions §Save atomicity.
fn atomic_write_save(
    cleaned_mesh: &IndexedMesh,
    cleaned_stl_path: &Path,
    prep_toml_path: &Path,
    prep_toml_content: &str,
) -> Result<()> {
    let stl_tmp = cleaned_stl_path.with_extension("stl.tmp");
    let toml_tmp = prep_toml_path.with_extension("toml.tmp");

    // STL write (binary; cleaned scans are large — text STL would
    // 10x the file size with no benefit).
    save_stl(cleaned_mesh, &stl_tmp, true)
        .with_context(|| format!("writing {}", stl_tmp.display()))?;
    // TOML write.
    if let Err(e) = std::fs::write(&toml_tmp, prep_toml_content) {
        // Roll back STL tmp; surface the TOML error.
        let _ = std::fs::remove_file(&stl_tmp);
        return Err(anyhow::Error::new(e).context(format!("writing {}", toml_tmp.display())));
    }
    // Atomic renames.
    if let Err(e) = std::fs::rename(&stl_tmp, cleaned_stl_path) {
        let _ = std::fs::remove_file(&stl_tmp);
        let _ = std::fs::remove_file(&toml_tmp);
        return Err(anyhow::Error::new(e).context(format!("renaming {}", stl_tmp.display())));
    }
    if let Err(e) = std::fs::rename(&toml_tmp, prep_toml_path) {
        // STL already landed; remove it to keep atomicity contract
        // ("neither final file lands if either write fails").
        let _ = std::fs::remove_file(cleaned_stl_path);
        let _ = std::fs::remove_file(&toml_tmp);
        return Err(anyhow::Error::new(e).context(format!("renaming {}", toml_tmp.display())));
    }
    Ok(())
}

/// Compute the **post-Reorient** AABB of the raw scan AABB in
/// physics-frame **millimeters**. Used by the Recenter panel's
/// `[Center origin]` and `[Floor -> z=0]` click handlers to figure out
/// where the rotated mesh actually sits in space.
///
/// Walks the 8 corners of the raw AABB, rotates each by `rot_physics`
/// (no translation applied — translation is what we're trying to
/// compute), and accumulates the rotated min/max. Returns mm bounds
/// because the Recenter sliders + status messages all work in mm
/// (workshop convention; matches the Scan Info panel's AABB display
/// units).
fn rotated_aabb_around_centroid_physics_mm(
    rot_physics: UnitQuaternion<f64>,
    raw_aabb_m: &Aabb,
) -> (Vector3<f64>, Vector3<f64>) {
    // Pivot rotation around the raw AABB centroid: corner_centered =
    // corner - centroid, rotated_centered = R * corner_centered,
    // world = rotated_centered + centroid. This keeps the mesh
    // ROTATING IN PLACE — the AABB CENTER stays at `centroid * 1000`
    // mm regardless of `rot_physics`; only the AABB extents rotate.
    // Without this pivot the [Center origin] / [Floor -> z=0] click
    // handlers would compute the FULL swing of the off-origin mesh
    // through space, which the user perceives as "rotation moves the
    // model" rather than "rotation rotates the model in place".
    let centroid = raw_aabb_m.center();
    let lo = raw_aabb_m.min;
    let hi = raw_aabb_m.max;
    let mut min_mm = Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max_mm = Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for &x in &[lo.x, hi.x] {
        for &y in &[lo.y, hi.y] {
            for &z in &[lo.z, hi.z] {
                let centered_mm = Vector3::new(
                    (x - centroid.x) * 1000.0,
                    (y - centroid.y) * 1000.0,
                    (z - centroid.z) * 1000.0,
                );
                let rotated_centered = rot_physics.transform_vector(&centered_mm);
                let world_mm = Vector3::new(
                    rotated_centered.x + centroid.x * 1000.0,
                    rotated_centered.y + centroid.y * 1000.0,
                    rotated_centered.z + centroid.z * 1000.0,
                );
                min_mm.x = min_mm.x.min(world_mm.x);
                min_mm.y = min_mm.y.min(world_mm.y);
                min_mm.z = min_mm.z.min(world_mm.z);
                max_mm.x = max_mm.x.max(world_mm.x);
                max_mm.y = max_mm.y.max(world_mm.y);
                max_mm.z = max_mm.z.max(world_mm.z);
            }
        }
    }
    (min_mm, max_mm)
}

/// Bake a physics-frame point through Reorient + Recenter with the
/// rotation pivoted at `centroid` (the scan's raw AABB centroid).
/// Returns the world-frame point:
///
/// ```text
/// world = rotation * (point - centroid) + centroid + translation_m
/// ```
///
/// The centroid pivot is what makes rotation feel like "rotate in
/// place" — without it, rotating an off-origin mesh swings the
/// centroid through space because the bake formula pivots at the
/// physics-frame origin by default. The pivot compensation
/// `(I - R) * centroid` is folded into the bake formula uniformly
/// so the cleaned STL on disk, the viewport mesh + wireframe, and
/// the cap / centerline overlays all agree.
fn bake_vertex_with_pivot(
    point: &Point3<f64>,
    rotation: UnitQuaternion<f64>,
    centroid: &Point3<f64>,
    translation_m: Vector3<f64>,
) -> Point3<f64> {
    let centered = point.coords - centroid.coords;
    let rotated_centered = rotation.transform_vector(&centered);
    Point3::from(rotated_centered + centroid.coords + translation_m)
}

/// Compute the principal-axis rotation that aligns a scan's
/// long axis with `+Z` (the cast-frame demolding axis per
/// [`SCAN_UP_AXIS`]).
///
/// PCA on the mean-centered vertex positions: 3×3 covariance,
/// symmetric eigendecomposition, principal eigenvector = the
/// largest-variance direction (the "long axis" of the scan). The
/// returned [`UnitQuaternion`] is the shortest rotation that maps
/// that principal axis to `+Z`.
///
/// Sign convention: the principal eigenvector is determined only
/// up to sign. This helper picks the side whose `z` component is
/// non-negative (i.e., the side already closer to `+Z`) — the
/// resulting rotation is therefore at most 90° from identity.
/// Users who want the opposite end pointed up flip post-hoc with
/// a 180° follow-on rotation (e.g. by setting roll = 180°).
///
/// Returns `None` for:
/// - fewer than 3 vertices (PCA underdetermined)
/// - degenerate mesh whose covariance has all near-zero
///   eigenvalues (all vertices coincident; no principal axis)
///
/// **Resolves cf-scan-prep deferred item §5 "Auto-PCA initial
/// orientation guess"** from `docs/SCAN_PREP_DESIGN.md` — the
/// manual-slider Reorient panel introduces tilt that propagates
/// downstream to the cleaned STL + centerline + (via cf-cast-cli)
/// the mold geometry. Auto-PCA gives a deterministic starting
/// orientation that the user can then nudge with the existing
/// sliders.
fn compute_pca_orientation(vertices: &[Point3<f64>]) -> Option<UnitQuaternion<f64>> {
    if vertices.len() < 3 {
        return None;
    }

    // Mean-center.
    let n = vertices.len() as f64;
    let mut centroid = Vector3::zeros();
    for v in vertices {
        centroid += v.coords;
    }
    centroid /= n;

    // 3×3 covariance accumulation (Σ d · d^T / n). Symmetric by
    // construction, so we use SymmetricEigen for the
    // eigendecomposition below.
    let mut cov = Matrix3::zeros();
    for v in vertices {
        let d = v.coords - centroid;
        cov += d * d.transpose();
    }
    cov /= n;

    let eigen = cov.symmetric_eigen();

    // Largest eigenvalue's column = principal axis.
    let (mut max_i, mut max_val) = (0_usize, eigen.eigenvalues[0]);
    for i in 1..3 {
        if eigen.eigenvalues[i] > max_val {
            max_i = i;
            max_val = eigen.eigenvalues[i];
        }
    }
    // Degeneracy guard: all-zero (or all near-zero) eigenvalues =
    // no meaningful principal direction (e.g. all vertices
    // coincident). Threshold relative to the largest eigenvalue
    // scale; absolute 1e-30 m² catches the all-zeros case
    // independently.
    if max_val <= 1e-30 {
        return None;
    }
    let mut principal: Vector3<f64> = eigen.eigenvectors.column(max_i).into_owned();

    // Sign pick: orient the principal axis toward `+Z` so the
    // resulting rotation is the SHORTEST rotation (≤ 90°).
    if principal.z < 0.0 {
        principal = -principal;
    }

    // `rotation_between` returns `None` for anti-parallel inputs;
    // the sign-pick above ensures `principal.z >= 0`, so the
    // anti-parallel case (principal == -Z) is precluded. Identity
    // case (principal == +Z, already aligned) yields the identity
    // quaternion.
    UnitQuaternion::rotation_between(&principal, &Vector3::z())
}

/// Pre-computed always-on viewport overlay geometry, cached at startup
/// (`docs/SCAN_PREP_DESIGN.md` §Visualization layer). Combines:
///
/// - Axis gizmo arrow length (Bevy world units) — sized to the
///   rendered scan's diagonal so the arrows are visible relative to
///   the mesh without overwhelming it.
/// - Pre-swap-and-scale scan AABB center + dimensions in Bevy-local
///   space — fed to `gizmos.cuboid(...)` each Update tick + transformed
///   by the live Reorient rotation so the wireframe rotates with the
///   mesh.
///
/// All three fields are immutable post-construction (raw AABB never
/// changes; render_scale never changes); the user-mutable transforms
/// (rotation now, translation + clip + caps in commits #7-#9) are
/// applied per-frame on top of this static base.
#[derive(Resource, Debug, Clone, Copy)]
struct OverlayLengths {
    /// Length of each axis arrow drawn from origin, in Bevy world
    /// units (which equal physics meters × `render_scale`). Sized to
    /// `0.6 × scaled_diagonal` so the tips poke just past the
    /// unrotated bbox extents — visible against the mesh, not lost in
    /// the corner.
    arrow_length: f32,
    /// Center of the unrotated scan AABB in Bevy-local space (post
    /// `UpAxis::PlusZ` swap, scaled by `render_scale`). Per-frame
    /// wireframe Transform applies `mesh_rotation * this` as its
    /// translation so the wireframe rotates around the world origin
    /// in lockstep with the mesh entity.
    bbox_center_bevy: Vec3,
    /// Full dimensions of the unrotated scan AABB in Bevy-local space
    /// (post `UpAxis::PlusZ` swap, scaled by `render_scale`). Fed
    /// directly as the wireframe Transform's `scale` — gizmos.cuboid
    /// draws a unit cube and the Transform stretches it to the bbox
    /// extents.
    bbox_dims_bevy: Vec3,
    /// Raw scan AABB in physics meters (post unit conversion, pre
    /// `render_scale` lift, pre up-axis swap). Cached so the Recenter
    /// panel's `[Center origin]` / `[Floor -> z=0]` click handlers
    /// can compute the post-Reorient bbox bounds without re-walking
    /// the 10M-vertex scan on every click.
    raw_aabb_m: Aabb,
    /// Per-axis slider range for the Recenter panel, in millimeters
    /// (`±2 × max(raw_aabb)` per spec §Panel specifications §4).
    /// Scan-derived because the slider's "reasonable" extent depends
    /// on the scan's own size — a 130 mm-tall scan wants ~±260 mm
    /// slack to recenter freely; a 1 m-tall body part wants ~±2000 mm.
    recenter_slider_range_mm: f64,
}

/// Convert a physics-frame rotation quaternion to the Bevy-frame
/// equivalent under the `UpAxis::PlusZ` swap.
///
/// `UpAxis::PlusZ` maps physics `(x, y, z)` → bevy `(x, z, y)` — a
/// parity-flipping permutation (det = -1). Conjugating a rotation by
/// such a transformation transforms its axis by the swap **and**
/// negates the angle (axes transform as pseudovectors under improper
/// changes of basis). In quaternion components this collapses to a
/// scalar formula:
///
/// - `(w, x, y, z)` in physics → `(w, -x, -z, -y)` in bevy
///
/// Derivation: for `q = (w, sin(θ/2) * a)` representing rotation `θ`
/// about axis `a`, the bevy-frame equivalent is
/// `(w, sin(-θ/2) * M_pb·a) = (w, -sin(θ/2) * (a_x, a_z, a_y))`. The
/// matrix-conjugation form `R_bevy = M_pb · R_physics · M_pb^-1`
/// agrees on the matrix side; this scalar form is equivalent and
/// cheaper.
fn physics_quat_to_bevy_for_plus_z(q: UnitQuaternion<f64>) -> Quat {
    // f64 → f32 cast is intentional for Bevy.
    #[allow(clippy::cast_possible_truncation)]
    Quat::from_xyzw(-q.i as f32, -q.k as f32, -q.j as f32, q.w as f32)
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
        Ok((scan_mesh, auto_center_offset_m, auto_pca_quat)) => {
            let scan_info = ScanInfo::from_loaded(&cli.path, &scan_mesh, cli.stl_units);
            // Clone the loaded mesh into `OriginalScanMesh` so the
            // Simplify panel's `[Reset to original]` action can restore
            // unsimplified geometry without re-reading the STL.
            let original = OriginalScanMesh(scan_mesh.clone());
            let output_dir = SaveOutputDir(resolve_output_dir(&cli));
            let source = SourceStlPath(cli.path.clone());
            let stl_units = StlUnitsResource(cli.stl_units);
            let auto_center = AutoCenterOffset(auto_center_offset_m);
            let auto_pca = AutoPcaQuaternion(auto_pca_quat);
            run_render_app(
                scan_mesh,
                original,
                scan_info,
                source,
                output_dir,
                stl_units,
                auto_center,
                auto_pca,
            );
        }
        Err(err) => {
            let msg = format!("Failed to load {}: {err:#}", cli.path.display());
            eprintln!("{msg}");
            run_error_overlay_app(msg);
        }
    }
    Ok(())
}

/// Resolve the Save-panel output directory from CLI args. Honors
/// `--output-dir <path>` when supplied; else falls back to the input
/// scan's parent directory. Spec §Architectural decisions §Save
/// behavior.
///
/// **Fallback when `--output-dir` is absent AND `cli.path` has no
/// parent** (extremely rare: only happens when the user passes a
/// bare filename like `scan.stl` with no directory component on a
/// system where the parent resolves to empty — practically never).
/// Falls back to the current working directory.
fn resolve_output_dir(cli: &Cli) -> PathBuf {
    if let Some(dir) = &cli.output_dir {
        return dir.clone();
    }
    cli.path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."))
}

/// Load the STL at `cli.path`, convert vertex coordinates into meters
/// per `cli.stl_units`, **auto-center the scan at the physics origin**
/// (CSP.3.5), and **auto-orient the principal axis to +Z** (CSP.4a).
/// Returns the loaded+centered+oriented mesh plus the auto-center
/// offset (in meters) plus the auto-PCA quaternion (or `None` if the
/// PCA was degenerate) so the saved `.prep.toml` can preserve both
/// transforms as provenance.
///
/// # Why auto-center + auto-PCA at load
///
/// CSP.4 redesign collapses the spec's manual Reorient + Recenter +
/// Clip workflow into a centerline-driven trim flow. For that to be
/// usable, the scan's frame must be canonical-on-load:
///
/// - **Auto-center**: AABB centroid → physics origin (CSP.3.5;
///   scanners drop captured geometry at arbitrary offsets from
///   their internal frame; without this, every downstream number
///   carries that offset).
/// - **Auto-PCA**: principal axis → `+Z` cast-frame demolding
///   direction. After this, the centerline runs roughly along
///   `+Z` (slightly tilted for asymmetric scans), the
///   tip-vs-floor distinction is unambiguous, and the user's
///   centerline-trim sliders operate on intuitive distances along
///   that axis.
///
/// # What gets recorded
///
/// `[scan_prep].auto_center_offset_m` + `auto_pca_quaternion` in
/// the saved `.prep.toml` capture both auto-transforms. To
/// reconstruct the source-frame position of a cleaned-STL vertex,
/// invert the user transforms (recorded elsewhere), then invert the
/// auto-PCA, then subtract auto_center_offset_m.
///
/// # Errors
///
/// Bubbles up [`mesh_io::load_stl`]'s I/O + parse errors verbatim.
/// The caller wraps the message into the error-overlay path.
fn try_load_scan(cli: &Cli) -> Result<(IndexedMesh, Vector3<f64>, Option<UnitQuaternion<f64>>)> {
    let mut mesh = load_stl(&cli.path)?;
    scale_vertices_in_place(&mut mesh, cli.stl_units.to_meters_factor());
    let auto_center_offset_m = auto_center_in_place(&mut mesh);
    let auto_pca_quat = auto_pca_in_place(&mut mesh);
    Ok((mesh, auto_center_offset_m, auto_pca_quat))
}

/// Apply the PCA-derived rotation to all vertices in `mesh`, taking
/// the principal axis to `+Z`. Returns the quaternion that was
/// applied (or `None` if PCA was degenerate — coincident vertices,
/// < 3 verts, or all-zero covariance).
///
/// Operates on the mesh after auto-center, so the rotation pivot is
/// the origin (= scan centroid). The result has its principal axis
/// aligned with cast-frame `+Z`.
///
/// Skips when the resulting quaternion is near-identity (within 1
/// µrad sin(θ/2) of identity) — the rotation would be a no-op
/// modulo FP drift, and skipping the vertex walk keeps already-
/// upright fixtures bit-exact.
fn auto_pca_in_place(mesh: &mut IndexedMesh) -> Option<UnitQuaternion<f64>> {
    let q = compute_pca_orientation(&mesh.vertices)?;
    // Near-identity check: |q.i|² + |q.j|² + |q.k|² < 1e-12 means
    // the vector part is sub-µrad; rotation is effectively identity.
    let vec_sq = q.i * q.i + q.j * q.j + q.k * q.k;
    if vec_sq < 1e-12 {
        return Some(q);
    }
    for v in &mut mesh.vertices {
        *v = q.transform_point(v);
    }
    Some(q)
}

/// Translate `mesh`'s vertices so the AABB centroid lands at physics
/// origin. Returns the offset that was applied (`= -aabb.center()`
/// from the input mesh in meters).
///
/// No-op for an empty mesh (no vertices to walk). When `centroid`
/// is already at origin, returns near-zero offset and skips the
/// walk to avoid FP-drift on bit-exact-centered fixtures.
fn auto_center_in_place(mesh: &mut IndexedMesh) -> Vector3<f64> {
    if mesh.vertices.is_empty() {
        return Vector3::zeros();
    }
    let centroid = mesh.aabb().center();
    // FP-drift guard: if the centroid is already within 1 µm of the
    // origin, skip the walk — we'd be moving vertices by sub-FP-
    // precision amounts that meshopt couldn't see anyway.
    let centroid_v = centroid.coords;
    if centroid_v.norm() < 1e-6 {
        return Vector3::zeros();
    }
    let offset = -centroid_v;
    for v in &mut mesh.vertices {
        v.x += offset.x;
        v.y += offset.y;
        v.z += offset.z;
    }
    offset
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
///
/// # Precondition: `target_face_count < original.faces.len()`
///
/// meshopt is a reduction-only algorithm; its C++ side asserts
/// `target_index_count <= index_count` in `meshopt_simplifyEdge`
/// (simplifier.cpp:2286) and SIGABRTs the process if violated. We
/// defensively early-return the cloned input unchanged when the target
/// is at or above the current face count, so callers that forget the
/// precondition see a no-op + finite `elapsed_secs` instead of a
/// process abort. `handle_simplify_actions` pre-checks this case and
/// surfaces a user-facing status message before invoking us; the guard
/// here is belt-and-suspenders.
fn simplify_mesh(original: &IndexedMesh, target_face_count: usize) -> SimplifyResult {
    let start = Instant::now();

    if target_face_count >= original.faces.len() {
        return SimplifyResult {
            mesh: original.clone(),
            elapsed_secs: start.elapsed().as_secs_f64(),
        };
    }

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
#[allow(clippy::too_many_arguments)]
fn run_render_app(
    scan_mesh: IndexedMesh,
    original: OriginalScanMesh,
    scan_info: ScanInfo,
    source: SourceStlPath,
    output_dir: SaveOutputDir,
    stl_units: StlUnitsResource,
    auto_center: AutoCenterOffset,
    auto_pca: AutoPcaQuaternion,
) {
    #[allow(clippy::cast_possible_truncation)] // f64 → f32 is intentional for Bevy.
    let raw_diagonal = scan_mesh.aabb().diagonal() as f32;
    let render_scale = compute_render_scale(raw_diagonal);
    println!(
        "loaded {} vertices, {} faces; bbox diagonal = {raw_diagonal:.4} m; \
         render_scale = {render_scale:.2}×",
        scan_mesh.vertices.len(),
        scan_mesh.faces.len(),
    );

    let overlays = build_overlay_lengths(&scan_mesh, render_scale, SCAN_UP_AXIS);

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
        .insert_resource(ReorientState::default())
        .insert_resource(RecenterState::default())
        .insert_resource(ClipState::default())
        .insert_resource(CapState::default())
        .insert_resource(CapPendingAction::default())
        .insert_resource(CenterlineTrimState::default())
        .insert_resource(SavePendingAction::default())
        .insert_resource(source)
        .insert_resource(output_dir)
        .insert_resource(stl_units)
        .insert_resource(auto_center)
        .insert_resource(auto_pca)
        .insert_resource(StatusBar::default())
        .insert_resource(overlays)
        .add_systems(Startup, (setup_render_scene, init_status_for_load))
        // Apply/Reset handler runs before auto-clear so a newly-set
        // status's TTL is checked against the same tick's `Time`.
        // `apply_world_transform_to_scan_entity` is idempotent (no-op
        // when the desired Bevy rotation / translation already match
        // the entity's current Transform) so order vs. the others
        // doesn't change steady-state behavior. The earlier split
        // between reorient + recenter systems merged into one because
        // the centroid-pivot compensation couples translation to
        // rotation.
        .add_systems(
            Update,
            (
                block_orbit_input_when_over_egui.before(orbit_camera_input),
                handle_simplify_actions,
                apply_world_transform_to_scan_entity,
                update_clip_drop_pct,
                handle_cap_actions,
                handle_save_action,
                mark_cap_stale_on_transform_change,
                auto_clear_status,
                draw_reference_overlays,
                draw_cap_overlays,
                draw_trim_plane_overlays,
                exit_on_esc,
            ),
        )
        .add_systems(EguiPrimaryContextPass, scan_prep_panel)
        .run();
}

/// Pre-compute the axis arrow length + Bevy-frame bbox center/dims for
/// the always-on viewport overlays. Called once in `run_render_app`
/// after the raw scan AABB + render_scale are known; the resulting
/// values are immutable for the app's lifetime (user-mutable transforms
/// stack on top per-frame via `draw_reference_overlays`).
fn build_overlay_lengths(scan: &IndexedMesh, render_scale: f32, up: UpAxis) -> OverlayLengths {
    let raw_aabb = scan.aabb();
    let scaled_aabb = scale_aabb(&raw_aabb, render_scale);

    // Center + half-extents in physics frame.
    let center_physics = scaled_aabb.center();
    // Bevy-local frame: apply the UpAxis swap so the overlay aligns
    // with the spawned mesh's vertex positions (which are also
    // post-swap via `spawn_face_mesh`).
    let center_bevy = Vec3::from_array(up.to_bevy_point(&center_physics));

    // Dimensions transform under the swap as a vector. We construct a
    // direction-only Point3 with each axis carrying the corresponding
    // bbox dimension; the swap permutes the components the same way it
    // permutes coordinate axes. (The f64 → f32 narrowing happens
    // inside `up.to_bevy_point` — no `as` cast in this statement.)
    let dims_physics_pt = mesh_types::Point3::new(
        scaled_aabb.max.x - scaled_aabb.min.x,
        scaled_aabb.max.y - scaled_aabb.min.y,
        scaled_aabb.max.z - scaled_aabb.min.z,
    );
    let dims_bevy = Vec3::from_array(up.to_bevy_point(&dims_physics_pt));

    #[allow(clippy::cast_possible_truncation)] // f64 → f32 for Bevy.
    let scaled_diagonal = scaled_aabb.diagonal() as f32;

    // Spec §Panel specifications §4: Recenter sliders run from
    // `-2 × max(raw_aabb)` to `+2 × max(raw_aabb)` in mm. Multiplying
    // by 1000 converts the raw meter extent to mm; the larger of the
    // three axis extents bounds the slider so even the longest axis
    // has comfortable +/- slack.
    let raw_dx_mm = (raw_aabb.max.x - raw_aabb.min.x) * 1000.0;
    let raw_dy_mm = (raw_aabb.max.y - raw_aabb.min.y) * 1000.0;
    let raw_dz_mm = (raw_aabb.max.z - raw_aabb.min.z) * 1000.0;
    let recenter_slider_range_mm = 2.0 * raw_dx_mm.max(raw_dy_mm).max(raw_dz_mm);

    OverlayLengths {
        arrow_length: scaled_diagonal * 0.6,
        bbox_center_bevy: center_bevy,
        bbox_dims_bevy: dims_bevy,
        raw_aabb_m: raw_aabb,
        recenter_slider_range_mm,
    }
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
/// On Apply (target < current face count): runs [`simplify_mesh`],
/// updates `ScanMesh` + `ScanInfo` vertex/face counts, despawns the
/// existing `ScanMeshEntity` and respawns from the simplified mesh,
/// sets a `Normal`-kind status message with TTL
/// `STATUS_NORMAL_TTL_SECS` per spec §Panel specifications §2.
///
/// On Apply (target >= current face count): no-op for the mesh +
/// entity (meshopt can't add faces; running it with target > current
/// would SIGABRT). Surfaces a `Warning`-kind status message naming the
/// current/target counts and pointing at `[Reset to original]` as the
/// workflow recovery, then returns.
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

    // `current` (not "original"): the face count of `scan.0` AT THE
    // START OF THIS HANDLER. After a prior Apply, scan.0 holds the
    // simplified mesh, so `current` is the post-prior-simplify count
    // (not the unsimplified loaded mesh — that lives in
    // `original: Res<OriginalScanMesh>`). Naming as "current" to avoid
    // the two-meanings-of-original confusion inside this function.
    let current_face_count = scan.0.faces.len();
    let target = simplify_state.target_face_count;

    // Pre-check the Apply no-op case. Two reasons we treat
    // `target >= current` as a no-op rather than calling meshopt:
    //
    // 1. Crash prevention: meshopt is reduction-only. Its C++ side
    //    asserts `target_index_count <= index_count` in
    //    `meshopt_simplifyEdge` (simplifier.cpp:2286) and SIGABRTs the
    //    process if violated. So `target > current` would abort.
    // 2. UX: even `target == current` is wasted work (meshopt would
    //    early-exit with the input unchanged, but the user-facing
    //    achievement message "Reduced 200k -> 200k faces in 0.5s"
    //    reads as a broken button). Treating equality as a no-op
    //    keeps the messaging honest.
    //
    // The user-friendly workflow when they want *more* faces back is
    // `[Reset to original]` first.
    if matches!(action, SimplifyAction::Apply) && target >= current_face_count {
        status.text = format!(
            "Already at {} faces (target {} is not lower). Use [Reset to original] first if you want a less-decimated mesh.",
            human_count(current_face_count),
            human_count(target),
        );
        status.kind = StatusKind::Warning;
        status.auto_clear_at_secs = Some(time.elapsed_secs_f64() + STATUS_NORMAL_TTL_SECS);
        return;
    }

    let (new_mesh, status_text) = match action {
        SimplifyAction::Apply => {
            let result = simplify_mesh(&scan.0, target);
            let achieved = result.mesh.faces.len();
            let text = format!(
                "Reduced {} -> {} faces in {:.1}s",
                human_count(current_face_count),
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

/// Update system: project the current `ReorientState` + `RecenterState`
/// onto the `ScanMeshEntity`'s Bevy `Transform` component every tick,
/// pivoting the rotation around the scan's raw AABB centroid so the
/// mesh visually rotates in place.
///
/// Bevy's `Transform` composes as `world_pos = translation + rotation *
/// (local_pos * scale)`. To pivot rotation around the centroid (in
/// scaled bevy-local space, that's [`OverlayLengths::bbox_center_bevy`])
/// we solve for the translation that keeps the centroid fixed under
/// rotation:
///
/// ```text
/// world_centroid = T + R * c_bevy  (Bevy's formula)
/// pivot-rotation wants: world_centroid = recenter_world + c_bevy
/// →  T = recenter_world + (I - R) * c_bevy
/// ```
///
/// Cheap by design: per-entity equality check; only writes when
/// rotation or translation differs from the existing Transform. Runs
/// every Update tick regardless of input-`is_changed` flags because
/// `ScanMeshEntity` can be re-spawned by `handle_simplify_actions`
/// with an identity Transform; this system then restores the live
/// transforms on the next tick automatically.
///
/// Replaces the prior split `apply_reorient_to_transform` +
/// `apply_recenter_to_transform` systems — those wrote disjoint
/// channels (rotation / translation) of the same component, but the
/// centroid-pivot compensation introduces a rotation-dependency in
/// the translation channel, so the two channels are no longer
/// independent and the systems merge into one.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn apply_world_transform_to_scan_entity(
    reorient: Res<ReorientState>,
    recenter: Res<RecenterState>,
    overlays: Res<OverlayLengths>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut entities: Query<&mut Transform, With<ScanMeshEntity>>,
) {
    let rotation_bevy = physics_quat_to_bevy_for_plus_z(reorient.quaternion_physics());
    let recenter_world = recenter.translation_world(*up, render_scale.0);
    // (I - R) * c_bevy = pivot compensation.
    let pivot_compensation = overlays.bbox_center_bevy - rotation_bevy * overlays.bbox_center_bevy;
    let target_translation = recenter_world + pivot_compensation;
    for mut transform in &mut entities {
        if transform.rotation != rotation_bevy {
            transform.rotation = rotation_bevy;
        }
        if transform.translation != target_translation {
            transform.translation = target_translation;
        }
    }
}

/// Update system: draw the always-on viewport reference overlays each
/// frame (`docs/SCAN_PREP_DESIGN.md` §Visualization layer).
///
/// At commit #6 this renders:
///
/// 1. **Three colored axis arrows** at world origin, sized to
///    `OverlayLengths::arrow_length`. The colors match the
///    cast-frame convention (X = red, Y = green, Z = blue) but the
///    on-screen directions follow the `UpAxis::PlusZ` swap that
///    `spawn_face_mesh` applies to the mesh: red points right
///    (cast +X = Bevy +X), blue points up (cast +Z = Bevy +Y,
///    the demolding direction), green points into the screen
///    (cast +Y = Bevy +Z). The blue "up" arrow is the user's
///    visual reference for "is the mesh aligned to the cast +Z
///    demolding axis?"
/// 2. **A wireframe of the rotated scan AABB** — the unrotated
///    Bevy-local bbox center + dims (cached in `OverlayLengths`),
///    rotated by the current `ReorientState` quaternion in lockstep
///    with the mesh entity. As the user drags Reorient sliders the
///    wireframe rotates live with the mesh, so when its edges become
///    parallel to the axis arrows the mesh is rotation-aligned to
///    the world frame.
///
/// Banked but not yet rendered (future polish if workshop iteration
/// shows it's needed): ground grid at `z=0`. The iter-1 fixture's
/// natural origin sits inside the scan, so a `z=0` grid wouldn't be
/// usefully visible until commit #7's Recenter panel translates the
/// cavity floor to the origin.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
#[allow(clippy::too_many_arguments)] // Bevy systems take each Res / Gizmos as a separate param.
fn draw_reference_overlays(
    overlays: Res<OverlayLengths>,
    reorient: Res<ReorientState>,
    recenter: Res<RecenterState>,
    clip: Res<ClipState>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut gizmos: Gizmos,
) {
    let l = overlays.arrow_length;

    // X axis (cast +X) — red. Bevy +X is screen-right.
    gizmos.arrow(Vec3::ZERO, Vec3::X * l, Color::srgb(1.0, 0.30, 0.30));
    // Y axis (cast +Y) — green. UpAxis::PlusZ swap maps physics +Y to
    // Bevy +Z (depth into screen).
    gizmos.arrow(Vec3::ZERO, Vec3::Z * l, Color::srgb(0.30, 1.0, 0.30));
    // Z axis (cast +Z; demolding direction) — blue. UpAxis::PlusZ
    // swap maps physics +Z to Bevy +Y (screen up).
    gizmos.arrow(Vec3::ZERO, Vec3::Y * l, Color::srgb(0.40, 0.60, 1.00));

    // Transformed bbox wireframe — composes the live Reorient rotation
    // + Recenter translation under centroid-pivot rotation, matching
    // `apply_world_transform_to_scan_entity`. The wireframe center sits
    // at the mesh centroid in world frame regardless of rotation
    // (centroid is the pivot point); only the wireframe orientation +
    // its translation under Recenter change.
    let rotation = physics_quat_to_bevy_for_plus_z(reorient.quaternion_physics());
    let recenter_world = recenter.translation_world(*up, render_scale.0);
    let wireframe_transform = Transform {
        translation: recenter_world + overlays.bbox_center_bevy,
        rotation,
        scale: overlays.bbox_dims_bevy,
    };
    gizmos.cube(wireframe_transform, Color::srgb(0.55, 0.55, 0.60));

    // Clip-plane disc — a wireframe rectangle in the cast-frame XY
    // plane at world `z = clip.z_mm`, indicating where the (eventual)
    // clip will happen. Rendered only when the panel toggle is on so
    // a disabled clip doesn't add visual clutter. Drawn as 4 gizmo
    // lines forming a rectangle (Bevy gizmos can't fill polygons;
    // gizmos.cube would be a 3D box not a flat plane).
    if clip.enabled {
        // Convert clip_z from cast mm to bevy world units: mm × 0.001
        // (meters) × render_scale. cast +Z = Bevy +Y under the swap.
        #[allow(clippy::cast_possible_truncation)] // f64 → f32 for Bevy.
        let clip_y_bevy = (clip.z_mm * 0.001) as f32 * render_scale.0;
        // Half-extent in Bevy world units. Use the largest Bevy bbox
        // axis so the disc clearly extends past the mesh regardless of
        // current rotation orientation.
        let half = overlays
            .bbox_dims_bevy
            .x
            .max(overlays.bbox_dims_bevy.y)
            .max(overlays.bbox_dims_bevy.z)
            * 0.75;
        // Rectangle corners in Bevy XZ plane at y = clip_y_bevy.
        let p00 = Vec3::new(-half, clip_y_bevy, -half);
        let p10 = Vec3::new(half, clip_y_bevy, -half);
        let p11 = Vec3::new(half, clip_y_bevy, half);
        let p01 = Vec3::new(-half, clip_y_bevy, half);
        let disc_color = Color::srgb(0.95, 0.45, 0.20);
        gizmos.line(p00, p10, disc_color);
        gizmos.line(p10, p11, disc_color);
        gizmos.line(p11, p01, disc_color);
        gizmos.line(p01, p00, disc_color);
        // Also draw the diagonals so the disc reads as a translucent
        // surface rather than just a wireframe outline.
        gizmos.line(p00, p11, disc_color);
        gizmos.line(p10, p01, disc_color);
    }
}

/// Update system: handle the Cap panel's `[Scan]` action. Reads the
/// `CapPendingAction::rescan` flag; on `true`, runs boundary loop
/// detection on the current `ScanMesh`, fits planes, orients normals,
/// computes the centerline polyline, and writes everything to
/// `CapState`. Idempotent — does nothing when no scan is pending.
///
/// **Why not run on every state change**: detection + plane fits +
/// centerline are O(N) over the mesh; can be slow on un-simplified
/// scans (~50 ms+ for 10M verts). Running on user-explicit trigger
/// keeps the UI responsive; the staleness pattern (re-Scan to
/// refresh) makes the user-explicit cadence visible.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn handle_cap_actions(
    mut pending: ResMut<CapPendingAction>,
    mut cap: ResMut<CapState>,
    scan: Res<ScanMesh>,
    mut status: ResMut<StatusBar>,
    time: Res<Time>,
) {
    if !pending.rescan {
        return;
    }
    pending.rescan = false;

    let raw_loops = detect_boundary_loops(&scan.0);
    cap.loops = raw_loops
        .iter()
        .filter(|loop_data| loop_data.is_valid())
        .map(|loop_data| build_detected_cap_loop(&scan.0, loop_data))
        .collect();

    // Centerline: use the first cap loop's outward normal as the
    // spine direction. If no loops detected (closed mesh — unusual
    // for cf-scan-prep input but possible), skip the centerline.
    cap.centerline_polyline = if let Some(first_loop) = cap.loops.first() {
        compute_centerline_polyline(&scan.0, first_loop.plane_normal, 30)
    } else {
        Vec::new()
    };

    cap.stale = false;

    let n_loops = cap.loops.len();
    let centerline_msg = if cap.centerline_polyline.is_empty() {
        String::new()
    } else {
        format!(
            "; centerline = {} segments",
            cap.centerline_polyline.len().saturating_sub(1)
        )
    };
    status.text = format!(
        "Scanned: {n_loops} boundary loop{}{centerline_msg}",
        if n_loops == 1 { "" } else { "s" },
    );
    status.kind = StatusKind::Normal;
    status.auto_clear_at_secs = Some(time.elapsed_secs_f64() + STATUS_NORMAL_TTL_SECS);
}

/// Update system: set `CapState::stale = true` when Reorient,
/// Recenter, Clip, or `ScanMesh` changes after a scan has populated
/// `CapState::loops`. Spec §Panel specifications §6: the loop
/// indices and plane fits are tied to the mesh as-it-was at scan
/// time; subsequent transforms / simplify / clip operations invalidate
/// them. CSP.2 added Clip to the watch list because the clip now
/// bakes into the cleaned STL — a clip-then-save cycle without a
/// re-Scan would emit cap faces that reference pre-clip positions.
///
/// **Cheap check**: only reads change flags and writes a single bool
/// via `bypass_change_detection` so we don't re-trigger ourselves.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn mark_cap_stale_on_transform_change(
    mut cap: ResMut<CapState>,
    scan: Res<ScanMesh>,
    reorient: Res<ReorientState>,
    recenter: Res<RecenterState>,
    clip: Res<ClipState>,
) {
    // Nothing to invalidate if no scan has happened yet.
    if cap.loops.is_empty() {
        return;
    }
    if cap.stale {
        return;
    }
    if scan.is_changed() || reorient.is_changed() || recenter.is_changed() || clip.is_changed() {
        cap.bypass_change_detection().stale = true;
    }
}

/// Update system: draw boundary linestrips + centerline polyline as
/// always-on Cap overlays when `CapState::loops` is populated. Each
/// detected loop draws a red linestrip following its boundary edges
/// (closing back to vertex 0). The centerline polyline draws as a
/// cyan linestrip from base to tip — option-3 hedge for curve-
/// following cf-cast.
///
/// When the cap is **stale** (transforms changed since scan),
/// overlays draw with reduced opacity (dimmer color) so the user
/// sees they need to re-scan.
///
/// **Per-loop color disambiguation** (spec §Panel specifications §6):
/// loops cycle through a small palette so multi-loop scans visually
/// distinguish each boundary. With 1-3 loops typical, a 4-color
/// cycle suffices.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
#[allow(clippy::too_many_arguments)] // Bevy gizmo draw system pulls many Res.
fn draw_cap_overlays(
    cap: Res<CapState>,
    scan: Res<ScanMesh>,
    reorient: Res<ReorientState>,
    recenter: Res<RecenterState>,
    overlays: Res<OverlayLengths>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut gizmos: Gizmos,
) {
    if cap.loops.is_empty() {
        return;
    }
    let rotation_bevy = physics_quat_to_bevy_for_plus_z(reorient.quaternion_physics());
    let recenter_world = recenter.translation_world(*up, render_scale.0);
    let bbox_center_bevy = overlays.bbox_center_bevy;

    // Dim factor: stale caps draw at 35% brightness so they read as
    // "needs refresh" without disappearing.
    let dim = if cap.stale { 0.35 } else { 1.0 };

    // Per-loop palette: cycle through 4 distinguishable colors so
    // multi-loop scans read as separate boundaries.
    let palette = [
        Color::srgb(1.0 * dim, 0.30 * dim, 0.30 * dim),  // red
        Color::srgb(1.0 * dim, 0.65 * dim, 0.20 * dim),  // orange
        Color::srgb(0.85 * dim, 0.85 * dim, 0.20 * dim), // yellow
        Color::srgb(0.95 * dim, 0.40 * dim, 0.85 * dim), // magenta
    ];

    for (loop_idx, cap_loop) in cap.loops.iter().enumerate() {
        let color = palette[loop_idx % palette.len()];
        // Boundary linestrip: connect consecutive vertices, closing
        // back to vertex 0.
        let positions: Vec<Vec3> = cap_loop
            .vertex_indices
            .iter()
            .map(|&idx| {
                // Index validity: loop indices were captured at scan
                // time. After a Simplify, scan.0.vertices may be
                // shorter than expected; bounds-check defensively to
                // avoid a panic. The `stale` flag + dimmed render
                // signal the user that the topology mismatch
                // happened.
                let phys = scan
                    .0
                    .vertices
                    .get(idx as usize)
                    .copied()
                    .unwrap_or_else(Point3::origin);
                project_mesh_local_to_world(
                    &phys,
                    *up,
                    rotation_bevy,
                    recenter_world,
                    render_scale.0,
                    bbox_center_bevy,
                )
            })
            .collect();
        for window in positions.windows(2) {
            gizmos.line(window[0], window[1], color);
        }
        // Close the loop.
        if positions.len() >= 2 {
            gizmos.line(positions[positions.len() - 1], positions[0], color);
        }
    }

    // Centerline polyline — cyan, also dimmed if stale.
    if !cap.centerline_polyline.is_empty() {
        let centerline_color = Color::srgb(0.25 * dim, 0.85 * dim, 1.0 * dim);
        let centerline_world: Vec<Vec3> = cap
            .centerline_polyline
            .iter()
            .map(|p| {
                project_mesh_local_to_world(
                    p,
                    *up,
                    rotation_bevy,
                    recenter_world,
                    render_scale.0,
                    bbox_center_bevy,
                )
            })
            .collect();
        for window in centerline_world.windows(2) {
            gizmos.line(window[0], window[1], centerline_color);
        }
    }
}

/// Update system: draw orange circles in the viewport at each
/// centerline-trim cut plane (CSP.4b.2). Shows the user — live, as
/// they drag the trim sliders — exactly where the cut will happen
/// at save time. Two circles render when both trims are non-zero;
/// one when only the corresponding slider is non-zero; none when
/// both are zero (or the centerline hasn't been scanned yet).
///
/// Each circle:
/// - **Center**: along the centerline polyline at the trim distance
///   from the corresponding end (tip = polyline\[0\] + tangent ×
///   trim_tip_mm; floor = polyline\[N-1\] - tangent × trim_floor_mm).
/// - **Normal**: the centerline tangent at that end (perpendicular
///   to the cut plane = the circle plane).
/// - **Radius**: half the largest scan AABB extent, sized so the
///   circle always pokes past the scan in any orientation.
/// - **Color**: bright orange, distinct from the cyan centerline +
///   the cap-loop palette.
///
/// Bakes the centerline-frame positions through the live Reorient +
/// Recenter exactly the way `draw_cap_overlays` projects the
/// centerline polyline itself — so the overlay tracks the mesh
/// when the user nudges the manual sliders too.
#[allow(clippy::needless_pass_by_value)]
#[allow(clippy::too_many_arguments)]
fn draw_trim_plane_overlays(
    cap: Res<CapState>,
    trim: Res<CenterlineTrimState>,
    reorient: Res<ReorientState>,
    recenter: Res<RecenterState>,
    overlays: Res<OverlayLengths>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut gizmos: Gizmos,
) {
    if cap.centerline_polyline.len() < 2 {
        return;
    }
    if trim.trim_tip_mm <= 0.0 && trim.trim_floor_mm <= 0.0 {
        return;
    }
    let rotation_bevy = physics_quat_to_bevy_for_plus_z(reorient.quaternion_physics());
    let recenter_world = recenter.translation_world(*up, render_scale.0);
    let bbox_center_bevy = overlays.bbox_center_bevy;

    // Radius sized to clearly poke past the scan in any orientation.
    // Use the max raw AABB extent so the circle's diameter is at
    // least the longest scan dimension. Times render_scale to land
    // in Bevy world units.
    let raw_aabb = overlays.raw_aabb_m;
    let max_extent_m = (raw_aabb.max.x - raw_aabb.min.x)
        .max(raw_aabb.max.y - raw_aabb.min.y)
        .max(raw_aabb.max.z - raw_aabb.min.z);
    #[allow(clippy::cast_possible_truncation)]
    let radius_world = (max_extent_m * 0.5 * 1.2) as f32 * render_scale.0;

    let color = Color::srgb(1.0, 0.55, 0.10);
    let cl = &cap.centerline_polyline;

    // Closure: project a physics-frame (point, tangent) pair into
    // Bevy world frame + draw the perpendicular circle. Used for
    // both tip + floor ends so the projection math doesn't drift
    // between them.
    let mut draw_at = |plane_center_phys: Point3<f64>, tangent_phys: Vector3<f64>| {
        let plane_center_world = project_mesh_local_to_world(
            &plane_center_phys,
            *up,
            rotation_bevy,
            recenter_world,
            render_scale.0,
            bbox_center_bevy,
        );
        let tangent_swap = up.to_bevy_point(&Point3::from(tangent_phys));
        let tangent_bevy_raw = Vec3::from_array(tangent_swap);
        let tangent_bevy = (rotation_bevy * tangent_bevy_raw).normalize_or_zero();
        if tangent_bevy.length_squared() > 0.0 {
            let isometry = Isometry3d::new(plane_center_world, Quat::IDENTITY);
            draw_perpendicular_circle(&mut gizmos, isometry, tangent_bevy, radius_world, color);
        }
    };

    // CSP.4b.3 — walk the polyline arc instead of linear-
    // extrapolating from the first/last segment's tangent.
    // Initial CSP.4b version diverged from the actual centerline
    // path for curved scans (sock fixture's PCA-induced curvature
    // surfaced this in eyes-on-pixels feedback).
    if trim.trim_tip_mm > 0.0 {
        if let Some((plane_center_phys, tangent_phys)) =
            point_along_polyline_at_arc_distance(cl, trim.trim_tip_mm * 0.001)
        {
            draw_at(plane_center_phys, tangent_phys);
        }
    }

    if trim.trim_floor_mm > 0.0 {
        let total_length_m = polyline_arc_length_m(cl);
        let target_m = total_length_m - trim.trim_floor_mm * 0.001;
        if target_m > 0.0 {
            if let Some((plane_center_phys, tangent_phys)) =
                point_along_polyline_at_arc_distance(cl, target_m)
            {
                draw_at(plane_center_phys, tangent_phys);
            }
        }
    }
}

/// Draw a wireframe circle of radius `radius` centered at
/// `isometry.translation`, lying in the plane perpendicular to
/// `normal`. Bevy's `gizmos.circle` takes a rotation that orients
/// the circle's local +Z to the desired normal direction — this
/// helper computes that rotation from a normal vector.
fn draw_perpendicular_circle(
    gizmos: &mut Gizmos,
    base: Isometry3d,
    normal: Vec3,
    radius: f32,
    color: Color,
) {
    // Bevy's `gizmos.circle` draws in the XY plane of its local
    // frame (i.e., normal = local +Z). Build a rotation that maps
    // local +Z to the supplied `normal`.
    let circle_rotation = Quat::from_rotation_arc(Vec3::Z, normal);
    let isometry = Isometry3d::new(base.translation, circle_rotation);
    gizmos.circle(isometry, radius, color);
}

/// Update system: zero out the accumulated mouse motion + scroll
/// resources before `cf_bevy_common::orbit_camera_input` reads them
/// when egui has captured the pointer (cursor over a panel area
/// **or** egui is processing an active interaction). Without this
/// guard, scrolling the side-panel `ScrollArea` *also* zoomed the
/// orbit camera, and right-click-dragging on a panel button leaked
/// into a viewport pan.
///
/// **Why mutate the resources, not the events**: cf-bevy-common's
/// `orbit_camera_input` reads `Res<AccumulatedMouseMotion>` /
/// `Res<AccumulatedMouseScroll>` (per-frame accumulators), not
/// `MessageReader<MouseWheel>`. Clearing event queues wouldn't
/// affect the orbit camera; zeroing the accumulator deltas does.
///
/// **System ordering**: registered with `.before(orbit_camera_input)`
/// so the zero-out lands before the orbit camera reads. Both write
/// to / read from these accumulator resources, so Bevy's scheduler
/// would serialize them regardless, but the explicit `.before` makes
/// the contract obvious.
///
/// Known edge case: a click started on egui that drags into the
/// viewport (e.g., releasing a panel button while moving the cursor
/// off the panel) can leave the orbit camera mid-drag if the
/// pointer-capture check flips mid-drag. Acceptable for now; the
/// common case (scrolling over the sidebar) is what this fixes.
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
#[allow(clippy::too_many_arguments)] // egui panel sees one ResMut per panel section.
fn scan_prep_panel(
    mut contexts: EguiContexts,
    info: Res<ScanInfo>,
    overlays: Res<OverlayLengths>,
    scan: Res<ScanMesh>,
    mut simplify_state: ResMut<SimplifyState>,
    mut reorient_state: ResMut<ReorientState>,
    mut recenter_state: ResMut<RecenterState>,
    mut clip_state: ResMut<ClipState>,
    mut cap_state: ResMut<CapState>,
    mut cap_pending: ResMut<CapPendingAction>,
    mut centerline_trim_state: ResMut<CenterlineTrimState>,
    mut save_pending: ResMut<SavePendingAction>,
    source: Res<SourceStlPath>,
    output_dir: Res<SaveOutputDir>,
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
            // Wrap the panel sections in a vertical ScrollArea —
            // with 6+ collapsing sections (Scan Info / Simplify /
            // Reorient / Recenter / Clip / Cap, plus #10-#12 to
            // come), the total content exceeds typical viewport
            // height on laptop screens. Without scrolling, sections
            // below the fold are unreachable. `auto_shrink([false,
            // true])` claims full available width but only as much
            // height as content needs (with scrollbar on overflow).
            egui::ScrollArea::vertical()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    render_scan_info_section(ui, &info);
                    render_simplify_section(ui, &info, &mut simplify_state);
                    render_reorient_section(ui, &mut reorient_state, &scan.0);
                    render_recenter_section(ui, &mut recenter_state, &reorient_state, &overlays);
                    render_clip_section(ui, &mut clip_state, &overlays);
                    render_cap_section(
                        ui,
                        &mut cap_state,
                        &mut cap_pending,
                        &mut reorient_state,
                        &mut recenter_state,
                    );
                    render_centerline_trim_section(ui, &mut centerline_trim_state, &cap_state);
                    render_save_section(
                        ui,
                        &source.0,
                        &output_dir.0,
                        &cap_state,
                        &mut save_pending,
                    );
                });
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
            ui.add_space(4.0);
            // Slice 9.8 — communicate the save-time budget semantics.
            // Without this, users assume the slider only affects the
            // viewport (true pre-9.8) and Save unexpectedly writes a
            // full-resolution STL.
            ui.small(
                "Note: this slider is ALSO the save-time face budget. \
                 When you click Save, the cleaned STL is simplified to \
                 the target if it has more faces. Drag to max if you \
                 want the full-resolution mesh saved.",
            );
        });
}

/// `Reorient` section (`docs/SCAN_PREP_DESIGN.md` §Panel specifications
/// §3). Three Euler-angle sliders (intrinsic XYZ in degrees) + five
/// scanner-frame → cast-frame snap shortcuts + a Reset rotation button
/// + a read-only quaternion display.
///
/// **Why the buttons use ASCII `->` not Unicode `→`**: same egui font
/// glyph coverage constraint that drove the commit #4 STL-units fix —
/// `ProggyClean` ships Latin-1 + basic punctuation only, and U+2192
/// renders as `□` (missing-glyph box). Spec §3 button labels were
/// amended to ASCII in lockstep with this commit; see slice-ship-log.
///
/// The state mutation pattern matches `render_simplify_section`:
/// `&mut state` flows through, slider drags update fields directly,
/// snap buttons set known-good triples. `apply_world_transform_to_scan_entity`
/// consumes the state every Update tick and projects it onto the Bevy
/// Transform.
fn render_reorient_section(ui: &mut egui::Ui, state: &mut ReorientState, mesh: &IndexedMesh) {
    egui::CollapsingHeader::new("Reorient")
        .default_open(true)
        .show(ui, |ui| {
            ui.add(egui::Slider::new(&mut state.roll_deg, -180.0..=180.0).text("roll (deg)"));
            ui.add(egui::Slider::new(&mut state.pitch_deg, -180.0..=180.0).text("pitch (deg)"));
            ui.add(egui::Slider::new(&mut state.yaw_deg, -180.0..=180.0).text("yaw (deg)"));

            ui.add_space(4.0);
            // Auto-orient — PCA on vertex positions, principal axis
            // → +Z (cast-frame demolding convention). Closes
            // SCAN_PREP_DESIGN.md deferred item §5. The resulting
            // Euler angles drop straight into the slider source-of-
            // truth so the user can still nudge afterwards.
            if ui.button("Auto-orient (PCA)").clicked() {
                if let Some(q) = compute_pca_orientation(&mesh.vertices) {
                    let (roll, pitch, yaw) = q.euler_angles();
                    state.roll_deg = roll.to_degrees();
                    state.pitch_deg = pitch.to_degrees();
                    state.yaw_deg = yaw.to_degrees();
                }
            }

            ui.add_space(4.0);
            // Five snap shortcuts for the common scanner-frame →
            // cast-frame mismatches. Each sets the three slider values
            // to a known triple; the derived quaternion + Bevy
            // Transform follow on the next tick.
            if ui.button("Snap +Z up").clicked() {
                *state = ReorientState::default();
            }
            if ui.button("Snap -Z up").clicked() {
                *state = ReorientState {
                    roll_deg: 180.0,
                    pitch_deg: 0.0,
                    yaw_deg: 0.0,
                };
            }
            if ui.button("Snap +Y -> +Z").clicked() {
                *state = ReorientState {
                    roll_deg: 90.0,
                    pitch_deg: 0.0,
                    yaw_deg: 0.0,
                };
            }
            if ui.button("Snap -Y -> +Z").clicked() {
                *state = ReorientState {
                    roll_deg: -90.0,
                    pitch_deg: 0.0,
                    yaw_deg: 0.0,
                };
            }
            if ui.button("Snap +X -> +Z").clicked() {
                *state = ReorientState {
                    roll_deg: 0.0,
                    pitch_deg: -90.0,
                    yaw_deg: 0.0,
                };
            }

            ui.add_space(4.0);
            if ui.button("Reset rotation").clicked() {
                *state = ReorientState::default();
            }

            ui.add_space(4.0);
            let q = state.quaternion_physics();
            ui.label(format!(
                "q (w,x,y,z) = ({:+.3}, {:+.3}, {:+.3}, {:+.3})",
                q.w, q.i, q.j, q.k,
            ));
        });
}

/// `Recenter` section (`docs/SCAN_PREP_DESIGN.md` §Panel specifications
/// §4). Three translation sliders (mm, range `±2 × max(raw_aabb)`
/// per-scan) + `[Center origin]`, `[Floor -> z=0]`, `[Reset
/// translation]` buttons.
///
/// **Click-time math for `[Center origin]` and `[Floor -> z=0]`**:
/// both use the current `ReorientState` quaternion to figure out where
/// the rotated bbox actually sits, then set the slider values so the
/// rotated mesh lands at its target position. The center / floor stays
/// frozen at the click-time configuration — if the user changes
/// Reorient afterwards, the centering becomes stale and they re-click
/// to re-center. Spec confirms these are buttons (discrete events) not
/// continuous behaviors.
fn render_recenter_section(
    ui: &mut egui::Ui,
    state: &mut RecenterState,
    reorient: &ReorientState,
    overlays: &OverlayLengths,
) {
    egui::CollapsingHeader::new("Recenter")
        .default_open(true)
        .show(ui, |ui| {
            let range = overlays.recenter_slider_range_mm;
            ui.add(egui::Slider::new(&mut state.tx_mm, -range..=range).text("X (mm)"));
            ui.add(egui::Slider::new(&mut state.ty_mm, -range..=range).text("Y (mm)"));
            ui.add(egui::Slider::new(&mut state.tz_mm, -range..=range).text("Z (mm)"));

            ui.add_space(4.0);
            // [Center origin]: under centroid-pivot rotation the AABB
            // CENTER stays at the raw centroid regardless of rotation —
            // so the math collapses to "translate the centroid to
            // world origin," rotation-independent.
            if ui.button("Center origin").clicked() {
                let c = overlays.raw_aabb_m.center();
                state.tx_mm = -c.x * 1000.0;
                state.ty_mm = -c.y * 1000.0;
                state.tz_mm = -c.z * 1000.0;
            }
            // [Floor -> z=0]: needs the rotated mesh's minimum z; the
            // centroid-pivot helper returns the rotated AABB whose
            // center is the centroid in mm (so min.z reflects how the
            // current rotation orients the mesh around its own
            // centroid).
            if ui.button("Floor -> z=0").clicked() {
                let (min_mm, _max_mm) = rotated_aabb_around_centroid_physics_mm(
                    reorient.quaternion_physics(),
                    &overlays.raw_aabb_m,
                );
                state.tz_mm = -min_mm.z;
            }

            ui.add_space(4.0);
            if ui.button("Reset translation").clicked() {
                *state = RecenterState::default();
            }
        });
}

/// `Clip floor` section (`docs/SCAN_PREP_DESIGN.md` §Panel specifications
/// §5). One enable checkbox + one Z slider + a live drop-percentage
/// readout. The disc visualization renders in `draw_reference_overlays`
/// when the toggle is on.
///
/// **Algorithm landing at #8 but not yet applied to the displayed
/// mesh.** The triangle-clipping algorithm
/// ([`clip_mesh_against_world_z`]) is implemented + unit-tested at
/// this commit but sits dormant. It gets exercised at commit #9 (Cap
/// detection consumes the clipped mesh for boundary identification)
/// and commit #12 (Save bakes the clip into the cleaned STL). Until
/// then the rendered mesh stays whole; the user reads the percentage
/// + disc position to verify intent.
///
/// Drop percentage is read from `ClipState::drop_pct` — the cached
/// value populated by [`update_clip_drop_pct`] on relevant state
/// changes. The panel render itself stays O(1).
fn render_clip_section(ui: &mut egui::Ui, state: &mut ClipState, overlays: &OverlayLengths) {
    egui::CollapsingHeader::new("Clip floor")
        .default_open(true)
        .show(ui, |ui| {
            ui.checkbox(&mut state.enabled, "Enabled");

            let range = overlays.recenter_slider_range_mm;
            ui.add(egui::Slider::new(&mut state.z_mm, -range..=range).text("Z (mm)"));

            if state.enabled {
                ui.label(format!("Drops {:.1}% verts", state.drop_pct));
                if state.drop_pct > 95.0 {
                    ui.colored_label(
                        egui::Color32::from_rgb(240, 200, 80),
                        "⚠ Clip removes nearly all geometry — verify Z value",
                    );
                }
            }
        });
}

/// `Cap open boundaries` section (`docs/SCAN_PREP_DESIGN.md` §Panel
/// specifications §6). Detects open boundary loops on the current
/// scan, fits a plane to each via SVD, and displays per-loop
/// metadata for the user to decide which ones to cap when the
/// cleaned STL is saved.
///
/// **Scope at commit #9**: detection + display + visualization
/// (linestrips + centerline) + per-loop include checkboxes + `[Scan]`
/// re-run trigger. Cap-triangulation + bake-into-STL deferred to
/// commit #12 (Save panel) — until then, "Apply" is just a state
/// commitment; nothing modifies the mesh.
///
/// **Stale-loop banner**: when transforms or simplify change after a
/// scan, the displayed list dims and a "Transform changed — re-Scan
/// to refresh" notice appears. The user clicks `[Scan]` again to
/// re-fit planes on the now-current mesh.
fn render_cap_section(
    ui: &mut egui::Ui,
    state: &mut CapState,
    pending: &mut CapPendingAction,
    reorient: &mut ReorientState,
    recenter: &mut RecenterState,
) {
    egui::CollapsingHeader::new("Cap open boundaries")
        .default_open(true)
        .show(ui, |ui| {
            if ui.button("Scan").clicked() {
                pending.rescan = true;
            }

            if state.loops.is_empty() {
                ui.label(
                    egui::RichText::new("(no scan yet — click Scan to detect boundary loops)")
                        .italics(),
                );
                return;
            }

            // One-click alignment: rotate so the first loop's outward
            // normal points to world -Z (cavity-floor convention) AND
            // translate so its centroid lands at world origin. Solves
            // the "manually lining it up is bound to give imperfect
            // results" UX gap — the SVD plane fit already knows
            // exactly where the boundary plane is.
            if ui.button("Snap boundary -> floor").clicked() {
                if let Some(loop_0) = state.loops.first() {
                    let target = Vector3::new(0.0, 0.0, -1.0);
                    let rotation = UnitQuaternion::rotation_between(&loop_0.plane_normal, &target)
                        .unwrap_or_else(|| {
                            UnitQuaternion::from_axis_angle(
                                &nalgebra::Vector3::x_axis(),
                                std::f64::consts::PI,
                            )
                        });
                    let (roll, pitch, yaw) = rotation.euler_angles();
                    reorient.roll_deg = roll.to_degrees();
                    reorient.pitch_deg = pitch.to_degrees();
                    reorient.yaw_deg = yaw.to_degrees();
                    let rotated_centroid = rotation.transform_point(&loop_0.plane_centroid);
                    recenter.tx_mm = -rotated_centroid.x * 1000.0;
                    recenter.ty_mm = -rotated_centroid.y * 1000.0;
                    recenter.tz_mm = -rotated_centroid.z * 1000.0;
                    // Re-scan: the loop indices still reference the
                    // same mesh vertices, but the staleness flag will
                    // flip from the Reorient/Recenter mutations. A
                    // queued rescan clears it on the next tick.
                    pending.rescan = true;
                }
            }

            if state.stale {
                ui.colored_label(
                    egui::Color32::from_rgb(240, 200, 80),
                    "⚠ Transform changed — re-Scan to refresh",
                );
            }

            ui.add_space(4.0);
            ui.label(format!(
                "Found {} boundary loop{}:",
                state.loops.len(),
                if state.loops.len() == 1 { "" } else { "s" },
            ));
            for (loop_idx, cap_loop) in state.loops.iter_mut().enumerate() {
                ui.horizontal(|ui| {
                    let label = format!(
                        "Loop {loop_idx}  ({} verts, R² {:.3})",
                        cap_loop.vertex_indices.len(),
                        cap_loop.plane_fit_r_squared,
                    );
                    ui.checkbox(&mut cap_loop.include, label);
                });
            }

            ui.add_space(4.0);
            if !state.centerline_polyline.is_empty() {
                ui.label(format!(
                    "Centerline: {} segments (cyan polyline)",
                    state.centerline_polyline.len().saturating_sub(1),
                ));
            }
        });
}

/// `Centerline trim` section (CSP.4b — user-driven trim along the
/// centerline polyline). Drives two perpendicular-plane cuts that get
/// applied at save time + auto-capped, producing a workshop-ready
/// trimmed mesh without requiring the user to fiddle with manual
/// rotation / translation / clip-plane sliders.
///
/// Layout:
///
/// - "Run Cap → Scan first" hint when the centerline polyline is
///   empty (the trim has no axis to cut along).
/// - Two sliders: `trim from tip (mm)` + `trim from floor (mm)`.
///   Range bounded by half the centerline arc length, so trimming
///   both ends to slider-max produces zero-length output (no
///   negative-length geometry).
/// - `[Reset trim]` button → zeros both sliders.
///
/// The actual trim algorithm runs in [`trim_mesh_along_centerline`] at
/// save time, taking the post-bake centerline polyline (built from
/// `cap.centerline_polyline` through the user's Reorient + Recenter
/// transforms) and clipping the mesh perpendicular to the polyline
/// tangent at each end. Auto-capping of the new cut boundaries
/// follows immediately via [`auto_cap_open_boundaries`].
fn render_centerline_trim_section(
    ui: &mut egui::Ui,
    state: &mut CenterlineTrimState,
    cap: &CapState,
) {
    egui::CollapsingHeader::new("Centerline trim")
        .default_open(true)
        .show(ui, |ui| {
            if cap.centerline_polyline.len() < 2 {
                ui.label(
                    egui::RichText::new(
                        "(no centerline yet — click Cap > Scan first to detect a centerline)",
                    )
                    .italics(),
                );
                return;
            }

            // Centerline arc length in mm; used to bound the
            // sliders so the user can't trim more than the polyline
            // provides per end.
            let total_length_mm: f64 = cap
                .centerline_polyline
                .windows(2)
                .map(|w| (w[1].coords - w[0].coords).norm() * 1000.0)
                .sum();
            // Per-end max = half the total. If both sliders sit at
            // half, the polyline is fully consumed → trim cuts meet
            // at the midpoint. Beyond that is undefined; the slider
            // bound protects against it.
            let max_trim_mm = total_length_mm * 0.5;

            ui.label(format!("Centerline arc length: {total_length_mm:.1} mm"));
            ui.add(
                egui::Slider::new(&mut state.trim_tip_mm, 0.0..=max_trim_mm)
                    .text("trim from tip (mm)"),
            );
            ui.add(
                egui::Slider::new(&mut state.trim_floor_mm, 0.0..=max_trim_mm)
                    .text("trim from floor (mm)"),
            );

            ui.add_space(4.0);
            if ui.button("Reset trim").clicked() {
                state.trim_tip_mm = 0.0;
                state.trim_floor_mm = 0.0;
            }

            ui.add_space(4.0);
            ui.small(
                "Trim is applied at save time. The cleaned STL gets cut perpendicular to \
                 the centerline at each end + auto-capped so the result is watertight.",
            );
        });
}

/// `Save` section (`docs/SCAN_PREP_DESIGN.md` §Panel specifications §9).
/// Writes the cleaned STL + the `.prep.toml` provenance file to the
/// configured output directory.
///
/// Layout:
///
/// - Filename previews (`<stem>.cleaned.stl` + `<stem>.prep.toml`) so
///   the user sees exactly what will be written.
/// - Output directory (hover tooltip = full path).
/// - Per-loop "uncapped" warning if any detected loops are excluded —
///   they leave the cleaned mesh non-watertight, which downstream
///   cf-cast SDF queries can't handle. The user can still save anyway
///   (their judgment call on whether the warning applies).
/// - `[Save]` button queues `SavePendingAction::save_now`;
///   `handle_save_action` runs the actual write next Update tick.
///
/// Per spec §Architectural decisions §Save behavior: **no confirmation
/// dialog**. The atomic-write pattern (`.tmp` + rename) means a click
/// is fully reversible at the FS level if the user notices a mistake
/// immediately afterward, and the .prep.toml records the full state.
fn render_save_section(
    ui: &mut egui::Ui,
    source_path: &Path,
    output_dir: &Path,
    cap_state: &CapState,
    save_pending: &mut SavePendingAction,
) {
    egui::CollapsingHeader::new("Save")
        .default_open(true)
        .show(ui, |ui| {
            let stem = source_path
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("scan");
            let cleaned_stl_name = format!("{stem}.cleaned.stl");
            let prep_toml_name = format!("{stem}.prep.toml");

            ui.label("Outputs:");
            ui.label(format!("  - {cleaned_stl_name}"));
            ui.label(format!("  - {prep_toml_name}"));

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Dir:");
                let dir_label = output_dir
                    .file_name()
                    .and_then(|s| s.to_str())
                    .unwrap_or(".");
                ui.label(dir_label)
                    .on_hover_text(output_dir.display().to_string());
            });

            let included = cap_state.loops.iter().filter(|l| l.include).count();
            let excluded = cap_state.loops.len().saturating_sub(included);
            if excluded > 0 {
                ui.add_space(4.0);
                ui.colored_label(
                    egui::Color32::from_rgb(240, 200, 80),
                    format!(
                        "⚠ {excluded} boundary loop{} excluded — cleaned mesh \
                         will not be watertight.",
                        if excluded == 1 { "" } else { "s" },
                    ),
                );
            }
            if cap_state.loops.is_empty() {
                ui.add_space(4.0);
                ui.colored_label(
                    egui::Color32::from_rgb(240, 200, 80),
                    "⚠ No boundary loops detected — Scan in Cap panel first \
                     for a watertight save.",
                );
            }

            ui.add_space(4.0);
            if ui.button("Save").clicked() {
                save_pending.save_now = true;
            }
        });
}

/// Update system: consume `SavePendingAction::save_now` (if any) by
/// building the cleaned mesh + the `.prep.toml` and atomically
/// writing both files to the output directory.
///
/// Pipeline:
///
/// 1. Build cleaned mesh: apply transform.rotation + transform.translation
///    to every vertex; ear-clip + append cap triangles for each
///    included loop.
/// 2. Build `.prep.toml` string: scan-prep + simplify + transform +
///    clip + caps + (post-bake world-frame) centerline + output blocks
///    (with cleaned-mesh AABB).
/// 3. Atomic write: STL to `.stl.tmp`, TOML to `.toml.tmp`, rename
///    both. On any failure roll back both temp + final files so the
///    user doesn't end up with a half-written save.
/// 4. Update status bar: green Normal achievement on success
///    (`Saved <stem>.cleaned.stl (n triangles)`); red Error with
///    abbreviated cause on failure.
///
/// Per spec §Architectural decisions §FS error handling: the handler
/// never panics — disk-full / read-only / permission errors surface
/// as red status-bar messages so the user can retry after fixing the
/// underlying condition.
#[allow(clippy::too_many_arguments)]
fn handle_save_action(
    mut pending: ResMut<SavePendingAction>,
    scan: Res<ScanMesh>,
    original: Res<OriginalScanMesh>,
    reorient: Res<ReorientState>,
    recenter: Res<RecenterState>,
    clip: Res<ClipState>,
    cap: Res<CapState>,
    simplify: Res<SimplifyState>,
    centerline_trim: Res<CenterlineTrimState>,
    output_dir: Res<SaveOutputDir>,
    source: Res<SourceStlPath>,
    stl_units: Res<StlUnitsResource>,
    auto_center: Res<AutoCenterOffset>,
    auto_pca: Res<AutoPcaQuaternion>,
    mut status: ResMut<StatusBar>,
    time: Res<Time>,
) {
    if !pending.save_now {
        return;
    }
    pending.save_now = false;

    let stem = source
        .0
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("scan");
    let cleaned_stl_path = output_dir.0.join(format!("{stem}.cleaned.stl"));
    let prep_toml_path = output_dir.0.join(format!("{stem}.prep.toml"));

    // Reject non-finite transforms before touching the disk. Per spec
    // §Architectural decisions §FS error handling, malformed slider
    // values (NaN / Inf from corrupted state) surface as a red error
    // rather than writing garbage to disk. CSP.2: clip.z_mm joined
    // the precondition because clip now bakes into the cleaned STL —
    // a NaN clip plane would drop every triangle silently.
    let q = reorient.quaternion_physics();
    if !q.w.is_finite()
        || !q.i.is_finite()
        || !q.j.is_finite()
        || !q.k.is_finite()
        || !recenter.tx_mm.is_finite()
        || !recenter.ty_mm.is_finite()
        || !recenter.tz_mm.is_finite()
        || (clip.enabled && !clip.z_mm.is_finite())
        || !centerline_trim.trim_tip_mm.is_finite()
        || !centerline_trim.trim_floor_mm.is_finite()
    {
        status.text = "Save failed: non-finite transform / clip / trim values".into();
        status.kind = StatusKind::Error;
        status.auto_clear_at_secs = Some(time.elapsed_secs_f64() + 6.0);
        return;
    }

    let mut cleaned = build_cleaned_mesh(&scan.0, &reorient, &recenter, &clip, &cap);

    // CSP.4b — centerline trim. Apply user-requested trims to the
    // cleaned mesh + auto-cap the resulting boundaries. The
    // centerline in `cap.centerline_polyline` is in physics frame
    // (pre-bake); bake it through the user transforms first so the
    // trim planes match the cleaned mesh's world frame. Skip
    // entirely when no trim requested — keeps the trim-zero path
    // bit-exact with pre-CSP.4b behavior.
    let trim_rotation = reorient.quaternion_physics();
    let trim_translation = Vector3::new(
        recenter.tx_mm * 0.001,
        recenter.ty_mm * 0.001,
        recenter.tz_mm * 0.001,
    );
    let trim_pivot_centroid_m = scan.0.aabb().center();
    let centerline_world: Vec<Point3<f64>> = if (centerline_trim.trim_tip_mm > 0.0
        || centerline_trim.trim_floor_mm > 0.0)
        && cap.centerline_polyline.len() >= 2
    {
        cap.centerline_polyline
            .iter()
            .map(|p| {
                bake_vertex_with_pivot(p, trim_rotation, &trim_pivot_centroid_m, trim_translation)
            })
            .collect()
    } else {
        Vec::new()
    };
    let pre_trim_face_count = cleaned.faces.len();
    let mut trim_capped = 0_usize;
    let mut trim_status_suffix = String::new();
    if !centerline_world.is_empty() {
        cleaned = trim_mesh_along_centerline(
            &cleaned,
            &centerline_world,
            centerline_trim.trim_tip_mm,
            centerline_trim.trim_floor_mm,
        );
        trim_capped = auto_cap_open_boundaries(&mut cleaned);
        trim_status_suffix = format!(
            ", trim: {} -> {} faces, capped {} boundary loop{}",
            human_count(pre_trim_face_count),
            human_count(cleaned.faces.len()),
            trim_capped,
            if trim_capped == 1 { "" } else { "s" },
        );
    }

    // CSP.3b — mesh hygiene cleanup before simplify. Runs the four-
    // step pass (weld + degenerate + small-components + unreferenced)
    // so simplify operates on already-clean topology + the cleaned
    // STL passes downstream `simplify_decoder` (not just
    // `simplify_sloppy_decoder`). The report's non-zero terms feed
    // the status message as a confidence signal for the user.
    let cleanup = cleanup_cleaned_mesh_for_disk(&mut cleaned);
    let cleanup_status_suffix = if cleanup.total() == 0 {
        String::new()
    } else {
        format!(
            ", cleanup: {} welded / {} degenerate / {} small-component / {} unreferenced",
            cleanup.welded, cleanup.degenerate, cleanup.small_components, cleanup.unreferenced,
        )
    };

    // Slice 9.8 — Simplify panel slider acts as a face budget at save
    // time too, not just for the [Apply simplify] in-app action. If
    // the slider sits below the cleaned mesh's current face count,
    // run meshopt down to the target before writing — so saved STLs
    // honor the slider regardless of whether [Apply] was clicked
    // first. Default slider value is `SIMPLIFY_TARGET_DEFAULT` (200k),
    // which keeps 3M+ face raw scans from landing on disk unfiltered
    // and grinding cf-cast-cli / cf-device-design downstream.
    //
    // The user opts OUT by dragging the slider up to or above the
    // current face count.
    let pre_simplify_face_count = cleaned.faces.len();
    let mut simplify_status_suffix = String::new();
    if simplify.target_face_count < pre_simplify_face_count {
        let result = simplify_mesh(&cleaned, simplify.target_face_count);
        simplify_status_suffix = format!(
            ", simplified {} \u{2192} {} faces in {:.1}s",
            human_count(pre_simplify_face_count),
            human_count(result.mesh.faces.len()),
            result.elapsed_secs,
        );
        cleaned = result.mesh;
    }
    let triangle_count = cleaned.faces.len();

    let rotation = reorient.quaternion_physics();
    let translation = Vector3::new(
        recenter.tx_mm * 0.001,
        recenter.ty_mm * 0.001,
        recenter.tz_mm * 0.001,
    );
    let pivot_centroid_m = scan.0.aabb().center();
    // `[simplify]` provenance inputs: the as-loaded face count (in
    // OriginalScanMesh, untouched by Apply / Reset / save-time
    // budget) drives `original_face_count`; the final cleaned mesh's
    // face count is what landed on disk.
    let original_face_count = original.0.faces.len();
    let cleaned_aabb_m = cleaned.aabb();
    let toml_str = match build_prep_toml_string(
        &source.0,
        stl_units.0,
        auto_center.0,
        auto_pca.0,
        &reorient,
        &recenter,
        &clip,
        &cap,
        &centerline_trim,
        trim_capped,
        &format!("{stem}.cleaned.stl"),
        rotation,
        translation,
        pivot_centroid_m,
        simplify.target_face_count,
        original_face_count,
        triangle_count,
        &cleaned_aabb_m,
    ) {
        Ok(s) => s,
        Err(e) => {
            status.text = format!("Save failed: building .prep.toml ({e:#})");
            status.kind = StatusKind::Error;
            status.auto_clear_at_secs = Some(time.elapsed_secs_f64() + 6.0);
            return;
        }
    };

    match atomic_write_save(&cleaned, &cleaned_stl_path, &prep_toml_path, &toml_str) {
        Ok(()) => {
            status.text = format!(
                "Saved {stem}.cleaned.stl ({} triangles{trim_status_suffix}{simplify_status_suffix}{cleanup_status_suffix}) + {stem}.prep.toml",
                human_count(triangle_count),
            );
            status.kind = StatusKind::Normal;
            status.auto_clear_at_secs = Some(time.elapsed_secs_f64() + 6.0);
        }
        Err(e) => {
            status.text = format!("Save failed: {e:#}");
            status.kind = StatusKind::Error;
            status.auto_clear_at_secs = Some(time.elapsed_secs_f64() + 8.0);
        }
    }
}

/// Bottom status bar. Renders nothing when `status.text` is empty so an
/// empty status doesn't claim window space. Color-codes the text by
/// [`StatusKind`]: gray for Normal, yellow for Warning (auto-suggest
/// banner / soft caps), red for Error (Save-panel FS errors).
fn render_status_bar(ctx: &egui::Context, status: &StatusBar) {
    if status.text.is_empty() {
        return;
    }
    let color = match status.kind {
        StatusKind::Normal => egui::Color32::from_gray(220),
        StatusKind::Warning => egui::Color32::from_rgb(240, 200, 80),
        StatusKind::Error => egui::Color32::from_rgb(230, 80, 80),
    };
    egui::TopBottomPanel::bottom("cf-scan-prep-status")
        .resizable(false)
        .show(ctx, |ui| {
            ui.colored_label(color, &status.text);
        });
}

#[cfg(test)]
mod tests {
    // `unwrap()` + `expect()` are denied at the crate level for
    // production safety; allow them inside tests so assertions can
    // pull values out of `Option` / `Result` returns without writing
    // multi-line `match` ceremony for every fixture build.
    #![allow(clippy::unwrap_used, clippy::expect_used)]

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

    // ----- auto_center_in_place (CSP.3.5) ----------------------------

    /// `auto_center_in_place` moves a scan offset far from the
    /// physics origin to a centroid-at-origin position, and reports
    /// the offset it applied. Simulates the "scanner pointed down,
    /// captured geometry at negative z" workflow the CSP.3.5
    /// followup addresses.
    #[test]
    fn auto_center_in_place_moves_centroid_to_origin() {
        // A unit-cube-like mesh sitting at physics centroid
        // (0.05, 0.00, -0.170) — far from origin, like a scanner
        // dropped it there.
        let mut mesh = IndexedMesh::with_capacity(8, 0);
        for &(x, y, z) in &[
            (0.05 - 0.04, 0.00 - 0.03, -0.170 - 0.06),
            (0.05 + 0.04, 0.00 - 0.03, -0.170 - 0.06),
            (0.05 + 0.04, 0.00 + 0.03, -0.170 - 0.06),
            (0.05 - 0.04, 0.00 + 0.03, -0.170 - 0.06),
            (0.05 - 0.04, 0.00 - 0.03, -0.170 + 0.06),
            (0.05 + 0.04, 0.00 - 0.03, -0.170 + 0.06),
            (0.05 + 0.04, 0.00 + 0.03, -0.170 + 0.06),
            (0.05 - 0.04, 0.00 + 0.03, -0.170 + 0.06),
        ] {
            mesh.vertices.push(Point3::new(x, y, z));
        }
        let offset = auto_center_in_place(&mut mesh);
        // Offset = -centroid → expect (-0.05, 0, +0.170).
        assert!((offset.x - (-0.05)).abs() < 1e-12);
        assert!(offset.y.abs() < 1e-12);
        assert!((offset.z - 0.170).abs() < 1e-12);
        // Post-centering AABB centroid should be ~origin.
        let post_centroid = mesh.aabb().center();
        assert!(post_centroid.coords.norm() < 1e-12);
    }

    /// `auto_center_in_place` is a no-op (returns zero offset) for
    /// a mesh whose centroid is already within 1 µm of the origin.
    /// Guards against FP-drift on already-centered fixtures. Uses a
    /// symmetric ±1 mm cube — its AABB centroid lands at bit-exact
    /// origin, so the threshold guard fires.
    #[test]
    fn auto_center_in_place_no_op_when_already_centered() {
        let mut mesh = IndexedMesh::with_capacity(2, 0);
        mesh.vertices.push(Point3::new(-0.001, -0.001, -0.001));
        mesh.vertices.push(Point3::new(0.001, 0.001, 0.001));
        let mesh_before = mesh.clone();
        let offset = auto_center_in_place(&mut mesh);
        assert!(offset.norm() < 1e-12, "near-zero offset, got {offset:?}");
        // Vertices unchanged.
        assert_eq!(mesh.vertices, mesh_before.vertices);
    }

    /// `auto_center_in_place` is a no-op on empty meshes (defensive
    /// against the load-failure overlay path — never hit, but cheap
    /// to guarantee).
    #[test]
    fn auto_center_in_place_handles_empty_mesh() {
        let mut mesh = IndexedMesh::with_capacity(0, 0);
        let offset = auto_center_in_place(&mut mesh);
        assert_eq!(offset, Vector3::zeros());
        assert!(mesh.vertices.is_empty());
    }

    // ----- auto_pca_in_place (CSP.4a) --------------------------------

    /// Mesh elongated along physics +X: after auto-PCA-in-place, the
    /// long axis aligns with +Z (the principal-axis vertices' X
    /// components rotate into Z). Confirms the bake-in-place
    /// behavior matches the rotation `compute_pca_orientation`
    /// returns.
    #[test]
    fn auto_pca_in_place_rotates_long_x_axis_to_plus_z() {
        // 11 points along +X (centroid at origin), no spread on Y/Z.
        let mut mesh = IndexedMesh::with_capacity(11, 0);
        for i in -5..=5 {
            mesh.vertices
                .push(Point3::new(f64::from(i) * 0.01, 0.0, 0.0));
        }
        let pre_x_range_max = mesh
            .vertices
            .iter()
            .map(|v| v.x.abs())
            .fold(0.0_f64, f64::max);
        assert!((pre_x_range_max - 0.05).abs() < 1e-12);

        let q = auto_pca_in_place(&mut mesh).expect("PCA should succeed");

        // After rotation, the long axis runs along +Z (or -Z; the
        // sign-pick keeps the rotation short). The X spread
        // collapses to ~zero; the Z spread inherits the original X
        // spread.
        let post_z_range_max = mesh
            .vertices
            .iter()
            .map(|v| v.z.abs())
            .fold(0.0_f64, f64::max);
        let post_x_range_max = mesh
            .vertices
            .iter()
            .map(|v| v.x.abs())
            .fold(0.0_f64, f64::max);
        assert!(
            (post_z_range_max - 0.05).abs() < 1e-9,
            "post Z extent: {post_z_range_max}"
        );
        assert!(
            post_x_range_max < 1e-9,
            "post X extent should collapse: {post_x_range_max}"
        );

        // Quaternion identity-check: not identity (we actually rotated).
        let vec_sq = q.i * q.i + q.j * q.j + q.k * q.k;
        assert!(vec_sq > 1e-6, "expected non-identity rotation");
    }

    /// Already-PCA-aligned mesh (principal axis = +Z, vertices in
    /// the YZ plane with the long axis on Z): auto-PCA is the
    /// identity, vertex positions stay bit-exact.
    #[test]
    fn auto_pca_in_place_skips_vertex_walk_on_near_identity() {
        // 11 points along +Z, centroid at origin. PCA principal axis
        // = +Z already; rotation_between(+Z, +Z) = identity.
        let mut mesh = IndexedMesh::with_capacity(11, 0);
        for i in -5..=5 {
            mesh.vertices
                .push(Point3::new(0.0, 0.0, f64::from(i) * 0.01));
        }
        let mesh_before = mesh.clone();

        let q = auto_pca_in_place(&mut mesh).expect("PCA should succeed");
        // Quaternion should be near-identity.
        assert!(
            (q.w - 1.0).abs() < 1e-9,
            "expected near-identity q.w, got {}",
            q.w
        );
        // Vertices bit-exactly preserved (the near-identity skip
        // path didn't walk them).
        assert_eq!(mesh.vertices, mesh_before.vertices);
    }

    /// Degenerate input (all vertices coincident → no principal
    /// axis): auto-PCA returns `None`, vertices unchanged. Pairs
    /// with `compute_pca_orientation`'s `None` path.
    #[test]
    fn auto_pca_in_place_returns_none_on_degenerate() {
        let mut mesh = IndexedMesh::with_capacity(10, 0);
        for _ in 0..10 {
            mesh.vertices.push(Point3::origin());
        }
        let mesh_before = mesh.clone();
        let q = auto_pca_in_place(&mut mesh);
        assert!(q.is_none(), "expected None on degenerate input");
        assert_eq!(mesh.vertices, mesh_before.vertices);
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

    /// Regression for the iter-1-eyes-on-pixels crash: meshopt's C++
    /// side asserts `target_index_count <= index_count` in
    /// `meshopt_simplifyEdge` and SIGABRTs the process if violated.
    /// `simplify_mesh` defensively early-returns the input unchanged
    /// when `target_face_count >= original.faces.len()` so the FFI is
    /// never invoked with assertion-violating params.
    ///
    /// Covers the "drag slider right past current face count + Apply"
    /// workflow (e.g., after a prior Apply reduces to 100 faces, then
    /// the user drags the slider to 200 and clicks Apply again). The
    /// `handle_simplify_actions` system pre-checks this case and
    /// surfaces a user-facing status message, but the guard inside
    /// `simplify_mesh` itself is the bulletproof layer.
    #[test]
    fn simplify_mesh_above_input_face_count_is_no_op() {
        let original = grid_square(20);
        assert_eq!(original.faces.len(), 800);

        // Target >> input — would SIGABRT without the guard.
        let result = simplify_mesh(&original, 10_000);

        assert_eq!(
            result.mesh.faces.len(),
            original.faces.len(),
            "guard must return input unchanged when target >= current",
        );
        assert_eq!(result.mesh.vertices.len(), original.vertices.len());

        // Exactly-equal target also exits the guard (>=, not >).
        let result_eq = simplify_mesh(&original, 800);
        assert_eq!(result_eq.mesh.faces.len(), 800);
    }

    // ----- ReorientState + physics_quat_to_bevy ----------------------

    /// Default state is all-zeros (identity rotation). Pinned because
    /// changing the default silently rotates every loaded scan from
    /// the moment of load.
    #[test]
    fn reorient_state_default_is_zero() {
        let state = ReorientState::default();
        assert_eq!(state.roll_deg, 0.0);
        assert_eq!(state.pitch_deg, 0.0);
        assert_eq!(state.yaw_deg, 0.0);
    }

    /// All-zeros sliders derive the identity quaternion `(w=1, 0, 0, 0)`.
    /// Tests `from_euler_angles(0, 0, 0)`'s contract.
    #[test]
    fn reorient_default_quaternion_is_identity() {
        let q = ReorientState::default().quaternion_physics();
        assert!((q.w - 1.0).abs() < 1e-12);
        assert!(q.i.abs() < 1e-12);
        assert!(q.j.abs() < 1e-12);
        assert!(q.k.abs() < 1e-12);
    }

    /// `Snap +Y -> +Z` triple (roll=90°, pitch=0, yaw=0) derives the
    /// 90°-about-X quaternion: `(cos(45°), sin(45°), 0, 0)`. Pins the
    /// Euler convention (intrinsic XYZ — roll is the X-axis rotation).
    /// If nalgebra's `from_euler_angles` argument order ever changes,
    /// this test fails first.
    #[test]
    fn reorient_snap_plus_y_to_plus_z_is_90deg_about_x() {
        let state = ReorientState {
            roll_deg: 90.0,
            pitch_deg: 0.0,
            yaw_deg: 0.0,
        };
        let q = state.quaternion_physics();
        let expected_cos = (std::f64::consts::FRAC_PI_4).cos();
        let expected_sin = (std::f64::consts::FRAC_PI_4).sin();
        assert!((q.w - expected_cos).abs() < 1e-12);
        assert!((q.i - expected_sin).abs() < 1e-12);
        assert!(q.j.abs() < 1e-12);
        assert!(q.k.abs() < 1e-12);
    }

    // ----- compute_pca_orientation ----------------------------------

    /// Empty vertex slice returns `None` — PCA needs at least one
    /// point to define a centroid, three to define a principal axis.
    /// Pinned so a misuse at the call site surfaces visibly rather
    /// than panicking inside `symmetric_eigen` on a zero matrix.
    #[test]
    fn pca_orientation_too_few_vertices_returns_none() {
        let v: Vec<Point3<f64>> = vec![];
        assert!(compute_pca_orientation(&v).is_none());
        let v = vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)];
        assert!(compute_pca_orientation(&v).is_none());
    }

    /// Coincident vertices have zero covariance → no principal axis.
    /// Returns `None` rather than producing an arbitrary rotation
    /// from numerical noise.
    #[test]
    fn pca_orientation_degenerate_coincident_returns_none() {
        let v = vec![Point3::origin(); 10];
        assert!(compute_pca_orientation(&v).is_none());
    }

    /// Vertices stretched along physics-frame `+X` produce a rotation
    /// that maps `+X` to `+Z`. The shortest such rotation is `-90°`
    /// about `+Y`; via `from_axis_angle` that's quaternion
    /// `(cos(-45°), 0, sin(-45°), 0)`.
    #[test]
    fn pca_orientation_long_x_axis_rotates_x_to_z() {
        // 11 points along +X with no spread on Y or Z. Length 1.0 m
        // along X, zero variance on Y / Z → principal axis = +X
        // (or -X; sign-pick flips to +X for positive z-target).
        let v: Vec<Point3<f64>> = (0..11)
            .map(|i| Point3::new(f64::from(i) * 0.1, 0.0, 0.0))
            .collect();
        let q = compute_pca_orientation(&v).unwrap();

        // Apply the rotation to +X — should land at +Z.
        let rotated = q.transform_vector(&Vector3::x());
        assert!((rotated.x).abs() < 1e-9, "rotated.x = {}", rotated.x);
        assert!((rotated.y).abs() < 1e-9, "rotated.y = {}", rotated.y);
        assert!((rotated.z - 1.0).abs() < 1e-9, "rotated.z = {}", rotated.z);
    }

    /// Vertices already long along `+Z` produce the identity
    /// rotation — no work needed. Pins the "no-op fast path" against
    /// silently producing a small rotation from FP noise.
    #[test]
    fn pca_orientation_long_z_axis_yields_identity() {
        let v: Vec<Point3<f64>> = (0..11)
            .map(|i| Point3::new(0.0, 0.0, f64::from(i) * 0.1))
            .collect();
        let q = compute_pca_orientation(&v).unwrap();
        assert!((q.w - 1.0).abs() < 1e-9, "q.w = {}", q.w);
        assert!(q.i.abs() < 1e-9, "q.i = {}", q.i);
        assert!(q.j.abs() < 1e-9, "q.j = {}", q.j);
        assert!(q.k.abs() < 1e-9, "q.k = {}", q.k);
    }

    /// Sign-pick check: the principal eigenvector for vertices
    /// stretched along `-Z` could come out as `+Z` or `-Z`; the
    /// helper flips to `+Z` (the side closer to the `+Z` target) so
    /// the resulting rotation is identity rather than a 180° flip.
    /// Sigil for "no rotation needed when the long axis ALREADY
    /// points up, even if the eigenvector signs out the other way."
    #[test]
    fn pca_orientation_sign_pick_prefers_positive_z() {
        // 11 points along ±Z spanning 1 m, symmetric about origin.
        // Centroid = origin; principal eigenvector axis = ±Z line.
        let v: Vec<Point3<f64>> = (0..11)
            .map(|i| Point3::new(0.0, 0.0, (f64::from(i) - 5.0) * 0.1))
            .collect();
        let q = compute_pca_orientation(&v).unwrap();
        // Identity — sign-pick collapsed both ±Z choices onto +Z.
        assert!((q.w - 1.0).abs() < 1e-9, "q.w = {}", q.w);
    }

    /// Cuboid mass distribution with the long axis along physics
    /// `+Y` produces a rotation that maps `+Y` to `+Z` — i.e. the
    /// 90°-about-X rotation behind the existing `[Snap +Y -> +Z]`
    /// button. Sanity check that PCA picks up dominant variance
    /// along a non-axis-of-largest-index direction.
    #[test]
    fn pca_orientation_long_y_axis_rotates_y_to_z() {
        // Cuboid: 0.05 m × 1.0 m × 0.05 m (long along +Y). Use 9
        // points covering corners + center to mimic mass spread.
        let v = vec![
            Point3::new(-0.025, -0.5, -0.025),
            Point3::new(0.025, -0.5, -0.025),
            Point3::new(-0.025, 0.5, -0.025),
            Point3::new(0.025, 0.5, -0.025),
            Point3::new(-0.025, -0.5, 0.025),
            Point3::new(0.025, -0.5, 0.025),
            Point3::new(-0.025, 0.5, 0.025),
            Point3::new(0.025, 0.5, 0.025),
            Point3::origin(),
        ];
        let q = compute_pca_orientation(&v).unwrap();
        let rotated = q.transform_vector(&Vector3::y());
        assert!((rotated.x).abs() < 1e-9, "rotated.x = {}", rotated.x);
        assert!((rotated.y).abs() < 1e-9, "rotated.y = {}", rotated.y);
        assert!((rotated.z - 1.0).abs() < 1e-9, "rotated.z = {}", rotated.z);
    }

    // ----- physics_quat_to_bevy (continued) -------------------------

    /// Identity physics quaternion maps to identity Bevy quaternion
    /// under the `UpAxis::PlusZ` swap. Sanity check that the
    /// scalar-form conversion preserves identity.
    #[test]
    fn physics_quat_to_bevy_preserves_identity() {
        let q_bevy = physics_quat_to_bevy_for_plus_z(UnitQuaternion::identity());
        assert!((q_bevy.w - 1.0).abs() < 1e-6);
        assert!(q_bevy.x.abs() < 1e-6);
        assert!(q_bevy.y.abs() < 1e-6);
        assert!(q_bevy.z.abs() < 1e-6);
    }

    /// Physics rotation `+90° about X` should map to Bevy rotation
    /// `-90° about X` under the parity-flipping `UpAxis::PlusZ` swap.
    /// This is the load-bearing case behind the `[Snap +Y -> +Z]`
    /// button — physics says "rotate +Y onto +Z", Bevy view says
    /// "rotate -Y onto +Z" (since Bevy's +Y is screen-up which equals
    /// physics +Z post-swap).
    ///
    /// Verification: the derived quaternion components should be
    /// `(cos(45°), -sin(45°), 0, 0)`.
    #[test]
    fn physics_quat_to_bevy_negates_x_component_for_x_rotation() {
        let q_phys = UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::x_axis(),
            std::f64::consts::FRAC_PI_2,
        );
        let q_bevy = physics_quat_to_bevy_for_plus_z(q_phys);

        #[allow(clippy::cast_possible_truncation)]
        let expected_cos = (std::f64::consts::FRAC_PI_4).cos() as f32;
        #[allow(clippy::cast_possible_truncation)]
        let expected_neg_sin = -(std::f64::consts::FRAC_PI_4).sin() as f32;
        assert!((q_bevy.w - expected_cos).abs() < 1e-6);
        assert!((q_bevy.x - expected_neg_sin).abs() < 1e-6);
        assert!(q_bevy.y.abs() < 1e-6);
        assert!(q_bevy.z.abs() < 1e-6);
    }

    /// Physics rotation `-90° about Y` (the `[Snap +X -> +Z]` case)
    /// should map to Bevy rotation `+90° about Z` (since physics Y
    /// becomes Bevy Z under the swap, and the angle flips sign).
    ///
    /// Verification: the derived quaternion components should be
    /// `(cos(45°), 0, 0, +sin(45°))`.
    #[test]
    fn physics_quat_to_bevy_swaps_y_and_z_axes() {
        let q_phys = UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::y_axis(),
            -std::f64::consts::FRAC_PI_2,
        );
        let q_bevy = physics_quat_to_bevy_for_plus_z(q_phys);

        #[allow(clippy::cast_possible_truncation)]
        let expected_cos = (std::f64::consts::FRAC_PI_4).cos() as f32;
        #[allow(clippy::cast_possible_truncation)]
        let expected_pos_sin = (std::f64::consts::FRAC_PI_4).sin() as f32;
        assert!((q_bevy.w - expected_cos).abs() < 1e-6);
        assert!(q_bevy.x.abs() < 1e-6);
        assert!(q_bevy.y.abs() < 1e-6);
        assert!((q_bevy.z - expected_pos_sin).abs() < 1e-6);
    }

    // ----- RecenterState + rotated_aabb_around_centroid_physics_mm --

    /// Default state is all-zeros (no translation). Pinned because
    /// changing the default would silently translate every loaded
    /// scan from the moment of load.
    #[test]
    fn recenter_state_default_is_zero() {
        let state = RecenterState::default();
        assert_eq!(state.tx_mm, 0.0);
        assert_eq!(state.ty_mm, 0.0);
        assert_eq!(state.tz_mm, 0.0);
    }

    /// `translation_world` applies the `UpAxis::PlusZ` swap (physics
    /// `(x, y, z)` → bevy `(x, z, y)`) and the render_scale lift in
    /// one step. Pinned because both transforms are easy to drift
    /// silently (e.g., forget to swap Y/Z, or forget render_scale).
    #[test]
    fn recenter_translation_world_swaps_and_scales() {
        let state = RecenterState {
            tx_mm: 100.0, // 0.1 m physics +X
            ty_mm: 200.0, // 0.2 m physics +Y
            tz_mm: 300.0, // 0.3 m physics +Z
        };
        // render_scale = 10 → 0.1 m × 10 = 1.0 world units.
        let world = state.translation_world(UpAxis::PlusZ, 10.0);
        // Physics +X → Bevy +X.
        assert!((world.x - 1.0).abs() < 1e-5);
        // Physics +Y → Bevy +Z.
        assert!((world.z - 2.0).abs() < 1e-5);
        // Physics +Z → Bevy +Y.
        assert!((world.y - 3.0).abs() < 1e-5);
    }

    /// Identity rotation: `rotated_aabb_around_centroid_physics_mm`
    /// returns the raw AABB scaled to mm with no axis permutation.
    #[test]
    fn rotated_aabb_identity_preserves_bounds_in_mm() {
        let raw = Aabb::from_corners(
            mesh_types::Point3::new(-0.05, -0.10, -0.15),
            mesh_types::Point3::new(0.05, 0.10, 0.15),
        );
        let (min_mm, max_mm) =
            rotated_aabb_around_centroid_physics_mm(UnitQuaternion::identity(), &raw);

        assert!((min_mm.x + 50.0).abs() < 1e-9);
        assert!((min_mm.y + 100.0).abs() < 1e-9);
        assert!((min_mm.z + 150.0).abs() < 1e-9);
        assert!((max_mm.x - 50.0).abs() < 1e-9);
        assert!((max_mm.y - 100.0).abs() < 1e-9);
        assert!((max_mm.z - 150.0).abs() < 1e-9);
    }

    /// 90° rotation about physics X axis: physics +Y becomes physics
    /// +Z; physics +Z becomes physics -Y. The rotated bbox's
    /// Y-extent equals the original Z-extent and vice-versa (signs
    /// shift accordingly). Test load: `(±0.05, ±0.10, ±0.15)` (mm:
    /// `±50, ±100, ±150`) rotated `+90°` about X:
    ///
    /// - X unchanged → `±50` mm.
    /// - Original Y `±100` becomes... actually the +90°-about-X
    ///   rotation maps `(0, 1, 0) -> (0, 0, 1)` and
    ///   `(0, 0, 1) -> (0, -1, 0)`. So the rotated bbox's Y range
    ///   is `±150` (= original Z range), and the rotated Z range is
    ///   `±100` (= original Y range).
    #[test]
    fn rotated_aabb_90deg_about_x_swaps_y_z_extents() {
        let raw = Aabb::from_corners(
            mesh_types::Point3::new(-0.05, -0.10, -0.15),
            mesh_types::Point3::new(0.05, 0.10, 0.15),
        );
        let rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::x_axis(),
            std::f64::consts::FRAC_PI_2,
        );
        let (min_mm, max_mm) = rotated_aabb_around_centroid_physics_mm(rot, &raw);

        // X unchanged.
        assert!((min_mm.x + 50.0).abs() < 1e-6);
        assert!((max_mm.x - 50.0).abs() < 1e-6);
        // Y now spans `±150` (was Z).
        assert!((min_mm.y + 150.0).abs() < 1e-6);
        assert!((max_mm.y - 150.0).abs() < 1e-6);
        // Z now spans `±100` (was Y, sign-flipped by the +90° rotation
        // but symmetric bbox -> same bounds).
        assert!((min_mm.z + 100.0).abs() < 1e-6);
        assert!((max_mm.z - 100.0).abs() < 1e-6);
    }

    /// Off-center AABB under centroid-pivot rotation: the AABB CENTER
    /// stays at the centroid (in mm) regardless of rotation; only the
    /// extents rotate. Test load: AABB from `(0.10, -0.05, 0.20)` to
    /// `(0.20, 0.05, 0.40)` — centroid at `(0.15, 0.0, 0.30)`. Under
    /// `+90°` about Y the half-extents `(50, 50, 100)` mm in
    /// `(x, y, z)` rotate to `(100, 50, 50)` mm. After translating the
    /// rotated centered AABB back to centroid (in mm `(150, 0, 300)`),
    /// the world AABB is centroid ± rotated_half_extents.
    ///
    /// **This test is THE pivot-vs-origin discriminator** — under the
    /// old origin-pivot helper, the rotated AABB would be far from the
    /// raw bbox; here we pin it to stay locked at the centroid.
    #[test]
    fn rotated_aabb_off_center_pivots_around_centroid_not_origin() {
        let raw = Aabb::from_corners(
            mesh_types::Point3::new(0.10, -0.05, 0.20),
            mesh_types::Point3::new(0.20, 0.05, 0.40),
        );
        let rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::y_axis(),
            std::f64::consts::FRAC_PI_2,
        );
        let (min_mm, max_mm) = rotated_aabb_around_centroid_physics_mm(rot, &raw);

        // Centroid in mm: (150, 0, 300).
        // Half-extents before rotation (mm): (50, 50, 100) in (x,y,z).
        // +90° about Y sends original X-half-extent (50) to Z and
        // original Z-half-extent (100) to -X (-100 → magnitude 100 in
        // X). So rotated half-extents: (100, 50, 50) in (x,y,z).
        // World AABB: centroid ± rotated_half_extents.
        assert!(
            (min_mm.x - (150.0 - 100.0)).abs() < 1e-6,
            "min.x = {}",
            min_mm.x
        );
        assert!(
            (max_mm.x - (150.0 + 100.0)).abs() < 1e-6,
            "max.x = {}",
            max_mm.x
        );
        assert!(
            (min_mm.y - (0.0 - 50.0)).abs() < 1e-6,
            "min.y = {}",
            min_mm.y
        );
        assert!(
            (max_mm.y - (0.0 + 50.0)).abs() < 1e-6,
            "max.y = {}",
            max_mm.y
        );
        assert!(
            (min_mm.z - (300.0 - 50.0)).abs() < 1e-6,
            "min.z = {}",
            min_mm.z
        );
        assert!(
            (max_mm.z - (300.0 + 50.0)).abs() < 1e-6,
            "max.z = {}",
            max_mm.z
        );
        // Center of the rotated world AABB == centroid in mm.
        let center_x = 0.5 * (min_mm.x + max_mm.x);
        let center_y = 0.5 * (min_mm.y + max_mm.y);
        let center_z = 0.5 * (min_mm.z + max_mm.z);
        assert!((center_x - 150.0).abs() < 1e-6);
        assert!(center_y.abs() < 1e-6);
        assert!((center_z - 300.0).abs() < 1e-6);
    }

    // ----- bake_vertex_with_pivot -----------------------------------

    /// Identity rotation + zero translation: bake is the identity
    /// transform (returns the input point unchanged regardless of
    /// centroid). Confirms the formula `R*(v-c)+c+t` collapses to `v`
    /// at the trivial case.
    #[test]
    fn bake_vertex_with_pivot_identity_is_passthrough() {
        let v = Point3::new(0.5, -0.3, 1.2);
        let c = Point3::new(0.1, 0.2, 0.3);
        let baked = bake_vertex_with_pivot(&v, UnitQuaternion::identity(), &c, Vector3::zeros());
        assert!((baked.x - v.x).abs() < 1e-12);
        assert!((baked.y - v.y).abs() < 1e-12);
        assert!((baked.z - v.z).abs() < 1e-12);
    }

    /// Centroid itself is fixed under any pivot rotation (rotation
    /// around centroid leaves the centroid in place by definition).
    /// Translation shifts it linearly.
    #[test]
    fn bake_vertex_with_pivot_centroid_is_fixed_point() {
        let c = Point3::new(0.15, 0.0, 0.30);
        let rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::y_axis(),
            std::f64::consts::FRAC_PI_2,
        );
        let t = Vector3::new(0.05, -0.10, 0.20);
        let baked = bake_vertex_with_pivot(&c, rot, &c, t);
        // baked = R*(c-c)+c+t = c+t
        assert!((baked.x - (c.x + t.x)).abs() < 1e-12);
        assert!((baked.y - (c.y + t.y)).abs() < 1e-12);
        assert!((baked.z - (c.z + t.z)).abs() < 1e-12);
    }

    /// 180° rotation around an off-origin centroid reflects every
    /// point through the centroid. Sigil: an off-origin vertex
    /// reflected through a non-origin pivot does NOT land at the
    /// origin-reflected position — the pivot matters.
    #[test]
    fn bake_vertex_with_pivot_180_reflects_through_centroid() {
        let v = Point3::new(0.20, 0.0, 0.30);
        let c = Point3::new(0.15, 0.0, 0.30);
        let rot =
            UnitQuaternion::from_axis_angle(&nalgebra::Vector3::y_axis(), std::f64::consts::PI);
        let baked = bake_vertex_with_pivot(&v, rot, &c, Vector3::zeros());
        // Reflection of v through c on the X axis: c.x - (v.x - c.x) = 0.10.
        // (Y axis 180° rotation also flips Z, but z component of v - c
        // is 0, so z stays at c.z = 0.30.)
        assert!((baked.x - 0.10).abs() < 1e-12, "baked.x = {}", baked.x);
        assert!(baked.y.abs() < 1e-12, "baked.y = {}", baked.y);
        assert!((baked.z - 0.30).abs() < 1e-12, "baked.z = {}", baked.z);
    }

    // ----- ClipState + clip_mesh_against_world_z ---------------------

    /// Default ClipState is `enabled=false, z_mm=0.0`. Pinned because
    /// flipping the default to enabled would silently clip every load.
    #[test]
    fn clip_state_default_is_disabled_at_origin() {
        let state = ClipState::default();
        assert!(!state.enabled);
        assert_eq!(state.z_mm, 0.0);
    }

    /// `compute_drop_percentage` on an empty mesh returns 0 (no
    /// vertices to drop).
    #[test]
    fn drop_percentage_empty_mesh_is_zero() {
        let empty = IndexedMesh::with_capacity(0, 0);
        let pct = compute_drop_percentage(&empty, UnitQuaternion::identity(), 0.0, 0.0);
        assert_eq!(pct, 0.0);
    }

    /// Identity rotation + zero translation: vertices below `clip_z_mm`
    /// drop, vertices on/above stay. Test mesh has 4 vertices at
    /// z = -50, -10, 10, 50 mm (in physics). Clip at z = 0 → 2 below
    /// (50%).
    #[test]
    fn drop_percentage_counts_below_plane() {
        let mut mesh = IndexedMesh::with_capacity(4, 0);
        // Z values in physics meters; will be lifted to mm inside the
        // function.
        mesh.vertices.push(Point3::new(0.0, 0.0, -0.050));
        mesh.vertices.push(Point3::new(0.0, 0.0, -0.010));
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.010));
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.050));
        let pct = compute_drop_percentage(&mesh, UnitQuaternion::identity(), 0.0, 0.0);
        assert!(
            (pct - 50.0).abs() < 1e-9,
            "2 of 4 vertices below z=0 → 50% expected, got {pct}",
        );
    }

    /// `clip_mesh_against_world_z` on a triangle entirely above the
    /// plane: kept as-is, no new vertices, face count = 1.
    #[test]
    fn clip_keeps_triangle_entirely_above_plane() {
        let mut mesh = IndexedMesh::with_capacity(3, 1);
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.010));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.020));
        mesh.vertices.push(Point3::new(0.0, 1.0, 0.030));
        mesh.faces.push([0, 1, 2]);
        let clipped = clip_mesh_against_world_z(
            &mesh,
            UnitQuaternion::identity(),
            0.0,
            0.0, // clip at z=0; all vertices have z > 0
        );
        assert_eq!(clipped.faces.len(), 1);
        assert_eq!(clipped.vertices.len(), 3); // no intersections, no unreferenced
    }

    /// `clip_mesh_against_world_z` on a triangle entirely below the
    /// plane: dropped, face count = 0, no surviving vertices.
    #[test]
    fn clip_drops_triangle_entirely_below_plane() {
        let mut mesh = IndexedMesh::with_capacity(3, 1);
        mesh.vertices.push(Point3::new(0.0, 0.0, -0.030));
        mesh.vertices.push(Point3::new(1.0, 0.0, -0.020));
        mesh.vertices.push(Point3::new(0.0, 1.0, -0.010));
        mesh.faces.push([0, 1, 2]);
        let clipped = clip_mesh_against_world_z(&mesh, UnitQuaternion::identity(), 0.0, 0.0);
        assert_eq!(clipped.faces.len(), 0);
        // remove_unreferenced_vertices strips all 3 since they're
        // not used by any surviving face.
        assert_eq!(clipped.vertices.len(), 0);
    }

    /// `clip_mesh_against_world_z` on a 1-above-2-below triangle:
    /// 1 surviving triangle with 2 intersection points.
    ///
    /// Test setup: triangle at z = 0.030, -0.010, -0.020 (in meters)
    /// → first vertex above z=0, other two below. Clip at z = 0
    /// produces:
    ///   - 1 surviving triangle
    ///   - 3 vertices: the original above-vertex + 2 intersection
    ///     points on the plane (z = 0 exactly)
    #[test]
    fn clip_one_above_two_below_produces_one_triangle() {
        let mut mesh = IndexedMesh::with_capacity(3, 1);
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.030)); // above
        mesh.vertices.push(Point3::new(1.0, 0.0, -0.010)); // below
        mesh.vertices.push(Point3::new(0.0, 1.0, -0.020)); // below
        mesh.faces.push([0, 1, 2]);
        let clipped = clip_mesh_against_world_z(&mesh, UnitQuaternion::identity(), 0.0, 0.0);
        assert_eq!(clipped.faces.len(), 1);
        assert_eq!(clipped.vertices.len(), 3);

        // The two intersection points must have z = 0 (within FP eps).
        let mut on_plane_count = 0;
        for v in &clipped.vertices {
            if v.z.abs() < 1e-9 {
                on_plane_count += 1;
            }
        }
        assert_eq!(
            on_plane_count, 2,
            "expected 2 intersection points on z=0 plane, got {on_plane_count}",
        );
    }

    /// `clip_mesh_against_world_z` on a 2-above-1-below triangle:
    /// 2 surviving triangles (fan-triangulation of the quad) with 2
    /// intersection points.
    #[test]
    fn clip_two_above_one_below_produces_two_triangles() {
        let mut mesh = IndexedMesh::with_capacity(3, 1);
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.030)); // above
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.020)); // above
        mesh.vertices.push(Point3::new(0.0, 1.0, -0.010)); // below
        mesh.faces.push([0, 1, 2]);
        let clipped = clip_mesh_against_world_z(&mesh, UnitQuaternion::identity(), 0.0, 0.0);
        assert_eq!(clipped.faces.len(), 2);
        // 2 original above-vertices + 2 intersection points = 4
        assert_eq!(clipped.vertices.len(), 4);

        // The two intersection points must have z = 0.
        let mut on_plane_count = 0;
        for v in &clipped.vertices {
            if v.z.abs() < 1e-9 {
                on_plane_count += 1;
            }
        }
        assert_eq!(on_plane_count, 2);
    }

    /// Spec edge case: a vertex EXACTLY on the plane should be treated
    /// as above, so the surrounding triangle stays intact (no
    /// degenerate sub-triangles). Triangle with one vertex on the
    /// plane + two above: stays whole, no intersection points
    /// generated.
    #[test]
    fn clip_vertex_exactly_on_plane_keeps_triangle() {
        let mut mesh = IndexedMesh::with_capacity(3, 1);
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0)); // on plane
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.020)); // above
        mesh.vertices.push(Point3::new(0.0, 1.0, 0.030)); // above
        mesh.faces.push([0, 1, 2]);
        let clipped = clip_mesh_against_world_z(&mesh, UnitQuaternion::identity(), 0.0, 0.0);
        assert_eq!(
            clipped.faces.len(),
            1,
            "triangle with one vertex on plane should be kept intact",
        );
        assert_eq!(clipped.vertices.len(), 3);
    }

    /// Rotation interaction: a triangle that's "above" in world frame
    /// but "below" in mesh-local frame should be kept. Tests that the
    /// rotation is correctly applied when computing the plane in
    /// mesh-local coordinates.
    ///
    /// Setup: triangle at mesh-local z = -0.020 (would be below
    /// world z=0 without rotation). Apply a 180° rotation about X
    /// (flips Z). After rotation, the triangle is at world z = +0.020,
    /// which is ABOVE the world clip plane at z=0. Should be kept.
    #[test]
    fn clip_respects_rotation_when_computing_plane() {
        let mut mesh = IndexedMesh::with_capacity(3, 1);
        mesh.vertices.push(Point3::new(0.0, 0.0, -0.020));
        mesh.vertices.push(Point3::new(1.0, 0.0, -0.020));
        mesh.vertices.push(Point3::new(0.0, 1.0, -0.020));
        mesh.faces.push([0, 1, 2]);
        // 180° rotation about X flips Y and Z signs.
        let rot_180_about_x =
            UnitQuaternion::from_axis_angle(&nalgebra::Vector3::x_axis(), std::f64::consts::PI);
        let clipped = clip_mesh_against_world_z(&mesh, rot_180_about_x, 0.0, 0.0);
        assert_eq!(
            clipped.faces.len(),
            1,
            "after 180° X rotation the triangle is at world z=+0.020, above clip plane",
        );
    }

    // ----- Cap algorithms (plane fit / normal / centerline) ---------

    /// `fit_plane_to_points` on a perfectly planar XY-plane loop:
    /// normal should be ±Z; R² should be 1.0; centroid at origin.
    #[test]
    fn plane_fit_on_xy_loop_returns_z_normal_and_unit_r_squared() {
        let points = vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, -1.0, 0.0),
        ];
        let (centroid, normal, r_sq) = fit_plane_to_points(&points);

        assert!(centroid.x.abs() < 1e-9);
        assert!(centroid.y.abs() < 1e-9);
        assert!(centroid.z.abs() < 1e-9);
        // Normal is ±Z (SVD doesn't pick a sign).
        assert!(normal.x.abs() < 1e-9, "normal x: {}", normal.x);
        assert!(normal.y.abs() < 1e-9, "normal y: {}", normal.y);
        assert!(
            (normal.z.abs() - 1.0).abs() < 1e-9,
            "normal z: {}",
            normal.z
        );
        // Planar loop → R² ≈ 1.
        assert!((r_sq - 1.0).abs() < 1e-9, "r² = {r_sq}");
    }

    /// `fit_plane_to_points` on a non-planar (twisted) loop has
    /// R² < 1. Four points lifted slightly off the XY plane in
    /// alternating directions; the best-fit plane is still ~XY but
    /// the residual is non-zero.
    #[test]
    fn plane_fit_on_twisted_loop_has_r_squared_below_one() {
        let points = vec![
            Point3::new(1.0, 0.0, 0.05),
            Point3::new(0.0, 1.0, -0.05),
            Point3::new(-1.0, 0.0, 0.05),
            Point3::new(0.0, -1.0, -0.05),
        ];
        let (_, _, r_sq) = fit_plane_to_points(&points);
        assert!(r_sq < 1.0);
        assert!(
            r_sq > 0.5,
            "non-trivial twist; r² should still be high: {r_sq}"
        );
    }

    /// `fit_plane_to_points` on < 3 input points returns the
    /// fallback (R² = 0, normal = +Z). Degenerate input shouldn't
    /// panic.
    #[test]
    fn plane_fit_degenerate_input_returns_fallback() {
        let too_few = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
        let (_, _, r_sq) = fit_plane_to_points(&too_few);
        assert_eq!(r_sq, 0.0);
    }

    /// `orient_cap_normal_outward` flips the normal so it points
    /// away from the side containing more mesh vertices. Build a
    /// mesh with all vertices above the cap plane (z > 0); the
    /// outward normal should be -Z (pointing away from mesh-side).
    #[test]
    fn orient_cap_normal_flips_to_point_away_from_mesh_majority() {
        let mut mesh = IndexedMesh::with_capacity(5, 0);
        for v in &[
            (0.0, 0.0, 1.0),
            (1.0, 0.0, 1.0),
            (0.0, 1.0, 1.0),
            (-1.0, 0.0, 2.0),
            (0.0, -1.0, 2.0),
        ] {
            mesh.vertices.push(Point3::new(v.0, v.1, v.2));
        }
        let plane_centroid = Point3::origin();
        // Try with normal pointing +Z (toward the mesh majority).
        let oriented =
            orient_cap_normal_outward(&mesh, plane_centroid, Vector3::new(0.0, 0.0, 1.0));
        // Outward = away from majority = -Z.
        assert!(
            oriented.z < 0.0,
            "expected outward normal to point -Z; got {:?}",
            oriented,
        );
    }

    /// `compute_centerline_polyline` on an elongated mesh along
    /// physics +X axis: returned polyline points should themselves
    /// lie roughly along +X (each slab's centroid stays near the
    /// shape's mid-Y / mid-Z).
    #[test]
    fn centerline_along_elongated_x_axis_follows_x() {
        // Build a cigar-shaped point cloud: vertices at
        // (x, 0, 0) for x in [-1, 1] plus a few off-axis "skin"
        // vertices at each x. Centroid of each x-slab should be
        // ~(x, 0, 0).
        let mut mesh = IndexedMesh::with_capacity(100, 0);
        for i in 0..20 {
            #[allow(clippy::cast_precision_loss)]
            let x = (i as f64) * 0.1 - 1.0;
            for theta_step in 0..5 {
                #[allow(clippy::cast_precision_loss)]
                let theta = (theta_step as f64) * std::f64::consts::TAU / 5.0;
                mesh.vertices
                    .push(Point3::new(x, 0.1 * theta.cos(), 0.1 * theta.sin()));
            }
        }
        let polyline = compute_centerline_polyline(&mesh, Vector3::new(1.0, 0.0, 0.0), 10);
        assert!(
            polyline.len() >= 5,
            "expected ≥5 polyline pts; got {}",
            polyline.len()
        );
        // Each polyline point should have y ≈ 0 and z ≈ 0 (centroid
        // of a symmetric ring around the x-axis).
        for p in &polyline {
            assert!(p.y.abs() < 0.01, "polyline y drifted: {p:?}");
            assert!(p.z.abs() < 0.01, "polyline z drifted: {p:?}");
        }
        // Polyline x values should monotonically increase from min
        // to max along the spine.
        let xs: Vec<f64> = polyline.iter().map(|p| p.x).collect();
        for window in xs.windows(2) {
            assert!(
                window[1] > window[0],
                "polyline x should monotonically increase: {xs:?}",
            );
        }
    }

    /// `compute_centerline_polyline` on empty mesh / zero-direction
    /// input returns an empty Vec without panicking.
    #[test]
    fn centerline_degenerate_input_returns_empty() {
        let empty = IndexedMesh::with_capacity(0, 0);
        let polyline = compute_centerline_polyline(&empty, Vector3::new(1.0, 0.0, 0.0), 10);
        assert!(polyline.is_empty());

        let mut tiny = IndexedMesh::with_capacity(1, 0);
        tiny.vertices.push(Point3::new(0.0, 0.0, 0.0));
        let zero_dir = compute_centerline_polyline(&tiny, Vector3::zeros(), 10);
        assert!(zero_dir.is_empty());
    }

    // ----- Centerline trim (CSP.4b) ----------------------------------

    /// `trim_mesh_along_centerline` with both trims at 0 is a
    /// pure-clone no-op (returns the input unchanged). Pins the
    /// fast-path guard so future refactors don't accidentally walk
    /// vertices unnecessarily for un-trimmed saves.
    #[test]
    fn trim_mesh_along_centerline_no_op_when_both_trims_zero() {
        let mesh = one_triangle_at(0.01);
        let centerline = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 1.0)];
        let trimmed = trim_mesh_along_centerline(&mesh, &centerline, 0.0, 0.0);
        assert_eq!(trimmed.vertices, mesh.vertices);
        assert_eq!(trimmed.faces, mesh.faces);
    }

    /// Degenerate centerline (< 2 points) is a no-op. Defensive
    /// against the "Cap → Scan not yet clicked" path (CapState
    /// centerline empty); the save handler skips trim entirely in
    /// that case, but the function itself guards too.
    #[test]
    fn trim_mesh_along_centerline_no_op_when_centerline_too_short() {
        let mesh = one_triangle_at(0.01);
        let centerline = vec![Point3::new(0.0, 0.0, 0.0)];
        let trimmed = trim_mesh_along_centerline(&mesh, &centerline, 10.0, 10.0);
        assert_eq!(trimmed.vertices, mesh.vertices);
        assert_eq!(trimmed.faces, mesh.faces);
    }

    /// Trim from the floor end of a +Z-axis centerline drops
    /// vertices beyond the trim plane. Vertical "pole" of 11 points
    /// from z=0 to z=0.1 (100 mm), centerline tip→floor along +Z,
    /// trim floor by 30 mm → keep points with z <= 0.07.
    #[test]
    fn trim_mesh_along_centerline_floor_end_clips_high_z_vertices() {
        // Build a simple vertical column of triangles (pole), 11
        // segments stacked from z=0 to z=0.1. Each "stack-rung" is
        // a degenerate-ish 3-vertex triangle for testing.
        let mut mesh = IndexedMesh::with_capacity(11, 9);
        for i in 0..11 {
            let z = f64::from(i) * 0.01;
            mesh.vertices.push(Point3::new(0.0, 0.0, z));
        }
        // Connect into a chain of "triangles" with shared verts.
        // (Geometrically degenerate but valid topology; serves the
        // trim test.)
        for i in 0..9 {
            mesh.faces.push([i as u32, (i + 1) as u32, (i + 2) as u32]);
        }
        // Centerline from tip (z=0) to floor (z=0.1). Trim floor
        // by 30 mm → clip plane at z = 0.07.
        let centerline = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.1)];
        let trimmed = trim_mesh_along_centerline(&mesh, &centerline, 0.0, 30.0);
        // All surviving vertices' z <= 0.07 (the trim plane).
        for v in &trimmed.vertices {
            assert!(v.z <= 0.07 + 1e-9, "kept vertex z={} above trim plane", v.z);
        }
        // Some vertices survived.
        assert!(!trimmed.vertices.is_empty(), "all dropped by trim");
    }

    /// `trim_centerline_polyline` keeps the polyline whole when no
    /// trim is requested. Pins the no-op identity.
    #[test]
    fn trim_centerline_polyline_no_op_when_both_zero() {
        let polyline = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.05),
            Point3::new(0.0, 0.0, 0.1),
        ];
        let trimmed = trim_centerline_polyline(&polyline, 0.0, 0.0);
        assert_eq!(trimmed, polyline);
    }

    /// `trim_centerline_polyline` cuts the start + end of the
    /// polyline at the requested mm distances. 100 mm polyline,
    /// trim 30 mm tip + 20 mm floor → result is 50 mm long, from
    /// the 30 mm mark to the 80 mm mark.
    #[test]
    fn trim_centerline_polyline_clips_both_ends_at_arc_length() {
        let polyline = vec![
            Point3::new(0.0, 0.0, 0.0),   //  0 mm
            Point3::new(0.0, 0.0, 0.025), // 25 mm
            Point3::new(0.0, 0.0, 0.05),  // 50 mm
            Point3::new(0.0, 0.0, 0.075), // 75 mm
            Point3::new(0.0, 0.0, 0.1),   // 100 mm
        ];
        let trimmed = trim_centerline_polyline(&polyline, 30.0, 20.0);
        // First trimmed point: z = 0.030 (interpolated between 25
        // and 50 mm originals).
        assert!((trimmed.first().unwrap().z - 0.030).abs() < 1e-9);
        // Last trimmed point: z = 0.080.
        assert!((trimmed.last().unwrap().z - 0.080).abs() < 1e-9);
        // Total arc length collapsed from 100 mm to ~50 mm.
        let new_total_mm: f64 = trimmed
            .windows(2)
            .map(|w| (w[1].coords - w[0].coords).norm() * 1000.0)
            .sum();
        assert!((new_total_mm - 50.0).abs() < 1e-6);
    }

    /// `trim_centerline_polyline` returns empty when the trim
    /// consumes the entire polyline (`trim_tip + trim_floor >=
    /// total arc length`). The save handler's
    /// `centerline.is_empty()` check then omits the `[centerline]`
    /// block from the TOML.
    #[test]
    fn trim_centerline_polyline_returns_empty_when_fully_consumed() {
        let polyline = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.1), // 100 mm total
        ];
        // Trim 60 + 60 = 120 mm > 100 mm total.
        let trimmed = trim_centerline_polyline(&polyline, 60.0, 60.0);
        assert!(trimmed.is_empty());
    }

    /// CenterlineTrimState defaults are zero — the trim is opt-in.
    /// Pins the "no trim by default" semantic so future refactors
    /// don't accidentally enable a trim baseline.
    #[test]
    fn centerline_trim_state_default_is_zero() {
        let state = CenterlineTrimState::default();
        assert_eq!(state.trim_tip_mm, 0.0);
        assert_eq!(state.trim_floor_mm, 0.0);
    }

    /// CSP.4b.3 regression test — the polyline-walker primitive
    /// that drives both the trim algorithm + the live overlay.
    /// Pre-CSP.4b.3 trim/overlay code extrapolated linearly from
    /// the first segment's tangent, which diverged from the actual
    /// curve as the trim distance grew. This test builds a
    /// L-shaped polyline (sharp 90° bend at mid-arc) and asserts
    /// that walking past the bend lands on the SECOND-leg
    /// position, NOT linearly extrapolated from the first leg's
    /// tangent.
    #[test]
    fn point_along_polyline_walks_through_a_bend() {
        // L-shaped polyline:
        //   (0,0,0) → (0,0,0.05) → (0.05,0,0.05)
        // Total arc length: 0.05 + 0.05 = 0.10 m (100 mm).
        // First leg (+Z, 50 mm); second leg (+X, 50 mm); 90° bend
        // at index 1.
        let polyline = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.05),
            Point3::new(0.05, 0.0, 0.05),
        ];

        // 25 mm in: on the first leg. Expect (0, 0, 0.025), tangent = +Z.
        let (p, t) = point_along_polyline_at_arc_distance(&polyline, 0.025).unwrap();
        assert!((p.x - 0.0).abs() < 1e-12);
        assert!((p.z - 0.025).abs() < 1e-12);
        assert!((t.z - 1.0).abs() < 1e-9);
        assert!(t.x.abs() < 1e-9);

        // 75 mm in: 25 mm into the SECOND leg. Expect (0.025, 0,
        // 0.05), tangent = +X. The pre-CSP.4b.3 buggy code would
        // have walked 75 mm along the first leg's +Z tangent and
        // landed at (0, 0, 0.075) — way off the actual polyline.
        let (p, t) = point_along_polyline_at_arc_distance(&polyline, 0.075).unwrap();
        assert!(
            (p.x - 0.025).abs() < 1e-12,
            "x post-bend should be 0.025, got {}",
            p.x
        );
        assert!(
            (p.z - 0.05).abs() < 1e-12,
            "z post-bend should be 0.05, got {}",
            p.z
        );
        assert!(
            (t.x - 1.0).abs() < 1e-9,
            "tangent post-bend should be +X, got {t:?}"
        );
        assert!(t.z.abs() < 1e-9);
    }

    /// `point_along_polyline_at_arc_distance` returns `None` for a
    /// polyline with < 2 points (no segment to walk). Pairs with
    /// the trim algorithm's "skip if centerline too short" path.
    #[test]
    fn point_along_polyline_returns_none_when_too_short() {
        let single = vec![Point3::origin()];
        assert!(point_along_polyline_at_arc_distance(&single, 0.0).is_none());
        let empty: Vec<Point3<f64>> = Vec::new();
        assert!(point_along_polyline_at_arc_distance(&empty, 0.1).is_none());
    }

    /// `polyline_arc_length_m` sums Euclidean segment lengths.
    #[test]
    fn polyline_arc_length_sums_segments() {
        let polyline = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.05),
            Point3::new(0.05, 0.0, 0.05),
            Point3::new(0.05, 0.0, 0.0),
        ];
        // 50 + 50 + 50 mm = 0.15 m.
        let total = polyline_arc_length_m(&polyline);
        assert!((total - 0.15).abs() < 1e-12);
    }

    // ----- build_overlay_lengths -------------------------------------

    /// `build_overlay_lengths` derives the axis arrow length from the
    /// scaled diagonal (`0.6 × scaled_diagonal`) + applies the
    /// `UpAxis::PlusZ` swap to the bbox center/dims (physics Y/Z swap
    /// into Bevy frame).
    ///
    /// Fixture: a triangle whose vertices land at the corners of a
    /// `(0.1, 0.2, 0.3)` physics-space bbox. With `render_scale = 10`:
    /// scaled bbox = `(1.0, 2.0, 3.0)`; diagonal = `sqrt(14) ≈ 3.74`.
    /// Center (in physics) = `(0.5, 1.0, 1.5)`; after `PlusZ` swap →
    /// Bevy `(0.5, 1.5, 1.0)`. Dims swap to `(1.0, 3.0, 2.0)`.
    #[test]
    fn build_overlay_lengths_swaps_axes_and_scales_diagonal() {
        let mut mesh = IndexedMesh::with_capacity(3, 1);
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.1, 0.2, 0.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.3));
        mesh.faces.push([0, 1, 2]);

        let overlays = build_overlay_lengths(&mesh, 10.0, UpAxis::PlusZ);

        // Bevy bbox center: physics (0.05, 0.10, 0.15) × 10 swapped to
        // (0.5, 1.5, 1.0).
        assert!((overlays.bbox_center_bevy.x - 0.5).abs() < 1e-5);
        assert!((overlays.bbox_center_bevy.y - 1.5).abs() < 1e-5);
        assert!((overlays.bbox_center_bevy.z - 1.0).abs() < 1e-5);

        // Bevy bbox dims: physics (0.1, 0.2, 0.3) × 10 swapped to
        // (1.0, 3.0, 2.0).
        assert!((overlays.bbox_dims_bevy.x - 1.0).abs() < 1e-5);
        assert!((overlays.bbox_dims_bevy.y - 3.0).abs() < 1e-5);
        assert!((overlays.bbox_dims_bevy.z - 2.0).abs() < 1e-5);

        // Arrow length: 0.6 × scaled diagonal = 0.6 × sqrt(14) ≈ 2.245.
        let expected = 0.6 * 14.0_f32.sqrt();
        assert!(
            (overlays.arrow_length - expected).abs() < 1e-5,
            "arrow_length {} should match 0.6 × scaled diagonal {expected}",
            overlays.arrow_length,
        );

        // Raw AABB cached in physics meters (no swap, no scale).
        assert!((overlays.raw_aabb_m.min.z - 0.0).abs() < 1e-9);
        assert!((overlays.raw_aabb_m.max.z - 0.3).abs() < 1e-9);

        // Slider range = 2 × max raw dim = 2 × 0.3 m × 1000 = 600 mm.
        assert!(
            (overlays.recenter_slider_range_mm - 600.0).abs() < 1e-6,
            "recenter_slider_range_mm {} should match 2 × 300 mm",
            overlays.recenter_slider_range_mm,
        );
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

    // ----- Save panel helpers ----------------------------------------

    /// A unit square at the origin (CCW in XY). 4 vertices, 2-triangle
    /// triangulation expected from `triangulate_polygon_2d_earclip`.
    fn unit_square_2d() -> Vec<(f64, f64)> {
        vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    }

    /// Ear-clipping a unit square produces exactly 2 triangles
    /// covering the full polygon area (4 area units of 0.5 each).
    /// Validates the basic ear-clip + signed-area path.
    #[test]
    fn earclip_unit_square_produces_two_triangles() {
        let verts = unit_square_2d();
        let tris = triangulate_polygon_2d_earclip(&verts);
        assert_eq!(tris.len(), 2);
        // Total signed area of the triangulation should equal the
        // square's area (1.0). Sums absolute signed area per tri.
        let total_area: f64 = tris
            .iter()
            .map(|t| {
                let a = verts[t[0] as usize];
                let b = verts[t[1] as usize];
                let c = verts[t[2] as usize];
                0.5 * ((b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)).abs()
            })
            .sum();
        assert!(
            (total_area - 1.0).abs() < 1e-12,
            "expected total area 1.0, got {total_area}",
        );
    }

    /// Reversing a CCW polygon to CW must still produce a valid
    /// 2-triangle triangulation. Pins the `signed_area < 0 -> reverse`
    /// branch of the ear-clip.
    #[test]
    fn earclip_cw_polygon_is_handled() {
        let mut verts = unit_square_2d();
        verts.reverse();
        let tris = triangulate_polygon_2d_earclip(&verts);
        assert_eq!(tris.len(), 2);
    }

    /// Polygons with `n < 3` vertices produce no triangles (early
    /// return). Otherwise an `n=3` polygon should produce exactly
    /// 1 triangle.
    #[test]
    fn earclip_handles_degenerate_vertex_counts() {
        assert!(triangulate_polygon_2d_earclip(&[]).is_empty());
        assert!(triangulate_polygon_2d_earclip(&[(0.0, 0.0)]).is_empty());
        assert!(triangulate_polygon_2d_earclip(&[(0.0, 0.0), (1.0, 0.0)]).is_empty());
        let single = triangulate_polygon_2d_earclip(&[(0.0, 0.0), (1.0, 0.0), (0.5, 1.0)]);
        assert_eq!(single.len(), 1);
    }

    /// A convex pentagon (5 vertices) produces 3 triangles
    /// (`n - 2 = 3` for any simple polygon).
    #[test]
    fn earclip_convex_pentagon_produces_three_triangles() {
        // Regular-ish pentagon at unit radius.
        let verts = (0..5)
            .map(|i| {
                let theta = i as f64 * std::f64::consts::TAU / 5.0;
                (theta.cos(), theta.sin())
            })
            .collect::<Vec<_>>();
        let tris = triangulate_polygon_2d_earclip(&verts);
        assert_eq!(tris.len(), 3);
    }

    /// `iso8601_utc_from_unix_seconds` produces a parseable, sensible
    /// timestamp. Pins the date math against a known unix epoch:
    /// `1640995200` = `2022-01-01T00:00:00Z`.
    #[test]
    fn iso8601_timestamp_matches_known_unix_epoch() {
        let s = iso8601_utc_from_unix_seconds(1_640_995_200);
        assert_eq!(s, "2022-01-01T00:00:00Z");
    }

    /// `build_prep_toml_string` produces a string that re-parses as
    /// TOML and contains every spec-promised top-level block under
    /// the v1.0 completion schema (CSP.1 rename, 2026-05-15). Pins
    /// the on-disk shape against `docs/SCAN_PREP_DESIGN.md` §Output
    /// format so silent block-renames surface here.
    #[test]
    fn prep_toml_string_round_trips_through_toml_parser() -> Result<()> {
        let reorient = ReorientState::default();
        let recenter = RecenterState::default();
        let clip = ClipState::default();
        let cap = CapState::default();
        let rotation = reorient.quaternion_physics();
        let translation = nalgebra::Vector3::zeros();
        let cleaned_aabb = Aabb {
            min: Point3::new(-0.04, -0.03, -0.06),
            max: Point3::new(0.04, 0.03, 0.06),
        };
        let centerline_trim = CenterlineTrimState::default();
        let s = build_prep_toml_string(
            Path::new("/tmp/scan.stl"),
            StlUnits::Mm,
            Vector3::new(0.01, -0.02, 0.03), // auto_center_offset_m
            None,                            // CSP.4a auto_pca_quat — omit for default
            &reorient,
            &recenter,
            &clip,
            &cap,
            &centerline_trim, // CSP.4b — default = no trim
            0,                // CSP.4b — no boundaries auto-capped
            "scan.cleaned.stl",
            rotation,
            translation,
            Point3::origin(),
            200_000, // simplify target
            500_000, // original face count
            200_000, // achieved face count
            &cleaned_aabb,
        )?;
        let parsed: toml::Value = toml::from_str(&s)?;
        assert!(parsed.get("scan_prep").is_some());
        // CSP.3.5 — [scan_prep].auto_center_offset_m present and
        // matches the value passed in.
        let scan_prep = parsed.get("scan_prep").expect("scan_prep block");
        let offset = scan_prep
            .get("auto_center_offset_m")
            .and_then(|v| v.as_array())
            .expect("auto_center_offset_m missing or not an array");
        assert_eq!(offset.len(), 3);
        assert!((offset[0].as_float().unwrap() - 0.01).abs() < 1e-12);
        assert!((offset[1].as_float().unwrap() - (-0.02)).abs() < 1e-12);
        assert!((offset[2].as_float().unwrap() - 0.03).abs() < 1e-12);
        // CSP.4a — auto_pca_quaternion is `None` here (we passed None
        // in this test), so the field should be SKIPPED in the
        // serialized output (per `skip_serializing_if`).
        assert!(
            scan_prep.get("auto_pca_quaternion").is_none(),
            "auto_pca_quaternion should be omitted when None",
        );
        assert!(parsed.get("simplify").is_some(), "simplify block missing");
        assert!(parsed.get("transform").is_some(), "transform block missing");
        // Sub-tables under [transform] per spec §Output format.
        let transform = parsed.get("transform").expect("transform block");
        assert!(
            transform.get("rotation").is_some(),
            "transform.rotation missing"
        );
        assert!(
            transform.get("translation").is_some(),
            "transform.translation missing"
        );
        assert!(parsed.get("clip").is_some());
        assert!(parsed.get("caps").is_some());
        assert!(parsed.get("output").is_some());
        // [output.aabb_m] sub-table per spec §Output format.
        let output = parsed.get("output").expect("output block");
        assert!(output.get("aabb_m").is_some(), "output.aabb_m missing");
        // Legacy block names from the as-built schema must NOT
        // resurface — guards against accidental re-introduction.
        assert!(parsed.get("reorient").is_none(), "legacy reorient name");
        assert!(parsed.get("recenter").is_none(), "legacy recenter name");
        // No cap loops -> centerline block omitted.
        assert!(parsed.get("centerline").is_none());
        // CSP.4b — [centerline_trim] block always present, even
        // when no trim was requested.
        let trim = parsed
            .get("centerline_trim")
            .expect("centerline_trim block missing");
        assert!((trim.get("trim_tip_mm").unwrap().as_float().unwrap() - 0.0).abs() < 1e-12);
        assert!((trim.get("trim_floor_mm").unwrap().as_float().unwrap() - 0.0).abs() < 1e-12);
        assert_eq!(trim.get("capped_loops").unwrap().as_integer().unwrap(), 0);
        Ok(())
    }

    /// CSP.4a — when `auto_pca_quat` is `Some`, the `.prep.toml`
    /// `[scan_prep].auto_pca_quaternion` field round-trips as a
    /// 4-element array of `(w, x, y, z)` matching the input
    /// quaternion. Companion to the `None` case in
    /// `prep_toml_string_round_trips_through_toml_parser`.
    #[test]
    fn prep_toml_serializes_auto_pca_quaternion_when_present() -> Result<()> {
        let reorient = ReorientState::default();
        let recenter = RecenterState::default();
        let clip = ClipState::default();
        let cap = CapState::default();
        let rotation = reorient.quaternion_physics();
        let translation = nalgebra::Vector3::zeros();
        let cleaned_aabb = Aabb {
            min: Point3::new(0.0, 0.0, 0.0),
            max: Point3::new(0.1, 0.1, 0.1),
        };
        // A non-identity quaternion (~90° about +X).
        let q = UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::x_axis(),
            std::f64::consts::FRAC_PI_2,
        );
        let centerline_trim = CenterlineTrimState::default();
        let s = build_prep_toml_string(
            Path::new("/tmp/scan.stl"),
            StlUnits::Mm,
            Vector3::zeros(),
            Some(q),
            &reorient,
            &recenter,
            &clip,
            &cap,
            &centerline_trim,
            0,
            "scan.cleaned.stl",
            rotation,
            translation,
            Point3::origin(),
            200_000,
            500_000,
            200_000,
            &cleaned_aabb,
        )?;
        let parsed: toml::Value = toml::from_str(&s)?;
        let scan_prep = parsed.get("scan_prep").expect("scan_prep block");
        let arr = scan_prep
            .get("auto_pca_quaternion")
            .and_then(|v| v.as_array())
            .expect("auto_pca_quaternion missing or not an array");
        assert_eq!(arr.len(), 4);
        assert!((arr[0].as_float().unwrap() - q.w).abs() < 1e-12);
        assert!((arr[1].as_float().unwrap() - q.i).abs() < 1e-12);
        assert!((arr[2].as_float().unwrap() - q.j).abs() < 1e-12);
        assert!((arr[3].as_float().unwrap() - q.k).abs() < 1e-12);
        Ok(())
    }

    /// `build_cleaned_mesh` bakes Reorient + Recenter into vertex
    /// positions: an identity Reorient + +x translation moves all
    /// vertices by +x. Default state means no caps are appended,
    /// `ClipState::default()` keeps the clip disabled so the function
    /// returns the transform-baked mesh verbatim.
    #[test]
    fn build_cleaned_mesh_bakes_recenter_translation() {
        let mesh = one_triangle_at(0.001);
        let recenter = RecenterState {
            tx_mm: 100.0, // +100mm = +0.1m
            ..RecenterState::default()
        };
        let cleaned = build_cleaned_mesh(
            &mesh,
            &ReorientState::default(),
            &recenter,
            &ClipState::default(),
            &CapState::default(),
        );
        assert_eq!(cleaned.vertices.len(), 3);
        assert!(
            (cleaned.vertices[0].x - (0.001 + 0.1)).abs() < 1e-12,
            "first vertex should be translated by +0.1m, got {}",
            cleaned.vertices[0].x,
        );
    }

    /// CSP.2 — enabling Clip with a Z plane mid-mesh drops the
    /// triangles entirely below the plane. Builds a 5-tall vertical
    /// strip from a single triangle at meters 0 / 0.5 / 1, clips at
    /// world Z = 250 mm, and confirms the result keeps only the
    /// portion above. Pins the wiring from `ClipState` through
    /// `build_cleaned_mesh` step 4.
    #[test]
    fn build_cleaned_mesh_applies_clip_when_enabled() {
        // Triangle with vertices at z = 0.0, 0.5, 1.0 (meters).
        // Clip plane at world z = 0.25 m → vertex 0 (z=0) below;
        // vertices 1 + 2 above. True-plane intersection produces a
        // 4-vertex polygon (2 originals above + 2 new on the plane)
        // → 2 triangles after fan triangulation.
        let mut mesh = IndexedMesh::with_capacity(3, 1);
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.5));
        mesh.vertices.push(Point3::new(0.0, 1.0, 1.0));
        mesh.faces.push([0, 1, 2]);

        let clip = ClipState {
            enabled: true,
            z_mm: 250.0, // 0.25 m
            ..ClipState::default()
        };
        let cleaned = build_cleaned_mesh(
            &mesh,
            &ReorientState::default(),
            &RecenterState::default(),
            &clip,
            &CapState::default(),
        );

        // Every remaining vertex's z is at or above the clip plane.
        for v in &cleaned.vertices {
            assert!(
                v.z >= 0.25 - 1e-9,
                "vertex z={} below clip plane (0.25 m)",
                v.z,
            );
        }
        // Some faces survived (the above-plane portion).
        assert!(!cleaned.faces.is_empty(), "all faces dropped by clip");
    }

    /// CSP.2 — disabled Clip leaves the mesh untouched regardless of
    /// `z_mm`. Guards against accidental "always-on" wiring that
    /// would silently clip default-enabled-by-mistake states.
    #[test]
    fn build_cleaned_mesh_skips_clip_when_disabled() {
        let mesh = one_triangle_at(0.001);
        let clip = ClipState {
            enabled: false,
            z_mm: 100.0, // would clip everything if enabled
            ..ClipState::default()
        };
        let cleaned = build_cleaned_mesh(
            &mesh,
            &ReorientState::default(),
            &RecenterState::default(),
            &clip,
            &CapState::default(),
        );
        assert_eq!(cleaned.vertices.len(), 3);
        assert_eq!(cleaned.faces.len(), 1);
    }

    /// CSP.3a — for an included cap loop, the loop vertices are
    /// projected onto the fit plane BEFORE ear-clip so the resulting
    /// cap faces are planar. Builds a 4-vertex wobble-rim square
    /// loop (with one vertex pushed off-plane), wires it through a
    /// `CapState`, and confirms the post-build vertex positions sit
    /// on the fit plane to within FP noise. The pre-CSP.3a code path
    /// would leave the off-plane vertex untouched (the cap face
    /// would draw from that wobbly position → non-planar cap →
    /// mesh_sdf garbage).
    #[test]
    fn build_cleaned_mesh_projects_loop_verts_onto_fit_plane() {
        // 4 boundary vertices roughly on z=0 plane. Vertex 1 is
        // pushed UP to z=0.005 m (5 mm wobble) to simulate a low-
        // R² rim.
        let mut mesh = IndexedMesh::with_capacity(4, 0);
        mesh.vertices.push(Point3::new(0.000, 0.000, 0.0));
        mesh.vertices.push(Point3::new(0.010, 0.000, 0.005)); // wobble
        mesh.vertices.push(Point3::new(0.010, 0.010, 0.0));
        mesh.vertices.push(Point3::new(0.000, 0.010, 0.0));

        // Cap loop with the fit plane = world XY (centroid at
        // (5, 5, ~1) mm, normal = +Z). orient_cap_normal_outward
        // would flip the normal depending on mesh-majority, but
        // for this synthetic fixture +Z is fine.
        let cap_loop = DetectedCapLoop {
            vertex_indices: vec![0, 1, 2, 3],
            plane_centroid: Point3::new(0.005, 0.005, 0.0),
            plane_normal: Vector3::new(0.0, 0.0, 1.0),
            plane_fit_r_squared: 0.95,
            include: true,
        };
        let cap = CapState {
            loops: vec![cap_loop],
            ..CapState::default()
        };

        let cleaned = build_cleaned_mesh(
            &mesh,
            &ReorientState::default(),
            &RecenterState::default(),
            &ClipState::default(),
            &cap,
        );

        // All 4 boundary vertices must now lie on the z=0 plane
        // (centroid.z = 0, normal = +Z, so projection sends each
        // vertex's z component to 0).
        for v in &cleaned.vertices {
            assert!(
                v.z.abs() < 1e-12,
                "loop vertex z={} not projected onto fit plane",
                v.z,
            );
        }
        // Cap faces appended (2 triangles from a 4-vertex square).
        assert!(
            !cleaned.faces.is_empty(),
            "cap triangulation produced zero faces",
        );
    }

    /// Build the STL-style (per-triangle unshared) version of an
    /// IndexedMesh — `mesh_io::load_stl` produces this layout because
    /// binary STL stores each triangle's 3 vertices independently
    /// without index sharing. cf-scan-prep's hygiene pass is supposed
    /// to weld these back to the shared layout downstream
    /// `simplify_decoder` requires. Test helper, not production code.
    fn unshare_vertices(mesh: &IndexedMesh) -> IndexedMesh {
        let n_faces = mesh.faces.len();
        let mut out = IndexedMesh::with_capacity(n_faces * 3, n_faces);
        #[allow(clippy::cast_possible_truncation)]
        for face in &mesh.faces {
            let base = out.vertices.len() as u32;
            for &idx in face {
                out.vertices.push(mesh.vertices[idx as usize]);
            }
            out.faces.push([base, base + 1, base + 2]);
        }
        out
    }

    /// CSP.3b — `cleanup_cleaned_mesh_for_disk` welds STL-style
    /// unshared vertices into the shared-index layout that
    /// downstream `simplify_decoder` requires. Builds a 4×4
    /// grid-square (32 faces — comfortably above the 10-face
    /// small-component floor), STL-unshares it via the test helper,
    /// runs cleanup, and confirms the welded result has the shared-
    /// vertex topology the iter-1 fixture's downstream consumer was
    /// missing (`tools/cf-device-design/src/main.rs:419-425`).
    #[test]
    fn cleanup_welds_stl_style_unshared_vertices() {
        let shared = grid_square(4); // 32 faces, 25 shared verts
        let shared_vert_count = shared.vertices.len();
        let shared_face_count = shared.faces.len();
        assert!(shared_face_count >= CLEANUP_MIN_COMPONENT_FACES);
        let mut unshared = unshare_vertices(&shared);
        // STL-unshared form: 3 verts per face.
        assert_eq!(unshared.vertices.len(), shared_face_count * 3);

        let report = cleanup_cleaned_mesh_for_disk(&mut unshared);
        assert!(
            report.welded > 0,
            "weld_vertices report welded=0: {report:?}",
        );
        // Welded back down to the shared form (25 verts) with all
        // faces surviving.
        assert_eq!(unshared.vertices.len(), shared_vert_count);
        assert_eq!(unshared.faces.len(), shared_face_count);
    }

    /// CSP.3b — small-component strip drops scanner-noise islands
    /// below `CLEANUP_MIN_COMPONENT_FACES`. Builds a 20-face main
    /// component + a single 1-face island; cleanup keeps the main
    /// component + drops the island.
    #[test]
    fn cleanup_drops_small_component_islands() {
        // Main component: 5×5 grid (50 faces).
        let mut mesh = grid_square(5);
        let main_faces_before = mesh.faces.len();
        assert!(main_faces_before >= CLEANUP_MIN_COMPONENT_FACES);
        // Add a stray 1-face island far from the main grid (so
        // welding doesn't accidentally merge it in).
        let v0 = mesh.vertices.len() as u32;
        mesh.vertices.push(Point3::new(100.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(101.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(100.5, 1.0, 0.0));
        mesh.faces.push([v0, v0 + 1, v0 + 2]);

        let report = cleanup_cleaned_mesh_for_disk(&mut mesh);
        assert!(
            report.small_components >= 1,
            "small_components report: {report:?}",
        );
        // The 1-face island is gone; main component faces survive.
        assert_eq!(mesh.faces.len(), main_faces_before);
    }

    /// CSP.3b — `CleanupReport::total()` is zero for an
    /// already-clean input mesh. Pins the "no work needed → status
    /// message stays clean" path so future cleanup pipeline
    /// additions don't silently start surfacing spurious counts on
    /// pristine inputs.
    #[test]
    fn cleanup_report_total_zero_on_clean_input() {
        // 5×5 grid is already in shared-index form, no degenerates,
        // single component, no unreferenced verts.
        let mut mesh = grid_square(5);
        let report = cleanup_cleaned_mesh_for_disk(&mut mesh);
        assert_eq!(report.total(), 0, "clean mesh produced cleanup: {report:?}");
    }

    /// `resolve_output_dir` honors `--output-dir <path>` when supplied.
    #[test]
    fn resolve_output_dir_honors_explicit_flag() -> Result<(), clap::Error> {
        let cli = Cli::try_parse_from([
            "cf-scan-prep",
            "/scans/sock.stl",
            "--output-dir",
            "/out/foo",
        ])?;
        let dir = resolve_output_dir(&cli);
        assert_eq!(dir, PathBuf::from("/out/foo"));
        Ok(())
    }

    /// `resolve_output_dir` falls back to the input scan's parent
    /// directory when `--output-dir` is absent.
    #[test]
    fn resolve_output_dir_defaults_to_scan_parent() -> Result<(), clap::Error> {
        let cli = Cli::try_parse_from(["cf-scan-prep", "/scans/sock.stl"])?;
        let dir = resolve_output_dir(&cli);
        assert_eq!(dir, PathBuf::from("/scans"));
        Ok(())
    }

    /// Atomic-write produces both expected files when both writes
    /// succeed. Uses a tempfile-style temp dir constructed from
    /// `std::env::temp_dir()` + a unique stem so tests don't collide.
    #[test]
    fn atomic_write_save_lands_both_files() -> Result<()> {
        let tmp_root = std::env::temp_dir().join(format!(
            "cf-scan-prep-test-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos())
                .unwrap_or(0),
        ));
        std::fs::create_dir_all(&tmp_root)?;
        let stl_path = tmp_root.join("scan.cleaned.stl");
        let toml_path = tmp_root.join("scan.prep.toml");
        let mesh = one_triangle_at(0.001);
        atomic_write_save(&mesh, &stl_path, &toml_path, "key = \"value\"\n")?;
        assert!(stl_path.exists(), "cleaned STL should land at final path");
        assert!(toml_path.exists(), ".prep.toml should land at final path");
        // No `.tmp` files left over.
        assert!(!stl_path.with_extension("stl.tmp").exists());
        assert!(!toml_path.with_extension("toml.tmp").exists());
        // Cleanup.
        let _ = std::fs::remove_dir_all(&tmp_root);
        Ok(())
    }
}
