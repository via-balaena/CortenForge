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

use anyhow::Result;
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy::tasks::{AsyncComputeTaskPool, Task, futures_lite::future};
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use cf_bevy_common::prelude::*;
use cf_viewer::{
    RenderScale, compute_render_scale, scale_aabb, setup_camera_and_lighting, spawn_face_mesh,
};
use clap::{Parser, ValueEnum};
use mesh_io::load_stl;
use mesh_repair::{remove_unreferenced_vertices, weld_vertices};
use mesh_types::{Aabb, Bounded, IndexedMesh, Point3};
use nalgebra::{UnitQuaternion, Vector3};

use cf_scan_prep_core::*;

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

    /// **Debug-only.** Headless smoke test for the Apply-Simplify
    /// pipeline: loads the STL exactly like the GUI path
    /// (`try_load_scan` → auto-center + auto-PCA), runs
    /// [`simplify_mesh`] to the supplied target face count, prints
    /// per-pass diagnostics to stderr, and exits without opening a
    /// window. Used 2026-05-15 night to debug the
    /// `[Apply Simplify]` one-click bug surfaced post-PR #246
    /// (memo: `cf-scan-prep-simplify-one-click-bug`).
    ///
    /// Mutually exclusive with the normal GUI launch: when present,
    /// no Bevy window is opened.
    #[arg(long, value_name = "FACE_COUNT")]
    diagnose_simplify: Option<usize>,
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
    /// Has the user actively run `[Apply Simplify]` against the
    /// currently-loaded mesh? Drives the `.prep.toml`
    /// `[simplify].applied` provenance flag directly, instead of
    /// inferring from `achieved < original` face counts (which lies
    /// when the save-time cleanup pass drops faces but the user
    /// never clicked Apply). Set `true` in [`handle_simplify_actions`]
    /// on the `Apply` branch; reset to `false` on `Reset`.
    was_applied: bool,
}

impl Default for SimplifyState {
    fn default() -> Self {
        Self {
            target_face_count: SIMPLIFY_TARGET_DEFAULT,
            pending_action: None,
            was_applied: false,
        }
    }
}

/// In-flight simplify-task state. baby_shark decimation is on the
/// order of 10s of seconds on workshop-scale meshes (vs. meshopt's
/// ~3 s, the trade we made for manifold preservation in S1.1 2026-05-26);
/// running it on the Bevy main thread freezes the egui loop and
/// triggers the macOS spinning-rainbow "app is not responsive"
/// indicator. We instead spawn the decimation onto
/// `AsyncComputeTaskPool` and poll it non-blocking each frame, letting
/// the GUI continue to render an elapsed-time spinner.
///
/// `pending` is `None` when no task is in flight; `Some(...)` while
/// the worker is running. `started_at_secs` is set at spawn time so
/// the panel can render a live elapsed counter without re-reading
/// `Time` from inside the panel renderer.
#[derive(Resource, Default)]
struct SimplifyTask {
    pending: Option<Task<SimplifyResult>>,
    started_at_secs: f64,
}

/// Bundle of simplify-related system params, used by [`scan_prep_panel`]
/// to stay under Bevy's 16-system-param limit. `SystemParam` derive
/// lets the panel function declare one `SimplifyPanelParams` slot
/// instead of three (`SimplifyState` + `SimplifyTask` + `Time`).
#[derive(bevy::ecs::system::SystemParam)]
struct SimplifyPanelParams<'w> {
    state: ResMut<'w, SimplifyState>,
    task: Res<'w, SimplifyTask>,
    time: Res<'w, Time>,
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
    /// Weld coincident vertices ([`weld_vertices`] +
    /// [`remove_unreferenced_vertices`]) WITHOUT decimating. Collapses
    /// the raw STL's 3-verts-per-triangle soup into shared-index
    /// topology so the Cap → Scan boundary detector sees real edges
    /// (an unwelded mesh has every triangle as its own 3-vertex
    /// "loop"). Synchronous + fast; mutates [`ScanMesh`] in place.
    Weld,
}

/// Face-count threshold above which the load-time status bar surfaces
/// the auto-suggest banner. Per spec §Panel specifications §2.
const AUTO_SUGGEST_FACE_THRESHOLD: usize = 500_000;

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

// CSP.4c — `ClipState` removed. The Clip floor feature was a
// pre-CSP.4b world-Z axis-aligned cut; once centerline trim
// shipped (CSP.4b — perpendicular-to-spine cuts with auto-cap)
// the Clip floor's workshop use case was subsumed. User
// confirmed removal 2026-05-15. `clip_mesh_against_plane_eq`
// stays — it's the shared cut primitive that
// `clip_mesh_against_plane` (used by `trim_mesh_along_centerline`)
// builds on.

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

/// Centerline trim panel state (CSP.4b — apply-on-click model
/// since CSP.4b.6). The user dials slider values to position the
/// orange trim-plane circles live; clicking `[Apply trim]` commits
/// those values + re-derives the displayed mesh from the cuts.
///
/// **Slider vs applied**:
/// - `trim_tip_mm` / `trim_floor_mm` — slider state. Drives the
///   orange overlay circles in real time. Cheap to mutate (no mesh
///   recomputation per drag tick).
/// - `applied_tip_mm` / `applied_floor_mm` — committed state.
///   Drives the displayed mesh + the save-time cut. Mutated only
///   when the user clicks `[Apply trim]`.
///
/// CSP.4b.4 ran the trim live (every slider tick → re-derive
/// 200k-face mesh). User-reported 2026-05-15: "it feels very
/// resource intensive." Apply-on-click matches the existing Apply
/// Simplify pattern + amortizes the cost to discrete user actions.
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
    /// Slider value for the tip end, in mm. Drives the overlay
    /// circle; does NOT mutate the mesh until the user clicks
    /// `[Apply trim]`.
    trim_tip_mm: f64,
    /// Slider value for the floor end, in mm. Drives the overlay
    /// circle.
    trim_floor_mm: f64,
    /// Last value committed via `[Apply trim]` for the tip end,
    /// in mm. Drives the displayed mesh + the save-time cut.
    applied_tip_mm: f64,
    /// Last value committed via `[Apply trim]` for the floor end,
    /// in mm. Drives the displayed mesh + the save-time cut.
    applied_floor_mm: f64,
    /// Queued user action; consumed by
    /// [`update_displayed_mesh`] each Update tick. Mirrors the
    /// SimplifyState/CapPendingAction pattern.
    pending_action: Option<TrimAction>,

    // ----- CSP.4e Reconstruct state -----
    //
    // User-authorized directional workflow (2026-05-15): chop →
    // Apply trim → reconstruct UI appears → pick reference zone +
    // shape → Apply reconstruct. The reconstruct fields below are
    // only meaningful + only rendered when `applied_floor_mm > 0`
    // AND `trim_floor_mm == applied_floor_mm` (no drift since the
    // last Apply trim). Any drift, any Reset trim, any change to
    // the chop slider after Apply → the reconstruct fields RESET
    // and the panel un-spawns. This protects against the
    // state-machine trap of reconstruct staying applied to a
    // stale chop value.
    /// Reference-zone length in mm above the floor cut to sample
    /// the canonical cross-section from. Slider value (draft).
    /// 0.0 = no reconstruct preview yet.
    reconstruct_reference_mm: f64,
    /// Selected shape (Constant / Taper / Extrapolate). Draft
    /// value driven by the radio buttons.
    reconstruct_shape: ReconstructShape,
    /// `Some(...)` when the user has clicked `[Apply reconstruct]`;
    /// `None` while reconstruct is unapplied or un-spawned.
    /// Carries `(applied_reference_mm, applied_shape)` so the
    /// displayed-mesh snapshot can detect when reconstruct
    /// changes vs trim-only changes.
    applied_reconstruct: Option<AppliedReconstruct>,
    /// Queued user action from the Reconstruct buttons. Consumed
    /// next Update tick.
    reconstruct_pending: Option<ReconstructAction>,
}

impl Default for CenterlineTrimState {
    fn default() -> Self {
        Self {
            trim_tip_mm: 0.0,
            trim_floor_mm: 0.0,
            applied_tip_mm: 0.0,
            applied_floor_mm: 0.0,
            pending_action: None,
            reconstruct_reference_mm: 0.0,
            reconstruct_shape: ReconstructShape::Constant,
            applied_reconstruct: None,
            reconstruct_pending: None,
        }
    }
}

impl CenterlineTrimState {
    /// CSP.4e — reconstruct workflow is gated on a stable floor
    /// chop having been Applied. `available()` returns `true`
    /// when:
    /// - `applied_floor_mm > 0` (the user has committed a floor
    ///   chop)
    /// - `trim_floor_mm == applied_floor_mm` (no slider drift
    ///   since that commit; protects against reconstruct
    ///   applying to a stale chop)
    fn reconstruct_available(&self) -> bool {
        self.applied_floor_mm > 0.0
            && (self.trim_floor_mm - self.applied_floor_mm).abs() < f64::EPSILON
    }

    /// Wipe reconstruct fields back to default. Called when the
    /// "directional workflow" trip-wire fires: chop slider drift
    /// after Apply, full Reset trim, etc.
    fn reset_reconstruct(&mut self) {
        self.reconstruct_reference_mm = 0.0;
        self.reconstruct_shape = ReconstructShape::Constant;
        self.applied_reconstruct = None;
        self.reconstruct_pending = None;
    }
}

/// User action queued from the Centerline trim panel. Consumed by
/// [`update_displayed_mesh`] next Update tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TrimAction {
    /// Commit current slider values → applied values + re-derive
    /// the displayed mesh from the cuts.
    Apply,
    /// Zero both slider AND applied values + re-derive displayed
    /// mesh as the untrimmed base. Also resets the reconstruct
    /// state (the "hard reset to restart" the user spec'd).
    Reset,
}

/// User action queued from the Reconstruct sub-section
/// (CSP.4e). Consumed by [`update_displayed_mesh`] next Update
/// tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReconstructAction {
    /// Commit current reconstruct slider + shape selection →
    /// `applied_reconstruct = Some(...)` + re-derive displayed
    /// mesh. CSP.4e.1 — placeholder; the actual reconstruction
    /// geometry lands at CSP.4e.2+.
    Apply,
    /// Unapply reconstruct: `applied_reconstruct = None`. The
    /// chop stays in effect. (To wipe chop too, use Reset trim.)
    Reset,
}

/// User action queued by the Save panel's `[Save]` button. Consumed
/// next Update tick by `handle_save_action`, which builds the
/// cleaned mesh + the `.prep.toml` and writes both files atomically.
#[derive(Resource, Debug, Default)]
struct SavePendingAction {
    save_now: bool,
}

/// Save-time surface-smoothing state (2026-05-16, user-driven).
///
/// Controls the Taubin-smoothing pass applied in
/// [`cleanup_cleaned_mesh_for_disk`] before the cleaned STL is
/// written. The slider lives in the Save panel; iteration count
/// is per-scan because scanner-noise amplitude varies (raw
/// limb-tissue scans need 15-20 iters; high-quality reference
/// scans 3-5).
///
/// **Default** [`SMOOTHING_DEFAULT_ITERATIONS`] (8) — sweet
/// spot for typical body-part scans on the iter-1 fixture;
/// suppresses sub-mm scanner noise without erasing real
/// geometric detail. `0` disables smoothing entirely
/// (back-compat with pre-fix saves).
///
/// Surfaced in the saved `.prep.toml`'s `[smoothing]` block so
/// downstream consumers (cf-cast SDF sampling, cf-device-design
/// previews, future audit tooling) can see exactly how much
/// smoothing was applied to the cleaned mesh on disk.
#[derive(Resource, Debug, Clone, Copy)]
struct SmoothingState {
    iterations: usize,
}

impl Default for SmoothingState {
    fn default() -> Self {
        Self {
            iterations: SMOOTHING_DEFAULT_ITERATIONS,
        }
    }
}

/// Default iteration count for [`SmoothingState`] — 8 Taubin
/// passes. See [`SmoothingState`] docs for the per-scan-tuning
/// rationale + the suggested ranges for different scan
/// qualities.
const SMOOTHING_DEFAULT_ITERATIONS: usize = 8;

/// Upper slider bound for the Save panel's smoothing slider.
/// 30 iterations is enough to visibly over-smooth a noisy
/// limb scan; above that the user is degrading real geometric
/// detail (limb-end taper, anatomical features) without
/// returns. Display-only cap; the algorithm doesn't enforce it.
const SMOOTHING_MAX_ITERATIONS: usize = 30;

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

// CSP.4c — `compute_drop_percentage` + `update_clip_drop_pct`
// removed alongside `ClipState`.

/// Shell adapter — map the live Reorient / Recenter / Cap UI state to
/// the plain-data [`cf_scan_prep_core::build_cleaned_mesh`]. The Bevy
/// tool's job is to turn its egui state into core calls; the algorithm
/// itself lives in the headless core (shared with CortenForge Studio).
/// Shadows the glob-imported core fn of the same name.
fn build_cleaned_mesh(
    scan: &IndexedMesh,
    reorient: &ReorientState,
    recenter: &RecenterState,
    cap: &CapState,
) -> IndexedMesh {
    cf_scan_prep_core::build_cleaned_mesh(
        scan,
        reorient.quaternion_physics(),
        Vector3::new(
            recenter.tx_mm * 0.001,
            recenter.ty_mm * 0.001,
            recenter.tz_mm * 0.001,
        ),
        &cap.loops,
    )
}

/// Shell adapter — map the live UI state to the plain-data
/// [`cf_scan_prep_core::build_prep_toml_string`]. Shadows the
/// glob-imported core fn of the same name.
#[allow(clippy::too_many_arguments)]
fn build_prep_toml_string(
    source_stl: &Path,
    stl_units: StlUnits,
    auto_center_offset_m: Vector3<f64>,
    auto_pca_quat: Option<UnitQuaternion<f64>>,
    reorient: &ReorientState,
    recenter: &RecenterState,
    cap: &CapState,
    centerline_trim: &CenterlineTrimState,
    centerline_trim_capped: usize,
    cleaned_stl_name: &str,
    rotation_for_centerline: UnitQuaternion<f64>,
    translation_for_centerline_m: Vector3<f64>,
    pivot_centroid_m: Point3<f64>,
    simplify_target_face_count: usize,
    simplify_ran: bool,
    original_face_count: usize,
    achieved_face_count: usize,
    smoothing_iterations: usize,
    cleaned_aabb_m: &Aabb,
    reconstructed_floor: Option<ReconstructedFloorPlane>,
) -> Result<String> {
    cf_scan_prep_core::build_prep_toml_string(
        source_stl,
        stl_units.panel_label(),
        auto_center_offset_m,
        auto_pca_quat,
        reorient.quaternion_physics(),
        [reorient.roll_deg, reorient.pitch_deg, reorient.yaw_deg],
        Vector3::new(
            recenter.tx_mm * 0.001,
            recenter.ty_mm * 0.001,
            recenter.tz_mm * 0.001,
        ),
        &cap.centerline_polyline,
        &cap.loops,
        centerline_trim.applied_tip_mm,
        centerline_trim.applied_floor_mm,
        centerline_trim.applied_reconstruct,
        centerline_trim_capped,
        cleaned_stl_name,
        rotation_for_centerline,
        translation_for_centerline_m,
        pivot_centroid_m,
        simplify_target_face_count,
        simplify_ran,
        original_face_count,
        achieved_face_count,
        smoothing_iterations,
        cleaned_aabb_m,
        reconstructed_floor,
    )
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

fn main() -> Result<()> {
    let cli = Cli::parse();

    // Debug-only headless smoke test (`--diagnose-simplify N`).
    // Bypasses the Bevy window entirely; loads + simplifies + prints
    // + exits. Used to debug the one-click-vs-two-click bug.
    if let Some(target) = cli.diagnose_simplify {
        return run_diagnose_simplify(&cli, target);
    }

    match try_load_scan(&cli) {
        Ok((scan_mesh, auto_center_offset_m, auto_pca_quat)) => {
            // TODO (deferred from S1.1 d+c+e arc, 2026-05-26): surface
            // raw-scan quality (self-intersection + non-manifold counts)
            // here so the workshop user sees input quality at load
            // time. Bumped because BVH-backed self-intersection
            // detection on the 3.35M-face raw scan adds ~30 s to load;
            // doing it right needs background async detection, post-
            // Apply-Simplify deferred display, or a `--diagnose-scan`
            // opt-in flag. See:
            //   - `docs/CF_CAST_SCAN_MESH_DIRECT_RECON.md` §SMD-12
            //     (architectural arc framework)
            //   - memory `project-cf-scan-prep-raw-scan-validation-bookmark`
            //     (picker-upper writeup with the 3 perf approaches)
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

/// Headless `--diagnose-simplify` path: load + simplify + print + exit.
/// Mirrors the GUI's first-click behavior exactly (loads + auto-frame,
/// then calls [`simplify_mesh`]) so a reproducer can be observed without
/// any user interaction. Per-pass diagnostics come from
/// [`simplify_mesh`]'s own eprintlns (gated on
/// `CF_SCAN_PREP_SIMPLIFY_DIAG=1`); this function just prints the
/// before/after summary line.
fn run_diagnose_simplify(cli: &Cli, target: usize) -> Result<()> {
    let (mesh, _auto_center, _auto_pca) = try_load_scan(cli)?;
    eprintln!(
        "[diagnose] loaded: faces={} vertices={} target={}",
        mesh.faces.len(),
        mesh.vertices.len(),
        target,
    );
    let result = simplify_mesh(&mesh, target);
    eprintln!(
        "[diagnose] result: faces={} vertices={} elapsed={:.3}s",
        result.mesh.faces.len(),
        result.mesh.vertices.len(),
        result.elapsed_secs,
    );
    Ok(())
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
        .init_resource::<SimplifyTask>()
        .insert_resource(ReorientState::default())
        .insert_resource(RecenterState::default())
        .insert_resource(CapState::default())
        .insert_resource(CapPendingAction::default())
        .insert_resource(CenterlineTrimState::default())
        .insert_resource(SavePendingAction::default())
        .insert_resource(SmoothingState::default())
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
                // Poll the in-flight baby_shark task. Runs `.after`
                // `handle_simplify_actions` so the same-tick spawn
                // doesn't poll itself prematurely (rare race; explicit
                // ordering is cheap).
                poll_simplify_task.after(handle_simplify_actions),
                // CSP.4b.4 — live trim respawns the displayed
                // mesh entity. Runs after `handle_simplify_actions`
                // (sync Reset) AND after `poll_simplify_task` (async
                // Apply completion) so the simplify mutation lands
                // first; runs after `handle_cap_actions` so a fresh
                // Cap → Scan's centerline is visible to the trim on
                // the same tick. All `.after`s are explicit since
                // Bevy doesn't guarantee tuple-position ordering on
                // its own.
                update_displayed_mesh
                    .after(handle_simplify_actions)
                    .after(poll_simplify_task)
                    .after(handle_cap_actions),
                apply_world_transform_to_scan_entity,
                handle_cap_actions,
                // Auto-level the floor after a scan / applied trim. Runs
                // after handle_cap_actions so a fresh cap is visible the
                // same tick.
                auto_level_to_floor.after(handle_cap_actions),
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
) {
    // CSP.4b.4 — camera + lighting only. The mesh entity gets
    // spawned on the first Update tick by `update_displayed_mesh`,
    // which is the single owner of the rendered entity (so trim
    // changes + Apply Simplify + Reset to original all flow
    // through one place + the displayed mesh is always the
    // post-trim view).
    let raw_aabb = scan.0.aabb();
    let scaled_aabb = scale_aabb(&raw_aabb, render_scale.0);
    setup_camera_and_lighting(&mut commands, &scaled_aabb, *up);
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
/// On Reset: clones `OriginalScanMesh`, updates `ScanMesh`, sets a
/// confirmation status message.
///
/// CSP.4b.4 — this system NO LONGER respawns the rendered entity
/// or updates `ScanInfo`. Both now live in
/// [`update_displayed_mesh`] so the displayed mesh always reflects
/// the post-trim view (live trim preview). `ScanMesh` is the
/// "base, pre-trim" mesh that the trim system reads.
// Bevy systems take resources by value.
#[allow(clippy::needless_pass_by_value)]
fn handle_simplify_actions(
    mut simplify_state: ResMut<SimplifyState>,
    mut simplify_task: ResMut<SimplifyTask>,
    mut scan: ResMut<ScanMesh>,
    original: Res<OriginalScanMesh>,
    mut status: ResMut<StatusBar>,
    time: Res<Time>,
) {
    let Some(action) = simplify_state.pending_action.take() else {
        return;
    };

    // Refuse to start a new Apply while a previous one is still
    // running. Reset still works (it's synchronous + fast); but the
    // panel disables the Reset button too as a UX cue while a task is
    // in flight, so this branch should never fire in practice.
    if simplify_task.pending.is_some() {
        return;
    }

    // `current` (not "original"): the face count of `scan.0` AT THE
    // START OF THIS HANDLER. After a prior Apply, scan.0 holds the
    // simplified mesh, so `current` is the post-prior-simplify count
    // (not the unsimplified loaded mesh — that lives in
    // `original: Res<OriginalScanMesh>`).
    let current_face_count = scan.0.faces.len();
    let target = simplify_state.target_face_count;

    // Pre-check the Apply no-op case. baby_shark's `EdgeDecimator`
    // would early-exit when `min_faces_count >= current.faces.len()`
    // (it's a stop condition, not a SIGABRT trap — but the user-
    // facing "Reduced 200k -> 200k in 0.5s" still reads as a broken
    // button), so we still short-circuit here. The user-friendly
    // workflow when they want *more* faces back is `[Reset to
    // original]` first.
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

    match action {
        SimplifyAction::Apply => {
            // Spawn the decimation on the AsyncComputeTaskPool so the
            // GUI keeps rendering (60 FPS egui) instead of macOS
            // showing the spinning-rainbow unresponsive indicator
            // during baby_shark's ~10-40 s decimation. `poll_simplify_task`
            // picks up the result the frame after it completes.
            let scan_clone = scan.0.clone();
            let pool = AsyncComputeTaskPool::get();
            let task = pool.spawn(async move { simplify_mesh(&scan_clone, target) });
            simplify_task.pending = Some(task);
            simplify_task.started_at_secs = time.elapsed_secs_f64();
            status.text = format!(
                "Simplifying {} -> {} faces (baby_shark)…",
                human_count(current_face_count),
                human_count(target),
            );
            status.kind = StatusKind::Normal;
            // No auto-clear while task is in flight — the panel +
            // poll_simplify_task own the lifecycle.
            status.auto_clear_at_secs = None;
        }
        SimplifyAction::Reset => {
            // Reset is fast (single mesh clone); run it synchronously.
            let cloned = original.0.clone();
            let text = format!(
                "Restored original mesh ({} faces)",
                human_count(cloned.faces.len()),
            );
            simplify_state.was_applied = false;
            scan.0 = cloned;
            status.text = text;
            status.kind = StatusKind::Normal;
            status.auto_clear_at_secs = Some(time.elapsed_secs_f64() + STATUS_NORMAL_TTL_SECS);
        }
        SimplifyAction::Weld => {
            // Weld is fast (spatial hash, no decimation); run it
            // synchronously. Collapses the raw STL soup into shared
            // topology so Cap → Scan finds real boundary loops.
            let before_verts = scan.0.vertices.len();
            weld_vertices(&mut scan.0, SIMPLIFY_WELD_EPSILON_M);
            remove_unreferenced_vertices(&mut scan.0);
            let after_verts = scan.0.vertices.len();
            status.text = format!(
                "Welded {} -> {} vertices ({} faces) — Cap → Scan will now find real boundary loops",
                human_count(before_verts),
                human_count(after_verts),
                human_count(scan.0.faces.len()),
            );
            status.kind = StatusKind::Normal;
            status.auto_clear_at_secs = Some(time.elapsed_secs_f64() + STATUS_NORMAL_TTL_SECS);
        }
    }
}

/// Per-frame non-blocking poll of the in-flight simplify task. When
/// the worker finishes, swap the result into `ScanMesh`, flip
/// `was_applied`, and surface a completion status message.
#[allow(clippy::needless_pass_by_value)]
fn poll_simplify_task(
    mut simplify_task: ResMut<SimplifyTask>,
    mut simplify_state: ResMut<SimplifyState>,
    mut scan: ResMut<ScanMesh>,
    mut status: ResMut<StatusBar>,
    time: Res<Time>,
) {
    let Some(mut task) = simplify_task.pending.take() else {
        return;
    };
    let Some(result) = future::block_on(future::poll_once(&mut task)) else {
        // Still running — put the handle back.
        simplify_task.pending = Some(task);
        return;
    };

    let before = scan.0.faces.len();
    let achieved = result.mesh.faces.len();
    scan.0 = result.mesh;
    simplify_state.was_applied = true;
    status.text = format!(
        "Reduced {} -> {} faces in {:.1}s",
        human_count(before),
        format_count_with_separators(achieved),
        result.elapsed_secs,
    );
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

/// Snapshot key the live-trim system uses to detect when the
/// displayed mesh needs re-derivation (CSP.4b.4 + CSP.4b.6).
/// Any change in the snapshot triggers a respawn from a freshly
/// trimmed mesh.
///
/// Field shape:
/// - `trim_tip_mm` + `trim_floor_mm`: bit-pattern compared. **These
///   hold APPLIED values, not slider values** (CSP.4b.6 apply-on-
///   click model). Dragging the slider without clicking Apply does
///   NOT change the snapshot, so the mesh stays put — only the
///   orange overlay moves.
/// - `scan_face_count` + `scan_vertex_count`: proxy for "ScanMesh
///   identity changed" — Apply Simplify / Reset to original both
///   change the face count. Avoids the `Res::<>::is_changed()`
///   footgun (`project_bevy_is_changed_footgun` per the auto-
///   memory ledger) where any `&mut *res` deref flips the
///   change-token.
/// - `centerline_len`: proxy for "Cap → Scan ran." When the
///   centerline appears (or disappears via re-Scan on a closed
///   mesh) the trim semantics flip; force a respawn.
/// - `reconstruct`: CSP.4e — Apply/Reset of the reconstruct
///   sub-section drives mesh re-derivation. `None` at all times
///   in CSP.4e.1 because the algorithm is stubbed; `Some(...)`
///   from CSP.4e.2 onward triggers extruded-cross-section
///   geometry.
#[derive(Debug, Clone, Copy, PartialEq)]
struct DisplayedMeshSnapshot {
    trim_tip_mm: f64,
    trim_floor_mm: f64,
    scan_face_count: usize,
    scan_vertex_count: usize,
    centerline_len: usize,
    reconstruct: Option<AppliedReconstruct>,
}

/// Update system: maintain the rendered `ScanMeshEntity` as the
/// **live trimmed view** of `ScanMesh` (CSP.4b.4). Respawns the
/// entity whenever a relevant input changes:
///
/// - Trim sliders moved (`CenterlineTrimState` values differ).
/// - `ScanMesh` was mutated (Apply Simplify / Reset to original
///   changed face/vertex counts).
/// - Cap detection produced or lost a centerline (`centerline_len`
///   differs).
///
/// CSP.4b's initial cut applied the trim only at SAVE time, with
/// a live overlay (orange circles) hinting at the planned cuts.
/// User-reported 2026-05-15: "the cut should happen inside of the
/// program not at save time" — the mesh should visibly shrink as
/// the user drags the trim sliders so they can see-it-as-they-go.
/// This system performs that live cut.
///
/// **Cost**: trim + auto-cap on a 200k-face scan runs in ~40-50 ms
/// (one trim plane = O(F) walking faces; auto-cap = O(F) building
/// adjacency + O(loop_count × loop_vertices²) ear-clip). At 60 Hz
/// drag this is borderline-fluid → workable but noticeable. The
/// snapshot guard short-circuits when nothing relevant changed, so
/// steady-state cost is zero.
///
/// **Skip path**: when `CapState::stale` is set (transforms
/// changed since last Cap → Scan), the trim cuts would reference
/// an off-mesh centerline. Skip the trim — display the untrimmed
/// `ScanMesh` instead. The user re-clicks Cap → Scan to refresh.
#[allow(clippy::needless_pass_by_value)]
#[allow(clippy::too_many_arguments)]
fn update_displayed_mesh(
    scan: Res<ScanMesh>,
    cap: Res<CapState>,
    mut trim: ResMut<CenterlineTrimState>,
    mut info: ResMut<ScanInfo>,
    existing: Query<Entity, With<ScanMeshEntity>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut status: ResMut<StatusBar>,
    time: Res<Time>,
    mut last_snapshot: Local<Option<DisplayedMeshSnapshot>>,
) {
    // CSP.4b.6 — apply-on-click. Consume the queued
    // `TrimAction` (if any) BEFORE building the snapshot, so the
    // applied values match what the user just clicked.
    if let Some(action) = trim.pending_action.take() {
        match action {
            TrimAction::Apply => {
                trim.applied_tip_mm = trim.trim_tip_mm;
                trim.applied_floor_mm = trim.trim_floor_mm;
                // CSP.4e — a fresh chop commit invalidates any
                // existing reconstruct (it was applied to a
                // potentially-different floor cut). Wipe to keep
                // the directional workflow safe.
                trim.reset_reconstruct();
            }
            TrimAction::Reset => {
                trim.trim_tip_mm = 0.0;
                trim.trim_floor_mm = 0.0;
                trim.applied_tip_mm = 0.0;
                trim.applied_floor_mm = 0.0;
                // CSP.4e — Reset trim is the "hard reset" the
                // user spec'd; wipe reconstruct too.
                trim.reset_reconstruct();
            }
        }
    }

    // CSP.4e — directional-workflow trip-wire. If the user
    // dragged the chop slider after Apply trim (drift),
    // applied_floor_mm > 0 still holds but the floor cut is now
    // stale relative to what the user is previewing → un-spawn
    // reconstruct UI by wiping its state. `reconstruct_available()`
    // captures both clauses; mismatch implies drift OR no chop
    // committed, both of which require reconstruct to retreat.
    if !trim.reconstruct_available() && trim.applied_reconstruct.is_some() {
        trim.reset_reconstruct();
    }

    // CSP.4e — Reconstruct action handling. Same apply-on-click
    // model as TrimAction. CSP.4e.2 fix-forward (2026-05-15
    // user-reported "doesn't appear to generate the mesh back
    // down"): guard against reference_mm == 0 + emit status
    // messages so silent no-ops surface to the user.
    if let Some(action) = trim.reconstruct_pending.take() {
        match action {
            ReconstructAction::Apply => {
                if trim.reconstruct_reference_mm <= 0.0 {
                    status.text =
                        "Reconstruct skipped: reference zone is 0 mm — drag the slider above 0 to sample cross-sections."
                            .into();
                    status.kind = StatusKind::Warning;
                    status.auto_clear_at_secs = Some(time.elapsed_secs_f64() + 8.0);
                } else {
                    trim.applied_reconstruct = Some(AppliedReconstruct {
                        reference_mm: trim.reconstruct_reference_mm,
                        shape: trim.reconstruct_shape,
                    });
                    let shape_label = match trim.reconstruct_shape {
                        ReconstructShape::Constant => "Constant (average radius)",
                        ReconstructShape::Taper => "Taper toward floor",
                        ReconstructShape::Extrapolate => "Extrapolate reference trend",
                    };
                    status.text = format!(
                        "Applied reconstruct: reference {:.1} mm, shape = {}",
                        trim.reconstruct_reference_mm, shape_label,
                    );
                    status.kind = StatusKind::Normal;
                    status.auto_clear_at_secs = Some(time.elapsed_secs_f64() + 8.0);
                }
            }
            ReconstructAction::Reset => {
                trim.applied_reconstruct = None;
                status.text = "Reconstruct unapplied — mesh reverted to chopped-only state.".into();
                status.kind = StatusKind::Normal;
                status.auto_clear_at_secs = Some(time.elapsed_secs_f64() + 6.0);
            }
        }
    }

    // Snapshot drives change detection: re-respawn the entity when
    // any input that affects the displayed mesh has actually
    // changed. CSP.4b.6 — uses APPLIED values, not slider values,
    // so dragging the slider (without Apply) does NOT trigger a
    // respawn. Only the orange overlay moves with the slider.
    // CSP.4e — applied_reconstruct adds to the snapshot so the
    // displayed mesh re-derives when reconstruct is applied or
    // unapplied. At CSP.4e.1 this is a no-op visual change (the
    // algorithm is stubbed); at CSP.4e.2 the respawn will swap
    // in the reconstructed geometry.
    let snapshot = DisplayedMeshSnapshot {
        trim_tip_mm: trim.applied_tip_mm,
        trim_floor_mm: trim.applied_floor_mm,
        scan_face_count: scan.0.faces.len(),
        scan_vertex_count: scan.0.vertices.len(),
        centerline_len: cap.centerline_polyline.len(),
        reconstruct: trim.applied_reconstruct,
    };
    if last_snapshot.as_ref() == Some(&snapshot) {
        return;
    }

    // Derive displayed mesh from APPLIED trim values + the base
    // ScanMesh. Skip the trim if no applied trim or centerline
    // unavailable.
    //
    // CSP.4e.2 — if reconstruct is applied AND the floor was
    // chopped, replace the auto-cap call for the floor end with
    // the centerline-driven reconstruction (extruded average
    // cross-section + bottom flat cap). Tip-end (if also trimmed)
    // still gets the original flat auto-cap inside
    // `apply_reconstruction` via its tail call to
    // `auto_cap_open_boundaries` (PR #246 cold-read fix-forward).
    let displayed = if cap.centerline_polyline.len() >= 2
        && (trim.applied_tip_mm > 0.0 || trim.applied_floor_mm > 0.0)
    {
        let mut t = trim_mesh_along_centerline(
            &scan.0,
            &cap.centerline_polyline,
            trim.applied_tip_mm,
            trim.applied_floor_mm,
        );
        // CSP.4e.2.4 (2026-05-15) — weld the cut boundary's
        // duplicate intersection vertices so `detect_holes`
        // forms ONE closed loop at the cut, not hundreds of
        // 2-edge fragments. `clip_mesh_against_plane_eq` appends
        // a separate intersection vertex per crossing triangle —
        // adjacent triangles sharing an edge get TWO duplicates
        // at the same world position. Without welding,
        // `find_floor_loop_index` ignored the fragmented cut
        // boundary entirely + picked a scan-artifact loop
        // (24-vertex tiny shape, user-reported "white lines
        // forming a right angle" 2026-05-15).
        weld_vertices(&mut t, SIMPLIFY_WELD_EPSILON_M);
        match trim.applied_reconstruct {
            Some(ar) if trim.applied_floor_mm > 0.0 => {
                // CSP.4e.2 + 4e.3 — all three shapes (Constant,
                // Taper, Extrapolate) ship the extruded-sidewall
                // reconstruction. Per-shape per-ring radius logic
                // is inside `apply_reconstruction` itself.
                let trimmed_centerline = trim_centerline_polyline(
                    &cap.centerline_polyline,
                    trim.applied_tip_mm,
                    trim.applied_floor_mm,
                );
                t = apply_reconstruction(
                    t,
                    &trimmed_centerline,
                    trim.applied_floor_mm,
                    ar.reference_mm,
                    ar.shape,
                );
            }
            _ => {
                // No reconstruct applied — auto-cap all boundaries.
                let _capped = auto_cap_open_boundaries(&mut t);
            }
        }
        t
    } else {
        scan.0.clone()
    };

    info.vertex_count = displayed.vertices.len();
    info.face_count = displayed.faces.len();

    respawn_scan_entity(
        &mut commands,
        &existing,
        &displayed,
        meshes.as_mut(),
        materials.as_mut(),
        *up,
        render_scale.0,
    );

    *last_snapshot = Some(snapshot);
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

    // CSP.4c — Clip floor disc visualization removed alongside
    // `ClipState`. The orange trim-plane circles
    // (`draw_trim_plane_overlays`) cover the visual-feedback use
    // case the disc used to.
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
    //
    // CSP.4e.2.3 (2026-05-15) — apply 3-iteration moving-average
    // smoothing to the raw cross-section centroids. The raw
    // centroids wobble several mm per slab on a noisy surface
    // scan, which (a) tilts trim cut planes off the body's true
    // axis and (b) gives the reconstruction algorithm a wobbly
    // sampling frame. Smoothing once at storage time means every
    // downstream consumer (trim algo, trim overlay, reconstruct,
    // save TOML) sees the cleaned polyline.
    cap.centerline_polyline = if let Some(first_loop) = cap.loops.first() {
        let raw = compute_centerline_polyline(&scan.0, first_loop.plane_normal, 30);
        smooth_polyline(&raw, 3)
    } else {
        Vec::new()
    };

    cap.stale = false;

    let n_loops = cap.loops.len();

    // Unwelded-soup guard: a raw STL has no shared edges, so every
    // triangle is its own 3-vertex boundary loop. Detect that case and
    // point at the Weld button instead of listing thousands of loops.
    if mesh_looks_unwelded(scan.0.vertices.len(), scan.0.faces.len()) && n_loops > 100 {
        status.text = format!(
            "Detected {} boundary loops — the mesh is unwelded (raw STL soup). \
             Click [Weld vertices] in the Simplify panel, then re-Scan.",
            human_count(n_loops),
        );
        status.kind = StatusKind::Warning;
        status.auto_clear_at_secs = Some(time.elapsed_secs_f64() + STATUS_NORMAL_TTL_SECS);
        return;
    }

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

/// AUTOMATICALLY rotate the scan so the floor the device seats on is
/// horizontal — re-run whenever the floor reference changes (a new Cap →
/// Scan, or a committed [Apply trim]/[Apply reconstruct]). No button:
/// the leveling is part of the cap/trim pipeline so the floor is always
/// level to the *current* floor.
///
/// **Levels to the FLOOR the device actually seats on, in priority:**
/// (1) the reconstructed/chopped floor plane via
/// [`compute_reconstructed_floor_plane_physics`] once trim is applied —
/// the exact plane `apply_reconstruction` builds + the .prep.toml records
/// (stable look-back tangent); else (2) the predicted draft floor cut;
/// else (3) the raw detected cap loop (fresh scan, no trim). Because the
/// centerline curves, the reconstructed floor sits at a *different* plane
/// than the original cut-base cap — leveling the reconstructed plane is
/// what makes the final floor horizontal.
///
/// The reference is in the auto-PCA-baked `ScanMesh` frame — the same
/// frame `ReorientState` maps FROM — so the leveling rotation is exactly
/// `rotation_between(floor_normal, nearest ±Z)`, written into the Reorient
/// Euler angles (REPLACE). A `Local` key (loop size + applied trim) gates
/// re-running so slider drags don't thrash the orientation. Setting
/// Reorient does trip the "Transform changed" cap-stale notice — that's
/// expected/benign here (the level is idempotent + correct; Save bakes it
/// regardless of the overlay being stale).
#[allow(clippy::needless_pass_by_value)]
fn auto_level_to_floor(
    cap: Res<CapState>,
    trim: Res<CenterlineTrimState>,
    mut reorient: ResMut<ReorientState>,
    mut status: ResMut<StatusBar>,
    time: Res<Time>,
    // Tracks the floor reference that was last leveled to, so we re-level
    // only when it actually changes — NOT every frame, and NOT on a
    // mere slider drag (which doesn't touch `applied_*`).
    mut last_key: Local<Option<(usize, u64, u64)>>,
) {
    // Re-level on: a new Cap → Scan (loop vertex count changes), or a
    // committed trim/reconstruct (`applied_floor`/`applied_tip` change —
    // the floor the device seats on moved). `applied_*` only change on
    // [Apply trim]/[Apply reconstruct], so dragging sliders is inert.
    if cap.centerline_polyline.is_empty() {
        *last_key = None;
        return;
    }
    let loop_sig = cap.loops.first().map_or(0, |l| l.vertex_indices.len());
    let key = (
        loop_sig,
        trim.applied_floor_mm.to_bits(),
        trim.applied_tip_mm.to_bits(),
    );
    if *last_key == Some(key) {
        return;
    }
    *last_key = Some(key);

    let ttl = Some(time.elapsed_secs_f64() + STATUS_NORMAL_TTL_SECS);

    // Pick the plane the device actually seats on, in priority order:
    //   1. The ACTUAL reconstructed/chopped floor (applied trim) — same
    //      plane `apply_reconstruction` builds + the .prep.toml records,
    //      via the stable look-back tangent. This is the post-reconstruct
    //      ground truth the workshop wants.
    //   2. The PREDICTED floor cut from the draft `trim_floor` slider
    //      (before Apply trim) — the centerline tangent at the cut.
    //   3. The raw detected cap-loop normal — no trim at all.
    let (floor_normal, label) = if let Some(plane) = compute_reconstructed_floor_plane_physics(
        &cap.centerline_polyline,
        trim.applied_tip_mm,
        trim.applied_floor_mm,
    ) {
        (
            plane.normal,
            format!("reconstructed floor @ {:.0} mm", trim.applied_floor_mm),
        )
    } else if cap.centerline_polyline.len() >= 2 && trim.trim_floor_mm > 0.0 {
        let total_m = polyline_arc_length_m(&cap.centerline_polyline);
        let floor_cut_m = (total_m - trim.trim_floor_mm / 1000.0).max(0.0);
        match point_along_polyline_at_arc_distance(&cap.centerline_polyline, floor_cut_m) {
            Some((_, tangent)) => (
                tangent,
                format!("predicted floor cut @ {:.0} mm", trim.trim_floor_mm),
            ),
            None => (
                cap.loops.first().map_or(Vector3::z(), |l| l.plane_normal),
                "cap".to_string(),
            ),
        }
    } else if let Some(first) = cap.loops.first() {
        (first.plane_normal.normalize(), "cap loop".to_string())
    } else {
        status.text = "Level to cap: nothing to level to — Scan (and/or trim) first".to_string();
        status.kind = StatusKind::Warning;
        status.auto_clear_at_secs = ttl;
        return;
    };

    let tilt_deg = floor_normal
        .normalize()
        .z
        .abs()
        .clamp(0.0, 1.0)
        .acos()
        .to_degrees();
    let (roll_deg, pitch_deg, yaw_deg) = leveling_euler_deg(floor_normal);
    *reorient = ReorientState {
        roll_deg,
        pitch_deg,
        yaw_deg,
    };
    status.text = format!("Auto-leveled to {label} (corrected {tilt_deg:.1}° tilt)");
    status.kind = StatusKind::Normal;
    status.auto_clear_at_secs = ttl;
}

/// Reorient Euler angles (degrees, intrinsic XYZ) that rotate
/// `cap_normal` onto the NEAREST vertical axis (±Z) — the floor-leveling
/// rotation. Nearest-axis targeting avoids flipping a floor-down part
/// 180°. Returns `(roll, pitch, yaw)`; `(0, 0, 0)` if the normal is
/// already vertical (degenerate `rotation_between`).
fn leveling_euler_deg(cap_normal: Vector3<f64>) -> (f64, f64, f64) {
    let n = cap_normal.normalize();
    let target = if n.z >= 0.0 {
        Vector3::new(0.0, 0.0, 1.0)
    } else {
        Vector3::new(0.0, 0.0, -1.0)
    };
    UnitQuaternion::rotation_between(&n, &target).map_or((0.0, 0.0, 0.0), |q| {
        let (r, p, y) = q.euler_angles();
        (r.to_degrees(), p.to_degrees(), y.to_degrees())
    })
}

/// Update system: set `CapState::stale = true` when Reorient,
/// Recenter, or `ScanMesh` changes after a scan has populated
/// `CapState::loops`. Spec §Panel specifications §6: the loop
/// indices and plane fits are tied to the mesh as-it-was at scan
/// time; subsequent transforms / simplify operations invalidate
/// them. (CSP.2 originally watched a `[clip]` slider too; CSP.4d
/// removed clip from the tool entirely.)
///
/// **Cheap check**: only reads change flags and writes a single bool
/// via `bypass_change_detection` so we don't re-trigger ourselves.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn mark_cap_stale_on_transform_change(
    mut cap: ResMut<CapState>,
    scan: Res<ScanMesh>,
    reorient: Res<ReorientState>,
    recenter: Res<RecenterState>,
) {
    // Nothing to invalidate if no scan has happened yet.
    if cap.loops.is_empty() {
        return;
    }
    if cap.stale {
        return;
    }
    if scan.is_changed() || reorient.is_changed() || recenter.is_changed() {
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
    mut simplify: SimplifyPanelParams,
    mut reorient_state: ResMut<ReorientState>,
    mut recenter_state: ResMut<RecenterState>,
    mut cap_state: ResMut<CapState>,
    mut cap_pending: ResMut<CapPendingAction>,
    mut centerline_trim_state: ResMut<CenterlineTrimState>,
    mut save_pending: ResMut<SavePendingAction>,
    mut smoothing_state: ResMut<SmoothingState>,
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
            // total content exceeds typical viewport height on
            // laptop screens. Without scrolling, sections below
            // the fold are unreachable. `auto_shrink([false,
            // true])` claims full available width but only as
            // much height as content needs (with scrollbar on
            // overflow).
            //
            // CSP.4c — panel order = "centerline-driven flow":
            //   1. Scan Info (read-only info at load)
            //   2. Simplify (Apply simplify → 200k face workshop budget)
            //   3. Cap open boundaries (Scan → detect centerline)
            //   4. Centerline trim (drag sliders + Apply trim)
            //   5. Reorient / Recenter / Clip floor (collapsed —
            //      manual overrides for cases auto-PCA +
            //      auto-center + trim don't cover)
            //   6. Save (final action)
            //
            // Reorient + Recenter + Clip floor's panel functions
            // open with `default_open(false)` so they appear as
            // discoverable expanders without cluttering the
            // primary flow. Auto-PCA at load + Auto-center at
            // load + centerline trim are the load-bearing path;
            // the override panels are escape hatches.
            egui::ScrollArea::vertical()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    render_scan_info_section(ui, &info);
                    render_simplify_section(
                        ui,
                        &info,
                        &mut simplify.state,
                        &simplify.task,
                        simplify.time.elapsed_secs_f64(),
                    );
                    render_cap_section(ui, &mut cap_state, &mut cap_pending);
                    render_centerline_trim_section(ui, &mut centerline_trim_state, &cap_state);
                    // ----- Manual overrides (collapsed) -----
                    render_reorient_section(ui, &mut reorient_state, &scan.0);
                    render_recenter_section(ui, &mut recenter_state, &reorient_state, &overlays);
                    // ----- Save (last) -----
                    render_save_section(
                        ui,
                        &source.0,
                        &output_dir.0,
                        &cap_state,
                        &mut save_pending,
                        &mut smoothing_state,
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
fn render_simplify_section(
    ui: &mut egui::Ui,
    info: &ScanInfo,
    state: &mut SimplifyState,
    task: &SimplifyTask,
    now_secs: f64,
) {
    egui::CollapsingHeader::new("Simplify (decimate)")
        .default_open(true)
        .show(ui, |ui| {
            ui.label(format!("Current: {} faces", human_count(info.face_count)));

            let task_running = task.pending.is_some();

            ui.add_space(4.0);
            ui.label("Target face count:");
            ui.add_enabled(
                !task_running,
                egui::Slider::new(
                    &mut state.target_face_count,
                    SIMPLIFY_TARGET_MIN..=SIMPLIFY_TARGET_MAX,
                )
                .logarithmic(true)
                .text("faces"),
            );

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(!task_running, egui::Button::new("Apply simplify"))
                    .clicked()
                {
                    state.pending_action = Some(SimplifyAction::Apply);
                }
                if ui
                    .add_enabled(!task_running, egui::Button::new("Reset to original"))
                    .clicked()
                {
                    state.pending_action = Some(SimplifyAction::Reset);
                }
                if ui
                    .add_enabled(!task_running, egui::Button::new("Weld vertices"))
                    .on_hover_text(
                        "Merge coincident vertices into shared topology without decimating. \
                         Required before Cap → Scan if the mesh is still raw STL soup.",
                    )
                    .clicked()
                {
                    state.pending_action = Some(SimplifyAction::Weld);
                }
            });

            // Unwelded-soup warning. A freshly-loaded STL has 3 verts
            // per triangle (no shared edges), so Cap → Scan would report
            // one 3-vertex "loop" per triangle. Simplify welds as a side
            // effect, but a scan that arrives already at/under the target
            // never gets simplified — so flag it explicitly and point at
            // the Weld button.
            if mesh_looks_unwelded(info.vertex_count, info.face_count) {
                ui.add_space(4.0);
                ui.colored_label(
                    egui::Color32::from_rgb(240, 200, 80),
                    format!(
                        "⚠ Mesh looks unwelded ({} verts for {} faces). Click [Weld vertices] \
                         before Cap → Scan, or it will report one loop per triangle.",
                        human_count(info.vertex_count),
                        human_count(info.face_count),
                    ),
                );
            }

            if task_running {
                ui.add_space(6.0);
                let elapsed_secs = (now_secs - task.started_at_secs).max(0.0);
                ui.horizontal(|ui| {
                    ui.add(egui::Spinner::new());
                    ui.label(format!(
                        "Simplifying… {elapsed_secs:.1}s elapsed (baby_shark, ~10-40 s typical)"
                    ));
                });
            }

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
    // CSP.4c — collapsed by default. The auto-PCA bake at load
    // (CSP.4a) handles the common-case orientation; the sliders +
    // snap buttons here are manual override for cases where PCA
    // picked wrong.
    egui::CollapsingHeader::new("Reorient (manual override)")
        .default_open(false)
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
    // CSP.4c — collapsed by default. Auto-center at load (CSP.3.5)
    // puts the AABB centroid at origin; these sliders + buttons
    // are for users who want a different recentering target.
    egui::CollapsingHeader::new("Recenter (manual override)")
        .default_open(false)
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

// CSP.4c — `render_clip_section` removed alongside `ClipState`.

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
fn render_cap_section(ui: &mut egui::Ui, state: &mut CapState, pending: &mut CapPendingAction) {
    egui::CollapsingHeader::new("Cap open boundaries")
        .default_open(true)
        .show(ui, |ui| {
            if ui.button("Scan").clicked() {
                pending.rescan = true;
            }

            // Auto-leveling note: leveling is no longer a button. The
            // scan auto-levels the floor to the build axis, and an
            // applied trim/reconstruct re-levels to the new floor —
            // handled by `auto_level_to_floor`.
            ui.small("(floor auto-levels on Scan + after Apply trim/reconstruct)");

            if state.loops.is_empty() {
                ui.label(
                    egui::RichText::new("(no scan yet — click Scan to detect boundary loops)")
                        .italics(),
                );
                return;
            }

            // CSP.4c — `[Snap boundary -> floor]` button removed.
            // It rotated/translated based on the FIRST detected cap
            // loop's plane fit, which was the ORIGINAL rim (in
            // pre-trim coordinates). After Centerline-trim Apply,
            // the "floor" of the displayed mesh is the trim cut
            // plane, NOT that original rim. Clicking snap
            // post-trim would have aligned the no-longer-present
            // rim. Auto-PCA + Auto-center at load handle the
            // common alignment case; the Reorient + Recenter
            // panels (collapsed by default) cover manual override.

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

            // CSP.4b.6 — show the user the gap between slider
            // (next-cut preview) and applied (current mesh cut)
            // when they differ, so it's obvious when an Apply is
            // pending.
            let drifted = (state.trim_tip_mm - state.applied_tip_mm).abs() > 1e-6
                || (state.trim_floor_mm - state.applied_floor_mm).abs() > 1e-6;
            if drifted {
                ui.colored_label(
                    egui::Color32::from_rgb(240, 200, 80),
                    format!(
                        "⚠ Pending: applied tip {:.1} / floor {:.1} mm — click [Apply trim] to cut",
                        state.applied_tip_mm, state.applied_floor_mm,
                    ),
                );
            }

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                if ui.button("Apply trim").clicked() {
                    state.pending_action = Some(TrimAction::Apply);
                }
                if ui.button("Reset trim").clicked() {
                    state.pending_action = Some(TrimAction::Reset);
                }
            });

            ui.add_space(4.0);
            ui.small(
                "Drag the sliders to position the orange cut circles. \
                 Click [Apply trim] to perform the cuts on the displayed mesh. \
                 Save writes the displayed (already-applied) trim to disk.",
            );

            // ----- CSP.4e Reconstruct sub-section -----
            //
            // Only renders when a stable floor chop has been
            // applied (`reconstruct_available()` — applied_floor_mm
            // > 0 AND no slider drift since the Apply). The
            // directional-workflow trip-wire in
            // `update_displayed_mesh` un-spawns this state when
            // drift / Reset fires, so the panel re-disappears on
            // its own.
            if state.reconstruct_available() {
                ui.add_space(8.0);
                ui.separator();
                ui.add_space(4.0);
                ui.label(
                    egui::RichText::new("Reconstruct chopped floor")
                        .strong(),
                );
                ui.small(
                    "Replace the chopped region with an extruded average of \
                     cross-sections from a reference zone above the cut, so \
                     the cleaned mesh keeps its original length.",
                );
                ui.add_space(4.0);

                // Reference zone slider — bounded above by the
                // remaining mesh length on the floor side.
                // Sensible default range: 0..=applied_floor_mm
                // (sampling more above the cut than was chopped
                // is allowed, but capped to half the centerline
                // length so we don't walk past the tip).
                let max_ref_mm = max_trim_mm.min(total_length_mm - state.applied_floor_mm);
                ui.add(
                    egui::Slider::new(
                        &mut state.reconstruct_reference_mm,
                        0.0..=max_ref_mm,
                    )
                    .text("reference zone (mm)"),
                );

                // Shape radio buttons.
                ui.add_space(4.0);
                ui.label("Extrusion shape:");
                ui.radio_value(
                    &mut state.reconstruct_shape,
                    ReconstructShape::Constant,
                    "Constant (average radius)",
                );
                ui.radio_value(
                    &mut state.reconstruct_shape,
                    ReconstructShape::Taper,
                    "Taper toward floor",
                );
                ui.radio_value(
                    &mut state.reconstruct_shape,
                    ReconstructShape::Extrapolate,
                    "Extrapolate reference trend",
                );

                // Pending-apply drift indicator (mirrors the trim
                // panel's pattern).
                let reconstruct_drifted = match state.applied_reconstruct {
                    None => state.reconstruct_reference_mm > 0.0,
                    Some(applied) => {
                        (state.reconstruct_reference_mm - applied.reference_mm).abs() > 1e-6
                            || state.reconstruct_shape != applied.shape
                    }
                };
                if reconstruct_drifted {
                    let applied_desc = match state.applied_reconstruct {
                        None => "(not yet applied)".to_string(),
                        Some(applied) => format!(
                            "ref {:.1} mm / {}",
                            applied.reference_mm,
                            match applied.shape {
                                ReconstructShape::Constant => "Constant",
                                ReconstructShape::Taper => "Taper",
                                ReconstructShape::Extrapolate => "Extrapolate",
                            },
                        ),
                    };
                    ui.colored_label(
                        egui::Color32::from_rgb(240, 200, 80),
                        format!(
                            "⚠ Pending: applied = {applied_desc} — click [Apply reconstruct] to rebuild",
                        ),
                    );
                }

                ui.add_space(4.0);
                // CSP.4e.2 fix-forward — gate Apply reconstruct on
                // reference_zone > 0. Apply with reference=0 was a
                // silent no-op (the algorithm short-circuits to
                // flat-cap) and the user couldn't tell why nothing
                // happened. Disabling the button + hint makes the
                // gate visible.
                let can_apply = state.reconstruct_reference_mm > 0.0;
                ui.horizontal(|ui| {
                    ui.add_enabled_ui(can_apply, |ui| {
                        if ui.button("Apply reconstruct").clicked() {
                            state.reconstruct_pending = Some(ReconstructAction::Apply);
                        }
                    });
                    if state.applied_reconstruct.is_some()
                        && ui.button("Unapply reconstruct").clicked()
                    {
                        state.reconstruct_pending = Some(ReconstructAction::Reset);
                    }
                });

                if !can_apply {
                    ui.small(
                        "Drag the reference zone slider above 0 to enable Apply reconstruct.",
                    );
                }

                ui.add_space(4.0);
                ui.small(
                    "All three shapes (Constant / Taper / Extrapolate) generate \
                     extruded sidewall + flat cap. Taper pinches toward the new \
                     floor; Extrapolate continues the reference zone's natural taper.",
                );
            }
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
    smoothing_state: &mut SmoothingState,
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

            // Surface smoothing slider — applied at save time
            // via `cleanup_cleaned_mesh_for_disk`. Per-scan
            // because noise amplitude varies; raise for noisy
            // limb scans, lower to preserve fine detail.
            ui.add_space(6.0);
            ui.separator();
            ui.add_space(4.0);
            ui.label(egui::RichText::new("Surface smoothing").strong());
            ui.add(
                egui::Slider::new(
                    &mut smoothing_state.iterations,
                    0..=SMOOTHING_MAX_ITERATIONS,
                )
                .text("Taubin iterations"),
            );
            ui.small(
                "Suppresses sub-mm scanner noise so the cleaned STL represents the \
                 silicone-cast outcome (surface tension smooths sub-mm features \
                 physically). 0 = off (preserve all scan detail); raise for noisier \
                 scans, lower to preserve fine geometric features.",
            );

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
/// Inputs to [`save_cleaned_scan`], borrowed from the ECS resources by the
/// [`handle_save_action`] system. Bundling them lets the save pipeline be a
/// plain function, unit-testable without spinning up a Bevy world.
struct SaveInputs<'a> {
    scan: &'a IndexedMesh,
    original: &'a IndexedMesh,
    reorient: &'a ReorientState,
    recenter: &'a RecenterState,
    cap: &'a CapState,
    simplify: &'a SimplifyState,
    smoothing: &'a SmoothingState,
    centerline_trim: &'a CenterlineTrimState,
    source_stl: &'a Path,
    output_dir: &'a Path,
    stl_units: StlUnits,
    auto_center_offset_m: Vector3<f64>,
    auto_pca_quat: Option<UnitQuaternion<f64>>,
}

/// Structured result of a successful save — the pieces the status bar formats.
struct SaveOutcome {
    triangle_count: usize,
    trim_status_suffix: String,
    simplify_status_suffix: String,
    cleanup_status_suffix: String,
}

/// Why a save failed. The system maps each to a red status message.
#[derive(Debug)]
enum SaveError {
    /// A transform/trim value was NaN/Inf — refuse to write garbage to disk.
    NonFiniteTransform,
    /// `.prep.toml` serialization failed.
    TomlBuild(anyhow::Error),
    /// Writing the `.cleaned.stl` / `.prep.toml` to disk failed.
    Write(anyhow::Error),
}

/// The cleaned-scan save pipeline, extracted from [`handle_save_action`] so it
/// is a pure, unit-testable function: bake → centerline-trim →
/// reconstruct/auto-cap → hygiene cleanup → build `.prep.toml` → atomic write.
/// Behavior is identical to the prior inline system body (gated by a golden
/// test); the ECS system is now a thin shim that maps the result to the status
/// bar.
fn save_cleaned_scan(inputs: &SaveInputs) -> Result<SaveOutcome, SaveError> {
    let scan = inputs.scan;
    let reorient = inputs.reorient;
    let recenter = inputs.recenter;
    let cap = inputs.cap;
    let centerline_trim = inputs.centerline_trim;
    let smoothing = inputs.smoothing;
    let simplify = inputs.simplify;

    let stem = inputs
        .source_stl
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("scan");
    let cleaned_stl_path = inputs.output_dir.join(format!("{stem}.cleaned.stl"));
    let prep_toml_path = inputs.output_dir.join(format!("{stem}.prep.toml"));

    // Reject non-finite transforms before touching the disk. Per spec
    // §Architectural decisions §FS error handling, malformed slider
    // values (NaN / Inf from corrupted state) surface as a red error
    // rather than writing garbage to disk.
    let q = reorient.quaternion_physics();
    if !q.w.is_finite()
        || !q.i.is_finite()
        || !q.j.is_finite()
        || !q.k.is_finite()
        || !recenter.tx_mm.is_finite()
        || !recenter.ty_mm.is_finite()
        || !recenter.tz_mm.is_finite()
        || !centerline_trim.applied_tip_mm.is_finite()
        || !centerline_trim.applied_floor_mm.is_finite()
    {
        return Err(SaveError::NonFiniteTransform);
    }

    let mut cleaned = build_cleaned_mesh(scan, reorient, recenter, cap);

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
    let trim_pivot_centroid_m = scan.aabb().center();
    // CSP.4b.6 — use APPLIED trim values, not slider values. The
    // save-time cut must match the displayed mesh (what the user
    // sees), not the draft slider position.
    let centerline_world: Vec<Point3<f64>> = if (centerline_trim.applied_tip_mm > 0.0
        || centerline_trim.applied_floor_mm > 0.0)
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
            centerline_trim.applied_tip_mm,
            centerline_trim.applied_floor_mm,
        );
        // CSP.4e.2.4 — same weld fix as `update_displayed_mesh`.
        // Trim emits duplicate intersection vertices per shared
        // edge; without welding the cut boundary fragments and
        // `find_floor_loop_index` picks a scan-artifact loop.
        weld_vertices(&mut cleaned, SIMPLIFY_WELD_EPSILON_M);
        // CSP.4e.2 — save-time reconstruction branch. When the
        // user has applied reconstruct AND it's a shape this
        // commit supports (Constant), run the reconstructed-
        // geometry path; otherwise fall through to flat
        // auto-cap. Mirrors the `update_displayed_mesh` branch
        // so the on-disk STL matches the viewport.
        let did_reconstruct = match centerline_trim.applied_reconstruct {
            Some(ar) if centerline_trim.applied_floor_mm > 0.0 => {
                // CSP.4e.2 + 4e.3 — all three shapes (Constant,
                // Taper, Extrapolate) bake into the save output.
                let trimmed_centerline = trim_centerline_polyline(
                    &centerline_world,
                    centerline_trim.applied_tip_mm,
                    centerline_trim.applied_floor_mm,
                );
                cleaned = apply_reconstruction(
                    cleaned,
                    &trimmed_centerline,
                    centerline_trim.applied_floor_mm,
                    ar.reference_mm,
                    ar.shape,
                );
                true
            }
            _ => false,
        };
        if !did_reconstruct {
            trim_capped = auto_cap_open_boundaries(&mut cleaned);
        }
        let suffix_action = if did_reconstruct {
            "reconstructed".to_string()
        } else {
            format!(
                "capped {} boundary loop{}",
                trim_capped,
                if trim_capped == 1 { "" } else { "s" },
            )
        };
        trim_status_suffix = format!(
            ", trim: {} -> {} faces, {}",
            human_count(pre_trim_face_count),
            human_count(cleaned.faces.len()),
            suffix_action,
        );
    }

    // CSP.3b — mesh hygiene cleanup before simplify. Runs the four-
    // step pass (weld + degenerate + small-components + unreferenced)
    // so simplify operates on already-clean topology + the cleaned
    // STL passes downstream `simplify_decoder` (not just
    // `simplify_sloppy_decoder`). The report's non-zero terms feed
    // the status message as a confidence signal for the user.
    let cleanup = cleanup_cleaned_mesh_for_disk(&mut cleaned, smoothing.iterations);
    let cleanup_status_suffix = if cleanup.total() == 0 {
        String::new()
    } else {
        let smoothing_part = if cleanup.smoothing > 0 {
            format!(" / {} smoothing iters", cleanup.smoothing)
        } else {
            String::new()
        };
        format!(
            ", cleanup: {} welded / {} degenerate / {} small-component / {} unreferenced{}",
            cleanup.welded,
            cleanup.degenerate,
            cleanup.small_components,
            cleanup.unreferenced,
            smoothing_part,
        )
    };

    // CSP.4e fix-forward (2026-05-15 evening): slice 9.8's save-time
    // simplify-as-budget was removed. The in-app [Apply Simplify]
    // button is the user-controlled simplification entry point; a
    // hidden save-time re-simplify-if-target-exceeded was destructive
    // to user-applied reconstruction geometry (it collapsed
    // `apply_reconstruction`'s K=8 extrusion rings + flat-cap fan into
    // sliver "spike" triangles, user-reported 2026-05-15 evening).
    // Save now writes whatever the user has actively simplified +
    // trimmed + reconstructed in-app, byte-faithfully — no hidden
    // post-processing. The slider's `target_face_count` remains the
    // [Apply]-button target and feeds the `.prep.toml` provenance.
    let simplify_status_suffix = String::new();
    let triangle_count = cleaned.faces.len();

    let rotation = reorient.quaternion_physics();
    let translation = Vector3::new(
        recenter.tx_mm * 0.001,
        recenter.ty_mm * 0.001,
        recenter.tz_mm * 0.001,
    );
    let pivot_centroid_m = scan.aabb().center();
    // `[simplify]` provenance inputs: the as-loaded face count (in
    // OriginalScanMesh, untouched by Apply / Reset / save-time
    // budget) drives `original_face_count`; the final cleaned mesh's
    // face count is what landed on disk.
    let original_face_count = inputs.original.faces.len();
    let cleaned_aabb_m = cleaned.aabb();
    // Reconstructed-floor cap plane override (see
    // `ReconstructedFloorPlane` docstring). Computed in PRE-BAKE
    // physics frame from `cap.centerline_polyline` + the applied trim
    // values; `None` when reconstruction wasn't applied (zero floor
    // trim or shape unsupported), in which case the .prep.toml emits
    // the pre-reconstruction cap loops verbatim (legacy posture).
    let reconstructed_floor = if centerline_trim.applied_reconstruct.is_some()
        && centerline_trim.applied_floor_mm > 0.0
    {
        compute_reconstructed_floor_plane_physics(
            &cap.centerline_polyline,
            centerline_trim.applied_tip_mm,
            centerline_trim.applied_floor_mm,
        )
    } else {
        None
    };
    let toml_str = match build_prep_toml_string(
        inputs.source_stl,
        inputs.stl_units,
        inputs.auto_center_offset_m,
        inputs.auto_pca_quat,
        reorient,
        recenter,
        cap,
        centerline_trim,
        trim_capped,
        &format!("{stem}.cleaned.stl"),
        rotation,
        translation,
        pivot_centroid_m,
        simplify.target_face_count,
        simplify.was_applied,
        original_face_count,
        triangle_count,
        smoothing.iterations,
        &cleaned_aabb_m,
        reconstructed_floor,
    ) {
        Ok(s) => s,
        Err(e) => return Err(SaveError::TomlBuild(e)),
    };

    match atomic_write_save(&cleaned, &cleaned_stl_path, &prep_toml_path, &toml_str) {
        Ok(()) => Ok(SaveOutcome {
            triangle_count,
            trim_status_suffix,
            simplify_status_suffix,
            cleanup_status_suffix,
        }),
        Err(e) => Err(SaveError::Write(e)),
    }
}

/// ECS save system — a thin shim: guard the trigger, bundle the resources, run
/// [`save_cleaned_scan`], and map its result to the status bar.
#[allow(clippy::too_many_arguments)]
fn handle_save_action(
    mut pending: ResMut<SavePendingAction>,
    scan: Res<ScanMesh>,
    original: Res<OriginalScanMesh>,
    reorient: Res<ReorientState>,
    recenter: Res<RecenterState>,
    cap: Res<CapState>,
    simplify: Res<SimplifyState>,
    smoothing: Res<SmoothingState>,
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
        .unwrap_or("scan")
        .to_string();
    let inputs = SaveInputs {
        scan: &scan.0,
        original: &original.0,
        reorient: &reorient,
        recenter: &recenter,
        cap: &cap,
        simplify: &simplify,
        smoothing: &smoothing,
        centerline_trim: &centerline_trim,
        source_stl: &source.0,
        output_dir: &output_dir.0,
        stl_units: stl_units.0,
        auto_center_offset_m: auto_center.0,
        auto_pca_quat: auto_pca.0,
    };

    let (text, kind, dwell_secs) = match save_cleaned_scan(&inputs) {
        Ok(o) => (
            format!(
                "Saved {stem}.cleaned.stl ({} triangles{}{}{}) + {stem}.prep.toml",
                human_count(o.triangle_count),
                o.trim_status_suffix,
                o.simplify_status_suffix,
                o.cleanup_status_suffix,
            ),
            StatusKind::Normal,
            6.0,
        ),
        Err(SaveError::NonFiniteTransform) => (
            "Save failed: non-finite transform / trim values".to_string(),
            StatusKind::Error,
            6.0,
        ),
        Err(SaveError::TomlBuild(e)) => (
            format!("Save failed: building .prep.toml ({e:#})"),
            StatusKind::Error,
            6.0,
        ),
        Err(SaveError::Write(e)) => (format!("Save failed: {e:#}"), StatusKind::Error, 8.0),
    };
    status.text = text;
    status.kind = kind;
    status.auto_clear_at_secs = Some(time.elapsed_secs_f64() + dwell_secs);
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
        // Provenance: a freshly-constructed state has not yet seen
        // a user Apply click, so the `.prep.toml` `[simplify].applied`
        // flag should be `false` on a Save-without-Apply.
        assert!(!state.was_applied);
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

    #[test]
    fn mesh_looks_unwelded_flags_raw_stl_soup() {
        // Raw STL load: 3 verts per triangle → 3× faces. Flagged.
        assert!(mesh_looks_unwelded(600_000, 200_000));
        // Welded mesh: V ≈ F/2. Not flagged.
        assert!(!mesh_looks_unwelded(100_000, 200_000));
        // Exactly at the 2× threshold → flagged.
        assert!(mesh_looks_unwelded(400_000, 200_000));
        // Empty mesh → never flagged (avoids div-by-zero / false alarm).
        assert!(!mesh_looks_unwelded(0, 0));
    }

    #[test]
    fn leveling_euler_aligns_tilted_cap_normal_to_vertical() {
        // The iter-1 clone's cap normal (~5° lean toward -Z).
        let n = Vector3::new(0.061, -0.059, -0.996);
        let (r, p, y) = leveling_euler_deg(n);
        let q = UnitQuaternion::from_euler_angles(r.to_radians(), p.to_radians(), y.to_radians());
        let leveled = q * n.normalize();
        // Levels to the NEAREST axis (-Z) — floor stays down, no 180° flip.
        assert!(
            (leveled.z - (-1.0)).abs() < 1.0e-6,
            "cap normal should level to -Z, got {leveled:?}"
        );
        assert!(
            leveled.x.abs() < 1.0e-6 && leveled.y.abs() < 1.0e-6,
            "leveled normal should be vertical, got {leveled:?}"
        );
    }

    #[test]
    fn leveling_euler_targets_nearest_axis_no_flip() {
        // A normal leaning toward +Z levels to +Z (not flipped to -Z).
        let n = Vector3::new(0.05, 0.05, 0.997);
        let (r, p, y) = leveling_euler_deg(n);
        let q = UnitQuaternion::from_euler_angles(r.to_radians(), p.to_radians(), y.to_radians());
        let leveled = q * n.normalize();
        assert!(
            (leveled.z - 1.0).abs() < 1.0e-6,
            "should level to nearest axis +Z, got {leveled:?}"
        );
    }

    #[test]
    fn level_to_floor_cut_makes_curved_centerline_floor_horizontal() {
        // Curved centerline (tip at [0], floor at [last]) that tilts in
        // +X near the floor — so the floor cut plane is NOT horizontal.
        // The reconstruct builds its floor perpendicular to the tangent
        // at the floor cut; leveling that tangent must make it vertical
        // (→ floor horizontal), which the raw (tip-end) cap normal would
        // NOT achieve for a curved part.
        let poly = vec![
            Point3::new(0.0, 0.0, 0.10),
            Point3::new(0.005, 0.0, 0.05),
            Point3::new(0.030, 0.0, 0.0),
        ];
        let total = polyline_arc_length_m(&poly);
        let (_, floor_tangent) =
            point_along_polyline_at_arc_distance(&poly, total).expect("tangent at floor end");
        // Precondition: the floor-end tangent is genuinely tilted.
        assert!(
            floor_tangent.normalize().z.abs() < 0.97,
            "fixture floor tangent should be tilted, got {floor_tangent:?}"
        );
        let (r, p, y) = leveling_euler_deg(floor_tangent);
        let q = UnitQuaternion::from_euler_angles(r.to_radians(), p.to_radians(), y.to_radians());
        let leveled = q * floor_tangent.normalize();
        assert!(
            leveled.x.abs() < 1.0e-6 && leveled.y.abs() < 1.0e-6,
            "floor-cut tangent should level to vertical, got {leveled:?}"
        );
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

    // CSP.4c — ClipState + clip_mesh_against_world_z tests removed
    // alongside the feature.

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

    /// Cap-face 3D winding emission anchor for
    /// `auto_cap_open_boundaries` (see also the sister anchor on the
    /// `build_cleaned_mesh` path inside
    /// `build_cleaned_mesh_projects_loop_verts_onto_fit_plane`). The
    /// function must emit cap triangles whose 3D cross-product normals
    /// align with `orient_cap_normal_outward`'s returned OUTWARD
    /// direction. Without this assertion the emission could (and
    /// historically did, until the B arc fix at commit `99f2c512`)
    /// produce inward normals: cf-view's flat-shaded render path
    /// would compute inward face normals from the winding and shade
    /// cap faces DARK; mesh-io's `save_stl` would write inverted
    /// facet normals to disk for 3rd-party STL tools (Meshlab,
    /// ParaView, slicers). All in-tree SDF consumers tolerate either
    /// winding (FloodFillSign is topological-reachability-based;
    /// TriMeshDistance is unsigned; cf-cap-planes uses `.dot().abs()`),
    /// so the bug was visualization-only — but reader-grating.
    ///
    /// Fixture: 5-vertex square pyramid with the base OPEN (4 side
    /// triangles, no base triangulation). Apex at (0, 0, +1); base
    /// at z=0 with 4 corner verts. The open base is the boundary loop.
    /// `orient_cap_normal_outward` checks vertex distribution — 4 verts
    /// at z=0 (no sign), 1 at z=+1 (above) → returns -plane_normal →
    /// outward = -Z. After cap, every cap-face cross product must
    /// point -Z (outward, away from apex).
    #[test]
    fn auto_cap_open_boundaries_emits_outward_cap_normals() {
        // 5 verts + 4 side triangles + 2 cap triangles = 6 faces post-cap.
        let mut mesh = IndexedMesh::with_capacity(5, 6);
        // Base square (z=0) + apex (z=+1).
        mesh.vertices.push(Point3::new(-1.0, -1.0, 0.0)); // 0
        mesh.vertices.push(Point3::new(1.0, -1.0, 0.0)); // 1
        mesh.vertices.push(Point3::new(1.0, 1.0, 0.0)); // 2
        mesh.vertices.push(Point3::new(-1.0, 1.0, 0.0)); // 3
        mesh.vertices.push(Point3::new(0.0, 0.0, 1.0)); // 4 apex
        // 4 side triangles (CCW from outside — outward normals point
        // sideways + upward). Base is open.
        mesh.faces.push([0, 1, 4]);
        mesh.faces.push([1, 2, 4]);
        mesh.faces.push([2, 3, 4]);
        mesh.faces.push([3, 0, 4]);
        let initial_faces = mesh.faces.len();
        let capped = auto_cap_open_boundaries(&mut mesh);
        assert_eq!(capped, 1, "expected exactly 1 cap loop to be capped");
        let cap_faces = &mesh.faces[initial_faces..];
        assert!(
            !cap_faces.is_empty(),
            "auto-cap should append ≥1 cap triangle",
        );
        // Every cap face's 3D cross-product normal must point -Z
        // (outward, away from the apex above). Without the B-arc
        // fix the normals pointed +Z (inward toward apex), shading
        // dark under Bevy lighting in cf-view.
        for face in cap_faces {
            let v0 = mesh.vertices[face[0] as usize];
            let v1 = mesh.vertices[face[1] as usize];
            let v2 = mesh.vertices[face[2] as usize];
            let e1 = v1.coords - v0.coords;
            let e2 = v2.coords - v0.coords;
            let normal = e1.cross(&e2);
            assert!(
                normal.z < 0.0,
                "cap-face normal must point -Z (outward, away from apex); \
                 face {face:?} produced normal {normal:?}",
            );
        }
    }

    // ---- plane-mesh intersection + polygon centroid subroutines ----

    /// Build a unit-cube indexed mesh (corners at 0/1 on each axis,
    /// 8 verts + 12 triangles, CCW-outward winding). Hand-crafted
    /// fixture used by the [`intersect_plane_with_mesh`] tests; the
    /// general-purpose body-shape fixture helpers live alongside the
    /// algorithm tests further down.
    fn make_unit_cube_mesh() -> IndexedMesh {
        let mut mesh = IndexedMesh::with_capacity(8, 12);
        // Corner ordering: bit 0 = x, bit 1 = y, bit 2 = z.
        for i in 0..8 {
            mesh.vertices.push(Point3::new(
                f64::from(i & 1),
                f64::from((i >> 1) & 1),
                f64::from((i >> 2) & 1),
            ));
        }
        // 6 faces × 2 triangles, CCW from outside.
        let faces: [[u32; 3]; 12] = [
            // -Z (bottom)
            [0, 2, 1],
            [1, 2, 3],
            // +Z (top)
            [4, 5, 6],
            [5, 7, 6],
            // -Y (front)
            [0, 1, 4],
            [1, 5, 4],
            // +Y (back)
            [2, 6, 3],
            [3, 6, 7],
            // -X (left)
            [0, 4, 2],
            [2, 4, 6],
            // +X (right)
            [1, 3, 5],
            [3, 7, 5],
        ];
        for f in faces {
            mesh.faces.push(f);
        }
        mesh
    }

    /// Build a closed Z-axis frustum (cylinder if `radius_base ==
    /// radius_tip`; cone if `radius_tip == 0`) with `n_rings` axial
    /// rings × `n_segs` angular segments per ring, plus triangle-fan
    /// caps at z = ±height/2. Watertight; CCW-outward winding;
    /// centered on z-axis.
    ///
    /// `apply_noise(ring_idx, seg_idx, theta) -> f64` is added to
    /// the ring's interpolated radius per vertex. Use a closure
    /// returning `0.0` for a clean surface.
    fn make_closed_frustum_mesh(
        n_rings: usize,
        n_segs: usize,
        radius_base: f64,
        radius_tip: f64,
        height: f64,
        apply_noise: impl Fn(usize, usize, f64) -> f64,
    ) -> IndexedMesh {
        assert!(
            n_rings >= 2 && n_segs >= 3,
            "frustum needs >= 2 rings, >= 3 segs"
        );
        let vert_count = n_rings * n_segs + 2;
        let face_count = 2 * (n_rings - 1) * n_segs + 2 * n_segs;
        let mut mesh = IndexedMesh::with_capacity(vert_count, face_count);

        // Side-wall vertices: index = ring * n_segs + seg.
        for r in 0..n_rings {
            #[allow(clippy::cast_precision_loss)]
            let t = (r as f64) / ((n_rings - 1) as f64);
            let z = -height / 2.0 + t * height;
            let ring_radius = radius_base * (1.0 - t) + radius_tip * t;
            for k in 0..n_segs {
                #[allow(clippy::cast_precision_loss)]
                let theta = (k as f64) * std::f64::consts::TAU / (n_segs as f64);
                let radius = ring_radius + apply_noise(r, k, theta);
                mesh.vertices
                    .push(Point3::new(radius * theta.cos(), radius * theta.sin(), z));
            }
        }
        #[allow(clippy::cast_possible_truncation)]
        let bottom_center_idx = (n_rings * n_segs) as u32;
        let top_center_idx = bottom_center_idx + 1;
        mesh.vertices.push(Point3::new(0.0, 0.0, -height / 2.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, height / 2.0));

        // Side-wall: 2 CCW-outward triangles per quad.
        for r in 0..(n_rings - 1) {
            for k in 0..n_segs {
                #[allow(clippy::cast_possible_truncation)]
                let i00 = (r * n_segs + k) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i01 = (r * n_segs + (k + 1) % n_segs) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i10 = ((r + 1) * n_segs + k) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i11 = ((r + 1) * n_segs + (k + 1) % n_segs) as u32;
                mesh.faces.push([i00, i01, i11]);
                mesh.faces.push([i00, i11, i10]);
            }
        }
        // Bottom cap (normal -Z): CW when viewed from +Z above.
        for k in 0..n_segs {
            #[allow(clippy::cast_possible_truncation)]
            let i0 = k as u32;
            #[allow(clippy::cast_possible_truncation)]
            let i1 = ((k + 1) % n_segs) as u32;
            mesh.faces.push([bottom_center_idx, i1, i0]);
        }
        // Top cap (normal +Z): CCW when viewed from +Z above.
        let top_ring_base = (n_rings - 1) * n_segs;
        for k in 0..n_segs {
            #[allow(clippy::cast_possible_truncation)]
            let i0 = (top_ring_base + k) as u32;
            #[allow(clippy::cast_possible_truncation)]
            let i1 = (top_ring_base + (k + 1) % n_segs) as u32;
            mesh.faces.push([top_center_idx, i0, i1]);
        }
        mesh
    }

    /// Like [`make_closed_frustum_mesh`] but with **non-uniform
    /// angular sampling** per ring — vertices are placed at the
    /// user-provided `thetas` (in `[0, 2π)`, monotonic; same set
    /// repeated for every ring). Pure cylinder shape (`radius_base
    /// == radius_tip`), no noise.
    ///
    /// Used to verify density-independence: with `thetas` densely
    /// sampled on one side of the circle and sparsely on the other,
    /// the BOUNDARY shape is still the same circle but the vertex
    /// density is asymmetric — the polygon centroid is invariant
    /// (regression test for the iter-1 failure mode), while the
    /// prior vertex-centroid statistic would have been biased
    /// toward the dense side.
    fn make_density_biased_cylinder_mesh(
        n_rings: usize,
        thetas: &[f64],
        radius: f64,
        height: f64,
    ) -> IndexedMesh {
        let n_segs = thetas.len();
        assert!(
            n_rings >= 2 && n_segs >= 3,
            "density-biased cylinder needs >= 2 rings, >= 3 thetas"
        );
        let vert_count = n_rings * n_segs + 2;
        let face_count = 2 * (n_rings - 1) * n_segs + 2 * n_segs;
        let mut mesh = IndexedMesh::with_capacity(vert_count, face_count);

        for r in 0..n_rings {
            #[allow(clippy::cast_precision_loss)]
            let t = (r as f64) / ((n_rings - 1) as f64);
            let z = -height / 2.0 + t * height;
            for &theta in thetas {
                mesh.vertices
                    .push(Point3::new(radius * theta.cos(), radius * theta.sin(), z));
            }
        }
        #[allow(clippy::cast_possible_truncation)]
        let bottom_center_idx = (n_rings * n_segs) as u32;
        let top_center_idx = bottom_center_idx + 1;
        mesh.vertices.push(Point3::new(0.0, 0.0, -height / 2.0));
        mesh.vertices.push(Point3::new(0.0, 0.0, height / 2.0));

        for r in 0..(n_rings - 1) {
            for k in 0..n_segs {
                #[allow(clippy::cast_possible_truncation)]
                let i00 = (r * n_segs + k) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i01 = (r * n_segs + (k + 1) % n_segs) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i10 = ((r + 1) * n_segs + k) as u32;
                #[allow(clippy::cast_possible_truncation)]
                let i11 = ((r + 1) * n_segs + (k + 1) % n_segs) as u32;
                mesh.faces.push([i00, i01, i11]);
                mesh.faces.push([i00, i11, i10]);
            }
        }
        for k in 0..n_segs {
            #[allow(clippy::cast_possible_truncation)]
            let i0 = k as u32;
            #[allow(clippy::cast_possible_truncation)]
            let i1 = ((k + 1) % n_segs) as u32;
            mesh.faces.push([bottom_center_idx, i1, i0]);
        }
        let top_ring_base = (n_rings - 1) * n_segs;
        for k in 0..n_segs {
            #[allow(clippy::cast_possible_truncation)]
            let i0 = (top_ring_base + k) as u32;
            #[allow(clippy::cast_possible_truncation)]
            let i1 = (top_ring_base + (k + 1) % n_segs) as u32;
            mesh.faces.push([top_center_idx, i0, i1]);
        }
        mesh
    }

    /// Translate every vertex of a mesh by `offset` (utility for
    /// building off-axis fixtures — apply to a centered fixture to
    /// shift the whole body laterally).
    fn translate_mesh(mesh: &mut IndexedMesh, offset: Vector3<f64>) {
        for v in &mut mesh.vertices {
            v.coords += offset;
        }
    }

    /// Rotate every vertex of a mesh by `rotation` around origin
    /// (utility for building rotated-axis fixtures — apply to a
    /// Z-axis centered fixture to test XYZ-independence with a
    /// non-Z body axis).
    fn rotate_mesh(mesh: &mut IndexedMesh, rotation: UnitQuaternion<f64>) {
        for v in &mut mesh.vertices {
            v.coords = rotation * v.coords;
        }
    }

    /// Plane bisecting a unit cube perpendicular to +Z at `z=0.5`
    /// produces exactly one closed loop describing the unit square
    /// cross-section at z=0.5. **Contract test for
    /// `intersect_plane_with_mesh`.**
    ///
    /// The unit cube's 4 vertical edges each contribute one corner;
    /// each side face is split into 2 triangles whose internal
    /// DIAGONAL also crosses the plane, contributing 4 extra
    /// (collinear-on-the-square's-edges) intersection points. So
    /// the returned loop has 8 vertices around a square boundary,
    /// not the "minimal" 4 — this is correct algorithmic behavior
    /// (the polygon centroid is invariant to such mesh-diagonal
    /// subdivisions, verified by `polygon_centroid_3d_density_independent`).
    /// Test asserts the polygon's CENTROID + AREA, not vertex count.
    #[test]
    fn intersect_plane_unit_cube_bisecting_z_returns_square() {
        let mesh = make_unit_cube_mesh();
        let plane_pt = Point3::new(0.5, 0.5, 0.5);
        let plane_n = Vector3::new(0.0, 0.0, 1.0);
        let loops = intersect_plane_with_mesh(&plane_pt, &plane_n, &mesh);
        assert_eq!(
            loops.len(),
            1,
            "expected exactly 1 loop; got {}",
            loops.len()
        );
        let loop0 = &loops[0];
        assert!(
            loop0.len() >= 4,
            "expected ≥ 4 verts on square cross-section; got {}",
            loop0.len()
        );
        // Every vertex sits at z=0.5 and on the boundary of the
        // unit square (x or y coordinate equals 0 or 1).
        for p in loop0 {
            assert!((p.z - 0.5).abs() < 1e-9, "vert {p:?} not on z=0.5 plane");
            let on_boundary = (p.x.abs() < 1e-9 || (p.x - 1.0).abs() < 1e-9)
                || (p.y.abs() < 1e-9 || (p.y - 1.0).abs() < 1e-9);
            assert!(
                on_boundary,
                "vert {p:?} not on unit-square boundary at z=0.5"
            );
        }
        // Polygon-level geometric properties: area = 1.0, centroid
        // = (0.5, 0.5, 0.5).
        let area = polygon_area_3d(loop0, &plane_n);
        assert!((area - 1.0).abs() < 1e-9, "area should be 1.0; got {area}");
        let c = polygon_centroid_3d(loop0, &plane_n).expect("non-degenerate square");
        assert!((c.x - 0.5).abs() < 1e-9, "centroid x: {c:?}");
        assert!((c.y - 0.5).abs() < 1e-9, "centroid y: {c:?}");
        assert!((c.z - 0.5).abs() < 1e-9, "centroid z: {c:?}");
    }

    /// Plane parked outside the mesh's z-range produces no loops.
    /// Defensive test for the "no triangles crossed" path.
    #[test]
    fn intersect_plane_outside_mesh_returns_empty() {
        let mesh = make_unit_cube_mesh();
        let plane_pt = Point3::new(0.0, 0.0, 10.0);
        let plane_n = Vector3::new(0.0, 0.0, 1.0);
        let loops = intersect_plane_with_mesh(&plane_pt, &plane_n, &mesh);
        assert!(loops.is_empty(), "expected no loops; got {}", loops.len());
    }

    /// Defensive paths: empty mesh OR zero-magnitude normal return
    /// `Vec::new()` without panicking.
    #[test]
    fn intersect_plane_degenerate_input_returns_empty() {
        let empty = IndexedMesh::with_capacity(0, 0);
        let plane_pt = Point3::origin();
        let plane_n = Vector3::new(0.0, 0.0, 1.0);
        assert!(intersect_plane_with_mesh(&plane_pt, &plane_n, &empty).is_empty());

        let cube = make_unit_cube_mesh();
        let zero_n = Vector3::zeros();
        assert!(intersect_plane_with_mesh(&plane_pt, &zero_n, &cube).is_empty());
    }

    /// Polygon centroid of an axis-aligned unit square in the
    /// xy-plane sits at the square's geometric center. Contract
    /// test for [`polygon_centroid_3d`].
    #[test]
    fn polygon_centroid_3d_unit_square_at_center() {
        let square = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let n = Vector3::new(0.0, 0.0, 1.0);
        let c = polygon_centroid_3d(&square, &n).expect("non-degenerate square");
        assert!((c.x - 0.5).abs() < 1e-12, "x centroid: {c:?}");
        assert!((c.y - 0.5).abs() < 1e-12, "y centroid: {c:?}");
        assert!(c.z.abs() < 1e-12, "z centroid: {c:?}");
    }

    /// **Load-bearing test for density-independence.** A unit
    /// square traversed with 4 vertices vs. the SAME square with
    /// 4 extra collinear midpoints (8 verts on the boundary, same
    /// SHAPE) produces the same centroid. This is the geometric
    /// property that makes the centerline algorithm density-
    /// independent: adding redundant boundary samples does not
    /// shift the polygon centroid, unlike the failed vertex-centroid
    /// statistic (which would average toward the dense side).
    #[test]
    fn polygon_centroid_3d_density_independent() {
        let n = Vector3::new(0.0, 0.0, 1.0);
        let sparse = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        // Same boundary, traversed with midpoints between each
        // pair of corners (8 verts total, all on the boundary).
        let dense = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.5, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.5, 0.0),
        ];
        let c_sparse = polygon_centroid_3d(&sparse, &n).unwrap();
        let c_dense = polygon_centroid_3d(&dense, &n).unwrap();
        let diff = (c_sparse.coords - c_dense.coords).norm();
        assert!(
            diff < 1e-12,
            "density-asymmetric boundary shifted centroid: sparse={c_sparse:?}, dense={c_dense:?}, diff={diff}"
        );
    }

    /// Polygon centroid is invariant to traversal winding (CW vs
    /// CCW with respect to `plane_normal`): the signed-area sum in
    /// the numerator AND denominator both flip sign, canceling out.
    /// Defensive test — the centerline algorithm doesn't control the
    /// winding of loops produced by [`intersect_plane_with_mesh`].
    #[test]
    fn polygon_centroid_3d_winding_invariant() {
        let n = Vector3::new(0.0, 0.0, 1.0);
        let ccw = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let cw = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ];
        let c_ccw = polygon_centroid_3d(&ccw, &n).unwrap();
        let c_cw = polygon_centroid_3d(&cw, &n).unwrap();
        let diff = (c_ccw.coords - c_cw.coords).norm();
        assert!(
            diff < 1e-12,
            "winding changed centroid: ccw={c_ccw:?}, cw={c_cw:?}"
        );
    }

    /// `polygon_centroid_3d` returns `None` for degenerate inputs:
    /// < 3 vertices OR collinear vertices (zero projected area).
    #[test]
    fn polygon_centroid_3d_degenerate_returns_none() {
        let n = Vector3::new(0.0, 0.0, 1.0);
        let two = [Point3::origin(), Point3::new(1.0, 0.0, 0.0)];
        assert!(polygon_centroid_3d(&two, &n).is_none());
        let collinear = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];
        assert!(polygon_centroid_3d(&collinear, &n).is_none());
    }

    /// `polygon_area_3d` of a unit square is 1.0; winding-invariant.
    #[test]
    fn polygon_area_3d_unit_square() {
        let n = Vector3::new(0.0, 0.0, 1.0);
        let ccw = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        assert!((polygon_area_3d(&ccw, &n) - 1.0).abs() < 1e-12);
        let cw = [ccw[0], ccw[3], ccw[2], ccw[1]];
        assert!((polygon_area_3d(&cw, &n) - 1.0).abs() < 1e-12);
    }

    /// `compute_centerline_polyline` on a closed Z-axis cylinder
    /// produces polyline points along the Z axis (every centroid
    /// at (0, 0, z)) with monotonically increasing z. **Contract
    /// test for the per-slab area-weighted polygon centroid
    /// algorithm.** XYZ-independence (non-Z body axes) is
    /// separately covered by [`centerline_algorithm_xyz_independent`].
    #[test]
    fn centerline_along_closed_cylinder_z_axis_follows_z() {
        let mesh = make_closed_frustum_mesh(10, 16, 0.1, 0.1, 2.0, |_, _, _| 0.0);
        let polyline = compute_centerline_polyline(&mesh, Vector3::new(0.0, 0.0, 1.0), 10);
        assert!(
            polyline.len() >= 5,
            "expected ≥5 polyline pts; got {}",
            polyline.len()
        );
        for p in &polyline {
            assert!(p.x.abs() < 1e-9, "polyline x drifted: {p:?}");
            assert!(p.y.abs() < 1e-9, "polyline y drifted: {p:?}");
        }
        let zs: Vec<f64> = polyline.iter().map(|p| p.z).collect();
        for window in zs.windows(2) {
            assert!(
                window[1] > window[0],
                "polyline z should monotonically increase: {zs:?}",
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

    /// A tapering frustum (radius 1.0 at z=−0.5 → 0.05 at z=+0.5,
    /// 20 rings × 16 segs, with asymmetric per-vertex noise) has a
    /// centerline that stays on its true axis at EVERY slab,
    /// including the narrow-tip end. **Load-bearing test for the
    /// per-slab area-weighted polygon centroid algorithm vs. the
    /// failed vertex-centroid statistic** — vertex-centroid would
    /// be biased toward the noise direction at small-cross-section
    /// slabs; polygon-centroid is geometry-only, so the noise's
    /// effect averages out around the slab boundary.
    #[test]
    fn centerline_tapered_cylinder_tip_stays_on_axis() {
        const N_RINGS: usize = 20;
        const N_SEGS: usize = 16;
        const RADIUS_BASE: f64 = 1.0;
        const RADIUS_TIP: f64 = 0.05;
        const NOISE_MAG: f64 = 0.005; // 0.5 % of the base radius

        let mesh = make_closed_frustum_mesh(
            N_RINGS,
            N_SEGS,
            RADIUS_BASE,
            RADIUS_TIP,
            1.0,
            |ring, seg, _theta| {
                // Asymmetric noise: amplitude tied to (ring + seg)
                // so different rings have different noise patterns.
                NOISE_MAG * (((ring * 7 + seg * 13) % 11) as f64 / 11.0 - 0.5)
            },
        );

        let polyline = compute_centerline_polyline(&mesh, Vector3::new(0.0, 0.0, 1.0), N_RINGS);
        assert!(
            polyline.len() >= N_RINGS - 2,
            "expected ~N_RINGS polyline pts; got {}",
            polyline.len()
        );

        // Every polyline point should be on the body axis (x ≈ 0,
        // y ≈ 0) within the per-slab noise envelope. The polygon
        // centroid is unbiased w.r.t. uniform-around-the-ring noise;
        // residual drift comes from second-order chord-area
        // asymmetry (bounded by ~ NOISE_MAG for a 16-segment
        // polygon).
        for (i, p) in polyline.iter().enumerate() {
            assert!(
                p.x.abs() < NOISE_MAG,
                "x drift at slab {i}: {p:?} (NOISE_MAG = {NOISE_MAG})"
            );
            assert!(
                p.y.abs() < NOISE_MAG,
                "y drift at slab {i}: {p:?} (NOISE_MAG = {NOISE_MAG})"
            );
        }
    }

    // ---- New algorithm regression tests (docs/CENTERLINE_SPEC.md §6.1) ----

    /// **Spec test #1 (contract)** — a perfectly symmetric closed
    /// cylinder along Z produces a centerline pinned to the Z axis
    /// at every slab. Tightest tolerance of the suite.
    #[test]
    fn centerline_algorithm_axisymmetric_cylinder_along_axis() {
        let mesh = make_closed_frustum_mesh(20, 32, 0.05, 0.05, 0.2, |_, _, _| 0.0);
        let polyline = compute_centerline_polyline(&mesh, Vector3::new(0.0, 0.0, 1.0), 30);
        assert_eq!(polyline.len(), 30);
        for (i, p) in polyline.iter().enumerate() {
            assert!(p.x.abs() < 1e-9, "x drift at slab {i}: {p:?}");
            assert!(p.y.abs() < 1e-9, "y drift at slab {i}: {p:?}");
        }
    }

    /// **Spec test #2 (regression for iter-1 failure mode)** — a
    /// closed cylinder translated by Δx = 5mm off the world Z axis
    /// produces a centerline pinned to the BODY'S axis (i.e. at
    /// (5mm, 0, z)), NOT to the world Z axis. This is the iteration-5
    /// failure mode the algorithm switch fixes.
    #[test]
    fn centerline_algorithm_offset_cylinder_tracks_body_axis() {
        const OFFSET_X: f64 = 0.005;
        let mut mesh = make_closed_frustum_mesh(20, 32, 0.05, 0.05, 0.2, |_, _, _| 0.0);
        translate_mesh(&mut mesh, Vector3::new(OFFSET_X, 0.0, 0.0));
        let polyline = compute_centerline_polyline(&mesh, Vector3::new(0.0, 0.0, 1.0), 30);
        assert_eq!(polyline.len(), 30);
        for (i, p) in polyline.iter().enumerate() {
            assert!(
                (p.x - OFFSET_X).abs() < 1e-9,
                "x should track OFFSET_X at slab {i}: {p:?}"
            );
            assert!(p.y.abs() < 1e-9, "y should be 0 at slab {i}: {p:?}");
        }
    }

    /// **Spec test #3 (load-bearing: density-independence)** — a
    /// cylinder sampled with 80% of its angular vertices in the
    /// right semi-circle and 20% in the left semi-circle has the
    /// SAME centerline as a uniformly-sampled cylinder of the same
    /// radius. This is the property that escapes ALL five prior
    /// failed algorithms (Kasa, Kasa+prior, PCA, vertex-centroid,
    /// AABB-midpoint were each biased by sampling asymmetry).
    ///
    /// Vertex-centroid baseline (FOR THE OLD ALGORITHM, would have
    /// FAILED): mean x of the boundary samples for the dense fixture
    /// is ~ +0.022 m (vs. radius 0.05 → 44% bias), which would have
    /// pulled the prior centerline off-axis by ~22mm on a 50mm-radius
    /// body. The new algorithm produces sub-mm residual.
    #[test]
    fn centerline_algorithm_density_independent() {
        const RADIUS: f64 = 0.05;
        // 80% (24/30) of thetas densely packed in the right
        // semi-circle (theta ∈ (-π/2, π/2)); 20% (6/30) sparsely
        // in the left semi-circle. Boundary shape is still the
        // same circle.
        let mut thetas: Vec<f64> = Vec::new();
        for k in 0..24 {
            #[allow(clippy::cast_precision_loss)]
            let f = (k as f64 + 0.5) / 24.0;
            thetas.push(-std::f64::consts::FRAC_PI_2 + f * std::f64::consts::PI);
        }
        for k in 0..6 {
            #[allow(clippy::cast_precision_loss)]
            let f = (k as f64 + 0.5) / 6.0;
            thetas.push(std::f64::consts::FRAC_PI_2 + f * std::f64::consts::PI);
        }
        let mesh = make_density_biased_cylinder_mesh(20, &thetas, RADIUS, 0.2);
        let polyline = compute_centerline_polyline(&mesh, Vector3::new(0.0, 0.0, 1.0), 30);
        assert_eq!(polyline.len(), 30);
        // Residual chord-area asymmetry: ~(1 - cos(π / n_dense))
        // for the dense side × similar on sparse side. With 24 vs 6
        // segments, residual x-drift is at most a few percent of
        // RADIUS (sub-mm at RADIUS=50mm). The vertex-centroid
        // statistic on the SAME fixture would drift by ~50% of
        // RADIUS — this tolerance is 100× tighter.
        let tol_x = 0.05 * RADIUS;
        for (i, p) in polyline.iter().enumerate() {
            assert!(
                p.x.abs() < tol_x,
                "density-biased x drift at slab {i}: {p:?} (tol = {tol_x})"
            );
            assert!(
                p.y.abs() < tol_x,
                "density-biased y drift at slab {i}: {p:?} (tol = {tol_x})"
            );
        }
    }

    /// **Spec test #4 (degenerate-slab interpolation)** — a needle
    /// cone (radius 1.0 at z=−0.5 → 0.00001 at z=+0.5) has its
    /// tip-end slabs fall below MIN_SLAB_AREA_M2 (π × 1e-5² ≈
    /// 3e-10 m² < 1e-8 m²). Those degenerate slabs must be filled
    /// in by linear interpolation / axial extrapolation; the
    /// polyline length should match `n_slices` exactly (no gaps)
    /// and the filled-in points should stay on the body axis.
    #[test]
    fn centerline_algorithm_degenerate_slabs_filled_by_interpolation() {
        let mesh = make_closed_frustum_mesh(20, 32, 1.0, 1e-5, 1.0, |_, _, _| 0.0);
        let polyline = compute_centerline_polyline(&mesh, Vector3::new(0.0, 0.0, 1.0), 30);
        assert_eq!(
            polyline.len(),
            30,
            "polyline length should equal n_slices even with degenerate slabs"
        );
        // All points (interior + filled-in tip) should be on the
        // body axis. The interpolation from non-degenerate
        // neighbors stays on axis since those neighbors are on axis
        // for a symmetric cone.
        for (i, p) in polyline.iter().enumerate() {
            assert!(p.x.abs() < 1e-9, "x drift at slab {i}: {p:?}");
            assert!(p.y.abs() < 1e-9, "y drift at slab {i}: {p:?}");
        }
    }

    /// **Spec test #5 (XYZ-independence)** — a closed cylinder
    /// whose body axis has been rotated 30° around Y produces a
    /// centerline that follows the ROTATED body axis, not the
    /// world Z axis. The algorithm uses `spine_hint` (not world
    /// directions) for slicing, so any spine_hint direction works.
    /// See `project_scans_axis_orientation` memo.
    #[test]
    fn centerline_algorithm_xyz_independent() {
        let mut mesh = make_closed_frustum_mesh(20, 32, 0.05, 0.05, 0.2, |_, _, _| 0.0);
        let axis_angle_deg = 30.0;
        let rotation = UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            axis_angle_deg * std::f64::consts::PI / 180.0,
        );
        rotate_mesh(&mut mesh, rotation);
        let rotated_axis = rotation * Vector3::new(0.0, 0.0, 1.0);
        let polyline = compute_centerline_polyline(&mesh, rotated_axis, 30);
        assert_eq!(polyline.len(), 30);
        // Each polyline point should lie on the line through origin
        // in direction `rotated_axis`. Cross-product magnitude with
        // the axis direction tells us the perpendicular distance —
        // should be ~ 0 for an on-axis point.
        for (i, p) in polyline.iter().enumerate() {
            let perp = p.coords.cross(&rotated_axis).norm();
            assert!(
                perp < 1e-9,
                "perp distance from rotated axis at slab {i}: {perp} (point {p:?})"
            );
        }
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

    /// CSP.4e.2.3 — `smooth_polyline` pins the endpoints and
    /// flattens an interior zigzag. Fixture: 5-point polyline
    /// where the middle point is offset perpendicular to the
    /// otherwise-straight line. After 3 iterations of 3-tap
    /// moving average, the middle point should be pulled
    /// substantially back toward the straight line; endpoints
    /// `polyline[0]` and `polyline[N-1]` should be unchanged.
    #[test]
    fn smooth_polyline_flattens_interior_pins_endpoints() {
        let raw = vec![
            Point3::new(0.0, 0.0, 0.0), // endpoint
            Point3::new(0.01, 0.0, 0.0),
            Point3::new(0.02, 0.005, 0.0), // wobble +5 mm
            Point3::new(0.03, 0.0, 0.0),
            Point3::new(0.04, 0.0, 0.0), // endpoint
        ];
        let smoothed = smooth_polyline(&raw, 3);
        assert_eq!(smoothed.len(), raw.len());
        // Endpoints bit-exact preserved.
        assert_eq!(smoothed[0], raw[0]);
        assert_eq!(smoothed[4], raw[4]);
        // Middle point's y substantially reduced (started at
        // 5 mm; after 3 iterations of 3-tap should be < 2 mm).
        assert!(
            smoothed[2].y.abs() < raw[2].y.abs(),
            "middle should smooth toward 0, got y = {}",
            smoothed[2].y,
        );
        assert!(
            smoothed[2].y.abs() < 0.002,
            "after 3 iterations 5 mm wobble should drop below 2 mm, got {}",
            smoothed[2].y,
        );
    }

    /// `smooth_polyline` is a no-op for < 3-point input (no
    /// interior to smooth) and for `iterations == 0`.
    #[test]
    fn smooth_polyline_no_op_when_too_short_or_zero_iters() {
        let two_pt = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
        assert_eq!(smooth_polyline(&two_pt, 5), two_pt);
        let three_pt = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.5, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ];
        assert_eq!(smooth_polyline(&three_pt, 0), three_pt);
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

    // ----- stable_inward_tangent --------------------------------------
    //
    // Helper introduced 2026-05-16 to fix `apply_reconstruction`'s
    // K-ring extrusion direction (previously derived from the
    // noisy last centerline segment, which tilted the extrusion
    // off the body's actual axis on the iter-1 sock-over-capsule
    // scan). Pinned here so future refactors don't silently
    // revert to the single-segment tangent.

    /// 2-point polyline → look-back walks the only segment fully
    /// and returns the head→tail (then negated) direction. The
    /// `lookback_m` exceeding the polyline arc-length falls
    /// through to the head endpoint, NOT an error.
    #[test]
    fn stable_inward_tangent_falls_back_to_last_segment_on_2_point_polyline() {
        let polyline = [Point3::new(0.0, 0.0, 1.0), Point3::new(0.0, 0.0, 0.0)];
        // Cut endpoint = (0,0,0); look-back at (0,0,1). Inward
        // direction (cut → body) = +Z.
        let dir = stable_inward_tangent(&polyline, 0.020).expect("tangent");
        assert!((dir - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
    }

    /// On a long polyline the look-back point sits at exactly
    /// `lookback_m` arc-length from the cut endpoint (interpolated
    /// within the relevant segment). Pin the resulting direction
    /// against a known-good handcomputed answer.
    #[test]
    fn stable_inward_tangent_walks_back_lookback_arc_length() {
        // 4-point centerline along +Z, segments of length 0.010 m.
        // Total arc length = 0.030 m.
        let polyline = [
            Point3::new(0.0, 0.0, 0.030), // body end
            Point3::new(0.0, 0.0, 0.020),
            Point3::new(0.0, 0.0, 0.010),
            Point3::new(0.0, 0.0, 0.000), // cut end
        ];
        // Look back 0.015 m → between segments [2] and [1] (i.e.
        // halfway through the segment from (0,0,0.010) to
        // (0,0,0.020)). Look-back point = (0,0,0.015).
        let dir = stable_inward_tangent(&polyline, 0.015).expect("tangent");
        // dir = (lookback - cut) / norm = (0,0,0.015) / 0.015 = +Z.
        assert!((dir - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
    }

    /// Look-back exceeding the polyline arc-length walks the
    /// whole polyline and returns the head→tail unit vector.
    /// Important so very-short trim-cut polylines don't crash.
    #[test]
    fn stable_inward_tangent_uses_full_polyline_when_lookback_exceeds_arc_length() {
        // Total arc length = 0.005 m; ask for 0.020 m look-back.
        let polyline = [Point3::new(0.0, 0.0, 0.005), Point3::new(0.0, 0.0, 0.000)];
        let dir = stable_inward_tangent(&polyline, 0.020).expect("tangent");
        assert!((dir - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
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
            &cap,
            &centerline_trim, // CSP.4b — default = no trim
            0,                // CSP.4b — no boundaries auto-capped
            "scan.cleaned.stl",
            rotation,
            translation,
            Point3::origin(),
            200_000, // simplify target
            true,    // simplify_ran — user clicked Apply
            500_000, // original face count
            200_000, // achieved face count
            8,       // smoothing iterations
            &cleaned_aabb,
            None, // reconstructed_floor — tests do not exercise reconstruction
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
        // CSP.4c — `[clip]` block removed with the feature. Pin
        // the absence so a future re-introduction surfaces here.
        assert!(parsed.get("clip").is_none(), "clip block should be absent");
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
        // CSP.4e.5 — reconstruct sub-block ABSENT when no
        // reconstruction was applied. Guards against accidental
        // always-on emission.
        assert!(
            trim.get("reconstruct").is_none(),
            "reconstruct sub-block should be omitted when applied_reconstruct = None",
        );
        Ok(())
    }

    /// CSP.4e.5 — when reconstruction WAS applied, the
    /// `[centerline_trim.reconstruct]` sub-block is emitted with
    /// `shape` + `reference_mm`. Pins the on-disk shape so future
    /// audit / migration tooling can recover the reconstruction
    /// parameters from the saved `.prep.toml`.
    #[test]
    fn prep_toml_emits_reconstruct_subblock_when_applied() -> Result<()> {
        let reorient = ReorientState::default();
        let recenter = RecenterState::default();
        let cap = CapState::default();
        let rotation = reorient.quaternion_physics();
        let translation = nalgebra::Vector3::zeros();
        let cleaned_aabb = Aabb {
            min: Point3::new(-0.04, -0.03, -0.06),
            max: Point3::new(0.04, 0.03, 0.06),
        };
        let centerline_trim = CenterlineTrimState {
            trim_floor_mm: 30.0,
            applied_floor_mm: 30.0,
            applied_reconstruct: Some(AppliedReconstruct {
                reference_mm: 25.0,
                shape: ReconstructShape::Taper,
            }),
            ..CenterlineTrimState::default()
        };
        let s = build_prep_toml_string(
            Path::new("/tmp/scan.stl"),
            StlUnits::Mm,
            Vector3::zeros(),
            None,
            &reorient,
            &recenter,
            &cap,
            &centerline_trim,
            0, // capped_loops — 0 when reconstruct ran (no auto-cap at floor)
            "scan.cleaned.stl",
            rotation,
            translation,
            Point3::origin(),
            200_000,
            true,
            500_000,
            200_000,
            8,
            &cleaned_aabb,
            None, // reconstructed_floor — tests do not exercise reconstruction
        )?;
        let parsed: toml::Value = toml::from_str(&s)?;
        let trim = parsed
            .get("centerline_trim")
            .expect("centerline_trim block missing");
        let reconstruct = trim
            .get("reconstruct")
            .expect("reconstruct sub-block missing when applied");
        assert_eq!(reconstruct.get("shape").unwrap().as_str().unwrap(), "taper");
        assert!(
            (reconstruct.get("reference_mm").unwrap().as_float().unwrap() - 25.0).abs() < 1e-12
        );
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
            &cap,
            &centerline_trim,
            0,
            "scan.cleaned.stl",
            rotation,
            translation,
            Point3::origin(),
            200_000,
            true,
            500_000,
            200_000,
            8,
            &cleaned_aabb,
            None, // reconstructed_floor — tests do not exercise reconstruction
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

    /// `[simplify].applied` reflects the `simplify_ran` parameter
    /// **directly**, NOT a `achieved < original` face-count
    /// inference. Pre-fix (2026-05-15 night, post slice-9.8 revert)
    /// the inference produced false positives whenever the save-time
    /// cleanup pass dropped any faces — even when the user never
    /// clicked Apply. Both directions pinned so future refactors
    /// can't silently re-introduce the inference.
    #[test]
    fn prep_toml_simplify_applied_reflects_simplify_ran_parameter() -> Result<()> {
        let reorient = ReorientState::default();
        let recenter = RecenterState::default();
        let cap = CapState::default();
        let rotation = reorient.quaternion_physics();
        let translation = nalgebra::Vector3::zeros();
        let cleaned_aabb = Aabb {
            min: Point3::new(0.0, 0.0, 0.0),
            max: Point3::new(0.1, 0.1, 0.1),
        };
        let centerline_trim = CenterlineTrimState::default();

        // Case A — user did NOT click Apply, but cleanup dropped
        // faces (`achieved < original`). Old inference would
        // mis-report `applied = true`; the new wiring honors the
        // explicit `simplify_ran = false`.
        let s_false = build_prep_toml_string(
            Path::new("/tmp/scan.stl"),
            StlUnits::Mm,
            Vector3::zeros(),
            None,
            &reorient,
            &recenter,
            &cap,
            &centerline_trim,
            0,
            "scan.cleaned.stl",
            rotation,
            translation,
            Point3::origin(),
            200_000,
            false, // simplify_ran — user did NOT click Apply
            500_000,
            499_900, // achieved < original (cleanup dropped 100 faces)
            0,       // smoothing iterations
            &cleaned_aabb,
            None, // reconstructed_floor — tests do not exercise reconstruction
        )?;
        let parsed_false: toml::Value = toml::from_str(&s_false)?;
        let applied_false = parsed_false
            .get("simplify")
            .and_then(|s| s.get("applied"))
            .and_then(|v| v.as_bool())
            .expect("simplify.applied missing or not a bool");
        assert!(
            !applied_false,
            "simplify.applied must be false when simplify_ran = false, \
             regardless of achieved < original",
        );

        // Case B — user DID click Apply. Honors `simplify_ran = true`.
        let s_true = build_prep_toml_string(
            Path::new("/tmp/scan.stl"),
            StlUnits::Mm,
            Vector3::zeros(),
            None,
            &reorient,
            &recenter,
            &cap,
            &centerline_trim,
            0,
            "scan.cleaned.stl",
            rotation,
            translation,
            Point3::origin(),
            200_000,
            true,
            500_000,
            200_000,
            0,
            &cleaned_aabb,
            None, // reconstructed_floor — tests do not exercise reconstruction
        )?;
        let parsed_true: toml::Value = toml::from_str(&s_true)?;
        let applied_true = parsed_true
            .get("simplify")
            .and_then(|s| s.get("applied"))
            .and_then(|v| v.as_bool())
            .expect("simplify.applied missing or not a bool");
        assert!(
            applied_true,
            "simplify.applied must be true when simplify_ran = true",
        );
        Ok(())
    }

    // ---- Reconstructed-floor plane override --------------------------

    #[test]
    fn compute_reconstructed_floor_plane_matches_apply_reconstruction_geometry() {
        // Synthetic 40 mm centerline along +Z; floor end at z = 0.
        // The reconstruction workflow is: trim removes the last
        // `applied_floor_mm` of arc length, then `apply_reconstruction`
        // extends the body by `applied_floor_mm` along the OUTWARD
        // direction from the trimmed endpoint. The trim-then-extend
        // pair restores the original body length with cleaner floor
        // geometry, so `bottom_center` lands at the original
        // polyline's floor endpoint (z = 0.000 in this fixture).
        let polyline = vec![
            Point3::new(0.0, 0.0, 0.040),
            Point3::new(0.0, 0.0, 0.020),
            Point3::new(0.0, 0.0, 0.010),
            Point3::new(0.0, 0.0, 0.000),
        ];
        let rf =
            compute_reconstructed_floor_plane_physics(&polyline, 0.0, 10.0).expect("Some plane");
        // Trim drops last 10 mm → trimmed_last = z=0.010; outward
        // extension by 10 mm in -Z → bottom_center z = 0.000.
        assert!((rf.centroid_m.x).abs() < 1e-9);
        assert!((rf.centroid_m.y).abs() < 1e-9);
        assert!(
            (rf.centroid_m.z).abs() < 1e-9,
            "bottom_center.z = {} (expected 0.000)",
            rf.centroid_m.z,
        );
        // Outward normal = -inward (+Z) = -Z.
        assert!((rf.normal.x).abs() < 1e-9);
        assert!((rf.normal.y).abs() < 1e-9);
        assert!((rf.normal.z - (-1.0)).abs() < 1e-9);
    }

    #[test]
    fn compute_reconstructed_floor_plane_iter1_reproducer() {
        // Reproduces the iter-1 numerical hypothesis check: the
        // synthetic offset along the cap normal should be ~ extension
        // along the tangent ≈ 40 mm (since recorded cap_centroid is
        // the pre-reconstruction boundary fit-plane at the trim cut,
        // and bottom_center is 40 mm beyond it along the centerline
        // tangent). Pin the math on a tangent-mostly-Z polyline so
        // future refactors of stable_inward_tangent don't silently
        // drift the recorded plane back toward the old stale position.
        let polyline = vec![
            Point3::new(0.001, -0.002, 0.060),
            Point3::new(0.001, -0.002, 0.040),
            Point3::new(0.001, -0.002, 0.020),
            Point3::new(0.001, -0.002, 0.000),
        ];
        let rf =
            compute_reconstructed_floor_plane_physics(&polyline, 0.0, 20.0).expect("Some plane");
        // Trim drops last 20 mm → trimmed_last = z=0.020; outward
        // (= -Z) extension by 20 mm → bottom_center z = 0.000.
        assert!((rf.centroid_m.z).abs() < 1e-9);
        // The override's centroid sits 20 mm below the trim cut
        // (trimmed_last z = 0.020) along the tangent. cf-device-design's
        // candidate-A clip would otherwise hit the body at trim_cut +
        // recorded-plane-offset; this assertion is the load-bearing
        // contract that the override moves it to bottom_center.
        let trim_cut_z = 0.020;
        let override_minus_trim = rf.centroid_m.z - trim_cut_z;
        assert!((override_minus_trim - (-0.020)).abs() < 1e-9);
    }

    #[test]
    fn compute_reconstructed_floor_plane_returns_none_for_zero_floor_trim() {
        let polyline = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.010)];
        assert!(compute_reconstructed_floor_plane_physics(&polyline, 0.0, 0.0).is_none());
    }

    #[test]
    fn compute_reconstructed_floor_plane_returns_none_for_short_polyline() {
        let polyline = vec![Point3::new(0.0, 0.0, 0.0)];
        assert!(compute_reconstructed_floor_plane_physics(&polyline, 0.0, 10.0).is_none());
        let polyline2 = vec![Point3::new(0.0, 0.0, 0.020), Point3::new(0.0, 0.0, 0.010)];
        // applied_floor_mm = 15 > polyline length (10 mm) → trim
        // consumes everything → None.
        assert!(compute_reconstructed_floor_plane_physics(&polyline2, 0.0, 15.0).is_none());
    }

    #[test]
    fn build_prep_toml_string_overrides_floor_cap_plane_when_reconstructed() {
        // Build a CapState with one large cap loop whose pre-recon
        // fit plane sits at (0, 0, -0.05) with normal (0, 0, -1). The
        // override should replace this with the reconstructed plane.
        let mut cap = CapState::default();
        cap.loops.push(DetectedCapLoop {
            vertex_indices: (0..32).collect(),
            plane_centroid: Point3::new(0.0, 0.0, -0.050),
            plane_normal: Vector3::new(0.0, 0.0, -1.0),
            plane_fit_r_squared: 0.99,
            include: true,
        });
        let override_plane = ReconstructedFloorPlane {
            centroid_m: Point3::new(0.001, 0.002, -0.060),
            normal: Vector3::new(0.0, 0.0, -1.0),
        };
        let cleaned_aabb = Aabb::new(
            Point3::new(-0.05, -0.05, -0.06),
            Point3::new(0.05, 0.05, 0.07),
        );
        let s = build_prep_toml_string(
            Path::new("/tmp/scan.stl"),
            StlUnits::Mm,
            Vector3::zeros(),
            None,
            &ReorientState::default(),
            &RecenterState::default(),
            &cap,
            &CenterlineTrimState::default(),
            0,
            "scan.cleaned.stl",
            UnitQuaternion::identity(),
            Vector3::zeros(),
            Point3::origin(),
            200_000,
            false,
            500_000,
            200_000,
            0,
            &cleaned_aabb,
            Some(override_plane),
        )
        .expect("build_prep_toml_string");
        let parsed: toml::Value = toml::from_str(&s).unwrap();
        let loops = parsed
            .get("caps")
            .and_then(|c| c.get("loops"))
            .and_then(|l| l.as_array())
            .expect("caps.loops");
        assert_eq!(loops.len(), 1);
        let centroid = loops[0]
            .get("plane_centroid_m")
            .and_then(|v| v.as_array())
            .expect("plane_centroid_m");
        assert!((centroid[0].as_float().unwrap() - 0.001).abs() < 1e-12);
        assert!((centroid[1].as_float().unwrap() - 0.002).abs() < 1e-12);
        assert!((centroid[2].as_float().unwrap() - (-0.060)).abs() < 1e-12);
        let r_sq = loops[0]
            .get("plane_fit_r_squared")
            .and_then(|v| v.as_float())
            .expect("plane_fit_r_squared");
        assert!((r_sq - 1.0).abs() < 1e-12, "override sets R² = 1.0");
    }

    #[test]
    fn build_prep_toml_string_no_override_leaves_floor_cap_plane_untouched() {
        // Same setup as the override test but `reconstructed_floor =
        // None` — emitted plane must equal the cap loop's detected
        // fit plane verbatim.
        let mut cap = CapState::default();
        cap.loops.push(DetectedCapLoop {
            vertex_indices: (0..32).collect(),
            plane_centroid: Point3::new(0.0, 0.0, -0.050),
            plane_normal: Vector3::new(0.0, 0.0, -1.0),
            plane_fit_r_squared: 0.99,
            include: true,
        });
        let cleaned_aabb = Aabb::new(
            Point3::new(-0.05, -0.05, -0.05),
            Point3::new(0.05, 0.05, 0.07),
        );
        let s = build_prep_toml_string(
            Path::new("/tmp/scan.stl"),
            StlUnits::Mm,
            Vector3::zeros(),
            None,
            &ReorientState::default(),
            &RecenterState::default(),
            &cap,
            &CenterlineTrimState::default(),
            0,
            "scan.cleaned.stl",
            UnitQuaternion::identity(),
            Vector3::zeros(),
            Point3::origin(),
            200_000,
            false,
            500_000,
            200_000,
            0,
            &cleaned_aabb,
            None,
        )
        .expect("build_prep_toml_string");
        let parsed: toml::Value = toml::from_str(&s).unwrap();
        let loops = parsed
            .get("caps")
            .and_then(|c| c.get("loops"))
            .and_then(|l| l.as_array())
            .expect("caps.loops");
        let centroid = loops[0]
            .get("plane_centroid_m")
            .and_then(|v| v.as_array())
            .expect("plane_centroid_m");
        assert!((centroid[2].as_float().unwrap() - (-0.050)).abs() < 1e-12);
        let r_sq = loops[0]
            .get("plane_fit_r_squared")
            .and_then(|v| v.as_float())
            .expect("plane_fit_r_squared");
        assert!((r_sq - 0.99).abs() < 1e-12);
    }

    /// End-to-end save: `save_cleaned_scan` runs the full pipeline (bake →
    /// hygiene cleanup → build `.prep.toml` → atomic write) on a synthetic cube
    /// and writes both artifacts with a parseable `.prep.toml`. This is the
    /// regression gate the extraction enables — the orchestration was previously
    /// locked inside an ECS system and could not be exercised in a unit test.
    #[test]
    fn save_cleaned_scan_writes_both_artifacts_end_to_end() {
        // A unit cube (12 triangles) — above CLEANUP_MIN_COMPONENT_FACES (10),
        // so it survives the hygiene cleanup with all faces intact.
        let mut scan = IndexedMesh::with_capacity(8, 12);
        for c in [
            (0.0, 0.0, 0.0),
            (0.04, 0.0, 0.0),
            (0.04, 0.04, 0.0),
            (0.0, 0.04, 0.0),
            (0.0, 0.0, 0.04),
            (0.04, 0.0, 0.04),
            (0.04, 0.04, 0.04),
            (0.0, 0.04, 0.04),
        ] {
            scan.vertices.push(Point3::new(c.0, c.1, c.2));
        }
        for f in [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [3, 7, 6],
            [3, 6, 2],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ] {
            scan.faces.push(f);
        }

        let dir = std::env::temp_dir().join("cf_scan_prep_save_cleaned_scan_e2e");
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("create temp dir");
        let source = std::path::PathBuf::from("/tmp/widget.stl");

        let reorient = ReorientState::default();
        let recenter = RecenterState::default();
        let cap = CapState::default();
        let simplify = SimplifyState::default();
        let smoothing = SmoothingState::default();
        let trim = CenterlineTrimState::default();
        let inputs = SaveInputs {
            scan: &scan,
            original: &scan,
            reorient: &reorient,
            recenter: &recenter,
            cap: &cap,
            simplify: &simplify,
            smoothing: &smoothing,
            centerline_trim: &trim,
            source_stl: &source,
            output_dir: &dir,
            stl_units: StlUnits::Mm,
            auto_center_offset_m: Vector3::zeros(),
            auto_pca_quat: None,
        };

        let outcome = save_cleaned_scan(&inputs).expect("save_cleaned_scan should succeed");

        // Identity transform + no trim + a clean closed solid → all 12 faces.
        assert_eq!(outcome.triangle_count, 12);
        let stl_path = dir.join("widget.cleaned.stl");
        let toml_path = dir.join("widget.prep.toml");
        assert!(stl_path.exists(), "cleaned STL written");
        assert!(toml_path.exists(), "prep.toml written");
        assert!(
            std::fs::metadata(&stl_path).expect("stl metadata").len() > 0,
            "cleaned STL non-empty",
        );
        let toml_str = std::fs::read_to_string(&toml_path).expect("read prep.toml");
        toml::from_str::<toml::Value>(&toml_str).expect("prep.toml parses");

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// `build_cleaned_mesh` bakes Reorient + Recenter into vertex
    /// positions: an identity Reorient + +x translation moves all
    /// vertices by +x. Default state means no caps are appended.
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
            &CapState::default(),
        );
        assert_eq!(cleaned.vertices.len(), 3);
        assert!(
            (cleaned.vertices[0].x - (0.001 + 0.1)).abs() < 1e-12,
            "first vertex should be translated by +0.1m, got {}",
            cleaned.vertices[0].x,
        );
    }

    // CSP.4c — `build_cleaned_mesh_applies_clip_when_enabled` +
    // `build_cleaned_mesh_skips_clip_when_disabled` removed
    // alongside `ClipState`. Centerline trim is now the single
    // user-facing mesh-cutting feature.

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
        // B arc — sister-anchor of
        // `auto_cap_open_boundaries_emits_outward_cap_normals`.
        // `build_cleaned_mesh`'s cap step has the same emission
        // logic as `auto_cap_open_boundaries`; both should produce
        // cap-face 3D cross-product normals aligned to the OUTWARD
        // `plane_normal`. The cap loop here has `plane_normal = +Z`
        // and `cap_loop.include = true`, with no other mesh faces
        // to bias `orient_cap_normal_outward`'s majority check (the
        // loop verts themselves sit on the plane → signed = 0 →
        // neither above/below counted, returns `plane_normal`
        // unchanged). So the post-fix emitted cap faces should have
        // 3D normals pointing +Z.
        for face in &cleaned.faces {
            let v0 = cleaned.vertices[face[0] as usize];
            let v1 = cleaned.vertices[face[1] as usize];
            let v2 = cleaned.vertices[face[2] as usize];
            let normal = (v1.coords - v0.coords).cross(&(v2.coords - v0.coords));
            assert!(
                normal.z > 0.0,
                "build_cleaned_mesh cap-face normal must point +Z (outward, \
                 matching plane_normal); face {face:?} produced normal {normal:?}",
            );
        }
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

        // Smoothing disabled — this test asserts the weld/degenerate
        // hygiene path, not the smoothing pass.
        let report = cleanup_cleaned_mesh_for_disk(&mut unshared, 0);
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

        let report = cleanup_cleaned_mesh_for_disk(&mut mesh, 0);
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
        let report = cleanup_cleaned_mesh_for_disk(&mut mesh, 0);
        assert_eq!(report.total(), 0, "clean mesh produced cleanup: {report:?}");
    }

    /// Smoothing with non-zero iterations counts toward
    /// [`CleanupReport::total()`] AND actually moves vertices.
    /// Pins the slider's `iterations > 0` path so a future
    /// refactor doesn't silently disconnect the slider from
    /// the smoothing call.
    #[test]
    fn cleanup_applies_taubin_smoothing_when_iterations_positive() {
        let mut mesh = grid_square(5);
        let original_verts = mesh.vertices.clone();
        let report = cleanup_cleaned_mesh_for_disk(&mut mesh, 5);
        assert_eq!(
            report.smoothing, 5,
            "smoothing iters not threaded: {report:?}"
        );
        // grid_square produces vertices ON a plane; Taubin should
        // not significantly displace them (Laplacian of a planar
        // mesh is ~zero), but some boundary verts will drift
        // toward the interior. At least one vertex should differ.
        let any_moved = mesh
            .vertices
            .iter()
            .enumerate()
            .any(|(i, v)| (v.coords - original_verts[i].coords).norm() > 1e-12);
        assert!(
            any_moved,
            "Taubin smoothing with 5 iters did not move any vertex on grid_square(5)",
        );
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
