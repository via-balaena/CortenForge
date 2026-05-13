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
use mesh_types::{Aabb, Bounded, IndexedMesh, Point3};
use meshopt::{SimplifyOptions, simplify_decoder};
use nalgebra::{UnitQuaternion, Vector3};

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
    /// `apply_recenter_to_transform` can write the result directly to
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

/// True-plane-intersection mesh clip against a horizontal-in-cast-frame
/// plane at world `z = clip_z_m` (`docs/SCAN_PREP_DESIGN.md` §Panel
/// specifications §5; spec §Architectural decisions §Clip cut style).
///
/// For each triangle:
/// - **All 3 vertices on/above** the plane → keep as-is.
/// - **All 3 below** → drop.
/// - **Mixed** → compute intersection points on the crossing edges +
///   triangulate the surviving polygon (3 or 4 vertices) via a fan
///   from the first vertex. The result is a clean planar cut along
///   `z = clip_z`, suitable for capping (boundary stays planar so
///   `mesh-repair`'s plane fit at commit #9 gets `R² ≈ 1.0`).
///
/// **Edge case from spec §Risks §"True plane intersection edge cases"**:
/// vertices exactly on the plane (`dist == 0`) are classified as
/// "above" so the surrounding triangle is kept; this avoids degenerate
/// sub-triangles in the cut.
///
/// Rejected alternative: vertex-drop (drop any triangle whose all 3
/// vertices are below the plane). Cheaper (~30 LOC) but produces
/// jagged ragged boundaries that lie off the clip plane → degrades
/// the Cap panel's auto-fit `R²`. Spec mandates true plane
/// intersection for this reason.
///
/// Output mesh's vertex buffer = original vertices + new intersection
/// vertices appended; `remove_unreferenced_vertices` strips the
/// dropped (all-below) original vertices afterwards so the result is
/// tight.
#[allow(dead_code)] // Used at commit #9 (Cap) and #12 (Save); shipped at #8 for the unit-tested algorithm.
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
fn rotated_aabb_physics_mm(
    rot_physics: UnitQuaternion<f64>,
    raw_aabb_m: &Aabb,
) -> (Vector3<f64>, Vector3<f64>) {
    let lo = raw_aabb_m.min;
    let hi = raw_aabb_m.max;
    let mut min_mm = Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max_mm = Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for &x in &[lo.x, hi.x] {
        for &y in &[lo.y, hi.y] {
            for &z in &[lo.z, hi.z] {
                let corner_mm = Vector3::new(x * 1000.0, y * 1000.0, z * 1000.0);
                let rotated = rot_physics.transform_vector(&corner_mm);
                min_mm.x = min_mm.x.min(rotated.x);
                min_mm.y = min_mm.y.min(rotated.y);
                min_mm.z = min_mm.z.min(rotated.z);
                max_mm.x = max_mm.x.max(rotated.x);
                max_mm.y = max_mm.y.max(rotated.y);
                max_mm.z = max_mm.z.max(rotated.z);
            }
        }
    }
    (min_mm, max_mm)
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
        .insert_resource(StatusBar::default())
        .insert_resource(overlays)
        .add_systems(Startup, (setup_render_scene, init_status_for_load))
        // Apply/Reset handler runs before auto-clear so a newly-set
        // status's TTL is checked against the same tick's `Time`.
        // `apply_reorient_to_transform` + `apply_recenter_to_transform`
        // are idempotent (no-op when the desired Bevy rotation /
        // translation already match the entity's current Transform)
        // so order vs. the others doesn't change steady-state
        // behavior. Both write to disjoint channels (rotation /
        // translation) of the same component, so they're conflict-free
        // beyond the Bevy scheduler's per-entity serialization.
        .add_systems(
            Update,
            (
                handle_simplify_actions,
                apply_reorient_to_transform,
                apply_recenter_to_transform,
                update_clip_drop_pct,
                auto_clear_status,
                draw_reference_overlays,
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

/// Update system: project the current `ReorientState` quaternion onto
/// the `ScanMeshEntity`'s Bevy `Transform` component every tick.
/// Per spec §6 implementation arc: "Live transforms via Bevy
/// `Transform` component, not mesh rebuild — slider drags update one
/// component value, no per-tick vertex-buffer copy."
///
/// Cheap by design: a single `Quat` derivation + per-entity equality
/// check + (only if different) one `Transform.rotation` write. The
/// equality check is the steady-state optimization — when the user
/// isn't touching the panel and the rotation hasn't changed, this
/// system is effectively a no-op.
///
/// Runs every Update tick regardless of `ReorientState::is_changed()`
/// because the `ScanMeshEntity` itself can be re-spawned by
/// `handle_simplify_actions` with an identity rotation; this system
/// then restores the user's reorient on the next tick automatically
/// without bespoke threading through the respawn path.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn apply_reorient_to_transform(
    state: Res<ReorientState>,
    mut entities: Query<&mut Transform, With<ScanMeshEntity>>,
) {
    let q_bevy = physics_quat_to_bevy_for_plus_z(state.quaternion_physics());
    for mut transform in &mut entities {
        if transform.rotation != q_bevy {
            transform.rotation = q_bevy;
        }
    }
}

/// Update system: project the current `RecenterState` translation onto
/// the `ScanMeshEntity`'s Bevy `Transform.translation` every tick.
/// Mirror of [`apply_reorient_to_transform`] for the translation
/// channel — same idempotent equality-check pattern; same automatic
/// re-application after `handle_simplify_actions` respawns the entity
/// with identity Transform.
///
/// Bevy's `Transform` composes as `world_pos = translation +
/// rotation * (local_pos * scale)`, so this system writing
/// `translation` while [`apply_reorient_to_transform`] writes
/// `rotation` is conflict-free — the two channels compose into the
/// final world transform without either overwriting the other.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn apply_recenter_to_transform(
    state: Res<RecenterState>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut entities: Query<&mut Transform, With<ScanMeshEntity>>,
) {
    let target = state.translation_world(*up, render_scale.0);
    for mut transform in &mut entities {
        if transform.translation != target {
            transform.translation = target;
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
    // + Recenter translation in the same `translation + rotation × ...`
    // order that Bevy uses for `ScanMeshEntity.Transform`, so the
    // wireframe stays glued to the mesh through both panels' edits.
    let rotation = physics_quat_to_bevy_for_plus_z(reorient.quaternion_physics());
    let recenter_world = recenter.translation_world(*up, render_scale.0);
    let wireframe_transform = Transform {
        translation: recenter_world + rotation * overlays.bbox_center_bevy,
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
    mut simplify_state: ResMut<SimplifyState>,
    mut reorient_state: ResMut<ReorientState>,
    mut recenter_state: ResMut<RecenterState>,
    mut clip_state: ResMut<ClipState>,
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
            render_reorient_section(ui, &mut reorient_state);
            render_recenter_section(ui, &mut recenter_state, &reorient_state, &overlays);
            render_clip_section(ui, &mut clip_state, &overlays);
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
/// snap buttons set known-good triples. `apply_reorient_to_transform`
/// consumes the state every Update tick and projects it onto the Bevy
/// Transform.
fn render_reorient_section(ui: &mut egui::Ui, state: &mut ReorientState) {
    egui::CollapsingHeader::new("Reorient")
        .default_open(true)
        .show(ui, |ui| {
            ui.add(egui::Slider::new(&mut state.roll_deg, -180.0..=180.0).text("roll (deg)"));
            ui.add(egui::Slider::new(&mut state.pitch_deg, -180.0..=180.0).text("pitch (deg)"));
            ui.add(egui::Slider::new(&mut state.yaw_deg, -180.0..=180.0).text("yaw (deg)"));

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
            if ui.button("Center origin").clicked() {
                let (min_mm, max_mm) =
                    rotated_aabb_physics_mm(reorient.quaternion_physics(), &overlays.raw_aabb_m);
                state.tx_mm = -0.5 * (min_mm.x + max_mm.x);
                state.ty_mm = -0.5 * (min_mm.y + max_mm.y);
                state.tz_mm = -0.5 * (min_mm.z + max_mm.z);
            }
            if ui.button("Floor -> z=0").clicked() {
                let (min_mm, _max_mm) =
                    rotated_aabb_physics_mm(reorient.quaternion_physics(), &overlays.raw_aabb_m);
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

    // ----- RecenterState + rotated_aabb_physics_mm -------------------

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

    /// Identity rotation: `rotated_aabb_physics_mm` returns the raw
    /// AABB scaled to mm with no axis permutation.
    #[test]
    fn rotated_aabb_identity_preserves_bounds_in_mm() {
        let raw = Aabb::from_corners(
            mesh_types::Point3::new(-0.05, -0.10, -0.15),
            mesh_types::Point3::new(0.05, 0.10, 0.15),
        );
        let (min_mm, max_mm) = rotated_aabb_physics_mm(UnitQuaternion::identity(), &raw);

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
        let (min_mm, max_mm) = rotated_aabb_physics_mm(rot, &raw);

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
}
