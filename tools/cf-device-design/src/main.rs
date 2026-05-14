//! cf-device-design — layered-silicone-device design + testing suite
//! (`docs/ENGINEERING_SUITE_DESIGN.md`).
//!
//! Slices shipped:
//! - Slice 2: crate scaffold + STL/prep load + centerline overlay +
//!   Scan Info panel.
//! - Slice 3: decimated scan proxy (`EnvelopeProxyMesh`) +
//!   per-vertex radial directions.
//! - Slice 4: Cavity panel — scan-derived inner void, dialed inward
//!   by a uniform inset.
//! - Slice 5: Layers panel — 1–6 concentric silicone layers, each
//!   with its own thickness slider + material + visibility toggle.
//!   Surfaces render as solid depth-tested Bevy meshes (cavity + one
//!   per layer); the outermost layer's outer surface IS the device's
//!   outer skin (no separate "outer envelope" concept).
//! - Slice 6: Validations panel — per-layer pour volume + mass
//!   graded against the 2 lb single-pour budget, cavity self-
//!   intersection check, minimum-castable-wall check. All derived,
//!   read-only; computed each frame by `compute_validations`.
//! - Slice 6.5: per-layer Slacker recipe — each layer picks a base
//!   silicone + a Slacker ratio from Smooth-On's TB curves (`mod
//!   slacker`), and the Layers panel reads back the effective Shore
//!   hardness, tack, and the mix in grams.
//!
//! In progress: Insertion Sim (FEM, slice 7) — `mod insertion_sim`
//! holds the Route-A SDF bridge: 7.0 the measurement spike, 7.1 the
//! geometry + per-layer Yeoh material builder
//! (`build_insertion_geometry`), 7.2 the single static FEM solve
//! (`run_single_insertion_step`). Quasi-static ramp + UI: 7.3+.
//!
//! Pending slices: Insertion Sim solve + UI (slice 7.2+) / Save /
//! Open (slice 8). `docs/ENGINEERING_SUITE_DESIGN.md` predates the
//! build and is stale — trust the code.

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
use mesh_repair::{remove_unreferenced_vertices, weld_vertices};
use mesh_types::{Bounded, IndexedMesh, Point3};
use meshopt::simplify_sloppy_decoder;
use nalgebra::Vector3;
use serde::Deserialize;

/// Slice 7 insertion-sim pipeline — Route-A SDF bridge.
///
/// `#[cfg(test)]`-gated through sub-commits 7.0–7.3: 7.0 seeded the
/// SDF-bridge spike, 7.1 the geometry + per-layer Yeoh material
/// builder, 7.2–7.3 the solve. Nothing in the shipping binary calls
/// it yet — it is exercised by `--ignored` integration tests. The
/// gate comes off at 7.4, when the Insertion Sim panel wires it in.
/// See the module docs.
#[cfg(test)]
mod insertion_sim;

/// Cast-frame demolding-axis convention: `+Z` is up. Inherited from
/// cf-scan-prep + cf-cast — every CortenForge cast tool assumes the
/// `UpAxis::PlusZ` swap from physics frame to Bevy frame.
const DEVICE_UP_AXIS: UpAxis = UpAxis::PlusZ;

#[derive(Parser, Debug)]
#[command(
    name = "cf-device-design",
    about = "Layered-silicone-device design + testing suite (composes a cleaned scan into a device design: cavity + concentric silicone layers)",
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
    /// STL. The `[centerline].points_m` block drives the per-vertex
    /// radial split that shapes the cavity + layer surfaces (see
    /// [`EnvelopeProxyMesh`]); absent, the centerline overlay doesn't
    /// render and the radials fall back to Z-stripped per-vertex
    /// normals.
    #[arg(long, value_name = "PATH")]
    prep_toml: Option<PathBuf>,
}

/// Bevy resource carrying the loaded cleaned scan in physics-frame
/// meters. Mirror of cf-scan-prep's `ScanMesh` (same posture).
#[derive(Resource)]
struct ScanMesh(IndexedMesh);

/// Whether the scan mesh entity is visible this frame. Toggled by
/// the "Show scan mesh" checkbox in the Scan Info panel. Useful
/// when inspecting the cavity mesh, which sits INSIDE the scan and
/// is occluded by the scan mesh when both are drawn.
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
struct ScanMeshVisible(bool);

impl Default for ScanMeshVisible {
    fn default() -> Self {
        Self(true)
    }
}

/// Bevy resource carrying scan-info readouts surfaced in the Scan
/// Info panel. Computed once at startup; immutable post-construction.
#[derive(Resource, Debug, Clone)]
struct ScanInfo {
    file_label: String,
    vertex_count: usize,
    face_count: usize,
    /// Raw AABB extents in millimeters (workshop convention).
    aabb_mm_extents: [f64; 3],
    /// Bbox diagonal in meters — shown in the Scan Info panel + the
    /// startup log as a quick size sanity check.
    bbox_diagonal_m: f64,
    /// Centerline polyline length in physics meters (sum of
    /// segment distances). Zero if the centerline is absent or
    /// empty. Shown in the Scan Info panel.
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

/// Cavity panel state — the user-dialed `inset_m` by which the
/// cavity surface sits INSIDE the scan surface. Casting context:
/// the cavity is the void the appendage slides into; smaller than
/// the scan so the silicone "skin" between cavity and scan
/// stretches over the appendage, providing the snug fit. Stretch
/// strain ≈ `inset_m / local_scan_radius` — too little = sloppy
/// fit, too much = silicone tears.
///
/// Slice 4 v1: a single uniform inset along the entire scan proxy.
/// A later slice may add the insertable-length clip
/// (`docs/ENGINEERING_SUITE_DESIGN.md` § 4) so the cavity covers
/// only the inserted portion, with a tangent-perpendicular end
/// cap.
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
struct CavityState {
    /// Distance (meters) by which the cavity surface sits inside
    /// the scan surface, measured along the per-vertex radial
    /// direction ([`EnvelopeProxyMesh::vertex_radial_directions`]).
    /// The cavity mesh displaces each proxy vertex by `-radial *
    /// inset_m`. The radial is horizontal for below-/alongside-
    /// centerline vertices and full-3D for the dome — see the
    /// field docs on [`EnvelopeProxyMesh`].
    inset_m: f64,
    /// Whether the cavity mesh entity is drawn this frame. User
    /// toggle in the Cavity panel.
    visible: bool,
}

impl CavityState {
    /// Default state. Inset defaults to
    /// [`CAVITY_DEFAULT_INSET_M`] (3 mm) — the minimum-acceptable
    /// buildable design: above Ecoflex's ~2 mm castability
    /// threshold, and ~10 % radial pre-strain on a typical scan
    /// cross-section. User dials UP for more pre-strain or DOWN to
    /// experiment.
    fn default_for_scan() -> Self {
        Self {
            inset_m: CAVITY_DEFAULT_INSET_M,
            visible: true,
        }
    }

    /// Slider range for the cavity inset, in meters.
    fn inset_slider_range_m() -> (f64, f64) {
        (0.0, 0.015)
    }
}

/// Silicone-material catalog the Layers panel offers. Each entry is
/// `(anchor_key, display_label, density_kg_m3)`.
///
/// The `anchor_key` strings mirror cf-cast's `cure::lookup` keys;
/// the densities mirror `sim-soft`'s `silicone_table.rs` anchor
/// values (Ecoflex 00-10 = 1040, the rest of the Ecoflex line +
/// Dragon Skin 10A/15 = 1070, Dragon Skin 20A/30A = 1080 kg/m³).
/// Both are mirrored by-name, not by-import: cf-device-design
/// carries the catalog standalone (no cf-cast / sim-soft dep) so
/// this tool stays composable. cf-cast-cli re-validates the chosen
/// anchor + resolves the runtime density at `.design.toml` ingest.
const LAYER_MATERIALS: &[(&str, &str, f64)] = &[
    ("ECOFLEX_00_10", "Ecoflex 00-10 (super-soft)", 1040.0),
    ("ECOFLEX_00_20", "Ecoflex 00-20 (soft)", 1070.0),
    ("ECOFLEX_00_30", "Ecoflex 00-30 (medium-soft)", 1070.0),
    ("ECOFLEX_00_50", "Ecoflex 00-50 (firm-soft)", 1070.0),
    ("DRAGON_SKIN_10A", "Dragon Skin 10A (soft)", 1070.0),
    ("DRAGON_SKIN_15", "Dragon Skin 15 (medium)", 1070.0),
    ("DRAGON_SKIN_20A", "Dragon Skin 20A (firm)", 1080.0),
    ("DRAGON_SKIN_30A", "Dragon Skin 30A (firmest)", 1080.0),
];

/// Bulk density (kg/m³) for a layer's `material_anchor_key`, looked
/// up in [`LAYER_MATERIALS`]. Falls back to 1070 kg/m³ (the Ecoflex/
/// Dragon-Skin line median) for an unrecognized key — defensive
/// only; every key the Layers panel can set comes from the catalog.
fn material_density(anchor_key: &str) -> f64 {
    LAYER_MATERIALS
        .iter()
        .find(|(key, _, _)| *key == anchor_key)
        .map_or(1070.0, |(_, _, density)| *density)
}

/// Maximum number of concentric silicone layers in the device wall.
/// 6 is a generous workshop cap; real designs typically use 1–3.
/// Setting a finite cap keeps panel scroll predictable + the
/// per-frame draw cost bounded.
const LAYER_COUNT_MAX: usize = 6;

/// One concentric silicone layer in the device wall. Layers are
/// ordered innermost-first: `layers[0]`'s inner surface = the
/// cavity surface, `layers[i]`'s outer surface = `layers[i+1]`'s
/// inner surface, `layers[N-1]`'s outer surface = the device's
/// outer skin.
///
/// Every layer carries its own user-dialed `thickness_m` (slice 5
/// polish 3 pivot — there is no derived "last layer"). The device
/// wall total is the sum of layer thicknesses; the outer skin sits
/// at `sum(thickness) - cavity.inset_m` radially from the scan.
#[derive(Debug, Clone, Copy, PartialEq)]
struct LayerSpec {
    /// Radial thickness (meters), user-dialed via a panel slider.
    /// For layer 0 this is the distance from the cavity surface to
    /// layer 0's outer surface; for layer `i > 0` it's the Δ from
    /// the prior layer's outer surface.
    thickness_m: f64,
    /// Smooth-On product anchor key (from [`LAYER_MATERIALS`]).
    /// `'static` lifetime because the entries are compile-time
    /// constants. This is the layer's *base* silicone — Slacker (if
    /// any) is dialed separately via `slacker_fraction`.
    material_anchor_key: &'static str,
    /// Slacker mass as a fraction of the base silicone's combined
    /// Part A + Part B mass — `0.0` is the base with no Slacker. The
    /// Layers panel only offers the discrete fractions on the base
    /// material's Smooth-On TB curve ([`slacker::support`]); a
    /// base-material change that orphans the stored value snaps it
    /// back to `0.0` via [`resolve_slacker_fraction`].
    slacker_fraction: f64,
    /// Whether this layer's outer-surface mesh is drawn this frame.
    /// Per-layer visibility (slice 5 polish 5) — each layer toggles
    /// independently so the user can isolate any single layer for
    /// inspection.
    visible: bool,
}

impl LayerSpec {
    /// Slider range for per-layer thickness (m). 1 mm minimum
    /// matches FDM-printable mold-wall minimum; 20 mm upper bound
    /// is generous for any single layer.
    fn thickness_slider_range_m() -> (f64, f64) {
        (0.001, 0.020)
    }
}

/// Bevy resource carrying the ordered layer stack. Default state
/// is a single Ecoflex 00-30 layer — the simplest buildable
/// configuration. The user adds layers + dials each thickness via
/// the Layers panel; the device wall total is the sum of layer
/// thicknesses.
///
/// Invariant: `1 <= layers.len() <= LAYER_COUNT_MAX`. The panel's
/// add/remove controls preserve this (the "Remove layer" button is
/// hidden when only one layer remains; "+ Add layer" is hidden at
/// the cap).
#[derive(Resource, Debug, Clone, PartialEq)]
struct LayersState {
    layers: Vec<LayerSpec>,
}

impl LayersState {
    /// Default state: single Ecoflex 00-30 layer. The user adds
    /// more layers via the panel as needed.
    fn default_for_scan() -> Self {
        Self {
            layers: vec![LayerSpec {
                thickness_m: 0.005,
                material_anchor_key: "ECOFLEX_00_30",
                slacker_fraction: 0.0,
                visible: true,
            }],
        }
    }
}

/// Decimated proxy of the cleaned scan, shared by every surface
/// mesh (cavity + per-layer outer surfaces). Each surface entity's
/// vertex positions are the proxy's positions displaced by a
/// signed offset along the per-vertex radial direction:
///
/// - cavity: `-cavity.inset_m`
/// - layer i outer surface: `sum(layer.thickness[0..=i]) - cavity.inset_m`
///
/// Decimated to ~`ENVELOPE_PROXY_TARGET_FACES` (1500 faces) at
/// startup — the iter-1 fixture's 3.3M-face cleaned mesh would be
/// impossible to re-mesh on every slider tick at full resolution.
#[derive(Resource, Debug, Clone)]
struct EnvelopeProxyMesh {
    /// Decimated scan vertices in physics-frame meters.
    vertices: Vec<Point3<f64>>,
    /// Per-vertex unit displacement direction. Used by both the
    /// cavity (`-radial * inset`) and per-layer (`+radial * offset`)
    /// surface meshes. Split by Z-region (computed in
    /// [`compute_envelope_proxy_mesh`] step 5):
    ///
    /// - Vertices BELOW the centerline's Z-range (cleaned-cap rim +
    ///   the bottom gap): HORIZONTAL-only radial — the Z component
    ///   of `(v - nearest_centerline_pt)` is zeroed. Keeps the
    ///   bottom plane pinned at its Z level under any displacement.
    /// - Vertices WITHIN / ABOVE the centerline's Z-range (side
    ///   walls + dome): FULL 3D radial — `(v - nearest_centerline_pt)`
    ///   normalized. Side walls get a ≈-horizontal direction
    ///   naturally; the dome gets a vertical component so it grows
    ///   outward AND upward as layers thicken.
    ///
    /// Degenerate fallback (vertex coincident with the centerline
    /// point): per-vertex normal, Z-stripped for below-centerline
    /// vertices; finally `(1, 0, 0)` if even that is degenerate.
    vertex_radial_directions: Vec<Vector3<f64>>,
    /// Triangle face indices (post-decimation). Each face is three
    /// `u32` indices into `vertices`. The cavity + per-layer
    /// surface renderers build Bevy `Mesh` assets from the
    /// displaced vertex positions + this connectivity.
    faces: Vec<[u32; 3]>,
    /// Original face count (pre-decimation) — surfaced in the panel
    /// so the user knows the proxy is approximate.
    original_face_count: usize,
    /// Minimum radial distance (meters) from any non-degenerate
    /// proxy vertex to its nearest centerline point — i.e. the
    /// smallest `‖v − nearest_centerline_pt‖` over the vertices,
    /// measured in the same Z-region-split metric as the radial
    /// directions (horizontal-only below the centerline, full-3D
    /// within/above).
    ///
    /// This is the cavity-collapse threshold: displacing the cavity
    /// inward by `inset_m` moves each vertex `inset_m` along its
    /// radial, so once `inset_m ≥ min_radial_distance_m` at least
    /// one vertex has reached / crossed the centerline and the
    /// cavity mesh self-intersects. The Validations panel (slice 6)
    /// reads this for the cavity self-intersection check.
    ///
    /// `f64::INFINITY` when no centerline is available (< 2 points)
    /// — without a centerline there is no radial-collapse metric, so
    /// the check is reported as not-applicable.
    min_radial_distance_m: f64,
}

/// Target face count after decimation for the shared scan proxy.
/// 1500 faces is dense enough to show the scan's gross shape
/// (cylinder + dome + cleaned cap), sparse enough that the cavity
/// and per-layer surface meshes can be regenerated on every slider
/// tick without a frame hitch.
const ENVELOPE_PROXY_TARGET_FACES: usize = 1500;

/// Weld epsilon (meters) for the pre-decimation vertex weld.
/// Matches cf-scan-prep's `SIMPLIFY_WELD_EPSILON_M = 1e-6` — the
/// cleaned scan's STL load produces 3-per-triangle unshared
/// vertices that meshopt needs welded to find collapsible edges.
const ENVELOPE_PROXY_WELD_EPSILON_M: f64 = 1e-6;

/// `meshopt::simplify_sloppy_decoder`'s target-error parameter.
/// Effectively unbounded (1000 % of mesh diagonal) — we want the
/// decimation to ALWAYS reach the target face count, sacrificing
/// geometric fidelity if necessary. simplify_sloppy is topology-
/// non-preserving and per its docs "always able to reach target
/// triangle count," so this error cap exists only as a defensive
/// upper bound; under normal operation the face-count target is
/// the binding constraint.
///
/// Switched from `simplify_decoder` (slice 4 polish) to
/// `simplify_sloppy_decoder` (slice 5 perf pass) after observing
/// that the iter-1 cleaned scan retains its full 3.34 M face count
/// through topology-preserving simplification — the scan has
/// disconnected components / degenerate triangles that block the
/// regular collapse algorithm. The sloppy variant ignores topology
/// and reliably hits ~1500 faces.
const ENVELOPE_PROXY_SIMPLIFY_TARGET_ERROR: f32 = 10.0;

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

/// Default cavity inset (meters). 3 mm = the minimum-acceptable
/// starting point for a buildable silicone device:
/// - Above Ecoflex 00-30's ~2 mm castability threshold (thinner
///   walls tear under stretch).
/// - Yields ~10 % radial pre-strain on a typical 25–35 mm-diameter
///   scan cross-section — meaningful snug fit, well within
///   elongation-at-break (~900 %).
///
/// The user dials UP for more pre-strain or DOWN to experiment
/// (down to 0 = cavity-coincident-with-scan, useful as a debug
/// reference but not a buildable design).
const CAVITY_DEFAULT_INSET_M: f64 = 0.003;

/// Decimate the cleaned scan to a ~`ENVELOPE_PROXY_TARGET_FACES`-
/// face proxy mesh + compute the per-vertex radial directions.
/// Returns the proxy struct shared by the cavity + per-layer
/// mesh-update systems for per-frame mesh regeneration.
///
/// Pipeline:
/// 1. Weld duplicated vertices (STL load produces 3-per-triangle
///    unshared vertices that meshopt can't decimate without
///    shared topology).
/// 2. `meshopt::simplify_sloppy_decoder` — topology-non-preserving
///    decimation that ALWAYS reaches the target face count.
///    Required for the iter-1 cleaned scan, which retains its full
///    3.34 M face count through topology-preserving simplification
///    (disconnected components + degenerate triangles block the
///    regular collapse algorithm).
/// 3. Strip unreferenced vertices left over from the decimation.
/// 4. Compute face normals from triangle CCW order, then per-vertex
///    normals as area-weighted averages of incident face normals
///    (used only as the degenerate fallback for radial directions).
/// 5. Compute per-vertex radial directions, split by Z-region (see
///    the `vertex_radial_directions` field docs).
///
/// Cost: O(N_orig_faces) for weld + meshopt + O(N_decimated) for
/// normals + radials. For the iter-1 fixture (3.3 M cleaned faces →
/// 1500 proxy faces) this runs in ~5–10 s at startup; one-time.
fn compute_envelope_proxy_mesh(
    scan_mesh: &IndexedMesh,
    centerline: &[Point3<f64>],
) -> EnvelopeProxyMesh {
    let original_face_count = scan_mesh.faces.len();

    // Step 1: weld unshared vertices.
    let mut welded = scan_mesh.clone();
    weld_vertices(&mut welded, ENVELOPE_PROXY_WELD_EPSILON_M);

    // Step 2: meshopt simplify. Target index count = target faces × 3.
    // f64 → f32 cast is intentional; meshopt's C API operates on f32.
    #[allow(clippy::cast_possible_truncation)]
    let positions: Vec<[f32; 3]> = welded
        .vertices
        .iter()
        .map(|p| [p.x as f32, p.y as f32, p.z as f32])
        .collect();
    let indices: Vec<u32> = welded.faces.iter().flatten().copied().collect();
    let target_index_count = ENVELOPE_PROXY_TARGET_FACES.saturating_mul(3);
    let mut result_error = 0.0_f32;
    // `simplify_sloppy` ignores topology — required for the iter-1
    // cleaned scan (3.34 M faces with disconnected components +
    // degenerate triangles that block topology-preserving collapse).
    // Guaranteed to reach the target face count per the meshopt
    // docs ("always able to reach target triangle count").
    let simplified_indices = if target_index_count < indices.len() {
        simplify_sloppy_decoder(
            &indices,
            &positions,
            target_index_count,
            ENVELOPE_PROXY_SIMPLIFY_TARGET_ERROR,
            Some(&mut result_error),
        )
    } else {
        // Mesh is already at or below the target; skip decimation.
        indices.clone()
    };

    // Step 3: reassemble + strip unreferenced vertices.
    let mut proxy = welded;
    proxy.faces = simplified_indices
        .chunks_exact(3)
        .map(|tri| [tri[0], tri[1], tri[2]])
        .collect();
    remove_unreferenced_vertices(&mut proxy);

    // Step 4: face normals (CCW triangle winding × right-hand rule
    // = outward for cf-scan-prep's cleaned-mesh convention).
    let face_normals: Vec<Vector3<f64>> = proxy
        .faces
        .iter()
        .map(|face| {
            let v0 = proxy.vertices[face[0] as usize].coords;
            let v1 = proxy.vertices[face[1] as usize].coords;
            let v2 = proxy.vertices[face[2] as usize].coords;
            let e1 = v1 - v0;
            let e2 = v2 - v0;
            // Un-normalized so the magnitude carries the face area
            // weight (= 2 × triangle area) into the vertex-normal
            // accumulation. Area-weighting biases the per-vertex
            // normal toward bigger neighboring triangles, which
            // gives a more physically meaningful "outward" at
            // vertices on a curved surface.
            e1.cross(&e2)
        })
        .collect();

    // Per-vertex normals: sum face normals (which carry area
    // weight via their pre-normalized magnitudes), then normalize.
    let mut vertex_normals: Vec<Vector3<f64>> = vec![Vector3::zeros(); proxy.vertices.len()];
    for (face, fn_vec) in proxy.faces.iter().zip(face_normals.iter()) {
        for &idx in face {
            vertex_normals[idx as usize] += fn_vec;
        }
    }
    // Normalize; fall back to +Z for any vertex with no incident
    // faces (defensive — should not happen after remove_unrefed).
    for n in &mut vertex_normals {
        let norm = n.norm();
        if norm > 1e-12 {
            *n /= norm;
        } else {
            *n = Vector3::new(0.0, 0.0, 1.0);
        }
    }

    // Step 5: per-vertex radial directions. Split by Z-region so
    // the bottom rim stays pinned while the dome grows in 3D:
    //
    // - Vertices BELOW the centerline's Z-range (the cleaned-cap
    //   rim + the bottom gap where the centerline doesn't reach):
    //   HORIZONTAL-only radial = `(vertex - nearest_centerline_pt)`
    //   with Z zeroed. Keeps the bottom plane fixed at its Z level;
    //   displacement grows the rim laterally, never axially.
    //
    // - Vertices WITHIN / ABOVE the centerline's Z-range (side
    //   walls + dome): FULL 3D radial = `(vertex -
    //   nearest_centerline_pt)` normalized. For side walls this is
    //   naturally ≈ horizontal (vertex is beside the centerline);
    //   for the dome it has a vertical component so the dome grows
    //   outward AND upward when layers thicken.
    //
    // The split point is `centerline_min_z` — below it the
    // centerline doesn't run alongside the geometry, so a 3D radial
    // would tilt downward (the centerline endpoint is ABOVE the rim
    // vertex) and drag the rim down. There's no visible
    // discontinuity at the threshold because the 3D radial just
    // above `centerline_min_z` is itself ≈ horizontal.
    // Each closure returns `(radial_direction, radial_distance)`:
    // the unit displacement direction plus the distance from the
    // vertex to its nearest centerline point in the same metric.
    // Degenerate vertices carry `f64::INFINITY` distance so they do
    // not drag the cavity-collapse minimum to zero.
    let centerline_min_z = centerline.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
    let radial_pairs: Vec<(Vector3<f64>, f64)> = if centerline.len() >= 2 {
        proxy
            .vertices
            .iter()
            .zip(vertex_normals.iter())
            .map(|(v, n)| {
                let nearest_i = nearest_centerline_index(v, centerline);
                let diff = v.coords - centerline[nearest_i].coords;
                let below_centerline = v.z < centerline_min_z;
                let raw = if below_centerline {
                    // Bottom rim / gap region: horizontal only.
                    Vector3::new(diff.x, diff.y, 0.0)
                } else {
                    // Side wall + dome: full 3D radial.
                    diff
                };
                let len = raw.norm();
                if len > 1e-9 {
                    (raw / len, len)
                } else {
                    // Degenerate (vertex coincident with the
                    // centerline point). Fall back to the per-vertex
                    // normal, Z-stripped for below-centerline
                    // vertices (keep them axially fixed), full 3D
                    // otherwise. Distance is INFINITY — a coincident
                    // vertex has no meaningful radial collapse depth.
                    let fallback = if below_centerline {
                        Vector3::new(n.x, n.y, 0.0)
                    } else {
                        *n
                    };
                    let len_f = fallback.norm();
                    let dir = if len_f > 1e-9 {
                        fallback / len_f
                    } else {
                        Vector3::new(1.0, 0.0, 0.0)
                    };
                    (dir, f64::INFINITY)
                }
            })
            .collect()
    } else {
        // No centerline available; use per-vertex normals (with Z
        // stripped) as the radial proxy. Consistent posture with the
        // centerline-present path's below-centerline branch. Without
        // a centerline there is no radial-collapse metric, so every
        // distance is INFINITY (cavity self-intersection check is
        // reported not-applicable downstream).
        vertex_normals
            .iter()
            .map(|n| {
                let n_xy = Vector3::new(n.x, n.y, 0.0);
                let len = n_xy.norm();
                let dir = if len > 1e-9 {
                    n_xy / len
                } else {
                    Vector3::new(1.0, 0.0, 0.0)
                };
                (dir, f64::INFINITY)
            })
            .collect()
    };

    let min_radial_distance_m = radial_pairs
        .iter()
        .map(|(_, d)| *d)
        .fold(f64::INFINITY, f64::min);
    let vertex_radial_directions = radial_pairs.into_iter().map(|(dir, _)| dir).collect();

    EnvelopeProxyMesh {
        vertices: proxy.vertices,
        vertex_radial_directions,
        faces: proxy.faces,
        original_face_count,
        min_radial_distance_m,
    }
}

/// Index of the centerline polyline point closest to `v` by
/// Euclidean distance. Returns 0 for an empty centerline.
fn nearest_centerline_index(v: &Point3<f64>, centerline: &[Point3<f64>]) -> usize {
    let mut best_i = 0;
    let mut best_dist_sq = f64::INFINITY;
    for (i, p) in centerline.iter().enumerate() {
        let d = (v.coords - p.coords).norm_squared();
        if d < best_dist_sq {
            best_dist_sq = d;
            best_i = i;
        }
    }
    best_i
}

// ============================================================
// Geometric validations (slice 6)
// ============================================================
//
// Cheap, derived-from-state checks surfaced in the Validations
// panel: per-layer pour volume + mass against the 2 lb single-pour
// budget, cavity self-intersection, and minimum castable wall
// thickness. Computed fresh each frame from `EnvelopeProxyMesh` +
// `CavityState` + `LayersState` — `compute_validations` is a pure
// function so the whole check set is unit-testable without Bevy.
//
// Pour volume uses the divergence-theorem signed mesh volume on the
// displaced proxy (not cf-cast's SDF voxel integration): the
// suite's geometry model is displaced proxy meshes, it carries no
// `cf_design::Solid` representation, and the closed-form mesh
// volume is both dependency-free and more accurate than coarse-
// voxel counting.

/// Per-silicone single-pour mass budget (kg) for the layered-
/// silicone-device v1 cast — 2 lb via NIST's exact pound-to-kg
/// conversion (1 lb = 0.453_592_37 kg). Mirrors cf-cast's
/// `pour_volume::DEFAULT_MASS_BUDGET_KG` by-value (no cf-cast dep,
/// same standalone-catalog posture as [`LAYER_MATERIALS`]).
const MASS_BUDGET_KG: f64 = 0.907_184_74;

/// Fraction of [`MASS_BUDGET_KG`] below which a single pour grades
/// green; between this fraction and 1.0× the budget grades yellow;
/// above 1.0× grades red. 0.8 reserves a 20 % working margin before
/// the budget becomes a hard concern.
const BUDGET_GREEN_FRACTION: f64 = 0.8;

/// Minimum castable layer thickness (meters). Below ~2 mm a poured
/// silicone wall is fragile and tears under insertion stretch (cf.
/// Ecoflex 00-30's ~2 mm castability threshold, cited in
/// [`CAVITY_DEFAULT_INSET_M`]). The per-layer thickness slider
/// floor is 1 mm — so sub-2 mm layers are reachable, and the
/// min-wall check flags them.
const MIN_CASTABLE_THICKNESS_M: f64 = 0.002;

/// Mass-budget severity grade for one cast pour (or the device
/// total), graded against [`MASS_BUDGET_KG`]. Variant declaration
/// order is the severity order — `Ord` is derived so the worst
/// per-layer status is `layers.iter().map(...).max()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum BudgetStatus {
    /// Comfortably under budget (< [`BUDGET_GREEN_FRACTION`]× the
    /// budget).
    Green,
    /// Approaching the budget ([`BUDGET_GREEN_FRACTION`]×..=1.0× the
    /// budget).
    Yellow,
    /// Over budget (> 1.0× the budget) — not pourable in one pour.
    Red,
}

/// Grade a single pour mass (kg) against [`MASS_BUDGET_KG`].
fn grade_budget(mass_kg: f64) -> BudgetStatus {
    if mass_kg > MASS_BUDGET_KG {
        BudgetStatus::Red
    } else if mass_kg >= MASS_BUDGET_KG * BUDGET_GREEN_FRACTION {
        BudgetStatus::Yellow
    } else {
        BudgetStatus::Green
    }
}

/// Per-layer validation readout — one entry per cast layer.
#[derive(Debug, Clone, PartialEq)]
struct LayerValidation {
    /// Index into [`LayersState::layers`], innermost-first.
    layer_index: usize,
    /// Shell volume (m³) — the new silicone this layer's pour adds,
    /// = V(this layer's outer surface) − V(its inner surface). The
    /// inner surface is the cavity surface for layer 0, or the
    /// prior layer's outer surface for layer `i > 0`.
    shell_volume_m3: f64,
    /// Pour mass (kg) = `shell_volume_m3` × the layer material's
    /// bulk density (from [`material_density`]).
    pour_mass_kg: f64,
    /// Mass-budget grade for this single pour.
    status: BudgetStatus,
    /// Whether this layer's thickness is at or above
    /// [`MIN_CASTABLE_THICKNESS_M`].
    thickness_castable: bool,
}

/// Whole-device validation readout, computed fresh each frame by
/// [`compute_validations`] from the proxy mesh + cavity + layer
/// state.
#[derive(Debug, Clone, PartialEq)]
struct DeviceValidations {
    /// One entry per layer, innermost-first.
    layers: Vec<LayerValidation>,
    /// Sum of every layer's pour mass (kg). Informational: same-
    /// material layers may be combined into one pour or each poured
    /// separately — the per-layer [`LayerValidation::status`] is the
    /// pour-by-pour budget constraint, this total is context.
    total_mass_kg: f64,
    /// Worst per-layer budget status — drives the panel's headline
    /// color. [`BudgetStatus::Green`] for an empty layer stack.
    worst_status: BudgetStatus,
    /// Whether the cavity mesh self-intersects at the current inset:
    /// `true` when `cavity.inset_m ≥ proxy.min_radial_distance_m`
    /// (at least one vertex has reached / crossed the centerline).
    /// Always `false` when no centerline is available — the check
    /// is reported not-applicable in that case.
    cavity_self_intersects: bool,
    /// The inset (m) at which the cavity first collapses onto the
    /// centerline = `proxy.min_radial_distance_m`. `f64::INFINITY`
    /// when no centerline is available (check not applicable).
    cavity_collapse_inset_m: f64,
    /// `true` when every layer is at or above
    /// [`MIN_CASTABLE_THICKNESS_M`].
    min_wall_ok: bool,
}

/// Enclosed volume (m³) of the proxy mesh with every vertex
/// displaced by `offset_m` along its per-vertex radial direction
/// (negative `offset_m` = inward, e.g. the cavity surface).
///
/// Divergence-theorem signed volume: `⅙·Σ_faces a·(b×c)` over the
/// displaced triangles, where `a, b, c` are the face's displaced
/// vertices. cf-scan-prep's cleaned mesh winds CCW / outward, so a
/// closed mesh yields a positive volume that grows monotonically
/// with `offset_m`. For the slightly-open cleaned scan the small
/// open-boundary error is common to every offset and largely
/// cancels in the shell differences [`compute_validations`] takes.
///
/// Pure geometry in physics-frame meters — no render-scale / UpAxis
/// mapping (those are display-only concerns).
fn signed_volume_m3(proxy: &EnvelopeProxyMesh, offset_m: f64) -> f64 {
    let displaced = |i: u32| -> Vector3<f64> {
        let idx = i as usize;
        proxy.vertices[idx].coords + proxy.vertex_radial_directions[idx] * offset_m
    };
    let mut six_volume = 0.0_f64;
    for face in &proxy.faces {
        let a = displaced(face[0]);
        let b = displaced(face[1]);
        let c = displaced(face[2]);
        six_volume += a.dot(&b.cross(&c));
    }
    six_volume / 6.0
}

/// Compute the whole-device validation readout from the proxy mesh
/// plus the current cavity + layer state. Pure function — the
/// Validations panel calls this each frame; the tests call it
/// directly.
///
/// Shell volumes: layer 0's inner surface is the cavity surface (at
/// offset `−inset_m`); layer `i > 0`'s inner surface is layer
/// `i − 1`'s outer surface. Layer `i`'s outer surface is at offset
/// `cumulative_thickness − inset_m`. A pour's shell volume is
/// `V(outer) − V(inner)` — the new material that pour adds.
fn compute_validations(
    proxy: &EnvelopeProxyMesh,
    cavity: &CavityState,
    layers: &LayersState,
) -> DeviceValidations {
    // The cavity surface is layer 0's inner surface.
    let mut prev_inner_volume = signed_volume_m3(proxy, -cavity.inset_m);
    let mut cumulative_thickness_m = 0.0_f64;
    let mut layer_validations = Vec::with_capacity(layers.layers.len());
    let mut total_mass_kg = 0.0_f64;
    let mut worst_status = BudgetStatus::Green;
    let mut min_wall_ok = true;

    for (layer_index, layer) in layers.layers.iter().enumerate() {
        cumulative_thickness_m += layer.thickness_m;
        let outer_offset_m = cumulative_thickness_m - cavity.inset_m;
        let outer_volume = signed_volume_m3(proxy, outer_offset_m);
        // The outer surface is a strictly larger radial offset than
        // the inner surface, so it always encloses more volume; the
        // `abs` is a guard against floating-point sign noise on the
        // near-closed cleaned scan, not a real sign ambiguity.
        let shell_volume_m3 = (outer_volume - prev_inner_volume).abs();
        prev_inner_volume = outer_volume;

        let pour_mass_kg = shell_volume_m3 * material_density(layer.material_anchor_key);
        total_mass_kg += pour_mass_kg;
        let status = grade_budget(pour_mass_kg);
        worst_status = worst_status.max(status);
        let thickness_castable = layer.thickness_m >= MIN_CASTABLE_THICKNESS_M;
        if !thickness_castable {
            min_wall_ok = false;
        }

        layer_validations.push(LayerValidation {
            layer_index,
            shell_volume_m3,
            pour_mass_kg,
            status,
            thickness_castable,
        });
    }

    DeviceValidations {
        layers: layer_validations,
        total_mass_kg,
        worst_status,
        cavity_self_intersects: cavity.inset_m >= proxy.min_radial_distance_m,
        cavity_collapse_inset_m: proxy.min_radial_distance_m,
        min_wall_ok,
    }
}

mod slacker {
    //! Slacker recipe data — Smooth-On Slacker™ silicone-softening
    //! tables, transcribed verbatim from the Slacker Tactile Mutator
    //! Technical Bulletin (rev 011524DH).
    //!
    //! Slacker is a clear fluid pre-mixed into a platinum-cure
    //! silicone's Part B (then Part A is added) to soften the cured
    //! rubber and add surface tack. The per-layer recipe feature
    //! lets the user dial a base silicone + a Slacker ratio and read
    //! back the effective cured hardness — so the bench moment is
    //! "read the recipe off the panel" instead of deriving the ratio
    //! on the spot.
    //!
    //! This is the data layer only; the recipe-panel UI (a later
    //! commit) is the consumer. Of the eight catalog silicones only
    //! four have published Slacker data — the rest carry an honest
    //! [`Support::NotRecommended`] / [`Support::NoData`] marker
    //! rather than a guessed curve.
    //!
    //! Mix-ratio convention: the TB tabulates "X parts Slacker per
    //! 100A + 100B" of base silicone, i.e. per 200 parts base — so a
    //! "+50 Slacker" row is a [`Point::slacker_fraction`] of
    //! 50 / 200 = 0.25.
    //!
    //! The recipe-panel UI (the Layers section) is the consumer:
    //! [`support`] keys off a layer's base-material anchor key and
    //! the panel renders the resulting curve as a Slacker-ratio
    //! picker. The module's public surface is `pub(crate)`; the
    //! curve tables + the `point` table-builder stay private.

    use std::fmt;

    /// Durometer scale for a Slacker-recipe hardness reading.
    /// Slacker pushes silicones across scales: a Shore A base
    /// softens through Shore 00 (OO) into Shore 000 (OOO, the gel
    /// scale). These are distinct durometer scales — not cleanly
    /// interconvertible — so the recipe readout surfaces the scale
    /// explicitly instead of collapsing them to one number.
    // `OO` / `OOO` are the established Shore durometer-scale names;
    // clippy's `Ooo` suggestion would obscure them.
    #[allow(clippy::upper_case_acronyms)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub(crate) enum ShoreScale {
        /// Shore A — the firmest scale here; Dragon Skin's native
        /// grades.
        A,
        /// Shore 00 (OO) — soft rubbers; the Ecoflex line's native
        /// scale.
        OO,
        /// Shore 000 (OOO) — gels and ultra-soft rubbers; where
        /// most Slacker recipes land.
        OOO,
    }

    impl fmt::Display for ShoreScale {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self {
                ShoreScale::A => write!(f, "A"),
                ShoreScale::OO => write!(f, "00"),
                ShoreScale::OOO => write!(f, "000"),
            }
        }
    }

    /// A cured Shore hardness reading on a named durometer scale.
    /// Displays the way Smooth-On writes it — `10A`, `00-30`,
    /// `000-7`.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub(crate) struct ShoreHardness {
        /// Which durometer scale `points` is read on.
        pub(crate) scale: ShoreScale,
        /// Durometer points on `scale`. Whole numbers in the TB;
        /// `u32` because Slacker recipes snap to the tabulated data
        /// points (the OO/OOO scale split makes interpolating a
        /// single hardness number meaningless).
        pub(crate) points: u32,
    }

    impl fmt::Display for ShoreHardness {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self.scale {
                // Shore A is written suffixed: "10A".
                ShoreScale::A => write!(f, "{}A", self.points),
                // Shore 00 / 000 are written prefixed + hyphenated:
                // "00-30", "000-7".
                ShoreScale::OO | ShoreScale::OOO => {
                    write!(f, "{}-{}", self.scale, self.points)
                }
            }
        }
    }

    /// Cured surface tack at a given Slacker fraction, from the
    /// TB's per-row tack column.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub(crate) enum Tack {
        /// No tack — the base silicone, or a low Slacker fraction.
        None,
        /// Slight tack.
        Slight,
        /// Between slight and very ("Slight to Very" in the TB).
        SlightToVery,
        /// Very tacky / self-sticking.
        Very,
    }

    impl fmt::Display for Tack {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            let label = match self {
                Tack::None => "no tack",
                Tack::Slight => "slight tack",
                Tack::SlightToVery => "slight-to-very tack",
                Tack::Very => "very tacky",
            };
            write!(f, "{label}")
        }
    }

    /// One point on a base silicone's Slacker softening curve: a
    /// Slacker fraction plus the resulting cured hardness + tack.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub(crate) struct Point {
        /// Slacker mass as a fraction of the base silicone's
        /// combined Part A + Part B mass. `0.0` is the base with no
        /// Slacker (native hardness); the TB's "+50 / +100 / +150 /
        /// +200 Slacker per 100A+100B" rows are `0.25 / 0.50 /
        /// 0.75 / 1.00`.
        pub(crate) slacker_fraction: f64,
        /// Cured Shore hardness at this Slacker fraction.
        pub(crate) hardness: ShoreHardness,
        /// Cured surface tack at this Slacker fraction.
        pub(crate) tack: Tack,
    }

    /// Table-construction helper — keeps the const curves below
    /// readable. Module-internal; the curves are its only callers.
    const fn point(slacker_fraction: f64, scale: ShoreScale, points: u32, tack: Tack) -> Point {
        Point {
            slacker_fraction,
            hardness: ShoreHardness { scale, points },
            tack,
        }
    }

    /// What Slacker data exists for a given base silicone.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub(crate) enum Support {
        /// Smooth-On's TB tabulates a softening curve for this
        /// base. The slice leads with the `0.0`-fraction native-
        /// hardness point, then the TB's data points in
        /// increasing-Slacker order; the last point's fraction is
        /// the per-base recommended Slacker cap.
        Curve(&'static [Point]),
        /// Smooth-On's TB explicitly advises against Slacker with
        /// this base (Ecoflex 00-10).
        NotRecommended,
        /// No published Slacker data for this base (Dragon Skin
        /// 15 / 20A / 30A). Not a guess — the TB simply does not
        /// cover it.
        NoData,
    }

    // Per-base softening curves, transcribed verbatim from the
    // Slacker TB (rev 011524DH). Each leads with the native-
    // hardness point at slacker_fraction 0.0. Ecoflex bases stop at
    // 0.50 — the TB marks +150 / +200 Slacker "Not Recommended" for
    // the Ecoflex line; the Dragon Skin line has no such cap.

    /// Ecoflex 00-20 + Slacker — native Shore 00-20.
    const ECOFLEX_00_20_CURVE: &[Point] = &[
        point(0.0, ShoreScale::OO, 20, Tack::None),
        point(0.25, ShoreScale::OOO, 47, Tack::Slight),
        point(0.50, ShoreScale::OOO, 32, Tack::SlightToVery),
    ];

    /// Ecoflex 00-30 + Slacker — native Shore 00-30.
    const ECOFLEX_00_30_CURVE: &[Point] = &[
        point(0.0, ShoreScale::OO, 30, Tack::None),
        point(0.25, ShoreScale::OOO, 40, Tack::SlightToVery),
        point(0.50, ShoreScale::OOO, 20, Tack::Very),
    ];

    /// Ecoflex 00-50 + Slacker — native Shore 00-50.
    const ECOFLEX_00_50_CURVE: &[Point] = &[
        point(0.0, ShoreScale::OO, 50, Tack::None),
        point(0.25, ShoreScale::OOO, 55, Tack::Slight),
        point(0.50, ShoreScale::OOO, 35, Tack::SlightToVery),
    ];

    /// Dragon Skin 10A + Slacker — native Shore 10A. The TB's
    /// "Dragon Skin 10" row; runs to 1.0 (the Ecoflex +150 / +200
    /// cap does not apply to the Dragon Skin line).
    const DRAGON_SKIN_10A_CURVE: &[Point] = &[
        point(0.0, ShoreScale::A, 10, Tack::None),
        point(0.25, ShoreScale::OO, 30, Tack::None),
        point(0.50, ShoreScale::OOO, 50, Tack::Slight),
        point(0.75, ShoreScale::OOO, 20, Tack::SlightToVery),
        point(1.00, ShoreScale::OOO, 7, Tack::Very),
    ];

    /// Slacker data available for a layer's base silicone, keyed by
    /// the catalog `anchor_key` ([`super::LAYER_MATERIALS`]).
    pub(crate) fn support(anchor_key: &str) -> Support {
        match anchor_key {
            "ECOFLEX_00_20" => Support::Curve(ECOFLEX_00_20_CURVE),
            "ECOFLEX_00_30" => Support::Curve(ECOFLEX_00_30_CURVE),
            "ECOFLEX_00_50" => Support::Curve(ECOFLEX_00_50_CURVE),
            "DRAGON_SKIN_10A" => Support::Curve(DRAGON_SKIN_10A_CURVE),
            // The TB explicitly says do NOT use Slacker with Ecoflex
            // 00-10 (or Ecoflex GEL).
            "ECOFLEX_00_10" => Support::NotRecommended,
            // Dragon Skin 15 / 20A / 30A are not covered by the
            // Slacker TB — no guessed curve. `_` also catches any
            // off-catalog key defensively.
            "DRAGON_SKIN_15" | "DRAGON_SKIN_20A" | "DRAGON_SKIN_30A" => Support::NoData,
            _ => Support::NoData,
        }
    }

    #[cfg(test)]
    mod tests {
        use super::{
            DRAGON_SKIN_10A_CURVE, ECOFLEX_00_20_CURVE, ECOFLEX_00_30_CURVE, ECOFLEX_00_50_CURVE,
            Point, ShoreHardness, ShoreScale, Support, Tack, point, support,
        };

        /// The four TB-tabulated curves, paired with their catalog
        /// anchor key. Iterating this keeps the structural tests
        /// free of per-curve duplication.
        const ALL_CURVES: &[(&str, &[Point])] = &[
            ("ECOFLEX_00_20", ECOFLEX_00_20_CURVE),
            ("ECOFLEX_00_30", ECOFLEX_00_30_CURVE),
            ("ECOFLEX_00_50", ECOFLEX_00_50_CURVE),
            ("DRAGON_SKIN_10A", DRAGON_SKIN_10A_CURVE),
        ];

        /// Every catalog silicone + the [`Support`] variant it must
        /// resolve to. Adding a catalog entry without deciding its
        /// Slacker status trips `support_covers_every_catalog_silicone`.
        const EXPECTED: &[(&str, &str)] = &[
            ("ECOFLEX_00_10", "not_recommended"),
            ("ECOFLEX_00_20", "curve"),
            ("ECOFLEX_00_30", "curve"),
            ("ECOFLEX_00_50", "curve"),
            ("DRAGON_SKIN_10A", "curve"),
            ("DRAGON_SKIN_15", "no_data"),
            ("DRAGON_SKIN_20A", "no_data"),
            ("DRAGON_SKIN_30A", "no_data"),
        ];

        fn variant_tag(s: Support) -> &'static str {
            match s {
                Support::Curve(_) => "curve",
                Support::NotRecommended => "not_recommended",
                Support::NoData => "no_data",
            }
        }

        #[test]
        fn support_covers_every_catalog_silicone() {
            // Pins that every catalog silicone has a deliberate
            // Slacker decision, and that the catalog + this table
            // stay in sync (count + per-key variant).
            let catalog: Vec<&str> = super::super::LAYER_MATERIALS
                .iter()
                .map(|(k, _, _)| *k)
                .collect();
            let expected_keys: Vec<&str> = EXPECTED.iter().map(|(k, _)| *k).collect();
            assert_eq!(catalog.len(), EXPECTED.len());
            for key in &catalog {
                assert!(
                    expected_keys.contains(key),
                    "catalog silicone {key} has no Slacker decision in EXPECTED",
                );
            }
            for (key, tag) in EXPECTED {
                assert_eq!(variant_tag(support(key)), *tag, "{key}");
            }
        }

        #[test]
        fn support_wires_each_curve_key_to_its_table() {
            // The four curve keys resolve to Curve(_) pointing at
            // the matching const table — guards against a copy-
            // paste swap in `support`'s match arms.
            for (key, curve) in ALL_CURVES {
                assert!(
                    matches!(support(key), Support::Curve(c) if c == *curve),
                    "{key} did not resolve to its own curve",
                );
            }
        }

        #[test]
        fn curve_structural_invariants() {
            for (key, curve) in ALL_CURVES {
                // Leads with the native-hardness point: 0.0 Slacker,
                // no tack.
                assert!(
                    (curve[0].slacker_fraction - 0.0).abs() < f64::EPSILON,
                    "{key}: curve must lead with slacker_fraction 0.0",
                );
                assert_eq!(
                    curve[0].tack,
                    Tack::None,
                    "{key}: native point must have no tack",
                );
                // Slacker fraction strictly increases along the curve.
                for pair in curve.windows(2) {
                    assert!(
                        pair[1].slacker_fraction > pair[0].slacker_fraction,
                        "{key}: slacker_fraction must strictly increase",
                    );
                }
            }
        }

        #[test]
        fn ecoflex_bases_cap_at_half_slacker_dragon_skin_runs_full() {
            // The last point's fraction is the per-base recommended
            // Slacker cap: Ecoflex bases stop at 0.50 (TB marks
            // +150/+200 "Not Recommended"); Dragon Skin 10A runs to
            // 1.00.
            let cap = |curve: &[Point]| curve[curve.len() - 1].slacker_fraction;
            assert!((cap(ECOFLEX_00_20_CURVE) - 0.50).abs() < f64::EPSILON);
            assert!((cap(ECOFLEX_00_30_CURVE) - 0.50).abs() < f64::EPSILON);
            assert!((cap(ECOFLEX_00_50_CURVE) - 0.50).abs() < f64::EPSILON);
            assert!((cap(DRAGON_SKIN_10A_CURVE) - 1.00).abs() < f64::EPSILON);
        }

        #[test]
        fn curve_hardness_values_match_tb() {
            // Verbatim transcription pin against the Slacker TB
            // (rev 011524DH). A drift in any tabulated hardness or
            // tack value trips here.
            assert_eq!(
                ECOFLEX_00_20_CURVE,
                &[
                    point(0.0, ShoreScale::OO, 20, Tack::None),
                    point(0.25, ShoreScale::OOO, 47, Tack::Slight),
                    point(0.50, ShoreScale::OOO, 32, Tack::SlightToVery),
                ][..],
            );
            assert_eq!(
                ECOFLEX_00_30_CURVE,
                &[
                    point(0.0, ShoreScale::OO, 30, Tack::None),
                    point(0.25, ShoreScale::OOO, 40, Tack::SlightToVery),
                    point(0.50, ShoreScale::OOO, 20, Tack::Very),
                ][..],
            );
            assert_eq!(
                ECOFLEX_00_50_CURVE,
                &[
                    point(0.0, ShoreScale::OO, 50, Tack::None),
                    point(0.25, ShoreScale::OOO, 55, Tack::Slight),
                    point(0.50, ShoreScale::OOO, 35, Tack::SlightToVery),
                ][..],
            );
            assert_eq!(
                DRAGON_SKIN_10A_CURVE,
                &[
                    point(0.0, ShoreScale::A, 10, Tack::None),
                    point(0.25, ShoreScale::OO, 30, Tack::None),
                    point(0.50, ShoreScale::OOO, 50, Tack::Slight),
                    point(0.75, ShoreScale::OOO, 20, Tack::SlightToVery),
                    point(1.00, ShoreScale::OOO, 7, Tack::Very),
                ][..],
            );
        }

        #[test]
        fn shore_hardness_displays_like_smooth_on() {
            // Shore A suffixed; Shore 00 / 000 prefixed + hyphenated.
            assert_eq!(
                ShoreHardness {
                    scale: ShoreScale::A,
                    points: 10,
                }
                .to_string(),
                "10A",
            );
            assert_eq!(
                ShoreHardness {
                    scale: ShoreScale::OO,
                    points: 30,
                }
                .to_string(),
                "00-30",
            );
            assert_eq!(
                ShoreHardness {
                    scale: ShoreScale::OOO,
                    points: 7,
                }
                .to_string(),
                "000-7",
            );
        }

        #[test]
        fn tack_displays_human_labels() {
            assert_eq!(Tack::None.to_string(), "no tack");
            assert_eq!(Tack::Slight.to_string(), "slight tack");
            assert_eq!(Tack::SlightToVery.to_string(), "slight-to-very tack");
            assert_eq!(Tack::Very.to_string(), "very tacky");
        }
    }
}

// ============================================================
// Mesh-based surface rendering (slice 5 polish 2)
// ============================================================
//
// Each surface (the cavity + one per layer) is a Bevy entity with
// a triangle Mesh asset + opaque StandardMaterial. Depth-test does
// the right thing — near fragments occlude far fragments — so the
// user no longer sees the far side rendering through the near side
// (the gizmo-wireframe artifact that slice 5 polish 2 replaced).
//
// The mesh asset is regenerated when its source state changes
// (`is_changed()` check on the relevant resource). Re-meshing one
// surface is sub-millisecond on the iter-1 decimated proxy
// (1500 faces).

/// Marker component for the cavity mesh entity. One entity at
/// runtime.
#[derive(Component)]
struct CavityEntity;

/// Marker component for a per-layer outer-surface mesh entity.
/// Bevy spawns / despawns one entity per layer whenever the layer
/// stack changes. The layer index isn't stored on the component
/// because the despawn-all-then-respawn update pattern doesn't
/// need it.
#[derive(Component)]
struct LayerSurfaceEntity;

/// Build a Bevy `Mesh` from the proxy mesh's connectivity, with
/// each vertex displaced along its radial direction by `offset_m`
/// in physics frame, then mapped through the cast-frame `UpAxis`
/// swap + `render_scale` lift to Bevy frame.
fn build_displaced_proxy_mesh(
    proxy: &EnvelopeProxyMesh,
    offset_m: f64,
    up: UpAxis,
    render_scale: f32,
) -> Mesh {
    let positions: Vec<[f32; 3]> = proxy
        .vertices
        .iter()
        .zip(proxy.vertex_radial_directions.iter())
        .map(|(v, r)| {
            let displaced_physics = Point3::from(v.coords + r * offset_m);
            let bevy = up.to_bevy_point(&displaced_physics);
            #[allow(clippy::cast_possible_truncation)] // f64 → f32 for Bevy.
            [
                bevy[0] * render_scale,
                bevy[1] * render_scale,
                bevy[2] * render_scale,
            ]
        })
        .collect();
    let indices: Vec<u32> = proxy.faces.iter().flatten().copied().collect();

    let mut mesh = Mesh::new(
        bevy::mesh::PrimitiveTopology::TriangleList,
        bevy::asset::RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_indices(bevy::mesh::Indices::U32(indices));
    // Compute smooth (per-vertex-averaged) normals from the
    // triangle winding. Bevy's PBR shader needs vertex normals for
    // lighting; without them surfaces render uniform-dark. We use
    // smooth-normals (not flat) because flat-normals require
    // unindexed geometry — and the proxy mesh is indexed for
    // memory + render efficiency.
    mesh.compute_smooth_normals();
    mesh
}

/// Palette for the per-layer outer-surface mesh entities. Repeats
/// if the layer count exceeds the palette length.
const LAYER_SURFACE_PALETTE: &[(f32, f32, f32)] = &[
    (0.95, 0.80, 0.35), // amber
    (0.45, 0.70, 0.95), // sky blue
    (0.75, 0.55, 0.95), // lavender
    (0.55, 0.95, 0.65), // mint
    (0.95, 0.45, 0.70), // pink
];

/// Cavity surface color (`StandardMaterial::base_color`). The
/// cavity is the inner void surface; coral distinguishes it from
/// the layer palette + scan + axis arrows. Material is double-
/// sided so the user sees the inside when layers are hidden.
const CAVITY_COLOR: (f32, f32, f32) = (0.95, 0.55, 0.45);

/// Spawn the cavity mesh entity at startup. Layer entities spawn
/// lazily on the first state-change tick.
#[allow(clippy::needless_pass_by_value)]
fn spawn_cavity_mesh(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    proxy: Res<EnvelopeProxyMesh>,
    cavity: Res<CavityState>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
) {
    if proxy.vertices.is_empty() || proxy.faces.is_empty() {
        return;
    }
    let cavity_mesh = meshes.add(build_displaced_proxy_mesh(
        &proxy,
        -cavity.inset_m,
        *up,
        render_scale.0,
    ));
    let cavity_material = materials.add(StandardMaterial {
        base_color: Color::srgb(CAVITY_COLOR.0, CAVITY_COLOR.1, CAVITY_COLOR.2),
        double_sided: true,
        cull_mode: None,
        ..default()
    });
    commands.spawn((
        Mesh3d(cavity_mesh),
        MeshMaterial3d(cavity_material),
        CavityEntity,
    ));
}

/// Regenerate the cavity mesh asset when `CavityState` changes.
#[allow(clippy::needless_pass_by_value)]
fn update_cavity_mesh(
    state: Res<CavityState>,
    proxy: Res<EnvelopeProxyMesh>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut q: Query<(&mut Mesh3d, &mut Visibility), With<CavityEntity>>,
) {
    if !state.is_changed() {
        return;
    }
    for (mut mesh_handle, mut visibility) in &mut q {
        let new_mesh = build_displaced_proxy_mesh(&proxy, -state.inset_m, *up, render_scale.0);
        mesh_handle.0 = meshes.add(new_mesh);
        *visibility = if state.visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}

/// Synchronize the per-layer mesh entities with `LayersState`. One
/// entity per layer, each at its OUTER-surface offset = cumulative
/// `sum(thickness[0..=i]) - cavity.inset_m` from the scan surface.
/// Layer N-1's outer surface is the device's outer skin (no
/// separate "outer envelope" concept). Each entity's visibility
/// tracks its own `LayerSpec::visible` flag.
///
/// Despawn-and-respawn on any state change — layer count is small
/// (≤ `LAYER_COUNT_MAX = 6`) so the cost is negligible.
#[allow(clippy::needless_pass_by_value, clippy::too_many_arguments)]
fn update_layer_meshes(
    layers: Res<LayersState>,
    cavity: Res<CavityState>,
    proxy: Res<EnvelopeProxyMesh>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    existing: Query<Entity, With<LayerSurfaceEntity>>,
) {
    if !layers.is_changed() && !cavity.is_changed() {
        return;
    }
    for entity in &existing {
        commands.entity(entity).despawn();
    }
    if layers.layers.is_empty() || proxy.vertices.is_empty() {
        return;
    }
    let mut cumulative_thickness_m = 0.0_f64;
    for (i, layer) in layers.layers.iter().enumerate() {
        cumulative_thickness_m += layer.thickness_m;
        let offset_m = cumulative_thickness_m - cavity.inset_m;
        let mesh = meshes.add(build_displaced_proxy_mesh(
            &proxy,
            offset_m,
            *up,
            render_scale.0,
        ));
        let (r, g, b) = LAYER_SURFACE_PALETTE[i % LAYER_SURFACE_PALETTE.len()];
        let material = materials.add(StandardMaterial {
            base_color: Color::srgb(r, g, b),
            double_sided: true,
            cull_mode: None,
            ..default()
        });
        let visibility = if layer.visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
        commands.spawn((
            Mesh3d(mesh),
            MeshMaterial3d(material),
            visibility,
            LayerSurfaceEntity,
        ));
    }
}

/// Apply the [`ScanMeshVisible`] toggle to the scan-mesh entity's
/// `Visibility` each frame. Cheap (one component write per frame
/// when the toggle changes; Bevy short-circuits Visibility writes
/// that don't change the value).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
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

/// Render the Cavity egui section. The cavity is the scan-derived
/// inner void the appendage slides into; its surface sits `inset_m`
/// inside the scan along the per-vertex radial direction, so the
/// silicone skin between the cavity and the scan stretches over the
/// appendage for a snug fit.
///
/// Slice 4 v1 ships uniform-inset only. A later slice may add the
/// insertable-length clip + tangent-perpendicular end cap.
fn render_cavity_section(ui: &mut egui::Ui, state: &mut CavityState) {
    egui::CollapsingHeader::new("Cavity")
        .default_open(true)
        .show(ui, |ui| {
            ui.checkbox(&mut state.visible, "Show cavity");
            let (min_m, max_m) = CavityState::inset_slider_range_m();
            let mut inset_mm = state.inset_m * 1000.0;
            let min_mm = min_m * 1000.0;
            let max_mm = max_m * 1000.0;
            if ui
                .add(egui::Slider::new(&mut inset_mm, min_mm..=max_mm).text("inset (mm)"))
                .changed()
            {
                state.inset_m = inset_mm * 0.001;
            }
            ui.label("(silicone skin stretches inset_m over appendage)");
        });
}

/// Render the Layers egui section. Surfaces the ordered layer
/// stack innermost-first. Every layer carries its own controls:
/// a visibility checkbox, a thickness slider, a material dropdown,
/// a Slacker recipe control, and a remove button (hidden when only
/// one layer remains). A "+ Add layer" button appends a new
/// outermost layer up to [`LAYER_COUNT_MAX`].
///
/// The device wall total is the sum of layer thicknesses; the
/// outer skin sits at `sum(thickness) - cavity.inset_m` radially
/// from the scan. Those derived figures are shown as read-only
/// labels at the bottom of the section.
///
/// `validations` is the shared [`DeviceValidations`] — the Slacker
/// control reads each layer's pour mass from it for the mix-in-
/// grams readout. A layer added this frame is not yet in
/// `validations.layers` (it is computed once at the top of the
/// panel); the control shows a `—` placeholder for that one frame.
///
/// [`update_layer_meshes`] regenerates the per-layer surface mesh
/// entities whenever `LayersState` or `CavityState` mutates.
fn render_layers_section(
    ui: &mut egui::Ui,
    layers: &mut LayersState,
    cavity: &CavityState,
    validations: &DeviceValidations,
) {
    egui::CollapsingHeader::new("Layers")
        .default_open(true)
        .show(ui, |ui| {
            let n = layers.layers.len();
            let (t_floor_m, t_ceiling_m) = LayerSpec::thickness_slider_range_m();
            let t_floor_mm = t_floor_m * 1000.0;
            let t_ceiling_mm = t_ceiling_m * 1000.0;

            // Track which layer the user requested removal of (if
            // any). egui's immediate-mode loop can't mutate the Vec
            // while iterating, so we defer until after the loop.
            let mut remove_index: Option<usize> = None;
            for i in 0..n {
                ui.group(|ui| {
                    ui.label(format!("Layer {} ({})", i, layer_position_label(i, n)));
                    ui.checkbox(&mut layers.layers[i].visible, "Show layer");
                    let mut t_mm = layers.layers[i].thickness_m * 1000.0;
                    let slider_text = if i == 0 {
                        "thickness from cavity (mm)"
                    } else {
                        "Δ from prior layer (mm)"
                    };
                    if ui
                        .add(
                            egui::Slider::new(&mut t_mm, t_floor_mm..=t_ceiling_mm)
                                .text(slider_text),
                        )
                        .changed()
                    {
                        layers.layers[i].thickness_m = t_mm * 0.001;
                    }
                    render_material_dropdown(ui, i, &mut layers.layers[i].material_anchor_key);
                    let pour_mass_kg = validations.layers.get(i).map(|lv| lv.pour_mass_kg);
                    render_slacker_control(ui, i, &mut layers.layers[i], pour_mass_kg);
                    if n > 1 && ui.button("Remove layer").clicked() {
                        remove_index = Some(i);
                    }
                });
                ui.add_space(2.0);
            }
            if let Some(idx) = remove_index {
                layers.layers.remove(idx);
            }

            if layers.layers.len() < LAYER_COUNT_MAX && ui.button("+ Add layer").clicked() {
                // Append at outermost position with 3 mm default.
                // New layer starts visible, no Slacker; user can
                // toggle via the per-layer "Show layer" checkbox.
                layers.layers.push(LayerSpec {
                    thickness_m: 0.003,
                    material_anchor_key: "ECOFLEX_00_30",
                    slacker_fraction: 0.0,
                    visible: true,
                });
            }

            // Derived geometric readouts: the device's outer skin
            // is the outermost layer's outer surface, at offset
            // `sum(thickness) - cavity.inset` radially from the
            // scan surface.
            let sum_layers_m: f64 = layers.layers.iter().map(|l| l.thickness_m).sum();
            let derived_outer_offset_m = sum_layers_m - cavity.inset_m;
            ui.separator();
            ui.label(format!(
                "Wall total: {:.1} mm  (sum of layers)",
                sum_layers_m * 1000.0,
            ));
            ui.label(format!(
                "  outer skin: +{:.1} mm from scan",
                derived_outer_offset_m * 1000.0,
            ));
            ui.label(format!(
                "  cavity: -{:.1} mm from scan",
                cavity.inset_m * 1000.0,
            ));
        });
}

/// Human-readable position label for a layer in the stack
/// ("single", "innermost", "middle", "outermost"). The innermost
/// layer touches the cavity surface; the outermost layer's outer
/// surface is the device's outer skin.
fn layer_position_label(i: usize, n: usize) -> &'static str {
    if n == 1 {
        "single"
    } else if i == 0 {
        "innermost"
    } else if i == n - 1 {
        "outermost"
    } else {
        "middle"
    }
}

/// Material-selection ComboBox for one layer. `id_source = i`
/// distinguishes multiple dropdowns in the same parent UI.
fn render_material_dropdown(ui: &mut egui::Ui, layer_index: usize, selected: &mut &'static str) {
    let current_label = LAYER_MATERIALS
        .iter()
        .find(|(key, _, _)| *key == *selected)
        .map(|(_, label, _)| *label)
        .unwrap_or("(unknown)");
    egui::ComboBox::from_id_salt(("layer-material", layer_index))
        .selected_text(current_label)
        .show_ui(ui, |ui| {
            for (key, label, _) in LAYER_MATERIALS {
                ui.selectable_value(selected, key, *label);
            }
        });
}

/// The Slacker fraction a layer should actually use: `requested` if
/// it is an exact data point on the base silicone's Smooth-On TB
/// curve, otherwise `0.0` (no Slacker — the always-valid native
/// default).
///
/// Called every frame before the Slacker control renders, so a
/// base-material change that orphans the stored fraction (the new
/// material has a different curve, or no Slacker support at all)
/// snaps cleanly back to "no Slacker" instead of silently keeping a
/// fraction that is not on the new curve.
fn resolve_slacker_fraction(anchor_key: &str, requested: f64) -> f64 {
    match slacker::support(anchor_key) {
        slacker::Support::Curve(curve) => {
            let on_curve = curve
                .iter()
                .any(|p| (p.slacker_fraction - requested).abs() < f64::EPSILON);
            if on_curve { requested } else { 0.0 }
        }
        // No Slacker curve for this base — force the native silicone.
        slacker::Support::NotRecommended | slacker::Support::NoData => 0.0,
    }
}

/// Part A / Part B / Slacker masses (grams) to weigh out for one
/// layer's pour. Returns `(part_ab_g, slacker_g)` where `part_ab_g`
/// is the mass of *each* of Part A and Part B (the base silicone is
/// 1:1 A:B by weight).
///
/// `pour_mass_kg` is the layer's shell mass from
/// [`compute_validations`]; `slacker_fraction` is Slacker mass as a
/// fraction of the base A+B mass, so the cured mix obeys
/// `pour_mass = 2·part_ab·(1 + slacker_fraction)`.
///
/// Approximation: `pour_mass_kg` is `shell_volume × base density` —
/// this treats the cured base+Slacker mix density as ≈ the base
/// silicone's (the Slacker TB publishes no density). Close enough
/// for a bench weigh-out, where the user adds a waste margin
/// regardless.
fn slacker_recipe_grams(pour_mass_kg: f64, slacker_fraction: f64) -> (f64, f64) {
    let pour_mass_g = pour_mass_kg * 1000.0;
    let part_ab_g = pour_mass_g / (2.0 * (1.0 + slacker_fraction));
    let slacker_g = slacker_fraction * pour_mass_g / (1.0 + slacker_fraction);
    (part_ab_g, slacker_g)
}

/// Dropdown label for one Slacker-curve point: the native (`0.0`)
/// point reads `No Slacker — Shore 00-30`; every other point reads
/// `+25% Slacker — Shore 000-40 (slight-to-very tack)` (percentage,
/// effective Shore hardness, cured tack).
fn slacker_point_label(point: &slacker::Point) -> String {
    if point.slacker_fraction > 0.0 {
        format!(
            "+{:.0}% Slacker — Shore {} ({})",
            point.slacker_fraction * 100.0,
            point.hardness,
            point.tack,
        )
    } else {
        format!("No Slacker — Shore {}", point.hardness)
    }
}

/// Render one layer's Slacker recipe control. For a base silicone
/// with TB data this is a dropdown of the curve's Slacker ratios
/// (each labelled with its effective Shore hardness + tack) plus a
/// mix-in-grams readout for the selected ratio; for a base without
/// data it is a disabled note explaining why.
///
/// `pour_mass_kg` is the layer's shell mass (from the shared
/// [`DeviceValidations`]), or `None` for a layer added this frame
/// that the validation pass has not caught up to yet.
fn render_slacker_control(
    ui: &mut egui::Ui,
    layer_index: usize,
    layer: &mut LayerSpec,
    pour_mass_kg: Option<f64>,
) {
    // Snap the stored fraction to a value valid for the current
    // base material before rendering — guards a base-material
    // change that orphaned it.
    layer.slacker_fraction =
        resolve_slacker_fraction(layer.material_anchor_key, layer.slacker_fraction);

    match slacker::support(layer.material_anchor_key) {
        slacker::Support::Curve(curve) => {
            let selected_label = curve
                .iter()
                .find(|p| (p.slacker_fraction - layer.slacker_fraction).abs() < f64::EPSILON)
                .map_or_else(|| "No Slacker".to_string(), slacker_point_label);
            egui::ComboBox::from_id_salt(("layer-slacker", layer_index))
                .selected_text(selected_label)
                .show_ui(ui, |ui| {
                    for p in curve {
                        ui.selectable_value(
                            &mut layer.slacker_fraction,
                            p.slacker_fraction,
                            slacker_point_label(p),
                        );
                    }
                });
            // Mix-in-grams readout for the selected ratio.
            match pour_mass_kg {
                Some(mass_kg) => {
                    let (part_ab_g, slacker_g) =
                        slacker_recipe_grams(mass_kg, layer.slacker_fraction);
                    if layer.slacker_fraction > 0.0 {
                        ui.label(format!(
                            "  Mix: {part_ab_g:.1} g A + {part_ab_g:.1} g B + {slacker_g:.1} g Slacker",
                        ));
                    } else {
                        ui.label(format!("  Mix: {part_ab_g:.1} g A + {part_ab_g:.1} g B"));
                    }
                }
                None => {
                    ui.label("  Mix: —");
                }
            }
        }
        slacker::Support::NotRecommended => {
            ui.add_enabled(
                false,
                egui::Label::new("Slacker: not recommended for this silicone"),
            );
        }
        slacker::Support::NoData => {
            ui.add_enabled(
                false,
                egui::Label::new("Slacker: no published data for this silicone"),
            );
        }
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

/// Right-side egui sidebar carrying the design-suite panels.
/// Renders Scan Info + Cavity + Layers + Validations; remaining
/// stubs surface the planned panel order (Insertion Sim /
/// Save-Open).
///
/// [`compute_validations`] runs once at the top of the panel, from
/// the current resource state, and the resulting [`DeviceValidations`]
/// is shared by two consumers: the Layers section (each layer's
/// pour mass drives its Slacker mix-in-grams readout) and the
/// Validations section. A slider edit this frame lands in those
/// readouts one frame later — imperceptible at frame rate, and it
/// keeps both sections reading a single consistent computation
/// rather than each re-deriving it.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn device_design_panel(
    mut contexts: EguiContexts,
    info: Res<ScanInfo>,
    proxy: Res<EnvelopeProxyMesh>,
    mut scan_visible: ResMut<ScanMeshVisible>,
    mut cavity: ResMut<CavityState>,
    mut layers: ResMut<LayersState>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    let validations = compute_validations(&proxy, &cavity, &layers);
    egui::SidePanel::right("cf-device-design-panels")
        .resizable(false)
        .default_width(320.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    render_scan_info_section(ui, &info, &mut scan_visible);
                    render_cavity_section(ui, &mut cavity);
                    render_layers_section(ui, &mut layers, &cavity, &validations);
                    render_validations_section(ui, &validations);
                    render_panel_stubs(ui);
                });
        });
    Ok(())
}

fn render_scan_info_section(
    ui: &mut egui::Ui,
    info: &ScanInfo,
    scan_visible: &mut ScanMeshVisible,
) {
    egui::CollapsingHeader::new("Scan Info")
        .default_open(true)
        .show(ui, |ui| {
            ui.checkbox(&mut scan_visible.0, "Show scan mesh");
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
/// panel names so the user can see the planned layout. Order tracks
/// the reprioritized slice ladder: the FEM insertion sim (slice 7)
/// is pulled ahead of Features / Texture because the simulation IS
/// the engineering payoff of the suite.
fn render_panel_stubs(ui: &mut egui::Ui) {
    ui.separator();
    for name in ["Insertion Sim (slice 7)", "Save / Open (slice 8)"] {
        ui.add_enabled(false, egui::Label::new(name));
    }
}

/// egui color for a [`BudgetStatus`] — green / amber / red.
fn budget_color(status: BudgetStatus) -> egui::Color32 {
    match status {
        BudgetStatus::Green => egui::Color32::from_rgb(90, 200, 110),
        BudgetStatus::Yellow => egui::Color32::from_rgb(230, 180, 60),
        BudgetStatus::Red => egui::Color32::from_rgb(225, 90, 80),
    }
}

/// Render the Validations egui section — per-layer pour volume +
/// mass graded against the 2 lb single-pour budget, plus the cavity
/// self-intersection and minimum-castable-wall checks.
///
/// All figures are derived; the section is read-only. The headline
/// color is the worst per-layer budget status. See
/// [`compute_validations`] for the geometry behind each number.
fn render_validations_section(ui: &mut egui::Ui, v: &DeviceValidations) {
    egui::CollapsingHeader::new("Validations")
        .default_open(true)
        .show(ui, |ui| {
            let n = v.layers.len();
            ui.colored_label(
                budget_color(v.worst_status),
                format!(
                    "Pour mass — budget {:.0} g / 2 lb per pour",
                    MASS_BUDGET_KG * 1000.0,
                ),
            );
            for lv in &v.layers {
                ui.colored_label(
                    budget_color(lv.status),
                    format!(
                        "  Layer {} ({}): {:.1} cm³ · {:.1} g",
                        lv.layer_index,
                        layer_position_label(lv.layer_index, n),
                        lv.shell_volume_m3 * 1.0e6,
                        lv.pour_mass_kg * 1000.0,
                    ),
                );
            }
            ui.label(format!(
                "Total: {:.1} g across {} pour{}",
                v.total_mass_kg * 1000.0,
                n,
                if n == 1 { "" } else { "s" },
            ));

            ui.separator();

            // Cavity self-intersection.
            if v.cavity_collapse_inset_m.is_infinite() {
                ui.label("Cavity: n/a (no centerline)");
            } else if v.cavity_self_intersects {
                ui.colored_label(
                    budget_color(BudgetStatus::Red),
                    format!(
                        "Cavity self-intersects (inset ≥ {:.1} mm)",
                        v.cavity_collapse_inset_m * 1000.0,
                    ),
                );
            } else {
                ui.colored_label(
                    budget_color(BudgetStatus::Green),
                    format!(
                        "Cavity: OK (collapses at inset ≥ {:.1} mm)",
                        v.cavity_collapse_inset_m * 1000.0,
                    ),
                );
            }

            // Minimum castable wall thickness.
            if v.min_wall_ok {
                ui.colored_label(
                    budget_color(BudgetStatus::Green),
                    format!(
                        "Min wall: OK (every layer ≥ {:.1} mm)",
                        MIN_CASTABLE_THICKNESS_M * 1000.0,
                    ),
                );
            } else {
                for lv in v.layers.iter().filter(|lv| !lv.thickness_castable) {
                    ui.colored_label(
                        budget_color(BudgetStatus::Red),
                        format!(
                            "Layer {} below {:.1} mm castability floor",
                            lv.layer_index,
                            MIN_CASTABLE_THICKNESS_M * 1000.0,
                        ),
                    );
                }
            }
        });
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
    // Decimate the cleaned scan to a shared proxy mesh (~1500
    // faces) + per-vertex radial directions. The cavity + per-layer
    // surface meshes are regenerated from this proxy on every
    // slider tick. One-time decimation cost at startup; ~5–10 s on
    // the iter-1 3.3 M-face fixture, sub-second on small inputs.
    let envelope_proxy = compute_envelope_proxy_mesh(&scan_mesh, &centerline_points);
    println!(
        "envelope proxy: {} faces, {} vertices (decimated from {} faces)",
        envelope_proxy.faces.len(),
        envelope_proxy.vertices.len(),
        envelope_proxy.original_face_count,
    );
    let cavity = CavityState::default_for_scan();
    let layers = LayersState::default_for_scan();

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
        .insert_resource(envelope_proxy)
        .insert_resource(cavity)
        .insert_resource(layers)
        .insert_resource(ScanMeshVisible::default())
        .add_systems(Startup, (setup_render_scene, spawn_cavity_mesh).chain())
        .add_systems(
            Update,
            (
                block_orbit_input_when_over_egui.before(orbit_camera_input),
                draw_reference_overlays,
                update_cavity_mesh,
                update_layer_meshes,
                apply_scan_mesh_visibility,
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

    fn approx_eq(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    // ----- Slice 4 v1 — cavity state -------------------------------

    #[test]
    fn cavity_default_inset_is_three_mm() {
        // 3 mm = minimum-acceptable buildable design (≥ Ecoflex's
        // ~2 mm castability threshold + ~10 % radial pre-strain).
        let state = CavityState::default_for_scan();
        assert!(approx_eq(state.inset_m, 0.003, 1e-12));
    }

    #[test]
    fn cavity_inset_slider_range_zero_to_fifteen_mm() {
        let (min_m, max_m) = CavityState::inset_slider_range_m();
        assert!(approx_eq(min_m, 0.0, 1e-12));
        assert!(approx_eq(max_m, 0.015, 1e-12));
    }

    #[test]
    fn default_surfaces_are_visible() {
        // Sanity: launching the app shows the cavity + every
        // default layer. If we ever flip a default to "hidden,"
        // the user would see a near-empty viewport on launch —
        // worth a regression pin.
        let cv = CavityState::default_for_scan();
        let ls = LayersState::default_for_scan();
        assert!(cv.visible);
        assert!(ls.layers.iter().all(|l| l.visible));
    }

    // ----- Slice 5 v1 — layers state -------------------------------

    #[test]
    fn layers_default_is_single_visible_ecoflex_layer() {
        let layers = LayersState::default_for_scan();
        assert_eq!(layers.layers.len(), 1);
        assert_eq!(layers.layers[0].material_anchor_key, "ECOFLEX_00_30");
        assert!(approx_eq(layers.layers[0].thickness_m, 0.005, 1e-12));
        assert!(layers.layers[0].visible);
    }

    #[test]
    fn layer_count_max_is_six() {
        // Workshop-cap pin: more than 6 layers makes the panel
        // unwieldy and the per-frame per-layer surface-mesh
        // regeneration cost climb. Pinning so a future "let me have
        // 100 layers" accident is caught.
        assert_eq!(LAYER_COUNT_MAX, 6);
    }

    #[test]
    fn layer_material_catalog_covers_all_silicone_anchor_keys() {
        // Mirrors cf-cast's cure-table anchor keys; verify count +
        // each entry name to catch typos / drift from cf-cast's
        // table. If cf-cast ever adds / removes a silicone, this
        // test fails and we update both sides.
        let keys: Vec<&str> = LAYER_MATERIALS.iter().map(|(k, _, _)| *k).collect();
        assert_eq!(keys.len(), 8);
        assert!(keys.contains(&"ECOFLEX_00_10"));
        assert!(keys.contains(&"ECOFLEX_00_20"));
        assert!(keys.contains(&"ECOFLEX_00_30"));
        assert!(keys.contains(&"ECOFLEX_00_50"));
        assert!(keys.contains(&"DRAGON_SKIN_10A"));
        assert!(keys.contains(&"DRAGON_SKIN_15"));
        assert!(keys.contains(&"DRAGON_SKIN_20A"));
        assert!(keys.contains(&"DRAGON_SKIN_30A"));
    }

    #[test]
    fn layer_material_densities_mirror_silicone_table() {
        // Densities mirror `sim-soft`'s `silicone_table.rs` anchor
        // values by-name. If sim-soft revises an anchor density, this
        // pin fails and we update the catalog to match.
        assert!(approx_eq(material_density("ECOFLEX_00_10"), 1040.0, 1e-9));
        assert!(approx_eq(material_density("ECOFLEX_00_20"), 1070.0, 1e-9));
        assert!(approx_eq(material_density("ECOFLEX_00_30"), 1070.0, 1e-9));
        assert!(approx_eq(material_density("ECOFLEX_00_50"), 1070.0, 1e-9));
        assert!(approx_eq(material_density("DRAGON_SKIN_10A"), 1070.0, 1e-9));
        assert!(approx_eq(material_density("DRAGON_SKIN_15"), 1070.0, 1e-9));
        assert!(approx_eq(material_density("DRAGON_SKIN_20A"), 1080.0, 1e-9));
        assert!(approx_eq(material_density("DRAGON_SKIN_30A"), 1080.0, 1e-9));
        // Every catalog density lands in the silicone band.
        for (key, _, density) in LAYER_MATERIALS {
            assert!(
                (1040.0..=1080.0).contains(density),
                "{key}: density {density} kg/m³ outside silicone band",
            );
        }
        // Unknown key → defensive median fallback.
        assert!(approx_eq(material_density("NOT_A_REAL_KEY"), 1070.0, 1e-9));
    }

    #[test]
    fn layer_position_labels() {
        assert_eq!(layer_position_label(0, 1), "single");
        assert_eq!(layer_position_label(0, 3), "innermost");
        assert_eq!(layer_position_label(1, 3), "middle");
        assert_eq!(layer_position_label(2, 3), "outermost");
        assert_eq!(layer_position_label(5, 6), "outermost");
    }

    #[test]
    fn layer_thickness_slider_range_one_to_twenty_mm() {
        let (min_m, max_m) = LayerSpec::thickness_slider_range_m();
        assert!(approx_eq(min_m, 0.001, 1e-12));
        assert!(approx_eq(max_m, 0.020, 1e-12));
    }

    #[test]
    fn default_scan_mesh_visible() {
        // Sanity pin: the scan mesh is shown by default. The user
        // can hide it via the Scan Info panel checkbox to inspect
        // the cavity, which sits INSIDE the scan and is occluded by
        // it when both are drawn.
        assert!(ScanMeshVisible::default().0);
    }

    // ----- Slice 3 — decimated scan proxy --------------------------

    #[test]
    fn envelope_proxy_unit_cube_yields_decimated_mesh_with_radials() {
        // Unit cube fixture has 12 faces; with
        // ENVELOPE_PROXY_TARGET_FACES = 1500 the sloppy decimator
        // skips decimation (already under target). Proxy should
        // match the input face count, weld to 8 shared vertices,
        // and expose one unit-length radial per vertex.
        let mesh = unit_cube_mesh();
        let proxy = compute_envelope_proxy_mesh(&mesh, &[]);
        assert_eq!(proxy.original_face_count, 12);
        assert_eq!(proxy.faces.len(), 12);
        // After welding ±0.05 cube vertices match exactly; 8 shared.
        assert_eq!(proxy.vertices.len(), 8);
        assert_eq!(proxy.vertex_radial_directions.len(), proxy.vertices.len());
        for r in &proxy.vertex_radial_directions {
            assert!(
                approx_eq(r.norm(), 1.0, 1e-9),
                "radial not unit-length: |r| = {}",
                r.norm()
            );
        }
    }

    #[test]
    fn envelope_proxy_empty_mesh_yields_empty_proxy() {
        let mesh = IndexedMesh::new();
        let proxy = compute_envelope_proxy_mesh(&mesh, &[]);
        assert_eq!(proxy.original_face_count, 0);
        assert!(proxy.vertices.is_empty());
        assert!(proxy.vertex_radial_directions.is_empty());
        assert!(proxy.faces.is_empty());
    }

    #[test]
    fn envelope_proxy_radials_horizontal_when_vertices_align_with_centerline() {
        // Cube fixture spanning Z ∈ [-0.05, 0.05] with a Z-aligned
        // centerline at the same Z extents. Every cube vertex sits
        // at the SAME Z as its nearest centerline point, so
        // `(v - centerline_pt)` has zero Z component — radial is
        // purely horizontal on every vertex regardless of the
        // below-/above-centerline branch. Unit length on every path.
        let mesh = unit_cube_mesh();
        let centerline = vec![
            Point3::new(0.0, 0.0, -0.05),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.05),
        ];
        let proxy = compute_envelope_proxy_mesh(&mesh, &centerline);
        for r in &proxy.vertex_radial_directions {
            assert!(
                r.z.abs() < 1e-9,
                "radial {:?} has nonzero Z (vertex aligns with centerline Z)",
                r
            );
            assert!(
                (r.norm() - 1.0).abs() < 1e-9,
                "radial {:?} not unit length",
                r
            );
        }
    }

    #[test]
    fn envelope_proxy_dome_vertices_get_vertical_radial_component() {
        // A vertex ABOVE the centerline's Z-range must get a radial
        // with a positive Z component (full 3D radial) so the dome
        // grows outward AND upward. Build a 2-point Z centerline
        // capped at z = 0.0; a single vertex at (0, 0, 0.1) is
        // directly above the top centerline point. Its radial must
        // be +Z (the dome-growth direction).
        //
        // We can't easily inject a custom proxy mesh, so this test
        // exercises the radial branch logic directly: with
        // centerline_min_z = -0.05 and a vertex at z = 0.1 (above
        // it), the diff (0, 0, 0.1) - (0, 0, 0.0) = (0, 0, 0.1)
        // normalizes to +Z.
        //
        // The unit cube proxy doesn't have a vertex above its
        // centerline's max, so this is verified via a small
        // synthetic single-triangle mesh whose apex sits above the
        // centerline.
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(-0.01, -0.01, -0.05)); // base rim
        mesh.vertices.push(Point3::new(0.01, -0.01, -0.05));
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.10)); // apex, above centerline
        mesh.faces.push([0, 1, 2]);
        let centerline = vec![Point3::new(0.0, 0.0, -0.05), Point3::new(0.0, 0.0, 0.0)];
        let proxy = compute_envelope_proxy_mesh(&mesh, &centerline);
        // The apex vertex (z = 0.10, above centerline_max) must have
        // a radial with a positive Z component.
        let apex_idx = proxy
            .vertices
            .iter()
            .position(|v| v.z > 0.05)
            .expect("apex vertex should survive decimation");
        assert!(
            proxy.vertex_radial_directions[apex_idx].z > 0.1,
            "apex radial {:?} should have a clear +Z component for dome growth",
            proxy.vertex_radial_directions[apex_idx],
        );
    }

    #[test]
    fn envelope_proxy_no_centerline_yields_horizontal_radials() {
        // Without a centerline (< 2 points), the radial directions
        // are still purely horizontal — derived from the per-vertex
        // normals with Z stripped + renormalized. Pinning this
        // guarantees no axial Z bleeds into displacement regardless
        // of centerline presence.
        let mesh = unit_cube_mesh();
        let proxy = compute_envelope_proxy_mesh(&mesh, &[]);
        assert_eq!(proxy.vertex_radial_directions.len(), proxy.vertices.len());
        for r in &proxy.vertex_radial_directions {
            assert!(r.z.abs() < 1e-9, "radial {:?} has nonzero Z component", r);
            assert!(
                (r.norm() - 1.0).abs() < 1e-9,
                "radial {:?} not unit length",
                r
            );
        }
    }

    /// `Aabb` re-export from `mesh_types` (transitive from
    /// `cf_geometry`) is the canonical aabb type the suite consumes
    /// for the scan AABB readouts + render-scale sizing. Pin
    /// compile-time visibility so a future workspace dep shuffle
    /// doesn't silently break the loader.
    #[test]
    fn aabb_canonical_type_in_scope() {
        use mesh_types::Aabb;
        let a = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        assert!((a.diagonal() - (3.0_f64).sqrt()).abs() < 1e-12);
    }

    // ----- Slice 6 — geometric validations -------------------------

    /// Z-aligned centerline spanning the unit cube fixture's Z
    /// extents — shared by the slice-6 validation tests so every
    /// cube vertex gets a horizontal full-3D radial.
    fn cube_centerline() -> Vec<Point3<f64>> {
        vec![
            Point3::new(0.0, 0.0, -0.05),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.05),
        ]
    }

    #[test]
    fn signed_volume_recovers_unit_cube_volume_at_zero_offset() {
        // The unit cube fixture is 0.1 m on a side → 1e-3 m³. At
        // offset 0 the proxy vertices are undisplaced, so the
        // divergence-theorem volume must recover the cube volume
        // (positive — cf-scan-prep meshes wind CCW / outward).
        let proxy = compute_envelope_proxy_mesh(&unit_cube_mesh(), &cube_centerline());
        let vol = signed_volume_m3(&proxy, 0.0);
        assert!(
            approx_eq(vol, 1.0e-3, 1e-9),
            "unit-cube signed volume = {vol}, expected 1e-3",
        );
    }

    #[test]
    fn signed_volume_is_monotonic_in_offset() {
        // Displacing every vertex outward along its radial enlarges
        // the enclosed volume; inward shrinks it. Pins both the sign
        // convention and that the radial displacement grows the
        // surface the way the cavity / layer meshes assume.
        let proxy = compute_envelope_proxy_mesh(&unit_cube_mesh(), &cube_centerline());
        let inward = signed_volume_m3(&proxy, -0.01);
        let zero = signed_volume_m3(&proxy, 0.0);
        let outward = signed_volume_m3(&proxy, 0.01);
        assert!(
            inward < zero && zero < outward,
            "expected inward ({inward}) < zero ({zero}) < outward ({outward})",
        );
    }

    #[test]
    fn grade_budget_thresholds() {
        // Green below 0.8× budget, yellow in [0.8×, 1.0×], red above.
        assert_eq!(grade_budget(0.1), BudgetStatus::Green);
        assert_eq!(grade_budget(MASS_BUDGET_KG * 0.5), BudgetStatus::Green);
        assert_eq!(grade_budget(MASS_BUDGET_KG * 0.8), BudgetStatus::Yellow);
        assert_eq!(grade_budget(MASS_BUDGET_KG * 0.99), BudgetStatus::Yellow);
        assert_eq!(grade_budget(MASS_BUDGET_KG), BudgetStatus::Yellow);
        assert_eq!(grade_budget(MASS_BUDGET_KG * 1.01), BudgetStatus::Red);
    }

    #[test]
    fn budget_status_orders_by_severity() {
        // `worst_status` relies on the derived `Ord` matching the
        // severity order Green < Yellow < Red.
        assert!(BudgetStatus::Green < BudgetStatus::Yellow);
        assert!(BudgetStatus::Yellow < BudgetStatus::Red);
        assert_eq!(
            [BudgetStatus::Green, BudgetStatus::Red, BudgetStatus::Yellow]
                .into_iter()
                .max(),
            Some(BudgetStatus::Red),
        );
    }

    #[test]
    fn compute_validations_one_entry_per_layer_with_positive_shells() {
        // Two-layer stack on the cube proxy: one validation per
        // layer, every shell volume + mass strictly positive, and
        // the total mass is the sum of the per-layer pour masses.
        let proxy = compute_envelope_proxy_mesh(&unit_cube_mesh(), &cube_centerline());
        let cavity = CavityState {
            inset_m: 0.003,
            visible: true,
        };
        let layers = LayersState {
            layers: vec![
                LayerSpec {
                    thickness_m: 0.005,
                    material_anchor_key: "ECOFLEX_00_30",
                    slacker_fraction: 0.0,
                    visible: true,
                },
                LayerSpec {
                    thickness_m: 0.004,
                    material_anchor_key: "DRAGON_SKIN_20A",
                    slacker_fraction: 0.0,
                    visible: true,
                },
            ],
        };
        let v = compute_validations(&proxy, &cavity, &layers);
        assert_eq!(v.layers.len(), 2);
        for lv in &v.layers {
            assert!(lv.shell_volume_m3 > 0.0, "shell volume must be positive");
            assert!(lv.pour_mass_kg > 0.0, "pour mass must be positive");
        }
        let summed: f64 = v.layers.iter().map(|lv| lv.pour_mass_kg).sum();
        assert!(
            approx_eq(v.total_mass_kg, summed, 1e-12),
            "total {} != sum of layers {}",
            v.total_mass_kg,
            summed,
        );
        // Pour mass uses the per-layer material density.
        let l1 = &v.layers[1];
        assert!(approx_eq(
            l1.pour_mass_kg,
            l1.shell_volume_m3 * material_density("DRAGON_SKIN_20A"),
            1e-12,
        ));
    }

    #[test]
    fn compute_validations_flags_cavity_self_intersection() {
        // Below the proxy's min radial distance the cavity is fine;
        // at or above it the cavity mesh self-intersects.
        let proxy = compute_envelope_proxy_mesh(&unit_cube_mesh(), &cube_centerline());
        let collapse = proxy.min_radial_distance_m;
        assert!(collapse.is_finite() && collapse > 0.0);
        let layers = LayersState::default_for_scan();

        let safe = compute_validations(
            &proxy,
            &CavityState {
                inset_m: collapse * 0.5,
                visible: true,
            },
            &layers,
        );
        assert!(!safe.cavity_self_intersects);
        assert!(approx_eq(safe.cavity_collapse_inset_m, collapse, 1e-12));

        let collapsed = compute_validations(
            &proxy,
            &CavityState {
                inset_m: collapse * 1.5,
                visible: true,
            },
            &layers,
        );
        assert!(collapsed.cavity_self_intersects);
    }

    #[test]
    fn compute_validations_cavity_check_not_applicable_without_centerline() {
        // No centerline → min_radial_distance_m is INFINITY → the
        // cavity self-intersection check can never fire.
        let proxy = compute_envelope_proxy_mesh(&unit_cube_mesh(), &[]);
        assert!(proxy.min_radial_distance_m.is_infinite());
        let v = compute_validations(
            &proxy,
            &CavityState {
                inset_m: 0.5,
                visible: true,
            },
            &LayersState::default_for_scan(),
        );
        assert!(!v.cavity_self_intersects);
        assert!(v.cavity_collapse_inset_m.is_infinite());
    }

    #[test]
    fn compute_validations_min_wall_flags_sub_floor_layers() {
        let proxy = compute_envelope_proxy_mesh(&unit_cube_mesh(), &cube_centerline());
        let cavity = CavityState {
            inset_m: 0.003,
            visible: true,
        };
        // A 1 mm layer is below the 2 mm castability floor.
        let thin = LayersState {
            layers: vec![LayerSpec {
                thickness_m: 0.001,
                material_anchor_key: "ECOFLEX_00_30",
                slacker_fraction: 0.0,
                visible: true,
            }],
        };
        let v_thin = compute_validations(&proxy, &cavity, &thin);
        assert!(!v_thin.min_wall_ok);
        assert!(!v_thin.layers[0].thickness_castable);

        // A 3 mm layer clears the floor.
        let thick = LayersState {
            layers: vec![LayerSpec {
                thickness_m: 0.003,
                material_anchor_key: "ECOFLEX_00_30",
                slacker_fraction: 0.0,
                visible: true,
            }],
        };
        let v_thick = compute_validations(&proxy, &cavity, &thick);
        assert!(v_thick.min_wall_ok);
        assert!(v_thick.layers[0].thickness_castable);
    }

    #[test]
    fn compute_validations_worst_status_is_the_max_layer_status() {
        // `worst_status` must equal the most-severe per-layer grade.
        let proxy = compute_envelope_proxy_mesh(&unit_cube_mesh(), &cube_centerline());
        let cavity = CavityState {
            inset_m: 0.003,
            visible: true,
        };
        let layers = LayersState {
            layers: vec![
                LayerSpec {
                    thickness_m: 0.005,
                    material_anchor_key: "ECOFLEX_00_30",
                    slacker_fraction: 0.0,
                    visible: true,
                },
                LayerSpec {
                    thickness_m: 0.012,
                    material_anchor_key: "DRAGON_SKIN_30A",
                    slacker_fraction: 0.0,
                    visible: true,
                },
            ],
        };
        let v = compute_validations(&proxy, &cavity, &layers);
        let expected = v
            .layers
            .iter()
            .map(|lv| lv.status)
            .max()
            .unwrap_or(BudgetStatus::Green);
        assert_eq!(v.worst_status, expected);
    }

    #[test]
    fn mass_budget_is_two_pounds_to_kg_exact() {
        // Mirrors cf-cast's `DEFAULT_MASS_BUDGET_KG` by-value: 2 lb
        // via the NIST exact conversion 1 lb = 0.453_592_37 kg.
        let lb_to_kg = 0.453_592_37_f64;
        assert!((MASS_BUDGET_KG - (lb_to_kg + lb_to_kg)).abs() < f64::EPSILON);
    }

    // ----- Slice 6.5 — per-layer Slacker recipe --------------------

    #[test]
    fn default_layer_has_no_slacker() {
        // A fresh layer is plain base silicone — no Slacker.
        let layers = LayersState::default_for_scan();
        assert!(approx_eq(layers.layers[0].slacker_fraction, 0.0, 1e-12));
    }

    #[test]
    fn resolve_slacker_fraction_keeps_on_curve_values() {
        // A fraction that IS a data point on the base material's
        // curve passes through unchanged.
        assert!(approx_eq(
            resolve_slacker_fraction("ECOFLEX_00_30", 0.25),
            0.25,
            1e-12,
        ));
        assert!(approx_eq(
            resolve_slacker_fraction("DRAGON_SKIN_10A", 0.75),
            0.75,
            1e-12,
        ));
        // 0.0 (the native point) is on every curve.
        assert!(approx_eq(
            resolve_slacker_fraction("ECOFLEX_00_30", 0.0),
            0.0,
            1e-12,
        ));
    }

    #[test]
    fn resolve_slacker_fraction_resets_orphaned_values() {
        // 0.75 is on the Dragon Skin 10A curve but NOT the Ecoflex
        // curves (Ecoflex caps at 0.50) — switching base material
        // to an Ecoflex orphans it, so it resets to 0.0.
        assert!(approx_eq(
            resolve_slacker_fraction("ECOFLEX_00_30", 0.75),
            0.0,
            1e-12,
        ));
        // An arbitrary off-curve value resets too.
        assert!(approx_eq(
            resolve_slacker_fraction("ECOFLEX_00_30", 0.123),
            0.0,
            1e-12,
        ));
        // Bases with no Slacker curve always resolve to 0.0.
        assert!(approx_eq(
            resolve_slacker_fraction("ECOFLEX_00_10", 0.25),
            0.0,
            1e-12,
        ));
        assert!(approx_eq(
            resolve_slacker_fraction("DRAGON_SKIN_15", 0.25),
            0.0,
            1e-12,
        ));
    }

    #[test]
    fn slacker_recipe_grams_splits_pour_mass_by_ratio() {
        // No Slacker: the pour is pure base silicone, 1:1 A:B.
        let (ab, slk) = slacker_recipe_grams(0.200, 0.0);
        assert!(approx_eq(ab, 100.0, 1e-9), "part A/B = {ab}");
        assert!(approx_eq(slk, 0.0, 1e-9), "slacker = {slk}");

        // 50 % Slacker on a 300 g pour → 100 g A + 100 g B + 100 g
        // Slacker (the TB's "100A + 100B + 100 Slacker" row).
        let (ab, slk) = slacker_recipe_grams(0.300, 0.50);
        assert!(approx_eq(ab, 100.0, 1e-9), "part A/B = {ab}");
        assert!(approx_eq(slk, 100.0, 1e-9), "slacker = {slk}");

        // 25 % Slacker: A + B + Slacker must sum back to the pour
        // mass, and Slacker must be 0.25 × the base A+B mass.
        let (ab, slk) = slacker_recipe_grams(0.250, 0.25);
        assert!(approx_eq(2.0 * ab + slk, 250.0, 1e-9));
        assert!(approx_eq(slk, 0.25 * 2.0 * ab, 1e-9));
    }

    #[test]
    fn slacker_point_label_distinguishes_native_from_slacker_points() {
        // The native (0.0) point reads "No Slacker"; a Slacker
        // point carries its percentage, hardness, and tack.
        let slacker::Support::Curve(curve) = slacker::support("ECOFLEX_00_30") else {
            unreachable!("Ecoflex 00-30 has a Slacker curve");
        };
        assert_eq!(slacker_point_label(&curve[0]), "No Slacker — Shore 00-30",);
        assert_eq!(
            slacker_point_label(&curve[1]),
            "+25% Slacker — Shore 000-40 (slight-to-very tack)",
        );
    }
}
