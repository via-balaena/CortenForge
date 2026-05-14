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
//!
//! Pending slices: Validations / Features → Texture / Save / Open.
//! See `docs/ENGINEERING_SUITE_DESIGN.md` for the full ladder.

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

/// Whether the scan mesh entity is visible this frame. Toggled by
/// the "Show scan mesh" checkbox in the Scan Info panel. Useful
/// when inspecting the cavity wireframe, which sits INSIDE the
/// scan and is occluded by the scan mesh when both are drawn.
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
    /// threshold, ~10 % radial pre-strain on a typical scan
    /// cross-section, and separated from the outer envelope at
    /// launch (no z-fighting when both sliders sit at 0). User
    /// dials UP for more pre-strain or DOWN to experiment.
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

/// Silicone-material catalog the Layers panel offers. Mirrors the
/// `anchor_key` strings cf-cast's `cure::lookup` resolves at cast
/// time. cf-device-design carries the catalog standalone (no
/// cf-cast dep) so this tool stays composable; cf-cast-cli
/// validates the chosen anchor at design.toml ingest.
const LAYER_MATERIALS: &[(&str, &str)] = &[
    ("ECOFLEX_00_10", "Ecoflex 00-10 (super-soft)"),
    ("ECOFLEX_00_20", "Ecoflex 00-20 (soft)"),
    ("ECOFLEX_00_30", "Ecoflex 00-30 (medium-soft)"),
    ("ECOFLEX_00_50", "Ecoflex 00-50 (firm-soft)"),
    ("DRAGON_SKIN_10A", "Dragon Skin 10A (soft)"),
    ("DRAGON_SKIN_15", "Dragon Skin 15 (medium)"),
    ("DRAGON_SKIN_20A", "Dragon Skin 20A (firm)"),
    ("DRAGON_SKIN_30A", "Dragon Skin 30A (firmest)"),
];

/// Maximum number of concentric silicone layers between the cavity
/// and the outer envelope. 6 is a generous workshop cap; real
/// designs typically use 1–3. Setting a finite cap keeps panel
/// scroll predictable + the per-frame draw cost bounded.
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
    /// constants.
    material_anchor_key: &'static str,
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
/// - Separates the cavity wireframe from the outer envelope at
///   launch, avoiding z-fighting when both sliders sit at 0.
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
    let centerline_min_z = centerline.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
    let vertex_radial_directions: Vec<Vector3<f64>> = if centerline.len() >= 2 {
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
                    raw / len
                } else {
                    // Degenerate (vertex coincident with the
                    // centerline point). Fall back to the per-vertex
                    // normal, Z-stripped for below-centerline
                    // vertices (keep them axially fixed), full 3D
                    // otherwise.
                    let fallback = if below_centerline {
                        Vector3::new(n.x, n.y, 0.0)
                    } else {
                        *n
                    };
                    let len_f = fallback.norm();
                    if len_f > 1e-9 {
                        fallback / len_f
                    } else {
                        Vector3::new(1.0, 0.0, 0.0)
                    }
                }
            })
            .collect()
    } else {
        // No centerline available; use per-vertex normals (with Z
        // stripped) as the radial proxy. Consistent posture with the
        // centerline-present path's below-centerline branch.
        vertex_normals
            .iter()
            .map(|n| {
                let n_xy = Vector3::new(n.x, n.y, 0.0);
                let len = n_xy.norm();
                if len > 1e-9 {
                    n_xy / len
                } else {
                    Vector3::new(1.0, 0.0, 0.0)
                }
            })
            .collect()
    };

    EnvelopeProxyMesh {
        vertices: proxy.vertices,
        vertex_radial_directions,
        faces: proxy.faces,
        original_face_count,
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
/// and a remove button (hidden when only one layer remains). A
/// "+ Add layer" button appends a new outermost layer up to
/// [`LAYER_COUNT_MAX`].
///
/// The device wall total is the sum of layer thicknesses; the
/// outer skin sits at `sum(thickness) - cavity.inset_m` radially
/// from the scan. Those derived figures are shown as read-only
/// labels at the bottom of the section.
///
/// [`update_layer_meshes`] regenerates the per-layer surface mesh
/// entities whenever `LayersState` or `CavityState` mutates.
fn render_layers_section(ui: &mut egui::Ui, layers: &mut LayersState, cavity: &CavityState) {
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
                // New layer starts visible; user can toggle via
                // the per-layer "Show layer" checkbox.
                layers.layers.push(LayerSpec {
                    thickness_m: 0.003,
                    material_anchor_key: "ECOFLEX_00_30",
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
        .find(|(key, _)| *key == *selected)
        .map(|(_, label)| *label)
        .unwrap_or("(unknown)");
    egui::ComboBox::from_id_salt(("layer-material", layer_index))
        .selected_text(current_label)
        .show_ui(ui, |ui| {
            for (key, label) in LAYER_MATERIALS {
                ui.selectable_value(selected, key, *label);
            }
        });
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
/// Renders Scan Info + Cavity + Layers as of slice 5; remaining
/// stubs surface the planned panel order (Validations / Features /
/// Save-Open).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn device_design_panel(
    mut contexts: EguiContexts,
    info: Res<ScanInfo>,
    mut scan_visible: ResMut<ScanMeshVisible>,
    mut cavity: ResMut<CavityState>,
    mut layers: ResMut<LayersState>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("cf-device-design-panels")
        .resizable(false)
        .default_width(320.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    render_scan_info_section(ui, &info, &mut scan_visible);
                    render_cavity_section(ui, &mut cavity);
                    render_layers_section(ui, &mut layers, &cavity);
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
/// panel names so the user can see the planned layout.
fn render_panel_stubs(ui: &mut egui::Ui) {
    ui.separator();
    for name in [
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
        let keys: Vec<&str> = LAYER_MATERIALS.iter().map(|(k, _)| *k).collect();
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
}
