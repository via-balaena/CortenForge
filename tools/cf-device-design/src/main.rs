//! cf-device-design — layered-silicone-device design + testing suite
//! (`docs/ENGINEERING_SUITE_DESIGN.md`).
//!
//! Slices shipped:
//! - Slice 2: crate scaffold + STL/prep load + centerline overlay +
//!   Scan Info panel.
//! - Slice 3: Outer Envelope panel + curve-following wireframe overlay
//!   (radius slider, parallel-transport bases at each centerline point,
//!   gizmo rings + longitudinal lines).
//!
//! Pending slices: Cavity / Layers / Features / Validations / Health /
//! Save / Open. See `docs/ENGINEERING_SUITE_DESIGN.md` for the full
//! ladder.

use std::collections::HashSet;
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
use meshopt::{SimplifyOptions, simplify_decoder};
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

/// Outer-envelope panel state — the user-dialed `clearance_m`
/// between the scan surface and the envelope. The envelope IS the
/// scan surface offset outward by clearance (slice 3 polish 10
/// pivot from per-arc rings to direct offset-surface).
///
/// No per-end cap toggles: the scan's natural shape (closed dome at
/// one end, cleaned flat cap at the other) drives the offset
/// surface directly.
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
struct OuterEnvelopeState {
    /// Distance (meters) by which the envelope wireframe sits
    /// outside the scan surface, measured along the per-vertex
    /// outward normal of the decimated scan proxy.
    clearance_m: f64,
    /// Whether the outer-envelope wireframe is drawn this frame.
    /// User toggle in the panel; lets them hide the green-blue
    /// surface while inspecting the cavity (or vice versa).
    visible: bool,
}

impl OuterEnvelopeState {
    /// Default state. Clearance defaults to
    /// [`ENVELOPE_DEFAULT_CLEARANCE_M`] (5 mm) so the envelope sits
    /// a visible distance outside the scan surface from launch.
    fn default_for_scan() -> Self {
        Self {
            clearance_m: ENVELOPE_DEFAULT_CLEARANCE_M,
            visible: true,
        }
    }

    /// Slider range for the clearance, in meters.
    fn clearance_slider_range_m() -> (f64, f64) {
        (0.0, 0.050)
    }
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
/// Slice 4 polish 2 will add the insertable-length clip
/// (`docs/ENGINEERING_SUITE_DESIGN.md` § 4) so the cavity covers
/// only the inserted portion, with a tangent-perpendicular end
/// cap.
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
struct CavityState {
    /// Distance (meters) by which the cavity wireframe sits inside
    /// the scan surface, measured along the per-vertex INWARD
    /// normal (the negation of the proxy mesh's outward normal).
    inset_m: f64,
    /// Whether the cavity wireframe is drawn this frame. User
    /// toggle in the panel.
    visible: bool,
}

impl CavityState {
    /// Default state. Inset defaults to
    /// [`CAVITY_DEFAULT_INSET_M`] (0 mm) — cavity coincides with
    /// the scan surface at launch. The user dials the inset
    /// upward (slider right) to shrink the cavity into the scan,
    /// producing the silicone-skin pre-strain.
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

/// Decimated proxy of the cleaned scan, used to render the outer-
/// envelope wireframe as a direct offset surface (slice 3 polish 10
/// — fundamental pivot from centerline-parameterized rings).
///
/// Each frame, the wireframe is the proxy mesh's edges with every
/// vertex DISPLACED outward by `OuterEnvelopeState::clearance_m`
/// along its per-vertex normal. The result IS the scan surface +
/// clearance, by construction — no per-bucket statistics, no
/// smoothing, no parametric capsule. Matches the downstream SDF
/// that cf-cast-cli's slice-9 build (`Solid::from_sdf(scan_sdf)
/// .offset(clearance)`) will produce.
///
/// The proxy is decimated to ~`ENVELOPE_PROXY_TARGET_FACES` (1500
/// faces by default) at startup — the iter-1 fixture's 3.3M-face
/// cleaned mesh would be impossible to render as gizmo lines at
/// full resolution. 1500 faces ≈ 4500 directed edges ≈ 2250 unique
/// edges, well within Bevy's gizmo throughput.
#[derive(Resource, Debug, Clone)]
struct EnvelopeProxyMesh {
    /// Decimated scan vertices in physics-frame meters.
    vertices: Vec<Point3<f64>>,
    /// Per-vertex outward normal (unit length). Computed as the
    /// area-weighted average of incident face normals. Falls back
    /// to `+Z` for vertices with no incident faces.
    vertex_normals: Vec<Vector3<f64>>,
    /// Deduplicated edges. Each `[a, b]` has `a < b`. One edge per
    /// unique (vertex, vertex) pair regardless of how many faces
    /// share it.
    edges: Vec<[u32; 2]>,
    /// Original face count (pre-decimation) — surfaced in the panel
    /// so the user knows the proxy is approximate.
    original_face_count: usize,
}

/// Target face count after decimation for the offset-surface
/// wireframe. 1500 faces is dense enough to show the scan's gross
/// shape (cylinder + dome + cleaned cap), sparse enough that Bevy
/// gizmos render fluidly + the user can see THROUGH the wireframe
/// to the scan.
const ENVELOPE_PROXY_TARGET_FACES: usize = 1500;

/// Weld epsilon (meters) for the pre-decimation vertex weld.
/// Matches cf-scan-prep's `SIMPLIFY_WELD_EPSILON_M = 1e-6` — the
/// cleaned scan's STL load produces 3-per-triangle unshared
/// vertices that meshopt needs welded to find collapsible edges.
const ENVELOPE_PROXY_WELD_EPSILON_M: f64 = 1e-6;

/// `meshopt::simplify_decoder`'s target-error parameter. Much higher
/// than cf-scan-prep's `SIMPLIFY_TARGET_ERROR = 0.05` because we're
/// reducing the iter-1 cleaned scan from 3.3M faces all the way down
/// to ~1500 (a ~2000× reduction). cf-scan-prep's panel typically
/// runs 10–50× reductions where 0.05 (= 5% of mesh diagonal) is
/// enough error budget; at 2000× the budget exhausts long before the
/// face target is reached and decimation stops early (leaving the
/// full mesh in the proxy).
///
/// 10.0 = "1000% of mesh diagonal" → effectively unbounded, lets
/// the decimation run until `target_index_count` is reached. The
/// resulting proxy is approximate (visual only, not measurement);
/// the cast-time SDF builds on the unmodified scan.
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

/// Default clearance (meters) added to the wrap-the-scan envelope
/// radius. 5 mm gives the user a visible gap between the wireframe
/// and the scan at default radius — communicates "the envelope is
/// outside the scan" instead of "envelope sits on the scan
/// surface."
const ENVELOPE_DEFAULT_CLEARANCE_M: f64 = 0.005;

/// Default cavity inset (meters). 0 mm = cavity coincides with the
/// scan surface at launch — the silicone skin has zero pre-strain
/// initially. The user dials the inset up (slider right) to shrink
/// the cavity into the scan; that delta IS the pre-strain. Lets
/// the user start from a known reference (scan-as-cavity) and
/// dial in the stretch they want for a specific silicone /
/// appendage combination.
const CAVITY_DEFAULT_INSET_M: f64 = 0.000;

/// Decimate the cleaned scan to a ~`ENVELOPE_PROXY_TARGET_FACES`-
/// face proxy mesh, compute per-vertex outward normals, and extract
/// deduplicated edges. Returns the proxy struct used by
/// [`draw_envelope_offset_overlay`] to render the scan's offset
/// surface as a wireframe.
///
/// Pipeline:
/// 1. Weld duplicated vertices (STL load produces 3-per-triangle
///    unshared vertices that meshopt can't decimate without
///    shared topology).
/// 2. `meshopt::simplify_decoder` with `LockBorder` (preserves any
///    open boundary edges on the cleaned mesh).
/// 3. Strip unreferenced vertices left over from the decimation.
/// 4. Compute face normals from triangle CCW order, then per-vertex
///    normals as area-weighted averages of incident face normals.
/// 5. Extract unique edges (each face contributes 3 edges; dedup
///    by sorted vertex-index pairs).
///
/// Cost: O(N_orig_faces) for weld + meshopt + O(N_decimated × 3) for
/// normals + edges. For the iter-1 fixture (3.3 M cleaned faces →
/// 1500 proxy faces) this runs in ~5–10 s at startup; one-time.
fn compute_envelope_proxy_mesh(scan_mesh: &IndexedMesh) -> EnvelopeProxyMesh {
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
    // No `LockBorder` — boundary preservation isn't required for a
    // visualization proxy, and locking the cleaned cap's circular
    // boundary loop on the iter-1 fixture starves the decimation of
    // edges it can collapse. Empty options = full mobility.
    let simplified_indices = if target_index_count < indices.len() {
        simplify_decoder(
            &indices,
            &positions,
            target_index_count,
            ENVELOPE_PROXY_SIMPLIFY_TARGET_ERROR,
            SimplifyOptions::empty(),
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

    // Step 5: deduplicated edges. Each face contributes 3 directed
    // edges; we collect them as sorted (min, max) tuples in a
    // HashSet to dedup, then materialize as a Vec.
    let mut edge_set: HashSet<(u32, u32)> = HashSet::new();
    for face in &proxy.faces {
        for (a, b) in [(face[0], face[1]), (face[1], face[2]), (face[2], face[0])] {
            let edge = if a < b { (a, b) } else { (b, a) };
            edge_set.insert(edge);
        }
    }
    let edges: Vec<[u32; 2]> = edge_set.into_iter().map(|(a, b)| [a, b]).collect();

    EnvelopeProxyMesh {
        vertices: proxy.vertices,
        vertex_normals,
        edges,
        original_face_count,
    }
}

/// Draw the outer-envelope wireframe as the scan-proxy's offset
/// surface (slice 3 polish 10 — fundamental pivot away from
/// centerline-parameterized rings).
///
/// Each frame: for every decimated-mesh vertex, displace by
/// `clearance_m` along its outward normal; for every deduplicated
/// edge, draw a gizmo line between the two endpoints' displaced
/// positions (transformed into Bevy frame via the cast-frame
/// `UpAxis::PlusZ` swap + `render_scale` lift).
///
/// The wireframe is the scan's outer surface, plus clearance,
/// everywhere — matches what cf-cast-cli's downstream SDF
/// (`Solid::from_sdf(scan_sdf).offset(clearance_m)`) builds. No
/// parameterization to overshoot; no statistics to overfit.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn draw_envelope_offset_overlay(
    proxy: Res<EnvelopeProxyMesh>,
    state: Res<OuterEnvelopeState>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut gizmos: Gizmos,
) {
    if !state.visible || proxy.vertices.is_empty() || proxy.edges.is_empty() {
        return;
    }
    // Pale green-blue; same color as the prior ring-based wireframe
    // so the visual identity stays consistent across the polish
    // iterations.
    let color = Color::srgb(0.55, 0.85, 0.70);

    // Pre-compute displaced + Bevy-framed positions for every
    // vertex so each edge endpoint reads a cached value (vs
    // recomputing twice per edge).
    let displaced_bevy: Vec<Vec3> = proxy
        .vertices
        .iter()
        .zip(proxy.vertex_normals.iter())
        .map(|(v, n)| {
            let displaced_physics = Point3::from(v.coords + n * state.clearance_m);
            Vec3::from_array(up.to_bevy_point(&displaced_physics)) * render_scale.0
        })
        .collect();

    for edge in &proxy.edges {
        let a = displaced_bevy[edge[0] as usize];
        let b = displaced_bevy[edge[1] as usize];
        gizmos.line(a, b, color);
    }
}

/// Draw the cavity-surface wireframe each frame: same proxy mesh
/// as the outer envelope, but displaced INWARD along each per-
/// vertex normal by `inset_m` (slice 4 v1).
///
/// The cavity is the void the appendage slides into; being SMALLER
/// than the scan means the silicone "skin" between the cavity
/// surface and the scan surface stretches over the appendage to
/// produce the snug fit. Matches what cf-cast-cli's downstream SDF
/// (`Solid::from_sdf(scan_sdf).offset(-inset_m)`) will build.
///
/// Slice 4 v1 limitation: the cavity wireframe covers the WHOLE
/// scan, including the cleaned-cap end. Slice 4 polish 2 will clip
/// to the insertable arc range with a tangent-perpendicular end
/// cap per the `docs/ENGINEERING_SUITE_DESIGN.md` § 4 spec.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn draw_cavity_overlay(
    proxy: Res<EnvelopeProxyMesh>,
    state: Res<CavityState>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut gizmos: Gizmos,
) {
    if !state.visible || proxy.vertices.is_empty() || proxy.edges.is_empty() {
        return;
    }
    // Coral / salmon — distinct from outer envelope's green-blue
    // (Color::srgb(0.55, 0.85, 0.70)) and from scan white / axis
    // arrows / cyan centerline.
    let color = Color::srgb(0.95, 0.55, 0.45);

    // Inward displacement = vertex.coords + signed_inward * inset_m
    // where `signed_inward` points toward the proxy mesh's centroid.
    // Using a centroid-anchored direction (rather than the per-vertex
    // surface normal) is robust against scans where the area-weighted
    // vertex normal happens to flip near boundary edges (cleaned-cap
    // rim) or degenerate-triangle patches: a vertex's "inward" toward
    // the proxy's interior is well-defined as `centroid - v`
    // regardless of triangle winding. The proxy is a closed-ish
    // surface so the centroid lies inside it.
    let centroid: Vector3<f64> = if proxy.vertices.is_empty() {
        Vector3::zeros()
    } else {
        #[allow(clippy::cast_precision_loss)] // count fits in f64.
        let n = proxy.vertices.len() as f64;
        let sum: Vector3<f64> = proxy
            .vertices
            .iter()
            .map(|p| p.coords)
            .fold(Vector3::zeros(), |a, b| a + b);
        sum / n
    };

    let displaced_bevy: Vec<Vec3> = proxy
        .vertices
        .iter()
        .map(|v| {
            let to_centroid = centroid - v.coords;
            let len = to_centroid.norm();
            let inward = if len > 1e-12 {
                to_centroid / len
            } else {
                Vector3::zeros()
            };
            let displaced_physics = Point3::from(v.coords + inward * state.inset_m);
            Vec3::from_array(up.to_bevy_point(&displaced_physics)) * render_scale.0
        })
        .collect();

    for edge in &proxy.edges {
        let a = displaced_bevy[edge[0] as usize];
        let b = displaced_bevy[edge[1] as usize];
        gizmos.line(a, b, color);
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

/// Number of segments around each cross-section ring of the
/// Render the Outer Envelope egui section. Slice 3 polish 10 pivot:
/// the envelope is the cleaned scan's offset surface — a decimated
/// proxy mesh with every vertex displaced outward by `clearance_m`
/// along its per-vertex normal. The user dials the clearance; the
/// surface IS the scan + offset, by construction.
///
/// Also surfaces the decimated proxy's face count so the user knows
/// the visualization is approximate, plus the original face count
/// for context.
fn render_outer_envelope_section(
    ui: &mut egui::Ui,
    state: &mut OuterEnvelopeState,
    proxy: &EnvelopeProxyMesh,
) {
    egui::CollapsingHeader::new("Outer Envelope")
        .default_open(true)
        .show(ui, |ui| {
            let (min_m, max_m) = OuterEnvelopeState::clearance_slider_range_m();
            let mut clearance_mm = state.clearance_m * 1000.0;
            let min_mm = min_m * 1000.0;
            let max_mm = max_m * 1000.0;
            if ui
                .add(egui::Slider::new(&mut clearance_mm, min_mm..=max_mm).text("clearance (mm)"))
                .changed()
            {
                state.clearance_m = clearance_mm * 0.001;
            }
            if proxy.vertices.is_empty() || proxy.edges.is_empty() {
                ui.label("(proxy mesh empty — no surface to render)");
            } else {
                // For closed manifolds Euler's relation gives
                // F ≈ 2E/3 (each face has 3 edges, each edge shared
                // by 2 faces). Approximate face count is for display
                // only; the surface is rendered as edges.
                let proxy_face_count = proxy.edges.len() * 2 / 3;
                ui.label(format!(
                    "Proxy: {} edges (~{} faces of {})",
                    human_count(proxy.edges.len()),
                    human_count(proxy_face_count),
                    human_count(proxy.original_face_count),
                ));
            }
        });
}

/// Render the Cavity egui section (slice 4 v1). The cavity is the
/// scan-derived inner void the appendage slides into; its surface
/// sits `inset_m` inside the scan along per-vertex normals so the
/// silicone shell between cavity and scan stretches over the
/// appendage for a snug fit.
///
/// Slice 4 v1 ships uniform-inset only. Polish 2 will add
/// insertable-length clip + tangent-perpendicular end cap.
fn render_cavity_section(ui: &mut egui::Ui, state: &mut CavityState) {
    egui::CollapsingHeader::new("Cavity")
        .default_open(true)
        .show(ui, |ui| {
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
/// Renders Scan Info + Outer Envelope + Cavity as of slice 4 v1;
/// remaining stubs surface the planned panel order.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn device_design_panel(
    mut contexts: EguiContexts,
    info: Res<ScanInfo>,
    proxy: Res<EnvelopeProxyMesh>,
    mut scan_visible: ResMut<ScanMeshVisible>,
    mut outer_envelope: ResMut<OuterEnvelopeState>,
    mut cavity: ResMut<CavityState>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("cf-device-design-panels")
        .resizable(false)
        .default_width(320.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    render_display_section(ui, &mut scan_visible, &mut outer_envelope, &mut cavity);
                    render_scan_info_section(ui, &info);
                    render_outer_envelope_section(ui, &mut outer_envelope, &proxy);
                    render_cavity_section(ui, &mut cavity);
                    render_panel_stubs(ui);
                });
        });
    Ok(())
}

/// Top-of-panel Display section: three checkboxes consolidating
/// visibility for the scan mesh + outer-envelope wireframe + cavity
/// wireframe. Each entity has independent on/off; defaults are all
/// ON. Lets the user isolate any single artifact for inspection
/// without hunting through individual sections.
fn render_display_section(
    ui: &mut egui::Ui,
    scan_visible: &mut ScanMeshVisible,
    outer_envelope: &mut OuterEnvelopeState,
    cavity: &mut CavityState,
) {
    egui::CollapsingHeader::new("Display")
        .default_open(true)
        .show(ui, |ui| {
            ui.checkbox(&mut scan_visible.0, "Scan mesh (solid)");
            ui.checkbox(&mut outer_envelope.visible, "Outer envelope (wireframe)");
            ui.checkbox(&mut cavity.visible, "Cavity (wireframe)");
        });
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
/// panel names so the user can see the planned layout.
fn render_panel_stubs(ui: &mut egui::Ui) {
    ui.separator();
    for name in [
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
    // Decimate the cleaned scan to a proxy mesh (~1500 faces) +
    // per-vertex outward normals + unique edges. Every frame the
    // outer-envelope wireframe displaces each vertex by the user-
    // dialed clearance and renders the proxy's edges as gizmo
    // lines. One-time cost at startup; ~5–10 s on the iter-1
    // 3.3 M-face fixture, sub-second on small inputs.
    let envelope_proxy = compute_envelope_proxy_mesh(&scan_mesh);
    let outer_envelope = OuterEnvelopeState::default_for_scan();
    let cavity = CavityState::default_for_scan();

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
        .insert_resource(outer_envelope)
        .insert_resource(cavity)
        .insert_resource(ScanMeshVisible::default())
        .add_systems(Startup, setup_render_scene)
        .add_systems(
            Update,
            (
                block_orbit_input_when_over_egui.before(orbit_camera_input),
                draw_reference_overlays,
                draw_envelope_offset_overlay,
                draw_cavity_overlay,
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

    // ----- Slice 3 polish 7 — clearance-only envelope state --------

    #[test]
    fn outer_envelope_default_clearance_is_five_mm() {
        let state = OuterEnvelopeState::default_for_scan();
        assert!(approx_eq(state.clearance_m, 0.005, 1e-12));
    }

    #[test]
    fn outer_envelope_clearance_slider_range_zero_to_fifty_mm() {
        let (min_m, max_m) = OuterEnvelopeState::clearance_slider_range_m();
        assert!(approx_eq(min_m, 0.0, 1e-12));
        assert!(approx_eq(max_m, 0.050, 1e-12));
    }

    // ----- Slice 4 v1 — cavity state -------------------------------

    #[test]
    fn cavity_default_inset_is_zero_mm() {
        let state = CavityState::default_for_scan();
        assert!(approx_eq(state.inset_m, 0.0, 1e-12));
    }

    #[test]
    fn cavity_inset_slider_range_zero_to_fifteen_mm() {
        let (min_m, max_m) = CavityState::inset_slider_range_m();
        assert!(approx_eq(min_m, 0.0, 1e-12));
        assert!(approx_eq(max_m, 0.015, 1e-12));
    }

    #[test]
    fn default_overlays_are_both_visible() {
        // Sanity: launching the app shows both wireframes by
        // default; the user can hide either via the panel
        // checkbox. If we ever flip a default to "hidden," the
        // user would see a black viewport on launch — a real
        // confusion-inducing failure mode worth a regression pin.
        let oe = OuterEnvelopeState::default_for_scan();
        let cv = CavityState::default_for_scan();
        assert!(oe.visible);
        assert!(cv.visible);
    }

    #[test]
    fn default_scan_mesh_visible() {
        // Same posture as the wireframe-visibility defaults: scan
        // is shown by default; user can hide via the Scan Info
        // panel checkbox to inspect the cavity wireframe (which
        // sits INSIDE the scan and is occluded by it).
        assert!(ScanMeshVisible::default().0);
    }

    // ----- Slice 3 polish 10 — offset-surface proxy mesh ------------

    #[test]
    fn envelope_proxy_unit_cube_yields_decimated_mesh_with_normals_and_edges() {
        // Unit cube fixture has 12 faces; with ENVELOPE_PROXY_TARGET_FACES
        // = 1500 the simplify_decoder skips decimation (already
        // under target). Proxy should match input face count, have
        // unit-length normals, and expose 18 unique edges (12 faces
        // × 3 directed edges, deduplicated by sorted vertex-pair).
        let mesh = unit_cube_mesh();
        let proxy = compute_envelope_proxy_mesh(&mesh);
        assert_eq!(proxy.original_face_count, 12);
        // After welding ±0.05 cube vertices match exactly; 8 shared.
        assert_eq!(proxy.vertices.len(), 8);
        assert_eq!(proxy.vertex_normals.len(), proxy.vertices.len());
        // Cube edge count: 12 (one per cube edge). Each cube edge is
        // shared by exactly 2 triangulated faces → dedup leaves 12
        // unique edges, plus the 6 cube-face diagonals from the
        // 2-triangle-per-face winding (one diagonal each) = 18.
        assert_eq!(proxy.edges.len(), 18);
        for n in &proxy.vertex_normals {
            assert!(
                approx_eq(n.norm(), 1.0, 1e-9),
                "vertex normal not unit-length: |n| = {}",
                n.norm()
            );
        }
    }

    #[test]
    fn envelope_proxy_unit_cube_normals_point_outward_from_origin() {
        // Cube vertices are at corners ±0.05; outward normal at
        // each vertex should point AWAY from the origin (each
        // corner's outward normal lies in the same octant as the
        // vertex). Robust check: vertex.coords · normal > 0 at
        // every vertex.
        let mesh = unit_cube_mesh();
        let proxy = compute_envelope_proxy_mesh(&mesh);
        for (v, n) in proxy.vertices.iter().zip(proxy.vertex_normals.iter()) {
            let dot = v.coords.dot(n);
            assert!(
                dot > 0.0,
                "normal at vertex {:?} points inward (v·n = {})",
                v.coords,
                dot
            );
        }
    }

    #[test]
    fn envelope_proxy_edges_are_sorted_and_unique() {
        // Each edge is stored as [a, b] with a < b; the full set is
        // unique. Verifies the HashSet-based dedup + sorted-pair
        // canonicalization in compute_envelope_proxy_mesh.
        let mesh = unit_cube_mesh();
        let proxy = compute_envelope_proxy_mesh(&mesh);
        let mut seen = HashSet::new();
        for edge in &proxy.edges {
            assert!(
                edge[0] < edge[1],
                "edge [{}, {}] not sorted",
                edge[0],
                edge[1]
            );
            assert!(
                seen.insert((edge[0], edge[1])),
                "edge ({}, {}) duplicated",
                edge[0],
                edge[1]
            );
        }
    }

    #[test]
    fn envelope_proxy_empty_mesh_yields_empty_proxy() {
        let mesh = IndexedMesh::new();
        let proxy = compute_envelope_proxy_mesh(&mesh);
        assert_eq!(proxy.original_face_count, 0);
        assert!(proxy.vertices.is_empty());
        assert!(proxy.vertex_normals.is_empty());
        assert!(proxy.edges.is_empty());
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
