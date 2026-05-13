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
use mesh_types::{Aabb, Bounded, IndexedMesh, Point3};
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
    /// Raw scan AABB in physics-frame meters. Cached at load time so
    /// the Outer Envelope default-radius heuristic can compute the
    /// max perpendicular distance from each centerline point to the
    /// scan's outer extent (= the min envelope radius that wraps
    /// the whole scan + clearance).
    scan_aabb_m: Aabb,
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

/// Bevy resource carrying the cached parallel-transport basis at
/// each centerline point. Computed once at startup so the per-frame
/// outer-envelope wireframe draw doesn't recompute (parallel
/// transport is O(N) and stable, no need to recompute every tick).
///
/// `bases[i] = (b1_i, b2_i)` — two unit vectors perpendicular to
/// the local tangent at centerline point `i`, smoothly propagated
/// from `bases[i-1]` so adjacent rings don't twist relative to
/// each other (parallel-transport convention).
///
/// Empty when [`Centerline`] is empty.
#[derive(Resource, Default, Clone)]
struct CenterlineBases {
    bases: Vec<(Vector3<f64>, Vector3<f64>)>,
}

/// Bevy resource carrying the scan's local cross-section radius at
/// each centerline polyline point. `radii_m[i]` = max perpendicular
/// distance from `centerline_points[i]` to any scan vertex that's
/// closest to that centerline point.
///
/// Computed once at startup ([`compute_scan_local_radii`]). The
/// outer-envelope wireframe uses these directly as ring radii (plus
/// the user-dialed clearance) so the envelope is SNUG to the scan
/// at every cross-section instead of guessing a constant-radius
/// capsule.
///
/// Empty when the centerline is absent (no arc positions to compute
/// radii at).
#[derive(Resource, Default, Clone)]
struct ScanLocalRadii {
    radii_m: Vec<f64>,
}

/// Outer-envelope panel state — the user-dialed constant radius for
/// the curve-following capsule along the scan centerline (per
/// `docs/ENGINEERING_SUITE_DESIGN.md`'s Outer Envelope section).
///
/// Per-end CAP TOGGLES (`closed_at_high_z` / `closed_at_low_z`)
/// control whether each centerline endpoint gets a hemispherical
/// cap (closed dome) or remains open (flat rim, cylinder extends
/// to the centerline endpoint). Defaults match the cf-scan-prep
/// `+Z`-up convention: the higher-Z endpoint is the scan's natural
/// dome (closed); the lower-Z endpoint is the cleaned cap region
/// (open / cylinder rim). User toggles either via the panel
/// checkboxes when the heuristic guesses wrong.
#[derive(Resource, Debug, Clone, Copy, PartialEq)]
struct OuterEnvelopeState {
    /// Clearance (meters) between the scan surface and the
    /// envelope wireframe at every cross-section. The wireframe ring
    /// radius at each centerline polyline point equals
    /// `scan_local_radius[i] + clearance_m`, so the envelope is
    /// programmatically SNUG to the scan everywhere — no parametric
    /// guessing of a constant cylinder radius. Slice 3 polish 7
    /// pivot from the prior "capsule of constant radius" model.
    clearance_m: f64,
    /// Render a hemispherical cap at the centerline endpoint with
    /// the higher Z coordinate. Default `true` — that endpoint is
    /// typically the scan's natural dome.
    closed_at_high_z: bool,
    /// Render a hemispherical cap at the centerline endpoint with
    /// the lower Z coordinate. Default `false` — that endpoint is
    /// typically where cf-scan-prep applied its cleaned-cap
    /// triangulation, i.e. the device's open rim.
    closed_at_low_z: bool,
}

impl OuterEnvelopeState {
    /// Build the slice-2-default starting state. Clearance defaults
    /// to [`ENVELOPE_DEFAULT_CLEARANCE_M`] (5 mm) so the envelope
    /// sits a visible distance outside the scan surface from launch.
    fn default_for_scan() -> Self {
        Self {
            clearance_m: ENVELOPE_DEFAULT_CLEARANCE_M,
            closed_at_high_z: true,
            closed_at_low_z: false,
        }
    }

    /// Slider range for the clearance, in meters. Min is zero
    /// (envelope sits ON the scan); max is generous so the user can
    /// dial a thick wrap if needed for thick-wall devices.
    fn clearance_slider_range_m() -> (f64, f64) {
        (0.0, 0.050)
    }
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

/// Cumulative arc length at each centerline polyline point.
/// `arc_positions[0] = 0.0`; `arc_positions[N-1] = total arc length`.
/// Empty for polylines with fewer than 2 points (no arc to measure).
fn cumulative_arc_positions_m(points: &[Point3<f64>]) -> Vec<f64> {
    if points.len() < 2 {
        return Vec::new();
    }
    let mut arc = Vec::with_capacity(points.len());
    arc.push(0.0);
    for pair in points.windows(2) {
        let segment_len = (pair[1] - pair[0]).norm();
        let last = arc[arc.len() - 1];
        arc.push(last + segment_len);
    }
    arc
}

/// Sample the centerline polyline at a specific arc-length position
/// (in meters from the start). Returns `(position, unit_tangent)` —
/// position is the linear interpolant between adjacent polyline
/// points; tangent is the unit direction of the surrounding segment
/// (piecewise constant within a segment).
///
/// `arc_positions` must be the result of [`cumulative_arc_positions_m`]
/// on the same polyline. `target_arc` is clamped to the valid range
/// `[0.0, arc_positions.last()]` for defensive behavior — out-of-range
/// asks default to the nearest endpoint.
///
/// Returns `None` for polylines with fewer than 2 points (no valid
/// arc parametrization).
fn sample_centerline_at_arc(
    points: &[Point3<f64>],
    arc_positions: &[f64],
    target_arc: f64,
) -> Option<(Point3<f64>, Vector3<f64>)> {
    if points.len() < 2 || arc_positions.len() != points.len() {
        return None;
    }
    let total = *arc_positions.last()?;
    let clamped = target_arc.clamp(0.0, total);
    // Find the segment containing `clamped`. Linear search is fine for
    // 30-point polylines; binary search if profiling shows hot spots.
    for i in 0..points.len() - 1 {
        let a_lo = arc_positions[i];
        let a_hi = arc_positions[i + 1];
        if clamped <= a_hi {
            let segment_len = a_hi - a_lo;
            let t = if segment_len < 1e-12 {
                0.0
            } else {
                (clamped - a_lo) / segment_len
            };
            let pos = Point3::from(points[i].coords + (points[i + 1] - points[i]) * t);
            let tangent_raw = points[i + 1] - points[i];
            let tangent_norm = tangent_raw.norm();
            let tangent = if tangent_norm < 1e-12 {
                Vector3::new(0.0, 0.0, 1.0)
            } else {
                tangent_raw / tangent_norm
            };
            return Some((pos, tangent));
        }
    }
    // Fall-through: `clamped == total`, return the last endpoint.
    let last = points.len() - 1;
    let tangent_raw = points[last] - points[last - 1];
    let tangent_norm = tangent_raw.norm();
    let tangent = if tangent_norm < 1e-12 {
        Vector3::new(0.0, 0.0, 1.0)
    } else {
        tangent_raw / tangent_norm
    };
    Some((points[last], tangent))
}

/// Build a `(b1, b2)` orthonormal basis perpendicular to `tangent` at
/// a sampled centerline arc position, anchored to the cached
/// parallel-transport bases at the surrounding polyline points.
/// Uses the basis at the LOWER-arc polyline point as the propagation
/// seed, then re-projects perpendicular to `tangent` (handles the
/// case where the interpolated tangent differs slightly from the
/// surrounding polyline tangents).
///
/// Falls back to [`initial_basis_b1`] if the surrounding-point basis
/// becomes degenerate after projection.
fn sample_basis_at_arc(
    bases: &[(Vector3<f64>, Vector3<f64>)],
    arc_positions: &[f64],
    target_arc: f64,
    tangent: &Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    if bases.is_empty() || arc_positions.len() != bases.len() {
        let b1 = initial_basis_b1(tangent);
        let b2 = tangent.cross(&b1).normalize();
        return (b1, b2);
    }
    let total = *arc_positions.last().unwrap_or(&0.0);
    let clamped = target_arc.clamp(0.0, total);
    // Find the lower-arc anchor index.
    let mut anchor = 0_usize;
    for i in 0..bases.len() - 1 {
        if clamped >= arc_positions[i] && clamped <= arc_positions[i + 1] {
            anchor = i;
            break;
        }
    }
    let (prev_b1, _) = bases[anchor];
    let b1 = parallel_transport_b1(prev_b1, tangent);
    let b2 = tangent.cross(&b1).normalize();
    (b1, b2)
}

/// Half-window width (in polyline-index units) used to smooth the
/// local-tangent estimate at each centerline point. The tangent at
/// index `i` is the unit vector from `points[max(0, i - W)]` to
/// `points[min(N - 1, i + W)]`, where `W = TANGENT_SMOOTHING_HALF_WINDOW`.
/// At `W = 2` we average over 5 consecutive points — wide enough to
/// suppress single-sample jitter near the cleaned-scan cap (a real
/// failure mode seen at iter-1 visual review on
/// `sock_over_capsule.cleaned.stl`), narrow enough to track 30°
/// curves cleanly on the 30-point iter-1 polyline.
const TANGENT_SMOOTHING_HALF_WINDOW: usize = 2;

/// Compute the local unit tangent at `points[i]` using a windowed
/// finite difference. Endpoints clamp to the polyline range
/// (effectively a forward / backward difference of up to
/// `TANGENT_SMOOTHING_HALF_WINDOW` steps); interior points use a
/// `2 × TANGENT_SMOOTHING_HALF_WINDOW + 1`-point central difference.
///
/// The smoothing was added to slice 3 polish after iter-1 visual
/// review surfaced "crazy rings" at the bottom of the
/// `sock_over_capsule` envelope — adjacent centerline points there
/// have Y direction flips that produced ~30° tangent jitter and
/// tilted the rings wildly. 5-point smoothing averages the noise
/// out without smearing real curvature.
///
/// Falls back to `+Z` when the polyline is degenerate (consecutive
/// coincident points within the window).
fn local_tangent(points: &[Point3<f64>], i: usize) -> Vector3<f64> {
    let len = points.len();
    if len < 2 {
        return Vector3::new(0.0, 0.0, 1.0);
    }
    let lo = i.saturating_sub(TANGENT_SMOOTHING_HALF_WINDOW);
    let hi = (i + TANGENT_SMOOTHING_HALF_WINDOW).min(len - 1);
    let raw = points[hi] - points[lo];
    let norm = raw.norm();
    if norm < 1e-12 {
        Vector3::new(0.0, 0.0, 1.0)
    } else {
        raw / norm
    }
}

/// Pick an initial perpendicular vector to `tangent` for the first
/// centerline point's basis. Uses `+Z` as the candidate, swaps to
/// `+X` when `tangent` is nearly parallel to `+Z` (so the
/// Gram-Schmidt projection doesn't collapse to a zero vector).
fn initial_basis_b1(tangent: &Vector3<f64>) -> Vector3<f64> {
    let world_z = Vector3::new(0.0, 0.0, 1.0);
    let candidate = if tangent.dot(&world_z).abs() > 0.9 {
        Vector3::new(1.0, 0.0, 0.0)
    } else {
        world_z
    };
    let projected = candidate - tangent * candidate.dot(tangent);
    let norm = projected.norm();
    if norm < 1e-12 {
        // Extremely defensive; shouldn't reach here given the
        // candidate-swap above.
        Vector3::new(0.0, 1.0, 0.0)
    } else {
        projected / norm
    }
}

/// Parallel-transport an existing basis vector `prev_b1` to the new
/// tangent direction. Projects `prev_b1` onto the plane perpendicular
/// to `tangent` and normalizes. Smooth across consecutive ring frames
/// — adjacent rings don't twist relative to each other, which keeps
/// the longitudinal lines straight along the device.
fn parallel_transport_b1(prev_b1: Vector3<f64>, tangent: &Vector3<f64>) -> Vector3<f64> {
    let projected = prev_b1 - tangent * prev_b1.dot(tangent);
    let norm = projected.norm();
    if norm < 1e-12 {
        initial_basis_b1(tangent)
    } else {
        projected / norm
    }
}

/// Compute the parallel-transport (b1, b2) basis at every centerline
/// point. b1 starts perpendicular to the first tangent; each
/// subsequent b1 is the previous b1 projected perpendicular to the
/// new tangent. b2 = tangent × b1 (right-hand rule), giving a
/// right-handed orthonormal basis at every point.
///
/// Returns an empty vec for polylines with fewer than 2 points.
fn compute_centerline_bases(points: &[Point3<f64>]) -> Vec<(Vector3<f64>, Vector3<f64>)> {
    if points.len() < 2 {
        return Vec::new();
    }
    let mut bases = Vec::with_capacity(points.len());
    let mut prev_b1: Option<Vector3<f64>> = None;
    for i in 0..points.len() {
        let tangent = local_tangent(points, i);
        let b1 = match prev_b1 {
            Some(b) => parallel_transport_b1(b, &tangent),
            None => initial_basis_b1(&tangent),
        };
        let b2 = tangent.cross(&b1).normalize();
        bases.push((b1, b2));
        prev_b1 = Some(b1);
    }
    bases
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
        scan_aabb_m: aabb,
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

/// Estimate the scan's extent past a centerline endpoint along an
/// outward direction. For each AABB corner, project `(corner -
/// endpoint)` onto `outward` and keep the max non-negative
/// projection. Returns 0.0 when no corner is forward of the
/// endpoint (degenerate; defensive).
///
/// Used by [`draw_outer_envelope_overlay`] to size each closed
/// end's ellipsoidal cap to the scan's actual dome height —
/// addresses iter-1 visual review feedback "the tip of the rings
/// taper too hard relative to the geometry at the tip" when the
/// hemispherical-cap height = R was much larger than the scan's
/// natural dome.
fn scan_extent_past_endpoint(
    endpoint: &Point3<f64>,
    outward: &Vector3<f64>,
    scan_aabb: &Aabb,
) -> f64 {
    let corners = [
        Point3::new(scan_aabb.min.x, scan_aabb.min.y, scan_aabb.min.z),
        Point3::new(scan_aabb.max.x, scan_aabb.min.y, scan_aabb.min.z),
        Point3::new(scan_aabb.min.x, scan_aabb.max.y, scan_aabb.min.z),
        Point3::new(scan_aabb.max.x, scan_aabb.max.y, scan_aabb.min.z),
        Point3::new(scan_aabb.min.x, scan_aabb.min.y, scan_aabb.max.z),
        Point3::new(scan_aabb.max.x, scan_aabb.min.y, scan_aabb.max.z),
        Point3::new(scan_aabb.min.x, scan_aabb.max.y, scan_aabb.max.z),
        Point3::new(scan_aabb.max.x, scan_aabb.max.y, scan_aabb.max.z),
    ];
    let mut max_extent = 0.0_f64;
    for c in &corners {
        let projection = (c - endpoint).dot(outward);
        if projection > max_extent {
            max_extent = projection;
        }
    }
    max_extent
}

/// Percentile of the per-bucket perpendicular-distance distribution
/// used as the bucket's "scan radius." 0.95 = 95th percentile —
/// excludes the top 5 % of distances per bucket, which naturally
/// rejects single-vertex spikes from scan noise. For a typical
/// per-bucket population of ~300 K vertices, the top 5 % is ~15 K
/// vertices, so spikes from hundreds-to-thousands of noisy vertices
/// get filtered; real anatomical features (tens of thousands of
/// vertices) survive.
///
/// Iter-1 visual review surfaced this: per-bucket MAX inflated the
/// dome's cap region whenever a single noisy vertex stood out. 95th
/// percentile + cross-bucket smoothing together yield a smooth
/// envelope that tracks the scan's gross shape without overreacting.
const RADII_BUCKET_PERCENTILE: f64 = 0.95;

/// Compute the scan's local cross-section radius at each centerline
/// polyline point. For each scan vertex, find the closest centerline
/// point and compute the perpendicular distance to the centerline
/// tangent line at that point. Per-bucket statistic: the
/// [`RADII_BUCKET_PERCENTILE`]-th percentile (95th by default) of
/// the distances collected for that bucket.
///
/// Cost: O(N_vertices × N_centerline) for the bucket-assignment +
/// distance computation + a sort of each bucket. For the iter-1
/// fixture (10 M vertices × 30 centerline points; ~333 K per bucket)
/// this runs in ~3–4 s at startup. One-time; results cached.
///
/// Memory: O(N_vertices) for the per-bucket distance Vecs (~80 MB
/// peak on iter-1 at f64). Freed once the percentile is extracted.
///
/// Used by [`draw_outer_envelope_overlay`] (slice 3 polish 7) to
/// snap the envelope wireframe to the scan's actual outer surface
/// at every cross-section + a uniform clearance.
fn compute_scan_local_radii(
    centerline_points: &[Point3<f64>],
    scan_mesh: &IndexedMesh,
) -> Vec<f64> {
    if centerline_points.len() < 2 {
        return Vec::new();
    }
    let n = centerline_points.len();
    let mut bucket_distances: Vec<Vec<f64>> = vec![Vec::new(); n];
    for vertex in &scan_mesh.vertices {
        // Find closest centerline polyline point (Euclidean
        // distance; linear search across ~30 points).
        let mut best_i = 0_usize;
        let mut best_dist_sq = f64::INFINITY;
        for (i, p) in centerline_points.iter().enumerate() {
            let delta = vertex.coords - p.coords;
            let d = delta.norm_squared();
            if d < best_dist_sq {
                best_dist_sq = d;
                best_i = i;
            }
        }
        // Perpendicular distance from the vertex to the centerline
        // tangent line at the closest centerline point.
        let tangent = local_tangent(centerline_points, best_i);
        let diff = vertex.coords - centerline_points[best_i].coords;
        let parallel = diff.dot(&tangent);
        let perp_sq = diff.norm_squared() - parallel * parallel;
        let perp = perp_sq.max(0.0).sqrt();
        bucket_distances[best_i].push(perp);
    }
    // Per-bucket percentile: sort the distances and pick the
    // RADII_BUCKET_PERCENTILE-th entry. Single-vertex outliers
    // beyond that percentile are excluded, smoothing real noise
    // without smearing genuine wide features (which involve many
    // more vertices than the top-percentile cut).
    let mut radii = Vec::with_capacity(n);
    for bucket in &mut bucket_distances {
        if bucket.is_empty() {
            radii.push(0.0);
            continue;
        }
        bucket.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
        // Bucket sizes are well under 2^53; precision is fine.
        let idx = ((bucket.len() - 1) as f64 * RADII_BUCKET_PERCENTILE) as usize;
        radii.push(bucket[idx]);
    }
    radii
}

/// Half-window for smoothing the per-centerline-point scan-local
/// radii across centerline buckets. `W = 3` averages each radius
/// with up to 3 neighbors on each side (a 7-point moving average),
/// diluting single-vertex spikes by ~1/7 of their original
/// magnitude. Buckets near the centerline endpoints automatically
/// shrink the effective window (clamped to the polyline range).
///
/// Tuned after iter-1 visual review "overfits/overreacts to a bump
/// on the head of the dome": a single noisy vertex at the scan's
/// dome inflated the endpoint bucket's max-perpendicular-distance,
/// which then dictated the cap equatorial radius. 7-point smoothing
/// reduces the spike to ~14 % of its original height — visually
/// negligible while preserving the scan's gross shape.
const RADII_SMOOTHING_HALF_WINDOW: usize = 3;

/// Smooth the per-centerline scan-local radii with a moving-average
/// filter across centerline-bucket indices. Reduces the impact of
/// single-vertex outliers that would otherwise blow out an entire
/// ring's radius (and, at endpoints, the cap's equatorial radius).
///
/// Window: `2 * RADII_SMOOTHING_HALF_WINDOW + 1` centerline points,
/// clamped to the polyline range at endpoints. Returns a Vec of the
/// same length as the input.
fn smooth_scan_local_radii(radii: &[f64]) -> Vec<f64> {
    if radii.is_empty() {
        return Vec::new();
    }
    let n = radii.len();
    let mut smoothed = Vec::with_capacity(n);
    for i in 0..n {
        let lo = i.saturating_sub(RADII_SMOOTHING_HALF_WINDOW);
        let hi = (i + RADII_SMOOTHING_HALF_WINDOW + 1).min(n);
        #[allow(clippy::cast_precision_loss)] // window size fits in f64.
        let count = (hi - lo) as f64;
        let sum: f64 = radii[lo..hi].iter().sum();
        smoothed.push(sum / count);
    }
    smoothed
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
/// outer-envelope wireframe. 24 segments → 15°-spaced ring vertices,
/// smooth enough at typical device radii without overwhelming the
/// gizmo line count.
const OUTER_ENVELOPE_RING_SEGMENTS: usize = 24;

/// Number of longitudinal lines connecting corresponding ring
/// vertices across centerline rings. Drawn at evenly-spaced angles
/// (0°, 90°, 180°, 270° at the default 4). Helps the wireframe read
/// as a 3D tube rather than disconnected rings.
const OUTER_ENVELOPE_LONGITUDINAL_LINES: usize = 4;

/// Number of intermediate latitude rings between the cylindrical
/// section's end ring and the hemispherical cap's apex point. 7
/// rings at θ ∈ {11.25°, 22.5°, 33.75°, 45°, 56.25°, 67.5°, 78.75°}
/// from the cylinder plane produce a smooth hemispherical
/// visualization. Together with the rim ring (θ = 0) and the apex
/// (θ = 90°) this is 9 latitudes per cap.
///
/// Bumped from 3 → 7 after iter-1 visual review: cap latitudes
/// were sparser than the cylindrical section's rings, masking the
/// tip taper. 7 latitudes over a typical 38 mm cap (default
/// radius on iter-1) lands at ~5 mm per latitude — comparable to
/// the cylinder section's per-polyline-point spacing.
const OUTER_ENVELOPE_CAP_LATITUDES: usize = 7;

/// Draw the outer-envelope wireframe overlay each frame. The
/// envelope is a curve-following capsule (`Solid::pipe(centerline,
/// radius_m)` in the SDF kernel; rendered here as a wireframe of
/// cross-section rings perpendicular to the centerline tangent at
/// each polyline point + longitudinal lines connecting them).
///
/// Wireframe-only (not a meshed surface) by design: marching-cubes
/// at every slider tick would lag. The wireframe gives instant
/// visual feedback as the radius changes; the meshed geometry
/// lands at save / validate time via cf-cast-cli (slice 9).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
#[allow(clippy::too_many_arguments)] // Gizmo draw systems pull many Res by convention.
fn draw_outer_envelope_overlay(
    centerline: Res<Centerline>,
    bases: Res<CenterlineBases>,
    scan_radii: Res<ScanLocalRadii>,
    state: Res<OuterEnvelopeState>,
    info: Res<ScanInfo>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut gizmos: Gizmos,
) {
    if centerline.points_m.len() < 2
        || bases.bases.len() != centerline.points_m.len()
        || scan_radii.radii_m.len() != centerline.points_m.len()
    {
        return;
    }
    let arc_positions = cumulative_arc_positions_m(&centerline.points_m);
    if arc_positions.is_empty() {
        return;
    }

    // Geometry semantic (slice 3 polish 7): the envelope is the
    // scan's outer surface offset by `clearance_m`. Each cylindrical
    // ring at centerline polyline point i has radius
    // `scan_local_radius[i] + clearance_m` — programmatically snug
    // to the scan everywhere, no parametric guessing of a constant
    // cylinder radius. Each closed end has an ellipsoidal cap whose
    // EQUATORIAL radius matches the local ring radius at that
    // endpoint, and whose HEIGHT comes from the scan's actual
    // extent past the endpoint along the outward direction.
    let clearance = state.clearance_m;
    let last_idx = centerline.points_m.len() - 1;
    let high_z_is_start = centerline.points_m[0].z >= centerline.points_m[last_idx].z;
    let (closed_at_start, closed_at_end) = if high_z_is_start {
        (state.closed_at_high_z, state.closed_at_low_z)
    } else {
        (state.closed_at_low_z, state.closed_at_high_z)
    };

    // Pale green-blue; distinguishes from scan (white) + axis arrows
    // (red/green/blue full saturation) + centerline (cyan).
    let color = Color::srgb(0.55, 0.85, 0.70);

    // Build cylindrical-section rings at every centerline polyline
    // point. Each ring's radius is the scan's local cross-section
    // radius at that point plus the user-dialed clearance.
    let mut rings_bevy: Vec<Vec<Vec3>> = Vec::with_capacity(arc_positions.len());
    let mut ring_radii: Vec<f64> = Vec::with_capacity(arc_positions.len());
    for (i, &arc) in arc_positions.iter().enumerate() {
        let Some((ring_center, tangent)) =
            sample_centerline_at_arc(&centerline.points_m, &arc_positions, arc)
        else {
            return;
        };
        let (b1, b2) = sample_basis_at_arc(&bases.bases, &arc_positions, arc, &tangent);
        let ring_radius = scan_radii.radii_m[i] + clearance;
        ring_radii.push(ring_radius);
        let mut ring = Vec::with_capacity(OUTER_ENVELOPE_RING_SEGMENTS);
        for seg in 0..OUTER_ENVELOPE_RING_SEGMENTS {
            #[allow(clippy::cast_precision_loss)] // seg fits in f64 easily.
            let theta = (seg as f64 / OUTER_ENVELOPE_RING_SEGMENTS as f64) * std::f64::consts::TAU;
            let offset = b1 * (ring_radius * theta.cos()) + b2 * (ring_radius * theta.sin());
            let physics_pt = Point3::from(ring_center.coords + offset);
            ring.push(Vec3::from_array(up.to_bevy_point(&physics_pt)) * render_scale.0);
        }
        rings_bevy.push(ring);
    }

    // Draw cylindrical ring lines (closed circles at each arc position).
    for ring in &rings_bevy {
        for seg in 0..ring.len() {
            let a = ring[seg];
            let b = ring[(seg + 1) % ring.len()];
            gizmos.line(a, b, color);
        }
    }

    // Draw longitudinal lines connecting corresponding ring vertices
    // across the cylindrical section.
    if rings_bevy.len() >= 2 {
        let step = OUTER_ENVELOPE_RING_SEGMENTS / OUTER_ENVELOPE_LONGITUDINAL_LINES;
        for k in 0..OUTER_ENVELOPE_LONGITUDINAL_LINES {
            let ring_idx = k * step;
            for i in 0..rings_bevy.len() - 1 {
                let a = rings_bevy[i][ring_idx];
                let b = rings_bevy[i + 1][ring_idx];
                gizmos.line(a, b, color);
            }
        }
    }

    // Ellipsoidal end caps. For each closed end, the cap base is at
    // the centerline ENDPOINT (the cylinder's terminal rim ring),
    // the outward tangent points away from the centerline interior,
    // and the cap height is computed from the scan AABB's extent
    // past the endpoint along that outward direction. The cap thus
    // hugs the actual dome height (e.g. ~8 mm on a sock-shaped
    // scan whose centerline endpoint sits ~5 mm inside the dome
    // surface), not a full R hemisphere.
    if closed_at_start {
        let start_point = centerline.points_m[0];
        let start_outward = -local_tangent(&centerline.points_m, 0);
        let start_cap_height =
            scan_extent_past_endpoint(&start_point, &start_outward, &info.scan_aabb_m) + clearance;
        let (start_b1, start_b2) = bases.bases[0];
        draw_hemispherical_cap(
            &start_point,
            &start_outward,
            &start_b1,
            &start_b2,
            ring_radii[0],
            start_cap_height,
            &rings_bevy[0],
            *up,
            render_scale.0,
            color,
            &mut gizmos,
        );
    }

    if closed_at_end {
        let end_point = centerline.points_m[last_idx];
        let end_outward = local_tangent(&centerline.points_m, last_idx);
        let end_cap_height =
            scan_extent_past_endpoint(&end_point, &end_outward, &info.scan_aabb_m) + clearance;
        let (end_b1, end_b2) = bases.bases[last_idx];
        let last_ring_idx = rings_bevy.len() - 1;
        draw_hemispherical_cap(
            &end_point,
            &end_outward,
            &end_b1,
            &end_b2,
            ring_radii[last_ring_idx],
            end_cap_height,
            &rings_bevy[last_ring_idx],
            *up,
            render_scale.0,
            color,
            &mut gizmos,
        );
    }
}

/// Draw an ellipsoidal end cap centered at `base_point` with its
/// axis along `outward_tangent` (away from the centerline
/// interior). Uses `(b1, b2)` as the rim-ring basis. `radius_m` =
/// equatorial ring radius (matches the cylinder section);
/// `height_m` = distance from base to apex along `outward_tangent`.
/// When `height_m == radius_m` the cap is a hemisphere; when
/// `height_m < radius_m` it's an oblate (flatter) dome — used in
/// slice 3 polish 6 to size the cap to the scan's actual extent
/// past the centerline endpoint instead of always drawing a full
/// R hemisphere.
///
/// Cap latitudes are parameterized by `t ∈ [0, 1]` (0 = rim, 1 =
/// apex). At each `t`:
/// `ring_radius = radius_m * sqrt(1 - t²)`,
/// `ring_offset_along_outward = height_m * t`. This matches an
/// ellipsoid `(x/R)² + (y/R)² + (z/H)² = 1` at the cap half —
/// equator at `t=0`, pole at `t=1`. Drawing pattern:
/// `OUTER_ENVELOPE_CAP_LATITUDES` intermediate ring circles +
/// `OUTER_ENVELOPE_LONGITUDINAL_LINES` arcs from `base_ring_bevy`
/// through the cap rings up to the apex.
#[allow(clippy::too_many_arguments)] // Helper packs the full cap-draw context.
fn draw_hemispherical_cap(
    base_point: &Point3<f64>,
    outward_tangent: &Vector3<f64>,
    b1: &Vector3<f64>,
    b2: &Vector3<f64>,
    radius_m: f64,
    height_m: f64,
    base_ring_bevy: &[Vec3],
    up: UpAxis,
    render_scale: f32,
    color: Color,
    gizmos: &mut Gizmos,
) {
    // Build intermediate cap rings + apex point in Bevy frame.
    // `cap_rings_bevy[lat]` is the ring at latitude t_lat.
    let mut cap_rings_bevy: Vec<Vec<Vec3>> = Vec::with_capacity(OUTER_ENVELOPE_CAP_LATITUDES);
    for lat in 1..=OUTER_ENVELOPE_CAP_LATITUDES {
        #[allow(clippy::cast_precision_loss)] // lat fits in f64.
        let t = lat as f64 / (OUTER_ENVELOPE_CAP_LATITUDES + 1) as f64;
        let ring_center = base_point.coords + outward_tangent * (height_m * t);
        let ring_radius = radius_m * (1.0 - t * t).max(0.0).sqrt();
        let mut ring = Vec::with_capacity(OUTER_ENVELOPE_RING_SEGMENTS);
        for seg in 0..OUTER_ENVELOPE_RING_SEGMENTS {
            #[allow(clippy::cast_precision_loss)]
            let phi = (seg as f64 / OUTER_ENVELOPE_RING_SEGMENTS as f64) * std::f64::consts::TAU;
            let offset = b1 * (ring_radius * phi.cos()) + b2 * (ring_radius * phi.sin());
            let physics_pt = Point3::from(ring_center + offset);
            ring.push(Vec3::from_array(up.to_bevy_point(&physics_pt)) * render_scale);
        }
        cap_rings_bevy.push(ring);
    }
    let apex_physics = Point3::from(base_point.coords + outward_tangent * height_m);
    let apex_bevy = Vec3::from_array(up.to_bevy_point(&apex_physics)) * render_scale;

    // Draw the intermediate cap rings (closed circles).
    for ring in &cap_rings_bevy {
        for seg in 0..ring.len() {
            let a = ring[seg];
            let b = ring[(seg + 1) % ring.len()];
            gizmos.line(a, b, color);
        }
    }

    // Longitudinal arcs from rim → intermediate rings → apex. At
    // each of the OUTER_ENVELOPE_LONGITUDINAL_LINES picked ring
    // indices, connect rim → cap[0] → cap[1] → ... → apex.
    let step = OUTER_ENVELOPE_RING_SEGMENTS / OUTER_ENVELOPE_LONGITUDINAL_LINES;
    for k in 0..OUTER_ENVELOPE_LONGITUDINAL_LINES {
        let ring_idx = k * step;
        let mut prev = base_ring_bevy[ring_idx];
        for cap_ring in &cap_rings_bevy {
            let next = cap_ring[ring_idx];
            gizmos.line(prev, next, color);
            prev = next;
        }
        gizmos.line(prev, apex_bevy, color);
    }
}

/// Render the Outer Envelope egui section. Slice 3 polish 7 pivot:
/// the envelope is now SCAN-DERIVED — its rings sit at every
/// centerline cross-section AT THE SCAN SURFACE plus a uniform
/// `clearance_m`. The user dials the clearance; the radius at each
/// arc position is computed from the scan, not chosen.
///
/// Also surfaces the scan's local-radius range (min / max across
/// centerline points) so the user sees what the underlying snug
/// numbers are.
fn render_outer_envelope_section(
    ui: &mut egui::Ui,
    state: &mut OuterEnvelopeState,
    info: &ScanInfo,
    scan_radii: &ScanLocalRadii,
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
            if info.centerline_point_count >= 2 && !scan_radii.radii_m.is_empty() {
                let min_r = scan_radii
                    .radii_m
                    .iter()
                    .copied()
                    .fold(f64::INFINITY, f64::min);
                let max_r = scan_radii.radii_m.iter().copied().fold(0.0_f64, f64::max);
                ui.label(format!(
                    "scan cross-section: {:.1} - {:.1} mm",
                    min_r * 1000.0,
                    max_r * 1000.0,
                ));
                ui.label(format!(
                    "envelope at clearance: {:.1} - {:.1} mm",
                    (min_r + state.clearance_m) * 1000.0,
                    (max_r + state.clearance_m) * 1000.0,
                ));
            } else {
                ui.label("(centerline missing — wireframe will not render)");
            }
            ui.add_space(4.0);
            // Per-end cap toggles — controls whether the envelope's
            // dome end (high-Z) and rim end (low-Z) each get a
            // hemispherical cap. Default closed at high-Z (scan
            // dome) + open at low-Z (cleaned-cap convention).
            ui.checkbox(&mut state.closed_at_high_z, "Closed dome (high Z end)");
            ui.checkbox(&mut state.closed_at_low_z, "Closed dome (low Z end)");
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
/// Renders Scan Info + Outer Envelope as of slice 3; remaining
/// stubs surface the planned panel order.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn device_design_panel(
    mut contexts: EguiContexts,
    info: Res<ScanInfo>,
    scan_radii: Res<ScanLocalRadii>,
    mut outer_envelope: ResMut<OuterEnvelopeState>,
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
                    render_outer_envelope_section(ui, &mut outer_envelope, &info, &scan_radii);
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
/// panel names so the user can see the planned layout.
fn render_panel_stubs(ui: &mut egui::Ui) {
    ui.separator();
    for name in [
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
    let bases = CenterlineBases {
        bases: compute_centerline_bases(&centerline_points),
    };
    // Compute the scan's local cross-section radii at each centerline
    // polyline point, then smooth across buckets to dilute single-
    // vertex spikes (e.g. a noisy dome apex inflating the endpoint's
    // ring → cap radius). One-time cost at startup; the wireframe
    // ring radii are derived from the smoothed values every frame.
    let raw_radii = compute_scan_local_radii(&centerline_points, &scan_mesh);
    let scan_radii = ScanLocalRadii {
        radii_m: smooth_scan_local_radii(&raw_radii),
    };
    let outer_envelope = OuterEnvelopeState::default_for_scan();

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
        .insert_resource(bases)
        .insert_resource(scan_radii)
        .insert_resource(outer_envelope)
        .add_systems(Startup, setup_render_scene)
        .add_systems(
            Update,
            (
                block_orbit_input_when_over_egui.before(orbit_camera_input),
                draw_reference_overlays,
                draw_outer_envelope_overlay,
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

    // ----- Slice 3 — local tangent + parallel-transport bases ------

    fn straight_z_centerline() -> Vec<Point3<f64>> {
        (0..5)
            .map(|i| Point3::new(0.0, 0.0, f64::from(i) * 0.02))
            .collect()
    }

    fn approx_eq(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    #[test]
    fn local_tangent_endpoint_forward_difference() {
        let pts = straight_z_centerline();
        let t = local_tangent(&pts, 0);
        // Forward difference (0,0,0) → (0,0,0.02) gives +Z unit.
        assert!(approx_eq(t.x, 0.0, 1e-12));
        assert!(approx_eq(t.y, 0.0, 1e-12));
        assert!(approx_eq(t.z, 1.0, 1e-12));
    }

    #[test]
    fn local_tangent_interior_central_difference() {
        let pts = straight_z_centerline();
        let t = local_tangent(&pts, 2);
        // Central difference (0,0,0.02) → (0,0,0.06) gives +Z unit.
        assert!(approx_eq(t.x, 0.0, 1e-12));
        assert!(approx_eq(t.y, 0.0, 1e-12));
        assert!(approx_eq(t.z, 1.0, 1e-12));
    }

    #[test]
    fn local_tangent_endpoint_backward_difference() {
        let pts = straight_z_centerline();
        let t = local_tangent(&pts, 4);
        // Backward difference (0,0,0.06) → (0,0,0.08) gives +Z unit.
        assert!(approx_eq(t.x, 0.0, 1e-12));
        assert!(approx_eq(t.y, 0.0, 1e-12));
        assert!(approx_eq(t.z, 1.0, 1e-12));
    }

    #[test]
    fn local_tangent_zero_segment_falls_back_to_plus_z() {
        let pts = vec![Point3::origin(), Point3::origin()];
        let t = local_tangent(&pts, 0);
        // Degenerate (consecutive coincident points): documented
        // fallback is +Z so the basis math doesn't NaN.
        assert!(approx_eq(t.z, 1.0, 1e-12));
    }

    #[test]
    fn local_tangent_windowed_smoothing_suppresses_single_sample_noise() {
        // 7-point straight-Y centerline with ONE noisy outlier in
        // the middle: point 3 perturbed in X. With smoothing window 2,
        // the 5-point central difference at point 3 averages over
        // points 1..=5, so the X noise cancels out and the tangent
        // remains nearly +Y. WITHOUT smoothing (3-point central diff)
        // the tangent at point 3 picks up the X kick from the two
        // adjacent points and tilts notably.
        let mut pts: Vec<Point3<f64>> = (0..7)
            .map(|i| Point3::new(0.0, f64::from(i) * 0.02, 0.0))
            .collect();
        // Single-sample perturbation in X at the middle point.
        pts[3] = Point3::new(0.01, pts[3].y, 0.0);

        let t = local_tangent(&pts, 3);
        // Smoothed central diff: (pts[5] - pts[1]) = (0, 0.08, 0).
        // Tangent should be very nearly +Y with a tiny residual.
        assert!(
            approx_eq(t.y, 1.0, 1e-9),
            "smoothed tangent.y should be ~1.0, got {}",
            t.y
        );
        assert!(t.x.abs() < 1e-9, "smoothed tangent.x = {}", t.x);
    }

    #[test]
    fn initial_basis_b1_orthogonal_to_tangent() {
        let t = Vector3::new(1.0, 0.0, 0.0);
        let b1 = initial_basis_b1(&t);
        assert!(approx_eq(b1.dot(&t), 0.0, 1e-12));
        assert!(approx_eq(b1.norm(), 1.0, 1e-12));
    }

    #[test]
    fn initial_basis_b1_handles_tangent_parallel_to_z() {
        // Tangent nearly parallel to the +Z candidate; helper must
        // swap to +X candidate so the projection doesn't collapse.
        let t = Vector3::new(0.0, 0.0, 1.0);
        let b1 = initial_basis_b1(&t);
        assert!(approx_eq(b1.dot(&t), 0.0, 1e-12));
        assert!(approx_eq(b1.norm(), 1.0, 1e-12));
    }

    #[test]
    fn parallel_transport_preserves_orthogonality() {
        // Start with a basis b1 perpendicular to the OLD tangent;
        // transport it to a NEW tangent direction. The transported
        // b1' should be perpendicular to the new tangent + unit-length.
        let old_t = Vector3::new(0.0, 0.0, 1.0);
        let b1 = initial_basis_b1(&old_t);
        let new_t = Vector3::new(0.0, 1.0, 0.0); // perpendicular to old
        let b1_new = parallel_transport_b1(b1, &new_t);
        assert!(approx_eq(b1_new.dot(&new_t), 0.0, 1e-12));
        assert!(approx_eq(b1_new.norm(), 1.0, 1e-12));
    }

    #[test]
    fn compute_centerline_bases_yields_orthonormal_right_handed_frames() {
        let pts = straight_z_centerline();
        let bases = compute_centerline_bases(&pts);
        assert_eq!(bases.len(), pts.len());
        for (i, (b1, b2)) in bases.iter().enumerate() {
            let t = local_tangent(&pts, i);
            assert!(
                approx_eq(b1.dot(&t), 0.0, 1e-9),
                "b1·t at {i} = {}",
                b1.dot(&t)
            );
            assert!(
                approx_eq(b2.dot(&t), 0.0, 1e-9),
                "b2·t at {i} = {}",
                b2.dot(&t)
            );
            assert!(
                approx_eq(b1.dot(b2), 0.0, 1e-9),
                "b1·b2 at {i} = {}",
                b1.dot(b2)
            );
            assert!(approx_eq(b1.norm(), 1.0, 1e-9));
            assert!(approx_eq(b2.norm(), 1.0, 1e-9));
            // Right-handed: b1 × b2 = +tangent (within sign).
            let cross = b1.cross(b2);
            assert!(
                approx_eq(cross.dot(&t), 1.0, 1e-9),
                "right-hand check at {i}: cross·t = {}",
                cross.dot(&t)
            );
        }
    }

    #[test]
    fn compute_centerline_bases_returns_empty_for_short_polylines() {
        assert!(compute_centerline_bases(&[]).is_empty());
        assert!(compute_centerline_bases(&[Point3::origin()]).is_empty());
    }

    // ----- Slice 3 polish 7 — clearance-only envelope state --------

    #[test]
    fn outer_envelope_default_clearance_is_five_mm() {
        let state = OuterEnvelopeState::default_for_scan();
        assert!(approx_eq(state.clearance_m, 0.005, 1e-12));
    }

    #[test]
    fn outer_envelope_defaults_to_closed_high_z_open_low_z() {
        let state = OuterEnvelopeState::default_for_scan();
        assert!(state.closed_at_high_z);
        assert!(!state.closed_at_low_z);
    }

    #[test]
    fn outer_envelope_clearance_slider_range_zero_to_fifty_mm() {
        let (min_m, max_m) = OuterEnvelopeState::clearance_slider_range_m();
        assert!(approx_eq(min_m, 0.0, 1e-12));
        assert!(approx_eq(max_m, 0.050, 1e-12));
    }

    // ----- Slice 3 polish 7 — scan-local cross-section radii -------

    #[test]
    fn compute_scan_local_radii_short_polyline_returns_empty() {
        let mesh = unit_cube_mesh();
        assert!(compute_scan_local_radii(&[], &mesh).is_empty());
        assert!(compute_scan_local_radii(&[Point3::origin()], &mesh).is_empty());
    }

    #[test]
    fn compute_scan_local_radii_unit_cube_endpoints_get_xy_half_diag() {
        // Unit cube ±50 mm corners; centerline along +Z at three
        // points. All 8 cube corners are closest to the END
        // centerline points (z = ±0.030) because the middle point
        // (z = 0) is equidistant from ±Z corners but the corners'
        // total Euclidean distance there exceeds the endpoint
        // distances. So radii[0] and radii[2] = XY half-diagonal =
        // sqrt(50² + 50²) ≈ 70.7 mm; radii[1] = 0 (no corner
        // closest).
        let mesh = unit_cube_mesh();
        let pts = vec![
            Point3::new(0.0, 0.0, -0.030),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.030),
        ];
        let radii = compute_scan_local_radii(&pts, &mesh);
        assert_eq!(radii.len(), 3);
        let expected = (50.0_f64 * 50.0 + 50.0 * 50.0).sqrt() / 1000.0;
        assert!(
            approx_eq(radii[0], expected, 1e-9),
            "got radii[0] = {}",
            radii[0]
        );
        assert!(
            approx_eq(radii[2], expected, 1e-9),
            "got radii[2] = {}",
            radii[2]
        );
        // Middle point has no closest vertex on this fixture.
        assert!(
            approx_eq(radii[1], 0.0, 1e-12),
            "got radii[1] = {}",
            radii[1]
        );
    }

    #[test]
    fn smooth_scan_local_radii_dilutes_single_spike() {
        // Spike at index 3 (height 0.040 amid a flat 0.020 sequence).
        // 7-point average (W = 3) at index 3: average of indices
        // [0, 1, 2, 3, 4, 5, 6] = (0.020 * 6 + 0.040) / 7 ≈ 0.0229.
        // Spike is reduced from 0.040 → 0.0229 (~43 % of original
        // above the baseline).
        let radii = vec![0.020, 0.020, 0.020, 0.040, 0.020, 0.020, 0.020];
        let smoothed = smooth_scan_local_radii(&radii);
        assert_eq!(smoothed.len(), 7);
        let expected_at_spike = (0.020 * 6.0 + 0.040) / 7.0;
        assert!(
            approx_eq(smoothed[3], expected_at_spike, 1e-9),
            "got {}, expected {}",
            smoothed[3],
            expected_at_spike,
        );
    }

    #[test]
    fn smooth_scan_local_radii_preserves_uniform_input() {
        let radii = vec![0.025; 10];
        let smoothed = smooth_scan_local_radii(&radii);
        for s in &smoothed {
            assert!(approx_eq(*s, 0.025, 1e-12));
        }
    }

    #[test]
    fn smooth_scan_local_radii_handles_endpoint_clamping() {
        // At index 0 with W = 3, the window clamps to [0, 4)
        // (4 values, not 7). Average of input[0..4].
        let radii = vec![0.010, 0.020, 0.030, 0.040, 0.050, 0.050, 0.050];
        let smoothed = smooth_scan_local_radii(&radii);
        let expected_at_0 = (0.010 + 0.020 + 0.030 + 0.040) / 4.0;
        assert!(
            approx_eq(smoothed[0], expected_at_0, 1e-9),
            "got {}, expected {}",
            smoothed[0],
            expected_at_0,
        );
    }

    #[test]
    fn smooth_scan_local_radii_empty_input_returns_empty() {
        assert!(smooth_scan_local_radii(&[]).is_empty());
    }

    #[test]
    fn compute_scan_local_radii_percentile_rejects_single_outlier() {
        // 1 noisy outlier vertex at (1.0, 0, 0) (radial = 1 m) vs
        // 100 normal vertices on a ring at radius 0.030. With
        // RADII_BUCKET_PERCENTILE = 0.95, the top 5 % of 101
        // distances = ~5 entries from the top — but only 1 entry is
        // the 1m outlier, so the 95th percentile lands at one of
        // the 0.030 normals.
        let mut m = IndexedMesh::new();
        for theta in 0..100 {
            #[allow(clippy::cast_precision_loss)]
            let angle = (theta as f64) * std::f64::consts::TAU / 100.0;
            m.vertices
                .push(Point3::new(0.030 * angle.cos(), 0.030 * angle.sin(), 0.0));
        }
        m.vertices.push(Point3::new(1.0, 0.0, 0.0)); // outlier
        let pts = vec![
            Point3::new(0.0, 0.0, -0.001),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.001),
        ];
        let radii = compute_scan_local_radii(&pts, &m);
        // All 101 vertices are closest to centerline point [1]
        // (z = 0). Its 95th percentile excludes the outlier.
        assert!(
            approx_eq(radii[1], 0.030, 1e-6),
            "got radii[1] = {} (expected ~0.030 m, outlier rejected)",
            radii[1],
        );
    }

    #[test]
    fn compute_scan_local_radii_returns_per_point_max() {
        // Build a mesh with two clusters of vertices: one at z=0
        // with radius 30 mm in XY, one at z=0.04 with radius 50 mm.
        // Centerline along Z through both. Each centerline point's
        // closest cluster should give the per-point max.
        let mut m = IndexedMesh::new();
        for theta in 0..8 {
            #[allow(clippy::cast_precision_loss)]
            let angle = (theta as f64) * std::f64::consts::TAU / 8.0;
            m.vertices
                .push(Point3::new(0.030 * angle.cos(), 0.030 * angle.sin(), 0.0));
            m.vertices
                .push(Point3::new(0.050 * angle.cos(), 0.050 * angle.sin(), 0.040));
        }
        let pts = vec![Point3::new(0.0, 0.0, -0.001), Point3::new(0.0, 0.0, 0.041)];
        let radii = compute_scan_local_radii(&pts, &m);
        assert_eq!(radii.len(), 2);
        // Point at z=-0.001 is closest to the z=0 cluster (30 mm).
        assert!(
            approx_eq(radii[0], 0.030, 1e-9),
            "got radii[0] = {}",
            radii[0]
        );
        // Point at z=+0.041 is closest to the z=0.040 cluster (50 mm).
        assert!(
            approx_eq(radii[1], 0.050, 1e-9),
            "got radii[1] = {}",
            radii[1]
        );
    }

    #[test]
    fn scan_extent_past_endpoint_zero_when_aabb_behind_endpoint() {
        // Endpoint at (0, 0, +0.100), outward = +Z; AABB entirely
        // BEHIND the endpoint (z < 0.100). All corner projections
        // are negative → returns 0.0 (defensive).
        let endpoint = Point3::new(0.0, 0.0, 0.100);
        let outward = Vector3::new(0.0, 0.0, 1.0);
        let aabb = Aabb::new(
            Point3::new(-0.030, -0.030, 0.0),
            Point3::new(0.030, 0.030, 0.050),
        );
        let e = scan_extent_past_endpoint(&endpoint, &outward, &aabb);
        assert!(approx_eq(e, 0.0, 1e-12), "got {e}");
    }

    #[test]
    fn scan_extent_past_endpoint_iter_1_top_yields_dome_height() {
        // Iter-1-shaped scan: endpoint at (0, 0, +0.080) with
        // outward = +Z; AABB extends to z = +0.087. Extent past =
        // 7 mm (the dome height).
        let endpoint = Point3::new(0.0, 0.0, 0.080);
        let outward = Vector3::new(0.0, 0.0, 1.0);
        let aabb = Aabb::new(
            Point3::new(-0.035, -0.035, -0.057),
            Point3::new(0.035, 0.035, 0.087),
        );
        let e = scan_extent_past_endpoint(&endpoint, &outward, &aabb);
        assert!(approx_eq(e, 0.007, 1e-12), "got {e}");
    }

    // (Prior `outer_envelope_slider_falls_back_to_diag_without_centerline`
    // test deleted in polish 7 — slider is now clearance-based, not
    // radius-based, so the bbox-diagonal fallback is no longer
    // relevant.)

    // ----- Slice 3 polish 2 — arc-sampling helpers ------------------

    #[test]
    fn cumulative_arc_positions_match_segment_distances() {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        let arcs = cumulative_arc_positions_m(&pts);
        assert_eq!(arcs, vec![0.0, 1.0, 2.0, 3.0]);
    }

    #[test]
    fn cumulative_arc_positions_empty_for_short_polylines() {
        assert!(cumulative_arc_positions_m(&[]).is_empty());
        assert!(cumulative_arc_positions_m(&[Point3::origin()]).is_empty());
    }

    #[test]
    fn sample_centerline_at_arc_midpoint_lands_between_polyline_points() {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];
        let arcs = cumulative_arc_positions_m(&pts);
        // Arc 0.5 is halfway through the first segment.
        let (pos, tangent) = sample_centerline_at_arc(&pts, &arcs, 0.5).unwrap();
        assert!(approx_eq(pos.x, 0.5, 1e-12));
        assert!(approx_eq(pos.y, 0.0, 1e-12));
        assert!(approx_eq(pos.z, 0.0, 1e-12));
        // Tangent is unit +X (first segment direction).
        assert!(approx_eq(tangent.x, 1.0, 1e-12));
    }

    #[test]
    fn sample_centerline_at_arc_clamps_out_of_range() {
        let pts = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
        let arcs = cumulative_arc_positions_m(&pts);
        // Negative arc → clamped to 0 (start).
        let (pos_neg, _) = sample_centerline_at_arc(&pts, &arcs, -5.0).unwrap();
        assert!(approx_eq(pos_neg.x, 0.0, 1e-12));
        // Beyond total → clamped to end.
        let (pos_far, _) = sample_centerline_at_arc(&pts, &arcs, 10.0).unwrap();
        assert!(approx_eq(pos_far.x, 1.0, 1e-12));
    }

    #[test]
    fn sample_centerline_at_arc_returns_none_for_short_polylines() {
        let pts: Vec<Point3<f64>> = vec![];
        let arcs: Vec<f64> = vec![];
        assert!(sample_centerline_at_arc(&pts, &arcs, 0.5).is_none());
    }

    #[test]
    fn sample_basis_at_arc_returns_orthonormal_basis() {
        let pts = straight_z_centerline();
        let arcs = cumulative_arc_positions_m(&pts);
        let bases = compute_centerline_bases(&pts);
        let (_, tangent) = sample_centerline_at_arc(&pts, &arcs, 0.05).unwrap();
        let (b1, b2) = sample_basis_at_arc(&bases, &arcs, 0.05, &tangent);
        assert!(approx_eq(b1.dot(&tangent), 0.0, 1e-9));
        assert!(approx_eq(b2.dot(&tangent), 0.0, 1e-9));
        assert!(approx_eq(b1.dot(&b2), 0.0, 1e-9));
        assert!(approx_eq(b1.norm(), 1.0, 1e-9));
        assert!(approx_eq(b2.norm(), 1.0, 1e-9));
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
