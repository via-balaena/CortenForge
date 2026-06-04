//! `cf-scan-prep-core` — the headless, Bevy-free mesh-editing core for scan
//! preprocessing.
//!
//! This is the pure-compute half of the `cf-scan-prep` tool, extracted so a
//! second frontend (CortenForge Studio's Slint+wgpu scan editor) can drive
//! the exact same algorithms the Bevy tool does, with no behavior drift. The
//! Bevy tool keeps the ECS/egui/rendering shell and calls into here; Studio
//! calls into here too.
//!
//! Everything here operates on [`mesh_types::IndexedMesh`] + nalgebra types.
//! Nothing here depends on Bevy, egui, or any renderer.

use std::collections::{HashMap, HashSet};
use std::path::Path;
use std::time::Instant;

use anyhow::{Context, Result};
use baby_shark::{
    decimation::{AlwaysDecimate, EdgeDecimator},
    mesh::{corner_table::CornerTableF, traits::TriangleMesh},
};
use mesh_io::save_stl;
use mesh_repair::{
    MeshAdjacency, TAUBIN_DEFAULT_LAMBDA, TAUBIN_DEFAULT_MU, holes, remove_degenerate_triangles,
    remove_small_components, remove_unreferenced_vertices, taubin_smooth_vertices, weld_vertices,
};
use mesh_types::{Aabb, Bounded, IndexedMesh, Point3};
use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use serde::Serialize;

/// Heuristic: does `mesh` look like raw, unwelded STL soup?
///
/// `mesh_io::load_stl` emits 3 unshared vertices per triangle, so a
/// freshly-loaded scan has `vertex_count == 3 × face_count`. A welded
/// mesh has `vertex_count ≈ 0.5 × face_count` (Euler: V ≈ F/2). The
/// `≥ 2 ×` threshold cleanly separates the two and stays robust to
/// partial welds. Used to surface the "Weld first" warning + to make
/// the Cap → Scan message actionable when it detects per-triangle loops.
pub fn mesh_looks_unwelded(vertex_count: usize, face_count: usize) -> bool {
    face_count > 0 && vertex_count >= face_count * 2
}

/// Default target face count for the Simplify panel slider per spec
/// §Architectural decisions §Simplify algorithm: 10× over the 50k
/// surface-continuity floor, well below the 500k "may be sluggish"
/// threshold, matches cf-cast's 2 mm-cell SDF sampling density
/// × surface-continuity headroom.
pub const SIMPLIFY_TARGET_DEFAULT: usize = 200_000;

/// Lower slider bound. Below this the simplified mesh loses too much
/// surface continuity for cast purposes.
pub const SIMPLIFY_TARGET_MIN: usize = 1_000;

/// Upper slider bound. Above this we're not really simplifying typical
/// scans anymore.
pub const SIMPLIFY_TARGET_MAX: usize = 1_000_000;

/// Spatial-hash welding tolerance applied pre-`meshopt::simplify` so
/// the STL's 3N unshared vertices collapse to ~N shared. 1 µm in
/// meters; tighter than any practical scan precision.
pub const SIMPLIFY_WELD_EPSILON_M: f64 = 1e-6;

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
pub const CLEANUP_DEGENERATE_AREA_M2: f64 = 1e-15;

/// Minimum face count for a connected component to survive the
/// `build_cleaned_mesh` cleanup pass. Components smaller than this
/// are dropped as scanner noise. 10 is conservative — a meaningful
/// shell (even a small cyst-like protrusion) has dozens of faces;
/// noise islands are typically 1-3 stray triangles from scanner
/// registration glitches or self-intersection artifacts. Spec
/// §Strategic context assumes the user has pre-trimmed to a single
/// shell externally; this threshold is the safety net for the
/// "user forgot" case.
pub const CLEANUP_MIN_COMPONENT_FACES: usize = 10;

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
pub struct DetectedCapLoop {
    /// Ordered vertex indices forming the boundary loop (closed: the
    /// last edge connects back to vertex 0).
    pub vertex_indices: Vec<u32>,
    /// Least-squares fit plane: position `centroid` and unit normal.
    /// Centroid is the average of loop vertex positions in mesh-local
    /// physics-frame meters; normal is outward-oriented per the
    /// mesh-side-of-plane heuristic. Stored at scan time for use by
    /// commit #12's ear-clipping triangulation (the bake step needs
    /// both centroid + normal to project loop vertices onto the fit
    /// plane before triangulating into cap faces).
    pub plane_centroid: Point3<f64>,
    pub plane_normal: Vector3<f64>,
    /// Plane-fit R²: 1.0 = perfectly planar loop; 0.0 = totally
    /// non-planar. Surfaced to the user so they can judge whether
    /// auto-fit produced a reasonable cap.
    pub plane_fit_r_squared: f64,
    /// User decision: include this loop in `[Apply caps]` /
    /// downstream save? Default `true` if vertex count >= 8 (per
    /// spec: small loops are often acceptable holes / scanner
    /// artifacts), `false` otherwise.
    pub include: bool,
}

/// Cross-section extrusion shape choice for the reconstruct
/// algorithm (CSP.4e). User-picked via radio buttons.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReconstructShape {
    /// Average radial profile from the reference zone, extruded
    /// straight down the centerline at constant radius. Simplest
    /// shape; ships first in CSP.4e.2.
    Constant,
    /// Constant profile linearly tapered toward a smaller radius
    /// at the new floor. Ships in CSP.4e.3.
    Taper,
    /// Fit a linear trend `r(angle, s) = a + b·s` across the
    /// reference zone; extrapolate `s` past the cut. Most
    /// faithful to natural taper. Ships in CSP.4e.4.
    Extrapolate,
}

/// What got committed by `[Apply reconstruct]`. Distinct from
/// the live slider/shape state so the user can drift the
/// sliders without un-applying the existing reconstruction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AppliedReconstruct {
    pub reference_mm: f64,
    pub shape: ReconstructShape,
}

/// Linear interpolation between two `Point3<f64>` positions.
/// `t = 0` returns `a`; `t = 1` returns `b`.
pub fn lerp_point(a: &Point3<f64>, b: &Point3<f64>, t: f64) -> Point3<f64> {
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
/// Shared core algorithm — [`clip_mesh_against_plane`] (the
/// centerline-trim cuts at each end) derives its inputs and then
/// forwards here. Pre-CSP.4d there was also a
/// `clip_mesh_against_world_z` for the now-retired Clip-floor
/// panel; that wrapper is gone, but `clip_mesh_against_plane_eq`
/// remained as the shared core.
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
pub fn clip_mesh_against_plane_eq(
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

// CSP.4c — `clip_mesh_against_world_z` removed alongside
// `ClipState`. The shared `clip_mesh_against_plane_eq` primitive
// below is still load-bearing for `trim_mesh_along_centerline`.

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
pub fn clip_mesh_against_plane(
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
pub fn point_along_polyline_at_arc_distance(
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
pub fn polyline_arc_length_m(polyline: &[Point3<f64>]) -> f64 {
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
pub fn trim_mesh_along_centerline(
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
pub fn trim_centerline_polyline(
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
/// (with outward-pointing 3D normals) that close every detected loop.
/// CSP.4b — runs after [`trim_mesh_along_centerline`] so the trim cuts
/// get sealed.
///
/// Mutates `mesh` in place. Returns the number of loops capped.
///
/// Same projection + winding logic as `build_cleaned_mesh`'s cap
/// step (CSP.3a): boundary vertices are projected onto the fit plane
/// before ear-clip so the cap is exactly planar. Cap faces use the
/// existing loop vertex indices (no new vertices appended for the
/// cap; only re-uses the now-projected boundary positions). Cap-face
/// 3D winding emits with cross-product normals aligned to the
/// outward-oriented plane normal — pinned by
/// `auto_cap_open_boundaries_emits_outward_cap_normals`.
pub fn auto_cap_open_boundaries(mesh: &mut IndexedMesh) -> usize {
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

        emit_centroid_fan_cap(mesh, &loop_data.vertices, &loop_points_projected, &verts_2d);
        capped += 1;
    }
    capped
}

/// Cap a single open boundary loop with a **centroid-fan**: append one
/// new vertex at the loop's 3D centroid (which sits on the fit plane
/// because `loop_points_projected` has already been snapped to it) and
/// emit one cap triangle per perimeter edge fanning out from that
/// centroid.
///
/// Replaces the [`triangulate_polygon_2d_earclip`] path that
/// `auto_cap_open_boundaries` + `build_cleaned_mesh`'s cap step used
/// pre-2026-05-26. Per S1.1 probe-8/recon: that path's fan-fallback
/// (kicked in whenever no ear satisfies the convex + empty-interior
/// check) emitted overlapping triangles from a degenerate fan anchor
/// on self-intersecting projected boundaries, producing duplicate
/// faces + non-manifold edges in the cap region that downstream
/// manifold3d-based consumers (cf-cast's `apply_mating_transforms`)
/// rejected.
///
/// **Why centroid-fan is always-manifold for our cap-fan inputs**:
/// for any STAR-SHAPED 2D polygon — which fit-plane-projected
/// boundary loops always are on the workshop scans — fanning from
/// the centroid to each consecutive perimeter edge produces a
/// non-overlapping, manifold triangulation by construction. The
/// centroid is inside the polygon (star-point), so every cap triangle
/// is contained in the polygon and adjacent triangles share exactly
/// one edge (the `(centroid, perim[i])` edge). Self-intersecting
/// polygons would still trip this, but cf-scan-prep's projection step
/// ([`project_loop_to_plane_2d`]) on workshop scans hasn't surfaced a
/// non-star-shaped case to date; if one ever does, the workshop user
/// can re-scan or pre-process upstream.
///
/// **Winding**: `verts_2d` lives in the `(u, v, plane_normal)`
/// right-handed basis returned by `project_loop_to_plane_2d`. A
/// 2D-CCW perimeter (`signed_area > 0`) emitted as
/// `[centroid, perim[i], perim[i+1]]` produces a 3D cross product
/// in the `+plane_normal` direction, which `orient_cap_normal_outward`
/// has already aligned with OUTWARD. For 2D-CW perimeters
/// (`signed_area < 0`) we swap the perimeter pair to land the same
/// outward direction. Pinned by
/// `auto_cap_open_boundaries_emits_outward_cap_normals`.
pub fn emit_centroid_fan_cap(
    mesh: &mut IndexedMesh,
    loop_vertex_indices: &[u32],
    loop_points_projected: &[Point3<f64>],
    verts_2d: &[(f64, f64)],
) {
    let n = loop_vertex_indices.len();
    if n < 3 || loop_points_projected.len() != n || verts_2d.len() != n {
        return;
    }
    // 3D centroid as the mean of the projected perimeter positions.
    // Because the perimeter was projected onto the fit plane upstream
    // (CSP.3a), the arithmetic mean is also on the plane → cap stays
    // planar.
    let mut sum = Vector3::zeros();
    for p in loop_points_projected {
        sum += p.coords;
    }
    #[allow(clippy::cast_precision_loss)]
    let centroid_3d = Point3::from(sum / n as f64);
    #[allow(clippy::cast_possible_truncation)]
    let centroid_global_idx = mesh.vertices.len() as u32;
    mesh.vertices.push(centroid_3d);

    // 2D signed area (shoelace) determines perimeter winding.
    let signed_area_2d: f64 = (0..n)
        .map(|i| {
            let (x0, y0) = verts_2d[i];
            let (x1, y1) = verts_2d[(i + 1) % n];
            x0 * y1 - x1 * y0
        })
        .sum::<f64>()
        * 0.5;

    for i in 0..n {
        let next_i = (i + 1) % n;
        let a = loop_vertex_indices[i];
        let b = loop_vertex_indices[next_i];
        if signed_area_2d >= 0.0 {
            mesh.faces.push([centroid_global_idx, a, b]);
        } else {
            // CW perimeter — flip to match the OUTWARD direction.
            mesh.faces.push([centroid_global_idx, b, a]);
        }
    }
}

// ----- Centerline reconstruction (CSP.4e.2) -------------------------
//
// Replace a chopped floor region with extruded average-cross-section
// geometry so the cleaned mesh keeps its original length. The user
// dials the reference-zone slider + picks a shape (Constant in 4e.2;
// Taper + Extrapolate in 4e.3/4e.4); this code consumes those values
// to build new mesh geometry.
//
// Algorithm (Constant shape):
// 1. Find the floor-end open boundary loop (closest to the polyline
//    endpoint nearest the chopped end).
// 2. Build a local frame at the cut: tangent (along the polyline),
//    plus two perpendicular basis vectors u, v.
// 3. Walk every mesh vertex; collect the ones within
//    `reference_zone_m` of the cut (along the tangent direction
//    INTO the mesh). For each, compute (angle = atan2(v·d, u·d),
//    radial = √(u² + v²)). Bin angles into M bins, take the median
//    radial per bin → canonical r(angle) profile.
// 4. Project boundary-loop vertices onto the cut plane and snap
//    them to the canonical profile (each vertex moves radially to
//    r(angle_at_that_vertex)). The loop topology stays connected
//    to the mesh body; only the rim vertices move.
// 5. Extrude K rings DOWN the centerline (each ring has the same L
//    angular positions as the boundary loop, at canonical radii
//    for Constant shape). Connect each pair of rings with triangle
//    strips.
// 6. Add a flat bottom cap fanned from a centroid vertex at the
//    new floor.

/// Number of angular bins used when sampling the radial profile.
/// 24 = one bin every 15°. Coarser than the typical boundary-loop
/// vertex count (hundreds) so each bin has multiple samples to
/// median-filter against scan noise.
pub const RECONSTRUCT_ANGLE_BINS: usize = 24;

/// Number of subdivisions along the extruded sidewall. 8 keeps the
/// reconstruction lightweight (8 × L new triangles per band, L =
/// boundary loop vertex count); enough for visual smoothness on
/// the sock fixture's mostly-straight floor region.
pub const RECONSTRUCT_RING_COUNT: usize = 8;

/// Decide which of `loops` is the "floor end" loop — the LARGEST
/// valid boundary loop. Used by [`apply_reconstruction`] to
/// single out the loop the user wants to reconstruct (the cut
/// rim) vs. scan-noise stragglers.
///
/// CSP.4e.2 fix-forward (2026-05-15): the initial implementation
/// picked the loop whose centroid was closest to the centerline
/// endpoint. On an unsimplified scan (iter-1 fixture: 1215
/// boundary loops, mostly 3-vertex stragglers), that heuristic
/// picked a tiny noise loop at random whose centroid happened to
/// be closest. The reconstruction then generated a degenerate
/// thin column at that location ("white vertical line" the user
/// reported). The cut rim is overwhelmingly the largest loop on
/// any practical scan, so picking by vertex count is robust.
///
/// Loops with fewer than `MIN_RIM_LOOP_VERTS` (10) vertices are
/// filtered out — they're scanner-noise stragglers, never the
/// actual rim. Among the remaining, the largest wins.
///
/// `centerline_last_point` is accepted for future use (e.g., a
/// distance-based tiebreaker when two large loops exist —
/// multi-shell scans) but unused in the current pick-by-count
/// implementation.
///
/// Returns the loop's index in `loops`, or `None` when no
/// sufficiently-large valid loop exists.
pub fn find_floor_loop_index(
    loops: &[holes::BoundaryLoop],
    _mesh: &IndexedMesh,
    _centerline_last_point: Point3<f64>,
) -> Option<usize> {
    const MIN_RIM_LOOP_VERTS: usize = 10;
    let mut best: Option<(usize, usize)> = None;
    for (i, lp) in loops.iter().enumerate() {
        if !lp.is_valid() {
            continue;
        }
        let count = lp.vertices.len();
        if count < MIN_RIM_LOOP_VERTS {
            continue;
        }
        if best.is_none_or(|(_, c)| count > c) {
            best = Some((i, count));
        }
    }
    best.map(|(i, _)| i)
}

/// Build an orthonormal basis `(u, v)` perpendicular to the unit
/// vector `n`. Used to project 3D points around the centerline
/// into a 2D (radial, angular) representation.
///
/// Standard Gram-Schmidt against a world axis that isn't parallel
/// to `n`. Matches the heuristic in [`project_loop_to_plane_2d`]
/// for consistency.
pub fn perpendicular_basis_for(n: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let world_axis = if n.x.abs() < 0.9 {
        Vector3::new(1.0, 0.0, 0.0)
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    };
    let u = (world_axis - n * world_axis.dot(&n)).normalize();
    let v = n.cross(&u).normalize();
    (u, v)
}

/// Reconstructed-floor plane recorded in the `.prep.toml` `[caps]`
/// block when the user applied centerline-driven floor reconstruction.
///
/// After [`apply_reconstruction`] extends the chopped floor by
/// `applied_floor_mm`, the cleaned mesh's actual closed-floor plane
/// sits at `centerline_last + (-inward_tangent) * extension_m` — NOT
/// at the original cut boundary's fit plane that
/// [`CapState::loops`] records (those were detected from the raw scan
/// BEFORE reconstruction added the extrusion + cap fan).
///
/// Without this override, downstream consumers (cf-cap-planes →
/// cf-device-design candidate-A pinned-floor) clip the cavity against
/// the stale pre-reconstruction plane, which lands MID-BODY (verified
/// 2.73 mm offset on iter-1 sock_over_capsule). The cavity's iso=0
/// then traces a mid-body slice and the marching-cubes reconstruction
/// shows "dripping-wax" rim artifacts at the cap plane (iter-1 visual
/// gate failure, 2026-05-17). Recording the post-reconstruction plane
/// here aligns the .prep.toml's `[caps]` contract with the actual
/// cleaned-STL floor.
#[derive(Debug, Clone, Copy)]
pub struct ReconstructedFloorPlane {
    /// Floor centroid in PRE-BAKE physics-frame meters — `bottom_center`
    /// from [`apply_reconstruction`]'s computation: trimmed centerline
    /// endpoint plus `extension_m` along the outward extrusion direction.
    pub centroid_m: Point3<f64>,
    /// Outward unit normal in PRE-BAKE physics-frame coordinates.
    /// Mirrors [`orient_cap_normal_outward`]'s convention: points AWAY
    /// from body interior (into the chopped-end half-space). Equals
    /// `-inward_tangent`.
    pub normal: Vector3<f64>,
}

/// Compute the reconstructed-floor plane in PRE-BAKE physics frame
/// from the in-memory centerline polyline + applied trim values.
///
/// Mirrors the `bottom_center` + `extrusion_dir` calculation inside
/// [`apply_reconstruction`] so the recorded plane matches the actual
/// reconstructed floor of the cleaned mesh. Operates on the physics-
/// frame polyline (cf-scan-prep's `CapState::centerline_polyline`)
/// directly; the bake transform that the cleaned mesh goes through
/// is applied to neither input nor output, so the result lives in
/// the same frame as [`DetectedCapLoop::plane_centroid`] / `plane_normal`
/// — which is what the .prep.toml's `[caps]` block contract expects
/// (cf-cap-planes' `parse_cap_planes` bakes the recorded plane through
/// the `[transform]` block at load time).
///
/// Returns `None` when reconstruction would not produce a usable floor
/// plane: zero floor trim, polyline too short after trimming, or a
/// degenerate inward tangent.
pub fn compute_reconstructed_floor_plane_physics(
    centerline_polyline_physics: &[Point3<f64>],
    applied_tip_mm: f64,
    applied_floor_mm: f64,
) -> Option<ReconstructedFloorPlane> {
    if applied_floor_mm <= 0.0 {
        return None;
    }
    let trimmed = trim_centerline_polyline(
        centerline_polyline_physics,
        applied_tip_mm,
        applied_floor_mm,
    );
    if trimmed.len() < 2 {
        return None;
    }
    let n_last = trimmed[trimmed.len() - 1];
    // Same tangent posture as `apply_reconstruction`: prefer the
    // ~20 mm look-back average for axis-stability, fall back to the
    // last-segment vector for short polylines.
    let inward_tangent = stable_inward_tangent(&trimmed, STABLE_INWARD_TANGENT_LOOKBACK_M)
        .or_else(|| {
            let n = trimmed.len();
            let tangent_raw = trimmed[n - 2].coords - trimmed[n - 1].coords;
            let norm = tangent_raw.norm();
            if norm < f64::EPSILON {
                None
            } else {
                Some(tangent_raw / norm)
            }
        })?;
    let extrusion_dir = -inward_tangent;
    let extension_m = applied_floor_mm * 0.001;
    let centroid_m = Point3::from(n_last.coords + extrusion_dir * extension_m);
    Some(ReconstructedFloorPlane {
        centroid_m,
        normal: extrusion_dir,
    })
}

/// Look-back distance for [`stable_inward_tangent`] in meters.
/// 20 mm averages enough centerline segments on a typical
/// workshop scan (segment density ~5 mm; 20 mm = ~4 segments)
/// to wash out the noisy single-segment tangent at the
/// post-trim cut endpoint, while staying short enough that the
/// resulting direction tracks the body's local axis (not the
/// whole-body PCA average).
pub const STABLE_INWARD_TANGENT_LOOKBACK_M: f64 = 0.020;

/// Estimate a stable inward tangent at the centerline polyline's
/// last (cut) endpoint by walking back along the polyline by
/// `lookback_m` arc-length, then taking the unit vector from
/// that look-back point to the cut endpoint and **negating** it
/// so the result points FROM the cut INTO the body (matching
/// the convention used by [`apply_reconstruction`]'s
/// `inward_tangent`).
///
/// Falls back to the head→cut direction when `lookback_m`
/// exceeds the polyline's total arc length, and to the
/// last-segment vector when the polyline has only two points.
/// Returns `None` for degenerate inputs (single-point polyline,
/// non-positive lookback, or coincident look-back/cut points).
///
/// **Why this helper exists** (2026-05-16): the pre-fix
/// `apply_reconstruction` used `centerline[n-2] - centerline[n-1]`
/// directly. On the iter-1 sock-over-capsule scan that single
/// segment wandered laterally relative to the body's main axis,
/// causing (a) the K-ring extrusion to extend off-axis and (b)
/// the blend pass to project scan vertices onto a tilted global
/// frame, producing visible spike artifacts. Look-back averaging
/// over ~20 mm tames both.
pub fn stable_inward_tangent(centerline: &[Point3<f64>], lookback_m: f64) -> Option<Vector3<f64>> {
    let n = centerline.len();
    if n < 2 || lookback_m <= 0.0 {
        return None;
    }
    let cut = centerline[n - 1];
    // Walk segments from the cut endpoint inward, accumulating
    // arc-length. Stop when `accumulated + this_segment >=
    // lookback_m` and interpolate within that segment so the
    // look-back point is exactly `lookback_m` from the cut.
    let mut accumulated = 0.0;
    // Default: if every segment is degenerate (zero-length), fall
    // back to centerline[n-2] (the immediate inward neighbor).
    let mut lookback_point = centerline[n - 2];
    let mut found = false;
    for i in (0..n - 1).rev() {
        let seg = centerline[i].coords - centerline[i + 1].coords;
        let seg_len = seg.norm();
        if seg_len < f64::EPSILON {
            continue;
        }
        if accumulated + seg_len >= lookback_m {
            let remaining = lookback_m - accumulated;
            let t = remaining / seg_len;
            lookback_point = centerline[i + 1] + seg * t;
            found = true;
            break;
        }
        accumulated += seg_len;
        lookback_point = centerline[i];
    }
    // If we exhausted the polyline without reaching `lookback_m`,
    // the last assignment of `lookback_point` is the head of the
    // polyline (centerline[0] or last non-degenerate inward
    // point) — that's the right fallback for short polylines.
    let _ = found;

    let dir = lookback_point.coords - cut.coords;
    let norm = dir.norm();
    if norm < f64::EPSILON {
        return None;
    }
    Some(dir / norm)
}

/// Bin the mesh vertices in the reference zone above the cut by
/// angular position around the centerline, then take the median
/// radial distance per bin. Returns a length-M array of radii (M =
/// [`RECONSTRUCT_ANGLE_BINS`]).
///
/// Robust to noise: per-bin **median** rather than mean. A single
/// stray vertex doesn't pull the bin's radius.
///
/// Bins with zero samples fall back to the overall median radius
/// (so the bottom-fan reconstruction stays well-defined even with
/// patchy reference data).
pub fn sample_radial_profile(
    mesh: &IndexedMesh,
    cut_point: Point3<f64>,
    inward_tangent: Vector3<f64>,
    u: Vector3<f64>,
    v: Vector3<f64>,
    reference_zone_m: f64,
) -> [f64; RECONSTRUCT_ANGLE_BINS] {
    let mut bins: Vec<Vec<f64>> = (0..RECONSTRUCT_ANGLE_BINS).map(|_| Vec::new()).collect();
    let bin_span = std::f64::consts::TAU / RECONSTRUCT_ANGLE_BINS as f64;
    for vtx in &mesh.vertices {
        let d = vtx.coords - cut_point.coords;
        let along = d.dot(&inward_tangent);
        // Sample only the slab ABOVE the cut (toward the mesh
        // interior), within `reference_zone_m`. `along > 0` ⇒ on
        // the interior side; `along < reference_zone_m` ⇒ within
        // the zone.
        if along <= 0.0 || along > reference_zone_m {
            continue;
        }
        let proj_u = d.dot(&u);
        let proj_v = d.dot(&v);
        let r = (proj_u * proj_u + proj_v * proj_v).sqrt();
        let angle = proj_v.atan2(proj_u);
        let normalized = if angle >= 0.0 {
            angle
        } else {
            angle + std::f64::consts::TAU
        };
        let bin = ((normalized / bin_span) as usize).min(RECONSTRUCT_ANGLE_BINS - 1);
        bins[bin].push(r);
    }

    // Compute per-bin median + a fallback "overall median" for
    // empty bins.
    let mut radii = [0.0_f64; RECONSTRUCT_ANGLE_BINS];
    let mut all_radii: Vec<f64> = bins.iter().flat_map(|b| b.iter().copied()).collect();
    let overall = if all_radii.is_empty() {
        0.0
    } else {
        all_radii.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        all_radii[all_radii.len() / 2]
    };
    for (i, bin) in bins.iter_mut().enumerate() {
        if bin.is_empty() {
            radii[i] = overall;
        } else {
            bin.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            radii[i] = bin[bin.len() / 2];
        }
    }
    radii
}

/// Same reference-zone walk as [`sample_radial_profile`] but
/// fits a **per-angle-bin linear regression** `r(s) = a + b·s`
/// instead of taking the median. Used by the Extrapolate shape
/// variant (CSP.4e.3.b) — extrapolating below the cut at `s < 0`
/// gives a profile that continues the reference-zone trend
/// (e.g., a sock that narrows toward the rim keeps narrowing
/// below the cut).
///
/// Returns `(intercepts, slopes)` — both length-M arrays:
/// - `intercepts[i]` = `a` for bin i (radius at `s = 0`, the cut)
/// - `slopes[i]`     = `b` for bin i (mm of radius per mm of s)
///
/// Per-bin samples with < 2 points fall back to (overall median,
/// slope 0) — flat reconstruction for that angle.
pub fn sample_radial_profile_linear_fit(
    mesh: &IndexedMesh,
    cut_point: Point3<f64>,
    inward_tangent: Vector3<f64>,
    u: Vector3<f64>,
    v: Vector3<f64>,
    reference_zone_m: f64,
) -> ([f64; RECONSTRUCT_ANGLE_BINS], [f64; RECONSTRUCT_ANGLE_BINS]) {
    let mut bins: Vec<Vec<(f64, f64)>> = (0..RECONSTRUCT_ANGLE_BINS).map(|_| Vec::new()).collect();
    let bin_span = std::f64::consts::TAU / RECONSTRUCT_ANGLE_BINS as f64;
    let mut all_radii: Vec<f64> = Vec::new();
    for vtx in &mesh.vertices {
        let d = vtx.coords - cut_point.coords;
        let s = d.dot(&inward_tangent);
        if s <= 0.0 || s > reference_zone_m {
            continue;
        }
        let proj_u = d.dot(&u);
        let proj_v = d.dot(&v);
        let r = (proj_u * proj_u + proj_v * proj_v).sqrt();
        let angle = proj_v.atan2(proj_u);
        let normalized = if angle >= 0.0 {
            angle
        } else {
            angle + std::f64::consts::TAU
        };
        let bin = ((normalized / bin_span) as usize).min(RECONSTRUCT_ANGLE_BINS - 1);
        bins[bin].push((s, r));
        all_radii.push(r);
    }
    // Fallback overall median for sparse bins.
    let overall = if all_radii.is_empty() {
        0.0
    } else {
        all_radii.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        all_radii[all_radii.len() / 2]
    };
    let mut intercepts = [0.0_f64; RECONSTRUCT_ANGLE_BINS];
    let mut slopes = [0.0_f64; RECONSTRUCT_ANGLE_BINS];
    for (i, samples) in bins.iter().enumerate() {
        if samples.len() < 2 {
            intercepts[i] = overall;
            slopes[i] = 0.0;
            continue;
        }
        #[allow(clippy::cast_precision_loss)]
        let n = samples.len() as f64;
        let sum_s: f64 = samples.iter().map(|(s, _)| s).sum();
        let sum_r: f64 = samples.iter().map(|(_, r)| r).sum();
        let sum_s2: f64 = samples.iter().map(|(s, _)| s * s).sum();
        let sum_sr: f64 = samples.iter().map(|(s, r)| s * r).sum();
        let denom = n * sum_s2 - sum_s * sum_s;
        if denom.abs() < f64::EPSILON {
            intercepts[i] = sum_r / n;
            slopes[i] = 0.0;
        } else {
            slopes[i] = (n * sum_sr - sum_s * sum_r) / denom;
            intercepts[i] = (sum_r - slopes[i] * sum_s) / n;
        }
    }
    (intercepts, slopes)
}

/// Taper rate for the Taper shape variant (CSP.4e.3.a). The new
/// floor's radius is `1 - TAPER_AT_FLOOR` × the canonical
/// profile; intermediate rings linearly interpolate. 0.3 = 30%
/// reduction at the floor → visible-but-not-extreme pinch.
pub const RECONSTRUCT_TAPER_AT_FLOOR: f64 = 0.3;

/// Linearly interpolate the canonical radius at angle `angle`
/// (radians, any value) from the M-bin profile.
pub fn sample_radius_at_angle(radii: &[f64; RECONSTRUCT_ANGLE_BINS], angle: f64) -> f64 {
    let bin_span = std::f64::consts::TAU / RECONSTRUCT_ANGLE_BINS as f64;
    let normalized = if angle >= 0.0 {
        angle
    } else {
        angle + std::f64::consts::TAU
    };
    let pos = (normalized / bin_span) % RECONSTRUCT_ANGLE_BINS as f64;
    let lo_bin = (pos as usize) % RECONSTRUCT_ANGLE_BINS;
    let hi_bin = (lo_bin + 1) % RECONSTRUCT_ANGLE_BINS;
    let t = pos - pos.floor();
    radii[lo_bin] * (1.0 - t) + radii[hi_bin] * t
}

/// Apply floor reconstruction to `mesh`. The mesh MUST already be
/// trim-cut (open boundary at the floor end); other open
/// boundaries (e.g., tip-end if tip was also trimmed) fall
/// through to flat auto-cap inside this function's degenerate
/// paths. CSP.4e.2 (Constant), CSP.4e.3 (Taper, Extrapolate).
///
/// `shape` controls how the per-ring radius is computed as we
/// extrude down the centerline from the cut to the new floor:
/// - `Constant` — every ring uses the canonical median profile
///   at the cut (cylindrical extrusion).
/// - `Taper`    — linear scaling from canonical at the cut to
///   `(1 - RECONSTRUCT_TAPER_AT_FLOOR) × canonical` at the new
///   floor.
/// - `Extrapolate` — per-angle linear regression `r(s) = a + b·s`
///   across the reference zone; extrapolate to `s < 0` (below
///   the cut) for each ring. Captures the natural taper of the
///   reference geometry.
///
/// `centerline` is the POST-trim polyline in the same frame as
/// `mesh` (physics-frame meters, pre-bake under the current live-
/// preview pipeline).
///
/// # Seam handling (2026-05-16)
///
/// The transition from the noisy scan above the cut to the smooth
/// reconstruction below used to produce a visible "lip": the
/// floor-loop's noisy radii didn't match the smoothed canonical
/// profile, leaving a ridge at the join. **Always-on fix**: the
/// floor-loop vertices are NOT snapped to the canonical profile;
/// they BECOME the top extrusion ring as-is, and each subsequent
/// ring `k` lerps toward the canonical profile via a smoothstep
/// weight (0 at the top, 1 at the new floor). The bottom flat
/// cap still sits on the fully-smooth canonical profile, so the
/// reconstruction doesn't carry the noise into the floor. Combined
/// with the [`stable_inward_tangent`]-driven extrusion direction,
/// this produces a clean seam without any user-tunable knob.
///
/// (A scan-side blend-zone slider was prototyped 2026-05-16 and
/// removed the same session: the local-frame projection it needed
/// produced visible artifacts on the iter-1 sock-over-capsule
/// fixture at any non-zero blend, and `blend_zone_mm = 0` already
/// looked "pretty much perfect" per user verification. The slider
/// was carrying surface area without earning it.)
pub fn apply_reconstruction(
    mut mesh: IndexedMesh,
    centerline: &[Point3<f64>],
    applied_floor_mm: f64,
    reference_zone_mm: f64,
    shape: ReconstructShape,
) -> IndexedMesh {
    if centerline.len() < 2 || applied_floor_mm <= 0.0 || reference_zone_mm <= 0.0 {
        // Nothing to reconstruct — fall back to flat-cap.
        auto_cap_open_boundaries(&mut mesh);
        return mesh;
    }
    let loops = detect_boundary_loops(&mesh);
    let centerline_last = centerline[centerline.len() - 1];
    let Some(floor_idx) = find_floor_loop_index(&loops, &mesh, centerline_last) else {
        auto_cap_open_boundaries(&mut mesh);
        return mesh;
    };

    // Build the local frame at the cut. `inward_tangent` points
    // FROM the cut endpoint AWAY from the chopped end (i.e., back
    // INTO the mesh body) — that's the direction of `polyline[N-2]
    // - polyline[N-1]` after trim trimmed the polyline.
    // Stable inward tangent: walk back along the centerline by
    // ~20 mm and take the dir-into-body. Replaces the prior
    // single-segment `centerline[n-2] - centerline[n-1]` which
    // wandered laterally on noisy polylines, tilting the K-ring
    // extrusion direction off the body's actual axis. Falls
    // back to the single-segment vector for short polylines.
    let inward_tangent =
        if let Some(t) = stable_inward_tangent(centerline, STABLE_INWARD_TANGENT_LOOKBACK_M) {
            t
        } else {
            let n = centerline.len();
            let tangent_raw = centerline[n - 2].coords - centerline[n - 1].coords;
            let tangent_norm = tangent_raw.norm();
            if tangent_norm < f64::EPSILON {
                auto_cap_open_boundaries(&mut mesh);
                return mesh;
            }
            tangent_raw / tangent_norm
        };
    let (u_axis, v_axis) = perpendicular_basis_for(inward_tangent);

    // Sample the canonical radial profile from the reference zone.
    // For Constant + Taper we only need per-bin medians; for
    // Extrapolate we ALSO need per-bin linear-regression slopes
    // so we can evaluate `r(s) = a + b·s` at `s < 0` (below the
    // cut). The closure below dispatches per-shape.
    let base_radii = sample_radial_profile(
        &mesh,
        centerline_last,
        inward_tangent,
        u_axis,
        v_axis,
        reference_zone_mm * 0.001,
    );
    let extrap_fit = if matches!(shape, ReconstructShape::Extrapolate) {
        Some(sample_radial_profile_linear_fit(
            &mesh,
            centerline_last,
            inward_tangent,
            u_axis,
            v_axis,
            reference_zone_mm * 0.001,
        ))
    } else {
        None
    };
    let extension_m = applied_floor_mm * 0.001;
    // `t` ∈ [0, 1]: 0 = top ring at the cut, 1 = bottom ring at
    // the new floor. Returns the per-angle radius for that ring.
    let radius_at = |angle: f64, t: f64| -> f64 {
        let base = sample_radius_at_angle(&base_radii, angle);
        match shape {
            ReconstructShape::Constant => base,
            ReconstructShape::Taper => base * (1.0 - RECONSTRUCT_TAPER_AT_FLOOR * t),
            ReconstructShape::Extrapolate => {
                // CSP.4e.3.b — for Extrapolate, evaluate the
                // per-angle linear fit at `s = -extension_m × t`
                // (negative s = below cut). Fall back to Constant
                // if the fit isn't available (shouldn't happen).
                if let Some((a_arr, b_arr)) = extrap_fit.as_ref() {
                    let a = sample_radius_at_angle(a_arr, angle);
                    let b = sample_radius_at_angle(b_arr, angle);
                    let s = -extension_m * t;
                    let r = a + b * s;
                    // Clamp non-negative — an aggressive trend
                    // could project to negative radius for a long
                    // extrusion. Below ~0 the mesh would
                    // self-cross the centerline.
                    r.max(0.0)
                } else {
                    base
                }
            }
        }
    };

    // Capture the floor-loop's per-vertex angle + radius as the
    // top extrusion ring. **Anti-lip**: we use the noisy raw
    // radii as-is (no snap to the canonical profile), so the
    // top ring matches the boundary the scan is welded to —
    // no geometric step at the seam. The K-ring loop below
    // smoothsteps each subsequent ring toward the canonical
    // profile, fading the noise out over K rings.
    let floor_loop = loops[floor_idx].clone();
    let l = floor_loop.vertices.len();
    let mut top_ring_angles = Vec::with_capacity(l);
    let mut top_ring_radii = Vec::with_capacity(l);
    for &vidx in &floor_loop.vertices {
        if let Some(p) = mesh.vertices.get(vidx as usize).copied() {
            let d = p.coords - centerline_last.coords;
            let proj_u = d.dot(&u_axis);
            let proj_v = d.dot(&v_axis);
            let angle = proj_v.atan2(proj_u);
            let r = (proj_u * proj_u + proj_v * proj_v).sqrt();
            top_ring_angles.push(angle);
            top_ring_radii.push(r);
        } else {
            top_ring_angles.push(0.0);
            top_ring_radii.push(0.0);
        }
    }

    // Generate K extrusion rings DOWN past the cut (in the
    // direction OPPOSITE the inward tangent — outward toward the
    // original chopped position). Each ring's per-angle radius is
    // a smoothstep lerp from `top_ring_radii` (k=0, the floor loop)
    // to the canonical `radius_at(angle, t_k)` (k=K, the new floor).
    // At blend_zone=0 this gradual smoothing IS the anti-lip — the
    // noisy top ring doesn't snap to the smooth profile abruptly.
    // At blend_zone > 0 the top ring is already on the canonical
    // profile (from the blend pass), so the lerp degenerates to
    // smooth-all-the-way.
    let extrusion_dir = -inward_tangent;
    let mut prev_ring: Vec<u32> = floor_loop.vertices.clone();
    for k in 1..=RECONSTRUCT_RING_COUNT {
        #[allow(clippy::cast_precision_loss)]
        let t_k = k as f64 / RECONSTRUCT_RING_COUNT as f64;
        let ring_blend = t_k * t_k * (3.0 - 2.0 * t_k);
        let ring_center = centerline_last.coords + extrusion_dir * extension_m * t_k;
        let mut this_ring: Vec<u32> = Vec::with_capacity(l);
        for (i, &angle) in top_ring_angles.iter().enumerate() {
            let r_smooth = radius_at(angle, t_k);
            let r_top = top_ring_radii[i];
            let r = r_top * (1.0 - ring_blend) + r_smooth * ring_blend;
            let new_pos = ring_center + r * (u_axis * angle.cos() + v_axis * angle.sin());
            #[allow(clippy::cast_possible_truncation)]
            let idx = mesh.vertices.len() as u32;
            mesh.vertices.push(Point3::from(new_pos));
            this_ring.push(idx);
        }
        // Triangle strip between prev_ring (top) and this_ring (bottom).
        // For each i: quad (prev[i], prev[i+1], this[i+1], this[i]).
        // Triangulate as (prev[i], prev[i+1], this[i+1]) +
        // (prev[i], this[i+1], this[i]). Winding chosen so the
        // outward normal points radially AWAY from the centerline.
        for i in 0..l {
            let a = prev_ring[i];
            let b = prev_ring[(i + 1) % l];
            let c = this_ring[(i + 1) % l];
            let d = this_ring[i];
            mesh.faces.push([a, b, c]);
            mesh.faces.push([a, c, d]);
        }
        prev_ring = this_ring;
    }

    // Bottom flat cap: fan from a centroid vertex at the extrusion
    // tip. Winding: outward normal points FURTHER along
    // `extrusion_dir` (away from the mesh body).
    let bottom_center = centerline_last.coords + extrusion_dir * extension_m;
    #[allow(clippy::cast_possible_truncation)]
    let center_idx = mesh.vertices.len() as u32;
    mesh.vertices.push(Point3::from(bottom_center));
    for i in 0..l {
        let a = prev_ring[i];
        let b = prev_ring[(i + 1) % l];
        mesh.faces.push([a, b, center_idx]);
    }

    // Verify winding empirically: pick the first sidewall triangle,
    // compute its normal, check if it points radially OUTWARD. If
    // not, flip all the sidewall + cap face winding. This is a
    // one-shot heuristic — the boundary loop's CCW-vs-CW direction
    // depends on which end was trimmed, so we can't pre-compute.
    let first_sidewall_face_idx = mesh.faces.len() - 2 * l * RECONSTRUCT_RING_COUNT - l;
    if let Some(&[a, b, c]) = mesh.faces.get(first_sidewall_face_idx) {
        let va = mesh.vertices[a as usize];
        let vb = mesh.vertices[b as usize];
        let vc = mesh.vertices[c as usize];
        let normal = (vb.coords - va.coords).cross(&(vc.coords - va.coords));
        // Radial outward at va: va - ring_axis_point_at_same_along.
        // Use the cut_point as a proxy for the radial-from-centerline
        // origin at the top ring (it's close enough for the sign check).
        let radial_at_va = va.coords - centerline_last.coords;
        // Project out the along-tangent component to get the pure radial.
        let radial_only = radial_at_va - inward_tangent * radial_at_va.dot(&inward_tangent);
        if normal.dot(&radial_only) < 0.0 {
            // Flip every new face we added (last 2*L*K + L faces).
            let new_face_count = 2 * l * RECONSTRUCT_RING_COUNT + l;
            let start = mesh.faces.len() - new_face_count;
            for face in mesh.faces.iter_mut().skip(start) {
                face.swap(1, 2);
            }
        }
    }

    // CSP.4e fix-forward (PR #246 cold-read review, 2026-05-15) —
    // seal any OTHER open boundaries we didn't reconstruct. The
    // floor end is closed by the extrusion sidewalls + bottom fan
    // above; this call only acts on remaining open loops (typically
    // the tip end if the user also trimmed there). Without it, a
    // dual-end-trim + reconstruct workflow emits a non-watertight
    // cleaned STL that breaks downstream cf-cast-cli offset/simplify.
    auto_cap_open_boundaries(&mut mesh);

    mesh
}

/// Detect all open boundary loops in `mesh`. Wraps `mesh-repair`'s
/// `detect_holes`, which builds adjacency + walks boundary edges into
/// closed vertex-index loops. Returns a `Vec<BoundaryLoop>` (each
/// loop's `vertices` is an ordered list of vertex indices closing
/// back to vertex 0).
pub fn detect_boundary_loops(mesh: &IndexedMesh) -> Vec<holes::BoundaryLoop> {
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
pub fn fit_plane_to_points(points: &[Point3<f64>]) -> (Point3<f64>, Vector3<f64>, f64) {
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
pub fn orient_cap_normal_outward(
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
pub fn build_detected_cap_loop(
    mesh: &IndexedMesh,
    loop_data: &holes::BoundaryLoop,
) -> DetectedCapLoop {
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

/// Identifies a mesh edge by the unordered pair of its vertex
/// indices (`lo <= hi`). Used as a hash key to deduplicate
/// plane-edge intersection points across the two faces that share
/// each interior mesh edge — two adjacent triangles' segments meet
/// at the SAME intersection point because they share the same edge
/// key, so segment chaining is robust without coordinate-tolerance
/// matching.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct MeshEdgeKey {
    lo: u32,
    hi: u32,
}

impl MeshEdgeKey {
    fn new(a: u32, b: u32) -> Self {
        Self {
            lo: a.min(b),
            hi: a.max(b),
        }
    }
}

/// SOS perturbation magnitude (meters) used by
/// [`intersect_plane_with_mesh`] to push vertices with signed
/// distance below this magnitude consistently to the positive side
/// of the plane. With every vertex strictly off-plane after the
/// perturbation, each plane-crossing triangle produces a well-
/// defined 2-edge segment (no vertex-on-plane degenerate cases).
/// 1e-12 m = 1 picometer — well below any geometric feature on
/// body-part scans (mm scale) and far below f64 precision around
/// typical coordinate magnitudes (≤ 1 m).
pub const PLANE_INTERSECTION_ON_PLANE_EPS_M: f64 = 1e-12;

/// Intersect a plane with a triangle mesh and return the resulting
/// cross-section as one or more closed polygon loops.
///
/// **Inputs**: `plane_point` is any point on the plane (used as the
/// plane's origin for signed-distance computation); `plane_normal`
/// MUST be unit-magnitude (caller normalizes once); `mesh` is a
/// watertight indexed triangle mesh (input meshes from cf-scan-prep
/// post-Cap are watertight by construction).
///
/// **Output**: each inner `Vec<Point3<f64>>` is the ordered vertex
/// sequence of a closed polygon loop; the implicit last edge
/// connects `loop[n-1]` back to `loop[0]`. Order around each loop
/// is consistent (traversal-walk order) but the WINDING (CCW vs.
/// CW with respect to `plane_normal`) is arbitrary — downstream
/// consumers like [`polygon_centroid_3d`] handle both orientations
/// via signed-area cancellation.
///
/// **Algorithm**:
///
/// 1. **Signed distance per vertex** with SOS perturbation — any
///    vertex with `|dist| < PLANE_INTERSECTION_ON_PLANE_EPS_M` is
///    snapped to `+EPS`. After this no vertex is exactly on the
///    plane, so every plane-crossing triangle has signs that are
///    strictly `(+, +, -)` or `(+, -, -)` — exactly 2 of its 3
///    edges have endpoints of opposite sign.
/// 2. **Per-face intersection segments** — for each plane-crossing
///    triangle, compute the linear interpolation parameter on its
///    two sign-flipping edges. Each endpoint is identified by a
///    [`MeshEdgeKey`] so the SAME intersection point is shared
///    between the two faces adjacent to that mesh edge (no
///    coordinate dedup needed — the key is exact).
/// 3. **Segment graph** — `adjacency: MeshEdgeKey → Vec<MeshEdgeKey>`
///    records the segments. In a watertight mesh each crossing edge
///    has exactly 2 neighbors (one from each adjacent face), so the
///    graph decomposes into disjoint cycles.
/// 4. **Loop extraction** — walk the graph starting from each
///    unvisited key, following adjacency (avoid the previous node)
///    until the start is revisited. Drop fragments shorter than 3
///    points (degenerate / open boundary remnants).
///
/// **Edge cases**:
/// - Empty mesh / zero-magnitude normal: returns `Vec::new()`.
/// - Plane outside the mesh: returns `Vec::new()` (no triangles
///   have mixed signs).
/// - Non-convex body (e.g., a torus slice): returns multiple loops
///   — the caller (centerline algorithm) picks the largest-area
///   loop for centroid computation.
/// - Open / non-watertight mesh at the slice: incomplete loops
///   (segments with dead ends) are dropped by the `len >= 3` filter
///   — caller sees fewer / smaller loops than expected.
pub fn intersect_plane_with_mesh(
    plane_point: &Point3<f64>,
    plane_normal: &Vector3<f64>,
    mesh: &IndexedMesh,
) -> Vec<Vec<Point3<f64>>> {
    if mesh.vertices.is_empty()
        || mesh.faces.is_empty()
        || plane_normal.norm_squared() < f64::EPSILON
    {
        return Vec::new();
    }
    // Caller's contract is to pass a unit normal; defensive
    // re-normalize is one sqrt and protects against drift.
    let n = plane_normal.normalize();

    // Step 1: signed distance per vertex, with SOS perturbation.
    let dists: Vec<f64> = mesh
        .vertices
        .iter()
        .map(|v| {
            let raw = n.dot(&(v.coords - plane_point.coords));
            if raw.abs() < PLANE_INTERSECTION_ON_PLANE_EPS_M {
                PLANE_INTERSECTION_ON_PLANE_EPS_M
            } else {
                raw
            }
        })
        .collect();

    // Step 2-3: per-face intersection + segment graph build.
    let mut point_at: HashMap<MeshEdgeKey, Point3<f64>> = HashMap::new();
    let mut adjacency: HashMap<MeshEdgeKey, Vec<MeshEdgeKey>> = HashMap::new();

    for face in &mesh.faces {
        let signs = [
            dists[face[0] as usize].signum(),
            dists[face[1] as usize].signum(),
            dists[face[2] as usize].signum(),
        ];
        let any_pos = signs.iter().any(|&s| s > 0.0);
        let any_neg = signs.iter().any(|&s| s < 0.0);
        if !(any_pos && any_neg) {
            continue;
        }

        // Find the two edges of this triangle whose endpoints have
        // opposite signs (these are the two edges the plane crosses).
        // After SOS perturbation, a mixed-sign triangle has exactly
        // 2 sign-flipping edges, so `crossing.len() == 2` is the
        // expected case; the defensive `if let` skips any anomaly.
        let mut crossing: Vec<MeshEdgeKey> = Vec::with_capacity(2);
        for e in 0..3 {
            let i = face[e];
            let j = face[(e + 1) % 3];
            if signs[e] != signs[(e + 1) % 3] {
                let key = MeshEdgeKey::new(i, j);
                crossing.push(key);

                // Compute the intersection point on this edge (once
                // per edge, shared across the two adjacent faces).
                point_at.entry(key).or_insert_with(|| {
                    let p_i = mesh.vertices[i as usize].coords;
                    let p_j = mesh.vertices[j as usize].coords;
                    let d_i = dists[i as usize];
                    let d_j = dists[j as usize];
                    // Linear-interpolation parameter where the
                    // signed distance crosses zero. With strict
                    // sign-flip guaranteed by SOS, `d_i - d_j` is
                    // bounded away from zero (same sign as d_i).
                    let t = d_i / (d_i - d_j);
                    Point3::from(p_i + t * (p_j - p_i))
                });
            }
        }
        if let &[a, b] = crossing.as_slice() {
            adjacency.entry(a).or_default().push(b);
            adjacency.entry(b).or_default().push(a);
        }
    }

    // Step 4: walk segment graph to extract closed loops.
    // Collect keys into a sorted Vec for deterministic loop-output
    // order across runs (HashMap key iteration is nondeterministic).
    let mut keys: Vec<MeshEdgeKey> = adjacency.keys().copied().collect();
    keys.sort_by_key(|k| (k.lo, k.hi));

    let mut visited: HashSet<MeshEdgeKey> = HashSet::new();
    let mut loops: Vec<Vec<Point3<f64>>> = Vec::new();

    for start in keys {
        if visited.contains(&start) {
            continue;
        }
        let mut loop_pts: Vec<Point3<f64>> = Vec::new();
        let mut current = start;
        let mut prev: Option<MeshEdgeKey> = None;
        loop {
            if visited.contains(&current) {
                // Either closed the loop (current == start, second
                // visit) or hit an already-traversed area. Either
                // way the walk ends.
                break;
            }
            visited.insert(current);
            loop_pts.push(point_at[&current]);
            // Pick the next neighbor that isn't the previous step.
            // In a watertight mesh each crossing edge has exactly 2
            // neighbors; on a clean loop the non-prev choice is
            // unique.
            let neighbors = &adjacency[&current];
            let next = neighbors.iter().find(|&&n| Some(n) != prev).copied();
            match next {
                Some(n) => {
                    prev = Some(current);
                    current = n;
                }
                None => break, // dead end (degenerate boundary)
            }
        }
        if loop_pts.len() >= 3 {
            loops.push(loop_pts);
        }
    }

    loops
}

/// Compute the area-weighted centroid of a closed 3D polygon
/// lying in a plane.
///
/// Uses fan triangulation from `polygon[0]` and the signed-area
/// projected onto `plane_normal`: each sub-triangle contributes
/// its centroid weighted by its signed area, divided by the total
/// signed area. The signed-area sum cancels out spurious
/// contributions from non-convex regions, so the result is correct
/// for any simple polygon (convex or non-convex) and is invariant
/// to winding direction (CW vs CCW with respect to `plane_normal`).
///
/// **Density-independence**: the formula depends only on the
/// polygon's BOUNDARY GEOMETRY — adding redundant collinear
/// vertices between existing ones (subdividing edges) does not
/// change the centroid. This is the load-bearing property that
/// makes the centerline algorithm density-independent.
///
/// **Caller's contract**: `plane_normal` MUST be unit-magnitude.
/// Polygon vertices MUST be approximately coplanar with that
/// normal; otherwise the signed-area projection under-counts
/// contributions tilted away from the plane.
///
/// **Returns** `None` when the polygon has fewer than 3 vertices
/// OR its total area projects to ≈ 0 (degenerate / collinear).
pub fn polygon_centroid_3d(
    polygon: &[Point3<f64>],
    plane_normal: &Vector3<f64>,
) -> Option<Point3<f64>> {
    if polygon.len() < 3 {
        return None;
    }
    let v0 = polygon[0].coords;
    let mut total_signed_area: f64 = 0.0;
    let mut centroid_accum: Vector3<f64> = Vector3::zeros();
    for i in 1..(polygon.len() - 1) {
        let v1 = polygon[i].coords;
        let v2 = polygon[i + 1].coords;
        let cross = (v1 - v0).cross(&(v2 - v0));
        let signed_area = 0.5 * cross.dot(plane_normal);
        let tri_centroid = (v0 + v1 + v2) / 3.0;
        centroid_accum += signed_area * tri_centroid;
        total_signed_area += signed_area;
    }
    if total_signed_area.abs() < f64::EPSILON {
        return None;
    }
    Some(Point3::from(centroid_accum / total_signed_area))
}

/// Unsigned area of a closed 3D polygon projected onto
/// `plane_normal`. Same fan-triangulation + signed-area summation
/// as [`polygon_centroid_3d`]; absolute value at the end so the
/// result is winding-invariant. Used by the centerline algorithm
/// to (a) gate degenerate slabs below `MIN_SLAB_AREA_M2` and (b)
/// pick the largest loop when a slab intersection produces
/// multiple loops (non-convex body / multi-component slice).
pub fn polygon_area_3d(polygon: &[Point3<f64>], plane_normal: &Vector3<f64>) -> f64 {
    if polygon.len() < 3 {
        return 0.0;
    }
    let v0 = polygon[0].coords;
    let mut total_signed_area: f64 = 0.0;
    for i in 1..(polygon.len() - 1) {
        let v1 = polygon[i].coords;
        let v2 = polygon[i + 1].coords;
        let cross = (v1 - v0).cross(&(v2 - v0));
        total_signed_area += 0.5 * cross.dot(plane_normal);
    }
    total_signed_area.abs()
}

/// Minimum projected slab area (m²) below which a slab's
/// intersection polygon is treated as degenerate — its centroid is
/// discarded and the polyline point at that slab is filled in by
/// interpolation from neighboring non-degenerate slabs. 1e-8 m² =
/// 0.01 mm² — well below any real cross-section on body-part scans
/// (smallest expected: dome-tip slabs of a few mm² = 1e-6 m²) and
/// well above the polygon-area numerical noise floor for f64
/// coordinates in meters.
pub const MIN_SLAB_AREA_M2: f64 = 1e-8;

/// Compute the scan's centerline as **N evenly-spaced points
/// along the body's true geometric axis**, with each point
/// derived from the area-weighted centroid of the per-slab
/// cross-section polygon.
///
/// Used by cf-cast's curve-following multi-piece mold generator AND
/// by cf-device-design's per-vertex radial direction computation.
/// Both consumers REQUIRE that the centerline track the body's
/// true visual center — an off-center centerline produces lopsided
/// layer dome surfaces (the iter-1 failure mode documented in
/// `docs/CENTERLINE_RECON_BOOKMARK.md`).
///
/// **Algorithm** (`docs/CENTERLINE_SPEC.md` §2.4, sixth iteration —
/// the one that escapes the density / extreme-point biases that
/// sank the prior five):
///
/// 1. Normalize `spine_hint` to a unit axis. This is the user's
///    chosen body direction — typically the cap loop's outward
///    normal, which after cf-scan-prep's auto-PCA-at-load is
///    aligned with the body's principal axis (`-Z` for floor caps).
/// 2. Project all vertices onto the axis to find the body's depth
///    range `[min_d, max_d]`.
/// 3. For each of `n_slices` evenly-spaced depths `d_i`, intersect
///    the slab plane at depth `d_i` (perpendicular to the axis)
///    with the mesh via [`intersect_plane_with_mesh`]. Pick the
///    largest-area loop (handles non-convex bodies / multi-
///    component slices) and compute its area-weighted polygon
///    centroid via [`polygon_centroid_3d`]. Slabs with area below
///    [`MIN_SLAB_AREA_M2`] are marked degenerate and filled in by
///    linear interpolation between non-degenerate neighbors (or
///    extrapolation along the axis at the extremes).
///
/// **Why per-slab area-weighted polygon centroid, not the prior
/// algorithms** (full recon at `docs/CENTERLINE_RECON_BOOKMARK.md`):
///
/// - All 5 prior iterations (per-slab Kasa, Kasa+centroid-prior,
///   PCA, vertex-centroid+spine, AABB+spine) were SAMPLE-biased:
///   their statistics depend on how vertices are distributed
///   around the surface, OR on the body's extreme points. Real
///   scans have non-uniform vertex density (denser on
///   scanner-facing side) and noisy extreme points (scanner spikes,
///   reconstruct artifacts), so all five drifted off-axis.
/// - The polygon centroid is **DENSITY-INDEPENDENT BY CONSTRUCTION**:
///   it depends only on the slab-mesh intersection polygon's
///   BOUNDARY GEOMETRY, not on the vertex density of the underlying
///   triangulation. Two scans of the same body with different
///   sampling produce the SAME polygon centroid (modulo
///   discretization error from boundary-edge subdivision, which is
///   sub-pixel for typical mesh resolutions). The geometric
///   property is verified by `polygon_centroid_3d_density_independent`.
///
/// **Performance**: O(n_slices × n_faces) per call ≈ 5M triangle-
/// plane intersections for `n_slices=30` and a 169k-face cleaned
/// scan. ~50ms single-threaded; fine for both Cap-step and
/// per-frame re-evaluation budgets.
///
/// **Trade-off**: the algorithm produces a STRAIGHT centerline (no
/// curvature support). For genuinely curved bodies (bent finger,
/// flexed arm), an outer iteration loop re-orienting slabs
/// perpendicular to the local polyline tangent would be needed —
/// banked as spec §2.7 stretch goal G.s2 until a curved-body
/// fixture surfaces.
///
/// **Empty / degenerate input**: returns `Vec::new()` if the mesh
/// has no vertices OR no faces (the algorithm needs faces to
/// intersect, not just vertices), the spine hint is zero-magnitude,
/// `n_slices == 0`, or all slabs end up degenerate (e.g., a
/// non-watertight mesh that no slab plane intersects).
/// Number of iterative re-orientation passes after the initial
/// spine_hint-perpendicular pass. Each pass re-runs the slab
/// sampling with slab planes perpendicular to the local polyline
/// tangent (instead of the global spine_hint axis), allowing the
/// centerline to track curved bodies (banana, bent limb) where a
/// single global axis can't perpendicularly cut all parts of the
/// body simultaneously. The polyline is `smooth_polyline`-damped
/// between passes so per-slab polygon-centroid noise doesn't
/// amplify into divergent tangent tilt (saw 5× drift amplification
/// without this damping on the noisy tapered-cone fixture).
///
/// `1` is the empirical sweet spot: a single re-orientation
/// pass takes spine_hint-aligned slabs → local-tangent-aligned
/// slabs (handles ≤ ~15° curvature well, approximate up to 30°),
/// and re-runs the sampling once at the better slab orientation.
/// Bumping to 2 measurably improves 30°+ curvature handling but
/// re-introduces noise amplification on small-body fixtures (the
/// tapered-cone test fails by ~2×). When a real 30°+ curved
/// body-part fixture surfaces, revisit — likely the right fix is
/// adaptive iteration count gated on detected curvature, not a
/// blanket bump.
pub const CENTERLINE_REORIENT_PASSES: usize = 1;

/// Area threshold (as a fraction of the per-call max polygon area)
/// above which a slab is classified as **main-body** vs **end-region**
/// for the iterative re-orientation tangent correction.
///
/// **Why this matters** (the sphere-cut bias, discovered on the iter-1
/// fixture 2026-05-16): for a body shaped like a hemispherical dome
/// capping a cylinder, oblique slab cuts (slab normal tilted from
/// the body axis) of the CYLINDER portion produce ELLIPSES whose
/// centers lie on the body axis (algorithm works correctly there).
/// Oblique slab cuts of the HEMISPHERE produce CIRCLES whose centers
/// trace a line **parallel to the slab normal** (not the body axis)
/// — this is a pure geometry property of plane-sphere intersection
/// independent of mesh tessellation. The result is a phantom curve
/// in the dome-region polyline that the local-tangent iteration can't
/// fix (the biased tangent equals the slab normal, so re-sampling
/// with that tangent doesn't change anything).
///
/// **Fix**: classify each pass-0 slab as main-body iff it's
/// single-loop AND its area is at least this fraction of the per-call
/// max. For end-region slabs (dome / floor extremes / small
/// fragmented), override the local tangent with the tangent of the
/// nearest main-body slab — this forces dome-region slabs in the
/// re-orientation pass to be perpendicular to the CYLINDER axis,
/// not the spine_hint-aligned biased dome direction. Centroids of
/// the re-sampled dome slabs then sit on the body axis line
/// extending through the cylinder.
///
/// 0.95 catches only the truly cylindrical slabs (where area is near
/// maximum) — strict enough to exclude transitioning dome slabs whose
/// tangents are still partially biased. Tested fixtures (uniform
/// cylinder, tapered cone, offset cylinder, density-biased, rotated)
/// all have either uniform area (all slabs main-body) or a clear
/// max-area cluster, so this threshold doesn't regress them.
pub const CENTERLINE_MAIN_BODY_AREA_FRACTION: f64 = 0.95;

/// One per-slab sample produced by intersecting a slab plane with
/// the mesh and computing its area-weighted polygon centroid.
/// Plus diagnostic metrics used by [`build_polyline_with_boundary_trim`]
/// to classify quality.
#[derive(Clone, Copy, Debug)]
pub struct SlabSample {
    centroid: Point3<f64>,
    area: f64,
    n_loops: usize,
}

/// Sample one slab: intersect the plane with the mesh, pick the
/// largest-area loop, return its centroid + diagnostic metrics.
/// `None` if no valid intersection (no loops, or the largest
/// loop's area is below `MIN_SLAB_AREA_M2` numerical floor, or
/// the polygon centroid is degenerate).
pub fn compute_slab_sample(
    mesh: &IndexedMesh,
    plane_pt: &Point3<f64>,
    plane_n: &Vector3<f64>,
) -> Option<SlabSample> {
    let loops = intersect_plane_with_mesh(plane_pt, plane_n, mesh);
    let n_loops = loops.len();
    let best = loops.into_iter().max_by(|a, b| {
        polygon_area_3d(a, plane_n)
            .partial_cmp(&polygon_area_3d(b, plane_n))
            .unwrap_or(std::cmp::Ordering::Equal)
    })?;
    let area = polygon_area_3d(&best, plane_n);
    if area < MIN_SLAB_AREA_M2 {
        return None;
    }
    let centroid = polygon_centroid_3d(&best, plane_n)?;
    Some(SlabSample {
        centroid,
        area,
        n_loops,
    })
}

/// Compute per-vertex tangent direction along a polyline.
/// Interior: central difference between neighbors. Endpoints:
/// forward / backward difference. Each tangent is unit-normalized;
/// zero-length segments produce a zero tangent (defensive — the
/// caller's slab plane normal would then be invalid, which the
/// downstream intersection routine rejects).
pub fn local_polyline_tangents(polyline: &[Point3<f64>]) -> Vec<Vector3<f64>> {
    let n = polyline.len();
    if n < 2 {
        return vec![Vector3::zeros(); n];
    }
    let mut tangents = Vec::with_capacity(n);
    for i in 0..n {
        let raw = if i == 0 {
            polyline[1].coords - polyline[0].coords
        } else if i == n - 1 {
            polyline[n - 1].coords - polyline[n - 2].coords
        } else {
            polyline[i + 1].coords - polyline[i - 1].coords
        };
        let unit = if raw.norm_squared() > f64::EPSILON {
            raw.normalize()
        } else {
            Vector3::zeros()
        };
        tangents.push(unit);
    }
    tangents
}

/// Replace end-region slabs' tangents with the tangent of the
/// nearest main-body slab. Fixes the dome / sphere-cut bias
/// described at [`CENTERLINE_MAIN_BODY_AREA_FRACTION`]: end-region
/// slabs' raw local tangents are biased along the slab normal
/// (sphere-cap geometry), so iterating with those tangents
/// re-creates the same biased orientation. Substituting the
/// nearest cylindrical-region tangent re-orients dome slabs to
/// be perpendicular to the body axis, putting their re-sampled
/// centroids on the body axis line.
///
/// If no slab is main-body (pathological input: every slab is
/// multi-loop or below the area threshold), returns the raw
/// tangents unchanged — the algorithm degrades to local-tangent
/// iteration rather than failing.
pub fn correct_tangents_for_end_regions(
    raw_tangents: &[Vector3<f64>],
    is_main_body: &[bool],
) -> Vec<Vector3<f64>> {
    let n = raw_tangents.len().min(is_main_body.len());
    if n == 0 {
        return Vec::new();
    }
    let any_main_body = is_main_body.iter().take(n).any(|&b| b);
    if !any_main_body {
        return raw_tangents[..n].to_vec();
    }
    let mut corrected = raw_tangents[..n].to_vec();
    for i in 0..n {
        if is_main_body[i] {
            continue;
        }
        // Linear scan for nearest main-body index — n ≤ 30 in
        // practice; a fancier data structure would be overkill.
        let nearest = is_main_body
            .iter()
            .take(n)
            .enumerate()
            .filter(|(_, b)| **b)
            .min_by_key(|(j, _)| j.abs_diff(i))
            .map(|(j, _)| j);
        if let Some(j) = nearest {
            corrected[i] = raw_tangents[j];
        }
    }
    corrected
}

/// Build the centerline polyline from per-slab samples with
/// **boundary-trim + local-tangent extrapolation**.
///
/// Algorithm:
///
/// 1. **Classify each slab as high-quality** iff the sample is
///    `Some` AND `n_loops == 1`. Single-loop slabs have unambiguous
///    polygon centroids; multi-loop slabs (typical at fragmented
///    dome-tip cross-sections) are unreliable because the
///    "largest loop" pick can be any of several similar-area
///    fragments. The single-loop criterion catches the iter-1
///    failure mode (dome-tip slabs at 10–22 loops; floor-extreme
///    slab at 2 loops) without rejecting clean tapered interiors
///    (where every slab is one loop regardless of area).
/// 2. **High-quality slabs**: use the sample's centroid directly.
/// 3. **Leading boundary low-quality slabs** (before the first
///    high-quality index): extrapolate from the local tangent
///    between the first two high-quality samples, stepping
///    backward one slab-index at a time. Keeps the boundary
///    polyline on the body's local axis line established by the
///    high-quality interior.
/// 4. **Trailing boundary low-quality slabs** (after the last
///    high-quality index): symmetric, forward extrapolation from
///    the local tangent between the last two high-quality samples.
/// 5. **Interior low-quality slabs** (between two high-quality
///    neighbors): linear interpolation by slab-index fraction.
/// 6. **All-degenerate fallback** (no high-quality slab anywhere;
///    pathological input): straight line along `fallback_axis`
///    through origin, evenly spaced over `[min_d, max_d]`.
pub fn build_polyline_with_boundary_trim(
    samples: &[Option<SlabSample>],
    fallback_axis: Vector3<f64>,
    min_d: f64,
    max_d: f64,
) -> Vec<Point3<f64>> {
    let n = samples.len();
    if n == 0 {
        return Vec::new();
    }

    // Collect (slab_index, centroid) pairs for high-quality slabs.
    // Quality = single-loop polygon (multi-loop slabs are fragmented
    // and the "largest loop" pick is unreliable; the iter-1 dome-tip
    // at 10-22 loops + the floor extreme at 2 loops both fail this
    // criterion while clean tapered interiors with 1 loop always pass
    // regardless of area).
    let hq: Vec<(usize, Point3<f64>)> = samples
        .iter()
        .enumerate()
        .filter_map(|(i, s)| {
            s.as_ref()
                .filter(|sample| sample.n_loops == 1)
                .map(|sample| (i, sample.centroid))
        })
        .collect();

    let Some(&(hq_first_idx, hq_first_centroid)) = hq.first() else {
        let mut polyline = Vec::with_capacity(n);
        for i in 0..n {
            #[allow(clippy::cast_precision_loss)]
            let t = (i as f64 + 0.5) / (n as f64);
            let depth = min_d + t * (max_d - min_d);
            polyline.push(Point3::from(fallback_axis * depth));
        }
        return polyline;
    };
    // last() is guaranteed Some since first() was Some.
    let Some(&(hq_last_idx, hq_last_centroid)) = hq.last() else {
        unreachable!()
    };

    let mut polyline = Vec::with_capacity(n);
    for i in 0..n {
        // Direct match against the hq list (small N — linear scan
        // is cheaper than a HashSet for n ≤ 30).
        if let Some(&(_, centroid)) = hq.iter().find(|(j, _)| *j == i) {
            polyline.push(centroid);
            continue;
        }
        let pt = if i < hq_first_idx {
            // Leading boundary: extrapolate backward from
            // hq_first_centroid along the local tangent to the
            // next high-quality slab.
            if let Some(&(j_next, p_next)) = hq.get(1) {
                #[allow(clippy::cast_precision_loss)]
                let step =
                    (p_next.coords - hq_first_centroid.coords) / (j_next - hq_first_idx) as f64;
                #[allow(clippy::cast_precision_loss)]
                let count = (hq_first_idx - i) as f64;
                Point3::from(hq_first_centroid.coords - step * count)
            } else {
                hq_first_centroid
            }
        } else if i > hq_last_idx {
            // Trailing boundary: extrapolate forward from
            // hq_last_centroid along the local tangent to the
            // previous high-quality slab.
            if hq.len() >= 2 {
                if let Some(&(j_prev, p_prev)) = hq.get(hq.len() - 2) {
                    #[allow(clippy::cast_precision_loss)]
                    let step =
                        (hq_last_centroid.coords - p_prev.coords) / (hq_last_idx - j_prev) as f64;
                    #[allow(clippy::cast_precision_loss)]
                    let count = (i - hq_last_idx) as f64;
                    Point3::from(hq_last_centroid.coords + step * count)
                } else {
                    hq_last_centroid
                }
            } else {
                hq_last_centroid
            }
        } else {
            // Interior gap: linear interpolate between nearest
            // high-quality neighbors on each side. Both Some since
            // hq_first_idx < i < hq_last_idx and i is not itself
            // in `hq` (the early `find` above handled that case).
            let left = hq.iter().rev().find(|(j, _)| *j < i);
            let right = hq.iter().find(|(j, _)| *j > i);
            if let (Some(&(j_l, p_l)), Some(&(j_r, p_r))) = (left, right) {
                #[allow(clippy::cast_precision_loss)]
                let frac = (i - j_l) as f64 / (j_r - j_l) as f64;
                Point3::from(p_l.coords + frac * (p_r.coords - p_l.coords))
            } else {
                // Defensive: shouldn't reach this branch given the
                // index range; fall back to hq_first.
                hq_first_centroid
            }
        };
        polyline.push(pt);
    }
    polyline
}

pub fn compute_centerline_polyline(
    mesh: &IndexedMesh,
    spine_hint: Vector3<f64>,
    n_slices: usize,
) -> Vec<Point3<f64>> {
    if mesh.vertices.is_empty()
        || mesh.faces.is_empty()
        || spine_hint.norm_squared() < f64::EPSILON
        || n_slices == 0
    {
        return Vec::new();
    }
    let axis = spine_hint.normalize();

    // Depth range of the body along the chosen axis.
    let depths_proj: Vec<f64> = mesh.vertices.iter().map(|p| axis.dot(&p.coords)).collect();
    let min_d = depths_proj.iter().copied().fold(f64::INFINITY, f64::min);
    let max_d = depths_proj
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);
    let range = max_d - min_d;
    if range < f64::EPSILON {
        return Vec::new();
    }

    // Pass 0: slabs perpendicular to spine_hint, evenly spaced
    // along the body's depth range.
    let mut plane_pts: Vec<Point3<f64>> = (0..n_slices)
        .map(|i| {
            #[allow(clippy::cast_precision_loss)]
            let t = (i as f64 + 0.5) / (n_slices as f64);
            Point3::from(axis * (min_d + t * range))
        })
        .collect();
    let mut plane_normals: Vec<Vector3<f64>> = vec![axis; n_slices];

    let mut polyline: Vec<Point3<f64>> = Vec::new();
    // Captured from pass 0 samples and reused for every subsequent
    // re-orientation pass's tangent correction (the cylinder vs.
    // dome classification is a property of the BODY, not of the
    // polyline orientation in any particular pass).
    let mut is_main_body: Vec<bool> = Vec::new();
    for pass_idx in 0..=CENTERLINE_REORIENT_PASSES {
        let samples: Vec<Option<SlabSample>> = plane_pts
            .iter()
            .zip(plane_normals.iter())
            .map(|(pt, n)| compute_slab_sample(mesh, pt, n))
            .collect();
        if pass_idx == 0 {
            // Main-body classification (pass 0 only): cylinder
            // region of the body, where polygon-centroid is
            // unbiased. Used to correct end-region tangents in the
            // re-orientation pass — see
            // [`CENTERLINE_MAIN_BODY_AREA_FRACTION`] and
            // [`correct_tangents_for_end_regions`].
            let max_area = samples
                .iter()
                .filter_map(|s| s.as_ref().map(|s| s.area))
                .fold(0.0_f64, f64::max);
            let threshold = CENTERLINE_MAIN_BODY_AREA_FRACTION * max_area;
            is_main_body = samples
                .iter()
                .map(|s| {
                    s.as_ref()
                        .is_some_and(|sample| sample.n_loops == 1 && sample.area >= threshold)
                })
                .collect();
        }
        polyline = build_polyline_with_boundary_trim(&samples, axis, min_d, max_d);

        if pass_idx < CENTERLINE_REORIENT_PASSES {
            // Re-orient for next pass: each slab plane passes
            // through the current polyline point with normal equal
            // to the corrected local polyline tangent (end-region
            // slabs' tangents are overridden with the tangent of
            // the nearest main-body slab — see
            // [`correct_tangents_for_end_regions`] for the sphere-
            // cut bias this fixes). Curved bodies still get
            // body-axis-aligned cuts at the bend via the main-body
            // tangent propagation.
            //
            // SMOOTH BEFORE TANGENT EXTRACTION — slab-to-slab
            // polygon-centroid noise on the order of the per-vertex
            // mesh noise translates directly into tangent tilt; over
            // multiple iterations, tilt amplifies into a divergent
            // off-axis bias (saw 5× drift amplification on the
            // tapered-cone test before this smoothing pass was
            // added). 5 iterations of 3-tap moving average reduces
            // tangent noise by ~ sqrt(5)× without flattening the
            // body's real curvature (kernel width ~7 samples on
            // a 30-slab polyline).
            let smoothed = smooth_polyline(&polyline, 5);
            let raw_tangents = local_polyline_tangents(&smoothed);
            plane_normals = correct_tangents_for_end_regions(&raw_tangents, &is_main_body);
            plane_pts = smoothed;
        }
    }

    polyline
}

/// Smooth a polyline by iterated 3-tap moving average over interior
/// points, with endpoint pinning. User-driven 2026-05-15: "the
/// actual line inside needs to be smooth before we trim."
///
/// `compute_centerline_polyline` produces cross-section centroids
/// from raw scan vertex bins. On a noisy surface scan the centroids
/// wobble several mm per slab — that wobble propagates downstream:
/// trim cut planes pivot off the wobbly local tangent;
/// `apply_constant_reconstruction` builds its sampling frame from
/// a wobbly tangent. Smoothing the polyline before any downstream
/// consumer sees it kills the noise.
///
/// Algorithm — `iterations` passes of the 3-tap update
/// `next_i = (curr_im1 + curr_i + curr_ip1) / 3` for interior
/// points; the first and last polyline points are PINNED (so
/// the trim distance semantics — "trim from tip = forward from
/// the first polyline point; trim from floor = backward from the
/// last polyline point" — stay anchored). For `iterations=3` the
/// effective filter footprint is ~5 samples wide; visibly
/// smooths without flattening overall curvature.
///
/// No-op for polylines with < 3 points (no interior to smooth).
pub fn smooth_polyline(polyline: &[Point3<f64>], iterations: usize) -> Vec<Point3<f64>> {
    if polyline.len() < 3 || iterations == 0 {
        return polyline.to_vec();
    }
    let mut current: Vec<Point3<f64>> = polyline.to_vec();
    let n = current.len();
    for _ in 0..iterations {
        let mut next = current.clone();
        for i in 1..(n - 1) {
            let avg = (current[i - 1].coords + current[i].coords + current[i + 1].coords) / 3.0;
            next[i] = Point3::from(avg);
        }
        current = next;
    }
    current
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
///
/// **No longer used in production** as of S1.1 2026-05-26 — both cap-
/// fan call sites (`auto_cap_open_boundaries` + `build_cleaned_mesh`)
/// switched to [`emit_centroid_fan_cap`] after the fan-fallback path
/// here was identified as the source of cleaned.stl's cap-region
/// duplicate-face + non-manifold-edge artifacts. Retained as a
/// generic 2D-polygon ear-clip with its regression tests so the
/// algorithm + its convention (CCW input → CCW output, fan-fallback
/// for non-simple polys) stay documented in-tree.
#[allow(dead_code)]
pub fn triangulate_polygon_2d_earclip(verts_2d: &[(f64, f64)]) -> Vec<[u32; 3]> {
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
///
/// Only used by [`triangulate_polygon_2d_earclip`] which itself is no
/// longer used in production (see its docstring for rationale).
#[allow(dead_code)]
pub fn point_in_triangle_2d(p: (f64, f64), a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> bool {
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
pub fn project_loop_to_plane_2d(
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
///
/// `smoothing_iterations` controls a final Taubin-smoothing pass
/// (added 2026-05-16, user-driven) that suppresses sub-mm scanner
/// noise so the cleaned STL represents the silicone cast's actual
/// outcome (surface tension during cure smooths sub-mm features
/// physically), not the noisy scan capture.
/// `0` = skip smoothing entirely (back-compat with pre-fix behavior).
/// Default per the Save panel slider is `SMOOTHING_DEFAULT_ITERATIONS`.
pub fn cleanup_cleaned_mesh_for_disk(
    mesh: &mut IndexedMesh,
    smoothing_iterations: usize,
) -> CleanupReport {
    let welded = weld_vertices(mesh, SIMPLIFY_WELD_EPSILON_M);
    let degenerate = remove_degenerate_triangles(mesh, CLEANUP_DEGENERATE_AREA_M2);
    let small_components = remove_small_components(mesh, CLEANUP_MIN_COMPONENT_FACES);
    let unreferenced = remove_unreferenced_vertices(mesh);
    let smoothing = taubin_smooth_vertices(
        mesh,
        smoothing_iterations,
        TAUBIN_DEFAULT_LAMBDA,
        TAUBIN_DEFAULT_MU,
    );
    CleanupReport {
        welded,
        degenerate,
        small_components,
        unreferenced,
        smoothing,
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
pub struct CleanupReport {
    pub welded: usize,
    pub degenerate: usize,
    pub small_components: usize,
    pub unreferenced: usize,
    /// Number of Taubin-smoothing iterations actually applied
    /// (returned by [`taubin_smooth_vertices`]; matches the
    /// `smoothing_iterations` parameter unless the mesh was
    /// degenerate). Surfaced in the Save status so the user
    /// can see how much smoothing landed in the file.
    pub smoothing: usize,
}

impl CleanupReport {
    /// Total operations across all five passes. Zero when the input
    /// was already disk-ready (no cleanup needed) AND the smoothing
    /// slider was at 0.
    pub fn total(self) -> usize {
        self.welded + self.degenerate + self.small_components + self.unreferenced + self.smoothing
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
pub struct PrepToml {
    pub scan_prep: PrepScanPrepBlock,
    pub simplify: PrepSimplifyBlock,
    pub smoothing: PrepSmoothingBlock,
    pub transform: PrepTransformBlock,
    pub caps: PrepCapsBlock,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub centerline: Option<PrepCenterlineBlock>,
    pub centerline_trim: PrepCenterlineTrimBlock,
    pub output: PrepOutputBlock,
}

#[derive(Serialize)]
pub struct PrepScanPrepBlock {
    pub source_stl: String,
    pub tool_version: &'static str,
    pub generated_at: String,
    pub stl_units_at_load: &'static str,
    /// Physics-frame translation auto-applied at load (CSP.3.5) so
    /// the scan's AABB centroid lands at origin. Provenance only —
    /// the cleaned STL is in the auto-centered frame (plus the
    /// user's transforms). To reconstruct the source-frame position
    /// of a cleaned-STL vertex, INVERT the user's Reorient +
    /// Recenter recorded in `[transform]`, then add this offset.
    pub auto_center_offset_m: [f64; 3],
    /// Auto-PCA rotation applied at load (CSP.4a) — quaternion in
    /// `(w, x, y, z)` order that takes the source's principal axis
    /// to `+Z`. Skipped (serialized as `null`) when PCA was
    /// degenerate (rare). Pairs with `auto_center_offset_m` for
    /// full source-frame reconstruction.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auto_pca_quaternion: Option<[f64; 4]>,
}

/// `[simplify]` provenance — what decimation, if any, was applied at
/// save time. Per spec §Output format. Records both the user-targeted
/// budget (slice 9.8: Simplify panel slider IS the save-time budget)
/// and the actually-achieved face count, plus the originally-loaded
/// count so the reduction ratio is visible.
#[derive(Serialize)]
pub struct PrepSimplifyBlock {
    /// `true` when meshopt was invoked at save time (target < pre-
    /// simplify cleaned-mesh face count). `false` when the cleaned
    /// mesh's face count was already at or below the target (no
    /// decimation; achieved == original).
    pub applied: bool,
    /// Algorithm identifier — pinned literal so downstream audits know
    /// what code path produced this file.
    pub algorithm: &'static str,
    /// Version string for the algorithm dependency. Tracks the
    /// workspace `meshopt` Cargo dep; bump this constant in lockstep
    /// when that dep is updated. Hard-coded literal because the
    /// meshopt-rs crate doesn't expose its version at runtime and a
    /// build-script lift for one provenance line would be overkill.
    pub algorithm_version: &'static str,
    /// Target face count from the Simplify panel slider at save time.
    pub target_face_count: usize,
    /// Actual face count of the cleaned mesh on disk. Equals
    /// `target_face_count` ± boundary-locked vertex topology slack
    /// when `applied`; equals `original_face_count` when `!applied`.
    pub achieved_face_count: usize,
    /// Face count of the as-loaded scan (in `OriginalScanMesh`,
    /// before any decimation or transforms). Distinct from
    /// `achieved_face_count` whenever simplify ran at all.
    pub original_face_count: usize,
    /// Always `true` — cf-scan-prep uses
    /// `meshopt::SimplifyOptions::LockBorder` (NOT
    /// `simplify_sloppy`) per spec §Architectural decisions §Simplify
    /// algorithm. Surfaced in the TOML for audit purposes.
    pub boundary_preserved: bool,
}

/// `[smoothing]` provenance — what surface smoothing was
/// applied at save time. Added 2026-05-16 with the Taubin
/// smoothing pass in [`cleanup_cleaned_mesh_for_disk`].
///
/// Downstream consumers can use this to know how aggressively
/// the cleaned mesh has been smoothed — e.g., cf-cast's SDF
/// sampling math is unaffected (it just samples whatever
/// surface is on disk), but a future re-mesh / re-process
/// pipeline can decide whether to apply additional smoothing
/// based on what's already there.
#[derive(Serialize)]
pub struct PrepSmoothingBlock {
    /// Algorithm identifier — pinned literal so downstream
    /// audits know what produced the smoothed vertices.
    pub algorithm: &'static str,
    /// Number of Taubin (shrink + expand) pass pairs applied.
    /// `0` = smoothing disabled at save time.
    pub iterations: usize,
    /// Shrink-pass weight `λ` (positive Laplacian step).
    pub lambda: f64,
    /// Expand-pass weight `μ` (negative Laplacian step).
    pub mu: f64,
}

/// Algorithm identifier pinned literal for
/// [`PrepSmoothingBlock::algorithm`]. Matches the function
/// name in `mesh-repair`'s public API so downstream audit
/// tooling can trace back to the implementation.
pub const SMOOTHING_ALGORITHM_NAME: &str = "taubin_smooth_vertices";

/// `[transform]` umbrella — `rotation` + `translation` sub-tables
/// match spec §Output format. Each renders as `[transform.rotation]`
/// + `[transform.translation]` in TOML.
#[derive(Serialize)]
pub struct PrepTransformBlock {
    pub rotation: PrepRotationBlock,
    pub translation: PrepTranslationBlock,
}

#[derive(Serialize)]
pub struct PrepRotationBlock {
    /// Physics-frame unit quaternion `(w, x, y, z)`. Source of truth
    /// for downstream reconstruction; the Euler angles below mirror
    /// the cf-scan-prep slider source-of-truth for human readability.
    pub quaternion: [f64; 4],
    pub roll_deg: f64,
    pub pitch_deg: f64,
    pub yaw_deg: f64,
}

#[derive(Serialize)]
pub struct PrepTranslationBlock {
    /// Physics-frame translation in meters. Spec §Output format
    /// names this `m`; the panel state stores mm but the on-disk
    /// units convention is meters (matches cf-cast / the rest of
    /// the workspace).
    pub m: [f64; 3],
}

// CSP.4c — `PrepClipBlock` removed alongside `ClipState`. The
// `.prep.toml` no longer emits a `[clip]` block. Downstream
// consumers (cf-cast-cli, cf-device-design) read only
// `[centerline]` and tolerate unknown sibling keys, so the
// removal is downstream-safe.

#[derive(Serialize)]
pub struct PrepCapsBlock {
    /// Whether ear-clipping was applied at save time to close the
    /// included loops. Always `true` if any loops were detected
    /// AND included.
    pub applied: bool,
    pub loops: Vec<PrepCapLoop>,
}

#[derive(Serialize)]
pub struct PrepCapLoop {
    pub loop_index: usize,
    pub vertex_count: usize,
    pub plane_fit_r_squared: f64,
    /// Physics-frame outward normal at scan time (pre-transform bake).
    pub plane_normal: [f64; 3],
    /// Physics-frame centroid at scan time.
    pub plane_centroid_m: [f64; 3],
    pub included: bool,
}

#[derive(Serialize)]
pub struct PrepCenterlineBlock {
    /// Polyline in **post-bake, post-trim world-frame meters**
    /// (matches the cleaned STL's coordinate system; v2 cf-cast
    /// consumes directly). CSP.4b — when the user dialed centerline
    /// trim, this is the polyline between the trim cut planes, not
    /// the full pre-trim polyline.
    pub points_m: Vec<[f64; 3]>,
    pub algorithm: &'static str,
}

/// `[centerline_trim]` provenance — what user-driven centerline
/// trim was applied at save time (CSP.4b). Always emitted, even
/// when no trim was requested (the explicit `0.0 / 0.0` record
/// makes "saved without trim" indistinguishable from an audit
/// perspective).
#[derive(Serialize)]
pub struct PrepCenterlineTrimBlock {
    pub trim_tip_mm: f64,
    pub trim_floor_mm: f64,
    /// Number of boundary loops auto-capped after the trim cuts.
    /// For a "trim only the floor end" save on a closed-tip sock,
    /// this is 1; for "trim both ends" it's 2; for "no trim" it's
    /// 0. When floor reconstruction is applied this drops to 0 +
    /// 1 (the tip-end auto-cap survives, the floor end is
    /// extrusion + flat cap instead of an auto-cap).
    pub capped_loops: usize,
    /// CSP.4e.5 — present only when floor reconstruction was
    /// applied at save time. The cleaned STL has the extruded
    /// sidewall + flat cap baked in; this provenance block
    /// records WHAT shape + how much reference zone the user
    /// dialed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reconstruct: Option<PrepReconstructSubBlock>,
}

/// `[centerline_trim.reconstruct]` provenance — emitted only
/// when the user clicked Apply reconstruct before Save. CSP.4e.5.
#[derive(Serialize)]
pub struct PrepReconstructSubBlock {
    /// "constant", "taper", or "extrapolate" — matches the
    /// `ReconstructShape` variant the user selected.
    pub shape: &'static str,
    /// Reference-zone length in mm that drove the radial profile
    /// sampling.
    pub reference_mm: f64,
}

#[derive(Serialize)]
pub struct PrepOutputBlock {
    pub cleaned_stl: String,
    /// AABB of the cleaned mesh on disk (post-transform, post-cap,
    /// post-save-time-simplify), in meters. Spec §Output format
    /// promised this so downstream tooling can sanity-check the
    /// cleaned scan extents without re-loading the STL.
    pub aabb_m: PrepAabbBlock,
}

#[derive(Serialize)]
pub struct PrepAabbBlock {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

/// Algorithm identifier for `[simplify].algorithm`. Pinned literal so
/// downstream audits can distinguish cf-scan-prep's boundary-preserving
/// quadric collapse from other decimation strategies (e.g.,
/// cf-device-design's `simplify_sloppy` proxy).
pub const SIMPLIFY_ALGORITHM_NAME: &str = "meshopt_quadric_edge_collapse";

/// Tracks the workspace `meshopt` Cargo dep. Update in lockstep with
/// `Cargo.toml`'s `meshopt = "X.Y.Z"`.
pub const SIMPLIFY_ALGORITHM_VERSION: &str = "0.6.2";

/// Minimal RFC 3339-ish timestamp without pulling in `chrono`. Uses
/// the OS clock + `std::time::SystemTime` for a UTC-shaped string
/// good enough for provenance. If the clock returns an error
/// (extremely unusual), falls back to a placeholder.
pub fn chrono_like_timestamp() -> String {
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
pub fn iso8601_utc_from_unix_seconds(unix_secs: u64) -> String {
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
pub fn unix_days_to_ymd(z: i64) -> (i64, u32, u32) {
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
pub fn atomic_write_save(
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
pub fn rotated_aabb_around_centroid_physics_mm(
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
pub fn bake_vertex_with_pivot(
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
pub fn compute_pca_orientation(vertices: &[Point3<f64>]) -> Option<UnitQuaternion<f64>> {
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

/// Format an integer count with `k` / `M` suffixes for compact display
/// in the Scan Info panel + Simplify panel + load-time auto-suggest
/// banner. Mirrors the spec's wording (`"18.4k"`, `"3.35M"`).
pub fn human_count(n: usize) -> String {
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
pub fn auto_pca_in_place(mesh: &mut IndexedMesh) -> Option<UnitQuaternion<f64>> {
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
pub fn auto_center_in_place(mesh: &mut IndexedMesh) -> Vector3<f64> {
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
pub fn scale_vertices_in_place(mesh: &mut IndexedMesh, factor: f64) {
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
pub struct SimplifyResult {
    pub mesh: IndexedMesh,
    pub elapsed_secs: f64,
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
pub fn simplify_mesh(original: &IndexedMesh, target_face_count: usize) -> SimplifyResult {
    let start = Instant::now();

    if target_face_count >= original.faces.len() {
        return SimplifyResult {
            mesh: original.clone(),
            elapsed_secs: start.elapsed().as_secs_f64(),
        };
    }

    // Step 1: weld unshared vertices into shared indices, then
    // compact the vertex array. STL load produces 3 unique vertex
    // slots per triangle (vertex soup); the decimator needs shared
    // vertex indices across adjacent triangles to find collapsible
    // edges. `remove_unreferenced_vertices` compacts away the
    // unreferenced entries `weld_vertices` leaves behind, which makes
    // the decimator's vertex-index space dense.
    let mut welded = original.clone();
    weld_vertices(&mut welded, SIMPLIFY_WELD_EPSILON_M);
    remove_unreferenced_vertices(&mut welded);

    // Env-var-gated diagnostics. Set `CF_SCAN_PREP_SIMPLIFY_DIAG=1` to
    // see pre/post stats + timing on stderr.
    let diag = std::env::var("CF_SCAN_PREP_SIMPLIFY_DIAG").is_ok_and(|v| !v.is_empty());
    if diag {
        eprintln!(
            "[simplify_mesh] start: target={} welded_faces={} welded_vertices={}",
            target_face_count,
            welded.faces.len(),
            welded.vertices.len(),
        );
    }

    // Step 2: convert `IndexedMesh` -> `CornerTableF` for baby_shark.
    // CornerTableF stores positions as f32 (matches the prior meshopt
    // path's f32 conversion; sub-1 µm precision at scan mm scale).
    #[allow(clippy::cast_possible_truncation)]
    let positions_f32: Vec<Vector3<f32>> = welded
        .vertices
        .iter()
        .map(|p| Vector3::new(p.x as f32, p.y as f32, p.z as f32))
        .collect();
    let flat_indices: Vec<usize> = welded
        .faces
        .iter()
        .flat_map(|tri| [tri[0] as usize, tri[1] as usize, tri[2] as usize])
        .collect();
    let mut ct_mesh = CornerTableF::from_vertex_and_face_slices(&positions_f32, &flat_indices);

    // Step 3: decimate with baby_shark's incremental QEM edge collapse.
    // `keep_boundary(true)` is the equivalent of meshopt's
    // `SimplifyOptions::LockBorder` — pins open-boundary loop vertices
    // so the downstream Cap panel sees the same loop topology after
    // simplification (the spec's load-bearing boundary-preservation
    // requirement). Unlike meshopt, baby_shark's `EdgeDecimator`
    // refuses any collapse that would create a non-manifold edge —
    // see S1.1 probe-8 (2026-05-26) for the empirical confirmation
    // on the workshop iter-1 scan.
    let decimate_start = Instant::now();
    let mut decimator: EdgeDecimator<f32, AlwaysDecimate> = EdgeDecimator::default()
        .decimation_criteria(AlwaysDecimate)
        .min_faces_count(Some(target_face_count))
        .keep_boundary(true);
    decimator.decimate(&mut ct_mesh);
    let decimate_elapsed = decimate_start.elapsed().as_secs_f64();

    // Step 4: convert CornerTableF back to IndexedMesh. baby_shark's
    // `VertexId` is opaque (an internal handle, possibly with gaps
    // after edge collapse); walk `vertices()` once to assign each VID
    // a dense `[0..n)` index, then walk `faces()` to emit triangles
    // using the dense indices.
    let mut out_mesh = IndexedMesh::new();
    let mut vid_to_dense: std::collections::HashMap<_, u32> = std::collections::HashMap::new();
    for vid in <CornerTableF as TriangleMesh>::vertices(&ct_mesh) {
        let pos = <CornerTableF as TriangleMesh>::position(&ct_mesh, vid);
        let dense_idx = out_mesh.vertices.len() as u32;
        out_mesh.vertices.push(Point3::new(
            f64::from(pos[0]),
            f64::from(pos[1]),
            f64::from(pos[2]),
        ));
        vid_to_dense.insert(vid, dense_idx);
    }
    for face_vids in <CornerTableF as TriangleMesh>::faces(&ct_mesh) {
        // Invariant: each `face_vids[i]` was emitted by `vertices()`
        // above, so the map lookup always succeeds. Use `if let` over
        // `expect` to satisfy the crate's `expect_used = deny` lint
        // without changing behavior — the no-match arm is unreachable
        // under the documented baby_shark TriangleMesh contract.
        if let (Some(&a), Some(&b), Some(&c)) = (
            vid_to_dense.get(&face_vids[0]),
            vid_to_dense.get(&face_vids[1]),
            vid_to_dense.get(&face_vids[2]),
        ) {
            out_mesh.faces.push([a, b, c]);
        }
    }

    if diag {
        eprintln!(
            "[simplify_mesh] decimated: in_faces={} out_faces={} out_verts={} \
             decimate_elapsed={:.3}s",
            welded.faces.len(),
            out_mesh.faces.len(),
            out_mesh.vertices.len(),
            decimate_elapsed,
        );
    }

    if diag {
        eprintln!(
            "[simplify_mesh] end: final_faces={} final_vertices={} elapsed={:.3}s",
            out_mesh.faces.len(),
            out_mesh.vertices.len(),
            start.elapsed().as_secs_f64(),
        );
    }

    SimplifyResult {
        mesh: out_mesh,
        elapsed_secs: start.elapsed().as_secs_f64(),
    }
}

// ===== Move 2: orchestration fns (plain-data) =====

/// Identify the floor loop in [`CapState::loops`] for the
/// reconstruction-plane override: the LARGEST valid loop, matching
/// [`find_floor_loop_index`]'s pick-by-count heuristic (`MIN_RIM_LOOP_VERTS`
/// = 10). The cut rim is overwhelmingly the largest loop on practical
/// scans; small loops are scanner-noise stragglers and should keep
/// their detected planes. Returns `None` when no sufficiently-large
/// loop exists (in which case the override is skipped).
pub fn floor_loop_index(loops: &[DetectedCapLoop]) -> Option<usize> {
    const MIN_RIM_LOOP_VERTS: usize = 10;
    loops
        .iter()
        .enumerate()
        .filter(|(_, cl)| cl.vertex_indices.len() >= MIN_RIM_LOOP_VERTS)
        .max_by_key(|(_, cl)| cl.vertex_indices.len())
        .map(|(i, _)| i)
}

/// Build the cleaned IndexedMesh from the working scan + Reorient +
/// Recenter + included cap loops.
///
/// Pipeline:
/// 1. Clone `scan` into `out`.
/// 2. Bake rotation + translation into vertex positions (in place).
///    After this step `out.vertices` are in world frame.
/// 3. For each included cap loop, ear-clip its 2D projection and
///    append triangles (using existing loop vertex indices — no new
///    vertices added). Cap faces inherit the world-frame coordinates
///    from step 2.
///
/// CSP.4c — Clip-floor step removed (the feature itself retired
/// alongside `ClipState`). Centerline trim handles the workshop
/// "shave the noisy end" use case the clip used to cover.
///
/// **Cap normal orientation**: the spec mandates that the cap's
/// outward normal point away from the mesh interior. We orient the
/// triangulation's vertex winding to match the loop's stored
/// `plane_normal` (which `build_detected_cap_loop` already flipped
/// to face outward).
pub fn build_cleaned_mesh(
    scan: &IndexedMesh,
    rotation: UnitQuaternion<f64>,
    translation_m: Vector3<f64>,
    cap_loops: &[DetectedCapLoop],
) -> IndexedMesh {
    let translation = translation_m;
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
    for cap_loop in cap_loops {
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

        // Centroid-fan replacement for the ear-clip path (S1.1
        // 2026-05-26) — produces a non-overlapping fan from a fresh
        // centroid vertex to each perimeter edge. Manifold by
        // construction for star-shaped projected polygons; eliminates
        // the duplicate-face + non-manifold-edge artifacts the ear-
        // clip's fan-fallback emitted on self-intersecting
        // projections. Same helper as `auto_cap_open_boundaries`.
        emit_centroid_fan_cap(
            &mut out,
            &cap_loop.vertex_indices,
            &loop_points_world,
            &verts_2d,
        );
    }

    out
}

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
pub fn build_prep_toml_string(
    source_stl: &Path,
    stl_units_label: &'static str,
    auto_center_offset_m: Vector3<f64>,
    auto_pca_quat: Option<UnitQuaternion<f64>>,
    rotation_physics: UnitQuaternion<f64>,
    euler_deg: [f64; 3],
    translation_m: Vector3<f64>,
    centerline_polyline: &[Point3<f64>],
    cap_loops: &[DetectedCapLoop],
    applied_tip_mm: f64,
    applied_floor_mm: f64,
    applied_reconstruct: Option<AppliedReconstruct>,
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
    let q = rotation_physics;
    let timestamp = chrono_like_timestamp();

    // Project the centerline polyline into world frame so v2 cf-cast
    // can consume it directly without redoing transform math. Same
    // centroid-pivot transform as `build_cleaned_mesh` uses for mesh
    // vertices. CSP.4b — if the user dialed trim, emit the
    // POST-TRIM polyline so the TOML record matches what's actually
    // in the cleaned STL on disk.
    let baked_polyline: Vec<Point3<f64>> = centerline_polyline
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
    // CSP.4b.6 — use APPLIED trim values (what the displayed +
    // saved mesh was cut with), not the slider values (which may
    // be ahead of the user's last Apply click).
    let trimmed_polyline =
        trim_centerline_polyline(&baked_polyline, applied_tip_mm, applied_floor_mm);
    let centerline_world: Vec<[f64; 3]> =
        trimmed_polyline.iter().map(|p| [p.x, p.y, p.z]).collect();

    let toml_struct = PrepToml {
        scan_prep: PrepScanPrepBlock {
            source_stl: source_stl.display().to_string(),
            tool_version: env!("CARGO_PKG_VERSION"),
            generated_at: timestamp,
            stl_units_at_load: stl_units_label,
            auto_center_offset_m: [
                auto_center_offset_m.x,
                auto_center_offset_m.y,
                auto_center_offset_m.z,
            ],
            auto_pca_quaternion: auto_pca_quat.map(|q| [q.w, q.i, q.j, q.k]),
        },
        simplify: PrepSimplifyBlock {
            // `applied = true` iff the user clicked [Apply Simplify]
            // against the currently-loaded mesh — threaded directly
            // from `SimplifyState::was_applied`. Replaces a prior
            // face-count-inference (`achieved < original`) which
            // produced false positives once save-time simplify was
            // retired (commit `a66a3cda`, 2026-05-15) and the
            // save-time cleanup pass became the only face-dropper
            // for the "user never clicked Apply" case.
            applied: simplify_ran,
            algorithm: SIMPLIFY_ALGORITHM_NAME,
            algorithm_version: SIMPLIFY_ALGORITHM_VERSION,
            target_face_count: simplify_target_face_count,
            achieved_face_count,
            original_face_count,
            boundary_preserved: true,
        },
        smoothing: PrepSmoothingBlock {
            algorithm: SMOOTHING_ALGORITHM_NAME,
            iterations: smoothing_iterations,
            lambda: TAUBIN_DEFAULT_LAMBDA,
            mu: TAUBIN_DEFAULT_MU,
        },
        transform: PrepTransformBlock {
            rotation: PrepRotationBlock {
                quaternion: [q.w, q.i, q.j, q.k],
                roll_deg: euler_deg[0],
                pitch_deg: euler_deg[1],
                yaw_deg: euler_deg[2],
            },
            translation: PrepTranslationBlock {
                m: [translation_m.x, translation_m.y, translation_m.z],
            },
        },
        caps: PrepCapsBlock {
            applied: cap_loops.iter().any(|l| l.include),
            loops: cap_loops
                .iter()
                .enumerate()
                .map(|(i, cl)| {
                    // Reconstruction override: when the user applied
                    // centerline-driven floor reconstruction, the
                    // cleaned mesh's actual floor sits at the
                    // post-reconstruction plane (NOT at this loop's
                    // pre-reconstruction fit plane). Identify the
                    // floor loop with the same pick-by-count
                    // heuristic `find_floor_loop_index` uses (the
                    // largest valid loop is overwhelmingly the cut
                    // rim on practical scans) and override its plane.
                    // Other loops (top-end caps, scanner-noise
                    // stragglers below the size threshold) keep their
                    // detected planes verbatim.
                    let is_floor_loop =
                        reconstructed_floor.is_some() && floor_loop_index(cap_loops) == Some(i);
                    let (centroid, normal, r_squared) = match reconstructed_floor {
                        Some(rf) if is_floor_loop => (rf.centroid_m, rf.normal, 1.0_f64),
                        _ => (cl.plane_centroid, cl.plane_normal, cl.plane_fit_r_squared),
                    };
                    PrepCapLoop {
                        loop_index: i,
                        vertex_count: cl.vertex_indices.len(),
                        plane_fit_r_squared: r_squared,
                        plane_normal: [normal.x, normal.y, normal.z],
                        plane_centroid_m: [centroid.x, centroid.y, centroid.z],
                        included: cl.include,
                    }
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
            // CSP.4b.6 — record APPLIED values (what was actually
            // cut into the saved STL), not slider values (the
            // user's draft preview position).
            trim_tip_mm: applied_tip_mm,
            trim_floor_mm: applied_floor_mm,
            capped_loops: centerline_trim_capped,
            // CSP.4e.5 — reconstruct sub-block. Present only when
            // floor reconstruction was applied (and the floor was
            // chopped — the gate match in `handle_save_action`
            // ensures both).
            reconstruct: applied_reconstruct.and_then(|ar| {
                if applied_floor_mm > 0.0 {
                    Some(PrepReconstructSubBlock {
                        shape: match ar.shape {
                            ReconstructShape::Constant => "constant",
                            ReconstructShape::Taper => "taper",
                            ReconstructShape::Extrapolate => "extrapolate",
                        },
                        reference_mm: ar.reference_mm,
                    })
                } else {
                    None
                }
            }),
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
