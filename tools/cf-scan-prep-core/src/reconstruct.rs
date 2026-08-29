//! Centerline-driven floor reconstruction (CSP.4e).
//!
//! Moved verbatim from the flat `lib.rs` (pure module split).

use crate::{auto_cap_open_boundaries, detect_boundary_loops, trim_centerline_polyline};
use mesh_repair::holes;
use mesh_types::{IndexedMesh, Point3};
use nalgebra::Vector3;

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
/// to `n`. Matches the heuristic in [`project_loop_to_plane_2d`](crate::project_loop_to_plane_2d)
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
/// [`PrepCapsBlock`](crate::PrepCapsBlock)'s `loops` records (those were detected from the raw scan
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
    /// Mirrors [`orient_cap_normal_outward`](crate::orient_cap_normal_outward)'s convention: points AWAY
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
/// the same frame as [`DetectedCapLoop::plane_centroid`](crate::DetectedCapLoop::plane_centroid) / `plane_normal`
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
