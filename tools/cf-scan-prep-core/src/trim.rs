//! Arc-length trimming of the mesh and its centerline polyline.
//!
//! Moved verbatim from the flat `lib.rs` (pure module split).

use crate::clip_mesh_against_plane;
use mesh_types::{IndexedMesh, Point3};
use nalgebra::Vector3;

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
/// This is the order [`compute_centerline_polyline`](crate::compute_centerline_polyline) returns when
/// driven by the first cap loop's outward normal (which points
/// outward from the rim, toward -Z under PCA convention).
///
/// `trim_tip_mm == 0 && trim_floor_mm == 0` is a fast-path no-op
/// (returns `mesh.clone()`). A degenerate centerline (< 2 points)
/// also returns the input unchanged.
///
/// The trimmed mesh has new open boundaries at each non-zero cut.
/// The caller is responsible for capping those — see
/// [`auto_cap_open_boundaries`](crate::auto_cap_open_boundaries) for the auto-cap step.
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
    if trim_tip_mm > 0.0
        && let Some((plane_point, tangent)) =
            point_along_polyline_at_arc_distance(centerline, trim_tip_mm * 0.001)
    {
        out = clip_mesh_against_plane(&out, plane_point, tangent);
    }

    // Floor-end trim: plane at arc-distance
    // `total_length - trim_floor_mm` from the tip (i.e.,
    // `trim_floor_mm` backward from the floor). Plane normal =
    // -tangent (points toward tip). Kept side = the "tip" side.
    if trim_floor_mm > 0.0 {
        let total_length_m = polyline_arc_length_m(centerline);
        let target_m = total_length_m - trim_floor_mm * 0.001;
        if target_m > 0.0
            && let Some((plane_point, tangent)) =
                point_along_polyline_at_arc_distance(centerline, target_m)
        {
            out = clip_mesh_against_plane(&out, plane_point, -tangent);
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
