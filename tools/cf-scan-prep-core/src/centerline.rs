//! Slab sampling and centerline polyline construction.
//!
//! Moved verbatim from the flat `lib.rs` (pure module split).

use crate::{intersect_plane_with_mesh, polygon_area_3d, polygon_centroid_3d};
use mesh_types::{IndexedMesh, Point3};
use nalgebra::Vector3;

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
