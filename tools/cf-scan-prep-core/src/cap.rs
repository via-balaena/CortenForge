//! Boundary-loop detection, plane fitting and cap emission.
//!
//! Moved verbatim from the flat `lib.rs` (pure module split).

use mesh_repair::{MeshAdjacency, holes};
use mesh_types::{IndexedMesh, Point3};
use nalgebra::{Matrix3, Vector3};
use std::collections::{HashMap, HashSet};

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

/// Detect all open boundary loops on `mesh`, fit a plane to each via
/// SVD, orient the normal outward, and append ear-clipped cap faces
/// (with outward-pointing 3D normals) that close every detected loop.
/// CSP.4b — runs after [`trim_mesh_along_centerline`](crate::trim_mesh_along_centerline) so the trim cuts
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

// ===== Move 2: orchestration fns (plain-data) =====

/// Identify the floor loop in the [`DetectedCapLoop`] list for the
/// reconstruction-plane override: the LARGEST valid loop, matching
/// [`find_floor_loop_index`](crate::find_floor_loop_index)'s pick-by-count heuristic (`MIN_RIM_LOOP_VERTS`
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
