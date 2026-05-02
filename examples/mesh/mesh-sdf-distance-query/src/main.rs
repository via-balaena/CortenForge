//! mesh-sdf-distance-query — signed-distance + inside/outside +
//! closest-point + bulk-query coverage of the `mesh-sdf` public surface.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.4. Fixture: unit
//! octahedron — 6 vertices `(±1, 0, 0)`, `(0, ±1, 0)`, `(0, 0, ±1)`
//! and 8 triangles (one per `(sx, sy, sz)` octant). The octahedron is
//! the L1-unit-ball; signed distance is `(|x|+|y|+|z| − 1) / √3` in
//! face-region. The 4 octants where `sx*sy*sz = -1` use parity-flipped
//! winding (R10) so all 8 cross-product normals are `(sx, sy, sz) / √3`.
//!
//! Origin corner-degeneracy (R5 + HE-1): `is_inside((0, 0, 0))`
//! reports false — +X ray hits 4 faces sharing vertex `(1, 0, 0)`,
//! count = 4 → even. Example uses off-axis interior `(0.05, 0.07,
//! 0.11)`. Drift-9 (spec §5.4 line 572): octahedron volume is
//! `(4/3)·r³` (L1-ball), NOT `(8/3)·r³`; continuous fraction inside
//! `[-2, 2]³` is `1/48 ≈ 2.083%`, NOT 4.17%. Discrete 10×10×10 grid
//! (spacing `4/9`, endpoint-inclusive) puts exactly 8 grid points at
//! `(±2/9, ±2/9, ±2/9)` strictly inside.
//!
//! Drift-10 (NEW v0.9 trigger surfaced by the bulk-grid scan): the
//! face-normal-of-closest-face sign test produces 6 vertex-region
//! FALSE-POSITIVES along the bbox boundary, e.g. `(-2, -2/3, 2/3)`
//! where 4 faces tie on closest-vertex `(-1, 0, 0)` and the strict-
//! `<` tie-break picks an outward direction whose dot with `to_point`
//! flips negative — the point reads `signed_distance < 0` despite
//! being geometrically OUTSIDE. So `signed < 0` reports 14 inside
//! while `point_in_mesh` (ray-casting) correctly reports 8. Failure
//! mode is fundamental to the face-normal sign convention and applies
//! to CONVEX geometry at vertex / edge regions (spec R5 framed it as
//! concave-only; corrected). v0.9 fix: pseudo-normal (Bærentzen-
//! Aanæs) or winding-number sign convention; spec §10 item 8.
//! Drift-9 + drift-10 spec edits land inline at this commit (same
//! precedent as drifts 7/8 in §5.2).

// Cross-product unit-normal computation reads as the textbook formula
// `e1.y*e2.z - e1.z*e2.y`; the `mul_add` rewrite obscures intent and
// produces bit-equivalent results on integer vertex coordinates.
// (Same precedent as bbox + cross-section + distance-to-mesh
// verify_*_geometry helpers.)
#![allow(clippy::suboptimal_flops)]
// PLY field-data is single-precision on disk; converting f64 sdf
// values to f32 for `extras["signed_distance"]` is intrinsic to the
// PLY format. Same rationale as ply-with-custom-attributes.
#![allow(clippy::cast_possible_truncation)]

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::{save_ply, save_ply_attributed};
use mesh_sdf::{
    SdfError, SignedDistanceField, closest_point_on_triangle, point_in_mesh,
    point_segment_distance_squared, ray_triangle_intersect, signed_distance, unsigned_distance,
};
use mesh_types::{AttributedMesh, IndexedMesh, Point3, Vector3};

// =============================================================================
// Constants
// =============================================================================

/// FP-exact integer octahedron coords + analytical `1/√3` normals +
/// closed-form SDF + vertex-clamped Euclidean all hit `1e-12`.
const VERTEX_TOL: f64 = 1e-12;
const NORMAL_TOL: f64 = 1e-12;
const DISTANCE_TOL: f64 = 1e-12;

const OCTAHEDRON_VERT_COUNT: usize = 6;
const OCTAHEDRON_FACE_COUNT: usize = 8;
const OCTAHEDRON_RADIUS: f64 = 1.0;

/// Bulk-query grid: 10×10×10 = 1000 points in `[-2, 2]³`
/// endpoint-inclusive (spacing `4/9`).
const GRID_SIZE: usize = 10;
const GRID_TOTAL: usize = GRID_SIZE * GRID_SIZE * GRID_SIZE;
const GRID_MIN: f64 = -2.0;
const GRID_MAX: f64 = 2.0;

/// 8 grid points at `(±2/9, ±2/9, ±2/9)`; next-nearest `|x|+|y|+|z|`
/// is `10/9 > 1`. Ray-casting agrees with analytical L1-ball.
const EXPECTED_INSIDE_COUNT_RAYCAST: usize = 8;

/// 8 true + 6 vertex-region false-positives (drift-10; §10 item 8).
const EXPECTED_INSIDE_COUNT_SIGNED: usize = 14;

// =============================================================================
// Fixture builder
// =============================================================================

/// Build a unit octahedron with vertices at `(±r, 0, 0)`, `(0, ±r, 0)`,
/// `(0, 0, ±r)` and 8 triangle faces (one per `(sx, sy, sz)` octant).
///
/// Vertex layout:
/// - 0: `(+r, 0, 0)` — `+X` apex
/// - 1: `(-r, 0, 0)` — `-X` apex
/// - 2: `(0, +r, 0)` — `+Y` apex
/// - 3: `(0, -r, 0)` — `-Y` apex
/// - 4: `(0, 0, +r)` — `+Z` apex
/// - 5: `(0, 0, -r)` — `-Z` apex
///
/// Face winding produces outward-facing normals on a CCW-from-outside
/// convention. The 4 octants where `sx*sy*sz = +1` use the standard
/// winding `[v_x, v_y, v_z]`; the 4 where `sx*sy*sz = -1` need the
/// flipped winding `[v_x, v_z, v_y]` per spec R10. Cross-product unit
/// normals come out as `(sx, sy, sz) / √3` for all 8 faces.
fn build_octahedron(r: f64) -> IndexedMesh {
    let vertices: Vec<Point3<f64>> = vec![
        Point3::new(r, 0.0, 0.0),  //  0  +X
        Point3::new(-r, 0.0, 0.0), //  1  -X
        Point3::new(0.0, r, 0.0),  //  2  +Y
        Point3::new(0.0, -r, 0.0), //  3  -Y
        Point3::new(0.0, 0.0, r),  //  4  +Z
        Point3::new(0.0, 0.0, -r), //  5  -Z
    ];
    let faces: Vec<[u32; 3]> = vec![
        [0, 2, 4], // f0  +x+y+z  (parity +1, standard)
        [0, 3, 5], // f1  +x-y-z  (parity +1, standard)
        [1, 2, 5], // f2  -x+y-z  (parity +1, standard)
        [1, 3, 4], // f3  -x-y+z  (parity +1, standard)
        [0, 5, 2], // f4  +x+y-z  (parity -1, flipped)
        [0, 4, 3], // f5  +x-y+z  (parity -1, flipped)
        [1, 4, 2], // f6  -x+y+z  (parity -1, flipped)
        [1, 5, 3], // f7  -x-y-z  (parity -1, flipped)
    ];
    IndexedMesh::from_parts(vertices, faces)
}

// =============================================================================
// verify_octahedron_geometry — math-pass-first invariant
// =============================================================================

/// Lock 6 vertex coords + 8 face-winding cross-product unit normals.
/// All anchors hit `1e-12`; integer vertex coordinates make the cross
/// products bit-exact and `1 / √3` is correctly-rounded by the libm.
fn verify_octahedron_geometry(mesh: &IndexedMesh, r: f64) {
    assert_eq!(
        mesh.vertices.len(),
        OCTAHEDRON_VERT_COUNT,
        "octahedron must have {OCTAHEDRON_VERT_COUNT} vertices",
    );
    assert_eq!(
        mesh.faces.len(),
        OCTAHEDRON_FACE_COUNT,
        "octahedron must have {OCTAHEDRON_FACE_COUNT} faces",
    );

    let expected_v: [[f64; 3]; OCTAHEDRON_VERT_COUNT] = [
        [r, 0.0, 0.0],
        [-r, 0.0, 0.0],
        [0.0, r, 0.0],
        [0.0, -r, 0.0],
        [0.0, 0.0, r],
        [0.0, 0.0, -r],
    ];
    for (i, expected) in expected_v.iter().enumerate() {
        let v = mesh.vertices[i];
        assert_relative_eq!(v.x, expected[0], epsilon = VERTEX_TOL);
        assert_relative_eq!(v.y, expected[1], epsilon = VERTEX_TOL);
        assert_relative_eq!(v.z, expected[2], epsilon = VERTEX_TOL);
    }

    // Expected outward unit normal `(sx, sy, sz) / √3` per octant.
    let inv_sqrt3 = 1.0 / 3.0_f64.sqrt();
    let expected_n: [[f64; 3]; OCTAHEDRON_FACE_COUNT] = [
        [inv_sqrt3, inv_sqrt3, inv_sqrt3],    //  f0  +x+y+z
        [inv_sqrt3, -inv_sqrt3, -inv_sqrt3],  //  f1  +x-y-z
        [-inv_sqrt3, inv_sqrt3, -inv_sqrt3],  //  f2  -x+y-z
        [-inv_sqrt3, -inv_sqrt3, inv_sqrt3],  //  f3  -x-y+z
        [inv_sqrt3, inv_sqrt3, -inv_sqrt3],   //  f4  +x+y-z
        [inv_sqrt3, -inv_sqrt3, inv_sqrt3],   //  f5  +x-y+z
        [-inv_sqrt3, inv_sqrt3, inv_sqrt3],   //  f6  -x+y+z
        [-inv_sqrt3, -inv_sqrt3, -inv_sqrt3], //  f7  -x-y-z
    ];
    for (i, expected) in expected_n.iter().enumerate() {
        let f = mesh.faces[i];
        let p0 = mesh.vertices[f[0] as usize];
        let p1 = mesh.vertices[f[1] as usize];
        let p2 = mesh.vertices[f[2] as usize];
        let e1 = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z];
        let e2 = [p2.x - p0.x, p2.y - p0.y, p2.z - p0.z];
        let n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        assert!(len > 0.0, "f{i} has degenerate cross product");
        let unit = [n[0] / len, n[1] / len, n[2] / len];
        assert_relative_eq!(unit[0], expected[0], epsilon = NORMAL_TOL);
        assert_relative_eq!(unit[1], expected[1], epsilon = NORMAL_TOL);
        assert_relative_eq!(unit[2], expected[2], epsilon = NORMAL_TOL);
    }
}

// =============================================================================
// verify_empty_mesh — error path + free-fn f64::MAX sentinel
// =============================================================================

/// `SignedDistanceField::new(empty)` returns `Err(SdfError::EmptyMesh)`
/// per `sdf.rs:48-51`; the one-shot free fns return `f64::MAX` as a
/// sentinel per `sdf.rs:236-239` + `sdf.rs:285-288`.
fn verify_empty_mesh() {
    let empty = IndexedMesh::new();
    let result = SignedDistanceField::new(empty.clone());
    assert!(result.is_err(), "empty mesh must return Err");
    let Err(err) = result else {
        unreachable!("checked is_err above");
    };
    assert!(matches!(err, SdfError::EmptyMesh));

    let q = Point3::new(0.0, 0.0, 0.0);
    assert_relative_eq!(signed_distance(q, &empty), f64::MAX, epsilon = 0.0);
    assert_relative_eq!(unsigned_distance(q, &empty), f64::MAX, epsilon = 0.0);
}

// =============================================================================
// verify_sdf_query_points — 14 analytical anchors
// =============================================================================

/// 14 query points covering interior face regions, vertex-clamped
/// off-mesh, and far-exterior; analytical SDF derived from the L1-ball
/// formula `(|x| + |y| + |z| − 1) / √3` for face-region queries and
/// vertex-clamped Euclidean distance for vertex-direction queries.
///
/// Returns `(generic_signed, face_center_signed, vertex_unsigned, far_signed)`
/// for the print-summary stage.
fn verify_sdf_query_points(sdf: &SignedDistanceField) -> (f64, f64, f64, f64) {
    let inv_sqrt3 = 1.0 / 3.0_f64.sqrt();
    let third = 1.0 / 3.0;

    // ---- (1) Generic interior `(0.05, 0.07, 0.11)` ----
    let generic = Point3::new(0.05, 0.07, 0.11);
    let generic_signed = -0.77 * inv_sqrt3;
    assert_relative_eq!(
        sdf.distance(generic),
        generic_signed,
        epsilon = DISTANCE_TOL
    );
    assert_relative_eq!(
        sdf.unsigned_distance(generic),
        0.77 * inv_sqrt3,
        epsilon = DISTANCE_TOL,
    );
    let cp = sdf.closest_point(generic);
    // Closest point lies on f0 plane `x+y+z = 1`, projected from
    // generic along (1,1,1)/√3. Each coord = generic.k + 0.77/3.
    assert_relative_eq!(cp.x, 0.05 + 0.77 / 3.0, epsilon = DISTANCE_TOL);
    assert_relative_eq!(cp.y, 0.07 + 0.77 / 3.0, epsilon = DISTANCE_TOL);
    assert_relative_eq!(cp.z, 0.11 + 0.77 / 3.0, epsilon = DISTANCE_TOL);
    assert!(sdf.is_inside(generic), "generic interior must be inside");

    // ---- (2) 6 face-center direction queries (interior, 6 octants) ----
    let face_center_signed = -0.1 * inv_sqrt3;
    let face_cases: [(f64, f64, f64); 6] = [
        (1.0, 1.0, 1.0),    //  f0  +++
        (1.0, -1.0, -1.0),  //  f1  +--
        (-1.0, 1.0, -1.0),  //  f2  -+-
        (-1.0, -1.0, 1.0),  //  f3  --+
        (1.0, 1.0, -1.0),   //  f4  ++-
        (-1.0, -1.0, -1.0), //  f7  ---
    ];
    for (i, (sx, sy, sz)) in face_cases.iter().enumerate() {
        let q = Point3::new(0.3 * sx, 0.3 * sy, 0.3 * sz);
        assert_relative_eq!(sdf.distance(q), face_center_signed, epsilon = DISTANCE_TOL);
        assert_relative_eq!(
            sdf.unsigned_distance(q),
            0.1 * inv_sqrt3,
            epsilon = DISTANCE_TOL,
        );
        // Closest point is the face centroid `(sx/3, sy/3, sz/3)`.
        let cp = sdf.closest_point(q);
        assert_relative_eq!(cp.x, sx * third, epsilon = DISTANCE_TOL);
        assert_relative_eq!(cp.y, sy * third, epsilon = DISTANCE_TOL);
        assert_relative_eq!(cp.z, sz * third, epsilon = DISTANCE_TOL);
        assert!(
            sdf.is_inside(q),
            "face-center query {i} ({sx},{sy},{sz})·0.3 must be inside"
        );
    }

    // ---- (3) 6 vertex-direction off-mesh queries (each clamps to a vertex) ----
    let vertex_unsigned = 1.0;
    let vertex_cases: [(Point3<f64>, Point3<f64>); 6] = [
        (Point3::new(2.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)),
        (Point3::new(-2.0, 0.0, 0.0), Point3::new(-1.0, 0.0, 0.0)),
        (Point3::new(0.0, 2.0, 0.0), Point3::new(0.0, 1.0, 0.0)),
        (Point3::new(0.0, -2.0, 0.0), Point3::new(0.0, -1.0, 0.0)),
        (Point3::new(0.0, 0.0, 2.0), Point3::new(0.0, 0.0, 1.0)),
        (Point3::new(0.0, 0.0, -2.0), Point3::new(0.0, 0.0, -1.0)),
    ];
    for (i, (q, expected_cp)) in vertex_cases.iter().enumerate() {
        // Clamped-to-vertex: unsigned == 1 exact; signed == +1 exact
        // (sign is determined by face normal of closest face dotted
        // with `q - face.v0`; for all 4 faces touching the closest
        // vertex, that dot product is `1/√3 > 0`, so sign = +1).
        assert_relative_eq!(sdf.distance(*q), vertex_unsigned, epsilon = DISTANCE_TOL);
        assert_relative_eq!(
            sdf.unsigned_distance(*q),
            vertex_unsigned,
            epsilon = DISTANCE_TOL,
        );
        let cp = sdf.closest_point(*q);
        assert_relative_eq!(cp.x, expected_cp.x, epsilon = DISTANCE_TOL);
        assert_relative_eq!(cp.y, expected_cp.y, epsilon = DISTANCE_TOL);
        assert_relative_eq!(cp.z, expected_cp.z, epsilon = DISTANCE_TOL);
        assert!(
            !sdf.is_inside(*q),
            "vertex-direction query {i} {q:?} must be outside",
        );
    }

    // ---- (4) Far exterior `(10, 10, 10)` ----
    let far = Point3::new(10.0, 10.0, 10.0);
    let far_signed = 29.0 * inv_sqrt3;
    assert_relative_eq!(sdf.distance(far), far_signed, epsilon = DISTANCE_TOL);
    assert_relative_eq!(
        sdf.unsigned_distance(far),
        far_signed,
        epsilon = DISTANCE_TOL
    );
    // Closest point is f0 centroid `(1/3, 1/3, 1/3)`.
    let cp = sdf.closest_point(far);
    assert_relative_eq!(cp.x, third, epsilon = DISTANCE_TOL);
    assert_relative_eq!(cp.y, third, epsilon = DISTANCE_TOL);
    assert_relative_eq!(cp.z, third, epsilon = DISTANCE_TOL);
    assert!(!sdf.is_inside(far), "far exterior must be outside");

    (
        generic_signed,
        face_center_signed,
        vertex_unsigned,
        far_signed,
    )
}

// =============================================================================
// verify_one_shot_equivalence — free fns match cached
// =============================================================================

/// One-shot `signed_distance` / `unsigned_distance` free fns produce
/// bit-equivalent values to the cached `SignedDistanceField` for the
/// same input mesh + query, since both walk identical code paths
/// (`compute_face_normals` + iterate `closest_point_on_triangle` + sign
/// from face-normal dot product).
fn verify_one_shot_equivalence(mesh: &IndexedMesh, sdf: &SignedDistanceField) {
    let queries = [
        Point3::new(0.05, 0.07, 0.11),
        Point3::new(0.3, 0.3, 0.3),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 10.0),
    ];
    for q in queries {
        let cached_signed = sdf.distance(q);
        let oneshot_signed = signed_distance(q, mesh);
        assert_eq!(
            cached_signed.to_bits(),
            oneshot_signed.to_bits(),
            "signed_distance free fn must bit-equal cached at {q:?}",
        );
        let cached_unsigned = sdf.unsigned_distance(q);
        let oneshot_unsigned = unsigned_distance(q, mesh);
        assert_eq!(
            cached_unsigned.to_bits(),
            oneshot_unsigned.to_bits(),
            "unsigned_distance free fn must bit-equal cached at {q:?}",
        );
    }
}

// =============================================================================
// verify_closest_point_on_triangle — 3 sub-cases
// =============================================================================

/// Direct calls to `closest_point_on_triangle` covering the 3 region
/// cases of Ericson's algorithm: face-interior, vertex region, edge
/// region. Triangle is the same one used by `mesh-sdf::query` tests
/// (vertices `(0, 0, 0)`, `(10, 0, 0)`, `(5, 10, 0)`).
fn verify_closest_point_on_triangle() {
    let v0 = Point3::new(0.0, 0.0, 0.0);
    let v1 = Point3::new(10.0, 0.0, 0.0);
    let v2 = Point3::new(5.0, 10.0, 0.0);

    // (a) Face-interior: query lies above face centroid; project to z=0.
    let q = Point3::new(5.0, 3.0, 5.0);
    let cp = closest_point_on_triangle(q, v0, v1, v2);
    assert_relative_eq!(cp.x, 5.0, epsilon = DISTANCE_TOL);
    assert_relative_eq!(cp.y, 3.0, epsilon = DISTANCE_TOL);
    assert_relative_eq!(cp.z, 0.0, epsilon = DISTANCE_TOL);

    // (b) Vertex region (vertex A): both `d1, d2 ≤ 0` so we return v0.
    let q = Point3::new(-5.0, -5.0, 0.0);
    let cp = closest_point_on_triangle(q, v0, v1, v2);
    assert_relative_eq!(cp.x, v0.x, epsilon = DISTANCE_TOL);
    assert_relative_eq!(cp.y, v0.y, epsilon = DISTANCE_TOL);
    assert_relative_eq!(cp.z, v0.z, epsilon = DISTANCE_TOL);

    // (c) Edge region (edge AB): query is below the v0-v1 edge.
    let q = Point3::new(5.0, -5.0, 0.0);
    let cp = closest_point_on_triangle(q, v0, v1, v2);
    assert_relative_eq!(cp.x, 5.0, epsilon = DISTANCE_TOL);
    assert_relative_eq!(cp.y, 0.0, epsilon = DISTANCE_TOL);
    assert_relative_eq!(cp.z, 0.0, epsilon = DISTANCE_TOL);
}

// =============================================================================
// verify_ray_triangle_intersect — 3 sub-cases
// =============================================================================

/// Direct calls to `ray_triangle_intersect` covering hit / miss /
/// parallel cases via Möller-Trumbore. Same triangle as above.
fn verify_ray_triangle_intersect() {
    let v0 = Point3::new(0.0, 0.0, 0.0);
    let v1 = Point3::new(10.0, 0.0, 0.0);
    let v2 = Point3::new(5.0, 10.0, 0.0);

    // (a) Hit: `(5, 3, 5)` shooting `-Z` enters triangle at `(5, 3, 0)`,
    //     `t = 5`.
    let origin = Point3::new(5.0, 3.0, 5.0);
    let dir = Vector3::new(0.0, 0.0, -1.0);
    let Some(t) = ray_triangle_intersect(origin, dir, v0, v1, v2) else {
        unreachable!("ray must hit triangle");
    };
    assert_relative_eq!(t, 5.0, epsilon = DISTANCE_TOL);

    // (b) Miss: ray origin far outside triangle xy bounds.
    let origin = Point3::new(100.0, 100.0, 5.0);
    let dir = Vector3::new(0.0, 0.0, -1.0);
    assert!(ray_triangle_intersect(origin, dir, v0, v1, v2).is_none());

    // (c) Parallel: ray direction along `+X` is parallel to the
    //     triangle plane (z=0); ray never reaches the plane.
    let origin = Point3::new(5.0, 3.0, 5.0);
    let dir = Vector3::new(1.0, 0.0, 0.0);
    assert!(ray_triangle_intersect(origin, dir, v0, v1, v2).is_none());
}

// =============================================================================
// verify_point_segment_distance_squared — 2 sub-cases
// =============================================================================

/// Direct calls to `point_segment_distance_squared`. Returns the
/// squared distance to the segment, with `t` clamped to `[0, 1]` so
/// past-endpoint queries clamp to the nearest endpoint.
fn verify_point_segment_distance_squared() {
    let a = Point3::new(0.0, 0.0, 0.0);
    let b = Point3::new(10.0, 0.0, 0.0);

    // (a) Perpendicular drop: query 5 units above segment midpoint.
    let q = Point3::new(5.0, 5.0, 0.0);
    let d2 = point_segment_distance_squared(q, a, b);
    assert_relative_eq!(d2, 25.0, epsilon = DISTANCE_TOL);

    // (b) Beyond endpoint: `t < 0` clamps to endpoint a; distance == 5.
    let q = Point3::new(-5.0, 0.0, 0.0);
    let d2 = point_segment_distance_squared(q, a, b);
    assert_relative_eq!(d2, 25.0, epsilon = DISTANCE_TOL);
}

// =============================================================================
// verify_point_in_mesh — 2 boolean queries (avoiding HE-1 origin)
// =============================================================================

/// `point_in_mesh` ray-casts in `+X` and counts intersections; odd
/// count means inside. The off-axis interior `(0.05, 0.07, 0.11)`
/// avoids the origin corner-degeneracy (HE-1).
fn verify_point_in_mesh(mesh: &IndexedMesh) {
    assert!(point_in_mesh(Point3::new(0.05, 0.07, 0.11), mesh));
    assert!(!point_in_mesh(Point3::new(10.0, 10.0, 10.0), mesh));
}

// =============================================================================
// Bulk grid query + PLY export
// =============================================================================

/// Generate the 1000-point cubic grid in `[-2, 2]³` (10 × 10 × 10,
/// spacing `4/9`, endpoint-inclusive) and cache `(point, signed_dist)`
/// pairs.
fn bulk_query(sdf: &SignedDistanceField) -> Vec<(Point3<f64>, f64)> {
    let mut grid = Vec::with_capacity(GRID_TOTAL);
    let span = GRID_MAX - GRID_MIN;
    #[allow(clippy::cast_precision_loss)]
    let denom = (GRID_SIZE - 1) as f64;
    for ix in 0..GRID_SIZE {
        for iy in 0..GRID_SIZE {
            for iz in 0..GRID_SIZE {
                #[allow(clippy::cast_precision_loss)]
                let x = GRID_MIN + (ix as f64) * span / denom;
                #[allow(clippy::cast_precision_loss)]
                let y = GRID_MIN + (iy as f64) * span / denom;
                #[allow(clippy::cast_precision_loss)]
                let z = GRID_MIN + (iz as f64) * span / denom;
                let p = Point3::new(x, y, z);
                grid.push((p, sdf.distance(p)));
            }
        }
    }
    grid
}

/// Verify discrete bulk-grid stats — TWO inside-counters disagree by
/// design:
///
/// 1. **Ray-casting** (`point_in_mesh`): 8 of 1000 (matches analytical
///    L1-ball `|x|+|y|+|z| < 1` for the off-axis grid points).
/// 2. **Signed-distance** (`signed_distance < 0`): 14 of 1000 (8 true
///    + 6 vertex-region false-positives at the bbox boundary).
///
/// And max unsigned distance `5/√3 ≈ 2.887` at the 8 bbox corners.
/// The divergence between the two counts is the v0.9 trigger for
/// pseudo-normal / winding-number sign convention (spec §10 item 8;
/// see module doc + README "Bulk grid" callout for derivation).
fn verify_bulk_query_stats(grid: &[(Point3<f64>, f64)], mesh: &IndexedMesh) -> (usize, usize, f64) {
    assert_eq!(grid.len(), GRID_TOTAL);

    let count_raycast = grid.iter().filter(|(p, _)| point_in_mesh(*p, mesh)).count();
    assert_eq!(
        count_raycast, EXPECTED_INSIDE_COUNT_RAYCAST,
        "ray-casting must report {EXPECTED_INSIDE_COUNT_RAYCAST} inside; got {count_raycast}",
    );
    #[allow(clippy::cast_precision_loss)]
    let fraction_raycast = count_raycast as f64 / GRID_TOTAL as f64;
    assert_relative_eq!(fraction_raycast, 0.008, epsilon = 0.0);

    let count_signed = grid.iter().filter(|(_, d)| *d < 0.0).count();
    assert_eq!(
        count_signed, EXPECTED_INSIDE_COUNT_SIGNED,
        "signed_distance < 0 must report {EXPECTED_INSIDE_COUNT_SIGNED}; got {count_signed}",
    );

    let max_unsigned = grid.iter().map(|(_, d)| d.abs()).fold(0.0_f64, f64::max);
    let expected_max = 5.0 / 3.0_f64.sqrt();
    assert_relative_eq!(max_unsigned, expected_max, epsilon = DISTANCE_TOL);

    (count_raycast, count_signed, max_unsigned)
}

/// Save the bulk grid as a PLY with `extras["signed_distance"]`
/// per-vertex scalar, no faces. Reuses the `save_ply_attributed`
/// pattern from `examples/mesh/ply-with-custom-attributes`.
fn save_grid_ply(grid: &[(Point3<f64>, f64)], path: &Path) -> Result<()> {
    let vertices: Vec<Point3<f64>> = grid.iter().map(|(p, _)| *p).collect();
    let faces: Vec<[u32; 3]> = Vec::new();
    let geometry = IndexedMesh::from_parts(vertices, faces);
    let mut attributed = AttributedMesh::new(geometry);
    let scalars: Vec<f32> = grid.iter().map(|(_, d)| *d as f32).collect();
    attributed.insert_extra("signed_distance", scalars)?;
    save_ply_attributed(&attributed, path, true)?;
    Ok(())
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

/// Bundled summary inputs for [`print_summary`]; avoids
/// `clippy::too_many_arguments` while keeping fields trivially
/// constructed at call site.
struct Summary<'a> {
    sdf: &'a SignedDistanceField,
    generic_signed: f64,
    face_center_signed: f64,
    vertex_unsigned: f64,
    far_signed: f64,
    count_raycast: usize,
    count_signed: usize,
    max_unsigned: f64,
}

/// Print the human-readable summary of fixture, query anchors, direct
/// primitive coverage, and bulk-grid stats. Extracted from `main` to
/// keep the entrypoint under clippy's `too_many_lines` cap.
fn print_summary(s: &Summary) {
    println!("==== mesh-sdf-distance-query ====");
    println!();
    println!(
        "input  : unit octahedron ({} verts + {} tris)",
        s.sdf.mesh().vertices.len(),
        s.sdf.mesh().faces.len(),
    );
    println!("         vertices at (±1, 0, 0), (0, ±1, 0), (0, 0, ±1)");
    println!("         8 faces (one per (sx, sy, sz) octant); parity-flipped");
    println!("         winding for the 4 octants where sx*sy*sz = -1");
    println!();
    println!("Empty-mesh edge case:");
    println!("  SignedDistanceField::new(empty) = Err(EmptyMesh)");
    println!("  signed_distance(p, empty)       = f64::MAX");
    println!("  unsigned_distance(p, empty)     = f64::MAX");
    println!();
    println!("SDF query points (14 total; analytical SDF (|x|+|y|+|z| - 1) / sqrt(3)):");
    println!(
        "  generic interior (0.05, 0.07, 0.11) signed  = {:.6}  (inside)",
        s.generic_signed,
    );
    println!(
        "  face-center direction ±(0.3, 0.3, 0.3) signed = {:.6}  (interior, 6 octants)",
        s.face_center_signed,
    );
    println!(
        "  vertex-direction off-mesh ±(2, 0, 0) etc. unsigned = {:.6}  (clamped vertex; 6 dirs)",
        s.vertex_unsigned,
    );
    println!(
        "  far exterior (10, 10, 10) signed = {:.6}  (= 29 / sqrt(3))",
        s.far_signed,
    );
    println!();
    println!("Direct geometric primitives:");
    println!("  closest_point_on_triangle: face-interior + edge + vertex region");
    println!("  ray_triangle_intersect:    hit (t = 5) + miss + parallel (None)");
    println!("  point_segment_distance_squared: perpendicular drop + beyond endpoint");
    println!();
    println!("Inside-test (point_in_mesh):");
    println!("  (0.05, 0.07, 0.11) → true   (off-axis interior, avoids HE-1)");
    println!("  (10, 10, 10)       → false  (far exterior)");
    println!();
    println!("Bulk grid (10×10×10 in [-2, 2]³, spacing 4/9):");
    #[allow(clippy::cast_precision_loss)]
    let percent_raycast = 100.0 * (s.count_raycast as f64) / (GRID_TOTAL as f64);
    #[allow(clippy::cast_precision_loss)]
    let percent_signed = 100.0 * (s.count_signed as f64) / (GRID_TOTAL as f64);
    println!(
        "  inside via point_in_mesh   = {} / {GRID_TOTAL} = {percent_raycast:.1}%  \
         (matches analytical L1-ball; continuous 1/48 ≈ 2.083%)",
        s.count_raycast,
    );
    println!(
        "  inside via signed < 0      = {} / {GRID_TOTAL} = {percent_signed:.1}%  \
         ({EXPECTED_INSIDE_COUNT_RAYCAST} true + {} vertex-region false-positives, drift-10)",
        s.count_signed,
        EXPECTED_INSIDE_COUNT_SIGNED - EXPECTED_INSIDE_COUNT_RAYCAST,
    );
    println!(
        "  max unsigned = {:.6}  (= 5 / sqrt(3); achieved at all 8 bbox corners)",
        s.max_unsigned,
    );
    println!();
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    let mesh = build_octahedron(OCTAHEDRON_RADIUS);
    verify_octahedron_geometry(&mesh, OCTAHEDRON_RADIUS);
    verify_empty_mesh();

    let Ok(sdf) = SignedDistanceField::new(mesh.clone()) else {
        unreachable!("octahedron has 8 faces; new must succeed");
    };

    let (generic_signed, face_center_signed, vertex_unsigned, far_signed) =
        verify_sdf_query_points(&sdf);
    verify_one_shot_equivalence(&mesh, &sdf);
    verify_closest_point_on_triangle();
    verify_ray_triangle_intersect();
    verify_point_segment_distance_squared();
    verify_point_in_mesh(&mesh);

    let grid = bulk_query(&sdf);
    let (count_raycast, count_signed, max_unsigned) = verify_bulk_query_stats(&grid, &mesh);

    // sdf.mesh() accessor is exercised by print_summary's header.
    print_summary(&Summary {
        sdf: &sdf,
        generic_signed,
        face_center_signed,
        vertex_unsigned,
        far_signed,
        count_raycast,
        count_signed,
        max_unsigned,
    });

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let mesh_path = out_dir.join("octahedron.ply");
    save_ply(&mesh, &mesh_path, false)?;
    println!(
        "artifact: out/octahedron.ply ({}v, {}f, ASCII)",
        mesh.vertices.len(),
        mesh.faces.len(),
    );
    let grid_path = out_dir.join("sdf_grid.ply");
    save_grid_ply(&grid, &grid_path)?;
    println!(
        "artifact: out/sdf_grid.ply ({GRID_TOTAL}v, 0f, binary; \
         extras[\"signed_distance\"] for external colormap)"
    );
    println!();
    println!(
        "OK — 6 vertex + 8 winding + empty-mesh + 14 SDF query + one-shot equivalence + \
         3 closest_point_on_triangle + 3 ray_triangle_intersect + \
         2 point_segment_distance_squared + 2 point_in_mesh + bulk-grid \
         (8 raycast / 14 signed-neg, max 5/sqrt(3)) anchors all green"
    );

    Ok(())
}
