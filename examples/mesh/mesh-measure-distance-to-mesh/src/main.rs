//! mesh-measure-distance-to-mesh — point-to-point + point-to-mesh +
//! symmetric Hausdorff distance composed from the public surface.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.3. Demonstrates the full
//! `mesh-measure::measure_distance` + `closest_point_on_mesh` +
//! `distance_to_mesh` public surface against two SEPARATE
//! axis-aligned `IndexedMesh` instances. Anchors landed across §6.2
//! #7 (skeleton at commit `16ac9d42`) and §6.2 #8 (16 vertex coords,
//! 24 face-winding unit-normal per-component anchors at `1e-12`
//! (cross products on FP-exact integer cube coords are bit-exact;
//! no need for the cosine-similarity floor used in cross-section's
//! sin/cos-derived UV-cylinder), point-to-point on the 3-4-5
//! triangle, 6 face + 8 corner + 1 center point-to-mesh queries on
//! `cube_a`, empty-mesh edge case, symmetric Hausdorff = sqrt(12)).
//!
//! Fixture: two vertex-disjoint unit cubes, each its own
//! `IndexedMesh` (per spec §5.3 — Hausdorff iteration requires
//! independent `distance_to_mesh` calls per cube):
//! - `cube_a` : axis-aligned unit cube at `[0, 1]³` (8 verts, 12 tris).
//! - `cube_b` : axis-aligned unit cube at `[2, 3]³` (8 verts, 12 tris).
//!
//! Closest-face distance between the cubes along `+X` is `1.0`;
//! farthest-corner-pair `(0, 0, 0)` ↔ `(3, 3, 3)` is at distance
//! `3√3 ≈ 5.196`. The symmetric Hausdorff value `sqrt(12) ≈ 3.464`
//! is the closest-point-on-cube_b from `cube_a`'s farthest vertex
//! `(0, 0, 0)` — clamped to `cube_b`'s `(2, 2, 2)` corner, NOT to
//! the face-interior `(2, 0, 0)` (that face's `(y, z) ∈ [2, 3]²`
//! bounds exclude `(0, 0)`). This `sqrt(12)` vs naive `sqrt(8)`
//! correction came from spec §8 round 1.
//!
//! `DistanceMeasurement::dx` / `dy` / `dz` are **absolute** values
//! per `mesh-measure/src/distance.rs:97-99` — `measure_distance(
//! from=(0, 0, 0), to=(-3, -4, 0))` returns `dx = 3.0`, NOT `-3.0`.
//! The point-to-point verifier exercises only the all-positive
//! direction `(0, 0, 0) → (3, 4, 0)` so the absolute-value behavior
//! is implicit; the README surfaces the convention explicitly.

// Cross-product unit-normal computation reads as the textbook formula
// `e1.y*e2.z - e1.z*e2.y`; the `mul_add` rewrite obscures intent and
// produces bit-equivalent results on integer vertex coordinates.
// (Same precedent as bbox + cross-section verify_fixture_geometry.)
#![allow(clippy::suboptimal_flops)]

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply;
use mesh_measure::{
    DistanceMeasurement, closest_point_on_mesh, distance_to_mesh, measure_distance,
};
use mesh_types::{IndexedMesh, Point3};

// =============================================================================
// Constants
// =============================================================================

/// Per-vertex coordinate tolerance — fixture inputs are FP-exact
/// integer corners.
const VERTEX_TOL: f64 = 1e-12;

/// Per-face winding cross-product unit-normal tolerance — analytical
/// ±x / ±y / ±z directions; FP-exact integer cross products.
const NORMAL_TOL: f64 = 1e-12;

/// Tolerance for distance + closest-point anchors. Distances on
/// FP-exact integer cube coords are bit-exact (Pythagorean
/// `sqrt` of integer sums); `1e-12` is comparison slack.
const DISTANCE_TOL: f64 = 1e-12;

/// Vertex count per cube (axis-aligned: 8 corners).
const CUBE_VERT_COUNT: usize = 8;

/// Face count per cube (12 tris = 2 per face × 6 faces).
const CUBE_FACE_COUNT: usize = 12;

/// Total vertex count of the combined PLY mesh (`cube_a` + `cube_b`).
const COMBINED_VERT_COUNT: usize = 16;

/// Total face count of the combined PLY mesh.
const COMBINED_FACE_COUNT: usize = 24;

/// Vertex-index offset between `cube_a` and `cube_b` in the combined mesh.
const COMBINED_VERT_OFFSET: u32 = 8;

/// Cube A axis-aligned bounds: corners at `(0|1, 0|1, 0|1)`.
const CUBE_A_MIN: f64 = 0.0;
const CUBE_A_MAX: f64 = 1.0;

/// Cube B axis-aligned bounds: corners at `(2|3, 2|3, 2|3)`.
const CUBE_B_MIN: f64 = 2.0;
const CUBE_B_MAX: f64 = 3.0;

/// Symmetric Hausdorff anchor — `sqrt(12) = 2·sqrt(3)`. Both
/// one-sided maxima are achieved at the diagonally-opposite corner
/// pair: `cube_a.(0, 0, 0)` clamps to `cube_b.(2, 2, 2)`, and
/// `cube_b.(3, 3, 3)` clamps to `cube_a.(1, 1, 1)`. Square distance
/// `4 + 4 + 4 = 12`.
const HAUSDORFF_EXPECTED: f64 = 12.0; // square value; sqrt taken below

// =============================================================================
// Fixture builders
// =============================================================================

/// Build an axis-aligned unit cube spanning `[min, max]³`.
///
/// Vertex layout (matches the bbox + printability-showcase
/// 12-face template):
/// - v0 `(min, min, min)` — bottom-front-left
/// - v1 `(max, min, min)` — bottom-front-right
/// - v2 `(max, max, min)` — bottom-back-right
/// - v3 `(min, max, min)` — bottom-back-left
/// - v4 `(min, min, max)` — top-front-left
/// - v5 `(max, min, max)` — top-front-right
/// - v6 `(max, max, max)` — top-back-right
/// - v7 `(min, max, max)` — top-back-left
fn unit_cube_at(min: f64, max: f64) -> IndexedMesh {
    let vertices: Vec<Point3<f64>> = vec![
        Point3::new(min, min, min), //  0  bottom-front-left
        Point3::new(max, min, min), //  1  bottom-front-right
        Point3::new(max, max, min), //  2  bottom-back-right
        Point3::new(min, max, min), //  3  bottom-back-left
        Point3::new(min, min, max), //  4  top-front-left
        Point3::new(max, min, max), //  5  top-front-right
        Point3::new(max, max, max), //  6  top-back-right
        Point3::new(min, max, max), //  7  top-back-left
    ];
    IndexedMesh::from_parts(vertices, cube_face_template(0))
}

/// Standard 12-tri unit-cube face template, with vertex indices
/// offset by `base`. Used by both [`unit_cube_at`] (base=0,
/// per-cube standalone) and [`build_combined`] (base=0 for `cube_a`,
/// base=8 for `cube_b`).
///
/// Winding produces outward-facing normals on a CCW-from-outside
/// convention:
/// - faces 0-1: bottom (-z)
/// - faces 2-3: top (+z)
/// - faces 4-5: front (-y)
/// - faces 6-7: back (+y)
/// - faces 8-9: left (-x)
/// - faces 10-11: right (+x)
fn cube_face_template(base: u32) -> Vec<[u32; 3]> {
    vec![
        // bottom (−z)
        [base, base + 3, base + 2],
        [base, base + 2, base + 1],
        // top (+z)
        [base + 4, base + 5, base + 6],
        [base + 4, base + 6, base + 7],
        // front (−y)
        [base, base + 1, base + 5],
        [base, base + 5, base + 4],
        // back (+y)
        [base + 3, base + 7, base + 6],
        [base + 3, base + 6, base + 2],
        // left (−x)
        [base, base + 4, base + 7],
        [base, base + 7, base + 3],
        // right (+x)
        [base + 1, base + 2, base + 6],
        [base + 1, base + 6, base + 5],
    ]
}

/// Build the 16-vertex / 24-tri combined mesh for visualization.
/// `cube_a` occupies vert indices 0..8 + face indices 0..12;
/// `cube_b` occupies vert indices 8..16 + face indices 12..24.
fn build_combined(cube_a: &IndexedMesh, cube_b: &IndexedMesh) -> IndexedMesh {
    let mut vertices: Vec<Point3<f64>> = Vec::with_capacity(COMBINED_VERT_COUNT);
    vertices.extend_from_slice(&cube_a.vertices);
    vertices.extend_from_slice(&cube_b.vertices);

    let mut faces: Vec<[u32; 3]> = Vec::with_capacity(COMBINED_FACE_COUNT);
    faces.extend(cube_face_template(0));
    faces.extend(cube_face_template(COMBINED_VERT_OFFSET));

    IndexedMesh::from_parts(vertices, faces)
}

// =============================================================================
// verify_cube_geometry — math-pass-first invariant per cube
// =============================================================================

/// Lock every visible property of one axis-aligned unit cube as a
/// numerical invariant (8 vert coords + 12 face-winding unit
/// normals). Per `feedback_math_pass_first_handauthored`, a
/// successful `cargo run --release` exit-0 with this verifier
/// active is equivalent to a clean visual inspection.
fn verify_cube_geometry(mesh: &IndexedMesh, label: &str, min: f64, max: f64) {
    assert_eq!(
        mesh.vertices.len(),
        CUBE_VERT_COUNT,
        "{label} must have {CUBE_VERT_COUNT} vertices; got {}",
        mesh.vertices.len(),
    );
    assert_eq!(
        mesh.faces.len(),
        CUBE_FACE_COUNT,
        "{label} must have {CUBE_FACE_COUNT} faces; got {}",
        mesh.faces.len(),
    );

    // (1) Per-vertex coordinates within VERTEX_TOL.
    let expected_v: [[f64; 3]; CUBE_VERT_COUNT] = [
        [min, min, min],
        [max, min, min],
        [max, max, min],
        [min, max, min],
        [min, min, max],
        [max, min, max],
        [max, max, max],
        [min, max, max],
    ];
    for (i, expected) in expected_v.iter().enumerate() {
        let v = mesh.vertices[i];
        assert_relative_eq!(v.x, expected[0], epsilon = VERTEX_TOL);
        assert_relative_eq!(v.y, expected[1], epsilon = VERTEX_TOL);
        assert_relative_eq!(v.z, expected[2], epsilon = VERTEX_TOL);
    }

    // (2) Per-face winding cross-product unit normals within NORMAL_TOL.
    let expected_n: [[f64; 3]; CUBE_FACE_COUNT] = [
        [0.0, 0.0, -1.0], //  0 bottom (−z)
        [0.0, 0.0, -1.0], //  1
        [0.0, 0.0, 1.0],  //  2 top (+z)
        [0.0, 0.0, 1.0],  //  3
        [0.0, -1.0, 0.0], //  4 front (−y)
        [0.0, -1.0, 0.0], //  5
        [0.0, 1.0, 0.0],  //  6 back (+y)
        [0.0, 1.0, 0.0],  //  7
        [-1.0, 0.0, 0.0], //  8 left (−x)
        [-1.0, 0.0, 0.0], //  9
        [1.0, 0.0, 0.0],  // 10 right (+x)
        [1.0, 0.0, 0.0],  // 11
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
        assert!(
            len > 0.0,
            "{label} face {i} has degenerate cross product (zero-area triangle)",
        );
        let unit = [n[0] / len, n[1] / len, n[2] / len];
        assert_relative_eq!(unit[0], expected[0], epsilon = NORMAL_TOL);
        assert_relative_eq!(unit[1], expected[1], epsilon = NORMAL_TOL);
        assert_relative_eq!(unit[2], expected[2], epsilon = NORMAL_TOL);
    }
}

// =============================================================================
// verify_point_to_point — measure_distance + direction + midpoint
// =============================================================================

/// Lock every `DistanceMeasurement` field on the canonical 3-4-5
/// Pythagorean triangle. All anchors are FP-exact integer
/// arithmetic at the source (`3² + 4² = 25 = 5²`); rationals
/// `3/5 = 0.6` and `4/5 = 0.8` round to one ULP.
fn verify_point_to_point() -> DistanceMeasurement {
    let from = Point3::origin();
    let to = Point3::new(3.0, 4.0, 0.0);
    let m = measure_distance(from, to);

    assert_relative_eq!(m.from.x, 0.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(m.from.y, 0.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(m.from.z, 0.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(m.to.x, 3.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(m.to.y, 4.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(m.to.z, 0.0, epsilon = VERTEX_TOL);

    assert_relative_eq!(m.distance, 5.0, epsilon = DISTANCE_TOL);
    assert_relative_eq!(m.dx, 3.0, epsilon = DISTANCE_TOL);
    assert_relative_eq!(m.dy, 4.0, epsilon = DISTANCE_TOL);
    assert_relative_eq!(m.dz, 0.0, epsilon = DISTANCE_TOL);

    let Some(dir) = m.direction_normalized() else {
        unreachable!("direction_normalized returned None on non-zero distance");
    };
    assert_relative_eq!(dir.x, 0.6, epsilon = DISTANCE_TOL);
    assert_relative_eq!(dir.y, 0.8, epsilon = DISTANCE_TOL);
    assert_relative_eq!(dir.z, 0.0, epsilon = DISTANCE_TOL);

    let mid = m.midpoint();
    assert_relative_eq!(mid.x, 1.5, epsilon = DISTANCE_TOL);
    assert_relative_eq!(mid.y, 2.0, epsilon = DISTANCE_TOL);
    assert_relative_eq!(mid.z, 0.0, epsilon = DISTANCE_TOL);

    // Zero-distance edge case — direction_normalized returns None.
    let zero = measure_distance(Point3::origin(), Point3::origin());
    assert!(
        zero.direction_normalized().is_none(),
        "direction_normalized on zero distance should return None"
    );

    m
}

// =============================================================================
// verify_point_to_mesh — face / corner / center queries on cube_a
// =============================================================================

/// 6 face-direction queries on `cube_a`, each placed at the face
/// CENTER offset by `1.0` along the outward normal. Closest point
/// is the face-center on the +/-X|Y|Z face; distance is `1.0`
/// FP-exact.
fn verify_point_to_mesh_face_directions(cube_a: &IndexedMesh) {
    // (query, expected closest point on cube_a). Closest point is
    // unambiguous: the query lies on the outward normal through the
    // face center, so the face-center is the unique nearest point
    // (no edge / corner ties).
    let cases: [(Point3<f64>, Point3<f64>); 6] = [
        (Point3::new(2.0, 0.5, 0.5), Point3::new(1.0, 0.5, 0.5)), // +X
        (Point3::new(-1.0, 0.5, 0.5), Point3::new(0.0, 0.5, 0.5)), // -X
        (Point3::new(0.5, 2.0, 0.5), Point3::new(0.5, 1.0, 0.5)), // +Y
        (Point3::new(0.5, -1.0, 0.5), Point3::new(0.5, 0.0, 0.5)), // -Y
        (Point3::new(0.5, 0.5, 2.0), Point3::new(0.5, 0.5, 1.0)), // +Z
        (Point3::new(0.5, 0.5, -1.0), Point3::new(0.5, 0.5, 0.0)), // -Z
    ];
    for (i, (query, expected_cp)) in cases.iter().enumerate() {
        let Some(d) = distance_to_mesh(cube_a, *query) else {
            unreachable!("face query {i} returned None on cube_a");
        };
        assert_relative_eq!(d, 1.0, epsilon = DISTANCE_TOL);

        let Some(cp) = closest_point_on_mesh(cube_a, *query) else {
            unreachable!("face query {i} closest_point returned None on cube_a");
        };
        assert_relative_eq!(cp.x, expected_cp.x, epsilon = DISTANCE_TOL);
        assert_relative_eq!(cp.y, expected_cp.y, epsilon = DISTANCE_TOL);
        assert_relative_eq!(cp.z, expected_cp.z, epsilon = DISTANCE_TOL);
    }
}

/// 8 corner-direction queries on `cube_a`, each placed `1.0`
/// outward from the cube along all three axes simultaneously.
/// Closest point is the corresponding cube CORNER; distance is
/// `sqrt(3)` FP-exact (Pythagorean of three unit offsets).
fn verify_point_to_mesh_corner_directions(cube_a: &IndexedMesh) {
    let sqrt3 = 3.0_f64.sqrt();
    // (query, expected closest corner of cube_a). Eight corner
    // diagonals from the centroid (0.5, 0.5, 0.5) outward through
    // each of the 8 cube corners, extended by 1.0 along each axis.
    let cases: [(Point3<f64>, Point3<f64>); 8] = [
        (Point3::new(2.0, 2.0, 2.0), Point3::new(1.0, 1.0, 1.0)), //  +x +y +z
        (Point3::new(-1.0, -1.0, -1.0), Point3::new(0.0, 0.0, 0.0)), //  -x -y -z
        (Point3::new(2.0, 2.0, -1.0), Point3::new(1.0, 1.0, 0.0)), // +x +y -z
        (Point3::new(-1.0, -1.0, 2.0), Point3::new(0.0, 0.0, 1.0)), // -x -y +z
        (Point3::new(2.0, -1.0, 2.0), Point3::new(1.0, 0.0, 1.0)), // +x -y +z
        (Point3::new(-1.0, 2.0, -1.0), Point3::new(0.0, 1.0, 0.0)), // -x +y -z
        (Point3::new(-1.0, 2.0, 2.0), Point3::new(0.0, 1.0, 1.0)), // -x +y +z
        (Point3::new(2.0, -1.0, -1.0), Point3::new(1.0, 0.0, 0.0)), // +x -y -z
    ];
    for (i, (query, expected_cp)) in cases.iter().enumerate() {
        let Some(d) = distance_to_mesh(cube_a, *query) else {
            unreachable!("corner query {i} returned None on cube_a");
        };
        assert_relative_eq!(d, sqrt3, epsilon = DISTANCE_TOL);

        let Some(cp) = closest_point_on_mesh(cube_a, *query) else {
            unreachable!("corner query {i} closest_point returned None on cube_a");
        };
        assert_relative_eq!(cp.x, expected_cp.x, epsilon = DISTANCE_TOL);
        assert_relative_eq!(cp.y, expected_cp.y, epsilon = DISTANCE_TOL);
        assert_relative_eq!(cp.z, expected_cp.z, epsilon = DISTANCE_TOL);
    }
}

/// Center query at the cube centroid `(0.5, 0.5, 0.5)`.
/// `distance_to_mesh` is unsigned, so the value is the half-edge
/// `0.5` regardless of inside/outside semantics. The closest point
/// is on the FIRST face triangle returned by `mesh.triangles()` —
/// per the bottom-cap face emission order, that's the bottom face
/// (`z = 0`) tri `[0, 3, 2]`; the projected closest point is
/// `(0.5, 0.5, 0)` lying on the diagonal between the two bottom
/// tris (so either tri returns the same point; the strict `<` tie-
/// break in `closest_point_on_mesh` keeps the first).
fn verify_point_to_mesh_center(cube_a: &IndexedMesh) {
    let query = Point3::new(0.5, 0.5, 0.5);
    let Some(d) = distance_to_mesh(cube_a, query) else {
        unreachable!("center query returned None on cube_a");
    };
    assert_relative_eq!(d, 0.5, epsilon = DISTANCE_TOL);

    let Some(cp) = closest_point_on_mesh(cube_a, query) else {
        unreachable!("center query closest_point returned None on cube_a");
    };
    // Closest is on the bottom face (z = 0). The (x, y) projection
    // lies on the diagonal of the two bottom-cap tris; the first
    // wins via `closest_point_on_mesh`'s strict `<` tie-break.
    assert_relative_eq!(cp.x, 0.5, epsilon = DISTANCE_TOL);
    assert_relative_eq!(cp.y, 0.5, epsilon = DISTANCE_TOL);
    assert_relative_eq!(cp.z, 0.0, epsilon = DISTANCE_TOL);
}

// =============================================================================
// verify_empty_mesh — Option::None edge case
// =============================================================================

/// Both `distance_to_mesh` and `closest_point_on_mesh` return
/// `None` on an empty mesh per `distance.rs:134-136`.
fn verify_empty_mesh() {
    let empty = IndexedMesh::new();
    let query = Point3::new(1.0, 2.0, 3.0);
    assert!(
        distance_to_mesh(&empty, query).is_none(),
        "distance_to_mesh on empty mesh should return None"
    );
    assert!(
        closest_point_on_mesh(&empty, query).is_none(),
        "closest_point_on_mesh on empty mesh should return None"
    );
}

// =============================================================================
// verify_hausdorff — symmetric, composed from primitives
// =============================================================================

/// One-sided Hausdorff `d_H(from → to) = max_v∈from min_p∈to ||v − p||`.
/// Iterates every vertex of `from` and computes
/// `distance_to_mesh(to, v)` (unsigned closest-face distance);
/// returns the maximum.
fn one_sided_hausdorff(from: &IndexedMesh, to: &IndexedMesh) -> f64 {
    assert!(
        !to.faces.is_empty(),
        "one_sided_hausdorff: target mesh must be non-empty"
    );
    let mut max_dist = 0.0_f64;
    for v in &from.vertices {
        let Some(d) = distance_to_mesh(to, *v) else {
            unreachable!(
                "distance_to_mesh returned None despite non-empty target ({} faces)",
                to.faces.len(),
            );
        };
        if d > max_dist {
            max_dist = d;
        }
    }
    max_dist
}

/// Symmetric Hausdorff `H(A, B) = max(d_H(A→B), d_H(B→A))`. For
/// our two-cube fixture, both directions hit `sqrt(12)` at the
/// diagonally-opposite corners — `cube_a.(0, 0, 0)` clamps to
/// `cube_b.(2, 2, 2)`, and `cube_b.(3, 3, 3)` clamps to
/// `cube_a.(1, 1, 1)`. The naive `sqrt(8)` (face-interior
/// projection at `(2, 0, 0)`) is wrong: `cube_b`'s `+X` face has
/// `(y, z) ∈ [2, 3]²`, which excludes the projection point
/// `(0, 0)` — so the closest point clamps to the corner.
fn verify_hausdorff(cube_a: &IndexedMesh, cube_b: &IndexedMesh) -> f64 {
    let expected = HAUSDORFF_EXPECTED.sqrt();
    let d_ab = one_sided_hausdorff(cube_a, cube_b);
    let d_ba = one_sided_hausdorff(cube_b, cube_a);
    assert_relative_eq!(d_ab, expected, epsilon = DISTANCE_TOL);
    assert_relative_eq!(d_ba, expected, epsilon = DISTANCE_TOL);
    let h = d_ab.max(d_ba);
    assert_relative_eq!(h, expected, epsilon = DISTANCE_TOL);
    h
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

/// Print the human-readable summary: input fixture, point-to-point
/// 3-4-5 metrics, point-to-mesh sample queries, empty-mesh
/// `Option::None` behavior, and the symmetric Hausdorff value.
/// Extracted from `main` to keep the entrypoint under clippy's
/// `too_many_lines` cap.
fn print_summary(
    cube_a: &IndexedMesh,
    cube_b: &IndexedMesh,
    p2p: &DistanceMeasurement,
    hausdorff: f64,
) {
    println!("==== mesh-measure-distance-to-mesh ====");
    println!();
    println!(
        "input  : two SEPARATE axis-aligned unit cubes ({} verts + {} tris each)",
        cube_a.vertices.len(),
        cube_a.faces.len(),
    );
    println!(
        "         cube_a : [{CUBE_A_MIN}, {CUBE_A_MAX}]³ ({} verts)",
        cube_a.vertices.len(),
    );
    println!(
        "         cube_b : [{CUBE_B_MIN}, {CUBE_B_MAX}]³ ({} verts)",
        cube_b.vertices.len(),
    );
    println!(
        "         closest-face gap along +X = {} mm  (cube_a.+X = 1; cube_b.-X = 2)",
        CUBE_B_MIN - CUBE_A_MAX,
    );
    println!();

    println!("Point-to-point measure_distance (3-4-5 Pythagorean):");
    println!(
        "  from = ({:.1}, {:.1}, {:.1})  to = ({:.1}, {:.1}, {:.1})",
        p2p.from.x, p2p.from.y, p2p.from.z, p2p.to.x, p2p.to.y, p2p.to.z,
    );
    println!(
        "  distance = {:.6}  (dx, dy, dz) = ({:.1}, {:.1}, {:.1})  [absolute values]",
        p2p.distance, p2p.dx, p2p.dy, p2p.dz,
    );
    let Some(dir) = p2p.direction_normalized() else {
        unreachable!("direction_normalized was None at print time");
    };
    let mid = p2p.midpoint();
    println!(
        "  direction_normalized = ({:.6}, {:.6}, {:.6})  midpoint = ({:.1}, {:.1}, {:.1})",
        dir.x, dir.y, dir.z, mid.x, mid.y, mid.z,
    );
    println!();

    let face_q = Point3::new(2.0, 0.5, 0.5);
    let corner_q = Point3::new(2.0, 2.0, 2.0);
    let center_q = Point3::new(0.5, 0.5, 0.5);
    let Some(face_d) = distance_to_mesh(cube_a, face_q) else {
        unreachable!("face_q distance was None at print time");
    };
    let Some(corner_d) = distance_to_mesh(cube_a, corner_q) else {
        unreachable!("corner_q distance was None at print time");
    };
    let Some(center_d) = distance_to_mesh(cube_a, center_q) else {
        unreachable!("center_q distance was None at print time");
    };
    println!("Point-to-mesh distance_to_mesh on cube_a (sample queries):");
    println!("  face  +X (2.0, 0.5, 0.5)   d = {face_d:.6}  (expected 1.0; 6 face dirs total)");
    println!(
        "  corner +x+y+z (2, 2, 2)    d = {corner_d:.6}  \
         (expected sqrt(3) ≈ {:.6}; 8 corners total)",
        3.0_f64.sqrt(),
    );
    println!(
        "  center (0.5, 0.5, 0.5)     d = {center_d:.6}  \
         (expected 0.5; unsigned, inside/outside agnostic)"
    );
    println!();

    let empty = IndexedMesh::new();
    println!("Empty-mesh edge case:");
    println!(
        "  distance_to_mesh(empty, p)      = {:?}  (expected None)",
        distance_to_mesh(&empty, face_q),
    );
    println!(
        "  closest_point_on_mesh(empty, p) = {:?}  (expected None)",
        closest_point_on_mesh(&empty, face_q),
    );
    println!();

    println!("Symmetric Hausdorff (composed from distance_to_mesh primitive):");
    println!("  d_H(cube_a → cube_b) = max over cube_a's 8 verts of distance_to_mesh(cube_b, v)");
    println!(
        "                       = sqrt(12) ≈ {:.6}  (achieved at vert (0, 0, 0) → \
         clamped corner (2, 2, 2))",
        HAUSDORFF_EXPECTED.sqrt(),
    );
    println!("  d_H(cube_b → cube_a) = sqrt(12) by symmetry  (vert (3, 3, 3) → corner (1, 1, 1))");
    println!(
        "  H(A, B) = max(d_AB, d_BA) = sqrt(12) ≈ {hausdorff:.6}  \
         [NOT sqrt(8) — face-interior projection (2, 0, 0) is outside the +X face's \
         (y, z) ∈ [2, 3]² bounds; clamped to corner instead]",
    );
    println!();
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    let cube_a = unit_cube_at(CUBE_A_MIN, CUBE_A_MAX);
    let cube_b = unit_cube_at(CUBE_B_MIN, CUBE_B_MAX);
    verify_cube_geometry(&cube_a, "cube_a", CUBE_A_MIN, CUBE_A_MAX);
    verify_cube_geometry(&cube_b, "cube_b", CUBE_B_MIN, CUBE_B_MAX);

    let p2p = verify_point_to_point();
    verify_point_to_mesh_face_directions(&cube_a);
    verify_point_to_mesh_corner_directions(&cube_a);
    verify_point_to_mesh_center(&cube_a);
    verify_empty_mesh();
    let hausdorff = verify_hausdorff(&cube_a, &cube_b);

    print_summary(&cube_a, &cube_b, &p2p, hausdorff);

    let combined = build_combined(&cube_a, &cube_b);
    assert_eq!(combined.vertices.len(), COMBINED_VERT_COUNT);
    assert_eq!(combined.faces.len(), COMBINED_FACE_COUNT);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let mesh_path = out_dir.join("two_cubes.ply");
    save_ply(&combined, &mesh_path, false)?;
    println!(
        "artifact: out/two_cubes.ply ({}v, {}f, ASCII)",
        combined.vertices.len(),
        combined.faces.len(),
    );
    println!();
    println!(
        "OK — 16 vertex + 24 winding + point-to-point + 6 face + 8 corner + 1 center + \
         empty-mesh + symmetric Hausdorff = sqrt(12) anchors all green"
    );

    Ok(())
}
