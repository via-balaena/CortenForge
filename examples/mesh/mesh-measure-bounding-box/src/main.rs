//! mesh-measure-bounding-box — AABB + OBB on a two-cube fixture.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.1. This commit (§6.2 #3)
//! lands the hand-authored two-cube fixture with per-vertex coord
//! anchors and per-face winding cross-product anchors. AABB + OBB
//! anchors land in §6.2 #4.
//!
//! Fixture: 16 verts + 24 tris in two vertex-disjoint cubes —
//! - **Cube A**: axis-aligned 10 mm cube at `[0, 10] × [-5, 5] × [0, 10]`
//!   (y-centered on the origin, so cube A's y-range sits inside cube
//!   B's `[-5√2, 5√2]`; combined-mesh AABB then has clean
//!   `depth = 10√2` and `center.y = 0` per spec §5.1) (verts 0–7).
//! - **Cube B**: 10 mm cube rotated 45° around Z, translated so its
//!   bbox-center is `(25, 0, 5)` (verts 8–15).
//!
//! Per §4.4 of the spec, the rotation uses `let s = f64::sqrt(0.5)`
//! for both cosine and sine coefficients of the 45° rotation matrix —
//! `f64::sqrt` is correctly-rounded per IEEE-754 (`f64::sin(π/4)` and
//! `f64::cos(π/4)` are NOT correctly-rounded; the 1-ULP divergence
//! `f64::sqrt(2)/2 = 0.70710678118654757` vs `f64::sin(π/4) =
//! 0.70710678118654746` would defeat the `1e-12` per-vertex tolerance).

// Cross-product unit-normal computation reads as the textbook formula
// `e1.y*e2.z - e1.z*e2.y`; the `mul_add` rewrite obscures intent and
// produces bit-equivalent results on integer vertex coordinates.
// (Same precedent as printability-showcase verify_fixture_geometry.)
#![allow(clippy::suboptimal_flops)]

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply;
use mesh_types::{IndexedMesh, Point3};

// =============================================================================
// Constants
// =============================================================================

/// Per-vertex coordinate tolerance — fixture inputs are FP-exact
/// (integer multiples + `f64::sqrt(0.5)`).
const VERTEX_TOL: f64 = 1e-12;

/// Per-face winding unit-normal tolerance.
const NORMAL_TOL: f64 = 1e-12;

/// Vertex count of cube A (axis-aligned). Cube B vertex indices are
/// `CUBE_A_VERT_COUNT + local_index` (verts 8..15).
const CUBE_A_VERT_COUNT: usize = 8;

/// Same as `CUBE_A_VERT_COUNT` but typed `u32` for face-index arithmetic.
const CUBE_A_VERT_COUNT_U32: u32 = 8;

/// Total vertex count.
const TOTAL_VERT_COUNT: usize = 16;

/// Total face count.
const TOTAL_FACE_COUNT: usize = 24;

/// Tilted-cube bbox-center — placed at `+x = 25` to leave a clear gap
/// between the two cubes (cube A spans `x ∈ [0, 10]`, cube B spans
/// `x ∈ [25 − 5√2, 25 + 5√2] ≈ [17.93, 32.07]`). Both cubes' y-ranges
/// straddle `y = 0` so the combined AABB is symmetric in y.
const TILTED_CENTER_X: f64 = 25.0;
const TILTED_CENTER_Y: f64 = 0.0;
const TILTED_CENTER_Z: f64 = 5.0;

/// Half-extent of each cube (full edge length 10 mm).
const HALF_EXTENT: f64 = 5.0;

/// `f64::sqrt(0.5) = √(1/2)` — the cosine and sine coefficient of a
/// 45° rotation. IEEE-754-correctly-rounded; deterministic across
/// libm versions.
fn rot_coef() -> f64 {
    f64::sqrt(0.5)
}

// =============================================================================
// Expected geometry (analytical anchors)
// =============================================================================

/// Hand-authored expected vertex coordinates.
///
/// Cube A: 8 corners of `[0, 10] × [-5, 5] × [0, 10]` (y-centered on
/// origin), indexed bottom-CCW then top-CCW (matches the
/// printability-showcase `build_body` template). Cube A's y-range
/// `[-5, 5]` sits strictly inside cube B's `[-5√2, 5√2]`, so the
/// combined-mesh AABB inherits cube B's y-extent.
///
/// Cube B: local cube `[−5, 5]³` rotated by `R(z, 45°)` then
/// translated to `(25, 0, 5)`. The rotation
/// `R · (x, y, z) = (s(x − y), s(x + y), z)` with `s = f64::sqrt(0.5)`
/// produces the 8 listed corners.
fn expected_vertices() -> [[f64; 3]; TOTAL_VERT_COUNT] {
    let s = rot_coef();
    let r = 10.0 * s; // cube-B radial offset = 10·√(1/2) = 5·√2

    [
        // ─── Cube A: axis-aligned at [0, 10] × [-5, 5] × [0, 10] ──────
        [0.0, -5.0, 0.0],   //  0  bottom-front-left
        [10.0, -5.0, 0.0],  //  1  bottom-front-right
        [10.0, 5.0, 0.0],   //  2  bottom-back-right
        [0.0, 5.0, 0.0],    //  3  bottom-back-left
        [0.0, -5.0, 10.0],  //  4  top-front-left
        [10.0, -5.0, 10.0], //  5  top-front-right
        [10.0, 5.0, 10.0],  //  6  top-back-right
        [0.0, 5.0, 10.0],   //  7  top-back-left
        // ─── Cube B: tilted 45° around Z, bbox-center (25, 0, 5) ─────
        // Local index 0 (−5, −5, −5) → R: (0, −10s, −5) → world: (25, −10s, 0)
        [TILTED_CENTER_X, -r, 0.0], //  8
        // Local 1 (+5, −5, −5) → R: (10s, 0, −5) → world: (25+10s, 0, 0)
        [TILTED_CENTER_X + r, 0.0, 0.0], //  9
        // Local 2 (+5, +5, −5) → R: (0, 10s, −5) → world: (25, 10s, 0)
        [TILTED_CENTER_X, r, 0.0], // 10
        // Local 3 (−5, +5, −5) → R: (−10s, 0, −5) → world: (25−10s, 0, 0)
        [TILTED_CENTER_X - r, 0.0, 0.0], // 11
        // Local 4..7 — same (x, y); z = +5 → world z = 10
        [TILTED_CENTER_X, -r, 10.0],      // 12
        [TILTED_CENTER_X + r, 0.0, 10.0], // 13
        [TILTED_CENTER_X, r, 10.0],       // 14
        [TILTED_CENTER_X - r, 0.0, 10.0], // 15
    ]
}

/// Hand-authored expected face winding unit normals.
///
/// Both cubes use the same 12-face template (2 tris per face × 6
/// faces). Cube A's face normals are axis-aligned (±x, ±y, ±z). Cube
/// B's lateral faces rotate by 45° in the XY plane:
///
/// - local `−y` (front)  → world `(s, −s, 0)`
/// - local `+y` (back)   → world `(−s, s, 0)`
/// - local `−x` (left)   → world `(−s, −s, 0)`
/// - local `+x` (right)  → world `(s, s, 0)`
///
/// Top / bottom faces stay along ±z under the Z-axis rotation.
fn expected_face_normals() -> [[f64; 3]; TOTAL_FACE_COUNT] {
    let s = rot_coef();

    [
        // ─── Cube A: 12 faces ─────────────────────────────────────────
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
        // ─── Cube B: 12 faces (rotated 45° around Z) ─────────────────
        [0.0, 0.0, -1.0], // 12 bottom
        [0.0, 0.0, -1.0], // 13
        [0.0, 0.0, 1.0],  // 14 top
        [0.0, 0.0, 1.0],  // 15
        [s, -s, 0.0],     // 16 front (local −y → (s, −s, 0))
        [s, -s, 0.0],     // 17
        [-s, s, 0.0],     // 18 back (local +y → (−s, s, 0))
        [-s, s, 0.0],     // 19
        [-s, -s, 0.0],    // 20 left (local −x → (−s, −s, 0))
        [-s, -s, 0.0],    // 21
        [s, s, 0.0],      // 22 right (local +x → (s, s, 0))
        [s, s, 0.0],      // 23
    ]
}

// =============================================================================
// Fixture builder
// =============================================================================

/// Build the two-cube fixture.
fn build_fixture() -> IndexedMesh {
    let expected = expected_vertices();
    let vertices: Vec<Point3<f64>> = expected
        .iter()
        .map(|v| Point3::new(v[0], v[1], v[2]))
        .collect();

    // 12-face template per cube (2 tris × 6 faces). Same winding as
    // printability-showcase build_body. Cube B uses the same template
    // shifted by CUBE_A_VERT_COUNT_U32 (= 8).
    let v0 = CUBE_A_VERT_COUNT_U32;
    let faces: Vec<[u32; 3]> = vec![
        // ─── Cube A: 12 faces ─────────────────────────────────────────
        // bottom (−z)
        [0, 3, 2],
        [0, 2, 1],
        // top (+z)
        [4, 5, 6],
        [4, 6, 7],
        // front (−y)
        [0, 1, 5],
        [0, 5, 4],
        // back (+y)
        [3, 7, 6],
        [3, 6, 2],
        // left (−x)
        [0, 4, 7],
        [0, 7, 3],
        // right (+x)
        [1, 2, 6],
        [1, 6, 5],
        // ─── Cube B: 12 faces (indices +8; world normals after 45° Z rotation) ─
        // bottom (−z)
        [v0, v0 + 3, v0 + 2],
        [v0, v0 + 2, v0 + 1],
        // top (+z)
        [v0 + 4, v0 + 5, v0 + 6],
        [v0 + 4, v0 + 6, v0 + 7],
        // front (+s, −s, 0)
        [v0, v0 + 1, v0 + 5],
        [v0, v0 + 5, v0 + 4],
        // back (−s, +s, 0)
        [v0 + 3, v0 + 7, v0 + 6],
        [v0 + 3, v0 + 6, v0 + 2],
        // left (−s, −s, 0)
        [v0, v0 + 4, v0 + 7],
        [v0, v0 + 7, v0 + 3],
        // right (+s, +s, 0)
        [v0 + 1, v0 + 2, v0 + 6],
        [v0 + 1, v0 + 6, v0 + 5], // right  (+s, +s, 0)
    ];

    IndexedMesh::from_parts(vertices, faces)
}

// =============================================================================
// verify_fixture_geometry — math-pass-first invariant
// =============================================================================

/// Lock every visible property of the hand-authored fixture as a
/// numerical invariant. Per `feedback_math_pass_first_handauthored`,
/// a successful `cargo run --release` exit-0 with this verifier active
/// is equivalent to a clean visual inspection: typos in the vertex
/// arrays, swapped winding on a face, and rotation-coefficient bugs
/// all surface here.
fn verify_fixture_geometry(mesh: &IndexedMesh) {
    assert_eq!(
        mesh.vertices.len(),
        TOTAL_VERT_COUNT,
        "fixture must have {TOTAL_VERT_COUNT} vertices; got {}",
        mesh.vertices.len(),
    );
    assert_eq!(
        mesh.faces.len(),
        TOTAL_FACE_COUNT,
        "fixture must have {TOTAL_FACE_COUNT} faces; got {}",
        mesh.faces.len(),
    );

    // (1) Per-vertex coordinates within VERTEX_TOL of the analytical
    //     expected coords.
    let expected_v = expected_vertices();
    for (i, expected) in expected_v.iter().enumerate() {
        let v = &mesh.vertices[i];
        assert_relative_eq!(v.x, expected[0], epsilon = VERTEX_TOL);
        assert_relative_eq!(v.y, expected[1], epsilon = VERTEX_TOL);
        assert_relative_eq!(v.z, expected[2], epsilon = VERTEX_TOL);
    }

    // (2) Per-face winding cross-product unit normals within NORMAL_TOL.
    let expected_n = expected_face_normals();
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
            "face {i} has degenerate cross product (zero-area triangle)",
        );
        let unit = [n[0] / len, n[1] / len, n[2] / len];
        assert_relative_eq!(unit[0], expected[0], epsilon = NORMAL_TOL);
        assert_relative_eq!(unit[1], expected[1], epsilon = NORMAL_TOL);
        assert_relative_eq!(unit[2], expected[2], epsilon = NORMAL_TOL);
    }
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    let mesh = build_fixture();
    verify_fixture_geometry(&mesh);

    println!("==== mesh-measure-bounding-box (§6.2 #3 — fixture only) ====");
    println!();
    println!(
        "input  : {}-vertex, {}-triangle two-cube fixture",
        mesh.vertices.len(),
        mesh.faces.len(),
    );
    println!(
        "         cube A : axis-aligned 10mm cube at [0, 10] × [-5, 5] × [0, 10] \
         (y-centered; {CUBE_A_VERT_COUNT} verts)"
    );
    println!(
        "         cube B : 10 mm cube tilted 45° around Z, bbox-center \
         ({TILTED_CENTER_X}, {TILTED_CENTER_Y}, {TILTED_CENTER_Z}) \
         (half-extent {HALF_EXTENT})"
    );
    println!();

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let mesh_path = out_dir.join("mesh.ply");
    save_ply(&mesh, &mesh_path, false)?;

    println!(
        "artifact: out/mesh.ply ({}v, {}f, ASCII)",
        mesh.vertices.len(),
        mesh.faces.len(),
    );
    println!();
    println!(
        "OK — fixture geometry verified (16 vertex anchors @ {VERTEX_TOL:.0e} + \
         24 winding anchors @ {NORMAL_TOL:.0e})"
    );
    println!("(AABB + OBB anchors land in §6.2 #4.)");

    Ok(())
}
