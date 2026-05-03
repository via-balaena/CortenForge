//! mesh-measure-bounding-box — AABB + OBB on a two-shape fixture.
//!
//! Demonstrates the full `mesh-measure::dimensions` +
//! `oriented_bounding_box` public surface against a hand-authored
//! two-shape fixture: 16 vertex + 24 winding anchors, Dimensions on
//! the combined mesh, OBB on the combined mesh, and OBB on the brick
//! alone.
//!
//! Fixture: 16 verts + 24 tris in two vertex-disjoint shapes —
//! - **Cube A**: axis-aligned 10 mm cube at `[0, 10] × [-5, 5] × [0, 10]`
//!   (y-centered on the origin so cube A's y-range sits inside the
//!   tilted brick's; combined-mesh AABB then has clean
//!   `center.y = 0` per spec §5.1) (verts 0–7).
//! - **Cube B**: 20 × 10 × 10 mm brick (long axis along local +X)
//!   rotated 45° around Z and translated so its bbox-center is
//!   `(25, 0, 5)` (verts 8–15). The brick is non-cubic so PCA's
//!   eigenvalues are distinct — the OBB recovers the brick's
//!   actual `(20, 10, 10)` extents while AABB inflates the rotated
//!   shadow to `(15√2, 15√2, 10) ≈ (21.21, 21.21, 10)`.
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
use mesh_measure::{Dimensions, OrientedBoundingBox, dimensions, oriented_bounding_box};
use mesh_types::{IndexedMesh, Point3};

// =============================================================================
// Constants
// =============================================================================

/// Per-vertex coordinate tolerance — fixture inputs are FP-exact
/// (integer multiples + `f64::sqrt(0.5)`).
const VERTEX_TOL: f64 = 1e-12;

/// Per-face winding unit-normal tolerance.
const NORMAL_TOL: f64 = 1e-12;

/// Tolerance for analytically-derived AABB anchors — closed-form
/// (only multiplications and sqrt of FP-exact integers); 1 ULP-level.
const ANALYTICAL_TOL: f64 = 1e-12;

/// Tolerance for PCA-derived OBB axis components and brick recovery —
/// nalgebra's `SymmetricEigen` is iterative and can vary at ~1e-15
/// per platform; 1e-9 is a comfortable upper bound.
const PCA_TOL: f64 = 1e-9;

/// Volume tolerance for the combined-mesh OBB — per spec §5.1, 5%
/// reflects covariance-matrix FP stability across libms.
const OBB_VOLUME_REL_TOL: f64 = 0.05;

/// Combined-mesh OBB anchor: principal-axis x-component (empirical,
/// from `atan(3/8) / 2 ≈ 10.275°`). Closed form is
/// `cos(atan(3/8) / 2) = sqrt((sqrt(73) + 8) / (2·sqrt(73)))`.
const COMBINED_OBB_AXIS_X: f64 = 0.983_953_550_115_310_3;

/// Combined-mesh OBB anchor: principal-axis y-component (empirical;
/// `sin(atan(3/8) / 2) = sqrt((sqrt(73) - 8) / (2·sqrt(73)))`).
const COMBINED_OBB_AXIS_Y: f64 = 0.178_424_805_493_736_6;

/// Combined-mesh OBB anchor: volume — empirical, locked at the
/// post-PCA value of `7169.483166351446`. Deterministic per nalgebra
/// `SymmetricEigen` on this fixture's covariance.
const COMBINED_OBB_VOLUME: f64 = 7_169.483_166_351_446;

/// Brick (cube B) analytical OBB volume — `BRICK_LONG × BRICK_SHORT
/// × BRICK_SHORT = 20 × 10 × 10 = 2000`.
const BRICK_OBB_VOLUME: f64 = BRICK_LONG * BRICK_SHORT * BRICK_SHORT;

/// Vertex count of cube A (axis-aligned). Cube B vertex indices are
/// `CUBE_A_VERT_COUNT + local_index` (verts 8..15).
const CUBE_A_VERT_COUNT: usize = 8;

/// Same as `CUBE_A_VERT_COUNT` but typed `u32` for face-index arithmetic.
const CUBE_A_VERT_COUNT_U32: u32 = 8;

/// Total vertex count.
const TOTAL_VERT_COUNT: usize = 16;

/// Total face count.
const TOTAL_FACE_COUNT: usize = 24;

/// Tilted-brick bbox-center — placed at `+x = 25` to leave a clear gap
/// between the two shapes (cube A spans `x ∈ [0, 10]`, cube B spans
/// `x ∈ [25 − 7.5√2, 25 + 7.5√2] ≈ [14.39, 35.61]`). Both shapes'
/// y-ranges straddle `y = 0` so the combined AABB is symmetric in y.
const TILTED_CENTER_X: f64 = 25.0;
const TILTED_CENTER_Y: f64 = 0.0;
const TILTED_CENTER_Z: f64 = 5.0;

/// Cube A is a 10 mm cube — full edge 10 mm, half-extent 5 mm.
const CUBE_A_EDGE: f64 = 10.0;

/// Brick (cube B) full edge lengths along local axes — long axis is
/// 20 mm along local +X; short axes are 10 mm along local Y / Z.
/// The 2:1 aspect ratio ensures non-degenerate PCA: the long-axis
/// eigenvalue separates cleanly from the two short-axis eigenvalues.
const BRICK_LONG: f64 = 20.0;
const BRICK_SHORT: f64 = 10.0;

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
/// printability-showcase `build_body` template).
///
/// Cube B: local 20×10×10 brick `[−10, 10] × [−5, 5] × [−5, 5]`
/// rotated by `R(z, 45°)` then translated to `(25, 0, 5)`. The
/// rotation `R · (x, y, z) = (s(x − y), s(x + y), z)` with
/// `s = f64::sqrt(0.5)` maps the four bottom corners to the offsets
/// `(±a, ±b)` and `(±b, ±a)` (in centroid-frame xy), where
/// `a = 5s = 2.5√2 ≈ 3.54` and `b = 15s = 7.5√2 ≈ 10.61`.
fn expected_vertices() -> [[f64; 3]; TOTAL_VERT_COUNT] {
    let s = rot_coef();
    let a = 5.0 * s; // short rotated offset = 2.5·√2 ≈ 3.54
    let b = 15.0 * s; // long rotated offset = 7.5·√2 ≈ 10.61

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
        // ─── Cube B: 20×10×10 brick tilted 45° around Z, center (25, 0, 5) ─
        // Local 0 (−10, −5, −5) → R: (−5s, −15s, −5) → world: (25−a, −b, 0)
        [TILTED_CENTER_X - a, -b, 0.0], //  8
        // Local 1 (+10, −5, −5) → R: (15s, 5s, −5) → world: (25+b, +a, 0)
        [TILTED_CENTER_X + b, a, 0.0], //  9
        // Local 2 (+10, +5, −5) → R: (5s, 15s, −5) → world: (25+a, +b, 0)
        [TILTED_CENTER_X + a, b, 0.0], // 10
        // Local 3 (−10, +5, −5) → R: (−15s, −5s, −5) → world: (25−b, −a, 0)
        [TILTED_CENTER_X - b, -a, 0.0], // 11
        // Local 4..7 — same (x, y) as 0..3; z = +5 → world z = 10
        [TILTED_CENTER_X - a, -b, 10.0], // 12
        [TILTED_CENTER_X + b, a, 10.0],  // 13
        [TILTED_CENTER_X + a, b, 10.0],  // 14
        [TILTED_CENTER_X - b, -a, 10.0], // 15
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
        [v0 + 1, v0 + 6, v0 + 5],
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

/// Rotate a world-space point into an OBB's local frame —
/// `local = R⁻¹ · (point − center)`. Mirrors `OrientedBoundingBox::contains`
/// but returns the local offset so anchors can apply per-axis tolerance.
/// Used by both `verify_combined_obb` and `verify_brick_obb`.
fn obb_local_offset(obb: &OrientedBoundingBox, point: Point3<f64>) -> nalgebra::Vector3<f64> {
    obb.rotation.inverse() * (point - obb.center)
}

// =============================================================================
// verify_dimensions — AABB anchors per spec §5.1
// =============================================================================

/// Lock every `Dimensions` field on the combined two-cube mesh against
/// the analytical AABB of `[0, 25 + 7.5√2] × [-7.5√2, 7.5√2] × [0, 10]`.
/// All anchors are closed-form (multiplications + `f64::sqrt(0.5)`)
/// and hold to `ANALYTICAL_TOL`.
fn verify_dimensions(dims: &Dimensions) {
    let s = rot_coef();
    let b = 15.0 * s; // long rotated offset = 7.5·√2

    let expected_width = TILTED_CENTER_X + b; // 25 + 7.5√2 ≈ 35.61
    let expected_depth = 2.0 * b; // 15√2 ≈ 21.21
    let expected_height = 10.0;
    let expected_volume = expected_width * expected_depth * expected_height;
    let expected_diagonal = expected_height
        .mul_add(
            expected_height,
            expected_width.mul_add(expected_width, expected_depth * expected_depth),
        )
        .sqrt();

    // Per-axis min / max.
    assert_relative_eq!(dims.min.x, 0.0, epsilon = ANALYTICAL_TOL);
    assert_relative_eq!(dims.min.y, -b, epsilon = ANALYTICAL_TOL);
    assert_relative_eq!(dims.min.z, 0.0, epsilon = ANALYTICAL_TOL);
    assert_relative_eq!(dims.max.x, expected_width, epsilon = ANALYTICAL_TOL);
    assert_relative_eq!(dims.max.y, b, epsilon = ANALYTICAL_TOL);
    assert_relative_eq!(dims.max.z, expected_height, epsilon = ANALYTICAL_TOL);

    // Derived extents.
    assert_relative_eq!(dims.width, expected_width, epsilon = ANALYTICAL_TOL);
    assert_relative_eq!(dims.depth, expected_depth, epsilon = ANALYTICAL_TOL);
    assert_relative_eq!(dims.height, expected_height, epsilon = ANALYTICAL_TOL);
    assert_relative_eq!(dims.diagonal, expected_diagonal, epsilon = ANALYTICAL_TOL);
    assert_relative_eq!(
        dims.bounding_volume,
        expected_volume,
        epsilon = ANALYTICAL_TOL
    );

    // Center is the midpoint of the combined AABB.
    assert_relative_eq!(
        dims.center.x,
        expected_width / 2.0,
        epsilon = ANALYTICAL_TOL
    );
    assert_relative_eq!(dims.center.y, 0.0, epsilon = ANALYTICAL_TOL);
    assert_relative_eq!(dims.center.z, 5.0, epsilon = ANALYTICAL_TOL);

    // Convenience accessors.
    assert_relative_eq!(dims.min_extent(), expected_height, epsilon = ANALYTICAL_TOL);
    assert_relative_eq!(dims.max_extent(), expected_width, epsilon = ANALYTICAL_TOL);
    assert_relative_eq!(
        dims.aspect_ratio(),
        expected_width / expected_height,
        epsilon = ANALYTICAL_TOL
    );
    assert!(
        !dims.is_cubic(0.01),
        "combined mesh is far from cubic (aspect ratio ≈ {:.2})",
        dims.aspect_ratio()
    );
    let size = dims.size();
    assert_relative_eq!(size.x, expected_width, epsilon = ANALYTICAL_TOL);
    assert_relative_eq!(size.y, expected_depth, epsilon = ANALYTICAL_TOL);
    assert_relative_eq!(size.z, expected_height, epsilon = ANALYTICAL_TOL);
}

// =============================================================================
// verify_combined_obb — OBB anchors on the combined mesh
// =============================================================================

/// Lock the combined-mesh `OrientedBoundingBox` against empirical
/// post-PCA values. The combined fixture (cube A + tilted brick) has
/// off-diagonal `Cov(x, y) ≠ 0` from the brick's elongation; PCA
/// finds a principal axis ~10.27° from world `+x` and the OBB
/// volume reduces from the AABB's `7553.30` to `7169.48` (5.1%
/// improvement). Less dramatic than the brick-alone OBB recovery
/// (see [`verify_brick_obb`]) but pedagogically real.
fn verify_combined_obb(obb: &OrientedBoundingBox, mesh: &IndexedMesh) {
    // Volume — empirical anchor with 5% tolerance per spec §5.1.
    assert_relative_eq!(
        obb.volume,
        COMBINED_OBB_VOLUME,
        max_relative = OBB_VOLUME_REL_TOL
    );

    // Principal axis components (cos / sin of ~10.27°, verified to 1e-9).
    let axis_x = obb.axis_x();
    assert_relative_eq!(axis_x.x.abs(), COMBINED_OBB_AXIS_X, epsilon = PCA_TOL);
    assert_relative_eq!(axis_x.y.abs(), COMBINED_OBB_AXIS_Y, epsilon = PCA_TOL);
    assert_relative_eq!(axis_x.z, 0.0, epsilon = PCA_TOL);

    // Axes orthogonal (per §5.1 implicit OrientedBoundingBox::axis_*
    // surface coverage).
    let axis_y = obb.axis_y();
    let axis_z = obb.axis_z();
    assert_relative_eq!(axis_x.dot(&axis_y), 0.0, epsilon = PCA_TOL);
    assert_relative_eq!(axis_y.dot(&axis_z), 0.0, epsilon = PCA_TOL);
    assert_relative_eq!(axis_z.dot(&axis_x), 0.0, epsilon = PCA_TOL);

    // Surface area > 0.
    assert!(
        obb.surface_area() > 0.0,
        "combined OBB has non-positive surface area: {}",
        obb.surface_area()
    );

    // All 16 input vertices satisfy `obb.contains(v) == true` per spec
    // §5.1 — implemented with a 1-ULP tolerance because PCA's iterative
    // eigen-decomposition computes `half_extents` and the inverse-rotation
    // mapping with ~1e-15 FP roundoff. Vertices that defined the OBB
    // (the extremes along each principal axis) project back to the
    // boundary of the local frame and `<=` comparison can trip by 1 ULP
    // (here: vertices 8 / 12 fail by `dy = 1.776e-15`). The strict
    // library behavior is captured as a v0.9 candidate gap in spec §10.
    for (i, v) in mesh.vertices.iter().enumerate() {
        let local = obb_local_offset(obb, *v);
        assert!(
            local.x.abs() <= obb.half_extents.x + PCA_TOL
                && local.y.abs() <= obb.half_extents.y + PCA_TOL
                && local.z.abs() <= obb.half_extents.z + PCA_TOL,
            "OBB containment fails for input vertex {i}: |local|=({:.6}, {:.6}, {:.6}) \
             vs half_extents=({:.6}, {:.6}, {:.6})",
            local.x.abs(),
            local.y.abs(),
            local.z.abs(),
            obb.half_extents.x,
            obb.half_extents.y,
            obb.half_extents.z,
        );
    }

    // (No "8 OBB corners within inflated AABB" anchor here — that
    // anchor only holds when the OBB rotation is identity. For a
    // non-degenerate OBB at ~10° tilt, corners can extend several mm
    // outside the AABB envelope. Geometrically: the input mesh sits
    // inside both AABB and OBB, but neither contains the other. The
    // 16-vertex `obb.contains()` anchor above is the proper enclosure
    // test. Captured as v0.9 candidate gap in spec §10.)
}

// =============================================================================
// verify_brick_obb — OBB on cube B sub-mesh recovers the original brick
// =============================================================================

/// Lock the OBB on cube B alone (the 20×10×10 tilted brick, extracted
/// as a sub-mesh) against the analytical recovery: `volume = 2000`,
/// sorted extents `(20, 10, 10)`, center at `(25, 0, 5)`. PCA on the
/// brick's 8 vertices has eigenvalues `(33.33, 8.33, 8.33)` (long-axis
/// distinct from the two equal short axes), so the principal axis is
/// uniquely the rotated `+X` and the OBB recovers the box exactly.
fn verify_brick_obb(obb: &OrientedBoundingBox, brick_verts: &[Point3<f64>]) {
    // Volume recovery.
    assert_relative_eq!(obb.volume, BRICK_OBB_VOLUME, epsilon = PCA_TOL);

    // Sorted extents recover (20, 10, 10).
    let e = obb.extents();
    let mut sorted = [e.x, e.y, e.z];
    // Descending sort by total_cmp — OBB extents are finite (no NaN);
    // total_cmp gives a strict total order without an Option.
    sorted.sort_by(|a, b| b.total_cmp(a));
    assert_relative_eq!(sorted[0], BRICK_LONG, epsilon = PCA_TOL);
    assert_relative_eq!(sorted[1], BRICK_SHORT, epsilon = PCA_TOL);
    assert_relative_eq!(sorted[2], BRICK_SHORT, epsilon = PCA_TOL);

    // Center is at the brick's bbox-center.
    assert_relative_eq!(obb.center.x, TILTED_CENTER_X, epsilon = PCA_TOL);
    assert_relative_eq!(obb.center.y, TILTED_CENTER_Y, epsilon = PCA_TOL);
    assert_relative_eq!(obb.center.z, TILTED_CENTER_Z, epsilon = PCA_TOL);

    // All 8 brick vertices contained (1-ULP-tolerance contains —
    // same FP-roundoff caveat as the combined-OBB anchor).
    for (i, v) in brick_verts.iter().enumerate() {
        let local = obb_local_offset(obb, *v);
        assert!(
            local.x.abs() <= obb.half_extents.x + PCA_TOL
                && local.y.abs() <= obb.half_extents.y + PCA_TOL
                && local.z.abs() <= obb.half_extents.z + PCA_TOL,
            "brick OBB does not contain brick vertex {i} = {v:?}"
        );
    }

    // Surface area = 2·(20·10 + 20·10 + 10·10) = 2·(200+200+100) = 1000 mm².
    assert_relative_eq!(obb.surface_area(), 1000.0, epsilon = PCA_TOL);
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    let mesh = build_fixture();
    verify_fixture_geometry(&mesh);

    let dims = dimensions(&mesh);
    let combined_obb = oriented_bounding_box(&mesh);
    verify_dimensions(&dims);
    verify_combined_obb(&combined_obb, &mesh);

    // Sub-mesh extraction: cube B alone (the brick), no faces — OBB
    // is computed from vertex positions only.
    let brick_verts: Vec<Point3<f64>> = mesh.vertices[CUBE_A_VERT_COUNT..].to_vec();
    let brick_mesh = IndexedMesh::from_parts(brick_verts.clone(), Vec::new());
    let brick_obb = oriented_bounding_box(&brick_mesh);
    verify_brick_obb(&brick_obb, &brick_verts);

    println!("==== mesh-measure-bounding-box ====");
    println!();
    println!(
        "input  : {}-vertex, {}-triangle two-shape fixture",
        mesh.vertices.len(),
        mesh.faces.len(),
    );
    println!(
        "         cube A : axis-aligned {CUBE_A_EDGE} mm cube at [0, 10] × [-5, 5] × [0, 10] \
         (y-centered; {CUBE_A_VERT_COUNT} verts)"
    );
    println!(
        "         cube B : {BRICK_LONG} × {BRICK_SHORT} × {BRICK_SHORT} mm brick tilted 45° \
         around Z, bbox-center ({TILTED_CENTER_X}, {TILTED_CENTER_Y}, {TILTED_CENTER_Z}) \
         (8 verts)"
    );
    println!();

    println!("AABB on combined mesh (dimensions):");
    println!(
        "  width × depth × height = {:.4} × {:.4} × {:.4}",
        dims.width, dims.depth, dims.height
    );
    println!(
        "  bounding_volume = {:.4} mm³  (= 3750·√2 + 2250)",
        dims.bounding_volume
    );
    println!(
        "  center = ({:.4}, {:.4}, {:.4})  diagonal = {:.4}",
        dims.center.x, dims.center.y, dims.center.z, dims.diagonal,
    );
    println!(
        "  aspect_ratio = {:.4}  is_cubic(0.01) = {}",
        dims.aspect_ratio(),
        dims.is_cubic(0.01),
    );
    println!();

    let combined_axis_x = combined_obb.axis_x();
    let principal_angle_deg = combined_axis_x
        .y
        .atan2(combined_axis_x.x)
        .to_degrees()
        .abs();
    println!("OBB on combined mesh (oriented_bounding_box):");
    println!(
        "  volume = {:.4} mm³  ({:.2}% reduction vs AABB; PCA finds compromise tilt)",
        combined_obb.volume,
        (1.0 - combined_obb.volume / dims.bounding_volume) * 100.0,
    );
    println!(
        "  principal axis = ({:.4}, {:.4}, {:.4})  ≈ {:.2}° from world +X",
        combined_axis_x.x, combined_axis_x.y, combined_axis_x.z, principal_angle_deg,
    );
    let combined_extents = combined_obb.extents();
    println!(
        "  extents = ({:.4}, {:.4}, {:.4})",
        combined_extents.x, combined_extents.y, combined_extents.z,
    );
    println!();

    let brick_extents = brick_obb.extents();
    let mut brick_sorted = [brick_extents.x, brick_extents.y, brick_extents.z];
    brick_sorted.sort_by(|a, b| b.total_cmp(a));
    println!("OBB on cube B alone (the 20×10×10 brick, sub-mesh):");
    println!(
        "  volume = {:.4} mm³  (recovers analytical {} — PCA finds 45° rotation)",
        brick_obb.volume, BRICK_OBB_VOLUME,
    );
    println!(
        "  extents (sorted) = ({:.4}, {:.4}, {:.4})",
        brick_sorted[0], brick_sorted[1], brick_sorted[2],
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
        "OK — 16 vertex + 24 winding + AABB Dimensions + combined OBB + \
         brick-alone OBB recovery anchors all green"
    );

    Ok(())
}
