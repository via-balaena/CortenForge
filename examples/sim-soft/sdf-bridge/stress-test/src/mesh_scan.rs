// `unreachable!()` calls in this binary are diagnostic guards on
// `let-else` branches that cannot fire (12-tri cube fixture is
// non-empty by construction → `TriMeshDistance::new` cannot
// return `Err(EmptyMesh)`). `xtask grade`'s Safety criterion counts
// un-justified `unreachable!()` macros; allow at file level since
// every call is a post-validation `Result::Err` impossibility, not
// a real panic site. Same precedent as
// `examples/mesh/mesh-sdf-distance-query`.
#![allow(clippy::unreachable)]
//! mesh-scan — a mesh-derived SDF satisfies [`cf_design::Sdf`].
//!
//! A scanned mesh is a triangle soup with no analytical SDF. Composing
//! a [`mesh_sdf::TriMeshDistance`] (parry BVH-backed unsigned distance)
//! with a [`mesh_sdf::Sign`] oracle via [`mesh_sdf::Signed<D, S>`] —
//! and dispatching through the [`cf_design::Sdf`] trait — turns it into
//! a first-class design primitive on equal footing with parametric
//! [`cf_design::Solid`] bodies, without per-mesh wrapper boilerplate.
//! This is the bridge that lets `mesh-scan` flow into `cf-design` flow
//! into `sim-soft` along the layered-silicone-device pipeline.
//!
//! Fixture: a 12-triangle axis-aligned cube with half-extent
//! `R = 1.0`, built programmatically so the SDF has a closed form and
//! every numerical anchor is hand-verifiable. Outward face winding
//! produces the 6 cube-face outward normals `±x̂`, `±ŷ`, `±ẑ` from the
//! 12 cross products. The "scan" framing is the workflow, not the
//! geometry — stand-in per the layered-silicone-device memo's
//! sanitization directive (no anatomical references in repo).
//!
//! The closed-form L∞-ball SDF for an axis-aligned cube `[−R, R]³` is
//! `φ(p) = √(max(d_x, 0)² + max(d_y, 0)² + max(d_z, 0)²) +
//! min(max(d_x, d_y, d_z), 0)` where `d_i = |p_i| − R`. The first term
//! is the Euclidean exterior distance (face / edge / vertex region);
//! the second is the negative of the closest-face-distance for points
//! strictly inside. This is the textbook box SDF (Inigo Quilez); we
//! re-derive it here so the closed-form anchors below are visibly
//! grounded.
//!
//! On-disk transit: the programmatic mesh is round-tripped through
//! `mesh_io::save_stl` → `mesh_io::load_mesh` → composed [`ScanSdf`]
//! to demo the literal scan-import workflow without a checked-in STL
//! asset. The round-tripped SDF agrees with the in-memory SDF
//! bit-exact at all named probes — STL stores binary f32 vertex
//! coords, but our cube vertices are integer `±1.0` which round-trip
//! losslessly through f32, and parry's internal f32 BVH path produces
//! the same closest-feature classification for both routes.
//!
//! Sign oracle on this fixture: [`mesh_sdf::PseudoNormalSign`] (parry
//! pseudo-normal). Fast on watertight + well-formed meshes; fragile
//! on cleaned body-part scans with inverted-winding cap fans or
//! high-valence apex vertices ([[project-mesh-sdf-oracle-decomposition-spec]]).
//! Production cleaned-scan code paths in cf-cast-cli + cf-device-design
//! ship [`mesh_sdf::FloodFillSign`] defense instead; see
//! `docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md` for the architectural
//! background.
//!
//! On *this* cube fixture both oracles agree everywhere — the bulk
//! grid's `heuristic_inside == STRICT_INTERIOR_COUNT` identity
//! (asserted in `main`) pins the absence of any sign-oracle failure
//! mode. `eval < 0` (strict-inequality heuristic-inside) and
//! [`Signed::is_inside`] now both route through the same
//! [`PseudoNormalSign`], so the historical "heuristic ↔ raycast
//! divergence" framing collapses; the residual divergence (exactly
//! `CLOSED_CUBE_COUNT − STRICT_INTERIOR_COUNT = 729 − 343 = 386` grid
//! points, pinned in `main`) is structural boundary-convention: probes
//! that land exactly on cube faces (`unsigned_distance == 0` →
//! `eval == 0` → `eval < 0` false even when `is_inside` is true).
//! For a
//! fixture where a sign oracle does demonstrably fail (and the D-arc
//! flood-fill defense actively rescues it), see
//! `examples/mesh/mesh-sdf-distance-query`'s inverted-cap pyramid.

// PLY field-data is single-precision on disk; converting f64 SDF
// values to f32 for `extras["signed_distance"]` is intrinsic to the
// PLY format. Same rationale as `sphere_eval` (sdf/stress-test) and
// `mesh-sdf-distance-query`.
#![allow(clippy::cast_possible_truncation)]
// Grid-axis coords read as the textbook `−half_extent + i · spacing`;
// the `mul_add` rewrite obscures intent and the result is
// bit-equivalent here. Same precedent as sphere_eval (sdf/stress-test).
#![allow(clippy::suboptimal_flops)]
// Grid axis indices are `0..17` so `usize as f64` is exactly
// representable (f64 mantissa is 52 bits; max index is 16).
#![allow(clippy::cast_precision_loss)]
// Cartesian coords spelt as `x, y, z` are reading-grade.
#![allow(clippy::many_single_char_names)]

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use cf_design::Sdf;
use mesh_io::{load_mesh, save_stl};
use mesh_sdf::{PseudoNormalSign, Signed, TriMeshDistance};
use mesh_types::{AttributedMesh, IndexedMesh, Point3, Vector3};

/// Local alias — the canonical parry-pseudo-normal composition.
/// [`PseudoNormalSign`] is the cheap parry pseudo-normal path; for
/// cleaned body-part scans prefer `mesh_sdf::flood_filled_sdf` per
/// `docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md`.
type ScanSdf = Signed<TriMeshDistance, PseudoNormalSign>;

// =============================================================================
// Constants
// =============================================================================

/// Cube half-extent along each axis. Dyadic so axis-aligned probes at
/// `(±R, 0, 0)` etc. land on the cube boundary at FP-exact coords.
const R: f64 = 1.0;

/// FP-exact comparison for face-region probes on dyadic coords —
/// closest-point-on-triangle reduces to a perpendicular drop, the
/// distance is the dyadic `|p_i| − R`, and the sign is determined by
/// the closest-face normal without further arithmetic.
const EXACT_TOL: f64 = 0.0;

/// Tolerance for edge- and vertex-region probes whose distances
/// involve `√(rational)` (e.g. `√0.5`, `√0.75`). Mesh-sdf computes
/// distance via `closest_point_on_triangle` (Ericson region case
/// resolution) plus a `norm()` call; the resulting FP error stays
/// well within `1e-12` for our well-conditioned cube.
const BAND_TOL: f64 = 1e-12;

/// Bulk-grid resolution per axis — 17 endpoint-inclusive points in
/// `[−2, +2]` at spacing 0.25 = 2⁻². The dyadic spacing keeps every
/// grid axis coordinate bit-exact in f64; non-dyadic alternatives
/// (e.g. 0.2) introduce 1-ULP slack that breaks closed-form
/// strict-interior counts at the boundary.
const GRID_RES: usize = 17;
const GRID_HALF_EXTENT: f64 = 2.0;
const GRID_SPACING: f64 = 0.25;
const GRID_TOTAL: usize = GRID_RES * GRID_RES * GRID_RES;

/// Closed-form count of grid points strictly inside the cube
/// (`|coord| ≤ R − GRID_SPACING = 0.75`). Per axis this is the set
/// `{−0.75, −0.5, −0.25, 0, 0.25, 0.5, 0.75}` = 7 values, all dyadic
/// and bit-exactly representable. `7³ = 343` total. Every point in
/// this set sits at least one grid step inside the cube along every
/// axis, so the F2 face-normal heuristic reports `eval < 0` reliably
/// across the whole set.
const STRICT_INTERIOR_COUNT: usize = 7 * 7 * 7;

/// Closed-form count of grid points inside the CLOSED cube `[−R, R]³`
/// (boundary inclusive). Per axis the set `|coord| ≤ R = 1.0` selects
/// the 9 dyadic grid values `{−1.0, −0.75, …, 0.75, 1.0}` (grid indices
/// 4..=12), all bit-exactly representable in f64; `9³ = 729` total. The
/// [`PseudoNormalSign`] oracle classifies exactly this closed set as
/// inside — the strict-interior bucket plus the boundary probes at
/// `|coord| == R` (unsigned distance 0, where the pseudo-normal returns
/// the inside half-space). So `raycast_inside` (the whole-grid
/// pseudo-normal inside-count) equals this geometric count structurally,
/// not as a captured empirical bit; the residual `eval < 0` ↔ `is_inside`
/// divergence is exactly the boundary shell `CLOSED_CUBE_COUNT −
/// STRICT_INTERIOR_COUNT = 729 − 343 = 386` (those faces have
/// `eval == 0`, so the strict-inequality heuristic excludes them while
/// the sign oracle includes them).
const CLOSED_CUBE_COUNT: usize = 9 * 9 * 9;

// =============================================================================
// Cube fixture
// =============================================================================

/// Build the 12-triangle axis-aligned cube fixture with half-extent
/// `R`. Vertex layout `(±R, ±R, ±R)` indexed in Gray-code-ish order so
/// face-grouping reads naturally:
///
/// ```text
/// 0: (−R, −R, −R)   4: (−R, −R, +R)
/// 1: (+R, −R, −R)   5: (+R, −R, +R)
/// 2: (+R, +R, −R)   6: (+R, +R, +R)
/// 3: (−R, +R, −R)   7: (−R, +R, +R)
/// ```
///
/// Each cube face = 2 triangles, wound CCW from outside so all 12
/// cross-product unit normals are `±x̂`, `±ŷ`, `±ẑ` (verified at
/// `verify_cube_geometry`).
fn build_cube_mesh(r: f64) -> IndexedMesh {
    let vertices: Vec<Point3<f64>> = vec![
        Point3::new(-r, -r, -r), //  0
        Point3::new(r, -r, -r),  //  1
        Point3::new(r, r, -r),   //  2
        Point3::new(-r, r, -r),  //  3
        Point3::new(-r, -r, r),  //  4
        Point3::new(r, -r, r),   //  5
        Point3::new(r, r, r),    //  6
        Point3::new(-r, r, r),   //  7
    ];
    // 12 outward-CCW triangles, grouped by cube face. Face groupings
    // and per-triangle normals are pinned in `verify_cube_geometry`.
    let faces: Vec<[u32; 3]> = vec![
        [0, 3, 2], // f0  −Z face, normal −ẑ
        [0, 2, 1], // f1  −Z face, normal −ẑ
        [4, 5, 6], // f2  +Z face, normal +ẑ
        [4, 6, 7], // f3  +Z face, normal +ẑ
        [0, 1, 5], // f4  −Y face, normal −ŷ
        [0, 5, 4], // f5  −Y face, normal −ŷ
        [3, 7, 6], // f6  +Y face, normal +ŷ
        [3, 6, 2], // f7  +Y face, normal +ŷ
        [0, 4, 7], // f8  −X face, normal −x̂
        [0, 7, 3], // f9  −X face, normal −x̂
        [1, 2, 6], // f10 +X face, normal +x̂
        [1, 6, 5], // f11 +X face, normal +x̂
    ];
    IndexedMesh::from_parts(vertices, faces)
}

/// Lock vertex coords + per-triangle outward unit normal at bit-exact
/// tolerance. Integer vertex coordinates make each cross product
/// produce a single nonzero component of magnitude `4·R²`, so the
/// normalized normal lands on `±x̂`, `±ŷ`, `±ẑ` exactly.
fn verify_cube_geometry(mesh: &IndexedMesh) {
    assert_eq!(mesh.vertices.len(), 8, "cube must have 8 vertices");
    assert_eq!(mesh.faces.len(), 12, "cube must have 12 triangles");

    let expected_v: [[f64; 3]; 8] = [
        [-R, -R, -R],
        [R, -R, -R],
        [R, R, -R],
        [-R, R, -R],
        [-R, -R, R],
        [R, -R, R],
        [R, R, R],
        [-R, R, R],
    ];
    for (i, expected) in expected_v.iter().enumerate() {
        let v = mesh.vertices[i];
        assert_relative_eq!(v.x, expected[0], epsilon = EXACT_TOL);
        assert_relative_eq!(v.y, expected[1], epsilon = EXACT_TOL);
        assert_relative_eq!(v.z, expected[2], epsilon = EXACT_TOL);
    }

    // 12 expected outward unit normals, one per triangle, matching
    // the face groupings above.
    let nx = Vector3::x();
    let ny = Vector3::y();
    let nz = Vector3::z();
    let expected_n: [Vector3<f64>; 12] = [
        -nz, -nz, // −Z face
        nz, nz, // +Z face
        -ny, -ny, // −Y face
        ny, ny, // +Y face
        -nx, -nx, // −X face
        nx, nx, // +X face
    ];
    for (i, expected) in expected_n.iter().enumerate() {
        let f = mesh.faces[i];
        let v0 = mesh.vertices[f[0] as usize];
        let v1 = mesh.vertices[f[1] as usize];
        let v2 = mesh.vertices[f[2] as usize];
        let n = (v1 - v0).cross(&(v2 - v0));
        let unit = n.normalize();
        assert_relative_eq!(unit, *expected, epsilon = EXACT_TOL);
    }
}

// =============================================================================
// Closed-form cube SDF
// =============================================================================

/// Closed-form signed distance for an axis-aligned cube `[−R, R]³`.
///
/// Standard box SDF: with `d_i = |p_i| − R`, the value is
/// `√(max(d_x, 0)² + max(d_y, 0)² + max(d_z, 0)²) +
/// min(max(d_x, d_y, d_z), 0)`. The first term is the Euclidean
/// distance from the closest point on the cube boundary to the probe
/// for exterior points (face / edge / vertex region collapses
/// automatically); the second is the negative of the closest-face
/// inset for strict-interior points.
fn analytical_cube_sdf(p: Point3<f64>) -> f64 {
    let d = Vector3::new(p.x.abs() - R, p.y.abs() - R, p.z.abs() - R);
    let exterior = (d.x.max(0.0).powi(2) + d.y.max(0.0).powi(2) + d.z.max(0.0).powi(2)).sqrt();
    let interior = d.x.max(d.y).max(d.z).min(0.0);
    exterior + interior
}

// =============================================================================
// verify_face_region_exterior — single |coord| > R
// =============================================================================

/// Six axis-aligned face-region probes at distance 0.5 outside one
/// face. Each probe lies on the perpendicular axis to a single cube
/// face, giving a clean closed-form `|p_i| − R = 0.5` SDF and
/// bit-exact arithmetic since all coords are dyadic.
fn verify_face_region_exterior(sdf: &dyn Sdf) {
    let probes = [
        (Point3::new(R + 0.5, 0.0, 0.0), 0.5),
        (Point3::new(-(R + 0.5), 0.0, 0.0), 0.5),
        (Point3::new(0.0, R + 0.5, 0.0), 0.5),
        (Point3::new(0.0, -(R + 0.5), 0.0), 0.5),
        (Point3::new(0.0, 0.0, R + 0.5), 0.5),
        (Point3::new(0.0, 0.0, -(R + 0.5)), 0.5),
    ];
    for (p, expected) in probes {
        assert_relative_eq!(sdf.eval(p), expected, epsilon = EXACT_TOL);
    }
}

// =============================================================================
// verify_edge_region_exterior — two |coord| > R
// =============================================================================

/// Three edge-region probes outside two cube faces simultaneously
/// (one per Cartesian-pair edge type). Closed-form
/// `√((|p_x| − R)² + (|p_y| − R)² + 0²) = √0.5 ≈ 0.7071` etc. Tolerance
/// `BAND_TOL` since `√0.5` in f64 is correctly-rounded but not exact.
fn verify_edge_region_exterior(sdf: &dyn Sdf) {
    let half = R + 0.5;
    let expected = 0.5_f64.sqrt();
    let probes = [
        Point3::new(half, half, 0.0),
        Point3::new(half, 0.0, half),
        Point3::new(0.0, half, half),
    ];
    for p in probes {
        assert_relative_eq!(sdf.eval(p), expected, epsilon = BAND_TOL);
    }
}

// =============================================================================
// verify_vertex_region_exterior — three |coord| > R
// =============================================================================

/// Eight vertex-region probes at the eight corners offset 0.5 beyond
/// each cube vertex. Closed-form `√(3 · 0.5²) = √0.75 ≈ 0.8660`,
/// `BAND_TOL` for the same reason as edge-region.
fn verify_vertex_region_exterior(sdf: &dyn Sdf) {
    let half = R + 0.5;
    let expected = 0.75_f64.sqrt();
    for sx in [1.0, -1.0_f64] {
        for sy in [1.0, -1.0_f64] {
            for sz in [1.0, -1.0_f64] {
                let p = Point3::new(sx * half, sy * half, sz * half);
                assert_relative_eq!(sdf.eval(p), expected, epsilon = BAND_TOL);
            }
        }
    }
}

// =============================================================================
// verify_interior — all |coord| < R
// =============================================================================

/// Five interior probes covering deep / near-surface / asymmetric
/// regions. SDF = `max(|p_x| − R, |p_y| − R, |p_z| − R)` for
/// strict-interior points; closest face is the one minimizing
/// `R − |p_i|`. All probe coords dyadic, all expected values dyadic,
/// `EXACT_TOL` throughout.
fn verify_interior(sdf: &dyn Sdf) {
    // Origin: equidistant from all 6 faces, SDF = 0 − R = −R.
    assert_relative_eq!(sdf.eval(Point3::origin()), -R, epsilon = EXACT_TOL);

    // Half-axis probes: one coord at 0.5, others 0; closest face is
    // the one along that axis, distance 0.5 inset → SDF = −0.5.
    for axis in [Vector3::x(), Vector3::y(), Vector3::z()] {
        let p_pt = Point3::from(0.5 * axis);
        assert_relative_eq!(sdf.eval(p_pt), -0.5, epsilon = EXACT_TOL);
    }

    // Asymmetric interior `(0.5, 0.25, −0.75)`: max d_i is at
    // |z| = 0.75 → d_z = −0.25, the smallest-magnitude inset, so the
    // closest face is +/−Z and SDF = −0.25. All coords dyadic; the
    // earlier draft used (0.5, 0.3, −0.7) but `1.0 − 0.7` is 1 ULP
    // off in f64 (0.7 not dyadic), breaking EXACT_TOL.
    assert_relative_eq!(
        sdf.eval(Point3::new(0.5, 0.25, -0.75)),
        -0.25,
        epsilon = EXACT_TOL,
    );

    // Near-surface interior `(0.75, 0, 0)`: closest face is +X,
    // distance 0.25 → SDF = −0.25. Dyadic to keep EXACT_TOL green
    // against mesh-sdf's `(point − closest).norm()` arithmetic.
    assert_relative_eq!(
        sdf.eval(Point3::new(0.75, 0.0, 0.0)),
        -0.25,
        epsilon = EXACT_TOL,
    );
}

// =============================================================================
// verify_grad_finite_and_outward_on_face_band — F2 caveat domain
// =============================================================================

/// On all six cube-face contact bands (within `~0.05 R` of `x = ±R`,
/// `y = ±R`, `z = ±R`), the central-difference gradient should
/// approximate the outward face normal `±x̂` / `±ŷ` / `±ẑ` with norm
/// near 1. Per F2's docstring, the gradient is a central
/// finite-difference approximation with `eps = 1e-6`, reliable on
/// face interiors (away from edges / vertices). We check magnitude
/// in `[0.9, 1.1]` and unit-direction alignment via `dot > 0.9`
/// (≈ 26° envelope) — face-region gradient is exact in real
/// arithmetic; truncation error stays well within the envelope.
fn verify_grad_finite_and_outward_on_face_band(sdf: &dyn Sdf) {
    let probes_with_normal = [
        (Point3::new(R + 0.05, 0.1, 0.2), Vector3::x()),
        (Point3::new(R - 0.05, 0.1, 0.2), Vector3::x()),
        (Point3::new(0.1, R + 0.05, 0.2), Vector3::y()),
        (Point3::new(0.1, 0.2, R + 0.05), Vector3::z()),
        (Point3::new(-(R + 0.05), 0.1, 0.2), -Vector3::x()),
        (Point3::new(0.1, -(R + 0.05), 0.2), -Vector3::y()),
        (Point3::new(0.1, 0.2, -(R + 0.05)), -Vector3::z()),
    ];
    for (p, expected_normal) in probes_with_normal {
        let g = sdf.grad(p);
        assert!(g.norm().is_finite(), "grad must be finite at {p:?}");
        assert!(
            (0.9..=1.1).contains(&g.norm()),
            "grad norm at {p:?} = {} outside [0.9, 1.1]",
            g.norm(),
        );
        // Gradient should align with the outward face normal at the
        // face band: dot product close to 1.
        assert!(
            g.dot(&expected_normal) > 0.9,
            "grad at {p:?} not aligned with {expected_normal:?}: dot = {}",
            g.dot(&expected_normal),
        );
    }
}

// =============================================================================
// verify_stl_round_trip — save → load preserves SDF
// =============================================================================

/// Write the cube to a binary STL file, load it back, and verify the
/// loaded mesh produces a `ScanSdf` composition that agrees with
/// the original at every named probe (closed-form face / interior
/// anchors). STL stores binary f32 vertex coords; integer `±R = ±1.0`
/// round-trips losslessly through f32, so the SDF values must be
/// bit-equivalent.
///
/// STL's lossy face-indexing (each triangle re-emits its 3 vertices
/// independently) means the loaded mesh has up to 36 vertices instead
/// of 8 — the SDF computation is unaffected since it only consumes
/// `mesh.faces` + `mesh.vertices` per face, not the index dedup. The
/// face count must match the original (12).
fn verify_stl_round_trip(mesh: &IndexedMesh, sdf: &ScanSdf, out_dir: &Path) -> Result<()> {
    let stl_path = out_dir.join("cube_scan.stl");
    save_stl(mesh, &stl_path, true)?;

    let loaded = load_mesh(&stl_path)?;
    assert_eq!(
        loaded.faces.len(),
        mesh.faces.len(),
        "STL round-trip must preserve face count",
    );

    let Ok(loaded_distance) = TriMeshDistance::new(loaded) else {
        unreachable!("loaded cube has 12 faces; new must succeed");
    };
    let loaded_sign = PseudoNormalSign::from_distance(&loaded_distance);
    let loaded_sdf: ScanSdf = Signed {
        distance: loaded_distance,
        sign: loaded_sign,
    };

    // Bit-exact agreement at face-region probes (dyadic distances).
    for axis in [Vector3::x(), Vector3::y(), Vector3::z()] {
        for sign in [1.0, -1.0_f64] {
            let p = Point3::from(sign * (R + 0.5) * axis);
            assert_relative_eq!(loaded_sdf.eval(p), sdf.eval(p), epsilon = EXACT_TOL);
        }
    }

    // Interior anchor.
    let origin = Point3::origin();
    assert_relative_eq!(
        loaded_sdf.eval(origin),
        sdf.eval(origin),
        epsilon = EXACT_TOL
    );

    Ok(())
}

// =============================================================================
// Bulk grid + PLY export
// =============================================================================

/// Per-grid-point sample: world-space position, signed distance via
/// the [`Sdf`] trait, and the pseudo-normal inside-test (the
/// authoritative inside/outside reference; the field name keeps the
/// historical `raycast` label — see the module docstring on the
/// [`PseudoNormalSign`] migration).
struct GridSample {
    p: Point3<f64>,
    eval: f64,
    inside_raycast: bool,
}

/// Membership in the CLOSED cube `[−R, R]³` (boundary inclusive). The
/// bulk-grid coordinates are dyadic (spacing `0.25 = 2⁻²`) and `R = 1.0`
/// is dyadic, so each `|coord| ≤ R` comparison is bit-exact — boundary
/// probes at `|coord| == R` land on grid indices 4 and 12 with no FP
/// slack. This is the closed-form oracle the [`PseudoNormalSign`]
/// inside-test is validated against (set-equality in
/// `verify_grid_consistency`).
fn is_closed_cube(g: &GridSample) -> bool {
    g.p.x.abs() <= R && g.p.y.abs() <= R && g.p.z.abs() <= R
}

/// Build the 17³ = 4913-point grid in `[−2, 2]³` at spacing 0.25.
/// Coordinates `i → −half_extent + i · spacing` are bit-exact in
/// f64 for every grid index because the spacing is dyadic
/// (`0.25 = 2⁻²`); ±R = ±1.0 lands exactly at axis indices 4 and
/// 12.
fn build_grid(sdf: &ScanSdf) -> Vec<GridSample> {
    let mut grid = Vec::with_capacity(GRID_TOTAL);
    for ix in 0..GRID_RES {
        let x = -GRID_HALF_EXTENT + (ix as f64) * GRID_SPACING;
        for iy in 0..GRID_RES {
            let y = -GRID_HALF_EXTENT + (iy as f64) * GRID_SPACING;
            for iz in 0..GRID_RES {
                let z = -GRID_HALF_EXTENT + (iz as f64) * GRID_SPACING;
                let p = Point3::new(x, y, z);
                grid.push(GridSample {
                    p,
                    eval: sdf.eval(p),
                    inside_raycast: sdf.is_inside(p),
                });
            }
        }
    }
    grid
}

/// Bulk-grid consistency over the SDF + raycast-inside fields:
///
/// 1. **Closed-form-vs-trait identity** — at every grid point,
///    `<ScanSdf as cf_design::Sdf>::eval(p)` equals
///    `analytical_cube_sdf(p)` within `BAND_TOL`. Both expressions
///    reduce to the same closest-point-on-triangle Euclidean
///    distance with sign from the closest-face normal, which agrees
///    with the analytical L∞-ball formula across the chosen grid.
///    This is the load-bearing trait-dispatch contract over a
///    representative bulk sample.
/// 2. **Heuristic strict-interior coverage** — every grid point with
///    `|coord| ≤ R − GRID_SPACING = 0.75` has `eval < 0`. The F2
///    face-normal sign heuristic is reliable on cube interiors; only
///    the far-field vertex / edge regions outside the cube are
///    documented to potentially flip sign (see F2 docstring at
///    `design/cf-design/src/sdf.rs:81-89`).
/// 3. **Pseudo-normal inside-set = closed cube** — the sign oracle
///    ([`PseudoNormalSign`], exposed as `is_inside`) classifies a grid
///    point as inside iff it lies in the CLOSED cube `[−R, R]³`
///    (`|coord| ≤ R` on every axis). Asserted as full set-equality over
///    the grid: `inside_raycast == is_closed_cube` at every point, with
///    the closed-cube bucket counted at `CLOSED_CUBE_COUNT = 9³ = 729`.
///    Dyadic spacing makes the `|coord| == R` boundary probes bit-exact,
///    so the boundary shell is unambiguously inside.
///
/// The `eval < 0` ↔ `is_inside` divergence printed in the museum plaque
/// is therefore exactly the boundary shell `CLOSED_CUBE_COUNT −
/// STRICT_INTERIOR_COUNT = 729 − 343 = 386`: those grid points sit on a
/// cube face (`unsigned_distance == 0` → `eval == 0`), so the
/// strict-inequality heuristic `eval < 0` excludes them while the sign
/// oracle includes them. Both counts are closed-form geometric identities
/// on this fixture, not captured empirical values. (The pre-parry `+X`
/// Möller-Trumbore ray-cast had an additional HE-1 face-diagonal
/// degeneracy that dropped the `y == z` interior probes; the
/// [`PseudoNormalSign`] path has no such degeneracy, which is why the
/// diagonal probes are now unconditionally inside. For a fixture where a
/// sign oracle genuinely fails, see `examples/mesh/mesh-sdf-distance-query`
/// — octahedron, vertex-region false-positives at the bbox boundary.)
///
/// Returns `(raycast_inside, heuristic_inside, divergence)` for the
/// museum-plaque summary.
fn verify_grid_consistency(grid: &[GridSample]) -> (usize, usize, usize) {
    assert_eq!(grid.len(), GRID_TOTAL);

    // (1) Closed-form-vs-trait identity.
    for g in grid {
        assert_relative_eq!(g.eval, analytical_cube_sdf(g.p), epsilon = BAND_TOL);
    }

    // Strict-interior subset filter.
    let inset = R - GRID_SPACING;
    let is_strict_interior =
        |g: &&GridSample| g.p.x.abs() <= inset && g.p.y.abs() <= inset && g.p.z.abs() <= inset;
    let strict_interior = grid.iter().filter(is_strict_interior).count();
    assert_eq!(
        strict_interior, STRICT_INTERIOR_COUNT,
        "strict-interior bucket must be 7³ = {STRICT_INTERIOR_COUNT}; got {strict_interior}",
    );

    // (2) Heuristic covers the strict-interior bucket.
    let strict_interior_all_heuristic =
        grid.iter().filter(is_strict_interior).all(|g| g.eval < 0.0);
    assert!(
        strict_interior_all_heuristic,
        "every strict-interior grid point must have eval < 0",
    );

    // (3) Pseudo-normal inside-set equals the closed cube [−R, R]³.
    // First pin the closed-cube bucket to its closed-form size 9³, then
    // assert the sign oracle's inside-classification matches closed-cube
    // membership at every grid point (set-equality, both directions).
    let closed_cube = grid.iter().filter(|g| is_closed_cube(g)).count();
    assert_eq!(
        closed_cube, CLOSED_CUBE_COUNT,
        "closed-cube bucket must be 9³ = {CLOSED_CUBE_COUNT}; got {closed_cube}",
    );
    for g in grid {
        assert_eq!(
            g.inside_raycast,
            is_closed_cube(g),
            "pseudo-normal is_inside at {:?} = {} disagrees with closed-cube \
             membership {} — sign-oracle boundary classification drifted",
            g.p,
            g.inside_raycast,
            is_closed_cube(g),
        );
    }

    // Counts.
    let raycast_inside = grid.iter().filter(|g| g.inside_raycast).count();
    let heuristic_inside = grid.iter().filter(|g| g.eval < 0.0).count();
    let divergence = grid
        .iter()
        .filter(|g| (g.eval < 0.0) != g.inside_raycast)
        .count();

    (raycast_inside, heuristic_inside, divergence)
}

/// Write the bulk grid as a vertices-only PLY with two per-vertex
/// scalars: `signed_distance` (analytic SDF via the [`Sdf`] trait,
/// divergent — negative interior, positive exterior) and
/// `inside_raycast` (binary 0 / 1, sequential — the authoritative
/// inside-test). cf-view's auto-detection picks divergent for
/// `signed_distance` and sequential for `inside_raycast`.
fn save_grid_ply(grid: &[GridSample], path: &Path) -> Result<()> {
    let vertices: Vec<Point3<f64>> = grid.iter().map(|g| g.p).collect();
    let faces: Vec<[u32; 3]> = Vec::new();
    let geometry = IndexedMesh::from_parts(vertices, faces);
    let mut attributed = AttributedMesh::new(geometry);
    let signed_distance: Vec<f32> = grid.iter().map(|g| g.eval as f32).collect();
    let inside_raycast: Vec<f32> = grid
        .iter()
        .map(|g| if g.inside_raycast { 1.0_f32 } else { 0.0_f32 })
        .collect();
    attributed.insert_extra("signed_distance", signed_distance)?;
    attributed.insert_extra("inside_raycast", inside_raycast)?;
    mesh_io::save_ply_attributed(&attributed, path, true)?;
    Ok(())
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

fn print_summary(
    raycast_inside: usize,
    heuristic_inside: usize,
    divergence: usize,
    cube_path: &Path,
    grid_path: &Path,
) {
    println!("==== mesh_scan ====");
    println!();
    println!("input  : programmatic 12-triangle cube (8v, 12f)");
    println!("         half-extent R = {R}; vertices at (±R, ±R, ±R)");
    println!("         routed: build → save_stl → load_mesh → ScanSdf composition");
    println!("         dispatched via cf_design::Sdf (the trait spine post-PR3 F1+F2)");
    println!();
    println!("Closed-form anchor groups (all assertions exit-0 on success):");
    println!("  cube_geometry              : 8 verts + 12 outward unit normals");
    println!("  face_region_exterior       : 6 axis-aligned, SDF = +0.5 bit-exact");
    println!("  edge_region_exterior       : 3 edge-direction, SDF = √0.5 within 1e-12");
    println!("  vertex_region_exterior     : 8 corner offsets, SDF = √0.75 within 1e-12");
    println!("  interior                   : origin + half-axis + asymmetric + near-surface");
    println!("  grad_finite_outward_on_face: 7 face-band probes, ‖grad‖ ∈ [0.9, 1.1]");
    println!();
    println!("STL round-trip:");
    println!("  save_stl + load_mesh + Signed composition → ");
    println!("  SDF agrees with in-memory at face-region + origin probes (bit-exact)");
    println!();
    println!(
        "Bulk grid {GRID_RES}³ = {GRID_TOTAL} points in [−{GRID_HALF_EXTENT}, +{GRID_HALF_EXTENT}]³ at spacing {GRID_SPACING}:"
    );
    println!("  closed-form-vs-trait id    : analytical_cube_sdf == eval at every point (1e-12)");
    println!(
        "  heuristic strict-interior  : {STRICT_INTERIOR_COUNT} pts (|coord| ≤ {0:.2}) all eval < 0",
        R - GRID_SPACING,
    );
    println!(
        "  pseudo-normal inside-set   : {CLOSED_CUBE_COUNT} pts = closed cube [−R, R]³ (9³); is_inside iff |coord| ≤ R",
    );
    println!(
        "  inside via sign oracle     : {raycast_inside:>5} (= 9³ closed cube; boundary-inclusive)"
    );
    println!(
        "  inside via heuristic < 0   : {heuristic_inside:>5} (= 7³ strict interior; face-normal sign)"
    );
    println!(
        "  boundary-shell divergence  : {divergence:>5} (= {CLOSED_CUBE_COUNT} − {STRICT_INTERIOR_COUNT}; faces have eval == 0 but is_inside)",
    );
    println!();
    println!("Artifacts:");
    println!("  STL  : {}", cube_path.display());
    println!("  PLY  : {}", grid_path.display());
    println!("         vertices-only point cloud + 2 per-vertex scalars:");
    println!(
        "           extras[\"signed_distance\"] — analytical SDF, divergent (interior < 0 < exterior)"
    );
    println!("           extras[\"inside_raycast\"]  — 0 / 1 categorical, sequential");
    println!("         open in cf-view, the workspace's unified visual-review viewer:");
    println!("           cargo run -p cf-viewer --release -- <path>");
    println!(
        "         default-picks inside_raycast (alphabetical first; binary mask of cube interior);"
    );
    println!(
        "         scalar dropdown switches to signed_distance (radial gradient — blue interior,"
    );
    println!("         red exterior, divergent bwr).");
}

// =============================================================================
// main
// =============================================================================

pub fn run() -> Result<()> {
    let mesh = build_cube_mesh(R);
    verify_cube_geometry(&mesh);

    let Ok(distance) = TriMeshDistance::new(mesh.clone()) else {
        unreachable!("cube has 12 faces; new must succeed");
    };
    let sign = PseudoNormalSign::from_distance(&distance);
    let sdf: ScanSdf = Signed { distance, sign };

    // Trait-dispatch contract: every closed-form anchor goes through
    // the cf_design::Sdf trait surface, not the inherent ScanSdf API.
    // This is the load-bearing F2 demonstration — a mesh-derived SDF
    // is a first-class Sdf primitive without per-mesh wrapper code.
    // Row 16 will exercise the parallel F3 path (sim_soft consumers
    // via the Sdf re-export); row 15 stays cf-design-side.
    let trait_sdf: &dyn Sdf = &sdf;
    verify_face_region_exterior(trait_sdf);
    verify_edge_region_exterior(trait_sdf);
    verify_vertex_region_exterior(trait_sdf);
    verify_interior(trait_sdf);
    verify_grad_finite_and_outward_on_face_band(trait_sdf);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;

    verify_stl_round_trip(&mesh, &sdf, &out_dir)?;

    let grid = build_grid(&sdf);
    let (raycast_inside, heuristic_inside, divergence) = verify_grid_consistency(&grid);

    // The F2-caveat-absent claim, pinned: every grid point with
    // `eval < 0` is a strict-interior probe — no false positives.
    assert_eq!(
        heuristic_inside, STRICT_INTERIOR_COUNT,
        "heuristic_inside count must equal STRICT_INTERIOR_COUNT \
         (any extra would be an F2-caveat false-positive on the cube fixture)",
    );

    // Pseudo-normal inside-count = the CLOSED cube [−R, R]³ grid-point
    // count, a closed-form geometric identity: 9 dyadic values per axis
    // in `|coord| ≤ R` (indices 4..=12) → 9³. `verify_grid_consistency`
    // already asserts the per-point set-equality; this pins the aggregate
    // count for the museum plaque.
    assert_eq!(
        raycast_inside, CLOSED_CUBE_COUNT,
        "pseudo-normal inside-count must equal the closed cube [−R, R]³ \
         grid-point count 9³ = {CLOSED_CUBE_COUNT}; drift ⇒ a PseudoNormalSign \
         boundary-classification change or an oracle swap",
    );
    // `divergence` (eval < 0 vs is_inside disagreement) is the boundary
    // shell: strict-interior points have eval < 0 AND is_inside, exterior
    // points have neither, and the CLOSED_CUBE − STRICT_INTERIOR face
    // shell has is_inside but eval == 0 (probes exactly on a cube face,
    // unsigned_distance == 0). So the count is the closed-form difference
    // 729 − 343 = 386, not an empirical capture.
    assert_eq!(
        divergence,
        CLOSED_CUBE_COUNT - STRICT_INTERIOR_COUNT,
        "eval-vs-is_inside divergence must equal the boundary shell \
         (closed − strict interior = {CLOSED_CUBE_COUNT} − {STRICT_INTERIOR_COUNT} \
         = {}); got {divergence}",
        CLOSED_CUBE_COUNT - STRICT_INTERIOR_COUNT,
    );

    let cube_path = out_dir.join("cube_scan.stl");
    let grid_path = out_dir.join("sdf_grid.ply");
    save_grid_ply(&grid, &grid_path)?;

    print_summary(
        raycast_inside,
        heuristic_inside,
        divergence,
        &cube_path,
        &grid_path,
    );

    Ok(())
}
