// `unreachable!()` calls below are diagnostic guards on `let-else`
// branches that cannot fire (fixtures are non-empty by construction →
// `TriMeshDistance::new` cannot return `Err(EmptyMesh)`). `xtask grade`
// counts un-justified `unreachable!()` macros; allow at file level since
// every call is a post-validation impossibility, not a real panic site.
#![allow(clippy::unreachable)]
// PLY field-data is single-precision on disk; converting f64 sdf values
// to f32 for `extras[...]` is intrinsic to the PLY format.
#![allow(clippy::cast_possible_truncation)]
// Cross-product unit-normal computation reads as the textbook formula
// `e1.y*e2.z - e1.z*e2.y`; the `mul_add` rewrite obscures intent and
// produces bit-equivalent results on integer vertex coordinates.
#![allow(clippy::suboptimal_flops)]
// `usize as f64` casts on grid indices (max GRID_SIZE = 10) for
// coordinate computation. Well within f64 mantissa exact range. Same
// allowance as the row 6/8/9/10/11 sim-soft examples.
#![allow(clippy::cast_precision_loss)]
// `main` orchestrates the example end-to-end: build fixtures, run
// both oracles, verify, save PLY, print summary. Each step is ~20-30
// LOC and isn't independently meaningful — splitting would obscure
// the demonstration narrative more than help.
#![allow(clippy::too_many_lines)]
// Programmatic fixtures (octahedron, inverted-cap pyramid) are
// non-empty by construction so `TriMeshDistance::new` cannot return
// `Err(EmptyMesh)`; expect is a diagnostic guard on a `Result::Err`
// impossibility, not a real panic site.
#![allow(clippy::expect_used)]

//! mesh-sdf-distance-query — oracle decomposition walkthrough.
//!
//! Post-D arc ([[project-mesh-sdf-oracle-decomposition-spec]],
//! `docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md`) the mesh-sdf API
//! decomposes signed-distance queries into two orthogonal oracles:
//! [`TriMeshDistance`] (parry BVH-backed unsigned distance) and a
//! `Sign` oracle ([`PseudoNormalSign`] or [`FloodFillSign`]),
//! composed via [`Signed<D, S>`]. This example walks the surface on
//! two contrasting fixtures:
//!
//! 1. **Well-formed octahedron** (CCW closed, 6 vertices, 8 faces) —
//!    both sign oracles agree everywhere. Demonstrates the basic API
//!    surface: composition, `.distance` / `.unsigned_distance` /
//!    `.is_inside` / `.closest_point` queries, bulk-grid evaluation.
//!
//! 2. **Pathological inverted-cap pyramid** (mimics the cap-fan
//!    failure mode root-caused at
//!    [[project-cf-cast-plug-layer-0-watertight-discovery]]) — the base
//!    winding is flipped so its outward normal points INTO the body.
//!    [`PseudoNormalSign`] produces a wrong-sign at the far-field
//!    `-Z` probe under the pyramid; [`FloodFillSign`] correctly handles
//!    it via topological reachability from the bounds corners. This
//!    is the load-bearing demonstration of the D arc — why
//!    `cf-cast-cli` and `cf-device-design` ship flood-fill sign
//!    defense on cleaned body-part scans.
//!
//! Output: `out/octahedron.ply` (fixture mesh) +
//! `out/octahedron_sdf_grid.ply` (per-grid-point signed distance +
//! sign-oracle agreement flags, for cf-view inspection).

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use cf_geometry::Aabb;
use mesh_io::{save_ply, save_ply_attributed};
use mesh_sdf::{
    FloodFillSign, PseudoNormalSign, Signed, TriMeshDistance, WALL_THRESHOLD_FACTOR_DEFAULT,
    flood_filled_sdf,
};
use mesh_types::{AttributedMesh, IndexedMesh, Point3};

// =============================================================================
// Constants
// =============================================================================

/// FP-exact integer octahedron coords + analytical `1/√3` normals hit
/// `1e-12`. SDF queries go through parry3d which uses f32 internally
/// (per [[project-mesh-sdf-parry-accel-spec]]), so distance assertions
/// relax to 1e-6 — well outside f32 roundoff, well inside any practical
/// surface-locating tolerance.
const VERTEX_TOL: f64 = 1e-12;
const NORMAL_TOL: f64 = 1e-12;
const DISTANCE_TOL: f64 = 1e-6;

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
/// is `10/9 > 1`. Both oracles agree on a well-formed octahedron.
const EXPECTED_INSIDE_COUNT: usize = 8;

/// Flood-fill cell size on the octahedron grid (50 mm-ish at the
/// chosen unit scale). Non-divisor of the octahedron's 1.0 radius so
/// no lattice node lands EXACTLY on the surface — see the
/// "`cell_size` + surface-aligned-lattice" pattern banked at
/// [[project-mesh-sdf-oracle-decomposition-spec]] D.5 patterns.
const FLOOD_FILL_CELL_SIZE: f64 = 0.0473;

// =============================================================================
// Fixture: well-formed octahedron
// =============================================================================

/// Build a unit octahedron with vertices at `(±r, 0, 0)`, `(0, ±r, 0)`,
/// `(0, 0, ±r)` and 8 triangle faces (one per `(sx, sy, sz)` octant).
///
/// Face winding produces outward-facing normals on a CCW-from-outside
/// convention. The 4 octants where `sx*sy*sz = +1` use the standard
/// winding; the 4 where `sx*sy*sz = -1` use the flipped winding so all
/// 8 cross-product normals come out as `(sx, sy, sz) / √3`.
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

/// Lock 6 vertex coords + 8 face-winding cross-product unit normals.
/// Integer vertex coords make the cross products bit-exact at 1e-12.
fn verify_octahedron_geometry(mesh: &IndexedMesh, r: f64) {
    assert_eq!(mesh.vertices.len(), OCTAHEDRON_VERT_COUNT);
    assert_eq!(mesh.faces.len(), OCTAHEDRON_FACE_COUNT);

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

    let inv_sqrt3 = 1.0 / 3.0_f64.sqrt();
    let expected_n: [[f64; 3]; OCTAHEDRON_FACE_COUNT] = [
        [inv_sqrt3, inv_sqrt3, inv_sqrt3],
        [inv_sqrt3, -inv_sqrt3, -inv_sqrt3],
        [-inv_sqrt3, inv_sqrt3, -inv_sqrt3],
        [-inv_sqrt3, -inv_sqrt3, inv_sqrt3],
        [inv_sqrt3, inv_sqrt3, -inv_sqrt3],
        [inv_sqrt3, -inv_sqrt3, inv_sqrt3],
        [-inv_sqrt3, inv_sqrt3, inv_sqrt3],
        [-inv_sqrt3, -inv_sqrt3, -inv_sqrt3],
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
        let unit = [n[0] / len, n[1] / len, n[2] / len];
        assert_relative_eq!(unit[0], expected[0], epsilon = NORMAL_TOL);
        assert_relative_eq!(unit[1], expected[1], epsilon = NORMAL_TOL);
        assert_relative_eq!(unit[2], expected[2], epsilon = NORMAL_TOL);
    }
}

// =============================================================================
// Fixture: pathological inverted-cap pyramid
// =============================================================================

/// Square-base pyramid `[(±1, ±1, 0), (0, 0, 1)]` with the BASE winding
/// INVERTED so the base's parry pseudo-normal points +Z (INTO the body)
/// instead of -Z (out of it). Mimics the pre-B cf-scan-prep auto-cap
/// failure mode where `auto_cap_open_boundaries` emitted inward-pointing
/// cap-fan triangles — see [[project-cf-scan-prep-cap-winding-concern]].
///
/// A `PseudoNormalSign` query at `(0, 0, -3)` finds the base as the
/// closest face, asks `(probe − closest) · normal`, gets
/// `(0, 0, -3) · (0, 0, +1) = -3 < 0`, and reports "inside" — the
/// canonical failure mode this fixture exists to demonstrate.
fn build_inverted_cap_pyramid() -> IndexedMesh {
    let vertices: Vec<Point3<f64>> = vec![
        Point3::new(-1.0, -1.0, 0.0), // 0
        Point3::new(1.0, -1.0, 0.0),  // 1
        Point3::new(1.0, 1.0, 0.0),   // 2
        Point3::new(-1.0, 1.0, 0.0),  // 3
        Point3::new(0.0, 0.0, 1.0),   // 4 apex
    ];
    let faces: Vec<[u32; 3]> = vec![
        // Base — INVERTED winding (normals point +Z into body, the bug).
        [0, 1, 2],
        [0, 2, 3],
        // Sides — correct outward CCW.
        [0, 1, 4],
        [1, 2, 4],
        [2, 3, 4],
        [3, 0, 4],
    ];
    IndexedMesh::from_parts(vertices, faces)
}

// =============================================================================
// Section 1 — Oracle composition walkthrough on the octahedron
// =============================================================================

/// 8 analytical anchor queries covering interior face regions, vertex
/// directions, and far-field. Pins [`TriMeshDistance`] +
/// [`PseudoNormalSign`] composition against the closed-form L1-ball
/// SDF `(|x| + |y| + |z| − 1) / √3` for face-region queries.
fn verify_octahedron_signed_pseudo_normal(
    sdf: &Signed<TriMeshDistance, PseudoNormalSign>,
) -> SignedSummary {
    let inv_sqrt3 = 1.0 / 3.0_f64.sqrt();
    let third = 1.0 / 3.0;

    // Generic interior `(0.05, 0.07, 0.11)`. L1-norm 0.23 → SDF
    // `(0.23 − 1) / √3 = -0.77 / √3` (negative inside).
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
        epsilon = DISTANCE_TOL
    );
    assert!(sdf.is_inside(generic));
    let cp = sdf.closest_point(generic);
    assert_relative_eq!(cp.x, 0.05 + 0.77 / 3.0, epsilon = DISTANCE_TOL);
    assert_relative_eq!(cp.y, 0.07 + 0.77 / 3.0, epsilon = DISTANCE_TOL);
    assert_relative_eq!(cp.z, 0.11 + 0.77 / 3.0, epsilon = DISTANCE_TOL);

    // 6 face-center direction interior queries — analytical anchor.
    let face_center_signed = -0.1 * inv_sqrt3;
    let face_cases: [(f64, f64, f64); 6] = [
        (1.0, 1.0, 1.0),
        (1.0, -1.0, -1.0),
        (-1.0, 1.0, -1.0),
        (-1.0, -1.0, 1.0),
        (1.0, 1.0, -1.0),
        (-1.0, -1.0, -1.0),
    ];
    for (sx, sy, sz) in face_cases {
        let q = Point3::new(0.3 * sx, 0.3 * sy, 0.3 * sz);
        assert_relative_eq!(sdf.distance(q), face_center_signed, epsilon = DISTANCE_TOL);
        assert!(sdf.is_inside(q));
        let cp = sdf.closest_point(q);
        assert_relative_eq!(cp.x, sx * third, epsilon = DISTANCE_TOL);
        assert_relative_eq!(cp.y, sy * third, epsilon = DISTANCE_TOL);
        assert_relative_eq!(cp.z, sz * third, epsilon = DISTANCE_TOL);
    }

    // 1 vertex-direction exterior probe — clamps to +X vertex.
    let vertex_q = Point3::new(2.0, 0.0, 0.0);
    let vertex_signed = 1.0;
    assert_relative_eq!(
        sdf.distance(vertex_q),
        vertex_signed,
        epsilon = DISTANCE_TOL
    );
    assert!(!sdf.is_inside(vertex_q));

    SignedSummary {
        generic_signed: sdf.distance(generic),
        face_center_signed: sdf.distance(Point3::new(0.3, 0.3, 0.3)),
        vertex_unsigned: sdf.unsigned_distance(vertex_q),
    }
}

/// Same anchor queries via [`FloodFillSign`] composition. On a
/// well-formed octahedron both sign oracles must agree.
fn verify_octahedron_signed_flood_fill(sdf: &Signed<TriMeshDistance, FloodFillSign>) {
    let inv_sqrt3 = 1.0 / 3.0_f64.sqrt();

    // Generic interior — unsigned distance is BVH-exact, sign comes
    // from flood-fill grid lookup. Both must match the pseudo-normal
    // result on this clean CCW fixture.
    let generic = Point3::new(0.05, 0.07, 0.11);
    let expected_signed = -0.77 * inv_sqrt3;
    assert_relative_eq!(
        sdf.distance(generic),
        expected_signed,
        epsilon = DISTANCE_TOL
    );
    assert!(sdf.is_inside(generic));

    let vertex_q = Point3::new(2.0, 0.0, 0.0);
    assert!(!sdf.is_inside(vertex_q));
    assert_relative_eq!(sdf.unsigned_distance(vertex_q), 1.0, epsilon = DISTANCE_TOL);
}

#[derive(Debug)]
struct SignedSummary {
    generic_signed: f64,
    face_center_signed: f64,
    vertex_unsigned: f64,
}

// =============================================================================
// Section 2 — Bulk-grid query (both oracles must agree on octahedron)
// =============================================================================

#[derive(Debug, Clone, Copy)]
struct GridSample {
    p: Point3<f64>,
    pseudo_normal_signed: f64,
    flood_fill_signed: f64,
    pseudo_normal_inside: bool,
    flood_fill_inside: bool,
}

fn build_grid(
    pseudo_normal_sdf: &Signed<TriMeshDistance, PseudoNormalSign>,
    flood_fill_sdf: &Signed<TriMeshDistance, FloodFillSign>,
) -> Vec<GridSample> {
    let step = (GRID_MAX - GRID_MIN) / (GRID_SIZE - 1) as f64;
    let mut out = Vec::with_capacity(GRID_TOTAL);
    for iz in 0..GRID_SIZE {
        for iy in 0..GRID_SIZE {
            for ix in 0..GRID_SIZE {
                let p = Point3::new(
                    GRID_MIN + ix as f64 * step,
                    GRID_MIN + iy as f64 * step,
                    GRID_MIN + iz as f64 * step,
                );
                out.push(GridSample {
                    p,
                    pseudo_normal_signed: pseudo_normal_sdf.distance(p),
                    flood_fill_signed: flood_fill_sdf.distance(p),
                    pseudo_normal_inside: pseudo_normal_sdf.is_inside(p),
                    flood_fill_inside: flood_fill_sdf.is_inside(p),
                });
            }
        }
    }
    out
}

// =============================================================================
// Section 3 — Pathological fixture comparison (the load-bearing demo)
// =============================================================================

/// Probe the inverted-cap pyramid at four points: one interior, one
/// far-field below the inverted base (where [`PseudoNormalSign`]
/// produces the wrong sign), one far-field above the apex, and one to
/// the side.
///
/// Asserts the canonical D arc claim: [`PseudoNormalSign`] produces a
/// wrong-sign far-field below the inverted base; [`FloodFillSign`]
/// produces correct outside signs from topological reachability.
fn compare_oracles_on_inverted_cap(
    pseudo_normal_sdf: &Signed<TriMeshDistance, PseudoNormalSign>,
    flood_fill_sdf: &Signed<TriMeshDistance, FloodFillSign>,
) -> OracleComparison {
    let probes = [
        // Interior — apex at (0, 0, 1), base at z=0; (0, 0, 0.5) is
        // inside the pyramid. Both oracles should agree.
        ("interior_apex_axis", Point3::new(0.0, 0.0, 0.5), true),
        // Far-field BELOW the inverted base (3 units below). The base's
        // flipped normal points +Z (into the body); pseudo-normal sign
        // computes `(probe − base_centroid) · (+Z) = -3 < 0` → claims
        // INSIDE. FloodFillSign correctly reports OUTSIDE because the
        // probe is reachable from the bounds corners.
        (
            "far_field_below_inverted_base",
            Point3::new(0.0, 0.0, -3.0),
            false,
        ),
        // Far-field above the apex — both oracles agree on outside.
        ("far_field_above_apex", Point3::new(0.0, 0.0, 3.0), false),
        // Far-field beside the body — both oracles agree on outside.
        ("far_field_lateral", Point3::new(3.0, 0.0, 0.5), false),
    ];

    let mut pseudo_normal_disagreements = 0usize;
    let mut flood_fill_disagreements = 0usize;
    let mut probe_records = Vec::with_capacity(probes.len());

    for (label, p, expected_inside) in probes {
        let pn_inside = pseudo_normal_sdf.is_inside(p);
        let ff_inside = flood_fill_sdf.is_inside(p);
        if pn_inside != expected_inside {
            pseudo_normal_disagreements += 1;
        }
        if ff_inside != expected_inside {
            flood_fill_disagreements += 1;
        }
        probe_records.push(ProbeRecord {
            label,
            p,
            expected_inside,
            pseudo_normal_inside: pn_inside,
            flood_fill_inside: ff_inside,
        });
    }

    // The load-bearing assertion: PseudoNormalSign is WRONG on the
    // far-field-below-inverted-base probe (1 disagreement expected),
    // FloodFillSign is right everywhere (0 disagreements expected).
    assert_eq!(
        pseudo_normal_disagreements, 1,
        "PseudoNormalSign should produce exactly 1 wrong sign on the inverted-cap fixture (far-field below inverted base)",
    );
    assert_eq!(
        flood_fill_disagreements, 0,
        "FloodFillSign should produce correct signs everywhere on the inverted-cap fixture",
    );

    OracleComparison {
        probe_records,
        pseudo_normal_disagreements,
        flood_fill_disagreements,
    }
}

#[derive(Debug)]
struct ProbeRecord {
    label: &'static str,
    p: Point3<f64>,
    expected_inside: bool,
    pseudo_normal_inside: bool,
    flood_fill_inside: bool,
}

#[derive(Debug)]
struct OracleComparison {
    probe_records: Vec<ProbeRecord>,
    pseudo_normal_disagreements: usize,
    flood_fill_disagreements: usize,
}

// =============================================================================
// PLY output for cf-view inspection
// =============================================================================

/// Save the fixture mesh plus a 10³ grid with per-point scalars
/// (signed distance via both oracles, plus a disagreement flag).
///
/// cf-view auto-detects PLY sequences; the dropdown switches between
/// `pseudo_normal_signed` (sequential blue→red diverging), `flood_fill_signed`
/// (same colormap), and `oracle_disagreement` (categorical 0/1).
///
/// On the well-formed octahedron the disagreement field should be
/// uniformly 0 — both oracles agree everywhere.
fn save_grid_ply(samples: &[GridSample], path: &Path) -> Result<()> {
    let mut grid_mesh = IndexedMesh::new();
    for s in samples {
        grid_mesh.vertices.push(s.p);
    }
    let pseudo_normal_signed: Vec<f32> = samples
        .iter()
        .map(|s| s.pseudo_normal_signed as f32)
        .collect();
    let flood_fill_signed: Vec<f32> = samples.iter().map(|s| s.flood_fill_signed as f32).collect();
    let oracle_disagreement: Vec<f32> = samples
        .iter()
        .map(|s| {
            if s.pseudo_normal_inside == s.flood_fill_inside {
                0.0
            } else {
                1.0
            }
        })
        .collect();

    let mut attributed = AttributedMesh::from(grid_mesh);
    attributed.insert_extra("pseudo_normal_signed", pseudo_normal_signed)?;
    attributed.insert_extra("flood_fill_signed", flood_fill_signed)?;
    attributed.insert_extra("oracle_disagreement", oracle_disagreement)?;
    save_ply_attributed(&attributed, path, /* binary = */ false)?;
    Ok(())
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;

    // ── Section 0: build + verify octahedron geometry ──────────────────
    let octahedron = build_octahedron(OCTAHEDRON_RADIUS);
    verify_octahedron_geometry(&octahedron, OCTAHEDRON_RADIUS);
    save_ply(
        &octahedron,
        out_dir.join("octahedron.ply"),
        /* binary = */ false,
    )?;

    // ── Section 1: oracle composition walkthrough on the octahedron ───
    //
    // Recommended construction pattern for Signed<TriMeshDistance,
    // PseudoNormalSign>: build TriMeshDistance once, then share its
    // Arc<TriMesh> with PseudoNormalSign via `from_distance` — one BVH
    // allocation feeds both the distance and the pseudo-normal sign
    // queries inside this composition. (Section 2's `flood_filled_sdf`
    // convenience helper rebuilds a separate TriMeshDistance internally,
    // so the example as a whole owns two BVHs — one per Signed.)
    let octa_distance =
        TriMeshDistance::new(octahedron.clone()).expect("octahedron is non-empty by construction");
    let octa_pn_sign = PseudoNormalSign::from_distance(&octa_distance);
    let octa_pn_sdf = Signed {
        distance: octa_distance,
        sign: octa_pn_sign,
    };
    let summary = verify_octahedron_signed_pseudo_normal(&octa_pn_sdf);

    // ── Section 2: flood-fill composition + agreement on octahedron ───
    //
    // `flood_filled_sdf` is the convenience constructor: build BVH once,
    // build flood-fill grid once, return the composed Signed.
    let octa_bounds = Aabb::new(
        Point3::new(GRID_MIN, GRID_MIN, GRID_MIN),
        Point3::new(GRID_MAX, GRID_MAX, GRID_MAX),
    );
    let (octa_ff_sdf, octa_ff_report) = flood_filled_sdf(
        octahedron,
        octa_bounds,
        FLOOD_FILL_CELL_SIZE,
        WALL_THRESHOLD_FACTOR_DEFAULT,
    )?;
    assert_eq!(
        octa_ff_report.inside_components, 1,
        "octahedron flood-fill must produce 1 connected inside region",
    );
    verify_octahedron_signed_flood_fill(&octa_ff_sdf);

    // ── Bulk-grid query: both oracles must agree on the octahedron ────
    let samples = build_grid(&octa_pn_sdf, &octa_ff_sdf);
    assert_eq!(samples.len(), GRID_TOTAL);
    let pn_inside_count = samples.iter().filter(|s| s.pseudo_normal_inside).count();
    let ff_inside_count = samples.iter().filter(|s| s.flood_fill_inside).count();
    assert_eq!(
        pn_inside_count, EXPECTED_INSIDE_COUNT,
        "PseudoNormalSign inside-count on octahedron must be {EXPECTED_INSIDE_COUNT}",
    );
    assert_eq!(
        ff_inside_count, EXPECTED_INSIDE_COUNT,
        "FloodFillSign inside-count on octahedron must be {EXPECTED_INSIDE_COUNT}",
    );
    let disagreement_count = samples
        .iter()
        .filter(|s| s.pseudo_normal_inside != s.flood_fill_inside)
        .count();
    assert_eq!(
        disagreement_count, 0,
        "well-formed octahedron: both sign oracles must agree on every grid point",
    );
    save_grid_ply(&samples, &out_dir.join("octahedron_sdf_grid.ply"))?;

    // ── Section 3: pathological inverted-cap pyramid — load-bearing ───
    //
    // The base winding is flipped so its outward normal points +Z (INTO
    // the body). PseudoNormalSign reads the base's pseudo-normal as +Z
    // and reports far-field probes below the base as INSIDE. FloodFillSign
    // labels by topological reachability from the bounds corners and
    // correctly reports them OUTSIDE.
    let inverted_pyramid = build_inverted_cap_pyramid();
    let inv_distance =
        TriMeshDistance::new(inverted_pyramid.clone()).expect("pyramid is non-empty");
    let inv_pn_sign = PseudoNormalSign::from_distance(&inv_distance);
    let inv_pn_sdf = Signed {
        distance: inv_distance,
        sign: inv_pn_sign,
    };
    let inv_bounds = Aabb::new(Point3::new(-4.0, -4.0, -4.0), Point3::new(4.0, 4.0, 4.0));
    let (inv_ff_sdf, inv_ff_report) = flood_filled_sdf(
        inverted_pyramid,
        inv_bounds,
        0.0743,
        WALL_THRESHOLD_FACTOR_DEFAULT,
    )?;
    assert_eq!(
        inv_ff_report.inside_components, 1,
        "inverted-cap pyramid flood-fill must produce 1 connected inside region",
    );
    let comparison = compare_oracles_on_inverted_cap(&inv_pn_sdf, &inv_ff_sdf);

    // ── stdout summary ────────────────────────────────────────────────
    println!("mesh-sdf-distance-query — oracle decomposition walkthrough");
    println!();
    println!(
        "Fixture A: well-formed octahedron ({OCTAHEDRON_VERT_COUNT} verts / {OCTAHEDRON_FACE_COUNT} faces)"
    );
    println!("  Signed<TriMeshDistance, PseudoNormalSign>:");
    println!(
        "    distance(0.05, 0.07, 0.11) = {:.9}",
        summary.generic_signed
    );
    println!(
        "    distance(0.3, 0.3, 0.3)    = {:.9}",
        summary.face_center_signed
    );
    println!(
        "    unsigned_distance(2,0,0)   = {:.9}",
        summary.vertex_unsigned
    );
    println!("  Bulk-grid (10³ probes in [-2, 2]³):");
    println!("    PseudoNormalSign inside-count: {pn_inside_count} / {GRID_TOTAL}");
    println!("    FloodFillSign    inside-count: {ff_inside_count} / {GRID_TOTAL}");
    println!("    oracle disagreements: {disagreement_count}");
    println!(
        "    flood-fill inside_components: {}",
        octa_ff_report.inside_components
    );
    println!();
    println!("Fixture B: inverted-cap pyramid (mimics cf-scan-prep cap-fan failure mode)");
    for r in &comparison.probe_records {
        let pn = if r.pseudo_normal_inside {
            "INSIDE"
        } else {
            "outside"
        };
        let ff = if r.flood_fill_inside {
            "INSIDE"
        } else {
            "outside"
        };
        let expected = if r.expected_inside {
            "INSIDE"
        } else {
            "outside"
        };
        let pn_status = if r.pseudo_normal_inside == r.expected_inside {
            "✓"
        } else {
            "✗"
        };
        let ff_status = if r.flood_fill_inside == r.expected_inside {
            "✓"
        } else {
            "✗"
        };
        println!("  probe={:30} at {:?}", r.label, r.p);
        println!(
            "    expected: {expected} | PseudoNormal: {pn} {pn_status} | FloodFill: {ff} {ff_status}",
        );
    }
    println!(
        "  PseudoNormalSign disagreements: {}",
        comparison.pseudo_normal_disagreements
    );
    println!(
        "  FloodFillSign    disagreements: {}",
        comparison.flood_fill_disagreements
    );
    println!();
    println!("Output: {}", out_dir.display());
    println!("  octahedron.ply             — fixture mesh");
    println!("  octahedron_sdf_grid.ply    — 1000 grid points + signed distances + agreement flag");
    println!();
    println!(
        "Open in cf-view: `cargo run --release -p cf-viewer -- {}/octahedron_sdf_grid.ply`",
        out_dir.display()
    );

    Ok(())
}
