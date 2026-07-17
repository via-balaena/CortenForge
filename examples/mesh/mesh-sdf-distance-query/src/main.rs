// PLY field-data is single-precision on disk; converting f64 sdf values
// to f32 for `extras[...]` is intrinsic to the PLY format.
#![allow(clippy::cast_possible_truncation)]
// `usize as f64` casts on grid indices (max GRID_SIZE = 10) for
// coordinate computation. Well within f64 mantissa exact range. Same
// allowance as the row 6/8/9/10/11 sim-soft examples.
#![allow(clippy::cast_precision_loss)]
// `main` orchestrates the example end-to-end: build fixtures, run
// both oracles, save PLY, print summary. Each step is ~20-30 LOC and
// isn't independently meaningful — splitting would obscure the
// demonstration narrative more than help.
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

const OCTAHEDRON_VERT_COUNT: usize = 6;
const OCTAHEDRON_FACE_COUNT: usize = 8;
const OCTAHEDRON_RADIUS: f64 = 1.0;

/// Bulk-query grid: 10×10×10 = 1000 points in `[-2, 2]³`
/// endpoint-inclusive (spacing `4/9`).
const GRID_SIZE: usize = 10;
const GRID_TOTAL: usize = GRID_SIZE * GRID_SIZE * GRID_SIZE;
const GRID_MIN: f64 = -2.0;
const GRID_MAX: f64 = 2.0;

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

/// Sample the octahedron's `Signed<TriMeshDistance, PseudoNormalSign>`
/// composition at three representative probes for the printed summary: a
/// generic interior point, a face-center direction, and a vertex-direction
/// exterior probe. The closed-form L1-ball values these reproduce —
/// `(|x| + |y| + |z| − 1) / √3`, including `closest_point` and `is_inside`
/// over generic + 6 face-center + vertex probes — are asserted in
/// `mesh-sdf`'s `signed_distance_matches_closed_form_l1_ball_on_octahedron`
/// lib test; this walkthrough reads the values back for display.
fn octahedron_pseudo_normal_summary(
    sdf: &Signed<TriMeshDistance, PseudoNormalSign>,
) -> SignedSummary {
    let generic = Point3::new(0.05, 0.07, 0.11);
    let vertex_q = Point3::new(2.0, 0.0, 0.0);
    SignedSummary {
        generic_signed: sdf.distance(generic),
        face_center_signed: sdf.distance(Point3::new(0.3, 0.3, 0.3)),
        vertex_unsigned: sdf.unsigned_distance(vertex_q),
    }
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
/// the side. Returns the per-probe comparison for the printout.
///
/// Demonstrates the canonical D arc contrast: [`PseudoNormalSign`]
/// produces a wrong-sign far-field below the inverted base;
/// [`FloodFillSign`] produces correct outside signs from topological
/// reachability. The claim is asserted in `mesh-sdf`'s `flood_fill.rs`
/// `pseudo_normal_wrong_flood_fill_right_on_inward_cap` lib test.
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

    // The load-bearing behavior this demonstrates — PseudoNormalSign
    // produces exactly 1 wrong sign (far-field below the inverted base)
    // while FloodFillSign is correct everywhere — is asserted in
    // `mesh-sdf`'s `flood_fill.rs`
    // `pseudo_normal_wrong_flood_fill_right_on_inward_cap` lib test (same
    // inverted-base fixture, same probe direction). Here we surface the
    // per-probe comparison for the printout.
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

    // ── Section 0: build octahedron fixture ────────────────────────────
    let octahedron = build_octahedron(OCTAHEDRON_RADIUS);
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
    let summary = octahedron_pseudo_normal_summary(&octa_pn_sdf);

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

    // ── Bulk-grid query: both oracles agree on the octahedron ─────────
    // The counts below are read back for the printout; the underlying
    // contracts (flood-fill produces one inside component on a clean
    // mesh, and both sign oracles agree on a well-formed mesh) are
    // asserted in `mesh-sdf`'s `flood_fill.rs` lib tests
    // (`flood_fill_sign_correct_on_closed_pyramid`,
    // `flood_fill_matches_pseudo_normal_on_outward_cap`).
    let samples = build_grid(&octa_pn_sdf, &octa_ff_sdf);
    let pn_inside_count = samples.iter().filter(|s| s.pseudo_normal_inside).count();
    let ff_inside_count = samples.iter().filter(|s| s.flood_fill_inside).count();
    let disagreement_count = samples
        .iter()
        .filter(|s| s.pseudo_normal_inside != s.flood_fill_inside)
        .count();
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
    let (inv_ff_sdf, _inv_ff_report) = flood_filled_sdf(
        inverted_pyramid,
        inv_bounds,
        0.0743,
        WALL_THRESHOLD_FACTOR_DEFAULT,
    )?;
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
