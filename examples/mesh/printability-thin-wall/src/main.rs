//! Visual demo of the §6.1 `ThinWall` detector (Gap C).
//!
//! Hand-authors a 24-triangle, double-walled hollow box whose top wall
//! has been deliberately thinned to 0.4 mm — well under the 1.0 mm FDM
//! `min_wall_thickness` threshold. The detector flags two clusters
//! (outer top + inner top) as Critical `ThinWall`, the cavity ceiling
//! flags as a Critical Overhang co-flag (its inward-facing normal
//! points down into the cavity), and `is_printable()` returns false.
//! Saves `out/mesh.ply` (the input fixture) and `out/issues.ply` (a
//! point-cloud of cluster + region centroids) for the visuals pass.
//!
//! ## Why this fixture
//!
//! `ThinWall` detection requires watertight + consistently-wound input
//! (the inward ray must hit a real opposite face). A simple
//! single-walled thin slab is open — `check_thin_walls` returns
//! `DetectorSkipped` Info instead of running. The double-walled
//! construction is what actually exercises the detector.
//!
//! ## Why two disjoint shells (no shared vertices)
//!
//! The cluster contract — flagged faces grouped by **edge-adjacency**
//! via `build_edge_to_faces` — relies on each edge appearing in
//! exactly two faces *within* a watertight shell. Splitting the shells
//! into vertex-disjoint sets keeps the outer and inner top faces in
//! separate clusters even though they sit at nearly the same z; that's
//! the load-bearing topological prediction (§7.1 of the v0.8 fix arc
//! spec) the example anchors.
//!
//! ## Inner-shell winding rule
//!
//! Each face's normal must point AWAY from the surrounding solid
//! material:
//!
//! - Outer cube faces: normals point OUTWARD (CCW-from-outside).
//! - Inner cavity faces: normals point INTO the cavity
//!   (CCW-from-cavity-side, i.e. REVERSED relative to a standalone
//!   outward-wound small box).
//!
//! Both shells together pass the watertight + consistent-winding
//! preconditions §6.1 requires.
//!
//! ## Numerical anchors (asserted in `main`)
//!
//! - 2 `ThinWall` clusters, both `thickness ≈ 0.4 mm` (within `1e-5`).
//! - Outer cluster centroid `(15, 10, 15)` area `600 mm²`.
//! - Inner cluster centroid `(15, 10, 14.6)` area `459 mm²`.
//! - 2 Critical `ThinWall` issues; `!is_printable()`.
//! - 1+ Overhang region (cavity ceiling, Critical).
//!
//! Three `TrappedVolume` assertions from §7.1 of the v0.8 fix arc spec
//! (sealed-cavity volume + centroid + count) are deferred to row #14b
//! — a tiny follow-up commit immediately after row #14 (when the
//! `TrappedVolume` detector first ships). The current `PrintValidation`
//! struct has no `trapped_volumes` field yet. See
//! `mesh-printability/CHANGELOG.md` `[Unreleased]/Added` for the
//! deferral note and §12.1 of the v0.8 fix arc spec for the row
//! #11 / row #14b split rationale.
//!
//! ## How to run
//!
//! ```text
//! cargo run -p example-mesh-printability-thin-wall --release
//! ```
//!
//! Output written to `examples/mesh/printability-thin-wall/out/`. Open
//! `mesh.ply` and `issues.ply` in `MeshLab` or `ParaView` for the
//! visuals pass — see the README's f3d-winding callout for
//! viewer-specific notes on the inner-shell winding.

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply;
use mesh_printability::{
    IssueSeverity, PrintIssueType, PrintValidation, PrinterConfig, validate_for_printing,
};
use mesh_types::{IndexedMesh, Point3};

/// Outer cube extents (mm). x ∈ [0, 30], y ∈ [0, 20], z ∈ [0, 15].
/// Side and bottom walls are 1.5 mm; top wall is thinned to 0.4 mm.
const OUTER_X: f64 = 30.0;
const OUTER_Y: f64 = 20.0;
const OUTER_Z: f64 = 15.0;

/// Inner cavity extents (mm). Top is at z = 14.6 (= 15.0 − 0.4); other
/// faces are offset inward by 1.5 mm from the outer faces.
const INNER_X_MIN: f64 = 1.5;
const INNER_X_MAX: f64 = 28.5;
const INNER_Y_MIN: f64 = 1.5;
const INNER_Y_MAX: f64 = 18.5;
const INNER_Z_MIN: f64 = 1.5;
const INNER_Z_MAX: f64 = 14.6;

/// Tolerance on reported cluster `thickness`. The detector reports
/// `min_dist + EPS_RAY_OFFSET` where `EPS_RAY_OFFSET = 1e-6` (the
/// inward starting offset of the ray) — so on a 0.4 mm wall the
/// reported thickness is exactly 0.4 mm modulo IEEE-754 add ordering.
/// 1e-5 leaves an order of magnitude of headroom for cross-platform
/// FP drift.
const THICKNESS_TOL: f64 = 1.0e-5;

/// Tolerance on cluster centroid coordinates (mm). The fixture's
/// vertex coordinates are exact-representable in f64 and the unweighted
/// face-centroid mean is exact arithmetic on those representations, so
/// 1e-9 is comfortably tight while leaving headroom for IEEE-754 add
/// ordering.
const CENTROID_TOL: f64 = 1.0e-9;

/// Tolerance on cluster area (mm²). Per-face area is `‖edge1 × edge2‖ / 2`
/// for axis-aligned right triangles — geometrically exact, no chord
/// error.
const AREA_TOL: f64 = 1.0e-9;

fn main() -> Result<()> {
    let mesh = build_hollow_box();
    let config = PrinterConfig::fdm_default();
    let validation = validate_for_printing(&mesh, &config)?;

    println!("==== mesh-printability-thin-wall ====");
    println!();
    println!("input  : 24-triangle hollow box");
    println!(
        "         outer {OUTER_X}×{OUTER_Y}×{OUTER_Z} mm; inner cavity x ∈ [{INNER_X_MIN}, {INNER_X_MAX}], y ∈ [{INNER_Y_MIN}, {INNER_Y_MAX}], z ∈ [{INNER_Z_MIN}, {INNER_Z_MAX}]"
    );
    println!("         top wall thinned to 0.4 mm; side and bottom walls 1.5 mm");
    println!("config : PrinterConfig::fdm_default() — min_wall_thickness = 1.0 mm");
    println!();
    println!("{}", validation.summary());
    println!();

    print_diagnostics(&validation);
    verify(&validation);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let mesh_path = out_dir.join("mesh.ply");
    let issues_path = out_dir.join("issues.ply");
    save_ply(&mesh, &mesh_path, false)?;
    save_issue_centroids(&validation, &issues_path)?;

    println!();
    println!("artifacts:");
    println!(
        "  out/mesh.ply   : {}v, {}f (ASCII)",
        mesh.vertices.len(),
        mesh.faces.len(),
    );
    println!(
        "  out/issues.ply : {} centroid point(s) (ASCII, vertex-only)",
        issue_centroid_count(&validation),
    );
    println!();
    println!("OK — ThinWall demonstration verified");

    Ok(())
}

/// Hand-author the 24-triangle hollow box.
///
/// Two vertex-disjoint shells (outer 0..=7, inner 8..=15). Each shell
/// is independently watertight; the two shells together produce a mesh
/// where every edge appears in exactly two faces (intra-outer or
/// intra-inner). Outer winding is CCW-from-outside (normals outward);
/// inner winding is REVERSED (normals point INTO the cavity).
fn build_hollow_box() -> IndexedMesh {
    let vertices = vec![
        // Outer cube (8 corners)
        Point3::new(0.0, 0.0, 0.0),             // 0  bottom-front-left
        Point3::new(OUTER_X, 0.0, 0.0),         // 1  bottom-front-right
        Point3::new(OUTER_X, OUTER_Y, 0.0),     // 2  bottom-back-right
        Point3::new(0.0, OUTER_Y, 0.0),         // 3  bottom-back-left
        Point3::new(0.0, 0.0, OUTER_Z),         // 4  top-front-left
        Point3::new(OUTER_X, 0.0, OUTER_Z),     // 5  top-front-right
        Point3::new(OUTER_X, OUTER_Y, OUTER_Z), // 6  top-back-right
        Point3::new(0.0, OUTER_Y, OUTER_Z),     // 7  top-back-left
        // Inner cavity (8 corners, vertex-disjoint at indices 8..=15)
        Point3::new(INNER_X_MIN, INNER_Y_MIN, INNER_Z_MIN), // 8
        Point3::new(INNER_X_MAX, INNER_Y_MIN, INNER_Z_MIN), // 9
        Point3::new(INNER_X_MAX, INNER_Y_MAX, INNER_Z_MIN), // 10
        Point3::new(INNER_X_MIN, INNER_Y_MAX, INNER_Z_MIN), // 11
        Point3::new(INNER_X_MIN, INNER_Y_MIN, INNER_Z_MAX), // 12
        Point3::new(INNER_X_MAX, INNER_Y_MIN, INNER_Z_MAX), // 13
        Point3::new(INNER_X_MAX, INNER_Y_MAX, INNER_Z_MAX), // 14
        Point3::new(INNER_X_MIN, INNER_Y_MAX, INNER_Z_MAX), // 15
    ];

    let faces: Vec<[u32; 3]> = vec![
        // ─── Outer shell, CCW-from-outside (normals outward) ──────────
        // Bottom (z = 0, normal −z)
        [0, 3, 2],
        [0, 2, 1],
        // Top (z = OUTER_Z, normal +z) — load-bearing thin face #1
        [4, 5, 6],
        [4, 6, 7],
        // Front (y = 0, normal −y)
        [0, 1, 5],
        [0, 5, 4],
        // Back (y = OUTER_Y, normal +y)
        [3, 7, 6],
        [3, 6, 2],
        // Left (x = 0, normal −x)
        [0, 4, 7],
        [0, 7, 3],
        // Right (x = OUTER_X, normal +x)
        [1, 2, 6],
        [1, 6, 5],
        // ─── Inner shell, REVERSED winding (normals point INTO cavity) ─
        // Each face's normal points away from the surrounding solid.
        // Inner bottom (z = INNER_Z_MIN, normal +z, into cavity above)
        [8, 9, 10],
        [8, 10, 11],
        // Inner top (z = INNER_Z_MAX, normal −z, into cavity below) —
        // load-bearing thin face #2; cavity-ceiling overhang co-flag.
        [12, 14, 13],
        [12, 15, 14],
        // Inner front (y = INNER_Y_MIN, normal +y, into cavity behind)
        [8, 12, 13],
        [8, 13, 9],
        // Inner back (y = INNER_Y_MAX, normal −y, into cavity in front)
        [11, 10, 14],
        [11, 14, 15],
        // Inner left (x = INNER_X_MIN, normal +x, into cavity to right)
        [8, 11, 15],
        [8, 15, 12],
        // Inner right (x = INNER_X_MAX, normal −x, into cavity to left)
        [9, 13, 14],
        [9, 14, 10],
    ];

    IndexedMesh::from_parts(vertices, faces)
}

/// Print region/issue diagnostics to stdout. Surfaces the cluster
/// centroids, areas, thicknesses, and the cavity-ceiling overhang
/// co-flag for the visuals pass.
fn print_diagnostics(v: &PrintValidation) {
    println!("ThinWall regions ({}):", v.thin_walls.len());
    for (i, region) in v.thin_walls.iter().enumerate() {
        println!(
            "  [{i}] center=({:+.4}, {:+.4}, {:+.4})  thickness={:.6} mm  area={:.3} mm²  faces={}",
            region.center.x,
            region.center.y,
            region.center.z,
            region.thickness,
            region.area,
            region.faces.len(),
        );
    }
    println!();
    println!("Overhang regions ({}):", v.overhangs.len());
    for (i, region) in v.overhangs.iter().enumerate() {
        println!(
            "  [{i}] center=({:+.4}, {:+.4}, {:+.4})  angle={:.3}°  area={:.3} mm²  faces={}",
            region.center.x,
            region.center.y,
            region.center.z,
            region.angle,
            region.area,
            region.faces.len(),
        );
    }
    println!();
    println!("Issues ({}):", v.issues.len());
    for issue in &v.issues {
        println!(
            "  [{:?} / {:?}] {}",
            issue.severity, issue.issue_type, issue.description,
        );
    }
}

/// Verify the §7.1 numerical anchors. Each assertion's failure message
/// names the invariant being checked (per `feedback_risk_mitigation_review`'s
/// pass-by-correctness rule), so a regression that flips Critical→Warning
/// or breaks the cluster split fails loudly with the bug it caught.
///
/// **Deferred per §12.1 row #11**: the three `TrappedVolume` assertions
/// from §7.1 (`trapped_volumes.len() == 1`, volume ≈ 6011.7 mm³,
/// centroid (15, 10, 8.05)) are NOT asserted here. The current
/// `PrintValidation` struct has no `trapped_volumes` field; the
/// `TrappedVolume` detector first ships at row #14 of the v0.8 fix
/// arc, and row #14b (a tiny follow-up commit immediately after)
/// backfills these three assertions.
fn verify(v: &PrintValidation) {
    // (1) Cluster count: outer top + inner top, edge-adjacency
    // partitions into exactly two components because the shells are
    // vertex-disjoint (no shared edges between them).
    assert_eq!(
        v.thin_walls.len(),
        2,
        "expected outer-top and inner-top clusters; topologically disjoint via edge-adjacency",
    );

    // (2 + 3) Identify the two clusters by their z-coordinate, then
    // assert the per-cluster geometric anchors. Sorting ascending by
    // `center.z` puts the inner cluster (z = 14.6) first, the outer
    // cluster (z = 15) second.
    let mut sorted: Vec<&_> = v.thin_walls.iter().collect();
    sorted.sort_by(|a, b| a.center.z.total_cmp(&b.center.z));
    let inner = sorted[0];
    let outer = sorted[1];

    assert_relative_eq!(inner.center.z, INNER_Z_MAX, epsilon = CENTROID_TOL);
    assert_relative_eq!(inner.center.x, OUTER_X / 2.0, epsilon = CENTROID_TOL);
    assert_relative_eq!(inner.center.y, OUTER_Y / 2.0, epsilon = CENTROID_TOL);
    assert_relative_eq!(outer.center.z, OUTER_Z, epsilon = CENTROID_TOL);
    assert_relative_eq!(outer.center.x, OUTER_X / 2.0, epsilon = CENTROID_TOL);
    assert_relative_eq!(outer.center.y, OUTER_Y / 2.0, epsilon = CENTROID_TOL);

    // (4) Cluster areas: outer 30 × 20 = 600, inner 27 × 17 = 459.
    // Axis-aligned right triangles ⇒ `len/2` cross-product magnitude
    // is geometrically exact.
    assert_relative_eq!(outer.area, OUTER_X * OUTER_Y, epsilon = AREA_TOL);
    let inner_area_expected = (INNER_X_MAX - INNER_X_MIN) * (INNER_Y_MAX - INNER_Y_MIN);
    assert_relative_eq!(inner.area, inner_area_expected, epsilon = AREA_TOL);

    // (5) Both clusters report the same physical thickness (0.4 mm).
    // Reported value is `min_dist + EPS_RAY_OFFSET` per §6.1 (the
    // 1 µm inward ray-start offset is added back in so the wall at
    // exactly `min_wall_thickness` would not flag — strict-less-than).
    let expected_thickness = OUTER_Z - INNER_Z_MAX; // 0.4 mm
    for region in &v.thin_walls {
        assert_relative_eq!(
            region.thickness,
            expected_thickness,
            epsilon = THICKNESS_TOL
        );
    }

    // (6) Both clusters surface as Critical ThinWall issues. Severity
    // band: `thickness < min_wall_thickness / 2` ⇒ Critical
    // (0.4 < 1.0/2 = 0.5).
    let critical_thin_walls = v
        .issues
        .iter()
        .filter(|i| {
            i.issue_type == PrintIssueType::ThinWall && i.severity == IssueSeverity::Critical
        })
        .count();
    assert_eq!(
        critical_thin_walls, 2,
        "two Critical ThinWall issues, one per cluster",
    );

    // (7) Critical issues block printability.
    assert!(
        !v.is_printable(),
        "0.4 mm wall under min_wall_thickness = 1.0 mm must block is_printable()",
    );

    // (8) Cavity-ceiling overhang co-flag: the inner top face's normal
    // is −z (points into the cavity), so its overhang_angle is 90° —
    // > 45° + 30° = 75° lifts it to Critical under FDM defaults.
    // Asserts ≥1 (rather than == 1) so this stays robust if a future
    // detector revision splits the cavity ceiling into multiple
    // components for any reason.
    assert!(
        !v.overhangs.is_empty(),
        "cavity ceiling (inner top, normal −z) must flag as Overhang under FDM max=45°",
    );

    // The detector should NOT have skipped — preconditions hold.
    let skipped_count = v
        .issues
        .iter()
        .filter(|i| i.issue_type == PrintIssueType::DetectorSkipped)
        .count();
    assert_eq!(
        skipped_count, 0,
        "watertight + consistently-wound fixture; ThinWall must not skip",
    );
}

/// Write region centroids as a vertex-only ASCII PLY. Per §7.0's
/// per-example helper template; duplicated per-example (not factored
/// out) per `feedback_simplify_examples`.
///
/// Currently aggregates `thin_walls` + `overhangs` + `support_regions`
/// — the populated region collections in v0.8. Future detectors
/// (`TrappedVolume` row #14, `LongBridge` row #12, etc.) will extend
/// their own collections, and each example's helper will pick those up
/// independently.
fn save_issue_centroids(v: &PrintValidation, path: &Path) -> Result<()> {
    let mut centroids: Vec<Point3<f64>> = Vec::new();
    centroids.extend(v.thin_walls.iter().map(|r| r.center));
    centroids.extend(v.overhangs.iter().map(|r| r.center));
    centroids.extend(v.support_regions.iter().map(|r| r.center));
    let mesh = IndexedMesh::from_parts(centroids, vec![]);
    save_ply(&mesh, path, false)?;
    Ok(())
}

/// Number of region centroids written to `out/issues.ply`.
const fn issue_centroid_count(v: &PrintValidation) -> usize {
    v.thin_walls.len() + v.overhangs.len() + v.support_regions.len()
}
