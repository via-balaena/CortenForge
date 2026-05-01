//! Visual demo of the §6.2 `LongBridge` detector (Gap G).
//!
//! Hand-authors a 24-vertex / 44-triangle H-shape — two 5×5×18 mm
//! pillars joined by a 35×5×2 mm horizontal slab (lintel) — as a single
//! watertight, consistently-wound boolean-union solid. The slab's
//! downward-facing bottom is exposed in three edge-disjoint regions:
//! a 20×5 middle bridge between the pillars (span 20 mm, exceeds the
//! `PrinterConfig::fdm_default()` `max_bridge_span = 10 mm` threshold
//! by 1.5×, lifting it to Critical) plus two 2.5×5 cantilevers at each
//! outer end (each span 5 mm, below threshold and silently dropped by
//! the bridge detector).
//!
//! Saves `out/mesh.ply` (the input fixture) and `out/issues.ply` (a
//! 4-point centroid cloud — 1 `LongBridge` midpoint at (15, 2.5, 18)
//! plus 3 `OverhangRegion` centroids at (-1.25, 2.5, 18), (15, 2.5, 18),
//! (31.25, 2.5, 18); the middle-bridge `LongBridge` midpoint and the
//! middle-bridge `OverhangRegion` centroid both land at (15, 2.5, 18),
//! so two of the four centroid points are coincident — that overlap is
//! the load-bearing multi-detector co-flag the example anchors).
//!
//! ## Why this fixture
//!
//! `LongBridge` requires faces whose outward normal sits within 30° of
//! `-up` ("near-horizontal downward") and the cluster's perpendicular-
//! plane bbox extent must exceed `max_bridge_span`. The H-shape gives
//! all three: clean axis-aligned downward faces, a known-large middle
//! span (20 mm), and two control clusters (2.5×5 cantilevers) below
//! the threshold so the detector exercises BOTH branches — emit and
//! silent-drop — on a single fixture.
//!
//! ## Why a manifold H-shape (not three boxes)
//!
//! `validate_for_printing` is layered: `check_basic_manifold` runs
//! before `check_long_bridges` and pushes a Critical `NotWatertight`
//! issue if any edge is incident on a face count other than 2. Three
//! independent boxes (12 + 12 + 12 = 36 triangles, three disjoint
//! shells) all share a y=0 wall and a y=5 wall along the entire
//! H-outline; the pillar-slab interfaces at z=18 would be 5×5 pairs of
//! overlapping faces with opposed normals — non-manifold. Building a
//! single boolean-union surface (the pillar tops at z=18 become
//! interior, the slab bottom is a polygon-with-2-rectangular-holes)
//! satisfies the manifold precondition AND keeps the `LongBridge`
//! cluster split clean (the 3 slab-bottom rectangles share no edges
//! with each other, by construction).
//!
//! ## Vertex / triangle count
//!
//! 24 unique vertices, 44 triangles. The `~16/~28` figure in the v0.8
//! fix arc spec §7.2 was an early estimate before the polygon-with-
//! 2-holes triangulation was worked out; the actual minimum on this
//! geometry is 24 vertices (8 pillar bases, 8 pillar tops, 4 slab
//! outer at z=18, 4 slab top at z=20) and 44 triangles broken down as
//! 10 tris per pillar × 2 pillars, 2 slab top, 6 slab front fan, 6
//! slab back fan, 2 slab left, 2 slab right, 6 slab bottom across 3
//! regions. Hand-traced manifold proof: every undirected edge appears
//! in exactly two faces (verified during pre-flight by full edge
//! enumeration); winding is consistent (every directed edge is matched
//! by its reverse on the other incident face).
//!
//! ## Numerical anchors (asserted in `main`)
//!
//! - `long_bridges.len() == 1` — only the middle bridge fires.
//! - Middle bridge `span ≈ 20.0 mm` (the 5 → 25 inter-pillar gap).
//! - Middle bridge `start.x ≈ 5.0`, `end.x ≈ 25.0` (bbox endpoints).
//! - 1 Critical `LongBridge` issue (20 > 10 × 1.5 = 15).
//! - `overhangs.len() == 3` — Gap-D-split clusters: left cantilever +
//!   middle bridge + right cantilever, each 90° downward.
//! - 3 Critical `ExcessiveOverhang` issues (90 > 45 + 30).
//! - 0 `TrappedVolume` issues — H-shape has no sealed cavity (asserted
//!   via `issues.iter().filter(...)` since `PrintValidation` has no
//!   `trapped_volumes` field until row #14 of the v0.8 fix arc; semantic
//!   equivalent to the spec's `validation.trapped_volumes.len() == 0`,
//!   no row #14b backfill needed).
//! - `!is_printable()` — Critical `LongBridge` + Critical
//!   `ExcessiveOverhang` both block.
//!
//! ## SLS tech-skip cross-check
//!
//! Re-validates the same fixture with `PrinterConfig::sls_default()`.
//! SLS has `requires_supports() == false`, so both `check_overhangs`
//! and `check_long_bridges` early-return without classifying — no
//! `DetectorSkipped` issue is emitted (§6.2 line 996; that variant is
//! for precondition skips, not technology-policy skips). Result:
//! `long_bridges.len() == 0` AND `overhangs.len() == 0` AND zero
//! `DetectorSkipped` issues — a clean tech-skip cross-check.
//!
//! ## How to run
//!
//! ```text
//! cargo run -p example-mesh-printability-long-bridge --release
//! ```
//!
//! Output written to `examples/mesh/printability-long-bridge/out/`. Open
//! `mesh.ply` and `issues.ply` in `MeshLab` or `ParaView` for the
//! visuals pass — see the README's f3d back-face-culling callout for
//! viewer-specific notes on the slab's downward-facing bottom.

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply;
use mesh_printability::{
    IssueSeverity, PrintIssueType, PrintValidation, PrinterConfig, validate_for_printing,
};
use mesh_types::{IndexedMesh, Point3};

// -- Geometry constants (mm) ------------------------------------------------

/// Pillar height — both pillars run from `z = 0` to `z = PILLAR_TOP`.
/// The slab attaches at `z = PILLAR_TOP`; pillar tops become interior.
const PILLAR_TOP: f64 = 18.0;

/// Slab top height — the slab fills `z ∈ [PILLAR_TOP, SLAB_TOP]`,
/// thickness 2 mm.
const SLAB_TOP: f64 = 20.0;

/// Pillar 1 footprint — `x ∈ [P1_X_MIN, P1_X_MAX]`.
const P1_X_MIN: f64 = 0.0;
const P1_X_MAX: f64 = 5.0;

/// Pillar 2 footprint — `x ∈ [P2_X_MIN, P2_X_MAX]`.
const P2_X_MIN: f64 = 25.0;
const P2_X_MAX: f64 = 30.0;

/// Common pillar `y` extent — both pillars span `y ∈ [P_Y_MIN, P_Y_MAX]`.
/// The slab uses the same `y` extent so the H is 5 mm thick along `y`.
const P_Y_MIN: f64 = 0.0;
const P_Y_MAX: f64 = 5.0;

/// Slab `x` extent — `x ∈ [SLAB_X_MIN, SLAB_X_MAX]`. The slab
/// cantilevers 2.5 mm past each pillar's outer edge (`-2.5..0` and
/// `30..32.5`).
const SLAB_X_MIN: f64 = -2.5;
const SLAB_X_MAX: f64 = 32.5;

// -- Numerical-anchor tolerances --------------------------------------------

/// Tolerance on integer-coordinate centroids and span endpoints. The
/// fixture's vertex coordinates are exact-representable in f64 and the
/// per-face-centroid mean is exact arithmetic on those representations,
/// so 1e-9 leaves an order of magnitude of headroom for IEEE-754 add
/// ordering on the running cluster-centroid sum.
const CENTROID_TOL: f64 = 1.0e-9;

/// Tolerance on the cluster's perpendicular-plane bbox extent (`span`).
/// Per §7.2 spec assertion #2: `1e-6` is the documented tolerance — the
/// spec is conservative because the bbox is computed via `max.x - min.x`
/// after a per-vertex `e1.dot(v)` projection, two FP ops that can
/// accumulate ~ULP drift. On this fixture (axis-aligned, `e1 = +x`) the
/// projections collapse to `v.x` and the subtraction is exact.
const SPAN_TOL: f64 = 1.0e-6;

fn main() -> Result<()> {
    let mesh = build_h_shape();

    let fdm = PrinterConfig::fdm_default();
    let validation = validate_for_printing(&mesh, &fdm)?;

    println!("==== mesh-printability-long-bridge ====");
    println!();
    println!("input  : 44-triangle hand-authored H-shape");
    println!(
        "         pillar 1: x ∈ [{P1_X_MIN}, {P1_X_MAX}], y ∈ [{P_Y_MIN}, {P_Y_MAX}], z ∈ [0, {PILLAR_TOP}]"
    );
    println!(
        "         pillar 2: x ∈ [{P2_X_MIN}, {P2_X_MAX}], y ∈ [{P_Y_MIN}, {P_Y_MAX}], z ∈ [0, {PILLAR_TOP}]"
    );
    println!(
        "         slab    : x ∈ [{SLAB_X_MIN}, {SLAB_X_MAX}], y ∈ [{P_Y_MIN}, {P_Y_MAX}], z ∈ [{PILLAR_TOP}, {SLAB_TOP}]"
    );
    println!("config : PrinterConfig::fdm_default() — max_bridge_span = 10.0 mm");
    println!();
    println!("{}", validation.summary());
    println!();

    print_diagnostics(&validation);
    verify_fdm(&validation);

    let sls = PrinterConfig::sls_default();
    let sls_validation = validate_for_printing(&mesh, &sls)?;
    println!();
    println!("---- SLS cross-check (max_overhang_angle = 90°, max_bridge_span = ∞) ----");
    println!("{}", sls_validation.summary());
    verify_sls_tech_skip(&sls_validation);

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
    println!("OK — LongBridge demonstration verified");

    Ok(())
}

/// Hand-author the 24-vertex / 44-triangle H-shape.
///
/// Vertex indexing (groups of 4):
/// - `0..=3`  pillar 1 base (z = 0)
/// - `4..=7`  pillar 1 top (z = `PILLAR_TOP`); pillar 1's top face is
///   interior (covered by the slab) — these vertices ARE referenced by
///   pillar sides + slab front/back/bottom but NOT by a "pillar top"
///   triangle.
/// - `8..=11`  pillar 2 base (z = 0)
/// - `12..=15` pillar 2 top (z = `PILLAR_TOP`); same convention.
/// - `16..=19` slab outer corners at z = `PILLAR_TOP` (one per
///   `{x_min, x_max}` × `{y_min, y_max}`; these sit *outside* the
///   pillar footprints).
/// - `20..=23` slab top (z = `SLAB_TOP`).
///
/// Triangle layout (44 total): pillar 1 has 5 outward-wound faces (no
/// top) × 2 = 10; pillar 2 same = 10; slab has 6 outward-wound faces
/// — top + bottom (split into 3 disjoint regions) + 4 sides; the slab
/// front + back are 8-gons (the bottom edge is broken by the four
/// pillar-attachment x-coordinates) triangulated as 6-triangle fans
/// from the top-corner vertex; left + right are 2 each; bottom regions
/// are 2 each × 3 = 6.
fn build_h_shape() -> IndexedMesh {
    let vertices = vec![
        // Pillar 1 base (z = 0): 0..=3
        Point3::new(P1_X_MIN, P_Y_MIN, 0.0), //  0
        Point3::new(P1_X_MAX, P_Y_MIN, 0.0), //  1
        Point3::new(P1_X_MAX, P_Y_MAX, 0.0), //  2
        Point3::new(P1_X_MIN, P_Y_MAX, 0.0), //  3
        // Pillar 1 top (z = PILLAR_TOP): 4..=7
        Point3::new(P1_X_MIN, P_Y_MIN, PILLAR_TOP), //  4
        Point3::new(P1_X_MAX, P_Y_MIN, PILLAR_TOP), //  5
        Point3::new(P1_X_MAX, P_Y_MAX, PILLAR_TOP), //  6
        Point3::new(P1_X_MIN, P_Y_MAX, PILLAR_TOP), //  7
        // Pillar 2 base (z = 0): 8..=11
        Point3::new(P2_X_MIN, P_Y_MIN, 0.0), //  8
        Point3::new(P2_X_MAX, P_Y_MIN, 0.0), //  9
        Point3::new(P2_X_MAX, P_Y_MAX, 0.0), // 10
        Point3::new(P2_X_MIN, P_Y_MAX, 0.0), // 11
        // Pillar 2 top (z = PILLAR_TOP): 12..=15
        Point3::new(P2_X_MIN, P_Y_MIN, PILLAR_TOP), // 12
        Point3::new(P2_X_MAX, P_Y_MIN, PILLAR_TOP), // 13
        Point3::new(P2_X_MAX, P_Y_MAX, PILLAR_TOP), // 14
        Point3::new(P2_X_MIN, P_Y_MAX, PILLAR_TOP), // 15
        // Slab outer corners at z = PILLAR_TOP: 16..=19
        Point3::new(SLAB_X_MIN, P_Y_MIN, PILLAR_TOP), // 16
        Point3::new(SLAB_X_MAX, P_Y_MIN, PILLAR_TOP), // 17
        Point3::new(SLAB_X_MAX, P_Y_MAX, PILLAR_TOP), // 18
        Point3::new(SLAB_X_MIN, P_Y_MAX, PILLAR_TOP), // 19
        // Slab top (z = SLAB_TOP): 20..=23
        Point3::new(SLAB_X_MIN, P_Y_MIN, SLAB_TOP), // 20
        Point3::new(SLAB_X_MAX, P_Y_MIN, SLAB_TOP), // 21
        Point3::new(SLAB_X_MAX, P_Y_MAX, SLAB_TOP), // 22
        Point3::new(SLAB_X_MIN, P_Y_MAX, SLAB_TOP), // 23
    ];

    let faces: Vec<[u32; 3]> = vec![
        // ─── Pillar 1 (10 tris, no top) ──────────────────────────────────
        // Bottom (z = 0, normal -z)
        [0, 3, 2],
        [0, 2, 1],
        // Front (y = P_Y_MIN, normal -y)
        [0, 1, 5],
        [0, 5, 4],
        // Back (y = P_Y_MAX, normal +y)
        [3, 7, 6],
        [3, 6, 2],
        // Left (x = P1_X_MIN, normal -x)
        [0, 4, 7],
        [0, 7, 3],
        // Right (x = P1_X_MAX, normal +x)
        [1, 2, 6],
        [1, 6, 5],
        // ─── Pillar 2 (10 tris, no top) ──────────────────────────────────
        // Bottom (z = 0, normal -z)
        [8, 11, 10],
        [8, 10, 9],
        // Front (y = P_Y_MIN, normal -y)
        [8, 9, 13],
        [8, 13, 12],
        // Back (y = P_Y_MAX, normal +y)
        [11, 15, 14],
        [11, 14, 10],
        // Left (x = P2_X_MIN, normal -x)
        [8, 12, 15],
        [8, 15, 11],
        // Right (x = P2_X_MAX, normal +x)
        [9, 10, 14],
        [9, 14, 13],
        // ─── Slab top (z = SLAB_TOP, normal +z) ──────────────────────────
        [20, 21, 22],
        [20, 22, 23],
        // ─── Slab front (y = P_Y_MIN, normal -y, fan from v20) ───────────
        // 8-gon outline (bottom edge broken at v4, v5, v12, v13):
        // v16 → v4 → v5 → v12 → v13 → v17 → v21 → v20 → close.
        [20, 16, 4],
        [20, 4, 5],
        [20, 5, 12],
        [20, 12, 13],
        [20, 13, 17],
        [20, 17, 21],
        // ─── Slab back (y = P_Y_MAX, normal +y, fan from v23) ────────────
        // 8-gon outline (bottom edge broken at v14, v15, v6, v7):
        // v18 → v14 → v15 → v6 → v7 → v19 → v23 → v22 → close.
        [23, 22, 18],
        [23, 18, 14],
        [23, 14, 15],
        [23, 15, 6],
        [23, 6, 7],
        [23, 7, 19],
        // ─── Slab left (x = SLAB_X_MIN, normal -x) ───────────────────────
        [19, 16, 20],
        [19, 20, 23],
        // ─── Slab right (x = SLAB_X_MAX, normal +x) ──────────────────────
        [17, 18, 22],
        [17, 22, 21],
        // ─── Slab bottom (z = PILLAR_TOP, normal -z, 3 disjoint regions) ─
        // Left cantilever (x ∈ [SLAB_X_MIN, P1_X_MIN], 2.5 × 5)
        [16, 19, 7],
        [16, 7, 4],
        // Middle bridge (x ∈ [P1_X_MAX, P2_X_MIN], 20 × 5) ← target
        [5, 6, 15],
        [5, 15, 12],
        // Right cantilever (x ∈ [P2_X_MAX, SLAB_X_MAX], 2.5 × 5)
        [13, 14, 18],
        [13, 18, 17],
    ];

    IndexedMesh::from_parts(vertices, faces)
}

/// Print region/issue diagnostics to stdout. Surfaces the cluster
/// `start`/`end`/`span`/`area` for each `LongBridgeRegion`, and the
/// per-overhang centroid + angle + area, so the visuals pass can
/// cross-reference the centroids against `out/issues.ply`.
fn print_diagnostics(v: &PrintValidation) {
    println!("LongBridge regions ({}):", v.long_bridges.len());
    for (i, region) in v.long_bridges.iter().enumerate() {
        let mid = region_midpoint(region);
        println!(
            "  [{i}] start=({:+.4}, {:+.4}, {:+.4})  end=({:+.4}, {:+.4}, {:+.4})  span={:.6} mm  midpoint=({:+.4}, {:+.4}, {:+.4})  faces={}",
            region.start.x,
            region.start.y,
            region.start.z,
            region.end.x,
            region.end.y,
            region.end.z,
            region.span,
            mid.x,
            mid.y,
            mid.z,
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

/// Verify the §7.2 numerical anchors under FDM defaults. Each
/// assertion's failure message names the invariant being checked (per
/// `feedback_risk_mitigation_review`'s pass-by-correctness rule).
///
/// Spec assertion #7 (`validation.trapped_volumes.len() == 0`) is
/// implemented as a `PrintIssueType::TrappedVolume` issue-count filter
/// rather than a literal field check. The `trapped_volumes` field
/// doesn't exist on `PrintValidation` until row #14 of the v0.8 fix arc
/// (§6.3 detector); the variant `PrintIssueType::TrappedVolume` is
/// defined in v0.7 so the filter compiles + runs today and stays
/// semantically correct after row #14 ships (the H-shape genuinely has
/// no sealed cavity, so the detector will never fire on this fixture).
/// No row #14b backfill is needed for this example.
fn verify_fdm(v: &PrintValidation) {
    // (1) Exactly one cluster fires LongBridge: the middle bridge.
    // The two cantilevers cluster independently but their span
    // (max(2.5, 5.0) = 5 mm) ≤ max_bridge_span (10 mm), so
    // emit_long_bridge_component silently drops them.
    assert_eq!(
        v.long_bridges.len(),
        1,
        "only the middle bridge (span 20 mm) exceeds max_bridge_span = 10 mm; cantilevers (span 5 mm) are silently dropped",
    );

    // (2) Span = inter-pillar gap = P2_X_MIN - P1_X_MAX = 25 - 5 = 20 mm.
    let bridge = &v.long_bridges[0];
    let expected_span = P2_X_MIN - P1_X_MAX;
    assert_relative_eq!(bridge.span, expected_span, epsilon = SPAN_TOL);

    // (3) start.x = 5, end.x = 25 (bbox endpoints back-projected from
    // the cluster centroid's z-elevation). The middle-bridge cluster's
    // perpendicular-plane bbox is (e1=+x, e2=+y) with e1-extent 20 ≥
    // e2-extent 5, so axis = e1 and start/end bracket the x-extent.
    // start.y/end.y are the perpendicular-axis midpoint — for this
    // fixture (y_min = 0, y_max = 5), midpoint = 2.5. start.z/end.z
    // are the cluster centroid's z, which is exactly PILLAR_TOP since
    // every cluster vertex sits at z = PILLAR_TOP.
    assert_relative_eq!(bridge.start.x, P1_X_MAX, epsilon = SPAN_TOL);
    assert_relative_eq!(bridge.end.x, P2_X_MIN, epsilon = SPAN_TOL);
    let y_mid = f64::midpoint(P_Y_MIN, P_Y_MAX);
    assert_relative_eq!(bridge.start.y, y_mid, epsilon = SPAN_TOL);
    assert_relative_eq!(bridge.end.y, y_mid, epsilon = SPAN_TOL);
    assert_relative_eq!(bridge.start.z, PILLAR_TOP, epsilon = SPAN_TOL);
    assert_relative_eq!(bridge.end.z, PILLAR_TOP, epsilon = SPAN_TOL);

    // (4) LongBridge issue is Critical (span > max_bridge_span × 1.5
    // = 15 → Critical band).
    let critical_long_bridges = v
        .issues
        .iter()
        .filter(|i| {
            i.issue_type == PrintIssueType::LongBridge && i.severity == IssueSeverity::Critical
        })
        .count();
    assert_eq!(
        critical_long_bridges, 1,
        "20 mm span exceeds max_bridge_span (10 mm) × 1.5 = 15 mm → Critical, not Warning",
    );

    // (5) Three overhang regions: left cantilever + middle bridge +
    // right cantilever. Gap-D-split: each connected downward-region
    // becomes its own OverhangRegion via edge-adjacency clustering,
    // and the three slab-bottom rectangles are vertex-disjoint with
    // each other (the pillar-attachment cutouts at z = PILLAR_TOP are
    // interior, so the rectangles share no edges).
    assert_eq!(
        v.overhangs.len(),
        3,
        "Gap-D-split: left cantilever + middle bridge + right cantilever are 3 edge-disjoint clusters",
    );

    // (6) All three overhang clusters report 90° tilt (faces have
    // normal -z, anti-parallel to up = +z) and severity Critical
    // (90 > 45 + 30 = 75).
    let critical_overhangs = v
        .issues
        .iter()
        .filter(|i| {
            i.issue_type == PrintIssueType::ExcessiveOverhang
                && i.severity == IssueSeverity::Critical
        })
        .count();
    assert_eq!(
        critical_overhangs, 3,
        "all three downward slab-bottom regions are 90° → Critical",
    );

    // (7) Three overhang regions sorted ascending by center.x must
    // land at -1.25, 15.0, 31.25 (the analytical mean of the per-face
    // centroids in each cluster). The detector's component iteration
    // order is min-face-idx, so the natural ordering depends on
    // the face indexing chosen in build_h_shape; sort here for an
    // ordering-independent check.
    let mut by_x: Vec<&_> = v.overhangs.iter().collect();
    by_x.sort_by(|a, b| a.center.x.total_cmp(&b.center.x));
    assert_relative_eq!(by_x[0].center.x, -1.25, epsilon = CENTROID_TOL);
    assert_relative_eq!(by_x[1].center.x, 15.0, epsilon = CENTROID_TOL);
    assert_relative_eq!(by_x[2].center.x, 31.25, epsilon = CENTROID_TOL);
    for region in &by_x {
        assert_relative_eq!(region.center.y, 2.5, epsilon = CENTROID_TOL);
        assert_relative_eq!(region.center.z, PILLAR_TOP, epsilon = CENTROID_TOL);
    }

    // (8) Co-flag: middle-bridge LongBridge midpoint = middle-bridge
    // OverhangRegion centroid = (15, 2.5, 18). Verifying both equal
    // (15, 2.5, 18) confirms the load-bearing multi-detector overlap
    // the README documents.
    let bridge_mid = region_midpoint(bridge);
    let middle_overhang = by_x[1];
    assert_relative_eq!(bridge_mid.x, 15.0, epsilon = CENTROID_TOL);
    assert_relative_eq!(bridge_mid.y, 2.5, epsilon = CENTROID_TOL);
    assert_relative_eq!(bridge_mid.z, PILLAR_TOP, epsilon = CENTROID_TOL);
    assert_relative_eq!(
        bridge_mid.x,
        middle_overhang.center.x,
        epsilon = CENTROID_TOL
    );
    assert_relative_eq!(
        bridge_mid.y,
        middle_overhang.center.y,
        epsilon = CENTROID_TOL
    );
    assert_relative_eq!(
        bridge_mid.z,
        middle_overhang.center.z,
        epsilon = CENTROID_TOL
    );

    // (9) Sealed-cavity check (§7.2 spec assertion #7). The
    // PrintValidation struct has no trapped_volumes field until row
    // #14 of the v0.8 fix arc; assert via the issue filter instead —
    // semantically equivalent and stays correct after the detector
    // ships (H-shape has no sealed cavity by construction).
    let trapped = v
        .issues
        .iter()
        .filter(|i| i.issue_type == PrintIssueType::TrappedVolume)
        .count();
    assert_eq!(
        trapped, 0,
        "H-shape has no sealed cavity — TrappedVolume must not fire",
    );

    // (10) Critical issues block printability.
    assert!(
        !v.is_printable(),
        "Critical LongBridge + Critical ExcessiveOverhang must each independently block is_printable()",
    );

    // The detector should NOT have skipped — preconditions hold.
    let skipped_count = v
        .issues
        .iter()
        .filter(|i| i.issue_type == PrintIssueType::DetectorSkipped)
        .count();
    assert_eq!(
        skipped_count, 0,
        "FDM has requires_supports = true and the H-shape is manifold; LongBridge must not skip",
    );
}

/// Verify the SLS tech-skip cross-check (§7.2 spec assertion #8).
///
/// SLS has `requires_supports() == false`, so BOTH `check_overhangs`
/// (`validation.rs:270`) and `check_long_bridges` (`validation.rs:1303`)
/// early-return before classifying any face — no detector emits a
/// `DetectorSkipped` issue (per §6.2 line 996; that variant is
/// reserved for `ThinWall` / `TrappedVolume` precondition skips, not
/// technology-policy skips). As a backup invariant, even if the
/// detectors DID run on SLS the predicates would not fire on this
/// fixture: SLS's `max_overhang_angle = 90°` is not strictly less than
/// the slab's 90° face tilt, and `max_bridge_span = ∞` rejects any
/// finite span.
fn verify_sls_tech_skip(v: &PrintValidation) {
    assert_eq!(
        v.long_bridges.len(),
        0,
        "SLS does not require supports → check_long_bridges silent-skips",
    );
    assert_eq!(
        v.overhangs.len(),
        0,
        "SLS does not require supports → check_overhangs silent-skips",
    );
    let skipped_count = v
        .issues
        .iter()
        .filter(|i| i.issue_type == PrintIssueType::DetectorSkipped)
        .count();
    assert_eq!(
        skipped_count, 0,
        "LongBridge silent-skip on SLS does NOT push DetectorSkipped (per §6.2 line 996)",
    );
}

/// Compute the midpoint of a `LongBridgeRegion`'s `(start, end)`
/// segment. Used both for diagnostic printing and the issues PLY
/// centroid emission so the two stay in lockstep.
fn region_midpoint(region: &mesh_printability::LongBridgeRegion) -> Point3<f64> {
    Point3::new(
        (region.start.x + region.end.x) * 0.5,
        (region.start.y + region.end.y) * 0.5,
        (region.start.z + region.end.z) * 0.5,
    )
}

/// Write region centroids as a vertex-only ASCII PLY. Per §7.0's
/// per-example helper template; duplicated per-example (not factored
/// out) per `feedback_simplify_examples`.
///
/// Aggregates `long_bridges` (midpoint of `(start, end)`) +
/// `overhangs` (`center`) — the populated region collections relevant
/// for this fixture. The resulting PLY has 4 vertex points (1 bridge
/// midpoint + 3 overhang centroids), with two of the four (the
/// middle-bridge `LongBridge` midpoint and the middle-bridge
/// `OverhangRegion` centroid) coincident at (15, 2.5, 18). `MeshLab`'s
/// "Show Vertices" with a large point size + `ParaView`'s `Glyph`
/// filter both render coincident points cleanly without dedup.
fn save_issue_centroids(v: &PrintValidation, path: &Path) -> Result<()> {
    let mut centroids: Vec<Point3<f64>> = Vec::new();
    centroids.extend(v.long_bridges.iter().map(region_midpoint));
    centroids.extend(v.overhangs.iter().map(|r| r.center));
    let mesh = IndexedMesh::from_parts(centroids, vec![]);
    save_ply(&mesh, path, false)?;
    Ok(())
}

/// Number of region centroids written to `out/issues.ply`.
const fn issue_centroid_count(v: &PrintValidation) -> usize {
    v.long_bridges.len() + v.overhangs.len()
}
