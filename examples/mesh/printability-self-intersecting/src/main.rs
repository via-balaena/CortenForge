//! Visual demo of the §6.4 `SelfIntersecting` detector (Gap I).
//!
//! Hand-authors two interpenetrating cylinders — one along `+X`, one
//! along `+Y`, each 30 mm long × 5 mm radius — concatenated with
//! vertex-index offset and **without** any boolean cleanup. The two
//! shells overlap geometrically near the origin (a 3-D plus-sign cross)
//! but share no vertices, edges, or faces; mesh-repair flags every
//! triangle pair where one cylinder's lateral surface punches through
//! the other's. Saves `out/mesh.ply` (the input fixture) and
//! `out/issues.ply` (a vertex-only point cloud at each region's
//! `approximate_location`, plus the overhang co-flag centroids).
//!
//! ## Why two interpenetrating cylinders
//!
//! `validate_for_printing` re-uses `mesh_repair::detect_self_intersections`
//! for §6.4. To exercise the wrapper end-to-end we need a fixture where
//! at least one face pair physically intersects but is NOT
//! adjacency-skipped (mesh-repair's `skip_adjacent` filters
//! vertex-shared face pairs). Two independently-authored cylinders that
//! share **no** vertices and pass through each other's lateral surfaces
//! produce dozens of un-skipped pair flags — a faithful "broken-mesh-
//! from-STL-export" example a user might encounter in the wild after
//! booleaning two CAD primitives without union cleanup.
//!
//! ## Independent watertightness — the load-bearing topological choice
//!
//! Each cylinder is hand-authored as 16 lateral segments + 2 cap fans
//! (34 vertices + 64 triangles per cylinder — 2 cap centres + 2 × 16
//! rim verts; 2 caps × 16 cap-tris + 32 lateral-tris). The CCW-from-outside
//! winding is constructed by basis: cylinder A uses `(axis, perp_u,
//! perp_v) = (+X, +Y, +Z)`; cylinder B uses `(+Y, +Z, +X)` — both
//! right-handed (`perp_u × perp_v = axis`). Each cylinder is
//! independently watertight + consistently wound. The combined mesh is
//! NOT manifold (the lateral surfaces interpenetrate) but the
//! `SelfIntersecting` detector runs unconditionally on any mesh — there
//! is no watertightness precondition for §6.4.
//!
//! ## Four interpenetration "rings" — analytical prediction
//!
//! Cylinder A's lateral surface satisfies `y² + z² = 25` for `x ∈
//! [-15, +15]`; B's satisfies `x² + z² = 25` for `y ∈ [-15, +15]`.
//! Surface intersection: subtract → `y² = x²` → `y = ±x`, combined with
//! `z² = 25 - x²` gives four space-curves (rings), each at `y = ±x`,
//! `z = ±√(25 − x²)`, `x ∈ [-5, +5]`. The four rings meet at the four
//! "corner" points `(±5, ±5, 0)`. mesh-repair flags every
//! `(A_face, B_face)` pair where the discretized A-tri and B-tri both
//! span the ring; the count depends on tessellation but stays bounded
//! by the analytical ring length × segments-per-ring.
//!
//! ## Cap-cap and cap-lateral pairs do NOT contribute
//!
//! Cylinder A's caps sit at `x = ±15`. Cylinder B's lateral surface is
//! confined to `|x| ≤ 5` (because `x² + z² = 25` gives `|x| ≤ 5`), so
//! A's caps cannot intersect B's lateral. Symmetrically for B's caps
//! vs A's lateral. And A's caps are at `x = ±15` whereas B's caps are
//! at `y = ±15` — no cap-cap overlap. Conclusion: every flagged pair
//! is `(A_lateral, B_lateral)`.
//!
//! ## `ExcessiveOverhang` co-flag — documented, not load-bearing
//!
//! Cylinders placed with horizontal axes (post-`place_on_build_plate`
//! the assembly spans `z ∈ [0, 10]`) have lateral arcs whose normals
//! span every direction perpendicular to the axis. The bottom-most
//! ring of lateral faces (touching `mesh_min_along_up = 0`) is build-
//! plate-filtered per Gap M; faces just above the bottom are not, so
//! each cylinder produces ≥ 1 `OverhangRegion`. The example documents
//! this co-flag (cylinders printed flat have unavoidable overhang on
//! the underside) but it is NOT the load-bearing assertion — Gap I is
//! about self-intersection.
//!
//! ## Single-cylinder regression — the convex-mesh anchor
//!
//! After the main fixture validates, the example re-runs validation on
//! cylinder A alone. A single cylinder is convex; its lateral surface
//! cannot cross itself, and adjacent face pairs are skipped by
//! mesh-repair's vertex-shared adjacency filter. So
//! `self_intersecting.len() == 0`. This locks the "no false positives
//! on convex single-mesh" invariant — the lateral-overhang co-flag
//! still fires (the cylinder is still horizontal), but
//! `self_intersecting` stays empty.
//!
//! ## Numerical anchors (asserted in `main`)
//!
//! 1. `self_intersecting.len() >= 4` — interpenetrating cylinders
//!    produce at least four pair flags (one per ring lower bound).
//! 2. `self_intersecting.len() <= 100` — mesh-repair's `max_reported`
//!    cap. The truncation-suffix predicate
//!    `description.contains("search truncated")` is asserted iff
//!    `len() == 100`.
//! 3. All entries' `face_a < face_b` (canonical ordering per §6.4).
//! 4. All entries' `approximate_location` is within `±5 mm`
//!    component-wise of the post-placement origin `(0, 0, 5)` — the
//!    intersection region is bounded by `|x| ≤ 5`, `|y| ≤ 5`,
//!    `z ∈ [0, 10]`.
//! 5. Every `SelfIntersecting` `PrintIssue` is `Critical`;
//!    `is_printable() == false`.
//! 6. `overhangs.len() >= 1` (lateral underside co-flag, per cylinder).
//! 7. Single-cylinder regression: `self_intersecting.len() == 0`.
//!
//! ## "Couldn't I just call `mesh_repair::detect_self_intersections`?"
//!
//! Yes — and `validate_for_printing` does internally. The point of
//! this example is to show that the §6.4 wrapper exposes
//! self-intersection through the same higher-level API as every other
//! detector: same `PrintValidation` shape, same severity classifier,
//! same `is_printable()` gate. Power users can still call
//! `mesh_repair::intersect::detect_self_intersections` directly with a
//! tuned `IntersectionParams` (the §3 spec calls for re-exporting
//! these types from `mesh-printability`; row #16 landed without that
//! re-export, open as a v0.9 candidate). §6.4 is the "print-validation
//! default" wrapper.
//!
//! ## How to run
//!
//! ```text
//! cargo run -p example-mesh-printability-self-intersecting --release
//! ```
//!
//! Output written to `examples/mesh/printability-self-intersecting/out/`.
//! Open `mesh.ply` and `issues.ply` in `MeshLab`, `ParaView`, or `f3d`
//! for the visuals pass — see the README's f3d-quirks callout for
//! viewer-specific notes on the cross geometry.

use std::path::Path;

use anyhow::Result;
use mesh_io::save_ply;
use mesh_printability::{
    IssueSeverity, PrintIssueType, PrintValidation, PrinterConfig, place_on_build_plate,
    validate_for_printing,
};
use mesh_types::{IndexedMesh, Point3, Vector3};

// -- Geometry constants (mm) ------------------------------------------------

/// Cylinder length. Each cylinder spans `[-LENGTH/2, +LENGTH/2]` along
/// its axis before placement.
const LENGTH: f64 = 30.0;

/// Cylinder radius. The lateral surface satisfies `(perp_u² + perp_v²)
/// = RADIUS²` for each cylinder's basis.
const RADIUS: f64 = 5.0;

/// Number of azimuthal segments per cylinder. The §7.4 spec line 1693
/// fixes 16 segs as the canonical resolution: at this tessellation each
/// of the four interpenetration rings traverses ~ 8 segments per
/// cylinder, and the discretized `(A_lateral, B_lateral)` tri-pair
/// count empirically exceeds mesh-repair's `max_reported = 100` cap
/// (observed `intersection_count ≈ 101–104` across runs, varying with
/// Rayon scheduling) — so the truncation-suffix path of anchor #2 is
/// exercised, not the under-cap branch.
const SEGMENTS: u32 = 16;

/// Post-`place_on_build_plate` assembly origin maps to `(0, 0, 5)`:
/// each cylinder spans `z ∈ [-RADIUS, +RADIUS]` before placement; the
/// translation lifts `z_min = -RADIUS` to `0`, so the origin's `z = 0`
/// becomes `z = RADIUS = 5`.
const POST_PLACE_ORIGIN_Z: f64 = RADIUS;

/// Component-wise tolerance for the `approximate_location` proximity
/// anchor (anchor #4). The intersection rings span `|x| ≤ 5`, `|y| ≤
/// 5`, `z ∈ [0, 10]` post-placement, so every region's midpoint sits
/// within `±5 mm` of `(0, 0, POST_PLACE_ORIGIN_Z)` component-wise. Set
/// to `RADIUS` (5 mm) — exactly the analytical bound; tessellation
/// chord shrinkage moves midpoints inward, never outward, so any drift
/// keeps anchors within bounds.
const LOCATION_TOL: f64 = RADIUS;

/// mesh-repair `max_reported = 100` cap (per §6.4 description). Locks
/// anchor #2 against the upper bound + drives the truncation-suffix
/// conditional.
const MAX_REPORTED: usize = 100;

fn main() -> Result<()> {
    // ─── Fixture: two interpenetrating cylinders (no boolean union) ──────
    let cylinder_a = make_cylinder(
        Point3::origin(),
        Vector3::new(1.0, 0.0, 0.0), // axis = +X
        Vector3::new(0.0, 1.0, 0.0), // perp_u = +Y
        Vector3::new(0.0, 0.0, 1.0), // perp_v = +Z (perp_u × perp_v = +X = axis)
        LENGTH,
        RADIUS,
        SEGMENTS,
    );
    let cylinder_b = make_cylinder(
        Point3::origin(),
        Vector3::new(0.0, 1.0, 0.0), // axis = +Y
        Vector3::new(0.0, 0.0, 1.0), // perp_u = +Z
        Vector3::new(1.0, 0.0, 0.0), // perp_v = +X (perp_u × perp_v = +Y = axis)
        LENGTH,
        RADIUS,
        SEGMENTS,
    );
    let assembly = place_on_build_plate(&concat(&cylinder_a, &cylinder_b));

    println!("==== mesh-printability-self-intersecting ====");
    println!();
    println!(
        "input  : 2 interpenetrating cylinders ({SEGMENTS}-segment lateral, {LENGTH} mm × {RADIUS} mm radius each)",
    );
    println!("         cylinder A axis +X, cylinder B axis +Y, both centred at origin");
    println!(
        "         {} vertices + {} triangles total ({}v + {}f per cylinder, vertex-disjoint)",
        assembly.vertices.len(),
        assembly.faces.len(),
        cylinder_a.vertices.len(),
        cylinder_a.faces.len(),
    );
    println!(
        "         post-`place_on_build_plate`: bbox z ∈ [0, {:.1}]; origin maps to (0, 0, {:.1})",
        2.0 * RADIUS,
        POST_PLACE_ORIGIN_Z,
    );
    println!(
        "config : PrinterConfig::fdm_default() — params don't affect SelfIntersecting (always Critical)"
    );
    println!();

    let config = PrinterConfig::fdm_default();
    let validation = validate_for_printing(&assembly, &config)?;

    println!("{}", validation.summary());
    println!();
    print_diagnostics(&validation);
    verify(&validation);

    // ─── Single-cylinder regression — anchor #7 ──────────────────────────
    let single = place_on_build_plate(&cylinder_a);
    let single_validation = validate_for_printing(&single, &config)?;
    println!();
    println!("---- Single-cylinder regression ----");
    println!("{}", single_validation.summary());
    assert_eq!(
        single_validation.self_intersecting.len(),
        0,
        "single convex cylinder must produce zero SelfIntersecting regions \
         (lateral cannot cross itself; adjacent pairs are skipped) — anchor #7",
    );
    let single_si_present = single_validation
        .issues
        .iter()
        .any(|i| i.issue_type == PrintIssueType::SelfIntersecting);
    assert!(
        !single_si_present,
        "single convex cylinder must surface no SelfIntersecting issue — anchor #7",
    );
    println!("OK — single-cylinder regression: 0 SelfIntersecting regions");

    // ─── Artifacts ───────────────────────────────────────────────────────
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let mesh_path = out_dir.join("mesh.ply");
    let issues_path = out_dir.join("issues.ply");
    save_ply(&assembly, &mesh_path, false)?;
    save_issue_centroids(&validation, &issues_path)?;

    println!();
    println!("artifacts:");
    println!(
        "  out/mesh.ply   : {}v, {}f (ASCII)",
        assembly.vertices.len(),
        assembly.faces.len(),
    );
    println!(
        "  out/issues.ply : {} centroid point(s) (ASCII, vertex-only; \
         {} self-intersect locations + {} overhang centroids)",
        issue_centroid_count(&validation),
        validation.self_intersecting.len(),
        validation.overhangs.len(),
    );
    println!();
    println!("OK — SelfIntersecting interpenetrating-cylinders demonstration verified");

    Ok(())
}

/// Hand-author a CCW-from-outside cylinder along an arbitrary right-
/// handed basis `(axis, perp_u, perp_v)` with `perp_u × perp_v = axis`.
///
/// **Vertex layout** (`2 + 2 × segments` total):
/// - `0`               negative-end cap centre (`center - axis × length/2`)
/// - `1`               positive-end cap centre (`center + axis × length/2`)
/// - `2..2+SEG`        negative rim ring (rim vertex `s` at azimuth
///   `θ_s = s × 2π / SEG`; position is
///   `neg_centre + RADIUS × (perp_u·cos θ + perp_v·sin θ)`)
/// - `2+SEG..2+2·SEG`  positive rim ring (same azimuth pattern at the
///   positive end)
///
/// **Face layout** (`4 × segments` total):
/// - Negative cap fan: `(neg_centre, neg_rim[s+1], neg_rim[s])` —
///   outward normal `-axis`. The reversed-pair winding is needed
///   because the cross-product `(rim[s] - centre) × (rim[s+1] -
///   centre)` points along `+axis` (the cap's interior side); flipping
///   the last two indices gives the outward normal.
/// - Positive cap fan: `(pos_centre, pos_rim[s], pos_rim[s+1])` —
///   outward normal `+axis` (no flip needed; rim azimuth advances CCW
///   when viewed from `+axis`).
/// - Lateral strip per segment `s`: two triangles
///   `(neg_rim[s], neg_rim[s+1], pos_rim[s])` and
///   `(neg_rim[s+1], pos_rim[s+1], pos_rim[s])` — radial outward
///   normals at azimuth `(s + 0.5) × 2π / SEG`. Verified by hand cross-
///   product at `s = 0` for the `(+X, +Y, +Z)` basis: the first
///   triangle's normal is `(0, +RADIUS · sin θ_1, 0) ≈ +perp_u` for
///   small `θ_1` — exactly outward radial.
fn make_cylinder(
    center: Point3<f64>,
    axis: Vector3<f64>,
    perp_u: Vector3<f64>,
    perp_v: Vector3<f64>,
    length: f64,
    radius: f64,
    segments: u32,
) -> IndexedMesh {
    let half_len = length / 2.0;
    let n = segments;
    let n_usize = n as usize;

    let mut vertices: Vec<Point3<f64>> = Vec::with_capacity(2 + 2 * n_usize);

    // Cap centres (indices 0, 1).
    let neg_centre = center + axis * (-half_len);
    let pos_centre = center + axis * half_len;
    vertices.push(neg_centre);
    vertices.push(pos_centre);

    // Rim vertices: negative ring (indices 2..2+SEG), then positive ring
    // (indices 2+SEG..2+2·SEG). Authored sequentially so the index
    // arithmetic in `neg_rim` / `pos_rim` below stays trivial.
    for end_offset in [-half_len, half_len] {
        let end_centre = center + axis * end_offset;
        for s in 0..n {
            let theta = f64::from(s) * std::f64::consts::TAU / f64::from(n);
            let rim_pt =
                end_centre + perp_u * (radius * theta.cos()) + perp_v * (radius * theta.sin());
            vertices.push(rim_pt);
        }
    }

    let mut faces: Vec<[u32; 3]> = Vec::with_capacity(4 * n_usize);

    let neg_centre_idx: u32 = 0;
    let pos_centre_idx: u32 = 1;
    let neg_rim = |s: u32| -> u32 { 2 + (s % n) };
    let pos_rim = |s: u32| -> u32 { 2 + n + (s % n) };

    for s in 0..n {
        let s_next = (s + 1) % n;
        // Negative cap (outward = -axis): reversed-pair winding.
        faces.push([neg_centre_idx, neg_rim(s_next), neg_rim(s)]);
        // Positive cap (outward = +axis): forward winding.
        faces.push([pos_centre_idx, pos_rim(s), pos_rim(s_next)]);
        // Lateral strip (outward = radial): two triangles per segment.
        faces.push([neg_rim(s), neg_rim(s_next), pos_rim(s)]);
        faces.push([neg_rim(s_next), pos_rim(s_next), pos_rim(s)]);
    }

    IndexedMesh::from_parts(vertices, faces)
}

/// Concatenate two meshes by appending B's vertices after A's and
/// offsetting B's face indices by A's vertex count. NO welding — the
/// resulting mesh has two vertex-disjoint shells. For self-intersection
/// demos this is the load-bearing construction: the §6.4 detector sees
/// non-adjacent face pairs (vertex-disjoint ⇒ no `skip_adjacent`
/// filtering) that physically interpenetrate.
fn concat(a: &IndexedMesh, b: &IndexedMesh) -> IndexedMesh {
    // `a.vertices.len()` is bounded by mesh-author input; well under
    // u32::MAX in any realistic example. Cast is safe.
    #[allow(clippy::cast_possible_truncation)]
    let offset = a.vertices.len() as u32;
    let mut vertices = a.vertices.clone();
    vertices.extend_from_slice(&b.vertices);
    let mut faces = a.faces.clone();
    faces.extend(
        b.faces
            .iter()
            .map(|f| [f[0] + offset, f[1] + offset, f[2] + offset]),
    );
    IndexedMesh::from_parts(vertices, faces)
}

/// Print region/issue diagnostics for the assembly validation. Surfaces
/// the §6.4 self-intersecting pair list (face indices + approximate
/// location), the lateral-overhang co-flag, and the `PrintIssue` list.
fn print_diagnostics(v: &PrintValidation) {
    println!("SelfIntersecting regions ({}):", v.self_intersecting.len());
    for (i, region) in v.self_intersecting.iter().enumerate() {
        println!(
            "  [{i}] face_a = {:>4}  face_b = {:>4}  approx = ({:+.4}, {:+.4}, {:+.4})",
            region.face_a,
            region.face_b,
            region.approximate_location.x,
            region.approximate_location.y,
            region.approximate_location.z,
        );
    }
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
    println!("Issues ({}):", v.issues.len());
    for issue in &v.issues {
        println!(
            "  [{:?} / {:?}] {}",
            issue.severity, issue.issue_type, issue.description,
        );
    }
}

/// Verify the seven §7.4 numerical anchors. Each `assert!` carries a
/// spec-traceable message naming the anchor and the violating data.
fn verify(v: &PrintValidation) {
    let n = v.self_intersecting.len();

    // Anchor #1 — interpenetrating cylinders flag at least four pairs.
    assert!(
        n >= 4,
        "anchor #1: interpenetrating cylinders must produce ≥ 4 \
         self-intersection pairs (got {n})",
    );

    // Anchor #2 — bounded by mesh-repair's max_reported cap. Truncation
    // suffix iff the cap is hit.
    assert!(
        n <= MAX_REPORTED,
        "anchor #2: self_intersecting.len() ({n}) must not exceed \
         mesh-repair's max_reported cap ({MAX_REPORTED})",
    );
    let truncated = v
        .issues
        .iter()
        .filter(|i| i.issue_type == PrintIssueType::SelfIntersecting)
        .any(|i| i.description.contains("search truncated"));
    if n == MAX_REPORTED {
        assert!(
            truncated,
            "anchor #2: when self_intersecting.len() == max_reported \
             ({MAX_REPORTED}), the issue description must carry the \
             truncation suffix",
        );
    } else {
        assert!(
            !truncated,
            "anchor #2: when self_intersecting.len() ({n}) < max_reported \
             ({MAX_REPORTED}), the truncation suffix must NOT be present",
        );
    }

    // Anchor #3 — canonical face_a < face_b ordering per §6.4.
    for (i, region) in v.self_intersecting.iter().enumerate() {
        assert!(
            region.face_a < region.face_b,
            "anchor #3: region [{i}] violates canonical ordering: \
             face_a = {} ≥ face_b = {}",
            region.face_a,
            region.face_b,
        );
    }

    // Anchor #4 — approximate_location within ±LOCATION_TOL of post-
    // placement origin (0, 0, POST_PLACE_ORIGIN_Z) component-wise.
    for (i, region) in v.self_intersecting.iter().enumerate() {
        let p = region.approximate_location;
        assert!(
            p.x.abs() <= LOCATION_TOL,
            "anchor #4: region [{i}] x = {:.4} outside ±{LOCATION_TOL} mm of origin",
            p.x,
        );
        assert!(
            p.y.abs() <= LOCATION_TOL,
            "anchor #4: region [{i}] y = {:.4} outside ±{LOCATION_TOL} mm of origin",
            p.y,
        );
        assert!(
            (p.z - POST_PLACE_ORIGIN_Z).abs() <= LOCATION_TOL,
            "anchor #4: region [{i}] z = {:.4} outside ±{LOCATION_TOL} mm of \
             post-placement origin z = {POST_PLACE_ORIGIN_Z:.1}",
            p.z,
        );
    }

    // Anchor #5 — every SelfIntersecting issue is Critical;
    // is_printable() is false.
    let si_total = v
        .issues
        .iter()
        .filter(|i| i.issue_type == PrintIssueType::SelfIntersecting)
        .count();
    let si_critical = v
        .issues
        .iter()
        .filter(|i| {
            i.issue_type == PrintIssueType::SelfIntersecting
                && i.severity == IssueSeverity::Critical
        })
        .count();
    assert!(
        si_total >= 1,
        "anchor #5: at least one SelfIntersecting PrintIssue must surface \
         (got {si_total})",
    );
    assert_eq!(
        si_critical, si_total,
        "anchor #5: every SelfIntersecting PrintIssue must be Critical \
         ({si_critical}/{si_total} were Critical)",
    );
    assert!(
        !v.is_printable(),
        "anchor #5: Critical SelfIntersecting must block is_printable()",
    );

    // Anchor #6 — lateral-overhang co-flag. Documented but not load-
    // bearing; the assertion locks the post-Gap-M predicate's behaviour
    // on horizontal cylinders.
    assert!(
        !v.overhangs.is_empty(),
        "anchor #6: horizontally-placed cylinders must flag ≥ 1 \
         lateral-overhang co-flag (post-Gap-M predicate; got 0)",
    );

    println!();
    println!(
        "verified: 6 fixture-side anchors (#1–#6); single-cylinder regression (#7) runs separately"
    );
}

/// Write region centroids as a vertex-only ASCII PLY. `SelfIntersectingRegion`
/// exposes `approximate_location` instead of the `.center` field that
/// other typed regions (`thin_walls`, `overhangs`, etc.) expose, so the
/// mapping is collection-specific. Aggregates self-intersect locations
/// (the load-bearing point cloud) plus the lateral-overhang co-flag
/// centroids (so `issues.ply` carries the full picture for the visuals
/// pass).
fn save_issue_centroids(v: &PrintValidation, path: &Path) -> Result<()> {
    let mut centroids: Vec<Point3<f64>> = Vec::new();
    centroids.extend(v.self_intersecting.iter().map(|r| r.approximate_location));
    centroids.extend(v.overhangs.iter().map(|r| r.center));
    let mesh = IndexedMesh::from_parts(centroids, vec![]);
    save_ply(&mesh, path, false)?;
    Ok(())
}

/// Number of region centroids written to `out/issues.ply`.
const fn issue_centroid_count(v: &PrintValidation) -> usize {
    v.self_intersecting.len() + v.overhangs.len()
}
