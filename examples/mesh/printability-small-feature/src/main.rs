//! Visual demo of the §6.5 `SmallFeature` detector (Gap J).
//!
//! Hand-authors a 30 mm solid cube on the build plate plus a tiny
//! isolated 0.2 mm × 0.2 mm hexagonal-prism burr (CAD leftover from an
//! imperfect boolean cut) sitting alongside it on the build plate.
//! The two components share **no** vertices, edges, or faces — they
//! are geometrically and topologically disjoint. The §6.5 detector
//! partitions the mesh under edge-adjacency (`partition_all_faces_into_components`),
//! evaluates each component's AABB max-extent against
//! `config.min_feature_size`, and emits one `SmallFeatureRegion` +
//! one `SmallFeature` `PrintIssue` per qualifying component. The
//! main cube (max extent 30 mm) sails past `min_feature_size = 0.8 mm`
//! and is not flagged; only the 0.2 mm burr emits a region.
//!
//! ## `SmallFeature` severity is `Warning`, not `Critical`
//!
//! `classify_small_feature_severity` (§6.5) maps `max_extent <
//! min_feature_size / 2.0` (here `0.2 < 0.4`) to `Warning`. Per §6.5
//! line 1266, small features are advisory: a tiny isolated burr is
//! a CAD-pipeline diagnostic, not a print-physics failure mode the
//! way trapped voids or thin walls are. So `SmallFeature` alone does
//! not block `is_printable()`.
//!
//! ## `ThinWall` co-flag — load-bearing observation
//!
//! Although the §7.5 spec line 1746 anticipates `is_printable() ==
//! true`, the §6.1 `ThinWall` detector (which had not yet landed when
//! the §7.5 spec was first drafted) ALSO fires on the burr — the
//! inward ray-cast from each of the burr's faces hits the opposite
//! face within `min_wall_thickness / 2 = 0.5 mm` (the prism's
//! diameter is 0.2 mm), placing each flagged face in `ThinWall`'s
//! `Critical` band. The two detectors agree from different angles:
//! `SmallFeature` sees "this component is too small overall" while
//! `ThinWall` sees "every face is closer to its opposite than
//! `min_wall_thickness` allows". Pedagogically clean — two
//! independent detectors on the same defect surface complementary
//! diagnostics, and `is_printable()` correctly returns `false`
//! because of the `ThinWall Critical` co-flag, not because of
//! `SmallFeature`.
//!
//! §7.5 anchor #8's predicted `is_printable() == true` is the only
//! spec deviation in this commit (the `ThinWall` Critical band is a
//! v0.7 → v0.8 detector addition the row #19 spec line did not
//! account for); anchors #1–#7 hold as written. The deviation is
//! surfaced in the commit body and README per
//! `feedback_explicit_deferrals`.
//!
//! ## Numerical anchors (asserted in `main`)
//!
//! Anchors per §7.5 lines 1748–1756, with anchor #8 corrected to
//! reflect the `ThinWall` co-flag:
//!
//! 1. `small_features.len() == 1` — exactly the burr; main cube
//!    (30 mm) is too big to flag.
//! 2. `region.center` within `1e-6` of `(35, 15, 0.1)`.
//! 3. `region.face_count == 24` — locked-in by 12 lateral + 6 top
//!    fan + 6 bottom fan hex-prism construction.
//! 4. `region.max_extent ∈ [0.199, 0.201]` (FP tolerance on the
//!    `0.2 mm` vertex-to-vertex hex diameter / prism height).
//! 5. `region.volume ≈ 5.196e-3 mm³` within `1e-3 max_relative` —
//!    divergence-theorem volume of a hex prism with circumradius
//!    `0.1 mm` and height `0.2 mm`: `A · h = (3√3/2) · r² · h ≈
//!    2.598e-2 · 0.2 ≈ 5.196e-3`.
//! 6. `SmallFeature` issue severity `Warning` (`0.2 < 0.8/2 = 0.4`).
//! 7. `overhangs.len() == 0` — both component bottoms are
//!    build-plate-filtered; lateral hex faces have `normal.z = 0`
//!    (vertical walls, NOT overhang).
//! 8. **CORRECTED**: `is_printable() == false` — `ThinWall` Critical
//!    co-flag on the burr blocks printability. Documented spec
//!    deviation per the module-doc comment above.
//! 9. At least one `ThinWall` `Critical` issue exists, locking the
//!    co-flag observation that drives anchor #8's correction.
//!
//! ## How to run
//!
//! ```text
//! cargo run -p example-mesh-printability-small-feature --release
//! ```
//!
//! Output written to `examples/mesh/printability-small-feature/out/`.
//! Open `mesh.ply` in `MeshLab`, `ParaView`, or `f3d` for the visuals
//! pass. The burr is barely visible at default zoom (0.2 mm vs the
//! cube's 30 mm — 1:150 scale ratio); zoom to `(35, 15, 0.1)` at
//! ≥ 100× magnification to see the 24-triangle hex prism. See the
//! README's f3d callout for the `--up +Z` recommendation.

use std::path::Path;

use anyhow::Result;
use mesh_io::save_ply;
use mesh_printability::{
    IssueSeverity, PrintIssueType, PrintValidation, PrinterConfig, validate_for_printing,
};
use mesh_types::{IndexedMesh, Point3};

// -- Geometry constants (mm) ------------------------------------------------

/// Side length of the main solid cube. The cube spans `(0,0,0)` to
/// `(CUBE_DIM, CUBE_DIM, CUBE_DIM)` so `z_min = 0` (sits on the build
/// plate; bottom face is build-plate-filtered).
const CUBE_DIM: f64 = 30.0;

/// Hex-prism circumradius (rim vertices sit on a circle of this radius).
/// AABB extent in `x`: `2·r = 0.2 mm` (vertex-to-vertex span); flat-to-
/// flat span is `r · √3 ≈ 0.173 mm`. The detector uses the AABB extent,
/// so `max_extent` reads as `2·r = 0.2 mm`.
const BURR_RADIUS: f64 = 0.1;

/// Hex-prism height (top rim at `z = BURR_HEIGHT`, bottom rim at `z = 0`).
/// `max_extent = max(2·r, h) = max(0.2, 0.2) = 0.2 mm`.
const BURR_HEIGHT: f64 = 0.2;

/// Burr center (XY): 5 mm clearance from the cube's `+X` face. The burr
/// vertically spans `z ∈ [0, BURR_HEIGHT]` (sits on the build plate).
const BURR_CX: f64 = 35.0;
const BURR_CY: f64 = 15.0;

/// Number of azimuthal segments in the hex prism. Locked at 6 — the
/// §6.5 detector's `face_count == 24` anchor ties to this resolution
/// (12 lateral + 6 top fan + 6 bottom fan = 24).
const HEX_SEGMENTS: u32 = 6;

/// Expected centroid of the burr's vertex positions: 14 unique vertices
/// (2 hub centres + 12 rim verts) average to the prism's geometric
/// midpoint via symmetry — `(BURR_CX, BURR_CY, BURR_HEIGHT/2)`.
fn expected_burr_center() -> Point3<f64> {
    Point3::new(BURR_CX, BURR_CY, BURR_HEIGHT / 2.0)
}

/// Expected hex-prism volume via divergence theorem on the closed
/// shell: `A · h` where `A = (3√3/2) · r²` is the regular-hexagon area.
/// For `r = 0.1, h = 0.2`: `A = 2.598e-2 mm², V = 5.196e-3 mm³`.
fn expected_burr_volume() -> f64 {
    let hex_area = 1.5 * 3.0_f64.sqrt() * BURR_RADIUS * BURR_RADIUS;
    hex_area * BURR_HEIGHT
}

fn main() -> Result<()> {
    // ─── Fixture: 30 mm cube + 0.2 mm hex-prism burr (vertex-disjoint) ───
    let cube = build_main_cube();
    let burr = build_hex_prism_burr();
    let assembly = concat(&cube, &burr);

    println!("==== mesh-printability-small-feature ====");
    println!();
    println!(
        "input  : 30 mm solid cube + 0.2 mm hex-prism burr (vertex-disjoint, both watertight)"
    );
    println!(
        "         cube  : {} verts, {} tris  (spans (0,0,0)–({CUBE_DIM},{CUBE_DIM},{CUBE_DIM}))",
        cube.vertices.len(),
        cube.faces.len(),
    );
    println!(
        "         burr  : {} verts, {} tris  (radius {BURR_RADIUS} mm × height {BURR_HEIGHT} mm; \
         center ({BURR_CX}, {BURR_CY}, {:.1}))",
        burr.vertices.len(),
        burr.faces.len(),
        BURR_HEIGHT / 2.0,
    );
    println!(
        "         total : {} verts, {} tris",
        assembly.vertices.len(),
        assembly.faces.len(),
    );
    println!(
        "config : PrinterConfig::fdm_default()  (min_feature_size = 0.8 mm; min_wall = 1.0 mm)"
    );
    println!();

    let config = PrinterConfig::fdm_default();
    let validation = validate_for_printing(&assembly, &config)?;

    println!("{}", validation.summary());
    println!();
    print_diagnostics(&validation);
    verify(&validation);

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
        "  out/issues.ply : {} centroid point(s) (ASCII, vertex-only)",
        issue_centroid_count(&validation),
    );
    println!();
    println!("OK — SmallFeature CAD-burr demonstration verified");

    Ok(())
}

/// Hand-author an 8-vertex 12-triangle solid cube spanning
/// `(0,0,0)–(CUBE_DIM, CUBE_DIM, CUBE_DIM)` with outward (CCW-from-
/// outside) winding. Vertex/winding layout matches `validation.rs::tests`'s
/// `create_watertight_cube` helper (the canonical workspace cube).
fn build_main_cube() -> IndexedMesh {
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0),                // 0
        Point3::new(CUBE_DIM, 0.0, 0.0),           // 1
        Point3::new(CUBE_DIM, CUBE_DIM, 0.0),      // 2
        Point3::new(0.0, CUBE_DIM, 0.0),           // 3
        Point3::new(0.0, 0.0, CUBE_DIM),           // 4
        Point3::new(CUBE_DIM, 0.0, CUBE_DIM),      // 5
        Point3::new(CUBE_DIM, CUBE_DIM, CUBE_DIM), // 6
        Point3::new(0.0, CUBE_DIM, CUBE_DIM),      // 7
    ];
    let faces = vec![
        // Bottom (-Z): outward = -Z.
        [0, 2, 1],
        [0, 3, 2],
        // Top (+Z): outward = +Z.
        [4, 5, 6],
        [4, 6, 7],
        // Front (-Y): outward = -Y.
        [0, 1, 5],
        [0, 5, 4],
        // Back (+Y): outward = +Y.
        [3, 6, 2],
        [3, 7, 6],
        // Left (-X): outward = -X.
        [0, 4, 7],
        [0, 7, 3],
        // Right (+X): outward = +X.
        [1, 2, 6],
        [1, 6, 5],
    ];
    IndexedMesh::from_parts(vertices, faces)
}

/// Hand-author a 14-vertex 24-triangle hex prism with outward winding,
/// centred at `(BURR_CX, BURR_CY)` in XY and spanning
/// `z ∈ [0, BURR_HEIGHT]`.
///
/// **Vertex layout** (14 total):
/// - `0`             top hub  (`(BURR_CX, BURR_CY, BURR_HEIGHT)`)
/// - `1`             bottom hub  (`(BURR_CX, BURR_CY, 0)`)
/// - `2..8`          top rim, 6 verts at azimuth `k · 60°` for `k = 0..6`
/// - `8..14`         bottom rim, same 6 azimuths at `z = 0`
///
/// **Face layout** (24 total, OUTWARD winding):
/// - **Top fan** (6 tris, normal `+Z`): `[top_hub, t_curr, t_next]` for
///   each `k`. `(t_curr - hub) × (t_next - hub)` evaluates to
///   `(0, 0, R² · sin(60°))` — `+Z` outward (verified by hand at `k=0`).
/// - **Bottom fan** (6 tris, normal `-Z`): `[bot_hub, b_next, b_curr]`
///   per `k` — reversed pair so `(b_next - hub) × (b_curr - hub)` flips
///   to `-Z` outward.
/// - **Lateral** (12 tris, normal radial outward): per segment, two
///   triangles `[b_curr, b_next, t_next]` + `[b_curr, t_next, t_curr]`.
///   At `k=0`: cross product `(b_next - b_curr) × (t_next - b_curr) =
///   (h · R · sin 60°, h · R · (1 - cos 60°), 0)`, projection on radial
///   midpoint `(cos 30°, sin 30°, 0)` is positive → outward.
///
/// The CCW-rim/reversed-bottom-fan/CCW-lateral combination produces a
/// consistently-wound watertight shell: each undirected edge is incident
/// on exactly two faces, and each directed edge appears in at most one
/// face. The `is_watertight_and_consistent_winding` precondition holds
/// for both the burr alone and the combined cube + burr.
fn build_hex_prism_burr() -> IndexedMesh {
    let mut vertices: Vec<Point3<f64>> = Vec::with_capacity(2 + 2 * HEX_SEGMENTS as usize);

    // Hub centres (indices 0, 1).
    vertices.push(Point3::new(BURR_CX, BURR_CY, BURR_HEIGHT)); // 0: top hub
    vertices.push(Point3::new(BURR_CX, BURR_CY, 0.0)); // 1: bottom hub

    // Top rim (indices 2..8): 6 verts at azimuth `k · 60°`, `z = BURR_HEIGHT`.
    for k in 0..HEX_SEGMENTS {
        let theta = std::f64::consts::TAU * f64::from(k) / f64::from(HEX_SEGMENTS);
        vertices.push(Point3::new(
            BURR_RADIUS.mul_add(theta.cos(), BURR_CX),
            BURR_RADIUS.mul_add(theta.sin(), BURR_CY),
            BURR_HEIGHT,
        ));
    }

    // Bottom rim (indices 8..14): same azimuths at `z = 0`.
    for k in 0..HEX_SEGMENTS {
        let theta = std::f64::consts::TAU * f64::from(k) / f64::from(HEX_SEGMENTS);
        vertices.push(Point3::new(
            BURR_RADIUS.mul_add(theta.cos(), BURR_CX),
            BURR_RADIUS.mul_add(theta.sin(), BURR_CY),
            0.0,
        ));
    }

    let mut faces: Vec<[u32; 3]> = Vec::with_capacity(4 * HEX_SEGMENTS as usize);

    let top_hub: u32 = 0;
    let bot_hub: u32 = 1;
    let t = |k: u32| -> u32 { 2 + (k % HEX_SEGMENTS) };
    let b = |k: u32| -> u32 { 2 + HEX_SEGMENTS + (k % HEX_SEGMENTS) };

    for k in 0..HEX_SEGMENTS {
        let k_next = (k + 1) % HEX_SEGMENTS;
        // Top fan: `[hub, t_curr, t_next]` — normal +Z.
        faces.push([top_hub, t(k), t(k_next)]);
        // Bottom fan: `[hub, b_next, b_curr]` — normal -Z.
        faces.push([bot_hub, b(k_next), b(k)]);
        // Lateral (radial outward): two triangles per segment.
        faces.push([b(k), b(k_next), t(k_next)]);
        faces.push([b(k), t(k_next), t(k)]);
    }

    IndexedMesh::from_parts(vertices, faces)
}

/// Concatenate two meshes by appending B's vertices after A's and
/// offsetting B's face indices by A's vertex count. NO welding — the
/// resulting mesh has two vertex-disjoint shells. The §6.5 detector
/// partitions under edge-adjacency, so vertex-disjoint shells become
/// distinct components.
fn concat(a: &IndexedMesh, b: &IndexedMesh) -> IndexedMesh {
    // `a.vertices.len()` is bounded by mesh-author input; well under
    // u32::MAX in any realistic example.
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
/// the §6.5 `SmallFeature` region (centroid + `max_extent` + volume + face
/// count) plus the `ThinWall` co-flag (driving anchor #8/#9) plus the
/// summary `PrintIssue` list.
fn print_diagnostics(v: &PrintValidation) {
    println!("SmallFeature regions ({}):", v.small_features.len());
    for (i, region) in v.small_features.iter().enumerate() {
        println!(
            "  [{i}] center=({:+.4}, {:+.4}, {:+.4})  max_extent={:.6} mm  \
             volume={:.6} mm³  face_count={}",
            region.center.x,
            region.center.y,
            region.center.z,
            region.max_extent,
            region.volume,
            region.face_count,
        );
    }
    println!("ThinWall regions ({}):", v.thin_walls.len());
    for (i, region) in v.thin_walls.iter().enumerate() {
        println!(
            "  [{i}] center=({:+.4}, {:+.4}, {:+.4})  thickness={:.6} mm  \
             area={:.6} mm²  faces={}",
            region.center.x,
            region.center.y,
            region.center.z,
            region.thickness,
            region.area,
            region.faces.len(),
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

/// Verify the §7.5 numerical anchors (#1–#7 hold as written; #8 is
/// corrected to `is_printable() == false` per the module-doc deviation
/// note; #9 is added to lock the `ThinWall` co-flag that drives #8).
fn verify(v: &PrintValidation) {
    // Anchor #1 — exactly one SmallFeature region (the burr; cube must NOT flag).
    assert_eq!(
        v.small_features.len(),
        1,
        "anchor #1: exactly one SmallFeature region expected (the burr; main cube max_extent 30 mm \
         ≫ 0.8 mm and must not flag)",
    );
    let region = &v.small_features[0];

    // Anchor #2 — region centroid within 1e-6 of (35, 15, 0.1).
    let target = expected_burr_center();
    let dx = region.center.x - target.x;
    let dy = region.center.y - target.y;
    let dz = region.center.z - target.z;
    let dist = dx.mul_add(dx, dy.mul_add(dy, dz * dz)).sqrt();
    assert!(
        dist < 1e-6,
        "anchor #2: region centroid ({:+.9}, {:+.9}, {:+.9}) deviates from \
         expected ({}, {}, {:.1}) by {:.3e} mm (> 1e-6)",
        region.center.x,
        region.center.y,
        region.center.z,
        target.x,
        target.y,
        target.z,
        dist,
    );

    // Anchor #3 — face_count == 24 (locked-in by 12 lateral + 6 top fan + 6 bottom fan).
    assert_eq!(
        region.face_count, 24,
        "anchor #3: hex-prism face_count locked at 24 (12 lateral + 6 top fan + 6 bottom fan); \
         got {}",
        region.face_count,
    );
    assert_eq!(
        region.faces.len(),
        24,
        "anchor #3: faces vector length must match face_count (24); got {}",
        region.faces.len(),
    );

    // Anchor #4 — max_extent within [0.199, 0.201].
    assert!(
        (0.199..=0.201).contains(&region.max_extent),
        "anchor #4: max_extent {:.9} mm outside expected [0.199, 0.201] (vertex-to-vertex \
         hex diameter 2·r = 0.2 mm; FP roundoff tolerated)",
        region.max_extent,
    );

    // Anchor #5 — divergence-theorem volume within 1e-3 max_relative of 5.196e-3.
    let expected_vol = expected_burr_volume();
    let rel_err = (region.volume - expected_vol).abs() / expected_vol;
    assert!(
        rel_err < 1e-3,
        "anchor #5: region.volume {:.9} mm³ deviates from expected (3√3/2)·r²·h = {:.9} mm³ \
         by relative error {:.3e} (> 1e-3)",
        region.volume,
        expected_vol,
        rel_err,
    );

    // Anchor #6 — SmallFeature issue severity Warning (0.2 < 0.8/2 = 0.4).
    let small_warning = v.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::SmallFeature && i.severity == IssueSeverity::Warning
    });
    assert!(
        small_warning,
        "anchor #6: a SmallFeature PrintIssue with Warning severity must surface for the burr \
         (0.2 < 0.8/2 = 0.4 → Warning band)",
    );

    // Anchor #7 — overhangs.len() == 0 (build-plate filter handles both
    // component bottoms; lateral hex faces are vertical walls with
    // normal.z = 0, not overhangs).
    assert_eq!(
        v.overhangs.len(),
        0,
        "anchor #7: overhangs.len() must be 0 (cube bottom + burr bottom both build-plate-filtered; \
         hex lateral faces have normal.z = 0 so are vertical walls, not overhangs); got {}",
        v.overhangs.len(),
    );

    // Anchor #8 — CORRECTED: is_printable() == false because of ThinWall co-flag.
    // The §7.5 spec line 1746 anticipated `is_printable() == true`, but the
    // §6.1 ThinWall detector (added in v0.8 row #10) fires Critical on the
    // burr's 0.2 mm extent (< min_wall_thickness/2 = 0.5 mm), correctly
    // flagging the burr from a complementary angle. Documented as the only
    // §7.5 spec deviation in this commit's body + module-doc.
    assert!(
        !v.is_printable(),
        "anchor #8 (corrected): is_printable() must be false — the ThinWall Critical co-flag on \
         the burr's 0.2 mm diameter blocks printability. SmallFeature alone is Warning, but the \
         co-flag from §6.1 ThinWall is Critical.",
    );

    // Anchor #9 — at least one ThinWall Critical issue exists (the co-flag
    // that drives anchor #8's correction).
    let thin_wall_critical = v
        .issues
        .iter()
        .any(|i| i.issue_type == PrintIssueType::ThinWall && i.severity == IssueSeverity::Critical);
    assert!(
        thin_wall_critical,
        "anchor #9: at least one ThinWall Critical issue expected (the burr's 0.2 mm extent \
         falls in ThinWall's `< min_wall/2 = 0.5 mm` Critical band)",
    );

    println!();
    println!(
        "verified: 9 anchors (#1–#7 per §7.5 + #8 corrected for ThinWall co-flag + #9 locks the \
         co-flag observation)"
    );
}

/// Write region centroids as a vertex-only ASCII PLY. Aggregates
/// `SmallFeature` centroids (the load-bearing point cloud) plus
/// `ThinWall` centroids (the co-flag, also localizing the burr) so
/// `issues.ply` carries the full picture for the visuals pass.
fn save_issue_centroids(v: &PrintValidation, path: &Path) -> Result<()> {
    let mut centroids: Vec<Point3<f64>> = Vec::new();
    centroids.extend(v.small_features.iter().map(|r| r.center));
    centroids.extend(v.thin_walls.iter().map(|r| r.center));
    let mesh = IndexedMesh::from_parts(centroids, vec![]);
    save_ply(&mesh, path, false)?;
    Ok(())
}

/// Number of region centroids written to `out/issues.ply`.
const fn issue_centroid_count(v: &PrintValidation) -> usize {
    v.small_features.len() + v.thin_walls.len()
}
