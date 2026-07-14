//! Tests for printability validation (extracted from validation.rs).

#![allow(
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::cast_sign_loss,
    clippy::panic,
    clippy::float_cmp
)]

use super::*;
use mesh_types::Point3;

fn create_cube_mesh() -> IndexedMesh {
    // Simple cube mesh (not watertight for testing)
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
        Point3::new(0.0, 0.0, 10.0),
        Point3::new(10.0, 0.0, 10.0),
        Point3::new(10.0, 10.0, 10.0),
        Point3::new(0.0, 10.0, 10.0),
    ];

    // Bottom and top faces only (not complete cube)
    let faces = vec![
        [0, 1, 2],
        [0, 2, 3], // Bottom
        [4, 6, 5],
        [4, 7, 6], // Top
    ];

    IndexedMesh::from_parts(vertices, faces)
}

fn create_watertight_cube() -> IndexedMesh {
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
        Point3::new(0.0, 0.0, 10.0),
        Point3::new(10.0, 0.0, 10.0),
        Point3::new(10.0, 10.0, 10.0),
        Point3::new(0.0, 10.0, 10.0),
    ];

    // All 6 faces (12 triangles)
    let faces = vec![
        // Bottom (Z=0)
        [0, 2, 1],
        [0, 3, 2],
        // Top (Z=10)
        [4, 5, 6],
        [4, 6, 7],
        // Front (Y=0)
        [0, 1, 5],
        [0, 5, 4],
        // Back (Y=10)
        [3, 6, 2],
        [3, 7, 6],
        // Left (X=0)
        [0, 4, 7],
        [0, 7, 3],
        // Right (X=10)
        [1, 2, 6],
        [1, 6, 5],
    ];

    IndexedMesh::from_parts(vertices, faces)
}

#[test]
fn test_empty_mesh_error() {
    let mesh = IndexedMesh::new();
    let config = PrinterConfig::fdm_default();
    let result = validate_for_printing(&mesh, &config);
    assert!(matches!(result, Err(PrintabilityError::EmptyMesh)));
}

#[test]
fn test_no_faces_error() {
    let mut mesh = IndexedMesh::new();
    mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
    let config = PrinterConfig::fdm_default();
    let result = validate_for_printing(&mesh, &config);
    assert!(matches!(result, Err(PrintabilityError::NoFaces)));
}

// The five `validate_for_printing(...).expect("Should succeed")` calls in the
// tests below operate on fixtures constructed inline (no I/O, no fallible
// setup): an `expect` failure here would indicate a regression in the
// detector itself, not a malformed fixture. Each site is annotated
// individually so the lint stays active for any future statements added
// inside these test bodies.
#[test]
fn test_build_volume_check() {
    let mesh = create_cube_mesh();
    let config = PrinterConfig::fdm_default().with_build_volume(5.0, 5.0, 5.0);

    let result = validate_for_printing(&mesh, &config).expect("Should succeed");
    assert!(
        result
            .issues
            .iter()
            .any(|i| i.issue_type == PrintIssueType::ExceedsBuildVolume)
    );
    assert!(!result.is_printable());
}

#[test]
fn test_build_volume_ok() {
    let mesh = create_watertight_cube();
    let config = PrinterConfig::fdm_default().with_build_volume(100.0, 100.0, 100.0);

    let result = validate_for_printing(&mesh, &config).expect("Should succeed");
    assert!(
        !result
            .issues
            .iter()
            .any(|i| i.issue_type == PrintIssueType::ExceedsBuildVolume)
    );
}

#[test]
fn test_not_watertight_detection() {
    let mesh = create_cube_mesh(); // Incomplete cube
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config).expect("Should succeed");
    assert!(
        result
            .issues
            .iter()
            .any(|i| i.issue_type == PrintIssueType::NotWatertight)
    );
}

#[test]
fn test_watertight_mesh() {
    let mesh = create_watertight_cube();
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config).expect("Should succeed");
    assert!(
        !result
            .issues
            .iter()
            .any(|i| i.issue_type == PrintIssueType::NotWatertight)
    );
}

#[test]
fn test_validation_summary() {
    let config = PrinterConfig::fdm_default();
    let mut validation = PrintValidation::new(config);

    assert_eq!(validation.summary(), "Mesh is ready for printing");

    validation.issues.push(PrintIssue::new(
        PrintIssueType::ThinWall,
        IssueSeverity::Warning,
        "test",
    ));

    assert!(validation.summary().contains("Printable with issues"));

    validation.issues.push(PrintIssue::new(
        PrintIssueType::NotWatertight,
        IssueSeverity::Critical,
        "test",
    ));

    assert!(validation.summary().contains("Not printable"));
}

#[test]
fn test_issue_counts() {
    let config = PrinterConfig::fdm_default();
    let mut validation = PrintValidation::new(config);

    validation.issues.push(PrintIssue::new(
        PrintIssueType::ThinWall,
        IssueSeverity::Warning,
        "",
    ));
    validation.issues.push(PrintIssue::new(
        PrintIssueType::ThinWall,
        IssueSeverity::Warning,
        "",
    ));
    validation.issues.push(PrintIssue::new(
        PrintIssueType::NotWatertight,
        IssueSeverity::Critical,
        "",
    ));
    validation.issues.push(PrintIssue::new(
        PrintIssueType::Other,
        IssueSeverity::Info,
        "",
    ));

    assert_eq!(validation.critical_count(), 1);
    assert_eq!(validation.warning_count(), 2);
    assert!(!validation.is_printable());
}

#[test]
fn test_sls_no_overhang_check() {
    let mesh = create_watertight_cube();
    let config = PrinterConfig::sls_default();

    let result = validate_for_printing(&mesh, &config).expect("Should succeed");
    // SLS doesn't check overhangs
    assert!(
        !result
            .issues
            .iter()
            .any(|i| i.issue_type == PrintIssueType::ExcessiveOverhang)
    );
}

#[test]
fn test_support_volume_calculation() {
    let config = PrinterConfig::fdm_default();
    let mut validation = PrintValidation::new(config);

    validation
        .support_regions
        .push(SupportRegion::new(Point3::new(0.0, 0.0, 0.0), 100.0, 10.0));
    validation
        .support_regions
        .push(SupportRegion::new(Point3::new(0.0, 0.0, 0.0), 200.0, 20.0));

    assert!((validation.total_support_volume() - 300.0).abs() < f64::EPSILON);
}

// ---- §5.9 Gap M overhang predicate tests ----------------------------
//
// Each `validate_for_printing(...).expect(...)` site below carries an
// explicit per-site `#[allow(clippy::expect_used)]`. The fixtures are
// hand-built inline (no I/O, no fallible setup): an `expect` failure
// would indicate a regression in the detector itself, not a malformed
// fixture. The shared `panic!`-via-`let-else` pattern was avoided
// because the workspace lints `clippy::panic` at warn (which becomes
// an error under the `-D warnings` clippy gate), and the existing
// `validation.rs::tests` already uses the same per-site `expect_used`
// pattern as of commit #1b.
//
// TODO(commit #8 / Gap L §5.6): land
// `test_overhang_borderline_via_y_up_orientation` here. It exercises
// `PrinterConfig::with_build_up_direction(Vector3::new(0, 1, 0))`,
// which lands with Gap L (commit #8). The test slot is reserved at
// §5.9 of the v0.8 fix arc spec; the body lands once the API exists.
// Keeping the 9 §5.9 tests here in commit #2 is achieved by splitting
// the FP-fragile `test_overhang_45deg_tilt_borderline_not_flagged`
// into a `just_below_45deg_not_flagged` / `just_above_45deg_flagged`
// pair (1° margin on either side keeps both bit-stable across f64
// rounding of the strict-greater-than boundary).

/// Build a fixture for Gap-M unit tests: a top-facing ground-anchor
/// triangle at z=0 and a test face at z=`z_offset` tilted by
/// `beta_rad` from horizontal in the `+Y` / `-Z` plane. The test face
/// has normal `(0, cos(beta), -sin(beta))`, so it points
/// downward when `beta` > 0 (with `overhang_angle = beta`), is
/// horizontal at `beta` = 0 (vertical wall, `dot` = 0), and points
/// upward when `beta` < 0. The anchor at z=0 keeps
/// `mesh_min_along_up` < `face_min_along_up` so the build-plate
/// filter does not block the predicate's flag/not-flag decision for
/// any `beta` in (-π/2, π/2).
fn make_overhang_fixture(beta_rad: f64, z_offset: f64) -> IndexedMesh {
    let vertices = vec![
        // Ground anchor at z=0 (top-facing, normal = (0, 0, +1)):
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        // Test face at z_offset, tilted by beta in the Y-Z plane:
        Point3::new(0.0, 0.0, z_offset),
        Point3::new(1.0, 0.0, z_offset),
        Point3::new(0.0, beta_rad.sin(), z_offset + beta_rad.cos()),
    ];
    let faces = vec![[0, 1, 2], [3, 5, 4]];
    IndexedMesh::from_parts(vertices, faces)
}

#[test]
fn test_overhang_roof_flagged() {
    // Pure roof: face normal = (0, 0, -1), overhang_angle = 90°.
    // Anchor at z=0 ensures the build-plate filter does not apply.
    let mesh = make_overhang_fixture(std::f64::consts::FRAC_PI_2, 5.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the roof fixture");

    assert_eq!(
        result.overhangs.len(),
        1,
        "pure roof (overhang_angle = 90°) should flag under FDM max=45°"
    );
}

#[test]
fn test_overhang_vertical_wall_not_flagged() {
    // Vertical wall: face normal horizontal, dot = 0,
    // overhang_angle = 0°. Strict-greater-than check rejects.
    let mesh = make_overhang_fixture(0.0, 5.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the vertical-wall fixture");

    assert_eq!(
        result.overhangs.len(),
        0,
        "vertical wall (overhang_angle = 0°) must not flag under any positive threshold"
    );
}

#[test]
fn test_overhang_top_face_not_flagged() {
    // Top-facing face: normal = (0, 0, +1), dot = +1,
    // overhang_angle = -90°. Predicate rejects.
    let mesh = make_overhang_fixture(-std::f64::consts::FRAC_PI_2, 5.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the top-face fixture");

    assert_eq!(
        result.overhangs.len(),
        0,
        "top-facing face (overhang_angle = -90°) must not flag"
    );
}

// The §5.9 boundary-case test was authored as a single test at exactly
// 45°, but the f64 subtraction `acos(dot) - FRAC_PI_2` carries ~2 ULP
// of upward rounding at exactly 45° tilt: a hand-rolled exact-45°
// fixture would actually flag, contradicting the test's intent. The
// pair below brackets the threshold at 44° / 46° (1° margin = ~17 orders
// of magnitude above f64 ULP), so both halves are bit-stable across
// platforms and bound the implementation's flag boundary in (44°, 46°).
// The strict-greater-than convention itself is locked in by the
// predicate's `>` operator and called out in the predicate's
// doc-comment in `check_overhangs`.

#[test]
fn test_overhang_just_below_45deg_not_flagged() {
    // Tilt 44° (just below the FDM max=45° threshold).
    let mesh = make_overhang_fixture(44.0_f64.to_radians(), 5.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the 44° fixture");

    assert_eq!(
        result.overhangs.len(),
        0,
        "44° tilt (just below max=45°) must not flag"
    );
}

#[test]
fn test_overhang_just_above_45deg_flagged() {
    // Tilt 46° (just above the FDM max=45° threshold).
    let mesh = make_overhang_fixture(46.0_f64.to_radians(), 5.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the 46° fixture");

    assert_eq!(
        result.overhangs.len(),
        1,
        "46° tilt (just above max=45°) should flag"
    );
}

#[test]
fn test_overhang_60deg_tilt_flagged() {
    // 60° tilt: well above 45° threshold, robust to FP drift.
    let mesh = make_overhang_fixture(60.0_f64.to_radians(), 5.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the 60° fixture");

    assert_eq!(result.overhangs.len(), 1, "60° tilt should flag");
}

#[test]
fn test_overhang_30deg_tilt_not_flagged() {
    // 30° tilt: well below 45° threshold, robust to FP drift.
    let mesh = make_overhang_fixture(30.0_f64.to_radians(), 5.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the 30° fixture");

    assert_eq!(result.overhangs.len(), 0, "30° tilt must not flag");
}

#[test]
fn test_overhang_build_plate_face_not_flagged() {
    // Watertight cube on the build plate (z ∈ [0, 10]). The cube's
    // bottom face has overhang_angle = 90° but face_min == mesh_min
    // → build-plate filter applies → not flagged.
    let mesh = create_watertight_cube();
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the cube-on-plate fixture");

    assert_eq!(
        result.overhangs.len(),
        0,
        "cube-on-plate bottom must be filtered by the build-plate check"
    );
}

#[test]
fn test_overhang_suspended_roof_flagged() {
    // Locks in mesh-min-relativity of the build-plate filter via a
    // contrast pair: the SAME roof face flags or does not flag based
    // purely on whether a separate ground anchor is present at z=0.
    let config = PrinterConfig::fdm_default();

    // Variant A: roof alone at z=20, no anchor. mesh_min == face_min
    // == 20 → build-plate filter applies → not flagged.
    let mesh_no_anchor = IndexedMesh::from_parts(
        vec![
            Point3::new(0.0, 0.0, 20.0),
            Point3::new(0.0, 1.0, 20.0),
            Point3::new(1.0, 0.0, 20.0),
        ],
        vec![[0, 1, 2]],
    );
    let no_anchor = validate_for_printing(&mesh_no_anchor, &config)
        .expect("validation should succeed for the no-anchor variant");
    assert_eq!(
        no_anchor.overhangs.len(),
        0,
        "roof alone (mesh_min == face_min) is filtered as build-plate"
    );

    // Variant B: same roof + small ground anchor at z=0. mesh_min = 0,
    // face_min = 20. (20 - 0) ≫ EPS_GEOMETRIC → not filtered → flagged.
    let mesh_anchored = IndexedMesh::from_parts(
        vec![
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(-2.0, -1.0, 0.0),
            Point3::new(-1.5, -2.0, 0.0),
            Point3::new(0.0, 0.0, 20.0),
            Point3::new(0.0, 1.0, 20.0),
            Point3::new(1.0, 0.0, 20.0),
        ],
        vec![[0, 1, 2], [3, 4, 5]],
    );
    let anchored = validate_for_printing(&mesh_anchored, &config)
        .expect("validation should succeed for the anchored variant");
    assert_eq!(
        anchored.overhangs.len(),
        1,
        "anchored roof (mesh_min = 0, face_min = 20) flags — filter is mesh-min-relative, not face-z-absolute"
    );
}

// ---- §5.2 Gap B max-angle tracking tests ---------------------------
//
// Lock in the post-§5.2 semantic: `OverhangRegion.angle` is the actual
// maximum of `overhang_angle` (in degrees) across the flagged faces.
// Pre-Gap-B v0.7 reported `config.max_overhang_angle + 10.0` (a
// geometry-independent constant); §5.2 replaces this with
// `max_overhang_angle_rad.to_degrees()`. Composes with Gap M (§5.9):
// post-v0.8 `overhang_angle ∈ [0°, 90°]` for downward-facing flagged
// faces. Tolerance epsilon = 1e-6 per §4.5 / §5.2 acceptance.

#[test]
fn test_overhang_angle_tracks_max() {
    // Single-face fixture at β = 60° tilt: per the
    // `make_overhang_fixture` contract, the test face has
    // `overhang_angle = β` (face normal = (0, cos β, -sin β),
    // dot = -sin β, overhang_angle = acos(-sin β) - π/2 = β).
    let mesh = make_overhang_fixture(60.0_f64.to_radians(), 5.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the 60° fixture");

    assert_eq!(result.overhangs.len(), 1, "60° tilt must flag");
    approx::assert_relative_eq!(result.overhangs[0].angle, 60.0, epsilon = 1e-6);
}

#[test]
fn test_overhang_angle_uses_steepest_face() {
    // Two test faces at β = 50° and β = 70°, plus a ground anchor at
    // z = 0. Both flag under FDM max = 45°; the reported angle is the
    // max (~70°). Critical lock-in for the "max-of" semantic: pre-Gap-B
    // v0.7 would report 55° here regardless of geometry.
    let beta1 = 50.0_f64.to_radians();
    let beta2 = 70.0_f64.to_radians();
    let z_offset = 5.0_f64;
    let vertices = vec![
        // Ground anchor at z = 0 (top-facing, not flagged):
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
        // Test face 1 at β = 50°, x = 0:
        Point3::new(0.0, 0.0, z_offset),
        Point3::new(1.0, 0.0, z_offset),
        Point3::new(0.0, beta1.sin(), z_offset + beta1.cos()),
        // Test face 2 at β = 70°, x = 5:
        Point3::new(5.0, 0.0, z_offset),
        Point3::new(6.0, 0.0, z_offset),
        Point3::new(5.0, beta2.sin(), z_offset + beta2.cos()),
    ];
    let faces = vec![[0, 1, 2], [3, 5, 4], [6, 8, 7]];
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the two-face fixture");

    // Post-Gap-D: the two flagged faces share no edge or vertex
    // (one at x = 0, one at x = 5), so they split into two disjoint
    // regions. Each region has one face, so its `angle` field equals
    // that single face's overhang angle. Assertions use min/max over
    // the regions to be independent of HashMap iteration order in
    // the partition output.
    assert_eq!(
        result.overhangs.len(),
        2,
        "two flagged faces with no shared edge or vertex split into two regions post-Gap-D"
    );
    let max_angle = result
        .overhangs
        .iter()
        .map(|r| r.angle)
        .fold(0.0_f64, f64::max);
    let min_angle = result
        .overhangs
        .iter()
        .map(|r| r.angle)
        .fold(f64::INFINITY, f64::min);
    approx::assert_relative_eq!(max_angle, 70.0, epsilon = 1e-6);
    approx::assert_relative_eq!(min_angle, 50.0, epsilon = 1e-6);
}

#[test]
fn test_overhang_no_overhang_no_region() {
    // Vertical-wall fixture (β = 0): overhang_angle = 0, no faces
    // flag. The `overhang_faces.is_empty()` guard prevents the
    // zero-init `max_overhang_angle_rad` from leaking out as a region
    // with `angle = 0` (Gap B risk row 3 lock-in, §8.1).
    let mesh = make_overhang_fixture(0.0, 5.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the vertical-wall fixture");

    assert!(
        result.overhangs.is_empty(),
        "no flagged faces → no region (Gap B empty-case guard)"
    );
}

// ---- §5.3 Gap D connected-region partition tests --------------------
//
// Lock in the post-§5.3 semantic: `check_overhangs` partitions
// flagged faces into edge-connected components via the
// `build_edge_to_faces` helper (commit #4). Each component emits
// one `OverhangRegion` (mean of face centroids, max overhang_angle
// in degrees, summed area) and one matching `SupportRegion`
// (volume = component_area × 5.0, height = 10.0 mm placeholder).
// Pre-Gap-D v0.7 lumped ALL flagged faces into one region using
// `overhang_faces[0]`'s centroid (geometry-blind).
//
// Adjacency contract: two flagged faces share a component IFF they
// share a manifold edge (incident on exactly 2 faces). Non-manifold
// edges (>2 incident faces) and open edges (1 incident face) do
// NOT contribute adjacency. Faces sharing only a vertex are NOT
// adjacent.
//
// Invariants:
//   1. `validation.support_regions.len() == validation.overhangs.len()`
//      preserved at component granularity.
//   2. Σ(component.area) == total_overhang_area pre-Gap-D
//      (partition is exhaustive — every flagged face lands in
//      exactly one component).
//   3. Components emerge in min-face-idx order; faces within each
//      component sorted ascending. Independent of HashMap
//      iteration order.

#[test]
fn test_overhang_single_connected_region() {
    // Two coplanar roof triangles sharing the diagonal edge (3, 5):
    // both have downward normal (0, 0, -1), both flag, and they
    // form a single connected component → one OverhangRegion.
    let mesh = IndexedMesh::from_parts(
        vec![
            // Anchor at z=0 (top-facing, not flagged):
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
            // Two roof triangles forming a 1×1 square at z=5:
            Point3::new(0.0, 0.0, 5.0), // 3
            Point3::new(0.0, 1.0, 5.0), // 4
            Point3::new(1.0, 0.0, 5.0), // 5
            Point3::new(1.0, 1.0, 5.0), // 6
        ],
        vec![[0, 1, 2], [3, 4, 5], [4, 6, 5]],
    );
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the connected-roof fixture");

    assert_eq!(
        result.overhangs.len(),
        1,
        "two flagged faces sharing a manifold edge form one connected region"
    );
    assert_eq!(
        result.overhangs[0].faces.len(),
        2,
        "the single region contains both flagged faces"
    );
    assert_eq!(
        result.support_regions.len(),
        result.overhangs.len(),
        "1:1 support/overhang invariant preserved at component granularity"
    );
}

#[test]
fn test_overhang_two_disjoint_regions() {
    // Two roof triangles at separated x positions sharing no
    // vertices (geometric gap). Both flag. Faces are disjoint:
    // every face index appears in exactly one region's
    // `faces` list (partition is exhaustive and non-overlapping).
    let mesh = IndexedMesh::from_parts(
        vec![
            // Anchor at z=0:
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(20.0, 0.0, 0.0),
            Point3::new(0.0, 5.0, 0.0),
            // Roof 1 at x ∈ [0, 1]:
            Point3::new(0.0, 0.0, 5.0),
            Point3::new(0.0, 1.0, 5.0),
            Point3::new(1.0, 0.0, 5.0),
            // Roof 2 at x ∈ [10, 11] (no shared vertices):
            Point3::new(10.0, 0.0, 5.0),
            Point3::new(10.0, 1.0, 5.0),
            Point3::new(11.0, 0.0, 5.0),
        ],
        vec![[0, 1, 2], [3, 4, 5], [6, 7, 8]],
    );
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the two-disjoint-roofs fixture");

    assert_eq!(
        result.overhangs.len(),
        2,
        "two flagged faces with no shared geometry form two regions"
    );
    // Face-disjoint membership: union of region face lists has the
    // same length as the sum of region face counts (no duplicates).
    let mut all_faces: Vec<u32> = result
        .overhangs
        .iter()
        .flat_map(|r| r.faces.iter().copied())
        .collect();
    let total_count = all_faces.len();
    all_faces.sort_unstable();
    all_faces.dedup();
    assert_eq!(
        all_faces.len(),
        total_count,
        "regions are face-disjoint (partition is non-overlapping)"
    );
    assert_eq!(
        result.support_regions.len(),
        result.overhangs.len(),
        "1:1 support/overhang invariant preserved at component granularity"
    );
}

#[test]
fn test_overhang_region_centroid_is_component_centroid() {
    // Single triangular overhang at known position.
    // Triangle vertices (1, 0, 5), (1, 1, 5), (2, 0, 5) →
    // analytical centroid ((1+1+2)/3, (0+1+0)/3, (5+5+5)/3)
    //                   = (4/3, 1/3, 5).
    // Single-face component → component centroid = face centroid.
    let mesh = IndexedMesh::from_parts(
        vec![
            // Anchor at z=0:
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
            // Single overhang at known position:
            Point3::new(1.0, 0.0, 5.0),
            Point3::new(1.0, 1.0, 5.0),
            Point3::new(2.0, 0.0, 5.0),
        ],
        vec![[0, 1, 2], [3, 4, 5]],
    );
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the single-face-overhang fixture");

    assert_eq!(result.overhangs.len(), 1);
    let center = result.overhangs[0].center;
    approx::assert_relative_eq!(center.x, 4.0_f64 / 3.0, epsilon = 1e-6);
    approx::assert_relative_eq!(center.y, 1.0_f64 / 3.0, epsilon = 1e-6);
    approx::assert_relative_eq!(center.z, 5.0, epsilon = 1e-6);
}

#[test]
fn test_overhang_no_overhangs() {
    // Watertight cube on the build plate: bottom faces are
    // filtered by the M.2 build-plate filter; all other faces are
    // top-facing or vertical. Result: zero overhangs and zero
    // support regions. Locks the §5.3 acceptance bullet
    // "no overhangs → no support_regions" at component
    // granularity.
    let mesh = create_watertight_cube();
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the cube-on-plate fixture");

    assert!(
        result.overhangs.is_empty(),
        "no overhangs after build-plate filter on cube-on-plate"
    );
    assert!(
        result.support_regions.is_empty(),
        "no support regions when no overhangs (1:1 invariant at zero)"
    );
}

#[test]
fn test_overhang_face_adjacency_via_shared_edge() {
    // Locks the manifold-edge adjacency contract: two flagged
    // faces share a component IFF they share a manifold edge.
    // Vertex-only sharing produces disjoint components. Two
    // contrast variants in one test for direct comparison.
    let config = PrinterConfig::fdm_default();

    // Variant A — edge-shared: two roof triangles share edge (4, 5).
    // Both have downward normal (0, 0, -1) → both flag → ONE region.
    let mesh_edge_shared = IndexedMesh::from_parts(
        vec![
            // Anchor at z=0:
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
            // Two coplanar roofs at z=5:
            Point3::new(0.0, 0.0, 5.0), // 3
            Point3::new(0.0, 1.0, 5.0), // 4
            Point3::new(1.0, 0.0, 5.0), // 5
            Point3::new(1.0, 1.0, 5.0), // 6
        ],
        vec![[0, 1, 2], [3, 4, 5], [4, 6, 5]],
    );
    let edge_shared = validate_for_printing(&mesh_edge_shared, &config)
        .expect("validation should succeed for the edge-shared variant");
    assert_eq!(
        edge_shared.overhangs.len(),
        1,
        "two faces sharing a manifold edge form one connected region"
    );

    // Variant B — vertex-only: the second roof shares only vertex 5
    // with the first (no shared edge). Both flag → TWO regions.
    let mesh_vertex_only = IndexedMesh::from_parts(
        vec![
            // Anchor at z=0:
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(0.0, 10.0, 0.0),
            // Roof 1:
            Point3::new(0.0, 0.0, 5.0), // 3
            Point3::new(0.0, 1.0, 5.0), // 4
            Point3::new(1.0, 0.0, 5.0), // 5 (shared with roof 2)
            // Roof 2 — shares only vertex 5:
            Point3::new(1.0, 1.0, 5.0), // 6
            Point3::new(2.0, 0.0, 5.0), // 7
        ],
        vec![[0, 1, 2], [3, 4, 5], [5, 6, 7]],
    );
    let vertex_only = validate_for_printing(&mesh_vertex_only, &config)
        .expect("validation should succeed for the vertex-only variant");
    assert_eq!(
        vertex_only.overhangs.len(),
        2,
        "two faces sharing only a vertex form two disjoint regions"
    );
}

// ---- §5.4 Gap E severity classifier tests ---------------------------
//
// Lock in the post-§5.4 semantic: ExcessiveOverhang severity is
// classified by the per-region max overhang angle relative to
// `config.max_overhang_angle` per the §4.3 angle bands:
//
//   - observed > threshold + 30°               → Critical
//   - threshold + 15° < observed ≤ threshold + 30°   → Warning
//   - threshold       < observed ≤ threshold + 15°   → Info
//
// Pre-Gap-E v0.7 used an area-based ternary
// (`if total_area > 1000.0 { Warning } else { Info }`), capping
// severity at Warning even for near-90° roofs and silently allowing
// `is_printable()` to return true on slicer-rejected meshes.
// Post-v0.8 a Critical overhang flips `is_printable()` to false,
// matching FDM-slicer convention (PrusaSlicer / Cura).
//
// FP-fragility: the test fixtures bracket each band edge by ≥5° to
// stay clear of `make_overhang_fixture`'s acos-based ULP drift at
// exact boundaries (see §8.1 Gap E risk row 2). Per the
// `make_overhang_fixture` contract (§5.9), the test face's
// `overhang_angle` equals β by construction, so an 80° β produces
// an 80° flagged-face angle ± a few ULPs — well within any band's
// interior at the chosen 50° / 65° / 80° fixture set.

#[test]
fn test_overhang_severity_critical_at_steep() {
    // 80° on FDM threshold 45°: 80 > 75 (= 45 + 30) → Critical.
    let mesh = make_overhang_fixture(80.0_f64.to_radians(), 5.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the 80° fixture");

    let overhang_issue = result
        .issues
        .iter()
        .find(|i| i.issue_type == PrintIssueType::ExcessiveOverhang)
        .expect("80° overhang must produce an ExcessiveOverhang issue");
    assert_eq!(overhang_issue.severity, IssueSeverity::Critical);
}

#[test]
fn test_overhang_severity_warning_at_medium() {
    // 65° on FDM threshold 45°: 60 (= 45 + 15) < 65 ≤ 75 → Warning.
    let mesh = make_overhang_fixture(65.0_f64.to_radians(), 5.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the 65° fixture");

    let overhang_issue = result
        .issues
        .iter()
        .find(|i| i.issue_type == PrintIssueType::ExcessiveOverhang)
        .expect("65° overhang must produce an ExcessiveOverhang issue");
    assert_eq!(overhang_issue.severity, IssueSeverity::Warning);
}

#[test]
fn test_overhang_severity_info_at_borderline() {
    // 50° on FDM threshold 45°: 45 < 50 ≤ 60 (= 45 + 15) → Info.
    let mesh = make_overhang_fixture(50.0_f64.to_radians(), 5.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the 50° fixture");

    let overhang_issue = result
        .issues
        .iter()
        .find(|i| i.issue_type == PrintIssueType::ExcessiveOverhang)
        .expect("50° overhang must produce an ExcessiveOverhang issue");
    assert_eq!(overhang_issue.severity, IssueSeverity::Info);
}

/// Build a closed (watertight, manifold) overhang fixture for the
/// §5.4 `is_printable()` polarity tests below. Cross-section is an
/// "L" extruded along +y; the L's underside-of-arm segment is
/// tilted to land at `target_overhang_angle_deg`
/// (tilt-from-vertical, FDM convention). The L's leg sits on the
/// build plate (z=0); the slanted underside of the arm hovers
/// above (`face_min_along_up` = 1.0 > `mesh_min` = 0.0), so the
/// build-plate filter does not block the overhang flag. The
/// remaining five side walls are vertical, horizontal-top, or on
/// the build plate, so the only flagged region is the slanted
/// underside (2 triangles forming one connected component).
///
/// Construction rationale: a *closed* mesh is required for these
/// tests because the simpler 2-triangle `make_overhang_fixture`
/// emits a `NotWatertight` Critical issue (six open edges) that
/// would taint `is_printable()` independently of the overhang
/// severity under test.
fn make_closed_overhang_fixture(target_overhang_angle_deg: f64) -> IndexedMesh {
    let theta_rad = target_overhang_angle_deg.to_radians();
    let sin_t = theta_rad.sin();
    let cos_t = theta_rad.cos();

    let vertices = vec![
        // Front cross-section (y = 0):
        Point3::new(0.0, 0.0, 0.0), // 0  P1 — leg bottom-left
        Point3::new(1.0, 0.0, 0.0), // 1  P2 — leg bottom-right
        Point3::new(1.0, 0.0, 1.0), // 2  P3 — leg top-right (start of overhang)
        Point3::new(1.0 + sin_t, 0.0, 1.0 + cos_t), // 3  P4 — overhang outer end
        Point3::new(1.0 + sin_t, 0.0, 2.0 + cos_t), // 4  P5 — arm top-right
        Point3::new(0.0, 0.0, 2.0 + cos_t), // 5  P6 — arm top-left
        // Back cross-section (y = 1):
        Point3::new(0.0, 1.0, 0.0),                 // 6  P1'
        Point3::new(1.0, 1.0, 0.0),                 // 7  P2'
        Point3::new(1.0, 1.0, 1.0),                 // 8  P3'
        Point3::new(1.0 + sin_t, 1.0, 1.0 + cos_t), // 9  P4'
        Point3::new(1.0 + sin_t, 1.0, 2.0 + cos_t), // 10 P5'
        Point3::new(0.0, 1.0, 2.0 + cos_t),         // 11 P6'
    ];
    let faces = vec![
        // Bottom of leg (outward -z; build-plate filtered):
        [0, 6, 7],
        [0, 7, 1],
        // Right side of leg (outward +x; vertical wall, not flagged):
        [1, 7, 8],
        [1, 8, 2],
        // Slanted underside of arm — the OVERHANG
        // (outward (cos θ, 0, -sin θ); flagged at angle θ):
        [2, 8, 9],
        [2, 9, 3],
        // Right side of arm (outward +x; vertical wall):
        [3, 9, 10],
        [3, 10, 4],
        // Top of arm (outward +z; top face):
        [4, 10, 11],
        [4, 11, 5],
        // Left side (outward -x; vertical wall):
        [5, 11, 6],
        [5, 6, 0],
        // Front face (outward -y; hexagon fanned from P6 = vertex
        // 5). Fanning from P1 (the original choice) overlaps the
        // pair `[P1,P2,P3]` and `[P1,P3,P4]` because P4 sits at a
        // SMALLER polar angle than P3 from P1's perspective on the
        // L-shape (concave reflex at P3) — Gap I's
        // `check_self_intersecting` correctly flagged the bug.
        // Fanning from convex P6 in the monotonic angular order
        // P5 → P4 → P3 → P2 → P1 produces non-overlapping
        // triangles with consistent `-y` outward normals.
        [5, 3, 4],
        [5, 2, 3],
        [5, 1, 2],
        [5, 0, 1],
        // Back face (outward +y; hexagon fanned from P6' = vertex
        // 11) — same ear-clipping rationale, traversal reversed
        // to flip the normal to `+y`.
        [11, 10, 9],
        [11, 9, 8],
        [11, 8, 7],
        [11, 7, 6],
    ];
    IndexedMesh::from_parts(vertices, faces)
}

#[test]
fn test_is_printable_blocks_critical_overhang() {
    // Gap E primary regression: an 80° single-region overhang on
    // FDM threshold 45° must flip `is_printable()` to false. The
    // closed L-prism keeps the only Critical-issue surface to the
    // overhang itself (avoids the `NotWatertight` Critical that
    // would taint the polarity check). Pre-Gap-E v0.7's area-based
    // heuristic capped severity at Warning for small-area faces,
    // so `is_printable()` was true even on near-roof overhangs —
    // the bug Gap E fixes.
    let mesh = make_closed_overhang_fixture(80.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the closed 80° fixture");

    assert!(
        !result.is_printable(),
        "80° overhang on 45° FDM threshold must be Critical and block is_printable()"
    );
}

#[test]
fn test_is_printable_allows_borderline_overhang() {
    // 50° single-region overhang on FDM threshold 45°: Info
    // severity, does not block `is_printable()` (only Critical
    // does). Pairs with `test_is_printable_blocks_critical_overhang`
    // to lock the band-to-printable mapping at both polarities;
    // the closed L-prism keeps `NotWatertight` from clouding the
    // result.
    let mesh = make_closed_overhang_fixture(50.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the closed 50° fixture");

    assert!(
        result.is_printable(),
        "50° overhang on 45° FDM threshold is Info; does not block is_printable()"
    );
}

// §5.5 Gap F — directed-edge winding-orientation cluster.
//
// `check_basic_manifold` runs two side-by-side passes: the undirected
// edge-sharing pass (existing v0.7 logic) and the directed-edge pass
// (Gap F). Both push under `PrintIssueType::NonManifold` with distinct
// descriptions, so these tests discriminate the new winding-inconsistency
// signal by string-matching `"winding inconsistency"` in the issue
// description — the load-bearing anchor convention from §5.5.
//
// Fixture polarity:
// - `test_winding_inconsistent_two_same_direction_faces` is the positive
//   case (issue must fire).
// - `test_winding_consistent_watertight_cube` is the regression-guard
//   that pre-flight verified `create_watertight_cube` has consistent
//   CCW-from-outside winding (§8.1 Gap F risk row 1 mitigation).
// - `test_winding_consistent_disjoint_faces` confirms the detector
//   doesn't false-positive on faces that share only a vertex.
// - `test_winding_inconsistent_does_not_break_open_edge_check` verifies
//   the directed-edge pass doesn't perturb the existing open-edge
//   detector when winding is consistent.

#[test]
fn test_winding_inconsistent_two_same_direction_faces() {
    // Two triangles sharing edge (0,1), both traversing it in the same
    // direction: face [0,1,2] has directed edge (0,1); face [0,1,3]
    // also has directed edge (0,1). One face is "inside out" relative
    // to the other — Gap F's positive case.
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
    ];
    let faces = vec![[0, 1, 2], [0, 1, 3]];
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the same-direction-faces fixture");

    assert!(
        result
            .issues
            .iter()
            .any(|i| i.issue_type == PrintIssueType::NonManifold
                && i.description.contains("winding inconsistency")),
        "directed-edge pass must flag NonManifold Critical with 'winding inconsistency' description on a same-direction shared edge"
    );
}

#[test]
fn test_winding_consistent_watertight_cube() {
    // Regression guard: the existing `create_watertight_cube` fixture
    // is wound CCW-from-outside (verified by directed-edge inspection
    // in §8.1 Gap F risk row 1 pre-flight). The directed pass must NOT
    // emit a winding-inconsistency issue on it. If this test fails,
    // the cube fixture has a silent winding defect that v0.7 missed —
    // surface as plan-contradicting evidence.
    let mesh = create_watertight_cube();
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the watertight cube");

    assert!(
        !result
            .issues
            .iter()
            .any(|i| i.issue_type == PrintIssueType::NonManifold
                && i.description.contains("winding inconsistency")),
        "watertight cube has consistent CCW-from-outside winding; no winding-inconsistency issue should fire"
    );
}

#[test]
fn test_winding_consistent_disjoint_faces() {
    // Two triangles sharing only a single vertex (vertex 2): no shared
    // edge exists, so no directed-edge collision is possible. The
    // detector must not false-positive on vertex-only adjacency.
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.5, 1.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
    ];
    let faces = vec![[0, 1, 2], [2, 3, 4]];
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the disjoint-faces fixture");

    assert!(
        !result
            .issues
            .iter()
            .any(|i| i.issue_type == PrintIssueType::NonManifold
                && i.description.contains("winding inconsistency")),
        "two faces sharing only a vertex have no shared edge; no winding-inconsistency issue should fire"
    );
}

// -- Gap L (§5.6): build_up_direction parametrization --------------------

/// Rotate a mesh -90° around the +X axis. Maps each vertex
/// `(x, y, z)` to `(x, z, -y)`, sending the mesh's `+Z` axis to
/// world `+Y`. Used to exercise the `+Y` build-up direction
/// symmetric with the default `+Z` configuration.
fn rotate_neg_90_about_x(mesh: &IndexedMesh) -> IndexedMesh {
    let vertices: Vec<Point3<f64>> = mesh
        .vertices
        .iter()
        .map(|v| Point3::new(v.x, v.z, -v.y))
        .collect();
    let faces = mesh.faces.clone();
    IndexedMesh::from_parts(vertices, faces)
}

#[test]
fn test_overhang_with_y_up_orientation() {
    // Equivalence: the same overhang fixture flagged under +Z up
    // must flag the same number of regions under +Y up after the
    // mesh is rotated -90° around +X (which sends mesh +Z → world +Y).
    // Both branches dispatch through `check_overhangs`'s build-up
    // direction read, so any failure here means propagation broke.
    let mesh_z = make_overhang_fixture(std::f64::consts::FRAC_PI_2, 5.0);
    let mesh_y = rotate_neg_90_about_x(&mesh_z);

    let config_z = PrinterConfig::fdm_default();
    let config_y =
        PrinterConfig::fdm_default().with_build_up_direction(Vector3::new(0.0, 1.0, 0.0));

    let result_z = validate_for_printing(&mesh_z, &config_z)
        .expect("validation should succeed for +Z-up roof fixture");
    let result_y = validate_for_printing(&mesh_y, &config_y)
        .expect("validation should succeed for +Y-up rotated roof fixture");

    assert_eq!(result_z.overhangs.len(), 1, "+Z up should flag the roof");
    assert_eq!(
        result_z.overhangs.len(),
        result_y.overhangs.len(),
        "overhang count must be symmetric across +Z and +Y build-up directions"
    );
}

#[test]
fn test_overhang_with_oblique_up() {
    // Builder normalizes a non-axis-aligned input. (0, 1, 1) →
    // (0, 1/√2, 1/√2). The validator must accept any unit vector
    // as the build-up direction; the dot-product math is fully
    // direction-agnostic.
    let config = PrinterConfig::fdm_default().with_build_up_direction(Vector3::new(0.0, 1.0, 1.0));
    let s = std::f64::consts::FRAC_1_SQRT_2;
    assert!((config.build_up_direction.x - 0.0).abs() < f64::EPSILON);
    assert!((config.build_up_direction.y - s).abs() < f64::EPSILON);
    assert!((config.build_up_direction.z - s).abs() < f64::EPSILON);

    // Validation should run without panic on a fixture under the
    // oblique up vector (no NaN / Inf escapes).
    let mesh = make_overhang_fixture(std::f64::consts::FRAC_PI_2, 5.0);
    let _ = validate_for_printing(&mesh, &config)
        .expect("validation should succeed with oblique build-up direction");
}

#[test]
fn test_winding_inconsistent_does_not_break_open_edge_check() {
    // `create_cube_mesh` is an incomplete cube (bottom + top only)
    // with consistent winding within each face. The directed-edge
    // pass must not perturb the undirected open-edge check: result
    // still reports `NotWatertight` Critical, and no winding-
    // inconsistency issue fires.
    let mesh = create_cube_mesh();
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the incomplete-cube fixture");

    assert!(
        result
            .issues
            .iter()
            .any(|i| i.issue_type == PrintIssueType::NotWatertight),
        "incomplete cube must still flag NotWatertight (open edges)"
    );
    assert!(
        !result
            .issues
            .iter()
            .any(|i| i.issue_type == PrintIssueType::NonManifold
                && i.description.contains("winding inconsistency")),
        "incomplete cube has consistent winding; no winding-inconsistency issue should fire"
    );
}

// -- §6.1: ThinWall detector ---------------------------------------

/// Build a closed 10×10×`thickness` cuboid centred on the +XY quadrant
/// at z ∈ [0, thickness], wound CCW-from-outside. Watertight +
/// consistently wound; the §6.1 `ThinWall` detector preconditions
/// hold. The cuboid's top + bottom faces oppose each other at
/// distance `thickness` along the build-up axis; the four side faces
/// oppose each other at distance 10 mm in the orthogonal axes.
fn make_thin_slab(thickness: f64) -> IndexedMesh {
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
        Point3::new(0.0, 0.0, thickness),
        Point3::new(10.0, 0.0, thickness),
        Point3::new(10.0, 10.0, thickness),
        Point3::new(0.0, 10.0, thickness),
    ];
    let faces = vec![
        [0, 2, 1],
        [0, 3, 2], // Bottom (-z normal)
        [4, 5, 6],
        [4, 6, 7], // Top (+z normal)
        [0, 1, 5],
        [0, 5, 4], // Front (-y)
        [3, 6, 2],
        [3, 7, 6], // Back (+y)
        [0, 4, 7],
        [0, 7, 3], // Left (-x)
        [1, 2, 6],
        [1, 6, 5], // Right (+x)
    ];
    IndexedMesh::from_parts(vertices, faces)
}

#[test]
fn test_thin_wall_detected_on_thin_slab() {
    // 10×10×0.4 mm slab, FDM `min_wall_thickness = 1.0`. The top and
    // bottom faces oppose each other across 0.4 mm; their inward
    // rays hit the opposite face at 0.4 mm — flagged. Side faces
    // oppose at 10 mm — not flagged. By edge-adjacency the top and
    // bottom triangles cluster *separately* (closed-shell topology:
    // top tris share no edge with bottom tris), so the assertion is
    // exactly 2 clusters, both reporting thickness ≈ 0.4 mm. Same
    // topological pattern as the §7.1 hollow box (2 clusters per
    // pre-flight hand-trace).
    let mesh = make_thin_slab(0.4);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the thin-slab fixture");

    assert_eq!(
        result.thin_walls.len(),
        2,
        "thin slab: top + bottom flagged faces cluster into 2 disjoint regions"
    );
    for region in &result.thin_walls {
        assert!(
            (region.thickness - 0.4).abs() < 1e-5,
            "expected thickness ≈ 0.4 mm, got {}",
            region.thickness
        );
    }
    let critical_thin = result
        .issues
        .iter()
        .filter(|i| {
            i.issue_type == PrintIssueType::ThinWall && i.severity == IssueSeverity::Critical
        })
        .count();
    assert_eq!(
        critical_thin, 2,
        "0.4 < 1.0/2 = 0.5 → both clusters Critical"
    );
}

#[test]
fn test_thin_wall_no_issue_on_thick_cube() {
    // A 10 mm watertight cube under FDM `min_wall_thickness = 1.0`:
    // every face's inward ray hits the opposite face at 10 mm,
    // 10 ≫ 1.0 → no `ThinWall` regions. Verifies the detector
    // doesn't false-flag effectively-solid geometry.
    let mesh = create_watertight_cube();
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the watertight-cube fixture");

    assert_eq!(
        result.thin_walls.len(),
        0,
        "10 mm walls under min_wall=1.0 must not flag"
    );
}

#[test]
fn test_thin_wall_severity_critical_at_quarter_min() {
    // 0.25 mm slab, min_wall=1.0 → 0.25 < 1.0/2 = 0.5 → Critical.
    let mesh = make_thin_slab(0.25);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the 0.25 mm slab");

    assert!(!result.thin_walls.is_empty(), "0.25 mm slab must flag");
    let any_critical = result
        .issues
        .iter()
        .any(|i| i.issue_type == PrintIssueType::ThinWall && i.severity == IssueSeverity::Critical);
    assert!(any_critical, "0.25 < 0.5 must classify as Critical");
}

#[test]
fn test_thin_wall_severity_warning_at_three_quarter_min() {
    // 0.75 mm slab, min_wall=1.0 → 0.5 ≤ 0.75 < 1.0 → Warning.
    let mesh = make_thin_slab(0.75);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the 0.75 mm slab");

    assert!(!result.thin_walls.is_empty(), "0.75 mm slab must flag");
    let any_warning = result
        .issues
        .iter()
        .any(|i| i.issue_type == PrintIssueType::ThinWall && i.severity == IssueSeverity::Warning);
    let any_critical = result
        .issues
        .iter()
        .any(|i| i.issue_type == PrintIssueType::ThinWall && i.severity == IssueSeverity::Critical);
    assert!(any_warning, "0.75 ≥ 0.5 must classify as Warning");
    assert!(!any_critical, "0.75 ≥ 0.5 must NOT classify as Critical");
}

#[test]
fn test_thin_wall_borderline_no_issue() {
    // Strictly-less predicate (`min_dist < min_wall_thickness`):
    // 1.0 mm slab under min_wall=1.0 must NOT flag.
    let mesh = make_thin_slab(1.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the borderline 1.0 mm slab");

    assert_eq!(
        result.thin_walls.len(),
        0,
        "1.0 mm under min_wall=1.0 must not flag (strict-less predicate)"
    );
}

/// Build a subdivided thin-slab mesh: each of the 6 faces of a
/// 10 × 10 × `thickness` mm box is split into `subdiv × subdiv`
/// triangles. Total face count = `12 × subdiv²`. Workshop-scale
/// fixture for the BVH-vs-reference regression test + runtime
/// gate without coupling to specific hardware via the production
/// 400 k-face gasket mesh.
fn make_subdivided_slab(thickness: f64, subdiv: u32) -> IndexedMesh {
    let n = subdiv as usize;
    // Build a regular grid of vertices on each of the 6 faces.
    // Use a flat vertex Vec; index calculation below.
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    let subdiv_f = f64::from(subdiv);

    // Helper: emit (subdiv+1)² grid vertices spanning param (u, v)
    // ∈ [0, 1]², projected via `pos_at(u, v)`.
    let mut emit_grid = |pos_at: &dyn Fn(f64, f64) -> Point3<f64>, faces: &mut Vec<[u32; 3]>| {
        let base = vertices.len() as u32;
        for j in 0..=n {
            for i in 0..=n {
                let u = (i as u32) as f64 / subdiv_f;
                let v = (j as u32) as f64 / subdiv_f;
                vertices.push(pos_at(u, v));
            }
        }
        // Emit two tris per grid cell.
        let row = subdiv + 1;
        for j in 0..n {
            for i in 0..n {
                let r0 = base + (j as u32) * row + (i as u32);
                let r1 = r0 + 1;
                let r2 = r0 + row;
                let r3 = r2 + 1;
                faces.push([r0, r2, r1]);
                faces.push([r1, r2, r3]);
            }
        }
    };

    // Six faces: bottom (-Z) / top (+Z) / -Y / +Y / -X / +X.
    emit_grid(&|u, v| Point3::new(u * 10.0, v * 10.0, 0.0), &mut faces);
    emit_grid(
        &|u, v| Point3::new(u * 10.0, v * 10.0, thickness),
        &mut faces,
    );
    // (Side faces omitted — for the thin-wall test only the top +
    // bottom flagging matters; side walls at 10 mm don't flag.
    // Including them inflates face count without helping the test
    // case discriminate. The detector's watertight precondition
    // requires a closed mesh, so add minimal-tri side strips.)
    // Simpler: build a watertight slab. Use a non-subdivided side
    // wall (just 2 tris each = 8 side tris) so total = 12·n² + 8.
    let v0 = vertices.len() as u32;
    // Bottom + top rim verts shared via existing grid — but for
    // simplicity emit standalone side-wall verts at the 4 corners.
    vertices.extend([
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
        Point3::new(0.0, 0.0, thickness),
        Point3::new(10.0, 0.0, thickness),
        Point3::new(10.0, 10.0, thickness),
        Point3::new(0.0, 10.0, thickness),
    ]);
    // 4 side faces × 2 tris each, using the 8 corner verts above.
    faces.extend([
        // -Y face (verts at y=0): bottom-front-left, top-front-left, etc.
        [v0, v0 + 1, v0 + 5],
        [v0, v0 + 5, v0 + 4],
        // +X face (verts at x=10):
        [v0 + 1, v0 + 2, v0 + 6],
        [v0 + 1, v0 + 6, v0 + 5],
        // +Y face (verts at y=10):
        [v0 + 2, v0 + 3, v0 + 7],
        [v0 + 2, v0 + 7, v0 + 6],
        // -X face (verts at x=0):
        [v0 + 3, v0, v0 + 4],
        [v0 + 3, v0 + 4, v0 + 7],
    ]);

    IndexedMesh::from_parts(vertices, faces)
}

#[test]
fn thin_wall_bvh_matches_reference_o_n_squared() {
    // S1 of `docs/CF_CAST_F4_SPATIAL_INDEX_RECON.md` §B-3 #1:
    // BVH-accelerated `flag_thin_wall_faces` must produce
    // bit-identical-to-1µm flagged-face set + per-face min_dist
    // as the O(face²) `flag_thin_wall_faces_reference` on a
    // representative mesh.
    //
    // 0.4 mm slab is the canonical thin-wall fixture used by the
    // existing test_thin_wall_detected_on_thin_slab — flags top +
    // bottom triangles, leaves sides un-flagged. Same flagged
    // set MUST emerge from both paths.
    let mesh = make_thin_slab(0.4);
    let config = PrinterConfig::fdm_default();

    let bvh_flagged = flag_thin_wall_faces(&mesh, &config);
    let ref_flagged = flag_thin_wall_faces_reference(&mesh, &config);

    assert_eq!(
        bvh_flagged.len(),
        ref_flagged.len(),
        "BVH flagged-face count {} ≠ reference {}",
        bvh_flagged.len(),
        ref_flagged.len()
    );
    for (face_idx, ref_meta) in &ref_flagged {
        let bvh_meta = bvh_flagged.get(face_idx).unwrap_or_else(|| {
            panic!(
                "face {face_idx} flagged by reference but NOT by BVH \
                     (reference thickness {:.6} mm)",
                ref_meta.thickness
            )
        });
        // 1 µm tolerance — f32 parry3d cast has ~1e-7 relative
        // drift, well below `min_wall_thickness` resolution.
        let dt = (bvh_meta.thickness - ref_meta.thickness).abs();
        assert!(
            dt < 1e-3,
            "face {face_idx}: BVH thickness {:.6} mm vs reference {:.6} mm \
                 differ by {:.6} mm (> 1 µm tolerance)",
            bvh_meta.thickness,
            ref_meta.thickness,
            dt
        );
        // Area is computed identically in both paths (no FP
        // contamination from BVH) — must match exactly.
        assert_eq!(
            bvh_meta.area, ref_meta.area,
            "face {face_idx}: area mismatch (computed in both paths the same way)"
        );
    }
}

#[test]
fn thin_wall_bvh_runtime_under_target_on_subdivided_slab() {
    // §B-3 #2: BVH-vs-reference speedup gate. Subdivided 0.4 mm slab
    // (subdiv=20 → 12·400 + 8 = 4808 faces). Reference path is O(face²)
    // ≈ 23 M ray-tri tests; BVH path is O(face log face). A real regression
    // (BVH silently falling back toward O(n²)) collapses the ratio to ~1×.
    //
    // The gate is RELATIVE (no absolute wall-clock) to avoid hardware
    // coupling. Two robustness measures, because in CI this suite runs in
    // DEBUG (the cross-os + tests-debug jobs; the release-heavy job does not
    // cover mesh-printability) on timing-noisy shared runners:
    //  - the BVH path is sub-millisecond, so one scheduler hiccup can dominate
    //    its measurement → take the MIN over several reps (the min is the
    //    least noise-contaminated estimate of true compute time);
    //  - debug overhead inflates the BVH's richer code path more than the
    //    reference's tight loop, compressing the ratio → the floor is a
    //    conservative 3× (vs the ≥100× algorithmic / ~300-500× production
    //    expectation), which still catches an O(n²) regression at ~1×.
    let mesh = make_subdivided_slab(0.4, 20);
    let config = PrinterConfig::fdm_default();

    // Reference: one run (long enough to be timing-stable on its own).
    let t_ref = std::time::Instant::now();
    let ref_flagged = flag_thin_wall_faces_reference(&mesh, &config);
    let ref_elapsed = t_ref.elapsed();

    // BVH: warm-up + best-of-5 (the min removes scheduler-jitter spikes —
    // the source of the macos flakiness on this sub-millisecond path).
    let bvh_count = flag_thin_wall_faces(&mesh, &config).len(); // warm-up
    let mut bvh_elapsed = std::time::Duration::MAX;
    for _ in 0..5 {
        let t = std::time::Instant::now();
        let n = flag_thin_wall_faces(&mesh, &config).len();
        bvh_elapsed = bvh_elapsed.min(t.elapsed());
        assert_eq!(n, bvh_count, "BVH flagged-face count must be deterministic");
    }

    // Sanity: both paths agree.
    assert_eq!(
        bvh_count,
        ref_flagged.len(),
        "BVH + reference must agree on flagged-face count (BVH {}, ref {})",
        bvh_count,
        ref_flagged.len(),
    );

    let ratio = ref_elapsed.as_secs_f64() / bvh_elapsed.as_secs_f64().max(1e-9);
    assert!(
        ratio > 3.0,
        "BVH speedup ratio {ratio:.1}× below 3× floor \
             (ref {:.3}ms, BVH {:.3}ms)",
        ref_elapsed.as_secs_f64() * 1000.0,
        bvh_elapsed.as_secs_f64() * 1000.0,
    );
}

#[test]
fn thin_wall_bvh_handles_degenerate_inputs() {
    // S1 of §B-3 #3: BVH + reference paths must both gracefully
    // return empty flagged set on degenerate inputs (empty mesh,
    // single-face, NaN-vertex, zero-area face). NO panics.
    let config = PrinterConfig::fdm_default();

    // Empty mesh.
    let empty = IndexedMesh::from_parts(Vec::new(), Vec::new());
    assert!(flag_thin_wall_faces(&empty, &config).is_empty());
    assert!(flag_thin_wall_faces_reference(&empty, &config).is_empty());

    // Single-face mesh (BVH builds, but no opposite face for
    // ray-cast → no thin walls).
    let single = IndexedMesh::from_parts(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        vec![[0, 1, 2]],
    );
    assert!(flag_thin_wall_faces(&single, &config).is_empty());
    assert!(flag_thin_wall_faces_reference(&single, &config).is_empty());

    // NaN vertex (BVH builder rejects; reference path's normal
    // calc returns NaN, `if NaN < 1e-10` is false but `if t < min`
    // is also false → no flagged faces).
    let nan_mesh = IndexedMesh::from_parts(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(f64::NAN, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.5, 0.5, 0.1),
        ],
        vec![[0, 1, 2], [0, 2, 3]],
    );
    // BVH path: build_parry_trimesh returns None → empty flagged.
    assert!(flag_thin_wall_faces(&nan_mesh, &config).is_empty());
    // Reference path: NaN propagates through normal computation,
    // skip via `len < 1e-10` is false but downstream `t < min`
    // comparisons return false → no flagged faces.
    // (May still flag if NaN happens to be < INFINITY in the
    // accumulator's first iteration; that's acceptable behavior
    // — the test asserts NO PANIC, not a specific flagged set on
    // NaN input.)
    let _ = flag_thin_wall_faces_reference(&nan_mesh, &config);

    // Zero-area face (degenerate triangle: three colinear verts).
    let zero_area = IndexedMesh::from_parts(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0), // colinear with 0, 1
            Point3::new(0.5, 0.5, 0.0), // gives a real second tri
        ],
        vec![[0, 1, 2], [0, 1, 3]], // first tri is zero-area
    );
    // Both paths skip the zero-area face via the `len < 1e-10`
    // guard.
    let _ = flag_thin_wall_faces(&zero_area, &config);
    let _ = flag_thin_wall_faces_reference(&zero_area, &config);
}

#[test]
fn test_thin_wall_skipped_on_open_mesh() {
    // Open mesh (top of slab removed → 4 open edges along the top
    // perimeter): watertight precondition fails → `DetectorSkipped`
    // emitted, no `ThinWallRegion`s populated.
    let mut mesh = make_thin_slab(0.4);
    // Drop the top tris (faces 2 + 3 in the slab face list). Drain
    // and rebuild the face list omitting indices 2..4.
    let faces: Vec<[u32; 3]> = mesh
        .faces
        .iter()
        .enumerate()
        .filter(|(idx, _)| *idx != 2 && *idx != 3)
        .map(|(_, f)| *f)
        .collect();
    mesh = IndexedMesh::from_parts(mesh.vertices, faces);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the open-mesh fixture");

    let any_skipped = result.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::DetectorSkipped
            && i.description.contains("ThinWall")
            && i.description.contains("watertight")
    });
    assert!(
        any_skipped,
        "open mesh must emit DetectorSkipped with ThinWall + watertight in description"
    );
    assert_eq!(
        result.thin_walls.len(),
        0,
        "open mesh must not populate thin_walls"
    );
}

#[test]
fn test_thin_wall_skipped_on_inconsistent_winding() {
    // Slab with one top tri's vertex order flipped → directed edge
    // (4, 6) appears in BOTH the flipped face and the unflipped
    // sister face → consistent-winding precondition fails →
    // `DetectorSkipped` emitted.
    let mesh = make_thin_slab(0.4);
    // Flip face index 2 from [4, 5, 6] to [4, 6, 5].
    let mut faces: Vec<[u32; 3]> = mesh.faces.clone();
    faces[2] = [4, 6, 5];
    let mesh = IndexedMesh::from_parts(mesh.vertices, faces);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the flipped-winding fixture");

    let any_skipped = result.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::DetectorSkipped && i.description.contains("ThinWall")
    });
    assert!(
        any_skipped,
        "inconsistent winding must emit DetectorSkipped from ThinWall"
    );
    assert_eq!(
        result.thin_walls.len(),
        0,
        "winding-inconsistent mesh must not populate thin_walls"
    );
}

#[test]
fn test_thin_wall_two_disjoint_clusters() {
    // Two slabs offset along +X by 30 mm: each slab independently
    // produces 2 clusters (top + bottom topologically disjoint per
    // closed-shell hand-trace). Total: 4 clusters across the two
    // disjoint components.
    let slab_a = make_thin_slab(0.4);
    let mut vertices: Vec<Point3<f64>> = slab_a.vertices.clone();
    let mut faces: Vec<[u32; 3]> = slab_a.faces.clone();

    let offset_x = 30.0;
    // Hand-built fixture: an `expect` failure here would indicate
    // the slab helper itself was malformed, not a detector regression.
    let base = u32::try_from(slab_a.vertices.len()).expect("slab vertex count fits in u32");
    for v in &slab_a.vertices {
        vertices.push(Point3::new(v.x + offset_x, v.y, v.z));
    }
    for f in &slab_a.faces {
        faces.push([f[0] + base, f[1] + base, f[2] + base]);
    }
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the two-slab fixture");

    assert_eq!(
        result.thin_walls.len(),
        4,
        "two disjoint slabs each contribute 2 clusters → 4 total"
    );
    for region in &result.thin_walls {
        assert!(
            (region.thickness - 0.4).abs() < 1e-5,
            "all clusters report thickness ≈ 0.4 mm"
        );
    }
}

#[test]
fn test_thin_wall_sort_stable_across_runs() {
    // Same input, two runs: cluster ordering matches deterministically.
    // Verifies §4.4 sort stability — `partition_flagged_into_components`
    // emerges in min-face-idx order regardless of `HashMap` iteration
    // permutation.
    let mesh = make_thin_slab(0.4);
    let config = PrinterConfig::fdm_default();

    let r1 = validate_for_printing(&mesh, &config).expect("validation 1");
    let r2 = validate_for_printing(&mesh, &config).expect("validation 2");

    assert_eq!(r1.thin_walls.len(), r2.thin_walls.len());
    for (a, b) in r1.thin_walls.iter().zip(r2.thin_walls.iter()) {
        assert_eq!(a.faces, b.faces, "cluster face order must match");
        assert!(
            (a.thickness - b.thickness).abs() < f64::EPSILON,
            "cluster thickness must match bit-for-bit"
        );
    }
}

// ---- §6.2 Gap G LongBridge tests --------------------------------------
//
// Closed-mesh bridge fixtures: a tiny ground-anchor cuboid at `z=0`
// plus an elevated slab. Two disjoint cuboids are jointly watertight
// + consistently wound (each component closes on its own; vertices
// are disjoint so no edge crosses components). The slab's bottom
// face is the bridge candidate; the anchor pins `mesh_min_along_up = 0`
// so the slab's bottom isn't filtered. Slab thickness ≥ 1 mm to
// avoid co-flagging as `ThinWall` under default FDM
// `min_wall_thickness = 1.0`.

/// Append a closed cuboid (CCW-from-outside) to `(vertices, faces)`.
/// `min`/`max` are diagonally-opposite corners; six rectangular
/// faces / twelve triangles. Mirrors the winding of
/// `cube_vertices_and_faces` in `tests/stress_inputs.rs`.
fn append_closed_cuboid(
    vertices: &mut Vec<Point3<f64>>,
    faces: &mut Vec<[u32; 3]>,
    min: Point3<f64>,
    max: Point3<f64>,
) {
    // `vertices.len()` is bounded by the test fixtures' small tri
    // counts (≤ 100); usize → u32 is safe at this scale.
    let base: u32 = vertices.len() as u32;
    vertices.extend_from_slice(&[
        Point3::new(min.x, min.y, min.z),
        Point3::new(max.x, min.y, min.z),
        Point3::new(max.x, max.y, min.z),
        Point3::new(min.x, max.y, min.z),
        Point3::new(min.x, min.y, max.z),
        Point3::new(max.x, min.y, max.z),
        Point3::new(max.x, max.y, max.z),
        Point3::new(min.x, max.y, max.z),
    ]);
    let cf = |a: u32, b: u32, c: u32| [base + a, base + b, base + c];
    faces.extend_from_slice(&[
        cf(0, 2, 1),
        cf(0, 3, 2),
        cf(4, 5, 6),
        cf(4, 6, 7),
        cf(0, 1, 5),
        cf(0, 5, 4),
        cf(3, 6, 2),
        cf(3, 7, 6),
        cf(0, 4, 7),
        cf(0, 7, 3),
        cf(1, 2, 6),
        cf(1, 6, 5),
    ]);
}

/// Build a closed bridge fixture: a 2×2×2 anchor cuboid at
/// `z ∈ [0, 2]` (sets `mesh_min_along_up = 0`) plus a slab of
/// `x_extent × y_extent × slab_thickness` at `z ∈ [elevation,
/// elevation + slab_thickness]`. The slab is x-offset by `+10` so
/// its vertices are disjoint from the anchor's.
fn make_closed_bridge_fixture(
    x_extent: f64,
    y_extent: f64,
    slab_thickness: f64,
    elevation: f64,
) -> IndexedMesh {
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    // Anchor at (-3..-1, -3..-1, 0..2) — x/y-offset to avoid overlap
    // with the slab's bbox.
    append_closed_cuboid(
        &mut vertices,
        &mut faces,
        Point3::new(-3.0, -3.0, 0.0),
        Point3::new(-1.0, -1.0, 2.0),
    );
    // Slab at (0..x_extent, 0..y_extent, elevation..elevation+thickness).
    append_closed_cuboid(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, elevation),
        Point3::new(x_extent, y_extent, elevation + slab_thickness),
    );
    IndexedMesh::from_parts(vertices, faces)
}

#[test]
fn test_long_bridge_horizontal_slab_exceeds_span() {
    // 20×5 slab at z=10, max_bridge_span = 10 (FDM default).
    // span = max(20, 5) = 20; flagged at Critical (20 > 10*1.5).
    let mesh = make_closed_bridge_fixture(20.0, 5.0, 1.5, 10.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

    assert_eq!(
        result.long_bridges.len(),
        1,
        "20×5 slab elevated above plate must produce 1 LongBridge region"
    );
    let region = &result.long_bridges[0];
    assert!(
        (region.span - 20.0).abs() < 1e-6,
        "span must equal the bbox max-extent (20 mm); got {}",
        region.span
    );
    // Issue exists at LongBridge type with Critical severity.
    let critical = result
        .issues
        .iter()
        .filter(|i| {
            i.issue_type == PrintIssueType::LongBridge && i.severity == IssueSeverity::Critical
        })
        .count();
    assert_eq!(critical, 1, "20 mm > 10*1.5 → Critical");
}

#[test]
fn test_long_bridge_short_span_no_issue() {
    // 5×5 slab, max_bridge_span = 10. span = 5 ≤ 10 → no flag.
    let mesh = make_closed_bridge_fixture(5.0, 5.0, 1.5, 10.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

    assert_eq!(
        result.long_bridges.len(),
        0,
        "5×5 slab span (5) ≤ max_bridge_span (10); no LongBridge region"
    );
}

#[test]
fn test_long_bridge_borderline_warning() {
    // 12×5 slab, max=10. span = 12; 10 < 12 ≤ 10*1.5 = 15 → Warning.
    let mesh = make_closed_bridge_fixture(12.0, 5.0, 1.5, 10.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

    assert_eq!(result.long_bridges.len(), 1);
    let warning = result
        .issues
        .iter()
        .filter(|i| {
            i.issue_type == PrintIssueType::LongBridge && i.severity == IssueSeverity::Warning
        })
        .count();
    assert_eq!(warning, 1, "span 12 in (10, 15] → Warning");
}

#[test]
fn test_long_bridge_well_above_critical() {
    // 20×5 slab, max=10. 20 > 10*1.5 = 15 → Critical (boundary
    // bracketed by 5 mm above the Warning/Critical edge).
    let mesh = make_closed_bridge_fixture(20.0, 5.0, 1.5, 10.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

    assert_eq!(result.long_bridges.len(), 1);
    let critical = result
        .issues
        .iter()
        .filter(|i| {
            i.issue_type == PrintIssueType::LongBridge && i.severity == IssueSeverity::Critical
        })
        .count();
    assert_eq!(critical, 1, "span 20 > 10*1.5 → Critical");
}

#[test]
fn test_long_bridge_skipped_for_sls() {
    // SLS config: `requires_supports() == false` → silent skip.
    // No `LongBridge` regions, no `DetectorSkipped` issue (per §6.2
    // line 996, distinct from `ThinWall`'s `DetectorSkipped` on
    // non-watertight).
    let mesh = make_closed_bridge_fixture(20.0, 5.0, 1.5, 10.0);
    let config = PrinterConfig::sls_default();

    let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

    assert_eq!(result.long_bridges.len(), 0, "SLS skips bridges silently");
    let skipped_naming_bridge = result.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::DetectorSkipped
            && i.description.to_lowercase().contains("bridge")
    });
    assert!(
        !skipped_naming_bridge,
        "SLS skip is silent — no DetectorSkipped issue naming bridges"
    );
}

#[test]
fn test_long_bridge_bottom_face_not_flagged() {
    // 20×5 slab at z=0 (sitting on build plate). The slab's bottom
    // face has `face_min = 0 = mesh_min` → build-plate filter
    // excludes; no bridge. Single closed cuboid, no anchor needed
    // (its own bottom is mesh-min).
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_closed_cuboid(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(20.0, 5.0, 1.5),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

    assert_eq!(
        result.long_bridges.len(),
        0,
        "slab on build plate: bottom filtered, no LongBridge"
    );
}

#[test]
fn test_long_bridge_two_disjoint_bridges() {
    // Two parallel 20×5 slabs separated by 30 mm along +X plus a
    // single ground anchor. Three disjoint closed cuboids; each
    // slab's bottom is its own cluster. Two LongBridge regions.
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_closed_cuboid(
        &mut vertices,
        &mut faces,
        Point3::new(-3.0, -3.0, 0.0),
        Point3::new(-1.0, -1.0, 2.0),
    );
    append_closed_cuboid(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 10.0),
        Point3::new(20.0, 5.0, 11.5),
    );
    append_closed_cuboid(
        &mut vertices,
        &mut faces,
        Point3::new(30.0, 0.0, 10.0),
        Point3::new(50.0, 5.0, 11.5),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

    assert_eq!(
        result.long_bridges.len(),
        2,
        "two disjoint slab bottoms → 2 LongBridge regions"
    );
    for region in &result.long_bridges {
        assert!(
            (region.span - 20.0).abs() < 1e-6,
            "each slab spans 20 mm; got {}",
            region.span
        );
    }
}

#[test]
fn test_long_bridge_with_y_up_orientation() {
    // Same 20×5 slab geometry, rotated 90° around +X (y → z, z → -y),
    // validated with `+Y up`. The bridge face's outward normal in
    // the rotated frame is (0, -1, 0) = -up; flagged identically.
    // Anchor at (-3..-1, 0..2, 1..3) (y plays z-role).
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    // Anchor in the y-up frame: y ∈ [0, 2] is "elevation 0..2", and
    // x/z extents define the footprint.
    append_closed_cuboid(
        &mut vertices,
        &mut faces,
        Point3::new(-3.0, 0.0, -3.0),
        Point3::new(-1.0, 2.0, -1.0),
    );
    // Slab in the y-up frame: y ∈ [10, 11.5] (1.5 mm thick), x ∈
    // [0, 20], z ∈ [0, 5]. Span = max(20, 5) = 20 in the
    // (e1, e2) = ((1,0,0), (0,0,-1)) basis.
    append_closed_cuboid(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 10.0, 0.0),
        Point3::new(20.0, 11.5, 5.0),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default().with_build_up_direction(Vector3::new(0.0, 1.0, 0.0));

    let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

    assert_eq!(
        result.long_bridges.len(),
        1,
        "+Y up rotation produces same flag count as +Z up"
    );
    let region = &result.long_bridges[0];
    assert!(
        (region.span - 20.0).abs() < 1e-6,
        "span identical to +Z fixture; got {}",
        region.span
    );
}

#[test]
fn test_long_bridge_cantilever_currently_flagged() {
    // 20-mm cantilever face anchored on one end. v0.8 cannot
    // distinguish a cantilever from a bridge: both produce a single
    // cluster of horizontal-down faces above the build plate.
    // Locks the v0.8 limitation; v0.9 followup adds support-end
    // analysis. Fixture is identical to
    // `test_long_bridge_well_above_critical`: the v0.8 detector
    // sees both shapes the same way.
    let mesh = make_closed_bridge_fixture(20.0, 5.0, 1.5, 10.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

    assert_eq!(
        result.long_bridges.len(),
        1,
        "cantilever currently flags as a bridge (v0.8 limitation; v0.9 followup)"
    );
}

#[test]
fn test_long_bridge_diagonal_underflagged_documented() {
    // 14×14 horizontal patch at z=10, `max_bridge_span = 15`.
    // span = max(14, 14) = 14 < 15 → no flag, even though the true
    // Euclidean diagonal (14·√2 ≈ 19.8 mm) exceeds the limit. v0.8
    // axis-aligned bbox is conservative for diagonals; v0.9 OBB
    // followup will catch this case.
    let mesh = make_closed_bridge_fixture(14.0, 14.0, 1.5, 10.0);
    let mut config = PrinterConfig::fdm_default();
    config.max_bridge_span = 15.0;

    let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

    assert_eq!(
        result.long_bridges.len(),
        0,
        "diagonal patch: bbox extent (14) < max (15); v0.8 underflags by design"
    );
}

#[test]
fn test_long_bridge_regions_sorted_by_start_xyz() {
    // §4.4 ordering: `long_bridges` sorted by `(start.x, start.y,
    // start.z)`. Two parallel slabs appended in REVERSE x-order
    // (high-x slab first → lower face indices). Without §4.4 sort,
    // emission order would be high-x then low-x. With the sort,
    // low-x must precede high-x.
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_closed_cuboid(
        &mut vertices,
        &mut faces,
        Point3::new(-3.0, -3.0, 0.0),
        Point3::new(-1.0, -1.0, 2.0),
    );
    // High-x slab appended FIRST (gets lower face indices).
    append_closed_cuboid(
        &mut vertices,
        &mut faces,
        Point3::new(30.0, 0.0, 10.0),
        Point3::new(50.0, 5.0, 11.5),
    );
    // Low-x slab appended SECOND.
    append_closed_cuboid(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 10.0),
        Point3::new(20.0, 5.0, 11.5),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

    assert_eq!(result.long_bridges.len(), 2);
    let first = &result.long_bridges[0];
    let second = &result.long_bridges[1];
    assert!(
        first.start.x < second.start.x,
        "§4.4: long_bridges must sort by start.x; got [{}, {}]",
        first.start.x,
        second.start.x
    );
}

#[test]
fn test_long_bridge_co_flags_with_overhang() {
    // §6.2 edge case: cluster overlapping with overhang region —
    // both detectors flag independently (not a duplicate-flag bug).
    // The slab bottom has overhang_angle = 90° (>> FDM 45°
    // threshold) AND span 20 > max=10. Both regions emit.
    let mesh = make_closed_bridge_fixture(20.0, 5.0, 1.5, 10.0);
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

    assert_eq!(result.long_bridges.len(), 1);
    assert!(
        !result.overhangs.is_empty(),
        "slab bottom (90° overhang) must also flag as ExcessiveOverhang"
    );
}

// ---- §6.3 Gap H — TrappedVolume detector tests --------------------------
//
// The §6.3 spec calls the load-bearing fixture "sphere_inside_cube" but
// the implementation here uses a cube-cavity-in-cube. Rationale: the
// detector is winding- and curvature-agnostic (operates on voxelized
// parity); a cube cavity is sufficient to exercise every code path. The
// §9.2.5 stress fixture `stress_h_sphere_inside_cube_volume_within_10pct`
// (in `tests/stress_inputs.rs`) is the geometry-faithful sphere variant.

/// Build a watertight outer cube with a single inner-cube cavity.
/// Outer cube vertices `[0..8]` are wound CCW-from-outside (mirrors
/// `create_watertight_cube`). Inner cavity vertices `[8..16]` are wound
/// CCW-from-inside the cavity (normals point into the cavity). The
/// detector's parity-based inside-test is winding-agnostic so either
/// inner winding works; this convention matches the implicit-surface
/// "body is the inside set" mental model.
fn make_cube_with_inner_cavity(outer_size: f64, inner_size: f64) -> IndexedMesh {
    let cs = (outer_size - inner_size) / 2.0;
    let i_max = cs + inner_size;
    let vertices = vec![
        // Outer cube vertices [0..8]
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(outer_size, 0.0, 0.0),
        Point3::new(outer_size, outer_size, 0.0),
        Point3::new(0.0, outer_size, 0.0),
        Point3::new(0.0, 0.0, outer_size),
        Point3::new(outer_size, 0.0, outer_size),
        Point3::new(outer_size, outer_size, outer_size),
        Point3::new(0.0, outer_size, outer_size),
        // Inner cavity vertices [8..16]
        Point3::new(cs, cs, cs),
        Point3::new(i_max, cs, cs),
        Point3::new(i_max, i_max, cs),
        Point3::new(cs, i_max, cs),
        Point3::new(cs, cs, i_max),
        Point3::new(i_max, cs, i_max),
        Point3::new(i_max, i_max, i_max),
        Point3::new(cs, i_max, i_max),
    ];
    let faces = vec![
        // Outer cube — CCW-from-outside.
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [3, 6, 2],
        [3, 7, 6],
        [0, 4, 7],
        [0, 7, 3],
        [1, 2, 6],
        [1, 6, 5],
        // Inner cavity — wound CCW-from-INSIDE the cavity.
        [8, 9, 10],
        [8, 10, 11],
        [12, 14, 13],
        [12, 15, 14],
        [8, 13, 9],
        [8, 12, 13],
        [11, 10, 14],
        [11, 14, 15],
        [8, 15, 12],
        [8, 11, 15],
        [9, 14, 10],
        [9, 13, 14],
    ];
    IndexedMesh::from_parts(vertices, faces)
}

/// Build a watertight outer cube with two disjoint inner-cube cavities
/// at `(cavity_size/2 + offset_a, …)` and `(outer_size − cavity_size/2 −
/// offset_a, …)` along the diagonal — well-separated so flood-fill
/// labels them as distinct components. 24 vertices, 36 faces.
fn make_cube_with_two_inner_cavities(
    outer_size: f64,
    cavity_size: f64,
    offset_a: f64,
) -> IndexedMesh {
    let outer_min = Point3::new(0.0, 0.0, 0.0);
    let outer_max = Point3::new(outer_size, outer_size, outer_size);
    let a_lo = offset_a;
    let a_hi = a_lo + cavity_size;
    let b_lo = outer_size - offset_a - cavity_size;
    let b_hi = b_lo + cavity_size;
    let vertices = vec![
        // Outer cube [0..8]
        outer_min,
        Point3::new(outer_max.x, 0.0, 0.0),
        Point3::new(outer_max.x, outer_max.y, 0.0),
        Point3::new(0.0, outer_max.y, 0.0),
        Point3::new(0.0, 0.0, outer_max.z),
        Point3::new(outer_max.x, 0.0, outer_max.z),
        outer_max,
        Point3::new(0.0, outer_max.y, outer_max.z),
        // Cavity A [8..16]
        Point3::new(a_lo, a_lo, a_lo),
        Point3::new(a_hi, a_lo, a_lo),
        Point3::new(a_hi, a_hi, a_lo),
        Point3::new(a_lo, a_hi, a_lo),
        Point3::new(a_lo, a_lo, a_hi),
        Point3::new(a_hi, a_lo, a_hi),
        Point3::new(a_hi, a_hi, a_hi),
        Point3::new(a_lo, a_hi, a_hi),
        // Cavity B [16..24]
        Point3::new(b_lo, b_lo, b_lo),
        Point3::new(b_hi, b_lo, b_lo),
        Point3::new(b_hi, b_hi, b_lo),
        Point3::new(b_lo, b_hi, b_lo),
        Point3::new(b_lo, b_lo, b_hi),
        Point3::new(b_hi, b_lo, b_hi),
        Point3::new(b_hi, b_hi, b_hi),
        Point3::new(b_lo, b_hi, b_hi),
    ];
    let faces = vec![
        // Outer cube — CCW-from-outside.
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [3, 6, 2],
        [3, 7, 6],
        [0, 4, 7],
        [0, 7, 3],
        [1, 2, 6],
        [1, 6, 5],
        // Cavity A — CCW-from-INSIDE the cavity.
        [8, 9, 10],
        [8, 10, 11],
        [12, 14, 13],
        [12, 15, 14],
        [8, 13, 9],
        [8, 12, 13],
        [11, 10, 14],
        [11, 14, 15],
        [8, 15, 12],
        [8, 11, 15],
        [9, 14, 10],
        [9, 13, 14],
        // Cavity B — same winding pattern, offset by +8.
        [16, 17, 18],
        [16, 18, 19],
        [20, 22, 21],
        [20, 23, 22],
        [16, 21, 17],
        [16, 20, 21],
        [19, 18, 22],
        [19, 22, 23],
        [16, 23, 20],
        [16, 19, 23],
        [17, 22, 18],
        [17, 21, 22],
    ];
    IndexedMesh::from_parts(vertices, faces)
}

/// Coarse-voxel `PrinterConfig` for fast unit tests: voxel = 0.2 mm
/// regardless of technology (`min_feature_size = 0.8`,
/// `layer_height = 0.4`). At a 6 mm outer cube, this produces a 34³ ≈
/// 39 k-voxel grid that finishes in <100 ms in debug mode while still
/// exercising every algorithmic path. The technology field still
/// drives `classify_trapped_volume_severity`'s SLA / SLS / MJF /
/// FDM branch.
fn coarse_voxel_config(tech: PrintTechnology) -> PrinterConfig {
    let mut c = match tech {
        PrintTechnology::Sla => PrinterConfig::sla_default(),
        PrintTechnology::Sls => PrinterConfig::sls_default(),
        PrintTechnology::Mjf => PrinterConfig::mjf_default(),
        PrintTechnology::Fdm | PrintTechnology::Other => PrinterConfig::fdm_default(),
    };
    c.technology = tech;
    c.min_feature_size = 0.8;
    c.layer_height = 0.4;
    c
}

/// Build a `VoxelGrid` for `mesh` at `voxel_size`, mirroring
/// `check_trapped_volumes`'s sizing (AABB padded by 2 voxels;
/// `dims = ceil(extent / voxel_size).max(1)`), all states
/// `VOXEL_UNKNOWN`. Used by the BVH/reference byte-identity test.
fn build_trapped_grid(mesh: &IndexedMesh, voxel_size: f64) -> VoxelGrid {
    let (mn, mx) = compute_bounds(mesh);
    let pad = 2.0 * voxel_size;
    let origin = Point3::new(mn.x - pad, mn.y - pad, mn.z - pad);
    let dim =
        |lo: f64, hi: f64| -> u32 { ((hi - lo + 2.0 * pad) / voxel_size).ceil().max(1.0) as u32 };
    let dims = (dim(mn.x, mx.x), dim(mn.y, mx.y), dim(mn.z, mx.z));
    let total = dims.0 as usize * dims.1 as usize * dims.2 as usize;
    VoxelGrid {
        dims,
        origin,
        voxel_size,
        states: vec![VOXEL_UNKNOWN; total],
    }
}

#[test]
fn mark_inside_voxels_bvh_is_byte_identical_to_reference() {
    // S3 gate (docs/CF_CAST_F4_SPATIAL_INDEX_RECON.md): the BVH-culled
    // `mark_inside_voxels` must produce a byte-for-byte identical voxel
    // grid to the O(ny·nz·n_faces) reference on watertight meshes with
    // interior structure (rows with 0 / 2 / 4 crossings). A divergence
    // here means the AABB cull dropped a face the +X ray actually
    // pierces — the one way the speedup could change geometry.
    let fixtures = [
        ("solid_cube", create_watertight_cube()),
        ("single_cavity", make_cube_with_inner_cavity(6.0, 2.0)),
        (
            "two_cavities",
            make_cube_with_two_inner_cavities(8.0, 1.0, 1.5),
        ),
    ];
    for (name, mesh) in fixtures {
        // Two voxel sizes: one giving a coarse grid, one finer, so the
        // assertion spans different row counts + candidate densities.
        for &voxel_size in &[0.5_f64, 0.23_f64] {
            let mut g_ref = build_trapped_grid(&mesh, voxel_size);
            let mut g_bvh = build_trapped_grid(&mesh, voxel_size);
            assert_eq!(
                g_ref.dims, g_bvh.dims,
                "{name}: grid sizing must match before marking"
            );
            mark_inside_voxels_reference(&mut g_ref, &mesh);
            mark_inside_voxels(&mut g_bvh, &mesh);
            assert_eq!(
                g_ref.states, g_bvh.states,
                "{name} @ voxel {voxel_size}: BVH-culled inside-mark must be \
                     byte-identical to the brute-force reference"
            );
            // Sanity: the cavity fixtures must actually mark some inside
            // voxels (else the test would pass vacuously on empty grids).
            assert!(
                g_ref.states.contains(&VOXEL_INSIDE),
                "{name} @ voxel {voxel_size}: expected some VOXEL_INSIDE"
            );
        }
    }
}

#[test]
fn mark_inside_voxels_falls_back_to_reference_when_bvh_build_rejects() {
    // The BVH path's bit-identity claim explicitly rests on a fallback to
    // the reference marker when `build_parry_trimesh` rejects the mesh
    // (degenerate input the watertight precondition would normally exclude:
    // here a face index past the vertex count). Confirm the fallback fires,
    // does not panic, and produces a grid identical to the reference.
    let mut mesh = make_cube_with_inner_cavity(6.0, 2.0);
    // Append a triangle with clearly out-of-bounds indices → build_parry_trimesh None.
    mesh.faces.push([u32::MAX, u32::MAX - 1, u32::MAX - 2]);
    assert!(
        build_parry_trimesh(&mesh).is_none(),
        "fixture must make the parry TriMesh builder reject (forcing the fallback)"
    );

    let mut g_ref = build_trapped_grid(&mesh, 0.5);
    let mut g_bvh = build_trapped_grid(&mesh, 0.5);
    mark_inside_voxels_reference(&mut g_ref, &mesh);
    mark_inside_voxels(&mut g_bvh, &mesh); // must internally fall back
    assert_eq!(
        g_ref.states, g_bvh.states,
        "fallback path must be byte-identical to the reference marker"
    );
}

#[test]
fn test_trapped_volume_no_cavity() {
    // Solid 6 mm cube (no inner cavity) under FDM coarse-voxel config.
    // Flood-fill from grid corner reaches every non-inside voxel →
    // no trapped voxels → 0 regions.
    let mesh = create_watertight_cube();
    let config = coarse_voxel_config(PrintTechnology::Fdm);
    let result =
        validate_for_printing(&mesh, &config).expect("validation should succeed for solid cube");

    assert_eq!(
        result.trapped_volumes.len(),
        0,
        "solid cube must produce no trapped-volume regions"
    );
    let any_trapped = result
        .issues
        .iter()
        .any(|i| i.issue_type == PrintIssueType::TrappedVolume);
    assert!(!any_trapped, "solid cube must emit no TrappedVolume issues");
}

#[test]
fn test_trapped_volume_skipped_on_open_mesh() {
    // Open mesh (4-face square at z=0; 4 open edges → not watertight)
    // → DetectorSkipped Info; trapped_volumes stays empty.
    let mesh = create_cube_mesh();
    let config = coarse_voxel_config(PrintTechnology::Fdm);
    let result =
        validate_for_printing(&mesh, &config).expect("validation should succeed for open mesh");

    let any_skipped = result.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::DetectorSkipped
            && i.description.contains("TrappedVolume")
            && i.description.contains("watertight")
    });
    assert!(
        any_skipped,
        "open mesh must emit DetectorSkipped with TrappedVolume + watertight in description"
    );
    assert_eq!(
        result.trapped_volumes.len(),
        0,
        "open mesh must not populate trapped_volumes"
    );
}

#[test]
fn test_trapped_volume_sphere_inside_cube() {
    // 6 mm outer cube with 2 mm inner cavity (cube cavity used as
    // cavity proxy per module-doc deviation note). Expected analytical
    // cavity volume: 2³ = 8 mm³. At voxel 0.2 mm, the cavity occupies
    // ~10³ = 1000 voxels = 1000 × 0.008 = 8 mm³ exactly. Bbox is
    // approximately 2×2×2 mm (within ±voxel_size on each side).
    let mesh = make_cube_with_inner_cavity(6.0, 2.0);
    let config = coarse_voxel_config(PrintTechnology::Fdm);
    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for cube-cavity fixture");

    assert_eq!(
        result.trapped_volumes.len(),
        1,
        "single inner cavity must produce exactly one trapped-volume region"
    );
    let region = &result.trapped_volumes[0];
    // Cavity center is at outer_size/2 = 3.0 mm in each axis.
    assert!(
        (region.center.x - 3.0).abs() < 0.2 + 1e-9
            && (region.center.y - 3.0).abs() < 0.2 + 1e-9
            && (region.center.z - 3.0).abs() < 0.2 + 1e-9,
        "cavity centroid must be near (3, 3, 3); got {:?}",
        region.center
    );
    let bbox_extent_x = region.bounding_box.1.x - region.bounding_box.0.x;
    let bbox_extent_y = region.bounding_box.1.y - region.bounding_box.0.y;
    let bbox_extent_z = region.bounding_box.1.z - region.bounding_box.0.z;
    assert!(
        (bbox_extent_x - 2.0).abs() < 0.4
            && (bbox_extent_y - 2.0).abs() < 0.4
            && (bbox_extent_z - 2.0).abs() < 0.4,
        "cavity bbox must be ~2×2×2 mm; got {bbox_extent_x:.3} × {bbox_extent_y:.3} × {bbox_extent_z:.3}"
    );
}

#[test]
fn test_trapped_volume_two_disjoint_cavities() {
    // 8 mm outer cube with 2 disjoint 1 mm cubic cavities at offset 1.5
    // from opposite corners. Walls between cavities ≥ 3 mm = 15
    // voxels, so flood-fill labels them as 2 separate components.
    let mesh = make_cube_with_two_inner_cavities(8.0, 1.0, 1.5);
    let config = coarse_voxel_config(PrintTechnology::Fdm);
    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for two-cavity fixture");

    assert_eq!(
        result.trapped_volumes.len(),
        2,
        "two disjoint cavities must produce exactly two trapped-volume regions"
    );
}

#[test]
fn test_trapped_volume_critical_for_sla() {
    // SLA + 2 mm cavity (volume 8 mm³) ≫ res_volume = 0.8³ = 0.512
    // → severity Critical (trapped uncured resin is a hard failure).
    let mesh = make_cube_with_inner_cavity(6.0, 2.0);
    let config = coarse_voxel_config(PrintTechnology::Sla);
    let result = validate_for_printing(&mesh, &config).expect("validation should succeed for SLA");

    let critical_trapped = result.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::TrappedVolume && i.severity == IssueSeverity::Critical
    });
    assert!(
        critical_trapped,
        "SLA + cavity above resolution → Critical severity"
    );
}

#[test]
fn test_trapped_volume_critical_for_sls() {
    // SLS + 2 mm cavity → severity Critical (trapped unsintered powder).
    let mesh = make_cube_with_inner_cavity(6.0, 2.0);
    let config = coarse_voxel_config(PrintTechnology::Sls);
    let result = validate_for_printing(&mesh, &config).expect("validation should succeed for SLS");

    let critical_trapped = result.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::TrappedVolume && i.severity == IssueSeverity::Critical
    });
    assert!(
        critical_trapped,
        "SLS + cavity above resolution → Critical severity"
    );
}

#[test]
fn test_trapped_volume_critical_for_mjf() {
    // MJF + 2 mm cavity → severity Critical (trapped unfused powder).
    let mesh = make_cube_with_inner_cavity(6.0, 2.0);
    let config = coarse_voxel_config(PrintTechnology::Mjf);
    let result = validate_for_printing(&mesh, &config).expect("validation should succeed for MJF");

    let critical_trapped = result.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::TrappedVolume && i.severity == IssueSeverity::Critical
    });
    assert!(
        critical_trapped,
        "MJF + cavity above resolution → Critical severity"
    );
}

#[test]
fn test_trapped_volume_info_for_fdm() {
    // FDM + 2 mm cavity → severity Info (sealed cavity prints fine on
    // extrusion). Cavity is detected; the flag is just informational.
    let mesh = make_cube_with_inner_cavity(6.0, 2.0);
    let config = coarse_voxel_config(PrintTechnology::Fdm);
    let result = validate_for_printing(&mesh, &config).expect("validation should succeed for FDM");

    let any_trapped_critical = result.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::TrappedVolume && i.severity == IssueSeverity::Critical
    });
    assert!(
        !any_trapped_critical,
        "FDM trapped volume must NOT be Critical"
    );
    let any_trapped_info = result.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::TrappedVolume && i.severity == IssueSeverity::Info
    });
    assert!(any_trapped_info, "FDM trapped volume must be Info");
}

#[test]
fn test_trapped_volume_info_below_min_feature() {
    // Tiny cavity (0.6 mm cube → analytical 0.216 mm³) under config
    // with min_feature_size = 0.8 (res_volume = 0.512 mm³). Volume is
    // below resolution → severity Info regardless of technology.
    let mesh = make_cube_with_inner_cavity(6.0, 0.6);
    let config = coarse_voxel_config(PrintTechnology::Sla);
    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for sub-resolution cavity");

    // The detector may emit a region (it depends on whether the
    // 0.6 mm cavity at voxel 0.2 mm contains any non-leak voxels).
    // What's load-bearing: any trapped issue reported is Info, never
    // Critical/Warning.
    let any_critical = result.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::TrappedVolume && i.severity == IssueSeverity::Critical
    });
    assert!(
        !any_critical,
        "sub-resolution cavity must NOT be Critical even on SLA"
    );
    for issue in &result.issues {
        if issue.issue_type == PrintIssueType::TrappedVolume {
            assert_eq!(
                issue.severity,
                IssueSeverity::Info,
                "all sub-resolution trapped issues must be Info"
            );
        }
    }
}

#[test]
fn test_trapped_volume_volume_within_10pct_of_analytical() {
    // 6 mm outer cube + 2 mm cube cavity → analytical volume 8 mm³.
    // Voxel-discretized volume must be within ±10% of analytical
    // (per §9.6 + §6.3 line 1136 tolerance band; absorbs voxel
    // discretization noise + cross-platform ULP variance).
    let mesh = make_cube_with_inner_cavity(6.0, 2.0);
    let config = coarse_voxel_config(PrintTechnology::Fdm);
    let result = validate_for_printing(&mesh, &config).expect("validation should succeed");

    assert_eq!(result.trapped_volumes.len(), 1);
    let analytical_volume = 2.0_f64.powi(3); // cube cavity volume
    let voxel_volume = result.trapped_volumes[0].volume;
    approx::assert_relative_eq!(voxel_volume, analytical_volume, max_relative = 0.10);
}

#[test]
fn test_trapped_volume_sort_stable_across_runs() {
    // Two disjoint cavities → two regions. §4.4 mandates sort by
    // (center.x, center.y, center.z) via total_cmp; back-to-back
    // runs must produce identical centers in the same order.
    let mesh = make_cube_with_two_inner_cavities(8.0, 1.0, 1.5);
    let config = coarse_voxel_config(PrintTechnology::Fdm);

    let r1 = validate_for_printing(&mesh, &config).expect("validation should succeed (run 1)");
    let r2 = validate_for_printing(&mesh, &config).expect("validation should succeed (run 2)");

    assert_eq!(r1.trapped_volumes.len(), 2);
    assert_eq!(r2.trapped_volumes.len(), 2);
    for (a, b) in r1.trapped_volumes.iter().zip(r2.trapped_volumes.iter()) {
        assert!((a.center.x - b.center.x).abs() < 1e-9);
        assert!((a.center.y - b.center.y).abs() < 1e-9);
        assert!((a.center.z - b.center.z).abs() < 1e-9);
    }
    // Ascending §4.4 sort: first center has smaller (x, y, z).
    assert!(
        r1.trapped_volumes[0].center.x <= r1.trapped_volumes[1].center.x,
        "§4.4: trapped_volumes must sort by center.x ascending"
    );
}

#[test]
fn test_trapped_volume_large_extent_uses_axis_cap() {
    // S3 of `docs/CF_CAST_F4_SPATIAL_INDEX_RECON.md` §B-12: large
    // parts have their voxel_size scaled up by the `MAX_VOXELS
    // _PER_AXIS = 500` cap so the trapped-volumes algorithm
    // completes in bounded time + memory. Pre-S3 this test
    // verified the 1 GB memory pre-flight skipped a 2000 mm
    // fixture; post-S3 the voxel-axis cap keeps the grid at
    // ≤ 500³ × 1 byte = 125 MB regardless of part extent, so
    // the 1 GB skip never fires in practice + the algorithm
    // runs normally.
    let outer_extent = 2000.0;
    let mesh = make_cube_with_inner_cavity(outer_extent, outer_extent / 3.0);
    let config = coarse_voxel_config(PrintTechnology::Fdm);
    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed under voxel-axis cap");

    // No DetectorSkipped emitted — the algorithm runs to
    // completion with a scaled voxel_size.
    let trapped_skipped = result.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::DetectorSkipped && i.description.contains("TrappedVolume")
    });
    assert!(
        !trapped_skipped,
        "voxel-axis cap should allow trapped_volumes to complete \
             instead of skipping for large parts"
    );
    // The cavity (outer_extent / 3 = ~667 mm) should still flag
    // as a trapped volume despite the coarsened voxel — 667 mm
    // is far above any per-axis cap-induced voxel size
    // (2000 / 500 = 4 mm voxel).
    assert!(
        !result.trapped_volumes.is_empty(),
        "voxel-axis-capped algorithm must still detect the inner cavity"
    );
}

// ===== §6.4 Gap I — SelfIntersecting detector tests ==================

/// Two interpenetrating triangles that share NO vertices. Triangle A
/// lies in the `z = 0` plane with `x ∈ [0, 10]`, `y ∈ [0, 10]`;
/// triangle B lies in the `y = 5` plane with `z ∈ [-5, 5]`. B's
/// edges cross `z = 0` at `(4, 5, 0)` and `(6, 5, 0)` — a segment
/// lying inside A's interior. mesh-repair flags the pair via
/// Möller-Trumbore at `epsilon = 1e-10`.
fn make_two_intersecting_triangles() -> IndexedMesh {
    let vertices = vec![
        // Triangle A — XY plane.
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(5.0, 10.0, 0.0),
        // Triangle B — y=5 plane, vertically oriented.
        Point3::new(3.0, 5.0, -5.0),
        Point3::new(7.0, 5.0, -5.0),
        Point3::new(5.0, 5.0, 5.0),
    ];
    let faces = vec![[0, 1, 2], [3, 4, 5]];
    IndexedMesh::from_parts(vertices, faces)
}

#[test]
fn test_self_intersecting_two_overlapping_triangles() {
    let mesh = make_two_intersecting_triangles();
    let config = PrinterConfig::fdm_default();
    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed on two-triangle fixture");

    assert!(
        !result.self_intersecting.is_empty(),
        "two interpenetrating non-adjacent triangles must produce ≥ 1 region"
    );
    let any_critical = result.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::SelfIntersecting && i.severity == IssueSeverity::Critical
    });
    assert!(
        any_critical,
        "self-intersection issue must be Critical (slicer behavior undefined)"
    );
}

#[test]
fn test_self_intersecting_clean_cube_no_issue() {
    // Clean watertight cube — no self-intersections.
    let mesh = create_watertight_cube();
    let config = PrinterConfig::fdm_default();
    let result =
        validate_for_printing(&mesh, &config).expect("validation should succeed on clean cube");

    assert_eq!(
        result.self_intersecting.len(),
        0,
        "clean watertight cube must produce 0 self-intersecting regions"
    );
    let any_si_issue = result
        .issues
        .iter()
        .any(|i| i.issue_type == PrintIssueType::SelfIntersecting);
    assert!(
        !any_si_issue,
        "no self-intersecting issue should be emitted on a clean cube"
    );
}

#[test]
fn test_self_intersecting_adjacent_triangles_skipped() {
    // Two triangles sharing an edge `(0)-(1)`. mesh-repair's
    // `skip_adjacent = true` filters edge-sharing pairs out before
    // Möller-Trumbore, so no intersection is reported.
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.5, 1.0, 0.0),
        Point3::new(0.5, -1.0, 0.0),
    ];
    let faces = vec![[0, 1, 2], [0, 1, 3]];
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();
    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed on edge-adjacent pair");

    assert_eq!(
        result.self_intersecting.len(),
        0,
        "edge-adjacent triangles must be skipped (skip_adjacent = true)"
    );
}

#[test]
fn test_self_intersecting_truncation_at_100() {
    // Light unit-test version — assert issue-description structure
    // on the non-truncated path (one intersecting pair → no
    // truncation suffix). The heavy 100-cap fixture lives at
    // `stress_i_truncation_at_100` per §9.2.6.
    let mesh = make_two_intersecting_triangles();
    let config = PrinterConfig::fdm_default();
    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed on two-triangle fixture");

    let issue = result
        .issues
        .iter()
        .find(|i| i.issue_type == PrintIssueType::SelfIntersecting);
    let issue = issue.expect("self-intersecting issue must be present");
    assert!(
        issue
            .description
            .contains("self-intersecting triangle pair"),
        "issue description must mention 'self-intersecting triangle pair'; got `{}`",
        issue.description
    );
    assert!(
        !issue.description.contains("truncated"),
        "single-pair fixture must NOT contain truncation suffix; got `{}`",
        issue.description
    );
}

#[test]
fn test_self_intersecting_approximate_location_in_midpoint() {
    // For `make_two_intersecting_triangles`:
    // - Triangle A centroid = ((0+10+5)/3, (0+0+10)/3, 0) = (5, 10/3, 0).
    // - Triangle B centroid = ((3+7+5)/3, 5, (-5-5+5)/3) = (5, 5, -5/3).
    // - Midpoint = (5, (10/3 + 5)/2, (0 + -5/3)/2) = (5, 25/6, -5/6).
    let mesh = make_two_intersecting_triangles();
    let config = PrinterConfig::fdm_default();
    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed on two-triangle fixture");

    assert_eq!(result.self_intersecting.len(), 1);
    let region = &result.self_intersecting[0];
    let expected_x = 5.0;
    let expected_y = f64::midpoint(10.0 / 3.0, 5.0);
    let expected_z = -5.0 / 6.0;
    approx::assert_relative_eq!(
        region.approximate_location.x,
        expected_x,
        epsilon = EPS_GEOMETRIC
    );
    approx::assert_relative_eq!(
        region.approximate_location.y,
        expected_y,
        epsilon = EPS_GEOMETRIC
    );
    approx::assert_relative_eq!(
        region.approximate_location.z,
        expected_z,
        epsilon = EPS_GEOMETRIC
    );
}

#[test]
fn test_self_intersecting_critical_blocks_is_printable() {
    let mesh = make_two_intersecting_triangles();
    let config = PrinterConfig::fdm_default();
    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed on two-triangle fixture");

    assert!(
        !result.is_printable(),
        "any self-intersecting pair must flip is_printable() to false"
    );
}

#[test]
fn test_self_intersecting_sort_stable() {
    // Two consecutive runs on the same fixture must produce the
    // same `(face_a, face_b)` ordering — locks §4.4 sort.
    let mesh = make_two_intersecting_triangles();
    let config = PrinterConfig::fdm_default();
    let r1 = validate_for_printing(&mesh, &config).expect("validation should succeed (run 1)");
    let r2 = validate_for_printing(&mesh, &config).expect("validation should succeed (run 2)");

    assert_eq!(r1.self_intersecting.len(), r2.self_intersecting.len());
    for (a, b) in r1.self_intersecting.iter().zip(r2.self_intersecting.iter()) {
        assert_eq!(a.face_a, b.face_a);
        assert_eq!(a.face_b, b.face_b);
    }
}

#[test]
fn test_self_intersecting_face_indices_unique_per_pair() {
    // Canonical `face_a < face_b` per §6.4 line 1162.
    let mesh = make_two_intersecting_triangles();
    let config = PrinterConfig::fdm_default();
    let result = validate_for_printing(&mesh, &config)
        .expect("validation should succeed on two-triangle fixture");

    for region in &result.self_intersecting {
        assert!(
            region.face_a < region.face_b,
            "§6.4 canonical ordering: face_a ({}) must be < face_b ({})",
            region.face_a,
            region.face_b
        );
    }
}

// ===== §6.5 Gap J — SmallFeature unit tests =========================

/// Build an axis-aligned 1-triangle "fragment" with extent
/// `(side, side, 0.0)` at +Z = z, anchored at origin `(cx, cy)`.
fn make_isolated_triangle(cx: f64, cy: f64, z: f64, side: f64) -> IndexedMesh {
    let vertices = vec![
        Point3::new(cx, cy, z),
        Point3::new(cx + side, cy, z),
        Point3::new(cx, cy + side, z),
    ];
    let faces = vec![[0, 1, 2]];
    IndexedMesh::from_parts(vertices, faces)
}

/// Build a watertight 12-triangle cube at `origin` with side `side`,
/// using the same outward winding as `create_watertight_cube`. Vertex
/// indices start at `index_offset` (so the fixture can be merged into
/// a multi-component mesh with no index collisions).
fn append_unit_cube_at(
    vertices: &mut Vec<Point3<f64>>,
    faces: &mut Vec<[u32; 3]>,
    origin: Point3<f64>,
    side: f64,
) {
    // Cast: `vertices.len()` ≤ mesh vertex count, well under 2^32.
    let i = vertices.len() as u32;
    let x0 = origin.x;
    let y0 = origin.y;
    let z0 = origin.z;
    let x1 = x0 + side;
    let y1 = y0 + side;
    let z1 = z0 + side;
    vertices.extend_from_slice(&[
        Point3::new(x0, y0, z0),
        Point3::new(x1, y0, z0),
        Point3::new(x1, y1, z0),
        Point3::new(x0, y1, z0),
        Point3::new(x0, y0, z1),
        Point3::new(x1, y0, z1),
        Point3::new(x1, y1, z1),
        Point3::new(x0, y1, z1),
    ]);
    faces.extend_from_slice(&[
        // Bottom (Z = z0), top (Z = z1) — outward winding matches
        // `create_watertight_cube`.
        [i, i + 2, i + 1],
        [i, i + 3, i + 2],
        [i + 4, i + 5, i + 6],
        [i + 4, i + 6, i + 7],
        [i, i + 1, i + 5],
        [i, i + 5, i + 4],
        [i + 3, i + 6, i + 2],
        [i + 3, i + 7, i + 6],
        [i, i + 4, i + 7],
        [i, i + 7, i + 3],
        [i + 1, i + 2, i + 6],
        [i + 1, i + 6, i + 5],
    ]);
}

fn make_unit_cube_at(origin: Point3<f64>, side: f64) -> IndexedMesh {
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_unit_cube_at(&mut vertices, &mut faces, origin, side);
    IndexedMesh::from_parts(vertices, faces)
}

#[test]
fn test_small_feature_floating_triangle_detected() {
    // 100 mm cube + isolated 0.1 mm triangle. min_feature_size = 0.4
    // → triangle flags as Warning (0.1 < 0.4 / 2 = 0.2); cube does
    // not (max_extent 100 ≫ 0.4).
    let (mesh, _tri) = make_cube_plus_floating_triangle();
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 0.4;
    let result = validate_for_printing(&mesh, &config)
        .expect("validation must succeed on cube + floating-triangle fixture");

    assert_eq!(
        result.small_features.len(),
        1,
        "exactly one small-feature region (the 0.1 mm triangle)"
    );
    let region = &result.small_features[0];
    assert!(
        (region.max_extent - 0.1).abs() < 1e-9,
        "max_extent ≈ 0.1 mm; got {}",
        region.max_extent
    );
    assert_eq!(region.face_count, 1);

    // Per-site `expect_used` allow: assertion-target lookup; if the
    // detector's emission contract breaks, panicking with a clear
    // message is the right test failure mode.
    let issue = result
        .issues
        .iter()
        .find(|i| matches!(i.issue_type, PrintIssueType::SmallFeature))
        .expect("SmallFeature issue must be emitted");
    assert_eq!(
        issue.severity,
        IssueSeverity::Warning,
        "0.1 < 0.4 / 2 = 0.2 → Warning"
    );
}

#[test]
fn test_small_feature_borderline_no_issue() {
    // max_extent = 0.5 mm, min_feature_size = 0.4 → 0.5 NOT < 0.4 →
    // not flagged. The detector uses strict `<`, not `≤`.
    let mesh = make_isolated_triangle(0.0, 0.0, 0.0, 0.5);
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 0.4;
    let result = validate_for_printing(&mesh, &config)
        .expect("validation must succeed on borderline fixture");

    assert_eq!(
        result.small_features.len(),
        0,
        "max_extent ≥ min_feature_size → not flagged"
    );
    assert!(
        !result
            .issues
            .iter()
            .any(|i| matches!(i.issue_type, PrintIssueType::SmallFeature)),
        "no SmallFeature issue at the borderline"
    );
}

#[test]
fn test_small_feature_below_half_threshold_warning() {
    // max_extent = 0.1 mm, min_feature_size = 0.4 → flagged
    // (0.1 < 0.4) AND severity = Warning (0.1 < 0.4 / 2 = 0.2).
    let mesh = make_isolated_triangle(0.0, 0.0, 0.0, 0.1);
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 0.4;
    let result = validate_for_printing(&mesh, &config)
        .expect("validation must succeed on below-half-threshold fixture");

    assert_eq!(result.small_features.len(), 1);
    // Per-site `expect_used` allow: assertion-target lookup; if the
    // detector's emission contract breaks, panicking with a clear
    // message is the right test failure mode.
    let issue = result
        .issues
        .iter()
        .find(|i| matches!(i.issue_type, PrintIssueType::SmallFeature))
        .expect("SmallFeature issue must be emitted");
    assert_eq!(issue.severity, IssueSeverity::Warning);
}

#[test]
fn test_small_feature_just_below_threshold_info() {
    // max_extent = 0.3 mm, min_feature_size = 0.4 → flagged
    // (0.3 < 0.4) AND severity = Info (0.3 ≥ 0.4 / 2 = 0.2).
    let mesh = make_isolated_triangle(0.0, 0.0, 0.0, 0.3);
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 0.4;
    let result = validate_for_printing(&mesh, &config)
        .expect("validation must succeed on just-below-threshold fixture");

    assert_eq!(result.small_features.len(), 1);
    // Per-site `expect_used` allow: assertion-target lookup; if the
    // detector's emission contract breaks, panicking with a clear
    // message is the right test failure mode.
    let issue = result
        .issues
        .iter()
        .find(|i| matches!(i.issue_type, PrintIssueType::SmallFeature))
        .expect("SmallFeature issue must be emitted");
    assert_eq!(issue.severity, IssueSeverity::Info);
}

#[test]
fn test_small_feature_clean_main_body_not_flagged() {
    // 100 mm watertight cube; max_extent 100 ≫ 0.8 (FDM default) →
    // 0 small-feature regions.
    let mesh = make_unit_cube_at(Point3::new(0.0, 0.0, 0.0), 100.0);
    let config = PrinterConfig::fdm_default();
    let result = validate_for_printing(&mesh, &config)
        .expect("validation must succeed on clean 100 mm cube");

    assert_eq!(
        result.small_features.len(),
        0,
        "main body must not be flagged (max_extent 100 ≫ 0.8)"
    );
}

#[test]
fn test_small_feature_two_floating_fragments() {
    // 100 mm cube + 2 disjoint 0.1 mm fragments at +X = 200 / 300 mm
    // (well outside the cube) → 2 small-feature regions (cube
    // unaffected). Fragments share no vertex indices with the cube
    // or each other, so they form separate components.
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_unit_cube_at(&mut vertices, &mut faces, Point3::new(0.0, 0.0, 0.0), 100.0);

    // Fragment A at (200, 0, 0).
    // Cast as in `append_unit_cube_at`.
    let base_a = vertices.len() as u32;
    vertices.extend_from_slice(&[
        Point3::new(200.0, 0.0, 0.0),
        Point3::new(200.1, 0.0, 0.0),
        Point3::new(200.0, 0.1, 0.0),
    ]);
    faces.push([base_a, base_a + 1, base_a + 2]);

    // Fragment B at (300, 0, 0).
    let base_b = vertices.len() as u32;
    vertices.extend_from_slice(&[
        Point3::new(300.0, 0.0, 0.0),
        Point3::new(300.1, 0.0, 0.0),
        Point3::new(300.0, 0.1, 0.0),
    ]);
    faces.push([base_b, base_b + 1, base_b + 2]);

    let mesh = IndexedMesh::from_parts(vertices, faces);
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 0.4;
    let result = validate_for_printing(&mesh, &config)
        .expect("validation must succeed on cube + 2-fragment fixture");

    assert_eq!(
        result.small_features.len(),
        2,
        "two fragments expected; cube unaffected"
    );
}

#[test]
fn test_small_feature_face_adjacency_via_edge_only() {
    // Two triangles sharing only vertex index 2 (no shared edge) →
    // 2 separate components per the §6.5 line 1280 edge-adjacency
    // contract.
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0),   // 0
        Point3::new(0.1, 0.0, 0.0),   // 1
        Point3::new(0.05, 0.05, 0.0), // 2 (shared)
        Point3::new(0.1, 0.05, 0.0),  // 3
        Point3::new(0.05, 0.1, 0.0),  // 4
    ];
    let faces = vec![[0, 1, 2], [2, 3, 4]];
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 0.4;
    let result = validate_for_printing(&mesh, &config)
        .expect("validation must succeed on vertex-only-shared fixture");

    assert_eq!(
        result.small_features.len(),
        2,
        "vertex-only sharing is not edge-adjacency; expect 2 components"
    );
    for region in &result.small_features {
        assert_eq!(region.face_count, 1);
    }
}

#[test]
fn test_small_feature_volume_via_divergence_theorem() {
    // Unit cube (1 mm side, exact-representable verts), min_feature
    // = 2.0 → flagged (max_extent 1.0 < 2.0). Volume must be ≈ 1.0
    // mm³ within 1e-6 (FP-stable on exact-representable inputs).
    let mesh = make_unit_cube_at(Point3::new(0.0, 0.0, 0.0), 1.0);
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 2.0;
    let result = validate_for_printing(&mesh, &config)
        .expect("validation must succeed on unit-cube fixture");

    assert_eq!(result.small_features.len(), 1);
    let region = &result.small_features[0];
    assert!(
        (region.volume - 1.0).abs() < 1e-6,
        "divergence-theorem volume ≈ 1.0 mm³ within 1e-6; got {}",
        region.volume
    );
}

#[test]
fn test_small_feature_open_component_no_panic() {
    // 5-of-6 face cube (top removed) at side = 1 mm, min_feature =
    // 2.0 → flagged. Open mesh; signed_volume is non-physical but
    // `abs(...)` guarantees finite, non-negative output.
    let x0 = 0.0;
    let y0 = 0.0;
    let z0 = 0.0;
    let x1 = 1.0;
    let y1 = 1.0;
    let z1 = 1.0;
    let vertices = vec![
        Point3::new(x0, y0, z0),
        Point3::new(x1, y0, z0),
        Point3::new(x1, y1, z0),
        Point3::new(x0, y1, z0),
        Point3::new(x0, y0, z1),
        Point3::new(x1, y0, z1),
        Point3::new(x1, y1, z1),
        Point3::new(x0, y1, z1),
    ];
    // 5 sides, no top: bottom + front + back + left + right.
    let faces = vec![
        [0, 2, 1],
        [0, 3, 2], // bottom
        [0, 1, 5],
        [0, 5, 4], // front
        [3, 6, 2],
        [3, 7, 6], // back
        [0, 4, 7],
        [0, 7, 3], // left
        [1, 2, 6],
        [1, 6, 5], // right
    ];
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 2.0;
    let result = validate_for_printing(&mesh, &config)
        .expect("validation must succeed on open-component fixture");

    assert_eq!(result.small_features.len(), 1);
    let region = &result.small_features[0];
    assert!(
        region.volume.is_finite() && region.volume >= 0.0,
        "open-component volume must be finite and non-negative; got {}",
        region.volume
    );
}

#[test]
fn test_small_feature_sort_stable() {
    // Same input run twice → identical region order. Verifies §4.4
    // sort-by-min-face-index. Two fragments with face indices in
    // intentionally non-sorted Vec construction order; expected
    // sort puts fragment A (lower face index) first.
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    // Cast as in `append_unit_cube_at`.
    let base_a = vertices.len() as u32;
    vertices.extend_from_slice(&[
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.1, 0.0, 0.0),
        Point3::new(0.0, 0.1, 0.0),
    ]);
    faces.push([base_a, base_a + 1, base_a + 2]); // face 0
    let base_b = vertices.len() as u32;
    vertices.extend_from_slice(&[
        Point3::new(50.0, 0.0, 0.0),
        Point3::new(50.1, 0.0, 0.0),
        Point3::new(50.0, 0.1, 0.0),
    ]);
    faces.push([base_b, base_b + 1, base_b + 2]); // face 1

    let mesh = IndexedMesh::from_parts(vertices, faces);
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 0.4;

    let r1 = validate_for_printing(&mesh, &config).expect("validation run 1 must succeed");
    let r2 = validate_for_printing(&mesh, &config).expect("validation run 2 must succeed");

    assert_eq!(r1.small_features.len(), 2);
    assert_eq!(r1.small_features.len(), r2.small_features.len());
    for (a, b) in r1.small_features.iter().zip(r2.small_features.iter()) {
        assert_eq!(
            a.faces, b.faces,
            "§4.4 sort stability: identical face vectors across runs"
        );
    }
    // Lowest-face-idx fragment first per §4.4.
    assert_eq!(r1.small_features[0].faces, vec![0]);
    assert_eq!(r1.small_features[1].faces, vec![1]);
}

// =============================================================================
// Gap-scan backfill — behaviors previously guarded ONLY by the
// `example-printability-stress-test` modules, promoted to CI-gated lib
// tests (the example asserts remain as the secondary net). Audit source:
// the printability fold's example→guard gap-scan.
// =============================================================================

/// 100 mm cube + an isolated 0.1 mm floating triangle at x = 200 mm (well
/// outside the cube, so the two are not edge-adjacent). Under
/// `min_feature_size = 0.4` the triangle flags as a `SmallFeature`
/// Warning and the cube does not. Returns the mesh + the triangle's 3 vertices
/// (for centroid assertions). Shared by the two floating-triangle tests.
fn make_cube_plus_floating_triangle() -> (IndexedMesh, [Point3<f64>; 3]) {
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_unit_cube_at(&mut vertices, &mut faces, Point3::new(0.0, 0.0, 0.0), 100.0);
    let base = u32::try_from(vertices.len()).expect("vertex count fits in u32");
    let tri = [
        Point3::new(200.0, 0.0, 0.0),
        Point3::new(200.1, 0.0, 0.0),
        Point3::new(200.0, 0.1, 0.0),
    ];
    vertices.extend_from_slice(&tri);
    faces.push([base, base + 1, base + 2]);
    (IndexedMesh::from_parts(vertices, faces), tri)
}

#[test]
fn test_thin_wall_cross_technology_severity_matrix() {
    // Gap #1: the headline "one part, four verdicts" behavior. A FIXED
    // 0.45 mm wall is steered into a DIFFERENT severity band by each
    // technology's `min_wall_thickness`, on the SAME geometry. Every
    // other ThinWall test runs under `fdm_default` (min_wall = 1.0) and
    // varies the slab thickness; nothing else drives the detector through
    // a non-FDM `PrinterConfig`, so the per-tech config path + the
    // divergence itself were example-only.
    //
    // 0.45 mm is a clean INTERIOR point for every tech's bands — at least
    // 0.05 mm from each threshold, far outside the ±1e-6 ray-offset FP
    // window — so no verdict sits on a strict-`<` boundary knife-edge
    // (the strict-`<` boundary itself is guarded by
    // test_thin_wall_borderline_no_issue):
    //   FDM min_wall 1.0: 0.45 < 0.5 = min_wall/2  → Critical.
    //   SLA min_wall 0.4: 0.45 ≥ 0.4               → not flagged.
    //   SLS min_wall 0.7: 0.35 ≤ 0.45 < 0.7        → Warning.
    //   MJF min_wall 0.5: 0.25 ≤ 0.45 < 0.5        → Warning.
    let mesh = make_thin_slab(0.45);
    let matrix = [
        (
            PrinterConfig::fdm_default(),
            "FDM",
            Some(IssueSeverity::Critical),
        ),
        (PrinterConfig::sla_default(), "SLA", None),
        (
            PrinterConfig::sls_default(),
            "SLS",
            Some(IssueSeverity::Warning),
        ),
        (
            PrinterConfig::mjf_default(),
            "MJF",
            Some(IssueSeverity::Warning),
        ),
    ];

    for (config, tech, want) in matrix {
        let result = validate_for_printing(&mesh, &config)
            .unwrap_or_else(|e| panic!("validation should succeed under {tech}: {e}"));
        let severities: Vec<IssueSeverity> = result
            .issues
            .iter()
            .filter(|i| i.issue_type == PrintIssueType::ThinWall)
            .map(|i| i.severity)
            .collect();
        match want {
            None => assert!(
                result.thin_walls.is_empty() && severities.is_empty(),
                "{tech}: 0.45 mm ≥ min_wall must NOT flag ThinWall; got {severities:?}"
            ),
            Some(sev) => {
                assert!(
                    !result.thin_walls.is_empty(),
                    "{tech}: 0.45 mm wall below min_wall must flag ThinWall"
                );
                assert!(
                    !severities.is_empty() && severities.iter().all(|s| *s == sev),
                    "{tech}: every ThinWall issue must be {sev:?} on the 0.45 mm wall; \
                     got {severities:?}"
                );
            }
        }
    }
}

#[test]
fn test_small_feature_region_center_is_component_vertex_mean() {
    // Gap #2: the DETECTOR-COMPUTED region centroid. `regions.rs` only
    // exercises the `SmallFeatureRegion::new` constructor; nothing
    // asserts `small_features[*].center` as produced by
    // `validate_for_printing`, so a component-centroid regression would
    // pass the whole suite. Shares the floating-triangle fixture with
    // test_small_feature_floating_triangle_detected.
    let (mesh, tri) = make_cube_plus_floating_triangle();
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 0.4;
    let result = validate_for_printing(&mesh, &config).expect("validation must succeed");

    assert_eq!(
        result.small_features.len(),
        1,
        "only the 0.1 mm triangle flags"
    );
    let center = result.small_features[0].center;
    let mean_x = (tri[0].x + tri[1].x + tri[2].x) / 3.0;
    let mean_y = (tri[0].y + tri[1].y + tri[2].y) / 3.0;
    let mean_z = (tri[0].z + tri[1].z + tri[2].z) / 3.0;
    assert!(
        (center.x - mean_x).abs() < 1e-9
            && (center.y - mean_y).abs() < 1e-9
            && (center.z - mean_z).abs() < 1e-9,
        "SmallFeature centroid must be the component vertex mean ({mean_x}, {mean_y}, {mean_z}); \
         got ({}, {}, {})",
        center.x,
        center.y,
        center.z
    );
}

#[test]
fn test_small_feature_and_thin_wall_co_flag_blocks_printability() {
    // Gap #3: cross-detector convergence. A sub-`min_feature`,
    // sub-`min_wall` isolated solid raises BOTH a SmallFeature Warning
    // AND a ThinWall Critical — and the Critical is what drives
    // `is_printable() == false`. Lib fixtures deliberately isolate one
    // detector; `stress_cross_unit_conversion_full_pipeline` notes the
    // co-flag but explicitly declines to assert `thin_walls`.
    //
    // A 0.3 mm watertight cube: max_extent 0.3 < min_feature/2 = 0.4 →
    // SmallFeature Warning; opposite walls 0.3 mm apart < min_wall/2 =
    // 0.5 → ThinWall Critical.
    let mesh = make_unit_cube_at(Point3::new(0.0, 0.0, 0.0), 0.3);

    let mut config = PrinterConfig::fdm_default(); // min_wall 1.0
    config.min_feature_size = 0.8;
    let result = validate_for_printing(&mesh, &config).expect("validation must succeed");

    let small_warning = result.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::SmallFeature && i.severity == IssueSeverity::Warning
    });
    let thin_critical = result
        .issues
        .iter()
        .any(|i| i.issue_type == PrintIssueType::ThinWall && i.severity == IssueSeverity::Critical);
    assert!(
        small_warning,
        "0.3 mm feature < 0.8/2 = 0.4 → SmallFeature Warning"
    );
    assert!(
        thin_critical,
        "0.3 mm wall < 1.0/2 = 0.5 → ThinWall Critical"
    );
    assert!(
        !result.is_printable(),
        "the ThinWall Critical co-flag must block printability (SmallFeature alone is advisory)"
    );
}

#[test]
fn test_long_bridge_endpoint_coordinates_span_the_bridge_axis() {
    // Gap #4: the back-projected `start`/`end` endpoint COORDINATES.
    // Lib tests assert `span` and `start.x` *ordering* but never the
    // endpoint values, so a back-projection bug (wrong axis, wrong
    // perpendicular-midpoint, wrong z-elevation) would slip through.
    // Fixture: 20×5 slab bottom at z = 10, span along X.
    let mesh = make_closed_bridge_fixture(20.0, 5.0, 1.5, 10.0);
    let result = validate_for_printing(&mesh, &PrinterConfig::fdm_default())
        .expect("validation should succeed");

    assert_eq!(result.long_bridges.len(), 1);
    let region = &result.long_bridges[0];
    // Pin the actual endpoint COORDINATES, not just |Δx| == span (which
    // holds by construction: endpoints = center ∓ axis·half_span). The
    // slab bottom spans x ∈ [0, 20] at the perpendicular midpoint
    // y = 2.5 (mid of [0, 5]), on the flagged bottom-face plane z = 10.
    // A wrong-perpendicular-midpoint or wrong-long-axis-center
    // back-projection shifts these values and is caught here.
    let x_lo = region.start.x.min(region.end.x);
    let x_hi = region.start.x.max(region.end.x);
    assert!(
        (x_lo - 0.0).abs() < 1e-6,
        "bridge low endpoint x = 0; got {x_lo}"
    );
    assert!(
        (x_hi - 20.0).abs() < 1e-6,
        "bridge high endpoint x = 20 (slab x-extent); got {x_hi}"
    );
    assert!(
        (region.start.y - 2.5).abs() < 1e-6 && (region.end.y - 2.5).abs() < 1e-6,
        "both endpoints at perpendicular midpoint y = 2.5; got start.y = {}, end.y = {}",
        region.start.y,
        region.end.y
    );
    assert!(
        (region.start.z - 10.0).abs() < 1e-6 && (region.end.z - 10.0).abs() < 1e-6,
        "endpoints on the bridge's z = 10 plane; got start.z = {}, end.z = {}",
        region.start.z,
        region.end.z
    );
    assert!(
        (region.span - 20.0).abs() < 1e-6,
        "span = 20 mm; got {}",
        region.span
    );
}

#[test]
fn test_overhang_area_equivalent_across_mesh_and_up_vector_rotation() {
    // Gap #5 (Gap L, area level): flagging the same physical overhang
    // two ways — rotating the MESH (-90° about X, default +Z up) vs
    // rotating the VALIDATOR's up-vector (+Y up on the original mesh) —
    // must yield the same overhang AREA, not merely the same region
    // count (the count-level equivalence is guarded by
    // test_overhang_with_y_up_orientation). This is the mesh-rotation ≡
    // up-vector-rotation invariant the orientation example demonstrates
    // on a leaning cylinder.
    let mesh_z = make_overhang_fixture(std::f64::consts::FRAC_PI_2, 5.0);
    let mesh_y = rotate_neg_90_about_x(&mesh_z);

    let result_z = validate_for_printing(&mesh_z, &PrinterConfig::fdm_default())
        .expect("validation should succeed for +Z-up roof");
    let result_y = validate_for_printing(
        &mesh_y,
        &PrinterConfig::fdm_default().with_build_up_direction(Vector3::new(0.0, 1.0, 0.0)),
    )
    .expect("validation should succeed for +Y-up rotated roof");

    let area_z: f64 = result_z.overhangs.iter().map(|r| r.area).sum();
    let area_y: f64 = result_y.overhangs.iter().map(|r| r.area).sum();
    assert!(
        area_z > 0.0,
        "+Z-up roof must flag a non-zero overhang area"
    );
    assert!(
        (area_z - area_y).abs() < 1e-9,
        "overhang AREA must match across mesh-rotation and up-vector-rotation routes; \
         got {area_z} (+Z up) vs {area_y} (+Y up)"
    );
}
