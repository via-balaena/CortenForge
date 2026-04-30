//! Stress-test gauntlet for mesh-printability per spec §9.
//!
//! Adversarial inputs that exercise the public `validate_for_printing` API
//! at the edges of detector preconditions, FP stability, and topology
//! corners. Each commit in the v0.8 fix arc appends the §9.<gap> stress
//! fixtures for the gap that commit lands; commits do not modify
//! previously-authored fixtures (per spec §9.5).
//!
//! This file lands in commit #2 (Gap M). Commit #2 covers:
//!
//! - §9.2.1 #1: existing-detector — empty-mesh error preserved.
//! - §9.2.2 #1–#4: Gap M predicate + build-plate filter.
//!
//! Deferred fixtures and where they land:
//!
//! - §9.2.1 #2 `stress_existing_single_triangle_open_mesh` and
//!   §9.2.1 #3 `stress_existing_two_faces_vertex_only_shared` reference
//!   the `DetectorSkipped` issue variant (commit #9 / §5.8) and the
//!   `ThinWall` + `TrappedVolume` detectors (commits #10 / §6.1 and
//!   #14 / §6.3). They land with commit #14 (the first commit at which
//!   all three of their assertion targets exist).
//!
//! - §9.2.2 #5 `stress_m_y_up_orientation_symmetric` exercises Gap L's
//!   `PrinterConfig::with_build_up_direction` (commit #8 / §5.6). It
//!   lands with commit #8.

use mesh_printability::{PrintabilityError, PrinterConfig, validate_for_printing};
use mesh_types::{IndexedMesh, Point3};

// ===== §9.2.1 Existing-detector stress fixtures ==========================

#[test]
fn stress_existing_empty_mesh_error_preserved() {
    // Empty `IndexedMesh`: `validate_for_printing` must short-circuit
    // before any detector runs.
    let mesh = IndexedMesh::new();
    let config = PrinterConfig::fdm_default();

    let result = validate_for_printing(&mesh, &config);

    assert!(
        matches!(result, Err(PrintabilityError::EmptyMesh)),
        "empty mesh must return Err(EmptyMesh); detectors never invoked"
    );
}

// ===== §9.2.2 Gap M stress fixtures ======================================

/// Build a small "ground anchor" triangle at z=0 (top-facing), used to keep
/// `mesh_min_along_up = 0` so the build-plate filter only applies to faces
/// genuinely at the bottom of the mesh.
const fn ground_anchor_triangle(start_index: u32) -> ([Point3<f64>; 3], [u32; 3]) {
    (
        [
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(-2.0, -1.0, 0.0),
            Point3::new(-1.5, -2.0, 0.0),
        ],
        [start_index, start_index + 1, start_index + 2],
    )
}

/// Build a 10 mm cube whose vertices start at index `start_index`. Faces
/// are wound CCW-from-outside (matches `create_watertight_cube` in
/// `validation.rs::tests`). `z_min` controls the cube's elevation.
fn cube_vertices_and_faces(z_min: f64, start_index: u32) -> (Vec<Point3<f64>>, Vec<[u32; 3]>) {
    let z_max = z_min + 10.0;
    let vertices = vec![
        Point3::new(0.0, 0.0, z_min),
        Point3::new(10.0, 0.0, z_min),
        Point3::new(10.0, 10.0, z_min),
        Point3::new(0.0, 10.0, z_min),
        Point3::new(0.0, 0.0, z_max),
        Point3::new(10.0, 0.0, z_max),
        Point3::new(10.0, 10.0, z_max),
        Point3::new(0.0, 10.0, z_max),
    ];
    let cf = |a: u32, b: u32, c: u32| [start_index + a, start_index + b, start_index + c];
    let faces = vec![
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
    ];
    (vertices, faces)
}

#[test]
fn stress_m_pure_roof_flagged() {
    // A pure-roof face (normal = (0, 0, -1), overhang_angle = 90°) above
    // a ground anchor at z=0. Validates that the corrected predicate
    // flags roofs (the v0.7 predicate did not). Mitigates the §8.4
    // High-tier "Gap M v0.7 anchor cascade" risk.
    let mesh = IndexedMesh::from_parts(
        vec![
            // Ground anchor (top-facing at z=0):
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            // Roof at z=10:
            Point3::new(0.0, 0.0, 10.0),
            Point3::new(0.0, 1.0, 10.0),
            Point3::new(1.0, 0.0, 10.0),
        ],
        vec![[0, 1, 2], [3, 4, 5]],
    );
    let config = PrinterConfig::fdm_default();

    // Hand-built fixture: an `expect` failure here would indicate a
    // detector regression, not a malformed fixture.
    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the roof + anchor fixture");

    assert_eq!(
        validation.overhangs.len(),
        1,
        "Gap M: a 90° roof not on the build plate must flag"
    );
}

#[test]
fn stress_m_solid_on_plate_bottom_filtered() {
    // 10 mm solid cube on the build plate (z ∈ [0, 10]). The cube's
    // bottom faces have overhang_angle = 90° but face_min = mesh_min = 0,
    // so the build-plate filter (M.2) rejects them. Validates that the
    // predicate fix does not regress solid-on-plate fixtures.
    let (vertices, faces) = cube_vertices_and_faces(0.0, 0);
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    // Hand-built fixture: an `expect` failure here would indicate a
    // detector regression, not a malformed fixture.
    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the cube-on-plate fixture");

    assert_eq!(
        validation.overhangs.len(),
        0,
        "Gap M.2: cube-on-plate bottom must be filtered by the build-plate check"
    );
}

#[test]
fn stress_m_floating_box_bottom_flagged() {
    // 10 mm cube lifted to z ∈ [10, 20] WITH a small ground anchor at
    // z=0. mesh_min = 0, cube's bottom face_min = 10. (10 - 0) > EPS →
    // build-plate filter does not apply → cube's bottom flags.
    //
    // Spec note: the §9.2.2 stress-fixture description reads "lifted
    // cube with no support" but the build-plate filter is mesh-min-
    // relative (per §5.9), so a bare lifted cube would self-anchor at
    // mesh_min = 10 and the bottom would still be filtered. The anchor
    // here makes the fixture's expected outcome (`overhangs.len() ≥ 1`)
    // consistent with §5.9's mesh-min-relative semantics — equivalent to
    // unit test `test_overhang_suspended_roof_flagged`'s anchored
    // variant. Mitigates the §8.4 anchor-cascade risk by proving the
    // predicate flags lifted bottom faces when geometry below them
    // anchors the mesh-min.
    let (anchor_verts, anchor_face) = ground_anchor_triangle(0);
    let mut vertices: Vec<Point3<f64>> = anchor_verts.to_vec();
    let mut faces: Vec<[u32; 3]> = vec![anchor_face];

    let cube_start: u32 = 3;
    let (cube_verts, cube_faces) = cube_vertices_and_faces(10.0, cube_start);
    vertices.extend(cube_verts);
    faces.extend(cube_faces);

    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    // Hand-built fixture: an `expect` failure here would indicate a
    // detector regression, not a malformed fixture.
    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the lifted-cube + anchor fixture");

    assert!(
        !validation.overhangs.is_empty(),
        "Gap M: lifted cube's bottom must flag when an anchor sets mesh_min below cube_min"
    );
}

#[test]
fn stress_m_layered_bottoms_filter_correctly() {
    // Two stacked 10 mm cubes: base at z ∈ [0, 10], tower at z ∈ [10, 20].
    // The two are separate watertight pieces sharing the z=10 interface
    // by adjacency only (no shared vertices). mesh_min = 0 (base bottom).
    // The base's bottom (z=0) is filtered (face_min = 0 = mesh_min).
    // The tower's bottom (z=10) is NOT filtered ((10 - 0) > EPS) and
    // flags. Validates that the filter is mesh-min-relative, not
    // geometric-z-relative.
    let (base_verts, base_faces) = cube_vertices_and_faces(0.0, 0);
    let (tower_verts, tower_faces) = cube_vertices_and_faces(10.0, 8);

    let mut vertices = base_verts;
    vertices.extend(tower_verts);

    let mut faces = base_faces;
    faces.extend(tower_faces);

    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    // Hand-built fixture: an `expect` failure here would indicate a
    // detector regression, not a malformed fixture.
    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the layered-cubes fixture");

    assert!(
        !validation.overhangs.is_empty(),
        "Gap M: tower bottom at z=10 must flag when base anchors mesh_min at 0"
    );
}
