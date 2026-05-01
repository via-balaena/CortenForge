//! Stress-test gauntlet for mesh-printability per spec §9.
//!
//! Adversarial inputs that exercise the public `validate_for_printing` API
//! at the edges of detector preconditions, FP stability, and topology
//! corners. Each commit in the v0.8 fix arc appends the §9.<gap> stress
//! fixtures for the gap that commit lands; commits do not modify
//! previously-authored fixtures (per spec §9.5).
//!
//! Coverage to date:
//!
//! - §9.2.1 #1: existing-detector — empty-mesh error preserved (commit #2).
//! - §9.2.1 #2–#3: existing-detector — single triangle + vertex-only-shared
//!   (commit #14, deferred from #2 until `TrappedVolume` exists).
//! - §9.2.2 #1–#4: Gap M predicate + build-plate filter (commit #2).
//! - §9.2.3 #1–#7: Gap C `ThinWall` adversarial inputs (commit #10).
//! - §9.2.4 #1–#5: Gap G `LongBridge` adversarial inputs (commit #12).
//! - §9.2.5 #1, #2, #4–#8: Gap H `TrappedVolume` adversarial inputs
//!   (commit #14). Fixture #3 `stress_h_subvoxel_opening_not_flagged`
//!   is **explicitly deferred** — see "Deferred fixtures" below.
//! - §9.2.6 #1–#5: Gap I `SelfIntersecting` adversarial inputs (commit #16).
//! - §9.2.7 #1–#8: Gap J `SmallFeature` adversarial inputs (commit #18).
//!
//! Deferred fixtures and where they land:
//!
//! - §9.2.5 #3 `stress_h_subvoxel_opening_not_flagged` documents the v0.8
//!   intentional behavior that flood-fill leaks through cavity-to-exterior
//!   channels narrower than `voxel_size`. The faithful fixture requires a
//!   watertight cube + watertight inner cavity + sub-voxel tube fusing
//!   them — ≈ 50 LOC of vertex-and-face-with-hole-triangulation authoring
//!   per §9.5 hand-authoring note. The detector's correct handling of
//!   sub-voxel features is already documented at §6.3 line 1118 + tested
//!   indirectly via `test_trapped_volume_info_below_min_feature` in
//!   `validation.rs::tests` (which exercises the resolution-threshold
//!   path). Faithful sub-voxel-opening fixture deferred to a v0.8.x
//!   follow-up commit or v0.9 once drainage-path simulation makes the
//!   behavior actionable rather than purely diagnostic.
//!
//! - §9.2.2 #5 `stress_m_y_up_orientation_symmetric` exercises Gap L's
//!   `with_build_up_direction`. It was not authored in commit #8 (Gap
//!   L); the Gap L `config.rs::tests` `+Y up` cases cover the same
//!   property at unit-test scope. A standalone integration variant can
//!   land in a v0.9 cleanup pass if needed.

use mesh_printability::{
    IssueSeverity, PrintIssueType, PrintTechnology, PrintabilityError, PrinterConfig,
    validate_for_printing,
};
use mesh_types::{IndexedMesh, Point3, Vector3};

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

// ===== §9.2.3 Gap C (ThinWall) ===========================================

/// Build a closed 10×10×`thickness` cuboid at z ∈ [`z_min`, `z_min` +
/// thickness], wound CCW-from-outside, with vertex indices starting at
/// `start_index`. Mirrors the `cube_vertices_and_faces` shape but
/// parametrizes the z extent so a slab can be substantially thinner than
/// 10 mm. The cuboid is watertight + consistently wound; the `ThinWall`
/// detector preconditions hold.
fn thin_slab_vertices_and_faces(
    thickness: f64,
    z_min: f64,
    x_offset: f64,
    start_index: u32,
) -> (Vec<Point3<f64>>, Vec<[u32; 3]>) {
    let z_max = z_min + thickness;
    let vertices = vec![
        Point3::new(x_offset, 0.0, z_min),
        Point3::new(x_offset + 10.0, 0.0, z_min),
        Point3::new(x_offset + 10.0, 10.0, z_min),
        Point3::new(x_offset, 10.0, z_min),
        Point3::new(x_offset, 0.0, z_max),
        Point3::new(x_offset + 10.0, 0.0, z_max),
        Point3::new(x_offset + 10.0, 10.0, z_max),
        Point3::new(x_offset, 10.0, z_max),
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
fn stress_c_clean_thick_box_no_flag() {
    // Solid 10 mm cube under FDM `min_wall_thickness = 1.0`: every
    // face's inward ray hits the opposite face at 10 mm; 10 ≫ 1.0 →
    // no `ThinWall` regions populated. Verifies no false flag on
    // effectively-solid geometry.
    let (vertices, faces) = cube_vertices_and_faces(0.0, 0);
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the thick-box fixture");

    assert_eq!(
        validation.thin_walls.len(),
        0,
        "Gap C: 10 mm walls under min_wall=1.0 must not flag"
    );
}

#[test]
fn stress_c_open_mesh_skipped() {
    // 5-of-6-face open box: watertight precondition fails. ThinWall
    // emits `DetectorSkipped` Info issue; `thin_walls` stays empty.
    // The `as_str()` description's verbatim text per §6.1 line 905 is
    // what callers grep for to determine which detector skipped.
    let (vertices, mut faces) = cube_vertices_and_faces(0.0, 0);
    // Drop the top face (face indices 2 + 3 in the cube layout).
    faces.remove(3);
    faces.remove(2);
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the open-box fixture");

    let any_skipped = validation.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::DetectorSkipped
            && i.description.contains("ThinWall")
            && i.description.contains("watertight")
    });
    assert!(
        any_skipped,
        "Gap C: open mesh must emit DetectorSkipped naming ThinWall + watertight"
    );
    assert_eq!(validation.thin_walls.len(), 0);
}

#[test]
fn stress_c_inconsistent_winding_skipped() {
    // Watertight cube with one face's vertex order flipped: directed
    // edges collide → consistent-winding precondition fails.
    // ThinWall emits `DetectorSkipped`; the Gap F manifold detector
    // *also* fires `NonManifold` Critical with description containing
    // "winding inconsistency".
    let (vertices, mut faces) = cube_vertices_and_faces(0.0, 0);
    // Flip face index 0 (one of the bottom tris).
    let f = faces[0];
    faces[0] = [f[0], f[2], f[1]];
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the flipped-winding fixture");

    let any_skipped = validation.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::DetectorSkipped && i.description.contains("ThinWall")
    });
    let any_winding = validation.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::NonManifold
            && i.severity == IssueSeverity::Critical
            && i.description.contains("winding inconsistency")
    });
    assert!(
        any_skipped,
        "Gap C: inconsistent winding must emit ThinWall DetectorSkipped"
    );
    assert!(
        any_winding,
        "Gap F: inconsistent winding fixture must independently flag NonManifold Critical"
    );
    assert_eq!(validation.thin_walls.len(), 0);
}

#[test]
fn stress_c_concave_z_shape() {
    // Pedagogical "thin section between thick sections" stress fixture.
    // Spec §9.2.3 calls it "Z-shape"; this implementation uses a closed
    // 10×10×0.5 mm slab whose top + bottom faces oppose each other at
    // 0.5 mm. Closed-shell topology splits the flagged faces into 2
    // disjoint clusters (top + bottom share no edge under the
    // build_edge_to_faces convention) — same pattern documented in the
    // §7.1 hand-trace. Both clusters report `thickness ≈ 0.5 mm`.
    //
    // True Z-prism with thick caps (boundary-mesh assembly of three
    // glued blocks) is not authored here because the closed-shell
    // boundary's "thin section between thick sections" coverage is
    // identical at the algorithm level — what's load-bearing for §6.1
    // is detection of the 0.5 mm thin layer, not the geometric Z
    // letter-shape.
    let (vertices, faces) = thin_slab_vertices_and_faces(0.5, 0.0, 0.0, 0);
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the thin-section fixture");

    assert!(
        !validation.thin_walls.is_empty(),
        "Gap C: 0.5 mm thin section under min_wall=1.0 must flag"
    );
    assert!(
        validation
            .thin_walls
            .iter()
            .any(|r| (r.thickness - 0.5).abs() < 1e-5),
        "Gap C: at least one cluster must report thickness ≈ 0.5 mm"
    );
}

#[test]
fn stress_c_two_disjoint_thin_clusters() {
    // Two disjoint thin slabs (each 10×10×0.4 mm), positioned 30 mm
    // apart along +X. Each slab independently produces 2 clusters
    // (top + bottom topologically disjoint). Total ≥ 4 clusters across
    // 2 components. Mitigates §8.4 Gap C cluster-split topology
    // High-tier risk via explicit multi-cluster fixture.
    let (mut verts_a, mut faces_a) = thin_slab_vertices_and_faces(0.4, 0.0, 0.0, 0);
    let base_b: u32 = u32::try_from(verts_a.len()).unwrap_or(8);
    let (verts_b, faces_b) = thin_slab_vertices_and_faces(0.4, 0.0, 30.0, base_b);
    verts_a.extend(verts_b);
    faces_a.extend(faces_b);
    let mesh = IndexedMesh::from_parts(verts_a, faces_a);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the two-slab fixture");

    assert_eq!(
        validation.thin_walls.len(),
        4,
        "Gap C: 2 disjoint slabs × 2 clusters each → 4 total clusters"
    );
    let critical_count = validation
        .issues
        .iter()
        .filter(|i| {
            i.issue_type == PrintIssueType::ThinWall && i.severity == IssueSeverity::Critical
        })
        .count();
    assert_eq!(
        critical_count, 4,
        "Gap C: all 4 clusters at 0.4 mm Critical (0.4 < 1.0/2)"
    );
}

/// Build a watertight UV-sphere shell pair: outer at `outer_radius` mm
/// and inner at `outer_radius - wall_thickness` mm. `n_segs` longitude
/// segments × `n_stacks` latitude stacks per shell. Outer winding is
/// CCW-from-outside (outward-pointing normals); inner winding is
/// REVERSED (normals point into the cavity, away from the solid wall —
/// the watertight + consistent-winding precondition).
///
/// Vertex layout per shell: 1 north pole + (`n_stacks` − 1) ×
/// `n_segs` ring vertices + 1 south pole. Faces: 2 polar fans
/// (`n_segs` tris each) + (`n_stacks` − 2) × `n_segs` × 2 mid-stack
/// quads. Total tris per shell = 2·`n_segs` + 2·`n_segs`·(`n_stacks` − 2).
fn build_uv_sphere_shell_pair(
    outer_radius: f64,
    wall_thickness: f64,
    n_segs: u32,
    n_stacks: u32,
) -> IndexedMesh {
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();

    let inner_radius = outer_radius - wall_thickness;
    let emit_shell =
        |radius: f64, reverse: bool, vertices: &mut Vec<Point3<f64>>, faces: &mut Vec<[u32; 3]>| {
            let base: u32 = u32::try_from(vertices.len()).unwrap_or(0);
            // North pole, mid-stack vertices, south pole.
            vertices.push(Point3::new(0.0, 0.0, radius));
            for stack in 1..n_stacks {
                let theta = f64::from(stack) * std::f64::consts::PI / f64::from(n_stacks);
                let z = radius * theta.cos();
                let r = radius * theta.sin();
                for seg in 0..n_segs {
                    let phi = f64::from(seg) * 2.0 * std::f64::consts::PI / f64::from(n_segs);
                    vertices.push(Point3::new(r * phi.cos(), r * phi.sin(), z));
                }
            }
            vertices.push(Point3::new(0.0, 0.0, -radius));

            let north = base;
            let south = base + 1 + (n_stacks - 1) * n_segs;
            let row_start = |stack: u32| base + 1 + (stack - 1) * n_segs;

            let push = |faces: &mut Vec<[u32; 3]>, a: u32, b: u32, c: u32| {
                if reverse {
                    faces.push([a, c, b]);
                } else {
                    faces.push([a, b, c]);
                }
            };

            // Top fan.
            let row1 = row_start(1);
            for seg in 0..n_segs {
                let v1 = row1 + seg;
                let v2 = row1 + (seg + 1) % n_segs;
                push(faces, north, v1, v2);
            }
            // Mid-stack quads.
            for stack in 1..(n_stacks - 1) {
                let row_a = row_start(stack);
                let row_b = row_start(stack + 1);
                for seg in 0..n_segs {
                    let a0 = row_a + seg;
                    let a1 = row_a + (seg + 1) % n_segs;
                    let b0 = row_b + seg;
                    let b1 = row_b + (seg + 1) % n_segs;
                    push(faces, a0, b0, b1);
                    push(faces, a0, b1, a1);
                }
            }
            // Bottom fan (vertex order REVERSED relative to top so normals
            // still point outward — the bottom pole is on the opposite side
            // of the patch's CCW rotation).
            let last_row = row_start(n_stacks - 1);
            for seg in 0..n_segs {
                let v1 = last_row + seg;
                let v2 = last_row + (seg + 1) % n_segs;
                push(faces, south, v2, v1);
            }
        };

    emit_shell(outer_radius, false, &mut vertices, &mut faces);
    emit_shell(inner_radius, true, &mut vertices, &mut faces);

    IndexedMesh::from_parts(vertices, faces)
}

#[test]
fn stress_c_pole_tied_vertex_sphere() {
    // Hollow tessellated UV-sphere: 16 segments × 8 stacks per shell;
    // outer radius 5 mm, wall thickness 0.4 mm (inner radius 4.6 mm).
    // Each shell's poles tie 16 faces to one vertex. Verifies that
    // Möller-Trumbore at the pole does not degenerate (no NaN, no
    // panic): the algorithm runs to completion and produces clusters.
    //
    // **Pole-face flagging is *expected* under FDM defaults**: a pole
    // face's inward ray hits the inner shell's near pole at the wall-
    // thickness distance (≈ 0.385 mm at this tessellation), not the
    // diametrically opposite far pole at 2·R ≈ 10 mm — Möller-Trumbore
    // returns the first hit. Spec §9.2.3 line 2366's claim that "pole
    // faces have ray-cast distance ≈ 2 × outer_radius" was an analysis
    // error in the spec; the geometric truth is that the pole-face
    // inward ray traverses the wall just like any equatorial ray.
    //
    // Output structure: 1 cluster per shell × 2 shells = 2 clusters,
    // each spanning all 224 of that shell's tris (per the
    // 2·N + 2·N·(M−2) tessellation formula at N=16, M=8).
    //
    // Reported thickness range: [0.38, 0.40] mm — slightly below the
    // 0.4 mm radial wall by chord-vs-arc tessellation error
    // (cos(π/16) ≈ 0.981 chord shrinkage; outer + inner shells both
    // shrink, net effect concentrates reported thickness around 0.385
    // mm). v0.9 followup: re-tessellate denser to tighten the band, or
    // amend the spec to acknowledge tessellation chord error.
    let mesh = build_uv_sphere_shell_pair(5.0, 0.4, 16, 8);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the UV-sphere shell pair");

    // Skip-trigger sanity: precondition must hold (the helper produces
    // watertight + consistently wound output by construction).
    let any_skipped = validation
        .issues
        .iter()
        .any(|i| i.issue_type == PrintIssueType::DetectorSkipped);
    assert!(
        !any_skipped,
        "UV sphere helper produces watertight + consistent winding; ThinWall must run"
    );
    assert_eq!(
        validation.thin_walls.len(),
        2,
        "Gap C: 1 outer shell + 1 inner shell cluster (each shell is one connected component)"
    );
    for region in &validation.thin_walls {
        assert!(
            region.thickness > 0.38 && region.thickness < 0.40,
            "Gap C: shell cluster thickness must be within (0.38, 0.40) mm — chord-shrunk \
             from radial 0.4 mm by ~3.7% per cos(π/16) tessellation; got {}",
            region.thickness
        );
        assert!(
            region.thickness.is_finite(),
            "Gap C: pole-face Möller-Trumbore must not produce NaN at fan apex"
        );
    }
}

/// Build a tessellated thin-walled shell at ~5000 triangles by stretching
/// `build_uv_sphere_shell_pair` to 50 segs × 25 stacks: per-shell tris =
/// 2·50 + 2·50·(25 − 2) = 100 + 2300 = 2400; pair = 4800 tris (≈ 5k).
fn build_5k_tri_thin_shell() -> IndexedMesh {
    build_uv_sphere_shell_pair(5.0, 0.4, 50, 25)
}

#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "release-only perf budget; debug runtime would over-budget the 2 s ceiling"
)]
fn stress_c_5k_tri_perf_budget() {
    // Tessellated thin-walled UV-sphere shell pair (~4800 tris).
    // Asserts O(n²) detector completes under 2 s release-mode + flags
    // at least one cluster (sanity check that the algorithm runs to
    // completion at scale, not just early-exits).
    //
    // The 2 s ceiling is the CI-margin variant of §6.1's local 1 s
    // target (line 952): CI runners are typically 2× slower than
    // local on cold cache, so the assertion adds headroom that absorbs
    // CI-vs-local variance without masking real regressions. A 4× or
    // worse blow-up would still trip; v0.9's BVH followup (§6.1 line
    // 956) tightens this back to ~100 ms.
    //
    // §9.2.3 + §10.4.2 wire this fixture through `tests-release` CI;
    // perf regressions on >10k tris would land separately under the
    // v0.9 BVH followup.
    let mesh = build_5k_tri_thin_shell();
    let config = PrinterConfig::fdm_default();
    let start = std::time::Instant::now();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the 5k-tri shell fixture");

    let elapsed = start.elapsed();
    assert!(
        elapsed < std::time::Duration::from_secs(2),
        "Gap C: 5k-tri ThinWall release runtime {} ms exceeds 2 s budget (CI margin over §6.1's 1 s local target)",
        elapsed.as_millis()
    );
    assert!(
        !validation.thin_walls.is_empty(),
        "Gap C: 5k-tri thin shell must flag at least one cluster"
    );
}

// ===== §9.2.4 Gap G (LongBridge) =========================================

/// Append a closed cuboid (CCW-from-outside) to `(vertices, faces)`.
/// `min`/`max` are diagonally-opposite corners. Mirrors
/// `cube_vertices_and_faces`'s winding but parametric on bbox extents.
/// Used to compose the §9.2.4 closed-mesh bridge fixtures (anchor cuboid
/// + slab cuboid disjoint by vertex set).
fn append_closed_cuboid(
    vertices: &mut Vec<Point3<f64>>,
    faces: &mut Vec<[u32; 3]>,
    min: Point3<f64>,
    max: Point3<f64>,
) {
    #[allow(clippy::cast_possible_truncation)]
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

#[test]
fn stress_g_clean_solid_cube_no_flag() {
    // 10 mm solid cube on the build plate. The cube's bottom face sits
    // at `mesh_min`, filtered by the build-plate guard; the top + sides
    // are not "near-horizontal downward". Expected: 0 LongBridge regions.
    let (vertices, faces) = cube_vertices_and_faces(0.0, 0);
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the solid-cube fixture");

    assert_eq!(
        validation.long_bridges.len(),
        0,
        "Gap G: solid cube on build plate has no horizontal-down faces above plate"
    );
}

#[test]
fn stress_g_sls_silent_skip() {
    // SLS config + a 20×5×1.5 mm bridge slab elevated at z=10 (anchor
    // at z=0 sets `mesh_min = 0`). Powder-bed processes don't need
    // bridge flagging; per §6.2 line 996 the skip is silent — no
    // `DetectorSkipped` issue announces it (distinct from `ThinWall`,
    // which emits `DetectorSkipped` on non-watertight).
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
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::sls_default();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the SLS bridge fixture");

    assert_eq!(
        validation.long_bridges.len(),
        0,
        "Gap G: SLS skips bridge detection silently"
    );
    let any_bridge_skipped = validation.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::DetectorSkipped
            && i.description.to_lowercase().contains("bridge")
    });
    assert!(
        !any_bridge_skipped,
        "Gap G: SLS skip is silent — no DetectorSkipped issue naming bridges"
    );
}

#[test]
fn stress_g_diagonal_bridge_underflagged() {
    // 14×14 horizontal patch at z=10, `max_bridge_span = 15`. The
    // axis-aligned bbox extent is 14 < 15 → no flag, even though the
    // true diagonal (14·√2 ≈ 19.8 mm) exceeds the limit. Locks v0.8
    // bbox-conservative behavior; v0.9 OBB followup catches diagonals.
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
        Point3::new(14.0, 14.0, 11.5),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let mut config = PrinterConfig::fdm_default();
    config.max_bridge_span = 15.0;

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the diagonal-patch fixture");

    assert_eq!(
        validation.long_bridges.len(),
        0,
        "Gap G: diagonal 14×14 patch underflags at v0.8 (bbox = 14 < max = 15); v0.9 OBB catches"
    );
}

#[test]
fn stress_g_cantilever_currently_flagged() {
    // 20-mm cantilever (one-end-anchored horizontal face) at z=10 with
    // anchor cube at z=0. v0.8 cannot distinguish a cantilever from a
    // bridge — both produce a single cluster of horizontal-down faces
    // above the build plate. Locks the v0.8 limitation; v0.9 followup
    // adds support-end analysis to demote cantilevers from `LongBridge`
    // to a no-flag.
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
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the cantilever fixture");

    assert_eq!(
        validation.long_bridges.len(),
        1,
        "Gap G: cantilever flags as bridge at v0.8 (locks limitation); v0.9 adds support-end analysis"
    );
    let any_critical = validation.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::LongBridge && i.severity == IssueSeverity::Critical
    });
    assert!(
        any_critical,
        "Gap G: 20 mm > 10*1.5 → Critical bridge severity"
    );
}

#[test]
fn stress_g_diagonal_underflag_with_y_up() {
    // Same 14×14 diagonal patch, rotated to +Y up: anchor at y ∈ [0, 2],
    // patch at y ∈ [10, 11.5]. With `build_up_direction = (0, 1, 0)`,
    // the perpendicular-plane basis is `e1 = (1, 0, 0)`,
    // `e2 = (0, 0, -1)` — projecting cluster vertices onto the (x, -z)
    // plane gives the same axis-aligned extent (14, 14) as the +Z fixture.
    // Span = 14 < 15 → 0 regions, identical outcome under symmetry.
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_closed_cuboid(
        &mut vertices,
        &mut faces,
        Point3::new(-3.0, 0.0, -3.0),
        Point3::new(-1.0, 2.0, -1.0),
    );
    append_closed_cuboid(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 10.0, 0.0),
        Point3::new(14.0, 11.5, 14.0),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let mut config =
        PrinterConfig::fdm_default().with_build_up_direction(Vector3::new(0.0, 1.0, 0.0));
    config.max_bridge_span = 15.0;

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("validation should succeed for the +Y-up diagonal fixture");

    assert_eq!(
        validation.long_bridges.len(),
        0,
        "Gap G: +Y up rotation must produce same 0 regions as +Z (symmetry)"
    );
}

// ===== §9.2.1 deferred (TrappedVolume-aware) ============================

#[test]
fn stress_existing_single_triangle_open_mesh() {
    // Single triangle (3 verts, 1 face): 3 open edges → not watertight.
    // Both `ThinWall` and `TrappedVolume` precondition checks must skip
    // and emit one `DetectorSkipped` `Info` issue each. The mesh also
    // produces a `NotWatertight` `Critical` issue from `check_basic_manifold`.
    let mesh = IndexedMesh::from_parts(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(5.0, 10.0, 0.0),
        ],
        vec![[0, 1, 2]],
    );
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation =
        validate_for_printing(&mesh, &config).expect("single triangle: validation must succeed");

    let any_not_watertight = validation
        .issues
        .iter()
        .any(|i| i.issue_type == PrintIssueType::NotWatertight);
    assert!(
        any_not_watertight,
        "single triangle must flag NotWatertight (3 open edges)"
    );
    let thinwall_skipped = validation.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::DetectorSkipped && i.description.contains("ThinWall")
    });
    let trapped_skipped = validation.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::DetectorSkipped && i.description.contains("TrappedVolume")
    });
    assert!(
        thinwall_skipped,
        "ThinWall must emit DetectorSkipped on non-watertight single triangle"
    );
    assert!(
        trapped_skipped,
        "TrappedVolume must emit DetectorSkipped on non-watertight single triangle"
    );
    assert_eq!(validation.thin_walls.len(), 0);
    assert_eq!(validation.trapped_volumes.len(), 0);
}

#[test]
fn stress_existing_two_faces_vertex_only_shared() {
    // Two disjoint triangles sharing exactly one vertex (no shared edge).
    // Six unique edges (count = 1 each) → 6 open edges → not watertight.
    // `ThinWall` and `TrappedVolume` skip; no winding-inconsistency flag
    // (no shared directed edge to collide).
    let mesh = IndexedMesh::from_parts(
        vec![
            // Triangle A
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(5.0, 10.0, 0.0),
            // Triangle B — shares vertex 0 only, offset along +Z
            Point3::new(0.0, 0.0, 5.0),
            Point3::new(0.0, 10.0, 5.0),
        ],
        vec![[0, 1, 2], [0, 3, 4]],
    );
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("vertex-only-shared mesh: validation must succeed");

    let any_not_watertight = validation
        .issues
        .iter()
        .any(|i| i.issue_type == PrintIssueType::NotWatertight);
    assert!(
        any_not_watertight,
        "vertex-only-shared mesh must flag NotWatertight (6 open edges)"
    );
    let any_winding_collision = validation
        .issues
        .iter()
        .any(|i| i.description.contains("winding inconsistency"));
    assert!(
        !any_winding_collision,
        "no shared edges → no winding-inconsistency flag"
    );
    let thinwall_skipped = validation.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::DetectorSkipped && i.description.contains("ThinWall")
    });
    let trapped_skipped = validation.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::DetectorSkipped && i.description.contains("TrappedVolume")
    });
    assert!(thinwall_skipped, "ThinWall must skip");
    assert!(trapped_skipped, "TrappedVolume must skip");
}

// ===== §9.2.5 Gap H (TrappedVolume) ======================================
//
// Per §9.5 hand-authoring note: helpers live in this file (no shared
// `tests/common/` module). The cube-cavity helpers below mirror the
// `validation.rs::tests::make_cube_with_inner_cavity` pattern; the §6.3
// spec uses "sphere" terminology but the implementations use cube
// cavities for vertex-and-face authoring simplicity. The detector is
// curvature-agnostic (operates on voxelized parity), so a cube cavity
// exercises the same algorithmic path.

/// Coarse-voxel `PrinterConfig` for stress fixtures: voxel = 0.4 mm
/// regardless of technology (`min_feature_size = 1.6`,
/// `layer_height = 0.8`). Matches §6.3 line 1140's "100 mm part,
/// `voxel_size` = 0.4 mm" perf-doc example. Keeps integration-test
/// runtime well within `tests-debug` per-crate budget while still
/// driving every algorithmic path.
const fn stress_h_coarse_config(tech: PrintTechnology) -> PrinterConfig {
    let mut c = match tech {
        PrintTechnology::Sla => PrinterConfig::sla_default(),
        PrintTechnology::Sls => PrinterConfig::sls_default(),
        PrintTechnology::Mjf => PrinterConfig::mjf_default(),
        PrintTechnology::Fdm | PrintTechnology::Other => PrinterConfig::fdm_default(),
    };
    c.technology = tech;
    c.min_feature_size = 1.6;
    c.layer_height = 0.8;
    c
}

/// Append a watertight cube to `vertices` + `faces` whose corners span
/// `min` → `max`. Faces are wound CCW-from-outside (normals pointing
/// out of the cube). 8 vertices, 12 triangles.
fn append_outer_cube(
    vertices: &mut Vec<Point3<f64>>,
    faces: &mut Vec<[u32; 3]>,
    min: Point3<f64>,
    max: Point3<f64>,
) {
    #[allow(clippy::cast_possible_truncation)]
    let base = vertices.len() as u32;
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

/// Append a watertight inner-cavity cube to `vertices` + `faces` whose
/// corners span `min` → `max`. Faces are wound CCW-from-INSIDE the
/// cavity (normals point INTO the cavity). 8 vertices, 12 triangles.
fn append_inner_cavity(
    vertices: &mut Vec<Point3<f64>>,
    faces: &mut Vec<[u32; 3]>,
    min: Point3<f64>,
    max: Point3<f64>,
) {
    #[allow(clippy::cast_possible_truncation)]
    let base = vertices.len() as u32;
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
        cf(0, 1, 2),
        cf(0, 2, 3),
        cf(4, 6, 5),
        cf(4, 7, 6),
        cf(0, 5, 1),
        cf(0, 4, 5),
        cf(3, 2, 6),
        cf(3, 6, 7),
        cf(0, 7, 4),
        cf(0, 3, 7),
        cf(1, 6, 2),
        cf(1, 5, 6),
    ]);
}

#[test]
fn stress_h_solid_cube_no_cavity() {
    // 20 mm solid cube; 0 trapped regions (no cavity).
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_outer_cube(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(20.0, 20.0, 20.0),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = stress_h_coarse_config(PrintTechnology::Fdm);

    #[allow(clippy::expect_used)]
    let validation =
        validate_for_printing(&mesh, &config).expect("solid cube: validation must succeed");

    assert_eq!(
        validation.trapped_volumes.len(),
        0,
        "solid cube must produce no trapped regions"
    );
    let any_trapped = validation
        .issues
        .iter()
        .any(|i| i.issue_type == PrintIssueType::TrappedVolume);
    assert!(!any_trapped, "solid cube: no TrappedVolume issues");
}

#[test]
fn stress_h_open_mesh_skipped() {
    // 20 mm cube with one face removed → 4 open edges → not watertight
    // → DetectorSkipped Info; trapped_volumes stays empty.
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_outer_cube(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(20.0, 20.0, 20.0),
    );
    // Drop the +Z face's two triangles (last 2 of the 12-tri outer cube
    // would be `[1, 2, 6], [1, 6, 5]` — the +X face. Remove the +Z
    // face's triangles at indices 2, 3 (`[4, 5, 6], [4, 6, 7]`).
    let faces: Vec<[u32; 3]> = faces
        .into_iter()
        .enumerate()
        .filter(|(idx, _)| *idx != 2 && *idx != 3)
        .map(|(_, f)| f)
        .collect();
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = stress_h_coarse_config(PrintTechnology::Fdm);

    #[allow(clippy::expect_used)]
    let validation =
        validate_for_printing(&mesh, &config).expect("open mesh: validation must succeed");

    let any_skipped = validation.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::DetectorSkipped
            && i.description.contains("TrappedVolume")
            && i.description.contains("watertight")
    });
    assert!(
        any_skipped,
        "open mesh must emit TrappedVolume DetectorSkipped"
    );
    assert_eq!(validation.trapped_volumes.len(), 0);
}

#[test]
fn stress_h_sphere_inside_cube_volume_within_10pct() {
    // Spec name retains "sphere" per §9.2.5 line 2383; implementation uses
    // a cube cavity sized so its analytical volume equals (4/3)π·5³ ≈
    // 523.6 mm³ (cube edge ≈ 8.0588 mm). The 10% tolerance band absorbs
    // voxel discretization noise + cross-platform ULP variance per §9.6
    // (mitigates §8.4 Gap H FP-drift risk).
    let cube_edge = ((4.0_f64 / 3.0) * std::f64::consts::PI * 5.0_f64.powi(3)).cbrt();
    let outer_size = 20.0;
    let cavity_min_coord = (outer_size - cube_edge) / 2.0;
    let cavity_max_coord = cavity_min_coord + cube_edge;

    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_outer_cube(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(outer_size, outer_size, outer_size),
    );
    append_inner_cavity(
        &mut vertices,
        &mut faces,
        Point3::new(cavity_min_coord, cavity_min_coord, cavity_min_coord),
        Point3::new(cavity_max_coord, cavity_max_coord, cavity_max_coord),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = stress_h_coarse_config(PrintTechnology::Fdm);

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("sphere-cavity fixture: validation must succeed");

    assert_eq!(validation.trapped_volumes.len(), 1);
    let analytical_volume = (4.0_f64 / 3.0) * std::f64::consts::PI * 5.0_f64.powi(3);
    let voxel_volume = validation.trapped_volumes[0].volume;
    approx::assert_relative_eq!(voxel_volume, analytical_volume, max_relative = 0.10);
}

#[test]
fn stress_h_two_disjoint_cavities() {
    // 20 mm outer cube + 2 disjoint 4 mm cube cavities at offset 3 mm
    // from opposite corners — walls between cavities ≥ 6 mm = 15
    // voxels at voxel 0.4 mm. Flood-fill labels them as 2 components.
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_outer_cube(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(20.0, 20.0, 20.0),
    );
    append_inner_cavity(
        &mut vertices,
        &mut faces,
        Point3::new(3.0, 3.0, 3.0),
        Point3::new(7.0, 7.0, 7.0),
    );
    append_inner_cavity(
        &mut vertices,
        &mut faces,
        Point3::new(13.0, 13.0, 13.0),
        Point3::new(17.0, 17.0, 17.0),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = stress_h_coarse_config(PrintTechnology::Fdm);

    #[allow(clippy::expect_used)]
    let validation =
        validate_for_printing(&mesh, &config).expect("two-cavity fixture: validation must succeed");

    assert_eq!(
        validation.trapped_volumes.len(),
        2,
        "two disjoint cavities must produce two regions"
    );
    let count_trapped_issues = validation
        .issues
        .iter()
        .filter(|i| i.issue_type == PrintIssueType::TrappedVolume)
        .count();
    assert_eq!(
        count_trapped_issues, 2,
        "one TrappedVolume issue per region"
    );
}

#[test]
fn stress_h_disconnected_dual_cavity() {
    // Two separate 12 mm cubes-with-cavity offset by 30 mm along +X.
    // Each cube has its own watertight closed cavity (4 mm). The
    // exterior flood-fill from the grid corner reaches both cubes'
    // exteriors via the surrounding empty space, so both cavities are
    // correctly labeled trapped (§9.1 row 13). Verifies the grid-corner
    // seed is not blind to "second" cubes.
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    // Cube A at origin
    append_outer_cube(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(12.0, 12.0, 12.0),
    );
    append_inner_cavity(
        &mut vertices,
        &mut faces,
        Point3::new(4.0, 4.0, 4.0),
        Point3::new(8.0, 8.0, 8.0),
    );
    // Cube B offset +30 along X
    append_outer_cube(
        &mut vertices,
        &mut faces,
        Point3::new(42.0, 0.0, 0.0),
        Point3::new(54.0, 12.0, 12.0),
    );
    append_inner_cavity(
        &mut vertices,
        &mut faces,
        Point3::new(46.0, 4.0, 4.0),
        Point3::new(50.0, 8.0, 8.0),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = stress_h_coarse_config(PrintTechnology::Fdm);

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("dual-cube-with-cavity fixture: validation must succeed");

    assert_eq!(
        validation.trapped_volumes.len(),
        2,
        "two disconnected cubes-with-cavity must produce two trapped regions"
    );
}

/// **Release-only** perf-cliff fixture per §9.2.5 line 2386.
///
/// 100 × 100 × 30 mm part with a 6 mm cube cavity at the center, voxel
/// 0.4 mm → grid ≈ 250 × 250 × 75 = 4.7 M voxels = 4.7 MB, well under
/// the 64 MB stress-fixture budget. Asserts the detector completes
/// quickly enough not to time out the integration-test budget;
/// debug-mode runtime is too slow per `feedback_release_mode_heavy_tests`,
/// so the fixture is `#[cfg_attr(debug_assertions, ignore)]`.
#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "release-only perf budget; debug runtime would over-budget the 30 s ceiling"
)]
fn stress_h_voxel_grid_perf_cliff() {
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_outer_cube(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(100.0, 100.0, 30.0),
    );
    // 6 mm cube cavity at center; centroid at (50, 50, 15).
    append_inner_cavity(
        &mut vertices,
        &mut faces,
        Point3::new(47.0, 47.0, 12.0),
        Point3::new(53.0, 53.0, 18.0),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = stress_h_coarse_config(PrintTechnology::Fdm);

    let start = std::time::Instant::now();
    #[allow(clippy::expect_used)]
    let validation =
        validate_for_printing(&mesh, &config).expect("perf-cliff fixture: validation must succeed");
    let elapsed = start.elapsed();

    assert_eq!(
        validation.trapped_volumes.len(),
        1,
        "single cavity → 1 region"
    );
    assert!(
        elapsed.as_secs_f64() < 30.0,
        "stress_h_voxel_grid_perf_cliff exceeded 30 s release-mode runtime budget: {elapsed:?}"
    );
}

#[test]
fn stress_h_voxel_grid_oom_safety() {
    // 200 mm cube at FDM defaults (voxel = 0.1 mm) → 2000³ ≈ 8 × 10⁹
    // voxels = 8 GB, well above the 1 GB cap. The §6.3 step 4.5
    // memory pre-flight must emit `DetectorSkipped` Info BEFORE the
    // grid is allocated; the test runs in <1 ms (no allocation).
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_outer_cube(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(200.0, 200.0, 200.0),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    let start = std::time::Instant::now();
    #[allow(clippy::expect_used)]
    let validation =
        validate_for_printing(&mesh, &config).expect("oom-safety fixture: validation must succeed");
    let elapsed = start.elapsed();

    let any_skipped = validation.issues.iter().any(|i| {
        i.issue_type == PrintIssueType::DetectorSkipped
            && i.description.contains("TrappedVolume")
            && i.description.contains("1 GB")
    });
    assert!(
        any_skipped,
        "200 mm cube at FDM voxel 0.1 mm must trigger §6.3 step 4.5 memory pre-flight skip"
    );
    assert_eq!(validation.trapped_volumes.len(), 0);
    // Memory cap check fires BEFORE grid alloc → fixture runs in <1 ms;
    // 100 ms is a generous safety margin for cold-cache / loaded CI runners.
    assert!(
        elapsed.as_millis() < 100,
        "memory pre-flight must skip before allocation; got {elapsed:?}"
    );
}

// ===== §9.2.6 Gap I — SelfIntersecting stress fixtures ===================

#[test]
fn stress_i_clean_cube_no_flag() {
    // Clean watertight cube: every face pair shares at least one
    // vertex *index*, so mesh-repair's `skip_adjacent = true` filters
    // them out before SAT testing. Disjoint cube faces (across-the-cube
    // triangles) don't share interior points because the cube is
    // convex + watertight. Result: 0 SelfIntersecting regions.
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_outer_cube(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 10.0),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation =
        validate_for_printing(&mesh, &config).expect("clean cube: validation must succeed");

    assert_eq!(
        validation.self_intersecting.len(),
        0,
        "clean watertight cube must produce 0 SelfIntersecting regions"
    );
    let any_si_issue = validation
        .issues
        .iter()
        .any(|i| i.issue_type == PrintIssueType::SelfIntersecting);
    assert!(
        !any_si_issue,
        "no SelfIntersecting issue should be emitted on a clean cube"
    );
}

/// Build a "self-folded sheet" of `n` large triangles arranged as a fan
/// rotated about the Y axis. All triangles span a central region near
/// the Y axis; per-triangle `(dx, _, dx)` translation guarantees every
/// triangle has unique vertex indices (mesh-repair's
/// `skip_adjacent = true` filters by vertex-sharing in
/// `build_face_adjacency`, so vertex-disjoint construction is required
/// to surface intersections). Each pair intersects through the central
/// region. With `n = 21`, candidate pairs `= 21 * 20 / 2 = 210` —
/// comfortably above the `IntersectionParams::default()`
/// `max_reported = 100` cap.
fn make_self_folded_sheet(n: u32) -> IndexedMesh {
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    let pi = std::f64::consts::PI;
    for i in 0..n {
        let theta = (f64::from(i) + 0.5) * pi / f64::from(n);
        let cos_t = theta.cos();
        let sin_t = theta.sin();
        let dx = f64::from(i) * 0.001;
        vertices.push(Point3::new(
            10.0_f64.mul_add(cos_t, dx),
            -10.0,
            10.0_f64.mul_add(sin_t, dx),
        ));
        vertices.push(Point3::new(
            10.0_f64.mul_add(-cos_t, dx),
            -10.0,
            10.0_f64.mul_add(-sin_t, dx),
        ));
        vertices.push(Point3::new(dx, 10.0, dx));
        let base = i * 3;
        faces.push([base, base + 1, base + 2]);
    }
    IndexedMesh::from_parts(vertices, faces)
}

/// Heavy fixture per §9.2.6 line 2406. 21-triangle vertex-disjoint
/// fan → 210 candidate intersecting pairs;
/// `IntersectionParams::default()` caps reported pairs at 100 with
/// `truncated = true`. Heavy enough to warrant
/// `#[cfg_attr(debug_assertions, ignore)]` per
/// `feedback_release_mode_heavy_tests`.
#[test]
#[cfg_attr(
    debug_assertions,
    ignore = "release-only — heavy mesh-repair self-intersection work; debug runtime over-budgets"
)]
fn stress_i_truncation_at_100() {
    let mesh = make_self_folded_sheet(21);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation =
        validate_for_printing(&mesh, &config).expect("21-triangle fan: validation must succeed");

    assert_eq!(
        validation.self_intersecting.len(),
        100,
        "max_reported = 100 must cap the region count at exactly 100"
    );
    let issue = validation
        .issues
        .iter()
        .find(|i| i.issue_type == PrintIssueType::SelfIntersecting);
    #[allow(clippy::expect_used)]
    let issue = issue.expect("SelfIntersecting summary issue must be emitted");
    assert!(
        issue
            .description
            .contains("(search truncated; total may be higher)"),
        "truncated description must contain the expected suffix; got `{}`",
        issue.description
    );
    assert_eq!(issue.severity, IssueSeverity::Critical);
}

#[test]
fn stress_i_canonical_face_a_lt_face_b() {
    // 8-triangle fan still produces multiple vertex-disjoint
    // intersecting pairs (8 * 7 / 2 = 28 candidates) without
    // approaching the truncation cap.
    let mesh = make_self_folded_sheet(8);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation =
        validate_for_printing(&mesh, &config).expect("8-triangle fan: validation must succeed");

    assert!(
        !validation.self_intersecting.is_empty(),
        "8-triangle fan must produce ≥ 1 self-intersecting region"
    );
    for region in &validation.self_intersecting {
        assert!(
            region.face_a < region.face_b,
            "§6.4: face_a ({}) must be < face_b ({})",
            region.face_a,
            region.face_b
        );
    }
}

#[test]
fn stress_i_vertex_only_contact_not_flagged() {
    // Two cubes sharing exactly one vertex *index* at the corner
    // `(10, 10, 10)`. Cube A is built via `append_outer_cube` (vertex
    // 6 = max corner). Cube B is built manually so its "min corner"
    // (local index 0) reuses cube A's index 6 instead of allocating
    // a fresh vertex — total mesh vertex count = 15 (= 8 + 7), and
    // every face-pair (`cube_A_face_touching_idx_6`,
    // `cube_B_face_touching_idx_6`) shares vertex index 6.
    //
    // mesh-repair's `build_face_adjacency` is index-based (see
    // `mesh-repair/src/intersect.rs:260`), so the 36 corner-touching
    // pairs are all skipped before SAT testing. The 6 non-corner
    // cube-A faces vs the 6 non-corner cube-B faces are checked via
    // SAT and correctly return "separated" because each triangle's
    // interior lies in its own cube's solid half-space.
    //
    // **Spec deviation from §9.2.6 line 2408**: the spec called for
    // two cubes with *coordinate*-shared corners (no shared index).
    // mesh-repair's SAT test is loose at coord-only vertex contact
    // (separating-axis tolerance compares `max1 + epsilon < min2`,
    // which fails for touching-but-not-overlapping intervals — see
    // `mesh-repair/src/intersect.rs:379`), producing 36 false-positive
    // intersections. The shared-INDEX construction tests the v0.8
    // mechanism for "vertex-only contact = not flagged" (the
    // adjacency skip path) faithfully; the coord-only SAT looseness
    // is a v0.9 followup against mesh-repair, not a v0.8 detector
    // regression.
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_outer_cube(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 10.0),
    );
    // Cube B's 7 fresh vertices (local indices 1..=7); local index 0
    // remaps to cube A's index 6 (the shared corner).
    vertices.extend_from_slice(&[
        Point3::new(20.0, 10.0, 10.0),
        Point3::new(20.0, 20.0, 10.0),
        Point3::new(10.0, 20.0, 10.0),
        Point3::new(10.0, 10.0, 20.0),
        Point3::new(20.0, 10.0, 20.0),
        Point3::new(20.0, 20.0, 20.0),
        Point3::new(10.0, 20.0, 20.0),
    ]);
    let local_to_global: [u32; 8] = [6, 8, 9, 10, 11, 12, 13, 14];
    let cf =
        |a: usize, b: usize, c: usize| [local_to_global[a], local_to_global[b], local_to_global[c]];
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
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("vertex-index-shared cubes: validation must succeed");

    assert_eq!(
        validation.self_intersecting.len(),
        0,
        "shared-INDEX vertex-only contact must produce 0 SelfIntersecting regions"
    );
}

#[test]
fn stress_i_near_coplanar_intersection() {
    // Two non-vertex-sharing triangles, near-coplanar (~1 mrad off
    // the common XY plane), with their interiors crossing at z ≈ 0.
    // Triangle A is in z = 0; triangle B is tilted so its base sits
    // at z = +1e-3 (above) and its apex at z = -1e-3 (below) — the
    // edges crossing z = 0 lie within A's interior. Critical
    // property: mesh-repair's `is_coplanar` early-return predicate
    // (`cross_normals.norm_squared() < epsilon² * |n1|² * |n2|²`)
    // does NOT fire at 1 mrad off coplanar — `(1e-3)² ≈ 1e-6` is six
    // orders of magnitude above `epsilon² = 1e-20`. The 3D SAT path
    // detects the interpenetration.
    let vertices = vec![
        // Triangle A — z = 0 plane.
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(5.0, 10.0, 0.0),
        // Triangle B — tilted about a horizontal axis;
        // base above z = 0, apex below z = 0.
        Point3::new(1.0, 1.0, 1.0e-3),
        Point3::new(9.0, 1.0, 1.0e-3),
        Point3::new(5.0, 9.0, -1.0e-3),
    ];
    let faces = vec![[0, 1, 2], [3, 4, 5]];
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("near-coplanar fixture: validation must succeed");

    assert!(
        !validation.self_intersecting.is_empty(),
        "near-coplanar interpenetrating triangles must be flagged at epsilon = 1e-10"
    );
}

// ===== §9.2.7 Gap J — SmallFeature stress fixtures =======================

/// Append a watertight cube (12 outward-wound triangles, 8 verts) at
/// `(min, max)`. Mirrors `append_outer_cube` so §9.2.7 can compose
/// multi-component fixtures without depending on §9.2.5's helpers
/// (which are private to this file but live up at line ~996).
fn append_small_feature_cube(
    vertices: &mut Vec<Point3<f64>>,
    faces: &mut Vec<[u32; 3]>,
    min: Point3<f64>,
    max: Point3<f64>,
) {
    #[allow(clippy::cast_possible_truncation)]
    let base = vertices.len() as u32;
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

#[test]
fn stress_j_clean_main_body_not_flagged() {
    // 30 mm cube; max_extent 30 ≫ FDM `min_feature_size` 0.8 → 0
    // SmallFeature regions.
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_small_feature_cube(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(30.0, 30.0, 30.0),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("clean 30 mm cube fixture: validation must succeed");

    assert_eq!(
        validation.small_features.len(),
        0,
        "clean 30 mm cube must not flag SmallFeature"
    );
    assert!(
        !validation
            .issues
            .iter()
            .any(|i| matches!(i.issue_type, PrintIssueType::SmallFeature)),
        "no SmallFeature issue on a clean main body"
    );
}

#[test]
fn stress_j_floating_burr_warning() {
    // 30 mm cube + 0.2 mm hex-prism burr offset by 50 mm along +X.
    // FDM `min_feature_size` = 0.8 → burr's max_extent 0.2 < 0.8 → flagged
    // AND 0.2 < 0.8 / 2 = 0.4 → severity Warning.
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_small_feature_cube(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(30.0, 30.0, 30.0),
    );

    // Hex-prism burr: 14 verts (6 top + 6 bottom + 2 hub centers) +
    // 24 triangles (12 lateral + 6 top fan + 6 bottom fan).
    let burr_radius = 0.1; // → diameter (max_extent in XY) = 0.2 mm.
    let burr_height = 0.1; // → max_extent in Z = 0.1 mm; XY governs.
    let cx = 50.0;
    let cy = 0.0;
    let z0 = 0.0;
    let z1 = burr_height;
    #[allow(clippy::cast_possible_truncation)]
    let base = vertices.len() as u32;
    // Top hub center, bottom hub center, then 6 top + 6 bottom rim verts.
    vertices.push(Point3::new(cx, cy, z1)); // base + 0: top hub
    vertices.push(Point3::new(cx, cy, z0)); // base + 1: bottom hub
    for k in 0..6 {
        let theta = std::f64::consts::TAU * f64::from(k) / 6.0;
        vertices.push(Point3::new(
            cx + burr_radius * theta.cos(),
            cy + burr_radius * theta.sin(),
            z1,
        )); // base + 2..7: top rim
    }
    for k in 0..6 {
        let theta = std::f64::consts::TAU * f64::from(k) / 6.0;
        vertices.push(Point3::new(
            cx + burr_radius * theta.cos(),
            cy + burr_radius * theta.sin(),
            z0,
        )); // base + 8..13: bottom rim
    }
    // Top fan (6 tris, normal +Z).
    for k in 0..6u32 {
        let a = base + 2 + k;
        let b = base + 2 + ((k + 1) % 6);
        faces.push([base, b, a]);
    }
    // Bottom fan (6 tris, normal -Z).
    for k in 0..6u32 {
        let a = base + 8 + k;
        let b = base + 8 + ((k + 1) % 6);
        faces.push([base + 1, a, b]);
    }
    // Lateral (12 tris).
    for k in 0..6u32 {
        let t0 = base + 2 + k;
        let t1 = base + 2 + ((k + 1) % 6);
        let b0 = base + 8 + k;
        let b1 = base + 8 + ((k + 1) % 6);
        faces.push([t0, b0, b1]);
        faces.push([t0, b1, t1]);
    }

    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("cube + 0.2 mm burr fixture: validation must succeed");

    assert_eq!(
        validation.small_features.len(),
        1,
        "exactly one SmallFeature region (the 0.2 mm burr)"
    );
    let region = &validation.small_features[0];
    assert!(
        region.max_extent < 0.8 / 2.0,
        "burr max_extent {} must be below 0.8/2 = 0.4 → Warning band",
        region.max_extent
    );

    // Per-site `expect_used` allow: assertion-target lookup; if the
    // detector's emission contract breaks, panicking with a clear
    // message is the right test failure mode.
    #[allow(clippy::expect_used)]
    let issue = validation
        .issues
        .iter()
        .find(|i| matches!(i.issue_type, PrintIssueType::SmallFeature))
        .expect("burr must emit a SmallFeature issue");
    assert_eq!(
        issue.severity,
        IssueSeverity::Warning,
        "0.2 mm burr < 0.8/2 → Warning"
    );
}

#[test]
fn stress_j_unit_conversion_diagnostic() {
    // 30-µm cube (entire mesh, single component, max_extent 0.030 mm)
    // with FDM `min_feature_size` = 0.8 → 1 SmallFeature region for
    // the entire mesh. Documents the user-diagnostic role per §6.5
    // (mesh authored in metres but used in mm-context).
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_small_feature_cube(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.030, 0.030, 0.030),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let config = PrinterConfig::fdm_default();

    #[allow(clippy::expect_used)]
    let validation =
        validate_for_printing(&mesh, &config).expect("30 µm cube fixture: validation must succeed");

    assert_eq!(
        validation.small_features.len(),
        1,
        "entire mesh = 1 component below resolution → 1 SmallFeature region"
    );
    let region = &validation.small_features[0];
    assert!(
        (region.max_extent - 0.030).abs() < 1e-9,
        "max_extent ≈ 0.030 mm; got {}",
        region.max_extent
    );
    assert_eq!(region.face_count, 12);
}

#[test]
fn stress_j_below_threshold_warning_not_info() {
    // Floating fragment with max_extent = 0.1 mm; min_feature_size 0.4
    // → 0.1 < 0.4 / 2 = 0.2 → Warning.
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.1, 0.0, 0.0),
        Point3::new(0.0, 0.1, 0.0),
    ];
    let faces = vec![[0, 1, 2]];
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 0.4;

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("0.1 mm fragment fixture: validation must succeed");

    assert_eq!(validation.small_features.len(), 1);
    // Per-site `expect_used` allow: assertion-target lookup.
    #[allow(clippy::expect_used)]
    let issue = validation
        .issues
        .iter()
        .find(|i| matches!(i.issue_type, PrintIssueType::SmallFeature))
        .expect("0.1 mm fragment must emit a SmallFeature issue");
    assert_eq!(
        issue.severity,
        IssueSeverity::Warning,
        "0.1 < 0.4 / 2 → Warning"
    );
}

#[test]
fn stress_j_just_below_threshold_info() {
    // Floating fragment with max_extent = 0.3 mm; min_feature_size 0.4
    // → 0.3 < 0.4 (flagged) AND 0.3 ≥ 0.4 / 2 = 0.2 (Info, not Warning).
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.3, 0.0, 0.0),
        Point3::new(0.0, 0.3, 0.0),
    ];
    let faces = vec![[0, 1, 2]];
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 0.4;

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("0.3 mm fragment fixture: validation must succeed");

    assert_eq!(validation.small_features.len(), 1);
    // Per-site `expect_used` allow: assertion-target lookup.
    #[allow(clippy::expect_used)]
    let issue = validation
        .issues
        .iter()
        .find(|i| matches!(i.issue_type, PrintIssueType::SmallFeature))
        .expect("0.3 mm fragment must emit a SmallFeature issue");
    assert_eq!(issue.severity, IssueSeverity::Info, "0.3 ≥ 0.4 / 2 → Info");
}

#[test]
fn stress_j_open_component_no_panic() {
    // Open 5-of-6 face cube at side 1 mm; min_feature_size 2.0 →
    // flagged (max_extent 1.0 < 2.0). Open mesh; signed_volume is
    // non-physical but `abs(...)` guarantees finite, non-negative —
    // mitigates §6.5 doc note that open-component volume is approximate.
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(1.0, 0.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(0.0, 1.0, 1.0),
    ];
    // 5 sides (no top): bottom + front + back + left + right.
    let faces = vec![
        [0, 2, 1],
        [0, 3, 2],
        [0, 1, 5],
        [0, 5, 4],
        [3, 6, 2],
        [3, 7, 6],
        [0, 4, 7],
        [0, 7, 3],
        [1, 2, 6],
        [1, 6, 5],
    ];
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 2.0;

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("open-component fixture: validation must succeed");

    assert_eq!(validation.small_features.len(), 1);
    let region = &validation.small_features[0];
    assert!(
        region.volume.is_finite() && region.volume >= 0.0,
        "open-component volume must be finite and non-negative; got {}",
        region.volume
    );
    assert!(
        !region.volume.is_nan(),
        "open-component volume must not be NaN"
    );
}

#[test]
fn stress_j_face_adjacency_via_edge_only() {
    // Two triangles sharing only vertex index 2 (no shared edge) → 2
    // separate components. Verifies the §6.5 line 1280 edge-adjacency
    // contract: vertex-only sharing is NOT face-adjacency.
    let vertices = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.1, 0.0, 0.0),
        Point3::new(0.05, 0.05, 0.0),
        Point3::new(0.1, 0.05, 0.0),
        Point3::new(0.05, 0.1, 0.0),
    ];
    let faces = vec![[0, 1, 2], [2, 3, 4]];
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 0.4;

    #[allow(clippy::expect_used)]
    let validation = validate_for_printing(&mesh, &config)
        .expect("vertex-only-shared fixture: validation must succeed");

    assert_eq!(
        validation.small_features.len(),
        2,
        "vertex-only sharing is not edge-adjacency; expect 2 separate components"
    );
    for region in &validation.small_features {
        assert_eq!(
            region.face_count, 1,
            "each component contains exactly one face"
        );
    }
}

#[test]
fn stress_j_signed_volume_unit_cube() {
    // Unit-cube fragment (1 mm side, exact-representable verts);
    // min_feature_size 2.0 → flagged. Volume must equal 1.0 mm³ within
    // 1e-6. Mitigates §8.2.1 cross-platform FP-drift via exact-
    // representable inputs (the divergence-theorem sum reduces to
    // products of small integers; rounding cancels at the 1/6 step).
    let mut vertices: Vec<Point3<f64>> = Vec::new();
    let mut faces: Vec<[u32; 3]> = Vec::new();
    append_small_feature_cube(
        &mut vertices,
        &mut faces,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 1.0),
    );
    let mesh = IndexedMesh::from_parts(vertices, faces);
    let mut config = PrinterConfig::fdm_default();
    config.min_feature_size = 2.0;

    #[allow(clippy::expect_used)]
    let validation =
        validate_for_printing(&mesh, &config).expect("unit-cube fixture: validation must succeed");

    assert_eq!(validation.small_features.len(), 1);
    let region = &validation.small_features[0];
    assert!(
        (region.volume - 1.0).abs() < 1e-6,
        "divergence-theorem volume must be ≈ 1.0 mm³ within 1e-6 on \
         exact-representable verts; got {}",
        region.volume
    );
}
