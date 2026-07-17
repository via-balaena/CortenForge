//! End-to-end repair-pipeline composition oracle.
//!
//! The library's unit tests isolate each repair stage (`repair.rs` welding /
//! degenerate / duplicate / unreferenced cleanup, `holes.rs` fill,
//! `winding.rs` fix). This integration test pins the *composition* the
//! stages produce when run in sequence on one mesh carrying all six defect
//! classes at once — the cross-stage interactions no single-stage unit test
//! expresses:
//!
//! - the `unreferenced_removed == 3` **cascade** (a hand-stranded vertex plus
//!   two orphans created *by* the weld and degenerate-removal stages),
//! - the `count_inconsistent_faces` **stage-ordering** oracle (exactly 1 after
//!   `repair_mesh`, still `>= 1` after `fill_holes` — the ear-clipped fill
//!   triangle's winding is nondeterministic — and 0 only after
//!   `fix_winding_order`),
//! - the terminal **watertight ∧ manifold ∧ outward** biconditional as the
//!   pipeline's end state.
//!
//! Ported from the `mesh-repair-walkthrough` example, which is now a runnable
//! demonstration of the same pipeline (it prints each stage's diagnostics but
//! no longer asserts); this integration test is the CI-gated home of the
//! stage-by-stage oracle.
//!
//! Run with: `cargo test -p mesh-repair --test repair_pipeline`

// Cube indices fit in u32 by platform convention; `vertices.len() as u32`
// for new-vertex indices is a representation choice, not narrowing.
#![allow(clippy::cast_possible_truncation)]
// `.expect()` on the fill/winding `Result`s: a panic IS the test failure mode.
#![allow(clippy::expect_used)]

use mesh_repair::{
    RepairParams, count_inconsistent_faces, fill_holes, fix_winding_order, repair_mesh,
    validate_mesh,
};
use mesh_types::{IndexedMesh, Point3, unit_cube};

/// Build a `unit_cube` plus six intentional defects — one per repair
/// operation in `mesh-repair`'s public surface:
///
/// 1. duplicate vertex (vert 8 = origin; face[1] re-routed onto it),
/// 2. degenerate triangle (collinear; area 0),
/// 3. duplicate face (exact copy of original face[0]),
/// 4. unreferenced vertex (vert 9; no face references it),
/// 5. reversed winding on face[5] (index ≠ 0 so BFS flips this one, not the other 11),
/// 6. small hole (face[11] deleted → 3-edge boundary loop).
///
/// Returns an 11-vertex / 13-face broken mesh.
fn build_broken_mesh() -> IndexedMesh {
    let mut mesh = unit_cube();
    assert_eq!(mesh.vertices.len(), 8);
    assert_eq!(mesh.faces.len(), 12);

    // unit_cube ordering: face[0] = [0,2,1] (dup target); face[1] = [0,3,2].
    let face0_copy = mesh.faces[0];
    assert_eq!(face0_copy, [0, 2, 1], "unit_cube face[0] convention");
    assert_eq!(mesh.faces[1], [0, 3, 2], "unit_cube face[1] convention");

    // Defect 5: reverse winding of face[5].
    mesh.faces[5].swap(1, 2);

    // Defect 6: delete face[11] → 3-edge boundary loop.
    mesh.faces.truncate(11);

    // Three extra vertices for defects 1, 4, 2.
    let dup_idx = mesh.vertices.len() as u32; // 8
    mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Point3::new(99.0, 99.0, 99.0)); // 9 — unreferenced
    let mid_idx = mesh.vertices.len() as u32; // 10
    mesh.vertices.push(Point3::new(0.5, 0.0, 0.0));

    // Defect 1: re-route face[1] = [0,3,2] onto the duplicate vertex.
    for slot in &mut mesh.faces[1] {
        if *slot == 0 {
            *slot = dup_idx;
            break;
        }
    }

    // Defect 2: append a degenerate (collinear) triangle.
    mesh.faces.push([0, mid_idx, 1]);
    // Defect 3: append an exact duplicate of original face[0].
    mesh.faces.push(face0_copy);

    assert_eq!(mesh.vertices.len(), 11);
    assert_eq!(mesh.faces.len(), 13);
    mesh
}

#[test]
fn end_to_end_pipeline_cleans_six_defect_cube() {
    let mut mesh = build_broken_mesh();

    // ── stage 0: initial validation ────────────────────────────────────
    // Defect side-effects make boundary/non-manifold counts hard to predict
    // exactly; anchor the deterministic counts and the negative topology.
    let report = validate_mesh(&mesh);
    assert_eq!(report.vertex_count, 11);
    assert_eq!(report.face_count, 13);
    assert!(!report.is_watertight);
    assert!(!report.is_manifold);
    assert!(report.degenerate_face_count >= 1);
    assert!(report.duplicate_face_count >= 1);
    assert!(
        report.boundary_edge_count >= 3,
        "at minimum the 3 hole edges"
    );

    // ── stage 1: repair_mesh (basic pipeline) ──────────────────────────
    let summary = repair_mesh(&mut mesh, &RepairParams::default());
    assert_eq!(summary.vertices_welded, 1, "vert 8 → vert 0");
    assert_eq!(summary.degenerates_removed, 1);
    assert_eq!(summary.duplicates_removed, 1);
    assert_eq!(
        summary.unreferenced_removed, 3,
        "cascade: hand-stranded vert 9 + post-weld orphan 8 + post-degen orphan 10",
    );
    assert_eq!(summary.final_vertices, 8);
    assert_eq!(summary.final_faces, 11);

    let report = validate_mesh(&mesh);
    assert_eq!(
        report.boundary_edge_count, 3,
        "only the hole's 3 edges remain"
    );
    assert_eq!(report.non_manifold_edge_count, 0);
    assert_eq!(report.degenerate_face_count, 0);
    assert_eq!(report.duplicate_face_count, 0);
    assert!(report.is_manifold);
    assert!(!report.is_watertight, "the hole is still open");
    assert_eq!(
        count_inconsistent_faces(&mesh),
        1,
        "repair_mesh does not fix winding: only reversed face[5] is inconsistent",
    );

    // ── stage 2: fill_holes ────────────────────────────────────────────
    let filled = fill_holes(&mut mesh, 10).expect("fill_holes");
    assert_eq!(filled, 1, "the deleted-face triangle");
    let report = validate_mesh(&mesh);
    assert_eq!(
        report.vertex_count, 8,
        "ear-clipping reuses boundary vertices"
    );
    assert_eq!(report.face_count, 12, "fill added 1 triangle (11 → 12)");
    assert_eq!(report.boundary_edge_count, 0);
    assert!(report.is_watertight, "surface is now closed");
    assert!(report.is_manifold);
    // `fill_holes` does not fix winding — face[5] is always still inconsistent.
    // The ear-clipped fill triangle's own winding is nondeterministic (boundary-
    // loop traversal order), so the count is 1 OR 2; only `>= 1` is deterministic.
    assert!(
        count_inconsistent_faces(&mesh) >= 1,
        "fill_holes does not fix winding: reversed face[5] is still inconsistent",
    );

    // ── stage 3: fix_winding_order ─────────────────────────────────────
    fix_winding_order(&mut mesh).expect("fix_winding_order");
    let report = validate_mesh(&mesh);
    assert_eq!(report.vertex_count, 8);
    assert_eq!(report.face_count, 12);
    assert_eq!(report.boundary_edge_count, 0);
    assert!(report.is_watertight, "terminal: watertight");
    assert!(report.is_manifold, "terminal: manifold");
    assert!(
        !report.is_inside_out,
        "terminal: outward winding (BFS seed face[0] is outward)"
    );
    assert_eq!(report.degenerate_face_count, 0);
    assert_eq!(report.duplicate_face_count, 0);
    assert_eq!(
        count_inconsistent_faces(&mesh),
        0,
        "stage-ordering oracle: inconsistent faces 1 (post-repair) → ≥1 (post-fill) → 0 (post-winding)",
    );
}
