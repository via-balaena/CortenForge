//! The repair pipeline pattern from broken to clean.
//!
//! Builds a `unit_cube` and corrupts it with six defects (one per repair
//! operation in `mesh-repair`'s public surface), then runs the pipeline in
//! three explicit stages — `repair_mesh` → `fill_holes` → `fix_winding_order`
//! — printing each stage's diagnostics so you can watch the mesh go from
//! broken to watertight, manifold, and outward-wound. The stage-by-stage
//! repair oracle is owned by `mesh-repair`'s `tests/repair_pipeline.rs`.
//!
//! `repair_mesh` is the **basic** pipeline (degenerate-removal + welding +
//! duplicate-face removal + unreferenced-vertex cleanup); hole-filling and
//! winding correction are separately-exposed APIs because they're heavier
//! operations and not always desired. The example calls all three to span
//! the full repair surface.
//!
//! See `examples/mesh/README.md` for cadence; see
//! `docs/studies/mesh_architecture/src/40-repair.md` for the repair-pattern
//! depth pass.

// `vertices.len() as u32` for new-vertex indices. Cube indices fit in u32 by
// platform convention; the cast is a representation choice, not narrowing.
#![allow(clippy::cast_possible_truncation)]

use std::path::Path;

use anyhow::Result;
use mesh_io::{load_ply, save_ply};
use mesh_repair::{
    MeshAdjacency, RepairParams, RepairSummary, count_inconsistent_faces, detect_holes, fill_holes,
    fix_winding_order, repair_mesh, validate_mesh,
};
use mesh_types::{IndexedMesh, Point3, unit_cube};

fn main() -> Result<()> {
    // ── stage 0: build the broken mesh ─────────────────────────────────
    let (mut mesh, dup_idx, unref_idx, mid_idx) = build_broken_mesh()?;

    // Snapshot the broken state to disk before any repair operation.
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let before_path = out_dir.join("before.ply");
    let after_path = out_dir.join("after.ply");
    save_ply(&mesh, &before_path, true)?;

    println!("==== mesh-repair-walkthrough ====");
    println!();
    print_defects(dup_idx, unref_idx, mid_idx);

    print_diagnostics("initial validation", &mesh);

    // ── stage 1: repair_mesh ───────────────────────────────────────────
    let summary = repair_mesh(&mut mesh, &RepairParams::default());
    print_repair_summary(&summary);
    println!();
    print_diagnostics("post-repair_mesh validation", &mesh);

    // ── stage 2: fill_holes ────────────────────────────────────────────
    let filled = fill_holes(&mut mesh, 10)?;
    println!();
    println!("stage 2 — fill_holes(max_edges=10)");
    println!("  filled              : {filled} hole(s)");
    println!();
    print_diagnostics("post-fill_holes validation", &mesh);

    // ── stage 3: fix_winding_order ─────────────────────────────────────
    fix_winding_order(&mut mesh)?;
    println!();
    println!("stage 3 — fix_winding_order");
    println!("  done                : winding propagated by BFS from face[0]");
    println!();
    print_diagnostics("final validation", &mesh);

    // ── persist + round-trip ───────────────────────────────────────────
    // The stage-by-stage repair oracle (each stage's exact counts +
    // watertight/manifold/winding transitions) is owned by mesh-repair's
    // `tests/repair_pipeline.rs` end-to-end lib test; this walkthrough
    // demonstrates the pattern and prints each stage's diagnostics.
    save_ply(&mesh, &after_path, true)?;
    let after_loaded = load_ply(&after_path)?;
    let before_loaded = load_ply(&before_path)?;

    println!();
    println!("artifacts:");
    println!(
        "  out/before.ply : {}v, {}f (round-trip)",
        before_loaded.vertices.len(),
        before_loaded.faces.len(),
    );
    println!(
        "  out/after.ply  : {}v, {}f (round-trip)",
        after_loaded.vertices.len(),
        after_loaded.faces.len(),
    );
    println!();
    println!("OK — repair pipeline demonstrated");

    Ok(())
}

/// Build the unit cube plus six intentional defects.
/// Returns the broken mesh and the three appended-vertex indices
/// (dup, unreferenced, midpoint) so the caller can label them in the
/// printout.
fn build_broken_mesh() -> Result<(IndexedMesh, u32, u32, u32)> {
    let mut mesh = unit_cube();
    anyhow::ensure!(
        mesh.vertices.len() == 8 && mesh.faces.len() == 12,
        "expected unit_cube to have 8 verts and 12 faces, got {}v {}f",
        mesh.vertices.len(),
        mesh.faces.len(),
    );

    // unit_cube ordering: face[0] = [0, 2, 1] (bottom-tri-1; duplicate target);
    // face[1] = [0, 3, 2] (bottom-tri-2; re-routed via the dup vert).
    let face0_copy = mesh.faces[0];
    anyhow::ensure!(
        face0_copy == [0, 2, 1],
        "expected unit_cube face[0] to be [0,2,1], got {face0_copy:?}",
    );
    anyhow::ensure!(
        mesh.faces[1] == [0, 3, 2],
        "expected unit_cube face[1] to be [0,3,2], got {:?}",
        mesh.faces[1],
    );

    // Defect 5: reverse winding of face[5] = [0,5,4] → [0,4,5].
    // Index ≠ 0 so BFS from face[0] flips this one face, not the other 11.
    mesh.faces[5].swap(1, 2);

    // Defect 6: delete face[11] = [1,6,5] (right-tri-2) → 3-edge boundary loop.
    mesh.faces.truncate(11);

    // Three extra vertices for defects 1, 4, and 2.
    let dup_idx = mesh.vertices.len() as u32; // = 8
    mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
    let unref_idx = mesh.vertices.len() as u32; // = 9
    mesh.vertices.push(Point3::new(99.0, 99.0, 99.0));
    let mid_idx = mesh.vertices.len() as u32; // = 10
    mesh.vertices.push(Point3::new(0.5, 0.0, 0.0));

    // Defect 1: re-route face[1] = [0,3,2] → [dup_idx, 3, 2].
    for slot in &mut mesh.faces[1] {
        if *slot == 0 {
            *slot = dup_idx;
            break;
        }
    }

    // Defect 2: append a degenerate triangle (collinear; area = 0).
    mesh.faces.push([0, mid_idx, 1]);

    // Defect 3: append an exact duplicate of original face[0].
    mesh.faces.push(face0_copy);

    anyhow::ensure!(
        mesh.vertices.len() == 11 && mesh.faces.len() == 13,
        "broken mesh construction: expected 11v/13f, got {}v/{}f",
        mesh.vertices.len(),
        mesh.faces.len(),
    );
    Ok((mesh, dup_idx, unref_idx, mid_idx))
}

/// Print the labeled defect inventory built into the broken mesh.
fn print_defects(dup_idx: u32, unref_idx: u32, mid_idx: u32) {
    println!("stage 0 — broken mesh built (11v 13f; 6 defects intentional)");
    println!("  1. duplicate vertex   : vert {dup_idx} = (0,0,0); face[1] re-routed to use it");
    println!("  2. degenerate triangle: face[12] = [0, {mid_idx}, 1] (collinear; area = 0)");
    println!("  3. duplicate face     : face[13] copies original face[0] = [0, 2, 1]");
    println!("  4. unreferenced vertex: vert {unref_idx} = (99,99,99); no face references it");
    println!("  5. reversed winding   : face[5] = [0,5,4] → [0,4,5] (indices 1 and 2 swapped)");
    println!("  6. small hole         : face[11] = [1,6,5] deleted; 3-edge boundary loop");
    println!();
}

/// Print a stage-labeled validation block: the full `MeshReport` Display
/// followed by the two extra scalars the report doesn't carry inline
/// (`count_inconsistent_faces` and `detect_holes` count + sizes).
fn print_diagnostics(label: &str, mesh: &IndexedMesh) {
    let report = validate_mesh(mesh);
    let inconsistent = count_inconsistent_faces(mesh);
    let adjacency = MeshAdjacency::build(&mesh.faces);
    let holes = detect_holes(mesh, &adjacency);
    let hole_sizes: Vec<usize> = holes
        .iter()
        .map(mesh_repair::BoundaryLoop::edge_count)
        .collect();

    println!("{label}:");
    print!("{report}");
    println!("  count_inconsistent_faces: {inconsistent}");
    println!(
        "  detect_holes            : {} loop(s), edges: {hole_sizes:?}",
        holes.len()
    );
}

/// Print the `RepairSummary` block — the one-line `Display` plus a
/// structured field-by-field readout (the cascade-cleanup count is the
/// example's headline number; see README for why it's 3 not 1).
fn print_repair_summary(summary: &RepairSummary) {
    println!();
    println!("stage 1 — repair_mesh(RepairParams::default())");
    println!("  one-line            : {summary}");
    println!(
        "  initial             : {}v, {}f",
        summary.initial_vertices, summary.initial_faces
    );
    println!(
        "  final               : {}v, {}f",
        summary.final_vertices, summary.final_faces
    );
    println!("  vertices_welded     : {}", summary.vertices_welded);
    println!("  degenerates_removed : {}", summary.degenerates_removed);
    println!("  duplicates_removed  : {}", summary.duplicates_removed);
    println!(
        "  unreferenced_removed: {}  (cascade — see README)",
        summary.unreferenced_removed,
    );
}
