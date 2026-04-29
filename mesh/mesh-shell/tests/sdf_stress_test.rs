//! Stress test for the SDF path engine fix (commit 11.5).
//!
//! Empirically validates the recon claims in `SDF_FIX_PLAN_COMMIT_11_5.md`
//! and the predicted fix layers. THROWAWAY — delete after the engine fix lands
//! and the integration tests in `generate.rs::tests` cover the same surface.
//!
//! Run with:
//!     cargo test -p mesh-shell --release --test sdf_stress_test -- --ignored --nocapture
//!
//! The `--ignored` flag avoids running these in the default `cargo test`
//! pass; `--nocapture` surfaces the empirical numbers we want to read.

#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::expect_used)]
#![allow(clippy::print_stdout)]
#![allow(clippy::suboptimal_flops)]
#![allow(clippy::doc_markdown)]

use mesh_offset::{OffsetConfig, offset_mesh};
use mesh_repair::{MeshAdjacency, remove_unreferenced_vertices, validate_mesh, weld_vertices};
use mesh_shell::{ShellBuilder, validate_shell};
use mesh_types::{IndexedMesh, Point3};

const WALL_THICKNESS_MM: f64 = 2.5;
const VOXEL_MM: f64 = 0.3;

fn closed_cube_10mm() -> IndexedMesh {
    let mut mesh = IndexedMesh::new();
    mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
    mesh.vertices.push(Point3::new(10.0, 10.0, 0.0));
    mesh.vertices.push(Point3::new(0.0, 10.0, 0.0));
    mesh.vertices.push(Point3::new(0.0, 0.0, 10.0));
    mesh.vertices.push(Point3::new(10.0, 0.0, 10.0));
    mesh.vertices.push(Point3::new(10.0, 10.0, 10.0));
    mesh.vertices.push(Point3::new(0.0, 10.0, 10.0));
    // Bottom (outward normal -z)
    mesh.faces.push([0, 2, 1]);
    mesh.faces.push([0, 3, 2]);
    // Top (outward normal +z)
    mesh.faces.push([4, 5, 6]);
    mesh.faces.push([4, 6, 7]);
    // Front (-y)
    mesh.faces.push([0, 1, 5]);
    mesh.faces.push([0, 5, 4]);
    // Back (+y)
    mesh.faces.push([2, 3, 7]);
    mesh.faces.push([2, 7, 6]);
    // Left (-x)
    mesh.faces.push([0, 4, 7]);
    mesh.faces.push([0, 7, 3]);
    // Right (+x)
    mesh.faces.push([1, 2, 6]);
    mesh.faces.push([1, 6, 5]);
    mesh
}

fn open_box_10mm() -> IndexedMesh {
    // Same as closed but missing top (4,5,6) + (4,6,7).
    // This matches commit 11's input shape.
    let mut mesh = IndexedMesh::new();
    mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
    mesh.vertices.push(Point3::new(10.0, 10.0, 0.0));
    mesh.vertices.push(Point3::new(0.0, 10.0, 0.0));
    mesh.vertices.push(Point3::new(0.0, 0.0, 10.0));
    mesh.vertices.push(Point3::new(10.0, 0.0, 10.0));
    mesh.vertices.push(Point3::new(10.0, 10.0, 10.0));
    mesh.vertices.push(Point3::new(0.0, 10.0, 10.0));
    mesh.faces.push([0, 2, 1]);
    mesh.faces.push([0, 3, 2]);
    mesh.faces.push([0, 1, 5]);
    mesh.faces.push([0, 5, 4]);
    mesh.faces.push([2, 3, 7]);
    mesh.faces.push([2, 7, 6]);
    mesh.faces.push([0, 4, 7]);
    mesh.faces.push([0, 7, 3]);
    mesh.faces.push([1, 2, 6]);
    mesh.faces.push([1, 6, 5]);
    mesh
}

fn print_shell_stats(label: &str, shell: &IndexedMesh) {
    let report = validate_mesh(shell);
    let shell_validation = validate_shell(shell);
    let v = shell.vertices.len();
    let f = shell.faces.len();
    println!(
        "  {label:32} verts={v:>6} faces={f:>6}  vert/face={:.2}",
        v as f64 / f.max(1) as f64
    );
    println!(
        "    boundary_edges={:>6}  non_manifold={:>3}  degenerate={:>3}  is_inside_out={}",
        report.boundary_edge_count,
        report.non_manifold_edge_count,
        report.degenerate_face_count,
        report.is_inside_out
    );
    println!(
        "    is_watertight={}  is_manifold={}  is_printable={}  has_consistent_winding={}",
        shell_validation.is_watertight,
        shell_validation.is_manifold,
        shell_validation.is_printable(),
        shell_validation.has_consistent_winding
    );
}

fn signed_volume(mesh: &IndexedMesh) -> f64 {
    let mut volume = 0.0;
    for face in &mesh.faces {
        let v0 = &mesh.vertices[face[0] as usize];
        let v1 = &mesh.vertices[face[1] as usize];
        let v2 = &mesh.vertices[face[2] as usize];
        volume += v0.x * (v1.y * v2.z - v2.y * v1.z)
            + v1.x * (v2.y * v0.z - v0.y * v2.z)
            + v2.x * (v0.y * v1.z - v1.y * v0.z);
    }
    volume / 6.0
}

/// Apply the bug 1+2 fix layer: weld + per-face flip on the outer mesh.
/// Mirrors what the proposed engine fix would do internally.
fn weld_and_flip(outer: &mut IndexedMesh) {
    let weld_eps = VOXEL_MM * 1e-3;
    let merged = weld_vertices(outer, weld_eps);
    let _orphans = remove_unreferenced_vertices(outer);
    println!("    [weld+flip] merged={merged} orphans removed");
    for face in &mut outer.faces {
        face.swap(1, 2);
    }
}

fn assemble_shell(inner: &IndexedMesh, outer: &IndexedMesh) -> IndexedMesh {
    let mut shell = IndexedMesh::new();
    shell.vertices.extend_from_slice(&inner.vertices);
    let inner_count = inner.vertices.len() as u32;
    shell.vertices.extend_from_slice(&outer.vertices);
    // Reverse inner winding (cavity-facing).
    for face in &inner.faces {
        shell.faces.push([face[0], face[2], face[1]]);
    }
    // Outer faces with index offset.
    for face in &outer.faces {
        shell.faces.push([
            face[0] + inner_count,
            face[1] + inner_count,
            face[2] + inner_count,
        ]);
    }
    shell
}

// ---------------------------------------------------------------------------
// Empirical baselines (current broken state)
// ---------------------------------------------------------------------------

#[test]
#[ignore = "stress test for commit 11.5 SDF engine fix; run on demand"]
fn stress_baseline_closed_cube_high_quality() {
    let inner = closed_cube_10mm();
    let result = ShellBuilder::new(&inner)
        .wall_thickness(WALL_THICKNESS_MM)
        .voxel_size(VOXEL_MM)
        .high_quality()
        .build()
        .expect("shell builder build()");

    println!("\n[BASELINE] closed cube, .high_quality() — current (broken) state");
    println!(
        "  inner: verts={} faces={}",
        inner.vertices.len(),
        inner.faces.len()
    );
    println!(
        "  shell_stats: inner_vc={} outer_vc={} rim_fc={} total_fc={} method={}",
        result.shell_stats.inner_vertex_count,
        result.shell_stats.outer_vertex_count,
        result.shell_stats.rim_face_count,
        result.shell_stats.total_face_count,
        result.shell_stats.wall_method
    );
    print_shell_stats("shell", &result.mesh);
    println!("    signed_volume = {:.2}", signed_volume(&result.mesh));

    // Plan claims:
    let outer_face_count =
        result.shell_stats.total_face_count - inner.faces.len() - result.shell_stats.rim_face_count;
    println!(
        "  outer_face_count={} ; outer_vert_count={} ; ratio={:.3}",
        outer_face_count,
        result.shell_stats.outer_vertex_count,
        result.shell_stats.outer_vertex_count as f64 / outer_face_count.max(1) as f64
    );
}

#[test]
#[ignore = "stress test for commit 11.5 SDF engine fix; run on demand"]
fn stress_baseline_open_box_high_quality() {
    let inner = open_box_10mm();
    let result = ShellBuilder::new(&inner)
        .wall_thickness(WALL_THICKNESS_MM)
        .voxel_size(VOXEL_MM)
        .high_quality()
        .build()
        .expect("shell builder build()");

    println!("\n[BASELINE] open box, .high_quality() — current (broken) state");
    println!(
        "  inner: verts={} faces={}",
        inner.vertices.len(),
        inner.faces.len()
    );
    println!(
        "  shell_stats: inner_vc={} outer_vc={} rim_fc={} total_fc={} method={}",
        result.shell_stats.inner_vertex_count,
        result.shell_stats.outer_vertex_count,
        result.shell_stats.rim_face_count,
        result.shell_stats.total_face_count,
        result.shell_stats.wall_method
    );
    print_shell_stats("shell", &result.mesh);
    println!("    signed_volume = {:.2}", signed_volume(&result.mesh));

    if let Some(v) = &result.shell_stats.validation {
        for issue in &v.issues {
            println!("    issue: {issue}");
        }
    }
}

// ---------------------------------------------------------------------------
// Spikes — predict what each fix layer produces independently
// ---------------------------------------------------------------------------

#[test]
#[ignore = "stress test for commit 11.5 SDF engine fix; run on demand"]
fn stress_spike_closed_cube_offset_only() {
    let inner = closed_cube_10mm();
    let cfg = OffsetConfig::default().with_resolution(VOXEL_MM);
    let outer = offset_mesh(&inner, WALL_THICKNESS_MM, &cfg).expect("offset_mesh");

    println!("\n[SPIKE] closed cube — outer from offset_mesh ALONE (no weld, no flip)");
    println!(
        "  outer: verts={} faces={}  vert/face={:.2}",
        outer.vertices.len(),
        outer.faces.len(),
        outer.vertices.len() as f64 / outer.faces.len().max(1) as f64
    );
    let report = validate_mesh(&outer);
    println!(
        "  outer report: boundary={} non_manifold={} is_watertight={} is_inside_out={}",
        report.boundary_edge_count,
        report.non_manifold_edge_count,
        report.is_watertight,
        report.is_inside_out
    );
    println!("  outer signed_volume = {:.2}", signed_volume(&outer));
    let adj = MeshAdjacency::build(&outer.faces);
    println!(
        "  outer adjacency: edges={} boundary={} non_manifold={}",
        adj.edge_count(),
        adj.boundary_edge_count(),
        adj.non_manifold_edge_count()
    );
}

#[test]
#[ignore = "stress test for commit 11.5 SDF engine fix; run on demand"]
fn stress_spike_closed_cube_weld_only() {
    let inner = closed_cube_10mm();
    let cfg = OffsetConfig::default().with_resolution(VOXEL_MM);
    let mut outer = offset_mesh(&inner, WALL_THICKNESS_MM, &cfg).expect("offset_mesh");

    let weld_eps = VOXEL_MM * 1e-3;
    let merged = weld_vertices(&mut outer, weld_eps);
    let _ = remove_unreferenced_vertices(&mut outer);

    println!("\n[SPIKE] closed cube — outer post-weld (no flip)");
    println!("  weld merged {merged} verts");
    println!(
        "  outer: verts={} faces={}  vert/face={:.2}",
        outer.vertices.len(),
        outer.faces.len(),
        outer.vertices.len() as f64 / outer.faces.len().max(1) as f64
    );
    let report = validate_mesh(&outer);
    println!(
        "  outer report: boundary={} non_manifold={} is_watertight={} is_inside_out={}",
        report.boundary_edge_count,
        report.non_manifold_edge_count,
        report.is_watertight,
        report.is_inside_out
    );
    println!("  outer signed_volume = {:.2}", signed_volume(&outer));
    println!(
        "  PREDICTION: weld alone closes the soup → is_watertight=true, is_inside_out STILL true"
    );
}

#[test]
#[ignore = "stress test for commit 11.5 SDF engine fix; run on demand"]
fn stress_spike_closed_cube_weld_plus_flip_assembled() {
    // This is what bug 1+2 fix would produce on a closed input.
    // PREDICTION: shell becomes printable (watertight, manifold, !is_inside_out).
    let inner = closed_cube_10mm();
    let cfg = OffsetConfig::default().with_resolution(VOXEL_MM);
    let mut outer = offset_mesh(&inner, WALL_THICKNESS_MM, &cfg).expect("offset_mesh");
    weld_and_flip(&mut outer);

    let shell = assemble_shell(&inner, &outer);
    println!(
        "\n[SPIKE] closed cube — bug 1+2 fix simulated, shell assembled (no rim — closed input)"
    );
    print_shell_stats("shell", &shell);
    println!("    signed_volume = {:.2}", signed_volume(&shell));
    println!(
        "  PREDICTION: is_watertight=true, is_manifold=true, is_inside_out=false, is_printable=true"
    );
}

#[test]
#[ignore = "stress test for commit 11.5 SDF engine fix; run on demand"]
fn stress_spike_open_box_weld_plus_flip_assembled() {
    // This is what bug 1+2 fix would produce on an OPEN input WITHOUT bug 3 fallback.
    // PREDICTION: still not watertight — 4 unstitched inner-boundary edges remain.
    // PREDICTION: outer is closed (no boundary loops), so rim algorithm would early-return.
    let inner = open_box_10mm();
    let cfg = OffsetConfig::default().with_resolution(VOXEL_MM);
    let mut outer = offset_mesh(&inner, WALL_THICKNESS_MM, &cfg).expect("offset_mesh");
    weld_and_flip(&mut outer);

    let outer_adj = MeshAdjacency::build(&outer.faces);
    let inner_adj = MeshAdjacency::build(&inner.faces);
    println!("\n[SPIKE] open box — bug 1+2 fix on outer (no rim, no fallback)");
    println!(
        "  inner: verts={} faces={}",
        inner.vertices.len(),
        inner.faces.len()
    );
    println!(
        "  inner_adj: boundary={} (expected 4 for open box top)",
        inner_adj.boundary_edge_count()
    );
    println!(
        "  outer post weld+flip: verts={} faces={} vert/face={:.2}",
        outer.vertices.len(),
        outer.faces.len(),
        outer.vertices.len() as f64 / outer.faces.len().max(1) as f64
    );
    println!(
        "  outer_adj: boundary={} non_manifold={} is_watertight={} is_inside_out={}",
        outer_adj.boundary_edge_count(),
        outer_adj.non_manifold_edge_count(),
        outer_adj.is_watertight(),
        validate_mesh(&outer).is_inside_out
    );
    println!("  outer signed_volume = {:.2}", signed_volume(&outer));

    let shell = assemble_shell(&inner, &outer);
    print_shell_stats("shell", &shell);
    println!("    signed_volume = {:.2}", signed_volume(&shell));
    println!("  PREDICTION: shell.boundary_edges == 4 (inner's open top, outer is closed wrap)");
    println!("  PREDICTION: !is_watertight, motivates bug 3 fallback");
}
