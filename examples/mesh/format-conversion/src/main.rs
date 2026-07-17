//! STL / PLY / OBJ round-trip — what each format preserves.
//!
//! Builds a unit cube (8 verts, 12 tris) and saves it through each of the
//! three major mesh formats independently (fan-from-source, not a chain).
//! Each artifact is reloaded and its five invariants — vertex count, face
//! count, signed volume, surface area, and axis-aligned bounding box —
//! printed in a comparison table alongside the source row.
//!
//! The headline is a divergence: STL files store three fresh vertices per
//! triangle and `mesh_io::load_stl` does **not** dedup on load, so the
//! reloaded STL has 36 vertices, not 8. PLY and OBJ both store explicit
//! vertex sharing and recover the original 8. All three preserve the
//! cube's *geometry* (volume ≈ 1, area ≈ 6, unit AABB); only PLY and OBJ
//! preserve its *topology*. Per-format count preservation and the STL
//! geometry-vs-topology divergence are owned by `mesh-io`'s round-trip lib
//! tests (including `stl_roundtrip_preserves_geometry_not_vertex_sharing`);
//! this example demonstrates the divergence rather than asserting it.
//!
//! See `examples/mesh/README.md` for cadence; see
//! `docs/studies/mesh_architecture/src/20-io.md` for the multi-format I/O
//! reference.

use std::path::Path;

use anyhow::Result;
use mesh_io::{load_obj, load_ply, load_stl, save_obj, save_ply, save_stl};
use mesh_types::{Aabb, Bounded, IndexedMesh, unit_cube};

/// Five invariants we read off each loaded mesh.
struct Summary {
    vertex_count: usize,
    face_count: usize,
    volume: f64,
    area: f64,
    aabb: Aabb,
}

fn summarize(mesh: &IndexedMesh) -> Summary {
    Summary {
        vertex_count: mesh.vertex_count(),
        face_count: mesh.face_count(),
        volume: mesh.signed_volume(),
        area: mesh.surface_area(),
        aabb: mesh.aabb(),
    }
}

fn print_row(label: &str, s: &Summary) {
    println!(
        "{label:<10} {:>5} {:>5} {:>10.6} {:>10.6}  min=({:.3},{:.3},{:.3}) max=({:.3},{:.3},{:.3})",
        s.vertex_count,
        s.face_count,
        s.volume,
        s.area,
        s.aabb.min.x,
        s.aabb.min.y,
        s.aabb.min.z,
        s.aabb.max.x,
        s.aabb.max.y,
        s.aabb.max.z,
    );
}

fn main() -> Result<()> {
    // Source: 8 verts, 12 tris, volume = 1, area = 6.
    let source = unit_cube();
    let s_source = summarize(&source);

    // Output directory anchored relative to the crate, not the cwd.
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let stl_path = out_dir.join("cube.stl");
    let ply_path = out_dir.join("cube.ply");
    let obj_path = out_dir.join("cube.obj");

    // STL: binary. Loader does NOT dedup — expect 36 verts back.
    save_stl(&source, &stl_path, true)?;
    let stl_loaded = load_stl(&stl_path)?;
    let s_stl = summarize(&stl_loaded);

    // PLY: binary. Stores explicit vertex sharing — expect 8 verts back.
    save_ply(&source, &ply_path, true)?;
    let ply_loaded = load_ply(&ply_path)?;
    let s_ply = summarize(&ply_loaded);

    // OBJ: ASCII (only option). Stores explicit vertex sharing.
    save_obj(&source, &obj_path)?;
    let obj_loaded = load_obj(&obj_path)?;
    let s_obj = summarize(&obj_loaded);

    // Numerical summary table.
    println!("==== format-conversion ====");
    println!(
        "{:<10} {:>5} {:>5} {:>10} {:>10}  aabb",
        "format", "verts", "faces", "volume", "area",
    );
    print_row("source", &s_source);
    print_row("stl", &s_stl);
    print_row("ply", &s_ply);
    print_row("obj", &s_obj);

    // The table above IS the comparison: read down the `verts` column to
    // see the headline divergence — STL does NOT dedup (36 = 12 tris × 3)
    // while PLY and OBJ recover the shared 8; all three preserve the
    // cube's geometry (volume ≈ 1, area ≈ 6, unit AABB). Format-preserving
    // round-trip correctness is owned by `mesh-io`'s per-format lib tests,
    // so this example demonstrates the workflow rather than asserting it.

    println!("output     : {}", out_dir.display());
    println!("OK — format conversion demonstrated");

    Ok(())
}
