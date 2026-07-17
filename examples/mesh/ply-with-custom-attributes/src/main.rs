//! Per-vertex custom-attribute PLY round-trip.
//!
//! Builds a unit cube, attaches `extras["height"]` = each vertex's
//! z-coordinate, saves via [`mesh_io::save_ply_attributed`], reloads via
//! [`mesh_io::load_ply_attributed`], and prints the recovered values.
//! A demonstration of the workflow — round-trip correctness (count
//! preservation + bit-equal extras) is owned by `mesh-io`'s
//! `roundtrip_attributed_*` lib tests, not asserted here.
//!
//! See `examples/mesh/README.md` for cadence; see
//! `docs/studies/mesh_architecture/src/80-examples.md` for the spec.

// f32 ⟷ f64 conversion is intrinsic to PLY (single-precision on disk vs
// double-precision in memory) — same allow rationale as mesh-io::ply.
#![allow(clippy::cast_possible_truncation)]

use std::path::Path;

use anyhow::Result;
use mesh_io::{load_ply_attributed, save_ply_attributed};
use mesh_types::{AttributedMesh, unit_cube};

fn main() -> Result<()> {
    // Build the unit cube (8 verts, 12 tris) and wrap as AttributedMesh.
    let mut mesh = AttributedMesh::from(unit_cube());

    // Populate `extras["height"]` = each vertex's z-coordinate (f32).
    let heights: Vec<f32> = mesh.geometry.vertices.iter().map(|v| v.z as f32).collect();
    mesh.insert_extra("height", heights)?;

    // Write the binary PLY to <crate>/out/cube.ply (cwd-independent).
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let out_path = out_dir.join("cube.ply");
    save_ply_attributed(&mesh, &out_path, true)?;

    // Reload the attributed PLY and read back the recovered values. This
    // is a demonstration of the save → load workflow — round-trip
    // correctness (vertex/face-count preservation + bit-equal `extras`) is
    // owned by `mesh-io`'s `roundtrip_attributed_*` lib tests, so this
    // example only illustrates the flow and prints what came back.
    let reloaded = load_ply_attributed(&out_path)?;
    // `.get` (not `[]` indexing) keeps this a pure demonstration — a
    // round-trip regression that dropped the extra would print `None`
    // here rather than panicking (the round-trip contract is lib-owned).
    let reloaded_heights = reloaded.extras.get("height");

    // Numerical summary for the user-facing numbers pass.
    println!("==== ply-with-custom-attributes ====");
    println!("vertex count : {} (expected 8)", reloaded.vertex_count());
    println!("face count   : {} (expected 12)", reloaded.face_count());
    println!(
        "extras keys  : {:?}",
        reloaded.extras.keys().collect::<Vec<_>>(),
    );
    println!("heights      : {reloaded_heights:?}");
    println!("output       : {}", out_path.display());
    println!("OK — round-trip demonstrated");

    Ok(())
}
