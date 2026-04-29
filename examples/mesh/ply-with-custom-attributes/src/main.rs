//! Per-vertex custom-attribute PLY round-trip.
//!
//! Builds a unit cube, attaches `extras["height"]` = each vertex's
//! z-coordinate, saves via [`mesh_io::save_ply_attributed`], reloads via
//! [`mesh_io::load_ply_attributed`], and asserts every numerical anchor
//! listed in the README.
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
    mesh.insert_extra("height", heights.clone())?;

    // Write the binary PLY to <crate>/out/cube.ply (cwd-independent).
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let out_path = out_dir.join("cube.ply");
    save_ply_attributed(&mesh, &out_path, true)?;

    // Reload and assert every round-trip invariant.
    let reloaded = load_ply_attributed(&out_path)?;
    assert_eq!(reloaded.vertex_count(), 8, "vertex count must roundtrip");
    assert_eq!(reloaded.face_count(), 12, "face count must roundtrip");
    assert_eq!(
        reloaded.extras.len(),
        1,
        "expected exactly one extra key (\"height\")",
    );
    assert!(
        reloaded.extras.contains_key("height"),
        "\"height\" extra missing after roundtrip",
    );
    let reloaded_heights = &reloaded.extras["height"];
    assert_eq!(
        reloaded_heights.len(),
        reloaded.vertex_count(),
        "heights length must equal vertex count",
    );
    assert_eq!(
        reloaded_heights, &heights,
        "extras[\"height\"] must be bit-equal after roundtrip",
    );
    for (i, v) in reloaded.geometry.vertices.iter().enumerate() {
        // Bit-equality (not approx) — values are 0.0 / 1.0, exactly
        // representable in f32; PLY writes f32 LE bytes and the loader
        // reads them back without any further conversion.
        assert_eq!(
            reloaded_heights[i].to_bits(),
            (v.z as f32).to_bits(),
            "height[{i}] must bit-equal reloaded vertex z-coord",
        );
    }

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
    println!("OK — round-trip verified");

    Ok(())
}
