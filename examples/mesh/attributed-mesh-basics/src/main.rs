//! `AttributedMesh` shape — `SoA` per-vertex slots, extras, length validation.
//!
//! No I/O. This example demonstrates the type itself: how the underlying
//! [`mesh_types::IndexedMesh`], the six built-in slots
//! (`normals` / `colors` / `zone_ids` / `clearances` / `offsets` / `uvs`),
//! and the extensible [`extras`] map relate, and what the platform's
//! length-mismatch error looks like in practice.
//!
//! Pairs with `ply-with-custom-attributes` (which proves extras survive
//! disk) — this example explains what extras and slots ARE in the first
//! place.
//!
//! See `examples/mesh/README.md` for cadence; see
//! `docs/studies/mesh_architecture/src/10-types.md` for the type reference.
//!
//! [`extras`]: mesh_types::AttributedMesh::extras

// `extras` values are `Vec<f32>` by schema; populating from f64 vertex
// coordinates requires the cast. Same intrinsic-conversion rationale as
// commit 5 (ply-with-custom-attributes).
#![allow(clippy::cast_possible_truncation)]

use anyhow::Result;
use mesh_types::{AttributedMesh, VertexColor, unit_cube};

fn main() -> Result<()> {
    // ── Build the cube and wrap as AttributedMesh ──────────────────────
    // Every slot starts as None / empty — attribution is opt-in.
    let mut mesh = AttributedMesh::from(unit_cube());

    // ── Populate three representative slots ────────────────────────────
    // 1. normals — geometry-derived, area-weighted (built-in helper).
    mesh.compute_normals();

    // 2. colors — each vertex's (R, G, B) encodes its (x, y, z) position.
    //    For unit_cube, this maps the 8 corners to the 8 RGB-cube extremes.
    let colors: Vec<VertexColor> = mesh
        .geometry
        .vertices
        .iter()
        .map(|v| VertexColor::from_float(v.x as f32, v.y as f32, v.z as f32))
        .collect();
    mesh.colors = Some(colors);

    // 3. extras["height"] — per-vertex z-coordinate, mirroring the anchor
    //    in `ply-with-custom-attributes` for cross-example continuity.
    let heights: Vec<f32> = mesh.geometry.vertices.iter().map(|v| v.z as f32).collect();
    mesh.insert_extra("height", heights)?;

    // The visuals pass for this no-I/O example is a structured printout of
    // the populated state (normals/colors/extras populated, the other
    // slots still None). The slot-length + opt-in invariants this shows
    // off are owned by `mesh-types`' `AttributedMesh` lib tests.
    print_summary(&mesh);

    // ── Length-mismatch demo ───────────────────────────────────────────
    // Mesh has 8 vertices; `bogus` has 3. `insert_extra` rejects the
    // length mismatch (validation owned by `mesh-types`'
    // `AttributeMismatchError` lib tests) and leaves the mesh unchanged.
    let bogus = vec![0.0_f32; 3];
    println!();
    println!("length-mismatch demo:");
    match mesh.insert_extra("bogus", bogus) {
        Err(err) => {
            println!("  caught: {err}");
            println!(
                "  fields: name={:?}, expected={}, actual={}",
                err.name, err.expected, err.actual,
            );
        }
        Ok(()) => {
            println!("  note: insert_extra accepted a length-mismatched vector (unexpected)");
        }
    }
    println!(
        "  mesh.extras.len() after the attempt = {} (unchanged)",
        mesh.extras.len(),
    );
    println!();
    println!("OK — attributed-mesh shape demonstrated");

    Ok(())
}

/// Format an `Option<usize>` slot length as `Some(N entries)` or `None`.
fn slot_summary(len: Option<usize>) -> String {
    len.map_or_else(|| String::from("None"), |n| format!("Some({n} entries)"))
}

/// Print the `AttributedMesh`'s full populated state. This IS the
/// example's visuals pass — read down the slot lines and confirm
/// populated/None matches the README.
fn print_summary(mesh: &AttributedMesh) {
    println!("==== attributed-mesh-basics ====");
    println!(
        "geometry      : {} vertices, {} faces",
        mesh.vertex_count(),
        mesh.face_count(),
    );
    println!(
        "normals       : {}",
        slot_summary(mesh.normals.as_ref().map(Vec::len)),
    );
    println!(
        "colors        : {}",
        slot_summary(mesh.colors.as_ref().map(Vec::len)),
    );
    println!(
        "zone_ids      : {}",
        slot_summary(mesh.zone_ids.as_ref().map(Vec::len)),
    );
    println!(
        "clearances    : {}",
        slot_summary(mesh.clearances.as_ref().map(Vec::len)),
    );
    println!(
        "offsets       : {}",
        slot_summary(mesh.offsets.as_ref().map(Vec::len)),
    );
    println!(
        "uvs           : {}",
        slot_summary(mesh.uvs.as_ref().map(Vec::len)),
    );
    println!("extras        : {{");
    for (k, v) in &mesh.extras {
        println!("                  {k:?}: {v:?}");
    }
    println!("                }}");
}
