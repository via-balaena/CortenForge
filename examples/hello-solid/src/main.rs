//! Hello Solid — The "Hello World" of Code-First Design
//!
//! Create shapes using implicit surfaces, combine them with boolean operations,
//! and export the result to STL. This is the fundamental paradigm: geometry as code.
//!
//! Run with: `cargo run -p example-hello-solid`

use cf_design::Solid;
use nalgebra::Vector3;

fn main() -> anyhow::Result<()> {
    println!("=== CortenForge: Hello Solid ===\n");

    // ── 1. Primitives ───────────────────────────────────────────────────
    // Every shape starts as an implicit surface (negative inside, positive outside).
    let sphere = Solid::sphere(10.0);
    let cube = Solid::cuboid(Vector3::new(8.0, 8.0, 8.0));

    // ── 2. Sharp booleans ───────────────────────────────────────────────
    // Union: merge two shapes.
    let sharp_union = sphere.clone().union(cube.clone());
    // Subtract: carve one shape from another.
    let sharp_subtract = sphere.clone().subtract(cube.clone());
    // Intersect: keep only the overlap.
    let sharp_intersect = sphere.clone().intersect(cube.clone());

    // ── 3. Smooth booleans ──────────────────────────────────────────────
    // Organic blends — the blend radius (k) controls how much material
    // fills the transition. This is what makes code-first design shine:
    // smooth, organic shapes that are trivial in code but hard in CAD.
    let smooth = sphere.smooth_union(cube, 3.0);

    // ── 4. Transforms + composition ─────────────────────────────────────
    // Build something more interesting: a rounded bracket with a hole.
    let body = Solid::cuboid(Vector3::new(20.0, 10.0, 5.0)).round(1.5);
    let hole = Solid::cylinder(4.0, 6.0).translate(Vector3::new(10.0, 0.0, 0.0));
    let bracket = body.subtract(hole);

    // ── 5. Mesh and export ──────────────────────────────────────────────
    // Convert implicit surfaces to triangle meshes at a given tolerance
    // (smaller = more triangles = smoother surface).
    let tolerance = 0.5;

    let meshes = [
        ("sharp_union", sharp_union),
        ("sharp_subtract", sharp_subtract),
        ("sharp_intersect", sharp_intersect),
        ("smooth_blend", smooth),
        ("bracket", bracket),
    ];

    for (name, solid) in &meshes {
        let mesh = solid.mesh(tolerance);
        let path = format!("{name}.stl");
        mesh_io::save_stl(&mesh, &path, true)?;
        println!(
            "  {name}.stl — {} vertices, {} faces",
            mesh.vertices.len(),
            mesh.faces.len()
        );
    }

    println!("\nAll shapes exported. Open in any 3D viewer (e.g. PrusaSlicer, MeshLab).");
    Ok(())
}
