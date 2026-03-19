//! Bio-Inspired Design Gallery
//!
//! Showcase CortenForge's bio-inspired primitives: superellipsoids, log spirals,
//! gyroids, helices, and organic blends. These shapes are trivial in code but
//! nearly impossible in traditional CAD.
//!
//! Run with: `cargo run -p example-bio-shapes`

use cf_design::{InfillKind, Solid};
use nalgebra::Vector3;

fn main() -> anyhow::Result<()> {
    println!("=== CortenForge: Bio-Inspired Shapes ===\n");

    let tolerance = 0.5;

    // ── 1. Superellipsoid ───────────────────────────────────────────────
    // Tunable primitive: n1=n2=1 is a sphere, n1=n2=0.5 is a diamond,
    // large n values approach a box. Nature uses these for seeds, shells.
    let superellipsoid = Solid::superellipsoid(Vector3::new(8.0, 6.0, 10.0), 0.8, 0.8);

    // ── 2. Logarithmic spiral ───────────────────────────────────────────
    // Found in nautilus shells, ram horns, and plant tendrils.
    // Parameters: initial radius, growth rate, tube thickness, turns.
    let spiral = Solid::log_spiral(2.0, 0.15, 1.5, 3.0);

    // ── 3. Helix ────────────────────────────────────────────────────────
    // Springs, DNA, vine tendrils. Parameters: radius, pitch, thickness, turns.
    let helix = Solid::helix(8.0, 6.0, 1.5, 4.0);

    // ── 4. Gyroid-infilled bone ─────────────────────────────────────────
    // Triply-periodic minimal surface inside a structural shape.
    // Nature uses TPMS for butterfly wings, sea urchin skeletons, bone trabecular.
    let bone = Solid::capsule(6.0, 15.0).infill(InfillKind::Gyroid, 1.5, 0.8, 1.0);

    // ── 5. Organic composition ──────────────────────────────────────────
    // Combine bio primitives with smooth booleans to create something
    // that looks grown, not manufactured.
    let body = Solid::superellipsoid(Vector3::new(12.0, 8.0, 5.0), 0.7, 0.7);
    let arm_l = Solid::capsule(3.0, 10.0).translate(Vector3::new(-12.0, 0.0, 2.0));
    let arm_r = Solid::capsule(3.0, 10.0).translate(Vector3::new(12.0, 0.0, 2.0));
    let creature = Solid::smooth_union_all(vec![body, arm_l, arm_r], 4.0);

    // ── Export ───────────────────────────────────────────────────────────
    let shapes = [
        ("superellipsoid", superellipsoid),
        ("log_spiral", spiral),
        ("helix", helix),
        ("gyroid_bone", bone),
        ("creature", creature),
    ];

    for (name, solid) in &shapes {
        let mesh = solid.mesh(tolerance);
        let path = format!("{name}.stl");
        mesh_io::save_stl(&mesh, &path, true)?;
        println!(
            "  {name}.stl — {} vertices, {} faces",
            mesh.vertices.len(),
            mesh.faces.len()
        );
    }

    println!("\nBio-inspired gallery exported.");
    Ok(())
}
