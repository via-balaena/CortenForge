//! Mesh Pipeline — Scan-to-Print Manufacturing
//!
//! Generate a mesh from cf-design, then run it through CortenForge's mesh
//! processing pipeline: validate, repair, measure, shell (hollow), add lattice
//! infill, analyze printability, and export. Shows the 27-crate mesh ecosystem.
//!
//! Run with: `cargo run -p example-mesh-pipeline`

use cf_design::Solid;
use nalgebra::Vector3;

fn main() -> anyhow::Result<()> {
    println!("=== CortenForge: Mesh Pipeline ===\n");

    // ── 1. Generate source mesh from implicit surface ───────────────────
    // Instead of loading a scan, we generate a test part using cf-design.
    // In production, you'd use mesh_io::load_mesh("scan.stl") here.
    let shape = Solid::superellipsoid(Vector3::new(15.0, 10.0, 20.0), 0.7, 0.7).round(0.5);
    let mut mesh = shape.mesh(0.8);

    println!(
        "1. Source mesh: {} vertices, {} faces",
        mesh.vertices.len(),
        mesh.faces.len()
    );

    // ── 2. Validate ─────────────────────────────────────────────────────
    let report = mesh_repair::validate_mesh(&mesh);
    println!("\n2. Validation:");
    println!("   Watertight:   {}", report.is_watertight);
    println!("   Manifold:     {}", report.is_manifold);
    println!("   Boundary edges: {}", report.boundary_edge_count);
    println!("   Degenerate faces: {}", report.degenerate_face_count);

    // ── 3. Repair (if needed) ───────────────────────────────────────────
    if report.has_issues() {
        let summary = mesh_repair::repair_mesh(&mut mesh, &mesh_repair::RepairParams::default());
        println!("\n3. Repair applied:");
        println!("   Vertices welded: {}", summary.vertices_welded);
        println!("   Degenerates removed: {}", summary.degenerates_removed);
        println!(
            "   Final: {} vertices, {} faces",
            summary.final_vertices, summary.final_faces
        );
    } else {
        println!("\n3. No repair needed.");
    }

    // ── 4. Measure ──────────────────────────────────────────────────────
    let dims = mesh_measure::dimensions(&mesh);
    println!("\n4. Dimensions:");
    println!("   Width:  {:.1} mm", dims.width);
    println!("   Depth:  {:.1} mm", dims.depth);
    println!("   Height: {:.1} mm", dims.height);
    println!("   Bounding volume: {:.0} mm^3", dims.bounding_volume);

    // ── 5. Shell (hollow out) ───────────────────────────────────────────
    let shell_result = mesh_shell::ShellBuilder::new(&mesh)
        .wall_thickness(2.0)
        .fast()
        .build()?;
    let shell_mesh = shell_result.mesh;
    println!("\n5. Shell:");
    println!("   Wall thickness: 2.0 mm");
    println!(
        "   Result: {} vertices, {} faces",
        shell_mesh.vertices.len(),
        shell_mesh.faces.len()
    );

    // ── 6. Lattice infill ───────────────────────────────────────────────
    let lattice_bounds = (dims.min, dims.max);
    let lattice = mesh_lattice::generate_lattice(
        &mesh_lattice::LatticeParams::gyroid(8.0).with_density(0.2),
        lattice_bounds,
    )?;
    println!("\n6. Lattice infill (gyroid):");
    println!("   Cell size: 8 mm, density: 20%");
    println!(
        "   Result: {} vertices, {} faces",
        lattice.vertex_count(),
        lattice.mesh.face_count()
    );

    // ── 7. Printability analysis ────────────────────────────────────────
    let config = mesh_printability::PrinterConfig::fdm_default();
    let validation = mesh_printability::validate_for_printing(&shell_mesh, &config)?;
    println!("\n7. Printability (FDM):");
    println!("   Printable: {}", validation.is_printable());
    println!("   Thin walls: {}", validation.thin_walls.len());
    println!("   Overhangs:  {}", validation.overhangs.len());
    println!("   {}", validation.summary());

    // ── 8. Export ────────────────────────────────────────────────────────
    mesh_io::save_stl(&mesh, "original.stl", true)?;
    mesh_io::save_stl(&shell_mesh, "shell.stl", true)?;
    mesh_io::save_stl(&lattice.mesh, "lattice.stl", true)?;

    println!("\n8. Exported:");
    println!("   original.stl — solid part");
    println!("   shell.stl    — hollow shell");
    println!("   lattice.stl  — gyroid infill");

    println!("\nPipeline complete. Open in PrusaSlicer to inspect.");
    Ok(())
}
