//! Design-to-Print — Shape-Conforming Lattice Infill
//!
//! Demonstrates the design-to-print pipeline:
//! 1. Design a solid part (rounded bracket with cutout)
//! 2. Generate a shell
//! 3. Fill with shape-conforming gyroid lattice
//! 4. Validate printability
//! 5. Visualize all three stages side-by-side in Bevy
//!
//! Run with: `cargo run -p example-design-to-print --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use
)]

use std::sync::Arc;

use bevy::prelude::*;
use cf_design::Solid;
use cf_geometry::IndexedMesh;
use mesh_lattice::{LatticeParams, generate_lattice};
use mesh_measure::dimensions;
use mesh_printability::{PrinterConfig, validate_for_printing};
use mesh_repair::{RepairParams, repair_mesh};
use mesh_shell::ShellBuilder;
use nalgebra::{Point3, Vector3};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::mesh::spawn_design_mesh;
use sim_bevy::scene::ExampleScene;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Design to Print".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .add_systems(Startup, setup)
        .run();
}

/// Run the design → shell → lattice → printability pipeline.
///
/// Returns `(design_mesh, shell_mesh, lattice_mesh, spacing)` for visualization.
fn run_pipeline() -> (IndexedMesh, IndexedMesh, IndexedMesh, f64) {
    // ── Stage 1: Design a solid ──────────────────────────────────────
    println!("Stage 1: Design solid");
    let solid = Solid::cuboid(Vector3::new(30.0, 20.0, 15.0))
        .round(3.0)
        .subtract(Solid::cylinder(4.0, 20.0).translate(Vector3::new(10.0, 0.0, 0.0)));

    // Strip the AttributedMesh wrapper at the cf-design boundary — repair /
    // dimensions / shell / lattice / printability all consume IndexedMesh.
    let mut design_mesh = solid.mesh(0.5).geometry;
    let repair_summary = repair_mesh(&mut design_mesh, &RepairParams::default());
    if repair_summary.had_changes() {
        println!("  Repaired: {repair_summary:?}");
    }

    let dims = dimensions(&design_mesh);
    println!(
        "  Dimensions: {:.1} x {:.1} x {:.1} mm",
        dims.width, dims.depth, dims.height
    );

    // ── Stage 2: Shell ───────────────────────────────────────────────
    println!("\nStage 2: Shell");
    let shell_result = ShellBuilder::new(&design_mesh)
        .wall_thickness(1.2)
        .fast()
        .build()
        .expect("shell generation");

    // ── Stage 3: Shape-conforming lattice infill ─────────────────────
    println!("\nStage 3: Lattice infill (gyroid, shape-conforming)");
    let sdf = Arc::new(move |p: nalgebra::Point3<f64>| solid.evaluate(&p));
    let lattice_params = LatticeParams::gyroid(6.0)
        .with_density(0.25)
        .with_shape_sdf(sdf);
    let bounds = (dims.min, dims.max);
    let lattice = generate_lattice(&lattice_params, bounds).expect("lattice generation");

    println!(
        "  Lattice: {} vertices, {} faces, density={:.2}",
        lattice.vertex_count(),
        lattice.triangle_count(),
        lattice.actual_density
    );

    // ── Printability check ───────────────────────────────────────────
    println!("\nPrintability check:");
    let report = validate_for_printing(&lattice.mesh, &PrinterConfig::fdm_default())
        .expect("printability validation");
    println!(
        "  Printable: {} — {}",
        report.is_printable(),
        report.summary()
    );

    let spacing = dims.width + 10.0;
    (design_mesh, shell_result.mesh, lattice.mesh, spacing)
}

/// Spawn a mesh entity at a physics-space position with the given color.
fn spawn_stage(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    mesh_data: &IndexedMesh,
    color: Color,
    position: Point3<f64>,
    label: &str,
) {
    println!(
        "  {label}: {} vertices, {} faces",
        mesh_data.vertices.len(),
        mesh_data.faces.len()
    );
    let attributed = cf_design::AttributedMesh::from(mesh_data.clone());
    spawn_design_mesh(commands, meshes, materials, &attributed, position, color);
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    println!("=== CortenForge: Design-to-Print ===\n");

    let (design_mesh, shell_mesh, lattice_mesh, spacing) = run_pipeline();

    // ── Bevy visualization: 3 stages side-by-side ────────────────────
    println!("\nSpawning visualization:");

    spawn_stage(
        &mut commands,
        &mut meshes,
        &mut materials,
        &design_mesh,
        Color::srgb(0.7, 0.75, 0.85),
        Point3::new(-spacing, 0.0, 0.0),
        "Design solid",
    );

    spawn_stage(
        &mut commands,
        &mut meshes,
        &mut materials,
        &shell_mesh,
        Color::srgb(0.9, 0.6, 0.3),
        Point3::origin(),
        "Shell",
    );

    spawn_stage(
        &mut commands,
        &mut meshes,
        &mut materials,
        &lattice_mesh,
        Color::srgb(0.3, 0.8, 0.4),
        Point3::new(spacing, 0.0, 0.0),
        "Lattice infill",
    );

    ExampleScene::new(150.0, 100.0).with_ground_y(-25.0).spawn(
        &mut commands,
        &mut meshes,
        &mut materials,
    );

    println!("\n  Orbit: left-drag | Pan: right-drag | Zoom: scroll");
}
