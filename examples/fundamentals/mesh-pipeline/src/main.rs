//! Mesh Pipeline — Scan-to-Print Manufacturing
//!
//! Generate a mesh from cf-design, then run it through CortenForge's mesh
//! processing pipeline: validate, repair, measure, shell (hollow), add lattice
//! infill, analyze printability, and view the stages side-by-side.
//!
//! Run with: `cargo run -p example-mesh-pipeline`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use bevy::prelude::*;
use cf_design::Solid;
use nalgebra::{Point3, Vector3};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::mesh::spawn_design_mesh;
use sim_bevy::scene::ExampleScene;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Mesh Pipeline".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // ── 1. Generate source mesh ───────────────────────────────────────
    let shape = Solid::cuboid(Vector3::new(15.0, 10.0, 20.0)).round(2.0);
    let mut mesh = shape.mesh(0.3);

    println!("=== CortenForge: Mesh Pipeline ===\n");
    println!(
        "1. Source: {} vertices, {} faces",
        mesh.vertices.len(),
        mesh.faces.len()
    );

    // ── 2. Validate ───────────────────────────────────────────────────
    let report = mesh_repair::validate_mesh(&mesh);
    println!("\n2. Validation:");
    println!("   Watertight: {}", report.is_watertight);
    println!("   Manifold:   {}", report.is_manifold);

    // ── 3. Repair (if needed) ─────────────────────────────────────────
    if report.has_issues() {
        let summary = mesh_repair::repair_mesh(&mut mesh, &mesh_repair::RepairParams::default());
        println!(
            "\n3. Repair: welded {} vertices, removed {} degenerates",
            summary.vertices_welded, summary.degenerates_removed
        );
    } else {
        println!("\n3. No repair needed.");
    }

    // ── 4. Measure ────────────────────────────────────────────────────
    let dims = mesh_measure::dimensions(&mesh);
    println!(
        "\n4. Dimensions: {:.1} x {:.1} x {:.1} mm",
        dims.width, dims.depth, dims.height
    );

    // ── 5. Shell ──────────────────────────────────────────────────────
    let shell_result = mesh_shell::ShellBuilder::new(&mesh)
        .wall_thickness(2.0)
        .fast()
        .build()
        .expect("shell should succeed");
    let shell_mesh = shell_result.mesh;
    println!(
        "\n5. Shell: {} vertices, {} faces (wall=2mm)",
        shell_mesh.vertices.len(),
        shell_mesh.faces.len()
    );

    // ── 6. Lattice infill ─────────────────────────────────────────────
    let lattice_bounds = (dims.min, dims.max);
    let lattice = mesh_lattice::generate_lattice(
        &mesh_lattice::LatticeParams::gyroid(8.0).with_density(0.2),
        lattice_bounds,
    )
    .expect("lattice should succeed");
    println!(
        "\n6. Lattice: {} vertices, {} faces (gyroid, 20%)",
        lattice.vertex_count(),
        lattice.mesh.face_count()
    );

    // ── 7. Printability ───────────────────────────────────────────────
    let config = mesh_printability::PrinterConfig::fdm_default();
    let validation = mesh_printability::validate_for_printing(&shell_mesh, &config)
        .expect("printability check should succeed");
    println!("\n7. Printability (FDM): {}", validation.summary());

    // ── Spawn stages side-by-side ─────────────────────────────────────
    let stages: Vec<(&str, cf_geometry::IndexedMesh, Color)> = vec![
        ("Original", mesh, Color::srgb(0.7, 0.75, 0.8)),
        ("Shell", shell_mesh, Color::srgb(0.3, 0.7, 0.5)),
        ("Lattice", lattice.mesh, Color::srgb(0.9, 0.6, 0.3)),
    ];

    let spacing = 50.0;
    let offset = (stages.len() as f32 - 1.0) * spacing / 2.0;

    for (i, (name, mesh_data, color)) in stages.iter().enumerate() {
        println!("  Spawned: {name}");
        let x = f64::from((i as f32).mul_add(spacing, -offset));
        spawn_design_mesh(
            &mut commands,
            &mut meshes,
            &mut materials,
            mesh_data,
            Point3::new(x, 0.0, 0.0),
            *color,
        );
    }

    ExampleScene::new(100.0, 80.0)
        .with_angles(0.5, 0.6)
        .with_ground_y(-25.0)
        .spawn(&mut commands, &mut meshes, &mut materials);

    println!("\n  Orbit: left-drag | Pan: right-drag | Zoom: scroll");
}
