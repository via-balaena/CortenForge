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
use nalgebra::Vector3;
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::mesh::triangle_mesh_from_indexed;

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
fn run_pipeline() -> (IndexedMesh, IndexedMesh, IndexedMesh, f32) {
    // ── Stage 1: Design a solid ──────────────────────────────────────
    println!("Stage 1: Design solid");
    let solid = Solid::cuboid(Vector3::new(30.0, 20.0, 15.0))
        .round(3.0)
        .subtract(Solid::cylinder(4.0, 20.0).translate(Vector3::new(10.0, 0.0, 0.0)));

    let mut design_mesh = solid.mesh(0.5);
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

    let spacing = dims.width as f32 + 10.0;
    (design_mesh, shell_result.mesh, lattice.mesh, spacing)
}

/// Spawn a mesh entity at a given X offset with the given color.
fn spawn_stage(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    mesh_data: &IndexedMesh,
    color: Color,
    x_offset: f32,
    label: &str,
) {
    let indexed = Arc::new(mesh_data.clone());
    let bevy_mesh = triangle_mesh_from_indexed(&indexed);

    println!(
        "  {label}: {} vertices, {} faces",
        indexed.vertices.len(),
        indexed.faces.len()
    );

    commands.spawn((
        Mesh3d(meshes.add(bevy_mesh)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: color,
            metallic: 0.3,
            perceptual_roughness: 0.5,
            double_sided: true,
            cull_mode: None,
            ..default()
        })),
        Transform::from_xyz(x_offset, 0.0, 0.0),
    ));
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
        -spacing,
        "Design solid",
    );

    spawn_stage(
        &mut commands,
        &mut meshes,
        &mut materials,
        &shell_mesh,
        Color::srgb(0.9, 0.6, 0.3),
        0.0,
        "Shell",
    );

    spawn_stage(
        &mut commands,
        &mut meshes,
        &mut materials,
        &lattice_mesh,
        Color::srgb(0.3, 0.8, 0.4),
        spacing,
        "Lattice infill",
    );

    // ── Camera ───────────────────────────────────────────────────────
    let mut orbit = OrbitCamera::new()
        .with_target(Vec3::ZERO)
        .with_angles(0.5, 0.5);
    orbit.max_distance = 500.0;
    orbit.min_distance = 1.0;
    orbit.orbit_speed = 0.008;
    orbit.pan_speed = 0.015;
    orbit.zoom_speed = 0.15;
    orbit.distance = 150.0;
    let mut cam_transform = Transform::default();
    orbit.apply_to_transform(&mut cam_transform);
    commands.spawn((Camera3d::default(), orbit, cam_transform));

    // ── Lighting ─────────────────────────────────────────────────────
    commands.spawn((
        DirectionalLight {
            illuminance: 12000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(30.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 4000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-20.0, 30.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Ground
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(100.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.5, 0.5, 0.5, 0.3),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -25.0, 0.0),
    ));

    println!("\n  Orbit: left-drag | Pan: right-drag | Zoom: scroll");
}
