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

use std::sync::Arc;

use bevy::prelude::*;
use cf_design::Solid;
use nalgebra::Vector3;
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::mesh::triangle_mesh_from_indexed;

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
    let stages: Vec<(&str, Arc<cf_geometry::IndexedMesh>, Color)> = vec![
        ("Original", Arc::new(mesh), Color::srgb(0.7, 0.75, 0.8)),
        ("Shell", Arc::new(shell_mesh), Color::srgb(0.3, 0.7, 0.5)),
        (
            "Lattice",
            Arc::new(lattice.mesh),
            Color::srgb(0.9, 0.6, 0.3),
        ),
    ];

    let spacing = 50.0;
    let offset = (stages.len() as f32 - 1.0) * spacing / 2.0;

    for (i, (name, indexed, color)) in stages.into_iter().enumerate() {
        let bevy_mesh = triangle_mesh_from_indexed(&indexed);
        println!("  Spawned: {name}");

        commands.spawn((
            Mesh3d(meshes.add(bevy_mesh)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                metallic: 0.3,
                perceptual_roughness: 0.5,
                ..default()
            })),
            Transform::from_xyz((i as f32).mul_add(spacing, -offset), 0.0, 0.0),
        ));
    }

    // ── Camera ────────────────────────────────────────────────────────
    let mut orbit = OrbitCamera::new()
        .with_target(Vec3::ZERO)
        .with_angles(0.5, 0.6);
    orbit.max_distance = 500.0;
    orbit.min_distance = 5.0;
    orbit.orbit_speed = 0.008;
    orbit.pan_speed = 0.015;
    orbit.zoom_speed = 0.15;
    orbit.distance = 100.0;
    let mut cam_transform = Transform::default();
    orbit.apply_to_transform(&mut cam_transform);
    commands.spawn((Camera3d::default(), orbit, cam_transform));

    // ── Lighting ──────────────────────────────────────────────────────
    commands.spawn((
        DirectionalLight {
            illuminance: 12000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(40.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 5000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-30.0, 30.0, 40.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(80.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.4, 0.4, 0.4, 0.2),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -25.0, 0.0),
    ));

    println!("\n  Orbit: left-drag | Pan: right-drag | Zoom: scroll");
}
