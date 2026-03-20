//! Finger Design — Boolean Heuristic Baseline
//!
//! Two-segment articulated finger built from composed SDF primitives.
//! This is the baseline to beat with a custom SDF.
//!
//! Run with: `cargo run -p example-finger-design --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation
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
                title: "CortenForge — Finger Design".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .add_systems(Startup, setup)
        .run();
}

// ============================================================================
// Finger geometry — boolean heuristic v2
// ============================================================================

/// Proximal phalange — base segment.
///
/// Wider-than-deep ellipsoidal cross-section (like a real finger bone).
/// Knuckle condyle protruding at the distal end.
/// Z-axis is the finger length (up in Bevy after coordinate conversion).
fn proximal_phalange() -> Solid {
    // Ellipsoid body: wider (X=5) than deep (Y=3.5), elongated along Z (=14)
    let body = Solid::ellipsoid(Vector3::new(5.0, 3.5, 14.0)).round(0.5);

    // Knuckle condyle — wide bulge at the tip (+Z end)
    let knuckle =
        Solid::ellipsoid(Vector3::new(5.5, 4.0, 4.0)).translate(Vector3::new(0.0, 0.0, 13.0));

    body.smooth_union(knuckle, 2.5)
}

/// Knuckle joint — visible hinge pin connecting the two phalanges.
///
/// Cylinder pin along X (the hinge axis) with rounded ends.
fn knuckle_joint() -> Solid {
    // Pin through the knuckle — extends wider than the finger
    Solid::capsule(1.5, 5.0).rotate(nalgebra::UnitQuaternion::from_axis_angle(
        &nalgebra::Vector3::z_axis(),
        std::f64::consts::FRAC_PI_2,
    ))
}

/// Distal phalange — fingertip segment.
///
/// Tapers from a wide base to a narrow, rounded fingertip.
/// Z-axis is the finger length.
fn distal_phalange() -> Solid {
    // Body: slightly narrower, shorter
    let body = Solid::ellipsoid(Vector3::new(4.5, 3.0, 11.0)).round(0.5);

    // Fingertip taper
    let tip = Solid::ellipsoid(Vector3::new(3.0, 2.5, 4.0)).translate(Vector3::new(0.0, 0.0, 10.0));

    body.smooth_union(tip, 3.0)
}

// ============================================================================
// Bevy visualization
// ============================================================================

fn spawn_solid(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    solid: &Solid,
    color: Color,
    position: Point3<f64>,
    label: &str,
) {
    let mesh_data = solid.mesh(0.3);
    println!(
        "  {label}: {} verts, {} faces",
        mesh_data.vertex_count(),
        mesh_data.face_count()
    );
    spawn_design_mesh(commands, meshes, materials, &mesh_data, position, color);
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    println!("=== Finger Design v2 (Boolean Heuristic) ===\n");

    let proximal = proximal_phalange();
    let distal = distal_phalange();
    let joint = knuckle_joint();

    let spacing = 25.0;
    let bone_color = Color::srgb(0.9, 0.85, 0.75);
    let tip_color = Color::srgb(0.8, 0.75, 0.68);
    let pin_color = Color::srgb(0.5, 0.5, 0.55);

    // Left: all 3 parts separated (positions in physics Z-up space)
    spawn_solid(
        &mut commands,
        &mut meshes,
        &mut materials,
        &proximal,
        bone_color,
        Point3::new(-spacing, 0.0, 0.0),
        "Proximal",
    );
    spawn_solid(
        &mut commands,
        &mut meshes,
        &mut materials,
        &joint,
        pin_color,
        Point3::new(-spacing, 0.0, 20.0),
        "Joint pin",
    );
    spawn_solid(
        &mut commands,
        &mut meshes,
        &mut materials,
        &distal,
        tip_color,
        Point3::new(-spacing, 0.0, 35.0),
        "Distal",
    );

    // Right: assembled — knuckle at Z≈13
    // Small gap between them for the joint space
    let joint_z = 13.0 + 11.0 + 1.0; // knuckle + distal base + gap
    let knuckle_z = 13.0; // where the knuckle center is
    spawn_solid(
        &mut commands,
        &mut meshes,
        &mut materials,
        &proximal,
        bone_color,
        Point3::new(spacing, 0.0, 0.0),
        "Proximal (asm)",
    );
    spawn_solid(
        &mut commands,
        &mut meshes,
        &mut materials,
        &joint,
        pin_color,
        Point3::new(spacing, 0.0, knuckle_z),
        "Joint (asm)",
    );
    spawn_solid(
        &mut commands,
        &mut meshes,
        &mut materials,
        &distal,
        tip_color,
        Point3::new(spacing, 0.0, joint_z),
        "Distal (asm)",
    );

    spawn_scene(&mut commands, &mut meshes, &mut materials);
    println!("\n  Left: separated | Right: assembled");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll");
}

fn spawn_scene(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    ExampleScene::new(80.0, 60.0)
        .with_target(Vec3::new(0.0, 12.0, 0.0))
        .with_angles(0.3, 0.4)
        .with_ground_y(-20.0)
        .spawn(commands, meshes, materials);
}
