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
/// Capsule-like body with a spherical knuckle head protruding at the tip.
/// No aggressive boolean ops — just `smooth_union` for the knuckle blend.
fn proximal_phalange() -> Solid {
    // Capsule gives us the rounded ends for free
    let body = Solid::capsule(4.0, 12.0); // radius 4, half-height 12 → 24mm + caps

    // Knuckle head — sphere protruding at the tip
    let knuckle = Solid::sphere(4.5).translate(Vector3::new(0.0, 14.0, 0.0));

    // Smooth blend — high k for very organic transition
    body.smooth_union(knuckle, 3.0)
}

/// Distal phalange — fingertip segment.
///
/// Capsule body tapering to a rounded tip. Shallow concave socket at
/// the base to cup the proximal knuckle head.
fn distal_phalange() -> Solid {
    // Slightly smaller capsule
    let body = Solid::capsule(3.5, 10.0); // radius 3.5, half-height 10

    // Fingertip — elongated ellipsoid for taper
    let tip = Solid::ellipsoid(Vector3::new(3.0, 5.0, 3.0)).translate(Vector3::new(0.0, 12.0, 0.0));
    let shape = body.smooth_union(tip, 4.0);

    // Socket — sphere centered WELL BELOW the base so only a shallow
    // scoop is cut from the bottom face. The further below, the shallower.
    let socket = Solid::sphere(5.0).translate(Vector3::new(0.0, -13.0, 0.0));

    shape.subtract(socket)
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
    offset: Vec3,
    label: &str,
) {
    // Use mesh_adaptive for better quality at boolean boundaries
    let mesh_data = solid.mesh_adaptive(0.3);
    println!(
        "  {label}: {} verts, {} faces",
        mesh_data.vertex_count(),
        mesh_data.face_count()
    );
    let indexed = Arc::new(mesh_data);
    let bevy_mesh = triangle_mesh_from_indexed(&indexed);
    commands.spawn((
        Mesh3d(meshes.add(bevy_mesh)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: color,
            metallic: 0.15,
            perceptual_roughness: 0.7,
            double_sided: true,
            cull_mode: None,
            ..default()
        })),
        Transform::from_translation(offset),
    ));
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    println!("=== Finger Design v2 (Boolean Heuristic) ===\n");

    let proximal = proximal_phalange();
    let distal = distal_phalange();

    let spacing = 25.0;
    let bone_color = Color::srgb(0.9, 0.85, 0.75);
    let tip_color = Color::srgb(0.8, 0.75, 0.68);

    // Left: parts separated
    spawn_solid(
        &mut commands,
        &mut meshes,
        &mut materials,
        &proximal,
        bone_color,
        Vec3::new(-spacing, 0.0, 0.0),
        "Proximal",
    );
    spawn_solid(
        &mut commands,
        &mut meshes,
        &mut materials,
        &distal,
        tip_color,
        Vec3::new(-spacing, 40.0, 0.0),
        "Distal",
    );

    // Right: assembled — knuckle head at y≈14, socket at y≈-13
    // Position distal so socket cups the knuckle
    let joint_y = 14.0 + 12.0; // knuckle center + distal half-length offset
    spawn_solid(
        &mut commands,
        &mut meshes,
        &mut materials,
        &proximal,
        bone_color,
        Vec3::new(spacing, 0.0, 0.0),
        "Proximal (asm)",
    );
    spawn_solid(
        &mut commands,
        &mut meshes,
        &mut materials,
        &distal,
        tip_color,
        Vec3::new(spacing, joint_y, 0.0),
        "Distal (asm)",
    );

    // Camera
    let mut orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, 12.0, 0.0))
        .with_angles(0.3, 0.4);
    orbit.distance = 80.0;
    orbit.orbit_speed = 0.008;
    orbit.pan_speed = 0.015;
    orbit.zoom_speed = 0.15;
    let mut cam_transform = Transform::default();
    orbit.apply_to_transform(&mut cam_transform);
    commands.spawn((Camera3d::default(), orbit, cam_transform));

    commands.spawn((
        DirectionalLight {
            illuminance: 15000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(30.0, 50.0, 40.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 5000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-20.0, 30.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(60.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.4, 0.4, 0.4, 0.3),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -20.0, 0.0),
    ));

    println!("\n  Left: separated | Right: assembled");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll");
}
