//! Hello Solid — The "Hello World" of Code-First Design
//!
//! Create shapes using implicit surfaces, combine them with boolean operations,
//! and view the results in 3D. This is the fundamental paradigm: geometry as code.
//!
//! Run with: `cargo run -p example-hello-solid`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
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
                title: "CortenForge — Hello Solid".into(),
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
    let tolerance = 0.3;

    // ── Primitives ────────────────────────────────────────────────────
    let sphere = Solid::sphere(10.0);
    let cube = Solid::cuboid(Vector3::new(8.0, 8.0, 8.0));

    // ── Booleans ──────────────────────────────────────────────────────
    let solids: Vec<(&str, Solid, Color)> = vec![
        (
            "Union",
            sphere.clone().union(cube.clone()),
            Color::srgb(0.9, 0.4, 0.3),
        ),
        (
            "Subtract",
            sphere.clone().subtract(cube.clone()),
            Color::srgb(0.3, 0.7, 0.4),
        ),
        (
            "Intersect",
            sphere.clone().intersect(cube.clone()),
            Color::srgb(0.3, 0.5, 0.9),
        ),
        (
            "Smooth blend",
            sphere.smooth_union(cube, 3.0),
            Color::srgb(0.9, 0.7, 0.2),
        ),
        (
            "Bracket",
            {
                let body = Solid::cuboid(Vector3::new(20.0, 10.0, 5.0)).round(1.5);
                let hole = Solid::cylinder(4.0, 6.0).translate(Vector3::new(10.0, 0.0, 0.0));
                body.subtract(hole)
            },
            Color::srgb(0.7, 0.5, 0.8),
        ),
    ];

    // ── Spawn meshes side-by-side ─────────────────────────────────────
    let spacing = 35.0;
    let offset = (solids.len() as f32 - 1.0) * spacing / 2.0;

    for (i, (name, solid, color)) in solids.into_iter().enumerate() {
        let indexed = Arc::new(solid.mesh(tolerance));
        let bevy_mesh = triangle_mesh_from_indexed(&indexed);

        println!(
            "  {name}: {} vertices, {} faces",
            indexed.vertices.len(),
            indexed.faces.len()
        );

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
    orbit.distance = 110.0;
    let mut cam_transform = Transform::default();
    orbit.apply_to_transform(&mut cam_transform);
    commands.spawn((Camera3d::default(), orbit, cam_transform));

    // ── Lighting ──────────────────────────────────────────────────────
    // Key light — from front-right, above
    commands.spawn((
        DirectionalLight {
            illuminance: 12000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(40.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Fill light — softer, from the left to reduce harsh shadows
    commands.spawn((
        DirectionalLight {
            illuminance: 5000.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-30.0, 30.0, 40.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Ground plane
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(80.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.4, 0.4, 0.4, 0.2),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -15.0, 0.0),
    ));

    println!("\n  Orbit: left-drag | Pan: right-drag | Zoom: scroll");
}
