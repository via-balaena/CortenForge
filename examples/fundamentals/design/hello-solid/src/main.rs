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
        let mesh_data = solid.mesh(tolerance);
        println!(
            "  {name}: {} vertices, {} faces",
            mesh_data.geometry.vertices.len(),
            mesh_data.geometry.faces.len()
        );
        let x = f64::from((i as f32).mul_add(spacing, -offset));
        spawn_design_mesh(
            &mut commands,
            &mut meshes,
            &mut materials,
            &mesh_data,
            Point3::new(x, 0.0, 0.0),
            color,
        );
    }

    ExampleScene::new(110.0, 80.0).with_angles(0.5, 0.6).spawn(
        &mut commands,
        &mut meshes,
        &mut materials,
    );

    println!("\n  Orbit: left-drag | Pan: right-drag | Zoom: scroll");
}
