//! Bio-Inspired Design Gallery
//!
//! Showcase CortenForge's bio-inspired primitives: superellipsoids, log spirals,
//! gyroids, helices, and organic blends. These shapes are trivial in code but
//! nearly impossible in traditional CAD.
//!
//! Run with: `cargo run -p example-bio-shapes`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss
)]

use bevy::prelude::*;
use cf_design::{InfillKind, Solid};
use nalgebra::{Point3, Vector3};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::mesh::spawn_design_mesh;
use sim_bevy::scene::ExampleScene;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Bio-Inspired Shapes".into(),
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

    let solids: Vec<(&str, Solid, Color)> = vec![
        (
            "Superellipsoid",
            Solid::superellipsoid(Vector3::new(8.0, 6.0, 10.0), 0.8, 0.8),
            Color::srgb(0.9, 0.6, 0.3),
        ),
        (
            "Log spiral",
            Solid::log_spiral(2.0, 0.15, 1.5, 3.0),
            Color::srgb(0.85, 0.8, 0.6),
        ),
        (
            "Helix",
            Solid::helix(8.0, 6.0, 1.5, 4.0),
            Color::srgb(0.4, 0.75, 0.5),
        ),
        (
            "Gyroid bone",
            Solid::capsule(6.0, 15.0).infill(InfillKind::Gyroid, 1.5, 0.8, 1.0),
            Color::srgb(0.9, 0.85, 0.75),
        ),
        (
            "Creature",
            {
                let body = Solid::superellipsoid(Vector3::new(12.0, 8.0, 5.0), 0.7, 0.7);
                let arm_l = Solid::capsule(3.0, 10.0).translate(Vector3::new(-12.0, 0.0, 2.0));
                let arm_r = Solid::capsule(3.0, 10.0).translate(Vector3::new(12.0, 0.0, 2.0));
                Solid::smooth_union_all(vec![body, arm_l, arm_r], 4.0)
            },
            Color::srgb(0.6, 0.5, 0.8),
        ),
    ];

    // ── Spawn meshes side-by-side ─────────────────────────────────────
    let spacing = 40.0;
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

    ExampleScene::new(130.0, 100.0)
        .with_angles(0.5, 0.6)
        .with_ground_y(-20.0)
        .spawn(&mut commands, &mut meshes, &mut materials);

    println!("\n  Orbit: left-drag | Pan: right-drag | Zoom: scroll");
}
