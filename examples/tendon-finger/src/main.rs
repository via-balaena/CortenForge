//! Tendon-Driven Finger — Mechanism Assembly + Visualization
//!
//! Build a 3-phalanx finger using cf-design's template system, route a flexor
//! tendon through it, attach a motor actuator, and view the assembled mechanism
//! in Bevy.
//!
//! This demonstrates the "design IS the simulation model" philosophy:
//! tendon channels are automatically subtracted from the geometry at build time.
//!
//! Run with: `cargo run -p example-tendon-finger`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation
)]

use std::sync::Arc;

use bevy::prelude::*;
use cf_design::{
    ActuatorDef, ActuatorKind, JointDef, JointKind, Material, Mechanism, TendonDef, TendonWaypoint,
    templates,
};
use nalgebra::{Point3, Vector3};
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::mesh::triangle_mesh_from_indexed;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Tendon-Driven Finger".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .add_systems(Startup, setup)
        .run();
}

fn build_finger() -> Mechanism {
    let pla = Material::new("PLA", 1250.0)
        .with_youngs_modulus(3.5e9)
        .with_color([0.9, 0.85, 0.7, 1.0]);

    let (finger_parts, finger_joints) = templates::finger("finger", 30.0, 3.5, 3, pla.clone());

    println!("Finger template:");
    println!("  {} phalanges", finger_parts.len());
    println!("  {} flex joints", finger_joints.len());
    for j in &finger_joints {
        println!(
            "    {} ({} -> {}), stiffness={:.0}",
            j.name(),
            j.parent(),
            j.child(),
            j.stiffness().unwrap_or(0.0),
        );
    }

    // Palm (mount point)
    let palm = cf_design::Part::new(
        "palm",
        cf_design::Solid::cuboid(Vector3::new(8.0, 12.0, 4.0)).round(1.0),
        pla,
    );

    let mut builder = Mechanism::builder("tendon_finger").part(palm);

    for part in finger_parts {
        builder = builder.part(part);
    }

    // Connect palm to proximal phalanx
    builder = builder.joint(
        JointDef::new(
            "knuckle",
            "palm",
            "finger_0",
            JointKind::Revolute,
            Point3::new(0.0, 12.0, 0.0),
            Vector3::x(),
        )
        .with_range(-0.1, 1.8),
    );

    for j in finger_joints {
        builder = builder.joint(j);
    }

    // Flexor tendon
    builder = builder.tendon(
        TendonDef::new(
            "flexor",
            vec![
                TendonWaypoint::new("palm", Point3::new(0.0, -8.0, 0.0)),
                TendonWaypoint::new("palm", Point3::new(0.0, 10.0, 0.0)),
                TendonWaypoint::new("finger_0", Point3::new(0.0, 0.0, 2.0)),
                TendonWaypoint::new("finger_1", Point3::new(0.0, 0.0, 2.0)),
                TendonWaypoint::new("finger_2", Point3::new(0.0, 0.0, 1.0)),
            ],
            1.0,
        )
        .with_stiffness(200.0)
        .with_damping(10.0),
    );

    builder = builder.actuator(
        ActuatorDef::new("motor", "flexor", ActuatorKind::Motor, (-50.0, 50.0))
            .with_ctrl_range(-1.0, 1.0),
    );

    builder.build()
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mechanism = build_finger();

    println!("\nMechanism '{}' built successfully.", mechanism.name());
    println!("  {} parts", mechanism.parts().len());
    println!("  {} joints", mechanism.joints().len());
    println!("  {} tendons", mechanism.tendons().len());
    println!("  {} actuators", mechanism.actuators().len());

    // Mesh and display each part
    let stl_kit = mechanism.to_stl_kit(0.5);

    let colors = [
        Color::srgb(0.7, 0.75, 0.8), // palm
        Color::srgb(0.9, 0.5, 0.3),  // finger_0
        Color::srgb(0.3, 0.7, 0.5),  // finger_1
        Color::srgb(0.3, 0.5, 0.9),  // finger_2
    ];

    for (i, (name, mesh_data)) in stl_kit.iter().enumerate() {
        let indexed = Arc::new(mesh_data.clone());
        let bevy_mesh = triangle_mesh_from_indexed(&indexed);

        println!(
            "  {name}: {} vertices, {} faces",
            indexed.vertices.len(),
            indexed.faces.len()
        );

        let color = colors[i % colors.len()];
        commands.spawn((
            Mesh3d(meshes.add(bevy_mesh)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                metallic: 0.4,
                perceptual_roughness: 0.4,
                ..default()
            })),
            Transform::IDENTITY,
        ));
    }

    // ── Camera ────────────────────────────────────────────────────────
    let orbit = OrbitCamera::new()
        .with_target(Vec3::new(0.0, 15.0, 0.0))
        .with_distance(60.0)
        .with_angles(0.5, 0.3);
    let mut cam_transform = Transform::default();
    orbit.apply_to_transform(&mut cam_transform);
    commands.spawn((Camera3d::default(), orbit, cam_transform));

    // ── Lighting ──────────────────────────────────────────────────────
    commands.spawn((
        DirectionalLight {
            illuminance: 15000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(20.0, 40.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(50.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.4, 0.4, 0.4, 0.2),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -5.0, 0.0),
    ));

    println!("\n  Orbit: left-drag | Pan: right-drag | Zoom: scroll");
}
