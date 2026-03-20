//! Design-to-Sim — Full Pipeline Capstone
//!
//! Design a simple gripper in cf-design, export to MJCF, load it into
//! sim-core, actuate it, and visualize with Bevy. This is the CortenForge
//! value proposition in one example: design in code, simulate immediately.
//!
//! Run with: `cargo run -p example-design-to-sim --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use
)]

use std::sync::Arc;

use bevy::prelude::*;
use cf_design::{
    ActuatorDef, ActuatorKind, JointDef, JointKind, Material, Mechanism, Part, Solid, TendonDef,
    TendonWaypoint,
};
use nalgebra::{Point3, Vector3};
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::convert::{quat_from_unit_quaternion, vec3_from_vector};
use sim_bevy::mesh::triangle_mesh_from_indexed;
use sim_bevy::model_data::{ModelBodyIndex, PhysicsData, PhysicsModel, step_model_data};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Design to Sim".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, (step_model_data, actuate_gripper))
        .add_systems(PostUpdate, sync_bodies)
        .run();
}

/// Build the gripper mechanism using cf-design.
fn build_gripper() -> Mechanism {
    let material = Material::new("PLA", 1250.0)
        .with_youngs_modulus(3.5e9)
        .with_color([0.8, 0.85, 0.9, 1.0]);

    let palm = Part::new(
        "palm",
        Solid::cuboid(Vector3::new(12.0, 8.0, 4.0)).round(1.0),
        material.clone(),
    );

    let left_finger = Part::new("finger_l", Solid::capsule(2.5, 8.0), material.clone());
    let right_finger = Part::new("finger_r", Solid::capsule(2.5, 8.0), material);

    Mechanism::builder("gripper")
        .part(palm)
        .part(left_finger)
        .part(right_finger)
        .joint(
            JointDef::new(
                "hinge_l",
                "palm",
                "finger_l",
                JointKind::Revolute,
                Point3::new(-10.0, 8.0, 0.0),
                Vector3::x(),
            )
            .with_range(-0.2, 1.2),
        )
        .joint(
            JointDef::new(
                "hinge_r",
                "palm",
                "finger_r",
                JointKind::Revolute,
                Point3::new(10.0, 8.0, 0.0),
                Vector3::x(),
            )
            .with_range(-0.2, 1.2),
        )
        .tendon(
            TendonDef::new(
                "flexor_l",
                vec![
                    TendonWaypoint::new("palm", Point3::new(-8.0, -5.0, 0.0)),
                    TendonWaypoint::new("palm", Point3::new(-8.0, 6.0, 0.0)),
                    TendonWaypoint::new("finger_l", Point3::new(0.0, 5.0, 0.0)),
                ],
                0.8,
            )
            .with_stiffness(150.0)
            .with_damping(8.0),
        )
        .tendon(
            TendonDef::new(
                "flexor_r",
                vec![
                    TendonWaypoint::new("palm", Point3::new(8.0, -5.0, 0.0)),
                    TendonWaypoint::new("palm", Point3::new(8.0, 6.0, 0.0)),
                    TendonWaypoint::new("finger_r", Point3::new(0.0, 5.0, 0.0)),
                ],
                0.8,
            )
            .with_stiffness(150.0)
            .with_damping(8.0),
        )
        .actuator(
            ActuatorDef::new("motor_l", "flexor_l", ActuatorKind::Motor, (-30.0, 30.0))
                .with_ctrl_range(-1.0, 1.0),
        )
        .actuator(
            ActuatorDef::new("motor_r", "flexor_r", ActuatorKind::Motor, (-30.0, 30.0))
                .with_ctrl_range(-1.0, 1.0),
        )
        .build()
}

/// Oscillate gripper fingers open/closed using a sine wave.
fn actuate_gripper(mut data: ResMut<PhysicsData>) {
    let t = data.time;
    let signal = (t * 2.0).sin();

    if !data.ctrl.is_empty() {
        data.ctrl[0] = signal;
    }
    if data.ctrl.len() > 1 {
        data.ctrl[1] = signal;
    }
}

/// Sync body transforms from physics — using body xpos/xquat (world frame).
fn sync_bodies(data: Res<PhysicsData>, mut query: Query<(&ModelBodyIndex, &mut Transform)>) {
    for (body_idx, mut transform) in &mut query {
        let i = body_idx.0;
        if i < data.xpos.len() {
            transform.translation = vec3_from_vector(&data.xpos[i]);
        }
        if i < data.xquat.len() {
            transform.rotation = quat_from_unit_quaternion(&data.xquat[i]);
        }
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mechanism = build_gripper();

    println!("=== CortenForge: Design-to-Sim ===");
    println!(
        "  Mechanism '{}': {} parts, {} joints, {} tendons",
        mechanism.name(),
        mechanism.parts().len(),
        mechanism.joints().len(),
        mechanism.tendons().len(),
    );

    // ── Design meshes for visualization ──────────────────────────────
    let stl_kit = mechanism.to_stl_kit(0.3);

    // ── MJCF → physics engine ────────────────────────────────────────
    let mjcf_xml = mechanism.to_mjcf(1.5);
    let model = sim_mjcf::load_model(&mjcf_xml).expect("generated MJCF should load");

    println!(
        "  Physics model: {} bodies, {} joints, {} DOFs",
        model.nbody, model.njnt, model.nv
    );
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    let mut data = model.make_data();
    let _ = data.forward(&model);

    // ── Spawn design meshes, tagged with body indices ────────────────
    // Parts map 1:1 to bodies (body 0 is world, body 1+ are parts)
    let colors = [
        Color::srgb(0.7, 0.75, 0.8), // palm
        Color::srgb(0.9, 0.4, 0.3),  // finger_l
        Color::srgb(0.3, 0.6, 0.9),  // finger_r
    ];

    for (i, (name, mesh_data)) in stl_kit.iter().enumerate() {
        let indexed = Arc::new(mesh_data.clone());
        let bevy_mesh = triangle_mesh_from_indexed(&indexed);
        let body_id = i + 1; // skip world body

        println!(
            "  {name} (body {body_id}): {} vertices, {} faces",
            indexed.vertices.len(),
            indexed.faces.len()
        );

        // Initial transform from physics
        let initial_transform = if body_id < data.xpos.len() {
            Transform {
                translation: vec3_from_vector(&data.xpos[body_id]),
                rotation: quat_from_unit_quaternion(&data.xquat[body_id]),
                scale: Vec3::ONE,
            }
        } else {
            Transform::IDENTITY
        };

        commands.spawn((
            ModelBodyIndex(body_id),
            Mesh3d(meshes.add(bevy_mesh)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: colors[i % colors.len()],
                metallic: 0.4,
                perceptual_roughness: 0.4,
                ..default()
            })),
            initial_transform,
        ));
    }

    // ── Camera ────────────────────────────────────────────────────────
    let mut orbit = OrbitCamera::new()
        .with_target(Vec3::ZERO)
        .with_angles(0.5, 0.5);
    orbit.max_distance = 500.0;
    orbit.min_distance = 0.5;
    orbit.orbit_speed = 0.008;
    orbit.pan_speed = 0.015;
    orbit.zoom_speed = 0.15;
    orbit.distance = 80.0;
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
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(50.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.5, 0.5, 0.5, 0.3),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -15.0, 0.0),
    ));

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}
