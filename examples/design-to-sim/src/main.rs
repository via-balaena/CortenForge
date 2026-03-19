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

use bevy::prelude::*;
use cf_design::{
    ActuatorDef, ActuatorKind, JointDef, JointKind, Material, Mechanism, Part, Solid, TendonDef,
    TendonWaypoint,
};
use nalgebra::{Point3, Vector3};
use sim_bevy::camera::{OrbitCamera, OrbitCameraPlugin};
use sim_bevy::convert::vec3_from_vector;
use sim_bevy::model_data::{ModelBodyIndex, PhysicsData, PhysicsModel, step_model_data};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
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

    // Palm: rounded box
    let palm = Part::new(
        "palm",
        Solid::cuboid(Vector3::new(12.0, 8.0, 4.0)).round(1.0),
        material.clone(),
    );

    // Two opposing fingers: simple capsules
    let left_finger = Part::new("finger_l", Solid::capsule(2.5, 8.0), material.clone());
    let right_finger = Part::new("finger_r", Solid::capsule(2.5, 8.0), material);

    Mechanism::builder("gripper")
        .part(palm)
        .part(left_finger)
        .part(right_finger)
        // Left finger hinge
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
        // Right finger hinge (mirrored)
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
        // Left flexor tendon
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
        // Right flexor tendon
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
        // Actuators
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

    // Drive both actuators in the same direction (pinch grip)
    if !data.ctrl.is_empty() {
        data.ctrl[0] = signal;
    }
    if data.ctrl.len() > 1 {
        data.ctrl[1] = signal;
    }
}

fn sync_bodies(data: Res<PhysicsData>, mut query: Query<(&ModelBodyIndex, &mut Transform)>) {
    for (body_idx, mut transform) in &mut query {
        let i = body_idx.0;
        if i < data.xipos.len() {
            transform.translation = vec3_from_vector(&data.xipos[i]);
        }
        if i < data.xquat.len() {
            let q = &data.xquat[i];
            transform.rotation = Quat::from_xyzw(
                q.coords[1] as f32,
                q.coords[2] as f32,
                q.coords[3] as f32,
                q.coords[0] as f32,
            );
        }
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // ── Design ──────────────────────────────────────────────────────────
    let mechanism = build_gripper();

    println!("=== CortenForge: Design-to-Sim ===");
    println!(
        "  Mechanism '{}': {} parts, {} joints, {} tendons",
        mechanism.name(),
        mechanism.parts().len(),
        mechanism.joints().len(),
        mechanism.tendons().len(),
    );

    // ── Export MJCF and load into physics engine ────────────────────────
    let mjcf_xml = mechanism.to_mjcf(1.5);
    let model = sim_mjcf::load_model(&mjcf_xml).expect("generated MJCF should load");

    println!(
        "  Physics model: {} bodies, {} joints, {} DOFs",
        model.nbody, model.njnt, model.nv
    );
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    let mut data = model.make_data();
    let _ = data.forward(&model);

    // Also save the MJCF and STLs for inspection
    let _ = std::fs::write("gripper.mjcf", &mjcf_xml);
    let stl_kit = mechanism.to_stl_kit(0.5);
    for (name, mesh) in &stl_kit {
        let _ = mesh_io::save_stl(mesh, format!("{name}.stl"), true);
    }

    // ── Visuals ─────────────────────────────────────────────────────────
    let part_colors = [
        Color::srgb(0.7, 0.75, 0.8), // palm
        Color::srgb(0.9, 0.4, 0.3),  // finger_l
        Color::srgb(0.3, 0.6, 0.9),  // finger_r
    ];

    // Create a visual for each non-world body
    for body_id in 1..model.nbody {
        let color_idx = (body_id - 1).min(part_colors.len() - 1);
        commands.spawn((
            ModelBodyIndex(body_id),
            Mesh3d(meshes.add(Capsule3d::new(0.05, 0.3))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: part_colors[color_idx],
                metallic: 0.5,
                perceptual_roughness: 0.3,
                ..default()
            })),
            Transform::default(),
        ));
    }

    // Camera
    let orbit = OrbitCamera::new()
        .with_target(Vec3::ZERO)
        .with_distance(0.8)
        .with_angles(0.5, 0.4);
    let mut cam_transform = Transform::default();
    orbit.apply_to_transform(&mut cam_transform);
    commands.spawn((Camera3d::default(), orbit, cam_transform));

    // Lighting
    commands.spawn((
        DirectionalLight {
            illuminance: 12000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(3.0, 5.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}
