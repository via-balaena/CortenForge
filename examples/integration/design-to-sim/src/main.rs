//! Design-to-Sim — Zero Export Steps
//!
//! Design a simple gripper in cf-design, build a sim-core Model directly
//! (SDF-native physics), and visualize with Bevy. Each part uses an SDF
//! grid for collision and a triangle mesh for rendering — both derived
//! from the same implicit surface.
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
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::model_data::{
    PhysicsData, PhysicsModel, spawn_model_geoms, step_model_data, sync_geom_transforms,
};
use sim_bevy::scene::ExampleScene;

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
        .add_systems(PostUpdate, sync_geom_transforms)
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

    // ── SDF-native physics model ────────────────────────────────────
    let model = mechanism.to_model(1.0, 0.8);

    println!(
        "  Physics model: {} bodies, {} joints, {} DOFs, {} geoms, {} meshes",
        model.nbody, model.njnt, model.nv, model.ngeom, model.nmesh
    );
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    let mut data = model.make_data();
    let _ = data.forward(&model);

    // ── Spawn ALL geoms (including mesh geoms) via the engine ────────
    spawn_model_geoms(&mut commands, &mut meshes, &mut materials, &model, &data);

    ExampleScene::new(80.0, 50.0).spawn(&mut commands, &mut meshes, &mut materials);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}
