//! Finger Design — Printable Articulated Mechanism with Simulation
//!
//! Two-segment finger with print-in-place knuckle joint and tendon channel.
//! Left side shows the separated parts (socket, pin bore, tendon channels
//! visible); right side shows the sim-driven animated finger.
//!
//! Run with: `cargo run -p example-finger-design --release`

#![allow(
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use
)]

use bevy::prelude::*;
use cf_design::{
    ActuatorDef, ActuatorKind, JointDef, JointKind, Material, Mechanism, Part, Solid, TendonDef,
    TendonWaypoint,
};
use nalgebra::{Point3, Vector3};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::mesh::spawn_design_mesh;
use sim_bevy::model_data::{
    PhysicsData, PhysicsModel, spawn_model_geoms, step_model_data, sync_geom_transforms,
};
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
        .add_systems(Update, (step_model_data, actuate_finger))
        .add_systems(PostUpdate, sync_geom_transforms)
        .run();
}

// ============================================================================
// Finger geometry
// ============================================================================

/// Proximal phalange with knuckle condyle and hinge pin.
///
/// The pin is `smooth_union`'d into the bone (anatomically correct — the pin
/// is part of the proximal, not a separate part). Z-axis is finger length.
fn proximal_phalange() -> Solid {
    let body = Solid::ellipsoid(Vector3::new(5.0, 3.5, 14.0)).round(0.5);

    let knuckle =
        Solid::ellipsoid(Vector3::new(5.5, 4.0, 4.0)).translate(Vector3::new(0.0, 0.0, 13.0));

    // Pin through the knuckle along X (the hinge axis)
    let pin = Solid::capsule(1.5, 5.0)
        .rotate(nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::z_axis(),
            std::f64::consts::FRAC_PI_2,
        ))
        .translate(Vector3::new(0.0, 0.0, 13.0));

    body.smooth_union(knuckle, 2.5).smooth_union(pin, 1.0)
}

/// Distal phalange with socket at origin, body extending upward.
///
/// The socket housing is centered at z=0 (mesh origin) so that
/// `Part::with_joint_origin(0,0,0)` aligns the socket directly with the
/// knuckle joint. The body extends in +Z from the socket to the fingertip.
///
/// Socket sizing: condyle(5.5/4.0/4.0) + 0.3mm clearance + 0.3mm wall.
fn distal_phalange() -> Solid {
    // Body centered at z=11 — extends from roughly z=0 to z=22
    let body = Solid::ellipsoid(Vector3::new(4.5, 3.0, 11.0))
        .round(0.5)
        .translate(Vector3::new(0.0, 0.0, 11.0));

    let tip = Solid::ellipsoid(Vector3::new(3.0, 2.5, 4.0)).translate(Vector3::new(0.0, 0.0, 21.0));

    // Socket housing at z=0 — condyle(5.5,4.0,4.0) + 0.3mm gap + 0.3mm wall
    let socket_housing = Solid::ellipsoid(Vector3::new(6.4, 4.9, 4.9));

    // Condyle void: condyle dims + 0.3mm clearance per side
    let condyle_void = Solid::ellipsoid(Vector3::new(5.8, 4.3, 4.3));

    // Pin bore: pin r=1.5 + 0.3mm clearance = r=1.8
    let pin_bore = Solid::capsule(1.8, 6.0).rotate(nalgebra::UnitQuaternion::from_axis_angle(
        &nalgebra::Vector3::z_axis(),
        std::f64::consts::FRAC_PI_2,
    ));

    body.smooth_union(tip, 3.0)
        .smooth_union(socket_housing, 2.0)
        .smooth_subtract(condyle_void, 0.5)
        .subtract(pin_bore)
}

// ============================================================================
// Mechanism
// ============================================================================

/// Build the articulated finger: 2 parts, 1 revolute joint, 1 tendon, 1 motor.
///
/// Tendon channels are carved through both phalanges for the flexor tendon.
fn build_finger() -> Mechanism {
    let material = Material::new("PLA", 1250.0)
        .with_youngs_modulus(3.5e9)
        .with_color([0.9, 0.85, 0.75, 1.0]);

    // Define the tendon first so we can carve channels through parts.
    // Distal waypoints are in the new mesh space (socket at z=0, body extends +Z).
    let flexor = TendonDef::new(
        "flexor",
        vec![
            TendonWaypoint::new("proximal", Point3::new(0.0, -3.0, 0.0)),
            TendonWaypoint::new("proximal", Point3::new(0.0, -3.0, 12.0)),
            TendonWaypoint::new("distal", Point3::new(0.0, -2.5, 2.0)),
            TendonWaypoint::new("distal", Point3::new(0.0, -2.5, 15.0)),
        ],
        0.8,
    )
    .with_stiffness(50.0)
    .with_damping(15.0);

    let proximal = Part::new("proximal", proximal_phalange(), material.clone())
        .with_tendon_channels(std::slice::from_ref(&flexor));

    let distal = Part::new("distal", distal_phalange(), material)
        .with_joint_origin(Vector3::new(0.0, 0.0, 0.0))
        .with_tendon_channels(std::slice::from_ref(&flexor));

    Mechanism::builder("finger")
        .part(proximal)
        .part(distal)
        .joint(
            JointDef::new(
                "knuckle",
                "proximal",
                "distal",
                JointKind::Revolute,
                Point3::new(0.0, 0.0, 13.0),
                Vector3::x(),
            )
            .with_range(-0.1, 2.0),
        )
        .tendon(flexor)
        .actuator(
            ActuatorDef::new("motor", "flexor", ActuatorKind::Motor, (-10.0, 10.0))
                .with_ctrl_range(-1.0, 1.0),
        )
        .build()
}

// ============================================================================
// Bevy systems
// ============================================================================

/// Oscillate finger flexion/extension with a sine wave.
fn actuate_finger(mut data: ResMut<PhysicsData>) {
    let t = data.time;
    let signal = (t * 1.5).sin();
    if !data.ctrl.is_empty() {
        data.ctrl[0] = signal;
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mechanism = build_finger();

    println!("=== Finger Design — Articulated Mechanism ===");
    println!(
        "  Mechanism '{}': {} parts, {} joints, {} tendons",
        mechanism.name(),
        mechanism.parts().len(),
        mechanism.joints().len(),
        mechanism.tendons().len(),
    );

    // ── Left: separated static parts ────────────────────────────────────
    let bone_color = Color::srgb(0.9, 0.85, 0.75);
    let tip_color = Color::srgb(0.8, 0.75, 0.68);

    for (i, part) in mechanism.parts().iter().enumerate() {
        let mesh_data = part.solid().mesh(0.3);
        let color = if i == 0 { bone_color } else { tip_color };
        let z_offset = i as f64 * 35.0;
        println!(
            "  {}: {} verts, {} faces",
            part.name(),
            mesh_data.vertex_count(),
            mesh_data.face_count()
        );
        spawn_design_mesh(
            &mut commands,
            &mut meshes,
            &mut materials,
            &mesh_data,
            Point3::new(-25.0, 0.0, z_offset),
            color,
        );
    }

    // ── Right: sim-driven animated finger ───────────────────────────────
    let mjcf_xml = mechanism.to_mjcf(1.5);
    let model = sim_mjcf::load_model(&mjcf_xml).expect("generated MJCF should load");

    println!(
        "  Physics: {} bodies, {} joints, {} DOFs, {} geoms, {} actuators",
        model.nbody, model.njnt, model.nv, model.ngeom, model.nu
    );
    println!("  Left: separated parts | Right: animated finger");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    let mut data = model.make_data();
    let _ = data.forward(&model);

    spawn_model_geoms(&mut commands, &mut meshes, &mut materials, &model, &data);

    ExampleScene::new(80.0, 60.0)
        .with_target(Vec3::new(0.0, 12.0, 0.0))
        .with_angles(0.3, 0.4)
        .with_ground_y(-20.0)
        .spawn(&mut commands, &mut meshes, &mut materials);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}
