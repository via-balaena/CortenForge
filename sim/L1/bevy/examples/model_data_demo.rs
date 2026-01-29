//! Model/Data Architecture Demo
//!
//! This example demonstrates the MuJoCo-style Model/Data architecture with Bevy.
//!
//! Key concepts:
//! - Model (static) contains the kinematic tree, joint definitions, masses
//! - Data (dynamic) contains qpos/qvel as the source of truth
//! - Body poses (xpos/xquat) are COMPUTED from qpos via forward kinematics
//!
//! Run with: `cargo run -p sim-bevy --example model_data_demo --features x11`

#![allow(clippy::needless_pass_by_value)]

use bevy::prelude::*;
use nalgebra::{UnitQuaternion, Vector3};
use sim_bevy::convert::vec3_from_vector;
use sim_bevy::model_data::{
    step_model_data, sync_model_data_to_bevy, ModelBodyIndex, PhysicsData, PhysicsModel,
};
use sim_core::{MjJointType, Model};

/// Pendulum length in meters
const PENDULUM_LENGTH: f64 = 1.0;
/// Pendulum mass in kg
const PENDULUM_MASS: f64 = 1.0;
/// Initial angle in radians
const INITIAL_ANGLE: f64 = std::f64::consts::PI / 3.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .init_resource::<DebugPrintState>()
        .add_systems(Startup, setup_physics_and_scene)
        .add_systems(Update, step_model_data)
        .add_systems(PostUpdate, (sync_model_data_to_bevy, debug_info))
        .run();
}

/// Create a simple pendulum Model programmatically.
///
/// This demonstrates how Model contains the static physics definition:
/// - Body tree structure (world → pendulum body)
/// - Joint definition (hinge at origin)
/// - Mass properties
fn create_pendulum_model() -> Model {
    let mut model = Model::empty();

    // Add pendulum body (child of world body 0)
    model.nbody = 2; // world + pendulum

    // Body tree - pendulum is child of world
    model.body_parent.push(0); // Pendulum's parent is world (body 0)
    model.body_rootid.push(1);
    model.body_jnt_adr.push(0); // First joint for pendulum
    model.body_jnt_num.push(1); // Pendulum has 1 joint
    model.body_dof_adr.push(0); // First DOF
    model.body_dof_num.push(1); // 1 DOF for hinge
    model.body_geom_adr.push(0);
    model.body_geom_num.push(0);

    // Body properties - pendulum body frame is at the bob position
    model
        .body_pos
        .push(Vector3::new(0.0, 0.0, -PENDULUM_LENGTH));
    model.body_quat.push(UnitQuaternion::identity());
    model.body_ipos.push(Vector3::zeros()); // COM at body origin
    model.body_iquat.push(UnitQuaternion::identity());
    model.body_mass.push(PENDULUM_MASS);
    // Point mass has zero principal inertia (simplified)
    model.body_inertia.push(Vector3::new(0.001, 0.001, 0.001));
    model.body_name.push(Some("pendulum".to_string()));

    // Joint definition (hinge at world origin, rotating around Y axis)
    model.njnt = 1;
    model.nq = 1; // 1 position coordinate for hinge
    model.nv = 1; // 1 velocity coordinate for hinge

    model.jnt_type.push(MjJointType::Hinge);
    model.jnt_body.push(1); // Joint belongs to pendulum body
    model.jnt_qpos_adr.push(0);
    model.jnt_dof_adr.push(0);
    model.jnt_pos.push(Vector3::zeros()); // Joint at body origin
    model.jnt_axis.push(Vector3::new(0.0, 1.0, 0.0)); // Rotate around Y
    model.jnt_limited.push(false);
    model
        .jnt_range
        .push((-std::f64::consts::PI, std::f64::consts::PI));
    model.jnt_stiffness.push(0.0);
    model.jnt_damping.push(0.0);
    model.jnt_armature.push(0.0);
    model.jnt_name.push(Some("hinge".to_string()));

    // DOF definition
    model.dof_body.push(1);
    model.dof_jnt.push(0);
    model.dof_parent.push(None);
    model.dof_armature.push(0.0);
    model.dof_damping.push(0.0);

    // Default qpos (hanging down)
    model.qpos0 = nalgebra::DVector::zeros(1);

    // Physics options
    model.timestep = 1.0 / 240.0;
    model.gravity = Vector3::new(0.0, 0.0, -9.81);
    model.solver_iterations = 10;
    model.solver_tolerance = 1e-8;

    model
}

/// Setup both physics (Model/Data) and visual scene.
fn setup_physics_and_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create physics model and data
    let model = create_pendulum_model();
    let mut data = model.make_data();

    // Set initial angle
    data.qpos[0] = INITIAL_ANGLE;

    // Run forward kinematics to compute initial body poses
    let _ = data.forward(&model);

    println!("========================================");
    println!("Model/Data Architecture Demo");
    println!("========================================");
    println!("Model (static):");
    println!("  nbody: {}", model.nbody);
    println!("  njnt:  {}", model.njnt);
    println!("  nq:    {} (position coordinates)", model.nq);
    println!("  nv:    {} (velocity DOFs)", model.nv);
    println!("----------------------------------------");
    println!("Data (dynamic):");
    println!(
        "  qpos[0]: {:.3} rad ({:.1}°)",
        data.qpos[0],
        data.qpos[0].to_degrees()
    );
    println!("  qvel[0]: {:.3} rad/s", data.qvel[0]);
    println!(
        "  xpos[1]: [{:.3}, {:.3}, {:.3}]",
        data.xpos[1].x, data.xpos[1].y, data.xpos[1].z
    );
    println!("========================================");

    // Spawn pendulum body entity linked to Model/Data
    // Use canonical coordinate conversion (Z-up physics → Y-up Bevy)
    let bob_pos = vec3_from_vector(&data.xpos[1]);

    commands.spawn((
        ModelBodyIndex(1), // Link to body 1 (pendulum)
        Mesh3d(meshes.add(Sphere::new(0.1))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.2, 0.2),
            metallic: 0.5,
            perceptual_roughness: 0.3,
            ..default()
        })),
        Transform::from_translation(bob_pos),
    ));

    // Pivot point visual
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.08))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(3.0, 1.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Lighting
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Ground plane for reference
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(2.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.5, 0.5, 0.5, 0.5),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, -PENDULUM_LENGTH as f32, 0.0),
    ));

    // Insert physics resources
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

/// Resource to track last print time
#[derive(Resource, Default)]
struct DebugPrintState {
    last_print_sim_time: f64,
}

/// Print debug information periodically
fn debug_info(data: Res<PhysicsData>, mut state: ResMut<DebugPrintState>) {
    // Print every 2 seconds of simulation time
    if data.time - state.last_print_sim_time > 2.0 {
        state.last_print_sim_time = data.time;

        let qpos = data.qpos[0];
        let qvel = data.qvel[0];
        let kinetic = data.energy_kinetic;
        let potential = data.energy_potential;

        println!(
            "t={:.1}s  θ={:+.2}rad ({:+.0}°)  ω={:+.2}rad/s  KE={:.4}J  PE={:.4}J",
            data.time,
            qpos,
            qpos.to_degrees(),
            qvel,
            kinetic,
            potential,
        );
    }
}
