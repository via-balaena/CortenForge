//! Revolute Joint Pendulum
//!
//! Single hinge pendulum defined in URDF. Swings under gravity. The arm has
//! a rod (cylinder) and tip mass (sphere) so the pendulum is clearly visible.
//! Verifies the oscillation period matches the analytical compound-pendulum
//! prediction and that energy is conserved.
//!
//! Run: `cargo run -p example-urdf-revolute --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::too_many_lines,
    clippy::approx_constant
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::physics_pos;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── URDF Model ───────────────────────────────────────────────────────────

/// Pendulum: heavy base (bracket) at height, arm with rod + tip mass.
/// The base is at z=2.0 so the pendulum hangs visibly in space.
/// Two collision shapes on the arm: a cylinder rod and a sphere tip.
/// COM at 1.0m below pivot (tip mass dominates).
const PENDULUM_URDF: &str = r#"<?xml version="1.0"?>
<robot name="pendulum">
    <link name="base">
        <inertial>
            <mass value="10.0"/>
            <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
    </link>
    <link name="arm">
        <inertial>
            <origin xyz="0 0 -1.0"/>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
        <collision>
            <origin xyz="0 0 -0.5"/>
            <geometry>
                <cylinder radius="0.02" length="1.0"/>
            </geometry>
        </collision>
        <collision>
            <origin xyz="0 0 -1.0"/>
            <geometry>
                <sphere radius="0.06"/>
            </geometry>
        </collision>
    </link>
    <joint name="hinge" type="revolute">
        <parent link="base"/>
        <child link="arm"/>
        <origin xyz="0 0 2.0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-3.14" upper="3.14"/>
    </joint>
</robot>
"#;

// ── Physics constants ────────────────────────────────────────────────────

const MASS: f64 = 1.0;
const GRAVITY: f64 = 9.81;
const COM_DIST: f64 = 1.0; // distance from pivot to COM
const I_CM: f64 = 0.01;
const INITIAL_ANGLE: f64 = std::f64::consts::FRAC_PI_6; // 30°

fn analytical_period() -> f64 {
    let i_pivot = I_CM + MASS * COM_DIST * COM_DIST;
    let t0 = 2.0 * std::f64::consts::PI * (i_pivot / (MASS * GRAVITY * COM_DIST)).sqrt();
    let correction = 1.0 + INITIAL_ANGLE * INITIAL_ANGLE / 16.0;
    t0 * correction
}

// ── Bevy App ─────────────────────────────────────────────────────────────

fn main() {
    let t = analytical_period();
    println!("=== CortenForge: URDF Revolute Joint Pendulum ===");
    println!("  URDF revolute → MJCF hinge, undamped");
    println!("  m={MASS}kg  L={COM_DIST}m  g={GRAVITY}");
    println!("  Analytical period T = {t:.4}s");
    println!("  Initial angle = 30°");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — URDF Revolute Pendulum".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<PendulumValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(12.0)
                .print_every(2.0)
                .display(|m, d| {
                    let angle = d.joint_qpos(m, 0)[0];
                    let vel = d.joint_qvel(m, 0)[0];
                    format!("angle={:+6.1}°  vel={vel:+.2} rad/s", angle.to_degrees())
                })
                .track_period(
                    "Period",
                    |m, d| (d.joint_qpos(m, 0)[0], d.time),
                    analytical_period(),
                    2.0,
                )
                .track_energy(0.5),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                pendulum_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Load via the URDF pipeline: URDF → MJCF → Model
    let model = sim_urdf::load_urdf_model(PENDULUM_URDF).expect("URDF should parse");
    let mut data = model.make_data();

    // Start at 30° from vertical
    data.qpos[0] = INITIAL_ANGLE;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} geoms, {} DOFs",
        model.nbody, model.njnt, model.ngeom, model.nv
    );
    println!(
        "  Joint type: {:?}, limited: {}, range: ({:.2}, {:.2})\n",
        model.jnt_type[0], model.jnt_limited[0], model.jnt_range[0].0, model.jnt_range[0].1
    );

    // Snapshot initial energy
    let e0 = data.energy_kinetic + data.energy_potential;
    commands.insert_resource(InitialEnergy(e0));

    // Spawn geoms (URDF geoms don't have names, so no material overrides)
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[],
    );

    // Camera targets the midpoint of the pendulum arc (pivot is at z=2.0,
    // tip is at z=1.0 → midpoint z=1.5 in physics coords)
    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 1.5),
        4.0,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Resources ────────────────────────────────────────────────────────────

#[derive(Resource)]
struct InitialEnergy(f64);

// ── HUD ──────────────────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    initial: Res<InitialEnergy>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();
    hud.section("URDF Revolute Pendulum");

    let angle = data.joint_qpos(&model, 0)[0];
    let vel = data.joint_qvel(&model, 0)[0];
    let energy = data.energy_kinetic + data.energy_potential;
    let drift = ((energy - initial.0) / initial.0.abs().max(1e-10)).abs();

    hud.scalar("angle (deg)", angle.to_degrees(), 1);
    hud.scalar("velocity (rad/s)", vel, 3);
    hud.scalar("energy (J)", energy, 6);
    hud.scalar("energy drift (%)", drift * 100.0, 4);
    hud.scalar("time", data.time, 1);
}

// ── Validation ───────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct PendulumValidation {
    reported: bool,
}

fn pendulum_diagnostics(
    model: Res<PhysicsModel>,
    _data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<PendulumValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    // Check joint type
    let is_hinge = model.jnt_type[0] == sim_core::MjJointType::Hinge;
    let is_limited = model.jnt_limited[0];

    // Check URDF → MJCF intermediate
    let mjcf = sim_urdf::urdf_to_mjcf(PENDULUM_URDF).expect("convert");
    let has_hinge = mjcf.contains(r#"type="hinge""#);

    let checks = vec![
        Check {
            name: "Revolute → hinge",
            pass: is_hinge,
            detail: format!("type={:?}", model.jnt_type[0]),
        },
        Check {
            name: "Joint is limited",
            pass: is_limited,
            detail: format!(
                "range=({:.2}, {:.2})",
                model.jnt_range[0].0, model.jnt_range[0].1
            ),
        },
        Check {
            name: "MJCF contains hinge",
            pass: has_hinge,
            detail: format!("found: {has_hinge}"),
        },
    ];
    let _ = print_report("URDF Revolute (structural)", &checks);
}
