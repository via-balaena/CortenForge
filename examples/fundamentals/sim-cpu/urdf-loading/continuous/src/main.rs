//! Continuous Joint Wheel
//!
//! A spinning wheel with no joint limits. The URDF `continuous` joint type
//! converts to an MJCF `hinge` with `limited="false"`. A constant torque is
//! applied via `qfrc_applied` and the wheel accelerates continuously — the
//! angle passes well beyond 2*pi without clamping.
//!
//! Run: `cargo run -p example-urdf-continuous --release`

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
    clippy::too_many_lines
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

/// Wheel on axle. The wheel is a short, wide cylinder (disc) so the spin
/// is clearly visible. Continuous joint about Z (vertical axis).
const WHEEL_URDF: &str = r#"<?xml version="1.0"?>
<robot name="wheel">
    <link name="axle">
        <inertial>
            <mass value="100.0"/>
            <inertia ixx="10" ixy="0" ixz="0" iyy="10" iyz="0" izz="10"/>
        </inertial>
    </link>
    <link name="wheel">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
        </inertial>
        <collision>
            <geometry>
                <cylinder radius="0.4" length="0.08"/>
            </geometry>
        </collision>
        <collision>
            <origin xyz="0.3 0 0"/>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
        </collision>
    </link>
    <joint name="spin" type="continuous">
        <parent link="axle"/>
        <child link="wheel"/>
        <origin xyz="0 0 1.0"/>
        <axis xyz="0 0 1"/>
    </joint>
</robot>
"#;

// ── Physics constants ────────────────────────────────────────────────────

const TORQUE: f64 = 0.1; // gentle torque so spin is visible
const I_ZZ: f64 = 0.02;

// ── Bevy App ─────────────────────────────────────────────────────────────

fn main() {
    let alpha = TORQUE / I_ZZ;
    println!("=== CortenForge: URDF Continuous Joint Wheel ===");
    println!("  URDF continuous → MJCF hinge (unlimited)");
    println!("  I_zz={I_ZZ}  tau={TORQUE}  alpha={alpha:.1} rad/s^2");
    println!("  Wheel accelerates continuously — no angle limit");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — URDF Continuous Wheel".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<WheelValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(8.0)
                .print_every(1.0)
                .display(|m, d| {
                    let angle = d.joint_qpos(m, 0)[0];
                    let vel = d.joint_qvel(m, 0)[0];
                    format!(
                        "angle={:.1}°  vel={vel:.2} rad/s  revs={:.1}",
                        angle.to_degrees(),
                        angle / (2.0 * std::f64::consts::PI),
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_torque, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                wheel_diagnostics,
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
    let model = sim_urdf::load_urdf_model(WHEEL_URDF).expect("URDF should parse");
    let data = model.make_data();

    println!(
        "  Model: {} bodies, {} joints, {} geoms",
        model.nbody, model.njnt, model.ngeom
    );
    println!(
        "  Joint type: {:?}, limited: {}\n",
        model.jnt_type[0], model.jnt_limited[0]
    );

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[],
    );

    spawn_example_camera(&mut commands, physics_pos(0.0, 0.0, 1.0), 2.5, 0.3, 0.8);
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Torque ───────────────────────────────────────────────────────────────

fn apply_torque(mut data: ResMut<PhysicsData>) {
    data.qfrc_applied[0] = TORQUE;
}

// ── HUD ──────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("URDF Continuous Wheel");

    let angle = data.joint_qpos(&model, 0)[0];
    let vel = data.joint_qvel(&model, 0)[0];
    let revs = angle / (2.0 * std::f64::consts::PI);

    hud.scalar("angle (deg)", angle.to_degrees(), 1);
    hud.scalar("velocity (rad/s)", vel, 2);
    hud.scalar("revolutions", revs, 2);
    hud.scalar("torque (Nm)", TORQUE, 3);
    hud.scalar("time", data.time, 1);
}

// ── Validation ───────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct WheelValidation {
    reported: bool,
}

fn wheel_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<WheelValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let is_hinge = model.jnt_type[0] == sim_core::MjJointType::Hinge;
    let is_unlimited = !model.jnt_limited[0];

    // Check that velocity matches alpha * t
    let expected_vel = (TORQUE / I_ZZ) * data.time;
    let measured_vel = data.qvel[0];
    let vel_err = ((measured_vel - expected_vel) / expected_vel).abs();

    // Check that angle exceeds 2*pi
    let past_2pi = data.qpos[0].abs() > 2.0 * std::f64::consts::PI;

    let checks = vec![
        Check {
            name: "Continuous → unlimited hinge",
            pass: is_hinge && is_unlimited,
            detail: format!(
                "type={:?}, limited={}",
                model.jnt_type[0], model.jnt_limited[0]
            ),
        },
        Check {
            name: "Velocity matches alpha*t",
            pass: vel_err < 0.05,
            detail: format!(
                "measured={measured_vel:.2}, expected={expected_vel:.2}, err={:.1}%",
                vel_err * 100.0
            ),
        },
        Check {
            name: "Angle past 2*pi (no clamping)",
            pass: past_2pi,
            detail: format!("angle={:.1} rad", data.qpos[0]),
        },
    ];
    let _ = print_report("URDF Continuous", &checks);
}
