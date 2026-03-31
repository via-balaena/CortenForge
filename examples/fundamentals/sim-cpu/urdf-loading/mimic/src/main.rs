//! Mimic Joint Coupling
//!
//! Two pendulum arms side by side. The follower mimics the leader with
//! multiplier=2, offset=0.1. They start at different angles — the constraint
//! pulls the follower to track at 2*leader + 0.1. You should see the right
//! arm (follower) swing at roughly double the angle of the left arm (leader).
//!
//! Run: `cargo run -p example-urdf-mimic --release`

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

/// Two arms side by side. Follower mimics leader with ratio 2:1 + offset 0.1.
/// Each arm has a rod + sphere tip for visibility.
const MIMIC_URDF: &str = r#"<?xml version="1.0"?>
<robot name="mimic_test">
    <link name="base">
        <inertial>
            <mass value="10.0"/>
            <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
    </link>
    <link name="leader_arm">
        <inertial>
            <origin xyz="0 0 -0.4"/>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
        <collision>
            <origin xyz="0 0 -0.2"/>
            <geometry>
                <cylinder radius="0.02" length="0.4"/>
            </geometry>
        </collision>
        <collision>
            <origin xyz="0 0 -0.4"/>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
        </collision>
    </link>
    <link name="follower_arm">
        <inertial>
            <origin xyz="0 0 -0.4"/>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
        <collision>
            <origin xyz="0 0 -0.2"/>
            <geometry>
                <cylinder radius="0.02" length="0.4"/>
            </geometry>
        </collision>
        <collision>
            <origin xyz="0 0 -0.4"/>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
        </collision>
    </link>
    <joint name="leader" type="revolute">
        <parent link="base"/>
        <child link="leader_arm"/>
        <origin xyz="0 0.3 2.0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-3.14" upper="3.14"/>
    </joint>
    <joint name="follower" type="revolute">
        <parent link="base"/>
        <child link="follower_arm"/>
        <origin xyz="0 -0.3 2.0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-6.28" upper="6.28"/>
        <mimic joint="leader" multiplier="2.0" offset="0.1"/>
    </joint>
</robot>
"#;

// ── Bevy App ─────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: URDF Mimic Joint Coupling ===");
    println!("  Two arms: follower = 2*leader + 0.1");
    println!("  URDF <mimic> → MJCF <equality><joint polycoef>");
    println!("  Leader starts at 30°, follower tracks at ~60°+offset");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — URDF Mimic Joint".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<MimicValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(8.0)
                .print_every(1.0)
                .display(|_m, d| {
                    let leader = d.qpos[0];
                    let follower = d.qpos[1];
                    let expected = 2.0 * leader + 0.1;
                    let err = (follower - expected).abs();
                    format!(
                        "leader={:+5.1}°  follower={:+5.1}°  err={err:.4}rad",
                        leader.to_degrees(),
                        follower.to_degrees(),
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                mimic_diagnostics,
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
    let model = sim_urdf::load_urdf_model(MIMIC_URDF).expect("URDF should parse");
    let mut data = model.make_data();

    // Leader starts at 30°, follower near expected position
    let leader_angle = std::f64::consts::FRAC_PI_6;
    data.qpos[0] = leader_angle;
    data.qpos[1] = 2.0 * leader_angle + 0.1;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} equality constraints\n",
        model.nbody, model.njnt, model.neq
    );

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 1.6),
        3.5,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ──────────────────────────────────────────────────────────────────

fn update_hud(_model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("URDF Mimic Joint (2:1 + 0.1)");

    let leader = data.qpos[0];
    let follower = data.qpos[1];
    let expected = 2.0 * leader + 0.1;
    let err = (follower - expected).abs();

    hud.scalar("leader (deg)", leader.to_degrees(), 1);
    hud.scalar("follower (deg)", follower.to_degrees(), 1);
    hud.scalar("expected (deg)", expected.to_degrees(), 1);
    hud.scalar("mimic error (rad)", err, 4);
    hud.scalar("time", data.time, 1);
}

// ── Validation ───────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct MimicValidation {
    max_err: f64,
    reported: bool,
}

fn mimic_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<MimicValidation>,
) {
    if data.time > 1.0 {
        let leader = data.qpos[0];
        let follower = data.qpos[1];
        let err = (follower - (2.0 * leader + 0.1)).abs();
        val.max_err = val.max_err.max(err);
    }

    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let checks = vec![
        Check {
            name: "Has equality constraint",
            pass: model.neq > 0,
            detail: format!("neq={}", model.neq),
        },
        Check {
            name: "Two joints in model",
            pass: model.njnt == 2,
            detail: format!("njnt={}", model.njnt),
        },
        Check {
            name: "Mimic tracks (max err < 0.05 rad)",
            pass: val.max_err < 0.05,
            detail: format!("max_err={:.4} rad", val.max_err),
        },
    ];
    let _ = print_report("URDF Mimic", &checks);
}
