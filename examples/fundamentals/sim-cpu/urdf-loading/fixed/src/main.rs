//! Fixed Joint Fusion
//!
//! Three URDF links: base → (fixed) sensor_mount → (revolute) arm.
//! With `fusestatic="true"`, the sensor_mount merges into the base. The
//! compiled model has fewer bodies than the URDF has links. Visually you
//! see a pendulum arm swinging — the fixed-joint link is invisible because
//! it was fused away.
//!
//! Run: `cargo run -p example-urdf-fixed --release`

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

/// Three links: base → (fixed) sensor_mount → (revolute) arm.
/// The arm has collision geometry so it's visible. The sensor_mount is
/// invisible — it only has inertia and gets fused into the base.
const FIXED_URDF: &str = r#"<?xml version="1.0"?>
<robot name="fixed_test">
    <link name="base">
        <inertial>
            <mass value="5.0"/>
            <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
        </inertial>
    </link>
    <link name="sensor_mount">
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>
    <link name="arm">
        <inertial>
            <origin xyz="0 0 -0.5"/>
            <mass value="1.0"/>
            <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.002"/>
        </inertial>
        <collision>
            <origin xyz="0 0 -0.25"/>
            <geometry>
                <cylinder radius="0.02" length="0.5"/>
            </geometry>
        </collision>
        <collision>
            <origin xyz="0 0 -0.5"/>
            <geometry>
                <sphere radius="0.06"/>
            </geometry>
        </collision>
    </link>
    <joint name="mount_fixed" type="fixed">
        <parent link="base"/>
        <child link="sensor_mount"/>
        <origin xyz="0 0 0.1"/>
    </joint>
    <joint name="arm_joint" type="revolute">
        <parent link="sensor_mount"/>
        <child link="arm"/>
        <origin xyz="0 0 2.0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57"/>
    </joint>
</robot>
"#;

// ── Bevy App ─────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: URDF Fixed Joint Fusion ===");
    println!("  3 URDF links, 1 fixed joint → fused into 2 bodies");
    println!("  The sensor_mount link disappears via fusestatic");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — URDF Fixed Joint Fusion".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<FusionValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(8.0)
                .print_every(2.0)
                .display(|m, d| {
                    let angle = d.joint_qpos(m, 0)[0];
                    format!("angle={:+6.1}°", angle.to_degrees())
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                fusion_diagnostics,
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
    let model = sim_urdf::load_urdf_model(FIXED_URDF).expect("URDF should parse");
    let mut data = model.make_data();

    data.qpos[0] = std::f64::consts::FRAC_PI_6; // 30°
    let _ = data.forward(&model);

    println!("  URDF: 3 links, 2 joints (1 fixed, 1 revolute)",);
    println!(
        "  Model: {} bodies, {} joints, {} DOFs (fixed joint eliminated)\n",
        model.nbody, model.njnt, model.nv
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
        physics_pos(0.0, 0.0, 1.5),
        3.5,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ──────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("URDF Fixed Joint Fusion");

    let angle = data.joint_qpos(&model, 0)[0];
    let vel = data.joint_qvel(&model, 0)[0];

    hud.raw("URDF: 3 links (base→fixed→arm)".into());
    hud.raw(format!(
        "Model: {} bodies, {} joints",
        model.nbody, model.njnt
    ));
    hud.scalar("angle (deg)", angle.to_degrees(), 1);
    hud.scalar("velocity (rad/s)", vel, 3);
    hud.scalar("time", data.time, 1);
}

// ── Validation ───────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct FusionValidation {
    reported: bool,
}

fn fusion_diagnostics(
    model: Res<PhysicsModel>,
    _data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<FusionValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let checks = vec![
        Check {
            name: "Fixed joint fused (nbody < 4)",
            pass: model.nbody < 4,
            detail: format!("nbody={}", model.nbody),
        },
        Check {
            name: "Only revolute survives (njnt=1)",
            pass: model.njnt == 1,
            detail: format!("njnt={}", model.njnt),
        },
        Check {
            name: "DOF = 1",
            pass: model.nv == 1,
            detail: format!("nv={}", model.nv),
        },
    ];
    let _ = print_report("URDF Fixed Joint Fusion", &checks);
}
