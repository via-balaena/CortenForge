//! Inertia Tensor Handling
//!
//! Two free-floating boxes in zero gravity, side by side:
//! - Left: diagonal inertia (symmetric) → `diaginertia` in MJCF
//! - Right: off-diagonal inertia (asymmetric) → `fullinertia` in MJCF
//!
//! Both start with the same angular velocity. Watch them precess
//! differently — the asymmetric body wobbles more chaotically.
//!
//! Run: `cargo run -p example-urdf-inertia --release`

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

// ── URDF for validation (separate models to check diaginertia vs fullinertia)

const FULL_URDF: &str = r#"<?xml version="1.0"?>
<robot name="full_inertia">
    <link name="world">
        <inertial>
            <mass value="100.0"/>
            <inertia ixx="10" ixy="0" ixz="0" iyy="10" iyz="0" izz="10"/>
        </inertial>
    </link>
    <link name="body">
        <inertial>
            <mass value="2.0"/>
            <inertia ixx="0.1" ixy="0.02" ixz="0.01" iyy="0.2" iyz="0.03" izz="0.3"/>
        </inertial>
    </link>
    <joint name="float" type="floating">
        <parent link="world"/>
        <child link="body"/>
    </joint>
</robot>
"#;

/// Combined MJCF with both bodies for visual rendering.
/// Left (x=-1): diagonal inertia, flat box.
/// Right (x=+1): off-diagonal (fullinertia), taller box.
const COMBINED_MJCF: &str = r#"
<mujoco model="inertia_comparison">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.002"/>

  <worldbody>
    <body name="diag" pos="-1 0 1">
      <freejoint name="free_diag"/>
      <inertial pos="0 0 0" mass="2.0" diaginertia="0.1 0.2 0.3"/>
      <geom name="diag_box" type="box" size="0.25 0.15 0.075"
            rgba="0.4 0.6 0.8 1"/>
    </body>
    <body name="full" pos="1 0 1">
      <freejoint name="free_full"/>
      <inertial pos="0 0 0" mass="2.0" fullinertia="0.1 0.2 0.3 0.02 0.01 0.03"/>
      <geom name="full_box" type="box" size="0.25 0.15 0.125"
            rgba="0.8 0.5 0.3 1"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Bevy App ─────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: URDF Inertia Tensor Handling ===");
    println!("  Left: diagonal inertia (symmetric precession)");
    println!("  Right: off-diagonal inertia (asymmetric wobble)");
    println!("  Same initial angular velocity — watch them diverge");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — URDF Inertia Tensors".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<InertiaValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(8.0)
                .print_every(2.0)
                .display(|_m, _d| String::new()),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                inertia_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Setup ────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Use combined MJCF with both bodies in one model
    let model = sim_mjcf::load_model(COMBINED_MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    // Same initial angular velocity for both bodies
    // Body 1 (diag): qvel[3..6], Body 2 (full): qvel[9..12]
    // (each free joint has 6 DOFs: 3 linear + 3 angular)
    data.qvel[3] = 2.0;
    data.qvel[4] = 1.0;
    data.qvel[5] = 0.5;
    data.qvel[9] = 2.0;
    data.qvel[10] = 1.0;
    data.qvel[11] = 0.5;
    let _ = data.forward(&model);

    // Verify the URDF fullinertia path works
    let mjcf_check = sim_urdf::urdf_to_mjcf(FULL_URDF).expect("convert");
    println!(
        "  URDF fullinertia conversion: {}\n",
        mjcf_check.contains("fullinertia")
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
        physics_pos(0.0, 0.0, 1.0),
        5.0,
        std::f32::consts::FRAC_PI_4,
        0.4,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ──────────────────────────────────────────────────────────────────

fn update_hud(_model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("URDF Inertia Tensors");

    let diag_omega = (data.qvel[3].powi(2) + data.qvel[4].powi(2) + data.qvel[5].powi(2)).sqrt();
    let full_omega = (data.qvel[9].powi(2) + data.qvel[10].powi(2) + data.qvel[11].powi(2)).sqrt();

    hud.raw("Left (blue): diagonal inertia".into());
    hud.scalar("  |omega|", diag_omega, 3);
    hud.raw("Right (orange): fullinertia".into());
    hud.scalar("  |omega|", full_omega, 3);
    hud.scalar("time", data.time, 1);
}

// ── Validation ───────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct InertiaValidation {
    reported: bool,
}

fn inertia_diagnostics(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<InertiaValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    // Check fullinertia path in URDF converter
    let mjcf = sim_urdf::urdf_to_mjcf(FULL_URDF).expect("convert");
    let has_full = mjcf.contains("fullinertia");

    // Check both spinning stably
    let diag_omega = (data.qvel[3].powi(2) + data.qvel[4].powi(2) + data.qvel[5].powi(2)).sqrt();
    let full_omega = (data.qvel[9].powi(2) + data.qvel[10].powi(2) + data.qvel[11].powi(2)).sqrt();

    // Check they diverged
    let diff = ((data.qvel[3] - data.qvel[9]).powi(2)
        + (data.qvel[4] - data.qvel[10]).powi(2)
        + (data.qvel[5] - data.qvel[11]).powi(2))
    .sqrt();

    let checks = vec![
        Check {
            name: "Off-diagonal → fullinertia in MJCF",
            pass: has_full,
            detail: format!("found: {has_full}"),
        },
        Check {
            name: "Diagonal body spins stably",
            pass: diag_omega > 0.1 && !diag_omega.is_nan(),
            detail: format!("|omega|={diag_omega:.3}"),
        },
        Check {
            name: "Full inertia body spins stably",
            pass: full_omega > 0.1 && !full_omega.is_nan(),
            detail: format!("|omega|={full_omega:.3}"),
        },
        Check {
            name: "Different precession (diverged)",
            pass: diff > 0.01,
            detail: format!("omega_diff={diff:.4}"),
        },
    ];
    let _ = print_report("URDF Inertia", &checks);
}
