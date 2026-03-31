//! Joint Dynamics — Damping and Friction
//!
//! Three identical pendulums side by side with different dynamics:
//! - Left (blue): no loss — swings forever
//! - Center (green): damping=0.5 — velocity-dependent, decays smoothly
//! - Right (red): friction=0.5 — velocity-independent, decays linearly then stops
//!
//! All start at 45°. Watch the damped and friction arms slow down while
//! the conservative arm keeps swinging.
//!
//! Run: `cargo run -p example-urdf-damping-friction --release`

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

// ── URDF for validation checks ───────────────────────────────────────────

const FRICTION_URDF: &str = r#"<?xml version="1.0"?>
<robot name="friction">
    <link name="base">
        <inertial>
            <mass value="10.0"/>
            <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
    </link>
    <link name="arm">
        <inertial>
            <origin xyz="0 0 -0.5"/>
            <mass value="1.0"/>
            <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.002"/>
        </inertial>
    </link>
    <joint name="hinge" type="revolute">
        <parent link="base"/>
        <child link="arm"/>
        <origin xyz="0 0 1.0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-3.14" upper="3.14"/>
        <dynamics damping="0" friction="0.5"/>
    </joint>
</robot>
"#;

// ── Combined MJCF with three pendulums ───────────────────────────────────

/// Three pendulums: no-loss, damped, friction. Each has a rod + sphere tip.
/// Colors: blue (no loss), green (damped), red (friction).
const COMBINED_MJCF: &str = r#"
<mujoco model="damping_friction">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002">
    <flag energy="enable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <!-- No loss (blue) -->
    <body name="arm_none" pos="-0.8 0 2.0">
      <joint name="j_none" type="hinge" axis="0 1 0" limited="true" range="-3.14 3.14"/>
      <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.02 0.02 0.002"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.45" size="0.02" rgba="0.3 0.5 0.8 1"/>
      <geom type="sphere" pos="0 0 -0.5" size="0.06" rgba="0.3 0.5 0.8 1"/>
    </body>

    <!-- Damped (green) -->
    <body name="arm_damp" pos="0 0 2.0">
      <joint name="j_damp" type="hinge" axis="0 1 0" damping="0.5" limited="true" range="-3.14 3.14"/>
      <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.02 0.02 0.002"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.45" size="0.02" rgba="0.3 0.7 0.3 1"/>
      <geom type="sphere" pos="0 0 -0.5" size="0.06" rgba="0.3 0.7 0.3 1"/>
    </body>

    <!-- Friction (red) -->
    <body name="arm_fric" pos="0.8 0 2.0">
      <joint name="j_fric" type="hinge" axis="0 1 0" frictionloss="0.5" limited="true" range="-3.14 3.14"/>
      <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.02 0.02 0.002"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.45" size="0.02" rgba="0.8 0.3 0.3 1"/>
      <geom type="sphere" pos="0 0 -0.5" size="0.06" rgba="0.8 0.3 0.3 1"/>
    </body>
  </worldbody>
</mujoco>
"#;

const INITIAL_ANGLE: f64 = std::f64::consts::FRAC_PI_4; // 45°

// ── Bevy App ─────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: URDF Damping & Friction ===");
    println!("  Left (blue): no loss — swings forever");
    println!("  Center (green): damping=0.5 — decays smoothly");
    println!("  Right (red): friction=0.5 — decays then stops");
    println!("  All start at 45°");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — URDF Damping & Friction".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<DampingValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(3.0)
                .display(|_m, d| {
                    format!(
                        "none={:+5.1}°  damp={:+5.1}°  fric={:+5.1}°",
                        d.qpos[0].to_degrees(),
                        d.qpos[1].to_degrees(),
                        d.qpos[2].to_degrees(),
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
                damping_diagnostics,
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
    let model = sim_mjcf::load_model(COMBINED_MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    // All three start at 45°
    for i in 0..3 {
        data.qpos[i] = INITIAL_ANGLE;
    }
    let _ = data.forward(&model);

    // Verify URDF friction → frictionloss conversion
    let mjcf = sim_urdf::urdf_to_mjcf(FRICTION_URDF).expect("convert");
    println!(
        "  URDF friction → frictionloss: {}\n",
        mjcf.contains("frictionloss")
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
        4.0,
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
    hud.section("Damping & Friction");

    hud.raw("Blue: no loss".into());
    hud.scalar("  angle", data.qpos[0].to_degrees(), 1);
    hud.raw("Green: damping=0.5".into());
    hud.scalar("  angle", data.qpos[1].to_degrees(), 1);
    hud.raw("Red: friction=0.5".into());
    hud.scalar("  angle", data.qpos[2].to_degrees(), 1);
    hud.scalar("time", data.time, 1);
}

// ── Validation ───────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct DampingValidation {
    peak_none: f64,
    peak_damp: f64,
    peak_fric: f64,
    reported: bool,
}

fn damping_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<DampingValidation>,
) {
    // Track peak velocities in the last few seconds
    if data.time > 10.0 {
        val.peak_none = val.peak_none.max(data.qvel[0].abs());
        val.peak_damp = val.peak_damp.max(data.qvel[1].abs());
        val.peak_fric = val.peak_fric.max(data.qvel[2].abs());
    }

    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    // Check URDF conversion
    let mjcf = sim_urdf::urdf_to_mjcf(FRICTION_URDF).expect("convert");
    let has_fl = mjcf.contains("frictionloss");

    // Check damping propagated
    let damping_ok = (model.jnt_damping[1] - 0.5).abs() < 0.001;

    // Check frictionloss propagated
    let fl_ok = (model.dof_frictionloss[2] - 0.5).abs() < 0.001;

    // No-loss should still be swinging, damped/friction should be much smaller
    let none_swinging = val.peak_none > 0.5;
    let damp_decayed = val.peak_damp < val.peak_none * 0.5;
    let fric_decayed = val.peak_fric < val.peak_none * 0.5;

    let checks = vec![
        Check {
            name: "URDF friction → frictionloss",
            pass: has_fl,
            detail: format!("found: {has_fl}"),
        },
        Check {
            name: "Damping=0.5 in model",
            pass: damping_ok,
            detail: format!("jnt_damping[1]={:.3}", model.jnt_damping[1]),
        },
        Check {
            name: "Frictionloss=0.5 in model",
            pass: fl_ok,
            detail: format!("dof_frictionloss[2]={:.3}", model.dof_frictionloss[2]),
        },
        Check {
            name: "No-loss still swinging",
            pass: none_swinging,
            detail: format!("peak_vel={:.3}", val.peak_none),
        },
        Check {
            name: "Damped arm decayed",
            pass: damp_decayed,
            detail: format!("peak={:.3} vs none={:.3}", val.peak_damp, val.peak_none),
        },
        Check {
            name: "Friction arm decayed",
            pass: fric_decayed,
            detail: format!("peak={:.3} vs none={:.3}", val.peak_fric, val.peak_none),
        },
    ];
    let _ = print_report("URDF Damping & Friction", &checks);
}
