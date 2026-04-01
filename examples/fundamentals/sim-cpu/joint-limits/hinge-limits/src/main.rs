//! Hinge Limits — Solver Tuning Comparison
//!
//! Three pendulums with range="-45 45" released from 60 degrees (beyond limit).
//! Each has different solreflimit tuning: stiff (blue), default (green), soft
//! (red). Demonstrates how solref parameters control limit response stiffness.
//!
//! Run with: `cargo run -p example-joint-limits-hinge-limits --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::cast_sign_loss,
    clippy::similar_names
)]

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::physics_pos;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report};

// ── MJCF Model ────────────────────────────────────────────────────────────

/// Three pendulums with different solreflimit tuning, all starting at 60°
/// (beyond the ±45° limit). Each hangs from a fixed pivot via hinge joint.
///
/// | Pendulum | solreflimit  | Behavior                     |
/// |----------|-------------|-------------------------------|
/// | Stiff    | 0.005, 1.0  | Sharp bounce, stays near limit|
/// | Default  | (default)   | Moderate bounce               |
/// | Soft     | 0.08, 1.0   | Visible penetration           |
const MJCF: &str = r#"
<mujoco model="hinge-limits">
  <compiler angle="degree"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>

  <worldbody>
    <!-- Stiff (blue) -->
    <body name="pivot_stiff" pos="-0.6 0 1.5">
      <geom name="pivot_stiff_geom" type="sphere" size="0.03"
            rgba="0.4 0.4 0.4 1" contype="0" conaffinity="0"/>
      <body name="rod_stiff" pos="0 0 0">
        <joint name="stiff" type="hinge" axis="0 1 0"
               limited="true" range="-45 45" ref="60"
               solreflimit="0.005 1.0"/>
        <geom name="rod_stiff_geom" type="capsule"
              fromto="0 0 0 0 0 -0.5" size="0.015" mass="0.5"
              rgba="0.3 0.5 0.85 1"/>
        <body name="tip_stiff" pos="0 0 -0.5">
          <geom name="tip_stiff_geom" type="sphere" size="0.04" mass="0.5"
                rgba="0.3 0.5 0.85 1"/>
        </body>
      </body>
    </body>

    <!-- Default (green) -->
    <body name="pivot_default" pos="0 0 1.5">
      <geom name="pivot_default_geom" type="sphere" size="0.03"
            rgba="0.4 0.4 0.4 1" contype="0" conaffinity="0"/>
      <body name="rod_default" pos="0 0 0">
        <joint name="default" type="hinge" axis="0 1 0"
               limited="true" range="-45 45" ref="60"/>
        <geom name="rod_default_geom" type="capsule"
              fromto="0 0 0 0 0 -0.5" size="0.015" mass="0.5"
              rgba="0.2 0.75 0.3 1"/>
        <body name="tip_default" pos="0 0 -0.5">
          <geom name="tip_default_geom" type="sphere" size="0.04" mass="0.5"
                rgba="0.2 0.75 0.3 1"/>
        </body>
      </body>
    </body>

    <!-- Soft (red) -->
    <body name="pivot_soft" pos="0.6 0 1.5">
      <geom name="pivot_soft_geom" type="sphere" size="0.03"
            rgba="0.4 0.4 0.4 1" contype="0" conaffinity="0"/>
      <body name="rod_soft" pos="0 0 0">
        <joint name="soft" type="hinge" axis="0 1 0"
               limited="true" range="-45 45" ref="60"
               solreflimit="0.08 1.0"/>
        <geom name="rod_soft_geom" type="capsule"
              fromto="0 0 0 0 0 -0.5" size="0.015" mass="0.5"
              rgba="0.85 0.2 0.2 1"/>
        <body name="tip_soft" pos="0 0 -0.5">
          <geom name="tip_soft_geom" type="sphere" size="0.04" mass="0.5"
                rgba="0.85 0.2 0.2 1"/>
        </body>
      </body>
    </body>
  </worldbody>

  <sensor>
    <jointlimitfrc name="stiff_lf" joint="stiff"/>
    <jointlimitfrc name="default_lf" joint="default"/>
    <jointlimitfrc name="soft_lf" joint="soft"/>
  </sensor>
</mujoco>
"#;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Hinge Limits — Solver Tuning ===");
    println!("  Blue  = stiff  (solref 0.005) — sharp bounce");
    println!("  Green = default (solref 0.02)  — moderate bounce");
    println!("  Red   = soft   (solref 0.08)  — visible penetration");
    println!("  All start at 60° (limit = ±45°)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Hinge Limits (Joint Limits)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(5.0)
                .print_every(0.5)
                .display(|m, d| {
                    let sid = m.joint_id("stiff").unwrap();
                    let did = m.joint_id("default").unwrap();
                    let fid = m.joint_id("soft").unwrap();
                    let sa = d.qpos[m.jnt_qpos_adr[sid]].to_degrees();
                    let da = d.qpos[m.jnt_qpos_adr[did]].to_degrees();
                    let fa = d.qpos[m.jnt_qpos_adr[fid]].to_degrees();
                    format!("stiff={sa:.1}° default={da:.1}° soft={fa:.1}°")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Setup ─────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let data = model.make_data();

    // Materials
    let mat_blue =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.85)));
    let mat_green =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.75, 0.3)));
    let mat_red = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.2, 0.2)));
    let mat_pivot =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.4, 0.4, 0.4)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("pivot_stiff_geom", mat_pivot.clone()),
            ("rod_stiff_geom", mat_blue.clone()),
            ("tip_stiff_geom", mat_blue),
            ("pivot_default_geom", mat_pivot.clone()),
            ("rod_default_geom", mat_green.clone()),
            ("tip_default_geom", mat_green),
            ("pivot_soft_geom", mat_pivot),
            ("rod_soft_geom", mat_red.clone()),
            ("tip_soft_geom", mat_red),
        ],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 1.2),
        2.5,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Joint Limits \u{2014} Hinge Limit Tuning");

    let joints = ["stiff", "default", "soft"];
    let labels = ["Stiff (blue)", "Default (grn)", "Soft (red) "];
    let solrefs = ["[0.005, 1.0]", "[0.02,  1.0]", "[0.08,  1.0]"];

    for ((name, label), sr) in joints.iter().zip(labels.iter()).zip(solrefs.iter()) {
        let jid = model.joint_id(name).expect("joint exists");
        let angle = data.qpos[model.jnt_qpos_adr[jid]].to_degrees();
        let frc = data.jnt_limit_frc[jid];
        hud.raw(format!(
            "{label}: angle={angle:6.1}\u{00b0}  limit_frc={frc:7.2}  solref={sr}"
        ));
    }

    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    stiff_max_frc: f64,
    soft_max_frc: f64,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    let stiff_id = model.joint_id("stiff").expect("stiff");
    let soft_id = model.joint_id("soft").expect("soft");

    state.stiff_max_frc = state.stiff_max_frc.max(data.jnt_limit_frc[stiff_id]);
    state.soft_max_frc = state.soft_max_frc.max(data.jnt_limit_frc[soft_id]);

    if !harness.reported() || state.reported {
        return;
    }
    state.reported = true;

    let default_id = model.joint_id("default").expect("default");

    let stiff_deg = data.qpos[model.jnt_qpos_adr[stiff_id]].to_degrees();
    let default_deg = data.qpos[model.jnt_qpos_adr[default_id]].to_degrees();
    let soft_deg = data.qpos[model.jnt_qpos_adr[soft_id]].to_degrees();
    let limit = 45.0;
    let tol = 5.0;

    let checks = vec![
        Check {
            name: "Stiff peak force > soft peak force",
            pass: state.stiff_max_frc > state.soft_max_frc,
            detail: format!(
                "stiff peak = {:.2}, soft peak = {:.2}",
                state.stiff_max_frc, state.soft_max_frc
            ),
        },
        Check {
            name: "All angles within range +/- 5 deg",
            pass: stiff_deg <= limit + tol && default_deg <= limit + tol && soft_deg <= limit + tol,
            detail: format!(
                "stiff={stiff_deg:.1}, default={default_deg:.1}, soft={soft_deg:.1} (limit={limit})"
            ),
        },
        Check {
            name: "JointLimitFrc > 0 for at least one joint",
            pass: state.stiff_max_frc > 0.0 || state.soft_max_frc > 0.0,
            detail: format!(
                "stiff peak = {:.2}, soft peak = {:.2}",
                state.stiff_max_frc, state.soft_max_frc
            ),
        },
    ];
    let _ = print_report("Hinge Limits (t=5s)", &checks);
}
