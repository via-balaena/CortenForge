//! Drag Target — Soft Weld Tracking
//!
//! A translucent green mocap sphere moves on a sinusoidal path. A solid box
//! is connected to the mocap body via a compliant weld constraint. The box
//! follows the target with spring-like lag.
//!
//! Demonstrates the basic mocap -> constraint -> dynamic body pipeline:
//! user sets `data.mocap_pos` each frame, the weld penalty force pulls the
//! dynamic body to follow.
//!
//! Run with: `cargo run -p example-mocap-bodies-drag-target --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::float_cmp,
    clippy::panic,
    clippy::cast_sign_loss
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

const MJCF: &str = r#"
<mujoco model="drag-target">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.002" integrator="RK4"/>

  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.3 0.3 0.3 1" contype="0" conaffinity="0"/>

    <body name="target" mocap="true" pos="0 0 1">
      <geom name="target_geom" type="sphere" size="0.08" mass="0.1"
            rgba="0.2 0.8 0.3 0.4" contype="0" conaffinity="0"/>
    </body>

    <body name="follower" pos="0 0 1">
      <freejoint name="free"/>
      <geom name="follower_geom" type="box" size="0.08 0.08 0.08" mass="1.0"
            rgba="0.85 0.45 0.15 1"/>
    </body>
  </worldbody>

  <equality>
    <weld body1="follower" body2="target" solref="0.02 1.0"/>
  </equality>
</mujoco>
"#;

// ── Constants ─────────────────────────────────────────────────────────────

/// Amplitude of sinusoidal path (meters).
const AMP: f64 = 0.8;
/// Frequency of sinusoidal path (Hz).
const FREQ: f64 = 0.3;
/// Height of the path (Z).
const PATH_Z: f64 = 1.0;

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Drag Target — Soft Weld Tracking ===");
    println!("  Green sphere = mocap target (sinusoidal path)");
    println!("  Orange box = dynamic follower (weld constraint)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Drag Target (Mocap Bodies)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(10.0)
                .print_every(1.0)
                .display(|m, d| {
                    let tid = m.body_id("target").expect("target exists");
                    let fid = m.body_id("follower").expect("follower exists");
                    let mocap_idx = m.body_mocapid[tid].expect("target is mocap");
                    let target = d.mocap_pos[mocap_idx];
                    let follower = d.xpos[fid];
                    let sep = (follower - target).norm();
                    format!(
                        "target=[{:.2}, {:.2}, {:.2}]  sep={sep:.3}",
                        target.x, target.y, target.z,
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (drive_mocap, step_physics_realtime).chain())
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

    println!(
        "  Model: {} bodies, {} DOFs, {} mocap, {} eq constraints",
        model.nbody, model.nv, model.nmocap, model.neq,
    );

    // ── Materials ──────────────────────────────────────────────────────
    let mat_target = materials.add(StandardMaterial {
        base_color: Color::srgba(0.2, 0.8, 0.3, 0.4),
        alpha_mode: AlphaMode::Blend,
        ..default()
    });
    let mat_follower =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.45, 0.15)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("target_geom", mat_target), ("follower_geom", mat_follower)],
    );

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, 0.0, 1.0),
        3.5,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Mocap Drive ───────────────────────────────────────────────────────────

/// Update mocap_pos to follow a sinusoidal path before stepping.
fn drive_mocap(model: Res<PhysicsModel>, mut data: ResMut<PhysicsData>) {
    let tid = model.body_id("target").expect("target exists");
    let mocap_idx = model.body_mocapid[tid].expect("target is mocap");
    let t = data.time;
    let phase = 2.0 * std::f64::consts::PI * FREQ * t;
    data.mocap_pos[mocap_idx] =
        nalgebra::Vector3::new(AMP * phase.sin(), AMP * phase.cos(), PATH_Z);
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Drag Target — Soft Weld Tracking");

    let tid = model.body_id("target").expect("target exists");
    let fid = model.body_id("follower").expect("follower exists");
    let mocap_idx = model.body_mocapid[tid].expect("target is mocap");
    let target = data.mocap_pos[mocap_idx];
    let follower = data.xpos[fid];
    let sep = (follower - target).norm();

    hud.raw(format!(
        "target: [{:.3}, {:.3}, {:.3}]",
        target.x, target.y, target.z
    ));
    hud.raw(format!(
        "follower: [{:.3}, {:.3}, {:.3}]",
        follower.x, follower.y, follower.z
    ));
    hud.scalar("separation", sep, 4);
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Default)]
struct DiagState {
    max_sep: f64,
    mocap_drift: f64,
    reported: bool,
}

fn diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut state: Local<DiagState>,
) {
    let tid = model.body_id("target").expect("target exists");
    let fid = model.body_id("follower").expect("follower exists");
    let mocap_idx = model.body_mocapid[tid].expect("target is mocap");
    let target = data.mocap_pos[mocap_idx];
    let follower = data.xpos[fid];
    let sep = (follower - target).norm();

    // Track the mocap body's actual xpos vs what we set.
    let mocap_xpos = data.xpos[tid];
    let mocap_err = (mocap_xpos - target).norm();
    if mocap_err > state.mocap_drift {
        state.mocap_drift = mocap_err;
    }
    // Skip first second — initial transient as follower catches up from rest.
    if data.time > 1.0 && sep > state.max_sep {
        state.max_sep = sep;
    }

    if harness.reported() && !state.reported {
        state.reported = true;

        let checks = vec![
            Check {
                name: "Mocap tracks scripted path exactly",
                pass: state.mocap_drift < 1e-10,
                detail: format!("max drift = {:.2e}", state.mocap_drift),
            },
            Check {
                name: "Follower stays within 0.3 m of target",
                pass: state.max_sep < 0.3,
                detail: format!("max separation = {:.4}", state.max_sep),
            },
        ];
        let _ = print_report("Drag Target (t=10s)", &checks);
    }
}
