//! Weld Body-to-Body — Rigid Glue
//!
//! Two free-floating bodies welded together — a box and a capsule fall as a
//! single rigid unit under gravity. The weld constraint locks all 6 DOFs of
//! relative motion (position + orientation), so the pair behaves as one object.
//!
//! Demonstrates: `<weld>` body-to-body, relative pose lock, penalty stiffness
//! visible as slight flex on impact.
//!
//! Validates:
//! - Relative position stays constant (< 2mm)
//! - Relative orientation stays constant (< 0.05 rad)
//! - Pair falls together (z-velocities match during freefall)
//!
//! Run with: `cargo run -p example-equality-weld-body-to-body --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]

use bevy::prelude::*;
use nalgebra::Vector3;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{
    PhysicsAccumulator, PhysicsData, PhysicsModel, spawn_model_geoms, step_physics_realtime,
    sync_geom_transforms,
};
use sim_core::validation::{Check, print_report, quat_rotation_angle};

// ── MJCF Model ──────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="weld-body-to-body">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
          iterations="50" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <geom name="ground" type="plane" size="2 2 0.01"/>

    <body name="base" pos="0 0 1.5">
      <freejoint/>
      <geom name="base_box" type="box" size="0.12 0.08 0.08" mass="1.0"/>
    </body>
    <body name="arm" pos="0 0 1.42">
      <freejoint/>
      <geom name="arm_cap" type="capsule" fromto="0 0 0 0 0 -0.3" size="0.04" mass="0.5"/>
    </body>
  </worldbody>

  <equality>
    <weld body1="base" body2="arm" solref="0.003 1.0"/>
  </equality>
</mujoco>
"#;

const BODY_BASE: usize = 1;
const BODY_ARM: usize = 2;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Weld Body-to-Body ===");
    println!("  Box + capsule welded together — fall as one rigid unit");
    println!("  6-DOF relative pose lock (position + orientation)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Weld Body-to-Body".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<Validation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(5.0)
                .print_every(1.0)
                .display(|_m, d| {
                    let base_z = d.xpos[1][2];
                    let arm_z = d.xpos[2][2];
                    format!("base_z={base_z:.3}  arm_z={arm_z:.3}")
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

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();
    let _ = data.forward(&model);

    let init_rel_pos = data.xpos[BODY_ARM] - data.xpos[BODY_BASE];
    let init_rel_quat = data.xquat[BODY_BASE].inverse() * data.xquat[BODY_ARM];

    println!(
        "  Model: {} bodies, {} joints, {} equality constraints\n",
        model.nbody, model.njnt, model.neq
    );

    let mat_base =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.7, 0.35, 0.15)));
    let mat_arm = materials.add(MetalPreset::BrushedMetal.material());

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("base_box", mat_base), ("arm_cap", mat_arm)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.75, 0.0),
        3.0,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(InitialState {
        rel_pos: init_rel_pos,
        rel_quat: init_rel_quat,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

#[derive(Resource)]
struct InitialState {
    rel_pos: Vector3<f64>,
    rel_quat: nalgebra::UnitQuaternion<f64>,
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(data: Res<PhysicsData>, init: Res<InitialState>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Weld Body-to-Body");

    let rel_pos = data.xpos[BODY_ARM] - data.xpos[BODY_BASE];
    let pos_err = (rel_pos - init.rel_pos).norm() * 1000.0;
    hud.scalar("pos err (mm)", pos_err, 2);

    let q = (init.rel_quat.inverse() * data.xquat[BODY_BASE].inverse() * data.xquat[BODY_ARM])
        .into_inner();
    let angle_err = quat_rotation_angle(q.w, q.i, q.j, q.k);
    hud.scalar("angle err (rad)", angle_err, 4);

    hud.scalar("base z", data.xpos[BODY_BASE][2], 3);
    hud.scalar("arm z", data.xpos[BODY_ARM][2], 3);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct Validation {
    max_pos_err: f64,
    max_angle_err: f64,
    freefall_vel_match: bool,
    freefall_checked: bool,
    reported: bool,
}

fn diagnostics(
    data: Res<PhysicsData>,
    init: Res<InitialState>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<Validation>,
) {
    if !val.freefall_checked {
        val.freefall_vel_match = true;
        val.freefall_checked = true;
    }

    let rel_pos = data.xpos[BODY_ARM] - data.xpos[BODY_BASE];
    let pos_err = (rel_pos - init.rel_pos).norm();
    val.max_pos_err = val.max_pos_err.max(pos_err);

    let q = (init.rel_quat.inverse() * data.xquat[BODY_BASE].inverse() * data.xquat[BODY_ARM])
        .into_inner();
    let angle_err = quat_rotation_angle(q.w, q.i, q.j, q.k);
    val.max_angle_err = val.max_angle_err.max(angle_err);

    // During freefall (first 0.3s), z-velocities should match
    if data.time < 0.3 {
        let base_vz = data.qvel[2];
        let arm_vz = data.qvel[8];
        if (base_vz - arm_vz).abs() > 0.10 * base_vz.abs().max(0.1) {
            val.freefall_vel_match = false;
        }
    }

    if harness.reported() && !val.reported {
        val.reported = true;
        let checks = vec![
            Check {
                name: "Relative pos locked",
                pass: val.max_pos_err < 0.002,
                detail: format!("max = {:.2} mm", val.max_pos_err * 1000.0),
            },
            Check {
                name: "Relative angle locked",
                pass: val.max_angle_err < 0.05,
                detail: format!("max = {:.4} rad", val.max_angle_err),
            },
            Check {
                name: "Pair falls together",
                pass: val.freefall_vel_match,
                detail: "z-velocities match during freefall".into(),
            },
        ];
        let _ = print_report("Weld Body-to-Body (t=5s)", &checks);
    }
}
