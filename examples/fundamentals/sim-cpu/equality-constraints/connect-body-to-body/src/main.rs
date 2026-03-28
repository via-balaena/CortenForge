//! Connect Body-to-Body — Chained Ball-and-Socket
//!
//! Two free bodies chained together via connect constraints. Link1 is connected
//! to the world at the origin. Link2 is connected to link1's tip. This creates
//! a double pendulum using only equality constraints (no joints).
//!
//! Demonstrates: body-to-body connect, anchor placement in body1's frame,
//! chaining multiple connect constraints.
//!
//! Validates:
//! - Pivot 1 stays at world origin (< 5mm)
//! - Pivot 2: link1 tip stays at link2 origin (< 5mm)
//! - Both links rotate freely
//! - Energy bounded (< 5% growth)
//!
//! Run with: `cargo run -p example-equality-connect-body-to-body --release`

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
use sim_core::validation::{Check, print_report};

// ── MJCF Model ──────────────────────────────────────────────────────────────
//
// Two free bodies forming a double pendulum via connect constraints.
// Link1 at origin → world connect. Link2 at link1 tip → body-to-body connect.

const MJCF: &str = r#"
<mujoco model="connect-body-to-body">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
          iterations="50" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="link1" pos="0 0 0">
      <freejoint damping="0.01"/>
      <geom name="rod1" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.03" mass="1"/>
      <geom name="tip1" type="sphere" pos="0 0 -0.5" size="0.06" mass="0.5"/>
    </body>

    <body name="link2" pos="0 0 -0.5">
      <freejoint damping="0.01"/>
      <geom name="rod2" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.025" mass="0.8"/>
      <geom name="tip2" type="sphere" pos="0 0 -0.4" size="0.05" mass="0.3"/>
    </body>
  </worldbody>

  <equality>
    <connect body1="link1" anchor="0 0 0" solref="0.005 1.0"/>
    <connect body1="link1" body2="link2" anchor="0 0 -0.5" solref="0.005 1.0"/>
  </equality>
</mujoco>
"#;

const BODY_LINK1: usize = 1;
const BODY_LINK2: usize = 2;
const ANCHOR_LOCAL: Vector3<f64> = Vector3::new(0.0, 0.0, -0.5);

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Connect Body-to-Body ===");
    println!("  Double pendulum via chained connect constraints");
    println!("  Link1 → world, Link2 → link1 tip");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Connect Body-to-Body".into(),
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
                    let e = d.energy_kinetic + d.energy_potential;
                    let w1 = Vector3::new(d.qvel[3], d.qvel[4], d.qvel[5]).norm();
                    let w2 = Vector3::new(d.qvel[9], d.qvel[10], d.qvel[11]).norm();
                    format!("w1={w1:.2}  w2={w2:.2}  E={e:.3}J")
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

    // Angular kick to start swinging
    data.qvel[4] = 2.0;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} equality constraints\n",
        model.nbody, model.njnt, model.neq
    );

    let mat_rod1 = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip1 =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));
    let mat_rod2 = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip2 =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.4, 0.85)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("rod1", mat_rod1),
            ("tip1", mat_tip1),
            ("rod2", mat_rod2),
            ("tip2", mat_tip2),
        ],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -0.4, 0.0),
        2.5,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(_model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Connect Body-to-Body");

    let pivot1_err = data.xpos[BODY_LINK1].norm() * 1000.0;
    hud.scalar("pivot1 err (mm)", pivot1_err, 2);

    let anchor_world =
        data.xpos[BODY_LINK1] + data.xquat[BODY_LINK1].transform_vector(&ANCHOR_LOCAL);
    let pivot2_err = (anchor_world - data.xpos[BODY_LINK2]).norm() * 1000.0;
    hud.scalar("pivot2 err (mm)", pivot2_err, 2);

    let w1 = Vector3::new(data.qvel[3], data.qvel[4], data.qvel[5]).norm();
    let w2 = Vector3::new(data.qvel[9], data.qvel[10], data.qvel[11]).norm();
    hud.scalar("w1 (rad/s)", w1, 2);
    hud.scalar("w2 (rad/s)", w2, 2);

    let energy = data.energy_kinetic + data.energy_potential;
    hud.scalar("energy (J)", energy, 3);
    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct Validation {
    max_pivot1_err: f64,
    max_pivot2_err: f64,
    max_angvel1: f64,
    max_angvel2: f64,
    initial_energy: Option<f64>,
    max_energy_growth: f64,
    reported: bool,
}

fn diagnostics(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<Validation>,
) {
    if val.initial_energy.is_none() {
        val.initial_energy = Some(data.energy_kinetic + data.energy_potential);
    }

    let err1 = data.xpos[BODY_LINK1].norm();
    val.max_pivot1_err = val.max_pivot1_err.max(err1);

    let anchor_world =
        data.xpos[BODY_LINK1] + data.xquat[BODY_LINK1].transform_vector(&ANCHOR_LOCAL);
    let err2 = (anchor_world - data.xpos[BODY_LINK2]).norm();
    val.max_pivot2_err = val.max_pivot2_err.max(err2);

    let w1 = Vector3::new(data.qvel[3], data.qvel[4], data.qvel[5]).norm();
    val.max_angvel1 = val.max_angvel1.max(w1);
    let w2 = Vector3::new(data.qvel[9], data.qvel[10], data.qvel[11]).norm();
    val.max_angvel2 = val.max_angvel2.max(w2);

    if let Some(e0) = val.initial_energy {
        let e = data.energy_kinetic + data.energy_potential;
        let growth = (e - e0) / e0.abs().max(1e-10);
        val.max_energy_growth = val.max_energy_growth.max(growth);
    }

    if harness.reported() && !val.reported {
        val.reported = true;
        let checks = vec![
            Check {
                name: "Pivot 1 anchored",
                pass: val.max_pivot1_err < 0.005,
                detail: format!("max err = {:.2} mm", val.max_pivot1_err * 1000.0),
            },
            Check {
                name: "Pivot 2 attached",
                pass: val.max_pivot2_err < 0.005,
                detail: format!("max err = {:.2} mm", val.max_pivot2_err * 1000.0),
            },
            Check {
                name: "Both rotate",
                pass: val.max_angvel1 > 0.1 && val.max_angvel2 > 0.1,
                detail: format!("w1={:.2} w2={:.2} rad/s", val.max_angvel1, val.max_angvel2),
            },
            Check {
                name: "Energy bounded",
                pass: val.max_energy_growth < 0.05,
                detail: format!("max growth = {:.2}%", val.max_energy_growth * 100.0),
            },
        ];
        let _ = print_report("Connect Body-to-Body (t=5s)", &checks);
    }
}
