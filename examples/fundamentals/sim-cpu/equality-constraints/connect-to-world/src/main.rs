//! Connect to World — Ball-and-Socket Pendulum
//!
//! A single free body connected to the world at the origin via `<connect>`.
//! The body swings as a pendulum — position is locked at the anchor, but
//! rotation is completely free (3-DOF position lock, 3-DOF rotation free).
//!
//! This is the simplest possible connect constraint: one body, one anchor,
//! connected to the world frame.
//!
//! Validates:
//! - Pivot stays at world origin (< 5mm)
//! - Rotation is free (angular velocity > 0.1 rad/s)
//! - Energy bounded (< 5% growth)
//!
//! Run with: `cargo run -p example-equality-connect-to-world --release`

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
// One free body with a capsule+sphere (rod with tip mass). Connect constraint
// locks the body origin to the world origin. The body swings freely under
// gravity like a ball-and-socket pendulum.

const MJCF: &str = r#"
<mujoco model="connect-to-world">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
          iterations="50" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="link" pos="0 0 0">
      <freejoint damping="0.01"/>
      <geom name="rod" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.03" mass="1"/>
      <geom name="ball" type="sphere" pos="0 0 -0.5" size="0.06" mass="0.5"/>
    </body>
  </worldbody>

  <equality>
    <connect body1="link" anchor="0 0 0" solref="0.005 1.0"/>
  </equality>
</mujoco>
"#;

const BODY_LINK: usize = 1;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Connect to World ===");
    println!("  Single free body locked to world origin");
    println!("  Position locked, rotation free (ball-and-socket)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Connect to World".into(),
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
                    let err = d.xpos[1].norm() * 1000.0;
                    let w = Vector3::new(d.qvel[3], d.qvel[4], d.qvel[5]).norm();
                    format!("pivot_err={err:.2}mm  w={w:.2}rad/s")
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

    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_ball =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("rod", mat_rod), ("ball", mat_ball)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -0.25, 0.0),
        2.0,
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
    hud.section("Connect to World");

    let pivot_err = data.xpos[BODY_LINK].norm() * 1000.0;
    hud.scalar("pivot err (mm)", pivot_err, 2);

    let angvel = Vector3::new(data.qvel[3], data.qvel[4], data.qvel[5]).norm();
    hud.scalar("angvel (rad/s)", angvel, 2);

    let energy = data.energy_kinetic + data.energy_potential;
    hud.scalar("energy (J)", energy, 3);

    if data.ne >= 3 {
        let f = Vector3::new(data.efc_force[0], data.efc_force[1], data.efc_force[2]).norm();
        hud.scalar("efc |F| (N)", f, 1);
    }

    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct Validation {
    max_pivot_err: f64,
    max_angvel: f64,
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

    let err = data.xpos[BODY_LINK].norm();
    val.max_pivot_err = val.max_pivot_err.max(err);

    let angvel = Vector3::new(data.qvel[3], data.qvel[4], data.qvel[5]).norm();
    val.max_angvel = val.max_angvel.max(angvel);

    if let Some(e0) = val.initial_energy {
        let e = data.energy_kinetic + data.energy_potential;
        let growth = (e - e0) / e0.abs().max(1e-10);
        val.max_energy_growth = val.max_energy_growth.max(growth);
    }

    if harness.reported() && !val.reported {
        val.reported = true;
        let checks = vec![
            Check {
                name: "Pivot anchored",
                pass: val.max_pivot_err < 0.005,
                detail: format!("max err = {:.2} mm", val.max_pivot_err * 1000.0),
            },
            Check {
                name: "Rotation free",
                pass: val.max_angvel > 0.1,
                detail: format!("max w = {:.3} rad/s", val.max_angvel),
            },
            Check {
                name: "Energy bounded",
                pass: val.max_energy_growth < 0.05,
                detail: format!("max growth = {:.2}%", val.max_energy_growth * 100.0),
            },
        ];
        let _ = print_report("Connect to World (t=5s)", &checks);
    }
}
